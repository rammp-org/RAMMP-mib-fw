/// Actuator system-identification logger for the MIB. See README.md.
///
/// Tasks:
///   sample   (prio 7)  every period: read the IMU, copy each node's latest MCP
///                      values, run the motion guards, print one D row
///   poll nX  (prio 6)  one per MCP266: SDO reads, as fast as the period allows
///   Twai     (prio 10) espp::Twai's receive task: hands frames to the nodes
///   console  (prio 4)  reads command lines; `dump` runs here
///   motion   (prio 8)  motion builds only: the bus work of moves and stops
///   heartbeat(prio 5)  motion builds only: the MIB's CANopen heartbeat, while armed
///
/// Every frame this firmware sends goes through send_frame(), which applies the
/// bus policy (bus_policy.hpp) before the TWAI driver sees it.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "bsp.hpp"
#include "bus_policy.hpp"
#include "dump.hpp"
#include "format.hpp"
#include "mcp_node.hpp"
#include "motion.hpp"
#include "twai.hpp"
#include "wire.hpp"

namespace sysid {
namespace {

constexpr size_t kMaxNodes = 8;
constexpr int kMaxRateHz = 100; // also the Kconfig range: the BNO055 fuses at 100 Hz

/// Longest possible D row, bytes, newline included. Each count is the widest value
/// the formatter can print, plus its comma:
///   "D" 1, t_us 13 (12 digits: 11 days), quaternion 4 x 9 ("-0.12345"),
///   gyro 3 x 9 ("-34.9066"), linear accel and gravity 6 x 8 ("-39.123"), cal 4
///   = 129, then newline 1;
///   per channel: enc 12, spd 12, cur_ma 8, duty 7, cmd 2, tgt 12, status 6, lim 2,
///   age_us 7 (at most CONFIG_SYSID_MAX_FIELD_AGE_MS = 500000 us) = 68;
///   per node: vbat_mv 6, temp_dc 6 = 12.
/// With 3 nodes: 130 + 6 x 68 + 3 x 12 = 574 bytes.
size_t worst_case_row_bytes(size_t nodes) { return 130 + nodes * (2 * 68 + 12); }

/// Highest row rate whose worst-case rows use at most 70 % of the console link (10
/// bits per byte on a UART, 8N1). USB-Serial-JTAG has no baud limit to speak of.
int max_rate_for_link(size_t nodes) {
#if CONFIG_ESP_CONSOLE_UART
  const double bytes_per_s = CONFIG_ESP_CONSOLE_UART_BAUDRATE / 10.0 * 0.70;
  return std::clamp(static_cast<int>(bytes_per_s / worst_case_row_bytes(nodes)), 1, kMaxRateHz);
#else
  (void)nodes;
  return kMaxRateHz;
#endif
}

#if CONFIG_SYSID_ENABLE_MOTION
constexpr const char *kMode = "motion";
constexpr uint8_t kHeartbeatNodeId = CONFIG_SYSID_HEARTBEAT_NODE_ID;
#else
constexpr const char *kMode = "passive";
constexpr uint8_t kHeartbeatNodeId = 0; // a passive build sends no heartbeat
#endif

std::atomic<int> g_period_ms{1000 / CONFIG_SYSID_SAMPLE_RATE_HZ};
// Every node ever polled. Reserved to kMaxNodes and only appended to (`nodes
// <list>` adds; removing just disables), so the TWAI receive task can walk the
// first g_node_count entries while the console task appends.
std::vector<std::unique_ptr<McpNode>> g_nodes;
std::atomic<size_t> g_node_count{0};
// `scan` borrows a client per id; the receive task feeds it under this mutex.
std::mutex g_scan_mutex;
espp::CanopenClient *g_scan_client{nullptr};
std::unique_ptr<espp::Twai> g_twai;
NodeSet g_read_only;
std::atomic<bool> g_can_bus_ok{false};
// TWAI error events (espp::Twai on_error: one per driver error interrupt), by
// type. Arbitration lost is normal on a shared bus and is not an error for the
// guards; the others are.
std::atomic<uint32_t> g_can_errors{0}; ///< all error events
std::atomic<uint32_t> g_can_arb_lost{0};
std::atomic<uint32_t> g_can_bit{0};
std::atomic<uint32_t> g_can_form{0};
std::atomic<uint32_t> g_can_stuff{0};
std::atomic<uint32_t> g_can_ack{0};
std::atomic<uint32_t> g_can_bus_off{0}; ///< entries into bus-off
#if CONFIG_SYSID_ENABLE_MOTION
uint32_t can_errors_excluding_arbitration() { return g_can_errors.load() - g_can_arb_lost.load(); }
#endif

/// The polled nodes, in order: entries that exist and are enabled.
template <typename F> void for_each_polled(F &&f) {
  const size_t n = g_node_count.load();
  for (size_t i = 0; i < n; ++i) {
    if (g_nodes[i]->enabled()) {
      f(i, *g_nodes[i]);
    }
  }
}
size_t polled_count() {
  size_t n = 0;
  for_each_polled([&](size_t, McpNode &) { ++n; });
  return n;
}
std::atomic<uint32_t> g_sample_overruns{0};
std::atomic<uint32_t> g_blocked_frames{0};
std::atomic<bool> g_header_requested{false};
espp::Bno055 *g_imu{nullptr};
#if CONFIG_SYSID_ENABLE_MOTION
NodeSet g_motion_nodes;
std::unique_ptr<Motion> g_motion;
std::atomic<bool> g_heartbeat_on{false};
std::atomic<int64_t> g_heartbeat_stop_at_us{0}; ///< 0 = no scheduled stop
#endif

std::string list_text(const NodeSet &set) {
  if (set.count() == 127 && !set.test(0)) {
    return "all (1-127)";
  }
  std::string text;
  for (int id = 1; id < 128; ++id) {
    if (set.test(id)) {
      text += (text.empty() ? "" : ",") + std::to_string(id);
    }
  }
  return text.empty() ? "none" : text;
}

// ------------------------------------------------------------------ transmit

/// THE function that hands frames to the TWAI driver. Every transmit in this
/// firmware comes through here (the CANopen clients' send callback and the
/// heartbeat), and the bus policy decides first. The policy is the last line of
/// defence for the read-only nodes; the commands above refuse them before
/// building a frame.
std::atomic<uint32_t> g_tx_retries{0}; ///< transmit attempts that the driver reported as not sent

bool send_frame(const espp::CanopenClient::CanFrame &frame) {
  const Verdict verdict = check_frame(
      {.id = frame.id, .extended = frame.extended, .rtr = frame.rtr, .dlc = frame.dlc,
       .data = frame.data.data()},
      g_read_only, kHeartbeatNodeId);
  if (!verdict.allow) {
    g_blocked_frames++;
    std::string bytes;
    for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
      bytes += fmt::format("{}{:02X}", i ? " " : "", frame.data[i]);
    }
    if (verdict.node != 0 && g_read_only.test(verdict.node)) {
      emit_event(fmt::format("BLOCKED write to read-only node {} (cob 0x{:03X}, bytes {}): {}",
                             verdict.node, frame.id, bytes, verdict.why));
    } else {
      emit_event(fmt::format("BLOCKED frame (cob 0x{:03X}, bytes {}): {}", frame.id, bytes,
                             verdict.why));
    }
    return false;
  }
  if (!g_twai) {
    return false;
  }
  const espp::Twai::Message message{
      .id = frame.id, .extended = frame.extended, .rtr = frame.rtr, .dlc = frame.dlc,
      .data = frame.data};
  std::error_code ec;
  // The TWAI driver reports some frames as not sent (TX buffer freed without
  // success: seen ~1.6 times a second on the chair's bus, with no bit, stuff or ACK
  // errors counted, so most likely after a lost arbitration). Send the same frame
  // again, a few times. A duplicate is harmless for everything this firmware
  // sends: SDO reads, SDO writes of the same value, and controlwords.
  bool sent = false;
  for (int attempt = 0; attempt < 4 && !sent; ++attempt) {
    sent = g_twai->transmit(message, ec);
    if (!sent) {
      g_tx_retries++;
    }
  }
  if (!sent) {
    // Diagnostic (bench 2026-11-17): say why, at most ~5 times a second.
    static std::atomic<int64_t> last_report_us{0};
    static std::atomic<uint32_t> failures{0};
    failures++;
    const int64_t now = now_us();
    if (now - last_report_us.load() > 200'000) {
      last_report_us = now;
      emit_event(fmt::format("tx failed (cob 0x{:03X}): {} [{} total]", frame.id, ec.message(),
                             failures.load()));
    }
  }
  return sent;
}

// ------------------------------------------------------------------ columns

/// The column names, in row order. The row formatter walks the same structure
/// (IMU, then per node: c1, c2, node fields), so the two cannot drift.
std::string column_header() {
  std::string h = "#cols:t_us,imu_qw,imu_qx,imu_qy,imu_qz,imu_gx,imu_gy,imu_gz,"
                  "imu_lax,imu_lay,imu_laz,imu_grx,imu_gry,imu_grz,imu_cal";
  for_each_polled([&](size_t, McpNode &node_ref) {
    const McpNode *node = &node_ref;
    for (int ch = 1; ch <= 2; ++ch) {
      const std::string p = fmt::format(",n{}c{}_", node->id(), ch);
      for (const char *suffix :
           {"enc", "spd", "cur_ma", "duty", "cmd", "tgt", "status", "lim", "age_us"}) {
        h += p + suffix;
      }
    }
    h += fmt::format(",n{0}_vbat_mv,n{0}_temp_dc", node->id());
  });
  return h;
}

/// One D row: "D" followed by ",<value>" or "," per column.
class Row {
public:
  Row() { text_.reserve(640); text_ += 'D'; }
  void empty() { text_ += ','; }
  void integer(int64_t v) { text_ += fmt::format(",{}", v); }
  void real(float v, int decimals) { text_ += fmt::format(",{:.{}f}", v, decimals); }
  /// Whether an MCP value is printed: node online, value present and fresh enough.
  static bool shown(const NodeData &node, const Field &f, int64_t t_us) {
    return node.link == Link::ONLINE && f.valid &&
           t_us - f.t_us <= static_cast<int64_t>(CONFIG_SYSID_MAX_FIELD_AGE_MS) * 1000;
  }
  void field(const NodeData &node, const Field &f, int64_t t_us) {
    if (shown(node, f, t_us)) {
      integer(f.value);
    } else {
      empty();
    }
  }
  const std::string &text() const { return text_; }

private:
  std::string text_;
};

// ------------------------------------------------------------------ metadata

void emit_mode() {
  emit_meta("mode", kMode);
  emit_meta("read_only_nodes", list_text(g_read_only));
#if CONFIG_SYSID_ENABLE_MOTION
  emit_meta("motion_nodes", list_text(g_motion_nodes & ~g_read_only));
#endif
}

void emit_header() {
  const esp_app_desc_t *app = esp_app_get_description();
  const int period = g_period_ms.load();
  std::string ids;
  for_each_polled(
      [&](size_t, McpNode &node) { ids += (ids.empty() ? "" : ",") + std::to_string(node.id()); });
  emit_meta("app", "actuator_sysid");
  emit_meta("fw", app->version); // `git describe` of the repo at build time
  emit_meta("idf", esp_get_idf_version());
  emit_mode();
  emit_meta("sample_rate_hz", fmt::format("{:.3f}", 1000.0 / period));
  emit_meta("sample_period_ms", std::to_string(period));
#if CONFIG_ESP_CONSOLE_UART
  emit_meta("console", fmt::format("uart{} {} baud; worst-case row {} bytes; max rate {} Hz",
                                   CONFIG_ESP_CONSOLE_UART_NUM, CONFIG_ESP_CONSOLE_UART_BAUDRATE,
                                   worst_case_row_bytes(polled_count()),
                                   max_rate_for_link(polled_count())));
#else
  emit_meta("console", "usb-serial-jtag");
#endif
  emit_meta("node_ids", ids);
  emit_meta("channels", "c1=M1,c2=M2");
  emit_meta("can", fmt::format("tx_gpio={} rx_gpio={} bitrate={} sdo_timeout_ms={}",
                               CONFIG_SYSID_CAN_TX_GPIO, CONFIG_SYSID_CAN_RX_GPIO,
                               CONFIG_SYSID_CAN_BITRATE, CONFIG_SYSID_SDO_TIMEOUT_MS));
#if CONFIG_SYSID_ENABLE_MOTION
  emit_meta("bus_access", "motion build: SDO reads; writes only from arm, pstep, clearfaults, "
                          "stop and disarm, to motion nodes only; heartbeat only while armed");
  emit_meta("motion", fmt::format("pstep_max_delta={} clamp_margin={} travel_margin={} "
                                  "window_ms={} velocity={} acceleration={} max_current_ma={} "
                                  "max_tilt_rate_dps={} arm_timeout_s={}",
                                  CONFIG_SYSID_PSTEP_MAX_DELTA_COUNTS,
                                  CONFIG_SYSID_CLAMP_MARGIN_COUNTS,
                                  CONFIG_SYSID_TRAVEL_MARGIN_COUNTS, CONFIG_SYSID_MOVE_WINDOW_MS,
                                  CONFIG_SYSID_MOVE_VELOCITY, CONFIG_SYSID_MOVE_ACCELERATION,
                                  CONFIG_SYSID_MAX_CURRENT_MA, CONFIG_SYSID_MAX_TILT_RATE_DPS,
                                  CONFIG_SYSID_ARM_TIMEOUT_S));
#else
  emit_meta("bus_access", "passive: SDO uploads only, never a write");
#endif
  // Units, one per column family.
  emit_meta("unit_t_us", "us since boot (esp_timer), taken after the IMU read and the MCP copy");
  emit_meta("unit_imu_q", "unit quaternion w,x,y,z; BNO055 IMU fusion mode, heading relative "
                          "to power-on");
  emit_meta("unit_imu_g", "rad/s, BNO055 sensor frame");
  emit_meta("unit_imu_la", "m/s^2, gravity removed");
  emit_meta("unit_imu_gr", "m/s^2, gravity vector");
  emit_meta("unit_imu_cal", "BNO055 CALIB_STAT layout: sys*64+gyro*16+accel*4+mag, each 0..3");
  emit_meta("unit_enc", "counts, CiA 402 position actual 0x6064/0x6864");
  emit_meta("unit_spd", "CiA 402 velocity actual 0x606C/0x686C, logged only: on the bench it "
                       "toggled 0 / +-1000 / +-2000 at rest and sometimes read -2147483648; "
                       "nothing uses it, derive speed from enc");
  emit_meta("unit_cur_ma", "mA, Basicmicro motor currents 0x2031:1/:2 x10 (units unconfirmed)");
  emit_meta("unit_duty", "-32767..32767 = -100..100 %, Basicmicro motor PWMs 0x2030:1/:2 "
                         "(units unconfirmed)");
  emit_meta("unit_cmd", "open-loop duty commanded by this firmware: always 0 (none is ever "
                        "commanded; position moves are in tgt)");
  emit_meta("unit_tgt", "counts, target of the pstep running on the channel; empty otherwise");
  emit_meta("unit_status", "CiA 402 statusword 0x6041/0x6841, decimal");
  emit_meta("unit_lim", "bits: 1=statusword bit 11 (internal limit) 2=node S4 triggered "
                        "4=node S5 triggered (0x205A flags; S bits only when readable)");
  emit_meta("unit_age_us", "row t_us minus the midpoint of the SDO read of enc; spd, cur and "
                           "duty are read in the same pass");
  emit_meta("unit_vbat_mv", "mV, main battery 0x2018");
  emit_meta("unit_temp_dc", "0.1 degC, board temperature 0x2052");
  emit_meta("refresh", "every pass: enc,spd,cur_ma,duty; one per pass in turn: status c1, "
                       "status c2, flags (lim), vbat, temp");
  emit_meta("empty_field", fmt::format("no data: node offline, field unreadable, or older than "
                                       "{} ms",
                                       CONFIG_SYSID_MAX_FIELD_AGE_MS));
  // Running statistics, so CAN latency and load are visible in the log itself.
  for_each_polled([](size_t, McpNode &node) {
    const PollStats s = node.stats();
    emit_meta(fmt::format("n{}_poll", node.id()),
              fmt::format("passes:{};failed_passes:{};timeouts:{};aborts:{};mean_pass_us:{};"
                          "max_pass_us:{};max_sdo_us:{}",
                          s.passes, s.failed_passes, s.timeouts, s.aborts,
                          s.passes ? s.pass_us_sum / s.passes : 0, s.pass_us_max, s.sdo_us_max));
    // Every SDO reply is checked against the request in flight (index:sub);
    // mismatched = late or stray replies, which espp also drops.
    const SdoAccounting a = node.sdo_accounting();
    emit_meta(fmt::format("n{}_sdo", node.id()),
              fmt::format("requests:{};replies:{};mismatched:{}", a.requests, a.replies,
                          a.mismatched));
  });
  emit_meta("sample_overruns", std::to_string(g_sample_overruns.load()));
  // can_bus_errors counts espp::Twai on_error events (one per TWAI error
  // interrupt), arbitration lost included; the breakdown is by error flag.
  emit_meta("can_bus_errors", std::to_string(g_can_errors.load()));
  std::string driver = "unavailable";
  if (g_twai) {
    twai_node_status_t status{};
    twai_node_record_t record{};
    std::error_code ec;
    if (g_twai->get_info(status, record, ec)) {
      driver = fmt::format("state:{};tec:{};rec:{};bus_err_num:{}", static_cast<int>(status.state),
                           status.tx_error_count, status.rx_error_count, record.bus_err_num);
    }
  }
  emit_meta("can_errors", fmt::format("arbitration_lost:{};bit:{};form:{};stuff:{};ack:{};"
                                      "bus_off_entries:{};driver {}",
                                      g_can_arb_lost.load(), g_can_bit.load(), g_can_form.load(),
                                      g_can_stuff.load(), g_can_ack.load(), g_can_bus_off.load(),
                                      driver));
  emit_meta("tx_retries", fmt::format("{}", g_tx_retries.load()));
  emit_meta("blocked_frames", std::to_string(g_blocked_frames.load()));
  emit_meta("tx_dropped_lines", std::to_string(dropped_lines()));
  emit_line(column_header());
}

// ------------------------------------------------------------------ sampling

#if CONFIG_SYSID_ENABLE_MOTION
/// Angular rate perpendicular to gravity, deg/s: how fast the chair is tilting,
/// whatever way the IMU is mounted, ignoring yaw. False if gravity is unknown.
bool tilt_rate_dps(const espp::Bno055::Data &d, float &rate) {
  const auto &g = d.gravity_mps2;
  const auto &w = d.gyro_radps;
  const float g_norm = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
  if (g_norm < 1.0f) {
    return false;
  }
  const float along = (w.x * g.x + w.y * g.y + w.z * g.z) / g_norm; // rate about gravity (yaw)
  const float total_sq = w.x * w.x + w.y * w.y + w.z * w.z;
  rate = std::sqrt(std::max(0.0f, total_sq - along * along)) * 180.0f / static_cast<float>(M_PI);
  return true;
}
#endif

void sample_task(void *) {
  std::vector<NodeData> snapshots(kMaxNodes);
  TickType_t last_wake = xTaskGetTickCount();
  int64_t next_header_us =
      now_us() + static_cast<int64_t>(CONFIG_SYSID_HEADER_PERIOD_S) * 1'000'000;
  for (;;) {
    if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(g_period_ms.load())) == pdFALSE) {
      g_sample_overruns++;
      last_wake = xTaskGetTickCount();
    }

    // IMU: one 46-byte burst over I2C (~1.5 ms).
    bool imu_ok = false;
    espp::Bno055::Data imu{};
    if (g_imu != nullptr) {
      std::error_code ec;
      imu_ok = g_imu->update(ec);
      if (imu_ok) {
        imu = g_imu->data();
      }
    }
    // The polled set is captured once per row, so the row matches one header.
    std::vector<size_t> polled;
    for_each_polled([&](size_t i, McpNode &node) {
      snapshots[i] = node.snapshot();
      polled.push_back(i);
    });
    const int64_t t = now_us();

#if CONFIG_SYSID_ENABLE_MOTION
    float tilt = 0.0f;
    const bool tilt_ok = imu_ok && tilt_rate_dps(imu, tilt);
    g_motion->check({.t_us = t,
                     .nodes = snapshots,
                     .imu_valid = tilt_ok,
                     .tilt_rate_dps = tilt,
                     .can_bus_ok = g_can_bus_ok.load(),
                     .can_errors = can_errors_excluding_arbitration()});
#endif

    if (g_header_requested.exchange(false) || t >= next_header_us) {
      emit_header();
      next_header_us = t + static_cast<int64_t>(CONFIG_SYSID_HEADER_PERIOD_S) * 1'000'000;
    }

    Row row;
    row.integer(t);
    if (imu_ok) {
      row.real(imu.orientation.w, 5);
      row.real(imu.orientation.x, 5);
      row.real(imu.orientation.y, 5);
      row.real(imu.orientation.z, 5);
      row.real(imu.gyro_radps.x, 4);
      row.real(imu.gyro_radps.y, 4);
      row.real(imu.gyro_radps.z, 4);
      row.real(imu.linear_accel_mps2.x, 3);
      row.real(imu.linear_accel_mps2.y, 3);
      row.real(imu.linear_accel_mps2.z, 3);
      row.real(imu.gravity_mps2.x, 3);
      row.real(imu.gravity_mps2.y, 3);
      row.real(imu.gravity_mps2.z, 3);
      const auto &cal = imu.calibration;
      row.integer(cal.system * 64 + cal.gyro * 16 + cal.accel * 4 + cal.mag);
    } else {
      for (int i = 0; i < 14; ++i) {
        row.empty();
      }
    }
    for (size_t i : polled) {
      const NodeData &node = snapshots[i];
      for (int ch = 0; ch < 2; ++ch) {
        const ChannelData &c = node.ch[ch];
        row.field(node, c.enc, t);
        row.field(node, c.spd, t);
        row.field(node, c.cur_ma, t);
        row.field(node, c.duty, t);
        row.integer(0); // _cmd: no open-loop duty is ever commanded
        int32_t target = 0;
#if CONFIG_SYSID_ENABLE_MOTION
        if (g_motion->target(i, ch, target)) {
          row.integer(target);
        } else {
          row.empty();
        }
#else
        (void)target;
        row.empty(); // _tgt: a passive build never moves anything
#endif
        row.field(node, c.status, t);
        int32_t lim = 0;
        if (node.link == Link::ONLINE && limit_bits(node, ch, lim)) {
          row.integer(lim);
        } else {
          row.empty();
        }
        // The age goes with the encoder value: shown exactly when enc is.
        if (Row::shown(node, c.enc, t)) {
          row.integer(t - c.enc.t_us);
        } else {
          row.empty();
        }
      }
      row.field(node, node.vbat_mv, t);
      row.field(node, node.temp_dc, t);
    }
    emit_line(row.text());
  }
}

// ------------------------------------------------------------------ commands

/// `stop`.
/// Motion build: Motion::stop() runs the stop sequence on every channel of every
/// motion node (motion.cpp, run_stop()) and reports duty and current afterwards;
/// read-only nodes are only reported, never sent anything.
/// Passive build: strictly read-only on the bus, so nothing is sent; this firmware
/// never commanded anything. It reports each channel's duty and current as last
/// polled, so the operator sees whether anything is driving.
void stop_everything() {
#if CONFIG_SYSID_ENABLE_MOTION
  g_motion->stop();
#else
  std::string readback;
  const int64_t now = now_us();
  for_each_polled([&](size_t, McpNode &node) {
    const NodeData data = node.snapshot();
    for (int ch = 0; ch < 2; ++ch) {
      readback += (readback.empty() ? "" : "; ") + output_readback(node.id(), data, ch, now);
    }
  });
  emit_event("stop: passive build, read-only, nothing commanded and nothing sent; readback: " +
             readback);
#endif
}

void report_nodes() {
  const size_t n = g_node_count.load();
  for (size_t i = 0; i < n; ++i) {
    McpNode *node = g_nodes[i].get();
    const NodeData d = node->snapshot();
    const char *link = !node->enabled()          ? "not polled"
                       : d.link == Link::ONLINE  ? "online"
                       : d.link == Link::OFFLINE ? "offline"
                                                 : "not yet answered";
    const std::string name = node->device_name();
    emit_meta(fmt::format("n{}_state", node->id()),
              fmt::format("{}{}{}{}", link, g_read_only.test(node->id()) ? "; read-only" : "",
                          name.empty() ? "" : "; ", name));
  }
}

McpNode *find_node(int id) {
  const size_t n = g_node_count.load();
  for (size_t i = 0; i < n; ++i) {
    if (g_nodes[i]->id() == id) {
      return g_nodes[i].get();
    }
  }
  return nullptr;
}

/// `nodes <list>`: poll exactly these ids. Known nodes are enabled or disabled;
/// new ids get a node (up to kMaxNodes ever). In a motion build this does not
/// touch the motion allow-list, and a new node is read-only unless it is in
/// SYSID_MOTION_NODE_IDS.
void set_polled_nodes(const std::string &list) {
  const std::vector<uint8_t> ids = parse_node_list(list.c_str());
  if (ids.empty()) {
    emit_event("error: nodes <id,id,...> (ids 1..127)");
    return;
  }
  const NodeSet wanted = to_node_set(ids);
  const size_t n = g_node_count.load();
  for (size_t i = 0; i < n; ++i) {
    g_nodes[i]->set_enabled(wanted.test(g_nodes[i]->id()));
  }
  std::string not_added;
  for (uint8_t id : ids) {
    if (find_node(id) != nullptr) {
      continue;
    }
    if (g_nodes.size() >= kMaxNodes) {
      not_added += " " + std::to_string(id);
      continue;
    }
    g_nodes.push_back(std::make_unique<McpNode>(id, send_frame, g_period_ms));
    g_node_count = g_nodes.size(); // publish the complete node, so replies reach it...
    g_nodes.back()->start();       // ...before it sends its first request
  }
  emit_event(fmt::format("nodes polled: {}{}", list_text(wanted),
                         not_added.empty()
                             ? std::string()
                             : fmt::format("; not added (at most {} nodes ever):{}", kMaxNodes,
                                           not_added)));
  // A different column set may not fit the console at the present rate.
  const int link_max = max_rate_for_link(polled_count());
  if (1000 / g_period_ms.load() > link_max) {
    g_period_ms = std::max(1, static_cast<int>(std::lround(1000.0 / link_max)));
    emit_event(fmt::format("rate lowered to {:.3f} Hz to fit the console link",
                           1000.0 / g_period_ms.load()));
  }
  g_header_requested = true;
}

/// `scan`: read 0x1000 (device type) from every id 1..127 with a short timeout,
/// and the device name from each that answers. SDO uploads only, through
/// send_frame(), so it is subject to the bus policy like everything else. A node
/// already polled is asked through its own client, so the two never overlap.
void scan_bus() {
  emit_event("scan start: SDO upload of 0x1000 to ids 1..127");
  int found = 0;
  for (uint8_t id = 1; id <= 127; ++id) {
    std::error_code ec;
    uint32_t device_type = 0;
    std::string name;
    std::error_code name_ec;
    if (McpNode *node = find_node(id); node != nullptr) {
      device_type = node->client().read_u32(0x1000, 0, ec);
      if (!ec) {
        name = node->client().read_string(0x1008, 0, name_ec);
      }
    } else {
      espp::CanopenClient probe({
          .node_id = id,
          .send = send_frame,
          .sdo_timeout = std::chrono::milliseconds(10),
          .log_level = espp::Logger::Verbosity::NONE,
      });
      {
        std::lock_guard<std::mutex> lock(g_scan_mutex);
        g_scan_client = &probe;
      }
      device_type = probe.read_u32(0x1000, 0, ec);
      if (!ec) {
        name = probe.read_string(0x1008, 0, name_ec);
      }
      std::lock_guard<std::mutex> lock(g_scan_mutex);
      g_scan_client = nullptr;
    }
    if (ec) {
      continue;
    }
    ++found;
    emit_line(fmt::format("#cfg,{},device_type,0x1000,0,0x{:08X}={}", id, device_type,
                          static_cast<int32_t>(device_type)));
    for (char &c : name) {
      if (c == ',') {
        c = ';';
      }
    }
    emit_line(fmt::format("#cfg,{},device_name,0x1008,0,{}", id,
                          name_ec ? "error:" + name_ec.message() : name));
  }
  emit_event(fmt::format("scan done: {} node(s) answered", found));
}

const char *kUsage = "hdr, rate <hz>, nodes [id,id,...], scan, stop, mark <text>, dump <node>"
#if CONFIG_SYSID_ENABLE_MOTION
                     ", arm <node> <ch>, clamp <node> <ch> <min> <max>, "
                     "pstep <node> <ch> <delta_counts>, clearfaults <node>"
#endif
    ;

void handle_command(const std::string &line) {
  emit_event("cmd " + line);
  std::istringstream in(line);
  std::string word;
  in >> word;
  if (word.empty()) {
    return;
  }
  if (word == "hdr") {
    g_header_requested = true;
  } else if (word == "rate") {
    int hz = 0;
    if (!(in >> hz) || hz < 1 || hz > kMaxRateHz) {
      emit_event(fmt::format("error: rate takes 1..{} Hz", kMaxRateHz));
      return;
    }
    // The link budget: refuse a rate whose worst-case rows would take more than
    // 70 % of the console's bytes per second.
    const int link_max = max_rate_for_link(polled_count());
    if (hz > link_max) {
      emit_event(fmt::format("error: rate {} Hz refused: worst-case rows of {} bytes need {} B/s, "
                             "over 70 % of the console link; at most {} Hz here",
                             hz, worst_case_row_bytes(polled_count()),
                             hz * worst_case_row_bytes(polled_count()), link_max));
      return;
    }
    g_period_ms = std::max(1, static_cast<int>(std::lround(1000.0 / hz)));
    emit_event(fmt::format("rate set: period {} ms ({:.3f} Hz)", g_period_ms.load(),
                           1000.0 / g_period_ms.load()));
    g_header_requested = true;
  } else if (word == "nodes") {
    std::string list;
    in >> list;
    if (!list.empty()) {
      set_polled_nodes(list);
    }
    report_nodes();
  } else if (word == "scan") {
    scan_bus();
  } else if (word == "stop") {
    stop_everything();
  } else if (word == "mark") {
    std::string text;
    std::getline(in, text);
    const size_t start = text.find_first_not_of(' ');
    emit_event("mark " + (start == std::string::npos ? std::string() : text.substr(start)));
  } else if (word == "dump") {
    int id = 0;
    McpNode *node = (in >> id) ? find_node(id) : nullptr;
    if (node == nullptr) {
      emit_event("error: dump <node>, one of #meta:node_ids");
      return;
    }
    dump_node(*node);
  } else if (word == "sdor") {
    // Read-only: one SDO upload of any object, tried as 4, 2 then 1 byte(s).
    int node_id = 0, sub = 0;
    std::string index_text;
    if (!(in >> node_id >> index_text >> sub)) {
      emit_event("error: sdor <node> <index hex> <sub>");
      return;
    }
    McpNode *node = find_node(node_id);
    if (node == nullptr) {
      emit_event(fmt::format("error: node {} is not polled", node_id));
      return;
    }
    const uint16_t index = static_cast<uint16_t>(std::strtoul(index_text.c_str(), nullptr, 16));
    std::error_code ec;
    uint32_t value = node->client().read_u32(index, static_cast<uint8_t>(sub), ec);
    int size = 4;
    if (ec) {
      value = node->client().read_u16(index, static_cast<uint8_t>(sub), ec);
      size = 2;
    }
    if (ec) {
      value = node->client().read_u8(index, static_cast<uint8_t>(sub), ec);
      size = 1;
    }
    if (ec) {
      emit_line(fmt::format("#cfg,{},sdor,0x{:04X},{},error:{} abort:0x{:08X}", node_id, index, sub,
                            ec.message(), node->client().last_abort_code()));
    } else {
      emit_line(fmt::format("#cfg,{},sdor,0x{:04X},{},0x{:X}={} ({} byte)", node_id, index, sub, value,
                            static_cast<int32_t>(value), size));
    }
#if CONFIG_SYSID_ENABLE_MOTION
  } else if (word == "arm") {
    int node = 0, ch = 0;
    if (!(in >> node >> ch)) {
      emit_event("error: arm <node> <ch 1|2>");
      return;
    }
    g_motion->arm(node, ch);
  } else if (word == "pstep") {
    int node = 0, ch = 0, delta = 0, velocity = 0, acceleration = 0;
    if (!(in >> node >> ch >> delta)) {
      emit_event("error: pstep <node> <ch 1|2> <delta_counts> [velocity] [acceleration]");
      return;
    }
    in >> velocity >> acceleration; // optional; 0 = defaults
    g_motion->pstep(node, ch, delta, velocity, acceleration);
  } else if (word == "pstream") {
    int node = 0, ch = 0, delta = 0, rate = 0, period_ms = 20, handshake = 0, accel = 0, lead_ms = 0;
    if (!(in >> node >> ch >> delta >> rate) || rate <= 0) {
      emit_event("error: pstream <node> <ch 1|2> <delta_counts> <rate_counts_per_s> [period_ms] "
                 "[handshake 0|1]");
      return;
    }
    in >> period_ms >> handshake >> accel >> lead_ms; // optional
    g_motion->pstep(node, ch, delta, 20000, 0, rate, period_ms, handshake, accel, lead_ms);
  } else if (word == "hb") {
    int on = 0, delay_ms = 0;
    if (!(in >> on)) {
      emit_event("error: hb <0|1> [delay_ms]");
      return;
    }
    in >> delay_ms; // optional: stop after this long instead of now
    if (on == 0 && delay_ms > 0) {
      g_heartbeat_stop_at_us = now_us() + static_cast<int64_t>(delay_ms) * 1000;
      emit_event(fmt::format("heartbeat will stop in {} ms", delay_ms));
    } else {
      g_motion->set_heartbeat_for_test(on != 0);
      emit_event(fmt::format("heartbeat {} by hand", on ? "started" : "STOPPED"));
    }
  } else if (word == "sdow") {
    int node = 0, sub = 0, size = 0;
    long long value = 0;
    std::string index_text;
    if (!(in >> node >> index_text >> sub >> size >> value)) {
      emit_event("error: sdow <node> <index hex> <sub> <size 1|2|4> <value>");
      return;
    }
    g_motion->write_object(node, static_cast<uint16_t>(std::strtoul(index_text.c_str(), nullptr, 16)),
                           static_cast<uint8_t>(sub), size, static_cast<int32_t>(value));
  } else if (word == "vgains") {
    int node = 0, ch = 0;
    int32_t kp = 0, ki = 0, kd = 0, qpps = 0;
    if (!(in >> node >> ch >> kp >> ki >> kd >> qpps)) {
      emit_event("error: vgains <node> <ch 1|2> <P> <I> <D> <QPPS> (P, I, D 16.16 raw)");
      return;
    }
    g_motion->set_velocity_gains(node, ch, kp, ki, kd, qpps);
  } else if (word == "clamp") {
    int node = 0, ch = 0;
    int32_t min = 0, max = 0;
    if (!(in >> node >> ch >> min >> max)) {
      emit_event("error: clamp <node> <ch 1|2> <min> <max>");
      return;
    }
    g_motion->set_clamp(node, ch, min, max);
  } else if (word == "gains") {
    int node = 0, ch = 0;
    int32_t kp = 0, ki = 0, kd = 0, max_i = 0, deadband = 0;
    if (!(in >> node >> ch >> kp >> ki >> kd >> max_i >> deadband)) {
      emit_event("error: gains <node> <ch 1|2> <P> <I> <D> <MaxI> <deadband> (raw controller units)");
      return;
    }
    g_motion->set_gains(node, ch, kp, ki, kd, max_i, deadband);
  } else if (word == "clearfaults") {
    int node = 0;
    if (!(in >> node)) {
      emit_event("error: clearfaults <node>");
      return;
    }
    g_motion->clear_faults(node);
#endif
  } else {
    emit_event(std::string("error: unknown command (") + kUsage + ")");
  }
}

void console_task(void *) {
  std::string line;
  bool previous_was_cr = false;
  char buffer[64];
  for (;;) {
    const int n = read_console(buffer, sizeof(buffer));
    if (n == 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    for (int i = 0; i < n; ++i) {
      const char c = buffer[i];
      // The \n of a \r\n pair finishes the line the \r already ended; it is not a
      // new keystroke and must not trip the console-byte guard.
      if (c == '\n' && previous_was_cr) {
        previous_was_cr = false;
        continue;
      }
      previous_was_cr = (c == '\r');
#if CONFIG_SYSID_ENABLE_MOTION
      g_motion->console_byte();
#endif
      if (c == '\r' || c == '\n') {
        handle_command(line);
        line.clear();
      } else if (line.size() < 200 && c >= ' ' && c < 127) {
        line += c;
      }
    }
  }
}

#if CONFIG_SYSID_ENABLE_MOTION
/// The MIB's CANopen heartbeat (COB-ID 0x700 + node id, state operational), sent
/// only while a channel is armed: the armed controller's heartbeat consumer
/// watches it (motion.cpp). Nothing at boot.
void heartbeat_task(void *) {
  espp::CanopenClient::CanFrame beat{};
  beat.id = 0x700u + kHeartbeatNodeId;
  beat.dlc = 1;
  beat.data[0] = 0x05;
  for (;;) {
    // `hb 0 <delay_ms>`: a scheduled stop, so the watchdog can be tested during a
    // move without sending a console byte (which would itself stop the move).
    const int64_t stop_at = g_heartbeat_stop_at_us.load();
    if (stop_at != 0 && now_us() >= stop_at) {
      g_heartbeat_stop_at_us = 0;
      g_heartbeat_on = false;
      emit_event("heartbeat STOPPED (scheduled)");
    }
    if (g_heartbeat_on.load()) {
      send_frame(beat);
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SYSID_HEARTBEAT_PERIOD_MS));
  }
}
#endif

// ------------------------------------------------------------------ bring-up

bool start_can() {
  g_twai = std::make_unique<espp::Twai>(espp::Twai::Config{
      .tx_gpio = CONFIG_SYSID_CAN_TX_GPIO,
      .rx_gpio = CONFIG_SYSID_CAN_RX_GPIO,
      .baudrate = CONFIG_SYSID_CAN_BITRATE,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 8,
      // Retries before a frame is reported failed. It was 3, and on the chair's bus
      // that failed dozens of SDO requests a minute ("tx failed ... I/O error", mostly
      // to node 2): the driver counts a lost arbitration as a failed attempt, and
      // with two nodes answering polls, three in a row is not rare. The completion timeout still gives
      // up quickly on a bus where nothing acknowledges.
      .tx_retry_count = -1, // retransmit until sent: 0 unsent frames in 31 s on the bench (3 and 15: ~1.6/s)
      .on_receive =
          [](const espp::Twai::Message &m) {
            const espp::CanopenClient::CanFrame frame{
                .id = m.id, .extended = m.extended, .rtr = m.rtr, .dlc = m.dlc, .data = m.data};
            // Each client keeps only its own node's frames.
            const size_t n = g_node_count.load();
            for (size_t i = 0; i < n; ++i) {
              g_nodes[i]->process_frame(frame);
            }
            std::lock_guard<std::mutex> lock(g_scan_mutex);
            if (g_scan_client != nullptr) {
              g_scan_client->process_frame(frame);
            }
          },
      .on_error =
          [](twai_error_flags_t flags) {
            g_can_errors++;
            if (flags.arb_lost) {
              g_can_arb_lost++;
            }
            if (flags.bit_err) {
              g_can_bit++;
            }
            if (flags.form_err) {
              g_can_form++;
            }
            if (flags.stuff_err) {
              g_can_stuff++;
            }
            if (flags.ack_err) {
              g_can_ack++;
            }
          },
      .on_state_change =
          [](const espp::Twai::StateChange &change) {
            g_can_bus_ok = (change.new_state == TWAI_ERROR_ACTIVE);
            if (change.new_state == TWAI_ERROR_BUS_OFF) {
              g_can_bus_off++;
            }
            emit_event(fmt::format("can state {} -> {} (0 active, 1 warning, 2 passive, 3 bus-off)",
                                   static_cast<int>(change.old_state),
                                   static_cast<int>(change.new_state)));
          },
      .log_level = espp::Logger::Verbosity::NONE,
  });
  std::error_code ec;
  if (!g_twai->initialize(ec)) {
    emit_event(fmt::format("can init failed on tx {} rx {}: {}", CONFIG_SYSID_CAN_TX_GPIO,
                           CONFIG_SYSID_CAN_RX_GPIO, ec.message()));
    return false;
  }
  g_can_bus_ok = true;
  emit_event(fmt::format("can up: tx {} rx {} {} bit/s", CONFIG_SYSID_CAN_TX_GPIO,
                         CONFIG_SYSID_CAN_RX_GPIO, CONFIG_SYSID_CAN_BITRATE));
  return true;
}

} // namespace
} // namespace sysid

extern "C" void app_main(void) {
  using namespace sysid;
  wire_init();

  // The effective read-only set handed to the transmit filter.
  //   passive: every node id, so the passive binary cannot write to any node even
  //            if a write call were added by mistake;
  //   motion:  SYSID_READONLY_NODE_IDS, plus every node not in
  //            SYSID_MOTION_NODE_IDS.
#if CONFIG_SYSID_ENABLE_MOTION
  g_motion_nodes = to_node_set(parse_node_list(CONFIG_SYSID_MOTION_NODE_IDS));
  g_read_only = to_node_set(parse_node_list(CONFIG_SYSID_READONLY_NODE_IDS));
  for (int id = 1; id < 128; ++id) {
    if (!g_motion_nodes.test(id)) {
      g_read_only.set(id);
    }
  }
#else
  for (int id = 1; id < 128; ++id) {
    g_read_only.set(id);
  }
#endif
  // First lines out: which binary this is, and which nodes it will never write.
  emit_mode();
  emit_event("boot: actuator_sysid " + std::string(esp_app_get_description()->version) + " (" +
             kMode + ")");
#if CONFIG_SYSID_ENABLE_MOTION
  if ((g_motion_nodes & g_read_only).any()) {
    emit_event("motion node list overlaps SYSID_READONLY_NODE_IDS: read-only wins for " +
               list_text(g_motion_nodes & g_read_only));
  }
#endif

  // Nodes before the bus. g_nodes never reallocates (reserved), so the TWAI
  // receive task can walk its first g_node_count entries while `nodes` appends.
  g_nodes.reserve(kMaxNodes);
  for (uint8_t id : parse_node_list(CONFIG_SYSID_NODE_IDS, kMaxNodes)) {
    g_nodes.push_back(std::make_unique<McpNode>(id, send_frame, g_period_ms));
  }
  g_node_count = g_nodes.size();
  if (g_nodes.empty()) {
    emit_event("no valid node ids in CONFIG_SYSID_NODE_IDS: logging the IMU only");
  }
#if CONFIG_SYSID_ENABLE_MOTION
  g_motion = std::make_unique<Motion>(g_nodes, g_node_count, g_period_ms, g_motion_nodes,
                                      g_read_only,
                                      [](bool on) { g_heartbeat_on = on; });
#endif

  // The boot rate must fit the link too (e.g. CONFIG_SYSID_SAMPLE_RATE_HZ=100 with
  // a 460800 baud console).
  if (const int link_max = max_rate_for_link(polled_count());
      1000 / g_period_ms.load() > link_max) {
    g_period_ms = std::max(1, static_cast<int>(std::lround(1000.0 / link_max)));
    emit_event(fmt::format("boot rate {} Hz exceeds 70 % of the console link; using {:.3f} Hz",
                           CONFIG_SYSID_SAMPLE_RATE_HZ, 1000.0 / g_period_ms.load()));
  }

  // The header next: a host that just opened the port (which may reset the
  // board) sees the columns before any row.
  emit_header();

  // Board: the bsp's init() is what sets up the board object init_imu() uses; it
  // also starts Ethernet as the usual DHCP server (harmless here, and the robot
  // LAN behaves as it does under the main firmware).
  auto &mib = mib::bsp::MIB::instance();
  if (!mib.init()) {
    emit_event("ethernet init failed (not needed for logging)");
  }
  if (mib.init_imu()) {
    g_imu = mib.imu();
    g_imu->set_log_level(espp::Logger::Verbosity::ERROR);
  } else {
    emit_event("imu init failed: IMU columns stay empty");
  }
  // From here on espp's own log lines would land between rows; keep them to errors.
  mib.set_log_level(espp::Logger::Verbosity::ERROR);
  mib.board().set_log_level(espp::Logger::Verbosity::ERROR);

  // The bus: from here the poll tasks send SDO upload requests, and nothing else
  // goes out until an operator command (motion build only).
  if (start_can()) {
    for (auto &node : g_nodes) {
      node->start();
    }
#if CONFIG_SYSID_ENABLE_MOTION
    xTaskCreate(heartbeat_task, "heartbeat", 3 * 1024, nullptr, 5, nullptr);
#endif
  }

  // Whatever the reset left in the receive buffer is not a command.
  char discard[64];
  while (read_console(discard, sizeof(discard)) > 0) {
  }
  xTaskCreate(console_task, "console", 6 * 1024, nullptr, 4, nullptr);
  xTaskCreate(sample_task, "sample", 8 * 1024, nullptr, 7, nullptr);
}
