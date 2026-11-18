#include "mcp_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "format.hpp"
#include "wire.hpp"

namespace sysid {

namespace {

// Basicmicro manufacturer objects: 0x2000 + the packet-serial command number.
constexpr uint16_t kMotorPwmsObject = 0x2030;     // command 48: M1, M2 duty, +/-32767
constexpr uint16_t kMotorCurrentsObject = 0x2031; // command 49: M1, M2 current, 10 mA units
constexpr uint16_t kStatusFlagsObject = 0x205A;   // command 90: status bit mask

// SDO abort codes meaning "this object is not in the dictionary": the field is
// left empty from then on instead of being asked for every pass.
constexpr uint32_t kAbortNoObject = 0x06020000;
constexpr uint32_t kAbortNoSubindex = 0x06090011;

// How long an offline node waits between attempts to reach it.
constexpr int kOfflineRetryMs = 200;

// The Basicmicro command-90 status bits, from the MCP manual (as the teammate's
// `flags` console command decodes them). Unverified on this firmware; the bench
// found the word reading 0 throughout.
struct FlagName {
  uint32_t bit;
  const char *name;
};
constexpr FlagName kFlagNames[] = {
    {0x00000001, "E-stop"},
    {0x00000002, "temperature error"},
    {0x00000004, "temperature 2 error"},
    {0x00000008, "main voltage high error"},
    {0x00000010, "logic voltage high error"},
    {0x00000020, "logic voltage low error"},
    {0x00000040, "M1 driver fault"},
    {0x00000080, "M2 driver fault"},
    {0x00000100, "M1 speed error"},
    {0x00000200, "M2 speed error"},
    {0x00000400, "M1 position error"},
    {0x00000800, "M2 position error"},
    {0x00001000, "M1 current error"},
    {0x00002000, "M2 current error"},
    {0x00010000, "M1 over-current warning"},
    {0x00020000, "M2 over-current warning"},
    {0x00040000, "main voltage high warning"},
    {0x00080000, "main voltage low warning"},
    {0x00100000, "temperature warning"},
    {0x00200000, "temperature 2 warning"},
    {0x00400000, "S4 signal triggered"},
    {0x00800000, "S5 signal triggered"},
    {0x01000000, "speed error limit warning"},
    {0x02000000, "position error limit warning"},
};

constexpr size_t index_of(auto item) { return static_cast<size_t>(item); }

} // namespace

bool limit_bits(const NodeData &node, int ch, int32_t &bits) {
  const Field &status = node.ch[ch].status;
  if (!status.valid) {
    return false;
  }
  bits = 0;
  if (status.value & kStatuswordInternalLimit) {
    bits |= kLimInternalLimit;
  }
  if (node.flags.valid) {
    const uint32_t flags = static_cast<uint32_t>(node.flags.value);
    if (flags & kFlagS4Triggered) {
      bits |= kLimS4;
    }
    if (flags & kFlagS5Triggered) {
      bits |= kLimS5;
    }
  }
  return true;
}

std::string output_readback(uint8_t node_id, const NodeData &node, int ch, int64_t t_us) {
  const ChannelData &c = node.ch[ch];
  const bool online = node.link == Link::ONLINE;
  auto value = [&](const Field &f) {
    return (online && f.valid) ? std::to_string(f.value) : std::string("?");
  };
  std::string text = fmt::format("n{}c{} duty={} cur_ma={} status={}", node_id, ch + 1,
                                 value(c.duty), value(c.cur_ma),
                                 (online && c.status.valid)
                                     ? fmt::format("0x{:04X}", static_cast<uint16_t>(c.status.value))
                                     : std::string("?"));
  if (online && c.duty.valid) {
    text += fmt::format(" ({} ms old)", (t_us - c.duty.t_us) / 1000);
  }
  return text;
}

McpNode::McpNode(uint8_t node_id, espp::CanopenClient::send_fn send,
                 const std::atomic<int> &period_ms)
    : node_id_(node_id)
    , period_ms_(period_ms)
    , client_({
          .node_id = node_id,
          .send =
              [this, send = std::move(send)](const espp::CanopenClient::CanFrame &frame) {
                note_request(frame);
                return send(frame);
              },
          .sdo_timeout = std::chrono::milliseconds(CONFIG_SYSID_SDO_TIMEOUT_MS),
          .on_heartbeat =
              [this](uint8_t id, espp::CanopenClient::NmtState state) { on_heartbeat(id, state); },
          // Silent: a timeout is normal for a missing node, and is counted in the
          // poll statistics instead of printed on the data channel.
          .log_level = espp::Logger::Verbosity::NONE,
      })
    , mcp_(client_, espp::Mcp266::Config{.log_level = espp::Logger::Verbosity::NONE}) {}

void McpNode::start() {
  const std::string name = "poll n" + std::to_string(node_id_);
  xTaskCreate([](void *self) { static_cast<McpNode *>(self)->task(); }, name.c_str(), 6 * 1024,
              this, 6, nullptr);
}

NodeData McpNode::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return data_;
}

PollStats McpNode::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

std::string McpNode::device_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return device_name_;
}

void McpNode::task() {
  size_t slow_index = 0;
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    if (!enabled_) {
      // Not polled (`nodes <list>`): nothing on the bus, nothing printed.
      clear_all();
      failed_passes_in_row_ = 0;
      vTaskDelay(pdMS_TO_TICKS(100));
      last_wake = xTaskGetTickCount();
      continue;
    }
    const int64_t start = now_us();
    const bool completed = run_pass(slow_index);
    const int64_t duration = now_us() - start;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (completed) {
        stats_.passes++;
        stats_.pass_us_sum += duration;
        stats_.pass_us_max = std::max(stats_.pass_us_max, duration);
      } else {
        stats_.failed_passes++;
      }
    }

    if (completed) {
      failed_passes_in_row_ = 0;
      set_link(Link::ONLINE);
      if (!have_device_name_) {
        // Once, for #meta and the `nodes` command: which controller this is.
        std::string name;
        uint32_t type = 0;
        std::error_code ec;
        have_device_name_ = true;
        if (mcp_.read_device_info(name, type, ec)) {
          std::lock_guard<std::mutex> lock(mutex_);
          device_name_ = fmt::format("{} (device type 0x{:08X})", name, type);
        }
      }
    } else if (++failed_passes_in_row_ >= CONFIG_SYSID_OFFLINE_AFTER_FAILED_PASSES) {
      set_link(Link::OFFLINE);
    }

    // Pace: a pass starts at most once per sample period. An offline node is
    // retried slowly so it does not load the bus with requests nobody answers.
    if (snapshot().link == Link::OFFLINE) {
      vTaskDelay(pdMS_TO_TICKS(kOfflineRetryMs));
      last_wake = xTaskGetTickCount();
    } else if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms_.load())) == pdFALSE) {
      // The pass took longer than the period: start the next one now, and stop
      // counting from a wake time in the past.
      last_wake = xTaskGetTickCount();
    }
  }
}

bool McpNode::run_pass(size_t &slow_index) {
  for (Item item : kFastItems) {
    if (read_item(item) == Result::NO_ANSWER) {
      return false;
    }
  }
  // One slow field per pass, skipping any the controller does not have.
  for (size_t tries = 0; tries < std::size(kSlowItems); ++tries) {
    const Item item = kSlowItems[slow_index];
    slow_index = (slow_index + 1) % std::size(kSlowItems);
    if (unsupported_[index_of(item)]) {
      continue;
    }
    if (read_item(item) == Result::NO_ANSWER) {
      return false;
    }
    break;
  }
  return true;
}

bool McpNode::read_mirror(uint16_t index, uint8_t subindex, bool is_signed, int32_t &value,
                          std::error_code &ec) {
  // The width is the Basicmicro manual's for the packet-serial reply: 2 bytes per
  // motor for currents and duties, 4 for the status word. A controller that
  // sends more (or does not state the size) has the value in the low bytes.
  const size_t width = (index == kStatusFlagsObject) ? 4 : 2;
  std::array<uint8_t, 4> buffer{};
  const size_t n = client_.sdo_upload(index, subindex, buffer, ec);
  if (n == 0) {
    return false;
  }
  const size_t used = std::min(n, width);
  uint32_t raw = 0;
  for (size_t i = 0; i < used; ++i) {
    raw |= static_cast<uint32_t>(buffer[i]) << (8 * i);
  }
  if (is_signed && used < 4 && (raw & (1u << (8 * used - 1)))) {
    raw |= ~((1u << (8 * used)) - 1); // sign-extend
  }
  value = static_cast<int32_t>(raw);
  return true;
}

McpNode::Result McpNode::read_item(Item item) {
  if (unsupported_[index_of(item)]) {
    return Result::REFUSED;
  }
  std::error_code ec;
  int32_t value = 0;
  const int64_t t0 = now_us();
  switch (item) {
  case Item::ENC_M1:
    mcp_.read_encoder(Axis::M1, value, ec);
    break;
  case Item::ENC_M2:
    mcp_.read_encoder(Axis::M2, value, ec);
    break;
  case Item::SPD_M1:
    mcp_.read_speed(Axis::M1, value, ec);
    break;
  case Item::SPD_M2:
    mcp_.read_speed(Axis::M2, value, ec);
    break;
  case Item::CUR_M1:
  case Item::CUR_M2: {
    int32_t tens_of_ma = 0;
    if (read_mirror(kMotorCurrentsObject, item == Item::CUR_M1 ? 1 : 2, true, tens_of_ma, ec)) {
      value = tens_of_ma * 10;
    }
    break;
  }
  case Item::DUTY_M1:
  case Item::DUTY_M2:
    read_mirror(kMotorPwmsObject, item == Item::DUTY_M1 ? 1 : 2, true, value, ec);
    break;
  case Item::STATUS_M1:
  case Item::STATUS_M2: {
    uint16_t statusword = 0;
    if (mcp_.read_statusword(item == Item::STATUS_M1 ? Axis::M1 : Axis::M2, statusword, ec)) {
      value = statusword;
    }
    break;
  }
  case Item::FLAGS:
    read_mirror(kStatusFlagsObject, 0, false, value, ec);
    break;
  case Item::VBAT: {
    float volts = 0.0f;
    if (mcp_.read_main_battery_voltage(volts, ec)) {
      value = static_cast<int32_t>(std::lround(volts * 1000.0f));
    }
    break;
  }
  case Item::TEMP: {
    float celsius = 0.0f;
    if (mcp_.read_temperature(celsius, ec)) {
      value = static_cast<int32_t>(std::lround(celsius * 10.0f));
    }
    break;
  }
  case Item::COUNT:
    return Result::REFUSED;
  }
  const int64_t t1 = now_us();

  if (!ec) {
    store(item, value, (t0 + t1) / 2);
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.sdo_us_max = std::max(stats_.sdo_us_max, t1 - t0);
    return Result::OK;
  }

  invalidate(item);
  if (ec == std::errc::protocol_error) {
    // The controller answered: an SDO abort, or a reply of the wrong size.
    const uint32_t abort_code = client_.last_abort_code();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.aborts++;
    }
    if (abort_code == kAbortNoObject || abort_code == kAbortNoSubindex) {
      unsupported_[index_of(item)] = true;
      emit_event(fmt::format("n{} {} not readable (SDO abort 0x{:08X}): column stays empty",
                             node_id_, item_name(item), abort_code));
    }
    return Result::REFUSED;
  }
  // No answer in time, or the request could not be sent.
  timeouts_++;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.timeouts++;
  }
  return Result::NO_ANSWER;
}

Field &McpNode::field_for(Item item) {
  switch (item) {
  case Item::ENC_M1: return data_.ch[0].enc;
  case Item::SPD_M1: return data_.ch[0].spd;
  case Item::CUR_M1: return data_.ch[0].cur_ma;
  case Item::DUTY_M1: return data_.ch[0].duty;
  case Item::STATUS_M1: return data_.ch[0].status;
  case Item::ENC_M2: return data_.ch[1].enc;
  case Item::SPD_M2: return data_.ch[1].spd;
  case Item::CUR_M2: return data_.ch[1].cur_ma;
  case Item::DUTY_M2: return data_.ch[1].duty;
  case Item::STATUS_M2: return data_.ch[1].status;
  case Item::FLAGS: return data_.flags;
  case Item::VBAT: return data_.vbat_mv;
  case Item::TEMP: return data_.temp_dc;
  case Item::COUNT: break;
  }
  return data_.flags; // unreachable
}

void McpNode::store(Item item, int32_t value, int64_t t_us) {
  int32_t old_value = 0;
  bool old_valid = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Field &field = field_for(item);
    old_value = field.value;
    old_valid = field.valid;
    field = {.valid = true, .value = value, .t_us = t_us};
    if (item == Item::ENC_M1 || item == Item::ENC_M2) {
      const int ch = item == Item::ENC_M1 ? 0 : 1;
      enc_history_[ch][enc_history_next_[ch]] = {value, t_us};
      enc_history_next_[ch] = (enc_history_next_[ch] + 1) % kEncoderWindow;
      enc_history_len_[ch] = std::min(enc_history_len_[ch] + 1, kEncoderWindow);
    }
  }
  report_changes(item, old_value, old_valid, value);
}

void McpNode::invalidate(Item item) {
  std::lock_guard<std::mutex> lock(mutex_);
  field_for(item).valid = false;
}

void McpNode::set_link(Link link) {
  Link previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous = data_.link;
    if (previous == link) {
      return;
    }
    data_.link = link;
    if (link == Link::OFFLINE) {
      // Nothing from a silent node is printed, however recent.
      data_ = NodeData{};
      data_.link = Link::OFFLINE;
      enc_history_len_ = {};
    }
  }
  if (link == Link::ONLINE) {
    emit_event(fmt::format("n{} online", node_id_));
  } else if (link == Link::OFFLINE) {
    emit_event(fmt::format("n{} offline: no answer in {} passes (SDO timeout {} ms); fields empty",
                           node_id_, CONFIG_SYSID_OFFLINE_AFTER_FAILED_PASSES,
                           CONFIG_SYSID_SDO_TIMEOUT_MS));
  }
}

void McpNode::on_heartbeat(uint8_t node_id, espp::CanopenClient::NmtState state) {
  // Every client sees every node's heartbeats; report only our own, on change.
  if (node_id != node_id_ || state == last_nmt_state_) {
    return;
  }
  last_nmt_state_ = state;
  using S = espp::CanopenClient::NmtState;
  switch (state) {
  case S::BootUp:
    // A boot-up message mid-run means the controller reset: it is back on its
    // EEPROM settings (position clamp [0, 0], see the Actuator DESIGN.md).
    emit_event(fmt::format("n{} boot-up message: controller reset", node_id_));
    break;
  case S::Operational:
    emit_event(fmt::format("n{} NMT operational", node_id_));
    break;
  case S::PreOperational:
    emit_event(fmt::format("n{} NMT pre-operational", node_id_));
    break;
  case S::Stopped:
    emit_event(fmt::format("n{} NMT stopped", node_id_));
    break;
  default:
    emit_event(fmt::format("n{} NMT state 0x{:02X}", node_id_, static_cast<unsigned>(state)));
    break;
  }
}

void McpNode::report_changes(Item item, int32_t old_value, bool old_valid, int32_t new_value) {
  if (item == Item::FLAGS) {
    if (old_valid && old_value == new_value) {
      return;
    }
    if (!old_valid && new_value == 0) {
      return; // first reading, nothing set
    }
    const uint32_t flags = static_cast<uint32_t>(new_value);
    std::string text = fmt::format("n{} flags 0x{:08X}", node_id_, flags);
    for (const auto &f : kFlagNames) {
      if (flags & f.bit) {
        text += fmt::format(" [{}]", f.name);
      }
    }
    if (flags == 0) {
      text += " (clear)";
    }
    emit_event(text);
    return;
  }
  if (item == Item::STATUS_M1 || item == Item::STATUS_M2) {
    const int ch = (item == Item::STATUS_M1) ? 1 : 2;
    const uint16_t before = old_valid ? static_cast<uint16_t>(old_value) : 0;
    const uint16_t now = static_cast<uint16_t>(new_value);
    if ((now ^ before) & kStatuswordFault) {
      emit_event(fmt::format("n{}c{} {}: statusword 0x{:04X}", node_id_, ch,
                             (now & kStatuswordFault) ? "fault" : "fault cleared", now));
    }
    if ((now ^ before) & kStatuswordInternalLimit) {
      emit_event(fmt::format("n{}c{} limit {}: statusword 0x{:04X}", node_id_, ch,
                             (now & kStatuswordInternalLimit) ? "active" : "clear", now));
    }
  }
}

void McpNode::clear_all() {
  std::lock_guard<std::mutex> lock(mutex_);
  data_ = NodeData{};
  enc_history_len_ = {};
}

bool McpNode::read_position_record(int ch, std::array<int32_t, 7> &record, std::error_code &ec) {
  const auto objects = ch == 0 ? espp::detail::mcp266::axis_m1() : espp::detail::mcp266::axis_m2();
  // In order, from sub 1: the controller appears to run the mirrored packet-serial
  // command (63 / 64) when sub 1 is read and to serve subs 2..7 from that reply.
  // Retried as a whole, from sub 1 each time: SDO transmits from the console task
  // fail sporadically on the bench (I/O error, cause not yet found), and a retry
  // of one sub alone might not be served from the same reply.
  for (int attempt = 0; attempt < 5; ++attempt) {
    ec.clear();
    for (uint8_t sub = 1; sub <= 7 && !ec; ++sub) {
      record[sub - 1] = client_.read_i32(objects.position_pid_get, sub, ec);
    }
    if (!ec) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  return false;
}

bool McpNode::read_clamp_checked(int ch, int32_t &clamp_min, int32_t &clamp_max,
                                 std::string &why) {
  std::array<int32_t, 7> first{}, second{};
  std::error_code ec;
  if (!read_position_record(ch, first, ec) || !read_position_record(ch, second, ec)) {
    why = "position PID record unreadable: " + ec.message();
    return false;
  }
  // Self-check: two independent reads must agree on the clamp.
  if (first[5] != second[5] || first[6] != second[6]) {
    why = fmt::format("clamp read twice disagrees: [{}, {}] then [{}, {}]", first[5], first[6],
                      second[5], second[6]);
    return false;
  }
  clamp_min = first[5];
  clamp_max = first[6];
  return true;
}

bool McpNode::settled_count(int ch, int64_t now, int32_t &median, int32_t &spread) const {
  std::array<int32_t, kEncoderWindow> values{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.link != Link::ONLINE || enc_history_len_[ch] < kEncoderWindow) {
      return false;
    }
    const int64_t max_age_us =
        static_cast<int64_t>(3 * kEncoderWindow) * period_ms_.load() * 1000;
    for (size_t i = 0; i < kEncoderWindow; ++i) {
      if (now - enc_history_[ch][i].t_us > max_age_us) {
        return false;
      }
      values[i] = enc_history_[ch][i].value;
    }
  }
  std::sort(values.begin(), values.end());
  median = values[kEncoderWindow / 2];
  spread = values.back() - values.front();
  return true;
}

void McpNode::note_request(const espp::CanopenClient::CanFrame &frame) {
  if (frame.id != 0x600u + node_id_ || frame.dlc < 4) {
    return;
  }
  sdo_requests_++;
  const uint8_t command = frame.data[0] & 0xE0;
  if (command == 0x60) { // upload segment request: the reply has no address
    expect_segment_ = true;
    return;
  }
  expect_segment_ = false;
  expected_address_ = (static_cast<uint32_t>(frame.data[1] | (frame.data[2] << 8)) << 8) |
                      frame.data[3];
}

void McpNode::note_reply(const espp::CanopenClient::CanFrame &frame) {
  if (frame.id != 0x580u + node_id_ || frame.dlc < 4) {
    return;
  }
  sdo_replies_++;
  const uint8_t command = frame.data[0] & 0xE0;
  if (command == 0x00) { // upload segment response
    if (!expect_segment_) {
      sdo_mismatched_++;
    }
    return;
  }
  const uint32_t address =
      (static_cast<uint32_t>(frame.data[1] | (frame.data[2] << 8)) << 8) | frame.data[3];
  if (expect_segment_ || address != expected_address_.load()) {
    sdo_mismatched_++;
  }
}

const char *McpNode::item_name(Item item) {
  switch (item) {
  case Item::ENC_M1: return "c1 enc (0x6064)";
  case Item::SPD_M1: return "c1 spd (0x606C)";
  case Item::CUR_M1: return "c1 cur (0x2031:1)";
  case Item::DUTY_M1: return "c1 duty (0x2030:1)";
  case Item::STATUS_M1: return "c1 status (0x6041)";
  case Item::ENC_M2: return "c2 enc (0x6864)";
  case Item::SPD_M2: return "c2 spd (0x686C)";
  case Item::CUR_M2: return "c2 cur (0x2031:2)";
  case Item::DUTY_M2: return "c2 duty (0x2030:2)";
  case Item::STATUS_M2: return "c2 status (0x6841)";
  case Item::FLAGS: return "flags (0x205A)";
  case Item::VBAT: return "vbat (0x2018)";
  case Item::TEMP: return "temp (0x2052)";
  case Item::COUNT: break;
  }
  return "?";
}

} // namespace sysid
