#include "motion.hpp"

#if CONFIG_SYSID_ENABLE_MOTION

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "freertos/task.h"

#include "format.hpp"
#include "wire.hpp"

namespace sysid {

namespace {

constexpr int kStalePeriods = 3;   // feedback older than this many sample periods is stale
constexpr int kSlowRingLength = 5; // statusword M1/M2, flags, vbat, temp (mcp_node.hpp)

// Stop sequence (run_stop()). The hold is the teammate's bench recovery for a live
// loop: a CiA 402 profile-position move to the current count, then this wait,
// then disable voltage.
constexpr int kHoldMs = 2000;
constexpr bool kHoldBeforeDisable = false; // see run_stop()
constexpr int kDisableAttempts = 8;        // disable-voltage write attempts per channel
// The MIB heartbeat + the controller's heartbeat consumer (0x1016). Bench,
// 2026-11-17: the MCP266 does act on it (a lapse faults an enabled drive within
// ~0.5 s, flag 0x10000000), but writing 0 to 0x1016:1 did not stop it watching, so
// every disarm faulted the drive, and SDO transmits from the console task failed
// (I/O error) about half the time while the heartbeat task was running. Off until
// that is understood; the controller-side position clamp bounds a move meanwhile.
constexpr bool kUseHeartbeat = true; // back on 2026-11-17 evening: the I/O errors were unsent frames, not this
// CiA 402 controlword "disable voltage": every state goes to SWITCH ON DISABLED,
// power stage off. On the bench it took PWM and current to 0 after a move.
constexpr uint16_t kControlwordDisableVoltage = 0x0000;
// After writing the mode of operation, espp's Mcp266::enable() waits this long
// before reading the state (the MCP does not echo the mode in 0x6061).
constexpr int kModeSettleMs = 25;

// CiA 301 heartbeat consumer: subindex 1 of 0x1016 holds (producer node id << 16) | timeout ms.
constexpr uint16_t kHeartbeatConsumerObject = 0x1016;

/// SDO download / upload with a few retries (see McpNode::read_position_record()).
/// Only for writes that are idempotent: the same value to the same object.
template <typename Client>
bool write_i32_retry(Client &client, uint16_t index, uint8_t sub, int32_t value,
                     std::error_code &ec) {
  for (int attempt = 0; attempt < 5; ++attempt) {
    ec.clear();
    if (client.write_i32(index, sub, value, ec)) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  return false;
}
template <typename Client>
int32_t read_i32_retry(Client &client, uint16_t index, uint8_t sub, std::error_code &ec) {
  int32_t value = 0;
  for (int attempt = 0; attempt < 5; ++attempt) {
    ec.clear();
    value = client.read_i32(index, sub, ec);
    if (!ec) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  return value;
}

using Axis = espp::Mcp266::Axis;
Axis axis_of(int ch) { return ch == 0 ? Axis::M1 : Axis::M2; }

} // namespace

Motion::Motion(std::vector<std::unique_ptr<McpNode>> &nodes, const std::atomic<size_t> &node_count,
               const std::atomic<int> &period_ms,
               const NodeSet &motion_nodes, const NodeSet &read_only_nodes,
               std::function<void(bool)> set_heartbeat)
    : nodes_(nodes)
    , node_count_(node_count)
    , period_ms_(period_ms)
    , motion_nodes_(motion_nodes)
    , read_only_(read_only_nodes)
    , set_heartbeat_(std::move(set_heartbeat)) {
  queue_ = xQueueCreate(4, sizeof(Request));
  xTaskCreate([](void *self) { static_cast<Motion *>(self)->worker_task(); }, "motion", 6 * 1024,
              this, 8, nullptr);
}

size_t Motion::index_of(int node_id) const {
  for (size_t i = 0; i < node_count_.load(); ++i) {
    if (nodes_[i]->id() == node_id) {
      return i;
    }
  }
  return node_count_.load();
}

void Motion::refuse(const char *what, const std::string &why) {
  emit_event(fmt::format("{} refused: {}", what, why));
}

// ---------------------------------------------------------------------------------
// The move itself
// ---------------------------------------------------------------------------------

/// One CiA 402 profile-position move of one channel to an absolute target. Every
/// frame it sends is listed in the README ("Frames on the bus"). It is
/// espp::Mcp266::move_to_position() minus the automatic fault reset: that one, in
/// Mcp266::enable(), writes controlword 0x0000 then 0x0080 to a faulted drive.
/// Here Ds402Drive::enable_operation() refuses a faulted drive instead, and
/// `clearfaults` is the only way to reset one.
bool Motion::profile_move(McpNode &node, int ch, int32_t target, bool nmt_start,
                          std::string &error) {
  std::error_code ec;
  // NMT start, addressed to this node only: the state the teammate's moves ran
  // in (Mcp266::start()). Whether profile moves also work pre-operational is
  // untested.
  if (nmt_start && !node.client().nmt_start(ec)) {
    error = "NMT start: " + ec.message();
    return false;
  }
  // Mode of operation = profile position (0x6060 / 0x6860), written directly as
  // Mcp266::enable() does.
  const uint16_t mode_object = ch == 0 ? 0x6060 : 0x6860;
  if (!node.client().write_i8(
          mode_object, 0,
          static_cast<int8_t>(espp::Ds402Drive::OperatingMode::ProfilePosition), ec)) {
    error = "mode write: " + ec.message();
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(kModeSettleMs));
  // Target object (0x607A / 0x687A) BEFORE the drive is enabled. Whether the MCP266
  // closes its position loop on the old 0x607A the moment it reaches OPERATION
  // ENABLED, before any new-set-point handshake, is unknown; after a controller
  // power cycle that old value is 0, which the clamp would turn into a run to the
  // window's lower edge. With the real target already in the object, an early
  // loop closure can only go where this move is going anyway.
  const uint16_t target_object = ch == 0 ? 0x607A : 0x687A;
  if (!node.client().write_i32(target_object, 0, target, ec)) {
    error = "target pre-write: " + ec.message();
    return false;
  }
  espp::Ds402Drive *drive = node.mcp().drive(axis_of(ch));
  if (drive == nullptr) {
    error = "no drive";
    return false;
  }
  // Shutdown -> switch on -> enable operation, from wherever the drive is; refuses
  // (operation_not_permitted) if the drive is in fault.
  if (!drive->enable_operation(ec)) {
    error = "enable: " + ec.message();
    return false;
  }
  int32_t velocity = 0, acceleration = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity = move_velocity_;
    acceleration = move_acceleration_;
  }
  if (!drive->set_profile_acceleration(acceleration, ec) ||
      !drive->set_profile_deceleration(acceleration, ec) ||
      !drive->set_profile_velocity(velocity, ec)) {
    error = "profile: " + ec.message();
    return false;
  }
  // Target, then the new-set-point handshake (absolute, change immediately).
  if (!drive->set_target_position(target, ec, true, false)) {
    error = "target: " + ec.message();
    return false;
  }
  return true;
}

bool Motion::read_windows(McpNode &node, int ch, Windows &w, std::string &error) {
  // Present count: a median, never one sample (encoder noise at rest is 50..70
  // counts peak to peak, and glitches of ~1800 counts lasting ~100 ms were seen).
  if (!node.settled_count(ch, now_us(), w.present, w.spread)) {
    error = fmt::format("present count (fewer than {} fresh encoder reads)", kEncoderWindow);
    return false;
  }
  if (w.spread > CONFIG_SYSID_ENCODER_SETTLED_SPREAD_COUNTS) {
    error = fmt::format("present count (not settled: last {} reads spread {} counts, limit {})",
                        kEncoderWindow, w.spread, CONFIG_SYSID_ENCODER_SETTLED_SPREAD_COUNTS);
    return false;
  }
  std::string why;
  if (!node.read_clamp_checked(ch, w.clamp_min, w.clamp_max, why)) {
    error = "position clamp (" + why + ")";
    return false;
  }
  const uint16_t sw_limit = ch == 0 ? 0x607D : 0x687D;
  std::error_code ec;
  w.sw_min = read_i32_retry(node.client(), sw_limit, 1, ec);
  if (!ec) {
    w.sw_max = read_i32_retry(node.client(), sw_limit, 2, ec);
  }
  if (ec) {
    error = "software position limits: " + ec.message();
    return false;
  }
  return true;
}

std::string Motion::describe(const Windows &w) {
  return fmt::format("clamp [{}, {}], 0x607D [{}, {}], present {} (median of {}, spread {})",
                     w.clamp_min, w.clamp_max, w.sw_min, w.sw_max, w.present, kEncoderWindow,
                     w.spread);
}

// ---------------------------------------------------------------------------------
// Worker: moves and stops, in order
// ---------------------------------------------------------------------------------

void Motion::post(const Request &request) {
  // Never waits: called from the sample task, which must keep its period. A full
  // queue already holds stops that will run.
  if (xQueueSend(queue_, &request, 0) != pdTRUE) {
    emit_event("motion queue full; earlier requests still pending");
  }
}

void Motion::worker_task() {
  Request request;
  for (;;) {
    if (xQueueReceive(queue_, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (request.kind == Request::Kind::MOVE) {
      run_move(request);
    } else {
      run_stop(request);
    }
  }
}

void Motion::run_move(const Request &request) {
  McpNode &node = *nodes_[request.node_index];
  {
    // A stop or a guard may have ended the move before its turn came.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || node_index_ != request.node_index || ch_ != request.ch) {
      emit_event(fmt::format("pstep n{}c{} cancelled before any frame was sent", node.id(),
                             request.ch + 1));
      return;
    }
  }
  // Checked again here, right before the first frame, so no path reaches the bus
  // for a node that may not be written.
  if (!writable(node.id())) {
    emit_event(fmt::format("pstep n{} not sent: node is read-only or not a motion node", node.id()));
    return;
  }
  std::string error;
  const bool ok = profile_move(node, request.ch, request.target, true, error);
  int32_t rate = 0, period_ms = 20, start = 0, final_target = 0, accel = 0, lead_ms = 0;
  int handshake = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ok) {
      emit_event(fmt::format("pstep n{}c{} target {} acknowledged by the controller", node.id(),
                             request.ch + 1, request.target));
    } else if (running_ && node_index_ == request.node_index && ch_ == request.ch) {
      trip("move_failed", error);
      return;
    } else {
      emit_event(fmt::format("pstep n{}c{} sequence failed after the move ended: {}", node.id(),
                             request.ch + 1, error));
      return;
    }
    rate = stream_rate_;
    period_ms = stream_period_ms_;
    accel = stream_accel_;
    lead_ms = stream_lead_ms_;
    handshake = stream_handshake_;
    start = request.target;
    final_target = final_target_;
  }
  if (rate <= 0) {
    return;
  }
  // pstream: walk the target. Ends when the move ends (window over, a guard, or a
  // stop), which the sample task signals by clearing running_; the stop request is
  // queued behind this function, so it runs within one period of that.
  const uint16_t target_object = request.ch == 0 ? 0x607A : 0x687A;
  const uint16_t controlword_object = request.ch == 0 ? 0x6040 : 0x6840;
  espp::Ds402Drive *drive = node.mcp().drive(axis_of(request.ch));
  const int direction = final_target >= start ? 1 : -1;
  const int64_t t_start = now_us();
  uint32_t writes = 0, failures = 0;
  int lean_phase = 0;
  int64_t worst_write_us = 0;
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_ || node_index_ != request.node_index || ch_ != request.ch) {
        break;
      }
    }
    const int64_t elapsed_us = now_us() - t_start;
    const int64_t span = std::abs(static_cast<int64_t>(final_target) - start);
    int64_t walked = 0;
    double speed_now = 0; // profile speed, counts/s
    if (accel <= 0) {
      walked = static_cast<int64_t>(rate) * elapsed_us / 1'000'000; // constant speed
      speed_now = walked < span ? rate : 0;
    } else {
      // Trapezoid: accelerate at `accel` to `rate`, cruise, decelerate at `accel`
      // to rest on the target (a triangle if the distance is too short to cruise).
      const double t = static_cast<double>(elapsed_us) / 1e6;
      const double a = accel, d = static_cast<double>(span);
      double v = rate;
      if (v * v / a > d) {
        v = std::sqrt(d * a);
      }
      const double t_ramp = v / a;
      const double t_cruise = (d - v * v / a) / v;
      const double t_total = 2 * t_ramp + t_cruise;
      double s = d;
      if (t < t_ramp) {
        s = 0.5 * a * t * t;
        speed_now = a * t;
      } else if (t < t_ramp + t_cruise) {
        s = 0.5 * v * t_ramp + v * (t - t_ramp);
        speed_now = v;
      } else if (t < t_total) {
        const double remaining = t_total - t;
        s = d - 0.5 * a * remaining * remaining;
        speed_now = a * remaining;
      }
      walked = static_cast<int64_t>(s);
    }
    // Lead: the controller's position stage commands speed = P x (target - position),
    // so a target moving at v is followed v / P behind. Writing the target ahead of
    // the profile by v x lead (lead = 1 / P) cancels that lag. Never past the end.
    walked += static_cast<int64_t>(speed_now * lead_ms / 1000.0);
    walked = std::min(walked, span);
    const int32_t target = static_cast<int32_t>(start + direction * walked);
    std::error_code ec;
    const int64_t before = now_us();
    bool sent = false;
    if (handshake == 1) {
      // espp's handshake: target, new-set-point bit, wait for the acknowledge bit
      // (25 ms sleeps between statusword polls), clear, wait again. ~50 ms.
      sent = drive != nullptr && drive->set_target_position(target, ec, true, false);
    } else if (handshake == 2) {
      // Lean handshake, no statusword polling. The new-set-point bit is held at
      // each level for a whole tick: even ticks write the target and set the bit
      // (controlword 0x003F: operation enabled + new set-point + change
      // immediately), odd ticks clear it (0x002F). So the target updates every two
      // ticks. Writing 0x3F and 0x2F back to back (about 1 ms apart) worked for
      // hundreds of updates and then not at all for one whole move on the bench:
      // the controller can miss a pulse that short.
      if (lean_phase == 0) {
        sent = node.client().write_i32(target_object, 0, target, ec) &&
               node.client().write_u16(controlword_object, 0, 0x003F, ec);
      } else {
        sent = node.client().write_u16(controlword_object, 0, 0x002F, ec);
      }
      lean_phase ^= 1;
    } else {
      sent = node.client().write_i32(target_object, 0, target, ec);
    }
    worst_write_us = std::max(worst_write_us, now_us() - before);
    writes++;
    if (!sent) {
      failures++;
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      target_ = target; // the _tgt column follows what was actually written
    }
    xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
  }
  emit_event(fmt::format("pstream n{}c{} ended: {} target writes ({} failed), slowest {} us",
                         node.id(), request.ch + 1, writes, failures, worst_write_us));
}

/// The strongest stop verified on the bench, channel by channel. Only channels of
/// writable nodes get frames; read-only and other nodes are only reported.
///   1. A channel whose duty reads non-zero (or cannot be read) may have a live
///      loop: hold it with a profile move to its current count, but only if the
///      controller's position clamp contains that count. A controller that reset
///      has the factory clamp [0, 0], and a move would then run the joint to 0.
///   2. If anything is held, wait kHoldMs.
///   3. Disable voltage (controlword 0).
///   4. Report each channel's duty, current and statusword as polled afterwards.
/// Leaves the drives in SWITCH ON DISABLED.
void Motion::run_stop(const Request &request) {
  struct Target {
    size_t node_index;
    int ch;
  };
  std::vector<Target> targets;
  for (size_t i = 0; i < node_count_.load(); ++i) {
    for (int ch = 0; ch < 2; ++ch) {
      if (request.kind == Request::Kind::STOP_ALL ||
          (i == request.node_index && ch == request.ch)) {
        targets.push_back({i, ch});
      }
    }
  }
  emit_event(fmt::format("stop sequence start ({}): {} channel(s)", request.reason, targets.size()));

  // 1. Hold the live ones at their current count. OFF since the first chair move
  // (2026-11-17): after a guard tripped, the hold kept the drive enabled for
  // another ~0.5 s, let the leg run on 170 counts and then reversed it. These EHAs
  // are not back-drivable, so disable voltage alone is the stop. The hold code is
  // kept only for a controller found with a live loop while SWITCH ON DISABLED.
  bool any_held = false;
  for (const Target &t : targets) {
    if (!kHoldBeforeDisable) {
      break;
    }
    McpNode &node = *nodes_[t.node_index];
    if (!writable(node.id())) {
      continue; // never a frame to a read-only or non-motion node
    }
    const NodeData data = node.snapshot();
    if (data.link == Link::OFFLINE) {
      continue; // not answering; step 3 will try it anyway
    }
    const Field &duty = data.ch[t.ch].duty;
    if (duty.valid && duty.value == 0) {
      continue; // not driving: disable voltage alone
    }
    Windows w{};
    std::string read_error;
    if (!read_windows(node, t.ch, w, read_error)) {
      emit_event(fmt::format("n{}c{} no hold: {}", node.id(), t.ch + 1, read_error));
      continue;
    }
    const int32_t count = w.present;
    // Both windows must hold the count: the controller would force the hold
    // target into either one, i.e. drive the joint to its edge.
    if (!(w.clamp_min < w.clamp_max && w.clamp_min <= count && count <= w.clamp_max &&
          w.sw_min <= count && count <= w.sw_max)) {
      emit_event(fmt::format("n{}c{} no hold: {}; a move would drive to the window's edge",
                             node.id(), t.ch + 1, describe(w)));
      continue;
    }
    std::string error;
    if (profile_move(node, t.ch, count, false, error)) {
      any_held = true;
      emit_event(fmt::format("n{}c{} hold at count {}", node.id(), t.ch + 1, count));
    } else {
      emit_event(fmt::format("n{}c{} hold failed ({})", node.id(), t.ch + 1, error));
    }
  }

  // 2. Let the held loops settle.
  if (any_held) {
    vTaskDelay(pdMS_TO_TICKS(kHoldMs));
  }

  // 3. Disable voltage on every writable channel in the request.
  std::string failures;
  std::string skipped;
  for (const Target &t : targets) {
    McpNode &node = *nodes_[t.node_index];
    if (!writable(node.id())) {
      skipped += fmt::format(" n{}c{}", node.id(), t.ch + 1);
      continue;
    }
    // Retried: SDO transmits were seen to fail sporadically on the bench (I/O
    // error), and a stop that is sent once and lost is not a stop.
    std::error_code ec;
    espp::Ds402Drive *drive = node.mcp().drive(axis_of(t.ch));
    bool sent = false;
    for (int attempt = 0; drive != nullptr && attempt < kDisableAttempts && !sent; ++attempt) {
      sent = drive->set_controlword(kControlwordDisableVoltage, ec);
      if (!sent) {
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }
    if (!sent) {
      failures += fmt::format(" n{}c{}({})", node.id(), t.ch + 1, ec.message());
    }
  }

  // 4. Read back: wait for a few poll passes, then report what the poller sees.
  vTaskDelay(pdMS_TO_TICKS(std::max(100, 3 * period_ms_.load())));
  std::string readback;
  const int64_t now = now_us();
  for (const Target &t : targets) {
    const NodeData data = nodes_[t.node_index]->snapshot();
    readback += (readback.empty() ? "" : "; ") +
                output_readback(nodes_[t.node_index]->id(), data, t.ch, now);
  }
  emit_event(fmt::format("stop sequence done: disable voltage {}{}; readback: {}",
                         failures.empty() ? std::string("sent") : "FAILED on" + failures,
                         skipped.empty() ? std::string()
                                         : "; nothing sent to (read-only / not motion):" + skipped,
                         readback));
}

// ---------------------------------------------------------------------------------
// Arming and guards
// ---------------------------------------------------------------------------------

/// A guard tripped: report which guard and why, stop the channel, and disarm, so
/// the next move needs a fresh `arm`.
std::string Motion::move_summary() const {
  int32_t final_median = 0, final_spread = 0;
  const bool have_final =
      nodes_[node_index_]->settled_count(ch_, now_us(), final_median, final_spread);
  return fmt::format("start {} (median), target {}, final {}, peak |duty| {} ({:.1f} %), peak "
                     "|current| {} mA, target reached {}",
                     start_enc_, target_,
                     have_final ? fmt::format("{} (median of {}, spread {})", final_median,
                                              kEncoderWindow, final_spread)
                                : std::string("?"),
                     peak_duty_, peak_duty_ * 100.0 / 32767.0, peak_current_ma_,
                     saw_target_reached_ ? "seen" : "never seen");
}

void Motion::trip(const std::string &guard, const std::string &detail) {
  if (running_) {
    emit_event(fmt::format("pstep n{}c{} ended by guard: {}", nodes_[node_index_]->id(), ch_ + 1,
                           move_summary()));
  }
  emit_event(fmt::format("guard {} tripped: {}; stopping n{}c{}", guard, detail,
                         nodes_[node_index_]->id(), ch_ + 1));
  running_ = false;
  Request request{.kind = Request::Kind::STOP_ONE,
                  .node_index = static_cast<uint8_t>(node_index_),
                  .ch = static_cast<uint8_t>(ch_),
                  .target = 0,
                  .reason = {}};
  std::strncpy(request.reason, ("guard " + guard).c_str(), sizeof(request.reason) - 1);
  post(request);
  disarm_locked("guard tripped");
}

void Motion::set_heartbeat_consumer(McpNode &node, bool enable) {
  // The comm-timeout feature CANopen offers: a heartbeat consumer on the
  // controller that raises a communication error when the MIB's heartbeat (sent
  // every CONFIG_SYSID_HEARTBEAT_PERIOD_MS by sysid_main.cpp) stops for longer
  // than CONFIG_SYSID_HEARTBEAT_CONSUMER_MS. Whether the MCP266 implements 0x1016,
  // and what it does on the error (0x6007, 0x1029), is unverified; the result is
  // reported either way and a refusal does not block arming. Only the armed node,
  // and turned off again at disarm.
  if (!writable(node.id()) || node.snapshot().link != Link::ONLINE) {
    return;
  }
  const uint32_t value =
      enable ? ((static_cast<uint32_t>(CONFIG_SYSID_HEARTBEAT_NODE_ID) << 16) |
                static_cast<uint32_t>(CONFIG_SYSID_HEARTBEAT_CONSUMER_MS))
             : 0;
  std::error_code ec;
  if (!node.client().write_u32(kHeartbeatConsumerObject, 1, value, ec)) {
    emit_event(fmt::format("n{} heartbeat consumer {} refused ({}, abort 0x{:08X})", node.id(),
                           enable ? "enable" : "disable", ec.message(),
                           node.client().last_abort_code()));
    return;
  }
  const uint32_t readback = node.client().read_u32(kHeartbeatConsumerObject, 1, ec);
  emit_event(fmt::format("n{} heartbeat consumer {}: wrote 0x{:08X}, read {}", node.id(),
                         enable ? "on" : "off", value,
                         ec ? std::string("failed") : fmt::format("0x{:08X}", readback)));
}

void Motion::disarm_locked(const std::string &why) {
  if (!armed_) {
    return;
  }
  armed_ = false;
  emit_event("disarmed: " + why);
  // Consumer off first, then the heartbeat, so the controller never sees it stop
  // while still watching for it.
  // The heartbeat and the controller's consumer are left running after the first
  // arm: writing 0 to 0x1016:1 did not stop the MCP266 watching, so stopping the
  // heartbeat here faulted an enabled drive (and raised a warning on an idle one)
  // 500 ms after every disarm.
}

void Motion::arm(int node_id, int ch_number) {
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("arm", fmt::format("node {} is not polled", node_id));
    return;
  }
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("arm", fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  if (ch_number != 1 && ch_number != 2) {
    refuse("arm", "channel must be 1 (M1) or 2 (M2)");
    return;
  }
  // What the operator decides from: the count and the windows (uploads only).
  Windows w{};
  std::string read_error;
  const std::string windows = read_windows(*nodes_[index], ch_number - 1, w, read_error)
                                  ? describe(w)
                                  : "not usable: " + read_error;
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    refuse("arm", "a move is running");
    return;
  }
  disarm_locked("re-armed"); // one armed channel at a time
  armed_ = true;
  armed_node_index_ = index;
  armed_ch_ = ch_number - 1;
  armed_until_us_ = now_us() + static_cast<int64_t>(CONFIG_SYSID_ARM_TIMEOUT_S) * 1'000'000;
  emit_event(fmt::format("armed n{}c{} for {} s (until t_us {}); {}", node_id, ch_number,
                         CONFIG_SYSID_ARM_TIMEOUT_S, armed_until_us_, windows));
  // Heartbeat first: a CiA 301 consumer starts watching at the first heartbeat.
  if (kUseHeartbeat) {
    set_heartbeat_(true);
    set_heartbeat_consumer(*nodes_[index], true);
  }
}

void Motion::pstep(int node_id, int ch_number, int delta, int velocity, int acceleration,
                   int stream_rate, int stream_period_ms, int stream_handshake, int stream_accel,
                   int stream_lead_ms) {
  if (stream_rate < 0 || stream_rate > 5000 || stream_period_ms < 5 || stream_period_ms > 200 ||
      stream_accel < 0 || stream_accel > 100000 || stream_handshake < 0 || stream_handshake > 2 ||
      stream_lead_ms < 0 || stream_lead_ms > 1000) {
    refuse("pstream", "rate 1..5000 counts/s, period 5..200 ms, accel 0..100000 counts/s^2, "
                      "handshake 0..2, lead 0..1000 ms");
    return;
  }
  if (velocity == 0) {
    velocity = CONFIG_SYSID_MOVE_VELOCITY;
  }
  if (acceleration == 0) {
    acceleration = CONFIG_SYSID_MOVE_ACCELERATION;
  }
  if (velocity < 1 || velocity > 20000 || acceleration < 1 || acceleration > 200000) {
    refuse("pstep", "velocity must be 1..20000 counts/s and acceleration 1..200000 counts/s^2");
    return;
  }
  // --- Refusals that need no bus. The node check comes first: nothing below
  // builds a frame for a node that may not be written. ---
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("pstep", fmt::format("node {} is not polled", node_id));
    return;
  }
  // Allow-list: only the EHA legs (nodes 1 and 3 by default), never a read-only
  // node (the carriages, node 2).
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("pstep", fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  if (ch_number != 1 && ch_number != 2) {
    refuse("pstep", "channel must be 1 (M1) or 2 (M2)");
    return;
  }
  const int ch = ch_number - 1;
  // Step size cap: bounds how far one command can send the joint.
  if (delta == 0 || std::abs(delta) > CONFIG_SYSID_PSTEP_MAX_DELTA_COUNTS) {
    refuse("pstep", fmt::format("delta {} outside 1..{} counts (either sign)", delta,
                                CONFIG_SYSID_PSTEP_MAX_DELTA_COUNTS));
    return;
  }

  // --- Reads (SDO uploads only): the present count and both windows the
  // controller limits a target to. ---
  McpNode &node = *nodes_[index];
  Windows w{};
  std::string read_error;
  if (!read_windows(node, ch, w, read_error)) {
    refuse("pstep", "not usable: " + read_error);
    return;
  }
  const int32_t present = w.present;
  const int64_t target64 = static_cast<int64_t>(present) + delta;
  const int64_t lo = std::min<int64_t>(present, target64);
  const int64_t hi = std::max<int64_t>(present, target64);
  // Hardware-side travel guard. The controller forces any target into its clamp
  // (MinPos/MaxPos) and its 0x607D limits. A clamp left by other firmware (the
  // bench motor's [10000, 18000] on node 3 M1) or a reset controller's [0, 0]
  // would turn a small step into a run to the window's edge. So both windows must
  // be ones `clamp` could have installed: non-degenerate, no wider than
  // SYSID_MAX_CLAMP_SPAN_COUNTS, and holding the present count and the target with
  // a margin. A firmware or host bug then cannot ask the controller for more.
  const int64_t margin = CONFIG_SYSID_CLAMP_MARGIN_COUNTS;
  const int64_t span_cap = CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS;
  const auto window_ok = [&](int64_t min, int64_t max) {
    return min < max && max - min <= span_cap && lo >= min + margin && hi <= max - margin;
  };
  if (!window_ok(w.clamp_min, w.clamp_max) || !window_ok(w.sw_min, w.sw_max)) {
    refuse("pstep", fmt::format("{}, target {}: both windows must span 1..{} counts and hold "
                                "present and target {} counts inside (set with clamp)",
                                describe(w), target64, span_cap, margin));
    return;
  }
  const int32_t target = static_cast<int32_t>(target64);

  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t t = now_us();
  const int64_t stale_us = static_cast<int64_t>(kStalePeriods) * period_ms_.load() * 1000;

  // Arming: a move needs a deliberate `arm` of exactly this channel in the last
  // few seconds, so a stray or replayed line on its own moves nothing.
  if (!armed_locked(t) || armed_node_index_ != index || armed_ch_ != ch) {
    refuse("pstep", fmt::format("n{}c{} is not armed (arm {} {} first)", node_id, ch_number,
                                node_id, ch_number));
    return;
  }
  // Arming never runs out mid-move: the whole window must fit.
  if (armed_until_us_ - t < static_cast<int64_t>(CONFIG_SYSID_MOVE_WINDOW_MS) * 1000) {
    refuse("pstep", fmt::format("less than the {} ms move window left on the arming; arm again",
                                CONFIG_SYSID_MOVE_WINDOW_MS));
    return;
  }
  // One move at a time.
  if (running_) {
    refuse("pstep", "a move is already running");
    return;
  }
  if (!have_inputs_) {
    refuse("pstep", "no samples yet");
    return;
  }
  // Every running guard needs its input; a guard that cannot see is not a guard,
  // so a missing input refuses the move.
  const NodeData data = node.snapshot();
  const ChannelData &c = data.ch[ch];
  if (data.link != Link::ONLINE) {
    refuse("pstep", "node not online");
    return;
  }
  if (!c.enc.valid || t - c.enc.t_us > stale_us) {
    refuse("pstep", "encoder feedback missing or stale");
    return;
  }
  // Fault: never moved out of, and never cleared automatically (`clearfaults`).
  if (!c.status.valid || t - c.status.t_us > kSlowRingLength * stale_us) {
    refuse("pstep", "statusword missing or stale");
    return;
  }
  if (c.status.value & kStatuswordFault) {
    refuse("pstep", fmt::format("drive in fault (statusword 0x{:04X}); clearfaults {} first",
                                static_cast<uint16_t>(c.status.value), node_id));
    return;
  }
  int32_t lim = 0;
  if (!limit_bits(data, ch, lim) || lim != 0) {
    refuse("pstep", fmt::format("limit input active or unknown (lim={})", lim));
    return;
  }
  if (!c.cur_ma.valid || t - c.cur_ma.t_us > stale_us) {
    refuse("pstep", "motor current missing or stale");
    return;
  }
  if (std::abs(c.cur_ma.value) > CONFIG_SYSID_MAX_CURRENT_MA) {
    refuse("pstep", fmt::format("current already {} mA (limit {})", c.cur_ma.value,
                                CONFIG_SYSID_MAX_CURRENT_MA));
    return;
  }
  if (!imu_valid_) {
    refuse("pstep", "IMU not reading");
    return;
  }
  if (tilt_rate_dps_ > CONFIG_SYSID_MAX_TILT_RATE_DPS) {
    refuse("pstep", fmt::format("chair already tilting at {:.1f} deg/s", tilt_rate_dps_));
    return;
  }
  if (!can_bus_ok_) {
    refuse("pstep", "CAN controller not error-active");
    return;
  }

  // --- Accepted: the guards watch from now, the worker sends the frames. ---
  running_ = true;
  node_index_ = index;
  ch_ = ch;
  // Streaming starts at the present count and walks to the target; a step goes
  // straight there. The travel guard always uses the final target.
  final_target_ = target;
  stream_rate_ = stream_rate;
  stream_period_ms_ = stream_period_ms;
  stream_handshake_ = stream_handshake;
  stream_accel_ = stream_accel;
  stream_lead_ms_ = stream_lead_ms;
  target_ = stream_rate > 0 ? present : target;
  move_velocity_ = velocity;
  move_acceleration_ = acceleration;
  start_enc_ = present;
  peak_duty_ = 0;
  peak_current_ma_ = 0;
  saw_target_reached_ = false;
  start_timeouts_ = node.timeouts();
  start_can_errors_ = UINT32_MAX; // taken by the next check()
  end_us_ = t + static_cast<int64_t>(CONFIG_SYSID_MOVE_WINDOW_MS) * 1000;
  emit_event(fmt::format("pstep n{}c{} from {} to {} (delta {}); profile velocity {} accel {}; {}; "
                         "window {} ms",
                         node_id, ch_number, present, target, delta, velocity, acceleration,
                         describe(w), CONFIG_SYSID_MOVE_WINDOW_MS));
  Request request{.kind = Request::Kind::MOVE,
                  .node_index = static_cast<uint8_t>(index),
                  .ch = static_cast<uint8_t>(ch),
                  .target = stream_rate > 0 ? present : target,
                  .reason = {}};
  if (stream_rate > 0) {
    emit_event(fmt::format("pstream n{}c{}: target walks at {} counts/s, one {} every {} ms",
                           node_id, ch_number, stream_rate,
                           stream_handshake == 1   ? "target write + espp new-set-point handshake"
                           : stream_handshake == 2 ? "target write + lean handshake (no polling)"
                                                   : "0x607A write (no handshake)",
                           stream_period_ms));
    emit_event(fmt::format("pstream n{}c{}: accel {} counts/s^2, target lead {} ms", node_id, ch_number,
                           stream_accel, stream_lead_ms));
  }
  post(request);
}

/// `clamp <node> <ch> <min> <max>`. Installs [min, max] as both windows the
/// controller limits a position target to:
///   * the position PID record's MinPos/MaxPos (Basicmicro command 61/62). The
///     setter takes the whole 7-field record, so the record is read first (command
///     63/64) and written back with the same P, I, D, MaxI and deadband: only the
///     clamp changes. Mcp266::configure_position_loop() is not used because it
///     seeds a fallback P gain when the stored P reads 0.
///   * the CiA 402 software position limits 0x607D:1/:2, as espp's
///     set_software_position_limits() and the teammate's Actuator also write.
/// Then reads the whole record and 0x607D back and reports before / after.
void Motion::set_clamp(int node_id, int ch_number, int32_t new_min, int32_t new_max) {
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("clamp", fmt::format("node {} is not polled", node_id));
    return;
  }
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("clamp", fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  if (ch_number != 1 && ch_number != 2) {
    refuse("clamp", "channel must be 1 (M1) or 2 (M2)");
    return;
  }
  const int ch = ch_number - 1;
  // Span cap: the window is the most one experiment can move the joint.
  if (!(new_min < new_max) ||
      static_cast<int64_t>(new_max) - new_min > CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS) {
    refuse("clamp", fmt::format("[{}, {}] must have min < max and span at most {} counts", new_min,
                                new_max, CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Only the armed channel, and never under a running move.
    if (!armed_locked(now_us()) || armed_node_index_ != index || armed_ch_ != ch) {
      refuse("clamp", fmt::format("n{}c{} is not armed (arm {} {} first)", node_id, ch_number,
                                  node_id, ch_number));
      return;
    }
    if (running_) {
      refuse("clamp", "a move is running");
      return;
    }
  }
  McpNode &node = *nodes_[index];
  auto &client = node.client();
  const auto objects = ch == 0 ? espp::detail::mcp266::axis_m1() : espp::detail::mcp266::axis_m2();
  const uint16_t sw_limit = ch == 0 ? 0x607D : 0x687D;

  Windows before{};
  std::string read_error;
  if (!read_windows(node, ch, before, read_error)) {
    refuse("clamp", "not usable: " + read_error);
    return;
  }
  // The joint must already be inside the new window: otherwise the controller
  // would force the next target to its edge.
  if (before.present < new_min || before.present > new_max) {
    refuse("clamp", fmt::format("present count {} is outside [{}, {}]", before.present, new_min,
                                new_max));
    return;
  }

  // The record in readback order [P, I, D, MaxI, Deadzone, MinPos, MaxPos],
  // through the one proven read path (whole record, from sub 1).
  std::error_code ec;
  std::array<int32_t, 7> record{};
  if (!node.read_position_record(ch, record, ec)) {
    refuse("clamp", fmt::format("position PID record 0x{:04X} unreadable: {}",
                                objects.position_pid_get, ec.message()));
    return;
  }
  // The record just read must agree with the checked clamp read_windows() got.
  if (record[5] != before.clamp_min || record[6] != before.clamp_max) {
    refuse("clamp", fmt::format("record clamp [{}, {}] disagrees with the checked read [{}, {}]",
                                record[5], record[6], before.clamp_min, before.clamp_max));
    return;
  }
  std::array<int32_t, 7> wanted = record;
  wanted[5] = new_min;
  wanted[6] = new_max;
  // Setter order is [D, P, I, MaxI, Deadzone, MinPos, MaxPos] (espp's remap).
  const auto setter = espp::detail::mcp266::position_pid_readback_to_setter(wanted);
  std::string failure;
  for (uint8_t sub = 1; sub <= 7 && failure.empty(); ++sub) {
    if (!write_i32_retry(client, objects.position_pid_set, sub, setter[sub - 1], ec)) {
      failure = fmt::format("write 0x{:04X}:{} failed: {}", objects.position_pid_set, sub,
                            ec.message());
    }
  }
  if (failure.empty() && (!write_i32_retry(client, sw_limit, 1, new_min, ec) ||
                          !write_i32_retry(client, sw_limit, 2, new_max, ec))) {
    failure = fmt::format("write 0x{:04X} failed: {}", sw_limit, ec.message());
  }

  // Read everything back, whatever happened.
  std::array<int32_t, 7> after{};
  const bool record_read = node.read_position_record(ch, after, ec);
  Windows now{};
  const bool windows_read = read_windows(node, ch, now, read_error);
  const bool gains_same = record_read && std::equal(record.begin(), record.begin() + 5, after.begin());
  const bool took = windows_read && now.clamp_min == new_min && now.clamp_max == new_max &&
                    now.sw_min == new_min && now.sw_max == new_max;
  emit_event(fmt::format(
      "clamp n{}c{} [{}, {}]: {}; before {}; after {}; gains P,I,D,MaxI,deadband {} -> {} ({})",
      node_id, ch_number, new_min, new_max,
      !failure.empty() ? "FAILED: " + failure : (took ? "installed" : "NOT confirmed by readback"),
      describe(before), windows_read ? describe(now) : "not usable: " + read_error,
      fmt::format("{},{},{},{},{}", record[0], record[1], record[2], record[3], record[4]),
      record_read ? fmt::format("{},{},{},{},{}", after[0], after[1], after[2], after[3], after[4])
                  : std::string("?"),
      gains_same ? "unchanged" : "CHANGED OR UNREADABLE"));
}

/// `gains <node> <ch> <P> <I> <D> <MaxI> <deadband>`, raw controller units (the
/// record as `dump` prints it: P, I, D are x1024). Tuning experiments only:
///   * armed channel only, never under a running move;
///   * only while a narrow clamp (span <= SYSID_MAX_CLAMP_SPAN_COUNTS) is installed
///     and holds the present count, so whatever the new gains do, the controller
///     cannot be asked for a target outside that window;
///   * hard caps on each field (kGain*Max);
///   * controller RAM only: nothing here writes settings to flash, so a power
///     cycle of the controller restores its stored gains.
/// The clamp fields are written back unchanged. Reads the record back and reports.
void Motion::set_gains(int node_id, int ch_number, int32_t kp, int32_t ki, int32_t kd,
                       int32_t max_i, int32_t deadband) {
  constexpr int32_t kGainPMax = 8192;      // 8.0; stock on this chair is ~0.1, tuned 2.0
  constexpr int32_t kGainIMax = 1024;
  constexpr int32_t kGainDMax = 2048;      // stock is ~0.85 to 1.06
  constexpr int32_t kGainMaxIMax = 100000;
  constexpr int32_t kDeadbandMax = 400;
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("gains", fmt::format("node {} is not polled", node_id));
    return;
  }
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("gains", fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  if (ch_number != 1 && ch_number != 2) {
    refuse("gains", "channel must be 1 (M1) or 2 (M2)");
    return;
  }
  const int ch = ch_number - 1;
  if (kp < 0 || kp > kGainPMax || ki < 0 || ki > kGainIMax || kd < 0 || kd > kGainDMax ||
      max_i < 0 || max_i > kGainMaxIMax || deadband < 0 || deadband > kDeadbandMax) {
    refuse("gains", fmt::format("out of range: P 0..{}, I 0..{}, D 0..{}, MaxI 0..{}, deadband 0..{}",
                                kGainPMax, kGainIMax, kGainDMax, kGainMaxIMax, kDeadbandMax));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_locked(now_us()) || armed_node_index_ != index || armed_ch_ != ch) {
      refuse("gains", fmt::format("n{}c{} is not armed (arm {} {} first)", node_id, ch_number,
                                  node_id, ch_number));
      return;
    }
    if (running_) {
      refuse("gains", "a move is running");
      return;
    }
  }
  McpNode &node = *nodes_[index];
  auto &client = node.client();
  const auto objects = ch == 0 ? espp::detail::mcp266::axis_m1() : espp::detail::mcp266::axis_m2();
  Windows w{};
  std::string read_error;
  if (!read_windows(node, ch, w, read_error)) {
    refuse("gains", "not usable: " + read_error);
    return;
  }
  const int64_t span = static_cast<int64_t>(w.clamp_max) - w.clamp_min;
  if (!(w.clamp_min < w.clamp_max) || span > CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS ||
      w.present < w.clamp_min || w.present > w.clamp_max) {
    refuse("gains", fmt::format("{}: install a clamp of span <= {} around the present count first",
                                describe(w), CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS));
    return;
  }
  std::error_code ec;
  std::array<int32_t, 7> record{};
  if (!node.read_position_record(ch, record, ec)) {
    refuse("gains", "position PID record unreadable: " + ec.message());
    return;
  }
  if (record[5] != w.clamp_min || record[6] != w.clamp_max) {
    refuse("gains", "record clamp disagrees with the checked read");
    return;
  }
  // Readback order [P, I, D, MaxI, Deadzone, MinPos, MaxPos]; clamp kept.
  std::array<int32_t, 7> wanted = record;
  wanted[0] = kp;
  wanted[1] = ki;
  wanted[2] = kd;
  wanted[3] = max_i;
  wanted[4] = deadband;
  const auto setter = espp::detail::mcp266::position_pid_readback_to_setter(wanted);
  std::string failure;
  for (uint8_t sub = 1; sub <= 7 && failure.empty(); ++sub) {
    if (!write_i32_retry(client, objects.position_pid_set, sub, setter[sub - 1], ec)) {
      failure = fmt::format("write 0x{:04X}:{} failed: {}", objects.position_pid_set, sub,
                            ec.message());
    }
  }
  std::array<int32_t, 7> after{};
  const bool read_ok = node.read_position_record(ch, after, ec);
  const bool took = read_ok && after == wanted;
  emit_event(fmt::format(
      "gains n{}c{}: {}; P,I,D,MaxI,deadband,min,max {} -> {}", node_id, ch_number,
      !failure.empty() ? "FAILED: " + failure : (took ? "installed" : "NOT confirmed by readback"),
      fmt::format("{},{},{},{},{},{},{}", record[0], record[1], record[2], record[3], record[4],
                  record[5], record[6]),
      read_ok ? fmt::format("{},{},{},{},{},{},{}", after[0], after[1], after[2], after[3],
                            after[4], after[5], after[6])
              : std::string("unreadable")));
}

/// `vgains <node> <ch> <P> <I> <D> <QPPS>`: the velocity PID record. Read mirror
/// 0x2037 / 0x2038 subs 1..4 = P, I, D (16.16), QPPS (Basicmicro command 55 / 56);
/// set mirror 0x201C / 0x201D subs 1..4 = D, P, I, QPPS (command 28 / 29; the
/// mirrors sit at 0x2000 + command number, as the position record's 0x203D / 0x203F
/// do). Same conditions as `gains`: armed channel, no move running, a narrow clamp
/// installed. Controller RAM only.
void Motion::set_velocity_gains(int node_id, int ch_number, int32_t kp, int32_t ki, int32_t kd,
                                int32_t qpps) {
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("vgains", fmt::format("node {} is not polled", node_id));
    return;
  }
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("vgains", fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  if (ch_number != 1 && ch_number != 2) {
    refuse("vgains", "channel must be 1 (M1) or 2 (M2)");
    return;
  }
  const int ch = ch_number - 1;
  if (kp < 0 || kp > 0x200000 || ki < 0 || ki > 0x80000 || kd < 0 || kd > 0x80000 || qpps < 1 ||
      qpps > 500000) {
    refuse("vgains", "out of range: P 0..0x200000, I 0..0x80000, D 0..0x80000 (16.16), QPPS 1..500000");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_locked(now_us()) || armed_node_index_ != index || armed_ch_ != ch) {
      refuse("vgains", fmt::format("n{}c{} is not armed (arm {} {} first)", node_id, ch_number,
                                   node_id, ch_number));
      return;
    }
    if (running_) {
      refuse("vgains", "a move is running");
      return;
    }
  }
  McpNode &node = *nodes_[index];
  auto &client = node.client();
  Windows w{};
  std::string read_error;
  if (!read_windows(node, ch, w, read_error)) {
    refuse("vgains", "not usable: " + read_error);
    return;
  }
  const int64_t span = static_cast<int64_t>(w.clamp_max) - w.clamp_min;
  if (!(w.clamp_min < w.clamp_max) || span > CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS ||
      w.present < w.clamp_min || w.present > w.clamp_max) {
    refuse("vgains", fmt::format("{}: install a clamp of span <= {} around the present count first",
                                 describe(w), CONFIG_SYSID_MAX_CLAMP_SPAN_COUNTS));
    return;
  }
  const uint16_t get_object = ch == 0 ? 0x2037 : 0x2038;
  const uint16_t set_object = ch == 0 ? 0x201C : 0x201D;
  const auto read_record = [&](std::array<int32_t, 4> &record, std::error_code &ec) {
    for (uint8_t sub = 1; sub <= 4; ++sub) {
      record[sub - 1] = read_i32_retry(client, get_object, sub, ec);
      if (ec) {
        return false;
      }
    }
    return true;
  };
  std::error_code ec;
  std::array<int32_t, 4> before{}, after{};
  if (!read_record(before, ec)) {
    refuse("vgains", "velocity PID record unreadable: " + ec.message());
    return;
  }
  const std::array<int32_t, 4> setter{kd, kp, ki, qpps}; // setter order D, P, I, QPPS
  std::string failure;
  for (uint8_t sub = 1; sub <= 4 && failure.empty(); ++sub) {
    if (!write_i32_retry(client, set_object, sub, setter[sub - 1], ec)) {
      failure = fmt::format("write 0x{:04X}:{} failed: {} (abort 0x{:08X})", set_object, sub,
                            ec.message(), client.last_abort_code());
    }
  }
  const bool read_ok = read_record(after, ec);
  const bool took = read_ok && after[0] == kp && after[1] == ki && after[2] == kd && after[3] == qpps;
  emit_event(fmt::format(
      "vgains n{}c{}: {}; P,I,D,QPPS {} -> {}", node_id, ch_number,
      !failure.empty() ? "FAILED: " + failure : (took ? "installed" : "NOT confirmed by readback"),
      fmt::format("{},{},{},{}", before[0], before[1], before[2], before[3]),
      read_ok ? fmt::format("{},{},{},{}", after[0], after[1], after[2], after[3])
              : std::string("unreadable")));
}

/// `sdow <node> <index> <sub> <size> <value>`: experiments on the controller's
/// "in position" behaviour. Only these objects (M1 / M2): position window
/// 0x6067 / 0x6867, position window time 0x6068 / 0x6868, following error window
/// 0x6065 / 0x6865, following error time out 0x6066 / 0x6866. Motion node, any
/// channel of it armed, no move running. Reads the object back.
void Motion::write_object(int node_id, uint16_t index, uint8_t sub, int size, int32_t value) {
  const size_t node_index = index_of(node_id);
  if (node_index == node_count_.load() || !writable(static_cast<uint8_t>(node_id))) {
    refuse("sdow", fmt::format("node {} is not a polled motion node", node_id));
    return;
  }
  const uint16_t base = index & static_cast<uint16_t>(~0x0800);
  if (base != 0x6065 && base != 0x6066 && base != 0x6067 && base != 0x6068) {
    refuse("sdow", fmt::format("0x{:04X} is not on the allow-list (0x6065..0x6068 / 0x6865..0x6868)",
                               index));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_locked(now_us()) || armed_node_index_ != node_index) {
      refuse("sdow", "arm a channel of this node first");
      return;
    }
    if (running_) {
      refuse("sdow", "a move is running");
      return;
    }
  }
  auto &client = nodes_[node_index]->client();
  std::error_code ec;
  bool ok = false;
  if (size == 1) {
    ok = client.write_u8(index, sub, static_cast<uint8_t>(value), ec);
  } else if (size == 2) {
    ok = client.write_u16(index, sub, static_cast<uint16_t>(value), ec);
  } else if (size == 4) {
    ok = client.write_u32(index, sub, static_cast<uint32_t>(value), ec);
  } else {
    refuse("sdow", "size must be 1, 2 or 4");
    return;
  }
  const uint32_t abort_code = ok ? 0 : client.last_abort_code();
  std::error_code read_ec;
  const uint32_t readback = size == 1   ? client.read_u8(index, sub, read_ec)
                            : size == 2 ? client.read_u16(index, sub, read_ec)
                                        : client.read_u32(index, sub, read_ec);
  emit_event(fmt::format("sdow n{} 0x{:04X}:{} = {} ({} byte): {}; reads back {}", node_id, index,
                         sub, value, size,
                         ok ? std::string("written")
                            : fmt::format("FAILED ({}, abort 0x{:08X})", ec.message(), abort_code),
                         read_ec ? std::string("?") : std::to_string(readback)));
}

void Motion::clear_faults(int node_id) {
  const size_t index = index_of(node_id);
  if (index == node_count_.load()) {
    refuse("clearfaults", fmt::format("node {} is not polled", node_id));
    return;
  }
  if (!writable(static_cast<uint8_t>(node_id))) {
    refuse("clearfaults",
           fmt::format("node {} is read-only or not in SYSID_MOTION_NODE_IDS", node_id));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ && node_index_ == index) {
      refuse("clearfaults", "a move is running on this node");
      return;
    }
  }
  McpNode &node = *nodes_[index];
  std::error_code ec;
  // espp Mcp266::reset_faults(): for each channel, read the statusword; on a channel
  // in FAULT or FAULT REACTION ACTIVE, write controlword 0x0000 then 0x0080 and poll
  // until it leaves the fault. Both channels of the device, faulted ones only.
  const bool ok = node.mcp().reset_faults(ec);
  uint16_t sw1 = 0, sw2 = 0;
  std::error_code ec1, ec2;
  node.mcp().read_statusword(Axis::M1, sw1, ec1);
  node.mcp().read_statusword(Axis::M2, sw2, ec2);
  emit_event(fmt::format(
      "clearfaults n{}: {}; CiA 402 fault reset on both channels of the device (espp "
      "Mcp266::reset_faults: per faulted channel controlword 0x0000 then 0x0080). It does NOT "
      "clear the controller's E-stop latch (that is Mcp266::reset_estop, command 200 at 0x20C8, "
      "device-wide; not called). Statusword now c1={} c2={}",
      node_id, ok ? "done" : "FAILED (" + ec.message() + ")",
      ec1 ? std::string("?") : fmt::format("0x{:04X}", sw1),
      ec2 ? std::string("?") : fmt::format("0x{:04X}", sw2)));
}

void Motion::check(const Inputs &in) {
  std::lock_guard<std::mutex> lock(mutex_);
  have_inputs_ = true;
  imu_valid_ = in.imu_valid;
  tilt_rate_dps_ = in.tilt_rate_dps;
  can_bus_ok_ = in.can_bus_ok;

  if (!running_) {
    // Arming expires on its own: it is permission for the next few seconds, not a mode.
    if (armed_ && in.t_us >= armed_until_us_) {
      disarm_locked(fmt::format("{} s elapsed", CONFIG_SYSID_ARM_TIMEOUT_S));
    }
    return;
  }
  if (start_can_errors_ == UINT32_MAX) {
    start_can_errors_ = in.can_errors;
  }

  const int64_t stale_us = static_cast<int64_t>(kStalePeriods) * period_ms_.load() * 1000;
  const NodeData &node = in.nodes[node_index_];
  const ChannelData &c = node.ch[ch_];

  // --- Guards on a running move. The first that fires stops the channel. ---

  // Time cap: the move has had its window; the stop sequence takes over (hold
  // if still driving, then disable voltage). The normal way a move ends.
  // What the move does, for its end event (logging only; no guard uses these).
  if (node.link == Link::ONLINE) {
    if (c.duty.valid) {
      peak_duty_ = std::max(peak_duty_, std::abs(c.duty.value));
    }
    if (c.cur_ma.valid) {
      peak_current_ma_ = std::max(peak_current_ma_, std::abs(c.cur_ma.value));
    }
    if (c.status.valid && (c.status.value & 0x0400)) { // CiA 402 bit 10: target reached
      saw_target_reached_ = true;
    }
  }
  if (in.t_us >= end_us_) {
    emit_event(fmt::format("pstep n{}c{} done (window over): {}", nodes_[node_index_]->id(),
                           ch_ + 1, move_summary()));
    running_ = false;
    Request request{.kind = Request::Kind::STOP_ONE,
                    .node_index = static_cast<uint8_t>(node_index_),
                    .ch = static_cast<uint8_t>(ch_),
                    .target = 0,
                    .reason = {}};
    std::strncpy(request.reason, "pstep window over", sizeof(request.reason) - 1);
    post(request);
    return;
  }
  // Stale feedback: the guards below are only as good as the data they read. No
  // fresh encoder within 3 sample periods means the controller or the bus has
  // stopped reporting, and the joint may be moving unobserved.
  if (node.link != Link::ONLINE || !c.enc.valid || in.t_us - c.enc.t_us > stale_us) {
    trip("stale_feedback",
         c.enc.valid ? fmt::format("encoder {} us old (limit {} us)", in.t_us - c.enc.t_us, stale_us)
                     : std::string("no encoder value"));
    return;
  }
  // CAN error: a bus that is not error-active, any bus error, or an SDO the moving
  // node did not answer. Each means a stop might not get through, so stop while
  // it still can.
  if (!in.can_bus_ok) {
    trip("can_error", "TWAI controller left the error-active state");
    return;
  }
  if (in.can_errors != start_can_errors_) {
    trip("can_error",
         fmt::format("{} bus error(s) during the move", in.can_errors - start_can_errors_));
    return;
  }
  // Bench, 2026-11-17: node 3 misses about one SDO reply in 400 with the bus
  // otherwise healthy (no bit/stuff/ack errors), so a single miss is not a fault;
  // the first pstep was cut short by exactly that. Several misses in one move are.
  // Real loss of feedback is caught sooner by stale_feedback above.
  if (nodes_[node_index_]->timeouts() - start_timeouts_ > 10) {
    trip("can_error", "the moving node missed more than 10 SDO replies during the move");
    return;
  }
  // Travel window: the joint must stay between start and target, plus a margin
  // for overshoot. Catches a wrong sign, a wrong channel, or a controller that
  // ignored the target.
  const int64_t lo = std::min(start_enc_, final_target_) - CONFIG_SYSID_TRAVEL_MARGIN_COUNTS;
  const int64_t hi = std::max(start_enc_, final_target_) + CONFIG_SYSID_TRAVEL_MARGIN_COUNTS;
  if (c.enc.value < lo || c.enc.value > hi) {
    trip("travel", fmt::format("encoder {} outside [{}, {}]", c.enc.value, lo, hi));
    return;
  }
  // Fault: the drive reports a fault mid-move.
  if (c.status.valid && (c.status.value & kStatuswordFault)) {
    trip("fault", fmt::format("statusword 0x{:04X}", static_cast<uint16_t>(c.status.value)));
    return;
  }
  // Limit switch: the EHA legs have none wired as far as known; kept for any node
  // that does. The bench found the MCP266 does not report them over CAN.
  int32_t lim = 0;
  if (limit_bits(node, ch_, lim) && lim != 0) {
    trip("limit", fmt::format("lim={}", lim));
    return;
  }
  // Current: a stall, a jam or a pinch shows as current before it shows anywhere
  // else.
  if (!c.cur_ma.valid || in.t_us - c.cur_ma.t_us > stale_us) {
    trip("current", "motor current missing or stale");
    return;
  }
  if (std::abs(c.cur_ma.value) > CONFIG_SYSID_MAX_CURRENT_MA) {
    trip("current", fmt::format("{} mA (limit {})", c.cur_ma.value, CONFIG_SYSID_MAX_CURRENT_MA));
    return;
  }
  // Tilt rate: the chair (or a person in it) is starting to tip. Also trips when
  // the IMU stops reading, since tilt is then unobserved.
  if (!in.imu_valid) {
    trip("tilt", "IMU not reading");
    return;
  }
  if (in.tilt_rate_dps > CONFIG_SYSID_MAX_TILT_RATE_DPS) {
    trip("tilt", fmt::format("{:.1f} deg/s (limit {})", in.tilt_rate_dps,
                             CONFIG_SYSID_MAX_TILT_RATE_DPS));
    return;
  }
}

void Motion::console_byte() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Any keystroke stops a running move: the operator's reflex (any key, Enter,
  // Ctrl-C) is always a stop, without having to type `stop` correctly.
  if (running_) {
    trip("console_byte", "byte received on the console");
  }
}

void Motion::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    emit_event(fmt::format("move on n{}c{} ended by stop command", nodes_[node_index_]->id(),
                           ch_ + 1));
    running_ = false;
  }
  disarm_locked("stop command");
  Request request{.kind = Request::Kind::STOP_ALL, .node_index = 0, .ch = 0, .target = 0, .reason = {}};
  std::strncpy(request.reason, "stop command", sizeof(request.reason) - 1);
  post(request);
}

bool Motion::target(size_t node_index, int ch, int32_t &counts) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_ && node_index == node_index_ && ch == ch_) {
    counts = target_;
    return true;
  }
  return false;
}

} // namespace sysid

#endif // CONFIG_SYSID_ENABLE_MOTION
