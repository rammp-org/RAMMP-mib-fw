#include "actuator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <span>
#include <thread>

namespace mib {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

/// The clamp installed while calibrating or homing: wide enough for any travel, narrow
/// enough that max - min still fits a 32-bit integer if the controller computes the span.
constexpr Actuator::Range kUnlimited{-1'000'000'000, 1'000'000'000};

// CiA 402 objects of M1; M2's are at +0x800 (Actuator::object()).
constexpr uint16_t kControlword = 0x6040;
constexpr uint16_t kModesOfOperation = 0x6060;
constexpr uint16_t kPositionWindow = 0x6067;
constexpr uint16_t kTargetPosition = 0x607A;
constexpr uint16_t kSoftwareLimit = 0x607D;
constexpr uint16_t kProfileVelocity = 0x6081;
// CiA 301 heartbeat consumer, node-wide: sub 1 = (producer id << 16) | timeout ms.
constexpr uint16_t kHeartbeatConsumer = 0x1016;
// Basicmicro manufacturer objects at 0x2000 + command number. Velocity PID: set with
// command 28/29 (subs 1..4 = D, P, I, QPPS), read with 55/56 (subs 1..4 = P, I, D, QPPS).
// Motor PWM (command 48) and current (49, x10 mA): sub 1 = M1, sub 2 = M2, 2 bytes signed.
constexpr uint16_t kVelocityPidSetM1 = 0x201C;
constexpr uint16_t kVelocityPidSetM2 = 0x201D;
constexpr uint16_t kVelocityPidGetM1 = 0x2037;
constexpr uint16_t kVelocityPidGetM2 = 0x2038;
constexpr uint16_t kMotorPwms = 0x2030;
constexpr uint16_t kMotorCurrents = 0x2031;

constexpr int8_t kProfilePositionMode = 1;
/// The MCP266 does not echo the mode in 0x6061; espp waits this long after writing it.
constexpr auto kModeSettle = 25ms;
// Controlwords. Operation enabled + new set-point + change immediately, then the same with
// the new set-point bit clear. Each level is held a whole tick: 0x3F and 0x2F written back
// to back (about 1 ms apart) worked for hundreds of updates on the bench and then for none
// of one whole move.
constexpr uint16_t kControlwordSet = 0x003F;
constexpr uint16_t kControlwordClear = 0x002F;
constexpr uint16_t kControlwordDisableVoltage = 0x0000;
constexpr uint16_t kStatuswordFault = 1u << 3;
constexpr uint16_t kStatuswordLimit = 1u << 11;

/// Never toggle the new-set-point bit faster than this, whatever the caller's period.
constexpr auto kMinHandshakeHold = 8ms;
/// Disable voltage attempts, and the pause between them.
constexpr int kDisableAttempts = 8;
constexpr auto kDisableRetryDelay = 5ms;
/// The statusword is read every this many ticks of a move (50 ms) to catch a fault.
constexpr uint32_t kStatusEveryTicks = 5;
/// A move is stopped after this many failed exchanges in a row, or when the last good
/// encoder read is older than kStaleFeedback. One failed SDO in a few hundred is normal on
/// the chair's bus (about one a minute); a run of them is not.
constexpr uint32_t kMaxConsecutiveFailures = 5;
constexpr auto kStaleFeedback = 150ms;
/// PWM and current are read back this long after a release (the bench saw PWM 0 within
/// 100 ms of disable voltage).
constexpr auto kReleaseReadbackDelay = 100ms;
/// Whole-step retries for the settings at initialize(), against a lost SDO.
constexpr int kSettingsAttempts = 3;
/// Homing: how often the approach is checked, and how long the motor is given to start.
constexpr auto kHomingPoll = 100ms;
constexpr auto kHomingGrace = 2s;

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

/// Position gain fields travel as x1024 fixed point; espp's API takes them as floats.
float gain_from_raw(uint32_t raw) { return static_cast<float>(raw) / 1024.0f; }
uint32_t gain_to_raw(float gain) { return static_cast<uint32_t>(std::lround(gain * 1024.0)); }

} // namespace

const char *to_string(Actuator::DriveState state) { return espp::Ds402Drive::to_string(state); }

const char *to_string(Actuator::Outcome outcome) {
  switch (outcome) {
  case Actuator::Outcome::NONE:
    return "none";
  case Actuator::Outcome::MOVING:
    return "moving";
  case Actuator::Outcome::ARRIVED:
    return "arrived";
  case Actuator::Outcome::FAILED:
    return "FAILED";
  case Actuator::Outcome::STOPPED:
    return "stopped";
  }
  return "?";
}

Actuator::Actuator(espp::CanopenClient &client, espp::Mcp266 &mcp, std::mutex &mcp_mutex,
                   const Config &config)
    : BaseComponent(config.name ? config.name : "Actuator", config.log_level)
    , client_(client)
    , mcp_(mcp)
    , mcp_mutex_(mcp_mutex)
    , config_(config)
    , range_(config.range)
    , shaper_delay_(std::max(std::chrono::milliseconds{0}, config.shaper_delay)) {}

uint16_t Actuator::object(uint16_t m1_index) const {
  return static_cast<uint16_t>(m1_index + (config_.axis == Axis::M2 ? 0x800 : 0));
}

// ------------------------------------------------------------------ online marking

bool Actuator::check_online() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (online_) {
    return true;
  }
  if (Clock::now() - offline_since_ >= config_.offline_retry) {
    return true; // let this call probe the controller again
  }
  logger_.debug("{}: skipped, controller offline", config_.name);
  return false;
}

void Actuator::note_result(bool ok, const std::error_code &ec, const char *what) {
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!online_) {
      logger_.info("{}: controller answering again", config_.name);
    }
    online_ = true;
    return;
  }
  if (ec != std::errc::timed_out) {
    // An SDO abort or a refused state transition is an answer; retrying at once is fine.
    logger_.warn("{}: {} failed: {}", config_.name, what, ec.message());
    return;
  }
  // timed_out comes both from a silent controller (SDO timeout) and from a drive that
  // answers but never reaches the requested CiA 402 state. Only the first should take
  // the controller offline, so probe it with one cheap read before deciding.
  bool silent;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    uint16_t statusword = 0;
    std::error_code probe_ec;
    silent = !mcp_.read_statusword(config_.axis, statusword, probe_ec) &&
             probe_ec == std::errc::timed_out;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!silent) {
    logger_.warn("{}: {} timed out; controller still answers", config_.name, what);
    return;
  }
  if (online_) {
    logger_.error("{}: {} got no answer; controller marked offline for {} ms", config_.name, what,
                  config_.offline_retry.count());
  }
  online_ = false;
  offline_since_ = Clock::now();
}

bool Actuator::require_commissioned(const char *what) const {
  if (config_.commissioned) {
    return true;
  }
  logger_.error("{}: {} refused: this leg is not commissioned (MIBconfig.hpp commissioned = "
                "false), so the MIB never writes to its controller channel",
                config_.name, what);
  return false;
}

bool Actuator::initialized() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return initialized_;
}

bool Actuator::online() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return online_;
}

bool Actuator::homed() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return homed_;
}

bool Actuator::ready() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return config_.commissioned && initialized_ && settings_ok_ &&
         (homed_ || !config_.homing.required);
}

bool Actuator::calibration_mode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return calibration_mode_;
}

void Actuator::set_shaper_delay(std::chrono::milliseconds delay) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  shaper_delay_ = std::max(std::chrono::milliseconds{0}, delay);
  logger_.info("{}: input shaper delay {} ms", config_.name, shaper_delay_.count());
}

std::chrono::milliseconds Actuator::shaper_delay() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return shaper_delay_;
}


bool Actuator::moving() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return result_.outcome == Outcome::MOVING;
}

Actuator::MoveResult Actuator::move_result() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return result_;
}

Actuator::Range Actuator::installed_limits() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return (calibration_mode_ || homing_) ? kUnlimited : range_;
}

// ------------------------------------------------------------------ controller settings

bool Actuator::read_settings_locked(SettingsReadback &rb, std::error_code &ec) {
  // CanopenClient's typed reads set ec on failure but do not clear it on success.
  ec.clear();
  ControllerSettings &s = rb.settings;
  s.position_window = client_.read_u32(object(kPositionWindow), 0, ec);
  if (ec) {
    return false;
  }
  // The whole record, from subindex 1: subindices 6 and 7 read 0 on their own (firmware
  // 1.4.9). espp reads it that way.
  float p = 0, i = 0, d = 0;
  if (!mcp_.read_position_pid(config_.axis, p, i, d, s.position_max_i, s.position_deadband,
                              rb.clamp.min, rb.clamp.max, ec)) {
    return false;
  }
  s.position_p = gain_to_raw(p);
  s.position_i = gain_to_raw(i);
  s.position_d = gain_to_raw(d);
  // Velocity record, readback order P, I, D, QPPS, also from subindex 1.
  const uint16_t get = config_.axis == Axis::M1 ? kVelocityPidGetM1 : kVelocityPidGetM2;
  uint32_t *velocity[] = {&s.velocity_p, &s.velocity_i, &s.velocity_d, &s.velocity_qpps};
  for (uint8_t sub = 1; sub <= 4; ++sub) {
    *velocity[sub - 1] = client_.read_u32(get, sub, ec);
    if (ec) {
      return false;
    }
  }
  rb.software_limit.min = client_.read_i32(object(kSoftwareLimit), 1, ec);
  if (!ec) {
    rb.software_limit.max = client_.read_i32(object(kSoftwareLimit), 2, ec);
  }
  if (!ec) {
    s.profile_velocity = client_.read_u32(object(kProfileVelocity), 0, ec);
  }
  if (!ec) {
    s.heartbeat_consumer = client_.read_u32(kHeartbeatConsumer, 1, ec);
  }
  return !ec;
}

bool Actuator::write_settings_locked(const Range &clamp, std::error_code &ec, bool &matched) {
  matched = false;
  const ControllerSettings &c = config_.controller;
  // Position window: inside it the controller commands no speed at all (its stored 100
  // counts was the "stalls ~90 short" of the stock setup).
  if (!client_.write_u32(object(kPositionWindow), 0, c.position_window, ec)) {
    return false;
  }
  // The whole position PID record with the clamp; espp writes it in the setter's
  // D, P, I order.
  if (!mcp_.set_position_pid(config_.axis, gain_from_raw(c.position_p), gain_from_raw(c.position_i),
                             gain_from_raw(c.position_d), c.position_max_i, c.position_deadband,
                             clamp.min, clamp.max, ec)) {
    return false;
  }
  // The whole velocity PID record, setter order D, P, I, QPPS.
  const uint16_t set = config_.axis == Axis::M1 ? kVelocityPidSetM1 : kVelocityPidSetM2;
  const uint32_t velocity[] = {c.velocity_d, c.velocity_p, c.velocity_i, c.velocity_qpps};
  for (uint8_t sub = 1; sub <= 4; ++sub) {
    if (!client_.write_u32(set, sub, velocity[sub - 1], ec)) {
      return false;
    }
  }
  if (!mcp_.set_software_position_limits(config_.axis, clamp.min, clamp.max, ec)) {
    return false;
  }
  // The profile velocity caps the speed command; high, so the stream sets the speed.
  if (!client_.write_u32(object(kProfileVelocity), 0, c.profile_velocity, ec)) {
    return false;
  }
  // The heartbeat consumer: once set, an enabled drive faults ~450 ms after the MIB's
  // heartbeat stops. Writing 0 later does not stop it watching, so it is never cleared.
  if (c.heartbeat_consumer != 0 &&
      !client_.write_u32(kHeartbeatConsumer, 1, c.heartbeat_consumer, ec)) {
    return false;
  }

  SettingsReadback rb{};
  if (!read_settings_locked(rb, ec)) {
    return false;
  }
  const ControllerSettings &r = rb.settings;
  matched = true;
  const auto compare = [&](const char *field, int64_t wrote, int64_t read) {
    if (wrote != read) {
      logger_.error("{}: {} wrote {} but reads back {}", config_.name, field, wrote, read);
      matched = false;
    }
  };
  compare("position window 0x6067", c.position_window, r.position_window);
  compare("position P", c.position_p, r.position_p);
  compare("position I", c.position_i, r.position_i);
  compare("position D", c.position_d, r.position_d);
  compare("position MaxI", c.position_max_i, r.position_max_i);
  compare("position deadband", c.position_deadband, r.position_deadband);
  compare("clamp min", clamp.min, rb.clamp.min);
  compare("clamp max", clamp.max, rb.clamp.max);
  compare("velocity P", c.velocity_p, r.velocity_p);
  compare("velocity I", c.velocity_i, r.velocity_i);
  compare("velocity D", c.velocity_d, r.velocity_d);
  compare("velocity QPPS", c.velocity_qpps, r.velocity_qpps);
  compare("0x607D min", clamp.min, rb.software_limit.min);
  compare("0x607D max", clamp.max, rb.software_limit.max);
  compare("profile velocity 0x6081", c.profile_velocity, r.profile_velocity);
  if (c.heartbeat_consumer != 0) {
    compare("heartbeat consumer 0x1016:1", c.heartbeat_consumer, r.heartbeat_consumer);
  }
  return true;
}

bool Actuator::settings_intact_locked(std::error_code &ec) {
  const ControllerSettings &c = config_.controller;
  const Range clamp = installed_limits();
  float p = 0, i = 0, d = 0;
  uint32_t max_i = 0, deadband = 0;
  Range read{};
  if (!mcp_.read_position_pid(config_.axis, p, i, d, max_i, deadband, read.min, read.max, ec)) {
    return false;
  }
  ec.clear();
  const uint32_t window = client_.read_u32(object(kPositionWindow), 0, ec);
  if (ec) {
    return false;
  }
  return gain_to_raw(p) == c.position_p && gain_to_raw(i) == c.position_i &&
         gain_to_raw(d) == c.position_d && max_i == c.position_max_i &&
         deadband == c.position_deadband && read.min == clamp.min && read.max == clamp.max &&
         window == c.position_window;
}

bool Actuator::read_controller_settings(SettingsReadback &readback) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = read_settings_locked(readback, ec);
  }
  note_result(ok, ec, "read_controller_settings");
  return ok;
}

bool Actuator::read_controller_limits(int32_t &min_counts, int32_t &max_counts) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    float p = 0, i = 0, d = 0;
    uint32_t max_i = 0, deadband = 0;
    ok = mcp_.read_position_pid(config_.axis, p, i, d, max_i, deadband, min_counts, max_counts, ec);
  }
  note_result(ok, ec, "read_controller_limits");
  return ok;
}

bool Actuator::apply_clamp(const Range &clamp) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok = false;
  bool matched = false;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    for (int attempt = 0; attempt < kSettingsAttempts && !ok; ++attempt) {
      ec.clear();
      ok = write_settings_locked(clamp, ec, matched);
    }
  }
  note_result(ok, ec, "install clamp");
  if (ok && !matched) {
    logger_.error("{}: clamp [{}, {}] did not read back as written", config_.name, clamp.min,
                  clamp.max);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  settings_ok_ = ok && matched;
  return settings_ok_;
}

bool Actuator::initialize() {
  if (!config_.commissioned) {
    logger_.info("{}: not commissioned: nothing is sent to its controller channel and moves "
                 "are refused",
                 config_.name);
    return false;
  }
  if (moving()) {
    logger_.error("{}: initialize refused, a move is in progress", config_.name);
    return false;
  }
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok = false;
  bool matched = false;
  uint16_t statusword = 0;
  bool have_statusword = false;
  const Range clamp = installed_limits();
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    for (int attempt = 0; attempt < kSettingsAttempts && !ok; ++attempt) {
      ec.clear();
      // NMT start, addressed to this node only. No fault reset (espp's Mcp266::start()
      // would clear a fault): a fault stays until clear_faults().
      ok = client_.nmt_start(ec) && write_settings_locked(clamp, ec, matched);
    }
    std::error_code sw_ec;
    have_statusword = mcp_.read_statusword(config_.axis, statusword, sw_ec);
  }
  note_result(ok, ec, "initialize");
  if (!ok) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = true;
    settings_ok_ = matched;
    homed_ = !config_.homing.required; // an absolute encoder is always "homed"
  }
  if (have_statusword && (statusword & kStatuswordFault)) {
    logger_.warn("{}: drive is in FAULT (statusword 0x{:04X}); moves are refused until "
                 "clear_faults()",
                 config_.name, statusword);
  }
  if (config_.homing.required && !restore_position()) {
    logger_.warn("{}: incremental encoder with no saved position; home() before moving",
                 config_.name);
  }
  const ControllerSettings &c = config_.controller;
  if (matched) {
    logger_.info("{}: settings written and read back: window {}, position P/I/D {}/{}/{} MaxI {} "
                 "deadband {}, velocity P/I/D {}/{}/{} QPPS {}, clamp [{}, {}], profile velocity "
                 "{}, heartbeat consumer 0x{:08X}; moves {} counts/s, {} counts/s^2, lead {} ms",
                 config_.name, c.position_window, c.position_p, c.position_i, c.position_d,
                 c.position_max_i, c.position_deadband, c.velocity_p, c.velocity_i, c.velocity_d,
                 c.velocity_qpps, clamp.min, clamp.max, c.profile_velocity, c.heartbeat_consumer,
                 config_.profile.velocity, config_.profile.acceleration, config_.lead.count());
  } else {
    logger_.error("{}: settings did NOT read back as written (see above): moves refused",
                  config_.name);
  }
  return matched;
}

// ------------------------------------------------------------------ position and status

bool Actuator::read_position_locked(int32_t &counts, std::error_code &ec) {
  return mcp_.read_encoder(config_.axis, counts, ec);
}

bool Actuator::read_position(int32_t &counts) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = read_position_locked(counts, ec);
  }
  note_result(ok, ec, "read_position");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_position_ = counts;
    position_time_ = Clock::now();
  }
  return ok;
}

bool Actuator::read_position_median(int32_t &median, int32_t &spread) {
  if (!check_online()) {
    return false;
  }
  std::array<int32_t, kMedianSamples> values{};
  std::error_code ec;
  bool ok = true;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    for (auto &value : values) {
      // One retry per sample: a single lost SDO should not fail a median.
      if (!read_position_locked(value, ec) && !read_position_locked(value, ec)) {
        ok = false;
        break;
      }
    }
  }
  note_result(ok, ec, "read_position_median");
  if (!ok) {
    return false;
  }
  std::sort(values.begin(), values.end());
  median = values[kMedianSamples / 2];
  spread = values.back() - values.front();
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_position_ = median;
  position_time_ = Clock::now();
  return true;
}

std::optional<int32_t> Actuator::current_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_position_;
}

std::chrono::milliseconds Actuator::position_age() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_position_) {
    return std::chrono::milliseconds::max();
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - position_time_);
}

std::optional<int32_t> Actuator::last_target() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_target_;
}

bool Actuator::read_status_locked(Status &status, std::error_code &ec) {
  // PWM and current mirrors: the value is in the low 2 bytes, signed.
  const auto read_mirror = [&](uint16_t index, int32_t &value) {
    std::array<uint8_t, 4> buffer{};
    const uint8_t sub = config_.axis == Axis::M1 ? 1 : 2;
    const size_t n = client_.sdo_upload(index, sub, std::span<uint8_t>(buffer), ec);
    if (n < 2) {
      if (!ec) {
        ec = std::make_error_code(std::errc::protocol_error);
      }
      return false;
    }
    value = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    return true;
  };
  int32_t pwm = 0;
  int32_t current = 0;
  if (!read_position_locked(status.count, ec) || !read_mirror(kMotorPwms, pwm) ||
      !read_mirror(kMotorCurrents, current) ||
      !mcp_.read_statusword(config_.axis, status.statusword, ec)) {
    return false;
  }
  status.pwm = static_cast<int16_t>(pwm);
  status.current_ma = current * 10;
  status.state = espp::Ds402Drive::state_from_statusword(status.statusword);
  status.fault = (status.statusword & kStatuswordFault) != 0;
  return true;
}

bool Actuator::read_status(Status &status) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = read_status_locked(status, ec);
  }
  note_result(ok, ec, "read_status");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_position_ = status.count;
    position_time_ = Clock::now();
  }
  return ok;
}

bool Actuator::get_drive_state(DriveState &state) {
  uint16_t statusword = 0;
  if (!read_statusword(statusword)) {
    return false;
  }
  state = espp::Ds402Drive::state_from_statusword(statusword);
  return true;
}

bool Actuator::read_statusword(uint16_t &statusword) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.read_statusword(config_.axis, statusword, ec);
  }
  note_result(ok, ec, "read_statusword");
  return ok;
}

bool Actuator::limit_active(bool &active) {
  uint16_t statusword = 0;
  if (!read_statusword(statusword)) {
    return false;
  }
  active = (statusword & kStatuswordLimit) != 0;
  return true;
}

bool Actuator::clear_faults() {
  if (!require_commissioned("clear_faults")) {
    return false;
  }
  if (moving()) {
    logger_.error("{}: clear_faults refused, a move is in progress", config_.name);
    return false;
  }
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok = false;
  uint16_t before = 0;
  uint16_t after = 0;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    auto *drive = mcp_.drive(config_.axis);
    std::error_code read_ec;
    mcp_.read_statusword(config_.axis, before, read_ec);
    // This channel only (espp's Mcp266::reset_faults() resets both channels of the device).
    ok = drive != nullptr && drive->fault_reset(ec);
    std::error_code after_ec;
    mcp_.read_statusword(config_.axis, after, after_ec);
  }
  note_result(ok, ec, "clear_faults");
  if (ok) {
    logger_.info("{}: fault reset: statusword 0x{:04X} -> 0x{:04X}", config_.name, before, after);
  } else {
    logger_.error("{}: fault reset did not take (statusword 0x{:04X} -> 0x{:04X}): a drive in "
                  "FAULT REACTION ACTIVE (a watchdog trip holds it ~11 s) takes it once in FAULT",
                  config_.name, before, after);
  }
  return ok;
}

// ------------------------------------------------------------------ drive control

bool Actuator::disable_locked(std::error_code &ec) {
  auto *drive = mcp_.drive(config_.axis);
  if (drive == nullptr) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  for (int attempt = 0; attempt < kDisableAttempts; ++attempt) {
    ec.clear();
    if (drive->set_controlword(kControlwordDisableVoltage, ec)) {
      return true;
    }
    std::this_thread::sleep_for(kDisableRetryDelay);
  }
  return false;
}

bool Actuator::enable_locked(int32_t start, std::error_code &ec) {
  ec.clear(); // get_state() sets ec on failure but does not clear it
  auto *drive = mcp_.drive(config_.axis);
  if (drive == nullptr) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const DriveState state = drive->get_state(ec);
  if (ec) {
    return false;
  }
  if (state == DriveState::Fault || state == DriveState::FaultReactionActive) {
    ec = std::make_error_code(std::errc::operation_not_permitted);
    return false;
  }
  if (state == DriveState::QuickStopActive &&
      !drive->set_controlword(kControlwordDisableVoltage, ec)) {
    return false; // left there by someone else; the enable sequence cannot leave it
  }
  // Profile position, written directly (the MCP266 does not echo it in 0x6061).
  if (!client_.write_i8(object(kModesOfOperation), 0, kProfilePositionMode, ec)) {
    return false;
  }
  std::this_thread::sleep_for(kModeSettle);
  // The target takes effect the moment the drive reaches OPERATION ENABLED, from whatever
  // 0x607A holds (after a controller power cycle: 0, which the clamp would turn into a run
  // to its edge). So it is the start count before the drive is enabled.
  if (!client_.write_i32(object(kTargetPosition), 0, start, ec)) {
    return false;
  }
  // Shutdown -> switch on -> enable operation. Refuses a faulted drive; no fault reset
  // (espp's Mcp266::move_to_position() would clear one).
  if (!drive->enable_operation(ec)) {
    return false;
  }
  return client_.write_u32(object(kProfileVelocity), 0, config_.controller.profile_velocity, ec);
}

// ------------------------------------------------------------------ moves

bool Actuator::do_move(int32_t counts, const Profile &profile, bool jog) {
  if (!require_commissioned(jog ? "jog" : "move")) {
    return false;
  }
  bool initialized = false;
  bool settings_ok = false;
  bool calibrating = false;
  bool homing = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized = initialized_;
    settings_ok = settings_ok_;
    calibrating = calibration_mode_;
    homing = homing_;
  }
  if (!initialized) {
    logger_.error("{}: move refused, not initialised (see the initialize() log)", config_.name);
    return false;
  }
  if (!settings_ok) {
    logger_.error("{}: move refused, the controller settings did not read back as written",
                  config_.name);
    return false;
  }
  // An unhomed incremental encoder has no meaningful count, so moves are refused, except
  // in calibration mode (jogs only, clamp lifted) and for homing itself.
  if (!ready() && !calibrating && !homing) {
    logger_.error("{}: move refused, needs homing (or calibration mode for a jog)", config_.name);
    return false;
  }
  if (calibrating && !jog) {
    logger_.error("{}: move to {} refused, only increment/decrement are allowed in calibration "
                  "mode (install a range and leave it for longer moves)",
                  config_.name, counts);
    return false;
  }
  if (profile.velocity == 0 || profile.acceleration == 0) {
    logger_.error("{}: move refused, profile speed and acceleration must be positive",
                  config_.name);
    return false;
  }
  return start_move(counts, profile);
}

bool Actuator::start_move(int32_t counts, const Profile &profile) {
  const int32_t target = clamp(counts);
  if (target != counts) {
    logger_.warn("{}: target {} clamped to {}", config_.name, counts, target);
  }
  const MotionProfile::Limits limits{.max_speed = static_cast<double>(profile.velocity),
                                     .acceleration = static_cast<double>(profile.acceleration)};
  // A move in progress: continue from the profile's position and speed, no bus traffic.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.outcome == Outcome::MOVING) {
      target_ = target;
      move_profile_ = profile;
      result_.target = target;
      last_target_ = target;
      if (phase_ == Phase::STREAM) {
        const auto now = Clock::now();
        const auto here = profile_.at(seconds(now - profile_t0_));
        profile_.plan(here.position, here.speed, target, limits);
        profile_t0_ = now;
        final_sent_ = false;
        final_latched_ = false;
        settled_since_.reset();
        profile_done_at_.reset();
        result_.profile_s = static_cast<float>(profile_.duration());
        logger_.info("{}: retarget to {} from {:.0f} at {:.0f} counts/s", config_.name, target,
                     here.position, here.speed);
      }
      return true; // in ACQUIRE the plan is made from the start count, with this target
    }
  }

  // A fresh move: the drive must not be faulted, and the controller must still hold the
  // settings (a controller that reset comes back on its flash settings and clamp).
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  uint16_t statusword = 0;
  bool intact = false;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.read_statusword(config_.axis, statusword, ec);
    if (ok) {
      intact = settings_intact_locked(ec);
      ok = !ec;
    }
  }
  note_result(ok, ec, "move");
  if (!ok) {
    return false;
  }
  if (statusword & kStatuswordFault) {
    logger_.error("{}: move refused, the drive is in FAULT (statusword 0x{:04X}); "
                  "clear_faults() first",
                  config_.name, statusword);
    return false;
  }
  if (!intact) {
    logger_.error("{}: the controller no longer holds the settings it was given (it reset, or "
                  "something rewrote them); writing them again",
                  config_.name);
    if (!apply_clamp(installed_limits())) {
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = Clock::now();
    phase_ = Phase::ACQUIRE;
    target_ = target;
    move_profile_ = profile;
    history_len_ = 0;
    history_next_ = 0;
    medians_len_ = 0;
    medians_next_ = 0;
    consecutive_failures_ = 0;
    last_good_read_ = now;
    ticks_ = 0;
    handshake_ = Handshake::SET;
    final_sent_ = false;
    final_latched_ = false;
    settled_since_.reset();
    profile_done_at_.reset();
    const uint32_t serial = result_.serial + 1;
    result_ = MoveResult{};
    result_.outcome = Outcome::MOVING;
    result_.serial = serial;
    result_.target = target;
    move_t0_ = now;
    last_target_ = target;
  }
  logger_.info("{}: move to {} at {} counts/s, {} counts/s^2", config_.name, target,
               profile.velocity, profile.acceleration);
  return true;
}

bool Actuator::move_absolute(int32_t counts) { return do_move(counts, config_.profile, false); }

bool Actuator::move_absolute(int32_t counts, const Profile &profile) {
  return do_move(counts, profile, false);
}

bool Actuator::move_relative(int32_t delta) { return move_relative(delta, config_.profile); }

bool Actuator::move_relative(int32_t delta, const Profile &profile) {
  if (!require_commissioned("move")) {
    return false;
  }
  int32_t from = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.outcome == Outcome::MOVING) {
      from = target_;
    }
  }
  int32_t spread = 0;
  if (!moving() && !read_position_median(from, spread)) {
    return false;
  }
  const int64_t target = static_cast<int64_t>(from) + delta;
  return do_move(static_cast<int32_t>(std::clamp<int64_t>(target, INT32_MIN, INT32_MAX)), profile,
                 false);
}

bool Actuator::increment() {
  int32_t here = 0;
  int32_t spread = 0;
  return require_commissioned("jog") && read_position_median(here, spread) &&
         do_move(here + config_.jog_step, config_.profile, true);
}

bool Actuator::decrement() {
  int32_t here = 0;
  int32_t spread = 0;
  return require_commissioned("jog") && read_position_median(here, spread) &&
         do_move(here - config_.jog_step, config_.profile, true);
}

bool Actuator::stop() {
  if (!config_.commissioned) {
    logger_.debug("{}: stop: not commissioned, never enabled by this firmware; nothing sent",
                  config_.name);
    return true;
  }
  // First end the move, so a tick in progress sends nothing more; then disable voltage.
  bool was_moving = false;
  uint32_t serial = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    serial = result_.serial;
    if (result_.outcome == Outcome::MOVING) {
      was_moving = true;
      result_.outcome = Outcome::STOPPED;
      std::snprintf(result_.reason, sizeof(result_.reason), "stop()");
      result_.elapsed_s = static_cast<float>(seconds(Clock::now() - move_t0_));
      if (const auto median = history_median()) {
        result_.final_count = *median;
      }
    }
    phase_ = Phase::IDLE;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = disable_locked(ec);
  }
  if (was_moving) {
    // The next ticks read PWM and current back into the result.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.serial == serial && result_.outcome == Outcome::STOPPED) {
      phase_ = Phase::RELEASED;
      released_at_ = Clock::now();
    }
  }
  if (ok) {
    if (was_moving) {
      logger_.warn("{}: stopped (disable voltage)", config_.name);
    }
  } else {
    logger_.error("{}: stop: disable voltage not acknowledged after {} attempts: {}", config_.name,
                  kDisableAttempts, ec.message());
  }
  note_result(ok, ec, "stop");
  return ok;
}

// ------------------------------------------------------------------ the tick

void Actuator::push_count(int32_t counts, Clock::time_point at) {
  history_[history_next_] = {counts, at};
  history_next_ = (history_next_ + 1) % history_.size();
  history_len_ = std::min(history_len_ + 1, history_.size());
  current_position_ = counts;
  position_time_ = at;
  // The median, every tick once the window is full, for the speed estimate.
  const auto median = history_median();
  if (!median) {
    return;
  }
  const Count oldest = medians_[medians_next_];
  const bool full = medians_len_ == medians_.size();
  medians_[medians_next_] = {*median, at};
  medians_next_ = (medians_next_ + 1) % medians_.size();
  medians_len_ = std::min(medians_len_ + 1, medians_.size());
  if (full) {
    const double dt = seconds(at - oldest.at);
    if (dt > 0) {
      const auto speed = static_cast<float>(std::fabs((*median - oldest.value) / dt));
      result_.peak_speed = std::max(result_.peak_speed, speed);
    }
  }
}

std::optional<int32_t> Actuator::history_median() const {
  if (history_len_ < history_.size()) {
    return std::nullopt;
  }
  std::array<int32_t, kMedianSamples> values{};
  for (size_t i = 0; i < history_.size(); ++i) {
    values[i] = history_[i].value;
  }
  std::nth_element(values.begin(), values.begin() + kMedianSamples / 2, values.end());
  return values[kMedianSamples / 2];
}

void Actuator::finish_locked(Outcome outcome, const char *reason) {
  uint32_t serial = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.outcome != Outcome::MOVING) {
      return; // ended meanwhile (stop())
    }
    serial = result_.serial;
    const auto now = Clock::now();
    phase_ = Phase::IDLE;
    result_.outcome = outcome;
    std::snprintf(result_.reason, sizeof(result_.reason), "%s", reason);
    result_.elapsed_s = static_cast<float>(seconds(now - move_t0_));
    if (profile_done_at_) {
      result_.release_s = static_cast<float>(seconds(now - *profile_done_at_));
    }
    if (const auto median = history_median()) {
      result_.final_count = *median;
    } else if (current_position_) {
      result_.final_count = *current_position_;
    }
  }
  std::error_code ec;
  const bool released = disable_locked(ec);
  MoveResult r;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.serial == serial) { // not replaced by a new move meanwhile
      phase_ = Phase::RELEASED;     // a later tick reads PWM and current back
      released_at_ = Clock::now();
    }
    r = result_;
  }
  if (!released) {
    logger_.error("{}: release (disable voltage) not acknowledged after {} attempts: {}",
                  config_.name, kDisableAttempts, ec.message());
  }
  if (outcome == Outcome::ARRIVED) {
    logger_.info("{}: arrived at {} (target {}, {:+} counts) in {:.2f} s, peak {:.0f} counts/s, "
                 "released {:.2f} s after the profile ended",
                 config_.name, r.final_count, r.target, r.final_count - r.target, r.elapsed_s,
                 r.peak_speed, r.release_s);
  } else {
    logger_.error("{}: move to {} FAILED at {} ({} counts short): {}; drive released", config_.name,
                  r.target, r.final_count, r.target - r.final_count, r.reason);
  }
}

bool Actuator::tick_locked() {
  const auto now = Clock::now();
  auto *drive = mcp_.drive(config_.axis);
  // Whether the move this tick started on is still the one in progress (stop() and a fresh
  // move change it from other tasks; they take state_mutex_ but not mcp_mutex_ first).
  uint32_t serial = 0;
  Phase phase;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    phase = phase_;
    serial = result_.serial;
    ++ticks_;
  }
  if (phase == Phase::IDLE || drive == nullptr) {
    return true;
  }
  if (phase == Phase::RELEASED) {
    // Read the output back once it has had time to fall, so the result shows the release.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (now - released_at_ < kReleaseReadbackDelay) {
        return true;
      }
    }
    Status status{};
    std::error_code status_ec;
    const bool have_status = read_status_locked(status, status_ec);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (phase_ != Phase::RELEASED || result_.serial != serial) {
      return true; // a new move started meanwhile
    }
    phase_ = Phase::IDLE;
    if (have_status) {
      result_.release_pwm = status.pwm;
      result_.release_current_ma = status.current_ma;
      result_.release_readback = true;
      current_position_ = status.count;
      position_time_ = now;
      logger_.info("{}: released: PWM {:.1f} %, {} mA, statusword 0x{:04X}", config_.name,
                   status.pwm * 100.0f / 32767.0f, status.current_ma, status.statusword);
    } else {
      logger_.warn("{}: released, but PWM/current could not be read back: {}", config_.name,
                   status_ec.message());
    }
    return have_status;
  }
  const auto still_ours = [&]() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return phase_ != Phase::IDLE && result_.serial == serial && result_.outcome == Outcome::MOVING;
  };

  // 1. The encoder, every tick: the median decides arrival, and its absence ends the move.
  std::error_code ec;
  int32_t count = 0;
  const bool read_ok = read_position_locked(count, ec);
  bool lost = false;
  uint32_t failures = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (read_ok) {
      push_count(count, now);
      last_good_read_ = now;
      consecutive_failures_ = 0;
    } else {
      ++consecutive_failures_;
      ++result_.sdo_failures;
    }
    failures = consecutive_failures_;
    lost =
        consecutive_failures_ >= kMaxConsecutiveFailures || now - last_good_read_ > kStaleFeedback;
  }
  if (lost) {
    char reason[96];
    std::snprintf(reason, sizeof(reason), "encoder feedback lost (%lu failed reads in a row)",
                  static_cast<unsigned long>(failures));
    finish_locked(Outcome::FAILED, reason);
    return false;
  }

  // 2. ACQUIRE: once the window is full, start from its median.
  if (phase == Phase::ACQUIRE) {
    std::optional<int32_t> start;
    int32_t spread = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start = history_median();
      if (start) {
        const auto [lo, hi] =
            std::minmax_element(history_.begin(), history_.end(),
                                [](const Count &a, const Count &b) { return a.value < b.value; });
        spread = hi->value - lo->value;
      }
    }
    if (!start) {
      return true; // still filling
    }
    if (spread > config_.max_start_spread) {
      char reason[96];
      std::snprintf(reason, sizeof(reason),
                    "encoder not settled at the start: %ld counts spread over %u reads",
                    static_cast<long>(spread), static_cast<unsigned>(kMedianSamples));
      finish_locked(Outcome::FAILED, reason);
      return true;
    }
    if (!still_ours()) {
      return true;
    }
    if (!enable_locked(*start, ec)) {
      char reason[96];
      if (ec == std::errc::operation_not_permitted) {
        std::snprintf(reason, sizeof(reason), "drive in fault at the start; clear_faults()");
      } else {
        std::snprintf(reason, sizeof(reason), "enable failed: %s", ec.message().c_str());
      }
      finish_locked(Outcome::FAILED, reason);
      return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (result_.serial != serial || result_.outcome != Outcome::MOVING) {
      return true; // stopped while enabling: stop() disables after this tick
    }
    const auto t0 = Clock::now();
    profile_.plan(*start, 0.0, target_,
                  {.max_speed = static_cast<double>(move_profile_.velocity),
                   .acceleration = static_cast<double>(move_profile_.acceleration)});
    profile_t0_ = t0;
    last_handshake_ = t0 - kMinHandshakeHold;
    handshake_ = Handshake::SET;
    phase_ = Phase::STREAM;
    result_.start = *start;
    result_.profile_s = static_cast<float>(profile_.duration());
    logger_.debug("{}: enabled at {}, streaming to {} ({:.2f} s profile)", config_.name, *start,
                  target_, profile_.duration());
    return true;
  }

  // 3. STREAM. A fault ends the move (and is left for clear_faults()).
  uint32_t ticks = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ticks = ticks_;
  }
  if (ticks % kStatusEveryTicks == 0) {
    uint16_t statusword = 0;
    if (mcp_.read_statusword(config_.axis, statusword, ec)) {
      if (statusword & kStatuswordFault) {
        char reason[96];
        std::snprintf(reason, sizeof(reason),
                      "drive faulted (statusword 0x%04X); clear_faults() before moving",
                      statusword);
        finish_locked(Outcome::FAILED, reason);
        return true;
      }
    } else {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++consecutive_failures_;
      ++result_.sdo_failures;
    }
  }

  // The lean handshake: SET writes the target and controlword 0x3F, the next tick's CLEAR
  // writes 0x2F. A bare 0x607A write is ignored by this controller.
  bool send = false;
  Handshake step = Handshake::SET;
  int32_t setpoint = 0;
  bool final_setpoint = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!final_latched_ && now - last_handshake_ >= kMinHandshakeHold) {
      send = true;
      step = handshake_;
      const double t = seconds(now - profile_t0_);
      const double lead_s = std::chrono::duration<double>(config_.lead).count();
      const double tau = std::chrono::duration<double>(shaper_delay_).count();
      const double sp = profile_.shaped_setpoint(t, lead_s, tau);
      const Range limits = (calibration_mode_ || homing_) ? kUnlimited : range_;
      setpoint = std::clamp(static_cast<int32_t>(std::lround(sp)), limits.min, limits.max);
      final_setpoint = profile_.at(t - tau).done;
    }
  }
  if (send && still_ours()) {
    bool ok;
    if (step == Handshake::SET) {
      ok = client_.write_i32(object(kTargetPosition), 0, setpoint, ec) &&
           drive->set_controlword(kControlwordSet, ec);
    } else {
      ok = drive->set_controlword(kControlwordClear, ec);
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_handshake_ = now;
    if (step == Handshake::SET) {
      // Whatever happened, clear next: a SET whose reply was lost may still have raised
      // the bit, and the next SET needs a rising edge.
      handshake_ = Handshake::CLEAR;
      final_sent_ = ok && final_setpoint;
    } else if (ok) {
      handshake_ = Handshake::SET;
      final_latched_ = final_sent_;
    }
    if (!ok) {
      ++consecutive_failures_;
      ++result_.sdo_failures;
    }
  }

  // 4. Arrival: the profile has finished, the final target is latched, and the median has
  //    stayed within tolerance of the target for settle_time. Or the deadline has passed.
  bool arrived = false;
  bool late = false;
  char reason[96] = {};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double t = seconds(now - profile_t0_);
    // The shaped profile ends one shaper delay after the planned one.
    const double shaped_duration =
        profile_.duration() + std::chrono::duration<double>(shaper_delay_).count();
    const bool profile_done = t >= shaped_duration;
    if (profile_done && !profile_done_at_) {
      profile_done_at_ = profile_t0_ + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(shaped_duration));
    }
    const auto median = history_median();
    if (profile_done && final_latched_ && median &&
        std::abs(*median - target_) <= config_.tolerance) {
      if (!settled_since_) {
        settled_since_ = now;
      }
      arrived = now - *settled_since_ >= config_.settle_time;
    } else {
      settled_since_.reset();
    }
    const auto deadline = profile_t0_ +
                          std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(shaped_duration)) +
                          config_.arrival_margin;
    if (!arrived && now > deadline) {
      late = true;
      std::snprintf(reason, sizeof(reason),
                    "not within %ld counts %.1f s after the profile ended (median %ld)",
                    static_cast<long>(config_.tolerance),
                    std::chrono::duration<double>(config_.arrival_margin).count(),
                    static_cast<long>(median.value_or(count)));
    }
  }
  if (arrived) {
    finish_locked(Outcome::ARRIVED, "");
  } else if (late) {
    finish_locked(Outcome::FAILED, reason);
  }
  return true;
}

bool Actuator::poll() {
  if (!config_.commissioned) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (phase_ == Phase::IDLE) {
      return true; // nothing to do, nothing on the bus
    }
  }
  std::lock_guard<std::mutex> lock(mcp_mutex_);
  return tick_locked();
}

// ------------------------------------------------------------------ calibration

int32_t Actuator::clamp(int32_t counts) const {
  const Range limits = installed_limits();
  return std::clamp(counts, limits.min, limits.max);
}

bool Actuator::set_calibration_mode(bool enabled) {
  if (!require_commissioned("calibration mode")) {
    return false;
  }
  if (moving()) {
    logger_.error("{}: calibration mode refused, a move is in progress", config_.name);
    return false;
  }
  bool write_now;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (calibration_mode_ == enabled) {
      return true;
    }
    calibration_mode_ = enabled;
    write_now = initialized_;
  }
  if (!write_now) {
    return true; // initialize() installs whichever limits apply then
  }
  const bool ok = apply_clamp(installed_limits());
  if (ok) {
    logger_.warn("{}: calibration mode {}", config_.name,
                 enabled ? "ON, range clamp lifted" : "off, range clamp restored");
  }
  return ok;
}

bool Actuator::set_range(const Range &range) {
  if (range.min > range.max) {
    logger_.error("{}: rejected range [{}, {}], min is above max", config_.name, range.min,
                  range.max);
    return false;
  }
  if (moving()) {
    logger_.error("{}: set_range refused, a move is in progress", config_.name);
    return false;
  }
  bool write_now;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    range_ = range;
    write_now = config_.commissioned && initialized_ && !calibration_mode_ && !homing_;
  }
  logger_.info("{}: range set to [{}, {}]", config_.name, range.min, range.max);
  bool ok = true;
  if (config_.store && !config_.store->save_range(config_.name, range)) {
    ok = false;
  }
  if (write_now) {
    ok = apply_clamp(range) && ok;
  }
  return ok;
}

Actuator::Range Actuator::range() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return range_;
}

// ------------------------------------------------------------------ homing

bool Actuator::set_encoder(int32_t counts) {
  if (!require_commissioned("set_encoder")) {
    return false;
  }
  if (moving()) {
    logger_.error("{}: set_encoder refused, a move is in progress", config_.name);
    return false;
  }
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    // A running position loop would chase its old target the moment the count changes
    // under it (seen on the bench: set to 0 at 20000, the motor drove back to 20000).
    ok = disable_locked(ec) && mcp_.set_encoder(config_.axis, counts, ec);
  }
  note_result(ok, ec, "set_encoder");
  if (!ok) {
    return false;
  }
  int32_t readback = 0;
  int32_t spread = 0;
  if (!read_position_median(readback, spread)) {
    return false;
  }
  if (std::abs(readback - counts) > config_.tolerance) {
    logger_.error("{}: set encoder to {} but it reads {}", config_.name, counts, readback);
    return false;
  }
  return true;
}

bool Actuator::restore_position() {
  if (!config_.homing.required) {
    return true;
  }
  if (!config_.store) {
    return false;
  }
  int32_t saved = 0;
  if (!config_.store->load_position(config_.name, saved)) {
    return false;
  }
  if (!set_encoder(saved)) {
    logger_.error("{}: could not restore saved position {}", config_.name, saved);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    homed_ = true;
    last_saved_position_ = saved;
    last_save_time_ = Clock::now();
  }
  logger_.warn("{}: position {} restored from memory; assumes the joint did not move while "
               "powered off. home() to be sure.",
               config_.name, saved);
  return true;
}

bool Actuator::save_position(bool force) {
  if (!config_.homing.required || !config_.store) {
    return true; // nothing to remember for an absolute encoder
  }
  if (!homed()) {
    return true; // an unhomed count is meaningless
  }
  int32_t counts = 0;
  if (position_age() < config_.position_save_interval) {
    counts = *current_position();
  } else if (!read_position(counts)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto since = Clock::now() - last_save_time_;
    const bool moved = std::abs(counts - last_saved_position_) >= config_.position_save_threshold;
    if (!force && (since < config_.position_save_interval || !moved)) {
      return true;
    }
    last_saved_position_ = counts;
    last_save_time_ = Clock::now();
  }
  return config_.store->save_position(config_.name, counts);
}

bool Actuator::home() {
  if (!require_commissioned("home")) {
    return false;
  }
  if (!config_.homing.required) {
    logger_.info("{}: absolute encoder, nothing to home", config_.name);
    return true;
  }
  if (moving()) {
    logger_.error("{}: home refused, a move is in progress", config_.name);
    return false;
  }
  if (!initialized() && !initialize()) {
    return false;
  }
  const Homing &h = config_.homing;
  logger_.warn("{}: homing toward {} at {} counts/s", config_.name, h.direction < 0 ? "min" : "max",
               h.profile.velocity);
  // Lift the clamp: the count is arbitrary right now, so no range applies.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    homing_ = true;
    homed_ = false;
  }
  const auto finish = [&](bool ok) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      homing_ = false;
    }
    // Restore the clamp whatever happened, so the axis is never left unlimited.
    if (!apply_clamp(installed_limits())) {
      ok = false;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      homed_ = ok;
    }
    if (ok) {
      logger_.info("{}: homed, count set to {}", config_.name, h.home_count);
      save_position(true);
    } else {
      logger_.error("{}: homing FAILED; moves refused until it succeeds", config_.name);
    }
    return ok;
  };
  if (!apply_clamp(kUnlimited)) {
    return finish(false);
  }

  const auto segment = [&](int32_t from) {
    const int64_t wanted =
        static_cast<int64_t>(from) + static_cast<int64_t>(h.direction) * h.segment;
    return start_move(
        static_cast<int32_t>(std::clamp<int64_t>(wanted, kUnlimited.min + 1, kUnlimited.max - 1)),
        h.profile);
  };
  int32_t start = 0;
  int32_t spread = 0;
  if (!read_position_median(start, spread) || !segment(start)) {
    stop();
    return finish(false);
  }
  // Wait for the switch to stop the motor: the median count stops changing.
  const auto begun = Clock::now();
  auto last_change = begun;
  int32_t last = start;
  bool moved = false;
  bool on_switch = false;
  while (Clock::now() - begun < h.timeout) {
    std::this_thread::sleep_for(kHomingPoll);
    if (h.cancel && h.cancel()) {
      logger_.warn("{}: homing cancelled", config_.name);
      stop();
      return finish(false);
    }
    const MoveResult r = move_result();
    if (r.outcome == Outcome::FAILED || r.outcome == Outcome::STOPPED) {
      logger_.error("{}: homing approach ended: {}", config_.name, r.reason);
      return finish(false);
    }
    std::optional<int32_t> median;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      median = history_median();
    }
    if (r.outcome == Outcome::ARRIVED) {
      // A whole segment without meeting the switch: another one from here.
      if (!segment(r.final_count)) {
        stop();
        return finish(false);
      }
      last_change = Clock::now();
      continue;
    }
    if (!median) {
      continue;
    }
    if (std::abs(*median - last) >= h.stall_counts) {
      last = *median;
      last_change = Clock::now();
      moved = true;
    } else if ((moved || Clock::now() - begun > kHomingGrace) &&
               Clock::now() - last_change >= h.stall_time) {
      on_switch = true;
      break;
    }
  }
  if (!on_switch) {
    logger_.error("{}: homing did not stall within {} s", config_.name, h.timeout.count());
    stop();
    return finish(false);
  }
  // Stopped. A fault stops the motor too, and is not the switch.
  uint16_t statusword = 0;
  const bool have_statusword = read_statusword(statusword);
  stop();
  if (!have_statusword || (statusword & kStatuswordFault)) {
    logger_.error("{}: the approach stopped with the drive {} (statusword 0x{:04X}): not taken "
                  "as the switch. Find out why, then clear_faults().",
                  config_.name, have_statusword ? "in FAULT" : "unreadable", statusword);
    return finish(false);
  }
  if (!moved) {
    logger_.warn("{}: never moved during homing; already on the switch, or the motor is not "
                 "driving",
                 config_.name);
  }
  return finish(set_encoder(h.home_count));
}

} // namespace mib
