#include "actuator.hpp"

#include <algorithm>
#include <thread>

#include "detail/mcp266_core.hpp"

namespace mib {

using namespace std::chrono_literals;

namespace {
/// The widest clamp the controller accepts; used while calibrating or homing.
constexpr Actuator::Range kUnlimited{-2'000'000'000, 2'000'000'000};
/// Basicmicro packet-serial commands mirrored into the CANopen manufacturer region.
constexpr uint8_t kSetEncoderM1 = 22;
constexpr uint8_t kSetEncoderM2 = 23;
/// Homing: the motor is given this long to start moving before a stall counts.
constexpr auto kHomingGrace = 2s;
constexpr auto kHomingPoll = 100ms;
} // namespace

Actuator::Actuator(espp::Mcp266 &mcp, espp::CanopenClient &client, std::mutex &mcp_mutex,
                   const Config &config)
    : BaseComponent(config.name ? config.name : "Actuator", config.log_level)
    , mcp_(mcp)
    , client_(client)
    , mcp_mutex_(mcp_mutex)
    , config_(config)
    , position_pid_get_(config.axis == Axis::M1
                            ? espp::detail::mcp266::axis_m1().position_pid_get
                            : espp::detail::mcp266::axis_m2().position_pid_get)
    , set_encoder_object_(espp::detail::mcp266::command_object(
          config.axis == Axis::M1 ? kSetEncoderM1 : kSetEncoderM2))
    , range_(config.range) {}

// ------------------------------------------------------------------ online marking

bool Actuator::check_online() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (online_) {
    return true;
  }
  if (std::chrono::steady_clock::now() - offline_since_ >= config_.offline_retry) {
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
    logger_.warn("{}: {} timed out waiting on the drive state; controller still answers",
                 config_.name, what);
    return;
  }
  if (online_) {
    logger_.error("{}: {} got no answer; controller marked offline for {} ms", config_.name,
                  what, config_.offline_retry.count());
  }
  online_ = false;
  offline_since_ = std::chrono::steady_clock::now();
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
  return initialized_ && (homed_ || !config_.homing.required);
}

bool Actuator::calibration_mode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return calibration_mode_;
}

// ------------------------------------------------------------------ limits

bool Actuator::apply_limits(const Range &range, std::error_code &ec) {
  std::lock_guard<std::mutex> lock(mcp_mutex_);
  // The factory position clamp is [0, 0], which forces every target to zero; this
  // widens it to the travel and seeds a P gain if the drive has none.
  if (!mcp_.configure_position_loop(config_.axis, range.min, range.max, ec)) {
    return false;
  }
  return mcp_.set_software_position_limits(config_.axis, range.min, range.max, ec);
}

Actuator::Range Actuator::installed_limits() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return calibration_mode_ ? kUnlimited : range_;
}

bool Actuator::verify_limits() {
  // Reads MinPos/MaxPos of the position PID record straight from the client because
  // espp::Mcp266 (1.3.6) exposes no read-back of the clamp it writes.
  const Range expected = installed_limits();
  std::error_code ec;
  int32_t min = 0;
  int32_t max = 0;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    min = client_.read_i32(position_pid_get_, 6, ec);
    if (!ec) {
      max = client_.read_i32(position_pid_get_, 7, ec);
    }
  }
  if (ec) {
    note_result(false, ec, "verify_limits");
    return false;
  }
  if (min == expected.min && max == expected.max) {
    return true;
  }
  logger_.error("{}: controller clamp is [{}, {}], expected [{}, {}]: it has reset; "
                "re-initialising",
                config_.name, min, max, expected.min, expected.max);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = false;
    homed_ = false;
  }
  return initialize();
}

bool Actuator::initialize() {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.reset_faults(ec);
  }
  if (ok) {
    ok = apply_limits(installed_limits(), ec);
  }
  note_result(ok, ec, "initialize");
  if (!ok) {
    return false;
  }
  Range range;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = true;
    homed_ = !config_.homing.required; // an absolute encoder is always "homed"
    range = range_;
  }
  if (config_.homing.required) {
    if (!restore_position()) {
      logger_.warn("{}: incremental encoder with no saved position; home() before moving",
                   config_.name);
    }
  }
  logger_.info("{}: initialised, range [{}, {}] counts{}{}", config_.name, range.min, range.max,
               config_.hardware_limits ? ", limit switches on the controller" : "",
               ready() ? "" : ", NOT ready (needs homing)");
  return true;
}

// ------------------------------------------------------------------ position

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
    position_time_ = std::chrono::steady_clock::now();
  }
  return ok;
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
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               position_time_);
}

std::optional<int32_t> Actuator::last_target() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_target_;
}

bool Actuator::is_target_reached(bool &reached) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.is_target_reached(config_.axis, reached, ec);
  }
  note_result(ok, ec, "is_target_reached");
  return ok;
}

bool Actuator::get_drive_state(DriveState &state) {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.get_state(config_.axis, state, ec);
  }
  note_result(ok, ec, "get_drive_state");
  return ok;
}

// ------------------------------------------------------------------ moves

bool Actuator::do_move(int32_t counts, const Profile &profile) {
  if (!ready()) {
    logger_.error("{}: move refused, actuator is not ready ({})", config_.name,
                  initialized() ? "needs homing" : "not initialised");
    return false;
  }
  if (!check_online()) {
    return false;
  }
  // Two SDOs to catch a controller that rebooted since we configured it; without this a
  // reset controller would force every target to its factory [0, 0] clamp.
  if (!verify_limits()) {
    return false;
  }
  const int32_t clamped = clamp(counts);
  if (clamped != counts) {
    logger_.warn("{}: target {} clamped to {}", config_.name, counts, clamped);
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.move_to_position(config_.axis, clamped, profile.velocity, profile.acceleration,
                               profile.deceleration, ec);
  }
  note_result(ok, ec, "move");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_target_ = clamped;
    logger_.debug("{}: moving to {} at {} counts/s", config_.name, clamped, profile.velocity);
  }
  return ok;
}

bool Actuator::move_absolute(int32_t counts) { return do_move(counts, config_.profile); }

bool Actuator::move_absolute(int32_t counts, const Profile &profile) {
  return do_move(counts, profile);
}

bool Actuator::move_relative(int32_t delta) { return move_relative(delta, config_.profile); }

bool Actuator::move_relative(int32_t delta, const Profile &profile) {
  int32_t current = 0;
  if (!read_position(current)) {
    return false;
  }
  const int64_t target = static_cast<int64_t>(current) + delta;
  return do_move(static_cast<int32_t>(std::clamp<int64_t>(target, INT32_MIN, INT32_MAX)),
                 profile);
}

bool Actuator::increment() { return move_relative(config_.jog_step); }

bool Actuator::decrement() { return move_relative(-config_.jog_step); }

bool Actuator::stop() {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    auto *drive = mcp_.drive(config_.axis);
    ok = drive != nullptr && drive->quick_stop(ec);
  }
  note_result(ok, ec, "stop");
  return ok;
}

// ------------------------------------------------------------------ calibration

int32_t Actuator::clamp(int32_t counts) const {
  const Range limits = installed_limits();
  return std::clamp(counts, limits.min, limits.max);
}

bool Actuator::set_calibration_mode(bool enabled) {
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
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  const bool ok = apply_limits(installed_limits(), ec);
  note_result(ok, ec, "set_calibration_mode");
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
  bool write_now;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    range_ = range;
    write_now = initialized_ && !calibration_mode_;
  }
  logger_.info("{}: range set to [{}, {}]", config_.name, range.min, range.max);
  bool ok = true;
  if (config_.store && !config_.store->save_range(config_.name, range)) {
    ok = false;
  }
  if (write_now) {
    if (!check_online()) {
      return false;
    }
    std::error_code ec;
    const bool applied = apply_limits(range, ec);
    note_result(applied, ec, "set_range");
    ok = ok && applied;
  }
  return ok;
}

Actuator::Range Actuator::range() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return range_;
}

// ------------------------------------------------------------------ homing

bool Actuator::set_encoder(int32_t counts) {
  // Basicmicro command 22/23 mirrored at 0x2016/0x2017; espp::Mcp266 (1.3.6) has no
  // wrapper for it, hence the direct client write.
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = client_.write_i32(set_encoder_object_, 0, counts, ec);
  }
  note_result(ok, ec, "set_encoder");
  if (!ok) {
    return false;
  }
  int32_t readback = 0;
  if (!read_position(readback)) {
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
    last_save_time_ = std::chrono::steady_clock::now();
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
    const auto since = std::chrono::steady_clock::now() - last_save_time_;
    const bool moved = std::abs(counts - last_saved_position_) >= config_.position_save_threshold;
    if (!force && (since < config_.position_save_interval || !moved)) {
      return true;
    }
    last_saved_position_ = counts;
    last_save_time_ = std::chrono::steady_clock::now();
  }
  return config_.store->save_position(config_.name, counts);
}

bool Actuator::home() {
  if (!config_.homing.required) {
    logger_.info("{}: absolute encoder, nothing to home", config_.name);
    return true;
  }
  if (!initialized() && !initialize()) {
    return false;
  }
  const Homing &h = config_.homing;
  logger_.warn("{}: homing toward {} at {} counts/s", config_.name, h.direction < 0 ? "min" : "max",
               h.profile.velocity);

  // Lift the clamp: the count is arbitrary right now, so no range applies.
  std::error_code ec;
  if (!apply_limits(kUnlimited, ec)) {
    note_result(false, ec, "home: lift limits");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    homed_ = true; // so do_move accepts the approach; cleared below on failure
  }

  int32_t start = 0;
  bool ok = read_position(start);
  const int32_t far = h.direction < 0 ? -1'900'000'000 : 1'900'000'000;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = ok && mcp_.move_to_position(config_.axis, far, h.profile.velocity,
                                     h.profile.acceleration, h.profile.deceleration, ec);
  }
  if (!ok) {
    note_result(false, ec, "home: approach");
  } else {
    // Wait for the limit switch to stop the motor: the count stops changing.
    const auto begun = std::chrono::steady_clock::now();
    auto last_change = begun;
    int32_t last = start;
    bool moved = false;
    ok = false;
    while (std::chrono::steady_clock::now() - begun < h.timeout) {
      std::this_thread::sleep_for(kHomingPoll);
      int32_t now_counts = 0;
      if (!read_position(now_counts)) {
        break;
      }
      if (std::abs(now_counts - last) >= h.stall_counts) {
        last = now_counts;
        last_change = std::chrono::steady_clock::now();
        moved = true;
      } else if ((moved || std::chrono::steady_clock::now() - begun > kHomingGrace) &&
                 std::chrono::steady_clock::now() - last_change >= h.stall_time) {
        ok = true; // stopped on the switch
        break;
      }
    }
    if (!ok) {
      logger_.error("{}: homing did not stall within {} s", config_.name, h.timeout.count());
    }
    stop();
    if (!moved && ok) {
      logger_.warn("{}: never moved during homing; already on the switch, or the motor is not "
                   "driving",
                   config_.name);
    }
  }

  if (ok) {
    ok = set_encoder(h.home_count);
  }

  // Restore the clamp whatever happened, so the axis is never left unlimited.
  std::error_code restore_ec;
  if (!apply_limits(installed_limits(), restore_ec)) {
    note_result(false, restore_ec, "home: restore limits");
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
}

} // namespace mib
