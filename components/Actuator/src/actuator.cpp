#include "actuator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <thread>

namespace mib {

using namespace std::chrono_literals;

namespace {
/// The clamp installed while calibrating or homing: wide enough for any travel, narrow
/// enough that max - min still fits a 32-bit integer if the controller computes the span.
constexpr Actuator::Range kUnlimited{-1'000'000'000, 1'000'000'000};
// How far ahead of the joint, in jog steps, a run of jogs may push the target (see jog()).
constexpr int64_t kJogLeadSteps = 10;
/// Homing: the motor is given this long to start moving before a stall counts.
constexpr auto kHomingGrace = 2s;
constexpr auto kHomingPoll = 100ms;
/// poll(): how often the target-reached flag is read while a release is pending, how long
/// the joint settles at the target before the release, how often a released joint's count
/// is read, and the least time between two corrections of a joint that keeps drifting.
constexpr auto kReleasePoll = 100ms;
constexpr auto kReleaseDelay = 500ms;
constexpr auto kWatchPoll = 200ms;
constexpr auto kCorrectionHoldoff = 1s;
/// A count that has stayed within this band for kStillTime is "still".
constexpr int32_t kStillCounts = 3;
constexpr auto kStillTime = 500ms;
/// A joint not arrived this long after its last setpoint is released where it stalled.
constexpr auto kStallRelease = 3s;
/// The trajectory never advances by more than this per step, so a stalled poll (the bus
/// waited on a silent controller) does not jump the setpoint when it resumes.
constexpr auto kTrajectoryMaxStep = 100ms;
} // namespace

Actuator::Actuator(espp::Mcp266 &mcp, std::mutex &mcp_mutex, const Config &config)
    : BaseComponent(config.name ? config.name : "Actuator", config.log_level)
    , mcp_(mcp)
    , mcp_mutex_(mcp_mutex)
    , config_(config)
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

bool Actuator::apply_motion_defaults(std::error_code &ec) {
  std::lock_guard<std::mutex> lock(mcp_mutex_);
  // The controller runs position moves on its stored default speed (a ceiling on the
  // per-move velocity) and default accel (the ramp), reloaded from EEPROM at power-up and
  // not persisted by its save command, so they are installed here every boot. The profile
  // velocity is the ceiling: a move may ask for less, never more.
  const uint16_t speed =
      static_cast<uint16_t>(std::min<uint32_t>(config_.profile.velocity, UINT16_MAX));
  return mcp_.set_default_speed(config_.axis, speed, ec) &&
         mcp_.set_default_accel(config_.axis, config_.profile.acceleration, ec);
}

Actuator::Range Actuator::installed_limits() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return calibration_mode_ ? kUnlimited : range_;
}

bool Actuator::read_controller_limits(int32_t &min_counts, int32_t &max_counts) {
  if (!check_online()) {
    return false;
  }
  // Two SDOs: just the MinPos/MaxPos clamp of the position PID record.
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.read_position_limits(config_.axis, min_counts, max_counts, ec);
  }
  note_result(ok, ec, "read_controller_limits");
  return ok;
}

bool Actuator::verify_limits() {
  const Range expected = installed_limits();
  int32_t min = 0;
  int32_t max = 0;
  if (!read_controller_limits(min, max)) {
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
  if (ok) {
    ok = apply_motion_defaults(ec);
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
  logger_.info("{}: initialised, range [{}, {}] counts, speed ceiling {} counts/s, ramp {} "
               "counts/s^2{}{}",
               config_.name, range.min, range.max, config_.profile.velocity,
               config_.profile.acceleration,
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

bool Actuator::moving() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return trajectory_.active || pending_release_;
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
  active = (statusword & (1u << 11)) != 0;
  return true;
}

bool Actuator::clear_faults() {
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool faults_ok;
  bool estop_ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    faults_ok = mcp_.reset_faults(ec);
    std::error_code estop_ec;
    estop_ok = mcp_.reset_estop(estop_ec); // device-wide; rejected when nothing is latched
    if (!estop_ok) {
      logger_.debug("{}: E-stop reset not accepted: {}", config_.name, estop_ec.message());
    }
  }
  note_result(faults_ok, ec, "clear_faults");
  if (faults_ok) {
    logger_.info("{}: faults cleared{}", config_.name, estop_ok ? " (E-stop reset accepted)" : "");
    release_quick_stop();
  }
  return faults_ok;
}

bool Actuator::release_quick_stop() {
  std::error_code ec;
  std::lock_guard<std::mutex> lock(mcp_mutex_);
  auto *drive = mcp_.drive(config_.axis);
  if (drive == nullptr) {
    return false;
  }
  if (drive->get_state(ec) != DriveState::QuickStopActive || ec) {
    return !ec;
  }
  // CiA 402 transition 16: quick stop active -> operation enabled, allowed when the
  // drive's quick-stop option holds position. Keeps the axis held if it works.
  if (command_state(espp::detail::ds402::CW_ENABLE_OPERATION, DriveState::OperationEnabled, ec)) {
    logger_.info("{}: quick stop released, axis re-enabled", config_.name);
    return true;
  }
  // Transition 12: disable voltage -> switch on disabled. The axis coasts until the next
  // move re-enables it.
  ec.clear();
  if (command_state(espp::detail::ds402::CW_DISABLE_VOLTAGE, DriveState::SwitchOnDisabled, ec)) {
    logger_.info("{}: quick stop released, axis disabled until the next move", config_.name);
    return true;
  }
  logger_.error("{}: could not leave quick stop: {}", config_.name, ec.message());
  return false;
}

bool Actuator::command_state(uint16_t controlword, DriveState wanted, std::error_code &ec) {
  auto *drive = mcp_.drive(config_.axis);
  if (drive == nullptr || !drive->set_controlword(controlword, ec)) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(25ms);
    const auto state = drive->get_state(ec);
    if (ec) {
      return false;
    }
    if (state == wanted) {
      return true;
    }
  }
  return false;
}

bool Actuator::disable_loop(std::error_code &ec) {
  auto *drive = mcp_.drive(config_.axis);
  if (drive == nullptr) {
    return false;
  }
  const auto state = drive->get_state(ec);
  if (ec) {
    return false;
  }
  if (state != DriveState::OperationEnabled && state != DriveState::QuickStopActive) {
    return true; // no loop running
  }
  if (!command_state(espp::detail::ds402::CW_DISABLE_VOLTAGE, DriveState::SwitchOnDisabled, ec)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  pending_release_ = false; // nothing left for poll() to release
  return true;
}

// ------------------------------------------------------------------ moves

bool Actuator::do_move(int32_t counts, const Profile &profile, bool jog) {
  if (!initialized()) {
    logger_.error("{}: move refused, actuator is not initialised (see the initialize() log)",
                  config_.name);
    return false;
  }
  // An unhomed incremental encoder has no meaningful count, so moves are refused, except
  // in calibration mode: that is the mode for jogging without a valid range or count, and
  // the clamp is lifted for it.
  if (!ready() && !calibration_mode()) {
    logger_.error("{}: move refused, needs homing (or calibration mode for a jog)",
                  config_.name);
    return false;
  }
  // With the clamp lifted, a long move has nothing to stop it; only jogs are allowed.
  if (calibration_mode() && !jog) {
    logger_.error("{}: move to {} refused, only increment/decrement are allowed in calibration "
                  "mode (install a range and leave it for longer moves)",
                  config_.name, counts);
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
    // A held jog button sends jogs past the end several times a second; that is the clamp
    // doing its job, not something to warn about each time.
    if (jog) {
      logger_.debug("{}: jog target {} clamped to {}", config_.name, counts, clamped);
    } else {
      logger_.warn("{}: target {} clamped to {}", config_.name, counts, clamped);
    }
  }
  if (!release_quick_stop()) {
    return false;
  }
  // A limit switch that stopped the previous move leaves the controller's position loop
  // wound up against an unreachable target; the next move, which releases the limit,
  // then runs away (seen on the bench). Restart the drive from "switch on disabled" so
  // the loop starts clean.
  {
    bool on_limit = false;
    if (limit_active(on_limit) && on_limit) {
      // The statusword says a switch is active but not which one. The move that was
      // stopped was heading toward it, so a new move in that same direction would only
      // push into the switch again: refuse it. A move the other way is allowed after the
      // restart.
      int32_t here = 0;
      const auto stopped_target = last_target();
      if (read_position(here) && stopped_target) {
        const bool stopped_forward = *stopped_target > here;
        const bool new_forward = clamped > here;
        if (clamped != here && stopped_forward == new_forward) {
          logger_.warn("{}: move to {} refused, a limit switch is active in that direction",
                       config_.name, clamped);
          return false;
        }
      }
      logger_.warn("{}: limit switch active; restarting the drive before moving away",
                   config_.name);
      std::error_code reset_ec;
      std::lock_guard<std::mutex> lock(mcp_mutex_);
      if (!disable_loop(reset_ec)) {
        logger_.error("{}: could not restart the drive: {}", config_.name, reset_ec.message());
        return false;
      }
    }
  }
  // The trapezoid starts where the joint is, or where the setpoint of a move still in
  // progress is (with its velocity), so a retarget mid-move stays smooth.
  int32_t here = 0;
  if (!read_position(here)) {
    return false;
  }
  double start = here;
  double start_velocity = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (trajectory_.active) {
      start = trajectory_.setpoint;
      start_velocity = trajectory_.velocity;
    }
  }
  const auto first = static_cast<int32_t>(std::lround(start));
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    // Enables the drive and sets the profile objects; the setpoints that follow from
    // poll() only need the target write.
    ok = mcp_.move_to_position(config_.axis, first, profile.velocity, profile.acceleration,
                               profile.deceleration, ec);
    if (ok) {
      // Still holding the controller: a poll() that was about to release the previous
      // move sees the new serial before it can touch the drive.
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      last_target_ = clamped;
      pending_release_ = true;
      watching_ = false;
      ++move_serial_;
      reached_since_.reset();
      still_reference_.reset();
      rest_position_.reset();
      trajectory_.active = clamped != first;
      stream_done_at_ =
          trajectory_.active ? std::nullopt : std::optional{std::chrono::steady_clock::now()};
      trajectory_.setpoint = start;
      trajectory_.velocity = start_velocity;
      trajectory_.target = clamped;
      trajectory_.profile = profile;
      trajectory_.last = std::chrono::steady_clock::now();
    }
  }
  note_result(ok, ec, "move");
  if (ok) {
    logger_.debug("{}: moving {} -> {} at {} counts/s, ramps {} / {}", config_.name, first, clamped,
                  profile.velocity, profile.acceleration, profile.deceleration);
  }
  return ok;
}

bool Actuator::step_trajectory() {
  int32_t setpoint = 0;
  bool done = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!trajectory_.active) {
      return true;
    }
    Trajectory &t = trajectory_;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(
                          std::min<std::chrono::nanoseconds>(now - t.last, kTrajectoryMaxStep))
                          .count();
    t.last = now;
    const double accel = std::max<double>(t.profile.acceleration, 1.0);
    const double decel = std::max<double>(t.profile.deceleration, 1.0);
    const double cruise = std::max<double>(t.profile.velocity, 1.0);
    const double remaining = t.target - t.setpoint; // signed
    const double direction = remaining < 0 ? -1.0 : 1.0;
    const double toward = t.velocity * direction; // speed along the way to the target
    if (toward < 0) {
      // Heading away (a retarget behind the joint): brake first.
      t.velocity += direction * decel * dt;
    } else if (toward * toward / (2.0 * decel) >= std::fabs(remaining)) {
      // Within the stopping distance: slow down, never below standstill.
      t.velocity = direction * std::max(toward - decel * dt, 0.0);
    } else {
      t.velocity = direction * std::min(toward + accel * dt, cruise);
    }
    t.setpoint += t.velocity * dt;
    const double left = (t.target - t.setpoint) * direction;
    if (left <= 0.0 || (std::fabs(t.velocity) < 1.0 && std::fabs(left) < 1.0)) {
      t.setpoint = t.target;
      t.velocity = 0;
      t.active = false;
      stream_done_at_ = now;
      done = true;
    }
    setpoint = static_cast<int32_t>(std::lround(t.setpoint));
  }
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    auto *drive = mcp_.drive(config_.axis);
    ok = drive != nullptr && drive->set_target_position(setpoint, ec);
  }
  note_result(ok, ec, "setpoint");
  if (!ok) {
    // Do not keep streaming into a controller that is not answering; the final target is
    // not sent either, so the joint stays short of it and the release watch stays off.
    std::lock_guard<std::mutex> lock(state_mutex_);
    trajectory_.active = false;
    pending_release_ = false;
    logger_.error("{}: move abandoned at setpoint {}: see the log", config_.name, setpoint);
    return false;
  }
  if (done) {
    logger_.debug("{}: trajectory done, final target {} sent", config_.name, setpoint);
  }
  return true;
}

bool Actuator::poll() {
  if (!step_trajectory()) {
    return false;
  }
  enum class Step { nothing, release, watch };
  Step step = Step::nothing;
  uint32_t serial = 0;
  int32_t target = 0;
  int32_t rest = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (now < next_poll_ || trajectory_.active) {
      return true; // nothing due, or setpoints still on their way to the target
    }
    if (pending_release_) {
      step = Step::release;
      serial = move_serial_;
      target = last_target_.value_or(0);
      next_poll_ = now + kReleasePoll;
    } else if (watching_ && last_target_ && rest_position_ && !calibration_mode_) {
      step = Step::watch;
      target = *last_target_;
      rest = *rest_position_;
      next_poll_ = now + kWatchPoll;
    }
  }
  if (step == Step::nothing) {
    return true;
  }
  if (step == Step::watch) {
    int32_t here = 0;
    if (!read_position(here)) {
      return false;
    }
    if (std::abs(here - rest) <= config_.tolerance) {
      return true;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (now - last_correction_ < kCorrectionHoldoff) {
        return true; // it moved again at once: not more often than this
      }
      last_correction_ = now;
    }
    logger_.warn("{}: at rest it moved from {} to {} ({} counts): correcting toward {}",
                 config_.name, rest, here, here - rest, target);
    return do_move(target, config_.profile);
  }

  // Arrived when the controller says so, or when the count has settled within tolerance
  // of the target: a P-only loop can stall short of the controller's own window and
  // never raise its flag, and a joint that is close and still is done by any measure.
  int32_t here = 0;
  if (!read_position(here)) {
    return false;
  }
  bool still = false;
  bool stalled = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!still_reference_ || std::abs(here - *still_reference_) >= kStillCounts) {
      still_reference_ = here;
      still_since_ = now;
    }
    still = now - still_since_ >= kStillTime;
    stalled = stream_done_at_ && now - *stream_done_at_ >= kStallRelease;
  }
  bool arrived = still && std::abs(here - target) <= config_.tolerance;
  if (!arrived && !stalled) {
    bool reached = false;
    if (!is_target_reached(reached)) {
      return false;
    }
    arrived = reached;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!arrived && !stalled) {
      reached_since_.reset();
      return true;
    }
    if (!reached_since_) {
      reached_since_ = now;
    }
    if (arrived && now - *reached_since_ < kReleaseDelay) {
      return true;
    }
  }
  std::error_code ec;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (!pending_release_ || move_serial_ != serial) {
        return true; // a newer move went out meanwhile; leave its loop alone
      }
    }
    if (!disable_loop(ec)) {
      logger_.error("{}: arrived at {} but the drive would not release: {}", config_.name, here,
                    ec.message());
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      pending_release_ = false; // do not hammer the bus; the next move re-enables anyway
      return false;
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    watching_ = true;
    rest_position_ = here;
  }
  if (arrived) {
    logger_.info("{}: arrived at {}, motor released", config_.name, here);
  } else {
    logger_.warn("{}: stalled at {}, {} counts short of {}, {} s after the last setpoint: "
                 "motor released (the loop cannot close that; tune it)",
                 config_.name, here, target - here, target,
                 std::chrono::duration_cast<std::chrono::seconds>(kStallRelease).count());
  }
  return true;
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

bool Actuator::jog(int32_t delta) {
  int32_t current = 0;
  if (!read_position(current)) {
    return false;
  }
  // A held jog button sends these faster than the joint moves, and a joint that has not
  // broken free yet has not moved at all (seen on the bench: a 500-count target gave no
  // motion). Jogging on from the previous target while it still lies ahead in the same
  // direction advances the target steadily; from the position, every jog would re-aim at
  // the same point. The lead is capped so the joint never chases a far-off target.
  int64_t base = current;
  if (const auto previous = last_target(); previous && delta != 0) {
    const int64_t lead = static_cast<int64_t>(*previous) - current;
    const bool ahead = delta > 0 ? lead > 0 : lead < 0;
    if (ahead && std::abs(lead) <= kJogLeadSteps * static_cast<int64_t>(std::abs(delta))) {
      base = *previous;
    }
  }
  const int64_t target = base + delta;
  return do_move(static_cast<int32_t>(std::clamp<int64_t>(target, INT32_MIN, INT32_MAX)),
                 config_.profile, true);
}

bool Actuator::increment() { return jog(config_.jog_step); }

bool Actuator::decrement() { return jog(-config_.jog_step); }

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
  if (ok) {
    // Whoever stopped it wants it still: no more setpoints, no release to finish, no
    // drift to correct.
    std::lock_guard<std::mutex> lock(state_mutex_);
    trajectory_.active = false;
    pending_release_ = false;
    watching_ = false;
  }
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
    watching_ = false; // a target from before the clamp changed is not one to go back to
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
  if (!check_online()) {
    return false;
  }
  std::error_code ec;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    // A running position loop would chase its old target the moment the count changes
    // under it (seen on the bench: set to 0 at 20000, the motor drove back to 20000).
    // Take the loop down first; the next move re-enables with a fresh target.
    ok = disable_loop(ec);
    if (!ok) {
      logger_.error("{}: cannot disable the loop before setting the encoder: {}", config_.name,
                    ec.message());
    } else {
      ok = mcp_.set_encoder(config_.axis, counts, ec);
    }
  }
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    watching_ = false; // the old target means nothing against the new count
    trajectory_.active = false;
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
    homed_ = true;            // so do_move accepts the approach; cleared below on failure
    pending_release_ = false; // the approach is commanded here, not through do_move()
    watching_ = false;
    trajectory_.active = false;
  }

  // One approach segment from `from`, in the homing direction. Returns false only when
  // the command was refused; an acknowledge timeout on a long move is treated as running.
  const auto command_segment = [&](int32_t from) {
    const int64_t wanted = static_cast<int64_t>(from) + static_cast<int64_t>(h.direction) * h.segment;
    const int32_t target = static_cast<int32_t>(
        std::clamp<int64_t>(wanted, kUnlimited.min + 1, kUnlimited.max - 1));
    std::error_code seg_ec;
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    const bool sent = mcp_.move_to_position(config_.axis, target, h.profile.velocity,
                                            h.profile.acceleration, h.profile.deceleration, seg_ec);
    if (!sent && seg_ec == std::errc::timed_out) {
      logger_.warn("{}: approach handshake timed out; assuming the move is running",
                   config_.name);
      return true;
    }
    if (!sent) {
      ec = seg_ec;
    }
    return sent;
  };

  int32_t start = 0;
  bool ok = read_position(start) && command_segment(start);
  if (!ok) {
    note_result(false, ec, "home: approach");
    stop(); // never leave an approach running after a failure
  } else {
    // Wait for the limit switch to stop the motor: the count stops changing.
    const auto begun = std::chrono::steady_clock::now();
    auto last_change = begun;
    int32_t last = start;
    bool moved = false;
    ok = false;
    bool cancelled = false;
    while (std::chrono::steady_clock::now() - begun < h.timeout) {
      std::this_thread::sleep_for(kHomingPoll);
      if (h.cancel && h.cancel()) {
        cancelled = true;
        break;
      }
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
        // Stopped. On the switch, or merely at the end of a segment: the controller
        // tells the two apart with its target-reached flag.
        bool reached = false;
        if (is_target_reached(reached) && reached) {
          logger_.info("{}: segment finished at {} without a stall; continuing", config_.name,
                       now_counts);
          if (!command_segment(now_counts)) {
            break;
          }
          last_change = std::chrono::steady_clock::now();
          continue;
        }
        ok = true; // stopped short of the target: the switch
        break;
      }
    }
    if (cancelled) {
      logger_.warn("{}: homing cancelled", config_.name);
    } else if (!ok) {
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
    // Driving into the switch may have latched a fault or a limit condition on the
    // controller; clear it before writing the count and before the next move.
    clear_faults();
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
