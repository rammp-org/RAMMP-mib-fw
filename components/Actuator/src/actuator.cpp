#include "actuator.hpp"

#include <algorithm>

namespace mib {

namespace {
/// The widest clamp the controller accepts; used while calibrating.
constexpr Actuator::Range kUnlimited{-2'000'000'000, 2'000'000'000};
} // namespace

Actuator::Actuator(espp::Mcp266 &mcp, std::mutex &mcp_mutex, const Config &config)
    : BaseComponent(config.name ? config.name : "Actuator", config.log_level)
    , mcp_(mcp)
    , mcp_mutex_(mcp_mutex)
    , config_(config)
    , range_(config.range) {}

// ------------------------------------------------------------------ online marking

bool Actuator::check_online(std::error_code &ec) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (online_) {
    return true;
  }
  const auto waited = std::chrono::steady_clock::now() - offline_since_;
  if (waited >= config_.offline_retry) {
    return true; // let this call probe the controller again
  }
  ec = std::make_error_code(std::errc::host_unreachable);
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

// ------------------------------------------------------------------ setup

bool Actuator::apply_limits(const Range &range, std::error_code &ec) {
  std::lock_guard<std::mutex> lock(mcp_mutex_);
  // The factory position clamp is [0, 0], which forces every target to zero; this
  // widens it to the travel and seeds a P gain if the drive has none.
  if (!mcp_.configure_position_loop(config_.axis, range.min, range.max, ec)) {
    return false;
  }
  return mcp_.set_software_position_limits(config_.axis, range.min, range.max, ec);
}

bool Actuator::initialize(std::error_code &ec) {
  ec.clear();
  if (!check_online(ec)) {
    return false;
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.reset_faults(ec);
  }
  if (ok) {
    Range range;
    bool calibrating;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      range = range_;
      calibrating = calibration_mode_;
    }
    ok = apply_limits(calibrating ? kUnlimited : range, ec);
  }
  note_result(ok, ec, "initialize");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initialized_ = true;
    logger_.info("{}: ready, range [{}, {}] counts{}", config_.name, range_.min, range_.max,
                 config_.hardware_limits ? ", limit switches on the controller" : "");
  }
  return ok;
}

bool Actuator::set_range(const Range &range, std::error_code &ec) {
  ec.clear();
  if (range.min > range.max) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  bool write_now;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    range_ = range;
    write_now = initialized_ && !calibration_mode_;
  }
  logger_.info("{}: range set to [{}, {}]", config_.name, range.min, range.max);
  if (!write_now) {
    return true;
  }
  if (!check_online(ec)) {
    return false;
  }
  const bool ok = apply_limits(range, ec);
  note_result(ok, ec, "set_range");
  return ok;
}

Actuator::Range Actuator::range() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return range_;
}

bool Actuator::set_calibration_mode(bool enabled, std::error_code &ec) {
  ec.clear();
  Range range;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (calibration_mode_ == enabled) {
      return true;
    }
    calibration_mode_ = enabled;
    range = range_;
    if (!initialized_) {
      return true; // initialize() will install whichever limits apply then
    }
  }
  if (!check_online(ec)) {
    return false;
  }
  const bool ok = apply_limits(enabled ? kUnlimited : range, ec);
  note_result(ok, ec, "set_calibration_mode");
  logger_.warn("{}: calibration mode {}", config_.name, enabled ? "ON, range clamp lifted" : "off");
  return ok;
}

int32_t Actuator::clamp(int32_t target) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (calibration_mode_) {
    return target;
  }
  return std::clamp(target, range_.min, range_.max);
}

// ------------------------------------------------------------------ reads

bool Actuator::get_position(int32_t &counts, std::error_code &ec) {
  ec.clear();
  if (!check_online(ec)) {
    return false;
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.read_encoder(config_.axis, counts, ec);
  }
  note_result(ok, ec, "get_position");
  if (ok) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_position_ = counts;
  }
  return ok;
}

std::optional<int32_t> Actuator::last_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_position_;
}

bool Actuator::is_target_reached(bool &reached, std::error_code &ec) {
  ec.clear();
  if (!check_online(ec)) {
    return false;
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.is_target_reached(config_.axis, reached, ec);
  }
  note_result(ok, ec, "is_target_reached");
  return ok;
}

bool Actuator::get_drive_state(DriveState &state, std::error_code &ec) {
  ec.clear();
  if (!check_online(ec)) {
    return false;
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.get_state(config_.axis, state, ec);
  }
  note_result(ok, ec, "get_drive_state");
  return ok;
}

// ------------------------------------------------------------------ moves

bool Actuator::do_move(int32_t target, const Profile &profile, std::error_code &ec) {
  ec.clear();
  bool ready;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ready = initialized_;
  }
  if (!ready) {
    logger_.error("{}: move refused, initialize() has not succeeded", config_.name);
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (!check_online(ec)) {
    return false;
  }
  const int32_t clamped = clamp(target);
  if (clamped != target) {
    logger_.warn("{}: target {} clamped to {}", config_.name, target, clamped);
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    ok = mcp_.move_to_position(config_.axis, clamped, profile.velocity, profile.acceleration,
                               profile.deceleration, ec);
  }
  note_result(ok, ec, "move");
  if (ok) {
    logger_.debug("{}: moving to {} at {} counts/s", config_.name, clamped, profile.velocity);
  }
  return ok;
}

bool Actuator::move_absolute(int32_t target, std::error_code &ec) {
  return do_move(target, config_.profile, ec);
}

bool Actuator::move_absolute(int32_t target, const Profile &profile, std::error_code &ec) {
  return do_move(target, profile, ec);
}

bool Actuator::move_relative(int32_t delta, std::error_code &ec) {
  return move_relative(delta, config_.profile, ec);
}

bool Actuator::move_relative(int32_t delta, const Profile &profile, std::error_code &ec) {
  int32_t current = 0;
  if (!get_position(current, ec)) {
    return false;
  }
  const int64_t target = static_cast<int64_t>(current) + delta;
  return do_move(static_cast<int32_t>(std::clamp<int64_t>(target, INT32_MIN, INT32_MAX)), profile,
                 ec);
}

bool Actuator::increment(std::error_code &ec) { return move_relative(config_.jog_step, ec); }

bool Actuator::decrement(std::error_code &ec) { return move_relative(-config_.jog_step, ec); }

bool Actuator::stop(std::error_code &ec) {
  ec.clear();
  if (!check_online(ec)) {
    return false;
  }
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mcp_mutex_);
    auto *drive = mcp_.drive(config_.axis);
    ok = drive != nullptr && drive->quick_stop(ec);
  }
  note_result(ok, ec, "stop");
  return ok;
}

} // namespace mib
