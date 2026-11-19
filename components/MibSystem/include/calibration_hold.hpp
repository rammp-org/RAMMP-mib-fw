#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace mib {

/// @brief The bookkeeping of a calibration hold: which axes the MIB has stopped commanding,
/// and until when.
///
/// While an axis is held the MIB publishes no MotorCommand to it, so the board's command
/// stream goes stale and its own console may drive it (a PACE RACER refuses console motion
/// while a stream is live). A hold is asked for over the service link (MIB control module,
/// CAL_HOLD), renewed by asking again, and ends on CAL_RELEASE, at its deadline, at the
/// total cap, or when MibSystem ends it (e-stop, firmware update).
///
/// Pure logic, no locking and no clock of its own: MibSystem owns the mutex and passes
/// `now`, so the host test (test/host/) drives it with a fake clock. What it does not know
/// (the system state, the e-stop latch, panel overrides, jogs) arrives as Conditions.
class CalibrationHold {
public:
  using Clock = std::chrono::steady_clock;
  using Ms = std::chrono::milliseconds;

  struct Config {
    Ms default_duration{60'000}; ///< A request with duration 0 gets this.
    Ms max_total{300'000};       ///< A hold ends this long after it began, renewed or not.
    Ms min_duration{1'000};      ///< Shorter requests are raised to this.
  };

  /// What the caller knows about the rest of the MIB at the time of a request.
  struct Conditions {
    bool idle{false};            ///< The system state is IDLE (needed to begin a hold).
    bool estop{false};           ///< The e-stop latch is active.
    bool ota{false};             ///< A firmware update is being written.
    bool override_active{false}; ///< The control panel overrides some axis.
    bool jogging{false};         ///< A joystick jog is moving some axis.
  };

  enum class Refusal : uint8_t {
    NONE,     ///< Granted.
    NO_AXES,  ///< The request named no axis.
    ESTOP,    ///< The e-stop latch is active.
    OTA,      ///< A firmware update is running.
    NOT_IDLE, ///< No hold is active and the MIB is not IDLE (driving, autonomy, ...).
    OVERRIDE, ///< A panel override is live: release it first.
    JOGGING,  ///< A jog is moving an axis: let go of it first.
  };

  enum class End : uint8_t {
    NONE,      ///< Still held, or never held.
    RELEASED,  ///< CAL_RELEASE for every held axis.
    TIMED_OUT, ///< Not renewed before its deadline.
    CAPPED,    ///< Reached max_total.
    ESTOP,     ///< The e-stop latched.
    OTA,       ///< A firmware update began.
    STATE,     ///< The system state left CALIBRATE some other way (see MibSystem).
  };

  explicit CalibrationHold(const Config &config)
      : config_(config) {}

  /// @return Why a request would be refused now, or NONE. Changes nothing.
  /// @note A renewal (a hold is active) does not need IDLE: the state is CALIBRATE by then.
  Refusal check(uint8_t mask, const Conditions &c, Clock::time_point now) const {
    if (mask == 0)
      return Refusal::NO_AXES;
    if (c.estop)
      return Refusal::ESTOP;
    if (c.ota)
      return Refusal::OTA;
    if (active(now))
      return Refusal::NONE;
    if (!c.idle)
      return Refusal::NOT_IDLE;
    if (c.override_active)
      return Refusal::OVERRIDE;
    if (c.jogging)
      return Refusal::JOGGING;
    return Refusal::NONE;
  }

  /// Begin or renew the hold for `mask` (bit n = axis n). The axes join any already held,
  /// and the deadline moves to now + duration (0 = the default), never past begin + max_total.
  /// @return True if this began a new hold (the caller moves the state to CALIBRATE).
  /// @note Call only after check() returned NONE, under the same lock.
  bool grant(uint8_t mask, Ms duration, Clock::time_point now) {
    const bool begins = !active(now);
    if (begins) {
      began_ = now;
      mask_ = 0;
      ended_ = End::NONE;
    }
    if (duration.count() <= 0)
      duration = config_.default_duration;
    duration = std::clamp(duration, config_.min_duration, config_.max_total);
    mask_ |= mask;
    until_ = std::min(now + duration, began_ + config_.max_total);
    return begins;
  }

  /// Stop holding the axes in `mask`; the hold ends when none is left.
  /// @return The axes still held.
  uint8_t release(uint8_t mask, Clock::time_point now) {
    if (!active(now))
      return 0;
    mask_ &= static_cast<uint8_t>(~mask);
    if (mask_ == 0)
      finish(End::RELEASED);
    return mask_;
  }

  /// End the hold at once for `why` (e-stop, firmware update, state change). No-op if none.
  void end(End why, Clock::time_point now) {
    if (active(now))
      finish(why);
  }

  /// @return True while some axis is held and the deadline has not passed.
  bool active(Clock::time_point now) const { return mask_ != 0 && now < until_; }

  /// @return The held axes (bit n = axis n), 0 when no hold is active.
  uint8_t mask(Clock::time_point now) const { return active(now) ? mask_ : 0; }

  /// @return Time left before the hold lapses, 0 when none is active.
  Ms remaining(Clock::time_point now) const {
    return active(now) ? std::chrono::duration_cast<Ms>(until_ - now) : Ms{0};
  }

  /// How the last hold ended, once it has: a lapsed deadline reads as TIMED_OUT or CAPPED.
  End ended(Clock::time_point now) const {
    if (ended_ != End::NONE || mask_ == 0 || now < until_)
      return ended_;
    return until_ == began_ + config_.max_total ? End::CAPPED : End::TIMED_OUT;
  }

  /// Forget a hold that has ended, so ended() reads NONE until the next one.
  void clear(Clock::time_point now) {
    if (!active(now)) {
      mask_ = 0;
      ended_ = End::NONE;
    }
  }

  static const char *refusal_text(Refusal r) {
    switch (r) {
    case Refusal::NONE:
      return "granted";
    case Refusal::NO_AXES:
      return "no such axis";
    case Refusal::ESTOP:
      return "e-stop is latched";
    case Refusal::OTA:
      return "a firmware update is running";
    case Refusal::NOT_IDLE:
      return "MIB is not IDLE: disable the drive first";
    case Refusal::OVERRIDE:
      return "a panel motor override is live: release it first";
    case Refusal::JOGGING:
      return "a motor jog is live: let go of it first";
    }
    return "refused";
  }

  static const char *end_text(End e) {
    switch (e) {
    case End::NONE:
      return "none";
    case End::RELEASED:
      return "released";
    case End::TIMED_OUT:
      return "timed out";
    case End::CAPPED:
      return "reached its time cap";
    case End::ESTOP:
      return "e-stop";
    case End::OTA:
      return "firmware update";
    case End::STATE:
      return "state changed";
    }
    return "?";
  }

private:
  void finish(End why) {
    ended_ = why;
    until_ = {};
  }

  Config config_;
  uint8_t mask_{0};
  Clock::time_point began_{};
  Clock::time_point until_{};
  End ended_{End::NONE};
};

} // namespace mib
