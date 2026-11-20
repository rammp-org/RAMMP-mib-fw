#pragma once

#include <array>
#include <cstddef>

namespace mib {

/// @brief A trapezoidal move in encoder counts, planned on the MIB and sampled once per tick.
///
/// The MCP266 applies no ramp to a position target (a step in its speed command is violent
/// even with the controller's speed cap set), so the Actuator streams one: this class says
/// where the profile is at time t and how fast it is going there. It accelerates at
/// `acceleration` to at most `max_speed`, cruises, and decelerates at the same rate to rest
/// on the target (a triangle when the distance is too short to cruise).
///
/// A move may start in motion, which is how a new target mid-move continues smoothly: plan
/// again from the old profile's position and speed. A start speed heading away from the new
/// target, or too fast to stop before it, is braked first at `acceleration`; the profile
/// then passes the target and comes back to it.
///
/// Pure arithmetic and no I/O, so it is host-tested (test/host).
class MotionProfile {
public:
  struct Limits {
    double max_speed;    ///< Counts per second, > 0.
    double acceleration; ///< Counts per second squared, > 0; also the deceleration.
  };

  struct Sample {
    double position; ///< Counts.
    double speed;    ///< Counts per second, signed: positive while the count increases.
    bool done;       ///< At rest on the target.
  };

  /// @brief Plan a move from `start`, moving at `start_speed`, to rest at `target`.
  /// @return False (and the profile left at rest on `start`) if a limit is not positive.
  bool plan(double start, double start_speed, double target, const Limits &limits);

  /// @brief The profile at `t` seconds after plan(). Before 0 it is the start; from
  ///        duration() on it is the target, at rest.
  Sample at(double t) const;

  /// @brief What to write to the controller as the position target at time `t`: the
  ///        profile position plus profile speed x `lead_s`, never past the final target.
  /// @details The controller's position stage commands a speed of P x (target - count), so a
  ///          target moving at v is followed v / P behind; leading the target by v x (1 / P)
  ///          cancels that lag. The lead is cut back so that a setpoint ahead of the profile
  ///          never lies beyond the target. (While the profile itself is past the target,
  ///          braking a retarget it could not stop for, the setpoint follows it.)
  double setpoint(double t, double lead_s) const;

  /// @brief setpoint() through a zero-vibration input shaper: the mean of the setpoint at `t`
  ///        and at `t - shaper_s`.
  /// @details Two half-size copies of the move, the second delayed by half the period of a
  ///          resonance, excite it in antiphase and cancel it. Never past the target (both
  ///          halves are not), continuous, and at rest on the target from
  ///          duration() + shaper_s on. `shaper_s` <= 0 is setpoint().
  double shaped_setpoint(double t, double lead_s, double shaper_s) const;

  /// @brief Seconds from plan() until the profile rests on the target.
  double duration() const { return duration_; }
  double target() const { return target_; }

private:
  /// One stretch of constant acceleration.
  struct Segment {
    double t0;       ///< Start time, seconds.
    double p0;       ///< Start position, counts.
    double v0;       ///< Start speed, counts/s, signed.
    double a;        ///< Acceleration, counts/s^2, signed.
    double duration; ///< Seconds.
  };
  void add(double &t, double &p, double &v, double a, double duration);

  // Braking, then accelerate or slow to the peak, cruise, decelerate: at most four.
  std::array<Segment, 4> segments_{};
  size_t count_{0};
  double start_{0};
  double target_{0};
  double duration_{0};
};

} // namespace mib
