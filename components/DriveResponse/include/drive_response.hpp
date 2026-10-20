#pragma once

namespace mib {

/// @brief Joystick-to-chassis velocity response modeled on the R-Net controller.
///
/// Shapes a normalized joystick command into a chassis velocity target, then
/// rate-limits the linear and angular axes independently, with separate
/// acceleration (speeding up) and deceleration (slowing down or reversing)
/// limits per axis.
///
/// The structure and default limits were fitted to a rosbag of the R-Net chair
/// driven through the Luci joystick (wheel_velocities, 71 s); see tools/drive_fit.
/// The fit tracks the recorded wheel speeds to about 0.6 rad/s RMS against a
/// 14.5 rad/s peak. The R-Net also showed about 200 ms of dead time between Luci
/// and the wheels; that is transport latency, not shaping, so it is not
/// reproduced here.
///
/// This class holds no locks and starts no tasks. The owner calls update() at a
/// fixed rate and serializes access across threads.
class DriveResponse {
public:
  /// @brief Speed and rate limits for one drive profile.
  struct Config {
    float max_forward_mps{0.0f};    ///< Linear speed at full forward stick, in m/s.
    float max_reverse_mps{0.0f};    ///< Linear speed at full reverse stick, in m/s.
    float max_angular_radps{0.0f};  ///< Spin speed at full side stick, in rad/s.
    /// Fraction of turn rate removed at full forward or reverse stick. Turn rate
    /// scales by (1 - turn_reduction * |linear|).
    float turn_reduction{0.0f};
    float linear_accel_mps2{0.0f};      ///< Linear rate limit when speeding up, in m/s^2.
    float linear_decel_mps2{0.0f};      ///< Linear rate limit when slowing or reversing, in m/s^2.
    float angular_accel_radps2{0.0f};   ///< Angular rate limit when speeding up, in rad/s^2.
    float angular_decel_radps2{0.0f};   ///< Angular rate limit when slowing or reversing, in rad/s^2.
  };

  /// @brief Chassis velocity produced by the response.
  struct Output {
    float linear_mps{0.0f};    ///< Forward speed, in m/s.
    float angular_radps{0.0f}; ///< Rotation speed, in rad/s. Positive is counter-clockwise.
  };

  explicit DriveResponse(const Config &config);

  /// @brief Check whether the configuration is usable.
  /// @return True if every speed and rate limit is positive and turn_reduction is in [0, 1).
  static bool is_valid(const Config &config);

  /// @brief Replace the limits without resetting the current velocity.
  /// @param config New limits. Ignored if invalid.
  /// @return True if the configuration was applied.
  /// @note The output ramps to the new target under the new limits, so changing
  ///       profile while driving never steps the wheel speeds.
  bool set_config(const Config &config);

  /// @brief Set the normalized joystick target.
  /// @param linear Normalized linear command in [-1, 1]; +1 is full forward.
  /// @param angular Normalized angular command in [-1, 1]; +1 is a full left turn.
  /// @note Values outside [-1, 1] are clamped; NaN is treated as 0.
  void set_target(float linear, float angular);

  /// @brief Advance the rate limiters by one step.
  /// @param dt_s Time since the last update, in seconds. Non-positive values are ignored.
  void update(float dt_s);

  /// @brief Get the current rate-limited chassis velocity.
  Output output() const { return output_; }

  /// @brief Zero the target and the output immediately.
  void reset();

private:
  Config config_;
  float target_linear_mps_{0.0f};
  float target_angular_radps_{0.0f};
  Output output_{};
};

} // namespace mib
