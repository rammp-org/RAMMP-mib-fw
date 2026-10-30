#pragma once

namespace mib {

/// @brief Wheel odometry for a differential-drive platform.
///
/// Integrates the two drive wheels' shaft angles into a planar pose, following
/// REP-103: x forward, y left, yaw counter-clockwise from x, SI units. The pose
/// starts at the origin of the odom frame (REP-105) and drifts with wheel slip.
///
/// \note Plain math with no hardware or logging dependency, so it can be
///       exercised off-target.
class DiffDriveOdometry {
public:
  /// @brief Physical geometry of the drive platform.
  struct Config {
    float wheel_diameter_m{0.0f};   ///< Driven wheel diameter, in meters.
    float wheel_separation_m{0.0f}; ///< Distance between the two driven wheels, in meters.
    bool invert_left{false};  ///< The left motor is mounted mirrored: shaft-positive is backward.
    bool invert_right{false}; ///< The right motor is mounted mirrored: shaft-positive is backward.
  };

  /// @brief Planar pose of the platform in the odom frame.
  struct Pose {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0}; ///< Wrapped to [-pi, pi].
  };

  /// @brief Body-frame velocity of the platform.
  struct Velocity {
    double linear_mps{0.0};    ///< Forward speed, in meters per second.
    double angular_radps{0.0}; ///< Rotation speed, counter-clockwise positive.
  };

  /// @brief Construct odometry for a given geometry, at the origin.
  /// @param config Wheel geometry and per-motor mounting direction.
  explicit DiffDriveOdometry(const Config &config);

  /// @brief Advance the pose to new wheel shaft angles.
  /// @param left_rad Left motor shaft angle, multiturn, counter-clockwise positive.
  /// @param right_rad Right motor shaft angle, multiturn, counter-clockwise positive.
  /// @note The first call after construction or resync() only records the
  ///       angles; the pose moves from the second call on.
  void update(float left_rad, float right_rad);

  /// @brief Forget the recorded wheel angles, keeping the pose.
  /// @note Call when a wheel's angle restarts, such as its controller
  ///       rebooting, so the jump is not integrated as motion.
  void resync() { synced_ = false; }

  /// @brief Convert wheel shaft speeds into a body-frame velocity.
  /// @param left_radps Left motor shaft speed, counter-clockwise positive.
  /// @param right_radps Right motor shaft speed, counter-clockwise positive.
  /// @return The platform's linear and angular velocity.
  Velocity velocity(float left_radps, float right_radps) const;

  /// @brief Get the integrated pose.
  /// @return The pose in the odom frame.
  const Pose &pose() const { return pose_; }

private:
  double wheel_radius_m_;
  double wheel_separation_m_;
  double left_sign_;
  double right_sign_;
  Pose pose_{};
  bool synced_{false};
  float last_left_rad_{0.0f};
  float last_right_rad_{0.0f};
};

} // namespace mib
