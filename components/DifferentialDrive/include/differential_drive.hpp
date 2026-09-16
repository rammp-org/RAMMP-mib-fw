#pragma once

namespace mib {

/// @brief Differential drive platform kinematics.
///
/// Converts a body-frame velocity command (linear + rotation) into the
/// individual left / right wheel speeds required to achieve it.
class DifferentialDrive {
public:
  /// @brief Physical geometry of the drive platform.
  struct Config {
    float wheel_diameter_m{0.0f};   ///< Driven wheel diameter, in meters.
    float wheel_separation_m{0.0f}; ///< Distance between the two driven wheels, in meters.
  };

  /// @brief Wheel speeds produced for a velocity command.
  struct WheelSpeeds {
    float left_rpm{0.0f};  ///< Left wheel speed, in revolutions per minute.
    float right_rpm{0.0f}; ///< Right wheel speed, in revolutions per minute.
  };

  /// @brief Construct the platform for a given geometry.
  /// @param config Wheel diameter and wheel separation of the platform.
  explicit DifferentialDrive(const Config &config);

  /// @brief Convert a velocity command into wheel speeds.
  /// @param linear_mps Forward speed of the platform, in meters per second.
  /// @param angular_radps Rotation speed of the platform, in radians per second.
  ///        Positive rotates the platform counter-clockwise (left turn).
  /// @return The left and right wheel speeds, in revolutions per minute.
  WheelSpeeds compute(float linear_mps, float angular_radps) const;

  /// @brief Get the configured wheel diameter.
  /// @return Wheel diameter, in meters.
  float wheel_diameter() const { return wheel_diameter_m_; }

  /// @brief Get the configured wheel separation.
  /// @return Distance between the driven wheels, in meters.
  float wheel_separation() const { return wheel_separation_m_; }

private:
  float wheel_diameter_m_;
  float wheel_separation_m_;
};

} // namespace mib
