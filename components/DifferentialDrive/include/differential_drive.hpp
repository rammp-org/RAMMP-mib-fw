#pragma once

#include "base_component.hpp"

namespace mib {

/// @brief Differential drive platform kinematics.
///
/// Converts a body-frame velocity command (linear + rotation) into the
/// individual left / right wheel speeds required to achieve it.
class DifferentialDrive : public espp::BaseComponent {
public:
  /// @brief Physical geometry of the drive platform.
  struct Config {
    float wheel_diameter_m{0.0f};   ///< Driven wheel diameter, in meters.
    float wheel_separation_m{0.0f}; ///< Distance between the two driven wheels, in meters.
    /// Negate the left wheel output, for a motor mounted facing the opposite way.
    bool invert_left{true};
    /// Negate the right wheel output, for a motor mounted facing the opposite way.
    bool invert_right{false};
  };

  /// @brief Wheel speeds produced for a velocity command.
  struct WheelSpeeds {
    float left_rpm{0.0f};  ///< Left wheel speed, in revolutions per minute.
    float right_rpm{0.0f}; ///< Right wheel speed, in revolutions per minute.
  };

  /// @brief Construct the platform for a given geometry.
  /// @param config Wheel geometry and per-motor output inversion.
  /// @note Wheel diameter and separation must both be greater than zero;
  ///       otherwise the platform is invalid and compute() returns zero speeds.
  explicit DifferentialDrive(const Config &config);

  /// @brief Check whether the configured geometry is usable.
  /// @return True if wheel diameter and separation are both greater than zero.
  bool is_valid() const { return valid_; }

  /// @brief Convert a velocity command into wheel speeds.
  /// @param linear_mps Forward speed of the platform, in meters per second.
  /// @param angular_radps Rotation speed of the platform, in radians per second.
  ///        Positive rotates the platform counter-clockwise (left turn).
  /// @return The left and right wheel speeds, in revolutions per minute, with
  ///         each wheel negated if its motor is configured as inverted.
  ///         Zero speeds if the configured geometry is invalid.
  WheelSpeeds compute(float linear_mps, float angular_radps) const;

  /// @brief Get the configured wheel diameter.
  /// @return Wheel diameter, in meters.
  float wheel_diameter() const { return wheel_diameter_m_; }

  /// @brief Get the configured wheel separation.
  /// @return Distance between the driven wheels, in meters.
  float wheel_separation() const { return wheel_separation_m_; }

  /// @brief Check whether the left motor output is inverted.
  /// @return True if the left wheel speed is negated.
  bool invert_left() const { return invert_left_; }

  /// @brief Check whether the right motor output is inverted.
  /// @return True if the right wheel speed is negated.
  bool invert_right() const { return invert_right_; }

private:
  float wheel_diameter_m_;
  float wheel_separation_m_;
  bool invert_left_;
  bool invert_right_;
  bool valid_{false};
};

} // namespace mib
