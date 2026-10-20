#pragma once

#include <chrono>
#include <cstdint>

namespace mib::config {

/// @brief State-machine task period.
inline constexpr auto state_task_interval = std::chrono::milliseconds{20};

/// @brief System, seat status, and drive diagnostic publication task period.
inline constexpr auto publication_task_interval = std::chrono::milliseconds{500};

/// @brief Motor command publication task period.
inline constexpr auto motor_command_task_interval = std::chrono::milliseconds{50};

/// @brief Enable DHCP server mode for the MIB Ethernet interface.
inline constexpr bool ethernet_dhcp_server = true;

/// @brief IPv4 address assigned to the MIB Ethernet interface.
inline constexpr std::uint8_t ethernet_ip[4] = {192, 168, 4, 1};

/// @brief IPv4 netmask used by the MIB Ethernet interface.
inline constexpr std::uint8_t ethernet_netmask[4] = {255, 255, 255, 0};

/// @brief IPv4 gateway address advertised by the MIB DHCP server.
inline constexpr std::uint8_t ethernet_gateway[4] = {192, 168, 4, 1};

/// @brief Differential-drive geometry and motor direction configuration.
struct DifferentialDriveConfig {
  float wheel_diameter_m;
  float wheel_separation_m;
  bool invert_left;
  bool invert_right;
};

inline constexpr DifferentialDriveConfig differential_drive{
    .wheel_diameter_m = 0.254f,
    .wheel_separation_m = 0.558f,
    .invert_left = false,
    .invert_right = true,
};

/// @brief Joystick magnitude treated as centered, normalized.
///
/// The planner only switches to its stopping profile when the target is exactly
/// zero, so a resting stick that reads slightly off zero must be snapped to it.
inline constexpr float joystick_deadband = 0.02f;

/// @brief Trajectory limits and joystick shaping for one MIB drive response profile.
///
/// The velocity, acceleration, and centripetal fields map onto
/// espp::TrajectoryPlanner::Config. Driving acceleration applies whenever the
/// target is non-zero; stopping acceleration applies when the stick is centered.
/// reverse_scale and turn_reduction shape the joystick before the planner:
/// reverse stick is scaled by reverse_scale, and turn stick by
/// (1 - turn_reduction * |linear stick|).
struct DriveProfileConfig {
  float max_linear_velocity_mps;
  float max_angular_velocity_radps;
  float max_linear_acceleration_mps2;
  float max_angular_acceleration_radps2;
  float stopping_linear_acceleration_mps2;
  float stopping_angular_acceleration_radps2;
  float max_centripetal_acceleration_mps2;
  float reverse_scale;
  float turn_reduction;
};

/// NORMAL matches the R-Net chair, fitted with tools/drive_fit from the
/// wheel_velocities rosbag (Luci joystick to wheel speed, 71 s). The R-Net hit
/// 14.5 rad/s at the wheel full forward, 10.4 rad/s full reverse, and 5.0 rad/s
/// per wheel spinning in place. The m/s and rad/s values assume the R-Net chair
/// has the wheel diameter and separation in differential_drive above.
///
/// The centripetal limit was set, not fitted: 1.5 m/s^2 is below the 2.6 m/s^2
/// peak the R-Net reached but fits the recording as well as no limit does.
///
/// stopping_angular_acceleration is the least certain value: the recording has
/// few stops from a turn, and fits land anywhere from 2.7 to 4.6 rad/s^2.
///
/// The planner's jerk limits stay off (trapezoidal), as on the R-Net. Switching
/// between a jerk-limited driving profile and a trapezoidal stopping profile
/// leaves a stale acceleration in the planner; resolve that before enabling them.
inline constexpr DriveProfileConfig drive_profile_normal{
    .max_linear_velocity_mps = 1.841f,
    .max_angular_velocity_radps = 2.318f,
    .max_linear_acceleration_mps2 = 1.001f,
    .max_angular_acceleration_radps2 = 2.968f,
    .stopping_linear_acceleration_mps2 = 1.636f,
    .stopping_angular_acceleration_radps2 = 4.621f,
    .max_centripetal_acceleration_mps2 = 1.500f,
    .reverse_scale = 0.707f,
    .turn_reduction = 0.167f,
};

/// LOW is NORMAL at half speed with the same acceleration limits.
inline constexpr DriveProfileConfig drive_profile_low{
    .max_linear_velocity_mps = drive_profile_normal.max_linear_velocity_mps * 0.5f,
    .max_angular_velocity_radps = drive_profile_normal.max_angular_velocity_radps * 0.5f,
    .max_linear_acceleration_mps2 = drive_profile_normal.max_linear_acceleration_mps2,
    .max_angular_acceleration_radps2 = drive_profile_normal.max_angular_acceleration_radps2,
    .stopping_linear_acceleration_mps2 = drive_profile_normal.stopping_linear_acceleration_mps2,
    .stopping_angular_acceleration_radps2 =
        drive_profile_normal.stopping_angular_acceleration_radps2,
    .max_centripetal_acceleration_mps2 = drive_profile_normal.max_centripetal_acceleration_mps2,
    .reverse_scale = drive_profile_normal.reverse_scale,
    .turn_reduction = drive_profile_normal.turn_reduction,
};

/// TODO: HIGH equals NORMAL until we record the R-Net at its top speed setting.
inline constexpr DriveProfileConfig drive_profile_high = drive_profile_normal;

} // namespace mib::config
