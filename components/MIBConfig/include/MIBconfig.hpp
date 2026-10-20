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

/// @brief Drive response update period.
inline constexpr auto drive_response_interval = std::chrono::milliseconds{20};

/// @brief Speed and rate limits for one MIB drive response profile.
///
/// Speeds are chassis speeds at full stick. Accel limits apply while speeding up;
/// decel limits apply while slowing, stopping, or reversing. Turn rate scales by
/// (1 - turn_reduction * |linear stick|).
struct DriveProfileConfig {
  float max_forward_mps;
  float max_reverse_mps;
  float max_angular_radps;
  float turn_reduction;
  float linear_accel_mps2;
  float linear_decel_mps2;
  float angular_accel_radps2;
  float angular_decel_radps2;
};

/// NORMAL matches the R-Net chair, fitted from the wheel_velocities rosbag
/// (Luci joystick to wheel speed, 71 s) with tools/drive_fit. Wheel speeds there
/// were 14.5 rad/s full forward, 10.4 rad/s full reverse, and 5.0 rad/s per wheel
/// spinning in place.
/// The m/s and rad/s values assume the R-Net chair has the wheel diameter and
/// separation in differential_drive above.
inline constexpr DriveProfileConfig drive_profile_normal{
    .max_forward_mps = 1.835f,
    .max_reverse_mps = 1.322f,
    .max_angular_radps = 2.297f,
    .turn_reduction = 0.140f,
    .linear_accel_mps2 = 0.923f,
    .linear_decel_mps2 = 1.712f,
    .angular_accel_radps2 = 2.250f,
    .angular_decel_radps2 = 4.100f,
};

/// LOW is NORMAL at half speed with the same rate limits.
inline constexpr DriveProfileConfig drive_profile_low{
    .max_forward_mps = drive_profile_normal.max_forward_mps * 0.5f,
    .max_reverse_mps = drive_profile_normal.max_reverse_mps * 0.5f,
    .max_angular_radps = drive_profile_normal.max_angular_radps * 0.5f,
    .turn_reduction = drive_profile_normal.turn_reduction,
    .linear_accel_mps2 = drive_profile_normal.linear_accel_mps2,
    .linear_decel_mps2 = drive_profile_normal.linear_decel_mps2,
    .angular_accel_radps2 = drive_profile_normal.angular_accel_radps2,
    .angular_decel_radps2 = drive_profile_normal.angular_decel_radps2,
};

/// TODO: HIGH equals NORMAL until we record the R-Net at its top speed setting.
inline constexpr DriveProfileConfig drive_profile_high = drive_profile_normal;

} // namespace mib::config
