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

/// @brief Trajectory limits and joystick shaping for one MIB drive response profile.
///
/// The velocity, acceleration, and centripetal fields map onto
/// espp::TrajectoryPlanner::Config. Driving acceleration applies whenever the
/// target is non-zero; stopping acceleration applies only when the target is
/// exactly (0, 0), so it relies on the joystick's center deadzone snapping a
/// resting stick to zero. Jerk limits apply to driving only (S-curve starts);
/// stopping stays trapezoidal so stops land without overshoot. A jerk of 0
/// makes that axis trapezoidal.
///
/// DriveProfileStore saves this struct to flash as is. Changing its fields
/// (adding, removing, reordering, or changing what one means) needs a new
/// VERSION, with the old layout kept as a frozen struct that migrate() converts
/// from; see components/DriveProfileStore/README.md.
struct DriveProfileConfig {
  /// Schema version of the saved drive profile files.
  static constexpr uint32_t VERSION = 1;

  float max_linear_velocity_mps;
  float max_reverse_linear_velocity_mps;
  float max_angular_velocity_radps;
  float max_linear_acceleration_mps2;
  float max_angular_acceleration_radps2;
  float max_linear_jerk_mps3;
  float max_angular_jerk_radps3;
  float stopping_linear_acceleration_mps2;
  float stopping_angular_acceleration_radps2;
  float max_centripetal_acceleration_mps2;
};

/// NORMAL matches the R-Net chair, fitted with tools/drive_fit from the
/// wheel_velocities rosbag (Luci joystick to wheel speed, 71 s). The R-Net hit
/// 14.5 rad/s at the wheel full forward, 10.4 rad/s full reverse, and 5.0 rad/s
/// per wheel spinning in place. The m/s and rad/s values assume the R-Net chair
/// has the wheel diameter and separation in differential_drive above.
///
/// The centripetal limit was set, not fitted: 1.5 m/s^2 is below the 2.6 m/s^2
/// peak the R-Net reached but fits the recording as well as no limit does. It
/// also stands in for the R-Net's reduced turn rate at speed (within 0.03 rad/s
/// RMS), which is why max_angular_velocity sits slightly under the R-Net's
/// in-place spin rate (2.11 vs about 2.3 rad/s).
///
/// stopping_angular_acceleration was set, not fitted: the recording never stops
/// from a pure spin, so the fit can't see it. 4.0 rad/s^2 is the R-Net's turn
/// deceleration measured while slowing a turn.
///
/// Driving jerk is an improvement over the R-Net, which is trapezoidal: each
/// axis ramps up to full driving acceleration over 0.1 s (jerk = accel / 0.1 s).
/// Replaying the recording, that halves the output's RMS jerk for 0.05 rad/s
/// more linear wheel-speed error; longer ramps start to lag the stick (see
/// tools/drive_fit/check_replay.py --jerk-sweep). Needs espp
/// trajectory_planner >= 1.3.9, which resumes from the applied acceleration
/// after a trapezoidal stop (esp-cpp/espp#850).
inline constexpr DriveProfileConfig drive_profile_normal{
    .max_linear_velocity_mps = 1.845f,
    .max_reverse_linear_velocity_mps = 1.303f,
    .max_angular_velocity_radps = 2.114f,
    .max_linear_acceleration_mps2 = 0.989f,
    .max_angular_acceleration_radps2 = 2.926f,
    .max_linear_jerk_mps3 = 9.89f,
    .max_angular_jerk_radps3 = 29.26f,
    .stopping_linear_acceleration_mps2 = 1.662f,
    .stopping_angular_acceleration_radps2 = 4.000f,
    .max_centripetal_acceleration_mps2 = 1.500f,
};

/// LOW is NORMAL at half speed with the same acceleration and jerk limits.
inline constexpr DriveProfileConfig drive_profile_low{
    .max_linear_velocity_mps = drive_profile_normal.max_linear_velocity_mps * 0.5f,
    .max_reverse_linear_velocity_mps = drive_profile_normal.max_reverse_linear_velocity_mps * 0.5f,
    .max_angular_velocity_radps = drive_profile_normal.max_angular_velocity_radps * 0.5f,
    .max_linear_acceleration_mps2 = drive_profile_normal.max_linear_acceleration_mps2,
    .max_angular_acceleration_radps2 = drive_profile_normal.max_angular_acceleration_radps2,
    .max_linear_jerk_mps3 = drive_profile_normal.max_linear_jerk_mps3,
    .max_angular_jerk_radps3 = drive_profile_normal.max_angular_jerk_radps3,
    .stopping_linear_acceleration_mps2 = drive_profile_normal.stopping_linear_acceleration_mps2,
    .stopping_angular_acceleration_radps2 =
        drive_profile_normal.stopping_angular_acceleration_radps2,
    .max_centripetal_acceleration_mps2 = drive_profile_normal.max_centripetal_acceleration_mps2,
};

/// TODO: HIGH equals NORMAL until we record the R-Net at its top speed setting.
inline constexpr DriveProfileConfig drive_profile_high = drive_profile_normal;

} // namespace mib::config
