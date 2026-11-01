#pragma once

#include <chrono>
#include <cstdint>

#include "driver/gpio.h"

namespace mib::config {

/// @brief State-machine task period.
inline constexpr auto state_task_interval = std::chrono::milliseconds{20};

/// @brief System, seat status, and drive diagnostic publication task period.
inline constexpr auto publication_task_interval = std::chrono::milliseconds{500};

/// @brief Motor command publication task period.
inline constexpr auto motor_command_task_interval = std::chrono::milliseconds{50};

/// @brief Telemetry sample period of the service link (the control panel's plots).
inline constexpr auto service_telemetry_interval = std::chrono::milliseconds{20};

/// @brief Serve the control panel over WebUSB on the P4's USB 2.0 HS port.
/// The ESP32-P4-ETH routes that port to J3, the 4-pin header, not to the USB-C.
inline constexpr bool service_usb_enabled = true;

/// @brief Serve the control panel over the console UART (the USB-C, through the CH343).
inline constexpr bool service_uart_enabled = true;

/// @brief Serve the control panel over Ethernet on this TCP port (0 = off): the panel
/// reaches it through tools/mib_panel/eth_bridge.py.
inline constexpr std::uint16_t service_tcp_port = 5555;

/// @brief Telemetry is decimated to every Nth sample while the UART runs at its 115200-baud
/// default. The panel raises the rate to 921600 while connected, and then gets every one.
inline constexpr int service_uart_telemetry_divider = 5;

/// @brief The control panel, without a scheme (https): Chrome offers it when the WebUSB port
/// is plugged in, and the console prints it at boot and every 30 s until a panel connects.
inline constexpr const char *service_landing_page_url = "rammp-org.github.io/RAMMP-mib-fw/";

/// @brief A panel motor override or panel drive input lapses this long after its last frame.
inline constexpr auto service_override_timeout = std::chrono::milliseconds{300};

/// @brief Ceilings on panel motor overrides; a limit of 0 from the panel means this value.
inline constexpr float service_max_velocity_radps = 8.0f;
inline constexpr float service_max_torque_nm = 20.0f;

/// @brief Time an unacknowledged MotorTuning is resent before the MIB gives up.
inline constexpr auto tuning_resend_timeout = std::chrono::milliseconds{2000};

/// @brief How long ESTOP holds before the MIB asks the e-stop latch to clear.
inline constexpr auto estop_hold_duration = std::chrono::seconds{3};

/// @brief How often that clear is re-requested while the latch refuses it (a switch still
/// held, or the joystick not heard yet). Matches the latch's clear window.
inline constexpr auto estop_clear_retry = std::chrono::seconds{1};

/// @brief Motor debug list (rammp::Motors) publication period.
inline constexpr auto motor_states_publish_interval = std::chrono::milliseconds{200};

/// @brief A board whose last MotorState is older than this is reported offline.
inline constexpr auto motor_state_timeout = std::chrono::milliseconds{500};

/// @brief A jog stops this long after the last MotorJog (rammp::MotorJog's contract).
inline constexpr auto motor_jog_timeout = std::chrono::milliseconds{300};

/// @brief Output-shaft speed of a motor debug jog, in rad/s.
inline constexpr float motor_jog_velocity_radps = 1.0f;

/// @brief IMU read task period.
inline constexpr auto imu_task_interval = std::chrono::milliseconds{10};

/// @brief IMU read retry period after a failed read, so a missing IMU does not flood the log.
inline constexpr auto imu_retry_interval = std::chrono::milliseconds{1000};

/// @brief I2C address of the BNO055 IMU on the internal I2C bus (SDA=GPIO7, SCL=GPIO8).
inline constexpr std::uint8_t imu_i2c_address = 0x28;

/// @brief Enable DHCP server mode for the MIB Ethernet interface.
inline constexpr bool ethernet_dhcp_server = true;

/// @brief IPv4 address assigned to the MIB Ethernet interface.
inline constexpr std::uint8_t ethernet_ip[4] = {192, 168, 4, 1};

/// @brief IPv4 netmask used by the MIB Ethernet interface.
inline constexpr std::uint8_t ethernet_netmask[4] = {255, 255, 255, 0};

/// @brief IPv4 gateway address advertised by the MIB DHCP server.
inline constexpr std::uint8_t ethernet_gateway[4] = {192, 168, 4, 1};

/// @brief NTP server the MIB syncs its clock to: the ROS host, resolved over mDNS.
inline constexpr const char *ntp_server = "snorlax.local";

/// @brief SNTP client restart period until the first sync. A ROS host that comes up after the MIB
/// is found within one period, not after lwIP's retry backoff (15 s doubling to 150 s).
inline constexpr auto time_sync_retry_interval = std::chrono::seconds{15};

/// @brief E-stop clear button: normally open to GND, held 2 s. Any GPIO the board leaves free.
/// GPIO_NUM_NC: no button wired, so only MibSystem calling latch_.start_clear_attempt() clears it.
inline constexpr gpio_num_t estop_clear_gpio = GPIO_NUM_NC;

/// @brief Bench testing without the HMI: false.
/// true: the latch watches the HMI as a deadman (estop-core's default): a silent HMI latches
/// the e-stop, and a clear needs the HMI heard. false: nothing is watched, and the MIB starts
/// a clear attempt once at boot, so it reaches IDLE on its own. Any node that sends ENGAGED
/// (an HMI's e-stop button, the control panel's E-STOP) still latches it either way; clear
/// it from the HMI or the panel's "Hold to clear". Set back to true for the chair.
inline constexpr bool estop_watch_hmi = false;

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

/// NORMAL matches the R-Net chair, fitted in #32 (tools/drive_fit) from the
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
/// stopping_linear_acceleration was tuned on the chair after the fit: 20 times
/// the fitted 1.662 m/s^2, so the chair stops sooner when the stick is released.
///
/// stopping_angular_acceleration was set, not fitted: the recording never stops
/// from a pure spin, so the fit can't see it. 4.0 rad/s^2 is the R-Net's turn
/// deceleration measured while slowing a turn.
///
/// Driving jerk is an improvement over the R-Net, which is trapezoidal: each
/// axis ramps up to full driving acceleration over 0.1 s (jerk = accel / 0.1 s).
/// Replaying the recording, that halves the output's RMS jerk for 0.05 rad/s
/// more linear wheel-speed error; longer ramps start to lag the stick (#32's
/// jerk sweep). Needs espp
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
    .stopping_linear_acceleration_mps2 = 1.662f * 20.0f,
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
