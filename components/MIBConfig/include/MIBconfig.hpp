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

/// @brief Trajectory limits for one MIB drive response profile.
struct DriveProfileConfig {
  float max_linear_velocity_mps;
  float max_angular_velocity_radps;
  float max_linear_acceleration_mps2;
  float max_angular_acceleration_radps2;
};

inline constexpr DriveProfileConfig drive_profile_low{
    .max_linear_velocity_mps = 0.35f,
    .max_angular_velocity_radps = 1.0f,
    .max_linear_acceleration_mps2 = 0.5f,
    .max_angular_acceleration_radps2 = 1.0f,
};

inline constexpr DriveProfileConfig drive_profile_normal{
    .max_linear_velocity_mps = 0.75f,
    .max_angular_velocity_radps = 2.0f,
    .max_linear_acceleration_mps2 = 1.0f,
    .max_angular_acceleration_radps2 = 2.0f,
};

inline constexpr DriveProfileConfig drive_profile_high{
    .max_linear_velocity_mps = 1.0f,
    .max_angular_velocity_radps = 3.14159f,
    .max_linear_acceleration_mps2 = 2.0f,
    .max_angular_acceleration_radps2 = 4.0f,
};

} // namespace mib::config
