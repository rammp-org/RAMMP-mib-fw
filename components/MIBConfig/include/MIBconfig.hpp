#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"

namespace mib::config {

/// @brief State-machine task period.
inline constexpr auto state_task_interval = std::chrono::milliseconds{20};

/// @brief System, seat status, and drive diagnostic publication task period.
inline constexpr auto publication_task_interval = std::chrono::milliseconds{500};

/// @brief Motor command publication task period.
inline constexpr auto motor_command_task_interval = std::chrono::milliseconds{50};

/// @brief Wheel odometry update and /odom, /tf publication period. Matches the
///        20 Hz rate the drive motor controllers report their state at.
inline constexpr auto odometry_task_interval = std::chrono::milliseconds{50};

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

/// @brief External controller TwistSetpoints stop the chair this long after the last admitted one.
inline constexpr auto twist_setpoint_timeout = std::chrono::milliseconds{300};

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

/// @brief Where the chair is, as a POSIX TZ string: MibStatus carries UTC plus this zone's
/// offset (daylight saving included), and the joystick's clock shows the local time.
inline constexpr const char *timezone = "EST5EDT,M3.2.0,M11.1.0"; // US Eastern

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
    .wheel_diameter_m = 0.355f,
    .wheel_separation_m = 0.6096f,
    .invert_left = false,
    .invert_right = true,
};

/// @brief Diagonal of the published pose covariance: x, y, z, roll, pitch, yaw.
///        A planar base cannot observe z, roll, or pitch, so they carry a large variance.
inline constexpr std::array<double, 6> odometry_pose_covariance_diagonal{0.001, 0.001, 1e6,
                                                                         1e6,   1e6,   0.03};

/// @brief Diagonal of the published twist covariance: vx, vy, vz, wx, wy, wz.
inline constexpr std::array<double, 6> odometry_twist_covariance_diagonal{0.001, 0.001, 1e6,
                                                                          1e6,   1e6,   0.03};

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

/// @brief After an e-stop quick-stops the leg actuators, how long to let the stop ramp
/// settle before each leg's position is read and commanded as its new target.
inline constexpr std::chrono::milliseconds estop_leg_settle{250};

/// @brief CAN bus to the MCP266 motor controllers (TWAI peripheral).
inline constexpr int can_tx_gpio = 17;
inline constexpr int can_rx_gpio = 16;
inline constexpr std::uint32_t can_bitrate = 1000000;

/// @brief The six legs whose joint is positioned by a linear actuator.
///
/// Each leg carries one joint with an absolute encoder, read and closed-loop controlled by
/// one channel of a Basicmicro MCP266. Two legs share one controller, three controllers
/// sit on the CAN bus. The carriages have limit switches wired to their controller; the
/// other legs rely on the software range alone.
enum class Leg : std::uint8_t {
  FRONT_CASTER = 0,
  REAR_CASTER,
  MAIN_LEFT,
  MAIN_RIGHT,
  LEFT_CARRIAGE,
  RIGHT_CARRIAGE,
};
inline constexpr std::size_t leg_count = 6;

/// @brief One leg actuator: where it is on the bus and how to move it.
///
/// Positions are the joint encoder's counts. The range is the calibrated travel and is
/// only a default: a range saved to NVS by the calibration routine overrides it at boot.
/// Kinematics (counts to wheel height) do not belong here; see the leg geometry below
/// once measured.
struct ActuatorConfig {
  Leg leg;
  const char *name;          ///< Short name, also the NVS key prefix (11 characters max).
  std::uint8_t node_id;      ///< CANopen node id of the MCP266.
  std::uint8_t channel;      ///< 0 = M1, 1 = M2.
  std::int32_t min_counts;   ///< Default calibrated travel, inclusive.
  std::int32_t max_counts;
  std::uint32_t velocity;    ///< Profile cruise speed, counts/s.
  std::uint32_t acceleration; ///< counts/s^2.
  std::uint32_t deceleration; ///< counts/s^2.
  std::int32_t jog_step;     ///< Counts per increment() / decrement().
  std::int32_t tolerance;    ///< Counts within which the joint counts as at target.
  bool hardware_limits;      ///< Limit switches wired to the controller.
  bool incremental_encoder;  ///< AB encoder: arbitrary count at power-up, must be homed.
  std::int8_t home_direction; ///< +1 or -1: which way the home switch lies (incremental only).
  std::int32_t home_count;   ///< Count installed at the home switch (incremental only).
};

// TODO(bench): bench values, not the chair's: three MCP266s at nodes 1 (casters), 2
// (carriages) and 3 (main legs) driving rotary motors on absolute encoders that count to
// 80000. Confirm the chair's node ids with the debugger's MCP266 / CAN page, calibrate
// the ranges, then replace these values.
// The four casters and main legs carry absolute encoders: calibrated once per
// installation. The carriages carry incremental (AB) optical encoders on the rail and
// limit switches: homed against the switch at min every boot, or restored from the
// position saved while running.
inline constexpr ActuatorConfig actuators[leg_count] = {
    {Leg::FRONT_CASTER, "FC", 1, 0, 0, 80000, 2000, 2000, 2000, 2000, 10, false, false, 0, 0},
    {Leg::REAR_CASTER, "RC", 1, 1, 0, 80000, 2000, 2000, 2000, 2000, 10, false, false, 0, 0},
    {Leg::MAIN_LEFT, "ML", 3, 0, 0, 80000, 2000, 2000, 2000, 2000, 10, false, false, 0, 0},
    {Leg::MAIN_RIGHT, "MR", 3, 1, 0, 80000, 2000, 2000, 2000, 2000, 10, false, false, 0, 0},
    {Leg::LEFT_CARRIAGE, "LCarr", 2, 0, 0, 20000, 500, 500, 500, 50, 10, true, true, -1, 0},
    {Leg::RIGHT_CARRIAGE, "RCarr", 2, 1, 0, 20000, 500, 500, 500, 50, 10, true, true, -1, 0},
};

inline constexpr const ActuatorConfig &actuator_config(Leg leg) {
  return actuators[static_cast<std::size_t>(leg)];
}

} // namespace mib::config
