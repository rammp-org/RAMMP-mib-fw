#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "arbiter.hpp"
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

/// @brief Which external controller may drive the base (docs/arbitration.md). ENFORCED: a
/// TwistSetpoint drives only with the token its sender was granted through
/// /base/acquire_control. DISABLED: every setpoint is admitted; ownership is still tracked and
/// reported on /base/control_status, and the e-stop still refuses everything. The joystick and
/// the push-assist board carry no token and are not arbitrated in either mode.
///
/// Compile-time, like the arm driver's read-only arbitration_mode launch parameter, and with
/// the same default: DISABLED until the orchestrator acquires the base for the task nodes.
/// idf.py -DMIB_ARBITRATION=enforced build turns the token check on; the mode in force is
/// reported as arbitration_enabled on /base/control_status.
#if defined(MIB_ARBITRATION_ENFORCED)
inline constexpr mib::ArbitrationMode arbitration_mode = mib::ArbitrationMode::ENFORCED;
#else
inline constexpr mib::ArbitrationMode arbitration_mode = mib::ArbitrationMode::DISABLED;
#endif

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

/// @brief IPv4 gateway of the MIB Ethernet interface, also offered to DHCP clients as their router.
/// @details 0.0.0.0 means no gateway: the robot LAN has no uplink, and ESP-IDF's DHCP server only
/// sends the router option when this is nonzero. Advertising the MIB as a router makes hosts that
/// also have Wi-Fi send their internet traffic here whenever Wi-Fi drops (#73).
inline constexpr std::uint8_t ethernet_gateway[4] = {0, 0, 0, 0};

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

/// Compile-time per robot: idf.py -DMIB_ROBOT=flatbot build. The default is the chair.
#if defined(MIB_ROBOT_FLATBOT)
/// The office rover: 9 in wheels, 28 in between the drive wheels. Its caster is 19.5 in
/// from them, which the differential drive maths does not use.
inline constexpr DifferentialDriveConfig differential_drive{
    .wheel_diameter_m = 0.2286f,
    .wheel_separation_m = 0.7112f,
    .invert_left = false,
    .invert_right = true,
};
#else
inline constexpr DifferentialDriveConfig differential_drive{
    .wheel_diameter_m = 0.355f,
    .wheel_separation_m = 0.6096f,
    .invert_left = false,
    .invert_right = true,
};
#endif

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

/// @brief Gains for the push-assist system.
struct PushAssistGains {
  float dead_band_n;    // per handle; below this a handle reads as 0
  float forward_full_n; // forward force past the dead bands for linear = 1
  float turn_full_n;    // left - right past the dead bands for |angular| = 1
  float reverse_max;    // cap on backing up when the handles are pulled, 0..1
};

//---Stand-in for Alex's code---
inline constexpr PushAssistGains push_assist_gains{
    .dead_band_n = 5.0f,
    .forward_full_n = 40.0f,
    .turn_full_n = 20.0f,
    .reverse_max = 0.3f,
};
//--------------------------------

/// @brief CAN bus to the MCP266 motor controllers (TWAI peripheral).
inline constexpr int can_tx_gpio = 17;
inline constexpr int can_rx_gpio = 16;
inline constexpr std::uint32_t can_bitrate = 1000000;

/// @brief How long an SDO waits for the controller's reply. Replies take under 2 ms; a
/// short timeout keeps one lost reply (about one a minute on the chair) to two ticks of a
/// move instead of ten.
inline constexpr auto can_sdo_timeout = std::chrono::milliseconds{20};

/// @brief The MIB's CANopen heartbeat (COB-ID 0x700 + node id, state operational), sent
/// from boot for as long as the firmware runs. Each commissioned controller's heartbeat
/// consumer watches it: if it stops for can_heartbeat_timeout, an enabled drive faults
/// (measured: ~450 ms). It cannot be switched off again (writing 0 to the consumer does not
/// stop the controller watching), so the heartbeat never stops on purpose.
inline constexpr std::uint8_t can_heartbeat_node_id = 127;
inline constexpr auto can_heartbeat_period = std::chrono::milliseconds{100};
inline constexpr auto can_heartbeat_timeout = std::chrono::milliseconds{500};

/// @brief How often MibSystem's leg task reads one idle leg's encoder for the control
/// panel's status (round robin, so each of the leg_count legs every leg_count x this).
/// A moving leg's count comes from its move ticks instead.
inline constexpr auto leg_position_refresh = std::chrono::milliseconds{250};

/// @brief The legs whose joint is positioned by a linear actuator (an EHA) on one channel of
/// a Basicmicro MCP266. This chair has no powered casters (no node 1 on its bus).
enum class Leg : std::uint8_t {
  MAIN_RIGHT = 0,
  MAIN_LEFT,
  LEFT_CARRIAGE,
  RIGHT_CARRIAGE,
};
inline constexpr std::size_t leg_count = 4;

/// @brief What the MIB writes to a leg's controller channel at boot (controller RAM only; its
/// flash keeps Motion Studio's values, which return at the next power cycle). Raw controller
/// units. What the gains mean is in components/Actuator/DESIGN.md.
struct ActuatorTuning {
  std::uint32_t position_window; ///< CiA 402 0x6067, counts. Stored: 100.
  std::uint32_t position_p;      ///< x1024: counts/s of speed command per count of error.
  std::uint32_t position_i;      ///< x1024.
  std::uint32_t position_d;      ///< x1024.
  std::uint32_t position_max_i;
  std::uint32_t position_deadband; ///< Counts.
  std::uint32_t velocity_p;        ///< 16.16.
  std::uint32_t velocity_i;        ///< 16.16.
  std::uint32_t velocity_d;        ///< 16.16.
  std::uint32_t velocity_qpps;     ///< Counts/s the velocity loop takes as full output.
  std::uint32_t profile_velocity;  ///< CiA 402 0x6081: caps the speed command; set high.
};

/// @brief One leg actuator: where it is on the bus, whether the MIB may drive it, and how.
///
/// Positions are the joint encoder's counts. The range is the travel, written to the
/// controller as its clamp; a range saved to NVS by the calibration routine overrides it at
/// boot. Kinematics (counts to wheel height) do not belong here.
struct ActuatorConfig {
  Leg leg;
  const char *name;     ///< Short name, also the NVS key prefix (11 characters max).
  std::uint8_t node_id; ///< CANopen node id of the MCP266.
  std::uint8_t channel; ///< 0 = M1, 1 = M2.
  /// False: the MIB never writes to this channel (no settings, no heartbeat consumer, no
  /// enable, no move, no stop) and refuses every move. Only explicit reads reach it.
  bool commissioned;
  std::int32_t min_counts; ///< Travel, inclusive: the controller's clamp.
  std::int32_t max_counts;
  /// Which way the EHA's stroke goes for increasing counts: +1 extends, -1 retracts, 0 not
  /// established. The two main legs are mounted mirrored and have OPPOSITE signs.
  std::int8_t stroke_sign;
  std::uint32_t max_speed;    ///< Streamed cruise speed, counts/s.
  std::uint32_t acceleration; ///< Counts/s^2, up and down.
  std::uint32_t lead_ms;      ///< Target lead: 1 / position P (P 4.0 -> 250 ms).
  std::int32_t tolerance;     ///< Counts within which the median count must settle.
  std::int32_t jog_step;      ///< Counts per increment() / decrement().
  ActuatorTuning tuning;
  bool hardware_limits;       ///< Limit switches wired to the controller.
  bool incremental_encoder;   ///< AB encoder: arbitrary count at power-up, must be homed.
  std::int8_t home_direction; ///< +1 or -1: which way the home switch lies (incremental only).
  std::int32_t home_count;    ///< Count installed at the home switch (incremental only).
};

/// @brief The right main leg's controller settings, proven on the chair on 2026-11-17
/// (docs/actuator-revamp/bench-findings.md: "1,000-count moves at 800 counts/s" and the
/// full-range check): window 10, position P 4.0 with no I/D/deadband, velocity P 1.0,
/// I 1701, QPPS 7143.
inline constexpr ActuatorTuning main_leg_tuning{
    .position_window = 10,
    .position_p = 4096,
    .position_i = 0,
    .position_d = 0,
    .position_max_i = 0,
    .position_deadband = 0,
    .velocity_p = 65536,
    .velocity_i = 1701,
    .velocity_d = 0,
    .velocity_qpps = 7143,
    .profile_velocity = 20000,
};

/// @brief NEW CHAIR BRING-UP (2026-11-19): deliberately gentle first settings for its main legs.
/// The chair's stored values (velocity P ~78,000-82,000 with QPPS 119-165, so ~1 % PWM per
/// count/s of speed error, position P ~0.1-0.15, window 100) would slam a step. These give
/// PWM = (65536/65536) / 7143 x (1.0 x error - speed): ~1.4 % per 100 counts of error, so a
/// leg may not move at all before the gains are raised on the chair.
inline constexpr ActuatorTuning bringup_tuning{
    .position_window = 10,
    .position_p = 1024,
    .position_i = 0,
    .position_d = 0,
    .position_max_i = 0,
    .position_deadband = 0,
    .velocity_p = 65536,
    .velocity_i = 1701,
    .velocity_d = 0,
    .velocity_qpps = 7143,
    .profile_velocity = 20000,
};

/// @brief Placeholder for legs never tuned: all gains 0, so a leg commissioned by mistake
/// produces no output. Tune a leg (Motion Studio or the Actuator example) before
/// commissioning it.
inline constexpr ActuatorTuning untuned{
    .position_window = 10,
    .position_p = 0,
    .position_i = 0,
    .position_d = 0,
    .position_max_i = 0,
    .position_deadband = 0,
    .velocity_p = 0,
    .velocity_i = 0,
    .velocity_d = 0,
    .velocity_qpps = 7143,
    .profile_velocity = 20000,
};

// Wiring read on the chair (bench-findings.md, "Wiring and identity"): node 3 M1 = right
// main leg, node 3 M2 = left main leg, node 2 = the carriages (which channel is which is
// not yet confirmed). The main legs carry absolute encoders; the carriages incremental
// (AB) encoders on the rail and limit switches.
inline constexpr ActuatorConfig actuators[leg_count] = {
    // NEW CHAIR BRING-UP (2026-11-19). Node 3 M1 / M2 as read over Ethernet: counts ~59,580 and
    // ~60,290; stored clamps 58,000..64,000 and 60,000..66,000. Which leg each is and which way
    // +counts goes are NOT known yet: stroke_sign 0. The range is the count read +-300 (inside
    // the stored clamp) so a leg that runs the wrong way leaves it within ~300 counts. Speed
    // capped at 200 counts/s. Widen the range only by calibration (LEG_CALIBRATE + jogs).
    {.leg = Leg::MAIN_RIGHT,
     .name = "MR",
     .node_id = 3,
     .channel = 0,
     .commissioned = true,
     .min_counts = 59280,
     .max_counts = 59880,
     .stroke_sign = 0,
     .max_speed = 200,
     .acceleration = 400,
     .lead_ms = 1000, // 1 / position P 1.0
     .tolerance = 50,
     .jog_step = 50,
     .tuning = bringup_tuning,
     .hardware_limits = false,
     .incremental_encoder = false,
     .home_direction = 0,
     .home_count = 0},
    {.leg = Leg::MAIN_LEFT,
     .name = "ML",
     .node_id = 3,
     .channel = 1,
     .commissioned = true,
     .min_counts = 60000,
     .max_counts = 60600,
     .stroke_sign = 0,
     .max_speed = 200,
     .acceleration = 400,
     .lead_ms = 1000,
     .tolerance = 50,
     .jog_step = 50,
     .tuning = bringup_tuning,
     .hardware_limits = false,
     .incremental_encoder = false,
     .home_direction = 0,
     .home_count = 0},
    // Carriages: NOT commissioned, and must never move by accident. Nothing at boot writes
    // to node 2, and homing is never automatic. Ranges and homing are placeholders.
    {.leg = Leg::LEFT_CARRIAGE,
     .name = "LCarr",
     .node_id = 2,
     .channel = 0,
     .commissioned = false,
     .min_counts = 0,
     .max_counts = 20000,
     .stroke_sign = 0,
     .max_speed = 500,
     .acceleration = 1000,
     .lead_ms = 250,
     .tolerance = 40,
     .jog_step = 100,
     .tuning = untuned,
     .hardware_limits = true,
     .incremental_encoder = true,
     .home_direction = -1,
     .home_count = 0},
    {.leg = Leg::RIGHT_CARRIAGE,
     .name = "RCarr",
     .node_id = 2,
     .channel = 1,
     .commissioned = false,
     .min_counts = 0,
     .max_counts = 20000,
     .stroke_sign = 0,
     .max_speed = 500,
     .acceleration = 1000,
     .lead_ms = 250,
     .tolerance = 40,
     .jog_step = 100,
     .tuning = untuned,
     .hardware_limits = true,
     .incremental_encoder = true,
     .home_direction = -1,
     .home_count = 0},
};

static_assert(
    [] {
      for (std::size_t i = 0; i < leg_count; ++i) {
        if (static_cast<std::size_t>(actuators[i].leg) != i) {
          return false;
        }
      }
      return true;
    }(),
    "actuators[] must be in Leg order: actuator_config() indexes it by Leg");

inline constexpr const ActuatorConfig &actuator_config(Leg leg) {
  return actuators[static_cast<std::size_t>(leg)];
}

} // namespace mib::config
