#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "arbiter.hpp"
#include "driver/gpio.h"
#include "seat_geometry.hpp"

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

// TODO(bench): ranges and profiles are placeholders; calibrate the ranges, then replace them.
// Node ids are those set in Motion Studio: 1 casters, 2 carriages, 3 main legs. The main
// legs' channels are confirmed from the chair's wiring: M1 is the right leg, M2 the left.
// Confirm the caster and carriage channels the same way.
// The four casters and main legs carry absolute encoders: calibrated once per
// installation. The carriages carry incremental (AB) optical encoders on the rail and
// limit switches: homed against the switch at min every boot, or restored from the
// position saved while running.
inline constexpr ActuatorConfig actuators[leg_count] = {
    {Leg::FRONT_CASTER, "FC", 1, 0, 0, 4095, 500, 500, 500, 50, 200, false, false, 0, 0},
    {Leg::REAR_CASTER, "RC", 1, 1, 0, 4095, 500, 500, 500, 50, 200, false, false, 0, 0},
    {Leg::MAIN_LEFT, "ML", 3, 1, 0, 4095, 500, 500, 500, 50, 200, false, false, 0, 0},
    {Leg::MAIN_RIGHT, "MR", 3, 0, 0, 4095, 500, 500, 500, 50, 200, false, false, 0, 0},
    {Leg::LEFT_CARRIAGE, "LCarr", 2, 0, 0, 20000, 500, 500, 500, 50, 200, true, true, -1, 0},
    {Leg::RIGHT_CARRIAGE, "RCarr", 2, 1, 0, 20000, 500, 500, 500, 50, 200, true, true, -1, 0},
};

inline constexpr const ActuatorConfig &actuator_config(Leg leg) {
  return actuators[static_cast<std::size_t>(leg)];
}

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

/// @brief Seat kinematics geometry (millimetres, degrees).
///
/// Derived from the Demo-Software URDF
/// (core/rammp_prototype_description/urdf/rammp_prototype_base.urdf), converted to millimetres.
/// Chassis frame = rammp base frame (URDF root link): x forward, y left, z up.
/// Each comment names the URDF joint(s) a number comes from.
inline constexpr SeatGeometry seat_geometry{
    .main_left =
        {
            // dw_main_plate_l origin + motor_swing_arm_l origin
            .pivot_mm = {.x_mm = 166.82f, .y_mm = 182.00f, .z_mm = 113.42f},
            // drive_motor_l origin + drive_wheel_l origin
            .pivot_to_wheel_mm = {.x_mm = 318.77f, .y_mm = 119.54f, .z_mm = -13.80f},
            .wheel_radius_mm = 184.06f, // drive_wheel_l collision cylinder
            .min_arm_deg = -45.0f,      // motor_swing_arm_l limits
            .max_arm_deg = 45.0f,
        },
    .main_right =
        {
            // dw_main_plate_r origin + motor_swing_arm_r origin
            .pivot_mm = {.x_mm = 166.82f, .y_mm = -187.12f, .z_mm = 113.42f},
            // drive_motor_r origin + drive_wheel_r origin
            .pivot_to_wheel_mm = {.x_mm = 318.77f, .y_mm = -119.55f, .z_mm = -13.80f},
            .wheel_radius_mm = 184.06f, // drive_wheel_r collision cylinder
            .min_arm_deg = -45.0f,      // motor_swing_arm_r limits
            .max_arm_deg = 45.0f,
        },
    .rear_caster =
        {
            // rear_caster_arm_mount origin + rear_caster_swing_arm origin
            .pivot_mm = {.x_mm = -31.75f, .y_mm = 0.11f, .z_mm = 8.89f},
            // rear_casteer_wheel_l / rear_caster_wheel_r origins, y averaged to one wheel
            .pivot_to_wheel_mm = {.x_mm = -209.55f, .y_mm = 0.0f, .z_mm = 12.70f},
            .wheel_radius_mm = 101.07f, // rear_caster_wheel_r collision cylinder
            .min_arm_deg = -45.0f,      // rear_caster_swing_arm limits
            .max_arm_deg = 45.0f,
        },

    // Pins relative to the swing-arm pivot, in the arm's (x, z) plane.
    // Arm pin: motor_elevator_l origin. Plate pin: motor_elevator_pivot_l origin.
    // TODO(bench): confirm the plate pin from CAD. motor_elevator_rod_link_l is the other
    // candidate (50.8 mm higher) and changes stroke by up to ~13 mm at the ends of travel.
    // TODO(bench): retracted length. Placeholder = pin-to-pin length at arm angle −45°,
    // to 0.01 mm (287.5836).
    .main_left_linkage =
        {
            .arm_pin_x_mm = -103.72f,
            .arm_pin_z_mm = 126.68f,
            .plate_pin_x_mm = 298.85f,
            .plate_pin_z_mm = 109.69f,
            .retracted_length_mm = 287.58f,
        },
    .main_right_linkage =
        {
            .arm_pin_x_mm = -103.72f,
            .arm_pin_z_mm = 126.68f,
            .plate_pin_x_mm = 298.85f,
            .plate_pin_z_mm = 109.69f,
            .retracted_length_mm = 287.58f,
        },

    // dw_main_plate_l / dw_main_plate_r limits.
    // TODO(bench): Demo-Software's calibrated carriage range was 0–300 mm.
    .min_carriage_mm = -400.0f,
    .max_carriage_mm = 400.0f,

    // Ground clearance of the swing-arm pivots, from the pivot heights above. 0 only keeps the
    // pivots themselves out of the floor (with a level seat, elevation down to -8.89 mm, the rear
    // caster pivot's height).
    // TODO(bench): measure the lowest chassis / carriage-plate point below each pivot and the
    // margin wanted, and raise this to match.
    .min_pivot_clearance_mm = 0.0f,

    // TODO(bench): every transmission below is a placeholder (1 mm per rev, 1:1, no offset)
    // until the actuator lead, gear ratios and directions are measured.
    .main_left_transmission = Transmission::linear(1.0f, 1.0f),
    .main_right_transmission = Transmission::linear(1.0f, 1.0f),
    // Encoder sits after the shock, on the swing arm: 1:1 is real; sign and offset are not.
    .rear_caster_transmission = Transmission::rotary(1.0f),
    .left_carriage_transmission = Transmission::linear(1.0f, 1.0f),
    .right_carriage_transmission = Transmission::linear(1.0f, 1.0f),
};

// A zero gear ratio would make actuator degrees -> joint position divide by zero.
static_assert(seat_geometry.main_left_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.main_right_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.rear_caster_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.left_carriage_transmission.deg_per_unit != 0.0f);
static_assert(seat_geometry.right_carriage_transmission.deg_per_unit != 0.0f);
} // namespace mib::config
