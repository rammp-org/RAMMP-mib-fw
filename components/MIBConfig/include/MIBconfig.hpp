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

/// @brief Joystick driving needs both drive boards reporting MotorState and neither in
/// FAULT; otherwise the MIB stops both wheels. false drives on whatever answers (bench use).
inline constexpr bool drive_requires_both_boards = true;
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
///
/// The planner limits linear and angular acceleration separately, but a wheel sees their
/// sum: a / r + alpha * (separation / 2) / r. Slowing from straight driving into a sharp
/// turn asks the inside wheel for both at once, and that is where the hub boards tripped
/// on overcurrent (2026-11-01). Jerk limits shape the start of each ramp (S-curve, espp
/// trajectory_planner >= 1.3.9); 0 keeps that axis trapezoidal. Stopping stays
/// trapezoidal so a stop lands without overshoot.
struct DriveProfileConfig {
  float max_linear_velocity_mps;
  float max_angular_velocity_radps;
  float max_linear_acceleration_mps2;
  float max_angular_acceleration_radps2;
  float max_linear_jerk_mps3{0.0f};
  float max_angular_jerk_radps3{0.0f};
  /// Stick released: decelerations toward (0, 0). 0 = the driving accelerations.
  float stopping_linear_acceleration_mps2{0.0f};
  float stopping_angular_acceleration_radps2{0.0f};
  /// |v * w| ceiling. Over it the planner scales v and w down together, so steering while
  /// driving also cuts forward speed. 0.1 is espp's default, which LOW and HIGH keep.
  float max_centripetal_acceleration_mps2{0.1f};
  /// Speed at full reverse stick. 0 = the same as max_linear_velocity_mps.
  float max_reverse_linear_velocity_mps{0.0f};
};

/// @brief MotorCommand.accel_limit sent to each drive board, as a multiple of the
/// profile's worst-case wheel acceleration. Above 1 so the board's ramp keeps up with
/// the planner and only smooths the 20 Hz command steps; 0 sends no limit (the board
/// steps its speed reference to every new command).
inline constexpr float drive_wheel_accel_limit_margin = 1.25f;

/// @brief Coast at rest (TEST 2026-11-01). With the stick centred and the planner stopped,
/// the MIB sends ControlMode::COAST (boards Hi-Z, no current) once both wheels are under
/// drive_coast_enter_radps or drive_coast_enter_ticks have passed, whichever is first. The
/// boards' position hold at rest rocked the chair (+-0.5 rad/s, +-5 A, 0.65 s period), and
/// waiting for the wheels to settle let it start. Roll guard: a wheel that travels more than
/// drive_roll_guard_travel_rad since coasting began (a slope, a push) switches to a
/// zero-speed hold until the stick moves. Distance, not speed: the chair springs back up to
/// ~0.5 rad/s right after a stop, which a speed guard mistook for rolling away.
inline constexpr bool drive_coast_at_rest = true;
inline constexpr float drive_coast_enter_radps = 0.2f;
inline constexpr int drive_coast_enter_ticks = 8; // x 50 ms: coast after 0.4 s at rest anyway
inline constexpr float drive_roll_guard_travel_rad = 1.0f; // ~13 cm at the rim

/// @brief Inside-wheel torque cap for tight turns (TEST 2026-11-01). In a tight arc the
/// planner asks the inside wheel for a crawl (0.4 rad/s against 2.7 outside); scrub holds it
/// still and the board's speed loop wound up to 20-30 A trying to move it. A wheel commanded
/// below drive_inside_wheel_speed_ratio of the other wheel's speed, with the other wheel at
/// least drive_inside_wheel_min_outer_radps, is sent this torque_limit instead of 0 (the
/// board's 30 A burst ceiling): 23.4 N.m is 15 A on the hub20 (Kt 1.56), the boards'
/// continuous rating. It lets that wheel stall and the chair pivot round it. Straight
/// driving and spins (equal wheel speeds) are never capped. 0 turns the cap off.
inline constexpr float drive_inside_wheel_torque_limit_nm = 23.4f;
inline constexpr float drive_inside_wheel_speed_ratio = 0.33f;
inline constexpr float drive_inside_wheel_min_outer_radps = 0.5f;

inline constexpr DriveProfileConfig drive_profile_low{
    .max_linear_velocity_mps = 0.35f,
    .max_angular_velocity_radps = 1.0f,
    .max_linear_acceleration_mps2 = 0.5f,
    .max_angular_acceleration_radps2 = 1.0f,
};

/// NORMAL was 0.75 m/s, 2.0 rad/s, 1.0 m/s^2 and 2.0 rad/s^2: up to 12.3 rad/s^2 at one
/// wheel. TEST 2026-11-01, second step: steady turning costs ~11 A of scrub plus ~2.5 A per
/// rad/s of wheel turn speed, and accelerating out of a turn stacked ~10 A on top of that
/// (drive_left tripped at 25 A of iq). So the turn rate is halved (wheel turn component at
/// most 2.2 rad/s, ~13 A steady) and both accelerations halved again: at most 3.7 rad/s^2
/// per wheel while driving. Jerk keeps the 0.2 s ramp-in. Stopping keeps the first step's
/// 0.6 m/s^2 and 1.2 rad/s^2, which ran clean, so releasing the stick still stops briskly.
/// Third step: the turn rate back up to 1.5 rad/s (spins ~16 A), and the centripetal limit
/// set to 1.0 m/s^2. Left at espp's 0.1 it allowed 0.2 rad/s at 0.5 m/s and halved the
/// command 938 times in an 81 s drive: steering felt dead and adding a little made the
/// chair lurch, because the limit cuts v together with w.
/// Fourth step: angular accel 0.6 -> 1.5 rad/s^2 (jerk keeps the 0.2 s ramp). The planner
/// uses the same limit to take a turn away, so at 0.6 the turn rate took 0.6-1.3 s to die
/// after the stick came back to centre, plus ~0.3 s of wheel lag: the chair overshot and
/// fishtailed lining up with a door. Per wheel that is still at most 5.7 rad/s^2 driving.
/// Reverse capped at 0.4 m/s: a full-speed reverse arc tripped drive_left (24 A of iq, ~35 A
/// of phase current at 63 rpm on the 16 V current-loop limit).
/// Fifth step, from the driver: spinning in place at 1.5 rad/s was dizzying and fast big
/// circles flung him outward. Turn rate 1.0 rad/s, angular accel 1.0 rad/s^2 (a turn still
/// dies in ~0.5 s; stick fully released stops it at 1.5), centripetal cap 0.5 m/s^2. With
/// the HMI's circular stick, |v*w| peaks at 0.75 * 1.0 / 2 = 0.375 m/s^2 on the diagonal,
/// so the cap is a backstop and does not cut commands (no lurch). The 1.5 rad/s^2 turn-in
/// also had drive_left's outside wheel at its 30 A ceiling for a second, and it tripped.
inline constexpr DriveProfileConfig drive_profile_normal{
    .max_linear_velocity_mps = 0.75f,
    .max_angular_velocity_radps = 1.0f,
    .max_linear_acceleration_mps2 = 0.3f,
    .max_angular_acceleration_radps2 = 1.0f,
    .max_linear_jerk_mps3 = 1.5f,
    .max_angular_jerk_radps3 = 5.0f,
    .stopping_linear_acceleration_mps2 = 0.6f,
    .stopping_angular_acceleration_radps2 = 1.5f,
    .max_centripetal_acceleration_mps2 = 0.5f,
    .max_reverse_linear_velocity_mps = 0.4f,
};

inline constexpr DriveProfileConfig drive_profile_high{
    .max_linear_velocity_mps = 1.0f,
    .max_angular_velocity_radps = 3.14159f,
    .max_linear_acceleration_mps2 = 2.0f,
    .max_angular_acceleration_radps2 = 4.0f,
};

} // namespace mib::config
