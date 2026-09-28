#pragma once

#include <chrono>
#include <cstddef>
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
    .invert_left = true,
    .invert_right = false,
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

// TODO(bench): node ids, channels, ranges and profiles are placeholders. Node ids follow
// espp's default of 10 for the first controller. Confirm each with the debugger's
// MCP266 / CAN page, calibrate the ranges, then replace these values.
// The four casters and main legs carry absolute encoders: calibrated once per
// installation. The carriages carry incremental (AB) optical encoders on the rail and
// limit switches: homed against the switch at min every boot, or restored from the
// position saved while running.
inline constexpr ActuatorConfig actuators[leg_count] = {
    {Leg::FRONT_CASTER, "FC", 10, 0, 0, 4095, 500, 500, 500, 50, 10, false, false, 0, 0},
    {Leg::REAR_CASTER, "RC", 10, 1, 0, 4095, 500, 500, 500, 50, 10, false, false, 0, 0},
    {Leg::MAIN_LEFT, "ML", 11, 0, 0, 4095, 500, 500, 500, 50, 10, false, false, 0, 0},
    {Leg::MAIN_RIGHT, "MR", 11, 1, 0, 4095, 500, 500, 500, 50, 10, false, false, 0, 0},
    {Leg::LEFT_CARRIAGE, "LCarr", 12, 0, 0, 20000, 500, 500, 500, 50, 10, true, true, -1, 0},
    {Leg::RIGHT_CARRIAGE, "RCarr", 12, 1, 0, 20000, 500, 500, 500, 50, 10, true, true, -1, 0},
};

inline constexpr const ActuatorConfig &actuator_config(Leg leg) {
  return actuators[static_cast<std::size_t>(leg)];
}

} // namespace mib::config
