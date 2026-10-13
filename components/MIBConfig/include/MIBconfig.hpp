#pragma once

#include <chrono>
#include <cstdint>

#include "seat_geometry.hpp"

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
