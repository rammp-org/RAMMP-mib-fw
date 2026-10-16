#pragma once

#include <chrono>
#include <cstdint>

#include "hal/adc_types.h"

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

// Load Cell Configuration
struct LoadCellConfig {
  float capacity_lbf;
  float fso_mv_per_v;  // from the unit's calibration sheet
  float excitation_mv;
  float rg_ohms;       
  float ref_mv;        
  adc_unit_t adc_unit;
  adc_channel_t adc_channel;
  adc_atten_t adc_atten;
  size_t samples;
  float filter_alpha;
 
};
 
inline constexpr LoadCellConfig load_cell{
    .capacity_lbf = 1000.0f,
    .fso_mv_per_v = 3.0000f,
    .excitation_mv = 10000.0f,
    .rg_ohms = 1280.0f,
    .ref_mv = 1650.0f,
    .adc_unit = ADC_UNIT_1,
    .adc_channel = ADC_CHANNEL_4, // GPIO20
    .adc_atten = ADC_ATTEN_DB_12, // ~0-3100 mV
    .samples = 16,
    .filter_alpha = 0.15f,
};

} // namespace mib::config
