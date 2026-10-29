#pragma once

/// @file ros2_interface.hpp
/// @brief The MIB's ROS 2 endpoints: topic and type names on the wire, and the payloads.
///
/// ROS 2 nodes (the Jetson) reach the MIB here, beside the rammp-rtps endpoints the
/// joystick and PACE RACER boards use. The types are rammp-interfaces-ros2's, so a ROS 2
/// node needs no MIB-specific package. docs/ros2.md is the handoff to the ROS 2 side:
/// keep it in step with this file.
///
/// ROS 2 mangles names before they reach DDS: topic /a/b is rt/a/b, and type
/// pkg/msg/T is pkg::msg::dds_::T_.

#include <cstdint>
#include <string>

#include "messages.hpp"

namespace mib::ros2 {

/// @brief rammp_base_interfaces/msg/MibState. Each struct is the wire layout (XCDR1,
///        fields in declaration order): do not reorder.
struct MibState {
  uint8_t system_state{0};               ///< One of the SYSTEM_STATE_* values below.
  std::string error_message{"No error"}; ///< Human-readable; "No error" when none.
};

/// @name MibState::system_state values (MibState.msg's SYSTEM_STATE_* constants)
/// @{
inline constexpr uint8_t SYSTEM_STATE_INITIALIZING = 0;
inline constexpr uint8_t SYSTEM_STATE_IDLE = 1;
inline constexpr uint8_t SYSTEM_STATE_ENABLED = 2;
inline constexpr uint8_t SYSTEM_STATE_ERROR = 3;
/// @}

/// @brief MibState's coarser state for one of the MIB's own states.
/// @param state The state the MIB reports to the joystick.
/// @return One of the SYSTEM_STATE_* values. Another node driving (AUTONOMOUS, SHARED)
///         is ENABLED; an e-stop is ERROR, with error_message saying so.
constexpr uint8_t to_mib_state(MIB::MibSystemState state) {
  switch (state) {
  case MIB::MibSystemState::INITIALIZING:
    return SYSTEM_STATE_INITIALIZING;
  case MIB::MibSystemState::IDLE:
    return SYSTEM_STATE_IDLE;
  case MIB::MibSystemState::ENABLED:
  case MIB::MibSystemState::AUTONOMOUS:
  case MIB::MibSystemState::SHARED:
    return SYSTEM_STATE_ENABLED;
  case MIB::MibSystemState::ERROR:
  case MIB::MibSystemState::ESTOP:
    return SYSTEM_STATE_ERROR;
  }
  return SYSTEM_STATE_ERROR;
}

/// MIB -> ROS 2: /rammp/mib/status, best-effort, resent every publication tick.
inline constexpr rammp::Topic<MibState> kMibState{"rt/rammp/mib/status",
                                                  "rammp_base_interfaces::msg::dds_::MibState_"};

/// ROS 2 -> MIB: /rammp/joystick/xy_twist, best-effort. rammp::XYTwist has the same layout
/// as rammp_peripheral_interfaces/msg/XYTwist. The MIB logs it and never drives on it:
/// driving from ROS 2 goes through arbitration (/base/setpoint/twist).
inline constexpr rammp::Topic<rammp::XYTwist> kXYTwist{
    "rt/rammp/joystick/xy_twist", "rammp_peripheral_interfaces::msg::dds_::XYTwist_"};

} // namespace mib::ros2
