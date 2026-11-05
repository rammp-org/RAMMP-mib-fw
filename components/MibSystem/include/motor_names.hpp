#pragma once

#include "messages.hpp"

namespace mib {

/// @brief Names for MotorState enums, as the control panel and the console log print them.
inline const char *board_state_name(rammp::BoardState s) {
  switch (s) {
  case rammp::BoardState::DISARMED:
    return "DISARMED";
  case rammp::BoardState::ARMED:
    return "ARMED";
  case rammp::BoardState::SAFE_STOPPING:
    return "SAFE_STOPPING";
  case rammp::BoardState::HOLDING:
    return "HOLDING";
  case rammp::BoardState::FAULT:
    return "FAULT";
  }
  return "UNKNOWN";
}

inline const char *fault_name(rammp::FaultCode f) {
  switch (f) {
  case rammp::FaultCode::NONE:
    return "NONE";
  case rammp::FaultCode::WATCHDOG:
    return "WATCHDOG";
  case rammp::FaultCode::OVERCURRENT:
    return "OVERCURRENT";
  case rammp::FaultCode::VDS_OCP:
    return "VDS_OCP";
  case rammp::FaultCode::OVERTEMP:
    return "OVERTEMP";
  case rammp::FaultCode::ENCODER:
    return "ENCODER";
  case rammp::FaultCode::DRV_FAULT:
    return "DRV_FAULT";
  case rammp::FaultCode::UNASSIGNED_AXIS:
    return "UNASSIGNED_AXIS";
  case rammp::FaultCode::SAMPLER:
    return "SAMPLER";
  case rammp::FaultCode::HALL:
    return "HALL";
  case rammp::FaultCode::STALL:
    return "STALL";
  }
  return "UNKNOWN";
}

} // namespace mib
