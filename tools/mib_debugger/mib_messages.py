"""DDS type definitions for the RAMMP MIB, mirroring external/rammp-rtps.

Every struct here IS the wire layout: classic little-endian CDR (XCDR1) with
fields in declaration order, matching the C++ headers in
external/rammp-rtps/components/rammp_rtps_messages/include/messages/.

The C++ side declares its enums as `enum class X : uint8_t`, so they occupy one
byte on the wire. IDL enums default to four bytes, so every such field is
declared here as uint8 and the readable names live in the IntEnum classes
alongside. Changing a field to a plain IDL enum would silently break the wire
format.

Topic and type names must match the firmware exactly. embeddedRTPS compares
both with strcmp in TopicData::matchesTopicOf, so a single differing character
means the endpoints never pair up and no data flows.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List

import cyclonedds.idl as idl
import cyclonedds.idl.types as pt
from cyclonedds.idl import IdlStruct


# --------------------------------------------------------------------------
# Enumerations. These mirror the C++ scoped enums; each is one byte on the wire.
# --------------------------------------------------------------------------

class DriveProfile(IntEnum):
    LOW = 0
    NORMAL = 1
    HIGH = 2


class MibSystemState(IntEnum):
    INITIALIZING = 0
    IDLE = 1
    ENABLED = 2
    ERROR = 3


class DriveRequest(IntEnum):
    DISABLE = 0
    ENABLE = 1


class SeatAxis(IntEnum):
    FRONT_BACK_TILT = 0
    LATERAL_TILT = 1
    ELEVATION = 2
    TRANSLATION = 3


class RequestedState(IntEnum):
    DISARMED = 0
    ARMED = 1
    SAFE_STOP = 2
    ESTOP = 3


class ControlMode(IntEnum):
    COAST = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3
    HOLD = 4


class BoardState(IntEnum):
    DISARMED = 0
    ARMED = 1
    SAFE_STOPPING = 2
    HOLDING = 3
    FAULT = 4


class FaultCode(IntEnum):
    NONE = 0
    WATCHDOG = 1
    OVERCURRENT = 2
    VDS_OCP = 3
    OVERTEMP = 4
    ENCODER = 5
    DRV_FAULT = 6
    UNASSIGNED_AXIS = 7
    SAMPLER = 8


class Buttons(IntEnum):
    NONE = 0x0
    JOYSTICK = 0x1


# --------------------------------------------------------------------------
# Joystick to MIB. These are the messages the debugger sends to drive the board.
# --------------------------------------------------------------------------

@dataclass
class XYTwist(IdlStruct, typename="rammp/msg/XYTwist"):
    """Stick position. 16 bytes on the wire."""
    x: pt.float32 = 0.0
    y: pt.float32 = 0.0
    twist: pt.float32 = 0.0
    buttons: pt.uint32 = 0


@dataclass
class DriveCommand(IdlStruct, typename="rammp/msg/DriveCommand"):
    """Enable or disable driving, with the profile to drive with. 2 bytes."""
    request: pt.uint8 = 0
    profile: pt.uint8 = 1


@dataclass
class SeatCommand(IdlStruct, typename="rammp/msg/SeatCommand"):
    """Absolute seat axis target. 8 bytes, the float aligning to offset 4."""
    axis: pt.uint8 = 0
    target: pt.float32 = 0.0


# --------------------------------------------------------------------------
# MIB to joystick.
# --------------------------------------------------------------------------

@dataclass
class SeatState(IdlStruct, typename="rammp/msg/seatState"):
    front_back_tilt: pt.float32 = 0.0
    lateral_tilt: pt.float32 = 0.0
    elevation: pt.float32 = 0.0
    translation: pt.float32 = 0.0


@dataclass
class MibStatus(IdlStruct, typename="rammp/msg/MibStatus"):
    """Status published by the MIB twice per second."""
    systemState: pt.uint8 = 0
    activeProfile: pt.uint8 = 1
    currentSeatState: SeatState = field(default_factory=SeatState)
    error_message: str = "No error"
    epoch_s: pt.int64 = 0
    speed: pt.float32 = 0.0
    utc_offset_min: pt.int16 = 0
    seq: pt.uint8 = 0
    status_text: str = ""
    error_footer: str = ""


# --------------------------------------------------------------------------
# MIB to motor controller, and back. The debugger watches the commands and can
# stand in for a PACE RACER board by publishing state.
# --------------------------------------------------------------------------

@dataclass
class MotorCommand(IdlStruct, typename="rammp/msg/MotorCommand"):
    """One axis's command. 28 bytes on the wire."""
    seq: pt.uint8 = 0
    requested_state: pt.uint8 = 0
    mode: pt.uint8 = 0
    clear_fault_req_id: pt.uint8 = 0
    position: pt.float32 = 0.0
    velocity: pt.float32 = 0.0
    torque: pt.float32 = 0.0
    torque_limit: pt.float32 = 0.0
    vel_limit: pt.float32 = 0.0
    accel_limit: pt.float32 = 0.0


@dataclass
class MotorState(IdlStruct, typename="rammp/msg/MotorState"):
    """One axis's state. 80 bytes on the wire, padding 2 bytes before uptime_ms."""
    api_version: pt.uint8 = 1
    axis_id: pt.uint8 = 255
    seq: pt.uint8 = 0
    last_cmd_seq: pt.uint8 = 0
    state: pt.uint8 = 0
    mode: pt.uint8 = 0
    fault_code: pt.uint8 = 0
    clear_fault_ack: pt.uint8 = 0
    vbus_measured: pt.uint8 = 0
    flags: pt.uint8 = 0
    cmd_age_ms: pt.uint16 = 0
    drv_status: pt.uint16 = 0
    uptime_ms: pt.uint32 = 0
    fw_version: pt.uint32 = 0
    position: pt.float32 = 0.0
    velocity: pt.float32 = 0.0
    torque_est: pt.float32 = 0.0
    iq: pt.float32 = 0.0
    id: pt.float32 = 0.0
    vbus: pt.float32 = 0.0
    ibus_est: pt.float32 = 0.0
    torque_limit_eff: pt.float32 = 0.0
    vel_limit_eff: pt.float32 = 0.0
    accel_limit_eff: pt.float32 = 0.0
    temps: pt.array[pt.float32, 4] = field(default_factory=lambda: [0.0] * 4)


# --------------------------------------------------------------------------
# Topic names, matching the RAMMP_TOPIC_* macros in the C++ headers.
# --------------------------------------------------------------------------

TOPIC_JOYSTICK_XY_TWIST = "rammp/joystick/xy_twist"
TOPIC_JOYSTICK_DRIVE_COMMAND = "rammp/joystick/drive_command"
TOPIC_JOYSTICK_SEAT_COMMAND = "rammp/joystick/seat_command"
TOPIC_MIB_STATUS = "rammp/mib/status"

AXIS_SEGMENTS = {
    0: "front_caster_left",
    1: "front_caster_right",
    2: "drive_left",
    3: "drive_right",
}

AXIS_LABELS = {
    0: "Front caster L",
    1: "Front caster R",
    2: "Drive L",
    3: "Drive R",
}


def motor_command_topic(axis_id: int) -> str:
    """Topic the MIB publishes commands for one axis on."""
    return "rammp/mib/motor_command/" + AXIS_SEGMENTS[axis_id]


def motor_state_topic(axis_id: int) -> str:
    """Topic a PACE RACER board publishes its state on."""
    return "rammp/" + AXIS_SEGMENTS[axis_id] + "/motor_state"


# --------------------------------------------------------------------------
# Runtime parameters. Mirrors components/MIBConfig/include/mib_params.hpp and
# must be kept in step with its MIB_PARAM_TABLE, row for row.
# --------------------------------------------------------------------------

TOPIC_JOYSTICK_PARAM_SET = "rammp/joystick/param_set"
TOPIC_MIB_PARAMS = "rammp/mib/params"

# (id, name, label, unit, min, max, firmware default)
PARAM_TABLE = [
    (0,  "INVERT_LEFT",          "Invert left wheel",    "bool",    0.0,  1.0,  1.0),
    (1,  "INVERT_RIGHT",         "Invert right wheel",   "bool",    0.0,  1.0,  0.0),
    (2,  "WHEEL_DIAMETER_M",     "Wheel diameter",       "m",       0.01, 2.0,  0.254),
    (3,  "WHEEL_SEPARATION_M",   "Wheel separation",     "m",       0.01, 3.0,  0.558),
    (4,  "JOYSTICK_X_SIGN",      "Joystick X sign",      "+1/-1",  -1.0,  1.0,  1.0),
    (5,  "LOW_MAX_LINEAR_V",     "LOW max linear",       "m/s",     0.0,  5.0,  0.35),
    (6,  "LOW_MAX_ANGULAR_V",    "LOW max angular",      "rad/s",   0.0,  10.0, 1.0),
    (7,  "LOW_MAX_LINEAR_A",     "LOW linear accel",     "m/s^2",   0.01, 10.0, 0.5),
    (8,  "LOW_MAX_ANGULAR_A",    "LOW angular accel",    "rad/s^2", 0.01, 20.0, 1.0),
    (9,  "NORMAL_MAX_LINEAR_V",  "NORMAL max linear",    "m/s",     0.0,  5.0,  0.75),
    (10, "NORMAL_MAX_ANGULAR_V", "NORMAL max angular",   "rad/s",   0.0,  10.0, 2.0),
    (11, "NORMAL_MAX_LINEAR_A",  "NORMAL linear accel",  "m/s^2",   0.01, 10.0, 1.0),
    (12, "NORMAL_MAX_ANGULAR_A", "NORMAL angular accel", "rad/s^2", 0.01, 20.0, 2.0),
    (13, "HIGH_MAX_LINEAR_V",    "HIGH max linear",      "m/s",     0.0,  5.0,  1.0),
    (14, "HIGH_MAX_ANGULAR_V",   "HIGH max angular",     "rad/s",   0.0,  10.0, 3.14159),
    (15, "HIGH_MAX_LINEAR_A",    "HIGH linear accel",    "m/s^2",   0.01, 10.0, 2.0),
    (16, "HIGH_MAX_ANGULAR_A",   "HIGH angular accel",   "rad/s^2", 0.01, 20.0, 4.0),
]
PARAM_COUNT = len(PARAM_TABLE)
PARAM_BY_NAME = {row[1]: row for row in PARAM_TABLE}
PARAM_DEFAULTS = [row[6] for row in PARAM_TABLE]


@dataclass
class ParamSet(IdlStruct, typename="rammp/msg/ParamSet"):
    """Set one parameter. 8 bytes on the wire."""
    seq: pt.uint8 = 0
    id: pt.uint8 = 0
    value: pt.float32 = 0.0


@dataclass
class ParamState(IdlStruct, typename="rammp/msg/ParamState"):
    """The whole parameter table as the MIB holds it."""
    seq: pt.uint8 = 0
    last_set_seq: pt.uint8 = 0
    last_set_ok: pt.uint8 = 0
    count: pt.uint8 = PARAM_COUNT
    values: pt.array[pt.float32, PARAM_COUNT] = field(
        default_factory=lambda: list(PARAM_DEFAULTS))
