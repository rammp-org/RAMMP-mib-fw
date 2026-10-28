"""DDS type definitions for the RAMMP MIB, mirroring external/rammp-rtps.

Every struct here IS the wire layout: classic little-endian CDR (XCDR1) with
fields in declaration order, matching the C++ headers in
external/rammp-rtps/components/rammp_rtps_messages/include/messages/. The
field types are the `t.*` wire types of rtps_io, which encodes and decodes
them; `rtps_io.encode(obj)` is what goes on the wire.

The C++ side declares its enums as `enum class X : uint8_t`, so they occupy one
byte on the wire, and every such field is declared here as `t.u8` with the
readable names in the IntEnum classes alongside.

Topic and type names must match the firmware exactly. embeddedRTPS compares
both with strcmp in TopicData::matchesTopicOf, so a single differing character
means the endpoints never pair up and no data flows.
"""

from dataclasses import field
from enum import IntEnum

from rtps_io import message, t


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
    HALL = 9
    STALL = 10


class Buttons(IntEnum):
    NONE = 0x0
    JOYSTICK = 0x1


# --------------------------------------------------------------------------
# Joystick to MIB. These are the messages the debugger sends to drive the board.
# --------------------------------------------------------------------------

@message("rammp/msg/XYTwist")
class XYTwist:
    """Stick position. 16 bytes on the wire."""
    x: t.f32 = 0.0
    y: t.f32 = 0.0
    twist: t.f32 = 0.0
    buttons: t.u32 = 0


@message("rammp/msg/DriveCommand")
class DriveCommand:
    """Enable or disable driving, with the profile to drive with. 2 bytes."""
    request: t.u8 = 0
    profile: t.u8 = 1


@message("rammp/msg/SeatCommand")
class SeatCommand:
    """Absolute seat axis target. 8 bytes, the float aligning to offset 4."""
    axis: t.u8 = 0
    target: t.f32 = 0.0


# --------------------------------------------------------------------------
# MIB to joystick.
# --------------------------------------------------------------------------

@message("rammp/msg/seatState")
class SeatState:
    front_back_tilt: t.f32 = 0.0
    lateral_tilt: t.f32 = 0.0
    elevation: t.f32 = 0.0
    translation: t.f32 = 0.0


@message("rammp/msg/MibStatus")
class MibStatus:
    """Status published by the MIB twice per second."""
    systemState: t.u8 = 0
    activeProfile: t.u8 = 1
    currentSeatState: SeatState = field(default_factory=SeatState)
    error_message: t.string = "No error"
    epoch_s: t.i64 = 0
    speed: t.f32 = 0.0
    utc_offset_min: t.i16 = 0
    seq: t.u8 = 0
    status_text: t.string = ""
    error_footer: t.string = ""


# --------------------------------------------------------------------------
# MIB to motor controller, and back. The debugger watches the commands, reads
# the boards' state, writes their gains, and can stand in for a PACE RACER
# board by publishing state.
# --------------------------------------------------------------------------

# RAMMP_MOTOR_API_VERSION from motor_message.hpp. A board reports the version
# it was built with; tuning needs 2, a negative gain (exactly 0) needs 3.
MOTOR_API_VERSION = 3

# The speed-loop defaults every PACE RACER profile ships with on halls
# (RACER-MOTOR-API.md, tuned on the NineBot 2026-09-28). SI on the wire.
TUNING_DEFAULTS = {"speed_kp": 2.39, "speed_ki": 4.77, "hall_pll_hz": 10.0}
# The board's own console prints the gains in A/rpm; this converts SI to that.
SI_TO_PER_RPM = 2.0 * 3.14159265358979323846 / 60.0

@message("rammp/msg/MotorCommand")
class MotorCommand:
    """One axis's command. 28 bytes on the wire."""
    seq: t.u8 = 0
    requested_state: t.u8 = 0
    mode: t.u8 = 0
    clear_fault_req_id: t.u8 = 0
    position: t.f32 = 0.0
    velocity: t.f32 = 0.0
    torque: t.f32 = 0.0
    torque_limit: t.f32 = 0.0
    vel_limit: t.f32 = 0.0
    accel_limit: t.f32 = 0.0


@message("rammp/msg/MotorState")
class MotorState:
    """One axis's state, motor API version 3. 92 bytes on the wire, padding 2
    bytes before uptime_ms. Version 2 added tuning_ack and the three gain
    fields; version 3 kept the layout."""
    api_version: t.u8 = MOTOR_API_VERSION
    axis_id: t.u8 = 255
    seq: t.u8 = 0
    last_cmd_seq: t.u8 = 0
    state: t.u8 = 0
    mode: t.u8 = 0
    fault_code: t.u8 = 0
    clear_fault_ack: t.u8 = 0
    vbus_measured: t.u8 = 0
    tuning_ack: t.u8 = 0
    cmd_age_ms: t.u16 = 0
    drv_status: t.u16 = 0
    uptime_ms: t.u32 = 0
    fw_version: t.u32 = 0
    position: t.f32 = 0.0
    velocity: t.f32 = 0.0
    torque_est: t.f32 = 0.0
    iq: t.f32 = 0.0
    id: t.f32 = 0.0
    vbus: t.f32 = 0.0
    ibus_est: t.f32 = 0.0
    torque_limit_eff: t.f32 = 0.0
    vel_limit_eff: t.f32 = 0.0
    accel_limit_eff: t.f32 = 0.0
    temps: t.array(t.f32, 4) = field(default_factory=lambda: [0.0] * 4)
    speed_kp: t.f32 = 0.0
    speed_ki: t.f32 = 0.0
    hall_pll_hz: t.f32 = 0.0


@message("rammp/msg/MotorTuning")
class MotorTuning:
    """Outer-loop gains for one axis, applied live by the board. 16 bytes.

    The board acts on a NEW tuning_req_id only and echoes it as
    MotorState.tuning_ack. A field of 0 means "leave as is"; a negative
    speed_kp or speed_ki sets that gain to exactly 0 (API version 3). With
    persist set the board also writes the values to NVS.
    """
    tuning_req_id: t.u8 = 0
    persist: t.u8 = 0
    reserved0: t.u8 = 0
    reserved1: t.u8 = 0
    speed_kp: t.f32 = 0.0
    speed_ki: t.f32 = 0.0
    hall_pll_hz: t.f32 = 0.0


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


def motor_tuning_topic(axis_id: int) -> str:
    """Topic a PACE RACER board takes its speed-loop gains on. The header
    names it MIB -> board, but any participant on the domain can write it,
    which is how the debugger tunes without a change to the MIB."""
    return "rammp/mib/motor_tuning/" + AXIS_SEGMENTS[axis_id]


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


@message("rammp/msg/ParamSet")
class ParamSet:
    """Set one parameter. 8 bytes on the wire."""
    seq: t.u8 = 0
    id: t.u8 = 0
    value: t.f32 = 0.0


@message("rammp/msg/ParamState")
class ParamState:
    """The whole parameter table as the MIB holds it."""
    seq: t.u8 = 0
    last_set_seq: t.u8 = 0
    last_set_ok: t.u8 = 0
    count: t.u8 = PARAM_COUNT
    values: t.array(t.f32, PARAM_COUNT) = field(
        default_factory=lambda: list(PARAM_DEFAULTS))


# --------------------------------------------------------------------------
# CAN bridge. Mirrors components/MIBConfig/include/mib_can_bridge.hpp. The MIB
# forwards frames untouched in both directions; the CANopen layer is in
# canopen_sdo.py and the MCP266 object dictionary in mcp266_objects.py.
# --------------------------------------------------------------------------

TOPIC_JOYSTICK_CAN_TX = "rammp/joystick/can_tx"
TOPIC_MIB_CAN_RX = "rammp/mib/can_rx"
TOPIC_MIB_CAN_STATUS = "rammp/mib/can_status"

CAN_FLAG_EXTENDED = 0x01
CAN_FLAG_RTR = 0x02

CAN_BUS_STATES = {0: "active", 1: "warning", 2: "passive", 3: "bus-off"}


@message("rammp/msg/CanFrame")
class CanFrame:
    """One classic CAN frame. 16 bytes on the wire."""
    seq: t.u8 = 0
    flags: t.u8 = 0
    dlc: t.u8 = 0
    reserved: t.u8 = 0
    id: t.u32 = 0
    data: t.array(t.u8, 8) = field(default_factory=lambda: [0] * 8)


@message("rammp/msg/CanStatus")
class CanStatus:
    """Bridge health, published with the other periodic status. 36 bytes."""
    initialized: t.u8 = 0
    enabled: t.u8 = 0
    bus_state: t.u8 = 0
    reserved: t.u8 = 0
    bitrate: t.u32 = 0
    tx_ok: t.u32 = 0
    tx_failed: t.u32 = 0
    rx_frames: t.u32 = 0
    rx_dropped: t.u32 = 0
    bus_errors: t.u32 = 0
    tx_error_count: t.u16 = 0
    rx_error_count: t.u16 = 0
    tx_gpio: t.i16 = -1
    rx_gpio: t.i16 = -1
