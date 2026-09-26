"""What the MCP266 exposes over CANopen, as a table the CAN page renders.

The Basicmicro MCP266 mirrors its packet-serial command set into the
manufacturer region of the CANopen object dictionary at index
0x2000 + command number. A command with one value lives at subindex 0; a
command with several values puts them at subindices 1..N in the command's own
field order. That order differs between a setter and its readback (the
position PID is written D, P, I but read back P, I, D), so every field here
carries its read address and its write address separately.

espp/mcp266 verified this mapping on real hardware for the position PID
(commands 61-64), main battery (24), temperature (82) and e-stop reset (200).
Everything else follows the same rule from the Basicmicro packet-serial
manual and is marked unverified; the page shows the SDO abort message when a
guess is wrong, which is how to find out.

Types are the wire sizes; `scale` turns the raw integer into the displayed
value (display = raw / scale) and back.
"""

import struct

# (wire size in bytes, signed)
TYPES = {
    "u8": (1, False), "u16": (2, False), "u32": (4, False),
    "i8": (1, True), "i16": (2, True), "i32": (4, True),
    "str": (0, False),
}

FIX16 = 65536.0   # velocity PID gains are 16.16 fixed point
X1024 = 1024.0    # position PID gains are scaled by 1024


def cmd(n):
    """Manufacturer object index of packet-serial command n."""
    return 0x2000 + n


def F(key, label, typ, read=None, write=None, unit="", scale=1.0, decode=None, note=""):
    return {"key": key, "label": label, "type": typ, "unit": unit, "scale": scale,
            "read": read, "write": write, "decode": decode, "note": note}


def G(key, title, kind, fields, verified=False, note=""):
    return {"key": key, "title": title, "kind": kind, "fields": fields,
            "verified": verified, "note": note}


def pid_velocity(axis, read_cmd, write_cmd):
    # readback: P, I, D, QPPS   setter: D, P, I, QPPS
    r, w = cmd(read_cmd), cmd(write_cmd)
    return G(f"velocity_pid_{axis}", f"Velocity PID {axis.upper()}", "setting", [
        F("p", "P", "u32", (r, 1), (w, 2), scale=FIX16),
        F("i", "I", "u32", (r, 2), (w, 3), scale=FIX16),
        F("d", "D", "u32", (r, 3), (w, 1), scale=FIX16),
        F("qpps", "QPPS (max speed)", "u32", (r, 4), (w, 4), unit="counts/s",
          note="Encoder counts per second at full power; the velocity loop's ceiling."),
    ], note=f"Read: command {read_cmd} at 0x{r:04X}. Write: command {write_cmd} at 0x{w:04X}, "
            "fields in the setter's D, P, I, QPPS order. Read object confirmed by espp; the "
            "write order follows the packet-serial manual.")


def pid_position(axis, read_cmd, write_cmd):
    # readback: P, I, D, MaxI, Deadzone, MinPos, MaxPos   setter: D, P, I, MaxI, Deadzone, MinPos, MaxPos
    r, w = cmd(read_cmd), cmd(write_cmd)
    return G(f"position_pid_{axis}", f"Position PID {axis.upper()}", "setting", [
        F("p", "P", "u32", (r, 1), (w, 2), scale=X1024),
        F("i", "I", "u32", (r, 2), (w, 3), scale=X1024),
        F("d", "D", "u32", (r, 3), (w, 1), scale=X1024),
        F("max_i", "Max I", "u32", (r, 4), (w, 4)),
        F("deadzone", "Deadzone", "u32", (r, 5), (w, 5), unit="counts"),
        F("min_pos", "Min position", "i32", (r, 6), (w, 6), unit="counts",
          note="Factory default is [0, 0], which clamps every target to zero."),
        F("max_pos", "Max position", "i32", (r, 7), (w, 7), unit="counts"),
    ], verified=True,
       note=f"Read: command {read_cmd} at 0x{r:04X}. Write: command {write_cmd} at 0x{w:04X}. "
            "Verified on hardware by espp/mcp266, including the D, P, I setter order.")


def cia402(axis, offset):
    return G(f"cia402_{axis}", f"CiA 402 drive {axis.upper()}", "telemetry", [
        F("statusword", "Statusword", "u16", (0x6041 + offset, 0), decode="statusword"),
        F("controlword", "Controlword", "u16", (0x6040 + offset, 0), (0x6040 + offset, 0)),
        F("mode", "Mode of operation", "i8", (0x6060 + offset, 0), (0x6060 + offset, 0),
          note="1 profile position, 3 profile velocity; the MCP266 does not echo it in 0x6061."),
        F("position", "Position actual", "i32", (0x6064 + offset, 0), unit="counts"),
        F("velocity", "Velocity actual", "i32", (0x606C + offset, 0), unit="counts/s"),
        F("profile_velocity", "Profile velocity", "u32", (0x6081 + offset, 0), (0x6081 + offset, 0),
          unit="counts/s", note="Cruise speed for profile-position moves."),
        F("profile_accel", "Profile acceleration", "u32", (0x6083 + offset, 0), (0x6083 + offset, 0),
          unit="counts/s^2"),
        F("sw_limit_min", "Software limit min", "i32", (0x607D + offset, 1), (0x607D + offset, 1), unit="counts"),
        F("sw_limit_max", "Software limit max", "i32", (0x607D + offset, 2), (0x607D + offset, 2), unit="counts"),
    ], verified=True,
       note=f"Standard CiA 402 objects; M2 mirrors M1 at +0x{0x800:X}. Used by espp/mcp266 for "
            "position moves. Writing the controlword drives the CiA 402 state machine directly; "
            "only do that with the motor free to move.")


GROUPS = [
    G("identity", "Device", "info", [
        F("device_type", "Device type (0x1000)", "u32", (0x1000, 0)),
        F("name", "Device name (0x1008)", "str", (0x1008, 0)),
        F("firmware", "Firmware version (cmd 21)", "str", (cmd(21), 0)),
        F("vendor", "Vendor id", "u32", (0x1018, 1)),
        F("product", "Product code", "u32", (0x1018, 2)),
        F("revision", "Revision", "u32", (0x1018, 3)),
        F("serial", "Serial number", "u32", (0x1018, 4)),
        F("error_register", "Error register (0x1001)", "u8", (0x1001, 0)),
    ], verified=True, note="Standard CiA 301 objects plus the mirrored firmware-version string."),

    G("power", "Power and temperature", "telemetry", [
        F("main_battery", "Main battery", "u16", (cmd(24), 0), unit="V", scale=10.0),
        F("logic_battery", "Logic battery", "u16", (cmd(25), 0), unit="V", scale=10.0),
        F("temperature", "Temperature", "u16", (cmd(82), 0), unit="°C", scale=10.0),
        F("temperature2", "Temperature 2", "u16", (cmd(83), 0), unit="°C", scale=10.0),
    ], verified=True, note="Commands 24 and 82 confirmed by espp; 25 and 83 follow the same rule."),

    G("motors", "Motor telemetry", "telemetry", [
        F("current_m1", "Current M1", "u16", (cmd(49), 1), unit="A", scale=100.0),
        F("current_m2", "Current M2", "u16", (cmd(49), 2), unit="A", scale=100.0),
        F("pwm_m1", "PWM M1", "i16", (cmd(48), 1), unit="%", scale=327.67),
        F("pwm_m2", "PWM M2", "i16", (cmd(48), 2), unit="%", scale=327.67),
        F("encoder_m1", "Encoder M1", "i32", (cmd(78), 1), unit="counts"),
        F("encoder_m2", "Encoder M2", "i32", (cmd(78), 2), unit="counts"),
        F("speed_m1", "Speed M1", "i32", (cmd(79), 1), unit="counts/s"),
        F("speed_m2", "Speed M2", "i32", (cmd(79), 2), unit="counts/s"),
        F("buffer_m1", "Command buffer M1", "u8", (cmd(47), 1)),
        F("buffer_m2", "Command buffer M2", "u8", (cmd(47), 2)),
        F("status", "Status flags", "u32", (cmd(90), 0), decode="status"),
    ], note="Commands 47, 48, 49, 78, 79 and 90 mirrored at 0x2000 + command with one subindex "
            "per reply field. Unverified."),

    cia402("m1", 0x000),
    cia402("m2", 0x800),

    pid_velocity("m1", 55, 28),
    pid_velocity("m2", 56, 29),
    pid_position("m1", 63, 61),
    pid_position("m2", 64, 62),

    G("battery_limits", "Battery voltage limits", "setting", [
        F("main_min", "Main battery min", "u16", (cmd(59), 1), (cmd(57), 1), unit="V", scale=10.0),
        F("main_max", "Main battery max", "u16", (cmd(59), 2), (cmd(57), 2), unit="V", scale=10.0),
        F("logic_min", "Logic battery min", "u16", (cmd(60), 1), (cmd(58), 1), unit="V", scale=10.0),
        F("logic_max", "Logic battery max", "u16", (cmd(60), 2), (cmd(58), 2), unit="V", scale=10.0),
    ], note="Commands 57-60. The controller cuts the motors outside these limits. Unverified."),

    G("max_current", "Current limits", "setting", [
        F("m1_max", "M1 max current", "u32", (cmd(135), 1), (cmd(133), 1), unit="A", scale=100.0),
        F("m1_min", "M1 min current", "u32", (cmd(135), 2), (cmd(133), 2), unit="A", scale=100.0),
        F("m2_max", "M2 max current", "u32", (cmd(136), 1), (cmd(134), 1), unit="A", scale=100.0),
        F("m2_min", "M2 min current", "u32", (cmd(136), 2), (cmd(134), 2), unit="A", scale=100.0),
    ], note="Commands 133-136 (set/get max current, in 10 mA units). Unverified."),

    G("duty_accel", "Default duty acceleration", "setting", [
        F("m1", "M1 duty accel", "u32", (cmd(81), 1), (cmd(68), 0)),
        F("m2", "M2 duty accel", "u32", (cmd(81), 2), (cmd(69), 0)),
    ], note="Commands 68/69 set one motor each; command 81 reads both. Unverified."),

    G("encoder_modes", "Encoder modes", "setting", [
        F("m1", "Encoder M1 mode", "u8", (cmd(91), 1), (cmd(92), 0), decode="encoder_mode"),
        F("m2", "Encoder M2 mode", "u8", (cmd(91), 2), (cmd(93), 0), decode="encoder_mode"),
    ], note="Bit 0: 0 quadrature, 1 absolute. Bit 7: RC/analog encoder support. Commands 91-93. Unverified."),

    G("pwm_mode", "PWM mode", "setting", [
        F("mode", "PWM mode", "u8", (cmd(149), 0), (cmd(148), 0),
          note="0 locked antiphase, 1 sign magnitude."),
    ], note="Commands 148/149. Unverified."),

    G("pin_functions", "S3 / S4 / S5 pin functions", "setting", [
        F("s3", "S3 mode", "u8", (cmd(75), 1), (cmd(74), 1)),
        F("s4", "S4 mode", "u8", (cmd(75), 2), (cmd(74), 2)),
        F("s5", "S5 mode", "u8", (cmd(75), 3), (cmd(74), 3)),
    ], note="Commands 74/75; mode values as listed in the Basicmicro manual. Unverified."),

    G("deadband", "RC / analog deadband", "setting", [
        F("reverse", "Reverse deadband", "u8", (cmd(77), 1), (cmd(76), 1), unit="0.1 %"),
        F("forward", "Forward deadband", "u8", (cmd(77), 2), (cmd(76), 2), unit="0.1 %"),
    ], note="Commands 76/77. Unverified."),

    G("set_encoders", "Set encoder counts", "setting", [
        F("m1", "Encoder M1", "i32", None, (cmd(22), 0), unit="counts"),
        F("m2", "Encoder M2", "i32", None, (cmd(23), 0), unit="counts"),
    ], note="Write-only, commands 22/23. Read the values back under Motor telemetry."),
]

# Actions: one write each. A value of None means the page asks for one.
ACTIONS = [
    {"key": "estop_reset", "label": "Reset E-stop", "index": cmd(200), "sub": 0, "type": "u8",
     "value": 1, "danger": False, "verified": True,
     "note": "Command 200. Verified by espp; harmless when nothing is latched."},
    {"key": "reset_encoders", "label": "Zero both encoders", "index": cmd(20), "sub": 0, "type": "u8",
     "value": 1, "danger": False, "verified": False, "note": "Command 20."},
    {"key": "write_eeprom", "label": "Save settings to EEPROM", "index": cmd(94), "sub": 0, "type": "u32",
     "value": 0xE22EAB7A, "danger": True, "verified": False,
     "note": "Command 94 (WRITENVM) with its magic word. Without this every write above is lost "
             "at power-off. Unverified over CAN: if it aborts, the MCP may need Motion Studio for "
             "persistence."},
    {"key": "restore_defaults", "label": "Restore factory defaults", "index": cmd(80), "sub": 0, "type": "u8",
     "value": 1, "danger": True, "verified": False,
     "note": "Command 80. Wipes every setting on the controller, including the ones Motion Studio "
             "set. Unverified."},
]

GROUP_BY_KEY = {g["key"]: g for g in GROUPS}
ACTION_BY_KEY = {a["key"]: a for a in ACTIONS}

# RoboClaw / MCP 32-bit status word, from the Basicmicro user manual.
STATUS_BITS = [
    (0x00000001, "E-stop"), (0x00000002, "Temperature error"), (0x00000004, "Temperature 2 error"),
    (0x00000008, "Main voltage high error"), (0x00000010, "Logic voltage high error"),
    (0x00000020, "Logic voltage low error"), (0x00000040, "M1 driver fault"),
    (0x00000080, "M2 driver fault"), (0x00000100, "M1 speed error"), (0x00000200, "M2 speed error"),
    (0x00000400, "M1 position error"), (0x00000800, "M2 position error"),
    (0x00001000, "M1 current error"), (0x00002000, "M2 current error"),
    (0x00010000, "M1 over-current warning"), (0x00020000, "M2 over-current warning"),
    (0x00040000, "Main voltage high warning"), (0x00080000, "Main voltage low warning"),
    (0x00100000, "Temperature warning"), (0x00200000, "Temperature 2 warning"),
    (0x00400000, "S4 signal triggered"), (0x00800000, "S5 signal triggered"),
    (0x01000000, "Speed error limit warning"), (0x02000000, "Position error limit warning"),
]

DS402_STATUS_BITS = [
    (1 << 0, "ready to switch on"), (1 << 1, "switched on"), (1 << 2, "operation enabled"),
    (1 << 3, "fault"), (1 << 4, "voltage enabled"), (1 << 5, "quick stop"),
    (1 << 6, "switch on disabled"), (1 << 7, "warning"), (1 << 10, "target reached"),
    (1 << 11, "internal limit active"), (1 << 12, "set-point ack"),
]


def ds402_state(statusword: int) -> str:
    low = statusword & 0x4F
    if low == 0x40:
        return "switch on disabled"
    if low == 0x0F:
        return "fault reaction active" if statusword & 0x0F == 0x0F else "fault"
    if low == 0x08:
        return "fault"
    m = statusword & 0x6F
    return {
        0x00: "not ready to switch on", 0x21: "ready to switch on", 0x23: "switched on",
        0x27: "operation enabled", 0x07: "quick stop active",
    }.get(m, "?")


def decode(field, raw):
    """Extra human reading for a raw integer, or None."""
    kind = field.get("decode")
    if kind == "status":
        names = [name for bit, name in STATUS_BITS if raw & bit]
        return ", ".join(names) if names else "normal"
    if kind == "statusword":
        names = [name for bit, name in DS402_STATUS_BITS if raw & bit]
        return f"{ds402_state(raw)}: " + (", ".join(names) if names else "no bits")
    if kind == "encoder_mode":
        return ("absolute" if raw & 1 else "quadrature") + (", RC/analog" if raw & 0x80 else "")
    return None


def to_display(field, raw_int):
    scale = field.get("scale", 1.0) or 1.0
    if scale == 1.0:
        return raw_int
    return raw_int / scale


def to_raw(field, value):
    scale = field.get("scale", 1.0) or 1.0
    raw = int(round(float(value) * scale))
    size, signed = TYPES[field["type"]]
    lo = -(1 << (8 * size - 1)) if signed else 0
    hi = (1 << (8 * size - 1)) - 1 if signed else (1 << (8 * size)) - 1
    if not lo <= raw <= hi:
        raise ValueError(f"{field['label']}: {value} is outside {field['type']} range")
    return raw


def read_field(client, field):
    """Read one field with the client; returns a dict for the page."""
    index, sub = field["read"]
    typ = field["type"]
    if typ == "str":
        text = client.read_string(index, sub)
        return {"raw": text, "value": text, "text": text}
    size, signed = TYPES[typ]
    raw = client.read_int(index, sub, size) if signed else client.read_uint(index, sub, size)
    value = to_display(field, raw)
    return {"raw": raw, "value": value, "text": decode(field, raw)}


def write_field(client, field, value):
    index, sub = field["write"]
    size, signed = TYPES[field["type"]]
    raw = to_raw(field, value)
    if signed:
        client.write_int(index, sub, raw, size)
    else:
        client.write_uint(index, sub, raw, size)
    return raw


def table_json():
    """The whole table, for /api/can_objects."""
    return {"groups": GROUPS, "actions": ACTIONS, "types": {k: v[0] for k, v in TYPES.items()}}
