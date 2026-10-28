"""Position control of one MCP266 channel through the MIB's CAN bridge.

A port of `mib::Actuator` (components/Actuator on the
feature/2-actuator-control branch) and of the parts of espp::Mcp266 and
espp::Ds402Drive it stands on, so the debugger can jog a joint, move it to an
absolute count and install its travel limits on the controller without any
firmware on the MIB beyond the raw CAN bridge. The sequences are the same ones
the firmware will run, SDO for SDO, which is the point: what works here works
there.

Positions are encoder counts, always. The controller closes the position loop
and runs the motion profile; this module only tells it where to go and keeps
its clamp in step with the configured range.

Two objects on the controller limit travel, and both are written together:

- the *position clamp*, MinPos/MaxPos in the position PID record (Basicmicro
  commands 61-64). It ships as [0, 0], which forces every target to zero.
- the CiA 402 *software position limits*, object 0x607D.

The MCP266 reverts to its EEPROM at power-up, so the clamp is re-read before
every move and re-installed when the controller has reset.
"""

import json
import os
import threading
import time

import canopen_sdo
from mcp266_objects import cmd

# ---------------------------------------------------------------- CiA 402

OBJ_CONTROLWORD = 0x6040
OBJ_STATUSWORD = 0x6041
OBJ_MODES_OF_OPERATION = 0x6060
OBJ_POSITION_ACTUAL = 0x6064
OBJ_TARGET_POSITION = 0x607A
OBJ_SOFTWARE_POSITION_LIMIT = 0x607D
OBJ_PROFILE_VELOCITY = 0x6081
OBJ_PROFILE_ACCELERATION = 0x6083
OBJ_PROFILE_DECELERATION = 0x6084

MODE_PROFILE_POSITION = 1

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_DISABLE_VOLTAGE = 0x0000
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080
CW_BIT_NEW_SETPOINT = 0x0010
CW_BIT_CHANGE_SET_IMMEDIATELY = 0x0020

SW_BIT_TARGET_REACHED = 0x0400
SW_BIT_LIMIT_ACTIVE = 0x0800
SW_BIT_SETPOINT_ACKNOWLEDGE = 0x1000

# Drive states, named as espp::Ds402Drive::State names them.
NOT_READY = "not ready to switch on"
SWITCH_ON_DISABLED = "switch on disabled"
READY_TO_SWITCH_ON = "ready to switch on"
SWITCHED_ON = "switched on"
OPERATION_ENABLED = "operation enabled"
QUICK_STOP_ACTIVE = "quick stop active"
FAULT_REACTION_ACTIVE = "fault reaction active"
FAULT = "fault"
UNKNOWN = "unknown"


def decode_state(statusword: int) -> str:
    """CiA 402 state from a statusword, as espp's decode_state does it."""
    low = statusword & 0x4F
    if low == 0x00:
        return NOT_READY
    if low == 0x40:
        return SWITCH_ON_DISABLED
    if low == 0x08:
        return FAULT
    if low == 0x0F:
        return FAULT_REACTION_ACTIVE
    return {
        0x21: READY_TO_SWITCH_ON, 0x23: SWITCHED_ON, 0x27: OPERATION_ENABLED,
        0x07: QUICK_STOP_ACTIVE,
    }.get(statusword & 0x6F, UNKNOWN)


# Timing, matching espp::Mcp266::Config and Actuator.
STATE_TIMEOUT_S = 1.0     # per CiA 402 transition and set-point handshake
POLL_S = 0.025            # statusword polling period while waiting
MODE_SETTLE_S = 0.025     # the MCP does not echo the mode in 0x6061; wait instead

# The clamp installed while calibrating: wide enough for any travel, narrow
# enough that max - min still fits a 32-bit integer.
UNLIMITED = (-1_000_000_000, 1_000_000_000)
# Seeded into a position PID record whose P gain reads back as zero, so the
# loop produces output at all. Coarse; tune the real gain on the CAN page.
DEFAULT_POSITION_P = 15491
# A controller that does not answer is left alone this long before the next
# call tries it again, so one absent node does not stall every page poll.
OFFLINE_RETRY_S = 5.0

# Manufacturer objects per channel: the position PID setter and readback
# (61/62 and 63/64) and the encoder-count setter (22/23).
AXES = {
    "m1": {"offset": 0x000, "pid_set": cmd(61), "pid_get": cmd(63), "encoder_set": cmd(22)},
    "m2": {"offset": 0x800, "pid_set": cmd(62), "pid_get": cmd(64), "encoder_set": cmd(23)},
}
ESTOP_RESET_OBJECT = cmd(200)


class ActuatorError(Exception):
    """A refused or failed actuator operation; the text says why."""


class Mcp266Axis:
    """One channel of one MCP266 over a CanopenClient.

    Every method is one or more SDO round trips and raises SdoError on a
    transport failure or ActuatorError on a drive that will not do what it
    is told. Nothing here is locked: the caller serialises sequences.
    """

    def __init__(self, client: canopen_sdo.CanopenClient, axis: str):
        if axis not in AXES:
            raise ActuatorError(f"unknown axis {axis!r}; use m1 or m2")
        self.client = client
        self.axis = axis
        self.objects = AXES[axis]
        self.offset = self.objects["offset"]

    def obj(self, index: int) -> int:
        return index + self.offset

    # -- reads
    def statusword(self) -> int:
        return self.client.read_uint(self.obj(OBJ_STATUSWORD), 0, 2)

    def state(self) -> str:
        return decode_state(self.statusword())

    def position(self) -> int:
        return self.client.read_int(self.obj(OBJ_POSITION_ACTUAL), 0, 4)

    def read_pid_raw(self):
        """[P, I, D, MaxI, Deadzone, MinPos, MaxPos], the readback's order."""
        return [self.client.read_int(self.objects["pid_get"], sub, 4) for sub in range(1, 8)]

    def read_limits(self):
        """The MinPos/MaxPos clamp of the position PID record, two SDOs."""
        return (self.client.read_int(self.objects["pid_get"], 6, 4),
                self.client.read_int(self.objects["pid_get"], 7, 4))

    def read_sw_limits(self):
        obj = self.obj(OBJ_SOFTWARE_POSITION_LIMIT)
        return (self.client.read_int(obj, 1, 4), self.client.read_int(obj, 2, 4))

    # -- state machine
    def set_controlword(self, controlword: int) -> None:
        self.client.write_uint(self.obj(OBJ_CONTROLWORD), 0, controlword, 2)

    def wait_state(self, predicate, what: str) -> str:
        deadline = time.monotonic() + STATE_TIMEOUT_S
        while True:
            state = self.state()
            if predicate(state):
                return state
            if time.monotonic() >= deadline:
                raise ActuatorError(f"timed out waiting for {what} (drive is '{state}')")
            time.sleep(POLL_S)

    def command_state(self, controlword: int, wanted: str) -> None:
        self.set_controlword(controlword)
        self.wait_state(lambda s: s == wanted, f"'{wanted}'")

    def fault_reset(self) -> None:
        """Rising edge on controlword bit 7, then wait for the fault to clear."""
        self.set_controlword(0x0000)
        self.set_controlword(CW_FAULT_RESET)
        self.wait_state(lambda s: s not in (FAULT, FAULT_REACTION_ACTIVE), "fault cleared")

    def reset_faults_if_any(self) -> bool:
        if self.state() in (FAULT, FAULT_REACTION_ACTIVE):
            self.fault_reset()
            return True
        return False

    def enable_operation(self) -> None:
        """Shutdown -> Switch on -> Enable operation, as Ds402Drive does it."""
        state = self.state()
        if state in (FAULT, FAULT_REACTION_ACTIVE):
            raise ActuatorError("drive is in fault; clear faults first")
        if state not in (READY_TO_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED):
            self.command_state(CW_SHUTDOWN, READY_TO_SWITCH_ON)
            state = READY_TO_SWITCH_ON
        if state == READY_TO_SWITCH_ON:
            self.command_state(CW_SWITCH_ON, SWITCHED_ON)
            state = SWITCHED_ON
        if state == SWITCHED_ON:
            self.command_state(CW_ENABLE_OPERATION, OPERATION_ENABLED)

    def enable(self, mode: int = MODE_PROFILE_POSITION) -> None:
        """Mcp266::enable: write the mode blind, clear a fault, walk to enabled."""
        self.client.write_int(self.obj(OBJ_MODES_OF_OPERATION), 0, mode, 1)
        time.sleep(MODE_SETTLE_S)
        self.reset_faults_if_any()
        self.enable_operation()

    def quick_stop(self) -> None:
        self.set_controlword(CW_QUICK_STOP)

    def disable_voltage(self) -> None:
        self.command_state(CW_DISABLE_VOLTAGE, SWITCH_ON_DISABLED)

    def disable_loop(self) -> None:
        """Leave any state in which the position loop runs, so the encoder count
        can be rewritten without the loop chasing its old target."""
        if self.state() in (OPERATION_ENABLED, QUICK_STOP_ACTIVE):
            self.disable_voltage()

    # -- limits
    def write_pid_raw(self, setter) -> None:
        """[D, P, I, MaxI, Deadzone, MinPos, MaxPos], the setter's order."""
        for sub, value in enumerate(setter, start=1):
            self.client.write_int(self.objects["pid_set"], sub, value, 4)

    def configure_position_loop(self, min_pos: int, max_pos: int) -> bool:
        """Install the clamp through a read-modify-write of the whole record,
        seeding a P gain only if the stored one is zero. Returns whether P was
        seeded. Verifies the clamp by reading it back."""
        readback = self.read_pid_raw()
        seeded = False
        if readback[0] == 0:
            readback[0] = DEFAULT_POSITION_P
            seeded = True
        readback[5] = min_pos
        readback[6] = max_pos
        p, i, d, max_i, deadzone, lo, hi = readback
        self.write_pid_raw([d, p, i, max_i, deadzone, lo, hi])
        got = self.read_limits()
        if got != (min_pos, max_pos):
            raise ActuatorError(f"position clamp did not take: read [{got[0]}, {got[1]}], "
                                f"wanted [{min_pos}, {max_pos}]")
        return seeded

    def set_sw_limits(self, min_pos: int, max_pos: int) -> None:
        obj = self.obj(OBJ_SOFTWARE_POSITION_LIMIT)
        self.client.write_int(obj, 1, min_pos, 4)
        self.client.write_int(obj, 2, max_pos, 4)

    # -- motion
    def move_to_position(self, target: int, velocity: int, accel: int, decel: int) -> None:
        """Enable the axis, set the profile, issue the target with the CiA 402
        new-set-point handshake, and wait for the acknowledge to clear again."""
        self.enable(MODE_PROFILE_POSITION)
        self.client.write_uint(self.obj(OBJ_PROFILE_ACCELERATION), 0, accel, 4)
        self.client.write_uint(self.obj(OBJ_PROFILE_DECELERATION), 0, decel, 4)
        self.client.write_uint(self.obj(OBJ_PROFILE_VELOCITY), 0, velocity, 4)
        self.client.write_int(self.obj(OBJ_TARGET_POSITION), 0, target, 4)
        controlword = CW_ENABLE_OPERATION | CW_BIT_CHANGE_SET_IMMEDIATELY
        self.set_controlword(controlword | CW_BIT_NEW_SETPOINT)
        deadline = time.monotonic() + STATE_TIMEOUT_S
        while not self.statusword() & SW_BIT_SETPOINT_ACKNOWLEDGE:
            if time.monotonic() >= deadline:
                raise ActuatorError("no set-point acknowledge from the drive")
            time.sleep(POLL_S)
        self.set_controlword(controlword)
        deadline = time.monotonic() + STATE_TIMEOUT_S
        while self.statusword() & SW_BIT_SETPOINT_ACKNOWLEDGE:
            if time.monotonic() >= deadline:
                raise ActuatorError("drive did not release the set-point acknowledge")
            time.sleep(POLL_S)

    # -- manufacturer commands
    def set_encoder(self, count: int) -> None:
        self.client.write_int(self.objects["encoder_set"], 0, count, 4)

    def estop_reset(self) -> bool:
        """Command 200, device-wide. Width unknown, so try u8 then u32. False
        when the controller rejects it, which it does when nothing is latched."""
        for size in (1, 4):
            try:
                self.client.write_uint(ESTOP_RESET_OBJECT, 0, 1, size)
                return True
            except canopen_sdo.SdoAbort:
                continue
        return False


# ------------------------------------------------------------- configuration

CONFIG_DEFAULTS = {
    "name": "actuator",
    "node": 10,
    "axis": "m1",
    "min": 0,
    "max": 12600,
    "jog_step": 500,
    "velocity": 500,
    "accel": 500,
    "decel": 500,
    "tolerance": 10,
    "hardware_limits": True,
}

DEFAULT_CONFIGS = [
    {"name": "Leg 1", "node": 10, "axis": "m1"},
    {"name": "Leg 2", "node": 10, "axis": "m2"},
    {"name": "Leg 3", "node": 11, "axis": "m1"},
    {"name": "Leg 4", "node": 11, "axis": "m2"},
]


def normalise_config(raw: dict) -> dict:
    cfg = dict(CONFIG_DEFAULTS)
    for key in CONFIG_DEFAULTS:
        if key in raw and raw[key] is not None:
            cfg[key] = raw[key]
    cfg["name"] = str(cfg["name"]).strip() or "actuator"
    cfg["node"] = int(cfg["node"])
    if not 1 <= cfg["node"] <= 127:
        raise ValueError(f"{cfg['name']}: node id must be 1..127")
    cfg["axis"] = str(cfg["axis"]).lower()
    if cfg["axis"] not in AXES:
        raise ValueError(f"{cfg['name']}: axis must be m1 or m2")
    for key in ("min", "max", "jog_step", "velocity", "accel", "decel", "tolerance"):
        cfg[key] = int(cfg[key])
    if cfg["min"] > cfg["max"]:
        raise ValueError(f"{cfg['name']}: min {cfg['min']} is above max {cfg['max']}")
    if cfg["velocity"] <= 0:
        raise ValueError(f"{cfg['name']}: velocity must be positive")
    if cfg["jog_step"] <= 0:
        raise ValueError(f"{cfg['name']}: jog step must be positive")
    cfg["hardware_limits"] = bool(cfg["hardware_limits"])
    return cfg


def load_configs(path: str):
    """The actuator list from `path`, or the defaults when there is none."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except (OSError, ValueError):
        raw = DEFAULT_CONFIGS
    configs = []
    for entry in raw:
        try:
            configs.append(normalise_config(entry))
        except (ValueError, TypeError):
            continue
    return configs


def save_configs(path: str, configs) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(configs, handle, indent=2)
        handle.write("\n")
    os.replace(tmp, path)


# ------------------------------------------------------------------ actuator

class Actuator:
    """One positioned axis, with the checks mib::Actuator makes before a move.

    `link` provides client_for(node) -> CanopenClient, node_lock(node) ->
    a lock serialising multi-SDO sequences on that controller, and
    log(text, level).
    """

    def __init__(self, link, config: dict):
        self.link = link
        self.config = config
        self._lock = threading.RLock()
        self.calibrating = False
        self.initialized = False
        self.last_target = None
        self.last = None            # last status read, for the page and the snapshot
        self.last_limits = None     # last clamp / software limits read
        self.last_error = None
        self.offline_until = 0.0
        self.touched = False        # answered at least once; STOP only bothers these

    # -- plumbing
    @property
    def name(self) -> str:
        return self.config["name"]

    def _axis(self) -> Mcp266Axis:
        return Mcp266Axis(self.link.client_for(self.config["node"]), self.config["axis"])

    def _log(self, text: str, level: str = "info") -> None:
        self.link.log(f"{self.name}: {text}", level)

    def online(self) -> bool:
        return time.monotonic() >= self.offline_until

    def _run(self, what: str, fn, force: bool = False):
        """Run one sequence under the controller's lock, turning transport
        failures into ActuatorError and marking a silent controller offline."""
        if not force and not self.online():
            raise ActuatorError(f"{self.name}: controller offline (no answer), retrying in "
                                f"{self.offline_until - time.monotonic():.0f} s")
        try:
            with self.link.node_lock(self.config["node"]):
                result = fn()
        except canopen_sdo.SdoAbort as exc:
            self.last_error = f"{what}: {exc}"
            raise ActuatorError(f"{self.name}: {what} failed: {exc}") from exc
        except canopen_sdo.SdoError as exc:
            # A timeout with no abort is a silent controller.
            self.offline_until = time.monotonic() + OFFLINE_RETRY_S
            self.last_error = f"{what}: no answer"
            raise ActuatorError(f"{self.name}: {what} got no answer from node "
                                f"{self.config['node']}") from exc
        except ActuatorError as exc:
            self.last_error = str(exc)
            raise ActuatorError(f"{self.name}: {what} failed: {exc}") from exc
        self.touched = True
        self.last_error = None
        return result

    def installed_limits(self):
        return UNLIMITED if self.calibrating else (self.config["min"], self.config["max"])

    def clamp(self, counts: int) -> int:
        lo, hi = self.installed_limits()
        return max(lo, min(hi, int(counts)))

    # -- reads
    def status(self, force: bool = False) -> dict:
        """Position and statusword, two SDOs. Cached in `last`."""
        with self._lock:
            def read():
                axis = self._axis()
                return axis.position(), axis.statusword()
            position, statusword = self._run("status", read, force)
            self.last = {
                "position": position, "statusword": statusword,
                "state": decode_state(statusword),
                "reached": bool(statusword & SW_BIT_TARGET_REACHED),
                "limit": bool(statusword & SW_BIT_LIMIT_ACTIVE),
                "ack": bool(statusword & SW_BIT_SETPOINT_ACKNOWLEDGE),
                "t": time.time(),
            }
            return self.last

    def limits(self) -> dict:
        """The clamp and the software limits the controller holds, four SDOs."""
        with self._lock:
            def read():
                axis = self._axis()
                return axis.read_limits(), axis.read_sw_limits()
            clamp, sw = self._run("read limits", read, force=True)
            expected = self.installed_limits()
            self.last_limits = {
                "clamp": list(clamp), "sw": list(sw), "expected": list(expected),
                "matches": tuple(clamp) == tuple(expected), "t": time.time(),
            }
            return self.last_limits

    # -- limits
    def _apply_limits(self, lo: int, hi: int) -> None:
        axis = self._axis()
        if axis.configure_position_loop(lo, hi):
            self._log(f"position P gain was 0 on the controller; seeded {DEFAULT_POSITION_P} "
                      "(coarse: tune it on the CAN page)", "warn")
        axis.set_sw_limits(lo, hi)

    def initialize(self) -> dict:
        """Clear latched faults and install the range as the controller's clamp
        and software limits. Once per controller power-up."""
        with self._lock:
            lo, hi = self.installed_limits()

            def run():
                axis = self._axis()
                cleared = axis.reset_faults_if_any()
                estop = axis.estop_reset()
                self._apply_limits(lo, hi)
                return cleared, estop
            cleared, estop = self._run("initialize", run, force=True)
            self.initialized = True
            self._log(f"initialised: clamp and 0x607D set to [{lo}, {hi}]"
                      + (", CiA 402 fault cleared" if cleared else "")
                      + (", E-stop reset accepted" if estop else "")
                      + (" (clamp LIFTED, calibrating)" if self.calibrating else ""), "good")
            return {"ok": True, "limits": self.limits()}

    def verify_limits(self) -> None:
        """Read the clamp back; a controller that has reset holds its factory
        [0, 0] and would force every target to zero. Re-initialise it."""
        expected = self.installed_limits()
        got = self._run("verify limits", lambda: self._axis().read_limits())
        if tuple(got) == tuple(expected):
            return
        self._log(f"controller clamp is [{got[0]}, {got[1]}], expected [{expected[0]}, "
                  f"{expected[1]}]: it has reset; re-initialising", "warn")
        self.initialized = False
        self.initialize()

    def set_range(self, lo: int, hi: int) -> dict:
        """Install a calibrated range: in the config and, unless calibrating,
        on the controller."""
        lo, hi = int(lo), int(hi)
        if lo > hi:
            raise ActuatorError(f"{self.name}: rejected range [{lo}, {hi}], min is above max")
        with self._lock:
            self.config["min"], self.config["max"] = lo, hi
            self.link.actuators_config_changed()
            if self.calibrating:
                self._log(f"range set to [{lo}, {hi}]; applied to the controller when the "
                          "clamp is restored", "info")
                return {"ok": True, "applied": False}
            self._run("set range", lambda: self._apply_limits(lo, hi), force=True)
            self.initialized = True
            self._log(f"range [{lo}, {hi}] written to the controller's clamp and 0x607D", "good")
            return {"ok": True, "applied": True, "limits": self.limits()}

    def set_calibration_mode(self, enabled: bool) -> dict:
        """Lift the clamp so the joint can be driven to its mechanical ends, or
        restore it. Only jogs are accepted while lifted."""
        enabled = bool(enabled)
        with self._lock:
            if self.calibrating == enabled:
                return {"ok": True}
            self.calibrating = enabled
            lo, hi = self.installed_limits()
            self._run("calibration mode", lambda: self._apply_limits(lo, hi), force=True)
            self.initialized = True
            self._log("calibration mode ON: clamp lifted, mind the mechanical ends; "
                      "only jogs are accepted" if enabled
                      else "calibration mode off: range clamp restored", "warn")
            return {"ok": True, "limits": self.limits()}

    # -- moves
    def _release_quick_stop(self, axis: Mcp266Axis) -> None:
        if axis.state() != QUICK_STOP_ACTIVE:
            return
        try:
            axis.command_state(CW_ENABLE_OPERATION, OPERATION_ENABLED)
            self._log("quick stop released, axis re-enabled", "info")
            return
        except ActuatorError:
            pass
        axis.command_state(CW_DISABLE_VOLTAGE, SWITCH_ON_DISABLED)
        self._log("quick stop released, axis disabled until the next move", "info")

    def _do_move(self, counts: int, jog: bool) -> dict:
        if self.calibrating and not jog:
            raise ActuatorError(f"{self.name}: move to {counts} refused, only jogs are allowed "
                                "while the clamp is lifted (install a range first)")
        if not self.initialized:
            self.initialize()
        self.verify_limits()
        clamped = self.clamp(counts)
        if clamped != counts:
            self._log(f"target {counts} clamped to {clamped}", "warn")
        cfg = self.config

        def run():
            axis = self._axis()
            self._release_quick_stop(axis)
            # A limit switch that stopped the previous move leaves the loop wound
            # up against an unreachable target; a new move the same way would
            # only push into the switch again. Refuse it, and restart the drive
            # before a move the other way.
            if axis.statusword() & SW_BIT_LIMIT_ACTIVE:
                here = axis.position()
                if self.last_target is not None and clamped != here:
                    if (self.last_target > here) == (clamped > here):
                        raise ActuatorError(f"move to {clamped} refused, a limit switch is "
                                            "active in that direction")
                self._log("limit switch active; restarting the drive before moving away", "warn")
                axis.disable_loop()
            axis.move_to_position(clamped, cfg["velocity"], cfg["accel"], cfg["decel"])
        self._run("move", run)
        self.last_target = clamped
        self._log(f"{'jog' if jog else 'move'} to {clamped} at {cfg['velocity']} counts/s", "good")
        return {"ok": True, "target": clamped}

    def move_absolute(self, counts: int) -> dict:
        with self._lock:
            return self._do_move(int(counts), jog=False)

    def jog(self, direction: int) -> dict:
        direction = 1 if int(direction) >= 0 else -1
        with self._lock:
            here = self.status()["position"]
            return self._do_move(here + direction * self.config["jog_step"], jog=True)

    def stop(self) -> dict:
        with self._lock:
            self._run("stop", lambda: self._axis().quick_stop())
            self._log("quick stop", "warn")
            return {"ok": True}

    def disable(self) -> dict:
        with self._lock:
            self._run("disable", lambda: self._axis().disable_voltage())
            self._log("voltage disabled; the axis coasts until the next move", "warn")
            return {"ok": True}

    def clear_faults(self) -> dict:
        with self._lock:
            def run():
                axis = self._axis()
                cleared = axis.reset_faults_if_any()
                estop = axis.estop_reset()
                self._release_quick_stop(axis)
                return cleared, estop
            cleared, estop = self._run("clear faults", run, force=True)
            self._log("faults cleared" + (" (CiA 402 fault reset)" if cleared else "")
                      + (" (E-stop reset accepted)" if estop else ""), "good")
            return {"ok": True}

    # -- encoder
    def set_encoder(self, counts: int) -> dict:
        """Install `counts` as the current encoder reading, with the loop down
        first so it does not chase its old target."""
        counts = int(counts)
        with self._lock:
            def run():
                axis = self._axis()
                axis.disable_loop()
                axis.set_encoder(counts)
                return axis.position()
            readback = self._run("set encoder", run, force=True)
            if abs(readback - counts) > self.config["tolerance"]:
                raise ActuatorError(f"{self.name}: set encoder to {counts} but it reads {readback}")
            self.last_target = None
            self._log(f"encoder count set to {counts}", "good")
            return {"ok": True, "position": readback}

    def save_eeprom(self) -> dict:
        """WRITENVM on this actuator's controller (both channels; the MCP266
        reverts to EEPROM at power-up without it)."""
        from mcp266_objects import ACTION_BY_KEY, TYPES
        action = ACTION_BY_KEY["write_eeprom"]
        size = TYPES[action["type"]][0]
        with self._lock:
            self._run("save to EEPROM", lambda: self.link.client_for(self.config["node"]).write_uint(
                action["index"], action["sub"], action["value"], size), force=True)
            self._log(f"settings saved to EEPROM on node {self.config['node']}", "warn")
            return {"ok": True}

    # -- snapshot
    def to_json(self) -> dict:
        return {
            "config": dict(self.config),
            "calibrating": self.calibrating,
            "initialized": self.initialized,
            "online": self.online(),
            "last_target": self.last_target,
            "last": self.last,
            "limits": self.last_limits,
            "error": self.last_error,
        }
