"""DDS link to the RAMMP MIB board.

Owns one DDS participant on domain 0, publishes the three joystick-side topics
the MIB subscribes to, and subscribes to everything the MIB publishes. All
state is kept behind a lock and handed to the web layer as a plain snapshot
dictionary.

Two background threads do the periodic work. The receive thread polls every
reader; the transmit thread republishes the joystick at a fixed rate, because
the MIB acts on the most recent sample and expects a continuous stream while
driving.
"""

import os
import threading
import time
from collections import deque
from typing import Dict, List, Optional

import mib_messages as msg

# The MIB runs a DHCP server on its Ethernet port and lives at this address.
# The debugging host normally receives a lease on the same subnet.
DEFAULT_MIB_ADDRESS = "192.168.4.1"

JOYSTICK_PUBLISH_HZ = 50.0
MOTOR_STATE_PUBLISH_HZ = 20.0
RECEIVE_POLL_HZ = 100.0

# A MibStatus older than this is treated as a lost board. The firmware
# publishes twice per second, so this allows several missed samples.
STATUS_TIMEOUT_S = 2.0

# If the browser stops asking for state while the joystick is engaged, the
# operator has lost sight of the tool. Release the stick rather than leave a
# non-zero command going out to the chassis.
CLIENT_WATCHDOG_S = 1.5

DRIVE_AXES = (2, 3)  # DRIVE_LEFT, DRIVE_RIGHT

# Copied from components/MIBConfig/include/MIBconfig.hpp. The page uses these
# to turn the raw motor-shaft commands back into chassis motion: the left
# motor is mirrored, so the firmware negates it and a forward command reads as
# opposite signs at the two shafts.
GEOMETRY = {
    "wheel_diameter_m": 0.254,
    "wheel_separation_m": 0.558,
    "invert_left": True,
    "invert_right": False,
}
ALL_AXES = (0, 1, 2, 3)


def build_cyclonedds_uri(interface=None, local=False):
    """Return a CycloneDDS XML configuration, or None to use the defaults.

    Multicast discovery has to leave through the interface facing the board. On
    a laptop with Wi-Fi up as well, CycloneDDS may otherwise pick the wrong one
    and never see the MIB, so the interface is worth naming explicitly.

    `local` targets the loopback interface and adds 127.0.0.1 as a unicast
    discovery peer. That is what the bundled simulator needs, because multicast
    between two processes on one macOS host is not reliable on its own.
    """
    if not interface and not local:
        return None

    name = interface or "lo0"
    peers = ""
    if local:
        peers = "<Discovery><Peers><Peer address=\"127.0.0.1\"/></Peers></Discovery>"

    return (
        "<CycloneDDS><Domain>"
        "<General><Interfaces>"
        f'<NetworkInterface name="{name}" priority="default" multicast="true"/>'
        "</Interfaces><AllowMulticast>true</AllowMulticast></General>"
        + peers +
        "</Domain></CycloneDDS>"
    )



def valid_samples(reader, datatype, count=10):
    """Take samples from a reader, dropping the invalid ones.

    DataReader.take also yields InvalidSample entries, which carry only
    lifecycle notifications (a writer disposing or unregistering) and have none
    of the message's fields. Touching one raises AttributeError, so they are
    filtered out here rather than at every call site.
    """
    return [s for s in reader.take(N=count) if isinstance(s, datatype)]


def empty_snapshot() -> dict:
    """The snapshot shape with no link, for the disconnected page."""
    return {
        "connected": False,
        "status_age": None,
        "status_count": 0,
        "board": None,
        "motors": {str(a): None for a in ALL_AXES},
        "joystick": {"x": 0.0, "y": 0.0, "twist": 0.0, "live": False},
        "drive_profile": "NORMAL",
        "motor_sim": False,
        "axis_labels": {str(a): msg.AXIS_LABELS[a] for a in ALL_AXES},
        "geometry": GEOMETRY,
        "params": None,
        "events": [],
    }


class MibLink:
    """Everything the debugger needs to talk to the board."""

    def __init__(self, domain_id: int = 0, interface: Optional[str] = None,
                 local: bool = False):
        self.domain_id = domain_id
        self.interface = interface
        self.local = local

        self._lock = threading.RLock()
        self._running = False
        self._threads: List[threading.Thread] = []

        # Latest command we are sending.
        self._joystick = msg.XYTwist(0.0, 0.0, 0.0, 0)
        self._joystick_live = False
        self._drive_profile = int(msg.DriveProfile.NORMAL)

        # Latest data received from the board.
        self._status: Optional[msg.MibStatus] = None
        self._status_time: float = 0.0
        self._status_count = 0
        self._motor_commands: Dict[int, dict] = {}

        # Runtime parameter table as last reported by the board.
        self._params: Optional[msg.ParamState] = None
        self._params_time: float = 0.0
        self._param_set_seq = 0        # last seq we sent; 0 means none, so start at 1
        self._param_pending: Optional[dict] = None

        # Optional stand-in for the PACE RACER controllers.
        self._motor_sim = False
        self._motor_sim_seq = 0
        self._sim_position = {axis: 0.0 for axis in ALL_AXES}

        self._events = deque(maxlen=300)
        self._started_at = time.time()
        self._last_poll = time.time()

        self._participant = None
        self._writers = {}
        self._readers = {}

    # ---------------------------------------------------------------- logging

    def log(self, text: str, level: str = "info") -> None:
        with self._lock:
            self._events.appendleft(
                {"t": time.strftime("%H:%M:%S"), "level": level, "text": text}
            )

    # ---------------------------------------------------------------- startup

    def start(self) -> None:
        """Create the participant and every endpoint, then start the threads."""
        uri = build_cyclonedds_uri(self.interface, self.local)
        if uri:
            os.environ["CYCLONEDDS_URI"] = uri

        # Imported here so an import error surfaces with a clear message rather
        # than at module import time.
        from cyclonedds.core import Policy, Qos
        from cyclonedds.util import duration
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter, Publisher
        from cyclonedds.sub import DataReader, Subscriber
        from cyclonedds.topic import Topic

        # The MIB creates every endpoint best-effort with no durability, so the
        # debugger must match or the endpoints will not pair.
        qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8))
        # Parameter writes are the one thing that must not be dropped, and the
        # firmware's reader for them is reliable, so this writer must match.
        reliable = Qos(Policy.Reliability.Reliable(duration(seconds=1)),
                       Policy.History.KeepLast(8))

        self._participant = DomainParticipant(self.domain_id)
        publisher = Publisher(self._participant)
        subscriber = Subscriber(self._participant)

        def writer(topic_name, datatype, q=qos):
            topic = Topic(self._participant, topic_name, datatype, qos=q)
            return DataWriter(publisher, topic, qos=q)

        def reader(topic_name, datatype):
            topic = Topic(self._participant, topic_name, datatype, qos=qos)
            return DataReader(subscriber, topic, qos=qos)

        self._writers["joystick"] = writer(msg.TOPIC_JOYSTICK_XY_TWIST, msg.XYTwist)
        self._writers["drive"] = writer(msg.TOPIC_JOYSTICK_DRIVE_COMMAND, msg.DriveCommand)
        self._writers["seat"] = writer(msg.TOPIC_JOYSTICK_SEAT_COMMAND, msg.SeatCommand)

        self._readers["status"] = reader(msg.TOPIC_MIB_STATUS, msg.MibStatus)
        self._readers["params"] = reader(msg.TOPIC_MIB_PARAMS, msg.ParamState)
        self._writers["param_set"] = writer(msg.TOPIC_JOYSTICK_PARAM_SET, msg.ParamSet, reliable)

        for axis in ALL_AXES:
            self._readers[("motor_cmd", axis)] = reader(
                msg.motor_command_topic(axis), msg.MotorCommand
            )
            self._writers[("motor_state", axis)] = writer(
                msg.motor_state_topic(axis), msg.MotorState
            )

        self._running = True
        self._spawn(self._receive_loop, "mib-rx")
        self._spawn(self._transmit_loop, "mib-tx")

        self.log(
            f"DDS participant started on domain {self.domain_id}"
            + (" on loopback" if self.local
               else f" via {self.interface}" if self.interface else ""),
            "good",
        )

    def _spawn(self, target, name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    def stop(self) -> None:
        """Stop the threads and delete every DDS entity.

        CycloneDDS reads its configuration when the first participant on a
        domain is created and keeps it until the last one is gone, so the
        participant has to be released here for a later connect on a
        different interface to take effect.
        """
        self._running = False
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads = []
        self._readers.clear()
        self._writers.clear()
        self._participant = None
        import gc
        gc.collect()
        os.environ.pop("CYCLONEDDS_URI", None)

    # ------------------------------------------------------------- rx thread

    def _receive_loop(self) -> None:
        period = 1.0 / RECEIVE_POLL_HZ
        while self._running:
            try:
                self._drain_status()
                self._drain_motor_commands()
                self._drain_params()
            except Exception as exc:  # keep the thread alive through transients
                self.log(f"Receive error: {exc}", "bad")
            time.sleep(period)

    def _drain_status(self) -> None:
        samples = valid_samples(self._readers["status"], msg.MibStatus)
        for sample in samples:
            with self._lock:
                previous = self._status
                self._status = sample
                self._status_time = time.time()
                self._status_count += 1
                changed = previous is None or previous.systemState != sample.systemState
            if changed:
                name = msg.MibSystemState(sample.systemState).name
                self.log(f"Board state is now {name}", "good")

    def _drain_params(self) -> None:
        samples = valid_samples(self._readers["params"], msg.ParamState)
        if not samples:
            return
        latest = samples[-1]
        with self._lock:
            self._params = latest
            self._params_time = time.time()
            pending = self._param_pending
            acked = pending and latest.last_set_seq == pending["seq"]
            if acked:
                self._param_pending = None
        if acked:
            name = pending["name"]
            if latest.last_set_ok:
                self.log(f"Board applied {name} = {pending['value']:g}", "good")
            else:
                self.log(f"Board REJECTED {name} = {pending['value']:g}", "bad")

    def _drain_motor_commands(self) -> None:
        for axis in ALL_AXES:
            samples = valid_samples(self._readers[("motor_cmd", axis)], msg.MotorCommand)
            if not samples:
                continue
            latest = samples[-1]
            with self._lock:
                self._motor_commands[axis] = {
                    "seq": latest.seq,
                    "requested_state": msg.RequestedState(latest.requested_state).name,
                    "mode": msg.ControlMode(latest.mode).name,
                    "velocity": round(latest.velocity, 4),
                    "position": round(latest.position, 4),
                    "torque": round(latest.torque, 4),
                    "age": 0.0,
                    "time": time.time(),
                }

    # ------------------------------------------------------------- tx thread

    def _transmit_loop(self) -> None:
        joystick_period = 1.0 / JOYSTICK_PUBLISH_HZ
        motor_period = 1.0 / MOTOR_STATE_PUBLISH_HZ
        next_joystick = time.time()
        next_motor = time.time()

        while self._running:
            now = time.time()
            try:
                if now >= next_joystick:
                    next_joystick = now + joystick_period
                    self._publish_joystick()
                if now >= next_motor:
                    next_motor = now + motor_period
                    self._publish_motor_state()
            except Exception as exc:
                self.log(f"Transmit error: {exc}", "bad")
            time.sleep(0.002)

    def note_poll(self) -> None:
        """Record that the front end is still watching."""
        with self._lock:
            self._last_poll = time.time()

    def _check_watchdog(self) -> None:
        with self._lock:
            if not self._joystick_live:
                return
            stale = time.time() - self._last_poll
        if stale > CLIENT_WATCHDOG_S:
            self.set_joystick_live(False)
            self.log(
                f"Watchdog released the joystick after {stale:.1f}s without a client",
                "bad")

    def _publish_joystick(self) -> None:
        self._check_watchdog()
        with self._lock:
            live = self._joystick_live
            sample = self._joystick
        # When not live we still send zeros, so a released control is seen as a
        # stop rather than as a stale non-zero sample.
        payload = sample if live else msg.XYTwist(0.0, 0.0, 0.0, 0)
        self._writers["joystick"].write(payload)

    def _publish_motor_state(self) -> None:
        with self._lock:
            if not self._motor_sim:
                return
            self._motor_sim_seq = (self._motor_sim_seq + 1) & 0xFF
            seq = self._motor_sim_seq
            commands = dict(self._motor_commands)

        uptime_ms = int((time.time() - self._started_at) * 1000.0) & 0xFFFFFFFF
        for axis in ALL_AXES:
            command = commands.get(axis)
            velocity = command["velocity"] if command else 0.0
            armed = command and command["requested_state"] == "ARMED"
            # Integrate the commanded velocity so position looks plausible.
            self._sim_position[axis] += velocity / MOTOR_STATE_PUBLISH_HZ
            state = msg.MotorState(
                api_version=1,
                axis_id=axis,
                seq=seq,
                last_cmd_seq=command["seq"] if command else 0,
                state=int(msg.BoardState.ARMED if armed else msg.BoardState.DISARMED),
                mode=int(msg.ControlMode.VELOCITY if armed else msg.ControlMode.COAST),
                fault_code=int(msg.FaultCode.NONE),
                vbus_measured=1,
                uptime_ms=uptime_ms,
                fw_version=(1 << 24),
                position=self._sim_position[axis],
                velocity=velocity,
                vbus=48.0,
                temps=[30.0, 30.5, 31.0, 29.5],
            )
            self._writers[("motor_state", axis)].write(state)

    # ------------------------------------------------------------- commands

    def set_joystick(self, x: float, y: float, twist: float, button: bool = False) -> None:
        clamp = lambda v: max(-1.0, min(1.0, float(v)))
        with self._lock:
            self._joystick = msg.XYTwist(
                clamp(x), clamp(y), clamp(twist),
                int(msg.Buttons.JOYSTICK if button else msg.Buttons.NONE),
            )

    def set_joystick_live(self, live: bool) -> None:
        with self._lock:
            self._joystick_live = bool(live)
            # Engaging counts as client activity, so the watchdog measures
            # silence from this moment rather than from the last page poll.
            self._last_poll = time.time()
            if not live:
                self._joystick = msg.XYTwist(0.0, 0.0, 0.0, 0)
        self.log("Joystick output " + ("engaged" if live else "released"),
                 "warn" if live else "info")

    def send_drive_command(self, enable: bool, profile: Optional[int] = None) -> None:
        with self._lock:
            if profile is not None:
                self._drive_profile = int(profile)
            chosen = self._drive_profile
        command = msg.DriveCommand(
            request=int(msg.DriveRequest.ENABLE if enable else msg.DriveRequest.DISABLE),
            profile=chosen,
        )
        self._writers["drive"].write(command)
        self.log(
            ("Requested drive ENABLE" if enable else "Requested drive DISABLE")
            + f" with profile {msg.DriveProfile(chosen).name}",
            "warn" if enable else "info",
        )

    def send_seat_command(self, axis: int, target: float) -> None:
        self._writers["seat"].write(msg.SeatCommand(axis=int(axis), target=float(target)))
        self.log(
            f"Seat command {msg.SeatAxis(int(axis)).name} to {target}", "info"
        )

    def set_param(self, param_id: int, value: float) -> dict:
        """Send one parameter write. The board confirms it in its next ParamState."""
        row = msg.PARAM_TABLE[int(param_id)]
        with self._lock:
            self._param_set_seq = self._param_set_seq % 255 + 1
            seq = self._param_set_seq
            self._param_pending = {"seq": seq, "name": row[1], "value": float(value),
                                   "time": time.time()}
        self._writers["param_set"].write(msg.ParamSet(seq=seq, id=int(param_id), value=float(value)))
        self.log(f"Sent {row[1]} = {float(value):g} (seq {seq})", "info")
        return {"ok": True, "seq": seq}

    def set_motor_sim(self, enabled: bool) -> None:
        with self._lock:
            self._motor_sim = bool(enabled)
        self.log(
            "Motor controller simulation " + ("on" if enabled else "off"), "info"
        )

    def stop_everything(self) -> None:
        """Release the joystick and ask the board to disable driving."""
        self.set_joystick_live(False)
        try:
            self.send_drive_command(False)
        except Exception as exc:
            self.log(f"Could not send disable: {exc}", "bad")
        self.log("STOP: joystick released and drive disable sent", "bad")

    # ------------------------------------------------------------- snapshot

    def snapshot(self) -> dict:
        now = time.time()
        with self._lock:
            status = self._status
            status_age = now - self._status_time if self._status_time else None
            connected = status_age is not None and status_age < STATUS_TIMEOUT_S

            board = None
            if status is not None:
                board = {
                    "systemState": msg.MibSystemState(status.systemState).name,
                    "activeProfile": msg.DriveProfile(status.activeProfile).name,
                    "error_message": status.error_message,
                    "speed": round(status.speed, 3),
                    "seq": status.seq,
                    "seat": {
                        "front_back_tilt": round(status.currentSeatState.front_back_tilt, 2),
                        "lateral_tilt": round(status.currentSeatState.lateral_tilt, 2),
                        "elevation": round(status.currentSeatState.elevation, 2),
                        "translation": round(status.currentSeatState.translation, 2),
                    },
                }

            motors = {}
            for axis in ALL_AXES:
                entry = self._motor_commands.get(axis)
                if entry is None:
                    motors[str(axis)] = None
                else:
                    copy = dict(entry)
                    copy["age"] = round(now - copy.pop("time"), 2)
                    motors[str(axis)] = copy

            params = None
            if self._params is not None:
                params = {
                    "values": list(self._params.values),
                    "count": self._params.count,
                    "last_set_seq": self._params.last_set_seq,
                    "last_set_ok": bool(self._params.last_set_ok),
                    "age": round(now - self._params_time, 2),
                    "pending": self._param_pending,
                }

            return {
                "connected": connected,
                "params": params,
                "status_age": round(status_age, 2) if status_age is not None else None,
                "status_count": self._status_count,
                "board": board,
                "motors": motors,
                "joystick": {
                    "x": self._joystick.x,
                    "y": self._joystick.y,
                    "twist": self._joystick.twist,
                    "live": self._joystick_live,
                },
                "drive_profile": msg.DriveProfile(self._drive_profile).name,
                "motor_sim": self._motor_sim,
                "axis_labels": {str(a): msg.AXIS_LABELS[a] for a in ALL_AXES},
                "geometry": GEOMETRY,
                "events": list(self._events)[:80],
            }
