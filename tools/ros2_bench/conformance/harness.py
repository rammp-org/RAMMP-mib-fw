"""Shared plumbing for the base's arbitration conformance runner.

The same interface as kinova-gen3-ros2's test/conformance/harness.py, so the arm's
checks_arbitration.py runs against the MIB unchanged. Names are relative and the node
lives in the "base" namespace, so "acquire_control" resolves to /base/acquire_control.

Attaches to an ALREADY RUNNING MIB; it never flashes or resets one.
"""

import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rammp_base_interfaces.msg import TwistSetpoint
from rammp_common_interfaces.msg import ControlStatus
from rammp_common_interfaces.srv import AcquireControl, ReleaseControl, RevokeControl

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
ZERO_TOKEN = [0] * 16


def tok(t):
    """A token as a list of plain ints. rclpy returns uint8[16] as a numpy array, and
    the outgoing message assertion refuses numpy.uint8."""
    return [int(x) for x in t]


# What the MIB advertises. A subscriber that does not match gets silence, which looks
# exactly like a broken publisher.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)


class NotOnTheBase(Exception):
    """A surface the arm serves and the base deliberately does not. The runner reports
    the check as SKIP with this reason, never as a pass."""


@dataclass
class Result:
    name: str
    status: str
    detail: str = ""


@dataclass
class Registry:
    checks: List = field(default_factory=list)

    def add(
        self,
        section: str,
        name: str,
        needs_motion: bool = False,
        needs_mode: Optional[str] = None,
    ):
        def deco(fn: Callable):
            self.checks.append((section, name, fn, needs_motion, needs_mode))
            return fn

        return deco


REGISTRY = Registry()


class SerialLog:
    """The bench firmware's status line, read from its USB console. Optional: it is the
    only way to see an ADMITTED command, which by design produces nothing on the wire."""

    LINE = re.compile(
        r"owned=(\w+) owner='([^']*)' generation=(\d+) estopped=(\w+) admitted=(\d+) "
        r"rejected=(\d+) halts=(\d+) twist=\(([-\d.]+), ([-\d.]+)\)"
    )

    def __init__(self, port: str):
        import serial  # pyserial, only needed with --serial

        self._serial = serial.Serial(port, 115200, timeout=0.2)
        self._latest = None
        self._seq = 0
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            line = self._serial.readline().decode(errors="replace")
            m = self.LINE.search(line)
            if m:
                with self._lock:
                    self._seq += 1
                    self._latest = {
                        "owned": m[1] == "true",
                        "owner_id": m[2],
                        "generation": int(m[3]),
                        "estopped": m[4] == "true",
                        "admitted": int(m[5]),
                        "rejected": int(m[6]),
                        "halts": int(m[7]),
                        "linear_x": float(m[8]),
                        "angular_z": float(m[9]),
                    }

    def next(self, timeout=6.0):
        """The first status line printed AFTER this call: the firmware prints every 2 s."""
        with self._lock:
            start = self._seq
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._seq > start:
                    return dict(self._latest)
            time.sleep(0.05)
        return None


class Ctx:
    """One ROS node in the base namespace, plus the helpers every check needs."""

    def __init__(self, node: Node, serial_log: Optional[SerialLog] = None):
        self.n = node
        self.serial = serial_log
        self._latest = {}
        self._subs = {}
        self._spec = {}
        self._stamp = {}
        # Every endpoint is made once, here, as a real client (the orchestrator, an
        # operator tool) makes its own. espp fails to match some endpoints created after
        # another was deleted (tools/ros2_bench/README.md), so nothing is made per call.
        self.sub(ControlStatus, "control_status", LATCHED_QOS, latched=True)
        self.twist_pub = self.n.create_publisher(
            TwistSetpoint, "setpoint/twist", COMMAND_QOS
        )
        self._clients = {
            "acquire_control": self.n.create_client(AcquireControl, "acquire_control"),
            "release_control": self.n.create_client(ReleaseControl, "release_control"),
            "revoke_control": self.n.create_client(RevokeControl, "revoke_control"),
        }
        for name, cli in self._clients.items():
            if not cli.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"service {name} never appeared")
        self.spin(1.0)  # let the MIB match these clients too

    # ---- topics -------------------------------------------------------------

    def sub(self, msg_type, topic, qos, latched=False):
        self._latest.setdefault(topic, None)
        self._spec[topic] = (msg_type, qos, latched)
        if topic not in self._subs:
            self._subs[topic] = self.n.create_subscription(
                msg_type, topic, lambda m, t=topic: self._store(t, m), qos
            )

    def _store(self, topic, msg):
        self._latest[topic] = msg
        self._stamp[topic] = time.time()

    def spin(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def latest(self, topic, timeout=5.0, fresh=False):
        """Newest message on a topic, or None. fresh=True waits for one that arrives
        after this call: the MIB publishes its status on every change and resends it
        every second, so a fresh sample is at most a second away."""
        since = time.time() if fresh else 0.0
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)
            if self._latest.get(topic) is not None and self._stamp.get(topic, 0.0) > since:
                if fresh:
                    self.spin(0.05)  # take the newest of a burst
                return self._latest[topic]
        return self._latest.get(topic) if not fresh else None

    # ---- services -----------------------------------------------------------

    def call(self, srv_type, name, req, timeout=8.0):
        cli = self._clients[name]
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.n, timeout_sec=0.02)
        if not fut.done():
            raise RuntimeError(f"service {name} timed out")
        return fut.result()

    def acquire(self, owner="conformance", token=None):
        return self.call(
            AcquireControl,
            "acquire_control",
            AcquireControl.Request(
                owner_id=owner, token=tok(token if token is not None else ZERO_TOKEN)
            ),
        )

    def release(self, token):
        return self.call(
            ReleaseControl, "release_control", ReleaseControl.Request(token=tok(token))
        )

    def revoke(self, reason="conformance cleanup"):
        return self.call(
            RevokeControl, "revoke_control", RevokeControl.Request(reason=reason)
        )

    # ---- commands -----------------------------------------------------------

    def send_twist(self, token, linear_x=0.0, angular_z=0.0, count=20, period=0.05):
        m = TwistSetpoint()
        m.twist.linear.x = float(linear_x)
        m.twist.angular.z = float(angular_z)
        m.token = tok(token)
        for _ in range(count):
            self.twist_pub.publish(m)
            self.spin(period)

    # ---- e-stop -------------------------------------------------------------

    def estop(self, engaged: bool, source="conformance", reason="", stamp=None, settle=1.0):
        raise NotOnTheBase(
            "the base serves no /estop: its e-stop is estop-core's rammp/estop latch"
        )

    # ---- state --------------------------------------------------------------

    def control_status(self, fresh=True, timeout=5.0):
        return self.latest("control_status", timeout=timeout, fresh=fresh)

    def cleanup(self):
        """Leave the base as we found it: no owner."""
        try:
            self.revoke()
        except Exception:
            pass
