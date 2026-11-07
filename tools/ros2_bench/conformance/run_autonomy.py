#!/usr/bin/env python3
"""Drive the chair through an autonomy session over ROS 2, as the Jetson will, against
a RUNNING MIB with the main firmware. THE CHAIR MOVES: a few nudges of --speed m/s
(default 0.1) for --nudge seconds (default 1.5) each way. Lift the wheels or clear the
floor around it.

    pixi run autonomy                 # from tools/ros2_bench, MIB in IDLE
    pixi run autonomy --manual        # also the stick touch and the e-stop, prompted
    pixi run autonomy --speed 0.05 --nudge 1.0

What it checks, in order:

    an untokened DriveCommand AUTONOMOUS switches the state
    with no owner, twists are refused and the chair does not move
    acquire_control grants
    the owner's twists drive (MibStatus.speed rises), untokened twists do not
    untokened twists are refused and the chair stops within the lapse
    release_control returns the chair to IDLE
    a grant alone does not drive: tokened twists in IDLE do nothing
    revoke_control in AUTONOMOUS returns the chair to IDLE
    a second acquire seizes: the first token stops working, the state stays, the new
    owner drives
    --manual: a joystick touch ends AUTONOMOUS and revokes the grant
    --manual: the e-stop latches the Arbiter, refuses acquire, and clears to no owner

Exit code is non-zero if anything failed. The chair is left in IDLE with no owner.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rammp_base_interfaces.msg import DriveCommand, MibStatus
from harness import FAIL, PASS, SKIP, ZERO_TOKEN, Ctx, Result

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
MARK = {PASS: f"{GREEN}PASS{RESET}", FAIL: f"{RED}FAIL{RESET}", SKIP: f"{YELLOW}SKIP{RESET}"}

STATUS_TOPIC = "/rammp/mib/status"
DRIVE_TOPIC = "/rammp/drive_command"
BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)
STATE_NAME = {
    MibStatus.SYSTEM_STATE_INITIALIZING: "INITIALIZING",
    MibStatus.SYSTEM_STATE_IDLE: "IDLE",
    MibStatus.SYSTEM_STATE_ENABLED: "ENABLED",
    MibStatus.SYSTEM_STATE_ERROR: "ERROR",
    MibStatus.SYSTEM_STATE_AUTONOMOUS: "AUTONOMOUS",
    MibStatus.SYSTEM_STATE_SHARED: "SHARED",
    MibStatus.SYSTEM_STATE_ESTOP: "ESTOP",
    MibStatus.SYSTEM_STATE_PUSH_ASSIST: "PUSH_ASSIST",
}
MOVING = 0.02  # m/s: above this the chair counts as moving (MibStatus.speed, 2 Hz)


class Chair:
    """The autonomy side of the Jetson's contract, on top of the arbitration Ctx."""

    def __init__(self, ctx: Ctx, speed: float, nudge: float):
        self.ctx = ctx
        self.speed = speed
        self.nudge = nudge
        # Every endpoint up front, like Ctx: espp mismatches endpoints made late.
        ctx.sub(MibStatus, STATUS_TOPIC, BEST_EFFORT)
        self.drive_pub = ctx.n.create_publisher(DriveCommand, DRIVE_TOPIC, BEST_EFFORT)
        ctx.spin(1.0)

    def state(self, timeout=3.0):
        m = self.ctx.latest(STATUS_TOPIC, timeout=timeout, fresh=True)
        return None if m is None else m.system_state

    def wait_state(self, want, timeout=4.0):
        end = time.time() + timeout
        last = None
        while time.time() < end:
            last = self.state(timeout=min(1.5, max(0.1, end - time.time())))
            if last == want:
                return True, last
        return False, last

    def request(self, request, repeats=5):
        """Best-effort, so send it a few times, as the DriveCommand contract allows."""
        m = DriveCommand(request=request, profile=DriveCommand.PROFILE_LOW, push_assist=False)
        for _ in range(repeats):
            self.drive_pub.publish(m)
            self.ctx.spin(0.1)

    def drive(self, token, linear_x, seconds):
        """Stream twists at 20 Hz and return the largest |speed| MibStatus showed."""
        peak = 0.0
        end = time.time() + seconds
        while time.time() < end:
            self.ctx.send_twist(token, linear_x, 0.0, count=1, period=0.05)
            m = self.ctx._latest.get(STATUS_TOPIC)
            if m is not None:
                peak = max(peak, abs(m.speed))
        return peak

    def settled_speed(self, seconds=1.0):
        """|speed| once the lapse has passed and a fresh status has arrived."""
        self.ctx.spin(seconds)
        m = self.ctx.latest(STATUS_TOPIC, timeout=2.0, fresh=True)
        return None if m is None else abs(m.speed)

    def rejected(self):
        return self.ctx.control_status(fresh=True).rejected_count


def name(state):
    return STATE_NAME.get(state, f"state {state}")


# ---- the checks ---------------------------------------------------------------------


def check_request_switches_state(ch: Chair):
    ch.request(DriveCommand.REQUEST_AUTONOMOUS)
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_AUTONOMOUS)
    if not ok:
        return Result("", FAIL, f"asked for AUTONOMOUS from IDLE, MibStatus shows {name(got)}")
    return Result("", PASS, "untokened DriveCommand AUTONOMOUS -> AUTONOMOUS")


def check_no_owner_no_motion(ch: Chair):
    before = ch.rejected()
    peak = ch.drive(ZERO_TOKEN, ch.speed, 1.0)
    after = ch.rejected()
    if after <= before:
        return Result("", FAIL, f"rejected_count {before} -> {after}: untokened twists not counted")
    if peak > MOVING:
        return Result("", FAIL, f"the chair moved ({peak:.3f} m/s) with nobody owning it")
    return Result("", PASS, f"{after - before} refused, speed stayed {peak:.3f} m/s")


def check_acquire(ch: Chair):
    a = ch.ctx.acquire(owner="autonomy_check")
    if not a.accepted:
        return Result("", FAIL, f"acquire refused: {a.message}")
    cs = ch.ctx.control_status()
    if not cs.owned or cs.owner_id != "autonomy_check":
        return Result("", FAIL, f"status owned={cs.owned} owner_id={cs.owner_id!r}")
    ch.token = list(a.token)
    return Result("", PASS, f"owned by {cs.owner_id}, generation {cs.generation}")


def check_owner_drives(ch: Chair):
    fwd = ch.drive(ch.token, ch.speed, ch.nudge)
    back = ch.drive(ch.token, -ch.speed, ch.nudge)
    ch.drive(ch.token, 0.0, 0.3)
    st = ch.state()
    if st != MibStatus.SYSTEM_STATE_AUTONOMOUS:
        return Result("", FAIL, f"state left AUTONOMOUS while driving: {name(st)}")
    if max(fwd, back) <= MOVING:
        return Result(
            "",
            FAIL,
            f"peak speed {max(fwd, back):.3f} m/s on the owner's twists: nothing moved "
            "(wheels free? drive boards online?)",
        )
    return Result("", PASS, f"forward peak {fwd:.3f} m/s, back {back:.3f} m/s")


def check_untokened_stops(ch: Chair):
    ch.drive(ch.token, ch.speed, 0.8)  # moving on the live token
    before = ch.rejected()
    near = list(ch.token)
    near[-1] ^= 0x01
    peak_after = ch.drive(near, ch.speed, 1.2)  # off by one byte: refused, chair lapses
    after = ch.rejected()
    speed = ch.settled_speed(0.5)
    st = ch.state()
    if after <= before:
        return Result("", FAIL, f"rejected_count {before} -> {after}: near-miss token admitted")
    if speed is None or speed > MOVING:
        return Result("", FAIL, f"still moving at {speed} m/s after 1.2 s of refused twists")
    if st != MibStatus.SYSTEM_STATE_AUTONOMOUS:
        return Result("", FAIL, f"refused twists changed the state to {name(st)}")
    return Result("", PASS, f"{after - before} refused, stopped (peak then {peak_after:.3f} m/s)")


def check_release_to_idle(ch: Chair):
    r = ch.ctx.release(ch.token)
    if not r.accepted:
        return Result("", FAIL, f"release refused: {r.message}")
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_IDLE)
    cs = ch.ctx.control_status()
    if not ok:
        return Result("", FAIL, f"released in AUTONOMOUS, MibStatus shows {name(got)}")
    if cs.owned:
        return Result("", FAIL, f"released but status still owned by {cs.owner_id!r}")
    return Result("", PASS, "release -> IDLE, no owner")


def check_grant_alone_does_not_drive(ch: Chair):
    a = ch.ctx.acquire(owner="autonomy_check")
    if not a.accepted:
        return Result("", FAIL, f"acquire refused: {a.message}")
    ch.token = list(a.token)
    before = ch.rejected()
    peak = ch.drive(ch.token, ch.speed, 1.0)
    after = ch.rejected()
    st = ch.state()
    if peak > MOVING:
        return Result("", FAIL, f"the chair moved ({peak:.3f} m/s) in IDLE on tokened twists")
    if st != MibStatus.SYSTEM_STATE_IDLE:
        return Result("", FAIL, f"tokened twists in IDLE changed the state to {name(st)}")
    return Result(
        "", PASS, f"owned, in IDLE: speed {peak:.3f} m/s, {after - before} counted as refused"
    )


def check_revoke_to_idle(ch: Chair):
    ch.request(DriveCommand.REQUEST_AUTONOMOUS)
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_AUTONOMOUS)
    if not ok:
        return Result("", FAIL, f"could not re-enter AUTONOMOUS: {name(got)}")
    ch.drive(ch.token, ch.speed, 0.8)
    r = ch.ctx.revoke(reason="autonomy_check operator revoke")
    if not r.accepted:
        return Result("", FAIL, f"revoke refused: {r.message}")
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_IDLE)
    cs = ch.ctx.control_status()
    if not ok:
        return Result("", FAIL, f"revoked in AUTONOMOUS, MibStatus shows {name(got)}")
    if cs.owned:
        return Result("", FAIL, f"revoked but status still owned by {cs.owner_id!r}")
    return Result("", PASS, "revoke -> IDLE, no owner")


def check_seize(ch: Chair):
    a = ch.ctx.acquire(owner="first_node")
    if not a.accepted:
        return Result("", FAIL, f"first acquire refused: {a.message}")
    first = list(a.token)
    ch.request(DriveCommand.REQUEST_AUTONOMOUS)
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_AUTONOMOUS)
    if not ok:
        return Result("", FAIL, f"could not enter AUTONOMOUS: {name(got)}")
    ch.drive(first, ch.speed, 0.8)
    b = ch.ctx.acquire(owner="second_node")
    if not b.accepted:
        return Result("", FAIL, f"second acquire refused: {b.message}")
    second = list(b.token)
    if b.generation <= a.generation:
        return Result("", FAIL, f"generation {a.generation} -> {b.generation} on a seize")
    before = ch.rejected()
    ch.drive(first, ch.speed, 1.0)  # the dispossessed node keeps sending
    after = ch.rejected()
    speed = ch.settled_speed(0.3)
    st = ch.state()
    if after <= before:
        return Result("", FAIL, "the seized token was still admitted")
    if speed is None or speed > MOVING:
        return Result("", FAIL, f"still moving at {speed} m/s on the seized token")
    if st != MibStatus.SYSTEM_STATE_AUTONOMOUS:
        return Result("", FAIL, f"a seize changed the state to {name(st)}, expected AUTONOMOUS")
    peak = ch.drive(second, ch.speed, ch.nudge)
    ch.drive(second, -ch.speed, ch.nudge)
    ch.drive(second, 0.0, 0.3)
    if peak <= MOVING:
        return Result("", FAIL, f"the new owner's twists did not drive (peak {peak:.3f} m/s)")
    r = ch.ctx.release(second)
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_IDLE)
    if not (r.accepted and ok):
        return Result("", FAIL, f"release by the new owner: accepted={r.accepted}, state {name(got)}")
    return Result(
        "", PASS, f"first token refused ({after - before}), state stayed, second drove {peak:.3f} m/s"
    )


def check_stick_touch(ch: Chair):
    a = ch.ctx.acquire(owner="autonomy_check")
    if not a.accepted:
        return Result("", FAIL, f"acquire refused: {a.message}")
    ch.request(DriveCommand.REQUEST_AUTONOMOUS)
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_AUTONOMOUS)
    if not ok:
        return Result("", FAIL, f"could not enter AUTONOMOUS: {name(got)}")
    print(f"        {YELLOW}>>> touch the joystick now (30 s){RESET}")
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_IDLE, timeout=30.0)
    cs = ch.ctx.control_status()
    if not ok:
        return Result("", FAIL, f"no stick touch seen, or it did not end autonomy: {name(got)}")
    if cs.owned:
        return Result("", FAIL, f"stick touch ended autonomy but the grant stayed ({cs.owner_id!r})")
    return Result("", PASS, "stick touch -> IDLE, grant revoked")


def check_estop(ch: Chair):
    print(f"        {YELLOW}>>> press the e-stop now (30 s){RESET}")
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_ESTOP, timeout=30.0)
    if not ok:
        return Result("", FAIL, f"no e-stop seen: {name(got)}")
    cs = ch.ctx.control_status()
    if not cs.estopped:
        return Result("", FAIL, "MibStatus shows ESTOP but control_status.estopped is false")
    a = ch.ctx.acquire(owner="autonomy_check")
    if a.accepted:
        return Result("", FAIL, "acquire accepted while e-stopped")
    print(f"        {YELLOW}>>> clear the e-stop now (30 s){RESET}")
    ok, got = ch.wait_state(MibStatus.SYSTEM_STATE_IDLE, timeout=30.0)
    if not ok:
        return Result("", FAIL, f"latch did not clear to IDLE: {name(got)}")
    cs = ch.ctx.control_status()
    if cs.estopped or cs.owned:
        return Result("", FAIL, f"after the clear: estopped={cs.estopped} owned={cs.owned}")
    return Result("", PASS, f"e-stop refused acquire ({a.message}); cleared to IDLE, no owner")


CHECKS = [
    ("an untokened DriveCommand AUTONOMOUS switches the state", check_request_switches_state),
    ("with no owner, twists are refused and the chair does not move", check_no_owner_no_motion),
    ("acquire_control grants", check_acquire),
    ("the owner's twists drive the chair", check_owner_drives),
    ("a wrong token is refused and the chair stops within the lapse", check_untokened_stops),
    ("release_control in AUTONOMOUS returns to IDLE with no owner", check_release_to_idle),
    ("a grant alone does not drive: tokened twists in IDLE do nothing", check_grant_alone_does_not_drive),
    ("revoke_control in AUTONOMOUS returns to IDLE with no owner", check_revoke_to_idle),
    ("a second acquire seizes: old token refused, state stays, new owner drives", check_seize),
]
MANUAL = [
    ("a joystick touch ends AUTONOMOUS and revokes the grant", check_stick_touch),
    ("the e-stop latches the Arbiter and clears to no owner", check_estop),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.1, help="m/s for each nudge (default 0.1)")
    ap.add_argument("--nudge", type=float, default=1.5, help="seconds per nudge (default 1.5)")
    ap.add_argument("--manual", action="store_true", help="also the prompted stick-touch and e-stop checks")
    args = ap.parse_args()

    rclpy.init()
    node = Node("mib_autonomy_check", namespace="base")
    ctx = Ctx(node)
    ch = Chair(ctx, args.speed, args.nudge)

    print("waiting for /base/control_status and /rammp/mib/status ...")
    cs = ctx.control_status(fresh=False, timeout=15.0)
    st = ch.state(timeout=15.0)
    if cs is None or st is None:
        print(f"{RED}no status from the MIB. Is it up, on this network, and running the main firmware?{RESET}")
        rclpy.shutdown()
        return 2
    print(f"attached. arbitration = {'enforced' if cs.arbitration_enabled else 'disabled'}, "
          f"state = {name(st)}, owned = {cs.owned}")
    if not cs.arbitration_enabled:
        print(f"{YELLOW}arbitration is DISABLED on this build: the refusal checks will fail{RESET}")
    if st != MibStatus.SYSTEM_STATE_IDLE:
        print(f"{RED}the MIB must start in IDLE (disable drive from the joystick or the panel){RESET}")
        rclpy.shutdown()
        return 2
    if cs.owned:
        print(f"someone owns the base ({cs.owner_id!r}); revoking to start clean")
        ctx.revoke(reason="autonomy_check start")
    print(f"the chair will move about {args.speed * args.nudge:.2f} m each way\n")

    results = []
    try:
        for label, fn in CHECKS + (MANUAL if args.manual else []):
            try:
                r = fn(ch)
            except Exception as e:  # a raising check is a failing one
                r = Result("", FAIL, f"{type(e).__name__}: {e}")
            r.name = label
            results.append(r)
            print(f"  {MARK[r.status]}  {label}")
            if r.detail:
                print(f"        {DIM}{r.detail}{RESET}")
            if r.status == FAIL and fn in (check_acquire, check_request_switches_state):
                print(f"{RED}cannot go on without that{RESET}")
                break
    finally:
        # Leave it as found: a revoke drops any grant and takes AUTONOMOUS back to IDLE.
        ctx.cleanup()
        node.destroy_node()
        rclpy.shutdown()

    counts = {k: sum(1 for r in results if r.status == k) for k in (PASS, FAIL, SKIP)}
    print(f"\n{counts[PASS]} passed, {counts[FAIL]} failed, {counts[SKIP]} skipped")
    return 1 if counts[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
