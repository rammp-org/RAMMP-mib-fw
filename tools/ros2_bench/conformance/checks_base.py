"""The base's own arbitration checks: what the arm's suite does not cover.

The wire format first (a latched status, a tokened twist that arrives and decodes),
then admission by token, then the pre-supplied token from rammp-interfaces-ros2 dev.
The checks that need to SEE an admitted command read the bench firmware's USB log
(--serial); without it they are skipped, never passed.
"""

import time

from harness import FAIL, PASS, REGISTRY, SKIP, ZERO_TOKEN, LATCHED_QOS, Result, tok
from rammp_common_interfaces.msg import ControlStatus

SEC = "base"


@REGISTRY.add(SEC, "control_status is latched: a late subscriber gets it at once")
def check_latched(ctx):
    # A subscription created now, long after the last change. The MIB resends every 1 s,
    # so arriving well inside that is the transient_local history, not the resend.
    got = {}
    start = time.time()
    sub = ctx.n.create_subscription(
        ControlStatus,
        "control_status",
        lambda m: got.setdefault("t", time.time() - start),
        LATCHED_QOS,
    )
    ctx.spin(0.01)
    end = time.time() + 3.0
    while "t" not in got and time.time() < end:
        ctx.spin(0.01)
    ctx.late_subscription = sub  # kept, not destroyed: see harness.Ctx
    if "t" not in got:
        return Result("", FAIL, "nothing within 3 s: the QoS does not match")
    latched = got["t"] < 0.5
    return Result(
        "",
        PASS,
        f"first sample after {got['t'] * 1000:.0f} ms "
        f"({'transient_local history' if latched else 'the periodic resend'})",
    )


@REGISTRY.add(SEC, "an untokened twist arrives, decodes, and is refused and counted")
def check_untokened_twist(ctx):
    ctx.acquire("conformance-owner")
    before = ctx.control_status().rejected_count
    ctx.send_twist(ZERO_TOKEN, 0.1, 0.1, count=20)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after - before < 15:
        return Result(
            "",
            FAIL,
            f"rejected_count {before} -> {after} for 20 sent: the twist is not "
            "reaching the MIB, or does not decode",
        )
    return Result("", PASS, f"rejected_count {before} -> {after} for 20 sent")


@REGISTRY.add(SEC, "a token off by its LAST byte is refused: all 16 bytes decode")
def check_last_byte(ctx):
    a = ctx.acquire("conformance-owner")
    near = tok(a.token)
    near[15] ^= 0x01
    before = ctx.control_status().rejected_count
    ctx.send_twist(near, 0.1, 0.1, count=20)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after - before < 15:
        return Result("", FAIL, f"rejected_count {before} -> {after} for 20 sent")
    return Result("", PASS, f"rejected_count {before} -> {after} for 20 sent")


@REGISTRY.add(SEC, "the live token is admitted, and the twist values round-trip")
def check_admitted(ctx):
    a = ctx.acquire("conformance-owner")
    before = ctx.control_status().rejected_count
    first = ctx.serial.next() if ctx.serial else None
    ctx.send_twist(a.token, 0.25, -0.5, count=40)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after != before:
        return Result("", FAIL, f"the owner's twists were refused ({before} -> {after})")
    if not ctx.serial:
        return Result("", SKIP, "not refused; pass --serial to see them admitted")
    line = ctx.serial.next()
    if line is None or first is None:
        return Result("", FAIL, "no status line on the serial console")
    admitted = line["admitted"] - first["admitted"]
    if admitted < 30:
        return Result("", FAIL, f"only {admitted} of 40 admitted")
    if abs(line["linear_x"] - 0.25) > 1e-3 or abs(line["angular_z"] + 0.5) > 1e-3:
        return Result(
            "",
            FAIL,
            f"sent (0.25, -0.5), MIB holds ({line['linear_x']}, {line['angular_z']}): "
            "the doubles are misaligned on the wire",
        )
    return Result("", PASS, f"{admitted} of 40 admitted, twist (0.25, -0.5) intact")


@REGISTRY.add(SEC, "a stale token stops working the moment someone else acquires")
def check_stale_after_seize(ctx):
    old = ctx.acquire("conformance-incumbent")
    ctx.acquire("conformance-usurper")
    before = ctx.control_status().rejected_count
    ctx.send_twist(old.token, 0.1, 0.0, count=20)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after - before < 15:
        return Result("", FAIL, f"the old owner is still admitted ({before} -> {after})")
    return Result("", PASS, f"old owner refused ({before} -> {after})")


@REGISTRY.add(SEC, "every change of owner halts the base")
def check_halts(ctx):
    if not ctx.serial:
        return Result("", SKIP, "pass --serial to count the halts")
    ctx.acquire("conformance-a")
    first = ctx.serial.next()
    b = ctx.acquire("conformance-b")  # seize: 1 halt
    ctx.release(b.token)  # release: 1 halt
    ctx.revoke()  # nobody owns it: no halt
    line = ctx.serial.next()
    halts = line["halts"] - first["halts"]
    if halts != 2:
        return Result("", FAIL, f"{halts} halts for a seize and a release, expected 2")
    return Result("", PASS, "seize and release halted once each; an idle revoke did not")


@REGISTRY.add(SEC, "acquire adopts a pre-supplied token", needs_mode="enforced")
def check_presupplied(ctx):
    key = [0] * 16
    key[3] = 0x5A
    r = ctx.acquire("conformance-orchestrator", token=key)
    if not r.accepted:
        return Result("", FAIL, f"refused: {r.message}")
    if tok(r.token) != key:
        return Result("", FAIL, "a different token came back: the field was ignored")
    before = ctx.control_status().rejected_count
    ctx.send_twist(key, 0.0, 0.0, count=20)
    ctx.spin(0.5)
    after = ctx.control_status().rejected_count
    if after != before:
        return Result("", FAIL, f"the adopted key is refused ({before} -> {after})")
    return Result("", PASS, f"key adopted at generation {r.generation}, and admitted")
