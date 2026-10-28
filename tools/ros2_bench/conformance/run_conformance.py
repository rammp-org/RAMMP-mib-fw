#!/usr/bin/env python3
"""Exercise the base's arbitration over ROS 2 against a RUNNING MIB.

    pixi run conformance                                # from tools/ros2_bench
    pixi run conformance --serial /dev/cu.usbmodemXXXX  # also read the MIB's log

Flash components/ArbitrationServer/example first. The runner attaches; it never flashes
or resets. Two sections:

    arbitration   kinova-gen3-ros2's checks_arbitration.py, unchanged: the same contract
                  the arm passes. Its /estop checks are SKIPPED, by name: the base's
                  e-stop is estop-core's rammp/estop latch, not the arm's /estop topic.
    base          what the arm's suite does not cover: the status latching, the tokened
                  twist arriving and decoding, admission by token, halts.

Exit code is non-zero if anything failed.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

import checks_arbitration  # noqa: F401  (registers checks on import)
import checks_base  # noqa: F401
from harness import FAIL, PASS, REGISTRY, SKIP, Ctx, NotOnTheBase, Result, SerialLog

SECTION_ORDER = ["arbitration", "base"]

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
MARK = {PASS: f"{GREEN}PASS{RESET}", FAIL: f"{RED}FAIL{RESET}", SKIP: f"{YELLOW}SKIP{RESET}"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sections", default=",".join(SECTION_ORDER))
    ap.add_argument("--serial", help="the MIB's USB console, to see admitted commands")
    args = ap.parse_args()
    wanted = [s.strip() for s in args.sections.split(",") if s.strip()]

    rclpy.init()
    node = Node("mib_conformance", namespace="base")
    ctx = Ctx(node, SerialLog(args.serial) if args.serial else None)

    print("waiting for /base/control_status ...")
    cs = ctx.control_status(fresh=False, timeout=15.0)
    if cs is None:
        print(f"{RED}no /base/control_status. Is the MIB up and on this network?{RESET}")
        rclpy.shutdown()
        return 2
    mode = "enforced" if cs.arbitration_enabled else "disabled"
    print(f"attached. arbitration = {mode}\n")

    results = []
    try:
        for sec in SECTION_ORDER:
            if sec not in wanted:
                continue
            print(f"{DIM}--- {sec} ---{RESET}")
            for s, name, fn, _, needs_mode in REGISTRY.checks:
                if s != sec:
                    continue
                if needs_mode and needs_mode != mode:
                    r = Result(name, SKIP, f"needs arbitration {needs_mode}, MIB is {mode}")
                else:
                    try:
                        r = fn(ctx)
                        r.name = name
                    except NotOnTheBase as e:
                        r = Result(name, SKIP, str(e))
                    except Exception as e:  # a raising check is a failing one
                        r = Result(name, FAIL, f"{type(e).__name__}: {e}")
                results.append((sec, r))
                print(f"  {MARK[r.status]}  {name}")
                if r.detail:
                    print(f"        {DIM}{r.detail}{RESET}")
            print()
    finally:
        ctx.cleanup()
        node.destroy_node()
        rclpy.shutdown()

    counts = {k: sum(1 for _, r in results if r.status == k) for k in (PASS, FAIL, SKIP)}
    print(f"{counts[PASS]} passed, {counts[FAIL]} failed, {counts[SKIP]} skipped")
    return 1 if counts[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
