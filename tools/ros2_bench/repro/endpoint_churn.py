#!/usr/bin/env python3
"""Repro: does the MIB match a subscription created after another was deleted?

Creates a /base/control_status subscription, waits for its first sample, destroys
it, and repeats; then does the same keeping every subscription alive. Prints the
seconds to the first sample per subscription, 99 for none within 8 s.

    pixi run bash -c 'source env.sh && python3 repro/endpoint_churn.py'

Against the MIB running components/ArbitrationServer/example on stock espp/rtps
1.3.10, cabled directly to a Mac on ROS 2 Jazzy (Fast DDS 2.14), MIB reset before each
run:
    destroy each: [0.15, 0.08, 99, 0.06, 0.08, 99, 0.08, 0.08, 99, 0.08, 0.08, 99]
    keep each:    all matched
    destroy each: [0.12, 0.02, 0.08, 99, 0.08, 0.08, 99, 0.08, 0.02, 99, 0.05, 0.08]
    keep each:    [99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99]
    destroy each: [0.11, 0.08, 0.08, 0.08, 0.01, 99, 0.08, 0.08, 99, 0.08, 0.08, 99]
    keep each:    all matched
A new process (a new participant) matches every subscription until it too has
destroyed a few. The same loop against a Fast DDS publisher on the Mac matches all
24 subscriptions.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rammp_common_interfaces.msg import ControlStatus

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def run(node, count, keep):
    kept, result = [], []
    for _ in range(count):
        got = {}
        start = time.time()
        sub = node.create_subscription(
            ControlStatus, "control_status", lambda m: got.setdefault("t", time.time() - start), QOS
        )
        while "t" not in got and time.time() - start < 8:
            rclpy.spin_once(node, timeout_sec=0.02)
        result.append(round(got.get("t", 99), 2))
        if keep:
            kept.append(sub)
        else:
            node.destroy_subscription(sub)
        pause = time.time()
        while time.time() - pause < 0.3:
            rclpy.spin_once(node, timeout_sec=0.02)
    return result


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    rclpy.init()
    node = Node("endpoint_churn", namespace="base")
    print("destroy each:", run(node, count, keep=False))
    print("keep each:   ", run(node, count, keep=True))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
