# ROS 2 bench

Talks to the MIB from ROS 2 over RTPS, as the Jetson will: Fast DDS on ROS 2
Jazzy, with the rammp-interfaces-ros2 types. No ROS install needed: pixi pulls
ROS 2 from RoboStack, natively on macOS and Linux.

```console
cd tools/ros2_bench
pixi run setup                          # once: fetch and build the interfaces
pixi run conformance                    # against a running MIB
pixi run conformance --serial /dev/cu.usbmodem5B901579871   # and read its console
```

Flash `components/ArbitrationServer/example` first.

## What it checks

`conformance/` attaches to a running MIB and reports PASS, FAIL or SKIP per
check; the exit code is non-zero on any FAIL.

- **arbitration**: kinova-gen3-ros2's `test/conformance/checks_arbitration.py`,
  fetched unchanged at `228c48e` by `pixi run setup`. The same contract the arm
  passes, run against the base: acquire, seize, release with the wrong and the right token,
  revoke. Its five `/estop` checks are SKIPPED by name: the base's e-stop is
  estop-core's `rammp/estop` latch.
- **base**: what the arm's suite does not cover. The status reaches a late
  subscriber; an untokened twist arrives, decodes and is counted as rejected; a
  token off by its last byte is refused (all 16 bytes decode); the owner's
  twists are admitted and their values arrive intact (the doubles are aligned);
  a seized owner's token stops working; every change of owner halts the base,
  and an idle revoke does not; a pre-supplied token is adopted. The checks that
  must see an admitted command read the MIB's console, so they need `--serial`.

`harness.py` has the same interface as the arm's, so the arm's checks run
unchanged; it makes every endpoint once, up front, as a real client does.

## Network

RTPS discovery is multicast: the MIB announces itself on 239.255.0.1 every 2 s,
and a host that never hears it never discovers the MIB, even though the MIB
discovers the host. Two things on the host's side have to be right.

**The host's DDS on the MIB's interface.** `env.sh` handles it. On a host with
several interfaces (Wi-Fi, a VPN) Fast DDS otherwise listens for discovery on
the wrong one (on a Mac it sees nothing from the MIB at all), and the MIB
replies to the first address the host announces, which may be Wi-Fi.
`fastdds_mib.xml` restricts Fast DDS to the interface on the MIB's subnet and
names the MIB as an initial peer. Fast DDS 2.x (Jazzy) reads
`FASTRTPS_DEFAULT_PROFILES_FILE` and 3.x `FASTDDS_DEFAULT_PROFILES_FILE`;
`env.sh` sets both. `MIB_IP` overrides the MIB's address (default 192.168.4.1).

**A network that delivers the multicast.** A direct cable does. The bench's
TP-Link TL-SG105 did not: it does IGMP snooping, and the MIB's announcements to
239.255.0.1 never reached the Mac through it, while unicast, broadcast and
multicast to groups the MIB had not joined all did. Cable the host directly, or
use a switch whose IGMP snooping can be turned off or that has a querier. The
robot's own switch between the Jetson and the MIB has the same requirement.
