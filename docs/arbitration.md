# Base arbitration: design

Who may command the base, how the base finds out, and how that stays separate
from what the base does. Written while the first piece, the `Arbiter`
component, was being built. Sections marked **planned** are not implemented.

## Why

Several things can ask the base to move: the joystick, and autonomy on the
Jetson. Later, a load cell for power-assisted pushing. Only one of them may be
in control at a time, the handover has to stop the base, and a human choosing
to drive must win.

The arm already solves this with a token arbiter. The base copies it
one-to-one, in the subset that the base needs, so that one orchestrator and one
operator tool on the Jetson speak the same protocol to both resources.

## Two state machines, kept apart

**Arbitration** answers *who may ask*. It is the `Arbiter` component: grant,
release, revoke, e-stop, status, and a token check on every command.

**The system state machine** in `MibSystem` answers *what an admitted command
means right now*. It decides whether the stick drives the chair or navigates
the joystick's GUI, whether seat commands are accepted, and what a halt does.
The Arbiter never sees it. It only calls back `on_halt`.

The state machine is being reworked separately and will grow states the
Arbiter does not care about (an `ESTOP` state is planned). Nothing here depends
on that.

## Who holds tokens

A token is a one-time password for one resource, minted by that resource's
arbiter. It is per resource, never per device: the base's arbiter mints base
tokens, the arm's mints arm tokens, and the joystick's own GUI needs none.

What a token buys is telling apart several senders on one channel. The receive
callback hands the firmware a sample with no sender, so when several processes
on the Jetson could publish base commands, only the token says which one was
granted.

| Source | How it is identified | Token on the wire? |
|---|---|---|
| Jetson (autonomy, teleop relay, operator tool) | the token it was granted | yes, on every command |
| Joystick | its own topics, which nothing else publishes on | no |
| Load cell (later) | its own topic | no |

The joystick must drive the base with no Jetson attached, so its messages stay
exactly as they are and the base keeps reading them. On the arm the joystick's
broadcast is turned into tokened commands by a teleop node on the Jetson. On
the base that node lives inside the firmware, **planned** as a small in-process
*joystick client* that holds a token from the base's own Arbiter and stamps
joystick input before it enters the arbiter like any other owner's. The Arbiter
itself never learns the word "joystick".

Ownership follows the system state rather than a client policy:

- Entering `DRIVE_ENABLED` acquires the base for the joystick. If someone else
  held it (the end of an autonomous action) that seizes, and the resulting
  `OWNERSHIP_SEIZED` halt is ignored because the joystick is the new owner.
- Entering `IDLE` releases it. The stick goes back to being a GUI input and the
  base shows no owner.
- A stick deflection during autonomy revokes the Jetson's grant. The Jetson sees
  `owned=false` and a generation bump in `control_status`.
- Seat commands only happen in IDLE with nothing moving, so they need no
  ownership change.

## What the base serves on the wire

The subset of the arm's surface that the base needs, with the same types:

| Arm surface | Type | Base equivalent |
|---|---|---|
| `/acquire_control` | `rammp_common_interfaces/AcquireControl`, seizes | `/base/acquire_control` |
| `/release_control` | `ReleaseControl`, token must match | `/base/release_control` |
| `/revoke_control` | `RevokeControl`, no token | `/base/revoke_control` |
| `/estop` | `EStop` topic, robot-wide | not served: the MIB's e-stop is estop-core's `rammp/estop` latch |
| `/control_status` | `ControlStatus`, on change, latched | `/base/control_status`, on change and every second |
| `/setpoint/twist` | `TwistSetpoint`: geometry twist plus token | `/base/setpoint/twist`, `rammp_base_interfaces/TwistSetpoint` |
| `arbitration_mode` | launch parameter | `ArbitrationMode` in the Arbiter's config |

Done: `components/ArbitrationServer` serves the first five rows' base column,
with C++ mirrors of the types in rammp-rtps, and its example subscribes to the
twist. `tools/ros2_bench` runs the arm's own arbitration conformance checks
against it from ROS 2, unchanged, and passes them (bar the `/estop` ones,
skipped).

Not ported: the stream session services (`OpenStream`, `CloseStream`,
`StreamStatus`, `ListControllers`). They exist on the arm because it has
several controllers at different rates and the client has to say which one it
is opening. The base has one, and we control both publishers. Adding them later
is additive.

The base's `TwistSetpoint` landed in `rammp_base_interfaces` (rammp-interfaces-ros2
dev), the same shape as the arm's. The base also needs tokened `DriveCommand` and `SeatCommand`
types in `rammp_base_interfaces`, plus a `ModeCommand` the Jetson sends to ask
the base for a control mode: MANUAL, COMBINED (the joystick drives with an
alternative stick mapping while also controlling the arm), or AUTONOMOUS. New
modes are enum values, not new messages.

The espp RTPS service and pub/sub wrappers default to the ROS 2 wire protocol,
so the MIB serves these to ROS 2 clients with the unmodified
`rammp_common_interfaces` types.

## On the wire

Measured on the bench: the MIB on Ethernet behind a TP-Link TL-SG105 switch, a
Mac running ROS 2 Jazzy (Fast DDS 2.14) through `tools/ros2_bench`.

The two questions this section used to leave open:

- **espp's CDR matches ROS 2.** An `AcquireControl` request serialized by espp
  is byte-identical to rclpy's: `uint8[16]` is a fixed array with no length
  prefix. On hardware, a token off by its last byte is refused and a twist of
  doubles arrives intact.
- **`control_status` reaches a late subscriber.** espp writers advertise
  transient_local, and a ROS 2 subscriber asking for transient_local gets the
  status within about 80 ms of matching. Unlike ROS latching it replays the
  writer's whole history (up to 10 samples, oldest first), so a client must keep
  the newest. The MIB also resends every second.

What it took for a ROS 2 host to reach the MIB at all, each one hiding the next:

1. **The network has to deliver the discovery multicast.** Behind the bench's
   TP-Link TL-SG105, packets the MIB sends to a multicast group it has itself
   joined never reach the host; packets to any other group do, and the moment
   the MIB leaves a group they arrive again. RTPS announces participants on
   239.255.0.1, which the MIB always joins, so the host never discovers the MIB,
   though the MIB discovers the host. Cabled directly, the announcements arrive
   every 2 s and everything works on stock espp. This is network configuration,
   not the firmware: the switch between the Jetson and the MIB must forward the
   group (IGMP snooping off, or a querier), and the host's DDS must be bound to
   the right interface (`tools/ros2_bench/README.md#network`).
2. **IPv4 reassembly was off.** Fast DDS packs its endpoint announcements into
   datagrams past the 1500-byte MTU. lwIP dropped the fragments, every
   retransmit was the same datagram, and discovery stalled before the service
   endpoints. `CONFIG_LWIP_IP4_REASSEMBLY=y`.
3. **espp's limits are sized for espp peers.** Three services and the status
   need more reliable endpoints than the default 5; every ROS 2 client takes a
   proxy slot on each endpoint it matches; ROS 2 parameter-service names exceed
   64 bytes. The settings are in `components/ArbitrationServer/README.md`.
4. **Endpoints created after a deletion sometimes never match.** A ROS 2 node
   that creates and destroys a subscription in a loop gets every third one
   unmatched, indefinitely, and sometimes none at all afterwards; a new process
   matches again. Reproduced on stock espp over a direct cable; the same loop
   against a Fast DDS publisher matches every time. Clients made once, as the
   orchestrator makes them, are unaffected; `ros2` CLI tools and clients made
   per call are not. To be raised with espp
   (`tools/ros2_bench/repro/endpoint_churn.py`).

## Modes, stick interrupts, and history (planned)

The Jetson asks the base for a control mode with a tokened `ModeCommand`:
MANUAL, COMBINED, or AUTONOMOUS, with the profile to use. It must hold the
base's token to send one, so it acquires first; if the user was driving, that
seizes and stops the chair. In the system state machine, IDLE, `DRIVE_ENABLED`
and `DRIVE_COMBINED` sit in a `MANUAL` composite with a shallow history
pseudostate. AUTONOMOUS leaves MANUAL from any leaf, and ending it with a MANUAL
mode request returns through history to whatever the user was doing. If that is
a drive state, its entry re-acquires the base for the joystick.

The stick is a robot-wide broadcast, so the arm and the base react to it the
same way during autonomy: **a deflection stops the autonomous owner.**

- On the base, the state machine's AUTONOMOUS state treats any deflection or
  button as abort. The transition revokes the Jetson's grant, so the Jetson sees
  `owned=false` and a generation bump in `control_status`, and lands in IDLE,
  **not** in history. A bump on the stick should stop the chair, not become a
  live drive command.
- On the arm, a Jetson-side node subscribes to the stick and the arm's
  `ControlStatus` and calls the arm's `/revoke_control` when the current owner is
  an autonomous one. That is as reliable as the arm itself, because the arm only
  moves while the Jetson is alive. No change to the arm driver. How that node
  knows which owners are interruptible is the orchestrator's business.

## Halt, deadline, and error (planned)

Every change of ownership, the e-stop, and a deadline expiry stop the drive
controller through the planner. What the state machine does next depends on
why. A halt with `OWNERSHIP_REVOKED` means nobody owns the base, so it drops to
IDLE and the stick returns to the GUI. A halt with `OWNERSHIP_SEIZED` means a
new owner has it and the chair has stopped; the state does not change, and the
new owner's next command decides what happens. That distinction is the one
place the base's Arbiter adds a halt reason the arm's does not have.

The **deadline** is a liveness watchdog on the owner-to-MIB link. A latest-wins
twist stream has no stop message; if the publisher dies the last twist would be
held forever. The arm expires a session when no admitted setpoint arrives
within `timeout_s`. The base uses a fixed constant in `MIBconfig.hpp`, checked
by the 20 ms state task while driving, and a rejected setpoint does **not**
refresh it: a sender with a bad token is not evidence the stream is healthy.
The joystick publishes continuously even at rest, so it is covered too; pick
the timeout from its rate, a handful of publish periods.

Deadline expiry is routine, every autonomy restart will trigger it, so it
halts and returns to IDLE. It is **not** an error. `ERROR`, with the motor
boards disarmed to coast, is for when the MIB cannot trust its own control:
board faults, bring-up failure, both drive boards silent. The MIB-to-motor link
is already watched downstream: a board that gets no command for 200 ms safe
stops and holds on its own.

The physical e-stop is the hardware safety chain and freezes everything
regardless of state. The software `/estop` stays in the port for one-to-one
with the arm and for operator tools. If the MIB can read the e-stop line, it
should latch the Arbiter so `control_status` reports it and commands are
refused until cleared. That is reporting, not safety.

## Profiles

The joystick chooses a drive profile in its `DriveCommand`; the Jetson chooses
one in its `ModeCommand`. Both read the active profile back in `MibStatus`. A
useful side effect: autonomy's twist in metres per second is clamped to the
profile's caps and smoothed by the planner's acceleration limits, so a bad
twist cannot command a jerk or exceed what the chair allows a human.

## Roadmap

1. **`Arbiter` component.** Done: `components/Arbiter`, with an example.
2. **RTPS server face.** Done: `components/ArbitrationServer`, tested from
   ROS 2 on hardware. Not done: an `/estop` subscription; the MIB's e-stop is
   estop-core's latch (RAMMP-mib-fw#36, #39), which should latch the Arbiter.
3. **Twist input.** The type is settled and the example subscribes to it;
   converting it to the drive controller's input is left for the wiring.
4. **Joystick client.** The in-process owner described above.
5. **Wire into `MibSystem`.** Handlers call `run_if_admitted`; `on_halt` feeds
   the state machine; the deadline runs on the state task; `arbitration_mode`
   in `MIBconfig.hpp`, defaulting to `DISABLED` so the joystick keeps working
   with no Jetson until it is flipped.
