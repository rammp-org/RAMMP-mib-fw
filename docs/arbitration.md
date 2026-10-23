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

## What the base serves on the wire (planned)

The subset of the arm's surface that the base needs, with the same types:

| Arm surface | Type | Base equivalent |
|---|---|---|
| `/acquire_control` | `rammp_common_interfaces/AcquireControl`, seizes | same type, base namespace |
| `/release_control` | `ReleaseControl`, token must match | same |
| `/revoke_control` | `RevokeControl`, no token | same |
| `/estop` | `EStop` topic, robot-wide | same topic, shared with the arm |
| `/control_status` | `ControlStatus`, on change | same, base namespace |
| `/setpoint/twist` | `TwistSetpoint`: geometry twist plus token | same, base namespace |
| `arbitration_mode` | launch parameter | constant in `MIBconfig.hpp` |

Not ported: the stream session services (`OpenStream`, `CloseStream`,
`StreamStatus`, `ListControllers`). They exist on the arm because it has
several controllers at different rates and the client has to say which one it
is opening. The base has one, and we control both publishers. Adding them later
is additive.

`TwistSetpoint` is moving out of `rammp_arm_interfaces` into a new
`rammp_control_interfaces` package. The firmware side will take the C++ mirror
from `rammp-rtps` once it lands; until then nothing in this repo depends on its
final shape. The base also needs tokened `DriveCommand` and `SeatCommand`
types in `rammp_base_interfaces`, plus a `ModeCommand` the Jetson sends to ask
the base for a control mode: MANUAL, COMBINED (the joystick drives with an
alternative stick mapping while also controlling the arm), or AUTONOMOUS. New
modes are enum values, not new messages.

The espp RTPS service and pub/sub wrappers default to the ROS 2 wire protocol,
so the MIB can serve these to ROS 2 clients with the unmodified
`rammp_common_interfaces` types. To verify at implementation time: that espp's
CDR layer serializes a 16-byte array the way ROS 2 expects, and whether an espp
publisher can offer transient-local durability so `control_status` is latched
like the arm's, or has to be resent periodically like `MibStatus`.

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
2. **RTPS server face.** Services, e-stop subscription with the clear age
   check, `control_status` publisher. Needs the C++ mirrors of the
   `rammp_common_interfaces` types in `rammp-rtps`.
3. **Twist input.** Subscribe to the base's `TwistSetpoint` once it has its
   final home, convert to the drive controller's input.
4. **Joystick client.** The in-process owner described above.
5. **Wire into `MibSystem`.** Handlers call `run_if_admitted`; `on_halt` feeds
   the state machine; the deadline runs on the state task; `arbitration_mode`
   in `MIBconfig.hpp`, defaulting to `DISABLED` so the joystick keeps working
   with no Jetson until it is flipped.
