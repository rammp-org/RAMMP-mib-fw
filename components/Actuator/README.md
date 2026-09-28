# Actuator component

`mib::Actuator` is one positioned axis of a Basicmicro MCP266. It is built on
`espp::Mcp266` (CANopen, CiA 402 profile position) and stays deliberately
small: the controller closes the position loop, runs the motion profile,
enforces its clamp and stops on limit switches; this class knows the axis's
calibrated travel, its default motion profile and a jog step, and turns "go
there" into the controller exchange. Kinematics live in a `JointModel` the
config names, so the component survives a redesign of the base.

## Positions

Three interchangeable forms, all through the calibrated range:

| Form | Meaning | Used for |
|---|---|---|
| counts | the encoder's own units | calibration, homing, jogging |
| fraction | 0 at the low calibrated end, 1 at the high end | layers that must not know counts |
| units | metres or degrees, via the `JointModel` | the state machine |

The current position is not held as state. `read_position()` asks the encoder
(one SDO), and `current_position()` returns whatever the last read produced
together with `position_age()`; it is as fresh as the owner's periodic poll.
Relative moves always read the encoder first, so "relative" means relative to
where the joint is now, not to the last target (`last_target()` exists for a
layer that wants the other meaning).

## API

| Method | Purpose |
|---|---|
| `initialize()` | Once per boot: clear faults, install the range as the controller's clamp and software limits, restore a saved position for an incremental encoder |
| `ready()` | Initialised and, where required, homed. Moves are refused otherwise |
| `read_position(counts)`, `read_fraction(f)`, `read_units(u)` | Ask the encoder |
| `current_position()`, `current_fraction()`, `current_units()`, `position_age()` | The last reading, without a bus exchange |
| `move_absolute(counts)`, `move_fraction(f)`, `move_to(units)` | Profile move, clamped to the range; `move_absolute` has a per-move `Profile` overload |
| `move_relative(delta)`, `move_relative_fraction(df)` | Read the encoder, add, move |
| `increment()`, `decrement()` | One jog step in counts |
| `stop()` | CiA 402 quick stop |
| `is_target_reached(reached)`, `get_drive_state(state)` | The controller's arrival flag and decoded CiA 402 state |

Every method returns true on success and logs the reason on failure, as the
rest of the firmware does; espp's error codes stay inside. Every method that
talks to the controller blocks for one or more SDO round trips, a few
milliseconds each, and must be called from a task that may wait.

Two actuators share one controller (its M1 and M2) through a mutex passed to
both. A controller that stops answering is marked offline and skipped until a
retry interval passes. A controller that answers but has lost its limits (it
reset while the MIB ran and came back with the factory `[0, 0]` clamp) is
caught by a two-SDO check before every move and re-initialised.

## Two procedures that must not be confused

**Calibration** is for absolute encoders and happens once per installation.
`set_calibration_mode(true)` lifts the controller's clamp, the joint is jogged
to its mechanical ends and the counts read, `set_range({min, max})` installs
the range on the controller and saves it through the `ActuatorStore`, and
`set_calibration_mode(false)` restores the clamp. The saved range overrides the
compiled default at every later boot.

**Homing** is for incremental (AB) encoders, whose count is arbitrary at
power-up, and happens every boot. `home()` drives toward the limit switch at
the homing profile until the controller stops the motor on it (detected as the
count no longer changing), installs `homing.home_count` as the encoder value,
restores the clamp and saves the position. To avoid homing at every boot the
owner calls `save_position()` from its periodic poll; it writes the count to
NVS, throttled by an interval and a minimum movement so flash is not worn, and
`initialize()` then restores it with `restore_position()`. That restore assumes
the joint did not move while powered off; `home()` when in doubt.

`ActuatorStore` keeps both kinds of record in NVS under separate keys.

## JointModel

`components/JointModel` holds the interface (`to_fraction`, `to_units`,
`unit`) and a `LinearJointModel` for a joint proportional to actuator travel.
The real models for the main legs, casters and carriages are written once the
base is measured; they are pure arithmetic, so they can be unit-tested on the
host.

## Example

[`example/`](example) is a standalone project that brings up one actuator on
one MCP266 channel, with its pins, node id, range, profile and a linear joint
model written in the example, and offers a serial console to exercise the API:
`status`, `pos`, `abs`, `to`, `frac`, `rel`, `inc`, `dec`, `wait`, `stop`, the
calibration commands `cal`, `range`, `forget`, the homing commands `home`,
`restore`, `save`, `setenc`, and `selftest`. It doubles as the bench test for
one actuator: `status` proves the controller answers, `pos` the encoder, `inc`
the direction, `selftest` the position loop. Build it from that directory with
`idf.py set-target esp32p4` then `idf.py build flash monitor`.

## Behaviour of the controller worth knowing

- The MCP266 reverts to its EEPROM at power-up, and its factory position clamp
  is `[0, 0]`, which forces every target to zero. `initialize()` widens it every
  boot.
- Only profile position mode moves the motor over CANopen on the firmware
  espp tested; velocity and duty commands are accepted but inert. Jogging is
  therefore a small closed-loop position move, not an open-loop nudge.
- The set-encoder write used by homing (Basicmicro command 22/23 mirrored at
  0x2016/0x2017) has not yet been verified over CAN. `home()` reads the count
  back and fails loudly if it did not take.
- Position loop gains are tuned on the controller (Motion Studio, or the
  debugger's MCP266 / CAN page on the `feature/21-mib-debugger` branch), not
  here.
