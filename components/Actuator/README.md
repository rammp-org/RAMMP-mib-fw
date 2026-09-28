# Actuator component

`mib::Actuator` is one positioned axis of a Basicmicro MCP266, addressed in the
joint's encoder counts. It is built on `espp::Mcp266` (CANopen, CiA 402
profile position) and stays deliberately small: the controller closes the
position loop, runs the motion profile, enforces its clamp and stops on limit
switches; this class knows the axis's calibrated travel, its default motion
profile and a jog step, and turns "go to this count" into the controller
exchange. It knows nothing about what the joint moves, so kinematics live
above it and the component survives a redesign of the base.

## API

| Method | Purpose |
|---|---|
| `initialize()` | Once per boot: clear faults, install the range as the controller's clamp and software limits |
| `get_position(counts)` | Read the joint position; also cached for `last_position()` |
| `move_absolute(target)` | Profile move to a count, clamped to the range; an overload takes a one-off `Profile` |
| `move_relative(delta)` | Read the position, add the delta, move |
| `increment()`, `decrement()` | Move one jog step; for calibration and manual positioning |
| `stop()` | CiA 402 quick stop |
| `is_target_reached(reached)` | The controller's arrival flag |
| `get_drive_state(state)` | Decoded CiA 402 state, for fault reporting |
| `set_calibration_mode(on)` | Lift or restore the clamp so the joint can reach its mechanical ends |
| `set_range(range)` | Install a calibrated range at runtime |

Every method returns true on success and logs the reason on failure, as the
rest of the firmware does; espp's error codes stay inside the component. Every
method that talks to the controller blocks for one or more SDO round trips, a
few milliseconds each, and must be called from a task that may wait.

Two actuators share one controller (its M1 and M2) through a mutex passed to
both. A controller that stops answering is marked offline and skipped until a
retry interval passes, so one dead controller costs the rest of the system one
timeout per interval rather than one per call; a probe read separates a silent
controller from one that merely refused a state change.

`ActuatorRangeStore` keeps calibrated ranges in NVS. A saved range overrides
the compiled default from `MIBconfig.hpp` at boot.

## Example

[`example/`](example) is a standalone project with a serial console that
brings up the bus, creates the six leg actuators from `MIBconfig.hpp`, and
exercises them command by command. It doubles as the bench test for new
hardware; its README lists the commands and the order to run them in.

## Behaviour of the controller worth knowing

- The MCP266 reverts to its EEPROM at power-up, and its factory position clamp
  is `[0, 0]`, which forces every target to zero. `initialize()` widens it every
  boot.
- Only profile position mode moves the motor over CANopen on the firmware
  espp tested; velocity and duty commands are accepted but inert. Jogging is
  therefore a small closed-loop position move, not an open-loop nudge.
- Position loop gains are tuned on the controller (Motion Studio, or the
  debugger's MCP266 / CAN page on the `feature/21-mib-debugger` branch), not
  here.
