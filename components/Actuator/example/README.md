# Actuator example and bench console

A standalone ESP-IDF project that brings up the CAN bus to the MCP266s the way
the MIB BSP does, creates one `mib::Actuator` per leg from `MIBconfig.hpp`, and
gives you a console over the serial monitor to exercise them. It is both the
usage example for the component and the first thing to run on new hardware.

## Build and run

From this directory, with the same ESP-IDF the firmware uses:

```
idf.py set-target esp32p4
idf.py build flash monitor
```

The example reads `mib::config::actuators`, so node ids, channels, pins and
ranges come from the same table the firmware uses. Change them there.

## Commands

| Command | What it does |
|---|---|
| `legs` | Every leg with its controller, range, jog step and whether it is online |
| `init` | NMT-start each controller and initialize each actuator again |
| `pos [leg]` | Read the joint position, or all six |
| `abs <leg> <counts>` | Move to an absolute count (clamped to the range) |
| `rel <leg> <delta>` | Move by a signed number of counts |
| `inc <leg>`, `dec <leg>` | Move one jog step |
| `wait <leg>` | Block until the drive reports target reached, printing progress |
| `stop [leg]` | Quick-stop one leg or all |
| `state <leg>` | CiA 402 drive state, target reached, online, range |
| `cal <leg> on\|off` | Lift or restore the range clamp for calibration |
| `range <leg> <min> <max>` | Install a calibrated range and save it to NVS |
| `forget <leg>` | Erase the saved range so the compiled default applies |
| `selftest <leg>` | `inc`, wait, `dec`, wait; passes if the joint returns to start |

Legs are `FC`, `RC`, `ML`, `MR`, `LCarr`, `RCarr` in any case, or `0` to `5`.

## First run on new hardware

Work through this in order; each step proves one thing.

1. **`legs`** right after boot. A leg shown `online, ready` has a controller
   that answered on that node id. `OFFLINE` means the node id, bit rate,
   wiring or power is wrong for that controller. Fix the table in
   `MIBconfig.hpp` and rebuild.
2. **`pos`**. A plausible, stable count per leg proves the encoder wiring. A
   count that changes when you push the joint by hand proves it is the right
   encoder.
3. **`inc FC`** then **`wait FC`** with the leg free to move. Watch which way
   it goes. If "up" is the wrong direction for the joint, swap the motor
   leads or invert the encoder on the controller; the actuator itself has no
   inversion flag on purpose, so that a count always means the same thing on
   the controller and on the MIB.
4. **`selftest FC`**. It jogs up and back and reports PASS when the joint
   returns to within the tolerance. A FAIL points at the position loop gains
   or deadzone on the controller, which are tuned in Motion Studio or on the
   debugger's MCP266 / CAN page.
5. **Calibrate.** `cal FC on` lifts the clamp. Walk the joint to one
   mechanical end with `dec FC`, read `pos FC`, then to the other with
   `inc FC` and read again. `range FC <min> <max>` installs the range and
   saves it to NVS; it is loaded at every later boot, by this example and by
   the firmware. `cal FC off` restores the clamp. For the carriages the ends
   are the limit switches: the controller stops on them, and the count it
   reports there is the limit.

## What the example does not cover

The BSP wraps the same bring-up in `init_actuators()`, and the firmware's
state machine will drive the actuators from its own task. This example talks
to the actuators directly from the console task, which is fine on the bench
because nothing else needs the bus.
