# MIB Debugger

A browser-based bench tool for the RAMMP MIB. It joins the board's DDS domain
over Ethernet, stands in for the joystick, and shows what the MIB sends to the
wheel controllers. Nothing in the firmware needs to change to use it.

New to the tool? [SETUP.md](SETUP.md) covers the install on each platform, and
[TUTORIAL.md](TUTORIAL.md) walks through the pages in order, from the first
connection to programming an MCP266 over CAN.

This tool lives on the `feature/21-mib-debugger` branch, which is not merged
to `main`. Anything it reveals that `main` should fix is tracked in
[`BRANCH_NOTES.md`](../../BRANCH_NOTES.md) at the repository root.

## What it does

- Shows the board's live system state, drive profile, seat state and status age.
- Moves the board between IDLE and DRIVE_ENABLED, and selects the drive profile.
- Drives with an on-screen joystick, hold-to-drive presets, or latched values.
- Shows the velocity command the MIB publishes for each wheel at 20 Hz.
- Sends absolute seat targets on all four axes.
- Optionally simulates the PACE RACER motor boards so the MIB sees them present.
- A STOP button releases the stick and sends drive DISABLE.
- Programs the Basicmicro MCP266 motor controllers over CAN: reads and writes
  their settings (PID gains, QPPS, voltage and current limits, encoder modes),
  shows their telemetry, saves to EEPROM, and sniffs the bus.
- Tunes the PACE RACER drive boards' speed loop (kp, ki, hall PLL) live over
  the motor API, with a graph of the commanded velocity against what each
  wheel's encoder reports, and step-response numbers to judge a change by.
- Moves the seat actuators: jogs each MCP266 channel, sends it to an absolute
  encoder count, and installs each joint's travel limits on the controller
  itself, with the CiA 402 sequences the firmware's Actuator class uses.

## Setup

Run once:

```
./setup.sh
```

This creates `.venv` with the one dependency, CycloneDDS, using a CPython 3.9
or 3.10 (the versions with prebuilt wheels). On Windows run `setup.cmd` from
PowerShell or cmd instead. [SETUP.md](SETUP.md) has the per-platform details
for macOS, Linux, WSL and Windows, including firewall rules and what to check
when the tool stays on "searching".

## Starting the tool

```
./run.sh
```

A browser window opens at http://127.0.0.1:8712/. The page starts disconnected.
The Connection panel at the top chooses a target, Board or Simulator, and an
interface, then Connect creates the DDS participant and Disconnect tears it
down. Everything below the Connection panel is greyed out until connected.

To skip the button, pass `--connect`, optionally with `--local` or
`--interface en5` to preselect the target.

## Running against the board

Connect the board's Ethernet port to your computer and let it hand you an
address. The firmware runs a DHCP server at 192.168.4.1. Choose Board, leave
the interface on Auto-detect, and press Connect. Auto-detect picks the
interface holding a 192.168.4.x address; the dropdown lists every interface
with an address if you would rather choose. The connection pill in the header
turns green once status samples arrive.
If it does not, `./doctor.sh` (`doctor.cmd` on Windows) runs the connection
checks one by one and names the failing one; see [SETUP.md](SETUP.md).

On Windows the dropdown shows the adapters' friendly names and the tool hands
CycloneDDS the adapter's address; `run.cmd` replaces `run.sh`. Allow Python
through Windows Firewall on both private and public networks when asked.
[SETUP.md](SETUP.md) has the details.

If the terminal prints lines like

```
recvUC: malformed packet received from vendor 13.37 length 48 state parse:shortmsg
```

the boards' firmware predates the CycloneDDS interop fix. Vendor 13.37 is
espp's RTPS stack on the MIB and the PACE RACER boards; its discovery
handshake has four defects that only show against CycloneDDS, and with them
the tool stays on "searching" for ever. The fix is an espp patch in
`interop/`, needed on every board on the chair; `BRANCH_NOTES.md` finding 10
has the analysis and `./doctor.sh --trace` shows which step is failing.

## Running without the board

A stand-in for the firmware lets you exercise the whole tool on a laptop. In one
terminal:

```
./sim.sh
```

Then in the page choose Simulator and press Connect. Or from the command line:

```
./run.sh --local --connect
```

The simulator mirrors the firmware's topics, state machine and guards, using the
geometry and profile limits from `MIBconfig.hpp`. It approximates the trajectory
planner with a plain acceleration limit and does not model the seat.

## Parameters page

The Parameters link in the header opens a second page listing every runtime
parameter the firmware exposes: wheel inversion, wheel geometry, the joystick X
sign, and the velocity and acceleration limits of the three drive profiles. Each
row shows the compiled-in default, the value the board currently reports, and a
control to set a new one. A write is validated on the board, applied at once,
and confirmed in the next parameter table it publishes; the header shows
whether the last write was applied or rejected.

Values are not persisted. A reboot restores the compiled defaults, and a
"Restore all firmware defaults" button does the same without a reboot. Once a
value is settled, promote it into `MIBconfig.hpp`.

This page depends on firmware support: the `mib::Params` store and the
`rammp/joystick/param_set` and `rammp/mib/params` topics defined in
`components/MIBConfig/include/mib_params.hpp`. A board running firmware without
them shows "no parameter table yet" and the page stays locked. The parameter
table in `mib_messages.py` must be kept in step with `MIB_PARAM_TABLE` in that
header, row for row; the page reports a mismatch if the counts differ.

## MCP266 / CAN page

The MCP266 / CAN link in the header opens a page that talks CANopen to the
Basicmicro MCP266 motor controllers through the MIB. The firmware on this
branch runs a raw CAN bridge (`components/MIBConfig/include/mib_can_bridge.hpp`):
frames written to `rammp/joystick/can_tx` go out on the MIB's TWAI peripheral
untouched, every frame the bus delivers comes back on `rammp/mib/can_rx`, and
`rammp/mib/can_status` reports bridge health twice per second. The MIB knows
nothing about CANopen. The two extra publishers push the MIB past the
embedded RTPS profile's four best-effort writers, so `sdkconfig.defaults`
raises `CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS` to 8; a boot log saying
"BEST_EFFORT writer capacity reached" means a build without it. The SDO client, NMT commands and the MCP266 object
dictionary live in `canopen_sdo.py` and `mcp266_objects.py`, so they can change
without a reflash.

Wiring and defaults, all taken from espp's MCP266 example: TWAI TX on GPIO 17,
RX on GPIO 16, 1 Mbit/s, node id 10. Pins and bit rate are constants in
`mib_can_bridge.hpp`; the node id is set on the page.

The MCP266 mirrors its packet-serial command set into the CANopen manufacturer
region at index `0x2000 + command number`. A command with one value sits at
subindex 0, a command with several at subindices 1..N in the command's own
order, which differs between a setter and its readback (the position PID is
written D, P, I and read back P, I, D). The object table carries both addresses
per field and does the remap. espp/mcp266 verified this mapping on hardware for
the position PID, main battery, temperature and e-stop reset; every other row
is marked **unverified** on the page and follows the same rule from the
Basicmicro manual. When a guess is wrong the controller answers with an SDO
abort and the page shows its text, which is how to find out.

What the page offers:

- **Bridge**: TWAI state, bit rate, pins and counters from CanStatus, so a
  missing transceiver or a bit-rate mismatch shows as tx failures rather than
  silence. Node id, NMT start / pre-op / stop / reset.
- **Device**: CiA 301 identity and the firmware version string.
- **Telemetry**: battery voltages, temperatures, currents, PWM, encoders,
  speeds, status flags, and the CiA 402 statusword per axis, each group with a
  one-shot Read or a 1 Hz auto refresh.
- **Settings**: velocity PID and QPPS, position PID with its min/max clamp,
  battery voltage limits, current limits, duty acceleration, encoder modes, PWM
  mode, pin functions, deadband, and set-encoder. Read fetches the live values,
  Write sends only the fields you changed and reads the group back.
- **Actions**: reset e-stop, zero encoders, save to EEPROM, restore factory
  defaults. The MCP reverts to EEPROM at power-up, so nothing written above
  survives a power cycle until it is saved.
- **Raw SDO** for any object on any node, and a raw frame sender.
- **Bus monitor** with CANopen decoding of every frame in both directions.

Two things to know from espp's testing of the MCP266 over CANopen: only profile
position mode produces motion (the velocity and duty commands are accepted but
inert), and the EEPROM save over CAN is untested. If saving aborts, set the
values in Basicmicro Motion Studio over USB and use this page for the rest.

The simulator carries a fake MCP266 on node 10 with plausible defaults, a
pretend EEPROM (save stores the live values, NMT reset node restores them) and
the same field-order quirk, so the whole page can be exercised off the bench.

## PACE tuning page

The PACE tuning link opens a page for the two PACE RACER drive boards. It
needs no firmware change on the MIB and no reflash: the boards take their
speed-loop gains on the motor API's `rammp/mib/motor_tuning/<axis>` topic
and report everything on `rammp/<axis>/motor_state`, and both are plain
best-effort DDS topics on the chair's Ethernet. The debugger joins the same
domain the MIB and the boards share, so it reads the boards' state and writes
their gains directly. The MIB's only part is to keep streaming MotorCommand,
which is the target the boards track. The `feature/tune-pace-parameters`
branch does the same thing from a serial console on the MIB (`set`, `save`,
`clr`, `tune`); this page is the same requests from the browser, with a graph.

Topics, from `motor_message.hpp` (RAMMP motor API version 3):

| direction | topic | what |
|---|---|---|
| MIB to board | `rammp/mib/motor_command/<axis>` | the velocity target, 20 Hz; the dashed line |
| board to everyone | `rammp/<axis>/motor_state` | velocity from the hall PLL, iq, state, fault, gains in force, acks; the solid line |
| debugger to board | `rammp/mib/motor_tuning/<axis>` | `speed_kp`, `speed_ki`, `hall_pll_hz`, `persist`, one request at a time |

What the page offers:

- **Drive boards**: for each wheel, state and mode, fault, API and firmware
  version, command age, velocity, iq and torque, bus, temperatures, the limits
  in force, the gains in force, and the tuning and clear-fault acks. A warning
  line when the MIB is not streaming to the board or its API is too old.
- **Speed-loop gains**: kp (A per rad/s), ki (A per rad) and PLL (Hz) per wheel,
  seeded from the board, with a "same for both" option. Apply sends the values
  once and the page waits for the board's `tuning_ack`; the request shows as
  applied, pending or not acked. Enter 0 to switch kp or ki off (sent as the
  API's negative "exactly 0", board API v3). Save writes the gains in force
  to the board's NVS and asks for a second click.
- **Step test**: profile, Enable and Disable, and buttons that latch the stick
  at 25, 50 or 100 % forward, 50 % reverse, a spin, or a custom value, plus
  Stop. A **pulse** latches the stick at a chosen value for a chosen time and
  the server releases it on a timer, so the target goes high, holds, and
  returns to zero without the operator: the test for judging how the wheel
  follows. The MIB's planner ramps the target at the profile's acceleration
  limit; "square it" raises the active profile's acceleration parameters to
  their maximum for the pulse and restores them a second after it ends, which
  needs this branch's runtime-parameter firmware and is greyed out otherwise.
- **Last step response**: per wheel and for the last two target changes, so a
  pulse shows both edges: the step size, 10–90 % rise time, overshoot, time
  to settle within 5 %, steady-state error and peak iq.
- **Velocity graph**: target (dashed) against measured (solid) for both
  wheels, a window from 5 to 120 s, an optional iq strip, a chassis-frame
  toggle that undoes the per-wheel inversion, pause, clear, and a CSV export
  of everything buffered (up to two and a half minutes). A dotted marker shows
  when each tuning request went out, so before and after are on one graph.

Two network findings from the boards' own bench notes apply here as well. A
switch with IGMP snooping and no querier can forward the laptop's multicast
to a board but not the board's back, so discovery fails one way and the
board never appears; a dumb switch or a direct cable avoids it. And nothing
on this page clears a latched fault, because the clear counter travels in
the MIB's MotorCommand stream: use the tune branch's `clr` or power-cycle
the board.

The simulator carries two fake drive boards with a PI speed loop on a
first-order wheel, the API's state machine and watchdog, tuning with its
ack, and a pretend NVS, so the whole page works off the bench and a gain
change visibly changes the response. Start it with `--no-boards` to leave
them out. Leave the "Simulate PACE RACER motor boards" checkbox on the
Debugger page off when the simulator's boards are running, or two writers
publish the same state.

## Actuators page

The Actuators link opens a page that positions the seat actuators through
the same CAN bridge, one MCP266 channel at a time. It is a port of
`mib::Actuator` from the `feature/2-actuator-control` branch (`actuator_ctl.py`
mirrors `components/Actuator/src/actuator.cpp` SDO for SDO), so what works
here is what the firmware will do, and it needs no firmware beyond the bridge.

- **Joints**: a list of (name, node id, channel M1/M2), each with its travel
  range in counts, jog step, motion profile and tolerance. Add and remove
  rows; the list is saved to `actuators.json` beside the tool. The defaults
  are four legs on nodes 10 and 11; put the real node ids in.
- **Jog − / Jog +** read the encoder and move one step from where the joint is.
  **Move** goes to an absolute count, clamped to the range; to min / middle /
  max are shortcuts. Every move first reads the clamp back from the
  controller, because the MCP266 reverts to its EEPROM at power-up and the
  factory clamp [0, 0] forces every target to zero: a controller that has
  reset is re-initialised before the target goes out.
- **Travel limits** writes the range to the controller as both Basicmicro's
  MinPos/MaxPos clamp (position PID record, commands 61-64, with the setter's
  D, P, I field order handled) and the CiA 402 software limits 0x607D. "← current"
  copies the live position into min or max. "Lift the clamp" is calibration
  mode: the clamp is widened so the joint can be jogged to its mechanical
  ends; absolute moves are refused until it is restored. **Save node to
  EEPROM** makes the limits survive a power cycle; until then they are
  re-written at the next move.
- **Encoder count** installs a count for the current position (commands
  22/23) with the loop taken down first, which is how an incremental encoder
  is homed by hand: drive to a known end, set the count that end should read.
- **Stop** is a CiA 402 quick stop (the drive holds), **Disable** drops the
  power stage, **Clear faults** is the fault reset plus the Basicmicro E-stop
  reset. The header's STOP quick-stops every joint this session has talked to
  as well as disabling the drive.
- A limit switch reported in the statusword refuses a further move in the
  same direction and restarts the drive before a move away, as the firmware
  does. A node that does not answer is skipped for five seconds so one absent
  controller does not stall the page.

The simulator carries two fake MCP266s (nodes 10 and 11) whose channels run a
CiA 402 state machine, a profile-position move against hard stops, the clamp,
0x607D and the set-encoder commands, so the whole page can be tried off the
chair.

## State machine coverage

The firmware exposes two transitions over the network: drive ENABLE moves IDLE
to DRIVE_ENABLED, and drive DISABLE moves it back. INIT and ERROR are entered by
the firmware itself. Nothing in the current firmware enters CALIBRATE, so no
tool can reach it until a command path is added.

The firmware's guards are worth knowing when a button seems to do nothing:

- ENABLE is only accepted from IDLE or DRIVE_ENABLED.
- Seat commands are only accepted in IDLE.
- The stick only moves the chassis in DRIVE_ENABLED. In every other state the
  firmware stops the drive controller on each sample.

## Safety

Disconnect releases the stick and sends drive DISABLE before dropping the
participant. The joystick publishes zeros whenever it is not held or latched. If the browser
stops polling for 1.5 seconds while the stick is engaged, the server releases it
on its own. Closing the tool with Ctrl-C sends a drive DISABLE first.

The motor board simulator publishes MotorState on all four axes. Leave it off if
real PACE RACER boards are on the network, otherwise two writers will disagree.

## Files

| File | Purpose |
|---|---|
| `mib_messages.py` | DDS types mirroring the C++ headers, byte for byte |
| `mib_link.py` | Participant, endpoints, state store, watchdog |
| `app.py` | JSON API and page server, connect and disconnect, standard library only |
| `static/index.html` | The main GUI |
| `static/params.html` | The Parameters page |
| `static/can.html` | The MCP266 / CAN page |
| `static/tune.html` | The PACE tuning page |
| `static/actuators.html` | The Actuators page |
| `actuator_ctl.py` | Port of `mib::Actuator`: CiA 402 enable and set-point handshake, clamp and 0x607D limits, jogs, calibration, encoder set |
| `SETUP.md` | Install and network setup for macOS, Linux, WSL and Windows |
| `TUTORIAL.md` | Step-by-step walkthrough of all five pages |
| `setup.cmd`, `run.cmd`, `sim.cmd` | Windows equivalents of the `.sh` scripts |
| `doctor.py` (`doctor.sh`, `doctor.cmd`) | Step-by-step connection check: lease, discovery multicast, DDS endpoints and samples |
| `canopen_sdo.py` | NMT and SDO client over the bridge, frame decoding for the monitor |
| `mcp266_objects.py` | The MCP266 object table: settings, telemetry, actions |
| `fake_mib.py` | Firmware stand-in for use without hardware, with two fake PACE RACER drive boards |

The message definitions must stay in step with
`external/rammp-rtps/components/rammp_rtps_messages`. The firmware matches
topic and type names exactly, and every enum there is one byte wide, so those
fields are declared as `uint8` here rather than as IDL enums.
