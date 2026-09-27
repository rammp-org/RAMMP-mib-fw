# MIB Debugger

A browser-based bench tool for the RAMMP MIB. It joins the board's DDS domain
over Ethernet, stands in for the joystick, and shows what the MIB sends to the
wheel controllers. Nothing in the firmware needs to change to use it.

New to the tool? [TUTORIAL.md](TUTORIAL.md) walks through it in order, from
install to programming an MCP266 over CAN.

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

## Setup

Run once:

```
./setup.sh
```

This creates `.venv` with the one dependency, CycloneDDS. Prebuilt wheels exist
for CPython 3.9 to 3.12 only, so the script picks a suitable interpreter, which
on a Mac is normally the one at `/usr/bin/python3`.

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
nothing about CANopen. The SDO client, NMT commands and the MCP266 object
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
| `TUTORIAL.md` | Step-by-step walkthrough of all three pages |
| `canopen_sdo.py` | NMT and SDO client over the bridge, frame decoding for the monitor |
| `mcp266_objects.py` | The MCP266 object table: settings, telemetry, actions |
| `fake_mib.py` | Firmware stand-in for use without hardware |

The message definitions must stay in step with
`external/rammp-rtps/components/rammp_rtps_messages`. The firmware matches
topic and type names exactly, and every enum there is one byte wide, so those
fields are declared as `uint8` here rather than as IDL enums.
