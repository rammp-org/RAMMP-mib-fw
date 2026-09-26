# MIB Debugger

A browser-based bench tool for the RAMMP MIB. It joins the board's DDS domain
over Ethernet, stands in for the joystick, and shows what the MIB sends to the
wheel controllers. Nothing in the firmware needs to change to use it.

## What it does

- Shows the board's live system state, drive profile, seat state and status age.
- Moves the board between IDLE and DRIVE_ENABLED, and selects the drive profile.
- Drives with an on-screen joystick, hold-to-drive presets, or latched values.
- Shows the velocity command the MIB publishes for each wheel at 20 Hz.
- Sends absolute seat targets on all four axes.
- Optionally simulates the PACE RACER motor boards so the MIB sees them present.
- A STOP button releases the stick and sends drive DISABLE.

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
| `fake_mib.py` | Firmware stand-in for use without hardware |

The message definitions must stay in step with
`external/rammp-rtps/components/rammp_rtps_messages`. The firmware matches
topic and type names exactly, and every enum there is one byte wide, so those
fields are declared as `uint8` here rather than as IDL enums.
