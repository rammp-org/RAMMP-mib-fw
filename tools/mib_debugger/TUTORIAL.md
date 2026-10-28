# MIB Debugger tutorial

A walkthrough of the bench tool, from an empty laptop to programming an MCP266
motor controller over CAN. The [README](README.md) is the reference; this is
the order to do things in the first time.

The tool has five pages, linked from every header:

| Page | URL | What it is for |
|---|---|---|
| Debugger | `/` | Connect, watch the board's state, drive it, watch what it sends to the wheels |
| Parameters | `/params` | Change geometry and profile limits on the running board without a reflash |
| MCP266 / CAN | `/can` | Read and write the MCP266 motor controllers' settings over CAN |
| PACE tuning | `/tune` | Tune the PACE RACER drive boards' speed loop and watch target against measured velocity |
| Actuators | `/actuators` | Jog the seat actuators, move them to absolute counts, and set their travel limits on the MCP266 |

Every page shares the header: a connection pill, the board's state badge, and
a red **STOP** button. STOP releases the stick and sends drive DISABLE from
anywhere, so keep it in reach whenever the chassis can move.

## 1. Install and start

Once, from `tools/mib_debugger`:

```
./setup.sh
```

This makes a Python virtual environment with CycloneDDS. It needs Python 3.9
or 3.10; on a Mac `/usr/bin/python3` is normally fine. On Windows run
`setup.cmd` from PowerShell or cmd, and later `run.cmd` and `sim.cmd`.
[SETUP.md](SETUP.md) has the per-platform details and the firewall rules.

Every time:

```
./run.sh
```

A browser opens at http://127.0.0.1:8712/. The page starts disconnected and
everything below the Connection panel is greyed out until you connect.

## 2. Learn the tool on the simulator first

You do not need the board to learn the pages. In a second terminal:

```
./sim.sh
```

That starts a stand-in for the firmware on loopback: same topics, same state
machine, same guards, plus two fake MCP266s for the CAN and Actuators pages. In the browser choose
**Simulator** as the target and press **Connect**. The header pill turns green
within a second or two and the state badge shows **IDLE**.

Everything in the rest of this tutorial works the same against the simulator,
which is the safe place to try Enable, the joystick and the CAN writes before
the real chassis is involved.

## 3. Connect to the board

Plug the board's Ethernet port into the laptop. The firmware runs a DHCP server
at 192.168.4.1 and hands the laptop an address on 192.168.4.x. Then:

1. Target: **Board**.
2. Interface: leave **Auto-detect**. It picks the interface holding a
   192.168.4.x address. The dropdown lists every interface with an address and
   marks the one on the board's subnet if you would rather choose.
3. **Connect**.

The Board panel shows status samples counting up twice per second. If the pill
stays on "searching" with no samples:

- check the interface dropdown shows an entry marked "(board subnet)"; if not,
  the laptop has no lease and the cable or DHCP is the problem,
- press **Disconnect**, pick the interface explicitly, and connect again.
  CycloneDDS chooses its multicast interface when the participant is created,
  which is why Disconnect and Connect are the way to change it.

**Disconnect** is safe at any time. It releases the stick and sends drive
DISABLE before it drops the participant.

## 4. Read the Debugger page

With the board connected, the page is in four parts.

**System state** shows the firmware's state machine as chips. Solid chips are
the ones you can reach from the network: click **IDLE** to send DISABLE, click
**DRIVE_ENABLED** to send ENABLE. INIT and ERROR are entered by the firmware
itself, and nothing in the current firmware enters CALIBRATE. The profile
buttons **LOW / NORMAL / HIGH** pick the velocity and acceleration limits; the
choice applies on the next Enable, or at once while already enabled.

**Board** shows how fresh the last status sample is. An age above two seconds
counts as a lost board. The **Simulate PACE RACER motor boards** checkbox makes
the tool publish MotorState on all four axes so the MIB sees wheel controllers
present. Leave it off when real PACE RACER boards are on the network, otherwise
two writers disagree.

**Drive** is the joystick. Nothing you do here moves the chassis unless the
state is DRIVE_ENABLED; in every other state the firmware stops the drive
controller on each sample.

**Motor commands from the MIB** shows what the MIB publishes to the two wheel
controllers at 20 Hz. **Chassis frame** turns the two shaft velocities back
into linear and angular motion and says in words what the chassis would do,
for example "forward, turning left (CCW)". **Motor shaft (raw)** shows the
per-axis rad/s exactly as sent, which is what a PACE RACER board sees.

**Seat** sends absolute targets on the four seat axes. The firmware accepts
them only in IDLE, stores the value and echoes it in its status; no actuator
moves yet.

**Events** is a log of everything the tool sent and every state change it saw.
When a button seems to do nothing, the answer is usually here, because the
firmware's guards are strict: Enable is only accepted from IDLE or
DRIVE_ENABLED, seat commands only in IDLE.

## 5. Drive from the bench

Do this with the wheels off the ground the first time.

1. Pick a profile. **LOW** caps linear speed at 0.35 m/s.
2. Press **Enable drive**. The badge turns to DRIVE_ENABLED and the Motor
   commands panel starts showing ARMED on both drive axes.
3. Hold the round pad with the mouse and move it. y is linear, x is angular.
   Releasing the mouse sends zero. The **▲ Forward**, **▼ Reverse**, **↺ Spin L**,
   **↻ Spin R** and **Creep** presets work the same way: they drive while held
   and stop when released.
4. Watch the Chassis frame readout. Push the pad to the right and read the
   Motion line. On current firmware it reports "turning left (CCW)", which is
   the joystick sign error tracked as finding 1 in `BRANCH_NOTES.md`.
5. **Latch current values** keeps publishing whatever the readout shows
   without holding anything, for measurements. **Release** or **STOP** ends it.
   If the browser stops polling for 1.5 s while latched, the tool releases the
   stick on its own.
6. **Disable drive** or **STOP** when done.

## 6. Tune with the Parameters page

Open **Parameters →** in the header. Every row is one value the firmware
normally compiles in: wheel inversion, wheel diameter and separation, the
joystick X sign, and the velocity and acceleration limits of the three
profiles. Each row shows the compiled default, what the board currently
reports, and an input.

Type a value and press **Set** (or Enter). The board validates it against the
row's range, applies it at once, and echoes the whole table; the header shows
whether the last write was applied or rejected. Geometry changes affect the
wheel commands immediately, so you can watch the Motor commands panel on the
Debugger page respond.

To test the turn-direction fix: set **Joystick X sign** to -1, go back to the
Debugger page, push the pad right, and the Motion line now says "turning right
(CW)".

Nothing here survives a reboot. **Restore all firmware defaults** puts the
table back without one. When a value is settled, promote it into
`MIBconfig.hpp`.

## 7. Program the MCP266s over CAN

Open **MCP266 / CAN →**. The MIB forwards raw CAN frames between the network
and its TWAI peripheral; everything CANopen happens in the tool.

### Check the bridge and find the controller

The **Bridge** panel is the first thing to read. TWAI must say "up, enabled"
and the bus state "active". The pins and bit rate shown are what the firmware
was built with (TX 17, RX 16, 1 Mbit/s). Then press **Read** in the **Device**
panel. A device name and firmware version string coming back proves the
transceiver wiring, the bit rate and the node id in one shot.

If nothing comes back:

- **TX failed** counting up while TX ok stays at zero means no node
  acknowledged the frame: wrong bit rate, no termination, a swapped TX/RX, or
  the controller not powered.
- TX ok counting up but every read timing out means the frames go out and
  something acknowledges them, but not the node you addressed. Change **Node
  id** and **Apply**, then read Device again. The header's heartbeat pill
  shows any node that is sending heartbeats and its NMT state.
- Look at the **Bus monitor** at the bottom. It shows every frame both ways
  with a decoded meaning, so a reply from an unexpected node is visible.

The MCP266 answers SDO reads and writes in pre-operational, so the NMT buttons
are not needed for settings. **Reset node** makes the controller reload its
EEPROM, which is the quick way to discard unsaved changes.

### Read before you write

Each **Settings** group has a **Read** button. Press it and the live column
fills in and the inputs are seeded with the same values. The address under
each field name is where it reads from and writes to, in the form
`0x2000 + packet-serial command number : subindex`.

Two badges matter:

- **verified**: espp confirmed this object mapping on real hardware.
- **unverified**: the row follows the same rule from the Basicmicro manual and
  has not been tried. If the guess is wrong the controller answers with an SDO
  abort, and the field shows its text, such as "object does not exist" or
  "subindex does not exist". That is the signal to correct the row in
  `mcp266_objects.py`, and once a row works on the bench, mark it verified
  there.

### Change a setting

The velocity loop's ceiling is **QPPS (max speed)** in the Velocity PID group,
the encoder counts per second at full power:

1. Press **Read** on Velocity PID M1.
2. Type the new QPPS. The input turns amber to show it differs from the live
   value.
3. Press **Write changed** (or Enter). Only the amber fields are sent, then the
   group is read back and the live column updates. The Events log records each
   write with the raw integer the controller received.

Gains are entered in the units Motion Studio shows. The tool converts to the
controller's fixed point (16.16 for the velocity loop, ×1024 for the position
loop). The position PID's **Min position / Max position** clamp is worth
knowing: it ships as [0, 0], which forces every position target to zero.

### Make it stick

The MCP266 reverts to its EEPROM at every power-up. Under **Actions**, **Save
settings to EEPROM** sends the WRITENVM command with its magic word. It asks
for confirmation because it is permanent. If it aborts, the mirror of that
command is not available over CAN on your firmware; set the values in
Motion Studio over USB and use this page for everything else.

**Restore factory defaults** also asks first. It wipes every setting on the
controller including what Motion Studio set.

### Telemetry and anything else

The **Telemetry** groups (battery voltages, temperatures, currents, PWM,
encoders, speeds, status flags, the CiA 402 statusword per axis) read on
demand or with **auto 1 Hz** ticked. Status flags and the statusword are
decoded into words.

**Raw SDO** reads or writes any object on the current node: index in hex,
subindex, size, and a signed flag. Choose "auto/string" as the size to read a
string object. The **Raw frame** line sends any CAN frame, for example NMT to
another node.

Two limits from espp's testing of the MCP266 over CANopen: only profile
position mode produces motion (the velocity and duty commands are accepted but
inert), and this page does not drive the motors. It configures them.

## 8. Tune the PACE RACER speed loop

Open **PACE tuning →**. This page talks to the two drive boards directly over
the same Ethernet, so it works with the MIB firmware you already have as long
as the MIB is driving. Wheels off the ground.

### See both boards

The **Drive boards** cards fill in as soon as a board's MotorState arrives,
armed or not. Check three things: the API version is 3 (2 works, but a gain
cannot be switched off), the command age is under 100 ms once drive is
enabled (the MIB is streaming to it), and the state is ARMED while driving.
A card that stays "no MotorState" means the board is off, has no axis set on
its console, or the switch is not passing its multicast to the laptop.

### Take a baseline

1. Pick **NORMAL** and press **Enable drive**. Both cards show ARMED.
2. Set the pulse to 50 % for 2 s, leave **square it** ticked, and press
   **⎍ Run pulse**. The stick latches, the chassis drives forward for two
   seconds, and the server lets go: the dashed target goes up, holds, and
   comes back to zero, and the solid measured line shows how each wheel
   followed. The countdown is under the buttons.
3. Read the **Last step response** table. It has two rows per wheel, the
   rising edge (▲) and the falling edge (▼): rise time, overshoot, settle
   time, steady error and peak current.

The step buttons do the same without the timer: **▲ 50 %** latches the stick
until **■ Stop**. Without "square it" the MIB's planner ramps the target at
the profile's acceleration limit, which is the chair's normal feel; with it
the ramp is as steep as the firmware allows, which is the harder test for the
loop. Repeat a pulse or two until the numbers are steady; that is the
baseline. Set the window to 30 s so a whole pulse fits.

### Change a gain and compare

The inputs show what each board is running. With **same values for both
wheels** ticked, typing in the left row fills the right.

1. Halve **ki** (the integrator is what bounces after a step) and press
   **Apply**. The request shows "sent, waiting for ack" and then "applied";
   the Events log prints the gains the board confirms, and a dotted green
   marker goes on the graph at that moment.
2. Run the same pulse again and compare the response left of the marker with
   the response right of it.
3. Then halve **kp**, then move the **PLL** toward 5 Hz for smoother feedback
   or toward 20 Hz if the loop hunts slowly.

Enter 0 for ki to switch the integrator off entirely; the wheel then settles
short of the target, which the steady error column shows. Press **⟲ from
board** on a row to discard what you typed and reload the board's values.
**Export CSV** saves the buffered trace for comparing runs later.

### Keep it

The board loses the gains at its next power cycle. **Save in force to NVS**
writes them; it asks for a second click because it is permanent. Each wheel
is its own board, so untick "same for both" if they need to differ.

## 9. Move the seat actuators

Open **Actuators →**. Each row in **Joints** is one channel of one MCP266:
pick the row to work on. The defaults are four legs on nodes 10 and 11; edit
the name, node id and channel under **Configuration** and press **Save**, and
the list is kept in `actuators.json` beside the tool.

Do this with nothing in the actuator's way the first time, and with the
chair's power switch in reach. Nothing here moves until you press a button,
and a move needs the drive in profile position mode with a non-zero position
P gain; the page seeds a coarse gain if the controller has none, but the real
gain is tuned on the CAN page under Position PID.

### Find the ends

1. Press **Read**. The position is the encoder count, the drive state is the
   CiA 402 state, and **Travel limits** shows the clamp the controller holds.
   A fresh controller shows `[0, 0]`, the factory clamp that forces every
   target to zero.
2. Tick **lift the clamp**. The clamp is widened so the joint can be driven
   past the range you have not measured yet; only jogs are accepted.
3. Press **Jog +** and **Jog −**. Each jog reads the encoder and moves one
   step (the step input, in counts) from where the joint is. Watch the count
   and the direction: if + runs the wrong way, the sign belongs in the
   kinematics layer, not here, so note it.
4. At one mechanical end press **← current** next to min; at the other, next
   to max. For an incremental encoder set the count first: at a known end,
   **Encoder count → Set** installs the count that end should read (the loop
   is taken down first so it does not chase its old target).
5. Untick **lift the clamp**, then **Write to controller**. The range is
   written as the MinPos/MaxPos clamp and the 0x607D software limits, read
   back, and shown with a "matches" badge.
6. **Save node to EEPROM** if the limits should survive a power cycle. It is
   permanent and covers both channels of that node. Without it the page
   re-writes the limits at the next move, so nothing is lost either way.

### Move

**Move** sends an absolute count, clamped to the range, with the profile
velocity in the inputs beside it. **to min**, **to middle** and **to max** are
shortcuts. The bar under the position shows where the joint is in its range
and a marker where the last target was. **■ Stop** is a quick stop (the drive
holds position) and the next move releases it; **Disable** drops the power
stage so the joint can be pushed by hand. If a limit switch is wired, the
statusword reports it, the page refuses another move in that direction, and
restarts the drive before a move away.

Every move reads the clamp back first. If the controller has power-cycled
since the limits were written, the Events log says "it has reset;
re-initialising" and the limits go in again before the target does.

## 10. Stopping

Press **STOP**, then **Disconnect**, then Ctrl-C in the terminal. Each step
sends drive DISABLE on its own, so the order only matters for tidiness.
