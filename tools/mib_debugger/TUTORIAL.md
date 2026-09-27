# MIB Debugger tutorial

A walkthrough of the bench tool, from an empty laptop to programming an MCP266
motor controller over CAN. The [README](README.md) is the reference; this is
the order to do things in the first time.

The tool has three pages, linked from every header:

| Page | URL | What it is for |
|---|---|---|
| Debugger | `/` | Connect, watch the board's state, drive it, watch what it sends to the wheels |
| Parameters | `/params` | Change geometry and profile limits on the running board without a reflash |
| MCP266 / CAN | `/can` | Read and write the MCP266 motor controllers' settings over CAN |

Every page shares the header: a connection pill, the board's state badge, and
a red **STOP** button. STOP releases the stick and sends drive DISABLE from
anywhere, so keep it in reach whenever the chassis can move.

## 1. Install and start

Once, from `tools/mib_debugger`:

```
./setup.sh
```

This makes a Python virtual environment with CycloneDDS. It needs a Python
between 3.9 and 3.12 because CycloneDDS ships no wheel for newer versions; on a
Mac `/usr/bin/python3` is normally fine.

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
machine, same guards, plus a fake MCP266 on the CAN page. In the browser choose
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

## 8. Stopping

Press **STOP**, then **Disconnect**, then Ctrl-C in the terminal. Each step
sends drive DISABLE on its own, so the order only matters for tidiness.
