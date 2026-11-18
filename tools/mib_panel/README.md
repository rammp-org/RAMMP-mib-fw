# MIB control panel

A browser control panel for the MIB: status, live plots, PACE RACER speed-loop
tuning (by hand and auto-tune), test drives of single motors, driving from a
pad, seat actuator targets, every telemetry channel the MIB declares, and its
log. It is a static page (`index.html` plus its scripts) with no build step and
no server.

## Connect

**Ethernet (the default).** Plug the laptop into the chair's Ethernet switch (the MIB
is the DHCP server, at 192.168.4.1), then run the bridge and open the page it serves:

```
python3 tools/mib_panel/eth_bridge.py        # http://localhost:8765/
```

Press **Connect**. A browser cannot open a TCP socket, so `eth_bridge.py` (standard
library only, any OS) relays a WebSocket to the MIB's service stream on tcp :5555,
bound to the laptop interface on the MIB's subnet so Wi-Fi or a VPN cannot take the
traffic. It is the PACE RACER's `atos-drive/tools/eth_bridge.py`, made
self-contained; the MIB's side follows that board's `EthLink`: one client at a
time, and a newcomer replaces a stale session. The copy on
<https://rammp-org.github.io/RAMMP-mib-fw/> can use a running bridge too: it
connects to `ws://localhost:8765` (Chrome may ask to allow access to devices on
the local network). `--target` and `--port` change the MIB address and the local
port.

**Web Serial (USB-C).** No bridge: open the page in desktop Chrome or Edge, pick
**Web Serial (USB-C)**, and choose the CH343 port. The USB-C is a CH343P
USB-UART bridge on the console UART, not native USB. The MIB prints the panel's
link on its console at boot and every 30 s until a panel connects. While
connected the panel runs the console at 921600 baud (50 Hz telemetry); the MIB
returns to 115200 when the panel disconnects or goes quiet for 3 s, so
`idf.py monitor` and flashing are unchanged. If the MIB reboots under the panel,
the panel drops back to 115200 and asks again. The board's DTR/RTS auto-reset
lines are held deasserted; **Reset MIB** pulses them the way `idf.py monitor` does.

| Link | Cable | Browser | Notes |
| --- | --- | --- | --- |
| Ethernet | the chair's switch | any, with `eth_bridge.py` on the laptop | The default. Also reaches the MIB with its USB-C sealed inside the chair. |
| Web Serial | USB-C | desktop Chrome / Edge | No install. Untested on phones. |
| WebUSB | the P4's USB 2.0 HS port, **J3** (4-pin header: 1 VCC_5V, 2 D−, 3 D+, 4 GND) | Chrome / Edge, Chrome for Android | Chrome offers the page when it is plugged in. Needs a J3-to-USB cable; wire D−, D+ and GND and leave pin 1 open unless the MIB is powered only from that cable. Not tried on hardware. |
| Simulated MIB | none | any | An in-page MIB with two modelled drive wheels (`mib_sim.js`). |

Hosting: the `control_panel_pages.yml` workflow publishes this folder to GitHub
Pages on every push to `main` that touches `tools/mib_panel/` (the Pages
environment deploys from `main` only).

## What it does

- **Overview**: MIB state, e-stop (engage from the header or with Esc; hold
  2 s to start a clear attempt), drive profile, network, and each PACE RACER's
  last `MotorState`: link age, board state, fault (with a clear-fault button),
  speed, command, current, bus voltage, temperature, gains in force, firmware
  and motor API version.
- **Motor tuning**: send `MotorTuning` (speed k<sub>p</sub>, k<sub>i</sub>, hall PLL,
  optionally saved to the board's NVS) and watch the board echo it. Hold-to-run
  test commands in velocity, torque, position or hold mode, and step tests
  with rise time, overshoot, settling, steady-state error and IAE.
- **Auto-tune**: the PACE dyno's speed-loop pole placement
  (`test-bench/web/characterize.html`, `stepSg`), on the chair. It measures the
  drag current at half the step speed, then the plant gain K = α / i<sub>q</sub>
  from a torque step (rescaled so the spin-up spans about 0.6 s of 50 Hz
  `MotorState`), sets k<sub>p</sub> = 2ω<sub>n</sub>/K and k<sub>i</sub> = ω<sub>n</sub>²/K for each
  ω<sub>n</sub> tried, verifies each with a speed step, and keeps the fastest one
  under the overshoot limit. Gains stay live until **Save to board**; an abort
  restores the previous ones. On the ground the step directions alternate so
  the chair doesn't keep creeping one way.
- **Drive**: enable or disable drive with a profile (the HMI's
  `DriveCommand`) and drive from a pad. The pad replaces the HMI's stick
  while held; let go and the MIB brakes to a stop within 300 ms. In
  AUTONOMOUS, touching the pad ends autonomy (back to IDLE) as the HMI's
  stick does.
- **Seat actuators**: the leg actuators the MIB drives over CAN (MCP266s, in
  encoder counts) and the MIB **poses**. A pose is one entry per leg from
  `config/poses.json` in the repo (Home is the first; `eth_bridge.py` serves it
  and the Pages workflow copies it beside the hosted page): a count to move to,
  or a delta to move by from where the leg is, delta 0 meaning the leg stays.
  **Go to Home** sends each leg of the pose its move, one `LEG_MOVE` per leg; the MIB takes a leg
  move only when IDLE with the e-stop clear, and stops (disables) a moving leg
  within one 10 ms tick of leaving IDLE or of the e-stop latching; nothing resumes
  afterwards. Only commissioned legs (`MIBconfig.hpp`) are moved; the others are
  shown and refused. A pose is a reference position and is shown
  read-only; **Hold to edit poses** unlocks it for a hardware or calibration
  change (set it from the legs' current positions or type the counts) and the
  same button locks it again. The edit lives only on the page: **Export
  poses.json** downloads the file to replace `config/poses.json` with and
  commit, which is what every panel shares. Below that, targets for
  the four seat axes (rammp-rtps seat table), which the MIB accepts and reports
  but does not yet map onto the legs.
- **Calibration**: each leg's **range**, the calibrated travel in encoder
  counts that the MIB clamps it to. **Lift clamp** sends `LEG_CALIBRATE`,
  the actuator's calibration mode, in which jogs may pass the current range;
  **holding** a jog button sends `LEG_JOG` by the step every 200 ms, each from
  where the leg's profile is then (so the leg stops within a step if the page
  goes quiet), and the release sends a jog of 0, which brings it to rest at its
  deceleration. Drive the
  leg to each mechanical end, take the position with ← into min or max, and
  **Set range** sends `LEG_RANGE`: the MIB clamps the leg to it, writes it to
  the controller and saves it in NVS, which overrides the compiled default in
  `MIBconfig.hpp` from then on. Restore the clamp when done; every boot
  restores it anyway. Jogs, like moves, are taken only in IDLE with the
  e-stop clear, and with the clamp lifted nothing limits travel but the
  controller's current limit and any limit switches.
- **Sensors and data**: every telemetry channel, with live values, plots of
  any six, and CSV export. Channels named `sensor.*` get a tile. Today that is
  the P4's die temperature; a sensor added to the firmware shows up by adding
  a `sensor.<name>` channel in `MibSystem::start_service_link()` and its value
  in `emit_telemetry()`.
- **Log**: the MIB's console, filterable and downloadable.

## Firmware updates

The **Firmware** tab flashes the MIB and the PACE RACERs over the air, so neither needs
its USB port for updates. It is the PACE RACER dashboard's flow over espp's `OtaService`;
both boards serve it.

```
python3 tools/mib_panel/eth_bridge.py --racer <pace-racer-fw>/components/pace-racer-board/atos-drive
```

1. **Target:**
   - **MIB** goes over the link you're connected with.
   - **PACE RACER:** press **Find RACERs**. The bridge broadcasts the RACERs' `PACE?`
     discovery on udp :5556; each answers with its axis, firmware and slot. Or type a
     board's IP. A RACER goes over Ethernet, through the bridge to its tcp :5555.
2. **Build:** the bridge lists every `build*/` with a `flasher_args.json`: this repo's, and
   with `--racer`, the RACER app's (`build-hub20`, `build-flatbot`, ...). The list shows only
   builds for the chosen target. You can also pick or drop a `.bin`.
3. **Flash:** the board makes itself safe, writes its other app slot and reboots into it.
   - The MIB disables drive, drops panel overrides and commands every motor DISARMED.
   - A RACER unloads its drive.
   - The panel refuses to flash a RACER while the MIB is driving.
   - Then the panel reconnects and marks the new image valid.
   - Unconfirmed, a MIB image confirms itself 20 s after RTPS comes up. A RACER's confirms
     once it holds a lease and serves its stream.
   - An image that never gets that far loses to the previous one at the next reset.
4. **Running image** reads the target's firmware, slot and state. **Mark valid** and
   **Roll back** act on the target.

The page refuses a file that isn't an ESP app image, and asks before flashing an image
built for the other board (a RACER image to the MIB, or the reverse). In the simulated
tests a 1 MB image wrote in a few seconds over localhost TCP; over real Ethernet and at
921600 baud it is not yet measured.

**Once, over USB:** the MIB's two-slot partition table and rollback bootloader have to
reach it once. Flash this firmware with `rm sdkconfig && idf.py build flash` over the USB-C
one last time; every update after that goes through the panel. A RACER's first
ethernet-capable image goes over USB too (`tools/provision.py` in pace-racer-fw).

## Safety

**Bench mode is on for now:** `estop_watch_hmi = false` in `MIBconfig.hpp`. The e-stop
latch doesn't watch the HMI as a deadman, and the MIB clears its boot latch by itself,
so it reaches IDLE with no HMI. The panel's E-STOP button and an HMI's e-stop button
still latch it, which puts the MIB in ESTOP. After a 3 s hold the MIB asks the latch to
clear, and keeps asking while a switch is held, so a panel E-STOP is a 3 s stop; drive
then has to be re-enabled. "Hold to clear" clears it sooner. Set it back to `true` for
the chair.

Panel motor commands are overrides the MIB applies only while it is IDLE with
the e-stop clear, only for 300 ms after the last one (the panel resends every
50 ms while a button is held), and capped at 8 rad/s and 20 N·m
(`MIBconfig.hpp`). Drive cannot be enabled while an override is active. The
panel stops every override when its tab is hidden, the window loses focus
during a pad drive, or the MIB refuses a command. None of this replaces a
person on the e-stop.

## Protocol

espp `stream_frame` frames on any of the links (TCP :5555, the UART, WebUSB), routed by module: telemetry (3, espp `Telemetry`),
OTA (0), console log (0x10), system (0x20: reset, info, UART rate), discovery (0xFF) and the MIB
control module (0x40), documented in
`components/MibSystem/include/mib_panel_protocol.hpp`. `mib_link.js` mirrors
the numbers.

## Files

| file | role |
| --- | --- |
| `index.html` | layout and styles |
| `panel.js` | the UI: status, charts, tuning, auto-tune, drive, seat, sensors, log |
| `mib_link.js` | frame codec, Ethernet / Web Serial / WebUSB / simulator transports, telemetry store, MIB requests |
| `eth_bridge.py` | serves the panel, lists firmware builds (`--racer` adds the RACER's), finds RACERs (`PACE?`), and relays the WebSocket to a board's tcp :5555 |
| `mib_sim.js` | the simulated MIB |
| `ax-tokens.css`, `ax-fx.js` | alex-style tokens and effects, vendored |
| `uPlot.iife.min.js` | uPlot 1.6.31 (MIT), vendored from espp `components/telemetry/web` |
