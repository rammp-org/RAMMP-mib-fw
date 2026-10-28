# MIB control panel

A browser control panel for the MIB: status, live plots, PACE RACER speed-loop
tuning (by hand and auto-tune), test drives of single motors, driving from a
pad, seat actuator targets, every telemetry channel the MIB declares, and its
log. It is a static page (`index.html` plus its scripts) with no build step and
no server.

## Connect

| Link | Cable | Browser | Notes |
| --- | --- | --- | --- |
| WebUSB | the P4's USB 2.0 HS port: **J3**, the 4-pin header | Chrome / Edge, Chrome for Android | Chrome offers the page when the MIB is plugged in. 50 Hz telemetry. |
| Web Serial | the USB-C (CH343 USB-UART, the console) | desktop Chrome / Edge | No plug-in prompt (the CH343 has fixed descriptors). Telemetry at 10 Hz on 115200 baud; the MIB's log is the stream itself. |
| Simulated MIB | none | any | An in-page MIB with two modelled drive wheels (`mib_sim.js`), for trying the panel. |

**The ESP32-P4-ETH's USB-C is not native USB.** It goes to a CH343P USB-UART
bridge (UART0) and stays the console and the flashing port. The P4's
native USB is only on J3 (pin 1 VCC_5V, 2 D−, 3 D+, 4 GND; Waveshare schematic
`ESP32-P4-ETH-datasheet.pdf`). WebUSB and the "go to this page" notification
need a cable from J3 to a USB connector, for example a panel-mount USB-C on the
MIB enclosure. Wire D−, D+ and GND; leave pin 1 unconnected unless the MIB is
powered only from that cable, so a laptop's 5 V is never tied to the MIB's own
5 V rail. Not yet tried on hardware.

Plugging J3 into a laptop: Chrome shows a notification for
`rammp-org.github.io/RAMMP-mib-fw/` (`service_landing_page_url` in
`MIBconfig.hpp`). That URL serves this folder once GitHub Pages is enabled for
the repository (Settings, Pages, Source: GitHub Actions); the
`control_panel_pages.yml` workflow deploys it on every push to `main` that
touches `tools/mib_panel/`.

Locally: `python3 -m http.server -d tools/mib_panel 8000` and open
`http://localhost:8000/`. WebUSB and Web Serial need https or localhost.

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
  from a torque step (rescaled so the spin-up spans about 0.6 s of 20 Hz
  `MotorState`), sets k<sub>p</sub> = 2ω<sub>n</sub>/K and k<sub>i</sub> = ω<sub>n</sub>²/K for each
  ω<sub>n</sub> tried, verifies each with a speed step, and keeps the fastest one
  under the overshoot limit. Gains stay live until **Save to board**; an abort
  restores the previous ones. On the ground the step directions alternate so
  the chair doesn't keep creeping one way.
- **Drive**: enable or disable drive with a profile (the HMI's
  `DriveCommand`) and drive from a pad. The pad replaces the HMI's stick
  while held; let go and the MIB brakes to a stop within 300 ms.
- **Seat actuators**: targets for the four seat axes (rammp-rtps seat table).
  The MIB accepts and reports them; it has no actuator driver yet.
- **Sensors and data**: every telemetry channel, with live values, plots of
  any six, and CSV export. Channels named `sensor.*` get a tile. Today that is
  the P4's die temperature; a sensor added to the firmware shows up by adding
  a `sensor.<name>` channel in `MibSystem::start_service_link()` and its value
  in `emit_telemetry()`.
- **Log**: the MIB's console, filterable and downloadable.

## Safety

Panel motor commands are overrides the MIB applies only while it is IDLE with
the e-stop clear, only for 300 ms after the last one (the panel resends every
50 ms while a button is held), and capped at 8 rad/s and 20 N·m
(`MIBconfig.hpp`). Drive cannot be enabled while an override is active. The
panel stops every override when its tab is hidden, the window loses focus
during a pad drive, or the MIB refuses a command. None of this replaces a
person on the e-stop.

## Protocol

espp `stream_frame` frames, routed by module: telemetry (3, espp `Telemetry`),
console log (0x10), system (0x20: reset, info), discovery (0xFF) and the MIB
control module (0x40), documented in
`components/MibSystem/include/mib_panel_protocol.hpp`. `mib_link.js` mirrors
the numbers.

## Files

| file | role |
| --- | --- |
| `index.html` | layout and styles |
| `panel.js` | the UI: status, charts, tuning, auto-tune, drive, seat, sensors, log |
| `mib_link.js` | frame codec, WebUSB / Web Serial / simulator transports, telemetry store, MIB requests |
| `mib_sim.js` | the simulated MIB |
| `ax-tokens.css`, `ax-fx.js` | alex-style tokens and effects, vendored |
| `uPlot.iife.min.js` | uPlot 1.6.31 (MIT), vendored from espp `components/telemetry/web` |
