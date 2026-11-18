# actuator_sysid firmware

A standalone MIB firmware for the actuator system-identification campaign. It logs
the BNO055 IMU and every MCP266 actuator controller on the CAN bus at a fixed rate,
as text lines on the console. The motion build adds bounded closed-loop steps over
CAN. The host side (`../host/`) captures and analyses the lines.

It talks to `espp::Mcp266` and `espp::CanopenClient` directly, not through the
`Actuator` component, so the measurement does not depend on that control logic. It
reuses `components/bsp` for the board and the IMU. The CAN bring-up is done here
(`sysid_main.cpp`, `start_can()`) because `components/bsp` on this branch has none.

## The chair's nodes

| Node | M1 (`c1`) | M2 (`c2`) | Hardware | This firmware |
|---|---|---|---|---|
| 2 | left carriage | right carriage | sliders, incremental encoders, limit switches | **read-only in every build**; never moved |
| 3 | main left leg | main right leg | electro-hydraulic actuators (EHA), not back-drivable, absolute encoders | logged; the motion build may move them (`SYSID_MOTION_NODE_IDS` = 3) |

This chair has no powered casters, so there is no node 1: the default
`SYSID_NODE_IDS` is `2,3`. Other chairs put their casters on node 1. `scan` lists
whatever answers, and `nodes 2,3,10` changes the polled set at run time. The
motion allow-list stays Kconfig-only.

### Controller facts (bench, node 3)

| Item | Value |
|---|---|
| Device | MCP266 2x60A, hardware 1.0.0, software 1.4.9 (0x1008 / 0x1009 / 0x100A) |
| Supported modes 0x6502 | 0x700: cyclic-synchronous position, velocity and torque only |
| Stored default speed / accel (0x2046 / 0x2047, 0x2044 / 0x2045) | not readable: protocol error, or abort 0x06010001 (write-only) |
| Abort-connection option 0x6007 | not supported (read fails) |
| PDO communication parameters | present, default COB-IDs (RPDO1 0x203, TPDO1 0x183 and so on); every mapping entry is 0 |
| Position PID, node 3 | M1: P 103, D 868, deadband 10, clamp [10000, 18000]. M2: P 120, D 1086, deadband 10, clamp [25000, 32000]. 0x607D matches both clamps |
| Encoder noise at rest | 50..70 counts peak to peak (sd 8) on both legs. M2 showed two ~1800-count glitches decaying over ~100 ms with the chair still |
| `_spd` (0x606C) | useless at rest: toggles 0 / ±1000 / ±2000, sometimes −2147483648. Logged only; nothing uses it. Derive speed from `_enc` |

### The clamp read bug (fixed)

The first passive `dump 3` showed the clamp as [0, 0] on its leading lines (0x203F
sub 6 and sub 7 read on their own). The same subindices read later, right after
subs 1..5 in order, gave [10000, 18000] (M2: [25000, 32000]), which 0x607D
confirms.

The width and type were right, and it was not a reply mismatch: espp checks every
reply's index:sub, and the values were exactly 0. The MCP266's mirror of the
multi-value packet-serial command 63 / 64 behaves as if it runs the command when
sub 1 is read and serves subs 2..7 from that reply. A read that starts at sub 6
gets an empty reply buffer. espp's `Mcp266::read_position_limits()` reads only
subs 6 and 7, so it has the same problem on this firmware: an upstream issue to
file.

Fix:
* There is one read path for the clamp, `McpNode::read_position_record()`: the
  whole record, subs 1..7 in order, which the log proves.
* `McpNode::read_clamp_checked()` reads the record twice and refuses if the two
  clamps differ.
* `dump`, `arm`, `clamp`, `pstep` and the stop's hold all go through it.
* The same reasoning may apply to the M2 values of 0x2030 / 0x2031 (sub 2). Each
  pass reads sub 1 first, so sub 2 comes from the reply fetched at sub 1 a few
  SDOs earlier. All channels were idle in this log, so sub 2 is unconfirmed.

Travel, from the operator's rough figures: right main leg about 10000..18000 counts,
left main leg about 17000..25000, so roughly 8000 counts each. The exact ends are
unknown, and so is caster travel; both are to be found experimentally. The bench
example that ran on this MIB wrote clamp [10000, 18000] and P = 103 to node 3 M1 at
every boot, so **the clamps on the chair cannot be trusted** until `dump` shows them
and `clamp` sets them.

## Builds

| Build | `CONFIG_SYSID_ENABLE_MOTION` | Bus access |
|---|---|---|
| passive (default) | n | SDO uploads only. The transmit filter treats **every** node as read-only, so no write can go out even if one were coded by mistake |
| motion | y | adds `arm`, `clamp`, `pstep`, `clearfaults` for nodes in `SYSID_MOTION_NODE_IDS` (default 3). Every other node, and `SYSID_READONLY_NODE_IDS` (default 2), is read-only at the transmit filter |

```console
cd tools/actuator_sysid/firmware
export PATH="/opt/homebrew/opt/python@3.11/bin:$PATH" && source ~/esp/esp-idf/export.sh

idf.py build                                              # passive -> build/
idf.py -B build_motion -DSDKCONFIG=build_motion/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.motion" build   # motion
```

For a 460800-baud console, add `sdkconfig.baud460800` to `SDKCONFIG_DEFAULTS` with
its own build dir, for example `-B build_460800` or `-B build_motion_460800`.

A Kconfig change to an existing build dir needs `reconfigure`, and a changed
*default* needs that build's `sdkconfig` deleted. Settings live under
`idf.py menuconfig` → "Actuator sysid".

Host test of the transmit filter (no ESP-IDF needed):

```console
c++ -std=c++20 -Imain test/bus_policy_test.cpp -o build/bus_policy_test && build/bus_policy_test
```

## Flash, and back to the main firmware (for a human, on the bench)

The port is the CH343 bridge on the MIB's USB-C (VID 0x1A86, PID 0x55D3).

```console
# from tools/actuator_sysid/firmware
idf.py -p /dev/cu.usbmodem5C372451831 flash                    # passive
idf.py -B build_motion -p /dev/cu.usbmodem5C372451831 flash    # motion
```

This writes four images: the bootloader at 0x2000, the partition table at 0x8000,
`ota_data_initial.bin` at 0xf000 and the app at 0x20000 (`build/flash_args`). The
partition table is the main firmware's own (`../../../partitions.csv`: nvs, otadata,
ota_0, ota_1). The fresh otadata is blank, so the bootloader boots ota_0, where the
app was written. That holds even if the main firmware had been running from ota_1
after a panel update.

The bootloader is replaced too. This project's bootloader has rollback off, so
there is nothing to confirm. NVS (0x9000) is not touched. `idf.py flash` uses
esptool's 460800 flash baud, which worked on this board's CH343; the console
baud is separate.

Back to the main firmware:

```console
# from the repo root
idf.py build && idf.py -p /dev/cu.usbmodem5C372451831 flash
```

That writes the main bootloader (rollback enabled), the same partition table, a
blank otadata and the main app to ota_0. A serially flashed image is not pending
verification, so rollback does not apply to it.

## Console and budget

UART0 (GPIO37/38) through the CH343, **921600 baud**, primary console, nothing on
the USB-Serial-JTAG port. Lines go into a 16 KB driver buffer. A line is written
only if all of it fits at that moment; otherwise the whole line is dropped and
counted (`#meta:tx_dropped_lines`). The sample loop never waits on the console.

Worst-case row with 3 nodes is **574 bytes** (`worst_case_row_bytes()` in
`sysid_main.cpp` itemises it); a typical row is about 380. 8N1 means 10 bits per
byte.

| Baud | Link | 100 Hz worst case | Rate cap (70 % of link) |
|---|---|---|---|
| 921600 (default) | 92.2 kB/s | 57.4 kB/s = 62 % | 112 Hz, so 100 Hz fits |
| 460800 (`sdkconfig.baud460800`) | 46.1 kB/s | too much | 56 Hz; the overlay boots at 50 Hz |

- `rate <hz>` refuses any rate above the cap, with an event that gives the numbers.
- A boot rate above the cap is lowered, also with an event.
- The CH343 itself goes far higher. Set `CONFIG_ESP_CONSOLE_UART_BAUDRATE` up to
  4000000 (for example 2000000), and open the host at the same rate.
- An esptool read on this board failed at 921600. If rows show up garbled or
  dropped at 921600, use the 460800 overlay.

The second-stage bootloader also prints at the console baud. The ROM's first lines
come at 115200 and read as noise; the host ignores any line that does not start
with `D,` or `#`. The app starts by writing a bare newline, so the first protocol
line always starts a line.

**Log isolation.** Every line is written whole while holding stdout's lock, which
every stdout writer takes. ESP-IDF log messages are re-emitted as `#log,<text>` at
WARN and above. espp components created here log at NONE; the bsp, board and IMU
log at ERROR after boot.

## Wire format

| Line | Meaning |
|---|---|
| `#meta:<key>=<value>` | metadata. **The first lines after boot are `#meta:mode=passive` or `#meta:mode=motion`, then `#meta:read_only_nodes=...`** (the effective set: `all (1-127)` in a passive build) and, in a motion build, `#meta:motion_nodes=...` |
| `#cols:t_us,imu_qw,...` | column names, in row order. Emitted at boot, every 10 s, on `hdr` and after `rate` |
| `D,<v>,<v>,...` | one sample; an **empty field means no data** |
| `#evt,<t_us>,<text>` | event; `<text>` runs to the end of the line |
| `#cfg,<node>,<name>,<index>,<sub>,<value>` | one object from `dump` |
| `#log,<text>` | an ESP-IDF log message |

### Columns

`t_us,imu_qw,imu_qx,imu_qy,imu_qz,imu_gx,imu_gy,imu_gz,imu_lax,imu_lay,imu_laz,imu_grx,imu_gry,imu_grz,imu_cal`
come first. Then, for each node in `SYSID_NODE_IDS` order, the 9 per-channel
columns for `n<id>c1_*`, the same 9 for `n<id>c2_*`, then `n<id>_vbat_mv` and
`n<id>_temp_dc`. The per-channel suffixes, in order, are
`_enc,_spd,_cur_ma,_duty,_cmd,_tgt,_status,_lim,_age_us`.

| Column | Unit | Source (espp 1.4.3) | Refresh |
|---|---|---|---|
| `t_us` | µs since boot | `esp_timer`, taken after the IMU read and the MCP copy | every row |
| `imu_q*` | unit quaternion | BNO055 IMU fusion mode, heading relative to power-on | every row |
| `imu_g*` | rad/s, sensor frame | `gyro_radps` | every row |
| `imu_la*`, `imu_gr*` | m/s² | linear acceleration, gravity | every row |
| `imu_cal` | `sys*64+gyro*16+accel*4+mag` | BNO055 CALIB_STAT layout | every row |
| `_enc` | counts | `Mcp266::read_encoder`, 0x6064 / 0x6864 | every pass |
| `_spd` | logged only, useless at rest (see controller facts) | `Mcp266::read_speed`, 0x606C / 0x686C | every pass |
| `_cur_ma` | mA = raw × 10 (**unconfirmed**) | `sdo_upload` 0x2031 sub 1 / 2 | every pass |
| `_duty` | ±32767 = ±100 % (**unconfirmed**) | `sdo_upload` 0x2030 sub 1 / 2 | every pass |
| `_cmd` | as `_duty` | open-loop duty commanded by this firmware: always 0, none is ever sent | every row |
| `_tgt` | counts | target of the `pstep` running on the channel; empty otherwise and always in passive | every row |
| `_status` | CiA 402 statusword, decimal | `Mcp266::read_statusword`, 0x6041 / 0x6841 | 1 pass in 5 |
| `_lim` | bits: 1 = statusword bit 11, 2 = node S4, 4 = node S5 | statusword and 0x205A (command 90) | 1 pass in 5 |
| `_age_us` | µs | `t_us` minus the midpoint of the `_enc` SDO (spd, cur and duty come from the same pass) | with `_enc` |
| `n<id>_vbat_mv` | mV | `read_main_battery_voltage`, 0x2018 | 1 pass in 5 |
| `n<id>_temp_dc` | 0.1 °C | `read_temperature`, 0x2052 | 1 pass in 5 |

The current (0x2031) and PWM (0x2030) objects are the ones the teammate read on
the bench. Their units and the one-subindex-per-motor layout come from the
Basicmicro manual and were not confirmed over CAN.

**Staleness.** Each node has its own poll task, so a silent node never holds up a
row. A pass reads 8 fast fields, then one slow field in turn.

A value is printed empty when:
* its node is offline (3 passes in a row without an answer, 20 ms SDO timeout;
  retried every 200 ms);
* the controller aborts that object (one event, then the field stays empty); or
* the value is older than 500 ms.

Metadata also carries:
* `n<id>_poll`: passes, timeouts, mean and max pass time, slowest SDO;
* `sample_overruns`, `can_bus_errors`, `blocked_frames`, `tx_dropped_lines`;
* the console baud and the rate cap.

## Commands

| Command | Build | Effect |
|---|---|---|
| `hdr` | both | re-emit metadata and `#cols` |
| `rate <hz>` | both | 1..100 Hz, refused above the console link's 70 % cap |
| `nodes [id,id,...]` | both | with a list: poll exactly those nodes (at most 8 ever; the header is re-emitted). Then `#meta:n<id>_state` for each: online, offline, not yet answered or not polled; read-only; device name |
| `scan` | both | SDO upload of 0x1000 (10 ms timeout) to every id 1..127, then `#cfg,<id>,device_type,...` and `#cfg,<id>,device_name,...` for each node that answers. Reads only |
| `mark <text>` | both | an event with your text |
| `dump <node>` | both, any node | read-only configuration dump (below) |
| `stop` | passive | **sends nothing.** Reports every channel's duty, current and statusword as last polled |
| `stop` | motion | the stop sequence on every channel of every motion node; read-only nodes only reported |
| `arm <node> <ch>` | motion | arms exactly that channel for 10 s; the event shows the clamp, 0x607D and the present count |
| `clamp <node> <ch> <min> <max>` | motion, armed channel | installs the travel window (below) |
| `pstep <node> <ch> <delta_counts>` | motion, armed channel | one CiA 402 profile-position move to present + delta |
| `clearfaults <node>` | motion | CiA 402 fault reset on both channels of the node |

`dump <node>` prints `#cfg` lines, first the decision data for each channel: the
present count, the clamp min/max and the 0x607D min/max. Then:
* device type, error register, device name, hardware and software version
  (0x1008, 0x1009, 0x100A), the Basicmicro firmware string (0x2015);
* heartbeat consumer and producer (0x1016, 0x1017), identity (0x1018);
* per channel: supported modes 0x6502, mode and mode display 0x6060 / 0x6061,
  abort-connection option 0x6007, quick-stop option 0x605A, controlword,
  statusword, 0x607D, profile velocity, acceleration and deceleration, profile type;
* the position PID record (P, I, D ×1024, max I, deadband, min/max clamp, from
  0x203F / 0x2040), the velocity PID (0x2037 / 0x2038), and the stored default
  speed and accel (0x2046 / 0x2047, 0x2044 / 0x2045 subs 1 and 2);
* status flags 0x205A, battery limits 0x203B, encoder modes 0x205B;
* RPDO and TPDO communication and mapping parameters 0x1400–0x1403,
  0x1600–0x1603, 0x1800–0x1803 and 0x1A00–0x1A03 (sub 0, then each mapped sub).

Each value is printed as one of:
* `0x00002710=10000`: the uploaded bytes in hex (the width shows the size), then
  the signed decimal;
* text, for a string object;
* `abort:0x06020000`: a valid result, meaning no such object;
* `error:...`.

## What goes on the bus

All SDO traffic is request on 0x600+N and reply on 0x580+N; the frames below are
the requests. "Upload" is a read (byte 0 = 0x40), "download" is a write. Objects
are shown for M1; M2 uses 0x68xx for the CiA 402 objects, and 0x2040 / 0x203E for
the PID record.

**At boot (both builds):** SDO uploads only. Nothing else, including no NMT, no
heartbeat and no SYNC.
* The poll tasks read 0x6064, 0x606C, 0x2031, 0x2030, 0x6041, 0x205A, 0x2018
  and 0x2052.
* The device name is read once (0x1000, then 0x1008 with upload-segment
  requests 0x60 / 0x70).

**Passive build:** only that, ever. espp calls on the path:
* `CanopenClient` and `Mcp266` constructors: no traffic.
* `Twai::initialize`: no traffic.
* `Mcp266::read_encoder`, `read_speed`, `read_statusword`,
  `read_main_battery_voltage`, `read_temperature` and `read_device_info`.
* `CanopenClient::sdo_upload` and `read_string`, for `dump`.

Never called: `Mcp266::start` (NMT start plus `reset_faults`), `reset_faults`,
`move_to_position`, `drive_duty`, `set_encoder` or any setter. Even so, the
transmit filter would block any write.

**`arm <node> <ch>`** (motion):
* uploads of 0x203F:1..7 twice (the clamp, self-checked) and 0x607D:1/2; the present count is the median of the last 9 polled reads, no extra SDO;
* the heartbeat 0x77F `[05]` starts, every 100 ms;
* download 0x1016:1 = 0x007F01F4 (consumer: MIB node 127, 500 ms), then its
  upload as a readback.

Disarm reverses it: download 0x1016:1 = 0, an upload readback, then the heartbeat
stops.

**`clamp <node> <ch> <min> <max>`** (motion, armed channel):
1. uploads of 0x203F:1..7 twice (the clamp, self-checked) and 0x607D:1/2; the present count is the median of the last 9 polled reads, no extra SDO;
2. uploads of 0x203F:1..7 (the record, in readback order P, I, D, MaxI, deadband,
   MinPos, MaxPos);
3. downloads of 0x203D:1..7 in the setter's order D, P, I, MaxI, deadband, min,
   max. The gains are the values just read, so only the clamp changes. espp's
   `configure_position_loop()` is not used, because it seeds a P gain when P reads 0;
4. downloads of 0x607D:1 = min and 0x607D:2 = max;
5. readback: uploads of 0x203F:1..7, then 0x203F:1..7 twice more and 0x607D:1/2. The
   event shows the windows and P, I, D, MaxI and deadband before and after.

**`pstep <node> <ch> <delta>`** (motion, armed channel). This is espp's
`Mcp266::move_to_position()` sequence without its automatic fault reset.
1. Checks: uploads of 0x203F:1..7 twice (the clamp, self-checked) and 0x607D:1/2; the present count is the median of the last 9 polled reads, no extra SDO.
2. NMT `0x000 [01 N]`: start that node only.
3. Download 0x6060 = 0x01 (profile position): `[2F 60 60 00 01 ..]`. Wait 25 ms.
4. `Ds402Drive::enable_operation`: upload 0x6041. Then, depending on the state it
   finds, each step is a download of the controlword 0x6040 followed by uploads of
   0x6041 every 25 ms until that state is reached (up to 1 s):
   * 0x0006 (shutdown) until READY TO SWITCH ON,
   * 0x0007 (switch on) until SWITCHED ON,
   * 0x000F (enable operation) until OPERATION ENABLED.

   A drive in FAULT is refused here (`operation_not_permitted`), never reset.
5. Downloads of 0x6083 and 0x6084 = acceleration (500) and 0x6081 = velocity (500).
6. Download 0x607A = target.
7. Download 0x6040 = 0x003F (enable, change immediately, new set-point). Uploads of
   0x6041 until bit 12 (set-point acknowledged) is set.
8. Download 0x6040 = 0x002F. Uploads of 0x6041 until bit 12 clears.
9. After the 3 s window: the stop sequence on that channel.

**Stop sequence** (`stop` on every motion channel, a guard trip on one channel, or
the end of a `pstep` window). Writable nodes only; read-only nodes get nothing and
are only reported.
1. For a channel whose duty is non-zero or unreadable: uploads of 0x203F:1..7
   twice and 0x607D:1/2. The count is the median of the last 9 polled reads,
   and only if they are settled; a moving or glitching joint gets no hold. If
   both windows hold the count, a hold is pstep steps 3–8 with target = that
   median, and no NMT.
2. If anything was held, wait 2 s. This is the teammate's verified recovery for
   a live loop.
3. Download 0x6040 = 0x0000 (disable voltage) to every channel in the stop. With
   `stop` that is both channels of node 3. Node 2 gets nothing.
4. After at least 100 ms, an event with the duty, current and statusword the
   poller then reads.

The drives are left in SWITCH ON DISABLED. The bench found that blocks Motion
Studio's PWM slider until a power cycle.

**`clearfaults <node>`** (motion): `Mcp266::reset_faults`.
1. Per channel: upload 0x6041.
2. On a channel in FAULT: download 0x6040 = 0x0000, then 0x0080, then uploads of
   0x6041 until it leaves FAULT.
3. Then two statusword uploads for the event.

It does **not** clear the controller's E-stop latch. That is
`Mcp266::reset_estop()`: command 200 at 0x20C8, device-wide, and not called.

Never used anywhere:
* 0x2041 (the buffered position mirror): on the bench it moved a motor outside
  the CiA 402 state machine and could not be stopped;
* open-loop duty 0x2020 (inert);
* cyclic-synchronous modes, PDOs, SYNC.

## Transmit filter (`main/bus_policy.hpp`)

Every frame passes `check_frame()` in `send_frame()`, the one function that hands
frames to TWAI. It is an allow-list:
* SDO requests. To a read-only node, only upload initiate (0x40) and upload
  segment (0x60 / 0x70) are allowed.
* NMT to one node that is not read-only. An NMT broadcast to node 0 is dropped.
* RPDOs to nodes that are not read-only. None are sent.
* The MIB's own heartbeat, in the motion build.

SYNC, TIME, LSS, remote, extended and unknown frames are dropped. A dropped frame
is counted and raises an event: `BLOCKED write to read-only node N (cob 0x..,
bytes ..)`, or `BLOCKED frame` for a broadcast.

It is the last line of defence, not the first. `arm`, `clamp`, `pstep` and
`clearfaults` refuse a non-motion or read-only node before building a frame, the
motion worker checks again before the first frame, and `stop` never sends to a
read-only node.

## Motion guards

`pstep` is refused before any write, with an event naming the reason, unless all of
these hold:

| Check | Protects against |
|---|---|
| node is in `SYSID_MOTION_NODE_IDS` and not read-only | moving the carriages, or any node not meant to move |
| `arm <node> <ch>` of exactly this channel, within 10 s, with the 3 s window still fitting | a stray or replayed line moving anything |
| one move at a time | ambiguous trips |
| \|delta\| ≤ `SYSID_PSTEP_MAX_DELTA_COUNTS` (**100**) | one command sending the joint far |
| clamp **and** 0x607D both non-degenerate, span ≤ `SYSID_MAX_CLAMP_SPAN_COUNTS` (**400**), both holding present and target ≥ `SYSID_CLAMP_MARGIN_COUNTS` (50) inside | a controller forcing the target to the edge of a stale or wrong window (node 3 M1 carried [10000, 18000] from the bench example; a reset gives [0, 0]). A host or firmware bug cannot ask for more than the window |
| statusword fresh and not in fault | moving out of a fault. Faults are never cleared automatically; use `clearfaults` |
| encoder and current fresh; current ≤ `SYSID_MAX_CURRENT_MA` (4000) | guards that cannot see |
| `_lim` = 0, IMU reading, tilt rate ≤ 30 °/s, CAN error-active | starting in a bad state |

While a move runs, these are checked every sample period on the data the row
prints. A trip runs the stop sequence on the channel and disarms.

| Guard | Protects against |
|---|---|
| time cap `SYSID_MOVE_WINDOW_MS` (3000) | the normal end: the stop sequence takes over |
| stale feedback: no encoder within 3 sample periods, or the node goes offline | moving unobserved |
| CAN: not error-active, any bus error, or an SDO the node missed | a bus on which a stop might not get through |
| travel: encoder outside [min(start, target) − 100, max(start, target) + 100] | wrong sign, wrong channel, a target that was ignored |
| fault bit set | a drive fault mid-move |
| `_lim` ≠ 0 | end of travel (the MCP likely does not report limits over CAN) |
| current > 4000 mA, or current missing | stall, jam, pinch |
| tilt rate (angular rate ⊥ gravity, mounting-independent) > 30 °/s, or IMU lost | the chair starting to tip |
| any console byte (except the LF of a CRLF) | the operator's reflex |
| sequence failure (enable or handshake refused) | a half-started move |

**Present count is never one sample.** Each channel keeps its last **N = 9**
polled encoder reads (about 90 ms at 100 Hz). Wherever a count decides
something, the present count is their median, and only if the 9 are fresh and
their spread (max − min) is at most `SYSID_ENCODER_SETTLED_SPREAD_COUNTS` (200,
about 3× the noise at rest). Otherwise the command is refused. That covers:
* the `pstep` start and target (target = median + delta);
* the `pstep` window checks;
* the `clamp` present-in-window check;
* the stop's hold target. If the joint is moving or glitching, the spread is
  too large, so there is no hold: disable voltage only. That way an 1800-count
  glitch can never become a hold target.

The running guards (travel window, stale feedback) use single samples. A single
outlier can only trip them, which stops the move: the safe direction.

**`pstep` end event.** When the window ends or a guard trips, one line reports:
the start count (median), target, final count (median of 9, with spread), peak
|duty|, peak |current| and whether the statusword ever showed target reached.

**CAN error counting.**
* `#meta:can_bus_errors` counts espp::Twai `on_error` events, one per TWAI error
  interrupt. The driver reports *arbitration lost* as one of them, which is normal
  when three pollers share the bus.
* `#meta:can_errors` breaks them down: arbitration_lost, bit, form, stuff, ack,
  and bus_off_entries. It also gives the driver's present state, TEC/REC and
  cumulative `bus_err_num` (`Twai::get_info`).
* The move guard trips on any non-arbitration error.

**SDO reply checking.**
* espp (`CanopenClient::process_frame`) accepts a reply only if its index:sub
  matches the request in flight; segment replies are matched by phase. A late
  reply to a *different* object is dropped.
* This firmware checks every reply against the address of the last request sent
  to that node, and counts misses in `#meta:n<id>_sdo` (requests, replies,
  mismatched).
* A late reply to the *same* object cannot be told apart, by espp or by anyone.
  At worst it hands back a value one pass old, which `_age_us` does not show.

**Defaults and their basis.**
* **Delta cap 100 counts**: about 1.25 % of a main leg's roughly 8000-count
  travel, and 2× the bench controller's 50-count deadband, so a step should move
  at all.
* **Clamp span cap 400 counts**: about 5 % of that travel. Both caps assume the
  operator's rough figures for the main legs. Caster travel is unknown. **Revisit
  both once the first passive log shows real counts.**
* Profile velocity and acceleration: 500 counts/s and 500 counts/s², the
  teammate's profile. The bench found the velocity honoured below the stored speed,
  and the accel ignored.

**Comm timeout.**
* At `arm`, the armed node gets a CANopen heartbeat consumer on the MIB's heartbeat
  (node 127, 500 ms), which runs only while armed. It is turned off again at
  disarm.
* Whether the MCP266 implements 0x1016, and what it does on a timeout (see 0x6007
  in `dump`), is unverified. The write and readback are events.

## Known unknowns (verify on the bench)

* Leg travel in counts (rough figures above), caster travel, and the clamps
  currently on each controller. Run `dump 1` and `dump 3` first.
* Units and layout of current (0x2031) and PWM (0x2030); speed units (0x606C).
* Signs: which way positive `_enc`, `_spd`, `_cur_ma` and `_duty` move each joint;
  IMU axes relative to the chair.
* Whether profile moves need NMT operational. `pstep` sends a targeted NMT start
  because the teammate's moves ran that way. An operational node may also start
  sending its default TPDOs.
* Whether a hold or move on an EHA behaves like the bench gearmotor (not
  back-drivable, so disable voltage should hold position).
* Status flag bit map (0x205A); limit switches are probably not reported over CAN.
* The MCP266's SDO turnaround, and so the real refresh rate (`#meta:n<id>_poll`).
* Whether 921600 is reliable on this CH343 for hours of logging (`tx_dropped_lines`
  counts drops only on our side; garbled rows show up on the host).
* Whether the MCP266 honours 0x1016, and the value of 0x6007.

## Why not the existing code

* `components/Actuator` drives and polls through the `Actuator` class, whose
  control logic is under review. The brief asked for measurement independent of
  it. Its example was the template for the CAN bring-up and the 0x205A decoding.
* `espp::Mcp266` is used for every read it offers. Raw SDOs are used where it has
  none: current, duty, flags and the `dump` objects.
* `Mcp266::move_to_position` is replaced by the same sequence without the hidden
  fault reset, and `configure_position_loop` by a clamp write that keeps the gains.
  Both differences are deliberate, and each is given above.
* `components/bsp` is reused for the board and the IMU. Its `init()` also starts
  Ethernet as the usual DHCP server.
