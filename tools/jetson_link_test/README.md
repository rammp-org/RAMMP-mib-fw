# MIB <-> Jetson link over RTPS (ROS 2 Humble, Cyclone DDS)

How the MIB (ESP32-P4, espp `rtps` / embeddedRTPS) and the Jetson (ROS 2 Humble,
`rmw_cyclonedds_cpp`) talk to each other over the PoE Ethernet link, what had to
change to make it work, and how to run it. The `ros2/rammp_msgs` package in this
directory is the ROS 2 side of the shared message definitions.

## What is happening on the wire

1. **Network.** The MIB runs a DHCP server on its Ethernet port (`192.168.4.1/24`,
   `MIBconfig.hpp`). The Jetson's wired interface is set to DHCP and gets
   `192.168.4.x`. The Jetson's other interface (Wi-Fi / lab network,
   `192.168.1.x`) stays up; the MIB can only reach the `192.168.4.0/24` side.
2. **Participant discovery (SPDP).** Both sides multicast their participant
   announcement to `239.255.0.1:7400` (domain 0). The announcement carries the
   participant's *default unicast locators*: the address and port where its
   endpoints can be reached. Cyclone announces one locator per interface it is
   configured to use.
3. **Endpoint discovery (SEDP).** Each side then tells the other, over reliable
   unicast, which topics it publishes and subscribes to, with the DDS topic name,
   type name and QoS. Two endpoints pair only if all three are compatible.
4. **Data.** Matched writers send samples straight to the reader's unicast
   locator. Everything the MIB publishes is best-effort, so there are no
   acknowledgements on user topics.

## What ROS 2 expects from the MIB

ROS 2 mangles names before they reach DDS, so the MIB must use the mangled form
or nothing pairs even though `ros2 topic list` shows the topic.

| | ROS 2 | On the wire |
|---|---|---|
| Topic | `/rammp/mib/status` | `rt/rammp/mib/status` |
| Type | `rammp_msgs/msg/MibStatus` | `rammp_msgs::msg::dds_::MibStatus_` |
| Reliability | subscriber must ask for `best_effort` | MIB publishes `BEST_EFFORT` |
| Domain | `ROS_DOMAIN_ID` unset or `0` | espp `rtps` domain 0 |

The names in `external/rammp-rtps` (`rammp/msg/MibStatus`, ...) are not ROS 2
names. On this branch the commit "use ROS 2 names on the status and joystick
endpoints" renames two endpoints in `components/MibSystem/src/mib_system.cpp`:

| Endpoint | DDS topic | DDS type |
|---|---|---|
| status publisher | `rt/rammp/mib/status` | `rammp_msgs::msg::dds_::MibStatus_` |
| joystick subscriber | `rt/rammp/joystick/xy_twist` | `rammp_msgs::msg::dds_::XYTwist_` |

While that is flashed, the physical joystick (still on the old names) does not
match those two endpoints. The struct layouts are unchanged;
`ros2/rammp_msgs/msg/*.msg` mirrors them field for field, in wire order.

## What had to be fixed in the MIB's RTPS stack

espp `rtps` (a de-vendored embeddedRTPS) is only validated against Fast DDS.
Cyclone DDS exercises four things Fast DDS does not, and each one alone stops
every sample: the MIB's topics were visible on the Jetson while `echo` stayed
silent and `pub` never reached the MIB. All four are fixed in a patched copy of
the component under `external/espp__rtps` (selected by `override_path` in
`main/idf_component.yml`); `external/espp__rtps/PATCHES.md` has the details and
the diff. The patch was root-caused and verified on a host build of espp
against a Cyclone peer on the `feature/21-mib-debugger` branch
(`tools/mib_debugger/interop/`, `BRANCH_NOTES.md` finding 10).

1. **Heartbeats and gaps addressed to the participant were dropped.** Cyclone
   sends HEARTBEAT/GAP with `readerId = ENTITYID_UNKNOWN`; the receiver found
   no reader by that id and discarded them, so the MIB never requested, and
   never received, the Jetson's endpoint announcements.
2. **Pre-emptive ACKNACK with sequence base 0.** The spec requires base >= 1;
   Cyclone forgives 0 only for eProsima/RTI vendor ids and drops it from
   embeddedRTPS (vendor 13.37), printing `recvUC: malformed packet received
   from vendor 13.37 ...` on the Jetson.
3. **ACKNACKs without INFO_DST, echoing reader id 0.** Cyclone could not tell
   which writer or reader they were for and ignored them.
4. **Placeholder unicast locator.** Cyclone omits the per-endpoint locator from
   SEDP and relies on the participant's default locator; the MIB kept a
   `192.168.0.42:7411` placeholder instead and sent every sample there.

Point 4 is also why the Jetson's Cyclone configuration matters: if Cyclone
advertises only the `192.168.1.x` address, the MIB has nothing reachable to
fall back to and logs `Dropping UDP send ... to 192.168.1.11 ... 'Not enough
space'` for every packet.

Every board on the chair that a Cyclone peer must talk to needs the patched
component: the MIB and both PACE RACER boards.

## Jetson setup (done once)

**Cyclone interface configuration.** Cyclone uses a single interface by default,
and on the Jetson that is the Wi-Fi/lab side. Listing both interfaces makes it
advertise both addresses; the MIB picks the `192.168.4.x` one, and ROS 2 traffic
with the rest of the lab keeps using the other. `~/cyclonedds.xml`
(interface names from `ip -4 -o addr show`):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="WIRED_IF" priority="default" multicast="default"/>
        <NetworkInterface name="OTHER_IF" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

The file only takes effect in a shell that exports `CYCLONEDDS_URI`, so the
export lives in `~/.zshrc` and every new terminal gets it:

```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

**Message package.** `ros2 topic echo` can only decode a type it has a package
for:

```bash
mkdir -p ~/rammp_ws/src
cp -r <this repo>/tools/jetson_link_test/ros2/rammp_msgs ~/rammp_ws/src/
cd ~/rammp_ws && colcon build --packages-select rammp_msgs
```

Source `/opt/ros/humble/setup.zsh` and `~/rammp_ws/install/setup.zsh` in the
shell (or in `~/.zshrc`). `ROS_DOMAIN_ID` stays unset.

**After changing any of the above:** `ros2 daemon stop`. The daemon is a
long-lived process that keeps the configuration it was started with, so
`ros2 topic list` can look right while a fresh `echo` or `pub` in a badly
configured shell does not.

## Per-terminal checklist (Jetson)

Needed in every new shell unless the lines are in `~/.zshrc`:

```bash
source /opt/ros/humble/setup.zsh          # 1. ROS 2
source ~/rammp_ws/install/setup.zsh       # 2. rammp_msgs (echo/pub need it to decode)
echo $CYCLONEDDS_URI                      # 3. expect file:///home/snorlax/cyclonedds.xml (from ~/.zshrc)
ros2 daemon stop                          # 4. only after changing cyclonedds.xml / CYCLONEDDS_URI / RMW
```

If every `ros2` command hangs, the daemon is wedged: `pkill -9 -f _ros2_daemon`,
then `ros2 topic list --no-daemon`. If that also hangs, check `eno1` still has
its `192.168.4.x` address (reflashing the MIB drops the PoE link briefly).

## Running it

MIB, from the repo root:

```bash
idf                          # ESP-IDF environment (see ~/.zshrc on the Mac)
idf.py -p PORT flash monitor
```

Console: `Board IP: 192.168.4.1`, then `Drive status: state=IDLE joystick x=0
y=0 ...` every 500 ms.

Jetson, after `ping -c 3 192.168.4.1` succeeds:

```bash
ros2 topic list -t                                   # /rammp/mib/status [rammp_msgs/msg/MibStatus]
ros2 topic echo --qos-reliability best_effort /rammp/mib/status
```

`MibStatus` samples stream with `seq` climbing. Without the `best_effort` flag
the subscription is reliable, never matches the MIB's writer, and prints nothing.

Jetson to MIB:

```bash
ros2 topic pub -r 10 --qos-reliability best_effort /rammp/joystick/xy_twist \
  rammp_msgs/msg/XYTwist "{x: 0.0, y: 0.2, twist: 0.0, buttons: 0}"
```

The MIB's `Drive status:` line shows `y=0.2` within a second (it logs the latest
sample in every state, drive does not have to be enabled).

## Troubleshooting

| Symptom | Cause |
|---|---|
| Jetson has no `192.168.4.x` address | wired interface not on DHCP, or link down (PoE power / cable) |
| `ros2 topic list` empty | stale daemon (`ros2 daemon stop`), Cyclone on the wrong interface (`echo $CYCLONEDDS_URI`), or multicast blocked (firewall UDP 7400-7500, IGMP snooping on a managed PoE switch) |
| `The message type 'rammp_msgs/msg/MibStatus' is invalid` | `~/rammp_ws/install/setup.zsh` not sourced in this shell |
| Topic listed, `echo` silent | missing `--qos-reliability best_effort`; or MIB flashed without the `external/espp__rtps` patch |
| MIB logs `Dropping UDP send ... to 192.168.1.x` | the node's Cyclone advertised only the off-link address: `CYCLONEDDS_URI` not exported in that shell |
| Jetson logs `malformed packet received from vendor 13.37` | MIB flashed without the `external/espp__rtps` patch |
| every `ros2` command hangs | wedged daemon: `pkill -9 -f _ros2_daemon`, then `ros2 topic list --no-daemon` |

## Known limits and follow-ups

- The patched firmware has not yet been verified end to end against the Jetson
  (the host harness on `feature/21-mib-debugger` passes). First thing to do
  after flashing: `echo` must stream `MibStatus`, `pub` must move `y` in the
  MIB's `Drive status:` line, and neither console may show `malformed packet`
  or `Dropping UDP send` lines.
- The four RTPS fixes are local patches; they should be sent upstream to
  esp-cpp/espp (`components/rtps`) so `external/espp__rtps` and the
  `override_path` can be removed. The PACE RACER firmware needs the same
  override until then.
- For the real integration, the ROS 2 names belong in `external/rammp-rtps`
  (topics `rt/rammp/...`, types `rammp_msgs::msg::dds_::<Type>_`, with the
  `.msg` files alongside the headers) so the joystick, PACE RACER and Jetson all
  use one set of names. The endpoint-rename commit on this branch is a stand-in
  for that and should not be merged as is.
