# MIB <-> Jetson RTPS link test

Checks that the MIB (ESP32-P4, espp `rtps`) and a Jetson running ROS 2 can see
each other's topics over the PoE Ethernet link. The MIB runs the normal firmware
with two endpoints renamed for ROS 2; the Jetson reads and writes them with the
`ros2` CLI and the small `ros2/rammp_msgs` package in this directory.

## What has to match (not just the `rt/` prefix)

ROS 2 maps names onto DDS as follows, with any RMW (Fast DDS or Cyclone DDS).
All of them must match on both sides or the endpoints are listed but never pair.

| | ROS 2 | On the wire (what the MIB must use) |
|---|---|---|
| Topic | `/rammp/mib/status` | `rt/rammp/mib/status` |
| Type | `rammp_msgs/msg/MibStatus` | `rammp_msgs::msg::dds_::MibStatus_` |
| Reliability | subscriber must be `best_effort` | MIB publishes `BEST_EFFORT` |
| Domain | `ROS_DOMAIN_ID` unset or `0` | espp `rtps` domain 0 |

The firmware's type names from `external/rammp-rtps` (`rammp/msg/MibStatus`,
`rammp/msg/XYTwist`) are **not** ROS 2 type names, so a ROS 2 node never pairs
with them. The struct layouts need no change: `ros2/rammp_msgs/msg/*.msg` mirrors
them field for field, in wire order.

## MIB side

The commit "use ROS 2 names on the status and joystick endpoints" on this branch
renames two endpoints in `components/MibSystem/src/mib_system.cpp`:

| Endpoint | DDS topic | DDS type |
|---|---|---|
| status publisher | `rt/rammp/mib/status` | `rammp_msgs::msg::dds_::MibStatus_` |
| joystick subscriber | `rt/rammp/joystick/xy_twist` | `rammp_msgs::msg::dds_::XYTwist_` |

While flashed with that commit the physical joystick no longer matches those two
endpoints (it still uses the old names). Flash from the repo root as usual:

```bash
idf.py -p PORT flash monitor
```

The console should reach `System state: INIT` -> `Board IP: 192.168.4.1` and
then log `Drive status: state=IDLE joystick x=... y=...` every 500 ms.

## Jetson side

The MIB is the DHCP server (`192.168.4.1/24`, from `MIBconfig.hpp`), so the
Jetson's wired interface must be set to DHCP (automatic). PoE injector or PoE
switch between the two; the network itself does not care about the power.

```bash
# Network
ip -4 addr show                 # expect 192.168.4.x on the wired interface
ping -c 3 192.168.4.1

# ROS 2 environment (see "Cyclone DDS" below for the RMW we actually run)
source /opt/ros/$ROS_DISTRO/setup.bash
unset ROS_DOMAIN_ID                           # MIB is on domain 0

# Build the message package once
mkdir -p ~/rammp_ws/src
cp -r <this repo>/tools/jetson_link_test/ros2/rammp_msgs ~/rammp_ws/src/
cd ~/rammp_ws && colcon build --packages-select rammp_msgs
source ~/rammp_ws/install/setup.bash
```

Read from the MIB:

```bash
ros2 topic list                                              # expect /rammp/mib/status
ros2 topic echo --qos-reliability best_effort /rammp/mib/status
```

Write to the MIB (shows up in the MIB's `Drive status:` log line, no need to
enable drive):

```bash
ros2 topic pub -r 10 --qos-reliability best_effort /rammp/joystick/xy_twist \
  rammp_msgs/msg/XYTwist "{x: 0.0, y: 0.2, twist: 0.0, buttons: 0}"
```

Pass: `echo` prints `MibStatus` samples with `seq` climbing, and the MIB log
shows the `x`/`y` you published.

## Cyclone DDS (Humble)

The Jetson stack runs `rmw_cyclonedds_cpp`. The espp `rtps` component the MIB
uses is only validated against Fast DDS (`rmw_fastrtps_cpp`, ROS 2 Jazzy); its
older READMEs listed Cyclone DDS as "partial / unverified", and Cyclone is the
stricter implementation about discovery packets. The naming and QoS rules above
are ROS 2 conventions and apply to both RMWs unchanged, so if Cyclone fails
where Fast DDS works, the gap is embeddedRTPS <-> Cyclone discovery, not the
messages. Humble vs Jazzy should not matter: ROS 2 types use the same XCDR1
wire encoding on both.

Run the test twice, Fast DDS first, so a Cyclone-specific failure is obvious:

```bash
# 1. Fast DDS (installed by default on Humble)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list && ros2 topic echo --qos-reliability best_effort /rammp/mib/status

# 2. Cyclone DDS
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp   # if not already present
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 daemon stop                                        # the daemon caches the previous RMW
ros2 topic list && ros2 topic echo --qos-reliability best_effort /rammp/mib/status
```

Cyclone uses **one** network interface by default and on a Jetson with Wi-Fi up
that is usually not the wired port. Pin it with `~/cyclonedds.xml` (replace
`eth0` with the interface that holds `192.168.4.x` in `ip -4 addr`):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

Fast DDS uses every interface, so it needs no such file.

What each outcome means under Cyclone:

| Observation | Meaning |
|---|---|
| `/rammp/mib/status` in `ros2 topic list` | Cyclone accepted the MIB's SPDP/SEDP discovery |
| `echo` prints samples | data path works; the firmware's endpoints can be consumed by the Jetson stack as-is |
| listed but `echo` silent | check `best_effort` first; then compare with Fast DDS |
| not listed under Cyclone, listed under Fast DDS | embeddedRTPS discovery not accepted by Cyclone; capture it with `sudo tcpdump -i eth0 udp port 7400 -w spdp.pcap` and raise it against the espp `rtps` component |

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Jetson has no `192.168.4.x` address | wired interface not on DHCP, or link down (PoE power / cable) |
| `ros2 topic list` is empty | different domain; stale `ros2 daemon` after switching RMW (`ros2 daemon stop`); Cyclone bound to the wrong interface (see `CYCLONEDDS_URI` above); or multicast blocked (firewall: allow UDP 7400-7500; managed PoE switch with IGMP snooping: disable it or use an injector) |
| Topic listed, `echo` prints nothing | subscription not `best_effort` |
| `echo` fails to decode | `rammp_msgs` not built/sourced, or a field was reordered relative to `mib_message.hpp` |
| MIB log never shows the published `x`/`y` | Jetson publisher not reaching 192.168.4.1: `ros2 topic info -v /rammp/joystick/xy_twist` should list the MIB's subscription |
