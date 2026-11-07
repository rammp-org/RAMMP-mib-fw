# The MIB on ROS 2

What a ROS 2 node, such as one on the Jetson, needs in order to talk to the MIB. The MIB speaks RTPS through espp, not ROS 2, but its Jetson-facing topics carry ROS 2 names and types (rammp-rtps#18), so a ROS 2 node reads and writes them with no bridge. The names live in [rammp-rtps](https://github.com/rammp-org/rammp-rtps) (`messages/*.hpp`) and the matching `.msg` files in [rammp-interfaces-ros2](https://github.com/rammp-org/rammp-interfaces-ros2); keep the two in step.

The MIB's other endpoints, which the joystick and the PACE RACER boards use (`rammp/...` names), are not visible to ROS 2 by design.

## Topics

| ROS 2 topic | Type | Direction | What it is |
|---|---|---|---|
| `/rammp/mib/status` | `rammp_base_interfaces/msg/MibStatus` | MIB → ROS 2 | The MIB's state, twice a second: `system_state` (0 INITIALIZING, 1 IDLE, 2 ENABLED, 3 ERROR, 4 AUTONOMOUS, 5 SHARED, 6 ESTOP, 7 PUSH_ASSIST), `active_profile`, the seat, `error_message`, a `seq` that wraps at 255. `speed` is the commanded ground speed in m/s (the average of the two wheel commands, 0 unless the chair is driving). `epoch_s` and `utc_offset_min` are 0 until the MIB's clock is set over SNTP. |
| `/rammp/drive_command` | `rammp_base_interfaces/msg/DriveCommand` | ROS 2 → MIB | Ask for a drive state: `request` 0 DISABLE, 1 ENABLE (the joystick's), 2 AUTONOMOUS, 3 SHARED; `profile` 0 LOW, 1 NORMAL, 2 HIGH; `push_assist` with DISABLE asks for PUSH_ASSIST, where the push-handle force sensor drives (profile forced to NORMAL) and which ends on its own when the handle readings stop for 300 ms. AUTONOMOUS and SHARED are accepted from IDLE, ENABLED and PUSH_ASSIST; a stick touch ends AUTONOMOUS; DISABLE ends any of them. In SHARED the Jetson reads the stick (`/rammp/joystick/xy_twist`) and decides whether it moves the arm or the base; the MIB drives on `/base/setpoint/twist` exactly as in AUTONOMOUS and does not read the stick itself. |
| `/rammp/joystick/xy_twist` | `rammp_peripheral_interfaces/msg/XYTwist` | joystick → MIB | The physical stick. ROS 2 can read it. Do not publish on it from ROS 2: it would fight the stick. |
| `/base/setpoint/twist` | `rammp_base_interfaces/msg/TwistSetpoint` | ROS 2 → MIB | Drive on `twist.linear.x` (m/s) and `twist.angular.z` (rad/s, positive counter-clockwise) in AUTONOMOUS and SHARED. The chair stops 300 ms after the setpoints stop. The `token` is not checked yet; the Arbiter is being wired into the drive path, after which an unowned setpoint is dropped ([docs/arbitration.md](arbitration.md)). |
| `/base/odom` | `nav_msgs/msg/Odometry` | MIB → ROS 2 | Wheel odometry, 20 Hz: pose in `odom`, twist in `base_link`. Nothing is published until both drive boards have reported, and nothing new while neither does, so a silent `/base/odom` means the boards are not talking. |
| `/tf` | `tf2_msgs/msg/TFMessage` | MIB → ROS 2 | The `odom` → `base_link` transform, alongside each odometry sample. |

`/base/odom` and `/tf` are reliable; every other topic is best-effort. The MIB's writers advertise transient-local durability with a history of one, so a late subscriber gets the latest sample; a volatile subscriber matches too. On the wire, ROS 2 prefixes topic names with `rt/` and turns `pkg/msg/T` into `pkg::msg::dds_::T_`, so the MIB publishes `rt/rammp/mib/status` with type `rammp_base_interfaces::msg::dds_::MibStatus_`.

## Requirements

- **espp/rtps 1.4.0 or newer** (`main/idf_component.yml`). 1.3.11 brought the CycloneDDS interop fixes (esp-cpp/espp#862); 1.4.0 adds the two that a ROS 2 host with many nodes needs: a CycloneDDS peer was dropped about 10 s after discovery because its repeated announcements were not delivered (esp-cpp/espp#871), and the MIB tracked at most 10 remote participants (esp-cpp/espp#870, now `CONFIG_RTPS_LIMIT_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS=32` in `sdkconfig.defaults`, beside `HISTORY_SIZE_STATEFUL` and the two `PROXIES_PER` caps). Older versions list the MIB's topics but exchange no samples, or lose the peer after 10 s.
- **Domain 0:** `ROS_DOMAIN_ID` unset, or `0`.
- **The type packages** on the ROS 2 side: `rammp_base_interfaces` and `rammp_peripheral_interfaces` from rammp-interfaces-ros2 at the release that has `MibStatus` and `DriveCommand` (rammp-interfaces-ros2#27), built and sourced. Without them `ros2 topic echo` cannot decode the samples.
- **Services do not work on CycloneDDS yet.** rmw_cyclonedds_cpp puts its request header in the payload; espp follows rmw_fastrtps_cpp. Topics are unaffected.

## Network

- **The link.** The MIB is the DHCP server on its Ethernet, at `192.168.4.1/24` (`MIBconfig.hpp`). Put the Jetson's wired interface on DHCP and it gets a `192.168.4.x` address, which can change across reboots.
- **The switch** must forward the RTPS discovery multicast (`239.255.0.1`, UDP 7400 and up). A Netgear GS308E does. A TP-Link TL-SG105 did not: its IGMP snooping drops it.
- **Pin CycloneDDS to the MIB's interface**, in every process that talks to the MIB, containers included. Left to itself CycloneDDS picks one interface, usually not the MIB's, and then advertises an address the MIB cannot route to: the MIB logs `Dropping UDP send ... to 192.168.1.x ... 'Not enough space'` for every packet it tries to send that process, and samples never arrive. Interface names come from `ip -4 -o addr show`.

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
      <General>
        <Interfaces>
          <NetworkInterface name="eno1" multicast="true"/>  <!-- the MIB's link -->
        </Interfaces>
        <AllowMulticast>true</AllowMulticast>
      </General>
    </Domain>
  </CycloneDDS>
  ```

  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
  ```

- **After changing any of this:** `ros2 daemon stop`, or use `--no-daemon`. The daemon keeps the configuration it started with, so `ros2 topic list` can look right while a new `echo` shows nothing.

## Checking it

On the ROS 2 host, once `ping 192.168.4.1` answers. Nothing here moves the wheels.

```bash
ros2 topic list -t --no-daemon --spin-time 10 | grep -E 'rammp|base|^/tf '
#   /rammp/drive_command       [rammp_base_interfaces/msg/DriveCommand]
#   /rammp/joystick/xy_twist   [rammp_peripheral_interfaces/msg/XYTwist]
#   /rammp/mib/status          [rammp_base_interfaces/msg/MibStatus]
#   /base/setpoint/twist       [rammp_base_interfaces/msg/TwistSetpoint]
#   /base/odom                 [nav_msgs/msg/Odometry]
#   /tf                        [tf2_msgs/msg/TFMessage]
ros2 topic echo /rammp/mib/status            # system_state 1, seq counting, twice a second
ros2 topic echo /base/odom --once            # pose and twist; silent until both drive boards report
ros2 topic pub --once /rammp/drive_command rammp_base_interfaces/msg/DriveCommand \
  "{request: 9, profile: 1, push_assist: false}"   # unknown request: refused, logged, no state change
```

The MIB's console then prints `Ignoring unknown drive request 9`. A `--once` publish can go out before the MIB's reader has matched the new publisher; if no line appears, send it again. For the real thing, `request: 2` takes the MIB to AUTONOMOUS (`system_state` 4) and arms the drive; a touch of the stick returns it to IDLE.

## Troubleshooting

| Symptom | Cause |
|---|---|
| The host has no `192.168.4.x` address | Its wired interface is not on DHCP, or the link is down (PoE power, cable). The Mac or laptop is on another port. |
| `ros2 topic list` does not show the MIB | A stale daemon; CycloneDDS on the wrong interface (`echo $CYCLONEDDS_URI`); multicast blocked (a firewall on UDP 7400 and up, IGMP snooping); or more participants on the domain than `CONFIG_RTPS_LIMIT_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` (32). |
| Topic listed, `echo` silent | CycloneDDS not pinned to the MIB's interface; or espp/rtps older than 1.3.11. |
| `echo` works, then stops after about 10 s | espp/rtps older than 1.4.0 (esp-cpp/espp#871): the MIB dropped the peer at its lease. Restarting the node buys another 10 s. |
| `Dropping UDP send ... to 192.168.1.x` on the MIB's console | A process on the host has CycloneDDS on another interface. Pin it (above). |
| `malformed packet received from vendor 13.37` on the host | espp/rtps older than 1.3.11. |
| `The message type ... is invalid` | The rammp-interfaces-ros2 packages are not sourced in this shell. |

Credit: Alex Chun Lin's #52 wrote the first version of this page; the CycloneDDS setup and the first troubleshooting table came from the `ewang360/MIB-to-Jetson-Communication` handoff on #51.
