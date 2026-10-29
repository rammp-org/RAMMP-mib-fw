# The MIB on ROS 2

This is what a ROS 2 node, such as the Jetson's, needs in order to talk to the MIB. It is the whole handoff: the MIB's ROS 2 names are defined here and in `components/MibSystem/include/ros2_interface.hpp`, not in rammp-rtps. Keep the two in step.

The MIB speaks RTPS through espp, not ROS 2. Its other endpoints, which the joystick and PACE RACER boards use, have names that ROS 2 cannot see. The endpoints below are the ones ROS 2 can see, and they use the types from [rammp-interfaces-ros2](https://github.com/rammp-org/rammp-interfaces-ros2).

## Topics

| ROS 2 topic | Type | Direction | QoS | What it is |
|---|---|---|---|---|
| `/rammp/mib/status` | `rammp_base_interfaces/msg/MibState` | MIB → ROS 2 | best-effort, volatile | The MIB's state, every 500 ms. `AUTONOMOUS` and `SHARED` are reported as `ENABLED`; an e-stop is `ERROR`, with `error_message` saying why. |
| `/rammp/joystick/xy_twist` | `rammp_peripheral_interfaces/msg/XYTwist` | ROS 2 → MIB | best-effort | **Logged only:** the MIB's console prints each sample, and it never drives on it. Driving from ROS 2 goes through arbitration (`/base/setpoint/twist`, #42). |

On the wire, ROS 2 adds the prefix `rt/` to topic names and turns `pkg/msg/T` into `pkg::msg::dds_::T_`. So the MIB publishes `rt/rammp/mib/status` with type `rammp_base_interfaces::msg::dds_::MibState_`.

A subscriber must ask for best-effort. The default reliable subscription never matches the MIB's writer, so it receives nothing:

```bash
ros2 topic echo --qos-reliability best_effort /rammp/mib/status
```

## Requirements

- **espp/rtps with esp-cpp/espp#862.** The Jetson runs ROS 2 on `rmw_cyclonedds_cpp`, and espp/rtps 1.3.10 and older exchange no samples with CycloneDDS. ROS 2 lists the MIB's topics, but `echo` stays silent, and the Jetson logs `malformed packet received from vendor 13.37`. `main/idf_component.yml` names the release to use.
- **Domain 0:** `ROS_DOMAIN_ID` unset, or `0`.
- **The two type packages** on the ROS 2 side: `rammp_base_interfaces` and `rammp_peripheral_interfaces` from rammp-interfaces-ros2, built and sourced. Without them, `ros2 topic echo` can't decode the samples.
- **Services don't work yet on CycloneDDS.** rmw_cyclonedds_cpp puts its request header in the payload; espp follows rmw_fastrtps_cpp, which doesn't. Topics are not affected.

## Network

- **The link.** The MIB runs a DHCP server on its Ethernet port, at `192.168.4.1/24` (`MIBconfig.hpp`). Put the Jetson's wired interface on DHCP, and it gets a `192.168.4.x` address.
- **The switch** must forward the RTPS discovery multicast (`239.255.0.1`, UDP 7400 and up). On the bench, a Netgear GS308E passed it. A TP-Link TL-SG105 didn't: its IGMP snooping drops it.
- **CycloneDDS's interfaces.** CycloneDDS uses one interface unless told otherwise, and on the Jetson that is usually the Wi-Fi.
  - List the MIB's interface first. The MIB sends to the first address the Jetson announces. When another interface came first on the bench, nothing reached the Jetson.
  - Interface names come from `ip -4 -o addr show`. The second interface is optional, for other ROS 2 traffic.

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
      <General>
        <Interfaces>
          <NetworkInterface name="WIRED_IF" multicast="true"/>  <!-- the MIB's link: first -->
          <NetworkInterface name="OTHER_IF" multicast="true"/>  <!-- optional -->
        </Interfaces>
        <AllowMulticast>true</AllowMulticast>
      </General>
    </Domain>
  </CycloneDDS>
  ```

  ```bash
  export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
  ```

- **After changing any of this:** run `ros2 daemon stop`, or use `--no-daemon`. The daemon keeps the configuration it started with, so `ros2 topic list` can look right while a new `echo` shows nothing.

## Checking it

On the Jetson, once `ping 192.168.4.1` answers:

```bash
ros2 topic list -t --no-daemon
#   /rammp/joystick/xy_twist [rammp_peripheral_interfaces/msg/XYTwist]
#   /rammp/mib/status [rammp_base_interfaces/msg/MibState]
ros2 topic echo --qos-reliability best_effort --once /rammp/mib/status
ros2 topic pub -r 10 --qos-reliability best_effort /rammp/joystick/xy_twist \
  rammp_peripheral_interfaces/msg/XYTwist "{x: 0.1, y: 0.4, twist: -0.2, buttons: 1}"
```

The MIB's console then shows `ROS 2 joystick (not driven on): x=0.1 y=0.4 twist=-0.2 buttons=1, ... ms ago`, once per status tick.

## Troubleshooting

| Symptom | Cause |
|---|---|
| The Jetson has no `192.168.4.x` address | The wired interface isn't on DHCP, or the link is down (PoE power, cable). |
| `ros2 topic list` doesn't show the MIB | A stale daemon, CycloneDDS on the wrong interface (`echo $CYCLONEDDS_URI`), or the multicast is blocked (a firewall on UDP 7400 and up, or a switch's IGMP snooping). |
| Topic listed, `echo` silent | A reliable subscription (add `--qos-reliability best_effort`); the MIB's interface not first in CycloneDDS's list; or the MIB built with espp/rtps older than #862. |
| `malformed packet received from vendor 13.37` on the Jetson | The MIB was built with espp/rtps older than #862. |
| `The message type ... is invalid` | The rammp-interfaces-ros2 packages aren't sourced in this shell. |

Credit: @ewang360 got the MIB and the Jetson talking (`ewang360/MIB-to-Jetson-Communication`). The topic names, the CycloneDDS setup and most of the troubleshooting table are hers.
