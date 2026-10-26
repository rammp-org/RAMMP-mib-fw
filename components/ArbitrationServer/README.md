# ArbitrationServer

The [Arbiter](../Arbiter/README.md)'s face on the wire: the base's half of the
arbitration protocol the arm serves from kinova-gen3-ros2, with the same ROS 2
types, so one client on the Jetson talks to both.

| Endpoint | Type | Notes |
|---|---|---|
| `/base/acquire_control` | `rammp_common_interfaces/srv/AcquireControl` | seizes; an all-zero `token` mints, anything else is adopted |
| `/base/release_control` | `rammp_common_interfaces/srv/ReleaseControl` | refused unless the token matches |
| `/base/revoke_control` | `rammp_common_interfaces/srv/RevokeControl` | no token: operator recovery |
| `/base/control_status` | `rammp_common_interfaces/msg/ControlStatus` | reliable; on change, and every second |

The C++ mirrors of these types, and their wire names, are in rammp-rtps
(`messages/control_message.hpp`, `services/control_service.hpp`). The services
run on espp's `ServiceServer`, the status on its `Publisher`, the poll on its
`Timer`.

Not here, on purpose: the tokened commands, which belong to whoever acts on
them (each handler wraps its work in `Arbiter::run_if_admitted()`), and the
e-stop, which on the MIB is estop-core's `rammp/estop` latch rather than the
arm's `/estop` topic.

## Configuration the firmware needs

What it takes for a ROS 2 host to reach the MIB, all measured on the bench (see
[docs/arbitration.md](../../docs/arbitration.md#on-the-wire)):

```
CONFIG_RTPS_LIMIT_NUM_STATEFUL_WRITERS=8       # 3 services + status, past the default 5
CONFIG_RTPS_LIMIT_NUM_STATEFUL_READERS=8
CONFIG_RTPS_LIMIT_MAX_TOPICNAME_LENGTH=128     # ROS 2 parameter-service names exceed 64
CONFIG_RTPS_LIMIT_MAX_TYPENAME_LENGTH=128
CONFIG_RTPS_LIMIT_NUM_READER_PROXIES_PER_WRITER=16  # one per ROS 2 client
CONFIG_RTPS_LIMIT_NUM_WRITER_PROXIES_PER_READER=16
CONFIG_LWIP_IP4_REASSEMBLY=y                   # Fast DDS discovery datagrams exceed the MTU
CONFIG_LWIP_IP_REASS_MAX_PBUFS=48
CONFIG_LWIP_UDP_RECVMBOX_SIZE=16
```

The example's `sdkconfig.defaults` has them.

## Example

`example/` serves arbitration on the MIB's Ethernet in ENFORCED mode, and
subscribes to `/base/setpoint/twist` (`rammp_base_interfaces/TwistSetpoint`),
counting the admitted twists instead of driving: nothing moves. Every 2 s it
prints the owner, the admitted and rejected counts, the halts and the last
admitted twist.

```console
cd components/ArbitrationServer/example
idf.py build flash monitor
```

Then exercise it from ROS 2 with [tools/ros2_bench](../../tools/ros2_bench/README.md).

The host must hear the MIB's discovery multicast. Cable it directly, or see the
network requirements in [tools/ros2_bench](../../tools/ros2_bench/README.md#network):
the bench's IGMP-snooping switch dropped it.
