# Local patches to espp/rtps 1.3.10

This directory is a copy of the `espp/rtps` component from the ESP component
registry (example/, interop/ and CHECKSUMS.json removed), selected through
`override_path` in `main/idf_component.yml`. It carries the four fixes needed
to talk to ROS 2 on Cyclone DDS (`rmw_cyclonedds_cpp`, the Jetson's RMW). All
are interoperability bugs in the embeddedRTPS engine, not RAMMP-specific, and
should go upstream to https://github.com/esp-cpp/espp (components/rtps).

The applied diff is `espp-rtps-cyclonedds-interop.patch` in this directory. It
was written and verified on a host build of espp against a Cyclone DDS peer
(espp writer -> Cyclone reader and Cyclone writer -> espp reader, reliable and
best-effort, no malformed-packet lines) on the `feature/21-mib-debugger`
branch; see `tools/mib_debugger/interop/README.md` and `BRANCH_NOTES.md`
finding 10 there. It applies unchanged to espp/rtps 1.3.5 through 1.3.10 (the
rtps sources are identical across those releases). Apply from the repo root:

    git apply -p3 --directory=external/espp__rtps external/espp__rtps/espp-rtps-cyclonedds-interop.patch Once a
release contains them, delete this directory and the `override_path` line.

Diff against the pristine copy: `git log --oneline -- external/espp__rtps`,
the first commit is the unmodified 1.3.10.

## 1. Heartbeats and gaps addressed to the participant are dropped

`src/messages/MessageReceiver.cpp`, `processHeartbeatSubmessage()` /
`processGapSubmessage()`.

Cyclone sends HEARTBEAT and GAP with `readerId = ENTITYID_UNKNOWN`, which RTPS
allows. The receiver looked the reader up by that id, found none, and
discarded the submessage, so the MIB never answered a heartbeat. Cyclone
delivers discovery data only on request (NACK), so the MIB never learned the
Jetson's endpoints. Fix: resolve the reader through the writer it is matched
to, as the DATA path already does.

## 2. Pre-emptive ACKNACK uses sequence base 0

`src/entities/StatefulReader.cpp`, `sendPreemptiveAckNack()`.

RTPS 2.2 section 8.3.5.5 requires `bitmapBase >= 1`. Cyclone
(`validate_AckNack` in `q_receive.c`) only forgives base 0 for eProsima and RTI
vendor ids; embeddedRTPS announces vendor 13.37, so Cyclone logs

    recvUC: malformed packet received from vendor 13.37 length 48 state parse:shortmsg

and drops it. Fix: base 1.

## 3. ACKNACKs carry no INFO_DST and echo the heartbeat's reader id

`src/entities/StatefulReader.cpp`, every ACKNACK path.

Cyclone resolves the target writer by (destination prefix, writerId) and keys
its reader table by the ACKNACK's reader id. Without INFO_DST the prefix is
null, and with reader id 0 (copied from the heartbeat) the entry is "not a
connection", so even a well-formed ACKNACK did nothing. Fix: precede every
ACKNACK with INFO_DST naming the writer's participant and put the reader's own
entity id in it. This is what Fast DDS itself sends.

## 4. Endpoint announcements without a locator keep a placeholder address

`src/discovery/TopicData.cpp`, `readFromBuffer()`; `src/discovery/SEDPAgent.cpp`.

`TopicData()` pre-fills `unicastLocator` with `192.168.0.42:7411`, an
embeddedRTPS leftover. Cyclone omits `PID_UNICAST_LOCATOR` from SEDP
(`sedp_write_reader` passes a null address set unless network partitions are
configured) and relies on the participant's default unicast locator from SPDP.
The placeholder passed as valid, so once matched every sample for a Cyclone
reader went to 192.168.0.42: `ros2 topic list` showed the MIB's topics while
`ros2 topic echo` stayed silent. Fix: reset the locators before parsing and
fall back to the remote participant's default unicast locator
(`ParticipantProxyData::m_defaultUnicastLocatorList`).

Nothing changes on the wire for Fast DDS peers: INFO_DST before ACKNACK and
base 1 are what Fast DDS sends, and the other two paths only run on input
Fast DDS never produces.
