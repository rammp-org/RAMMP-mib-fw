# Local patches to espp/rtps 1.3.10

This directory is a copy of the `espp/rtps` component from the ESP component
registry (example/, interop/ and CHECKSUMS.json removed), selected through
`override_path` in `main/idf_component.yml`. It carries two fixes needed to
talk to ROS 2 on Cyclone DDS (`rmw_cyclonedds_cpp`, the Jetson's RMW). Both
are interoperability bugs in the embeddedRTPS engine, not RAMMP-specific, and
should go upstream to https://github.com/esp-cpp/espp (components/rtps). Once a
release contains them, delete this directory and the `override_path` line.

Diff against the pristine copy: `git log --oneline -- external/espp__rtps`,
the first commit is the unmodified 1.3.10.

## 1. Pre-emptive ACKNACK uses sequence base 0

`src/entities/StatefulReader.cpp`, `sendPreemptiveAckNack()`.

RTPS 2.2 section 8.3.5.5 requires a SequenceNumberSet's `bitmapBase >= 1`.
embeddedRTPS sent base 0. Cyclone DDS (`validate_AckNack` in `q_receive.c`)
only forgives that for eProsima and RTI vendor ids; embeddedRTPS announces
vendor 13.37, so Cyclone logs

    recvUC: malformed packet received from vendor 13.37 length 48 state parse:shortmsg

and drops the message, and the Jetson's SEDP writer never learns the MIB's
discovery readers exist. Fast DDS accepts base 0, which is why this never
showed up against `rmw_fastrtps_cpp`. Fix: base 1.

## 2. Endpoints announced without a unicast locator are unreachable

`src/discovery/SEDPAgent.cpp`, `handlePublisherReaderMessage()` and
`handleSubscriptionReaderMessage()`.

RTPS 2.2 sections 8.5.3.3 / 9.6.2.2: an endpoint whose SEDP announcement has
no `PID_UNICAST_LOCATOR` is reachable at its participant's default unicast
locators from SPDP. Fast DDS always includes the per-endpoint locator;
Cyclone DDS (`sedp_write_reader` in `q_ddsi_discovery.c`) passes a null
address set unless network partitions are configured, so it never does.
embeddedRTPS built the reader/writer proxy from the per-endpoint locator
only, so a Cyclone subscriber was matched but every sample went to an invalid
locator: `ros2 topic list` showed the MIB's topics while `ros2 topic echo`
stayed silent. Fix: when the announcement carries no UDPv4 unicast locator,
copy the first default unicast locator of the announcing participant (already
parsed into `ParticipantProxyData::m_defaultUnicastLocatorList`).
