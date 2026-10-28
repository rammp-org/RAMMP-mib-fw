# espp RTPS: interop and multi-homed-host patches

Two patches against espp's `components/rtps`, both from bench work on the
chair in October 2026, kept here for upstreaming to esp-cpp/espp.

## Multicast on the configured interface

`espp-rtps-multicast-interface.patch`: the host transport joined the
discovery multicast group and sent to it without naming an interface, so
the OS default multicast route decided, which on a laptop with Wi-Fi beside
a wired link to the boards is the Wi-Fi. The patch has `Domain` hand the
transport the participant's own address and the transport pin joins
(`add_multicast_group`) and egress (`multicast_interface` on the socket
configs) to it. Harmless on an ESP32 with one interface. Verified against
the chair from a Mac with Wi-Fi up: with the pip wheel the participant's
announcements never appear on the chair's link; with the patch they do and
the boards' endpoint announcements come back. Until a wheel carries it,
SETUP.md's "A laptop with more than one network" has the two workarounds
(Wi-Fi off, or a route for 239.255.0.0/16).

## CycloneDDS interop

`espp-rtps-cyclonedds-interop.patch` is the fix for `BRANCH_NOTES.md` finding
10: four defects in espp's `components/rtps` that stop every sample between
the boards and a CycloneDDS peer. The debugger itself no longer needs it: it
runs espp's own stack since October 2026. The patch matters for anyone
pointing a CycloneDDS participant, or a ROS 2 node on the CycloneDDS RMW, at
the chair. It applies to espp at
rtps 1.3.5 through 1.3.10 (the rtps sources are identical across them):

```
cd <espp checkout>
git apply <this dir>/espp-rtps-cyclonedds-interop.patch
```

To use it, every board on the chair needs it: the MIB and both PACE RACER
boards. Each firmware pulls `espp/rtps` from the component registry, so until
an espp release carries the fix, point the firmware at a patched checkout with
an `override_path` entry for `espp/rtps` in its `main/idf_component.yml`.

## Reproducing without the chair

espp builds its RTPS stack for the host, so the handshake can be exercised
on a laptop against the two CycloneDDS scripts here. From the espp checkout,
after `git submodule update --init --recursive`:

```
cmake -S lib -B lib/build -DCMAKE_BUILD_TYPE=Release -DESPP_INSTALL=ON \
      -DESPP_BUILD_PYTHON=OFF -DCMAKE_INSTALL_PREFIX=$PWD/install
cmake --build lib/build --target install --parallel 8
cmake -S pc -B pc/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$PWD/install
cmake --build pc/build --target rtps_interop_pub rtps_interop_sub --parallel 8
```

On a Mac with only the command-line tools, add
`-DCMAKE_CXX_FLAGS="-isystem /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1"`
to both configure steps.

Then, with `<if>` the interface name CycloneDDS should use and `<ip>` its
address, in a venv with `pip install cyclonedds==0.10.5` (Python 3.9 or 3.10;
the debugger's own venv no longer has it):

```
# espp writer -> CycloneDDS reader (reliable; "be" for best-effort on both)
.venv/bin/python cyclone_sub.py <if> 15 "" spdp rel &
pc/build/rtps_interop_pub rt/chatter std_msgs::msg::dds_::String_ 1 40 250 <ip>

# CycloneDDS writer -> espp reader
.venv/bin/python cyclone_pub.py <if> 16 rel &
pc/build/rtps_interop_sub rt/chatter std_msgs::msg::dds_::String_ 1 10 14 <ip>
```

Unpatched: `samples received: 0` and a `malformed packet received from
vendor 13.37` line per discovery round. Patched: samples flow both ways and
the terminal stays quiet.
