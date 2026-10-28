"""Connection doctor for the MIB debugger.

Runs the checks a connection depends on, one after another, and says which
one fails. Use it when the page stays on "searching":

    .venv/bin/python doctor.py                 # auto-detect the board's interface
    .venv/bin/python doctor.py --interface en9 # or an address, 192.168.4.4
    .venv/bin/python doctor.py --local         # against the simulator (sim.sh)

The steps:

1. Interfaces: which one holds a 192.168.4.x lease, and does the MIB answer ping.
2. Multicast: listen on the RTPS discovery group for a few seconds and list who
   announces themselves. The MIB and the PACE RACER boards show up as vendor
   13.37 (espp's RTPS stack). Nothing from 192.168.4.x means the boards'
   multicast is not reaching this machine: switch, firewall, wrong interface.
3. RTPS: check which interface the OS sends discovery multicast on (the stack
   joins the group on the default one, so Wi-Fi beside the chair's adapter
   wins unless a route says otherwise), then join the domain the way the
   debugger does, count how many readers found a remote writer, and count the
   samples that arrive on the MIB's status topics and the boards' state topics.
"""

import argparse
import collections
import os
import re
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import app  # noqa: E402  (interface listing shared with the GUI)
import mib_link  # noqa: E402
import mib_messages as msg  # noqa: E402

MIB_ADDRESS = mib_link.DEFAULT_MIB_ADDRESS
RTPS_MULTICAST = "239.255.0.1"
RTPS_SPDP_PORT = 7400
VENDORS = {(13, 37): "espp (MIB / PACE RACER)", (1, 16): "CycloneDDS", (1, 15): "FastDDS",
           (1, 1): "RTI Connext", (1, 10): "RTI Micro"}


def banner(text):
    print()
    print(f"== {text}")


def verdict(ok, text):
    print(("   OK   " if ok else "   FAIL ") + text)
    return ok


# ------------------------------------------------------------------ step 1

def step_interfaces(args):
    banner("1. Interfaces")
    entries = app.list_interfaces()
    if not entries:
        verdict(False, "no interface with an IPv4 address found (is ifconfig or ip installed?)")
        return None, None
    for e in entries:
        print(f"   {e['name']:<24} {e['address']:<16}" + ("  <- board subnet" if e["mib"] else ""))
    if args.local:
        return "127.0.0.1", "lo"
    chosen = args.interface
    if chosen:
        match = [e for e in entries if e["name"] == chosen or e["address"] == chosen]
        if not match:
            verdict(False, f"{chosen!r} is not in the list above")
            return None, None
        entry = match[0]
    else:
        mib = [e for e in entries if e["mib"]]
        if not mib:
            verdict(False, f"no interface holds a {app.MIB_SUBNET_PREFIX}x address: no lease from the MIB. "
                           "Check the cable and link, and that the interface uses DHCP (or a static "
                           f"{app.MIB_SUBNET_PREFIX}x address) rather than an address on another subnet.")
            return None, None
        entry = mib[0]
    verdict(True, f"using {entry['name']} at {entry['address']}")
    ping = ["ping", "-n" if sys.platform == "win32" else "-c", "2", MIB_ADDRESS]
    try:
        rc = subprocess.run(ping, capture_output=True, text=True, timeout=8).returncode
    except (OSError, subprocess.TimeoutExpired):
        rc = -1
    verdict(rc == 0, f"ping {MIB_ADDRESS}" + ("" if rc == 0 else ": no answer (MIB off, no link, or firewall)"))
    return entry["address"], entry["name"]


# ------------------------------------------------------------------ step 2

def step_multicast(address, seconds):
    banner(f"2. Discovery multicast {RTPS_MULTICAST}:{RTPS_SPDP_PORT} on {address} for {seconds:.0f} s")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        sock.bind(("" if sys.platform != "win32" else address, RTPS_SPDP_PORT))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                        socket.inet_aton(RTPS_MULTICAST) + socket.inet_aton(address))
    except OSError as exc:
        verdict(False, f"could not listen: {exc}")
        return {}
    sock.settimeout(0.5)
    seen = collections.Counter()
    vendors = {}
    end = time.time() + seconds
    while time.time() < end:
        try:
            data, (src, _) = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if data[:4] != b"RTPS" or len(data) < 8:
            continue
        seen[src] += 1
        vendors[src] = (data[6], data[7])
    sock.close()
    for src, n in sorted(seen.items()):
        v = vendors[src]
        print(f"   {src:<16} {n:3d} packets   vendor {v[0]}.{v[1]} {VENDORS.get(v, '')}")
    boards = [s for s in seen if s.startswith(app.MIB_SUBNET_PREFIX) and vendors[s] == (13, 37)]
    mib = MIB_ADDRESS in boards
    verdict(mib, f"MIB announcements from {MIB_ADDRESS}" + ("" if mib else
            ": none received. The boards' multicast does not reach this interface "
            "(managed switch with IGMP snooping, firewall, or the wrong interface)"))
    others = [b for b in boards if b != MIB_ADDRESS]
    print(f"   other espp boards announcing: {', '.join(others) if others else 'none'}")
    return seen


# ------------------------------------------------------------------ step 3

def step_rtps(address, local, seconds, name=None):
    banner(f"3. RTPS participant on {'loopback' if local else address} for {seconds:.0f} s")
    if not local and name:
        egress = app.multicast_egress_interface()
        if egress is not None:
            if not verdict(egress == name, f"discovery multicast leaves through {egress}"
                           + ("" if egress == name else f", not {name}")):
                print("   " + app.multicast_route_advice(name))
                print("   (the stack joins the discovery group on the OS default interface; "
                      "step 2 joined it on the chosen one, which is why step 2 passed)")
    import rtps_io
    participant = rtps_io.Participant(interface_address="127.0.0.1" if local else address,
                                      log_level="error")
    try:
        participant.start()
    except RuntimeError as exc:
        verdict(False, str(exc))
        return False
    readers = {
        "rammp/mib/status": participant.reader(msg.TOPIC_MIB_STATUS, msg.MibStatus),
        "rammp/mib/can_status": participant.reader(msg.TOPIC_MIB_CAN_STATUS, msg.CanStatus),
    }
    for axis in mib_link.ALL_AXES:
        readers[msg.motor_state_topic(axis)] = participant.reader(msg.motor_state_topic(axis), msg.MotorState)
    counts = collections.Counter()
    last_status = None
    end = time.time() + seconds
    while time.time() < end:
        for topic, rd in readers.items():
            for sample in rd.take(16):
                counts[topic] += 1
                if topic == "rammp/mib/status":
                    last_status = sample
        time.sleep(0.1)
    matched = participant.matched_readers
    participant.stop()
    print(f"   readers that found a remote writer: {matched} of {len(readers)}")
    verdict(matched > 0, "endpoint discovery with at least one board" if matched > 0 else
            "no reader matched any writer: the boards were not discovered over RTPS "
            "(firewall blocking UDP, or a different interface than step 2)")
    print("   samples received:")
    for topic in readers:
        print(f"       {counts[topic]:4d}  {topic}")
    ok = counts["rammp/mib/status"] > 0
    verdict(ok, "MibStatus samples arriving" + (
        f" (state {msg.MibSystemState(last_status.systemState).name})" if ok else
        ": none in the window. If step 2 saw the MIB and a reader matched, the MIB is up but not "
        "publishing status (check its serial log); if nothing matched, see above."))
    return ok


def main():
    parser = argparse.ArgumentParser(description="Check the path between this machine and the MIB.")
    parser.add_argument("--interface", help="interface name or IPv4 address facing the board")
    parser.add_argument("--local", action="store_true", help="test against the simulator on loopback")
    parser.add_argument("--seconds", type=float, default=8.0, help="how long to listen in steps 2 and 3")
    args = parser.parse_args()

    address, name = step_interfaces(args)
    if address is None:
        return 1
    if not args.local:
        step_multicast(address, args.seconds)
    ok = step_rtps(address, args.local, args.seconds, name)
    banner("Result")
    print("   connected: the debugger will work on this interface." if ok else
          "   not connected: see the first FAIL above.")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
