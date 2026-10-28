"""Connection doctor for the MIB debugger.

Runs the checks a connection depends on, one after another, and says which
one fails. Use it when the page stays on "searching":

    .venv/bin/python doctor.py                 # auto-detect the board's interface
    .venv/bin/python doctor.py --interface en9 # or an address, 192.168.4.4
    .venv/bin/python doctor.py --local         # against the simulator (sim.sh)
    .venv/bin/python doctor.py --trace         # also write and summarise a CycloneDDS trace

The steps:

1. Interfaces: which one holds a 192.168.4.x lease, and does the MIB answer ping.
2. Multicast: listen on the RTPS discovery group for a few seconds and list who
   announces themselves. The MIB and the PACE RACER boards show up as vendor
   13.37 (espp's RTPS stack). Nothing from 192.168.4.x means the boards'
   multicast is not reaching this machine: switch, firewall, wrong interface.
3. DDS: join the domain the way the debugger does, list the participants and
   every topic each one offers and reads, then count the samples that arrive.
   Writers listed but no samples means the boards never learned this machine's
   readers, which is the endpoint-discovery problem, not a network one.

Lines reading "malformed packet received from vendor 13.37" are expected: the
boards send a pre-emptive ACKNACK CycloneDDS rejects, six per discovery round.
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

def step_dds(selector, local, seconds, trace_path):
    banner(f"3. DDS participant on {selector or 'default interface'} for {seconds:.0f} s")
    uri = mib_link.build_cyclonedds_uri(selector, local=local)
    if trace_path:
        trace = (f"<Tracing><Verbosity>finest</Verbosity><OutputFile>{trace_path}</OutputFile></Tracing>")
        uri = uri.replace("</Domain>", trace + "</Domain>")
    os.environ["CYCLONEDDS_URI"] = uri
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsParticipant,
                                    BuiltinTopicDcpsPublication, BuiltinTopicDcpsSubscription)
    from cyclonedds.core import Policy, Qos
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader, Subscriber
    from cyclonedds.topic import Topic

    dp = DomainParticipant(0)
    my_prefix = str(dp.guid)[:27]
    r_part = BuiltinDataReader(dp, BuiltinTopicDcpsParticipant)
    r_pub = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
    r_sub = BuiltinDataReader(dp, BuiltinTopicDcpsSubscription)
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(8))
    sub = Subscriber(dp)

    def reader(topic, datatype):
        return DataReader(sub, Topic(dp, topic, datatype, qos=qos), qos=qos)

    readers = {
        "rammp/mib/status": reader(msg.TOPIC_MIB_STATUS, msg.MibStatus),
        "rammp/mib/can_status": reader(msg.TOPIC_MIB_CAN_STATUS, msg.CanStatus),
    }
    for axis in mib_link.ALL_AXES:
        readers[msg.motor_state_topic(axis)] = reader(msg.motor_state_topic(axis), msg.MotorState)
    counts = collections.Counter()
    last_status = None
    end = time.time() + seconds
    while time.time() < end:
        for topic, rd in readers.items():
            for sample in rd.take(16):
                if sample.__class__.__name__ in ("MibStatus", "CanStatus", "MotorState"):
                    counts[topic] += 1
                    if topic == "rammp/mib/status":
                        last_status = sample
        time.sleep(0.1)

    participants = [str(p.key) for p in r_part.take(64)]
    pubs = collections.defaultdict(list)
    subs = collections.defaultdict(list)
    for w in r_pub.take(256):
        pubs[str(w.participant_key)[:27]].append(w.topic_name)
    for r in r_sub.take(256):
        subs[str(r.participant_key)[:27]].append(r.topic_name)
    remote = [p for p in participants if p[:27] != my_prefix]
    print(f"   participants discovered: {len(remote)} remote")
    for p in remote:
        pre = p[:27]
        print(f"   {pre}  writes {len(pubs.get(pre, []))} topics, reads {len(subs.get(pre, []))}")
        for t in sorted(pubs.get(pre, [])):
            print(f"       -> {t}")
    mib_pub = [pre for pre, topics in pubs.items() if "rammp/mib/status" in topics and pre != my_prefix]
    verdict(bool(mib_pub), "MIB endpoints discovered (rammp/mib/status offered)" if mib_pub else
            "no participant offers rammp/mib/status: the MIB was not discovered over DDS "
            "(firewall blocking unicast UDP, or a different interface than step 2)")
    print("   samples received:")
    for topic in readers:
        print(f"       {counts[topic]:4d}  {topic}")
    ok = counts["rammp/mib/status"] > 0
    verdict(ok, "MibStatus samples arriving" + (
        f" (state {msg.MibSystemState(last_status.systemState).name})" if ok else
        ": none. The MIB offers the topic but never sent to this machine, so it never "
        "learned this machine's reader. That is the endpoint-discovery handshake, not "
        "the network: the boards need the espp patch in interop/ (BRANCH_NOTES.md "
        "finding 10)."))
    if mib_pub and not ok:
        print("   hint: run again with --trace to see what the MIB answered.")
    del readers, r_part, r_pub, r_sub, sub, dp
    return ok, (mib_pub[0] if mib_pub else None)


# ------------------------------------------------------------------ trace

def summarise_trace(path, mib_prefix):
    banner(f"4. CycloneDDS trace summary ({path})")
    try:
        with open(path, "rb") as handle:
            text = handle.read().decode("utf-8", "replace")
    except OSError as exc:
        print(f"   could not read the trace: {exc}")
        return
    by_port = collections.Counter(re.findall(rf"from udp/{re.escape(MIB_ADDRESS)}:(\d+)", text))
    print("   packets from the MIB by source port: " + (", ".join(
        f"{p} x{n}" for p, n in sorted(by_port.items())) or "none"))
    print("   (7400 = discovery multicast, 7410 = discovery unicast, 7411 = user data)")
    malformed = len(re.findall(r"malformed packet received", text))
    print(f"   malformed (base-0 pre-emptive ACKNACK) packets dropped: {malformed}")
    if mib_prefix:
        # The trace prints a GUID prefix as three 32-bit groups without leading
        # zeros ("e8f60ae0:c574000d:2500a55a"); the API gives dashed hex.
        hexs = mib_prefix.replace("-", "")[:24]
        short = ":".join(hexs[i:i + 8].lstrip("0") or "0" for i in (0, 8, 16))
        acks = len(re.findall(rf"recv(?:UC)?: ACKNACK\([^)]*{re.escape(short)}", text))
        matched = f"scanning all rds of topic rammp/mib/status" in text
        print(f"   valid ACKNACKs from the MIB to this machine's discovery writers: {acks}")
        verdict(matched, "this machine matched the MIB's status writer to its reader")
        verdict(acks > 0, "the MIB acknowledged this machine's endpoint discovery" + ("" if acks else
                ": it never did, so it cannot know this machine's readers"))
    sends_uc = len(re.findall(rf"nn_xpack_send.*udp/{re.escape(MIB_ADDRESS)}:7410", text))
    sends_mc = len(re.findall(r"nn_xpack_send.*udp/239\.255\.0\.1:7400", text))
    print(f"   discovery sent by this machine: {sends_uc} unicast to the MIB, {sends_mc} multicast")


def main():
    parser = argparse.ArgumentParser(description="Check the path between this machine and the MIB.")
    parser.add_argument("--interface", help="interface name or IPv4 address facing the board")
    parser.add_argument("--local", action="store_true", help="test against the simulator on loopback")
    parser.add_argument("--seconds", type=float, default=8.0, help="how long to listen in steps 2 and 3")
    parser.add_argument("--trace", action="store_true", help="write a CycloneDDS trace and summarise it")
    parser.add_argument("--trace-file", default="mib_doctor_trace.log")
    args = parser.parse_args()

    address, name = step_interfaces(args)
    if address is None:
        return 1
    if args.local:
        selector = None
    else:
        step_multicast(address, args.seconds)
        selector = app.interface_for_cyclonedds(address if sys.platform == "win32" else name)
    ok, mib_prefix = step_dds(selector, args.local, args.seconds, args.trace_file if args.trace else None)
    if args.trace:
        summarise_trace(args.trace_file, mib_prefix)
    banner("Result")
    print("   connected: the debugger will work on this interface." if ok else
          "   not connected: see the first FAIL above.")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
