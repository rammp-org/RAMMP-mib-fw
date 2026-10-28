"""CycloneDDS reliable subscriber on rt/chatter (std_msgs String), the peer
for espp's rtps_interop_pub. Prints discovery and the samples it receives."""
import os, sys, time, collections
from dataclasses import dataclass
iface = sys.argv[1] if len(sys.argv) > 1 else "en0"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 15
trace = sys.argv[3] if len(sys.argv) > 3 else ""
mc = sys.argv[4] if len(sys.argv) > 4 else "true"
best_effort = len(sys.argv) > 5 and sys.argv[5] == "be"
uri = ('<CycloneDDS><Domain><General><Interfaces>'
       f'<NetworkInterface name="{iface}" priority="default" multicast="true"/>'
       f'</Interfaces><AllowMulticast>{mc}</AllowMulticast></General>')
if trace:
    uri += f"<Tracing><Verbosity>finest</Verbosity><OutputFile>{trace}</OutputFile></Tracing>"
uri += "</Domain></CycloneDDS>"
os.environ["CYCLONEDDS_URI"] = uri
from cyclonedds.idl import IdlStruct
from cyclonedds.domain import DomainParticipant
from cyclonedds.core import Policy, Qos
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication

@dataclass
class String_(IdlStruct, typename="std_msgs::msg::dds_::String_"):
    data: str

dp = DomainParticipant(0)
qos = Qos(Policy.Reliability.BestEffort if best_effort else Policy.Reliability.Reliable(max_blocking_time=1000000000), Policy.History.KeepLast(16))
rd = DataReader(Subscriber(dp), Topic(dp, "rt/chatter", String_, qos=qos), qos=qos)
pubs = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
n = 0; seen = set(); t0 = time.time()
while time.time() - t0 < seconds:
    for w in pubs.take(32):
        key = (w.topic_name, str(w.participant_key)[:27])
        if key not in seen:
            seen.add(key); print(f"[{time.time()-t0:5.1f}s] discovered writer {w.topic_name} [{w.type_name}] from {key[1]}", flush=True)
    for s in rd.take(32):
        if isinstance(s, String_):
            n += 1
            if n <= 3 or n % 10 == 0: print(f"[{time.time()-t0:5.1f}s] sample {n}: {s.data[:40]!r}", flush=True)
    time.sleep(0.05)
print(f"samples received: {n}")
