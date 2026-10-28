"""CycloneDDS publisher on rt/chatter (std_msgs String), the peer for espp's
rtps_interop_sub. argv: iface seconds reliability(rel|be)."""
import os, sys, time
from dataclasses import dataclass
iface = sys.argv[1] if len(sys.argv) > 1 else "en0"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 15
best_effort = len(sys.argv) > 3 and sys.argv[3] == "be"
os.environ["CYCLONEDDS_URI"] = ('<CycloneDDS><Domain><General><Interfaces>'
    f'<NetworkInterface name="{iface}" priority="default" multicast="true"/>'
    '</Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>')
from cyclonedds.idl import IdlStruct
from cyclonedds.domain import DomainParticipant
from cyclonedds.core import Policy, Qos
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.topic import Topic
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsSubscription

@dataclass
class String_(IdlStruct, typename="std_msgs::msg::dds_::String_"):
    data: str

dp = DomainParticipant(0)
qos = Qos(Policy.Reliability.BestEffort if best_effort else Policy.Reliability.Reliable(max_blocking_time=1000000000), Policy.History.KeepLast(16))
wr = DataWriter(Publisher(dp), Topic(dp, "rt/chatter", String_, qos=qos), qos=qos)
subs = BuiltinDataReader(dp, BuiltinTopicDcpsSubscription)
t0 = time.time(); n = 0; seen = set()
while time.time() - t0 < seconds:
    for r in subs.take(32):
        key = (r.topic_name, str(r.participant_key)[:27])
        if key not in seen: seen.add(key); print(f"[{time.time()-t0:5.1f}s] discovered reader {r.topic_name} from {key[1]}", flush=True)
    n += 1; wr.write(String_(data=f"cyclone {n}")); time.sleep(0.25)
print(f"samples written: {n}")
