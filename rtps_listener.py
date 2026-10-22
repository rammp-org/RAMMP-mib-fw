"""Listen to the MIB/joystick RTPS topics from the Mac and log every sample as JSON lines."""
import argparse, json, os, struct, sys, time
sys.path.insert(0, os.path.expanduser("~/code-stuff/pace-hmi-fw/scripts"))
import rtps_host as rh
import rammp_rtps as spec

TOPICS = {
    "rammp/mcb/diagnostics": "rammp/msg/Diagnostics",
    "rammp/mib/status": "rammp/msg/MibStatus",
    "rammp/joystick/xy_twist": "rammp/msg/XYTwist",
    "rammp/joystick/drive_command": "rammp/msg/DriveCommand",
    "rammp/mib/motor_command/drive_left": "rammp/msg/MotorCommand",
    "rammp/mib/motor_command/drive_right": "rammp/msg/MotorCommand",
}

class Sniffer(rh.RtpsHostHarness):
    def __init__(self, args, out):
        super().__init__(args)
        self.local_readers = [rh.ReaderConfig(t, ty, False, i) for i, (t, ty) in enumerate(TOPICS.items())]
        self.local_writers = [rh.WriterConfig("rammp/joystick/drive_command", "rammp/msg/DriveCommand", False, 0),
                              rh.WriterConfig("rammp/joystick/xy_twist", "rammp/msg/XYTwist", False, 1)]
        self.out = out

    def publish_raw(self, writer, payload):
        msg = self.build_data_message(writer, payload)
        for dest in self._build_user_targets(writer):
            self.send_user_datagram(msg, dest)

    def joystick_script(self, start_delay):
        """Act as the joystick: enable drive, sweep the stick, disable."""
        import math
        drive_w, xy_w = self.local_writers
        time.sleep(start_delay)
        def log(kind, **kw):
            self.out.write(json.dumps({"t": time.time(), "topic": "sent/" + kind, **kw}) + "\n"); self.out.flush()
        for _ in range(3):
            self.publish_raw(drive_w, spec.pack_drive_command(1, 1)); log("drive_command", request=1, profile=1); time.sleep(0.1)
        t0 = time.time()
        while time.time() - t0 < 12:
            ph = (time.time() - t0) / 12 * 2 * math.pi
            x, y = 0.6 * math.sin(2 * ph), 0.8 * math.sin(ph)
            self.publish_raw(xy_w, spec.encode(spec.MSG_XY_TWIST, x, y, 0.0, 0)); log("xy_twist", x=x, y=y)
            time.sleep(0.05)
        for _ in range(10):
            self.publish_raw(xy_w, spec.encode(spec.MSG_XY_TWIST, 0.0, 0.0, 0.0, 0)); log("xy_twist", x=0.0, y=0.0); time.sleep(0.05)
        for _ in range(3):
            self.publish_raw(drive_w, spec.pack_drive_command(0, 1)); log("drive_command", request=0, profile=1); time.sleep(0.1)

    def handle_user_packet(self, packet, sender_ip, sender_port):
        now = time.time()
        for prefix, writer_id, payload, reader_id in rh.parse_rtps_data_messages(packet):
            topic = self.topic_for_sample(prefix, writer_id, reader_id)
            if topic not in TOPICS:
                continue
            rec = {"t": now, "topic": topic, "from": sender_ip}
            try:
                if topic.endswith("diagnostics"):
                    seq, items = spec.unpack_diagnostics(payload); rec.update(seq=seq, items=items)
                elif topic.endswith("status"):
                    m = spec.unpack_mib_status(payload); rec["status"] = list(m) if m else None
                elif topic.endswith("xy_twist"):
                    rec["xy"] = list(spec.unpack_xy_twist(payload))
                elif topic.endswith("drive_command"):
                    rec["drive"] = list(spec.unpack_drive_command(payload))
                else:
                    body = payload[4:]
                    rec.update(seq=body[0], state=body[1], mode=body[2],
                               velocity=struct.unpack_from("<f", body, 8)[0])
            except Exception as e:  # keep listening; record the failure
                rec["error"] = repr(e); rec["raw"] = payload.hex()
            self.out.write(json.dumps(rec, default=str) + "\n"); self.out.flush()

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("duration", type=float)
    a = ap.parse_args()
    args = argparse.Namespace(node_name="imu_pr_sniffer", domain_id=0, participant_id=14,
        bind_address="192.168.4.4", advertised_address="192.168.4.4", multicast_interface="192.168.4.4",
        multicast_group="239.255.0.1", peer=["192.168.4.1"], peer_participant_ids=[0, 1, 2, 3], trace_packets=os.environ.get("TRACE")=="1",
        enclave="/", subscribe_topic=list(TOPICS), publish_topic="rammp/joystick/drive_command", publish_value=0, publish_interval=0.0,
        echo_received=False, reliable=False, type_name="rammp/msg/Diagnostics", subscribe_type_name=None,
        announce_period=1.0, duration=a.duration, quiet=os.environ.get("TRACE")!="1")
    ap2 = os.environ.get("JOY_DELAY")
    with open(a.out, "w") as f:
        sn = Sniffer(args, f)
        if ap2:
            import threading
            threading.Thread(target=sn.joystick_script, args=(float(ap2),), daemon=True).start()
        sn.run()
