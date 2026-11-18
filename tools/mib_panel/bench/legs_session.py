"""Long-lived panel-protocol session on the MIB console UART.

legs_session.py <logfile> <cmdfile> <jsonl>
- Appends every console byte (log text) to <logfile>, with '#host,<t>,...' marks.
- Parses service-link frames out of the stream; every MIB-module reply goes to <jsonl>
  as {"t": host seconds, "type": ..., "payload"/"status"...}.
- Executes each new line appended to <cmdfile>:
    status                  one STATUS request
    poll <ms>               request STATUS every <ms> (0 = off)
    move <leg> <counts>     LEG_MOVE absolute      (leg = index in MIBconfig.hpp Leg order)
    mover <leg> <delta>     LEG_MOVE relative
    jog <leg> <delta>       LEG_JOG once
    hold <leg> <step> <s>   LEG_JOG <step> every 200 ms for <s> seconds, then LEG_JOG 0
    estop                   ESTOP engage
    clear                   ESTOP start clear attempt
    QUIT                    close
Only leg 0 (MR) is allowed: no frame is ever built for ML or the carriages.
"""
import json, os, struct, sys, threading, time, zlib

import serial

PORT = "/dev/cu.usbmodem5C372451831"
BAUD = 115200
MIB = 0x40
LEG_MOVE, LEG_JOG, ESTOP, STATUS = 0x0A, 0x0C, 0x08, 0x09
ALLOWED_LEGS = {0}  # MR only (Alex, 2026-11-17): never ML, never the carriages

log_path, cmd_path, jsonl_path = sys.argv[1], sys.argv[2], sys.argv[3]
open(cmd_path, "a").close()
cmd_pos = os.path.getsize(cmd_path)

s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, BAUD, 0.01
s.dtr = False
s.rts = False
s.open()
t0 = time.time()
logf = open(log_path, "ab", buffering=0)
jsonf = open(jsonl_path, "a", buffering=1)
wlock = threading.Lock()


def now():
    return time.time() - t0


def mark(text):
    logf.write(f"\n#host,{now():.3f},{text}\n".encode())


def frame(mod, typ, payload=b""):
    head = struct.pack("<HBBBI", 0x4F54, 0x10, mod, typ, len(payload)) + payload
    return head + struct.pack("<I", zlib.crc32(head) & 0xFFFFFFFF)


def send(typ, payload=b"", what=""):
    with wlock:
        s.write(frame(MIB, typ, payload))
    if what:
        mark(f"sent {what}")
        jsonf.write(json.dumps({"t": now(), "sent": what}) + "\n")


def leg_ok(leg):
    if leg not in ALLOWED_LEGS:
        mark(f"REFUSED locally: leg {leg} is not MR; nothing sent")
        return False
    return True


buf = bytearray()


def parse(data):
    """Split frames out of the byte stream; everything else is console text."""
    buf.extend(data)
    out = bytearray()
    while True:
        i = buf.find(b"\x54\x4f")
        if i < 0:
            keep = 1 if buf.endswith(b"\x54") else 0
            out += buf[: len(buf) - keep]
            del buf[: len(buf) - keep]
            break
        out += buf[:i]
        del buf[:i]
        if len(buf) < 9:
            break
        flags, mod, typ = buf[2], buf[3], buf[4]
        hdr = 9 + (2 if flags & 0x02 else 0)
        if len(buf) < hdr:
            break
        n = struct.unpack_from("<I", buf, hdr - 4)[0]
        if n > 4096 or (flags >> 4) != 1:
            out += buf[:1]
            del buf[:1]
            continue
        if len(buf) < hdr + n + 4:
            break
        body = bytes(buf[: hdr + n])
        crc = struct.unpack_from("<I", buf, hdr + n)[0]
        if zlib.crc32(body) & 0xFFFFFFFF != crc:
            out += buf[:1]
            del buf[:1]
            continue
        payload = body[hdr:]
        del buf[: hdr + n + 4]
        if mod == MIB:
            rec = {"t": now(), "type": typ}
            if typ == 0x89:
                global status_sent
                status_sent = None
                try:
                    rec["status"] = json.loads(payload.decode())
                except ValueError:
                    rec["raw"] = payload.decode(errors="replace")
            else:
                rec["payload"] = payload.hex()
                if typ == 0x8F:
                    rec["error"] = payload[1:].decode(errors="replace")
            jsonf.write(json.dumps(rec) + "\n")
        elif mod == 0x10 and typ == 0x82:
            out += payload
    if out:
        logf.write(bytes(out))


poll_ms = 0
next_poll = 0.0
status_sent = None
hold = None  # (leg, step, until, next)

mark(f"session open {time.strftime('%Y-%m-%dT%H:%M:%S')}")
while True:
    data = s.read(65536)
    if data:
        parse(data)
    t = now()
    # One STATUS in flight at a time (as the panel does): a reply is ~5 KB, and requests
    # sent faster than the link drains them queue up ahead of every other command.
    if poll_ms and t >= next_poll and (status_sent is None or t - status_sent > 1.0):
        send(STATUS)
        status_sent = t
        next_poll = t + poll_ms / 1000.0
    if hold:
        leg, step, until, nxt = hold
        if t >= until:
            send(LEG_JOG, struct.pack("<Bi", leg, 0), f"jog {leg} 0 (release)")
            hold = None
        elif t >= nxt:
            send(LEG_JOG, struct.pack("<Bi", leg, step), f"jog {leg} {step}")
            hold = (leg, step, until, nxt + 0.2)
    if os.path.getsize(cmd_path) > cmd_pos:
        with open(cmd_path) as c:
            c.seek(cmd_pos)
            new = c.read()
            cmd_pos = c.tell()
        for line in new.splitlines():
            w = line.split()
            if not w:
                continue
            try:
                if w[0] == "QUIT":
                    mark("quit")
                    s.close()
                    sys.exit(0)
                elif w[0] == "baud":
                    # SET_BAUD (system module, UART only): OK at the old rate, then it switches.
                    # The MIB returns to 115200 3 s after the last byte from us, so keep polling.
                    rate = int(w[1])
                    with wlock:
                        s.write(frame(0x20, 0x03, struct.pack("<I", rate)))
                    time.sleep(0.3)
                    parse(s.read(65536))
                    s.baudrate = rate
                    mark(f"baud {rate}")
                elif w[0] == "status":
                    send(STATUS)
                elif w[0] == "poll":
                    poll_ms = int(w[1])
                    mark(f"poll {poll_ms} ms")
                elif w[0] in ("move", "mover"):
                    leg, counts = int(w[1]), int(w[2])
                    if leg_ok(leg):
                        rel = 1 if w[0] == "mover" else 0
                        send(LEG_MOVE, struct.pack("<BBi", leg, rel, counts), line)
                elif w[0] == "jog":
                    leg, delta = int(w[1]), int(w[2])
                    if leg_ok(leg):
                        send(LEG_JOG, struct.pack("<Bi", leg, delta), line)
                elif w[0] == "hold":
                    leg, step, secs = int(w[1]), int(w[2]), float(w[3])
                    if leg_ok(leg):
                        mark(f"hold {leg} {step} for {secs} s")
                        hold = (leg, step, t + secs, t)
                elif w[0] == "estop":
                    send(ESTOP, bytes([1]), "estop engage")
                elif w[0] == "clear":
                    send(ESTOP, bytes([2]), "estop clear")
                else:
                    mark(f"unknown command: {line}")
            except (IndexError, ValueError) as e:
                mark(f"bad command {line!r}: {e}")
