"""Live plot of every MCP266 encoder channel on the bus (nodes 3, 2 and 10, M1 and M2) from the Actuator example.

Polls the bench console with SDO reads only (`sdor <node> 0x6064 0 4` for M1, `0x6864` for M2),
round robin, about 11 ms per read. Serves a page on http://localhost:8790/ and logs
every sample to runs/encoder_scope_<time>.csv. Needs the Actuator example firmware
(components/Actuator/example, console at 921600 baud) and pyserial.

    python3 tools/encoder_scope/encoder_scope.py [--port /dev/cu.usbmodem...] [--http 8790]

Opening the port does not reset the MIB (DTR/RTS held deasserted).
"""
import argparse
import collections
import glob
import json
import os
import queue
import re
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
PANEL = os.path.join(HERE, "..", "mib_panel")
# (name, node, channel). M1's position actual is 0x6064, M2's 0x6864. Node 10 answers `scan` but has
# no leg in MIBconfig.hpp; the firmware needs a client for it (bench-only line in bsp.cpp).
ALL_CHANNELS = [("MR", 3, 1), ("ML", 3, 2), ("LCarr", 2, 1), ("RCarr", 2, 2), ("FC", 10, 1), ("RC", 10, 2)]
REPLY = re.compile(r"n\d+ 0x[0-9A-Fa-f]+:0 = (-?\d+)\s|(read n\d+ [^\n]*failed[^\n]*)|(no leg[^\n]*)")


def sdor(node, channel):
    return f"sdor {node} {'0x6064' if channel == 1 else '0x6864'} 0 4"
PROMPT = "actuator> "
HISTORY_S = 600
GUARD_SPEED = 4000  # counts/s (POST /guard to change) in a `wait` trace: oscillation, not a move


class Scope:
    def __init__(self, port, csv_path, channels):
        self.channels = channels
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 921600, 0.005
        self.ser.dtr = self.ser.rts = False
        self.ser.open()
        self.t0 = time.time()
        self.history = collections.deque()  # (t, ch, value or None)
        self.lock = threading.Lock()
        self.clients = []
        self.commands = queue.Queue()
        self.paused = False
        self.errors = 0
        self.drops = 0
        self.csv = open(csv_path, "w", buffering=1)
        self.csv.write("t_s,unix_s,channel,counts,error\n")
        self.csv_path = csv_path

    def _exchange(self, cmd, done, timeout):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        buf, t = "", time.time()
        guard = cmd.strip() in ("wait", "trace")
        tripped = False
        while time.time() - t < timeout:
            buf += self.ser.read(4096).decode(errors="replace")
            if guard and not tripped and self._too_fast(buf):
                # Any key ends `wait`; then disable the drive at once.
                tripped = True
                self.ser.write(b"x")
                time.sleep(0.05)
                self.ser.write(b"stop\n")
                buf += f"\nGUARD: traced speed over {GUARD_SPEED} counts/s, sent stop\n"
                print(f"{time.strftime('%H:%M:%S')} guard tripped, sent stop", flush=True)
            if done(buf):
                if tripped and "stop" not in buf.rsplit("GUARD", 1)[-1].split("sent stop", 1)[-1]:
                    pass
                break
        if tripped:  # collect the stop's reply too
            end = time.time() + 1.5
            while time.time() < end:
                buf += self.ser.read(4096).decode(errors="replace")
        return buf

    @staticmethod
    def _too_fast(buf):
        """True if the `wait` trace moved faster than GUARD_SPEED over any ~100 ms span."""
        tr = [(int(a), int(b)) for a, b in re.findall(r"at (\d+) (-?\d+)\r?\n", buf)]
        for k in range(max(0, len(tr) - 12), len(tr)):
            for j in range(k - 1, -1, -1):
                dt = tr[k][0] - tr[j][0]
                if dt >= 80:
                    if abs(tr[k][1] - tr[j][1]) / (dt / 1000) > GUARD_SPEED:
                        return True
                    break
        return False

    def broadcast(self, event, data):
        msg = f"event: {event}\ndata: {json.dumps(data)}\n\n".encode()
        with self.lock:
            for q in self.clients:
                q.put(msg)

    def run(self):
        """Poll forever. If the USB link drops (the CH343 re-enumerates), wait for the port and reopen."""
        while True:
            try:
                self._poll()
            except (serial.SerialException, OSError) as e:
                self.drops += 1
                t = time.time() - self.t0
                print(f"{time.strftime('%H:%M:%S')} serial link lost ({e}); reopening", flush=True)
                self.csv.write(f"{t:.4f},{t + self.t0:.4f},,,link lost: {e}\n")
                self.broadcast("link", {"up": False, "drops": self.drops})
                try:
                    self.ser.close()
                except Exception:
                    pass
                while True:
                    time.sleep(0.5)
                    try:
                        self.ser.open()
                        break
                    except (serial.SerialException, OSError):
                        continue
                print(f"{time.strftime('%H:%M:%S')} serial link back", flush=True)
                self.broadcast("link", {"up": True, "drops": self.drops})

    def _poll(self):
        i = 0
        pending = []
        last_push = time.time()
        while True:
            try:
                cmd, reply = self.commands.get_nowait()
                timeout = 15.0 if cmd.split()[0] in ("scan", "init", "wait", "trace") else 4.0
                out = self._exchange(cmd, lambda b: "\n" in b and re.sub(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[78]", "", b.split("\n", 1)[1]).rstrip().endswith(PROMPT.strip()), timeout)
                out = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[78]", "", out).replace(PROMPT.strip(), "").strip()
                t = time.time() - self.t0
                self.csv.write(f"{t:.4f},{t + self.t0:.4f},,,console: {cmd}\n")
                self.broadcast("console", {"cmd": cmd, "out": out})
                reply.put(out)
            except queue.Empty:
                pass
            if self.paused:
                time.sleep(0.05)
                continue
            name, node, ch = self.channels[i % len(self.channels)]
            cmd = sdor(node, ch)
            i += 1
            t_send = time.time()
            buf = self._exchange(cmd, lambda b: REPLY.search(b), 0.3)
            t = (t_send + time.time()) / 2 - self.t0
            m = REPLY.search(buf)
            value = int(m.group(1)) if m and m.group(1) else None
            err = "" if value is not None else (m.group(0) if m else "timeout")
            if value is None:
                self.errors += 1
            self.csv.write(f"{t:.4f},{t + self.t0:.4f},{name},{'' if value is None else value},{err}\n")
            sample = (round(t, 4), name, value)
            with self.lock:
                self.history.append(sample)
                while self.history and self.history[0][0] < t - HISTORY_S:
                    self.history.popleft()
            pending.append(sample)
            if time.time() - last_push > 0.1:
                self.broadcast("samples", {"s": pending, "errors": self.errors})
                pending, last_push = [], time.time()


def make_handler(scope):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def log_message(self, *a):
            pass

        def translate_path(self, path):
            name = path.split("?")[0].lstrip("/")
            if name in ("ax-tokens.css", "ax-fx.js", "uPlot.iife.min.js"):
                return os.path.join(PANEL, name)
            return super().translate_path(path)

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode()
            if self.path == "/cmd" and body.strip():
                reply = queue.Queue()
                scope.commands.put((body.strip(), reply))
                try:
                    out = reply.get(timeout=20).encode()
                except queue.Empty:
                    out = b"(no reply)"
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(out)
                return
            elif self.path == "/guard" and body.strip():
                global GUARD_SPEED
                GUARD_SPEED = int(body)
                print(f"guard now {GUARD_SPEED} counts/s", flush=True)
            elif self.path == "/pause":
                scope.paused = body == "1"
            self.send_response(204)
            self.end_headers()

        def do_GET(self):
            if self.path != "/events":
                return super().do_GET()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with scope.lock:
                hist = list(scope.history)
                scope.clients.append(q)
            info = {"csv": os.path.relpath(scope.csv_path, os.getcwd()), "port": scope.ser.port,
                    "paused": scope.paused, "errors": scope.errors,
                    "channels": [{"name": n, "node": d, "ch": c} for n, d, c in scope.channels]}
            try:
                self.wfile.write(f"event: hello\ndata: {json.dumps(info)}\n\n".encode())
                self.wfile.write(f"event: samples\ndata: {json.dumps({'s': hist, 'errors': scope.errors})}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        self.wfile.write(q.get(timeout=5))
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with scope.lock:
                    scope.clients.remove(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=None)
    ap.add_argument("--http", type=int, default=8790)
    ap.add_argument("--nodes", default="3,2,10", help="comma-separated MCP node ids to read (both channels each)")
    args = ap.parse_args()
    port = args.port
    if not port:
        found = sorted(glob.glob("/dev/cu.usbmodem*"))
        if len(found) != 1:
            sys.exit(f"pass --port: found {found or 'no'} /dev/cu.usbmodem* ports")
        port = found[0]
    os.makedirs(os.path.join(HERE, "runs"), exist_ok=True)
    csv_path = os.path.join(HERE, "runs", time.strftime("encoder_scope_%Y%m%d-%H%M%S.csv"))
    nodes = [int(n) for n in args.nodes.split(",")]
    channels = [c for n in nodes for c in ALL_CHANNELS if c[1] == n]
    scope = Scope(port, csv_path, channels)
    threading.Thread(target=scope.run, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(scope))
    print(f"encoder scope on http://localhost:{args.http}/  ({port}, logging {csv_path})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
