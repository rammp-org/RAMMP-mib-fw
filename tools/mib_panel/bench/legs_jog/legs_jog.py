"""Jog the new chair's main legs from a browser, with the guarded moves of newchair_leg.py.

    python3 tools/mib_panel/bench/legs_jog/legs_jog.py      # then open http://localhost:8793/

A browser cannot open TCP, so this serves the page and runs each move here: one move at a
time, through newchair_leg.move() (SDO polling, E-STOP on speed / wrong way / PWM / current /
stall / out of range). After a move that drew over 10 A the leg rests 3 s (the EHA motors are
rated 10 A). E-STOP is sent at once, whatever is running.
"""
import contextlib, io, json, math, os, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import newchair_leg as N  # noqa: E402  (connects to the MIB on import)

PANEL = os.path.abspath(os.path.join(HERE, "..", ".."))
COOL_AMPS, COOL_S = 10.0, 3.0

link_lock = threading.Lock()
_request = N.L.request
N.L.request = lambda *a, **k: _locked(_request, *a, **k)


def _locked(fn, *a, **k):
    with link_lock:
        return fn(*a, **k)


state = {"busy": None, "log": [], "cool_until": {0: 0.0, 1: 0.0}, "last": {}}


def log(line):
    state["log"] = (state["log"] + [time.strftime("%H:%M:%S ") + line])[-40:]


def run_move(leg, counts):
    name = ["MR", "ML"][leg]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            N.move(leg, counts)
    except Exception as e:  # noqa: BLE001
        N.estop()
        buf.write(f"  error {e!r}: E-STOP sent\n")
    out = buf.getvalue()
    for line in out.splitlines():
        log(f"{name} {counts:+d}: {line.strip()}")
    amps = 0.0
    for line in out.splitlines():
        if "max current" in line:
            try:
                amps = float(line.split("max current")[1].split("A")[0])
            except ValueError:
                pass
    if amps > COOL_AMPS:
        state["cool_until"][leg] = time.time() + COOL_S
    state["last"][leg] = out
    state["busy"] = None


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            return self._file(os.path.join(HERE, "index.html"), "text/html")
        if self.path in ("/ax-tokens.css", "/ax-fx.js"):
            return self._file(os.path.join(PANEL, self.path.lstrip("/")),
                              "text/css" if self.path.endswith(".css") else "text/javascript")
        if self.path == "/api/state":
            try:
                st = N.status() or {}
            except Exception as e:  # noqa: BLE001
                return self._json({"error": repr(e)}, 503)
            now = time.time()
            return self._json({
                "estop": st.get("estop", {}), "state": st.get("state"),
                "legs": st.get("legs", [])[:2], "imu": st.get("imu"),
                "busy": state["busy"], "log": state["log"][-14:],
                "cool": {k: max(0.0, v - now) for k, v in state["cool_until"].items()},
            })
        self.send_error(404)

    def _file(self, path, ctype):
        with open(path, "rb") as f:
            b = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/api/estop":
            N.estop()
            log("E-STOP sent from the page")
            return self._json({"ok": True})
        if self.path == "/api/clear":
            return self._json({"reply": N.req(0x08, bytes([2]))})
        if self.path == "/api/move":
            leg, counts = int(body["leg"]), int(body["counts"])
            if leg not in (0, 1) or abs(counts) > 400:
                return self._json({"error": "leg 0/1 and |counts| <= 400 only"}, 400)
            if state["busy"]:
                return self._json({"error": f"busy: {state['busy']}"}, 409)
            wait = state["cool_until"][leg] - time.time()
            if wait > 0:
                return self._json({"error": f"{['MR', 'ML'][leg]} resting {wait:.1f} s"}, 429)
            state["busy"] = f"{['MR', 'ML'][leg]} {counts:+d}"
            log(f"{state['busy']} started")
            threading.Thread(target=run_move, args=(leg, counts), daemon=True).start()
            return self._json({"ok": True})
        self.send_error(404)


if __name__ == "__main__":
    port = 8793
    print(f"legs jog on http://localhost:{port}/", flush=True)
    ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
