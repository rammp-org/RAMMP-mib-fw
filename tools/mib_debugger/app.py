"""Web front end for the RAMMP MIB debugger.

Serves a single page and a small JSON API from the standard library, so the
only third-party dependency in the whole tool is CycloneDDS. The page starts
disconnected; the Connect button in the browser creates the DDS participant,
and Disconnect tears it down again.

    python app.py                    # open the page, connect from the browser
    python app.py --local --connect  # connect to fake_mib.py straight away
    python app.py --interface en5 --connect
"""

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import mib_link

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

# The MIB hands out leases on this subnet, so the interface that holds such an
# address is the one facing the board.
MIB_SUBNET_PREFIX = "192.168.4."


def list_interfaces():
    """Return [{name, address, mib}] for every interface with an IPv4 address."""
    try:
        out = subprocess.run(["ifconfig"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    found = []
    current = None
    for line in out.splitlines():
        head = re.match(r"^([a-z0-9]+):", line)
        if head:
            current = head.group(1)
            continue
        addr = re.search(r"\binet (\d+\.\d+\.\d+\.\d+)", line)
        if current and addr:
            found.append({
                "name": current,
                "address": addr.group(1),
                "mib": addr.group(1).startswith(MIB_SUBNET_PREFIX),
            })
    # The board's subnet first, loopback last.
    found.sort(key=lambda i: (not i["mib"], i["name"] == "lo0", i["name"]))
    return found


def detect_mib_interface():
    """Return the name of the interface on the MIB's subnet, or None."""
    for entry in list_interfaces():
        if entry["mib"]:
            return entry["name"]
    return None


class Connection:
    """Owns the optional MibLink and the settings the page chose for it."""

    def __init__(self, mode="board", interface=None, domain=0):
        self.lock = threading.Lock()
        self.link = None
        self.mode = mode            # "board" or "local"
        self.interface = interface  # None means auto-detect
        self.domain = domain
        self.error = None

    def connect(self, mode=None, interface=None):
        with self.lock:
            if self.link is not None:
                return {"ok": True, "already": True}
            if mode:
                self.mode = mode
            self.interface = interface or None
            self.error = None

            chosen = None
            if self.mode == "board":
                chosen = self.interface or detect_mib_interface()
            link = mib_link.MibLink(domain_id=self.domain, interface=chosen,
                                    local=(self.mode == "local"))
            try:
                link.start()
            except Exception as exc:
                self.error = str(exc)
                raise
            if self.mode == "board" and not chosen:
                link.log(
                    f"No interface holds a {MIB_SUBNET_PREFIX}x address; using the "
                    "CycloneDDS default. Plug the board in or pick an interface.",
                    "warn")
            self.link = link
            return {"ok": True, "interface": chosen}

    def disconnect(self):
        with self.lock:
            link, self.link = self.link, None
        if link is None:
            return {"ok": True, "already": True}
        link.stop_everything()
        link.stop()
        return {"ok": True}

    def snapshot(self):
        link = self.link
        snap = link.snapshot() if link else mib_link.empty_snapshot()
        snap["link"] = {
            "up": link is not None,
            "mode": self.mode,
            "interface": self.interface,
            "detected": detect_mib_interface(),
            "error": self.error,
        }
        return snap


conn: Connection = None  # set in main()


class Handler(BaseHTTPRequestHandler):
    # Quieten the default per-request logging; the page polls several times a
    # second and would otherwise bury anything useful.
    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, content_type="application/json"):
        payload = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        pages = {"/": "index.html", "/index.html": "index.html",
                 "/params": "params.html", "/params.html": "params.html"}
        if self.path.split("?")[0] in pages:
            try:
                with open(os.path.join(STATIC_DIR, pages[self.path.split("?")[0]]), "rb") as handle:
                    self._send(200, handle.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, b"page is missing", "text/plain")
            return

        if self.path.startswith("/api/state"):
            if conn.link:
                conn.link.note_poll()
            self._send(200, json.dumps(conn.snapshot()))
            return

        if self.path.startswith("/api/param_table"):
            import mib_messages
            self._send(200, json.dumps([
                {"id": r[0], "name": r[1], "label": r[2], "unit": r[3],
                 "min": r[4], "max": r[5], "default": r[6]} for r in mib_messages.PARAM_TABLE]))
            return

        if self.path.startswith("/api/interfaces"):
            self._send(200, json.dumps(list_interfaces()))
            return

        self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        try:
            body = self._json_body()
        except json.JSONDecodeError:
            self._send(400, json.dumps({"error": "bad JSON"}))
            return

        try:
            if self.path.startswith("/api/connect"):
                result = conn.connect(body.get("mode"), body.get("interface"))
            elif self.path.startswith("/api/disconnect"):
                result = conn.disconnect()
            elif self.path.startswith("/api/command"):
                if conn.link is None:
                    self._send(409, json.dumps({"error": "not connected"}))
                    return
                conn.link.note_poll()  # a command proves the client is present
                result = dispatch(conn.link, body)
            else:
                self._send(404, json.dumps({"error": "not found"}))
                return
        except Exception as exc:
            if conn.link:
                conn.link.log(f"Request failed: {exc}", "bad")
            self._send(500, json.dumps({"error": str(exc)}))
            return

        self._send(200, json.dumps(result or {"ok": True}))


def dispatch(link, body: dict):
    """Apply one command from the front end."""
    action = body.get("action")

    if action == "joystick":
        link.set_joystick(
            body.get("x", 0.0), body.get("y", 0.0),
            body.get("twist", 0.0), body.get("button", False),
        )
        return {"ok": True}

    if action == "joystick_live":
        link.set_joystick_live(bool(body.get("live")))
        return {"ok": True}

    if action == "drive":
        link.send_drive_command(bool(body.get("enable")), body.get("profile"))
        return {"ok": True}

    if action == "seat":
        link.send_seat_command(int(body["axis"]), float(body["target"]))
        return {"ok": True}

    if action == "param_set":
        return link.set_param(int(body["id"]), float(body["value"]))

    if action == "motor_sim":
        link.set_motor_sim(bool(body.get("enabled")))
        return {"ok": True}

    if action == "stop":
        link.stop_everything()
        return {"ok": True}

    raise ValueError(f"unknown action {action!r}")


def main():
    global conn

    parser = argparse.ArgumentParser(description="Debug GUI for the RAMMP MIB.")
    parser.add_argument("--interface", default=None,
                        help="network interface facing the board, e.g. en5")
    parser.add_argument("--local", action="store_true",
                        help="preselect simulator mode (loopback, for fake_mib.py)")
    parser.add_argument("--connect", action="store_true",
                        help="connect at startup instead of waiting for the button")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id")
    parser.add_argument("--host", default="127.0.0.1", help="web server address")
    parser.add_argument("--port", type=int, default=8712, help="web server port")
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a browser window")
    args = parser.parse_args()

    conn = Connection(mode="local" if args.local else "board",
                      interface=args.interface, domain=args.domain)

    if args.connect:
        try:
            result = conn.connect()
            print("Connected" + (f" via {result['interface']}" if result.get("interface") else ""))
        except Exception as exc:
            print(f"Could not start the DDS participant: {exc}", file=sys.stderr)
            return 1

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}/"
    print(f"MIB debugger on {url}")
    print(f"  Mode          : {'simulator (loopback)' if conn.mode == 'local' else 'board'}")
    print(f"  Board expected at {mib_link.DEFAULT_MIB_ADDRESS}")
    print("  Use the Connect button in the page, or pass --connect")
    print("  Ctrl-C to stop")

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        conn.disconnect()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
