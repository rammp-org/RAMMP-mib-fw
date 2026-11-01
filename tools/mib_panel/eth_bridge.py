#!/usr/bin/env python3
"""Serve the MIB control panel and bridge its WebSocket to the MIB's ethernet
service stream (tcp :5555). A browser cannot open a TCP socket; this relays one.

    tools/mib_panel/eth_bridge.py               # http://localhost:8765/
    tools/mib_panel/eth_bridge.py --port 9000

Endpoints:
    GET  /, /<file>            the panel, from this directory
    GET  /builds               JSON list of firmware builds: this repo's build*/, and --racer's
    GET  /builds/<source>/<dir>/<file>   a file that build's flasher_args.json names
    GET  /discover             PACE RACER boards answering "PACE?" on udp :5556
    GET  /bridge.json          {"bridge": true, "default_target": "192.168.4.1"}: tells
                               the page it can use this origin's /ws
    WS   /ws?target=<ip>[:port]   raw bytes <-> tcp <ip>:<port, 5555>, one socket per page

The page on GitHub Pages can use this bridge too (it connects to
ws://localhost:<port>/ws). Standard library only: a minimal RFC 6455 server
(binary frames, masked client payloads, ping/pong, close), taken from the PACE
RACER's atos-drive/tools/eth_bridge.py, with its interface binding from
eth_link.py so a multi-homed laptop (Wi-Fi, VPN) still reaches the MIB.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))  # tools/mib_panel -> the firmware repo, where idf.py puts build*/
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
SERVICE_PORT = 5555
PAGES_ORIGIN = "https://rammp-org.github.io"
DEFAULT_TARGET = "192.168.4.1"  # the MIB is the chair network's DHCP server (MIBconfig.hpp)
IP_BOUND_IF = 25  # macOS setsockopt(IPPROTO_IP, ...) to pin a socket to one interface
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8", ".md": "text/plain; charset=utf-8", ".png": "image/png"}


# ---------------------------------------------------------------- interfaces (from eth_link.py)
_SKIP_IFACES = ("utun", "lo", "bridge", "awdl", "llw", "docker", "veth", "virbr", "tailscale",
                "Loopback", "vEthernet", "Tailscale")


def _prefix_to_mask(bits):
    return socket.inet_ntoa(((0xFFFFFFFF << (32 - bits)) & 0xFFFFFFFF).to_bytes(4, "big"))


def _ifaces():
    """[(name, ipv4, netmask)] of physical interfaces; empty if the OS tool is missing."""
    try:
        found = []
        if sys.platform == "darwin":
            name = None
            for ln in subprocess.run(["ifconfig"], capture_output=True, text=True).stdout.splitlines():
                if ln and not ln[0].isspace():
                    name = ln.split(":")[0]
                elif name and "inet " in ln:
                    p = ln.split()
                    mask = socket.inet_ntoa(int(p[p.index("netmask") + 1], 16).to_bytes(4, "big")) if "netmask" in p else "255.255.255.255"
                    found.append((name, p[1], mask))
        elif sys.platform.startswith("win"):
            ps = ("Get-NetIPAddress -AddressFamily IPv4 | "
                  "Select-Object InterfaceAlias,IPAddress,PrefixLength | ConvertTo-Json -Compress")
            out = subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True, text=True).stdout.strip()
            rows = json.loads(out) if out else []
            rows = [rows] if isinstance(rows, dict) else rows
            found = [(r["InterfaceAlias"], r["IPAddress"], _prefix_to_mask(int(r["PrefixLength"]))) for r in rows]
        else:
            for ln in subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True).stdout.splitlines():
                p = ln.split()
                if "inet" in p:
                    ip, bits = p[p.index("inet") + 1].split("/")
                    found.append((p[1], ip, _prefix_to_mask(int(bits))))
    except (OSError, ValueError, KeyError, IndexError):
        return []
    return [(n, ip, m) for n, ip, m in found if not n.startswith(_SKIP_IFACES) and not ip.startswith("127.")]


def _bind_for(sock, target_ip):
    """Bind the socket to the interface on the target's subnet, so a VPN or Wi-Fi route
    that also claims the range cannot swallow the traffic. Returns the interface name."""
    t = int.from_bytes(socket.inet_aton(target_ip), "big")
    for name, ip, mask in _ifaces():
        m = int.from_bytes(socket.inet_aton(mask), "big")
        if (t & m) == (int.from_bytes(socket.inet_aton(ip), "big") & m):
            sock.bind((ip, 0))
            try:
                if sys.platform == "darwin":
                    sock.setsockopt(socket.IPPROTO_IP, IP_BOUND_IF, socket.if_nametoindex(name))
                elif sys.platform.startswith("linux") and hasattr(socket, "SO_BINDTODEVICE"):
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, name.encode())
            except OSError:
                pass
            return name
    return None


# ---------------------------------------------------------------- builds
# Firmware builds the page can flash: every build*/ dir with a flasher_args.json under each
# source (an ESP-IDF project dir). The MIB repo is always a source; --racer adds the PACE
# RACER's atos-drive app. flasher_args.json names the app image the page sends over the air.
SOURCES = {"mib": REPO}


def _cstr(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace")


def builds():
    """[{id, source, dir, project, version, built, chip, app, files}], newest first."""
    out = []
    for source, root in SOURCES.items():
        try:
            dirs = sorted(os.listdir(root))
        except OSError:
            continue
        for d in dirs:
            bdir = os.path.join(root, d)
            try:
                with open(os.path.join(bdir, "flasher_args.json")) as fh:
                    fa = json.load(fh)
                app = os.path.join(bdir, fa["app"]["file"])
                with open(app, "rb") as fh:
                    head = fh.read(256)
            except (OSError, ValueError, KeyError):
                continue
            # esp_app_desc follows the 24-byte image header and an 8-byte segment header.
            if not d.startswith("build") or len(head) < 144 or head[0] != 0xE9 or \
                    struct.unpack_from("<I", head, 32)[0] != 0xABCD5432:
                continue
            files = sorted(({"offset": int(o, 16), "file": f} for o, f in fa.get("flash_files", {}).items()),
                           key=lambda x: x["offset"])
            out.append({"id": f"{source}/{d}", "source": source, "dir": d,
                        "project": _cstr(head[80:112]), "version": _cstr(head[48:80]),
                        "built": f"{_cstr(head[128:144])} {_cstr(head[112:128])}",
                        "chip": fa.get("extra_esptool_args", {}).get("chip", ""),
                        "app": {"offset": int(fa["app"]["offset"], 16), "file": fa["app"]["file"],
                                "size": os.path.getsize(app)},
                        "files": files, "mtime": os.path.getmtime(app)})
    return sorted(out, key=lambda b: -b["mtime"])


def build_file(path):
    """/builds/<source>/<dir>/<file in flasher_args> -> bytes, or None. Only listed files."""
    for b in builds():
        if path.startswith(b["id"] + "/"):
            rel = path[len(b["id"]) + 1:]
            if rel in {f["file"] for f in b["files"]} | {b["app"]["file"]}:
                with open(os.path.join(SOURCES[b["source"]], b["dir"], rel), "rb") as fh:
                    return fh.read()
    return None


# ---------------------------------------------------------------- PACE RACER discovery
DISCOVERY_PORT = 5556


def discover(timeout=1.5):
    """Broadcast "PACE?" from every interface (eth_link.discover); [board info dict + 'ip']."""
    socks = []
    for name, ip, mask in (_ifaces() or [(None, "0.0.0.0", "0.0.0.0")]):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            s.bind((ip, 0))
        except OSError:
            s.close()
            continue
        s.setblocking(False)
        socks.append(s)
        dsts = {"255.255.255.255"}
        if name:
            ipn = int.from_bytes(socket.inet_aton(ip), "big")
            mn = int.from_bytes(socket.inet_aton(mask), "big")
            dsts.add(socket.inet_ntoa(((ipn & mn) | (~mn & 0xFFFFFFFF)).to_bytes(4, "big")))
        for dst in dsts:
            try:
                s.sendto(b"PACE?", (dst, DISCOVERY_PORT))
            except OSError:
                pass
    boards, t0 = {}, time.time()
    while time.time() - t0 < timeout:
        for s in socks:
            try:
                data, (ip, _) = s.recvfrom(2048)
                info = json.loads(data.decode())
            except (OSError, ValueError):
                continue
            info["ip"] = ip
            boards[ip] = info
        time.sleep(0.05)
    for s in socks:
        s.close()
    return sorted(boards.values(), key=lambda b: b.get("axis", 99))


# ---------------------------------------------------------------- websocket
def ws_frame(opcode, payload=b""):
    hdr = bytes([0x80 | opcode])
    n = len(payload)
    if n < 126:
        hdr += bytes([n])
    elif n < 65536:
        hdr += bytes([126]) + struct.pack(">H", n)
    else:
        hdr += bytes([127]) + struct.pack(">Q", n)
    return hdr + payload


def ws_read(rfile):
    """-> (opcode, payload) or None on EOF. Client frames are always masked."""
    h = rfile.read(2)
    if len(h) < 2:
        return None
    opcode, masked, n = h[0] & 0x0F, h[1] & 0x80, h[1] & 0x7F
    if n == 126:
        n = struct.unpack(">H", rfile.read(2))[0]
    elif n == 127:
        n = struct.unpack(">Q", rfile.read(8))[0]
    mask = rfile.read(4) if masked else b""
    data = rfile.read(n)
    if masked:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return opcode, data


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    def reply(self, code, ctype, body):
        self.send_response(code)
        # The hosted copy of the panel may read the build list and discovery; no other site.
        if self.headers.get("Origin") == PAGES_ORIGIN:
            self.send_header("Access-Control-Allow-Origin", PAGES_ORIGIN)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/ws":
            return self.websocket(parse_qs(u.query).get("target", [DEFAULT_TARGET])[0])
        if u.path == "/builds":
            return self.reply(200, "application/json", json.dumps(builds()).encode())
        if u.path.startswith("/builds/"):
            body = build_file(u.path[len("/builds/"):])  # only files a listed build names
            return self.reply(200, "application/octet-stream", body) if body is not None else self.send_error(404, "no such build file")
        if u.path == "/discover":
            return self.reply(200, "application/json", json.dumps(discover()).encode())
        if u.path == "/bridge.json":
            return self.reply(200, "application/json", json.dumps({"bridge": True, "default_target": DEFAULT_TARGET}).encode())
        name = "index.html" if u.path == "/" else u.path.lstrip("/")
        # Only plain files in this directory: no path tricks.
        if "/" in name or name.startswith(".") or os.path.splitext(name)[1] not in TYPES or not os.path.isfile(os.path.join(HERE, name)):
            return self.send_error(404)
        with open(os.path.join(HERE, name), "rb") as fh:
            return self.reply(200, TYPES[os.path.splitext(name)[1]], fh.read())

    def websocket(self, target):
        key = self.headers.get("Sec-WebSocket-Key")
        host, _, port = target.partition(":")
        if self.headers.get("Upgrade", "").lower() != "websocket" or not key or not host:
            return self.send_error(400, "websocket upgrade with ?target=<ip>[:port] expected")
        port = int(port) if port else SERVICE_PORT
        tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        iface = _bind_for(tcp, host)
        tcp.settimeout(3.0)
        try:
            tcp.connect((host, port))
        except OSError as e:
            tcp.close()
            sys.stderr.write(f"connect {host}:{port} failed: {e}\n")
            return self.send_error(502, f"connect {host}:{port}: {e}")
        tcp.settimeout(None)
        tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sys.stderr.write(f"bridging to {host}:{port}{f' on {iface}' if iface else ''}\n")
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        wlock = threading.Lock()

        def board_to_ws():
            try:
                while True:
                    d = tcp.recv(65536)
                    if not d:
                        break
                    with wlock:
                        self.wfile.write(ws_frame(0x2, d))
                        self.wfile.flush()
            except OSError:
                pass
            finally:
                with wlock:
                    try:
                        self.wfile.write(ws_frame(0x8, struct.pack(">H", 1001)))
                        self.wfile.flush()
                    except OSError:
                        pass

        threading.Thread(target=board_to_ws, daemon=True).start()
        try:
            while True:
                fr = ws_read(self.rfile)
                if fr is None:
                    break
                op, data = fr
                if op in (0x1, 0x2):
                    tcp.sendall(data)
                elif op == 0x9:
                    with wlock:
                        self.wfile.write(ws_frame(0xA, data))
                        self.wfile.flush()
                elif op == 0x8:
                    break
        except OSError:
            pass
        finally:
            try:
                tcp.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            tcp.close()


def main():
    global DEFAULT_TARGET
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765, help="local HTTP / WebSocket port (default 8765)")
    ap.add_argument("--target", default=DEFAULT_TARGET, help=f"the MIB's address (default {DEFAULT_TARGET})")
    ap.add_argument("--racer", metavar="DIR", help="pace-racer-fw's atos-drive app dir, to flash its build*/ to the RACERs")
    a = ap.parse_args()
    DEFAULT_TARGET = a.target
    if a.racer:
        SOURCES["racer"] = os.path.abspath(os.path.expanduser(a.racer))
    for b in builds():
        print(f"  build {b['id']}: {b['project']} {b['version']} ({b['chip']}, {b['built']})", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    with socket.socket() as probe:
        via = _bind_for(probe, DEFAULT_TARGET.partition(":")[0])
    print(f"MIB control panel: http://localhost:{a.port}/   (MIB at {DEFAULT_TARGET}"
          f"{f', via {via}' if via else ', no interface on its subnet yet: plug into the chair'}; Ctrl-C to stop)",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
