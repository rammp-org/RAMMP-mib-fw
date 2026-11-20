"""Tiny service-link client over TCP :5555 (the MIB panel protocol, module 0x40)."""
import socket, struct, zlib, time, json, sys
MIB = 0x40
def frame(mod, typ, payload=b""):
    head = struct.pack("<HBBBI", 0x4F54, 0x10, mod, typ, len(payload)) + payload
    return head + struct.pack("<I", zlib.crc32(head) & 0xFFFFFFFF)
class Link:
    def __init__(self, host="192.168.4.1", port=5555, bind=None):
        # Bind to this Mac's address on the MIB's subnet (it changes with the DHCP lease), so
        # Wi-Fi or a VPN cannot take the traffic.
        if bind is None:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); probe.connect((host, 9))
            bind = probe.getsockname()[0]; probe.close()
        self.s = socket.socket(); self.s.bind((bind, 0)); self.s.settimeout(3); self.s.connect((host, port)); self.buf = b""
    def frames(self, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                d = self.s.recv(65536)
                if not d: break
                self.buf += d
            except socket.timeout:
                pass
            while True:
                i = self.buf.find(b"\x54\x4f")
                if i < 0 or len(self.buf) < i + 9: break
                magic, ver, mod, typ, n = struct.unpack("<HBBBI", self.buf[i:i+9])
                if len(self.buf) < i + 9 + n + 4: break
                payload = self.buf[i+9:i+9+n]; self.buf = self.buf[i+9+n+4:]
                yield mod, typ, payload
    def request(self, typ, payload=b"", want=None, timeout=3.0, mod=MIB):
        self.s.sendall(frame(mod, typ, payload))
        for m, t, p in self.frames(timeout):
            if m == mod and (want is None or t in want or t in (0x8F,)):
                return t, p
        return None, None
if __name__ == "__main__":
    L = Link()
    t, p = L.request(0x09, want=(0x89,))
    print(t and hex(t)); print(p.decode(errors="replace")[:3000] if p else "no status reply")
