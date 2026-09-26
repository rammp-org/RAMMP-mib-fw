"""A small CANopen (CiA 301) client that speaks through the MIB's CAN bridge.

The MIB forwards raw frames in both directions (see mib_can_bridge.hpp), so
the whole protocol lives here: NMT commands, expedited SDO reads and writes of
1 to 4 byte objects, segmented SDO reads for strings, and abort-code decoding.
This is the subset espp/canopen implements on the device, ported so the tool
can address any object on any node without a reflash.

One SDO transaction runs at a time per node: the MCP266 has a single SDO
channel and the bridge has no flow control beyond that. A lost frame shows up
as a timeout, and the transaction is retried a couple of times before giving
up, because the rx topic is best-effort.
"""

import struct
import threading
import time

COB_NMT = 0x000
COB_SYNC = 0x080
COB_EMCY_BASE = 0x080
COB_SDO_TX_BASE = 0x580   # server -> client (response)
COB_SDO_RX_BASE = 0x600   # client -> server (request)
COB_HEARTBEAT_BASE = 0x700

NMT_COMMANDS = {
    "start": 0x01,
    "stop": 0x02,
    "preop": 0x80,
    "reset_node": 0x81,
    "reset_comm": 0x82,
}

NMT_STATES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}

ABORT_CODES = {
    0x05030000: "toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040001: "invalid or unknown command specifier",
    0x05040002: "invalid block size",
    0x05040003: "invalid sequence number",
    0x05040004: "CRC error",
    0x05040005: "out of memory",
    0x06010000: "unsupported access to object",
    0x06010001: "attempt to read a write-only object",
    0x06010002: "attempt to write a read-only object",
    0x06020000: "object does not exist in the object dictionary",
    0x06040041: "object cannot be mapped to the PDO",
    0x06040042: "number and length of mapped objects would exceed PDO length",
    0x06040043: "general parameter incompatibility",
    0x06040047: "general internal incompatibility in the device",
    0x06060000: "access failed due to a hardware error",
    0x06070010: "data type does not match, length of service parameter does not match",
    0x06070012: "data type does not match, length of service parameter too high",
    0x06070013: "data type does not match, length of service parameter too low",
    0x06090011: "subindex does not exist",
    0x06090030: "invalid value for parameter",
    0x06090031: "value of parameter written too high",
    0x06090032: "value of parameter written too low",
    0x06090036: "maximum value is less than minimum value",
    0x08000000: "general error",
    0x08000020: "data cannot be transferred or stored to the application",
    0x08000021: "data cannot be transferred or stored because of local control",
    0x08000022: "data cannot be transferred or stored because of the present device state",
    0x08000023: "object dictionary dynamic generation fails or no object dictionary present",
}


def abort_message(code: int) -> str:
    return ABORT_CODES.get(code, "unknown abort code")


class SdoError(Exception):
    """Any failed SDO transaction: abort from the server, timeout, or bad reply."""

    def __init__(self, text, code=None):
        super().__init__(text)
        self.code = code


class SdoAbort(SdoError):
    def __init__(self, index, sub, code):
        super().__init__(f"SDO abort on 0x{index:04X}:{sub} - {abort_message(code)} (0x{code:08X})", code)
        self.index, self.sub = index, sub


class CanopenClient:
    """NMT master and SDO client for one node, over a frame transport.

    `transport` must provide send_can_frame(id, data) and
    wait_can_frame(predicate, timeout) -> frame dict or None, where a frame
    dict has keys id, data (bytes), dlc, extended, rtr.
    """

    def __init__(self, transport, node_id: int = 10, timeout: float = 0.4, retries: int = 2):
        self.transport = transport
        self.node_id = int(node_id)
        self.timeout = timeout
        self.retries = retries
        self._lock = threading.Lock()

    # ----------------------------------------------------------------- NMT

    def nmt(self, command: str, node: int = None) -> None:
        """Send an NMT command to this node, or to every node with node=0."""
        code = NMT_COMMANDS[command]
        target = self.node_id if node is None else int(node)
        self.transport.send_can_frame(COB_NMT, bytes([code, target & 0x7F]))

    # ----------------------------------------------------------------- SDO

    def _sdo_request(self, payload: bytes, index: int, sub: int):
        """Send one SDO request frame and wait for the matching response frame."""
        req_id = COB_SDO_RX_BASE + self.node_id
        resp_id = COB_SDO_TX_BASE + self.node_id

        def matches(frame):
            if frame["id"] != resp_id or frame["dlc"] < 8:
                return False
            cmd = frame["data"][0]
            # Aborts and initiate responses carry the multiplexer; segment
            # responses (upload segment: cmd 0x00..0x1F) do not.
            if cmd & 0xE0 in (0x40, 0x60, 0x80):
                return frame["data"][1] | (frame["data"][2] << 8) == index and frame["data"][3] == sub
            return True

        self.transport.send_can_frame(req_id, payload)
        frame = self.transport.wait_can_frame(matches, self.timeout)
        if frame is None:
            raise SdoError(f"SDO timeout on 0x{index:04X}:{sub} (node {self.node_id})")
        data = frame["data"]
        if data[0] == 0x80:
            code = struct.unpack_from("<I", data, 4)[0]
            raise SdoAbort(index, sub, code)
        return data

    def _with_retries(self, fn):
        last = None
        for attempt in range(self.retries + 1):
            try:
                return fn()
            except SdoAbort:
                raise           # the server answered; retrying will not change its mind
            except SdoError as exc:
                last = exc
                time.sleep(0.02 * (attempt + 1))
        raise last

    def read(self, index: int, sub: int = 0) -> bytes:
        """Read an object. Expedited (1-4 bytes) or segmented (strings)."""
        with self._lock:
            return self._with_retries(lambda: self._read(index, sub))

    def _read(self, index, sub) -> bytes:
        mux = struct.pack("<HB", index, sub)
        data = self._sdo_request(b"\x40" + mux + b"\x00\x00\x00\x00", index, sub)
        cmd = data[0]
        if cmd & 0xE0 != 0x40:
            raise SdoError(f"unexpected SDO response 0x{cmd:02X} to read of 0x{index:04X}:{sub}")
        if cmd & 0x02:  # expedited
            if cmd & 0x01:
                size = 4 - ((cmd >> 2) & 0x03)
            else:
                size = 4
            return bytes(data[4:4 + size])
        # segmented upload
        total = struct.unpack_from("<I", data, 4)[0] if cmd & 0x01 else None
        out = bytearray()
        toggle = 0
        while True:
            seg = self._sdo_request(bytes([0x60 | (toggle << 4)]) + b"\x00" * 7, index, sub)
            if seg[0] & 0xE0 != 0x00:
                raise SdoError(f"unexpected SDO segment response 0x{seg[0]:02X}")
            if (seg[0] >> 4) & 1 != toggle:
                raise SdoError("SDO toggle bit not alternated")
            n = (seg[0] >> 1) & 0x07
            out += seg[1:8 - n]
            if seg[0] & 0x01:
                break
            toggle ^= 1
            if len(out) > 4096:
                raise SdoError("segmented SDO upload too large")
        if total is not None:
            out = out[:total]
        return bytes(out)

    def write(self, index: int, sub: int, data: bytes) -> None:
        """Write a 1-4 byte object (expedited download)."""
        if not 1 <= len(data) <= 4:
            raise SdoError("expedited SDO write takes 1 to 4 bytes")
        with self._lock:
            self._with_retries(lambda: self._write(index, sub, data))

    def _write(self, index, sub, data):
        n = 4 - len(data)
        cmd = 0x23 | (n << 2)
        payload = bytes([cmd]) + struct.pack("<HB", index, sub) + data + b"\x00" * n
        resp = self._sdo_request(payload, index, sub)
        if resp[0] != 0x60:
            raise SdoError(f"unexpected SDO response 0x{resp[0]:02X} to write of 0x{index:04X}:{sub}")

    # ------------------------------------------------------------ typed helpers

    def read_uint(self, index, sub=0, size=None) -> int:
        raw = self.read(index, sub)
        if size and len(raw) != size:
            raw = raw[:size] if len(raw) > size else raw + b"\x00" * (size - len(raw))
        return int.from_bytes(raw, "little", signed=False)

    def read_int(self, index, sub=0, size=None) -> int:
        raw = self.read(index, sub)
        if size and len(raw) != size:
            raw = raw[:size] if len(raw) > size else raw + b"\x00" * (size - len(raw))
        return int.from_bytes(raw, "little", signed=True)

    def read_string(self, index, sub=0) -> str:
        return self.read(index, sub).split(b"\x00")[0].decode("ascii", "replace").strip()

    def write_uint(self, index, sub, value: int, size: int) -> None:
        self.write(index, sub, int(value).to_bytes(size, "little", signed=False))

    def write_int(self, index, sub, value: int, size: int) -> None:
        self.write(index, sub, int(value).to_bytes(size, "little", signed=True))


# ---------------------------------------------------------------- decoding

def describe_frame(frame_id: int, data: bytes, dlc: int) -> str:
    """One-line human reading of a CANopen frame, for the bus monitor."""
    fc = frame_id & 0x780
    node = frame_id & 0x7F
    if frame_id == COB_NMT:
        if len(data) >= 2:
            names = {v: k for k, v in NMT_COMMANDS.items()}
            return f"NMT {names.get(data[0], hex(data[0]))} node {data[1] or 'all'}"
        return "NMT"
    if frame_id == COB_SYNC:
        return "SYNC"
    if fc == COB_EMCY_BASE and node:
        if len(data) >= 3:
            code = data[0] | (data[1] << 8)
            return f"EMCY node {node} code 0x{code:04X} reg 0x{data[2]:02X}"
        return f"EMCY node {node}"
    if fc == COB_HEARTBEAT_BASE:
        state = NMT_STATES.get(data[0] & 0x7F, hex(data[0])) if data else "?"
        return f"heartbeat node {node}: {state}"
    if fc in (COB_SDO_RX_BASE, COB_SDO_TX_BASE) and len(data) >= 4:
        cmd = data[0]
        mux = f"0x{data[1] | (data[2] << 8):04X}:{data[3]}"
        who = "req" if fc == COB_SDO_RX_BASE else "resp"
        payload = data[4:8].hex()
        if cmd == 0x80:
            code = int.from_bytes(data[4:8], "little")
            return f"SDO abort {mux}: {abort_message(code)} (0x{code:08X})"
        if fc == COB_SDO_RX_BASE:
            if cmd & 0xE0 == 0x40:
                return f"SDO read {mux}"
            if cmd & 0xE0 == 0x20:
                size = 4 - ((cmd >> 2) & 3) if cmd & 1 else 4
                return f"SDO write {mux} = 0x{int.from_bytes(data[4:4 + size], 'little'):X} ({size} B)"
            if cmd & 0xE0 == 0x60:
                return f"SDO segment request (toggle {(cmd >> 4) & 1})"
        else:
            if cmd & 0xE0 == 0x40:
                if cmd & 0x02:
                    size = 4 - ((cmd >> 2) & 3) if cmd & 1 else 4
                    return f"SDO data {mux} = 0x{int.from_bytes(data[4:4 + size], 'little'):X} ({size} B)"
                return f"SDO segmented upload start {mux}"
            if cmd == 0x60:
                return f"SDO write ok {mux}"
            if cmd & 0xE0 == 0x00:
                n = (cmd >> 1) & 7
                return f"SDO segment {'last ' if cmd & 1 else ''}{bytes(data[1:8 - n])!r}"
        return f"SDO {who} cmd 0x{cmd:02X} {mux} {payload}"
    if 0x180 <= fc <= 0x480 and node:
        return f"PDO 0x{fc:03X} node {node}"
    return ""
