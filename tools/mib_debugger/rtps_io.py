"""RTPS transport for the debugger, on espp's own stack.

The MIB and the PACE RACER boards speak DDS through espp's rtps component.
This module gives the tool the same stack, from the `espp` wheel on PyPI, so
both ends run identical discovery code and there is no second DDS
implementation to interoperate with. Three things live here:

- a classic CDR (XCDR1, little-endian) codec driven by the field types a
  message class declares, byte-identical to what espp::cdr and CycloneDDS
  produce for these plain structs;
- the `message` decorator that turns a dataclass into a wire type with a
  DDS type name;
- `Participant`, a thin wrapper over `espp.RtpsParticipant` whose writers
  take message objects and whose readers hand them back, drained with
  `take()` from a thread-safe queue filled by espp's receive callback.

Samples cross the espp API as CDR-encapsulated bytes: a 4-byte header
(0x00 0x01 0x00 0x00 for little-endian CDR) followed by the body, with
primitives aligned to their size relative to the start of the body.
"""

import struct
import threading
from collections import deque
from dataclasses import dataclass, fields, is_dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

CDR_LE_HEADER = b"\x00\x01\x00\x00"


# ------------------------------------------------------------------ types

class Prim:
    """A CDR primitive: struct format character, size, alignment."""

    def __init__(self, name: str, fmt: str):
        self.name = name
        self.fmt = "<" + fmt
        self.size = struct.calcsize(fmt)

    def __repr__(self):
        return self.name


class Array:
    """A fixed-length array of a primitive."""

    def __init__(self, elem: Prim, count: int):
        self.elem = elem
        self.count = count

    def __repr__(self):
        return f"{self.elem.name}[{self.count}]"


class Types:
    u8 = Prim("u8", "B")
    i8 = Prim("i8", "b")
    u16 = Prim("u16", "H")
    i16 = Prim("i16", "h")
    u32 = Prim("u32", "I")
    i32 = Prim("i32", "i")
    u64 = Prim("u64", "Q")
    i64 = Prim("i64", "q")
    f32 = Prim("f32", "f")
    f64 = Prim("f64", "d")
    string = "string"

    @staticmethod
    def array(elem: Prim, count: int) -> Array:
        return Array(elem, count)


t = Types


def message(typename: str):
    """Class decorator: a dataclass whose field annotations are `t.*` types,
    `t.string`, or another message class becomes a wire type."""
    def wrap(cls):
        cls = dataclass(cls)
        cls.TYPENAME = typename
        cls.WIRE_FIELDS = [(f.name, f.type) for f in fields(cls)]
        return cls
    return wrap


# ------------------------------------------------------------------ codec

class _Writer:
    def __init__(self):
        self.buf = bytearray()

    def align(self, n: int) -> None:
        pad = (-len(self.buf)) % n
        self.buf += b"\x00" * pad

    def prim(self, p: Prim, value) -> None:
        self.align(p.size)
        self.buf += struct.pack(p.fmt, value)

    def string(self, s: str) -> None:
        data = s.encode("utf-8") + b"\x00"
        self.prim(t.u32, len(data))
        self.buf += data


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def align(self, n: int) -> None:
        self.pos += (-self.pos) % n

    def prim(self, p: Prim):
        self.align(p.size)
        if self.pos + p.size > len(self.data):
            raise ValueError("sample truncated")
        value = struct.unpack_from(p.fmt, self.data, self.pos)[0]
        self.pos += p.size
        return value

    def string(self) -> str:
        n = self.prim(t.u32)
        if self.pos + n > len(self.data):
            raise ValueError("sample truncated in string")
        raw = self.data[self.pos:self.pos + n]
        self.pos += n
        return raw.split(b"\x00")[0].decode("utf-8", "replace")


def _encode_into(w: _Writer, obj) -> None:
    for name, typ in obj.WIRE_FIELDS:
        value = getattr(obj, name)
        if isinstance(typ, Prim):
            w.prim(typ, value)
        elif isinstance(typ, Array):
            values = list(value)
            if len(values) != typ.count:
                raise ValueError(f"{name}: expected {typ.count} elements, got {len(values)}")
            for v in values:
                w.prim(typ.elem, v)
        elif typ is t.string:
            w.string(value)
        elif hasattr(typ, "WIRE_FIELDS"):
            _encode_into(w, value)
        else:
            raise TypeError(f"{name}: unsupported wire type {typ!r}")


def _decode_from(r: _Reader, cls):
    kwargs = {}
    for name, typ in cls.WIRE_FIELDS:
        if isinstance(typ, Prim):
            kwargs[name] = r.prim(typ)
        elif isinstance(typ, Array):
            kwargs[name] = [r.prim(typ.elem) for _ in range(typ.count)]
        elif typ is t.string:
            kwargs[name] = r.string()
        elif hasattr(typ, "WIRE_FIELDS"):
            kwargs[name] = _decode_from(r, typ)
        else:
            raise TypeError(f"{name}: unsupported wire type {typ!r}")
    return cls(**kwargs)


def encode(obj) -> bytes:
    """CDR-encapsulated bytes for a message object."""
    w = _Writer()
    _encode_into(w, obj)
    w.align(4)  # XCDR1 serialized data is padded to a multiple of 4
    return CDR_LE_HEADER + bytes(w.buf)


def decode(cls, data: bytes):
    """A message object from CDR-encapsulated bytes, or None if it does not
    parse (wrong representation or truncated)."""
    data = bytes(data)
    if len(data) < 4:
        return None
    if data[:2] not in (b"\x00\x01", b"\x00\x00"):
        return None
    if data[:2] == b"\x00\x00":
        # big-endian CDR: not produced by anything on the chair
        return None
    try:
        return _decode_from(_Reader(data[4:]), cls)
    except (ValueError, struct.error):
        return None


# ------------------------------------------------------------ participant

class Reader:
    """Samples of one topic, decoded, queued by espp's callback thread."""

    def __init__(self, topic: str, cls, depth: int = 256):
        self.topic = topic
        self.cls = cls
        self._queue = deque(maxlen=depth)
        self._lock = threading.Lock()
        self.received = 0
        self.undecodable = 0

    def _on_sample(self, data) -> None:
        sample = decode(self.cls, data)
        with self._lock:
            if sample is None:
                self.undecodable += 1
                return
            self.received += 1
            self._queue.append(sample)

    def take(self, N: int = 10) -> list:
        """Up to N queued samples, oldest first."""
        out = []
        with self._lock:
            while self._queue and len(out) < N:
                out.append(self._queue.popleft())
        return out


class Writer:
    def __init__(self, participant, topic: str, cls):
        self._participant = participant
        self.topic = topic
        self.cls = cls
        self.sent = 0

    def write(self, obj) -> bool:
        if not isinstance(obj, self.cls):
            raise TypeError(f"{self.topic}: expected {self.cls.__name__}")
        ok = self._participant._rtps.publish(self.topic, encode(obj))
        if ok:
            self.sent += 1
        return ok


class Participant:
    """One espp RTPS participant with typed writers and readers.

    `interface_address` is the IPv4 address of the interface to use; empty
    picks the first non-loopback interface, "127.0.0.1" is for the simulator.
    """

    def __init__(self, interface_address: str = "", log_level: str = "error",
                 on_matched: Optional[Callable[[str], None]] = None):
        import espp  # imported here so a missing wheel reports at connect time
        self._espp = espp
        cfg = espp.RtpsParticipant.Config()
        cfg.interface_address = interface_address or ""
        cfg.log_level = getattr(espp.Logger.Verbosity, log_level)
        self.matched_writers = 0   # our writers that gained a remote reader
        self.matched_readers = 0   # our readers that gained a remote writer

        # espp names the callbacks after the REMOTE side: on_publisher_matched
        # fires when a remote publisher pairs with one of our readers, and
        # on_subscriber_matched when a remote subscriber pairs with our writer.
        def remote_publisher_matched(*_):
            self.matched_readers += 1
            if on_matched:
                on_matched("reader")

        def remote_subscriber_matched(*_):
            self.matched_writers += 1
            if on_matched:
                on_matched("writer")
        cfg.on_publisher_matched = remote_publisher_matched
        cfg.on_subscriber_matched = remote_subscriber_matched
        self._cfg = cfg
        self._rtps = espp.RtpsParticipant(cfg)
        self._readers: List[Reader] = []
        self._writers: List[Writer] = []
        self.started = False

    def start(self) -> None:
        if not self._rtps.start():
            raise RuntimeError("espp RTPS participant failed to start (interface "
                               f"{self._cfg.interface_address or 'auto'})")
        self.started = True

    def writer(self, topic: str, cls, reliable: bool = False) -> Writer:
        if not self._rtps.add_writer(topic, cls.TYPENAME, reliable):
            raise RuntimeError(f"could not add writer for {topic} (endpoint pool full?)")
        w = Writer(self, topic, cls)
        self._writers.append(w)
        return w

    def reader(self, topic: str, cls, reliable: bool = False, depth: int = 256) -> Reader:
        r = Reader(topic, cls, depth)
        if not self._rtps.add_reader(topic, cls.TYPENAME, reliable, r._on_sample):
            raise RuntimeError(f"could not add reader for {topic} (endpoint pool full?)")
        self._readers.append(r)
        return r

    def stop(self) -> None:
        if self.started:
            self._rtps.stop()
            self.started = False
        self._readers = []
        self._writers = []
