"""The logger's wire format: line kinds, and an incremental parser for live use.

Device lines (newline-delimited text):
    #meta:key=value
    #cols:t_us,imu_qw,...          header; re-emitted, may change between builds
    D,<v>,<v>,...                  one row in header order; empty field = no data
    #evt,<t_us>,<text>             event; text runs to end of line
    #log,<text>                    ESP-IDF log line routed by the firmware (counted, ignored)
    #cfg,<node>,<name>,<index>,<sub>,<value>   read-only controller config (from `dump <node>`)
    anything else                  stray text: kept in raw.log, counted, ignored

Host lines (written into raw.log by capture.py, at line boundaries only, so the
device bytes stay byte-for-byte intact):
    #host,<unix_s>,<t_us_est>,<kind>,<text>
kind is note | cmd | sys. t_us_est is the device clock estimated from the last
row's t_us plus host time elapsed since it arrived (empty if no row yet).
"""

from __future__ import annotations

import math
import re
import time

ROW, COLS, META, EVT, HOST, LOG, CFG, STRAY, BLANK = ("row", "cols", "meta", "evt", "host", "log", "cfg", "stray",
                                                    "blank")

HOST_PREFIX = "#host,"

# The commands the firmware understands. Anything else typed at capture.py is a note.
FW_COMMANDS = ("hdr", "rate", "nodes", "stop", "mark", "arm", "pstep", "step", "dump", "clearfaults", "scan", "clamp",
               "disarm")

# Node 2 drives the two carriage sliders: never commanded from here (the firmware blocks it too).
NEVER_COMMAND_NODES = frozenset({2})
MOTION_RE = re.compile(r"^\s*(arm|pstep|step|clamp)\b\s*(?:n)?(\d+)?", re.I)


def motion_refusal(line: str) -> str | None:
    """Why a line must not be sent, or None. Motion commands naming a forbidden node are refused."""
    m = MOTION_RE.match(line)
    if not m:
        return None
    if m.group(2) is not None and int(m.group(2)) in NEVER_COMMAND_NODES:
        return f"refused: '{m.group(1)}' names node {m.group(2)} (carriage sliders, never commanded)"
    return None


def classify(line: str) -> str:
    if not line:
        return BLANK
    if line.startswith("D,"):
        return ROW
    if line.startswith("#cols:"):
        return COLS
    if line.startswith("#meta:"):
        return META
    if line.startswith("#evt,"):
        return EVT
    if line.startswith(HOST_PREFIX):
        return HOST
    if line.startswith("#log,"):
        return LOG
    if line.startswith("#cfg,"):
        return CFG
    if not line.strip():
        return BLANK
    return STRAY


def parse_cols(line: str) -> list[str]:
    return [c.strip() for c in line[len("#cols:"):].split(",")]


def parse_meta(line: str) -> tuple[str, str]:
    body = line[len("#meta:"):]
    k, _, v = body.partition("=")
    return k.strip(), v.strip()


def parse_evt(line: str) -> tuple[float, str] | None:
    parts = line.split(",", 2)
    if len(parts) < 3:
        return None
    try:
        return float(parts[1]), parts[2].strip()
    except ValueError:
        return None


def parse_cfg(line: str) -> dict | None:
    parts = line.split(",", 5)
    if len(parts) < 6:
        return None
    return {"node": parts[1].strip(), "name": parts[2].strip(), "index": parts[3].strip(), "sub": parts[4].strip(),
            "value": parts[5].strip()}


def parse_host(line: str) -> dict | None:
    parts = line.split(",", 4)
    if len(parts) < 5:
        return None
    try:
        unix_s = float(parts[1])
        t_us = float(parts[2]) if parts[2] else math.nan
    except ValueError:
        return None
    return {"unix_s": unix_s, "t_us": t_us, "kind": parts[3], "text": parts[4].strip()}


def format_host(kind: str, text: str, t_us_est: float | None = None, unix_s: float | None = None) -> bytes:
    unix_s = time.time() if unix_s is None else unix_s
    text = text.replace("\r", " ").replace("\n", " ")
    t = "" if t_us_est is None or not math.isfinite(t_us_est) else f"{t_us_est:.0f}"
    return f"{HOST_PREFIX}{unix_s:.6f},{t},{kind},{text}\n".encode("latin-1", "replace")


def parse_row(line: str, ncols: int) -> list[float] | None:
    """Values of a D row, NaN for empty fields; None if the row is garbled."""
    fields = line[2:].split(",")
    if len(fields) != ncols:
        return None
    out = []
    for f in fields:
        if f == "":
            out.append(math.nan)
            continue
        try:
            out.append(float(f))
        except ValueError:
            return None
    return out


class LiveParser:
    """Feeds whole lines, keeps the latest header and row, counts everything.

    capture.py uses this for the status line; runlog.py does the same job in bulk.
    """

    def __init__(self):
        self.cols: list[str] | None = None
        self.index: dict[str, int] = {}
        self.meta: dict[str, str] = {}
        self.counts = {k: 0 for k in (ROW, "garbled", COLS, META, EVT, HOST, LOG, CFG, STRAY)}
        self.cfg: dict[str, list] = {}
        self.last_row: list[float] | None = None
        self.last_row_host_time = 0.0
        self.row_times: list[float] = []  # host monotonic times of recent rows, for the rate

    def feed(self, raw: bytes, now: float | None = None) -> tuple[str, object]:
        now = time.monotonic() if now is None else now
        line = raw.decode("latin-1").rstrip("\r\n")
        kind = classify(line)
        if kind == ROW:
            vals = parse_row(line, len(self.cols)) if self.cols else None
            if vals is None:
                self.counts["garbled"] += 1
                return "garbled", line
            self.counts[ROW] += 1
            self.last_row = vals
            self.last_row_host_time = now
            self.row_times.append(now)
            if len(self.row_times) > 400:
                del self.row_times[:200]
            return ROW, vals
        if kind == COLS:
            self.cols = parse_cols(line)
            self.index = {c: i for i, c in enumerate(self.cols)}
        elif kind == META:
            k, v = parse_meta(line)
            self.meta[k] = v
        if kind in self.counts:
            self.counts[kind] += 1
        if kind == EVT:
            return EVT, parse_evt(line)
        if kind == CFG:
            c = parse_cfg(line)
            if c:
                self.cfg.setdefault(c["node"], []).append(c)
            return CFG, c
        return kind, line

    def value(self, name: str) -> float:
        i = self.index.get(name)
        if i is None or self.last_row is None:
            return math.nan
        return self.last_row[i]

    def row_rate(self, now: float | None = None, window: float = 2.0) -> float:
        now = time.monotonic() if now is None else now
        n = sum(1 for t in self.row_times if now - t <= window)
        return n / window

    def channels(self) -> list[str]:
        return channels_in(self.cols or [])


def channels_in(cols) -> list[str]:
    """Channel prefixes (n<node>c<ch>) present in a column list, in node/channel order."""
    import re

    found = set()
    for c in cols:
        m = re.match(r"^(n(\d+)c(\d+))_", c)
        if m:
            found.add((int(m.group(2)), int(m.group(3)), m.group(1)))
    return [p for _, _, p in sorted(found)]


def nodes_in(cols) -> list[str]:
    import re

    found = set()
    for c in cols:
        m = re.match(r"^n(\d+)(c\d+)?_", c)
        if m:
            found.add(int(m.group(1)))
    return [f"n{n}" for n in sorted(found)]
