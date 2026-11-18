"""Load a raw.log into arrays keyed by column name.

Tolerant by design: rows are mapped by the header in force when they arrived,
garbled rows and stray text are counted and skipped, empty fields become NaN,
a 32-bit t_us wrap and a firmware reboot (t_us jumping back) are stitched into
one increasing clock. The result is cached next to raw.log as parsed.npz so the
second analysis of a run starts in milliseconds.
"""

from __future__ import annotations

import io
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import wire

PARSER_VERSION = 5
_EMPTY = re.compile(r"(?<![^,\n])(?![^,\n])")


@dataclass
class Event:
    t_us: float
    t: float  # seconds from the first row
    text: str
    source: str  # "fw" or "host"
    kind: str = ""  # host events: note | cmd | sys

    def to_dict(self):
        return {"t_us": self.t_us, "t": self.t, "text": self.text, "source": self.source, "kind": self.kind}


@dataclass
class Run:
    path: Path  # run directory
    raw: Path
    t_us: np.ndarray
    cols: dict[str, np.ndarray]
    events: list[Event]
    meta: dict  # last value of each #meta key
    stats: dict
    capture_meta: dict = field(default_factory=dict)  # meta.json written by capture.py / simulate.py
    names: dict[str, str] = field(default_factory=dict)  # n1c1 -> "FC"
    cfg: dict = field(default_factory=dict)  # node -> [#cfg entries]

    @property
    def t(self) -> np.ndarray:
        return (self.t_us - self.t0_us) * 1e-6

    @property
    def t0_us(self) -> float:
        return float(self.t_us[0]) if len(self.t_us) else 0.0

    def col(self, name: str):
        return self.cols.get(name)

    def channels(self) -> list[str]:
        return wire.channels_in(self.cols)

    def nodes(self) -> list[str]:
        return wire.nodes_in(self.cols)

    def label(self, prefix: str) -> str:
        n = self.names.get(prefix)
        return f"{n} ({prefix})" if n else prefix

    def short(self, prefix: str) -> str:
        return self.names.get(prefix, prefix)

    def fw_events(self):
        return [e for e in self.events if e.source == "fw"]

    def rate_hz(self) -> float:
        for k in ("sample_rate_hz", "rate_hz", "rate"):
            if k in self.meta:
                try:
                    return float(self.meta[k])
                except ValueError:
                    pass
        if len(self.t_us) > 2:
            return 1e6 / float(np.median(np.diff(self.t_us)))
        return math.nan


def resolve(path) -> tuple[Path, Path]:
    """(run dir, raw.log) from a run directory or a log file."""
    p = Path(path)
    if p.is_dir():
        return p, p / "raw.log"
    return p.parent, p


def load_run(path, use_cache: bool = True, names: dict | None = None) -> Run:
    run_dir, raw = resolve(path)
    if not raw.exists():
        raise FileNotFoundError(f"no raw.log at {raw}")
    st = raw.stat()
    key = f"{PARSER_VERSION}:{st.st_size}:{st.st_mtime_ns}"
    cache = run_dir / "parsed.npz"
    run = None
    if use_cache and cache.exists():
        try:
            run = _load_cache(cache, key, run_dir, raw)
        except Exception:
            run = None
    if run is None:
        run = parse_bytes(raw.read_bytes(), run_dir, raw)
        if use_cache:
            try:
                _save_cache(cache, key, run)
            except OSError:
                pass
    cm = run_dir / "meta.json"
    if cm.exists():
        try:
            run.capture_meta = json.loads(cm.read_text())
        except ValueError:
            run.capture_meta = {}
    run.names = _names(run, names)
    return run


def _names(run: Run, override: dict | None) -> dict:
    out = {}
    for src in (_run_dir_names(run), run.capture_meta.get("axis_names") or {}, override or {}):
        out.update(src)
    return out


def _run_dir_names(run) -> dict:
    # runs/axis_names.json applies to every run under that folder
    for d in (run.path, run.path.parent):
        f = d / "axis_names.json"
        if f.exists():
            try:
                return json.loads(f.read_text())
            except ValueError:
                pass
    return {}



def parse_names(spec: str | None) -> dict:
    """'n1c1=FC,n1c2=RC' -> dict."""
    if not spec:
        return {}
    out = {}
    for part in spec.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def parse_bytes(data: bytes, run_dir: Path | None = None, raw: Path | None = None) -> Run:
    text = data.decode("latin-1")
    lines = text.split("\n")
    if lines and lines[-1] == "":
        lines.pop()

    stats = {k: 0 for k in ("lines", "rows", "garbled", "rows_before_header", "headers", "header_changes",
                            "meta", "events", "host", "log", "cfg", "stray", "blank", "wraps", "resets")}
    cfg: dict[str, list] = {}
    meta: dict[str, str] = {}
    events_raw: list[tuple[float, str, str, str]] = []  # (t_us corrected, text, source, kind)
    segments: list[tuple[list[str], list[str]]] = []  # (cols, row bodies)
    cur_cols: list[str] | None = None
    cur_rows: list[str] | None = None
    t_idx = -1
    stitch = _Stitch(stats)
    stray_samples: list[str] = []
    t_rows: list[float] = []

    for line in lines:
        stats["lines"] += 1
        if line.endswith("\r"):
            line = line[:-1]
        kind = wire.classify(line)
        if kind == wire.ROW:
            if cur_cols is None:
                stats["rows_before_header"] += 1
                stats["garbled"] += 1
                continue
            body = line[2:]
            if body.count(",") != len(cur_cols) - 1:
                stats["garbled"] += 1
                continue
            # the row's time, for wrap/reset stitching; the full parse is in bulk below
            try:
                if t_idx == 0:
                    raw_t = float(body[: body.index(",")] if "," in body else body)
                else:
                    raw_t = float(body.split(",")[t_idx])
            except (ValueError, IndexError):
                stats["garbled"] += 1
                continue
            t = stitch.map(raw_t)
            cur_rows.append(body)
            t_rows.append(t)
            stats["rows"] += 1
        elif kind == wire.COLS:
            cols = wire.parse_cols(line)
            stats["headers"] += 1
            if "t_us" not in cols:
                stats["garbled"] += 1
                continue
            if cur_cols is None or cols != cur_cols:
                if cur_cols is not None:
                    stats["header_changes"] += 1
                cur_cols = cols
                cur_rows = []
                segments.append((cur_cols, cur_rows))
                t_idx = cols.index("t_us")
        elif kind == wire.META:
            k, v = wire.parse_meta(line)
            meta[k] = v
            stats["meta"] += 1
        elif kind == wire.EVT:
            ev = wire.parse_evt(line)
            if ev is None:
                stats["stray"] += 1
                continue
            t_raw, txt = ev
            t = stitch.map(raw_t=t_raw, is_event=True)
            events_raw.append((t, txt, "fw", ""))
            stats["events"] += 1
        elif kind == wire.HOST:
            h = wire.parse_host(line)
            if h is None:
                stats["stray"] += 1
                continue
            events_raw.append((stitch.map(h["t_us"], is_event=True) if math.isfinite(h["t_us"]) else math.nan,
                               h["text"], "host", h["kind"]))
            stats["host"] += 1
        elif kind == wire.LOG:
            stats["log"] += 1
        elif kind == wire.CFG:
            c = wire.parse_cfg(line)
            if c:
                cfg.setdefault(c["node"], []).append(c)
            stats["cfg"] += 1
        elif kind == wire.BLANK:
            stats["blank"] += 1
        else:
            stats["stray"] += 1
            if len(stray_samples) < 20:
                stray_samples.append(line[:120])

    # bulk conversion per header segment
    all_cols: list[str] = []
    for cols, _ in segments:
        for c in cols:
            if c not in all_cols:
                all_cols.append(c)
    n_total = sum(len(r) for _, r in segments)
    out = {c: np.full(n_total, np.nan) for c in all_cols}
    keep = np.ones(n_total, dtype=bool)
    pos = 0
    for cols, rows in segments:
        n = len(rows)
        if n == 0:
            continue
        arr = _to_array(rows, len(cols))
        bad = np.isnan(arr[:, cols.index("t_us")])
        for j, c in enumerate(cols):
            out[c][pos:pos + n] = arr[:, j]
        keep[pos:pos + n] = ~bad
        pos += n
    t_us = np.asarray(t_rows, dtype=float)
    nbad = int((~keep).sum())
    if nbad:
        stats["garbled"] += nbad
        stats["rows"] -= nbad
        t_us = t_us[keep]
        out = {c: v[keep] for c, v in out.items()}
    out["t_us"] = t_us
    stats["stray_samples"] = stray_samples
    # the firmware may log duty raw (Basicmicro -32767..32767); analyses work in percent
    if meta.get("unit_duty", "").replace(" ", "").startswith("-32767..32767"):
        for c in out:
            if c.endswith("_duty"):
                out[c] = out[c] * (100.0 / 32767.0)
        stats["duty_scaled_from_raw"] = True
    t0 = float(t_us[0]) if len(t_us) else 0.0
    for tb in stitch.boundaries:
        events_raw.append((tb, "segment boundary: device clock restarted (reset or replug)", "host", "sys"))
    stats["segments"] = len(stitch.boundaries) + 1
    events = [Event(t, (t - t0) * 1e-6 if math.isfinite(t) else math.nan, txt, src, k)
              for t, txt, src, k in events_raw]
    _fill_host_times(events, t_us)
    events.sort(key=lambda e: (math.inf if not math.isfinite(e.t_us) else e.t_us))
    return Run(path=run_dir or Path("."), raw=raw or Path("raw.log"), t_us=t_us, cols=out,
               events=events, meta=meta, stats=stats, cfg=cfg)


class _Stitch:
    """One increasing clock from a t_us that may wrap (32-bit firmware) or restart (reset, replug).

    A wrap is only assumed when the last time was within 10 s of 2^32 and the new
    one within 10 s of zero; any other step backwards of more than 50 ms is a new
    segment, placed one row period after the last time seen. Events go through
    the same map, so an event printed after a reset but before the first row of
    the new segment lands in the new segment.
    """

    def __init__(self, stats):
        self.offset = 0.0
        self.last_raw = None
        self.last_t = -math.inf
        self.period = 10000.0
        self.stats = stats
        self.boundaries: list[float] = []

    def map(self, raw_t: float, is_event: bool = False) -> float:
        t = raw_t + self.offset
        if self.last_raw is not None and t < self.last_t - 50000:
            if self.last_raw > 2**32 - 10e6 and raw_t < 10e6:
                self.offset += 2**32
                self.stats["wraps"] += 1
            else:
                self.offset = self.last_t + self.period - raw_t
                self.stats["resets"] += 1
                self.boundaries.append(self.last_t + self.period)
            t = raw_t + self.offset
        if not is_event:
            if self.last_raw is not None and 0 < t - self.last_t < 1e5:
                self.period = 0.95 * self.period + 0.05 * (t - self.last_t)
        if not is_event or self.last_raw is None or t > self.last_t:
            self.last_t = max(self.last_t, t)
            self.last_raw = raw_t
        return t


def _fill_host_times(events, t_us):
    # a host event written before the first row has no device time: pin it to the first row
    if not len(t_us):
        return
    t0 = float(t_us[0])
    for e in events:
        if not math.isfinite(e.t_us):
            e.t_us = t0
            e.t = 0.0


def _to_array(rows: list[str], ncols: int) -> np.ndarray:
    """Fast path through numpy's C parser; per-row fallback when a field is garbage."""
    text = "\n".join(rows)
    text = _EMPTY.sub("nan", text)
    try:
        arr = np.loadtxt(io.StringIO(text), delimiter=",", dtype=float, ndmin=2)
        if arr.shape == (len(rows), ncols):
            return arr
    except ValueError:
        pass
    arr = np.full((len(rows), ncols), np.nan)
    for i, r in enumerate(rows):
        vals = wire.parse_row("D," + r, ncols)
        if vals is not None:
            arr[i] = vals
    return arr


def _save_cache(path: Path, key: str, run: Run):
    arrays = {f"c_{k}": v for k, v in run.cols.items()}
    side = {"key": key, "events": [e.to_dict() for e in run.events], "meta": run.meta, "stats": run.stats,
            "cfg": run.cfg}
    with open(path, "wb") as f:
        np.savez(f, __side__=np.frombuffer(json.dumps(side).encode(), dtype=np.uint8), **arrays)


def _load_cache(path: Path, key: str, run_dir: Path, raw: Path) -> Run | None:
    with np.load(path) as z:
        side = json.loads(bytes(z["__side__"]).decode())
        if side.get("key") != key:
            return None
        cols = {k[2:]: z[k] for k in z.files if k.startswith("c_")}
    events = [Event(**e) for e in side["events"]]
    return Run(path=run_dir, raw=raw, t_us=cols["t_us"], cols=cols, events=events, meta=side["meta"],
               stats=side["stats"], cfg=side.get("cfg", {}))
