"""Shared pieces for the analyses: options, JSON output, small numeric helpers."""

from __future__ import annotations

import datetime as _dt
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..attitude import Mount


@dataclass
class Opts:
    mount: Mount = field(default_factory=Mount)
    source: str = "auto"  # excitation source: auto | cmd | duty | events
    fc_hz: float = 5.0  # smoothness low-pass
    settle_s: float = 1.0
    move_thresh: float = 3.0  # counts that count as motion
    vel_win_s: float = 0.12
    imu_lag_ms: float | None = None
    tol_counts: float | None = None  # position settling band
    deadband_json: str | None = None
    compare_metrics: list = field(default_factory=list)
    compare_runs: list = field(default_factory=list)
    compare_analysis: str = "smoothness"


def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items() if not str(k).startswith("_")}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return jsonable(o.tolist())
    if isinstance(o, (np.floating, float)):
        f = float(o)
        return None if not math.isfinite(f) else round(f, 6)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def write_json(path: Path, res: dict):
    path.write_text(json.dumps(jsonable(res), indent=1))
    return path


def header(name, run, opts: Opts, extra=None) -> dict:
    h = {"analysis": name, "run": str(run.path), "generated": _dt.datetime.now().isoformat(timespec="seconds"),
         "mount": opts.mount.to_json(), "fw_meta": run.meta, "axis_names": run.names, "flags": []}
    if extra:
        h.update(extra)
    return h


def median_or_none(xs):
    xs = [x for x in xs if x is not None and np.isfinite(x)]
    return float(np.median(xs)) if xs else None


def slope(t, x):
    if len(t) < 3 or np.ptp(t) <= 0:
        return math.nan
    return float(np.polyfit(t, x, 1)[0])


def dir_name(sign):
    return "pos" if sign > 0 else "neg"


def meta_line(run) -> str:
    fw = run.meta.get("fw") or run.meta.get("fw_version") or run.meta.get("firmware") or "fw ?"
    return f"{run.path.name}  |  {fw}  |  {run.stats.get('rows', 0)} rows"
