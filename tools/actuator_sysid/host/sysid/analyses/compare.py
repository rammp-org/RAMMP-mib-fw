"""compare: the same metric across runs, as bars on one axis per metric, plus an overlay
of each run's trace (first move) when the analysis provides one.

Metrics are dotted paths into each run's <analysis>.json, with * wildcards:
    smoothness: summary.rms_jerk_corrected  summary.peak_ang_acc
    step:       channels.*.pos.gain  channels.*.neg.tau_s
    latency:    summary.dead_time_s
    pstep:      channels.*.summary.median_abs_ss_error_counts
    plant:      channels.*.pos.deadband_pct
"""

from __future__ import annotations

import fnmatch
import json
from pathlib import Path

from .. import plotstyle as ps

DEFAULT_METRICS = {
    "smoothness": ["summary.rms_jerk_corrected", "summary.peak_jerk", "summary.rms_ang_acc_corrected",
                   "summary.peak_ang_acc"],
    "step": ["channels.*.pos.gain", "channels.*.neg.gain", "channels.*.pos.tau_s"],
    "deadband": ["channels.*.pos.breakaway_pct", "channels.*.neg.breakaway_pct"],
    "latency": ["summary.dead_time_s", "summary.enc_to_imu_lag_s"],
    "kinematics": ["fit.pitch.rms_resid_deg", "fit.roll.rms_resid_deg"],
    "pstep": ["channels.*.summary.median_abs_ss_error_counts", "channels.*.summary.max_overshoot_counts",
              "channels.*.summary.median_rise_actual_s"],
    "plant": ["channels.*.pos.gain", "channels.*.neg.gain", "channels.*.pos.deadband_pct", "channels.*.neg.deadband_pct"],
    "rest": ["channels.*.creep_counts_per_min", "summary.pitch_mdeg_per_min"],
}


def flatten(d, prefix=""):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            out.update(flatten(v, f"{prefix}{k}."))
    elif isinstance(d, (int, float)) and not isinstance(d, bool):
        out[prefix[:-1]] = float(d)
    return out


def compute_from_jsons(items, analysis, metrics=None):
    """items: [(label, result dict)]."""
    metrics = metrics or DEFAULT_METRICS.get(analysis, [])
    table = {}
    for label, res in items:
        flat = flatten({k: v for k, v in res.items() if k not in ("trace", "events", "mount", "fw_meta")})
        for pat in metrics:
            for key, v in flat.items():
                if fnmatch.fnmatchcase(key, pat):
                    table.setdefault(pat, []).append({"run": label, "key": key, "value": v})
    return {"analysis": "compare", "of": analysis, "runs": [l for l, _ in items], "metrics": table,
            "traces": {l: r.get("trace") for l, r in items if r.get("trace")}, "flags": []}


def plot(res, path):
    metrics = list(res["metrics"])
    traces = res.get("traces") or {}
    n = len(metrics) + (1 if traces else 0)
    if n == 0:
        fig, axs = ps.figure(1, 1)
        ps.empty(axs[0, 0], "No matching metrics in these runs.")
        axs[0, 0].set_xlabel("Value")
        axs[0, 0].set_ylabel("Run")
        return ps.save(fig, path)
    cols = 2 if n > 1 else 1
    rows = (n + cols - 1) // cols
    fig, axs = ps.figure(rows, cols, w=13, h_per=3.0)
    k = 0
    for pat in metrics:
        a = axs[k // cols, k % cols]
        rows_ = res["metrics"][pat]
        multi = len({r["key"] for r in rows_}) > 1
        labels = [f"{r['run']}" + (f"  {_short(r['key'], pat)}" if multi else "") for r in rows_]
        ps.bars(a, labels, [round(r["value"], 4) for r in rows_], xlabel=_unit(pat), ylabel="Run")
        ps.title(a, pat.split(".")[-1].replace("_", " "), pat)
        k += 1
    if traces:
        a = axs[k // cols, k % cols]
        any_tr = None
        for i, (label, tr) in enumerate(list(traces.items())[:6]):
            a.plot(tr["t"], tr["y"], color=ps.series(i), lw=1.4, label=label)
            any_tr = tr
        a.set_xlabel(any_tr["xlabel"])
        a.set_ylabel(any_tr["ylabel"])
        if len(traces) <= 4:
            ps.label_lines(a)
        else:
            ps.legend_above(a, [(l, ps.series(i)) for i, l in enumerate(list(traces)[:6])])
        ps.title(a, "First move overlay", "aligned at the move start")
        k += 1
    for j in range(k, rows * cols):
        axs[j // cols, j % cols].set_visible(False)
    ps.page_title(fig, f"Compare {res['of']}", f"{len(res['runs'])} runs")
    return ps.save(fig, path)


def _short(key, pat):
    a, b = key.split("."), pat.split(".")
    return ".".join(x for x, y in zip(a, b) if y == "*") or key


def _unit(pat):
    last = pat.split(".")[-1]
    for suf, u in (("_s", "Time, s"), ("_pct", "Duty, %"), ("_deg", "Angle, deg"), ("_counts", "Counts, counts"),
                   ("gain", "Gain, counts/s per %"), ("jerk", "Jerk, m/s³"), ("ang_acc", "Angular accel, deg/s²"),
                   ("_A", "Current, A")):
        if suf in last:
            return u
    return "Value, as logged"


def summary_lines(res):
    out = []
    for pat, rows in res["metrics"].items():
        out.append(pat)
        for r in rows:
            out.append(f"   {r['run']:<40} {r['key']:<40} {r['value']:.5g}")
    return out


def load_json(run_dir: Path, analysis: str):
    f = Path(run_dir) / f"{analysis}.json"
    return json.loads(f.read_text()) if f.exists() else None
