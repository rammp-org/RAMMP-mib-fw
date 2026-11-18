"""Time-base check: is the row clock steady, and how many rows went missing?"""

from __future__ import annotations

import numpy as np


def check(run) -> dict:
    t = run.t_us
    out = {"rows": int(len(t))}
    if len(t) < 3:
        out["ok"] = False
        return out
    dt = np.diff(t)
    nominal_hz = run.rate_hz()
    meta_period = 1e6 / nominal_hz if np.isfinite(nominal_hz) and nominal_hz > 0 else np.nan
    period = float(np.median(dt))
    if np.isfinite(meta_period) and abs(period - meta_period) / meta_period < 0.2:
        period = meta_period
    steps = np.rint(dt / period)
    normal = steps == 1
    jitter = dt[normal] - period
    gaps = steps > 1
    dropped = int(np.sum(steps[gaps] - 1))
    nonmono = int(np.sum(dt <= 0))
    gap_idx = np.flatnonzero(gaps)
    biggest = sorted(((float(dt[i]) * 1e-3, float((t[i] - t[0]) * 1e-6)) for i in gap_idx), reverse=True)[:5]
    span = (t[-1] - t[0]) * 1e-6
    out.update({
        "nominal_rate_hz": round(1e6 / period, 3),
        "meta_rate_hz": nominal_hz if np.isfinite(nominal_hz) else None,
        "measured_rate_hz": round((len(t) - 1) / span, 3) if span > 0 else None,
        "duration_s": round(span, 3),
        "jitter_std_us": round(float(np.std(jitter)), 1) if len(jitter) else None,
        "jitter_p99_us": round(float(np.percentile(np.abs(jitter), 99)), 1) if len(jitter) else None,
        "gaps": int(gaps.sum()),
        "dropped_rows_est": dropped,
        "dropped_pct": round(100.0 * dropped / (len(t) + dropped), 3),
        "non_monotonic": nonmono,
        "largest_gaps_ms_at_s": [[round(a, 1), round(b, 2)] for a, b in biggest],
        "garbled_lines": run.stats.get("garbled", 0),
        "stray_lines": run.stats.get("stray", 0),
        "log_lines": run.stats.get("log", 0),
        "header_changes": run.stats.get("header_changes", 0),
        "clock_wraps": run.stats.get("wraps", 0),
        "clock_resets": run.stats.get("resets", 0),
    })
    out["ok"] = bool(out["dropped_pct"] < 1.0 and nonmono == 0 and (out["jitter_p99_us"] or 0) < 0.25 * period)
    return out
