"""Closed-loop position step metrics, one definition shared by the analysis and the simulator's truth.

Given samples x(t) after a target change at t0 from x0 to target:
  tol              max(3 counts, 2 % of the step): the settling band
  final            median position over the last 0.5 s of the window
  ss_error         target - final, counts (signed: + = stopped short when stepping up)
  rise_s           10 % to 90 % of the step to target; None if 90 % is never reached
  rise_actual_s    10 % to 90 % of the distance actually travelled (defined even when it stalls)
  overshoot        furthest excursion past the target in the step direction, counts and % of step
  settle_target_s  time after t0 from which |x - target| <= tol to the end; None if never
  settle_rest_s    time after t0 from which |x - final| <= tol to the end (when it stopped moving)
  stalled          came to rest outside the band: the "stalls short of target" case
  moving_at_end    still moving when the window ended (neither settled nor stalled; give it longer)
"""

from __future__ import annotations

import numpy as np


def _cross(t, frac_path, level):
    k = np.flatnonzero(frac_path >= level)
    return float(t[k[0]]) if len(k) else None


def _settle(t, x, ref, tol, t0):
    out = np.abs(x - ref) > tol
    if not out.any():
        return 0.0
    k = np.flatnonzero(out)[-1]
    if k == len(x) - 1:
        return None
    return float(t[k + 1] - t0)


def position_step(t, x, t0, x0, target, tol=None, tail_s=0.5):
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    step = target - x0
    tol = max(3.0, 0.02 * abs(step)) if tol is None else tol
    sgn = 1.0 if step >= 0 else -1.0
    tail = t >= t[-1] - tail_s
    final = float(np.median(x[tail]))
    frac = (x - x0) / step if step != 0 else np.zeros_like(x)
    t10, t90 = _cross(t, frac, 0.1), _cross(t, frac, 0.9)
    travelled = final - x0
    if abs(travelled) > tol:
        fa = (x - x0) / travelled
        a10, a90 = _cross(t, fa, 0.1), _cross(t, fa, 0.9)
    else:
        a10 = a90 = None
    over = float(max(0.0, np.max(sgn * (x - target))))
    s_t = _settle(t, x, target, tol, t0)
    s_r = _settle(t, x, final, tol, t0)
    return {
        "x0": float(x0), "target": float(target), "step_counts": float(step), "tol_counts": float(tol),
        "final": final, "ss_error_counts": float(target - final),
        "moved": bool(abs(travelled) > tol),
        "rise_s": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "rise_actual_s": (a90 - a10) if (a10 is not None and a90 is not None) else None,
        "delay_10pct_s": (t10 - t0) if t10 is not None else None,
        "overshoot_counts": over,
        "overshoot_pct": 100.0 * over / abs(step) if step else 0.0,
        "settle_target_s": s_t,
        "settle_rest_s": s_r,
        "stalled": bool(abs(target - final) > tol and s_r is not None),
        "moving_at_end": s_r is None,
    }
