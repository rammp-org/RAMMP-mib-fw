"""First-order-plus-dead-time velocity response, fitted to encoder position.

Fitting position, not a differentiated velocity, keeps the dead time and time
constant free of the smoothing a derivative needs; the encoder's one-count
quantisation is then just small noise. With the step applied at t0:

    s = t - t0 - L
    x(t) = x0                                   s <= 0
    x(t) = x0 + v_ss * (s - tau * (1 - e^(-s/tau)))   s > 0
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


def model(t, t0, x0, v_ss, tau, L):
    s = np.maximum(t - t0 - L, 0.0)
    return x0 + v_ss * (s - tau * (1.0 - np.exp(-s / tau)))


def model_v(t, t0, v_ss, tau, L, v0=0.0):
    s = t - t0 - L
    return np.where(s > 0, v0 + (v_ss - v0) * (1.0 - np.exp(-np.maximum(s, 0) / tau)), v0)


def fit(ts, x, t0, t_end, pre_s=0.25, tv=None, v=None, sigma_x=0.6, sigma_v=None):
    """Fit samples between t0 - pre_s and t_end. Returns dict or None.

    With tv, v (the controller's own speed readback at its read times) the fit is
    joint: position residuals / sigma_x and speed residuals / sigma_v. At low speeds
    position alone pins down L + tau well but not L and tau apart; the speed
    readback separates them. Without it, trust L + tau ("lag_total") over L and tau.
    """
    m = (ts >= t0 - pre_s) & (ts <= t_end)
    t, y = ts[m], x[m]
    if tv is not None and v is not None and len(tv):
        mv = (tv >= t0 - pre_s) & (tv <= t_end) & np.isfinite(v)
        tv, vv = tv[mv], v[mv]
        if len(tv) < 8:
            tv = vv = None
    else:
        tv = vv = None
    if len(t) < 8:
        return None
    pre = t < t0
    x0 = float(np.median(y[pre])) if pre.any() else float(y[0])
    tail = t > t0 + 0.6 * (t_end - t0)
    if tail.sum() >= 3:
        v_init = float(np.polyfit(t[tail], y[tail], 1)[0])
    else:
        v_init = (y[-1] - x0) / max(t_end - t0, 1e-3)
    if abs(v_init) < 1e-6:
        return None
    if tv is not None and sigma_v is None:
        pre_v = vv[tv < t0]
        sigma_v = max(float(np.std(pre_v)) if len(pre_v) > 3 else 1.0, 0.5)

    def resid(p):
        r = (model(t, t0, *p) - y) / sigma_x
        if tv is None:
            return r
        return np.concatenate([r, (model_v(tv, t0, p[1], p[2], p[3]) - vv) / sigma_v])

    best = None
    for tau0 in (0.05, 0.15, 0.4):
        for L0 in (0.02, 0.06, 0.15):
            p0 = [x0, v_init, tau0, L0]
            lo = [x0 - 50, -abs(v_init) * 5 - 1, 0.003, 0.0]
            hi = [x0 + 50, abs(v_init) * 5 + 1, 3.0, 0.6]
            try:
                r = least_squares(resid, p0, bounds=(lo, hi), loss="soft_l1", f_scale=2.0)
            except ValueError:
                continue
            if best is None or r.cost < best.cost:
                best = r
    if best is None:
        return None
    x0f, v, tau, L = best.x
    res = model(t, t0, *best.x) - y
    # parameter standard errors from the Jacobian
    try:
        J = best.jac
        rr = resid(best.x)
        cov = np.linalg.pinv(J.T @ J) * (np.sum(rr**2) / max(len(rr) - 4, 1))
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(4, np.nan)
    return {"x0": float(x0f), "v_ss": float(v), "tau": float(tau), "L": float(L),
            "se_v_ss": float(se[1]), "se_tau": float(se[2]), "se_L": float(se[3]),
            "lag_total": float(L + tau), "rms_counts": float(np.sqrt(np.mean(res**2))), "n": int(len(t)),
            "joint_speed": tv is not None}
