"""kinematics: chassis pitch and roll against every axis's encoder counts.

Uses the quasi-static samples (every axis still for settle_s) of a run where the
axes are moved one at a time, and fits, for pitch and for roll,

    angle = c + sum_i J_i * (enc_i - mean_i) + sum_i h_i * d_i

d_i is +1 / -1 for the direction axis i last moved in. With backlash b between
encoder and chassis, the chassis sees enc - d*b/2, so h_i = -J_i * b_i / 2 and
b_i = -2 h.J / |J|^2 over pitch and roll together. J (degrees per count) is the
map a levelling controller needs. Linearity: residual RMS and the largest
deviation a quadratic term per axis would add over the travelled range.
"""

from __future__ import annotations

import numpy as np

from .. import attitude, plotstyle as ps, signals
from .common import header, meta_line


def _design(run, opts):
    att = attitude.attitude(run, opts.mount)
    if "pitch_g" not in att:
        return None
    t = run.t
    lag = (opts.imu_lag_ms or 0.0) / 1000
    axes, E, D, still = [], [], [], np.ones(len(t), dtype=bool)
    last_move = np.full(len(t), -np.inf)
    for p in run.channels():
        ts, x = signals.fresh(run, p, "enc")
        if len(ts) < 20 or np.ptp(x) < 20:
            continue
        tg, xg, vg = signals.velocity(ts, x, win_s=0.3)
        v = np.interp(t - lag, tg, np.nan_to_num(vg))
        e = np.interp(t - lag, ts, x)
        # one-count encoder flicker reads as a few counts/s; real moves are tens
        mov = np.abs(v) > 5.0
        still &= np.abs(v) < 4.0
        idx = np.where(mov, np.arange(len(t)), -1)
        np.maximum.accumulate(idx, out=idx)
        lm = np.where(idx >= 0, t[np.clip(idx, 0, None)], -np.inf)
        last_move = np.maximum(last_move, lm)
        sgn = np.where(mov, np.sign(v), 0.0)
        dstate = signals.hold(np.where(sgn != 0, sgn, np.nan))
        axes.append(p)
        E.append(e)
        D.append(dstate)
    if not axes:
        return None
    mask = still & (t - last_move >= opts.settle_s * 0.5) & np.isfinite(att["pitch_g"])
    return {"axes": axes, "E": np.column_stack(E), "D": np.column_stack(D), "mask": mask,
            "pitch": att["pitch_g"], "roll": att["roll_g"], "pitch_q": att.get("pitch_q"), "roll_q": att.get("roll_q")}


def _fit(Y, E, D, use_d, quad=False):
    Ec = E - E.mean(axis=0)
    cols = [np.ones(len(Y)), *Ec.T]
    cols += [D[:, i] for i in range(D.shape[1]) if use_d[i]]
    if quad:
        cols += [(Ec[:, i] ** 2 - np.mean(Ec[:, i] ** 2)) for i in range(E.shape[1])]
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, Y, rcond=None)
    res = Y - A @ coef
    dof = max(len(Y) - A.shape[1], 1)
    cov = np.linalg.pinv(A.T @ A) * (res @ res / dof)
    return coef, res, np.sqrt(np.clip(np.diag(cov), 0, None))


def compute(run, opts) -> dict:
    res = header("kinematics", run, opts)
    d = _design(run, opts)
    if d is None or d["mask"].sum() < 30:
        res["flags"].append("warning: not enough quasi-static samples with moved axes for a kinematic fit")
        res["axes"] = []
        return res
    m = d["mask"]
    E, D = d["E"][m], d["D"][m]
    n = E.shape[1]
    use_d = [len(np.unique(D[:, i][D[:, i] != 0])) == 2 for i in range(n)]
    out = {"axes": d["axes"], "names": [run.short(p) for p in d["axes"]], "n_samples": int(m.sum())}
    J, H, stats, quad, HSE = {}, {}, {}, {}, {}
    for key in ("pitch", "roll"):
        Y = d[key][m]
        coef, r, se = _fit(Y, E, D, use_d)
        J[key] = coef[1:1 + n]
        hfull = np.zeros(n)
        hfull[np.array(use_d)] = coef[1 + n:1 + n + sum(use_d)]
        H[key] = hfull
        hse = np.full(n, np.nan)
        hse[np.array(use_d)] = se[1 + n:1 + n + sum(use_d)]
        HSE[key] = hse
        ss = np.sum((Y - Y.mean()) ** 2)
        stats[key] = {"intercept_deg": float(coef[0]), "rms_resid_deg": float(np.sqrt(np.mean(r ** 2))),
                      "max_resid_deg": float(np.max(np.abs(r))), "r2": float(1 - r @ r / ss) if ss > 0 else None,
                      "se_J": se[1:1 + n].tolist()}
        cq, rq, _ = _fit(Y, E, D, use_d, quad=True)
        qc = cq[-n:]
        Ec = E - E.mean(axis=0)
        quad[key] = [float(np.max(np.abs(qc[i] * (Ec[:, i] ** 2 - np.mean(Ec[:, i] ** 2))))) for i in range(n)]
        if d.get(f"{key}_q") is not None:
            Yq = d[f"{key}_q"][m]
            ok = np.isfinite(Yq)
            if ok.sum() > 30:
                cqq, _, _ = _fit(Yq[ok], E[ok], D[ok], use_d)
                stats[key]["J_from_quaternion"] = cqq[1:1 + n].tolist()
    back, back_se = [], []
    for i in range(n):
        dom = "pitch" if abs(J["pitch"][i]) >= abs(J["roll"][i]) else "roll"
        back_se.append(float(2 * HSE[dom][i] / abs(J[dom][i])) if use_d[i] and J[dom][i] != 0 else None)
        jp, jr = J["pitch"][i], J["roll"][i]
        hp, hr = H["pitch"][i], H["roll"][i]
        den = jp * jp + jr * jr
        back.append(float(-2 * (hp * jp + hr * jr) / den) if use_d[i] and den > 0 else None)
    out["jacobian_deg_per_count"] = {p: {"pitch": float(J["pitch"][i]), "roll": float(J["roll"][i])}
                                     for i, p in enumerate(d["axes"])}
    out["jacobian_matrix"] = {"rows": ["pitch", "roll"], "cols": d["axes"],
                              "deg_per_count": [J["pitch"].tolist(), J["roll"].tolist()]}
    out["backlash_counts"] = dict(zip(d["axes"], back))
    out["backlash_se_counts"] = dict(zip(d["axes"], back_se))
    out["fit"] = stats
    out["nonlinearity_deg"] = {k: dict(zip(d["axes"], v)) for k, v in quad.items()}
    out["range_counts"] = {p: float(np.ptp(E[:, i])) for i, p in enumerate(d["axes"])}
    res.update(out)
    for key in ("pitch", "roll"):
        if stats[key]["rms_resid_deg"] > 0.3:
            res["flags"].append(f"warning: {key} residual {stats[key]['rms_resid_deg']:.2f} deg RMS: not linear, "
                                "or something else moved")
    return res


def plot(run, res, path, opts):
    axes = res.get("axes") or []
    if not axes:
        fig, axs = ps.figure(1, 1)
        ps.empty(axs[0, 0], "No moved axes with quasi-static samples.")
        axs[0, 0].set_xlabel("Encoder, counts")
        axs[0, 0].set_ylabel("Angle, deg")
        return ps.save(fig, path)
    d = _design(run, opts)
    m = d["mask"]
    E, D = d["E"][m], d["D"][m]
    n = len(axes)
    fig, axs = ps.figure(2, n, w=3.3 * n + 1, h_per=3.2)
    for r, key in enumerate(("pitch", "roll")):
        Y = d[key][m]
        J = np.array([res["jacobian_deg_per_count"][p][key] for p in axes])
        Ec = E - E.mean(axis=0)
        base = res["fit"][key]["intercept_deg"]
        for i, p in enumerate(axes):
            a = axs[r, i]
            others = Y - base - (Ec @ J - Ec[:, i] * J[i])
            for sgn, c in ((1, ps.series(0)), (-1, ps.series(1))):
                mm = D[:, i] == sgn
                a.plot(E[mm, i], others[mm], ".", color=c, ms=2.5, alpha=0.6)
            xx = np.linspace(E[:, i].min(), E[:, i].max(), 10)
            a.plot(xx, J[i] * (xx - E[:, i].mean()), color=ps.colors()["ink"], lw=1.2)
            a.set_xlabel(f"{run.short(p)} encoder, counts")
            a.set_ylabel(f"{key.capitalize()} share, deg")
            b = res["backlash_counts"].get(p)
            ps.title(a, f"{run.short(p)} {key}", f"{J[i]:+.5f} deg/count" + (f", backlash {b:.1f}" if b is not None else ""))
            if r == 0 and i == 0:
                ps.legend_above(a, [("last moved +", ps.series(0)), ("last moved -", ps.series(1))], inside=True)
    ps.page_title(fig, "Kinematics: chassis angle per count", meta_line(run) +
                  f"  |  residual pitch {res['fit']['pitch']['rms_resid_deg']:.3f}, roll {res['fit']['roll']['rms_resid_deg']:.3f} deg RMS")
    return ps.save(fig, path)


def summary_lines(res):
    out = []
    for p in res.get("axes", []):
        j = res["jacobian_deg_per_count"][p]
        b = res["backlash_counts"][p]
        out.append(f"{res['axis_names'].get(p, p):>6} ({p}): pitch {j['pitch']:+.6f}  roll {j['roll']:+.6f} deg/count  "
                   f"backlash {'-' if b is None else f'{b:.1f} +- {res["backlash_se_counts"][p]:.1f}'} counts  range {res['range_counts'][p]:.0f}")
    if res.get("fit"):
        for k in ("pitch", "roll"):
            f = res["fit"][k]
            out.append(f"{k}: residual {f['rms_resid_deg']:.3f} deg RMS, max {f['max_resid_deg']:.3f}, R2 {f['r2']:.4f}")
    return out
