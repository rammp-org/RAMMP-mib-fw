"""rest: a long capture with nothing moving. Do the EHAs hold with the drive released, and how
noisy is every signal?

Per channel: encoder creep (slope of a straight line through the CAN reads, counts per
minute, with its standard error) and the total change. Chassis pitch and roll drift
(degrees per minute, from the gravity vector). Noise floor of every signal: the
standard deviation after removing that straight line, plus the BNO055 gyro bias.
Moves inside the capture (any _tgt or command) are cut out with a 2 s margin.
"""

from __future__ import annotations

import numpy as np

from .. import attitude, plotstyle as ps, signals
from .common import header, meta_line


def _line(t, y):
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    if len(t) < 10 or np.ptp(t) < 5:
        return None
    A = np.column_stack([t, np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = y - A @ coef
    # serial correlation inflates the naive error: use effective samples from the lag-1 autocorrelation
    rho = float(np.corrcoef(r[:-1], r[1:])[0, 1]) if len(r) > 3 and np.std(r) > 0 else 0.0
    rho = min(max(rho, 0.0), 0.99)
    n_eff = len(r) * (1 - rho) / (1 + rho)
    se = float(np.sqrt(np.sum(r ** 2) / max(n_eff - 2, 1) / np.sum((t - t.mean()) ** 2)))
    return {"slope": float(coef[0]), "se": se, "sd": float(np.std(r)), "n": int(len(t)), "span_s": float(np.ptp(t))}


def _quiet(run, t):
    busy = np.zeros(len(t), dtype=bool)
    for a, b, _ in signals.moves(run, settle_s=0.0):
        busy |= (t >= a - 2.0) & (t <= b + 2.0)
    return ~busy


def compute(run, opts) -> dict:
    res = header("rest", run, opts)
    t = run.t
    quiet = _quiet(run, t)
    res["quiet_s"] = float(quiet.sum() / max(run.rate_hz(), 1))
    if res["quiet_s"] < 30:
        res["flags"].append(f"warning: only {res['quiet_s']:.0f} s without moves; capture at least 2 minutes")
    chans = {}
    for p in run.channels():
        d = {"name": run.short(p)}
        ts, x = signals.fresh(run, p, "enc")
        if len(ts) > 10:
            q = _quiet(run, ts)
            fit = _line(ts[q], x[q])
            if fit:
                d["creep_counts_per_min"] = 60 * fit["slope"]
                d["creep_se_counts_per_min"] = 60 * fit["se"]
                d["enc_change_counts"] = float(x[q][-1] - x[q][0])
                d["enc_noise_counts"] = fit["sd"]
                if abs(d["creep_counts_per_min"]) > max(3 * d["creep_se_counts_per_min"], 0.2):
                    res["flags"].append(f"note: {run.label(p)} creeps {d['creep_counts_per_min']:+.2f} "
                                        f"+- {d['creep_se_counts_per_min']:.2f} counts/min with the drive released")
        for f, unit, scale in (("spd", "counts/s", 1), ("cur_ma", "A", 1e-3), ("duty", "%", 1)):
            tt, v = signals.fresh(run, p, f)
            if len(tt) > 10:
                q = _quiet(run, tt)
                if q.sum() > 10:
                    d[f"{f}_noise"] = {"sd": float(np.std(v[q] * scale)), "mean": float(np.mean(v[q] * scale)),
                                       "unit": unit}
        chans[p] = d
    res["channels"] = chans
    att = attitude.attitude(run, opts.mount)
    imu = {}
    for k in ("pitch_g", "roll_g", "pitch_q", "roll_q"):
        if k in att:
            fit = _line(t[quiet], att[k][quiet])
            if fit:
                imu[k] = {"drift_deg_per_min": 60 * fit["slope"], "drift_se_deg_per_min": 60 * fit["se"],
                          "noise_deg": fit["sd"], "mean_deg": float(np.nanmean(att[k][quiet]))}
    for key, unit, scale in (("gyro_c", "deg/s", 57.29578), ("la_c", "m/s^2", 1.0)):
        if key in att:
            v = att[key][quiet] * scale
            imu[key] = {"sd": np.nanstd(v, axis=0).tolist(), "mean": np.nanmean(v, axis=0).tolist(), "unit": unit,
                        "axes": ["x", "y", "z"]}
    res["imu"] = imu
    for k in ("pitch_g", "roll_g"):
        if k in imu and abs(imu[k]["drift_deg_per_min"]) > max(3 * imu[k]["drift_se_deg_per_min"], 0.002):
            res["flags"].append(f"note: chassis {k[:-2]} drifts {imu[k]['drift_deg_per_min'] * 1000:+.1f} "
                                f"mdeg/min at rest")
    if "gyro_c" in imu and max(abs(v) for v in imu["gyro_c"]["mean"]) > 0.5:
        res["flags"].append(f"warning: gyro bias {imu['gyro_c']['mean']} deg/s")
    nodes = {}
    for n in run.nodes():
        vb = run.col(f"{n}_vbat_mv")
        if vb is not None:
            fit = _line(t[quiet], vb[quiet] / 1000)
            if fit:
                nodes[n] = {"vbat_V_per_min": 60 * fit["slope"], "vbat_noise_V": fit["sd"]}
    res["nodes"] = nodes
    res["summary"] = {"creep_counts_per_min": {p: d.get("creep_counts_per_min") for p, d in chans.items()},
                      "pitch_mdeg_per_min": 1000 * imu["pitch_g"]["drift_deg_per_min"] if "pitch_g" in imu else None,
                      "roll_mdeg_per_min": 1000 * imu["roll_g"]["drift_deg_per_min"] if "roll_g" in imu else None}
    return res


def plot(run, res, path, opts):
    fig, axs = ps.figure(2, 2, w=13, h_per=3.3)
    t = run.t
    quiet = _quiet(run, t)
    a = axs[0, 0]
    items = []
    for i, p in enumerate(run.channels()):
        ts, x = signals.fresh(run, p, "enc")
        if len(ts) < 10:
            continue
        q = _quiet(run, ts)
        if q.sum() < 2:
            continue
        from scipy.ndimage import median_filter

        xq = median_filter(x[q], size=max(3, int(5.0 * len(x[q]) / max(np.ptp(ts[q]), 1)) | 1), mode="nearest")
        k = max(1, q.sum() // 3000)
        a.plot(ts[q][::k] / 60, (xq - xq[0])[::k], color=ps.series(i), lw=1.6)
        items.append((run.short(p), ps.series(i)))
    a.set_xlabel("Time, min")
    a.set_ylabel("Encoder change, counts")
    ps.title(a, "Encoder creep", "drive released; 5 s running median, change from the start")
    if items:
        ps.legend_above(a, items[:6], inside="lower left")
    a = axs[0, 1]
    att = attitude.attitude(run, opts.mount)
    for i, (k, lab) in enumerate((("pitch_g", "pitch"), ("roll_g", "roll"))):
        if k in att:
            y = signals.lowpass(signals.hold(att[k]), max(run.rate_hz(), 2), 0.5)
            qq = quiet & (t > t[0] + 5) & (t < t[-1] - 5)  # clear of the filter's edges
            kk = max(1, qq.sum() // 3000)
            if qq.sum() < 2:
                continue
            a.plot(t[qq][::kk] / 60, (y[qq] - np.nanmean(y[qq][:200]))[::kk], color=ps.series(i), lw=1.4,
                   label=lab)
    a.set_xlabel("Time, min")
    a.set_ylabel("Angle change, deg")
    imu_ = res["imu"]
    nz = (f"; noise pitch {imu_['pitch_g']['noise_deg']:.3f}, roll {imu_['roll_g']['noise_deg']:.3f} deg"
          if "pitch_g" in imu_ and "roll_g" in imu_ else "")
    ps.title(a, "Chassis attitude drift", "gravity vector, 0.5 Hz low-pass" + nz)
    if a.get_lines() and quiet.any():
        ps.label_lines(a)
    else:
        ps.empty(a, "No quiet stretch in this run.")
    a = axs[1, 0]
    ch = res["channels"]
    cr = [(d["name"], d["creep_counts_per_min"]) for d in ch.values() if d.get("creep_counts_per_min") is not None]
    labs = [f"{n} {'down' if v < 0 else 'up'}" for n, v in cr]
    vals = [round(abs(v), 3) for _, v in cr]
    if vals:
        ps.bars(a, labs, vals, xlabel="Creep, counts/min", ylabel="Channel")
    else:
        ps.empty(a, "No encoder data.")
        a.set_xlabel("Creep, counts/min")
        a.set_ylabel("Channel")
    ps.title(a, "Creep rate", "size of the slope through the quiet reads; down = toward fewer counts")
    a = axs[1, 1]
    labs, vals = [], []
    imu = res["imu"]
    if "gyro_c" in imu:
        for j, n in enumerate("xyz"):
            labs.append(f"gyro {n}")
            vals.append(round(imu["gyro_c"]["sd"][j], 4))
    if vals:
        ps.bars(a, labs, vals, xlabel="Gyro noise (SD), deg/s", ylabel="Axis")
    else:
        ps.empty(a, "No IMU data.")
        a.set_xlabel("Gyro noise (SD), deg/s")
        a.set_ylabel("Axis")
    bias = (" ".join(f"{v:+.3f}" for v in imu["gyro_c"]["mean"]) + " deg/s") if "gyro_c" in imu else "-"
    ps.title(a, "Gyro noise floor", f"at rest, chassis frame; bias x y z {bias}")
    ps.page_title(fig, "At rest: creep, drift, noise", meta_line(run) + f"  |  quiet {res['quiet_s'] / 60:.1f} min")
    return ps.save(fig, path)


def summary_lines(res):
    out = [f"quiet {res['quiet_s']:.0f} s"]
    for p, d in res["channels"].items():
        if d.get("creep_counts_per_min") is not None:
            out.append(f"{d['name']:>6}: creep {d['creep_counts_per_min']:+.3f} +- {d['creep_se_counts_per_min']:.3f} "
                       f"counts/min  change {d['enc_change_counts']:+.0f}  enc noise {d['enc_noise_counts']:.2f} counts")
    imu = res["imu"]
    for k in ("pitch_g", "roll_g"):
        if k in imu:
            out.append(f"{k[:-2]}: drift {1000 * imu[k]['drift_deg_per_min']:+.2f} +- "
                       f"{1000 * imu[k]['drift_se_deg_per_min']:.2f} mdeg/min  noise {imu[k]['noise_deg']:.4f} deg")
    if "gyro_c" in imu:
        out.append("gyro mean " + " ".join(f"{v:+.3f}" for v in imu["gyro_c"]["mean"]) + " deg/s  sd "
                   + " ".join(f"{v:.3f}" for v in imu["gyro_c"]["sd"]))
    return out
