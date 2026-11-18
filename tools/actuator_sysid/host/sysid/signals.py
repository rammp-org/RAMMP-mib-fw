"""Signal helpers: fresh CAN samples, smoothed derivatives, command plateaus, event refinement."""

from __future__ import annotations

import re
from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter


def fresh(run, prefix: str, field: str = "enc") -> tuple[np.ndarray, np.ndarray]:
    """(sample time s, value) for the CAN reads actually made.

    A row repeats the last read until a new one arrives; <prefix>_age_us says how
    old it is, so the read happened at t - age. Rows whose read time matches the
    previous row's are repeats and are dropped. Without an age column every row counts.
    """
    v = run.col(f"{prefix}_{field}")
    if v is None:
        return np.array([]), np.array([])
    t = run.t
    age = run.col(f"{prefix}_age_us")
    ts = t - age * 1e-6 if age is not None else t.copy()
    ok = np.isfinite(v) & np.isfinite(ts)
    ts, v = ts[ok], v[ok]
    if age is not None and len(ts) > 1:
        new = np.concatenate([[True], np.abs(np.diff(ts)) > 0.5e-3])
        ts, v = ts[new], v[new]
    order = np.argsort(ts, kind="stable")
    return ts[order], v[order]


def odd(n: int) -> int:
    n = max(int(n), 3)
    return n if n % 2 else n + 1


def velocity(ts: np.ndarray, x: np.ndarray, fs: float = 200.0, win_s: float = 0.12, max_gap_s: float = 0.25):
    """Smoothed derivative of irregular samples: resample to fs, Savitzky-Golay slope.

    Returns (t grid, x on grid, v on grid). Spans with no sample for max_gap_s are NaN.
    """
    if len(ts) < 5:
        return np.array([]), np.array([]), np.array([])
    tg = np.arange(ts[0], ts[-1], 1.0 / fs)
    xg = np.interp(tg, ts, x)
    w = odd(win_s * fs)
    if len(tg) <= w:
        return tg, xg, np.gradient(xg, tg) if len(tg) > 1 else np.zeros_like(tg)
    vg = savgol_filter(xg, w, 2, deriv=1, delta=1.0 / fs)
    gap = np.diff(ts) > max_gap_s
    if gap.any():
        for i in np.flatnonzero(gap):
            vg[(tg > ts[i]) & (tg < ts[i + 1])] = np.nan
    return tg, xg, vg


def uniform(t: np.ndarray, y: np.ndarray, fs: float | None = None):
    """Resample a row signal (N,) or (N,k) onto a uniform grid, interpolating over NaNs and gaps."""
    if fs is None:
        fs = 1.0 / float(np.median(np.diff(t)))
    tg = np.arange(t[0], t[-1], 1.0 / fs)
    y = np.asarray(y, dtype=float)
    if y.ndim == 1:
        ok = np.isfinite(y)
        return tg, np.interp(tg, t[ok], y[ok]), fs
    out = np.empty((len(tg), y.shape[1]))
    for j in range(y.shape[1]):
        ok = np.isfinite(y[:, j])
        out[:, j] = np.interp(tg, t[ok], y[ok, j])
    return tg, out, fs


def lowpass(y: np.ndarray, fs: float, fc: float, order: int = 4) -> np.ndarray:
    if fc >= 0.45 * fs:
        return y
    b, a = butter(order, fc / (fs / 2))
    return filtfilt(b, a, y, axis=0)


def hold(y: np.ndarray) -> np.ndarray:
    """Forward-fill NaNs (the last known value); leading NaNs become 0."""
    y = np.asarray(y, dtype=float).copy()
    ok = np.isfinite(y)
    if not ok.any():
        return np.zeros_like(y)
    idx = np.where(ok, np.arange(len(y)), 0)
    np.maximum.accumulate(idx, out=idx)
    out = y[idx]
    out[: np.argmax(ok)] = 0.0
    return out


# event texts that can mark a command edge (the firmware's wording is free text)
EDGE_WORDS = ("step", "stop", "profile", "cmd", "duty", "guard", "limit", "abort")


@dataclass
class Plateau:
    prefix: str
    duty: float
    prev_duty: float
    t_on: float
    t_off: float
    i_on: int
    i_off: int
    edge_on: str  # "event" or "row"
    edge_off: str

    @property
    def from_rest(self) -> bool:
        return abs(self.prev_duty) < 1e-9

    @property
    def sign(self) -> int:
        return 1 if self.duty > 0 else -1

    @property
    def dur(self) -> float:
        return self.t_off - self.t_on


def excitation_source(run, prefix: str, source: str = "auto") -> str:
    """Which signal says when a channel was driven: cmd, duty or events.

    auto: the firmware's _cmd column if it is ever nonzero; else the applied duty the
    controller reports (_duty), which also catches excitation from outside the MIB
    (the controller's own USB); else step text in events and operator marks.
    """
    if source != "auto":
        return source
    cmd = run.col(f"{prefix}_cmd")
    if cmd is not None and np.nanmax(np.abs(np.nan_to_num(cmd)), initial=0) > 1e-6:
        return "cmd"
    if tgt_spans(run, prefix):
        return "tgt"  # closed-loop moves: the duty is the controller's, not an open-loop excitation
    duty = run.col(f"{prefix}_duty")
    if duty is not None and np.nanmax(np.abs(np.nan_to_num(duty)), initial=0) > 0.5:
        return "duty"
    return "events"


def plateaus(run, prefix: str, source: str = "auto", events=None, min_dur: float = 0.0) -> list[Plateau]:
    """Constant-excitation spans with nonzero duty, from the chosen source (see excitation_source).

    cmd:    the command column changes on the first row after the firmware applied it,
            so the row time is late by up to one period. A firmware event between the
            previous row and this one marks the exact time; without one the edge is put
            half a period before the row.
    duty:   edges are midway between the last CAN read showing the old duty and the
            first showing the new (uncertain by half the read interval). Readback is
            rounded to 0.5 % so noise does not split a plateau.
    events: 'step <node> <ch> <duty> [ms]' in a firmware event or an operator mark/note,
            ending after ms, or at the next 'stop' / 'step end' / step for that channel.
            Operator marks are only as exact as the operator's hand.
    """
    src = excitation_source(run, prefix, source)
    if src == "tgt":
        return []
    if src == "cmd":
        out = _plateaus_cmd(run, prefix, events)
    elif src == "duty":
        out = _plateaus_duty(run, prefix)
    else:
        out = _plateaus_events(run, prefix)
    return [p for p in out if p.dur >= min_dur]


def _plateaus_cmd(run, prefix, events):
    cmd = run.col(f"{prefix}_cmd")
    if cmd is None:
        return []
    c = hold(cmd)
    t = run.t
    if len(t) < 2:
        return []
    evts = run.fw_events() if events is None else events
    evts = [e for e in evts if any(k in e.text.lower() for k in EDGE_WORDS)]
    ev_t = np.array([e.t for e in evts]) if evts else np.array([])
    return _segments(prefix, t, c, lambda i: _edge(t, i, ev_t), float(t[-1]))


def _edge(t, i, ev_t):
    lo, hi = t[i - 1], t[i]
    if len(ev_t):
        m = (ev_t > lo - 1e-6) & (ev_t <= hi + 1e-6)
        if m.any():
            return float(ev_t[m][-1]), "event"
    return float(0.5 * (lo + hi)), "row"


def _plateaus_duty(run, prefix):
    ts, d = fresh(run, prefix, "duty")
    if len(ts) < 3:
        return []
    q = np.round(d * 2) / 2
    q[np.abs(q) < 0.5] = 0.0

    def edge(i):
        return float(0.5 * (ts[i - 1] + ts[i])), "duty"

    return _segments(prefix, ts, q, edge, float(run.t[-1]))


def _segments(prefix, t, c, edge, t_end):
    change = np.flatnonzero(np.abs(np.diff(c)) > 1e-6) + 1
    bounds = np.concatenate([[0], change, [len(c)]])
    out = []
    for k in range(len(bounds) - 1):
        i0, i1 = bounds[k], bounds[k + 1]
        duty = c[i0]
        if abs(duty) < 1e-9 or i0 == 0:
            continue
        t_on, src_on = edge(i0)
        t_off, src_off = edge(i1) if i1 < len(c) else (t_end, "end")
        out.append(Plateau(prefix, float(duty), float(c[i0 - 1]), t_on, t_off, int(i0), int(i1), src_on, src_off))
    return out


_STEP_RE = re.compile(r"\bstep\s+n?(\d+)[\s,]*c?(\d+)\s+(-?[\d.]+)(?:\s+(\d+))?", re.I)
_END_RE = re.compile(r"\b(stop|abort|guard|step end|off)\b", re.I)


def _plateaus_events(run, prefix):
    m = re.match(r"n(\d+)c(\d+)", prefix)
    node, ch = int(m.group(1)), int(m.group(2))
    starts, ends = [], []
    for e in run.events:
        if e.source == "host" and e.kind == "cmd" and any(abs(e.t - f.t) < 0.25 for f in run.fw_events()
                                                          if _STEP_RE.search(f.text)):
            continue  # the firmware echoed this command with its own exact time
        s = _STEP_RE.search(e.text)
        if s and int(s.group(1)) == node and int(s.group(2)) == ch:
            ms = float(s.group(4)) / 1000 if s.group(4) else None
            starts.append((e.t, float(s.group(3)), ms, "event" if e.source == "fw" else "mark"))
        elif _END_RE.search(e.text) and ("n%dc%d" % (node, ch) in e.text or not re.search(r"n\d+c\d+", e.text)):
            ends.append(e.t)
    starts.sort()
    out = []
    prev_end, prev_duty = -1.0, 0.0
    for k, (t0, duty, ms, src) in enumerate(starts):
        nxt = starts[k + 1][0] if k + 1 < len(starts) else float(run.t[-1])
        cands = [nxt] + [t for t in ends if t > t0] + ([t0 + ms] if ms else [])
        t1 = min(cands)
        pd = prev_duty if abs(t0 - prev_end) < 0.02 else 0.0
        i0 = int(np.searchsorted(run.t, t0))
        i1 = int(np.searchsorted(run.t, t1))
        if abs(duty) > 1e-9:
            out.append(Plateau(prefix, duty, pd, t0, t1, i0, i1, src, src))
        prev_end, prev_duty = t1, duty
    return out


@dataclass
class Span:
    prefix: str
    target: float
    t_on: float
    t_off: float
    i_on: int
    i_off: int
    edge_on: str


def tgt_spans(run, prefix: str) -> list[Span]:
    """Closed-loop moves: spans where <prefix>_tgt holds one value (empty = no move active).

    The edge is refined by a firmware event (pstep / target) between the rows, as for _cmd.
    """
    tg = run.col(f"{prefix}_tgt")
    if tg is None or not np.isfinite(tg).any():
        return []
    tg = _fill_short_gaps(tg, max_rows=10)
    t = run.t
    evts = [e for e in run.fw_events() if any(k in e.text.lower() for k in EDGE_WORDS + ("target", "pstep"))]
    ev_t = np.array([e.t for e in evts]) if evts else np.array([])
    key = np.where(np.isfinite(tg), tg, np.inf)
    change = np.flatnonzero(key[1:] != key[:-1]) + 1
    bounds = np.concatenate([[0], change, [len(tg)]])
    out = []
    for k in range(len(bounds) - 1):
        i0, i1 = bounds[k], bounds[k + 1]
        if not np.isfinite(tg[i0]):
            continue
        t_on, src = _edge(t, i0, ev_t) if i0 > 0 else (float(t[0]), "row")
        t_off = _edge(t, i1, ev_t)[0] if i1 < len(tg) else float(t[-1])
        out.append(Span(prefix, float(tg[i0]), t_on, t_off, int(i0), int(i1), src))
    return out


def _fill_short_gaps(y, max_rows=10):
    """Empty fields inside a move (a dropped CAN read, a garbled field) must not split it:
    fill NaN runs of up to max_rows when the same value is on both sides."""
    y = np.asarray(y, dtype=float).copy()
    nan = ~np.isfinite(y)
    if not nan.any():
        return y
    d = np.diff(np.concatenate([[0], nan.astype(int), [0]]))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    for a, b in zip(starts, ends):
        if 0 < a and b < len(y) and b - a <= max_rows and y[a - 1] == y[b]:
            y[a:b] = y[a - 1]
    return y


def has_tgt(run) -> bool:
    return any(tgt_spans(run, p) for p in run.channels())


def moves(run, settle_s: float = 1.0, source: str = "auto") -> list[tuple[float, float, list[str]]]:
    """Merged spans where any channel is commanded, (t_start, t_end + settle, prefixes)."""
    spans = []
    closed = source in ("auto", "tgt") and has_tgt(run)
    for p in run.channels():
        if closed:
            spans += [(sp.t_on, sp.t_off, p) for sp in tgt_spans(run, p)]
        else:
            spans += [(pl.t_on, pl.t_off, p) for pl in plateaus(run, p, source)]
    spans.sort()
    out = []
    for a, b, p in spans:
        if out and a <= out[-1][1] + 0.05:
            out[-1][1] = max(out[-1][1], b)
            if p not in out[-1][2]:
                out[-1][2].append(p)
        else:
            out.append([a, b, [p]])
    t_end = run.t[-1] if len(run.t) else 0
    return [(a, min(b + settle_s, t_end), ps) for a, b, ps in out]


def lag_scan(t_ref: np.ndarray, X: np.ndarray, t_y: np.ndarray, y: np.ndarray, lags_s: np.ndarray):
    """For each lag, least-squares fit y(t) ~ X(t - lag) and return R^2.

    X is sampled at t_ref (uniform), y at t_y. Positive lag means y happens after X.
    Returns (r2 array, best lag with parabolic refinement).
    """
    X = np.atleast_2d(X.T).T if X.ndim == 1 else X
    r2 = np.full(len(lags_s), np.nan)
    ok_y = np.isfinite(y)
    for k, lag in enumerate(lags_s):
        tq = t_y - lag
        m = ok_y & (tq >= t_ref[0]) & (tq <= t_ref[-1])
        if m.sum() < 20:
            continue
        A = np.column_stack([np.interp(tq[m], t_ref, X[:, j]) for j in range(X.shape[1])] + [np.ones(m.sum())])
        if not np.all(np.isfinite(A)):
            good = np.all(np.isfinite(A), axis=1)
            A, yy = A[good], y[m][good]
        else:
            yy = y[m]
        coef, *_ = np.linalg.lstsq(A, yy, rcond=None)
        res = yy - A @ coef
        ss = np.sum((yy - yy.mean()) ** 2)
        r2[k] = 1 - np.sum(res**2) / ss if ss > 0 else np.nan
    if np.all(np.isnan(r2)):
        return r2, np.nan
    k = int(np.nanargmax(r2))
    best = float(lags_s[k])
    if 0 < k < len(lags_s) - 1 and np.isfinite(r2[k - 1]) and np.isfinite(r2[k + 1]):
        a, b, c = r2[k - 1], r2[k], r2[k + 1]
        den = a - 2 * b + c
        if den < 0:
            best += 0.5 * (a - c) / den * (lags_s[1] - lags_s[0])
    return r2, best
