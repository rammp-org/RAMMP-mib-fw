"""Figures for the actuator-control PR, from the 2026-11-17 bench logs. Alex style, dark.

Run from tools/actuator_sysid/host:  uv run python ../bench/make_figures.py
Writes PNGs to docs/actuator-revamp/figures and the plotted numbers to docs/actuator-revamp/data.
"""
import csv, math, os, statistics as st, sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.environ.get("ALEX_STYLE_DIR", os.path.expanduser("~/code-stuff/alex-style-guide/matplotlib")))
import alex_style as ax
import matplotlib.pyplot as plt
import benchlog

RUNS = os.environ.get("SYSID_RUNS", os.path.join(ROOT, "docs/actuator-revamp/runs"))  # raw logs are not in git
FIG = os.path.join(ROOT, "docs/actuator-revamp/figures")
DATA = os.path.join(ROOT, "docs/actuator-revamp/data")
os.makedirs(FIG, exist_ok=True); os.makedirs(DATA, exist_ok=True)
ax.use("dark")
C = ax.colors()
S = C["series"]

L5 = benchlog.load(f"{RUNS}/2026-11-17_05_motion_session.log")
L6 = benchlog.load(f"{RUNS}/2026-11-17_06_right_leg.log")
L7 = benchlog.load(f"{RUNS}/2026-11-17_07_small_fixes.log")
COLS = ["n3c1_enc", "n3c1_tgt", "n3c1_duty", "n3c1_cur_ma", "n3c1_status", "imu_gx", "imu_gy"]


def get(L, i, before=0.5, after=5.0):
    segs, moves = L
    m = moves[i]
    d = benchlog.series(segs[m.seg], m, COLS, before, after)
    d["duty_pct"] = [x / 327.67 for x in d["n3c1_duty"]]
    d["cur_a"] = [abs(x) / 1000 for x in d["n3c1_cur_ma"]]
    d["rate_dps"] = [max(abs(a), abs(b)) * 57.2958 for a, b in zip(d["imu_gx"], d["imu_gy"])]
    return m, d


def write_csv(name, header, rows):
    with open(os.path.join(DATA, name), "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)


def trapezoid(t, d, v, a):
    """Distance along a trapezoid profile of length d, speed v, accel a, at time t."""
    if v * v / a > d:
        v = math.sqrt(d * a)
    tr = v / a; tc = (d - v * v / a) / v
    if t <= 0: return 0.0
    if t < tr: return 0.5 * a * t * t
    if t < tr + tc: return 0.5 * v * tr + v * (t - tr)
    if t < 2 * tr + tc:
        r = 2 * tr + tc - t
        return d - 0.5 * a * r * r
    return d


def stream_start(d, start):
    """Time the streamed target first left the start count."""
    for t, g in zip(d["t"], d["n3c1_tgt"]):
        if not math.isnan(g) and abs(g - start) >= 2:
            return t - 0.02
    return 0.0


def peak_rate(d, t0=0.0, t1=4.0):
    return max(r for t, r in zip(d["t"], d["rate_dps"]) if t0 <= t <= t1)


# ---- Figure 1: stock step vs tuned streamed move ---------------------------------
m_a, a_ = get(L5, 0, 0.3, 4.5)            # stock gains, +400 step, window 100
m_b, b_ = get(L7, 8, 0.3, 4.5)            # tuned, +1000 at 800, lean handshake, lead 250 ms
ts = stream_start(b_, m_b.start)
prof = [m_b.start + trapezoid(t - ts, 1000, 800, 1600) for t in b_["t"]]
fig, (p1, p2) = plt.subplots(2, 1, figsize=(9, 7.4), constrained_layout=True)
p1.plot(a_["t"], [m_a.target if 0 <= t else m_a.start for t in a_["t"]], color=S[1], label="Target")
p1.plot(a_["t"], a_["n3c1_enc"], color=S[0], label="Encoder")
p1.set_xlabel("Time from command, s"); p1.set_ylabel("Right leg position, counts")
ax.label_lines(p1)
ax.title(p1, "Before: stock controller settings, 400-count step",
         "Jumps 170 counts in 0.15 s, then stalls about 195 counts short and holds 3 A")
p2.plot(b_["t"], prof, color=S[1], label="Profile")
p2.plot(b_["t"], b_["n3c1_enc"], color=S[0], label="Encoder")
p2.set_xlabel("Time from command, s"); p2.set_ylabel("Right leg position, counts")
ax.label_lines(p2)
ax.title(p2, "After: tuned settings, 1,000 counts at 800 counts/s",
         "MIB streams a trapezoid at 50 Hz; ends within 13 counts of target")
ax.save(fig, f"{FIG}/01_before_after.png", dpi=160)
write_csv("01_before_stock_step.csv", ["t_s", "target_counts", "encoder_counts", "pwm_pct", "current_a"],
          [(f"{t:.3f}", m_a.target if t >= 0 else m_a.start, e, f"{p:.2f}", f"{c:.2f}")
           for t, e, p, c in zip(a_["t"], a_["n3c1_enc"], a_["duty_pct"], a_["cur_a"])])
write_csv("01_after_tuned_stream.csv", ["t_s", "profile_counts", "written_target_counts", "encoder_counts", "pwm_pct", "current_a"],
          [(f"{t:.3f}", f"{q:.0f}", g, e, f"{p:.2f}", f"{c:.2f}")
           for t, q, g, e, p, c in zip(b_["t"], prof, b_["n3c1_tgt"], b_["n3c1_enc"], b_["duty_pct"], b_["cur_a"])])

# ---- Figure 2: peak chassis rate by method --------------------------------------
_, s_p = get(L6, 1, 0.3, 4.0)             # P 0.2, no D, -300 step
_, s_1 = get(L6, 49, 0.3, 4.0)            # tuned, one target, speed cap 800, -1000
tuned = [peak_rate(get(L7, i, 0.3, 4.5)[1]) for i in (7, 8, 9, 10)]
labels = ["Stock gains, 400-count step", "P 0.2, D 0, 300-count step",
          "Tuned gains, one 1,000-count target", "Tuned gains, streamed 1,000 counts at 800/s"]
vals = [round(peak_rate(a_), 1), round(peak_rate(s_p), 1), round(peak_rate(s_1), 1), round(st.mean(tuned), 1)]
fig, p = plt.subplots(figsize=(9, 3.4), constrained_layout=True)
ax.bars(p, labels, vals, highlight=labels[3], unit=" °/s", xlabel="Peak chassis rotation rate, °/s", ylabel="Move")
ax.title(p, "How hard each kind of move shakes the chair",
         "IMU gyro, largest axis. Red is the method the firmware now uses (mean of 4 moves, 2.4 to 3.8)")
ax.save(fig, f"{FIG}/02_chassis_rate.png", dpi=160)
write_csv("02_chassis_rate.csv", ["move", "peak_chassis_rate_deg_per_s"], list(zip(labels, vals)) +
          [(f"tuned streamed move {i + 1}", f"{v:.1f}") for i, v in enumerate(tuned)])

# ---- Figure 3: the position window ----------------------------------------------
m_w100, w100 = get(L6, 15, 0.3, 5.0)      # window 100, +300 at 100 counts/s
m_w10, w10 = get(L6, 17, 0.3, 5.0)        # window 10, same gains, same ramp
fig, p = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
for d, col, lab in ((w100, S[1], "Window 100 (stock)"), (w10, S[0], "Window 10")):
    p.plot(d["t"], [g - e for g, e in zip(d["n3c1_tgt"], d["n3c1_enc"])], color=col, label=lab)
p.set_xlabel("Time from command, s"); p.set_ylabel("Target minus encoder, counts")
ax.label_lines(p)
ax.title(p, "The controller stops driving inside its position window",
         "Same gains, same 300-count ramp at 100 counts/s (ends at 3 s). Only object 0x6067 differs")
ax.save(fig, f"{FIG}/03_position_window.png", dpi=160)
write_csv("03_position_window.csv", ["t_s", "error_window100_counts", "t2_s", "error_window10_counts"],
          [(f"{t:.3f}", f"{g - e:.0f}", f"{t2:.3f}", f"{g2 - e2:.0f}") for (t, g, e), (t2, g2, e2) in
           zip(zip(w100["t"], w100["n3c1_tgt"], w100["n3c1_enc"]), zip(w10["t"], w10["n3c1_tgt"], w10["n3c1_enc"]))])

# ---- Figures 4 and 5: across the travel -----------------------------------------
segs6, moves6 = L6
rows4, tilt = [], []
for i in range(28, 49):
    m = moves6[i]; seg = segs6[m.seg]
    import re
    e = re.search(r"final (-?\d+).*peak \|current\| (\d+) mA", m.end)
    rate = int(re.search(r"walks at (\d+)", m.stream)[1])
    final = int(e[1]); cur = int(e[2]) / 1000
    d = benchlog.series(seg, m, COLS, 0.2, 4.5)
    d["rate_dps"] = [max(abs(a), abs(b)) * 57.2958 for a, b in zip(d["imu_gx"], d["imu_gy"])]
    up = m.target > m.start
    rows4.append(((m.start + m.target) / 2, "up" if up else "down", rate, abs(m.target - final), cur, peak_rate(d)))
    # tilt at rest just before the next move
    nxt = moves6[i + 1].t0_us if i + 1 < len(moves6) else m.t0_us + 7_000_000
    ix = {k: j for j, k in enumerate(seg.cols)}
    rest = [r for r in seg.rows if nxt - 1_500_000 <= int(r[0]) <= nxt - 300_000]
    if len(rest) > 20:
        gx = st.mean(float(r[ix["imu_grx"]]) for r in rest); gy = st.mean(float(r[ix["imu_gry"]]) for r in rest)
        gz = st.mean(float(r[ix["imu_grz"]]) for r in rest)
        enc = st.median(float(r[ix["n3c1_enc"]]) for r in rest if r[ix["n3c1_enc"]] != "")
        tilt.append((enc, math.degrees(math.atan2(gx, gz)), math.degrees(math.atan2(gy, gz))))
fig, (p1, p2) = plt.subplots(2, 1, figsize=(9, 6.6), constrained_layout=True)
for direction, col in (("up", S[0]), ("down", S[1])):
    pts = sorted((r for r in rows4 if r[1] == direction and r[2] == 800))
    p1.plot([r[0] for r in pts], [r[3] for r in pts], color=col, marker="s", markersize=5,
            label=f"{direction.capitalize()}")
    p2.plot([r[0] for r in pts], [r[4] for r in pts], color=col, marker="s", markersize=5,
            label=f"{direction.capitalize()}")
p1.set_xlabel("Middle of the move, counts"); p1.set_ylabel("Distance from target at the end, counts")
ax.ref_line(p1, 30, "encoder noise, about ±30 counts", "warning")
p1.set_ylim(0, 45); ax.label_lines(p1)
ax.title(p1, "One set of gains across the right leg's travel",
         "1,000-count moves at 800 counts/s between 10,530 and 16,630 (stored range 10,000 to 18,000)")
p2.set_xlabel("Middle of the move, counts"); p2.set_ylabel("Peak motor current, A")
p2.set_ylim(0, 18); ax.label_lines(p2)
ax.title(p2, "Peak current in the same moves", "Up (+counts) lifts that side of the chair; down is load-assisted")
ax.save(fig, f"{FIG}/04_full_range.png", dpi=160)
write_csv("04_full_range.csv", ["mid_position_counts", "direction", "speed_counts_per_s", "end_error_counts",
                                "peak_current_a", "peak_chassis_rate_deg_per_s"],
          [(f"{r[0]:.0f}", r[1], r[2], r[3], f"{r[4]:.1f}", f"{r[5]:.1f}") for r in sorted(rows4)])

tilt.sort()
fig, p = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
p.plot([t[0] for t in tilt], [t[2] for t in tilt], color=S[0], marker="s", markersize=4, label="Board Y")
p.plot([t[0] for t in tilt], [t[1] for t in tilt], color=S[1], marker="s", markersize=4, label="Board X")
p.set_xlabel("Right leg position, counts"); p.set_ylabel("Chassis tilt at rest, °")
ax.label_lines(p)
ax.title(p, "Chassis tilt against right-leg position",
         "BNO055 gravity vector after each move, left leg fixed. About 2.0° per 1,000 counts on board Y")
ax.save(fig, f"{FIG}/05_tilt_map.png", dpi=160)
write_csv("05_tilt_map.csv", ["right_leg_counts", "tilt_board_x_deg", "tilt_board_y_deg"],
          [(f"{t[0]:.0f}", f"{t[1]:.2f}", f"{t[2]:.2f}") for t in tilt])

# ---- Figure 6: heartbeat watchdog ------------------------------------------------
segs7, moves7 = L7
m = moves7[3]; seg = segs7[m.seg]
hb = next(t for t, x in seg.events if "heartbeat STOPPED (scheduled)" in x and t > m.t0_us)
fault = next(t for t, x in seg.events if "n3c1 fault" in x and t > hb)
ix = {k: j for j, k in enumerate(seg.cols)}
w = [r for r in seg.rows if hb - 2_500_000 <= int(r[0]) <= hb + 2_000_000 and r[ix["n3c1_enc"]] != ""]
tt = [(int(r[0]) - hb) / 1e6 for r in w]; en = [float(r[ix["n3c1_enc"]]) for r in w]
pw = [float(r[ix["n3c1_duty"]] or 0) / 327.67 for r in w]
fig, (p1, p2) = plt.subplots(2, 1, figsize=(9, 6.2), constrained_layout=True, sharex=True)
p1.plot(tt, en, color=S[0], label="Encoder")
p1.set_xlabel("Time from the MIB heartbeat stopping, s"); p1.set_ylabel("Right leg position, counts")
p2.plot(tt, pw, color=S[0], label="PWM")
p2.set_xlabel("Time from the MIB heartbeat stopping, s"); p2.set_ylabel("Motor PWM, %")
for p in (p1, p2):
    p.axvline(0, color=C["warning"], linestyle=(0, (4, 3)), linewidth=1)
    p.axvline((fault - hb) / 1e6, color=C["critical"], linestyle=(0, (4, 3)), linewidth=1)
p1.annotate("heartbeat stops", xy=(0, 0), xycoords=("data", "axes fraction"), xytext=(-4, 8),
            textcoords="offset points", ha="right", color=C["muted"], fontsize=8)
p1.annotate(f"drive faults, {(fault - hb) / 1000:.0f} ms later", xy=((fault - hb) / 1e6, 0),
            xycoords=("data", "axes fraction"), xytext=(4, 8), textcoords="offset points", color=C["muted"], fontsize=8)
ax.title(p1, "The controller's heartbeat watchdog stops a move",
         "200 counts/s move; MIB heartbeat stopped on purpose mid-move. Consumer timeout 500 ms")
ax.title(p2, "Motor PWM in the same event", "After the fault the controller holds position at a few percent for about 11 s, then powers off")
ax.save(fig, f"{FIG}/06_watchdog.png", dpi=160)
write_csv("06_watchdog.csv", ["t_from_heartbeat_stop_s", "encoder_counts", "pwm_pct"],
          [(f"{t:.3f}", e, f"{p:.2f}") for t, e, p in zip(tt, en, pw)] )

# ---- Figure 7: target lead -------------------------------------------------------
m_n, n_ = get(L6, 27, 0.3, 4.0)           # P 4.0, no lead, +1000 at 800
m_l, l_ = get(L7, 8, 0.3, 4.0)            # P 4.0, lead 250 ms, +1000 at 800
fig, p = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
out = []
for m, d, col, lab in ((m_n, n_, S[1], "No lead"), (m_l, l_, S[0], "Lead 250 ms")):
    t0 = stream_start(d, m.start)
    err = [m.start + trapezoid(t - t0, 1000, 800, 1600) - e for t, e in zip(d["t"], d["n3c1_enc"])]
    p.plot([t - t0 for t in d["t"]], err, color=col, label=lab)
    out.append([(f"{t - t0:.3f}", f"{x:.0f}") for t, x in zip(d["t"], err)])
p.set_xlim(-0.2, 3.6)
p.set_xlabel("Time from the start of the profile, s"); p.set_ylabel("Profile minus encoder, counts")
ax.label_lines(p)
ax.title(p, "Writing the target ahead of the profile removes the lag",
         "1,000 counts up at 800 counts/s, position P 4.0. Lead = profile speed x 250 ms (1 / P)")
ax.save(fig, f"{FIG}/07_target_lead.png", dpi=160)
write_csv("07_target_lead.csv", ["t_no_lead_s", "lag_no_lead_counts", "t_lead_s", "lag_lead_counts"],
          [a + b for a, b in zip(out[0], out[1])])
print("figures:", sorted(os.listdir(FIG)))
print("fig2 values:", list(zip(labels, vals)), "tuned each:", [round(v, 1) for v in tuned])
print("fig4 rows:", len(rows4), "tilt points:", len(tilt), "fault ms:", (fault - hb) / 1000)

# ---- Figure 8: what the passive logger sees at rest (both main legs) -------------
L2 = benchlog.load(f"{RUNS}/2026-11-17_02_passive_dump.log")
seg = L2[0][0]
ix = {k: j for j, k in enumerate(seg.cols)}
t0 = int(seg.rows[0][0])
pts = [((int(r[0]) - t0) / 1e6, r[ix["n3c1_enc"]], r[ix["n3c2_enc"]]) for r in seg.rows]
pts = [(t, float(a), float(b)) for t, a, b in pts if a != "" and b != ""]
ra = st.median(p[1] for p in pts); rb = st.median(p[2] for p in pts)
fig, p = plt.subplots(figsize=(9, 4.2), constrained_layout=True)
p.plot([q[0] for q in pts], [q[2] - rb for q in pts], color=S[1], label="Left leg (n3 M2)")
p.plot([q[0] for q in pts], [q[1] - ra for q in pts], color=S[0], label="Right leg (n3 M1)")
p.set_xlabel("Time since boot, s"); p.set_ylabel("Encoder count minus its median, counts")
ax.label_lines(p)
ax.title(p, "Main-leg encoders at rest, drives off",
         "Passive logger, 100 Hz. Right leg noise is about ±30 counts; the left leg glitches by about 1,900 counts")
ax.save(fig, f"{FIG}/08_encoders_at_rest.png", dpi=160)
write_csv("08_encoders_at_rest.csv", ["t_s", "right_leg_minus_median_counts", "left_leg_minus_median_counts"],
          [(f"{t:.3f}", f"{a - ra:.0f}", f"{b - rb:.0f}") for t, a, b in pts])
