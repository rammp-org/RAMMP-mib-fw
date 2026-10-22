import sys, re, json, math, statistics as st, collections
sys.path.insert(0, "/Users/alxchunlin/code-stuff/alex-style-guide/matplotlib")
import alex_style as ax
import matplotlib.pyplot as plt
P = ax.use("dark"); S = P["series"]
OUT = "/tmp/imu_cap/plots/"
DROPPED = [0]
ANSI = re.compile(r"\x1b\[[0-9;]*m")
D = math.degrees

def swatches(a):
    """Square-swatch legend above the plot, per the style guide."""
    from matplotlib.lines import Line2D
    ls = [l for l in a.get_lines() if not l.get_label().startswith("_")]
    hs = [Line2D([], [], marker="s", ls="", markersize=7, color=l.get_color()) for l in ls]
    a.legend(hs, [l.get_label() for l in ls], loc="lower right", bbox_to_anchor=(1, 1.0), ncol=len(ls),
             frameon=False, fontsize=8, handletextpad=0.3, columnspacing=1.2, borderaxespad=0.2)

def load_serial(path):
    rows, logs = [], []
    for l in open(path):
        ts, _, line = l.partition("\t"); line = ANSI.sub("", line).strip()
        if line.startswith("D,"):
            f = line.split(",")
            if len(f) not in (4, 14) or not all(re.fullmatch(r"-?\d+", x) for x in f[1:]):
                DROPPED[0] += 1; continue
            rows.append(dict(host=float(ts), t=int(f[1]) / 1e6, read=int(f[2]), ok=f[3] == "1",
                             **({} if f[3] != "1" else dict(roll=int(f[4]) / 1000, pitch=int(f[5]) / 1000,
                                gz=int(f[9]) / 1000, ax=int(f[10]) / 100, ay=int(f[11]) / 100, az=int(f[12]) / 100))))
        elif line:
            logs.append((float(ts), line))
    return rows, logs

def load_rtps(path):
    return [json.loads(l) for l in open(path)]

s1, l1 = load_serial("/tmp/imu_cap/session_mib.txt");  r1 = load_rtps("/tmp/imu_cap/session_rtps.jsonl")
s2, l2 = load_serial("/tmp/imu_cap/session2_mib.txt"); r2 = load_rtps("/tmp/imu_cap/session2_rtps.jsonl")
s3, l3 = load_serial("/tmp/imu_cap/session3_mib.txt")
stats = {}

# ---------- Figure 1: tilt test and fusion vs gravity
T0 = s2[0]["host"]
ok2 = [r for r in s2 if r["ok"]]
bad2 = [r for r in s2 if not r["ok"]]
fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.0), gridspec_kw={"width_ratios": [1.5, 1]})
# break the line across the unplugged gap
def gapped(rows, key, conv=D, tkey="host", t0=T0, gap=0.2):
    xs, ys, prev = [], [], None
    for r in rows:
        x = r[tkey] - t0
        if prev is not None and x - prev > gap: xs.append(float("nan")); ys.append(float("nan"))
        xs.append(x); ys.append(conv(r[key])); prev = x
    return xs, ys
w80 = [r for r in ok2 if r["host"] - T0 <= 80]
a.plot(*gapped(w80, "roll"), color=S[0], label="Roll")
a.plot(*gapped(w80, "pitch"), color=S[1], label="Pitch")
un0, un1 = bad2[0]["host"] - T0, bad2[-1]["host"] - T0
rec = next(r["host"] - T0 for r in ok2 if r["host"] - T0 > un1)
a.axvspan(un0, rec, color=P["muted"], alpha=0.12, lw=0)
a.annotate("IMU unplugged", xy=((un0 + rec) / 2, 0.97), xycoords=("data", "axes fraction"), ha="center", va="top",
           color=P["muted"], fontsize=8)
a.set_xlim(0, 80)
a.set_xlabel("Time, s"); a.set_ylabel("Angle, deg")
ax.label_lines(a); swatches(a)
ax.title(a, "Fused tilt on the MIB", "BNO055 IMU mode, 100 Hz serial log; tilted by hand, then unplugged and replugged")
stats["pitch_max"] = max(D(r["pitch"]) for r in ok2); stats["roll_min"] = min(D(r["roll"]) for r in ok2)
stats["unplug"] = (un0, un1, rec)
# fusion vs gravity during the roll test
seg = [r for r in ok2 if 28 <= r["host"] - T0 <= 52]
xs = [r["host"] - T0 for r in seg]
b.plot(xs, [D(r["roll"]) for r in seg], color=S[0], lw=4, label="Fused roll")
b.plot(xs, [D(math.atan2(r["ax"], math.hypot(r["ay"], r["az"]))) for r in seg], color=S[2], label="Gravity roll")
b.set_xlabel("Time, s"); b.set_ylabel("Roll, deg")
ax.label_lines(b); swatches(b)
ax.title(b, "Fusion against gravity", "Gravity roll is atan2(a_x, |a_yz|)")
fig.tight_layout(); ax.save(fig, OUT + "imu_tilt.png", dpi=160)
# agreement stats (static-ish samples only: low gyro)
still = [r for r in seg if abs(r["gz"]) < 0.05]
diffs = [D(r["roll"]) - D(math.atan2(r["ax"], math.hypot(r["ay"], r["az"]))) for r in seg]
stats["roll_agree_still_median_abs"] = st.median(abs(D(r["roll"]) - D(math.atan2(r["ax"], math.hypot(r["ay"], r["az"])))) for r in still)
stats["roll_agree_median_abs"] = st.median(abs(d) for d in diffs)

# ---------- Figure 2: RTPS carries the same tilt + topic rates
fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.0), gridspec_kw={"width_ratios": [1.5, 1]})
diag2 = [r for r in r2 if r["topic"] == "rammp/mcb/diagnostics"]
a.plot(*gapped([r for r in ok2 if r["host"] - T0 <= 80], "pitch"), color=S[1], label="Pitch, MIB serial")
rx = [r["t"] - T0 for r in diag2 if r["items"][3] and r["t"] - T0 <= 80]
ry = [r["items"][3][1] / 10 for r in diag2 if r["items"][3] and r["t"] - T0 <= 80]
a.plot(rx, ry, color=S[3], marker="o", markersize=3.5, lw=0, label="Pitch, RTPS on Mac")
a.set_xlim(0, 80)
a.set_xlabel("Time, s"); a.set_ylabel("Pitch, deg")
ax.label_lines(a); swatches(a)
ax.title(a, "Diagnostics row I1 over RTPS", "rammp/mcb/diagnostics received on the Mac (2 Hz) vs the MIB's 100 Hz log")
empty = sum(1 for r in diag2 if not r["items"][3])
stats["diag_empty_during_unplug"] = empty
# topic rates from session 1 (all quiet, 174 s)
dur = r1[-1]["t"] - r1[0]["t"]
names = [("rammp/mib/motor_command/drive_left", "motor_command/drive_left"),
         ("rammp/mib/motor_command/drive_right", "motor_command/drive_right"),
         ("rammp/mcb/diagnostics", "mcb/diagnostics"), ("rammp/mib/status", "mib/status")]
cnt = collections.Counter(r["topic"] for r in r1)
rates = [round(cnt[t] / dur, 1) for t, _ in names]
ax.bars(b, [n for _, n in names], rates, unit=" Hz", xlabel="Received rate, Hz", ylabel="Topic")
ax.title(b, "MIB topics seen on the Mac", f"{int(dur)} s capture, samples received / time")
fig.tight_layout(); ax.save(fig, OUT + "rtps.png", dpi=160)
stats["rates"] = dict(zip([n for _, n in names], rates)); stats["rtps_dur"] = dur

# ---------- Figure 3: stand-in joystick drive test
xy = [r for r in r1 if r["topic"] == "sent/xy_twist"]
dc = [r for r in r1 if r["topic"] == "sent/drive_command"]
ml = [r for r in r1 if r["topic"].endswith("drive_left")]
mr = [r for r in r1 if r["topic"].endswith("drive_right")]
t0 = dc[0]["t"] - 3
win = lambda rs: [r for r in rs if t0 <= r["t"] <= dc[-1]["t"] + 4]
fig, (a, b) = plt.subplots(2, 1, figsize=(12, 5.6), sharex=True)
a.plot([r["t"] - t0 for r in win(xy)], [r["y"] for r in win(xy)], color=S[0], label="Stick y")
a.plot([r["t"] - t0 for r in win(xy)], [r["x"] for r in win(xy)], color=S[1], label="Stick x")
a.set_ylabel("Stick deflection, -1 to 1"); a.set_xlabel("Time, s")
swatches(a)
ax.title(a, "Stand-in joystick to MIB", "XYTwist sent from the Mac at 20 Hz after a DriveCommand ENABLE")
b.plot([r["t"] - t0 for r in win(ml)], [r["velocity"] for r in win(ml)], color=S[2], label="Left wheel")
b.plot([r["t"] - t0 for r in win(mr)], [r["velocity"] for r in win(mr)], color=S[3], label="Right wheel")
armed = [r["t"] - t0 for r in win(ml) if r["state"] == 1]
b.axvspan(min(armed), max(armed), color=P["good"], alpha=0.08, lw=0)
b.annotate("ARMED", xy=(min(armed) + 0.2, 0.95), xycoords=("data", "axes fraction"), va="top", color=P["muted"], fontsize=8)
b.set_xlabel("Time, s"); b.set_ylabel("Commanded velocity, rad/s")
swatches(b)
ax.title(b, "MIB motor commands received on the Mac", "rammp/mib/motor_command/drive_left and drive_right; the right wheel is negated by invert_right")
fig.tight_layout(); ax.save(fig, OUT + "drive_test.png", dpi=160)
stats["armed_s"] = (min(armed), max(armed)); stats["vel_range"] = (min(r["velocity"] for r in ml), max(r["velocity"] for r in ml))

# ---------- Figure 4: read loop timing + unplug log volume
fig, (a, b) = plt.subplots(1, 2, figsize=(12, 3.8), gridspec_kw={"width_ratios": [1.5, 1]})
ts = [r["t"] for r in s1][100:]
per = [(y - x) * 1000 for x, y in zip(ts, ts[1:])]
bins = [i * 0.5 for i in range(0, 61)]
a.hist(per, bins=bins, color=S[0], rwidth=0.9)
a.set_yscale("log")
a.set_xlabel("Time between IMU reads, ms"); a.set_ylabel("Reads (log scale)")
ax.title(a, "IMU read period", f"{len(per)} reads over {int(ts[-1]-ts[0])} s, board still; capture build prints every read")
stats["period_mean"] = st.mean(per); stats["period_med"] = st.median(per); stats["rate"] = 1000 / st.mean(per)
stats["over15"] = sum(p > 15 for p in per) / len(per); stats["read_med_us"] = st.median(r["read"] for r in s1 if r["ok"])
def err_rate(rows, logs):
    bad = [r["host"] for r in rows if not r["ok"]]
    lo, hi = bad[0] - 0.5, bad[-1] + 1.5
    n = sum(1 for t, l in logs if lo <= t <= hi and "/E]" in l)
    return n, hi - lo
n2, d2 = err_rate(s2, l2); n3, d3 = err_rate(s3, l3)
stats["err"] = (n2, d2, n3, d3)
ax.bars(b, ["Re-init every retry", "Probe, then re-init"], [round(n2 / d2, 1), round(n3 / d3, 1)],
        highlight="Re-init every retry", unit="/s", xlabel="Error log lines per second", ylabel="Recovery")
ax.title(b, "Console noise while unplugged", "Over each unplugged window. Red: the first version, before the fix")
fig.tight_layout(); ax.save(fig, OUT + "timing_recovery.png", dpi=160)

stats["dropped_lines"] = DROPPED[0]
for k, v in stats.items(): print(k, v)
