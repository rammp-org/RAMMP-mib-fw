"""stepsum.py [log] : summarize the last pstep in the session log (node 3, channel from the event)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, re, math, statistics as st, os
LOG = sys.argv[1] if len(sys.argv) > 1 else benchenv.log()
L = open(LOG, "rb").read().decode("utf-8", "replace").splitlines()
# last boot segment
b = max([i for i, l in enumerate(L) if "#evt," in l and "boot:" in l] or [0]); L = L[b:]
ev = [l for l in L if l.startswith("#evt,")]
ps = [l for l in ev if re.search(r"pstep n3c\d from", l)]
if not ps: print("no pstep"); sys.exit()
m = re.search(r"#evt,(\d+),pstep n3c(\d) from (-?\d+) to (-?\d+)", ps[-1]); T0 = int(m[1]); ch = m[2]; start = int(m[3]); tgt = int(m[4])
cols = [l for l in L if l.startswith("#cols:")][-1][6:].split(","); ix = {k: i for i, k in enumerate(cols)}
rows = [l[2:].split(",") for l in L if l.startswith("D,")]
rows = [r for r in rows if len(r) == len(cols) and r[0].isdigit() and T0 - 1.0e6 <= int(r[0]) <= T0 + 7.5e6]
P = f"n3c{ch}_"
def col(k): return [(float(r[ix[k]]) if r[ix[k]] != "" else float("nan")) for r in rows]
t = [(int(r[0]) - T0) / 1e6 for r in rows]; enc = col(P + "enc"); duty = [d / 327.67 for d in col(P + "duty")]; cur = [c / 1000 for c in col(P + "cur_ma")]
for l in ev:
    tt = int(l.split(",")[1])
    if T0 - 3.0e6 <= tt <= T0 + 8e6 and any(k in l for k in ("gains n", "pstep", "guard", "stop sequence done", "fault", "refused", "limit")): print(l[:330])
sgn = 1 if tgt > start else -1
# end of drive: last sample with nonzero duty
on = [i for i, d in enumerate(duty) if t[i] > 0 and abs(d) > 0.01]
if not on: print("NO OUTPUT: duty stayed 0"); sys.exit()
i0, i1 = on[0], on[-1]
med = lambda a: st.median(x for x in a if not math.isnan(x))
pre = med([e for e, tt in zip(enc, t) if tt < 0]); fin = med(enc[max(i0, i1 - 30):i1 + 1]); after = med([e for e, tt in zip(enc, t) if tt > t[i1] + 0.6] or [float('nan')])
ext = max(enc[i0:i1 + 1]) if sgn > 0 else min(enc[i0:i1 + 1])
print(f"ch{ch} start {pre:.0f} target {tgt} | at end of drive {fin:.0f} (err {tgt-fin:+.0f}) | after release {after:.0f} | extreme {ext:.0f} (overshoot {sgn*(ext-tgt):+.0f}) | drive on {t[i0]:.2f}..{t[i1]:.2f} s")
# time to reach 50% / 90% of the step actually achieved
ach = fin - pre
def tcross(fr):
    for i in range(i0, i1 + 1):
        if sgn * (enc[i] - pre) >= fr * abs(ach): return t[i] - t[i0]
    return float("nan")
print(f"moved {ach:+.0f} counts; t50 {tcross(0.5):.2f} s, t90 {tcross(0.9):.2f} s after drive on; peak |duty| {max(abs(d) for d in duty[i0:i1+1]):.1f} %, peak |cur| {max(abs(c) for c in cur[i0:i1+1]):.1f} A")
late = range(max(i0, i1 - 100), i1 + 1)
print(f"last 1 s of drive: duty {st.mean(duty[i] for i in late):+.2f} % (sd {st.pstdev([duty[i] for i in late]):.2f}), cur {st.mean(abs(cur[i]) for i in late):.2f} A, enc p-p {max(enc[i] for i in late)-min(enc[i] for i in late):.0f}")
# speed vs duty in 100 ms bins
print(" t     enc    err  duty%  cur_A  speed(c/s)")
k = i0
while k + 10 <= i1 + 1 and t[k] - t[i0] < 4.9:
    seg = range(k, k + 10); e0 = st.mean(enc[k:k + 3]); e1 = st.mean(enc[k + 7:k + 10]); v = (e1 - e0) / 0.07
    if t[k] - t[i0] < 1.2 or int(round((t[k] - t[i0]) * 10)) % 5 == 0:
        print(f"{t[k]-t[i0]:4.1f} {st.mean(enc[i] for i in seg):7.0f} {tgt-st.mean(enc[i] for i in seg):+6.0f} {st.mean(duty[i] for i in seg):+6.1f} {st.mean(abs(cur[i]) for i in seg):6.2f} {v:+8.0f}")
    k += 10
# cruise: between 20% and 80% of the travel achieved
try:
    ia = next(i for i in range(i0, i1 + 1) if sgn * (enc[i] - pre) >= 0.2 * abs(ach)); ib = next(i for i in range(i0, i1 + 1) if sgn * (enc[i] - pre) >= 0.8 * abs(ach))
    if ib - ia >= 5:
        n = ib - ia + 1; tm = sum(t[ia:ib+1]) / n; em = sum(enc[ia:ib+1]) / n
        slope = sum((t[i]-tm)*(enc[i]-em) for i in range(ia, ib+1)) / sum((t[i]-tm)**2 for i in range(ia, ib+1))
        dd = duty[ia:ib+1]; cc = [abs(c) for c in cur[ia:ib+1]]
        print(f"CRUISE 20-80%: {t[ib]-t[ia]:.2f} s, speed {slope:+.0f} counts/s, duty {st.mean(dd):+.2f} % (sd {st.pstdev(dd):.2f}), current {st.mean(cc):.2f} A")
    else: print(f"CRUISE: too short ({ib-ia} samples)")
except StopIteration: print("CRUISE: n/a")
gx = col("imu_gx"); gy = col("imu_gy")
print(f"peak gyro during drive: x {max(abs(g) for g in gx[i0:i1+1])*57.3:.1f} deg/s, y {max(abs(g) for g in gy[i0:i1+1])*57.3:.1f} deg/s")
