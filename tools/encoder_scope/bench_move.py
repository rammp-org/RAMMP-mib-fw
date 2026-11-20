"""mvt.py <delta> <speed> [label]: one relative move of the selected leg through the scope's console,
trace saved, one summary line printed. Flags trace steps >= 150 counts in one 20 ms sample."""
import sys, re, json, time, urllib.request, os
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "moves")
def mib(c):
    return urllib.request.urlopen(urllib.request.Request("http://localhost:8790/cmd", data=c.encode()), timeout=30).read().decode()
delta, speed = int(sys.argv[1]), int(sys.argv[2]); label = sys.argv[3] if len(sys.argv) > 3 else ""
r = mib(f"rel {delta} {speed}")
if "accepted" not in r:
    print("REFUSED/FAILED:", r.strip().splitlines()[-1]); sys.exit(1)
w = mib("trace")
rows = re.findall(r"at (\d+) (-?\d+)(?: imu ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+))?", w)
tr = [(int(r[0]), int(r[1])) for r in rows]
imu = [(int(r[0]), float(r[2]), float(r[3]), float(r[4]), float(r[5])) for r in rows if r[2]]
import math
def ang(g1, g2):
    d = sum(a*b for a, b in zip(g1, g2)) / (math.sqrt(sum(a*a for a in g1)) * math.sqrt(sum(b*b for b in g2)))
    return math.degrees(math.acos(max(-1, min(1, d))))
imu_txt = f"  tilt-rate peak {max(r[1] for r in imu):.1f} deg/s, tilt change {ang(imu[0][2:], imu[-1][2:]):.2f} deg" if len(imu) > 2 else "  (no imu)"
res = re.search(r"move #\d+: (\w+)\s+(-?\d+) -> (-?\d+) \(target\), ended at (-?\d+) \(([+-]?\d+) counts from target\), ([\d.]+) s, profile ([\d.]+) s, peak (\d+)", w)
reason = re.search(r"reason: ([^;\n]*)", w)
rel = re.search(r"after release: PWM ([-\d.]+) %, (-?\d+) mA", w)
t0 = tr[0][0] if tr else 0
jumps = [(t - t0, b - a) for (t, a), (_, b) in zip([(t, v) for t, v in tr], [(t, v) for t, v in tr[1:]]) if abs(b - a) >= 150]
# speed from the trace (counts/s over 100 ms)
sp = [abs(tr[i + 5][1] - tr[i][1]) / ((tr[i + 5][0] - tr[i][0]) / 1000) for i in range(len(tr) - 5) if tr[i + 5][0] > tr[i][0]]
name = time.strftime("%H%M%S") + f"_{label}{delta:+d}@{speed}"
json.dump({"delta": delta, "speed": speed, "trace": tr, "imu": imu, "raw": w}, open(os.path.join(D, name + ".json"), "w"))
if res:
    o, st, tg, end, err, el, prof, pk = res.groups()
    print(f"{delta:+6d} @{speed:4d}: {o:8s} {st}->{tg} end {err:>4s}  {el}s (profile {prof})  peak {pk}/s  trace-max {max(sp) if sp else 0:.0f}/s  "
          f"rel PWM {rel.group(1) if rel else '?'}% {rel.group(2) if rel else '?'}mA  boosts {(re.search(r"(\d+) boosted", w) or [0,"?"])[1]}  jumps {len(jumps)}" + imu_txt + (f"  REASON {reason.group(1)}" if reason else ""))
else:
    print("no result parsed:", w[-400:])
if "GUARD" in w: print("*** GUARD TRIPPED ***")
