"""walk.py "delta rate accel" ... : run streamed moves on node 3 M1 one after another; abort on anything odd."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, time, re, os, math, statistics as st
C=benchenv.CMDS; LOG=benchenv.log()
MAXERR=int(os.environ.get("MAXERR","70"))
def say(c): open(C,"a").write(c+"\n")
def tail(pos):
    with open(LOG,"rb") as f:
        f.seek(pos); return f.read().decode("utf-8","replace").splitlines()
def rest_tilt(lines, cols):
    ix={k:i for i,k in enumerate(cols)}; rows=[l[2:].split(",") for l in lines if l.startswith("D,")]
    rows=[r for r in rows if len(r)==len(cols)][-40:]
    if len(rows)<20: return None
    g=[st.mean(float(r[ix[k]]) for r in rows) for k in ("imu_grx","imu_gry","imu_grz")]
    return math.degrees(math.atan2(g[0],g[2])), math.degrees(math.atan2(g[1],g[2])), st.median(float(r[ix["n3c1_enc"]]) for r in rows if r[ix["n3c1_enc"]]!="")
cols=None
for l in open(LOG,"rb").read().decode("utf-8","replace").splitlines():
    if l.startswith("#cols:"): cols=l[6:].split(",")
print("move            start  target  final   err  peakPWM% peakA  cruise c/s  lag  gyro deg/s | tilt_x  tilt_y at rest")
for spec in sys.argv[1:]:
    delta, rate, accel = (int(x) for x in spec.split())
    pos=os.path.getsize(LOG); say("arm 3 1"); time.sleep(0.7)
    ev=[l for l in tail(pos) if "armed n3c1" in l]
    m=re.search(r"present (-?\d+)", ev[-1]) if ev else None
    if not m: print("ABORT: arm gave no present count:", ev[-1][:200] if ev else "no event"); break
    p=int(m[1]); t=p+delta; lo=min(p,t)-150; hi=max(p,t)+150
    pos=os.path.getsize(LOG); say(f"clamp 3 1 {lo} {hi}"); time.sleep(1.0)
    ce=[l for l in tail(pos) if "clamp n3c1" in l or "clamp refused" in l]
    if not ce or "installed" not in ce[-1] or "unchanged" not in ce[-1]: print("ABORT: clamp:", ce[-1][:260] if ce else "no event"); break
    say("arm 3 1"); time.sleep(0.5)
    pos=os.path.getsize(LOG); say(f"pstream 3 1 {delta} {rate} {os.environ.get('PERIOD','50')} {os.environ.get('HS','1')} {accel} {os.environ.get('LEAD','0')}")
    dur=abs(delta)/rate+(rate/accel if accel else 0)+2.2
    time.sleep(dur); say("stop"); time.sleep(1.6)
    L=tail(pos); evs=[l for l in L if l.startswith("#evt")]
    ps=[l for l in evs if "pstep n3c1 from" in l]; end=[l for l in evs if "ended by guard" in l or "done (window over)" in l]; guard=[l for l in evs if re.search(r"guard \w+ tripped:", l)]
    bad=[l for l in evs if any(k in l for k in ("refused","fault","FAILED","move_failed"))]
    if not ps or not end: print("ABORT: move did not run:", (bad or evs)[-1][:260] if (bad or evs) else "no events"); break
    T0=int(ps[-1].split(",")[1]); me=re.search(r"start (-?\d+) \(median\), target (-?\d+), final (-?\d+).*peak \|duty\| \d+ \(([\d.]+) %\), peak \|current\| (\d+) mA", end[-1])
    s0,tg,fin=int(me[1]),int(me[2]),int(me[3]); ix={k:i for i,k in enumerate(cols)}
    rows=[l[2:].split(",") for l in L if l.startswith("D,")]; rows=[r for r in rows if len(r)==len(cols) and r[0].isdigit() and r[ix["n3c1_enc"]]!=""]
    mv=[r for r in rows if T0<=int(r[0])<=T0+dur*1e6]
    sp=[]; lag=[]
    for k in range(0,len(mv)-25,5):
        w=mv[k:k+25]; v=(float(w[-1][ix["n3c1_enc"]])-float(w[0][ix["n3c1_enc"]]))/0.24; sp.append(abs(v))
        tg_=[float(r[ix["n3c1_tgt"]]) for r in w if r[ix["n3c1_tgt"]]!=""]
        if tg_: lag.append(abs(st.mean(tg_)-st.mean(float(r[ix["n3c1_enc"]]) for r in w)))
    gy=max(max(abs(float(r[ix["imu_gx"]])),abs(float(r[ix["imu_gy"]]))) for r in mv)*57.3
    rt=rest_tilt(L, cols); g=re.search(r"guard (\w+) tripped:", guard[-1])[1] if guard else "window"
    print(f"{delta:+5d}@{rate:<4d}a{accel:<5d} {s0:6d} {tg:6d} {fin:6d} {tg-fin:+5d}  {float(me[4]):6.1f}  {int(me[5])/1000:5.1f}   {max(sp) if sp else 0:6.0f}   {max(lag) if lag else 0:4.0f}   {gy:5.1f}     | {rt[0]:+6.2f} {rt[1]:+6.2f}" + ("" if g=="console_byte" else f"   [ended by: {g}]"), flush=True)
    if g not in ("console_byte","window"): print("ABORT: guard", guard[-1][:240]); break
    if abs(tg-fin)>MAXERR: print(f"ABORT: ended {tg-fin:+d} counts from target"); break
    if bad: print("ABORT:", bad[-1][:240]); break
    time.sleep(0.8)
