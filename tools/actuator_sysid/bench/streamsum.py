"""streamsum.py: summarize the last pstream/pstep using the logged _tgt column (0.2 s bins)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import re, statistics as st, sys
LOG=benchenv.log()
L=open(LOG,"rb").read().decode("utf-8","replace").splitlines()
b=max([i for i,l in enumerate(L) if "#evt," in l and "boot:" in l] or [0]); L=L[b:]
ev=[l for l in L if l.startswith("#evt,")]
ps=[l for l in ev if re.search(r"pstep n3c\d from",l)]
m=re.search(r"#evt,(\d+),pstep n3c(\d) from (-?\d+) to (-?\d+)",ps[-1]); T0=int(m[1]); ch=m[2]
for l in ev:
    tt=int(l.split(",")[1])
    if T0-3.2e6<=tt<=T0+8e6 and any(k in l for k in ("gains n","pstep n","pstream","guard","stop sequence done","fault","refused")): print(l[:300])
cols=[l for l in L if l.startswith("#cols:")][-1][6:].split(","); ix={k:i for i,k in enumerate(cols)}
rows=[l[2:].split(",") for l in L if l.startswith("D,")]
rows=[r for r in rows if len(r)==len(cols) and r[0].isdigit() and T0-0.4e6<=int(r[0])<=T0+6.0e6]
P=f"n3c{ch}_"
def g(r,k):
    v=r[ix[P+k]]; return float(v) if v!="" else float("nan")
step=float(sys.argv[1]) if len(sys.argv)>1 else 0.2
print("  t     tgt     enc   tgt-enc  duty%  cur_A  speed   gyro_x(deg/s)")
k=0; n=int(step*100)
while k+n<=len(rows):
    w=rows[k:k+n]; t=(int(w[0][0])-T0)/1e6
    tg=[g(r,"tgt") for r in w if r[ix[P+"tgt"]]!=""]; e=[g(r,"enc") for r in w]; d=[g(r,"duty")/327.67 for r in w]; c=[abs(g(r,"cur_ma"))/1000 for r in w]
    v=(st.mean(e[-3:])-st.mean(e[:3]))/((n-3)/100)
    gx=max(abs(float(r[ix["imu_gx"]])) for r in w)*57.3
    tgs=f"{st.mean(tg):7.0f}" if tg else "      -"; err=f"{st.mean(tg)-st.mean(e):+7.0f}" if tg else "      -"
    print(f"{t:5.1f} {tgs} {st.mean(e):7.0f} {err} {st.mean(d):+6.1f} {st.mean(c):6.2f} {v:+6.0f}   {gx:5.1f}")
    k+=n
