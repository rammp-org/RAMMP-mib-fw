"""single.py <delta> <speed_cap>: one target, controller speed cap (0x6081), no streaming. Node 3 M1."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, time, re, os, subprocess
C=benchenv.CMDS; LOG=benchenv.log()
def say(c): open(C,"a").write(c+"\n")
delta=int(sys.argv[1]); cap=int(sys.argv[2])
pos=os.path.getsize(LOG); say("arm 3 1"); time.sleep(0.7)
ev=[l for l in open(LOG,"rb").read()[pos:].decode("utf-8","replace").splitlines() if "armed n3c1" in l]
p=int(re.search(r"present (-?\d+)", ev[-1])[1]); t=p+delta
say(f"clamp 3 1 {min(p,t)-150} {max(p,t)+150}"); time.sleep(1.0); say("arm 3 1"); time.sleep(0.5)
say(f"pstep 3 1 {delta} {cap} 500"); time.sleep(abs(delta)/cap+2.6); say("stop"); time.sleep(1.5)
subprocess.run([sys.executable,benchenv.script("streamsum.py"),"0.25"])
