"""runstream.py <delta> <rate> ; env GAINS, VG, CLAMP="min max", PERIOD=50, ACCEL=0, HOLD=seconds before stop (default 5), BIN=0.5, PRE=0"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, time, subprocess, os
C=benchenv.CMDS
def say(c): open(C,"a").write(c+"\n")
time.sleep(float(os.environ.get("PRE","0")))
say("arm 3 1"); time.sleep(0.9)
if os.environ.get("CLAMP"): say("clamp 3 1 "+os.environ["CLAMP"]); time.sleep(1.0)
if os.environ.get("GAINS"): say("gains 3 1 "+os.environ["GAINS"]); time.sleep(1.0)
if os.environ.get("VG"): say("vgains 3 1 "+os.environ["VG"]); time.sleep(1.0)
say(f"pstream 3 1 {sys.argv[1]} {sys.argv[2]} {os.environ.get('PERIOD','50')} 1 {os.environ.get('ACCEL','0')}")
time.sleep(float(os.environ.get("HOLD","5"))); say("stop"); time.sleep(1.2)
subprocess.run([sys.executable,benchenv.script("streamsum.py"),os.environ.get("BIN","0.5")])
