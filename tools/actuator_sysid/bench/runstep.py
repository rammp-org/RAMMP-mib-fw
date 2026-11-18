"""runstep.py <delta> ; env: PROF="vel acc", GAINS="P I D MaxI db", VG="P I D QPPS", WAIT=seconds, PRE=seconds to wait first."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, time, subprocess, os
CMD = benchenv.CMDS
def say(c): open(CMD, "a").write(c + "\n")
time.sleep(float(os.environ.get("PRE", "0")))
say("arm 3 1"); time.sleep(0.9)
if os.environ.get("GAINS"): say("gains 3 1 " + os.environ["GAINS"]); time.sleep(1.0)
if os.environ.get("VG"): say("vgains 3 1 " + os.environ["VG"]); time.sleep(1.0)
say(f"pstep 3 1 {sys.argv[1]} " + os.environ.get("PROF", "")); time.sleep(float(os.environ.get("WAIT", "7.5")))
subprocess.run([sys.executable, benchenv.script("stepsum.py")])
