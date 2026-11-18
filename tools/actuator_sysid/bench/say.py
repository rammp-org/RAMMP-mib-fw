"""say.py <wait_s> [cmd ...]: append commands (1 s apart) to the session, wait, print new non-data lines and a row summary."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import sys, time, os
LOG = benchenv.log()
CMD = benchenv.CMDS
wait = float(sys.argv[1]); pos = os.path.getsize(LOG)
for i, c in enumerate(sys.argv[2:]):
    if c.startswith("sleep:"): time.sleep(float(c[6:])); continue
    open(CMD, "a").write(c + "\n"); time.sleep(0.3)
time.sleep(wait)
with open(LOG, "rb") as f:
    f.seek(pos); txt = f.read().decode("utf-8", "replace")
L = txt.splitlines(); D = [l for l in L if l.startswith("D,")]
skip = ("D,", "#cols", "#meta:unit", "#meta:refresh", "#meta:empty", "#meta:app", "#meta:idf", "#meta:channels", "#meta:sample_p")
show = [l for l in L if l and not l.startswith(skip)]
flt = os.environ.get("FLT")
if flt: show = [l for l in show if any(k in l for k in flt.split("|"))]
print("\n".join(l[:int(os.environ.get("W", "420"))] for l in show)); print(f"[{len(D)} data rows]")
