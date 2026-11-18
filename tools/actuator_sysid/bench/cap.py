"""Minimal raw capture: cap.py <out> <seconds> [t:cmd ...]  (commands sent at t seconds)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import serial, sys, time
out, secs = sys.argv[1], float(sys.argv[2])
cmds = sorted((float(a.split(":",1)[0]), a.split(":",1)[1]) for a in sys.argv[3:])
s = serial.Serial(); s.port = benchenv.port(); s.baudrate = 921600; s.timeout = 0.05
s.dtr = False; s.rts = False; s.open()
t0 = time.time(); n = 0
with open(out, "wb") as f:
    while time.time() - t0 < secs:
        while cmds and time.time() - t0 >= cmds[0][0]:
            c = cmds.pop(0)[1]; s.write((c + "\n").encode()); f.write(f"#host,{time.time()-t0:.3f},sent {c}\n".encode())
        b = s.read(65536); f.write(b); n += len(b)
s.close(); print(f"{n} bytes in {secs}s -> {out}")
