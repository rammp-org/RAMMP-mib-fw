"""Long-lived serial session: session.py <logfile> <cmdfile>. Appends everything read to logfile;
sends each new line appended to cmdfile. Line 'QUIT' in cmdfile sends 'stop' and exits."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import benchenv
import serial, sys, time, os
log, cmdf = (sys.argv[1] if len(sys.argv) > 1 else benchenv.log()), benchenv.CMDS
open(cmdf, "a").close(); pos = os.path.getsize(cmdf)
s = serial.Serial(); s.port = benchenv.port(); s.baudrate = 921600; s.timeout = 0.02
s.dtr = False; s.rts = False; s.open(); t0 = time.time()
with open(log, "ab", buffering=0) as f:
    f.write(f"#host,0.000,session open {time.strftime('%Y-%m-%dT%H:%M:%S')}\n".encode())
    while True:
        b = s.read(65536)
        if b: f.write(b)
        if os.path.getsize(cmdf) > pos:
            with open(cmdf) as c:
                c.seek(pos); new = c.read(); pos = c.tell()
            for line in new.splitlines():
                line = line.strip()
                if not line: continue
                if line == "QUIT":
                    s.write(b"stop\n"); f.write(f"\n#host,{time.time()-t0:.3f},sent stop (quit)\n".encode()); time.sleep(4); f.write(s.read(1<<20)); s.close(); sys.exit(0)
                s.write((line + "\n").encode()); f.write(f"\n#host,{time.time()-t0:.3f},sent {line}\n".encode())
