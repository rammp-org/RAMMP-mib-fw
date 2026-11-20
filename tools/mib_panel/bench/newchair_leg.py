"""Guarded leg moves on the new chair over Ethernet (MIB service link, tcp :5555).

    newchair_leg.py status
    newchair_leg.py move <leg 0|1> <counts> [abs]      relative by default
    newchair_leg.py range <leg> <min> <max>
    newchair_leg.py read <leg>                        position, PWM, current, statusword
    newchair_leg.py estop                            latch the panel E-STOP (legs stop within a tick)
    newchair_leg.py clear                            start an E-STOP clear attempt

Legs: 0 = MR (right, node 3 M1): -counts LOWERS, +counts lifts.
      1 = ML (left,  node 3 M2): +counts LOWERS, -counts lifts.

While a leg moves, its count, PWM and current are read over SDO every ~50 ms and the panel
E-STOP is sent at once if: speed > 600 counts/s, > 80 counts the wrong way, |PWM| > 30 %,
current > 8 A, or the count leaves the leg's range by > 100. Every move's trace is saved.
"""
import sys, time, json, struct, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mibnet import Link

GUARD_SPEED, GUARD_WRONG, GUARD_PWM, GUARD_AMPS, GUARD_OUT = 900, 80, 0.30, 20.0, 100
STALL_PWM, STALL_COUNTS, STALL_S = 0.05, 15, 2.5
OUT = "/tmp/newchair_moves"; os.makedirs(OUT, exist_ok=True)
L = Link()


def sdo(node, idx, sub=0, nb=4):
    t, p = L.request(0x0E, bytes([node, idx & 0xFF, idx >> 8, sub, nb]), want=(0x8E,), timeout=2)
    return struct.unpack("<i", p[4:8])[0] if t == 0x8E else None


def status():
    t, p = L.request(0x09, want=(0x89,), timeout=2)
    return json.loads(p) if t == 0x89 else None


def estop():
    L.s.sendall(__import__("mibnet").frame(0x40, 0x08, bytes([1])))


def req(typ, payload):
    t, p = L.request(typ, payload, want=(0x81,), timeout=3)
    return "OK" if t == 0x81 else ("ERROR: " + p[1:].decode(errors="replace") if p else "no reply")


def read(leg):
    off, sub = (0, 1) if leg == 0 else (0x800, 2)
    pos = sdo(3, 0x6064 + off)
    pwm = sdo(3, 0x2030, sub, 2)
    cur = sdo(3, 0x2031, sub, 2)
    sw = sdo(3, 0x6041 + off, 0, 2)
    return pos, pwm, cur, sw


def move(leg, counts, absolute=False):
    st = status(); lg = st["legs"][leg]; g0 = st.get("imu", {}).get("g")
    lo, hi = lg["min"], lg["max"]
    p0 = read(leg)[0]
    target = counts if absolute else p0 + counts
    want = 1 if target > p0 else -1
    print(f"{lg['name']}: at {p0}, range [{lo}, {hi}], target {target} ({'abs' if absolute else f'{counts:+d}'})")
    r = req(0x0A, bytes([leg, 0 if absolute else 1]) + struct.pack("<i", counts))
    if r != "OK":
        print("move refused:", r); return
    trace, t0, tripped, last_status = [], time.time(), None, 0
    try:
        while time.time() - t0 < 20:
            pos, pwm, cur, sw = read(leg)
            t = time.time() - t0
            if pos is None:
                continue
            trace.append((round(t, 3), pos, pwm, cur, sw))
            back = [x for x in trace if x[0] <= t - 0.1]
            speed = (pos - back[-1][1]) / (t - back[-1][0]) if back else 0
            why = None
            if abs(speed) > GUARD_SPEED: why = f"speed {speed:.0f} counts/s"
            elif want * (pos - p0) < -GUARD_WRONG: why = f"{pos - p0:+d} counts the wrong way"
            elif pwm is not None and abs(pwm) > GUARD_PWM * 32767: why = f"PWM {pwm * 100 / 32767:.1f} %"
            elif cur is not None and abs(cur) / 100 > GUARD_AMPS: why = f"current {cur / 100:.1f} A"
            elif pos < lo - GUARD_OUT or pos > hi + GUARD_OUT: why = f"count {pos} outside [{lo}, {hi}]"
            else:
                win = [x for x in trace if x[0] >= t - STALL_S]
                if t > STALL_S and win and trace[0][0] <= t - STALL_S:
                    busy = sum(1 for x in win if x[2] is not None and abs(x[2]) > STALL_PWM * 32767)
                    span = max(x[1] for x in win) - min(x[1] for x in win)
                    if busy >= len(win) / 2 and span < STALL_COUNTS:
                        why = f"stall: PWM > {STALL_PWM * 100:.0f} % for {STALL_S} s with {span} counts of motion"
            if why:
                estop(); tripped = why
                print(f"  GUARD at {t:.2f} s: {why} -> E-STOP sent")
                time.sleep(0.5)
                for _ in range(5):
                    p, w, c, s_ = read(leg); trace.append((round(time.time() - t0, 3), p, w, c, s_)); time.sleep(0.1)
                break
            if t - last_status > 0.25:
                last_status = t
                m = status()["legs"][leg]
                if not m["moving"] and m["last_move"]["outcome"] not in ("moving", "none") and t > 0.5:
                    break
            time.sleep(0.03)
    except KeyboardInterrupt:
        estop(); tripped = "keyboard"; print("  interrupted -> E-STOP sent")
    time.sleep(1.0)
    st1 = status(); m = st1["legs"][leg]; g1 = st1.get("imu", {}).get("g")
    import math
    if g0 and g1:
        dg = [b - a for a, b in zip(g0, g1)]
        ang = math.degrees(math.acos(max(-1, min(1, sum(a * b for a, b in zip(g0, g1)) / (math.hypot(*g0) * math.hypot(*g1))))))
        print(f"  IMU gravity {[round(x, 3) for x in g0]} -> {[round(x, 3) for x in g1]}: change {[round(x, 3) for x in dg]}, tilt {ang:.2f} deg")
    else:
        dg = None
    pwms = [abs(x[2]) for x in trace if x[2] is not None]; curs = [abs(x[3]) for x in trace if x[3] is not None]
    ps = [x[1] for x in trace]
    sp = [abs((trace[i][1] - trace[i - 3][1]) / (trace[i][0] - trace[i - 3][0])) for i in range(3, len(trace)) if trace[i][0] > trace[i - 3][0]]
    print(f"  end {ps[-1]} ({ps[-1] - p0:+d} from start, {ps[-1] - target:+d} from target); moved {min(ps)}..{max(ps)}; "
          f"max PWM {max(pwms) * 100 / 32767:.1f} %, max current {max(curs) / 100:.2f} A, max speed {max(sp) if sp else 0:.0f} counts/s")
    print(f"  MIB: {m['last_move']}{'  GUARD: ' + tripped if tripped else ''}")
    name = time.strftime("%H%M%S") + f"_leg{leg}_{'abs' if absolute else 'rel'}{counts:+d}.json"
    json.dump({"leg": leg, "counts": counts, "absolute": absolute, "start": p0, "target": target, "range": [lo, hi],
               "trace": trace, "result": m["last_move"], "guard": tripped, "g0": g0, "g1": g1}, open(os.path.join(OUT, name), "w"))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "status":
        st = status(); print(st["state"], "estop", st["estop"]["latched"])
        for lg in st["legs"]: print(" ", json.dumps(lg))
    elif cmd == "read":
        print(read(int(sys.argv[2])))
    elif cmd == "range":
        print(req(0x0B, bytes([int(sys.argv[2])]) + struct.pack("<ii", int(sys.argv[3]), int(sys.argv[4]))))
    elif cmd == "estop":
        estop(); time.sleep(0.3); print("E-STOP latched:", status()["estop"]["latched"])
    elif cmd == "clear":
        print(req(0x08, bytes([2])))
    elif cmd == "move":
        move(int(sys.argv[2]), int(sys.argv[3]), len(sys.argv) > 4 and sys.argv[4] == "abs")
