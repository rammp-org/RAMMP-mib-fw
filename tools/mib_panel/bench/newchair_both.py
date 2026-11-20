"""Both main legs together, kept level by the IMU: `both.py down 2000` / `both.py up 2000`.
Chunks of ~250 counts per leg sent together; every leg-guard of newchair_leg applies to each leg."""
import os, sys, time, json, struct; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import newchair_leg as N
REF_Z, K, CHUNK, MAXC = -1.68, 0.00037, 250, 150   # z per count of one leg; correction cap
G = N
def send_move(leg, counts):
    return N.req(0x0A, bytes([leg, 1]) + struct.pack("<i", counts))
def chunk(steps):  # steps {leg: counts}
    p0 = {l: N.read(l)[0] for l in steps}
    want = {l: (1 if c > 0 else -1) for l, c in steps.items()}
    for l, c in steps.items():
        r = send_move(l, c)
        if r != "OK": print(" refused", l, r); N.estop(); return None, "refused"
    t0 = time.time(); tr = {l: [] for l in steps}; peak = {l: [0, 0] for l in steps}; why = None
    while time.time() - t0 < 12:
        t = time.time() - t0
        for l in steps:
            pos, pwm, cur, sw = N.read(l)
            if pos is None: continue
            tr[l].append((t, pos, pwm, cur))
            peak[l][0] = max(peak[l][0], abs(pwm or 0)); peak[l][1] = max(peak[l][1], abs(cur or 0))
            back = [x for x in tr[l] if x[0] <= t - 0.1]
            spd = (pos - back[-1][1]) / (t - back[-1][0]) if back else 0
            win = [x for x in tr[l] if x[0] >= t - N.STALL_S]
            if abs(spd) > N.GUARD_SPEED: why = f"{l}: speed {spd:.0f}"
            elif want[l] * (pos - p0[l]) < -N.GUARD_WRONG: why = f"{l}: {pos - p0[l]:+d} the wrong way"
            elif pwm is not None and abs(pwm) > N.GUARD_PWM * 32767: why = f"{l}: PWM {pwm * 100 / 32767:.1f}%"
            elif cur is not None and abs(cur) / 100 > N.GUARD_AMPS: why = f"{l}: current {cur / 100:.1f} A"
            elif t > N.STALL_S and tr[l][0][0] <= t - N.STALL_S and \
                    sum(1 for x in win if x[2] is not None and abs(x[2]) > N.STALL_PWM * 32767) >= len(win) / 2 and \
                    max(x[1] for x in win) - min(x[1] for x in win) < N.STALL_COUNTS:
                why = f"{l}: stall"
            if why: break
        if why:
            N.estop(); print("  GUARD", why, "-> E-STOP"); time.sleep(1); break
        st = N.status()
        if t > 0.5 and not any(st["legs"][l]["moving"] for l in steps): break
        time.sleep(0.02)
    moved = {l: N.read(l)[0] - p0[l] for l in steps}
    return {"moved": moved, "peak": {l: (round(peak[l][0] * 100 / 32767, 1), round(peak[l][1] / 100, 1)) for l in steps}}, why
direction = sys.argv[1]; total = int(sys.argv[2])
sgn = {"down": {0: -1, 1: +1}, "up": {0: +1, 1: -1}}[direction]
done = {0: 0, 1: 0}; log = []
while (abs(done[0]) + abs(done[1])) / 2 < total - 60:
    z = N.status()["imu"]["g"][2]
    c = max(-MAXC, min(MAXC, (REF_Z - z) / K))      # >0: z must rise (right side down / left side up)
    left = total - (abs(done[0]) + abs(done[1])) / 2
    base = min(CHUNK, left)
    # z rises when MR goes down (-counts) or ML goes up (-counts)
    steps = {0: int(sgn[0] * base - c / 2), 1: int(sgn[1] * base - c / 2)}
    res, why = chunk(steps)
    z2 = N.status()["imu"]["g"][2]
    if res is None: break
    for l in (0, 1): done[l] += res["moved"][l]
    print(f"{direction} chunk: asked MR {steps[0]:+d} ML {steps[1]:+d} | moved MR {res['moved'][0]:+d} ML {res['moved'][1]:+d} | z {z:.2f}->{z2:.2f} | PWM%/A MR {res['peak'][0]} ML {res['peak'][1]}")
    log.append({"steps": steps, "res": res, "z": [z, z2], "why": why})
    if why or (abs(res["moved"][0]) < 40 and abs(res["moved"][1]) < 40): print("  stopping"); break
    time.sleep(3.0 if direction == "up" else 1.0)
json.dump(log, open(f"/tmp/newchair_moves/both_{direction}_{time.strftime('%H%M%S')}.json", "w"))
print(f"done {direction}: MR {done[0]:+d} ML {done[1]:+d}; now MR {N.read(0)[0]} ML {N.read(1)[0]} z {N.status()['imu']['g'][2]}")
