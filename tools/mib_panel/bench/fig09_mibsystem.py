"""Figure 09: MR driven by the main firmware (MibSystem leg task) from the panel protocol.
fig09_mibsystem.py <frames.jsonl> <out.png> <out.csv>"""
import csv, json, sys
sys.path.insert(0, "/Users/alxchunlin/code-stuff/alex-style-guide/matplotlib")
import alex_style as ax
import matplotlib.pyplot as plt

src, png, out_csv = sys.argv[1:4]
samples, sent = [], []
for line in open(src):
    r = json.loads(line)
    if "sent" in r:
        sent.append((r["t"], r["sent"]))
    elif "status" in r:
        mr = r["status"]["legs"][0]
        samples.append((r["t"], mr["pos"], mr["target"], mr["moving"], r["status"]["state"]))

def first(prefix, after=0.0):
    return next(t for t, w in sent if w.startswith(prefix) and t >= after)

t_up = first("mover 0 500"); t_down = first("mover 0 -500")
t_hold = first("jog 0 200"); t_rel = first("jog 0 0")
t_move = first("mover 0 -2000"); t_estop = first("estop engage")
windows = [
    ("step", "MR +500 then -500 (LEG_MOVE relative)", t_up - 0.5, t_down + 3.5, t_up,
     [(t_up, "+500 sent"), (t_down, "-500 sent")]),
    ("jog", "Held jog: LEG_JOG +200 every 200 ms for 2 s, then 0", t_hold - 0.5, t_rel + 2.0, t_hold,
     [(t_hold, "button down"), (t_rel, "release (jog 0)")]),
    ("estop", "LEG_MOVE -2000, panel E-STOP 1.2 s in", t_move - 0.5, t_estop + 2.0, t_move,
     [(t_move, "move sent"), (t_estop, "E-STOP")]),
]

ax.use("dark")
fig, axes = plt.subplots(3, 1, figsize=(10, 10.5))
rows = []
for a, (key, caption, t0, t1, zero, marks) in zip(axes, windows):
    pts = [s for s in samples if t0 <= s[0] <= t1 and s[1] is not None]
    x = [s[0] - zero for s in pts]
    a.plot(x, [s[1] for s in pts], label="encoder (STATUS)")
    for t, s in [(s[0], s) for s in pts]:
        rows.append([key, f"{t - zero:.3f}", s[1], "" if s[2] is None else s[2], int(s[3]), s[4]])
    pal = ax.PALETTE[ax._mode]
    for (t, text), status in zip(marks, ["warning", "critical"]):
        a.axvline(t - zero, color=pal[status], linestyle=(0, (4, 3)), linewidth=1)
        a.annotate(text, xy=(t - zero, 0), xycoords=("data", "axes fraction"), xytext=(4, 6),
                   textcoords="offset points", color=pal["muted"], fontsize=8)
    if key == "estop":
        ax.ref_line(a, pts[-1][2], f"move target {pts[-1][2]:,} (not reached)", status="off")
    a.set_xlabel("Time from the first command, s")
    a.set_ylabel("Right leg position, counts")
    ax.title(a, {"step": "Moves from the main firmware", "jog": "A held jog and its release",
                 "estop": "E-stop during a move"}[key], caption)
fig.tight_layout()
ax.save(fig, png)
with open(out_csv, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["test", "t_s", "encoder_counts", "last_target_counts", "moving", "state"])
    w.writerows(rows)
print(len(samples), "samples")
