"""Parse the bench logs written by the actuator_sysid firmware into boot segments and moves.

Each segment is one boot of the MIB. A move is one `pstep` / `pstream` event with the controller
settings that were in force (the last gains / vgains / sdow events before it).
"""
import re
from dataclasses import dataclass, field


@dataclass
class Move:
    seg: int
    t0_us: int
    ch: int
    start: int
    target: int
    kind: str = "step"          # "step" or "stream"
    stream: str = ""            # text of the pstream line(s)
    gains: str = ""             # last position record installed
    vgains: str = ""            # last velocity record installed
    window: str = ""            # last 0x6067 write
    profile: str = ""           # "profile velocity V accel A"
    end: str = ""               # the end event text


@dataclass
class Segment:
    cols: list
    rows: list = field(default_factory=list)     # lists of strings, one per data row
    events: list = field(default_factory=list)   # (t_us, text)
    meta: dict = field(default_factory=dict)


def load(path):
    segs, cols, cur = [], None, None
    state = {"gains": "", "vgains": "", "window": ""}
    moves = []
    for line in open(path, "rb").read().decode("utf-8", "replace").splitlines():
        if line.startswith("#cols:"):
            cols = line[6:].split(",")
            if cur is not None and not cur.rows:
                cur.cols = cols
        elif line.startswith("#evt,"):
            _, t, text = line.split(",", 2)
            if not t.isdigit():
                continue
            if "boot:" in text or cur is None:
                cur = Segment(cols=cols or [])
                segs.append(cur)
            cur.events.append((int(t), text))
            if text.startswith("gains n3c1: installed"):
                state["gains"] = text.split("->")[-1].strip()
            elif text.startswith("vgains n3c1: installed"):
                state["vgains"] = text.split("->")[-1].strip()
            elif text.startswith("sdow n3 0x6067"):
                state["window"] = re.search(r"= (\d+)", text)[1]
            m = re.match(r"pstep n3c(\d) from (-?\d+) to (-?\d+).*?(profile velocity \d+ accel \d+)?", text)
            if m:
                pv = re.search(r"profile velocity \d+ accel \d+", text)
                moves.append(Move(len(segs) - 1, int(t), int(m[1]), int(m[2]), int(m[3]),
                                  gains=state["gains"], vgains=state["vgains"], window=state["window"],
                                  profile=pv[0] if pv else ""))
            elif text.startswith("pstream n3c") and moves and "ended" not in text:
                moves[-1].kind = "stream"
                moves[-1].stream += text.split(":", 1)[1].strip() + "; "
            elif moves and ("ended by guard" in text or "done (window over)" in text) and not moves[-1].end:
                moves[-1].end = text
        elif line.startswith("#meta:") and cur is not None:
            k, _, v = line[6:].partition("=")
            cur.meta[k] = v
        elif line.startswith("D,") and cur is not None and cols:
            r = line[2:].split(",")
            if len(r) == len(cur.cols) and r[0].isdigit():
                cur.rows.append(r)
    return segs, moves


def series(seg, move, names, before=0.5, after=6.0):
    """Columns `names` (floats, NaN when empty) for rows around a move; time in s from the move event."""
    ix = {k: i for i, k in enumerate(seg.cols)}
    out = {"t": []}
    for n in names:
        out[n] = []
    for r in seg.rows:
        t = (int(r[0]) - move.t0_us) / 1e6
        if -before <= t <= after:
            out["t"].append(t)
            for n in names:
                v = r[ix[n]]
                out[n].append(float(v) if v != "" else float("nan"))
    return out


if __name__ == "__main__":
    import sys
    segs, moves = load(sys.argv[1])
    print(f"{len(segs)} boots, {len(moves)} moves")
    for i, m in enumerate(moves):
        print(f"{i:3d} seg{m.seg:2d} t={m.t0_us/1e6:8.1f} c{m.ch} {m.start:6d}->{m.target:6d} {m.kind:6s} "
              f"g[{m.gains}] v[{m.vgains}] w[{m.window}] {m.profile.replace('profile velocity','pv').replace('accel','a')} "
              f"{m.stream[:70]}")
