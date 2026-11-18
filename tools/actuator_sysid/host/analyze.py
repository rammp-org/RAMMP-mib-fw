#!/usr/bin/env python3
"""Analyse a captured (or simulated) run. Each subcommand writes <name>.png and <name>.json into the run folder.

    uv run analyze.py overview                 # the latest run under runs/
    uv run analyze.py step runs/20261117-101500_fc-steps --open
    uv run analyze.py all runs/<run>           # every analysis
    uv run analyze.py level-ref runs/<level run> [--mount 0,0,90]   # writes runs/level_ref.json
    uv run analyze.py compare smoothness runs/<a> runs/<b>
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from sysid import attitude, runlog  # noqa: E402
from sysid.analyses import ANALYSES, Opts, compare, run_analysis  # noqa: E402
from sysid.analyses.common import write_json  # noqa: E402

DEFAULT_RUNS = HERE / "runs"


def latest_run(runs_dir: Path) -> Path:
    cands = [p for p in runs_dir.glob("*/raw.log")]
    if not cands:
        sys.exit(f"no runs under {runs_dir}")
    return max(cands, key=lambda p: p.stat().st_mtime).parent


def resolve_run(arg: str | None, runs_dir: Path) -> Path:
    if arg in (None, "latest"):
        return latest_run(runs_dir)
    p = Path(arg)
    if p.exists():
        return p if p.is_dir() else p.parent
    q = runs_dir / arg
    if q.exists():
        return q
    hits = sorted(runs_dir.glob(f"*{arg}*"))
    if hits:
        return hits[-1]
    sys.exit(f"run not found: {arg}")


def parse_rpy(s):
    if not s:
        return None
    v = [float(x) for x in s.split(",")]
    if len(v) != 3:
        sys.exit("--mount wants roll,pitch,yaw in degrees")
    return tuple(v)


def common_args(ap):
    ap.add_argument("--runs-dir", type=Path, default=DEFAULT_RUNS)
    ap.add_argument("--names", help="axis names, e.g. n1c1=FC,n1c2=RC,n3c1=ML,n3c2=MR")
    ap.add_argument("--mount", help="board rotation in the chassis, roll,pitch,yaw degrees (ZYX)")
    ap.add_argument("--level-ref", help="level_ref.json, or a run captured on a level floor; 'none' to ignore")
    ap.add_argument("--source", default="auto", choices=["auto", "cmd", "duty", "events"],
                    help="what marks the excitation: firmware _cmd, applied _duty readback, or step events/marks")
    ap.add_argument("--fc", type=float, default=5.0, help="smoothness low-pass, Hz")
    ap.add_argument("--settle", type=float, default=1.0, help="seconds after a move still counted as the move")
    ap.add_argument("--move-thresh", type=float, default=3.0, help="counts that count as motion")
    ap.add_argument("--imu-lag-ms", type=float, help="IMU delay behind the encoders (from latency.json)")
    ap.add_argument("--tol", type=float, help="position settling band, counts (default max(3, 2%% of step))")
    ap.add_argument("--deadband", help="deadband.json to compare position stalls against")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--open", action="store_true", help="open the PNG when done (macOS 'open')")


def make_opts(a, run_dir: Path) -> Opts:
    mount = attitude.load_mount(run_dir, a.level_ref, parse_rpy(a.mount))
    lag = a.imu_lag_ms
    if lag is None and (run_dir / "latency.json").exists():
        try:
            v = json.loads((run_dir / "latency.json").read_text())["summary"].get("enc_to_imu_lag_s")
            lag = 1000 * v if v is not None else None
        except (ValueError, KeyError):
            lag = None
    return Opts(mount=mount, source=a.source, fc_hz=a.fc, settle_s=a.settle, move_thresh=a.move_thresh,
                imu_lag_ms=lag, tol_counts=a.tol, deadband_json=a.deadband)


def show(png, a):
    if a.open and png:
        subprocess.run(["open", str(png)], check=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in list(ANALYSES) + ["all"]:
        sp = sub.add_parser(name, help=(ANALYSES[name].__doc__ or "").strip().split("\n")[0] if name in ANALYSES
                            else "every analysis on one run")
        sp.add_argument("run", nargs="?", help="run folder, raw.log, a name fragment, or 'latest' (default)")
        common_args(sp)
    sp = sub.add_parser("level-ref", help="zero the board mounting from a capture on a level floor")
    sp.add_argument("run", nargs="?")
    sp.add_argument("--out", type=Path, help="default: <runs-dir>/level_ref.json")
    common_args(sp)
    sp = sub.add_parser("compare", help="overlay one analysis's metrics across runs")
    sp.add_argument("analysis", choices=list(ANALYSES))
    sp.add_argument("runs", nargs="+")
    sp.add_argument("--metric", action="append", help="dotted path with * wildcards; repeatable")
    sp.add_argument("--out", type=Path, help="output folder (default: the last run's folder)")
    sp.add_argument("--rerun", action="store_true", help="recompute each run's JSON even if present")
    common_args(sp)
    a = ap.parse_args(argv)
    names = runlog.parse_names(a.names)

    if a.cmd == "compare":
        items = []
        dirs = [resolve_run(r, a.runs_dir) for r in a.runs]
        for d in dirs:
            res = compare.load_json(d, a.analysis)
            if res is None or a.rerun:
                run = runlog.load_run(d, not a.no_cache, names)
                res, _, _ = run_analysis(a.analysis, run, make_opts(a, d), plot=not a.no_plot)
            items.append((d.name, res))
        res = compare.compute_from_jsons(items, a.analysis, a.metric)
        out = a.out or dirs[-1]
        out.mkdir(parents=True, exist_ok=True)
        js = write_json(out / f"compare_{a.analysis}.json", res)
        png = None if a.no_plot else compare.plot(res, out / f"compare_{a.analysis}.png")
        for line in compare.summary_lines(res):
            print(line)
        print(f"-> {png or js}")
        show(png, a)
        return 0

    run_dir = resolve_run(a.run, a.runs_dir)
    t0 = time.monotonic()
    run = runlog.load_run(run_dir, not a.no_cache, names)
    t_load = time.monotonic() - t0

    if a.cmd == "level-ref":
        nominal = attitude.Mount.from_rpy(parse_rpy(a.mount)) if a.mount else attitude.Mount()
        m = attitude.level_reference(run, nominal)
        out = a.out or (a.runs_dir / "level_ref.json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(m.to_json(), indent=1))
        r, p, y = m.rpy()
        print(f"board mounting: roll {r:+.3f}  pitch {p:+.3f}  yaw {y:+.3f} deg  -> {out}")
        print("every analysis of runs under that folder now uses it (--level-ref none to ignore)")
        return 0

    opts = make_opts(a, run_dir)
    print(f"{run_dir.name}: {run.stats['rows']} rows, {len(run.events)} events, loaded in {t_load:.2f} s; "
          f"mount from {opts.mount.source}")
    todo = list(ANALYSES) if a.cmd == "all" else [a.cmd]
    png = None
    for name in todo:
        t1 = time.monotonic()
        res, js, png = run_analysis(name, run, opts, plot=not a.no_plot)
        print(f"== {name} ({time.monotonic() - t1:.2f} s)")
        for line in ANALYSES[name].summary_lines(res):
            print("  " + line)
        for f in res.get("flags", []):
            print("  ! " + f)
        print(f"  -> {png or js}")
    show(png, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
