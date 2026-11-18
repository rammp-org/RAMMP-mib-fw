"""One module per analyze.py subcommand: compute(run, opts) -> dict, plot(run, res, png, opts), summary_lines(res)."""

from __future__ import annotations

import time
from pathlib import Path

from . import compare, deadband, kinematics, latency, overview, plantid, pstep, rest, smoothness, step
from .common import Opts, write_json

ANALYSES = {
    "overview": overview,
    "rest": rest,
    "pstep": pstep,
    "plant": plantid,
    "latency": latency,
    "kinematics": kinematics,
    "smoothness": smoothness,
    "step": step,  # open loop (legacy): needs a duty excitation
    "deadband": deadband,  # open loop (legacy)
}


def run_analysis(name: str, run, opts: Opts, out_dir: Path | None = None, plot: bool = True) -> tuple[dict, Path, Path | None]:
    mod = ANALYSES[name]
    out_dir = Path(out_dir or run.path)
    t0 = time.monotonic()
    res = mod.compute(run, opts)
    res["compute_s"] = round(time.monotonic() - t0, 3)
    png = None
    if plot:
        png = mod.plot(run, res, out_dir / f"{name}.png", opts)
    js = write_json(out_dir / f"{name}.json", res)
    return res, js, png


__all__ = ["ANALYSES", "Opts", "run_analysis", "compare"]
