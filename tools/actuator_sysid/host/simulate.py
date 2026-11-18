#!/usr/bin/env python3
"""Write a simulated run in the logger's exact wire format, with the parameters it was made from.

    uv run simulate.py steps                     # runs/<timestamp>_sim-steps/{raw.log,meta.json,sim_truth.json}
    uv run simulate.py all                       # one run per scenario
    uv run simulate.py steps --external          # excitation from outside the MIB: _cmd stays 0
    uv run simulate.py deadband --clean          # no noise, no glitches

Scenarios: passive (also the level reference), steps, deadband, kinematics,
profile_step, profile_ramp, position (closed-loop P-only targets).
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from sysid import plant  # noqa: E402
from sysid.analyses.common import jsonable  # noqa: E402


def write_run(scenario: str, out_root: Path, sim: plant.Sim, label: str | None = None, stamp: str | None = None) -> Path:
    raw, truth, _ = plant.simulate(scenario, sim)
    stamp = stamp or dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    d = out_root / f"{stamp}_{label or 'sim-' + scenario}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "raw.log").write_bytes(raw)
    meta = {"source": "simulate.py", "scenario": scenario, "label": label or f"sim-{scenario}",
            "start_time": dt.datetime.now().isoformat(timespec="seconds"), "axis_names": truth["axis_names"],
            "notes": [], "seed": sim.seed, "external_drive": sim.external_drive}
    (d / "meta.json").write_text(json.dumps(meta, indent=1))
    (d / "sim_truth.json").write_text(json.dumps(jsonable(truth), indent=1))
    return d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", choices=list(plant.SCENARIOS) + ["all"])
    ap.add_argument("--out", type=Path, default=HERE / "runs")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--clean", action="store_true", help="no noise, drops, garbage or stray lines")
    ap.add_argument("--external", action="store_true", help="drive from outside the MIB: _cmd 0, no step events")
    ap.add_argument("--label")
    a = ap.parse_args(argv)
    sim = plant.Sim(seed=a.seed, noise=not a.clean, external_drive=a.external)
    todo = plant.SCENARIOS if a.scenario == "all" else [a.scenario]
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    for sc in todo:
        d = write_run(sc, a.out, sim, a.label if len(todo) == 1 else None, stamp)
        print(d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
