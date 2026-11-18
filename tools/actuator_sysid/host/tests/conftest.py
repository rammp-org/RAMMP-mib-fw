"""Simulated runs shared by the tests, built once per session in a temp folder.

Every run is written by simulate.write_run (the same code as `uv run simulate.py`) and
read back through runlog.load_run, so the tests exercise the file path end to end.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from simulate import write_run  # noqa: E402
from sysid import attitude, plant, runlog  # noqa: E402
from sysid.analyses import Opts  # noqa: E402


class SimRun:
    def __init__(self, d: Path, sim: plant.Sim):
        self.dir = d
        self.sim = sim
        self.truth = json.loads((d / "sim_truth.json").read_text())
        self.run = runlog.load_run(d)
        self.opts = Opts(mount=attitude.Mount.from_rpy(sim.mount_rpy))


@pytest.fixture(scope="session")
def runs_root(tmp_path_factory):
    return tmp_path_factory.mktemp("runs")


@pytest.fixture(scope="session")
def make(runs_root):
    cache = {}

    def _make(scenario, label=None, **kw):
        key = (scenario, label, tuple(sorted(kw.items())))
        if key not in cache:
            sim = plant.Sim(**kw)
            d = write_run(scenario, runs_root, sim, label or f"{scenario}-{len(cache)}", stamp="t")
            cache[key] = SimRun(d, sim)
        return cache[key]

    return _make
