"""Kinematics (angle per count), smoothness, compare, and the legacy open-loop analyses."""

import numpy as np
import pytest

from sysid.analyses import compare, deadband, kinematics, smoothness, step

BASE = ("n1c1", "n1c2", "n3c1", "n3c2")


def test_kinematics_recovers_jacobian_and_backlash(make):
    r = make("kinematics")
    res = kinematics.compute(r.run, r.opts)
    assert set(res["axes"]) == set(BASE)  # node 2 never moved, so not in the map
    for p in BASE:
        j, t = res["jacobian_deg_per_count"][p], r.truth["jacobian_deg_per_count"][p]
        main = "pitch" if abs(t["pitch"]) > abs(t["roll"]) else "roll"
        assert j[main] == pytest.approx(t[main], rel=0.05)  # 5 %
        other = "roll" if main == "pitch" else "pitch"
        assert abs(j[other] - t[other]) < 0.05 * abs(t[main])  # cross-coupling right to 5 % of the main term
        b, se = res["backlash_counts"][p], res["backlash_se_counts"][p]
        assert abs(b - r.truth["axes"][p]["backlash"]) <= max(3.0, 3 * se)  # counts
    assert res["fit"]["pitch"]["rms_resid_deg"] < 0.1


@pytest.mark.parametrize("scenario", ["profile_fast", "profile_slow"])
def test_smoothness_matches_truth_without_noise(make, scenario):
    r = make(scenario, label=f"{scenario}-clean", noise=False)
    res = smoothness.compute(r.run, r.opts)
    tr = r.truth["smoothness"]
    assert len(res["moves"]) == len(tr)
    for m, t in zip(res["moves"], tr):
        assert m["rms_jerk"] == pytest.approx(t["rms_jerk"], rel=0.05)
        assert m["rms_ang_acc"] == pytest.approx(t["rms_ang_acc"], rel=0.05)
        assert m["peak_ang_acc"] == pytest.approx(t["peak_ang_acc"], rel=0.10)


def test_smoothness_flags_noise_and_compare_ranks_profiles(make):
    fast, slow = make("profile_fast"), make("profile_slow")
    rf, rs = smoothness.compute(fast.run, fast.opts), smoothness.compute(slow.run, slow.opts)
    # with BNO055 noise the seat jerk of these slow EHAs is under the floor: say so
    assert any("noise floor" in f for f in rf["flags"] + rs["flags"])
    # peak chassis rate is well above noise and ranks the tunings correctly
    assert rf["summary"]["peak_rate_dps"] > rs["summary"]["peak_rate_dps"]
    cmp = compare.compute_from_jsons([("fast", rf), ("slow", rs)], "smoothness", ["summary.peak_rate_dps"])
    vals = {row["run"]: row["value"] for row in cmp["metrics"]["summary.peak_rate_dps"]}
    assert vals["fast"] > vals["slow"]


def test_open_loop_step_legacy(make):
    r = make("steps")
    res = step.compute(r.run, r.opts)
    for p in BASE:
        a, d = r.truth["axes"][p], res["channels"][p]
        for dn in ("pos", "neg"):
            assert d[dn]["gain"] == pytest.approx(a[f"gain_{dn}"], rel=0.03)
            assert d[dn]["duty_offset_pct"] == pytest.approx(a[f"ukin_{dn}"], abs=0.5)
            # from rest an EHA waits for pressure to reach breakaway: the FOPDT 'dead time' includes that
            # build-up, so it can only be bounded here ('analyze.py plant' models breakaway and separates them)
            assert d[dn]["dead_time_s"] >= r.truth["dead_total"][p] - 0.005
    # coast after the excitation ends, per step against the simulator's own trajectory
    tr = {(t["prefix"], round(t["t_on"], 1)): t for t in r.truth["steps"]}
    for p in BASE:
        for s in res["channels"][p]["steps"]:
            t = tr[(p, round(s["t_on"], 1))]
            if s.get("coast_counts") is not None:
                assert s["coast_counts"] == pytest.approx(t["coast_counts"], abs=2.0)


def test_open_loop_external_drive_uses_duty_readback(make):
    r = make("steps", label="steps-ext", external_drive=True)
    assert np.nanmax(np.abs(r.run.col("n1c1_cmd"))) == 0
    res = step.compute(r.run, r.opts)
    for p in BASE:
        d = res["channels"][p]
        assert d["source"] == "duty"
        for dn in ("pos", "neg"):
            assert d[dn]["gain"] == pytest.approx(r.truth["axes"][p][f"gain_{dn}"], rel=0.03)
            # edges from 50 Hz readback (half a read interval); includes pressure build-up, as above
            assert d[dn]["dead_time_s"] >= r.truth["axes"][p]["dead_mech"] - 0.012


def test_open_loop_deadband_legacy(make):
    r = make("deadband")
    res = deadband.compute(r.run, r.opts)
    for p in BASE:
        a = r.truth["axes"][p]
        for dn in ("pos", "neg"):
            e = res["channels"][p][dn]
            lo, hi = e["breakaway_bracket_pct"]
            assert lo <= a[f"ubrk_{dn}"] <= hi
            assert e["kinetic_pct"] == pytest.approx(a[f"ukin_{dn}"], abs=0.5)


def test_open_loop_analyses_decline_closed_loop_runs(make):
    r = make("pstep")
    res = step.compute(r.run, r.opts)
    assert not res["channels"]
    assert any("closed-loop" in f for f in res["flags"])
