"""The primary analyses on closed-loop pstep runs: plant identification, pstep metrics, latency, rest.

Tolerances are stated next to each assertion; they are what the default noise
(BNO055-like IMU noise, one-count encoder flicker, 50 Hz CAN reads, dropped rows,
garbage) allows.
"""

import numpy as np
import pytest

from sysid.analyses import latency, plantid, pstep, rest

BASE = ("n1c1", "n1c2", "n3c1", "n3c2")


@pytest.fixture(scope="module")
def ps_run(make):
    return make("pstep")


@pytest.fixture(scope="module")
def plant_res(ps_run):
    return plantid.compute(ps_run.run, ps_run.opts)


def test_plant_id_recovers_gain_deadband_breakaway_and_lags(ps_run, plant_res):
    for p in BASE:
        a = ps_run.truth["axes"][p]
        d = plant_res["channels"][p]
        for dn in ("pos", "neg"):
            assert d[dn]["gain"] == pytest.approx(a[f"gain_{dn}"], rel=0.05)  # 5 %
            assert d[dn]["deadband_pct"] == pytest.approx(a[f"ukin_{dn}"], abs=0.5)  # 0.5 % duty
            assert d[dn]["breakaway_pct"] == pytest.approx(a[f"ubrk_{dn}"], abs=0.8)  # 0.8 % duty
        assert d["tau_s"] == pytest.approx(a["tau"], rel=0.20)  # 20 %
        assert d["dead_time_s"] == pytest.approx(a["dead_mech"], abs=0.015)  # 15 ms
        assert d["fit_rms_counts"] < 1.5
    # the gravity asymmetry is reported: the - direction is the one the load helps
    assert any("helped by the load" in f for f in plant_res["flags"])


def test_stall_pwm_equals_kp_times_stall_error(plant_res, ps_run):
    for p in BASE:
        for dn in ("pos", "neg"):
            e = plant_res["channels"][p][dn]
            if e.get("stall_pwm_pct") is None:
                continue
            assert e["kp_x_stall_error_pct"] == pytest.approx(e["stall_pwm_pct"], abs=0.3)
            # a P loop creeping toward its deadband stops just above it
            assert e["deadband_pct"] - 0.5 < e["stall_pwm_pct"] < e["deadband_pct"] + 1.5


def test_pstep_metrics_match_truth(ps_run):
    res = pstep.compute(ps_run.run, ps_run.opts)
    truth = {(t["prefix"], round(t["t_on"], 1)): t for t in ps_run.truth["position_steps"]}
    n = 0
    for p in BASE:
        ch = res["channels"][p]
        assert ch["loop"]["kp_pct_per_count"] == pytest.approx(ps_run.sim.kp, rel=0.03)
        assert ch["loop"]["looks_like"] == "P"
        for s in ch["steps"]:
            t = truth.get((p, round(s["t_on"], 1)))
            assert t is not None, f"no truth for step at {s['t_on']}"
            assert s["ss_error_counts"] == pytest.approx(t["ss_error_counts"], abs=2.5)  # counts
            assert s["moved"] == t["ever_moved"]
            assert s["stalled"] == t["stalled"]
            assert s["overshoot_counts"] == pytest.approx(t["overshoot_counts"], abs=2)
            assert s["pwm_rest_pct"] == pytest.approx(t["pwm_rest_pct"], abs=0.5)  # % duty
            assert s["i_rest_A"] == pytest.approx(t["i_rest_A"], abs=0.1)  # A
            if t["rise_actual_s"] and s["rise_actual_s"]:
                # the 90 % point sits on a slow approach: one count of encoder noise moves it ~0.1 s
                assert s["rise_actual_s"] == pytest.approx(t["rise_actual_s"], rel=0.15, abs=0.1)
            n += 1
    assert n == 4 * 12


def test_pstep_detects_overshoot_and_pi(make):
    r = make("pstep", label="pi", kp=0.5, ki=0.8)
    res = pstep.compute(r.run, r.opts)
    truth = {(t["prefix"], round(t["t_on"], 1)): t for t in r.truth["position_steps"]}
    over = []
    for p in BASE:
        ch = res["channels"][p]
        assert ch["loop"]["looks_like"] == "PI"
        assert ch["loop"]["kp_pct_per_count"] == pytest.approx(0.5, rel=0.03)
        assert ch["loop"]["ki_pct_per_count_s"] == pytest.approx(0.8, rel=0.05)
        for s in ch["steps"]:
            t = truth[(p, round(s["t_on"], 1))]
            assert s["overshoot_counts"] == pytest.approx(t["overshoot_counts"], abs=2.5)
            over.append(t["overshoot_counts"])
    assert max(over) > 3  # the case actually exercises overshoot


def test_latency_splits_can_from_mechanical(ps_run):
    res = latency.compute(ps_run.run, ps_run.opts)
    for p in BASE:
        d = res["channels"][p]
        assert d["can_latency_s"] == pytest.approx(ps_run.sim.can_latency, abs=0.004)  # 4 ms
        assert d["mechanical_dead_time_s"] == pytest.approx(ps_run.truth["axes"][p]["dead_mech"], abs=0.015)
    # at these speeds the chassis barely turns: the IMU lag must be flagged, not reported as fact
    se = res["summary"]["enc_to_imu_lag_se_s"]
    assert se is None or se > 0.008
    assert any("encoder->IMU lag uncertain" in f for f in res["flags"])


def test_latency_imu_lag_with_fast_moves(make):
    # open-loop steps at 3x the duty cap: the chassis turns ~1 deg/s
    r = make("steps", label="steps-x3", duty_scale=3.0)
    res = latency.compute(r.run, r.opts)
    s = res["summary"]
    assert s["enc_to_imu_lag_se_s"] < 0.008
    assert abs(s["enc_to_imu_lag_s"] - r.sim.imu_delay) < 3 * s["enc_to_imu_lag_se_s"] + 0.003


def test_rest_reports_creep_and_drift(make):
    r = make("rest")
    res = rest.compute(r.run, r.opts)
    t = r.truth["rest"]
    for p, d in res["channels"].items():
        assert d["creep_counts_per_min"] == pytest.approx(t["creep_counts_per_min"][p], abs=0.05)  # counts/min
    for k, tk in (("pitch_g", "pitch_deg_per_min"), ("roll_g", "roll_deg_per_min")):
        i = res["imu"][k]
        assert abs(i["drift_deg_per_min"] - t[tk]) < 3 * i["drift_se_deg_per_min"] + 2e-4
        assert i["noise_deg"] == pytest.approx(np.degrees(r.sim.sd_grav / 9.81), rel=0.25)
    assert sum("creeps" in f for f in res["flags"]) == 4  # the four loaded EHAs, not the carriages
