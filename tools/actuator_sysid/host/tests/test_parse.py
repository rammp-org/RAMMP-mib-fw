"""Parser, time base and attitude: the foundations every analysis stands on."""

import numpy as np

from sysid import attitude, plant, runlog, timebase, wire


def test_counts_match_what_the_simulator_injected(make):
    r = make("pstep")
    c, s = r.truth["counts"], r.run.stats
    assert s["rows"] == c["rows_written"]
    assert s["garbled"] == c["rows_garbled"]
    assert s["stray"] == c["stray_lines"]
    assert s["log"] == c["log_lines"]
    assert s["header_changes"] == c["header_changes"] == 1
    assert s["cfg"] == 9 and set(r.run.cfg) == {"1", "2", "3"}
    # empty fields are NaN, not zero
    n_nan = sum(int(np.isnan(v).sum()) for k, v in r.run.cols.items() if k.startswith("imu_"))
    assert n_nan > 0


def test_reordered_header_is_parsed_by_name(make):
    r = make("pstep")
    enc = r.run.col("n1c1_enc")
    ok = np.isfinite(enc)
    # a column mix-up at the header switch would show as a jump of hundreds of counts
    assert np.nanmax(np.abs(np.diff(enc[ok]))) < 15
    g = np.linalg.norm(attitude.imu_vec(r.run, "gr"), axis=1)
    assert abs(np.nanmedian(g) - 9.81) < 0.05


def test_timebase_counts_every_missing_row(make):
    r = make("pstep")
    tb = timebase.check(r.run)
    c = r.truth["counts"]
    assert tb["dropped_rows_est"] == c["rows_dropped"] + c["rows_garbled"]
    # jitter is per-row timestamp noise; a period difference carries it twice
    assert abs(tb["jitter_std_us"] - np.sqrt(2) * r.sim.jitter_us) < 0.2 * np.sqrt(2) * r.sim.jitter_us
    assert tb["non_monotonic"] == 0


def test_clock_reset_and_wrap_are_stitched():
    raw1, _, _ = plant.simulate("passive", plant.Sim(seed=3))
    raw2, _, _ = plant.simulate("passive", plant.Sim(seed=4, t_us0=1000))  # replug: clock from ~zero
    run = runlog.parse_bytes(raw1 + b"\x00\xff garbage from the ROM at another baud\n" + raw2)
    assert run.stats["resets"] == 1
    assert np.all(np.diff(run.t_us) > 0)
    assert any("segment boundary" in e.text for e in run.events)
    # a 32-bit firmware clock wrapping
    rows = ["#cols:t_us,x"] + [f"D,{(2**32 - 30000 + 10000 * k) % 2**32},{k}" for k in range(6)]
    run = runlog.parse_bytes(("\n".join(rows) + "\n").encode())
    assert run.stats["wraps"] == 1 and run.stats["resets"] == 0
    assert np.allclose(np.diff(run.t_us), 10000)


def test_garbage_only_costs_the_damaged_lines():
    text = ("#cols:t_us,a,b\nD,100,1,2\nD,200,3,\xff\xfe4\nD,3\x0000,5,6\nD,400,,8\nhello stray\n"
            "#evt,250,step 1 1 10 500\n#cfg,1,kp,0x2000,1,15\nD,500,9,10,11\n")
    run = runlog.parse_bytes(text.encode("latin-1"))
    assert list(run.t_us) == [100, 400]
    assert np.isnan(run.col("a")[1]) and run.col("b")[1] == 8
    assert run.stats["garbled"] == 3 and run.stats["stray"] == 1 and run.stats["cfg"] == 1


def test_node2_motion_is_refused():
    for line in ("arm 2 1", "pstep 2 2 -100", "step 2 1 10 500", "PSTEP n2 1 5"):
        assert wire.motion_refusal(line)
    for line in ("arm 1 1", "pstep 3 2 100", "dump 2", "clearfaults 2", "stop", "mark pstep 2 is a note"):
        assert wire.motion_refusal(line) is None


def test_level_reference_recovers_the_mounting(make):
    r = make("passive")
    m = attitude.level_reference(r.run)
    roll, pitch, _ = m.rpy()
    assert abs(roll - r.sim.mount_rpy[0]) < 0.05
    assert abs(pitch - r.sim.mount_rpy[1]) < 0.05
    att = attitude.attitude(r.run, m)
    assert abs(np.nanmean(att["pitch_g"])) < 0.02 and abs(np.nanmean(att["roll_g"])) < 0.02
    # the quaternion, read the right way round, agrees with the gravity vector
    assert att["quat_convention"]["convention"] == "body_to_world"
    assert abs(np.nanmean(att["pitch_q"] - att["pitch_g"])) < 0.05
