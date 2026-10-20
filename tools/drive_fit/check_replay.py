"""Replay a bag's joystick through the firmware drive path and compare to its wheels.

Builds replay_drive_profile.cpp with the host C++ compiler against the real
MIBconfig.hpp, DriveController's drive_profile.hpp, and the espp
TrajectoryPlanner source from managed_components (run idf.py build or
reconfigure once to fetch it), so it checks the code and numbers that ship.

Usage:
  python check_replay.py BAG_DIR [--profile normal] [--delay 0.163] [--out OUT_DIR]

--delay is the recorded chair's dead time from joystick topic to wheels (from
fit_drive_response.py). It lines the replay up with the recording; the firmware
does not add it.
"""

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from bag_reader import read_drive_log
from fit_drive_response import wheel_components
from plotting import plt, save, title

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def build_replay(workdir: Path) -> Path:
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        raise RuntimeError("no host C++ compiler found")
    planner = REPO / "managed_components/espp__trajectory_planner"
    if not planner.exists():
        raise RuntimeError(f"{planner} missing; run idf.py reconfigure to fetch it")
    binary = workdir / "replay_drive_profile"
    subprocess.run([
        compiler, "-std=c++20", "-O2", "-Wall", "-Wextra",
        "-I", str(HERE / "host_stubs"),
        "-I", str(REPO / "components/DriveController/include"),
        "-I", str(REPO / "components/MIBConfig/include"),
        "-I", str(planner / "include"),
        str(HERE / "replay_drive_profile.cpp"),
        str(planner / "src/trajectory_planner.cpp"),
        "-o", str(binary),
    ], check=True)
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--profile", choices=["low", "normal", "high"], default="normal")
    parser.add_argument("--delay", type=float, default=0.163, help="s")
    parser.add_argument("--wheel-diameter", type=float, default=0.254, help="m")
    parser.add_argument("--wheel-separation", type=float, default=0.558, help="m")
    parser.add_argument("--out", type=Path, default=Path("drive_fit_out"))
    args = parser.parse_args()

    log = read_drive_log(args.bag)
    linear, turn = wheel_components(log, invert_left=True, invert_right=False)

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        stick_csv = tmp / "stick.csv"
        np.savetxt(stick_csv, np.c_[log.t, log.linear_stick, log.side_stick], delimiter=",")
        binary = build_replay(tmp)
        result = subprocess.run([str(binary), str(stick_csv), args.profile, str(args.delay)],
                                check=True, capture_output=True, text=True)
    replay = np.loadtxt(result.stdout.splitlines(), delimiter=",")

    radius = args.wheel_diameter / 2.0
    half_track = args.wheel_separation / 2.0
    replay_t = replay[:, 0]
    replay_linear = replay[:, 1] / radius
    replay_turn = replay[:, 2] * half_track / radius
    measured_linear = np.interp(replay_t, log.t, linear)
    measured_turn = np.interp(replay_t, log.t, turn)

    rms_linear = np.sqrt(np.mean((replay_linear - measured_linear) ** 2))
    rms_turn = np.sqrt(np.mean((replay_turn - measured_turn) ** 2))
    print(f"Firmware '{args.profile}' profile vs bag: RMS error linear {rms_linear:.3f} rad/s, "
          f"turn {rms_turn:.3f} rad/s (peak linear {np.abs(linear).max():.1f} rad/s)")

    args.out.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    for a, measured, replayed, label in [
        (axes[0], linear, replay_linear, "Linear wheel speed, rad/s"),
        (axes[1], turn, replay_turn, "Turn wheel speed, rad/s"),
    ]:
        a.plot(log.t, measured, label="recorded chair")
        a.plot(replay_t, replayed, label=f"firmware {args.profile}")
        a.set_xlabel("Time, s")
        a.set_ylabel(label)
        a.legend(loc="upper right")
    title(axes[0], "Firmware replay", "Recorded wheel speeds vs the firmware drive path fed the same joystick input")
    save(fig, args.out / "replay.png")
    print(f"Wrote {args.out / 'replay.png'}")


if __name__ == "__main__":
    main()
