"""Replay a bag's joystick through the firmware drive path and compare to its wheels.

Builds replay/ with CMake against the real MIBconfig.hpp, DriveController's
drive_profile.hpp, and espp's cross-platform library (fetched by CPM at the
version the firmware uses), so it checks the code and numbers that ship. The
first build downloads and compiles espp, which takes a few minutes.

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


def build_replay(build_dir: Path) -> Path:
    if shutil.which("cmake") is None:
        raise RuntimeError("cmake not found; install CMake 3.21+")
    subprocess.run(["cmake", "-S", str(HERE / "replay"), "-B", str(build_dir),
                    "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "--target", "replay_drive_profile",
                    "--parallel"], check=True)
    return build_dir / "replay_drive_profile"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--profile", choices=["low", "normal", "high"], default="normal")
    parser.add_argument("--delay", type=float, default=0.163, help="s")
    parser.add_argument("--wheel-diameter", type=float, default=0.254, help="m")
    parser.add_argument("--wheel-separation", type=float, default=0.558, help="m")
    parser.add_argument("--out", type=Path, default=Path("drive_fit_out"))
    parser.add_argument("--build-dir", type=Path, default=HERE / "build",
                        help="CMake build dir for the replay; reused between runs")
    args = parser.parse_args()

    log = read_drive_log(args.bag)
    linear, turn = wheel_components(log, invert_left=True, invert_right=False)

    binary = build_replay(args.build_dir)
    with tempfile.TemporaryDirectory() as tmp:
        stick_csv = Path(tmp) / "stick.csv"
        np.savetxt(stick_csv, np.c_[log.t, log.linear_stick, log.side_stick], delimiter=",")
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
