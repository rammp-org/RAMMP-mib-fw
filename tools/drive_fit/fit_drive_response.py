"""Fit DriveResponse limits to a bag of an existing chair driven by joystick.

Usage:
  python fit_drive_response.py BAG_DIR [--out OUT_DIR] [--wheel-diameter 0.254] [--wheel-separation 0.558]

Prints a DriveProfileConfig block for components/MIBConfig/include/MIBconfig.hpp
and writes fit.png and params.json to OUT_DIR.
"""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from bag_reader import read_drive_log
from model import Params, simulate
from plotting import plt, save, title


def wheel_components(log, invert_left, invert_right):
    left = -log.left_wheel if invert_left else log.left_wheel
    right = -log.right_wheel if invert_right else log.right_wheel
    return (right + left) / 2.0, (right - left) / 2.0


def fit(log, linear, turn, start: Params):
    def cost(values):
        p = Params.from_array(values)
        if min(values) < 0 or p.turn_reduction >= 1.0:
            return 1e9
        out = simulate(p, log.t, log.linear_stick, log.side_stick)
        return np.mean((out[:, 0] - linear) ** 2 + (out[:, 1] - turn) ** 2)

    result = minimize(cost, start.to_array(), method="Nelder-Mead",
                      options={"maxiter": 4000, "xatol": 1e-3, "fatol": 1e-5})
    return Params.from_array(result.x)


def config_block(p: Params, name, wheel_diameter, wheel_separation):
    radius = wheel_diameter / 2.0
    turn_scale = radius / (wheel_separation / 2.0)  # turn wheel rad/s -> chassis rad/s
    return f"""inline constexpr DriveProfileConfig {name}{{
    .max_forward_mps = {p.forward * radius:.3f}f,
    .max_reverse_mps = {p.reverse * radius:.3f}f,
    .max_angular_radps = {p.turn * turn_scale:.3f}f,
    .turn_reduction = {p.turn_reduction:.3f}f,
    .linear_accel_mps2 = {p.linear_accel * radius:.3f}f,
    .linear_decel_mps2 = {p.linear_decel * radius:.3f}f,
    .angular_accel_radps2 = {p.turn_accel * turn_scale:.3f}f,
    .angular_decel_radps2 = {p.turn_decel * turn_scale:.3f}f,
}};"""


def plot_fit(log, linear, turn, out, path):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(log.t, log.linear_stick, label="forward/back")
    axes[0].plot(log.t, log.side_stick, label="left/right")
    axes[0].set_ylabel("Joystick, normalized")
    axes[1].plot(log.t, linear, label="measured")
    axes[1].plot(log.t, out[:, 0], label="fitted model")
    axes[1].set_ylabel("Linear wheel speed, rad/s")
    axes[2].plot(log.t, turn, label="measured")
    axes[2].plot(log.t, out[:, 1], label="fitted model")
    axes[2].set_ylabel("Turn wheel speed, rad/s")
    for a in axes:
        a.set_xlabel("Time, s")
        a.legend(loc="upper right")
    title(axes[0], "Drive response fit", "Joystick input vs measured and modeled wheel speed")
    save(fig, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or .db3 file")
    parser.add_argument("--out", type=Path, default=Path("drive_fit_out"))
    parser.add_argument("--profile-name", default="drive_profile_normal")
    parser.add_argument("--wheel-diameter", type=float, default=0.254, help="m")
    parser.add_argument("--wheel-separation", type=float, default=0.558, help="m")
    parser.add_argument("--no-invert-left", dest="invert_left", action="store_false",
                        help="by default the left wheel is negated (it reads negative driving forward)")
    parser.add_argument("--invert-right", action="store_true")
    args = parser.parse_args()

    log = read_drive_log(args.bag)
    linear, turn = wheel_components(log, args.invert_left, args.invert_right)
    p = fit(log, linear, turn, Params())
    out = simulate(p, log.t, log.linear_stick, log.side_stick)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "params.json").write_text(json.dumps(asdict(p), indent=2))
    plot_fit(log, linear, turn, out, args.out / "fit.png")

    print("Fitted parameters (wheel units):")
    for key, value in asdict(p).items():
        print(f"  {key:15s} {value:8.3f}")
    print(f"RMS error: linear {np.sqrt(np.mean((out[:, 0] - linear) ** 2)):.3f} rad/s, "
          f"turn {np.sqrt(np.mean((out[:, 1] - turn) ** 2)):.3f} rad/s, "
          f"peak linear {np.abs(linear).max():.1f} rad/s")
    print(f"\nMIBconfig.hpp block ({args.wheel_diameter} m wheels, {args.wheel_separation} m separation):")
    print(config_block(p, args.profile_name, args.wheel_diameter, args.wheel_separation))
    print(f"\nWrote {args.out / 'fit.png'} and {args.out / 'params.json'}")


if __name__ == "__main__":
    main()
