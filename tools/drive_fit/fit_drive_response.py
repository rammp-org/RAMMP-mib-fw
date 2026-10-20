"""Fit MIB drive profile settings to a bag of an existing chair driven by joystick.

Fits the espp::TrajectoryPlanner limits and DriveController's reverse scale
(see model.py), with the centripetal limit held fixed.

Usage:
  python fit_drive_response.py BAG_DIR [--out OUT_DIR] [--centripetal-limit 1.5]

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


def fit(log, linear, turn, simulate_kwargs, restarts=3):
    def cost(values):
        p = Params.from_array(values)
        if min(values) < 0 or p.reverse_scale > 1.0:
            return 1e9
        out = simulate(p, log.t, log.linear_stick, log.side_stick, **simulate_kwargs)
        return np.mean((out[:, 0] - linear) ** 2 + (out[:, 1] - turn) ** 2)

    # Nelder-Mead stalls in local minima here; restarting from the last result
    # with a fresh simplex reliably gets further.
    values = Params().to_array()
    for _ in range(restarts):
        values = minimize(cost, values, method="Nelder-Mead",
                          options={"maxiter": 4000, "xatol": 1e-3, "fatol": 1e-5}).x
    return Params.from_array(values)


def unidentified(log, linear, turn, p: Params, simulate_kwargs, tolerance=0.002):
    """Names of parameters the recording can't pin down.

    A parameter is unidentified when moving it 30% either way changes the RMS
    error by less than `tolerance` (fractional). Its fitted value is arbitrary;
    set it from other evidence instead.
    """
    def rms(q):
        out = simulate(q, log.t, log.linear_stick, log.side_stick, **simulate_kwargs)
        return np.sqrt(np.mean((out[:, 0] - linear) ** 2 + (out[:, 1] - turn) ** 2))

    base = rms(p)
    names = []
    for field_name, value in asdict(p).items():
        if field_name == "delay":
            continue
        worst = max(rms(Params(**{**asdict(p), field_name: value * factor})) for factor in (0.7, 1.3))
        if worst - base < tolerance * base:
            names.append(field_name)
    return names


def config_block(p: Params, name, wheel_diameter, wheel_separation, centripetal_limit):
    radius = wheel_diameter / 2.0
    turn_scale = radius / (wheel_separation / 2.0)  # turn wheel rad/s -> chassis rad/s
    return f"""inline constexpr DriveProfileConfig {name}{{
    .max_linear_velocity_mps = {p.forward * radius:.3f}f,
    .max_angular_velocity_radps = {p.turn * turn_scale:.3f}f,
    .max_linear_acceleration_mps2 = {p.linear_accel * radius:.3f}f,
    .max_angular_acceleration_radps2 = {p.turn_accel * turn_scale:.3f}f,
    .stopping_linear_acceleration_mps2 = {p.linear_stop_accel * radius:.3f}f,
    .stopping_angular_acceleration_radps2 = {p.turn_stop_accel * turn_scale:.3f}f,
    .max_centripetal_acceleration_mps2 = {centripetal_limit:.3f}f,
    .reverse_scale = {p.reverse_scale:.3f}f,
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
    title(axes[0], "Drive profile fit", "Joystick input vs measured and modeled wheel speed")
    save(fig, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or .db3 file")
    parser.add_argument("--out", type=Path, default=Path("drive_fit_out"))
    parser.add_argument("--profile-name", default="drive_profile_normal")
    parser.add_argument("--wheel-diameter", type=float, default=0.254, help="m")
    parser.add_argument("--wheel-separation", type=float, default=0.558, help="m")
    parser.add_argument("--centripetal-limit", type=float, default=1.5,
                        help="m/s^2, held fixed during the fit; 0 disables it")
    parser.add_argument("--no-invert-left", dest="invert_left", action="store_false",
                        help="by default the left wheel is negated (it reads negative driving forward)")
    parser.add_argument("--invert-right", action="store_true")
    args = parser.parse_args()

    log = read_drive_log(args.bag)
    linear, turn = wheel_components(log, args.invert_left, args.invert_right)
    simulate_kwargs = dict(centripetal_limit=args.centripetal_limit,
                           wheel_radius=args.wheel_diameter / 2.0,
                           half_track=args.wheel_separation / 2.0)
    p = fit(log, linear, turn, simulate_kwargs)
    out = simulate(p, log.t, log.linear_stick, log.side_stick, **simulate_kwargs)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "params.json").write_text(json.dumps(asdict(p), indent=2))
    plot_fit(log, linear, turn, out, args.out / "fit.png")

    print("Fitted parameters (wheel units):")
    for key, value in asdict(p).items():
        print(f"  {key:15s} {value:8.3f}")
    print(f"RMS error: linear {np.sqrt(np.mean((out[:, 0] - linear) ** 2)):.3f} rad/s, "
          f"turn {np.sqrt(np.mean((out[:, 1] - turn) ** 2)):.3f} rad/s, "
          f"peak linear {np.abs(linear).max():.1f} rad/s")
    loose = unidentified(log, linear, turn, p, simulate_kwargs)
    if loose:
        print(f"\nUnidentified by this recording (±30% changes RMS error <0.2%): {', '.join(loose)}."
              "\nTheir fitted values are arbitrary; set them from other evidence.")
    print(f"\nMIBconfig.hpp block ({args.wheel_diameter} m wheels, {args.wheel_separation} m separation):")
    print(config_block(p, args.profile_name, args.wheel_diameter, args.wheel_separation,
                       args.centripetal_limit))
    print(f"\nWrote {args.out / 'fit.png'} and {args.out / 'params.json'}")


if __name__ == "__main__":
    main()
