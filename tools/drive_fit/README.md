# drive_fit

Tools for fitting the MIB drive profiles to a recording of another chair, and
for checking the firmware against that recording. A profile is the
`DriveProfileConfig` in `MIBconfig.hpp`, mapped onto `espp::TrajectoryPlanner`
by `components/DriveController/include/drive_profile.hpp`. The NORMAL profile came from
these tools and the R-Net chair's `wheel_velocities` bag.

## What a bag needs

A rosbag2 (sqlite3) with:

| Topic | Type | Used for |
|---|---|---|
| `/luci/joystick_position` | `luci_messages/msg/LuciJoystick` | `forward_back`, `left_right`, -100..100 |
| `/base/joint_states` | `sensor_msgs/msg/JointState` | `drive_wheel_l` / `drive_wheel_r` velocity, rad/s |

No ROS install is needed; `bag_reader.py` decodes the messages directly. Drive
the chair through the whole stick range: full forward, full reverse, spins both
ways, diagonals, and some releases to a stop from speed.

## Setup

```sh
python3 -m venv .venv
.venv/bin/pip install -r tools/drive_fit/requirements.txt
```

The replay check also needs CMake 3.21+ and a C++23 compiler.

## Fit a profile

```sh
.venv/bin/python tools/drive_fit/fit_drive_response.py path/to/bag_dir --profile-name drive_profile_high
```

This prints the fitted parameters in wheel units, the RMS error, and a
`DriveProfileConfig` block to paste into `MIBconfig.hpp`. The model is
trapezoidal, so the block's jerk limits are 0; choose them with the jerk sweep
below. It also writes
`drive_fit_out/fit.png` (measured vs model) and `params.json`. It takes a few
minutes.

The centripetal limit is held at `--centripetal-limit` (default 1.5 m/s²) rather
than fitted, since a fit would just push it above anything the recording reached.

The fit also lists any parameter the recording can't pin down (moving it 30%
barely changes the error). Its fitted value is arbitrary, so set it from other
evidence. On the R-Net bag that's the stopping angular acceleration: the chair
never stops from a pure spin.

Wheel geometry only enters the conversion to m/s and rad/s. Pass
`--wheel-diameter` and `--wheel-separation` if the recorded chair differs from
the MIB defaults (0.254 m, 0.558 m).

## Check the firmware

```sh
.venv/bin/python tools/drive_fit/check_replay.py path/to/bag_dir --profile normal --delay 0.163
```

This builds `replay/` with CMake against the real `MIBconfig.hpp`,
`components/DriveController/include/drive_profile.hpp`, and espp's
cross-platform library, which CPM fetches at the tag the firmware uses
(`ESPP_VERSION` in `replay/CMakeLists.txt`; keep it in step with
`dependencies.lock`). The first build compiles espp and takes a few minutes;
later runs reuse `tools/drive_fit/build/`. It feeds the bag's joystick through
at the planner's 20 ms period and reports the RMS error against the recorded
wheel speeds. It writes `drive_fit_out/replay.png`.

`--delay` is the recorded chair's joystick-to-wheel dead time, taken from the
fit. It only lines the replay up with the recording; the firmware doesn't add
any delay.

## Choose driving jerk

```sh
.venv/bin/python tools/drive_fit/check_replay.py path/to/bag_dir --jerk-sweep
```

Replays with each axis's driving jerk set to accel / ramp time, for ramp times
from 0 (trapezoidal) to 0.75 s, and prints the fit error next to the RMS jerk
of the output. A trapezoidal recording can't tell you the right jerk, so this
shows what each amount of smoothing costs in fidelity. NORMAL uses 0.1 s.

## The model

`espp::TrajectoryPlanner`, every 20 ms, as configured by `drive_profile.hpp`:

1. Target = stick × max velocity, using `max_reverse_linear_velocity` for
   reverse stick, with speed and turn scaled down together if |v·w| exceeds
   `max_centripetal_acceleration_mps2`.
2. Each axis moves toward its target at the driving acceleration (S-curve where
   the profile sets a jerk limit), or the trapezoidal stopping acceleration when
   the target is exactly (0, 0). That relies on the joystick's center deadzone
   sending exact zeros at rest.

`model.py` is the same model in Python, trapezoidal and in wheel units, with an
added input delay so it can be fitted to a recording.
