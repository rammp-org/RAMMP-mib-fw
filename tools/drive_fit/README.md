# drive_fit

Tools for fitting the MIB drive profiles to a recording of another chair, and
for checking the firmware against that recording. A profile is the
`DriveProfileConfig` in `MIBconfig.hpp`: joystick shaping in `DriveController`
plus the `espp::TrajectoryPlanner` limits it feeds. The NORMAL profile came from
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

## Fit a profile

```sh
.venv/bin/python tools/drive_fit/fit_drive_response.py path/to/bag_dir --profile-name drive_profile_high
```

This prints the fitted parameters in wheel units, the RMS error, and a
`DriveProfileConfig` block to paste into `MIBconfig.hpp`. It also writes
`drive_fit_out/fit.png` (measured vs model) and `params.json`. It takes a few
minutes.

The centripetal limit is held at `--centripetal-limit` (default 1.5 m/s²) rather
than fitted, since a fit would just push it above anything the recording reached.
Values the recording barely exercises come out loosely determined; on the R-Net
bag that's the stopping angular acceleration (few stops from a turn).

Wheel geometry only enters the conversion to m/s and rad/s. Pass
`--wheel-diameter` and `--wheel-separation` if the recorded chair differs from
the MIB defaults (0.254 m, 0.558 m).

## Check the firmware

```sh
.venv/bin/python tools/drive_fit/check_replay.py path/to/bag_dir --profile normal --delay 0.163
```

This compiles `replay_drive_profile.cpp` with your host C++ compiler against the
real `MIBconfig.hpp`, `components/DriveController/include/drive_profile.hpp`,
and the espp `TrajectoryPlanner` source in `managed_components/` (run
`idf.py reconfigure` once to fetch it). `host_stubs/` stands in for the espp
logger, task, and timer. It feeds the bag's joystick through at the planner's
20 ms period and reports the RMS error against the recorded wheel speeds. It
writes `drive_fit_out/replay.png`.

`--delay` is the recorded chair's joystick-to-wheel dead time, taken from the
fit. It only lines the replay up with the recording; the firmware doesn't add
any delay.

## The model

Per joystick message, in `DriveController::set_target` (`shape_drive_input`):

1. Each axis inside `config::joystick_deadband` snaps to 0, so a resting stick
   reaches the planner as exactly (0, 0) and its stopping profile engages.
2. Reverse stick is scaled by `reverse_scale`.
3. Turn stick is scaled by (1 − `turn_reduction` × |linear stick|).

Then `espp::TrajectoryPlanner`, every 20 ms:

4. Target = shaped stick × max velocity, with speed and turn scaled down
   together if |v·w| exceeds `max_centripetal_acceleration_mps2`.
5. Each axis moves toward its target at the driving acceleration, or the
   stopping acceleration when the target is (0, 0).

`model.py` is the same model in Python, in wheel units, with an added input
delay so it can be fitted to a recording.
