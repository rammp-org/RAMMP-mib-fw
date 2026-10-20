# drive_fit

Tools for fitting the MIB drive response (`components/DriveResponse`) to a
recording of another chair, and for checking the firmware against that
recording. The NORMAL profile in `MIBconfig.hpp` came from these tools and the
R-Net chair's `wheel_velocities` bag.

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
`drive_fit_out/fit.png` (measured vs model) and `params.json`.

Wheel geometry only enters the conversion to m/s and rad/s. Pass
`--wheel-diameter` and `--wheel-separation` if the recorded chair differs from
the MIB defaults (0.254 m, 0.558 m).

## Check the firmware

```sh
.venv/bin/python tools/drive_fit/check_replay.py path/to/bag_dir --profile normal --delay 0.199
```

This compiles `replay_drive_response.cpp` with your host C++ compiler against
the real `DriveResponse` and `MIBconfig.hpp`, feeds it the bag's joystick at
the firmware's 20 ms update rate, and reports the RMS error against the recorded
wheel speeds. It writes `drive_fit_out/replay.png`.

`--delay` is the recorded chair's joystick-to-wheel dead time, taken from the
fit. It only lines the replay up with the recording; the firmware doesn't add
any delay.

## The model

Per update, in `DriveResponse`:

1. Target linear speed = stick × `max_forward_mps` (or `max_reverse_mps` when the stick is back).
2. Target turn rate = side stick × `max_angular_radps` × (1 − `turn_reduction` × |linear stick|).
3. Each axis moves toward its target at most `accel × dt` when speeding up in
   the same direction, or `decel × dt` when slowing, stopping, or reversing.

`model.py` is the same model in Python, in wheel units, with an added input
delay so it can be fitted to a recording.
