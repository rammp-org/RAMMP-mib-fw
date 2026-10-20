"""Python twin of the MIB drive path, in wheel units, plus an input delay.

Mirrors DriveController::set_target (joystick shaping) feeding espp::TrajectoryPlanner
in trapezoidal mode, which is how the MIB profiles configure it:

  linear  = stick < 0 ? stick * reverse_scale : stick
  target  = (linear * max_linear, side * max_angular), scaled down together if
            |v * w| exceeds the centripetal limit
  each axis moves toward its target at the driving_profile acceleration, or the
  stopping_profile acceleration when the target is exactly (0, 0) (the joystick's
  center deadzone delivers exact zeros; the Luci recording already has them)

Working in wheel rad/s means the fit does not depend on wheel geometry:
  linear wheel speed = (right_forward + left_forward) / 2
  turn wheel speed   = (right_forward - left_forward) / 2   (+ = counter-clockwise)
The centripetal limit is in chassis units, so simulate() takes the geometry for it.
"""

from dataclasses import dataclass, fields

import numpy as np


@dataclass
class Params:
    forward: float = 14.5  # linear wheel speed at full forward stick, rad/s
    turn: float = 5.0  # turn wheel speed at full side stick, rad/s
    linear_accel: float = 8.0  # driving_profile, rad/s^2
    turn_accel: float = 6.0
    linear_stop_accel: float = 12.0  # stopping_profile, rad/s^2
    turn_stop_accel: float = 9.0
    delay: float = 0.15  # s of dead time from joystick topic to wheels (not reproduced in firmware)
    reverse_scale: float = 0.72  # reverse speed as a fraction of forward

    def to_array(self):
        return np.array([getattr(self, f.name) for f in fields(self)])

    @classmethod
    def from_array(cls, values):
        return cls(*map(float, values))


def simulate(p: Params, t, linear_stick, side_stick, centripetal_limit=0.0,
             wheel_radius=0.127, half_track=0.279):
    """Return (n, 2) array of [linear, turn] wheel speed, rad/s.

    centripetal_limit is in m/s^2; 0 disables it, matching the planner.
    """
    out = np.zeros((len(t), 2))
    linear = turn = 0.0
    turn_to_radps = wheel_radius / half_track
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        j = np.searchsorted(t, t[i] - p.delay, side="right") - 1
        stick = linear_stick[j] if j >= 0 else 0.0
        side = side_stick[j] if j >= 0 else 0.0
        shaped_linear = stick * p.reverse_scale if stick < 0 else stick
        target_linear = shaped_linear * p.forward
        # Stick + right is a clockwise (negative) turn.
        target_turn = -side * p.turn
        if centripetal_limit > 0.0:
            lateral = abs(target_linear * wheel_radius * target_turn * turn_to_radps)
            if lateral > centripetal_limit:
                scale = np.sqrt(centripetal_limit / lateral)
                target_linear *= scale
                target_turn *= scale
        stopping = target_linear == 0.0 and target_turn == 0.0
        linear_rate = p.linear_stop_accel if stopping else p.linear_accel
        turn_rate = p.turn_stop_accel if stopping else p.turn_accel
        linear += np.clip(target_linear - linear, -linear_rate * dt, linear_rate * dt)
        turn += np.clip(target_turn - turn, -turn_rate * dt, turn_rate * dt)
        out[i] = linear, turn
    return out
