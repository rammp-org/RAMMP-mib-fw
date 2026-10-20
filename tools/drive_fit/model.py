"""Python twin of components/DriveResponse, in wheel units, plus an input delay.

Working in wheel rad/s means the fit does not depend on wheel geometry:
  linear wheel speed = (right_forward + left_forward) / 2
  turn wheel speed   = (right_forward - left_forward) / 2   (+ = counter-clockwise)
"""

from dataclasses import dataclass, fields

import numpy as np


@dataclass
class Params:
    forward: float = 14.5  # linear wheel speed at full forward stick, rad/s
    reverse: float = 10.5  # linear wheel speed at full reverse stick, rad/s
    turn: float = 5.0  # turn wheel speed at full side stick, rad/s
    linear_accel: float = 8.0  # rad/s^2 while speeding up
    linear_decel: float = 12.0  # rad/s^2 while slowing or reversing
    turn_accel: float = 6.0
    turn_decel: float = 9.0
    delay: float = 0.15  # s of dead time from joystick topic to wheels (not reproduced in firmware)
    turn_reduction: float = 0.15  # turn scales by (1 - turn_reduction * |linear stick|)

    def to_array(self):
        return np.array([getattr(self, f.name) for f in fields(self)])

    @classmethod
    def from_array(cls, values):
        return cls(*map(float, values))


def ramp(current, target, accel, decel, dt):
    """Same rule as DriveResponse: speeding up in the same direction uses accel, else decel."""
    speeding_up = current * target >= 0.0 and abs(target) > abs(current)
    step = (accel if speeding_up else decel) * dt
    return current + np.clip(target - current, -step, step)


def simulate(p: Params, t, linear_stick, side_stick):
    """Return (n, 2) array of [linear, turn] wheel speed, rad/s."""
    out = np.zeros((len(t), 2))
    linear = turn = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        j = np.searchsorted(t, t[i] - p.delay, side="right") - 1
        fwd = linear_stick[j] if j >= 0 else 0.0
        side = side_stick[j] if j >= 0 else 0.0
        target_linear = fwd * (p.forward if fwd > 0 else p.reverse)
        # Stick + right is a clockwise (negative) turn.
        target_turn = -side * p.turn * (1.0 - p.turn_reduction * abs(fwd))
        linear = ramp(linear, target_linear, p.linear_accel, p.linear_decel, dt)
        turn = ramp(turn, target_turn, p.turn_accel, p.turn_decel, dt)
        out[i] = linear, turn
    return out
