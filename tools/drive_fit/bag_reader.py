"""Read joystick input and wheel velocities from a ROS 2 sqlite3 bag.

No ROS install needed: the CDR messages are decoded by hand. Supports the two
topics the R-Net chair recorded:

  /luci/joystick_position  luci_messages/msg/LuciJoystick  (forward_back, left_right in -100..100)
  /base/joint_states       sensor_msgs/msg/JointState      (drive_wheel_l / drive_wheel_r velocity, rad/s)
"""

import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

JOYSTICK_TOPIC = "/luci/joystick_position"
JOINT_STATE_TOPIC = "/base/joint_states"


class _CdrReader:
    """Little-endian CDR reader. Alignment is relative to the 4-byte encapsulation header."""

    def __init__(self, data: bytes):
        self.data = data
        self.offset = 4

    def _align(self, size: int):
        self.offset += (-(self.offset - 4)) % size

    def _unpack(self, fmt: str, size: int):
        self._align(size)
        value = struct.unpack_from(fmt, self.data, self.offset)[0]
        self.offset += size
        return value

    def int32(self) -> int:
        return self._unpack("<i", 4)

    def uint32(self) -> int:
        return self._unpack("<I", 4)

    def float64(self) -> float:
        return self._unpack("<d", 8)

    def string(self) -> str:
        length = self.uint32()
        value = self.data[self.offset : self.offset + length - 1].decode()
        self.offset += length
        return value

    def float64_array(self) -> list[float]:
        return [self.float64() for _ in range(self.uint32())]


@dataclass
class DriveLog:
    """Joystick and wheel speed on a common time base (the joint-state timestamps)."""

    t: np.ndarray  # s, from the first joint-state sample
    linear_stick: np.ndarray  # -1..1, + forward
    side_stick: np.ndarray  # -1..1, + right
    left_wheel: np.ndarray  # rad/s, as reported (sign depends on motor mounting)
    right_wheel: np.ndarray  # rad/s, as reported


def _find_db3(bag: Path) -> Path:
    if bag.suffix == ".db3":
        return bag
    files = sorted(bag.glob("*.db3"))
    if not files:
        raise FileNotFoundError(f"no .db3 file in {bag}")
    return files[0]


def read_drive_log(bag: Path, left_joint="drive_wheel_l", right_joint="drive_wheel_r") -> DriveLog:
    con = sqlite3.connect(_find_db3(Path(bag)))
    topic_ids = {name: tid for tid, name in con.execute("select id, name from topics")}

    def rows(topic):
        if topic not in topic_ids:
            raise KeyError(f"topic {topic} not in bag; have {sorted(topic_ids)}")
        return con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp",
            (topic_ids[topic],),
        ).fetchall()

    joystick = []
    for stamp, data in rows(JOYSTICK_TOPIC):
        r = _CdrReader(data)
        forward_back, left_right = r.int32(), r.int32()
        joystick.append((stamp * 1e-9, forward_back / 100.0, left_right / 100.0))
    joystick = np.array(joystick)

    wheel_t, wheels = [], []
    for stamp, data in rows(JOINT_STATE_TOPIC):
        r = _CdrReader(data)
        r.int32(), r.uint32(), r.string()  # header stamp, frame_id
        names = [r.string() for _ in range(r.uint32())]
        r.float64_array()  # position
        velocity = r.float64_array()
        wheel_t.append(stamp * 1e-9)
        wheels.append((velocity[names.index(left_joint)], velocity[names.index(right_joint)]))
    wheel_t = np.array(wheel_t)
    wheels = np.array(wheels)

    # Hold the most recent joystick sample at each wheel sample.
    idx = np.searchsorted(joystick[:, 0], wheel_t, side="right") - 1
    held = joystick[np.clip(idx, 0, None)]
    held[idx < 0, 1:] = 0.0

    return DriveLog(
        t=wheel_t - wheel_t[0],
        linear_stick=held[:, 1],
        side_stick=held[:, 2],
        left_wheel=wheels[:, 0],
        right_wheel=wheels[:, 1],
    )
