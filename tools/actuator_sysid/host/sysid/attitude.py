"""Chassis pitch and roll from the BNO055's gravity vector and from its quaternion.

Frames (REP-103, as the chair's ROS 2 stack uses): chassis x forward, y left, z up.
pitch + = nose down, roll + = left side up (right side down). Angles in degrees.

The board is mounted at some rotation in the chassis. R_cb maps board coordinates
into chassis coordinates (its columns are the board axes seen from the chassis).
It comes from --mount roll,pitch,yaw (degrees, board relative to chassis, ZYX),
refined by a level reference: a capture with the chair standing on a level floor,
which removes whatever roll and pitch the board has in its mounting. A level
reference cannot see yaw, so a board turned 90 degrees still needs --mount.

The BNO055 gravity vector reads +9.8 along the board's up axis when level
(it is the accelerometer's specific force, not the pull of gravity).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


def rpy_to_R(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R) -> tuple[float, float, float]:
    pitch = np.degrees(np.arcsin(np.clip(-R[2, 0], -1, 1)))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return float(roll), float(pitch), float(yaw)


def quat_to_R(q: np.ndarray) -> np.ndarray:
    """(N,4) w,x,y,z -> (N,3,3) rotation taking body coordinates to world."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """(3,3) -> w,x,y,z."""
    m = R
    tr = np.trace(m)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


@dataclass
class Mount:
    R_cb: np.ndarray = field(default_factory=lambda: np.eye(3))
    source: str = "identity"
    nominal_rpy: tuple = (0.0, 0.0, 0.0)

    @classmethod
    def from_rpy(cls, rpy, source="--mount"):
        return cls(rpy_to_R(*rpy), source, tuple(rpy))

    def rpy(self):
        return R_to_rpy(self.R_cb)

    def to_json(self) -> dict:
        r, p, y = self.rpy()
        return {"R_cb": self.R_cb.round(9).tolist(), "mount_rpy_deg": [round(r, 4), round(p, 4), round(y, 4)],
                "nominal_rpy_deg": list(self.nominal_rpy), "source": self.source}

    @classmethod
    def from_json(cls, d: dict, source: str):
        return cls(np.array(d["R_cb"], dtype=float), source, tuple(d.get("nominal_rpy_deg", (0, 0, 0))))


def to_chassis(v_board: np.ndarray, mount: Mount) -> np.ndarray:
    """(N,3) board-frame vectors -> chassis frame."""
    return v_board @ mount.R_cb.T


def imu_vec(run, prefix: str) -> np.ndarray | None:
    """(N,3) from imu_<prefix>x/y/z columns, or None."""
    cols = [run.col(f"imu_{prefix}{a}") for a in "xyz"]
    if any(c is None for c in cols):
        return None
    return np.column_stack(cols)


def from_gravity(g_board: np.ndarray, mount: Mount) -> tuple[np.ndarray, np.ndarray]:
    f = to_chassis(g_board, mount)
    pitch = np.degrees(np.arctan2(-f[:, 0], np.hypot(f[:, 1], f[:, 2])))
    roll = np.degrees(np.arctan2(f[:, 1], f[:, 2]))
    return pitch, roll


def _quat_cols(run):
    cols = [run.col(f"imu_q{a}") for a in "wxyz"]
    if any(c is None for c in cols):
        return None
    return np.column_stack(cols)


def from_quat(q: np.ndarray, mount: Mount, convention: str = "body_to_world") -> tuple[np.ndarray, np.ndarray]:
    ok = np.all(np.isfinite(q), axis=1) & (np.linalg.norm(np.nan_to_num(q), axis=1) > 0.5)
    pitch = np.full(len(q), np.nan)
    roll = np.full(len(q), np.nan)
    if not ok.any():
        return pitch, roll
    R_wb = quat_to_R(q[ok])
    if convention == "world_to_body":
        R_wb = np.transpose(R_wb, (0, 2, 1))
    R_wc = R_wb @ mount.R_cb.T
    pitch[ok] = np.degrees(np.arcsin(np.clip(-R_wc[:, 2, 0], -1, 1)))
    roll[ok] = np.degrees(np.arctan2(R_wc[:, 2, 1], R_wc[:, 2, 2]))
    return pitch, roll


def quat_convention(run) -> dict:
    """Which way round the quaternion is: the one whose 'up' agrees with the gravity vector."""
    q = _quat_cols(run)
    g = imu_vec(run, "gr")
    if q is None or g is None:
        return {"convention": "body_to_world", "agreement_deg": None}
    ok = np.all(np.isfinite(q), axis=1) & np.all(np.isfinite(g), axis=1)
    ok &= np.linalg.norm(np.nan_to_num(q), axis=1) > 0.5
    if ok.sum() < 5:
        return {"convention": "body_to_world", "agreement_deg": None}
    idx = np.flatnonzero(ok)[:: max(1, ok.sum() // 2000)]
    R = quat_to_R(q[idx])
    gu = g[idx] / np.linalg.norm(g[idx], axis=1, keepdims=True)
    res = {}
    for conv, up in (("body_to_world", R[:, 2, :]), ("world_to_body", R[:, :, 2])):
        ang = np.degrees(np.arccos(np.clip(np.sum(up * gu, axis=1), -1, 1)))
        res[conv] = float(np.median(ang))
    best = min(res, key=res.get)
    return {"convention": best, "agreement_deg": round(res[best], 3), "other_deg": round(res[[k for k in res if k != best][0]], 3)}


def attitude(run, mount: Mount) -> dict:
    """pitch/roll from both sources, chassis-frame gyro and linear acceleration."""
    out = {}
    g = imu_vec(run, "gr")
    if g is not None:
        out["pitch_g"], out["roll_g"] = from_gravity(g, mount)
    q = _quat_cols(run)
    if q is not None:
        conv = quat_convention(run)
        out["quat_convention"] = conv
        out["pitch_q"], out["roll_q"] = from_quat(q, mount, conv["convention"])
    gy = imu_vec(run, "g")
    if gy is not None:
        out["gyro_c"] = to_chassis(gy, mount)
    la = imu_vec(run, "la")
    if la is not None:
        out["la_c"] = to_chassis(la, mount)
    return out


def level_reference(run, nominal: Mount | None = None) -> Mount:
    """Mount whose roll and pitch make the mean gravity vector of a level capture point straight up."""
    nominal = nominal or Mount()
    g = imu_vec(run, "gr")
    if g is None:
        raise ValueError("no imu_grx/y/z columns in this run")
    g = g[np.all(np.isfinite(g), axis=1)]
    if len(g) < 10:
        raise ValueError("too few gravity samples for a level reference")
    gm = np.median(g, axis=0)
    gc = nominal.R_cb @ (gm / np.linalg.norm(gm))
    ez = np.array([0.0, 0.0, 1.0])
    axis = np.cross(gc, ez)
    s = np.linalg.norm(axis)
    c = float(np.dot(gc, ez))
    if s < 1e-12:
        R_corr = np.eye(3)
    else:
        k = axis / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        ang = np.arctan2(s, c)
        R_corr = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    m = Mount(R_corr @ nominal.R_cb, f"level reference {run.path}", nominal.nominal_rpy)
    return m


def load_mount(run_dir: Path, level_ref: str | None = None, mount_rpy: tuple | None = None) -> Mount:
    """Mount for a run. Order: --level-ref (json or run), runs/level_ref.json next to the run, --mount, identity.

    --level-ref none disables the automatic lookup.
    """
    if level_ref and level_ref.lower() == "none":
        return Mount.from_rpy(mount_rpy) if mount_rpy else Mount()
    cand = []
    if level_ref:
        cand.append(Path(level_ref))
    else:
        cand += [run_dir / "level_ref.json", run_dir.parent / "level_ref.json"]
    for p in cand:
        if p.is_file() and p.suffix == ".json":
            return Mount.from_json(json.loads(p.read_text()), str(p))
        if p.is_dir() or (p.is_file() and p.name.endswith(".log")):
            from .runlog import load_run

            nominal = Mount.from_rpy(mount_rpy) if mount_rpy else Mount()
            return level_reference(load_run(p), nominal)
    return Mount.from_rpy(mount_rpy) if mount_rpy else Mount()
