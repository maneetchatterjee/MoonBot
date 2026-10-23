from __future__ import annotations

import mujoco
import numpy as np


def center_of_mass(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    masses = model.body_mass[1:]
    return np.sum(data.xipos[1:] * masses[:, None], axis=0) / np.sum(masses)


def quaternion_roll_pitch(quaternion: np.ndarray) -> tuple[float, float]:
    w, x, y, z = np.asarray(quaternion, dtype=np.float64)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


def static_stability_margin(
    base_position: np.ndarray,
    base_quaternion: np.ndarray,
    com_position: np.ndarray,
    half_length: float = 0.52,
    half_width: float = 0.43,
) -> float:
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, np.asarray(base_quaternion, dtype=np.float64))
    relative = rotation.reshape(3, 3).T @ (com_position - base_position)
    return float(min(half_length - abs(relative[0]), half_width - abs(relative[1])))


def longitudinal_slip_ratio(
    wheel_speeds: np.ndarray,
    base_speed: float,
    wheel_radius: float,
    epsilon: float = 0.1,
) -> float:
    surface_speed = np.asarray(wheel_speeds, dtype=np.float64) * wheel_radius
    denominator = np.maximum(np.maximum(np.abs(surface_speed), abs(base_speed)), epsilon)
    return float(np.mean(np.abs(surface_speed - base_speed) / denominator))


def integrated_absolute_effort(control: np.ndarray, timestep: float) -> float:
    return float(np.sum(np.abs(np.asarray(control, dtype=np.float64))) * timestep)
