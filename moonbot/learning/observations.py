from __future__ import annotations

import mujoco
import numpy as np

from moonbot.simulation.mujoco_backend import RoverState

OBSERVATION_SIZE = 76


def deployment_observation(
    state: RoverState,
    target_base_velocity: np.ndarray,
    target_end_effector_position: np.ndarray,
    previous_action: np.ndarray,
    arm_joint_ranges: np.ndarray,
    terrain_samples: np.ndarray | None = None,
    task_stage: int = 0,
) -> np.ndarray:
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, state.orientation)
    gravity_base = rotation.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
    ranges = np.asarray(arm_joint_ranges, dtype=np.float64)
    arm_center = np.mean(ranges, axis=1)
    arm_half_range = 0.5 * (ranges[:, 1] - ranges[:, 0])
    arm_normalized = (state.arm_positions - arm_center) / arm_half_range
    samples = np.zeros(8) if terrain_samples is None else np.asarray(terrain_samples)
    if samples.shape != (8,):
        raise ValueError("Terrain observations must contain eight samples")
    action = np.asarray(previous_action, dtype=np.float64)
    if action.shape != (14,):
        raise ValueError("Previous action must contain fourteen values")

    observation = np.concatenate(
        (
            gravity_base,
            np.clip(state.imu_angular_velocity / 5.0, -1.0, 1.0),
            np.clip(state.imu_linear_acceleration / 20.0, -1.0, 1.0),
            ((state.wheel_positions + np.pi) % (2.0 * np.pi) - np.pi) / np.pi,
            np.clip(state.wheel_velocities / 25.0, -1.0, 1.0),
            np.clip(arm_normalized, -1.0, 1.0),
            np.clip(state.arm_velocities / 3.0, -1.0, 1.0),
            np.clip(np.asarray(target_base_velocity) / 1.5, -1.0, 1.0),
            np.clip(
                (np.asarray(target_end_effector_position) - state.end_effector_position) / 2.0,
                -1.0,
                1.0,
            ),
            np.clip(action, -1.0, 1.0),
            np.clip(state.linear_velocity / 2.0, -1.0, 1.0),
            state.wheel_contacts,
            np.clip(state.finger_positions / 0.04, 0.0, 1.0),
            np.clip(state.fingertip_forces / 50.0, 0.0, 1.0),
            np.array([np.clip(task_stage / 7.0, 0.0, 1.0)]),
            np.clip(samples / 0.25, -1.0, 1.0),
        )
    ).astype(np.float32)
    if observation.shape != (OBSERVATION_SIZE,):
        raise RuntimeError(f"Observation construction produced {observation.shape}")
    return observation
