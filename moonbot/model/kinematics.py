from __future__ import annotations

import mujoco
import numpy as np


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    matrix = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(matrix, quaternion)
    return matrix.reshape(3, 3)


def rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    relative = np.asarray(target) @ np.asarray(current).T
    return 0.5 * np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ]
    )


def rotation_distance(target: np.ndarray, current: np.ndarray) -> float:
    """Return the geodesic SO(3) distance in radians."""
    relative = np.asarray(target, dtype=np.float64) @ np.asarray(
        current, dtype=np.float64
    ).T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def site_jacobian(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    dof_ids: np.ndarray | None = None,
) -> np.ndarray:
    translational = np.zeros((3, model.nv), dtype=np.float64)
    rotational = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, translational, rotational, site_id)
    jacobian = np.vstack((translational, rotational))
    return jacobian if dof_ids is None else jacobian[:, dof_ids]


def damped_least_squares(
    jacobian: np.ndarray,
    error: np.ndarray,
    damping: float = 0.05,
) -> np.ndarray:
    jacobian = np.asarray(jacobian, dtype=np.float64)
    error = np.asarray(error, dtype=np.float64)
    regularizer = damping**2 * np.eye(jacobian.shape[0])
    return jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + regularizer, error)


def solve_site_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_name: str,
    joint_names: tuple[str, ...],
    target_position: np.ndarray,
    target_rotation: np.ndarray | None = None,
    maximum_iterations: int = 200,
    tolerance: float = 1e-4,
    damping: float = 0.04,
    step_size: float = 0.35,
) -> tuple[np.ndarray, float, bool]:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    )
    if site_id < 0 or np.any(joint_ids < 0):
        raise ValueError("IK site or joint was not found")
    qpos_ids = model.jnt_qposadr[joint_ids]
    dof_ids = model.jnt_dofadr[joint_ids]
    target_position = np.asarray(target_position, dtype=np.float64)

    for _ in range(maximum_iterations):
        mujoco.mj_forward(model, data)
        position_error = target_position - data.site_xpos[site_id]
        jacobian = site_jacobian(model, data, site_id, dof_ids)
        if target_rotation is None:
            task_error = position_error
            task_jacobian = jacobian[:3]
        else:
            current_rotation = data.site_xmat[site_id].reshape(3, 3)
            orientation_error = rotation_error(target_rotation, current_rotation)
            task_error = np.concatenate((position_error, 0.35 * orientation_error))
            task_jacobian = jacobian
        error_norm = float(np.linalg.norm(position_error))
        if error_norm < tolerance:
            return data.qpos[qpos_ids].copy(), error_norm, True
        delta = damped_least_squares(task_jacobian, task_error, damping)
        data.qpos[qpos_ids] += step_size * delta
        for joint_id, qpos_id in zip(joint_ids, qpos_ids, strict=True):
            if model.jnt_limited[joint_id]:
                low, high = model.jnt_range[joint_id]
                data.qpos[qpos_id] = np.clip(data.qpos[qpos_id], low + 1e-4, high - 1e-4)

    mujoco.mj_forward(model, data)
    final_error = float(np.linalg.norm(target_position - data.site_xpos[site_id]))
    return data.qpos[qpos_ids].copy(), final_error, final_error < tolerance


def solve_site_pose_multistart(
    model: mujoco.MjModel,
    qpos: np.ndarray,
    site_name: str,
    joint_names: tuple[str, ...],
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    reference_joints: np.ndarray,
    *,
    attempts: int = 16,
    seed: int = 17,
    minimum_joint_margin: float = 0.08,
) -> np.ndarray:
    from scipy.optimize import least_squares

    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    )
    if site_id < 0 or np.any(joint_ids < 0):
        raise ValueError("IK site or joint was not found")
    qpos_ids = model.jnt_qposadr[joint_ids]
    ranges = model.jnt_range[joint_ids]
    target_position = np.asarray(target_position, dtype=np.float64)
    target_rotation = np.asarray(target_rotation, dtype=np.float64)
    reference_joints = np.asarray(reference_joints, dtype=np.float64)

    def residual(joints: np.ndarray) -> np.ndarray:
        data.qpos[qpos_ids] = joints
        mujoco.mj_forward(model, data)
        current_rotation = data.site_xmat[site_id].reshape(3, 3)
        return np.concatenate(
            (
                4.0 * (target_position - data.site_xpos[site_id]),
                0.5 * (current_rotation - target_rotation).ravel(),
                0.015 * (joints - reference_joints),
            )
        )

    rng = np.random.default_rng(seed)
    candidates: list[tuple[float, np.ndarray]] = []
    for attempt in range(attempts):
        initial = reference_joints if attempt == 0 else rng.uniform(ranges[:, 0], ranges[:, 1])
        result = least_squares(
            residual,
            initial,
            bounds=(ranges[:, 0] + 1e-4, ranges[:, 1] - 1e-4),
            max_nfev=1000,
        )
        data.qpos[qpos_ids] = result.x
        mujoco.mj_forward(model, data)
        position_error = float(np.linalg.norm(target_position - data.site_xpos[site_id]))
        rotation_error_norm = float(
            np.linalg.norm(0.5 * (data.site_xmat[site_id].reshape(3, 3) - target_rotation))
        )
        margin = float(
            np.min(np.minimum(result.x - ranges[:, 0], ranges[:, 1] - result.x))
        )
        if position_error < 0.01 and rotation_error_norm < 0.05 and margin > minimum_joint_margin:
            score = position_error + 0.05 * rotation_error_norm - 0.002 * margin
            candidates.append((score, result.x.copy()))
    if not candidates:
        raise RuntimeError(f"No joint-margin-safe pose found for target {target_position}")
    return min(candidates, key=lambda item: item[0])[1]


def solve_site_pose_near(
    model: mujoco.MjModel,
    qpos: np.ndarray,
    site_name: str,
    joint_names: tuple[str, ...],
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    initial_joints: np.ndarray,
    *,
    residual_tolerance: float = 0.025,
) -> np.ndarray:
    from scipy.optimize import least_squares

    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    )
    if site_id < 0 or np.any(joint_ids < 0):
        raise ValueError("IK site or joint was not found")
    qpos_ids = model.jnt_qposadr[joint_ids]
    ranges = model.jnt_range[joint_ids]
    target_position = np.asarray(target_position, dtype=np.float64)
    target_rotation = np.asarray(target_rotation, dtype=np.float64)

    def residual(joints: np.ndarray) -> np.ndarray:
        data.qpos[qpos_ids] = joints
        mujoco.mj_forward(model, data)
        current_rotation = data.site_xmat[site_id].reshape(3, 3)
        return np.concatenate(
            (
                4.0 * (target_position - data.site_xpos[site_id]),
                0.5 * (current_rotation - target_rotation).ravel(),
            )
        )

    result = least_squares(
        residual,
        np.asarray(initial_joints, dtype=np.float64),
        bounds=(ranges[:, 0] + 1e-4, ranges[:, 1] - 1e-4),
        max_nfev=300,
    )
    norm = float(np.linalg.norm(residual(result.x)))
    if norm > residual_tolerance:
        raise RuntimeError(f"Nearby pose IK residual {norm:.4f} exceeds {residual_tolerance:.4f}")
    return result.x
