from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import least_squares

from moonbot.simulation.mujoco_backend import MuJoCoBackend
from moonbot.simulation.terrain import TerrainManager, TerrainSpecification


def solve_mount(mount_x: float, mount_z: float, tilt_deg: float) -> dict[str, float]:
    backend = MuJoCoBackend()
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    model = backend.model
    link0_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link0")
    model.body_pos[link0_id] = np.array([mount_x, 0.0, mount_z])
    quaternion = np.zeros(4)
    mujoco.mju_axisAngle2Quat(
        quaternion,
        np.array([0.0, 1.0, 0.0]),
        np.deg2rad(tilt_deg),
    )
    model.body_quat[link0_id] = quaternion
    data = mujoco.MjData(model)
    mujoco.mj_setConst(model, data)
    backend.reset(
        base_position=np.array([0.20, 0.0, 0.417]),
        block_position=np.array([0.90, 0.0, 0.06]),
    )
    data.qpos[:] = backend.data.qpos
    target_position = np.array([0.90, 0.0, 0.09])
    target_axis = np.array([0.0, 0.0, -1.0])
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    joint_ids = np.array(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{index}")
            for index in range(1, 8)
        ]
    )
    qpos_ids = model.jnt_qposadr[joint_ids]
    ranges = model.jnt_range[joint_ids]

    def residual(joints: np.ndarray) -> np.ndarray:
        data.qpos[qpos_ids] = joints
        mujoco.mj_forward(model, data)
        current_axis = data.site_xmat[site_id].reshape(3, 3)[:, 2]
        return np.concatenate(
            (
                target_position - data.site_xpos[site_id],
                0.20 * np.cross(current_axis, target_axis),
            )
        )

    rng = np.random.default_rng(23)
    best = (float("inf"), float("inf"), float("inf"), np.zeros(7))
    for attempt in range(12):
        initial = (
            backend.state().arm_positions
            if attempt == 0
            else rng.uniform(ranges[:, 0], ranges[:, 1])
        )
        result = least_squares(
            residual,
            initial,
            bounds=(ranges[:, 0] + 1e-4, ranges[:, 1] - 1e-4),
            max_nfev=500,
        )
        error = residual(result.x)
        position_error = float(np.linalg.norm(error[:3]))
        axis_error = float(np.linalg.norm(error[3:]) / 0.20)
        score = position_error + 0.10 * axis_error
        if score < best[0]:
            best = (score, position_error, axis_error, result.x.copy())
    return {
        "mount_x": mount_x,
        "mount_z": mount_z,
        "tilt_deg": tilt_deg,
        "position_error_m": best[1],
        "axis_error": best[2],
        "score": best[0],
    }


def main() -> None:
    results = []
    for mount_x in (0.02, 0.14, 0.24):
        for mount_z in (0.11, 0.143):
            for tilt_deg in (0.0, 15.0, 30.0, 45.0):
                result = solve_mount(mount_x, mount_z, tilt_deg)
                results.append(result)
                print({key: round(value, 5) for key, value in result.items()})
    print("BEST", min(results, key=lambda item: item["score"]))


if __name__ == "__main__":
    main()
