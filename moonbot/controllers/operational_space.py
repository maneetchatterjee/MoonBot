from __future__ import annotations

import mujoco
import numpy as np

from moonbot.controllers.arm_pd import PANDA_HOME
from moonbot.model.kinematics import rotation_error, site_jacobian


class OperationalSpaceController:
    def __init__(
        self,
        model: mujoco.MjModel,
        site_name: str = "end_effector",
        position_kp: float = 180.0,
        position_kd: float = 28.0,
        orientation_kp: float = 35.0,
        nullspace_kp: float = 12.0,
        nullspace_kd: float = 2.5,
        joint_limit_gain: float = 0.08,
    ) -> None:
        self.model = model
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.joint_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{index}")
                for index in range(1, 8)
            ]
        )
        self.qpos_ids = model.jnt_qposadr[self.joint_ids]
        self.dof_ids = model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"arm_motor_{index}")
                for index in range(1, 8)
            ]
        )
        self.position_kp = position_kp
        self.position_kd = position_kd
        self.orientation_kp = orientation_kp
        self.nullspace_kp = nullspace_kp
        self.nullspace_kd = nullspace_kd
        self.joint_limit_gain = joint_limit_gain

    def _joint_limit_gradient(self, positions: np.ndarray) -> np.ndarray:
        ranges = self.model.jnt_range[self.joint_ids]
        distance_low = np.maximum(positions - ranges[:, 0], 0.04)
        distance_high = np.maximum(ranges[:, 1] - positions, 0.04)
        return self.joint_limit_gain * (1.0 / distance_low**2 - 1.0 / distance_high**2)

    def command(
        self,
        data: mujoco.MjData,
        target_position: np.ndarray,
        target_rotation: np.ndarray | None = None,
    ) -> np.ndarray:
        mujoco.mj_forward(self.model, data)
        jacobian = site_jacobian(self.model, data, self.site_id, self.dof_ids)
        current_position = data.site_xpos[self.site_id]
        position_error = np.asarray(target_position) - current_position
        site_velocity = jacobian @ data.qvel[self.dof_ids]
        task_force = np.concatenate(
            (
                self.position_kp * position_error - self.position_kd * site_velocity[:3],
                np.zeros(3),
            )
        )
        if target_rotation is not None:
            current_rotation = data.site_xmat[self.site_id].reshape(3, 3)
            task_force[3:] = (
                self.orientation_kp * rotation_error(np.asarray(target_rotation), current_rotation)
                - 4.0 * site_velocity[3:]
            )

        q = data.qpos[self.qpos_ids]
        qd = data.qvel[self.dof_ids]
        jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-3)
        nullspace = np.eye(7) - jacobian_pinv @ jacobian
        posture = (
            self.nullspace_kp * (PANDA_HOME - q)
            - self.nullspace_kd * qd
            + self._joint_limit_gradient(q)
        )
        torque = data.qfrc_bias[self.dof_ids] + jacobian.T @ task_force + nullspace @ posture
        limits = self.model.actuator_ctrlrange[self.actuator_ids]
        return np.clip(torque, limits[:, 0], limits[:, 1])
