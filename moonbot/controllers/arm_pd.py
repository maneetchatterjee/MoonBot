from __future__ import annotations

import mujoco
import numpy as np

PANDA_HOME = np.array([0.0, -0.35, 0.0, -1.8, 0.0, 1.5, -0.7853])


class GravityCompensatedArmPD:
    def __init__(
        self,
        model: mujoco.MjModel,
        target: np.ndarray | None = None,
        kp: float = 80.0,
        kd: float = 6.0,
        joint_names: tuple[str, ...] = tuple(f"joint{index}" for index in range(1, 8)),
    ) -> None:
        self.model = model
        self.joint_names = joint_names
        self.target = PANDA_HOME.copy() if target is None else np.asarray(target, dtype=np.float64)
        if self.target.shape != (len(joint_names),):
            raise ValueError(f"Arm target must contain {len(joint_names)} joint positions")
        self.kp = kp
        self.kd = kd
        self.actuator_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"arm_motor_{index}")
                for index in range(1, len(joint_names) + 1)
            ],
            dtype=int,
        )
        if np.any(self.actuator_ids < 0):
            raise ValueError("One or more arm actuators are missing")

    def command(self, data: mujoco.MjData) -> np.ndarray:
        mujoco.mj_forward(self.model, data)
        output = np.empty(len(self.joint_names), dtype=np.float64)
        for index, actuator_id in enumerate(self.actuator_ids):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            dof_id = int(self.model.jnt_dofadr[joint_id])
            gear = float(self.model.actuator_gear[actuator_id, 0])
            gravity = data.qfrc_bias[dof_id]
            feedback = (
                self.kp * (self.target[index] - data.qpos[qpos_id]) - self.kd * data.qvel[dof_id]
            )
            low, high = self.model.actuator_ctrlrange[actuator_id]
            output[index] = np.clip((gravity + feedback) / gear, low, high)
        return output
