from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from moonbot.controllers.arm_pd import GravityCompensatedArmPD
from moonbot.model.dynamics import center_of_mass, static_stability_margin
from moonbot.model.parameters import DEFAULT_MODEL_PATH


@dataclass(frozen=True)
class RoverState:
    time: float
    position: np.ndarray
    orientation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    wheel_positions: np.ndarray
    wheel_velocities: np.ndarray
    suspension_positions: np.ndarray
    steering_positions: np.ndarray
    arm_positions: np.ndarray
    arm_velocities: np.ndarray
    finger_positions: np.ndarray
    end_effector_position: np.ndarray
    end_effector_orientation: np.ndarray
    end_effector_velocity: np.ndarray
    center_of_mass: np.ndarray
    wheel_contacts: np.ndarray
    wheel_contact_forces: np.ndarray
    fingertip_forces: np.ndarray
    imu_angular_velocity: np.ndarray
    imu_linear_acceleration: np.ndarray
    block_position: np.ndarray
    block_orientation: np.ndarray
    actuator_forces: np.ndarray
    stability_margin: float
    grasp_attached: bool


class MuJoCoBackend:
    def __init__(self, model_path: str | Path = DEFAULT_MODEL_PATH) -> None:
        self.model_path = Path(model_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._locked_base_pose: np.ndarray | None = None
        self._end_effector_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
        )
        self._base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self._block_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "grasp_block")
        auxiliary_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"terrain_{index:02d}")
            for index in range(16)
        ]
        auxiliary = np.asarray(auxiliary_geom_ids, dtype=int)
        self.model.geom_contype[auxiliary] = 0
        self.model.geom_conaffinity[auxiliary] = 0
        self.model.geom_rgba[auxiliary, 3] = 0.0
        self._base_fix_equality_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_fix"
        )
        self.arm_controller = GravityCompensatedArmPD(self.model)
        self._wheel_joints = tuple(
            f"wheel_{position}_joint" for position in ("fl", "ml", "rl", "fr", "mr", "rr")
        )
        self._suspension_joints = (
            "left_rocker_joint",
            "left_bogie_joint",
            "right_rocker_joint",
            "right_bogie_joint",
        )
        self._steering_joints = (
            "front_left_steer_joint",
            "front_right_steer_joint",
            "back_left_steer_joint",
            "back_right_steer_joint",
        )
        self._arm_joints = tuple(f"joint{index}" for index in range(1, 8))
        self._finger_joints = ("finger_joint1", "finger_joint2")
        self._block_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "grasp_block_free"
        )
        self._wheel_geom_ids = np.array(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"wheel_{position}_geom")
                for position in ("fl", "ml", "rl", "fr", "mr", "rr")
            ]
        )
        self._wheel_actuator_ids = self._actuator_ids(
            tuple(f"wheel_{position}_motor" for position in ("fl", "ml", "rl", "fr", "mr", "rr"))
        )
        self._finger_actuator_ids = self._actuator_ids(("gripper_actuator",))
        self._steering_actuator_ids = self._actuator_ids(
            (
                "front_left_steer_actuator",
                "front_right_steer_actuator",
                "back_left_steer_actuator",
                "back_right_steer_actuator",
            )
        )

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def reset(
        self,
        *,
        base_position: np.ndarray | None = None,
        block_position: np.ndarray | None = None,
    ) -> RoverState:
        mujoco.mj_resetData(self.model, self.data)
        self._locked_base_pose = None
        if base_position is not None:
            self.data.qpos[:3] = np.asarray(base_position, dtype=np.float64)
        for name, target in zip(
            self.arm_controller.joint_names,
            self.arm_controller.target,
            strict=True,
        ):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = target
        for name in self._finger_joints:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = 0.04
        if block_position is not None:
            block_qpos = self.model.jnt_qposadr[self._block_joint_id]
            self.data.qpos[block_qpos : block_qpos + 3] = np.asarray(
                block_position, dtype=np.float64
            )
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def set_base_fixed(self, active: bool) -> None:
        if self._base_fix_equality_id < 0:
            raise ValueError("Base-fix equality is missing")
        self.data.eq_active[self._base_fix_equality_id] = 0
        self._locked_base_pose = self.data.qpos[:7].copy() if active else None
        if active:
            self.data.qvel[:6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_block_pose(
        self,
        position: np.ndarray,
        quaternion: np.ndarray | None = None,
    ) -> None:
        address = self.model.jnt_qposadr[self._block_joint_id]
        self.data.qpos[address : address + 3] = np.asarray(position, dtype=np.float64)
        self.data.qpos[address + 3 : address + 7] = (
            np.array([1.0, 0.0, 0.0, 0.0])
            if quaternion is None
            else np.asarray(quaternion, dtype=np.float64)
        )
        dof_address = self.model.jnt_dofadr[self._block_joint_id]
        self.data.qvel[dof_address : dof_address + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def gravity_compensation(self) -> np.ndarray:
        return self.arm_controller.command(self.data)

    def _actuator_ids(self, names: tuple[str, ...]) -> np.ndarray:
        identifiers = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in names]
        )
        if np.any(identifiers < 0):
            raise ValueError(f"Missing actuator in {names}")
        return identifiers

    def _joint_values(self, names: tuple[str, ...], velocity: bool = False) -> np.ndarray:
        values = []
        for name in names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            address = (
                self.model.jnt_dofadr[joint_id] if velocity else self.model.jnt_qposadr[joint_id]
            )
            values.append(self.data.qvel[address] if velocity else self.data.qpos[address])
        return np.asarray(values, dtype=np.float64)

    def step(
        self,
        wheel_torques: np.ndarray,
        arm_torques: np.ndarray | None = None,
        gripper_opening: float = 0.025,
        steering_angles: np.ndarray | None = None,
    ) -> RoverState:
        wheel_torques = np.asarray(wheel_torques, dtype=np.float64)
        if wheel_torques.shape != (6,):
            raise ValueError(f"Expected six wheel commands, received {wheel_torques.shape}")
        self.data.ctrl[self._wheel_actuator_ids] = wheel_torques
        if arm_torques is None:
            applied_arm_torques = self.gravity_compensation()
        else:
            applied_arm_torques = np.asarray(arm_torques, dtype=np.float64)
            if applied_arm_torques.shape != (7,):
                raise ValueError(
                    f"Expected seven arm commands, received {applied_arm_torques.shape}"
                )
            limits = self.model.actuator_ctrlrange[self.arm_controller.actuator_ids]
            applied_arm_torques = np.clip(applied_arm_torques, limits[:, 0], limits[:, 1])
        self.data.ctrl[self.arm_controller.actuator_ids] = applied_arm_torques
        steering = np.zeros(4) if steering_angles is None else np.asarray(steering_angles)
        if steering.shape != (4,):
            raise ValueError(f"Expected four steering commands, received {steering.shape}")
        self.data.ctrl[self._steering_actuator_ids] = np.clip(steering, -0.75, 0.75)
        self.data.ctrl[self._finger_actuator_ids] = float(np.clip(gripper_opening, 0.0, 0.04))
        mujoco.mj_step(self.model, self.data)
        if self._locked_base_pose is not None:
            self.data.qpos[:7] = self._locked_base_pose
            self.data.qvel[:6] = 0.0
            mujoco.mj_forward(self.model, self.data)
        return self.state()

    def _sensor(self, name: str) -> np.ndarray:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            raise ValueError(f"Missing sensor: {name}")
        start = self.model.sensor_adr[sensor_id]
        dimension = self.model.sensor_dim[sensor_id]
        return self.data.sensordata[start : start + dimension].copy()

    def _wheel_contact_data(self) -> tuple[np.ndarray, np.ndarray]:
        contacts = np.zeros(6, dtype=np.float64)
        forces = np.zeros(6, dtype=np.float64)
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            contact_force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(self.model, self.data, contact_index, contact_force)
            for wheel_index, geom_id in enumerate(self._wheel_geom_ids):
                if contact.geom1 == geom_id or contact.geom2 == geom_id:
                    contacts[wheel_index] = 1.0
                    forces[wheel_index] += np.linalg.norm(contact_force[:3])
        return contacts, forces

    def bilateral_block_contact(self, minimum_force: float = 0.2) -> bool:
        finger_ids = {
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger"): False,
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger"): False,
        }
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            bodies = {
                int(self.model.geom_bodyid[contact.geom1]),
                int(self.model.geom_bodyid[contact.geom2]),
            }
            if self._block_body_id not in bodies:
                continue
            contact_force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(self.model, self.data, contact_index, contact_force)
            for finger_id in finger_ids:
                if finger_id in bodies and np.linalg.norm(contact_force[:3]) >= minimum_force:
                    finger_ids[finger_id] = True
        return all(finger_ids.values())

    def block_ground_contact(self, minimum_force: float = 0.05) -> bool:
        ground_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            geom_ids = {int(contact.geom1), int(contact.geom2)}
            bodies = {
                int(self.model.geom_bodyid[contact.geom1]),
                int(self.model.geom_bodyid[contact.geom2]),
            }
            if ground_id not in geom_ids or self._block_body_id not in bodies:
                continue
            contact_force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(self.model, self.data, contact_index, contact_force)
            if np.linalg.norm(contact_force[:3]) >= minimum_force:
                return True
        return False

    def state(self) -> RoverState:
        wheel_contacts, wheel_forces = self._wheel_contact_data()
        com_position = center_of_mass(self.model, self.data)
        block_quaternion = self.data.xquat[self._block_body_id].copy()
        return RoverState(
            time=float(self.data.time),
            position=self.data.qpos[:3].copy(),
            orientation=self.data.qpos[3:7].copy(),
            linear_velocity=self.data.qvel[:3].copy(),
            angular_velocity=self.data.qvel[3:6].copy(),
            wheel_positions=self._joint_values(self._wheel_joints),
            wheel_velocities=self._joint_values(self._wheel_joints, velocity=True),
            suspension_positions=self._joint_values(self._suspension_joints),
            steering_positions=self._joint_values(self._steering_joints),
            arm_positions=self._joint_values(self._arm_joints),
            arm_velocities=self._joint_values(self._arm_joints, velocity=True),
            finger_positions=self._joint_values(self._finger_joints),
            end_effector_position=self.data.site_xpos[self._end_effector_id].copy(),
            end_effector_orientation=self.data.site_xmat[self._end_effector_id]
            .reshape(3, 3)
            .copy(),
            end_effector_velocity=self._sensor("end_effector_velocity"),
            center_of_mass=com_position,
            wheel_contacts=wheel_contacts,
            wheel_contact_forces=wheel_forces,
            fingertip_forces=np.array(
                [
                    self._sensor("left_fingertip_contact")[0],
                    self._sensor("right_fingertip_contact")[0],
                ]
            ),
            imu_angular_velocity=self._sensor("imu_gyro"),
            imu_linear_acceleration=self._sensor("imu_accelerometer"),
            block_position=self.data.xpos[self._block_body_id].copy(),
            block_orientation=block_quaternion,
            actuator_forces=self.data.actuator_force.copy(),
            stability_margin=static_stability_margin(
                self.data.xpos[self._base_body_id],
                self.data.xquat[self._base_body_id],
                com_position,
            ),
            grasp_attached=False,
        )
