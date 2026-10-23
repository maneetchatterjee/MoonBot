from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from moonbot.controllers.arm_pd import PANDA_HOME, GravityCompensatedArmPD
from moonbot.model.parameters import DEFAULT_MODEL_PATH

WHEEL_JOINTS = (
    "wheel_fl_joint",
    "wheel_ml_joint",
    "wheel_rl_joint",
    "wheel_fr_joint",
    "wheel_mr_joint",
    "wheel_rr_joint",
)
ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
SUSPENSION_JOINTS = (
    "left_rocker_joint",
    "left_bogie_joint",
    "right_rocker_joint",
    "right_bogie_joint",
)
FINGER_JOINTS = ("finger_joint1", "finger_joint2")
STEERING_JOINTS = (
    "front_left_steer_joint",
    "front_right_steer_joint",
    "back_left_steer_joint",
    "back_right_steer_joint",
)


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class ValidationReport:
    model_path: str
    checks: tuple[CheckResult, ...]

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)

    def to_dict(self) -> dict[str, object]:
        return {
            "model_path": self.model_path,
            "passed": self.passed,
            "checks": [
                {"name": check.name, "passed": bool(check.passed), "detail": check.detail}
                for check in self.checks
            ],
        }


def _name(model: mujoco.MjModel, objtype: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, objtype, index) or f"unnamed_{index}"


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if index < 0:
        raise ValueError(f"Joint not found: {name}")
    return index


def _actuator_gravity_compensation(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    controller = GravityCompensatedArmPD(model)
    data.ctrl[controller.actuator_ids] = controller.command(data)
    gripper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_actuator")
    data.ctrl[gripper_id] = 0.04
    for joint_name in STEERING_JOINTS:
        actuator_name = joint_name.replace("_joint", "_actuator")
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        data.ctrl[actuator_id] = 0.0


def _initialize_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    for joint_name, target in zip(ARM_JOINTS, PANDA_HOME, strict=True):
        joint_id = _joint_id(model, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = target
    for joint_name in FINGER_JOINTS:
        joint_id = _joint_id(model, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = 0.04
    mujoco.mj_forward(model, data)


def _settle(
    model: mujoco.MjModel,
    duration: float = 1.0,
    wheel_command: float = 0.0,
) -> mujoco.MjData:
    data = mujoco.MjData(model)
    _initialize_home(model, data)
    steps = round(duration / model.opt.timestep)
    for _ in range(steps):
        data.ctrl[:6] = wheel_command
        _actuator_gravity_compensation(model, data)
        mujoco.mj_step(model, data)
    return data


def _quat_to_roll_pitch(quat: np.ndarray) -> tuple[float, float]:
    w, x, y, z = quat
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


def _body_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError(f"Body not found: {name}")
    return data.xpos[body_id].copy()


def _disable_auxiliary_geoms(model: mujoco.MjModel) -> None:
    for name in (
        *tuple(f"terrain_{index:02d}" for index in range(16)),
    ):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0


def validate_model(path: str | Path = DEFAULT_MODEL_PATH) -> ValidationReport:
    source = Path(path).resolve()
    try:
        model = mujoco.MjModel.from_xml_path(str(source))
    except Exception as exc:
        return ValidationReport(
            model_path=str(source),
            checks=(CheckResult("xml_compilation", False, str(exc)),),
        )

    _disable_auxiliary_geoms(model)

    checks: list[CheckResult] = [
        CheckResult(
            "xml_compilation",
            True,
            f"Compiled nq={model.nq}, nv={model.nv}, nu={model.nu}, nbody={model.nbody}",
        )
    ]

    nonpositive_masses = [
        _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        for body_id in range(1, model.nbody)
        if model.body_mass[body_id] <= 0
    ]
    checks.append(
        CheckResult(
            "positive_link_masses",
            not nonpositive_masses,
            "All moving bodies have positive mass"
            if not nonpositive_masses
            else f"Nonpositive mass: {nonpositive_masses}",
        )
    )

    invalid_inertias = [
        _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        for body_id in range(1, model.nbody)
        if np.any(model.body_inertia[body_id] <= 0)
    ]
    checks.append(
        CheckResult(
            "positive_definite_inertias",
            not invalid_inertias,
            "All principal inertias are positive"
            if not invalid_inertias
            else f"Invalid inertia: {invalid_inertias}",
        )
    )

    invalid_ranges: list[str] = []
    for joint_id in range(model.njnt):
        if (
            model.jnt_limited[joint_id]
            and model.jnt_range[joint_id, 0] >= model.jnt_range[joint_id, 1]
        ):
            invalid_ranges.append(_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id))
    checks.append(
        CheckResult(
            "valid_joint_ranges",
            not invalid_ranges,
            "All limited joints have increasing ranges"
            if not invalid_ranges
            else f"Invalid ranges: {invalid_ranges}",
        )
    )

    mapped_joint_ids = model.actuator_trnid[:, 0]
    joint_actuator_mask = model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT
    tendon_actuator_mask = model.actuator_trntype == mujoco.mjtTrn.mjTRN_TENDON
    mapping_valid = (
        model.nu == 18
        and np.count_nonzero(joint_actuator_mask) == 17
        and len(set(mapped_joint_ids[joint_actuator_mask].tolist())) == 17
        and np.count_nonzero(tendon_actuator_mask) == 1
    )
    checks.append(
        CheckResult(
            "actuator_joint_mapping",
            bool(mapping_valid),
            "Mapped 17 unique joint actuators and one gripper-tendon actuator",
        )
    )

    actuator_limits_valid = bool(
        np.all(model.actuator_ctrllimited)
        and np.all(np.isfinite(model.actuator_ctrlrange))
        and np.all(model.actuator_ctrlrange[:, 0] < model.actuator_ctrlrange[:, 1])
        and np.all(np.abs(model.actuator_gear[:, 0]) > 0.0)
        and np.all(model.dof_damping >= 0.0)
        and np.all(model.dof_frictionloss >= 0.0)
    )
    checks.append(
        CheckResult(
            "actuator_limits_and_joint_dissipation",
            actuator_limits_valid,
            "Finite increasing control limits, nonzero gears, and nonnegative damping/friction",
        )
    )

    required_sensors = {
        "base_orientation": 4,
        "imu_gyro": 3,
        "imu_accelerometer": 3,
        "end_effector_position": 3,
        "end_effector_orientation": 4,
        "end_effector_velocity": 3,
        "left_fingertip_contact": 1,
        "right_fingertip_contact": 1,
    }
    sensor_frames_valid = True
    for sensor_name, dimension in required_sensors.items():
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        sensor_frames_valid &= bool(
            sensor_id >= 0
            and model.sensor_dim[sensor_id] == dimension
            and model.sensor_objid[sensor_id] >= 0
        )
    checks.append(
        CheckResult(
            "sensor_frames_and_dimensions",
            sensor_frames_valid,
            "IMU, end-effector, and fingertip sensors resolve to valid sites and dimensions",
        )
    )

    collision_geom_count = int(np.count_nonzero(model.geom_contype))
    mesh_scales_valid = bool(model.nmesh > 0 and np.all(model.mesh_scale > 0.0))
    wheel_collision_valid = all(
        model.geom_contype[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"wheel_{position}_geom")
        ]
        > 0
        for position in ("fl", "ml", "rl", "fr", "mr", "rr")
    )
    checks.append(
        CheckResult(
            "collision_geometry_and_mesh_scale",
            mesh_scales_valid and wheel_collision_valid and collision_geom_count >= 10,
            f"Positive mesh scales and {collision_geom_count} active collision geoms",
        )
    )

    initial = mujoco.MjData(model)
    _initialize_home(model, initial)
    penetrating = [
        float(initial.contact[index].dist)
        for index in range(initial.ncon)
        if initial.contact[index].dist < -1e-6
    ]
    checks.append(
        CheckResult(
            "no_initial_interpenetration",
            not penetrating,
            (
                f"Initial contacts={initial.ncon}; "
                f"minimum distance={min(penetrating, default=0.0):.3e} m"
            ),
        )
    )

    settled = _settle(model, duration=2.0)
    roll, pitch = _quat_to_roll_pitch(settled.qpos[3:7])
    stable = bool(
        np.all(np.isfinite(settled.qpos))
        and settled.qpos[2] > 0.20
        and abs(roll) < np.deg2rad(10)
        and abs(pitch) < np.deg2rad(10)
    )
    checks.append(
        CheckResult(
            "static_equilibrium",
            stable,
            (
                f"z={settled.qpos[2]:.4f} m, roll={np.rad2deg(roll):.3f} deg, "
                f"pitch={np.rad2deg(pitch):.3f} deg"
            ),
        )
    )

    driven = _settle(model, duration=0.6, wheel_command=4.0)
    forward_ok = bool(driven.qpos[0] > 0.05)
    checks.append(
        CheckResult(
            "wheel_rotation_direction",
            forward_ok,
            f"Positive six-wheel command produced dx={driven.qpos[0]:.4f} m",
        )
    )

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    shoulder_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link0")
    reach = float(np.linalg.norm(initial.site_xpos[site_id] - initial.xpos[shoulder_id]))
    fk_ok = 0.35 < reach < 0.90
    checks.append(
        CheckResult(
            "forward_kinematics_sanity",
            fk_ok,
            f"Home shoulder-to-tool distance={reach:.4f} m",
        )
    )

    gravity_data = _settle(model, duration=0.5)
    arm_positions = []
    for joint_name in ARM_JOINTS:
        joint_id = _joint_id(model, joint_name)
        arm_positions.append(gravity_data.qpos[model.jnt_qposadr[joint_id]])
    maximum_arm_drift = float(np.max(np.abs(np.asarray(arm_positions) - PANDA_HOME)))
    checks.append(
        CheckResult(
            "gravity_compensated_joint_hold",
            maximum_arm_drift < 0.02,
            f"Maximum home-joint drift={maximum_arm_drift:.6f} rad over 0.5 s",
        )
    )

    panda_bodies = (*tuple(f"link{index}" for index in range(8)), "hand")
    panda_complete = all(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0 for name in panda_bodies
    )
    checks.append(
        CheckResult(
            "menagerie_panda_topology",
            panda_complete,
            "Menagerie Panda links 0-7, hand, seven joints, and two fingers are present",
        )
    )

    suspension_present = all(_joint_id(model, name) >= 0 for name in SUSPENSION_JOINTS)
    differential_present = (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "rocker_differential") >= 0
    )
    checks.append(
        CheckResult(
            "rocker_bogie_topology",
            suspension_present and differential_present,
            "Four passive suspension joints and coupled rocker differential are present",
        )
    )

    steering_joint_ids = [_joint_id(model, name) for name in STEERING_JOINTS]
    steering_actuator_ids = [
        mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            name.replace("_joint", "_actuator"),
        )
        for name in STEERING_JOINTS
    ]
    steering_ranges = model.jnt_range[steering_joint_ids]
    steering_valid = bool(
        all(actuator_id >= 0 for actuator_id in steering_actuator_ids)
        and np.allclose(steering_ranges[:, 0], -0.75)
        and np.allclose(steering_ranges[:, 1], 0.75)
    )
    checks.append(
        CheckResult(
            "four_wheel_steering_topology",
            steering_valid,
            (
                "Front and rear steering joints have independent position actuators "
                "and +/-0.75 rad travel"
            ),
        )
    )

    articulated = mujoco.MjData(model)
    bogie_id = _joint_id(model, "left_bogie_joint")
    articulated.qpos[model.jnt_qposadr[bogie_id]] = 0.25
    mujoco.mj_forward(model, articulated)
    middle_height = _body_position(model, articulated, "wheel_ml")[2]
    rear_height = _body_position(model, articulated, "wheel_rl")[2]
    height_difference = abs(middle_height - rear_height)
    checks.append(
        CheckResult(
            "rocker_bogie_articulation",
            bool(height_difference > 0.05),
            f"A 0.25 rad bogie rotation separated wheel heights by {height_difference:.4f} m",
        )
    )

    finger_ranges = np.array([model.jnt_range[_joint_id(model, name)] for name in FINGER_JOINTS])
    gripper_valid = bool(
        np.allclose(finger_ranges[:, 0], 0.0) and np.allclose(finger_ranges[:, 1], 0.04)
    )
    checks.append(
        CheckResult(
            "parallel_gripper_travel",
            gripper_valid,
            f"Symmetric finger travel ranges={finger_ranges.tolist()}",
        )
    )

    first = _settle(model, duration=0.25)
    second = _settle(model, duration=0.25)
    deterministic = bool(
        np.array_equal(first.qpos, second.qpos) and np.array_equal(first.qvel, second.qvel)
    )
    checks.append(
        CheckResult(
            "deterministic_fixed_initial_state",
            deterministic,
            (
                "Repeated trajectories are bitwise identical"
                if deterministic
                else "Repeated trajectories differ"
            ),
        )
    )

    timestep_states = []
    for timestep in (0.002, 0.004):
        timestep_model = mujoco.MjModel.from_xml_path(str(source))
        _disable_auxiliary_geoms(timestep_model)
        timestep_model.opt.timestep = timestep
        timestep_states.append(_settle(timestep_model, duration=1.0))
    position_delta = float(
        np.linalg.norm(timestep_states[0].qpos[:3] - timestep_states[1].qpos[:3])
    )
    attitude_delta = max(
        abs(first_value - second_value)
        for first_value, second_value in zip(
            _quat_to_roll_pitch(timestep_states[0].qpos[3:7]),
            _quat_to_roll_pitch(timestep_states[1].qpos[3:7]),
            strict=True,
        )
    )
    checks.append(
        CheckResult(
            "physics_timestep_sensitivity",
            position_delta < 0.02 and attitude_delta < np.deg2rad(1.0),
            (
                f"2 ms versus 4 ms settle delta: position={position_delta:.5f} m, "
                f"attitude={np.rad2deg(attitude_delta):.4f} deg"
            ),
        )
    )

    return ValidationReport(model_path=str(source), checks=tuple(checks))
