from __future__ import annotations

from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, ClassVar

import gymnasium as gym
import mujoco
import numpy as np
import yaml
from gymnasium import spaces

from moonbot.controllers.arm_pd import PANDA_HOME
from moonbot.controllers.mpc import ReducedOrderMPC
from moonbot.controllers.operational_space import OperationalSpaceController
from moonbot.controllers.pid_base import PIDAxis, SkidSteerPID
from moonbot.controllers.residual_controller import ResidualController
from moonbot.controllers.safety_filter import ActionSafetyFilter, SafetyFilterResult
from moonbot.controllers.stability import StabilityCoordinator, StabilityDecision
from moonbot.learning.observations import OBSERVATION_SIZE, deployment_observation
from moonbot.learning.rewards import task_reward
from moonbot.model.dynamics import longitudinal_slip_ratio, quaternion_roll_pitch
from moonbot.model.kinematics import (
    quaternion_to_rotation,
    rotation_distance,
    solve_site_pose_near,
)
from moonbot.model.parameters import DEFAULT_MODEL_PATH, PROJECT_ROOT
from moonbot.simulation.mujoco_backend import MuJoCoBackend, RoverState
from moonbot.simulation.randomization import DomainRandomizer, load_randomization
from moonbot.simulation.sensors import ObservationDelay
from moonbot.simulation.terrain import TerrainManager, TerrainSpecification

TASKS = (
    "velocity_tracking",
    "waypoint_navigation",
    "arm_reaching",
    "mobile_tracking",
    "payload_transport",
    "mobile_manipulation",
)
CONTROL_MODES = ("direct", "pid", "mpc", "residual")
MANIPULATION_GRASP_ROTATION = np.diag([-1.0, 1.0, -1.0])
MANIPULATION_BASE_STANDOFF_M = 0.60
MANIPULATION_PREGRASP_HEIGHT_M = 0.20
MANIPULATION_GRASP_HEIGHT_M = 0.055
MANIPULATION_LIFT_HEIGHT_M = 0.315
MANIPULATION_HOLD_OPENING_M = 0.0125
MANIPULATION_JOINT_TARGETS = {
    "pregrasp": np.array(
        [0.2946421084, 1.0427988102, -0.3843504393, -2.1014585868, -0.4642962026,
         3.6974228401, -1.7059376437]
    ),
    "grasp": np.array(
        [0.1391711413, 1.4602657794, -0.2769638186, -1.4879182460, -0.5221012085,
         3.5186318976, -1.6333977914]
    ),
    "lift": np.array(
        [0.4441818588, 0.7254696670, -0.5459401420, -2.4508517428, -0.5577535906,
         3.7216962954, -1.6791280790]
    ),
    "transport": np.array(
        [-0.2305696874, 0.6094385798, 0.0977140963, -2.0305786163, -1.0992408834,
         3.1712539304, -1.4152929995]
    ),
    "lower": np.array(
        [0.0195565677, 1.4633827755, -0.0151813956, -1.1087958292, 0.0892689549,
         3.0954941052, -2.4222959699]
    ),
}


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"Expected a mapping in {path}")
    return result


def _pid_controller() -> SkidSteerPID:
    config = _load_yaml(PROJECT_ROOT / "configs" / "controller" / "pid.yaml")
    return SkidSteerPID(
        PIDAxis(**config["linear"]),
        PIDAxis(**config["angular"]),
        float(config["maximum_wheel_torque"]),
        float(config["maximum_torque_rate"]),
    )


class MoonBotEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata: ClassVar[dict[str, Any]] = {
        "render_modes": ["rgb_array"],
        "render_fps": 25,
    }

    def __init__(
        self,
        task: str = "velocity_tracking",
        control_mode: str = "residual",
        model_path: str | Path = DEFAULT_MODEL_PATH,
        terrain: str = "flat",
        terrain_difficulty: float = 0.0,
        domain_randomization: bool = False,
        safety_filter: bool = False,
        episode_seconds: float = 12.0,
        policy_rate_hz: int = 25,
        render_mode: str | None = None,
        randomization_config: str | Path | None = None,
        stability_coordinator: bool = True,
        reward_ablation: str | None = None,
        arm_base_coordination: bool = True,
        sensor_failure: str | None = None,
        ood_randomization: bool = False,
        fixed_observation_delay_steps: int | None = None,
        forced_external_push_n: float | None = None,
        curriculum_enabled: bool = False,
    ) -> None:
        super().__init__()
        if task not in TASKS:
            raise ValueError(f"Unknown task: {task}")
        if control_mode not in CONTROL_MODES:
            raise ValueError(f"Unknown control mode: {control_mode}")
        self.task = task
        self.control_mode = control_mode
        self.terrain_family = terrain
        self.terrain_difficulty = terrain_difficulty
        self.domain_randomization_enabled = domain_randomization
        self._allow_domain_randomization = domain_randomization
        self.use_safety_filter = safety_filter
        self.use_stability_coordinator = stability_coordinator
        self.arm_base_coordination = arm_base_coordination
        self.sensor_failure = sensor_failure
        self.ood_randomization = ood_randomization
        self.fixed_observation_delay_steps = fixed_observation_delay_steps
        self.forced_external_push_n = forced_external_push_n
        self.curriculum_enabled = curriculum_enabled
        self.curriculum_stage_index = 0
        self.render_mode = render_mode
        self.backend = MuJoCoBackend(model_path)
        self.terrain = TerrainManager(self.backend.model)
        randomization_path = (
            Path(randomization_config)
            if randomization_config
            else (PROJECT_ROOT / "configs" / "environment" / "randomization.yaml")
        )
        self.randomizer = DomainRandomizer(
            self.backend.model, load_randomization(randomization_path)
        )
        self.control_decimation = max(1, round(1.0 / (policy_rate_hz * self.backend.timestep)))
        self.control_dt = self.control_decimation * self.backend.timestep
        self.maximum_steps = round(episode_seconds / self.control_dt)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(14,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(OBSERVATION_SIZE,), dtype=np.float32)
        self.pid = _pid_controller()
        self.mpc = (
            ReducedOrderMPC(self.pid, timestep=self.control_dt, time_budget_s=0.25)
            if control_mode == "mpc"
            else None
        )
        self.osc = OperationalSpaceController(self.backend.model)
        residual_limits = np.array([8.0] * 6 + [10.0] * 7)
        self.residual = ResidualController(residual_limits, np.array([80.0] * 6 + [120.0] * 7))
        arm_limits = self.backend.model.actuator_ctrlrange[self.backend.arm_controller.actuator_ids]
        lower = np.concatenate((np.full(6, -30.0), arm_limits[:, 0]))
        upper = np.concatenate((np.full(6, 30.0), arm_limits[:, 1]))
        self.safety = ActionSafetyFilter(lower, upper, np.array([120.0] * 13))
        self.stability = StabilityCoordinator()
        self.arm_joint_ranges = self.backend.model.jnt_range[
            [
                mujoco.mj_name2id(self.backend.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{index}")
                for index in range(1, 8)
            ]
        ]
        reward_config = _load_yaml(PROJECT_ROOT / "configs" / "environment" / "tasks.yaml")
        self.reward_weights = dict(reward_config["reward_weights"])
        if reward_ablation is not None:
            if reward_ablation not in self.reward_weights:
                raise ValueError(f"Unknown reward ablation: {reward_ablation}")
            self.reward_weights[reward_ablation] = 0.0
        self.previous_action = np.zeros(14, dtype=np.float64)
        self.target_base_velocity = np.zeros(2)
        self.target_end_effector = np.zeros(3)
        self.target_end_effector_orientation = np.eye(3)
        self.target_heading = 0.0
        self.waypoint = np.zeros(3)
        self.place_target = np.array([0.65, 0.0, 0.76])
        self.delay = ObservationDelay(0)
        self.actuator_delay = ObservationDelay(0)
        self.randomization_sample = None
        self.external_push_step = -1
        self.disturbance_applied = False
        self.initial_block_position = np.zeros(3)
        self.step_count = 0
        self.previous_waypoint_distance = 0.0
        self.stage = 0
        self.manipulation_grasp_offset = np.array([0.0, 0.0, 0.015])
        self.manipulation_contact_steps = 0
        self.manipulation_joint_targets = {
            name: target.copy() for name, target in MANIPULATION_JOINT_TARGETS.items()
        }
        self.last_safety_result: SafetyFilterResult | None = None
        self.cumulative_energy = 0.0
        self.cumulative_slip = 0.0
        self._renderer: mujoco.Renderer | None = None

    def set_curriculum_stage(self, stage_index: int) -> None:
        """Schedule a curriculum stage; vector environments apply it on reset."""
        from moonbot.learning.curriculum import STAGES

        self.curriculum_stage_index = int(np.clip(stage_index, 0, len(STAGES) - 1))

    def _apply_curriculum(self) -> None:
        if not self.curriculum_enabled:
            return
        from moonbot.learning.curriculum import STAGES

        stage = STAGES[self.curriculum_stage_index]
        self.task = stage.task
        self.terrain_family = stage.terrain
        self.terrain_difficulty = stage.difficulty
        self.domain_randomization_enabled = bool(
            stage.randomization and self._allow_domain_randomization
        )

    @staticmethod
    def _yaw(state: RoverState) -> float:
        quaternion = state.orientation
        return float(
            np.arctan2(
                2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                1.0 - 2.0 * (quaternion[2] ** 2 + quaternion[3] ** 2),
            )
        )

    def _heading_error(self, state: RoverState) -> float:
        if self.task == "velocity_tracking":
            desired = self.target_heading
        elif self.task == "arm_reaching":
            desired = self._yaw(state)
        else:
            target = self.waypoint[:2]
            if self.task == "mobile_tracking":
                target = self.target_end_effector[:2] - np.array([0.55, 0.0])
            if self.task == "mobile_manipulation" and self.stage >= 1:
                return 0.0
            delta = target - state.position[:2]
            desired = float(np.arctan2(delta[1], delta[0]))
        return float((desired - self._yaw(state) + np.pi) % (2.0 * np.pi) - np.pi)

    def _task_targets(self, state: RoverState) -> None:
        if self.task == "velocity_tracking":
            self.target_base_velocity = self.np_random.uniform([-0.6, -0.5], [0.8, 0.5])
            self.target_end_effector = state.end_effector_position.copy()
        elif self.task in ("waypoint_navigation", "payload_transport"):
            self.waypoint = np.array(
                [self.np_random.uniform(2.0, 3.5), self.np_random.uniform(-0.8, 0.8), 0.0]
            )
            self.target_end_effector = state.end_effector_position.copy()
        elif self.task == "arm_reaching":
            self.target_end_effector = state.end_effector_position + self.np_random.uniform(
                [-0.12, -0.15, -0.12], [0.14, 0.15, 0.16]
            )
        elif self.task == "mobile_tracking":
            self.target_end_effector = state.end_effector_position + np.array([1.4, 0.35, 0.05])
        else:
            self.waypoint = np.array(
                [
                    state.block_position[0] - MANIPULATION_BASE_STANDOFF_M,
                    state.block_position[1],
                    0.0,
                ]
            )
            self.place_target = np.array([1.05, 0.0, 0.04])
            self.target_end_effector = state.block_position + np.array(
                [0.0, 0.0, MANIPULATION_PREGRASP_HEIGHT_M - state.block_position[2]]
            )
            self.target_end_effector_orientation = MANIPULATION_GRASP_ROTATION.copy()

    def _base_target(self, state: RoverState) -> np.ndarray:
        if self.task == "velocity_tracking":
            return self.target_base_velocity.copy()
        if self.task == "arm_reaching":
            return np.zeros(2)
        if not self.arm_base_coordination and self.task in (
            "mobile_tracking",
            "mobile_manipulation",
        ):
            return np.zeros(2)
        if self.task == "mobile_manipulation" and self.stage >= 1:
            yaw = self._yaw(state)
            delta = self.waypoint[:2] - state.position[:2]
            forward = np.cos(yaw) * delta[0] + np.sin(yaw) * delta[1]
            lateral = -np.sin(yaw) * delta[0] + np.cos(yaw) * delta[1]
            heading = (-yaw + np.pi) % (2.0 * np.pi) - np.pi
            return np.array(
                [
                    np.clip(1.2 * forward, -0.30, 0.30),
                    np.clip(1.5 * heading + 0.8 * lateral, -0.40, 0.40),
                ]
            )
        target = self.waypoint[:2]
        if self.task == "mobile_tracking":
            target = self.target_end_effector[:2] - np.array([0.55, 0.0])
        delta = target - state.position[:2]
        yaw = self._yaw(state)
        desired_heading = float(np.arctan2(delta[1], delta[0]))
        heading_error = (desired_heading - yaw + np.pi) % (2.0 * np.pi) - np.pi
        linear = np.clip(0.7 * np.linalg.norm(delta), 0.0, 0.7)
        if abs(heading_error) > 0.7:
            linear *= 0.25
        return np.array([linear, np.clip(1.4 * heading_error, -0.8, 0.8)])

    def _update_manipulation_stage(self, state: RoverState) -> None:
        if self.task != "mobile_manipulation":
            return
        block = state.block_position
        if self.stage == 0:
            self.target_end_effector = block + np.array(
                [0.0, 0.0, MANIPULATION_PREGRASP_HEIGHT_M - block[2]]
            )
            base_error = np.linalg.norm(state.position[:2] - self.waypoint[:2])
            if (
                base_error < 0.01
                and np.linalg.norm(state.linear_velocity[:2]) < 0.05
                and abs(self._yaw(state)) < 0.03
            ):
                self.backend.set_base_fixed(True)
                self._plan_manipulation_joint_targets(state)
                self.stage = 1
        elif self.stage == 1:
            self.target_end_effector = block + np.array(
                [0.0, 0.0, MANIPULATION_PREGRASP_HEIGHT_M - block[2]]
            )
            if np.linalg.norm(state.end_effector_position - self.target_end_effector) < 0.04:
                self.stage = 2
        elif self.stage == 2:
            self.target_end_effector = block + np.array(
                [0.0, 0.0, MANIPULATION_GRASP_HEIGHT_M - block[2]]
            )
            if self.backend.bilateral_block_contact():
                self.manipulation_contact_steps += 1
                if self.manipulation_contact_steps >= 1:
                    self.manipulation_grasp_offset = state.end_effector_position - block
                    self.stage = 3
            else:
                self.manipulation_contact_steps = 0
        elif self.stage == 3:
            if not self.backend.bilateral_block_contact():
                self.stage = 2
                self.manipulation_contact_steps = 0
                self.target_end_effector = block + np.array(
                    [0.0, 0.0, MANIPULATION_GRASP_HEIGHT_M - block[2]]
                )
                return
            self.target_end_effector = np.array(
                [
                    block[0] + self.manipulation_grasp_offset[0],
                    block[1] + self.manipulation_grasp_offset[1],
                    MANIPULATION_LIFT_HEIGHT_M,
                ]
            )
            if block[2] > 0.20 and self.backend.bilateral_block_contact():
                self.stage = 4
        elif self.stage == 4:
            self.target_end_effector = np.array(
                [
                    self.place_target[0] + self.manipulation_grasp_offset[0],
                    self.place_target[1] + self.manipulation_grasp_offset[1],
                    MANIPULATION_LIFT_HEIGHT_M,
                ]
            )
            if (
                np.linalg.norm(block[:2] - self.place_target[:2]) < 0.05
                and self.backend.bilateral_block_contact()
            ):
                self.stage = 5
        elif self.stage == 5:
            self.target_end_effector = self.place_target + self.manipulation_grasp_offset
            if (
                np.linalg.norm(block[:2] - self.place_target[:2]) < 0.05
                and self.backend.block_ground_contact()
            ):
                self.stage = 6
        elif self.stage == 6:
            self.target_end_effector = self.place_target + self.manipulation_grasp_offset
            if self.backend.block_ground_contact() and not self.backend.bilateral_block_contact():
                self.stage = 7

    def _plan_manipulation_joint_targets(self, state: RoverState) -> None:
        joint_names = tuple(f"joint{index}" for index in range(1, 8))
        block = state.block_position
        grasp = solve_site_pose_near(
            self.backend.model,
            self.backend.data.qpos,
            "end_effector",
            joint_names,
            np.array([block[0], block[1], MANIPULATION_GRASP_HEIGHT_M]),
            MANIPULATION_GRASP_ROTATION,
            MANIPULATION_JOINT_TARGETS["grasp"],
        )
        targets = {"grasp": grasp}
        previous = grasp
        for name, position in (
            ("pregrasp", np.array([block[0], block[1], MANIPULATION_PREGRASP_HEIGHT_M])),
            ("lift", np.array([block[0], block[1], MANIPULATION_LIFT_HEIGHT_M])),
            (
                "transport",
                np.array(
                    [self.place_target[0], self.place_target[1], MANIPULATION_LIFT_HEIGHT_M]
                ),
            ),
            (
                "lower",
                np.array(
                    [self.place_target[0], self.place_target[1], MANIPULATION_GRASP_HEIGHT_M]
                ),
            ),
        ):
            previous = solve_site_pose_near(
                self.backend.model,
                self.backend.data.qpos,
                "end_effector",
                joint_names,
                position,
                MANIPULATION_GRASP_ROTATION,
                previous,
            )
            targets[name] = previous
        self.manipulation_joint_targets = targets

    def _nominal_action(self, state: RoverState) -> tuple[np.ndarray, dict[str, Any]]:
        target = self._base_target(state)
        roll, pitch = quaternion_roll_pitch(state.orientation)
        slip = longitudinal_slip_ratio(
            state.wheel_velocities, float(state.linear_velocity[0]), 0.12
        )
        stability = (
            self.stability.evaluate(
                roll,
                pitch,
                state.stability_margin,
                slip,
                int(np.sum(state.wheel_contacts)),
            )
            if self.use_stability_coordinator
            else StabilityDecision(1.0, ())
        )
        metadata: dict[str, Any] = {
            "target_base_velocity": target,
            "stability_scale": stability.scale,
            "stability_triggers": stability.triggers,
            "slip_ratio": slip,
        }
        if self.task == "mobile_manipulation" and self.stage >= 1:
            yaw = self._yaw(state)
            delta = self.waypoint[:2] - state.position[:2]
            forward = np.cos(yaw) * delta[0] + np.sin(yaw) * delta[1]
            lateral = -np.sin(yaw) * delta[0] + np.cos(yaw) * delta[1]
            heading = (-yaw + np.pi) % (2.0 * np.pi) - np.pi
            linear_effort = np.clip(
                55.0 * forward - 16.0 * state.linear_velocity[0], -30.0, 30.0
            )
            angular_effort = np.clip(
                28.0 * heading
                + 18.0 * lateral
                - 10.0 * state.angular_velocity[2],
                -20.0,
                20.0,
            )
            wheel = np.array(
                [
                    linear_effort - angular_effort,
                    linear_effort - angular_effort,
                    linear_effort - angular_effort,
                    linear_effort + angular_effort,
                    linear_effort + angular_effort,
                    linear_effort + angular_effort,
                ]
            )
            wheel = np.clip(wheel, -30.0, 30.0)
        elif self.control_mode == "mpc":
            if self.mpc is None:
                raise RuntimeError("MPC controller was not initialized")
            result = self.mpc.command(
                float(target[0]),
                float(target[1]),
                float(state.linear_velocity[0]),
                float(state.angular_velocity[2]),
                stability.scale,
                slip,
            )
            wheel = result.wheel_torques
            metadata["mpc"] = {
                **asdict(result),
                "wheel_torques": result.wheel_torques.tolist(),
            }
        else:
            wheel = self.pid.command(
                float(target[0]),
                float(target[1]),
                float(state.linear_velocity[0]),
                float(state.angular_velocity[2]),
                self.control_dt,
            )
        wheel *= stability.scale
        if self.task == "mobile_manipulation":
            if self.stage <= 1:
                desired_joints = self.manipulation_joint_targets["pregrasp"]
            elif self.stage == 2:
                desired_joints = self.manipulation_joint_targets["grasp"]
            elif self.stage == 3:
                desired_joints = self.manipulation_joint_targets["lift"]
            elif self.stage == 4:
                desired_joints = self.manipulation_joint_targets["transport"]
            else:
                desired_joints = self.manipulation_joint_targets["lower"]
            delta = np.clip(desired_joints - self.backend.arm_controller.target, -0.035, 0.035)
            self.backend.arm_controller.target += delta
            arm = (
                300.0 * (self.backend.arm_controller.target - state.arm_positions)
                - 36.0 * state.arm_velocities
            )
            limits = self.backend.model.actuator_ctrlrange[
                self.backend.arm_controller.actuator_ids
            ]
            arm = np.clip(arm, limits[:, 0], limits[:, 1])
        elif self.task in ("arm_reaching", "mobile_tracking"):
            arm = self.osc.command(
                self.backend.data,
                self.target_end_effector,
                self.target_end_effector_orientation,
            )
        else:
            self.backend.arm_controller.target = PANDA_HOME.copy()
            arm = self.backend.gravity_compensation()
        return np.concatenate((wheel, arm)), metadata

    def _apply_action(
        self, state: RoverState, action: np.ndarray
    ) -> tuple[np.ndarray, float, dict[str, Any]]:
        nominal, metadata = self._nominal_action(state)
        if self.control_mode == "direct":
            arm_limits = self.backend.model.actuator_ctrlrange[
                self.backend.arm_controller.actuator_ids
            ][:, 1]
            requested = np.concatenate((30.0 * action[:6], arm_limits * action[6:13]))
        elif self.control_mode == "residual":
            requested = self.residual.combine(nominal, action[:13], self.control_dt)
        else:
            requested = nominal
        roll, pitch = quaternion_roll_pitch(state.orientation)
        if self.use_safety_filter:
            original_requested = requested.copy()
            self.last_safety_result = self.safety.project(
                requested,
                self.control_dt,
                roll=roll,
                pitch=pitch,
                stability_margin=state.stability_margin,
                arm_positions=state.arm_positions,
                arm_joint_ranges=self.arm_joint_ranges,
            )
            requested = self.last_safety_result.action
            metadata["safety"] = {
                **asdict(self.last_safety_result),
                "original_action": original_requested.tolist(),
                "filtered_action": self.last_safety_result.action.tolist(),
            }
        requested = self.actuator_delay.apply(requested).astype(np.float64)
        gripper = 0.04
        if self.task == "mobile_manipulation":
            ee_to_block = np.linalg.norm(state.end_effector_position - state.block_position)
            if (self.stage == 2 and ee_to_block < 0.10) or 3 <= self.stage <= 5:
                gripper = MANIPULATION_HOLD_OPENING_M
        gripper = float(np.clip(gripper + 0.005 * action[13], 0.0, 0.04))
        return requested, gripper, metadata

    def _observation(self, state: RoverState) -> np.ndarray:
        observation = deployment_observation(
            state,
            self.target_base_velocity,
            self.target_end_effector,
            self.previous_action,
            self.arm_joint_ranges,
            task_stage=self.stage,
        )
        noise_enabled = self.domain_randomization_enabled and (
            not self.curriculum_enabled or self.curriculum_stage_index >= 7
        )
        if noise_enabled:
            config = self.randomizer.config
            observation[3:6] += self.np_random.normal(
                0.0, float(config["imu_gyro_noise_std"]) / 5.0, size=3
            )
            observation[6:9] += self.np_random.normal(
                0.0, float(config["imu_accel_noise_std"]) / 20.0, size=3
            )
            encoder_noise = float(config["encoder_noise_std"])
            observation[9:35] += self.np_random.normal(0.0, encoder_noise, size=26)
            observation = np.clip(observation, -1.0, 1.0)
        if self.sensor_failure == "imu_dropout":
            observation[3:9] = 0.0
        elif self.sensor_failure == "wheel_encoder_dropout":
            observation[9:21] = 0.0
        elif self.sensor_failure not in (None, "none"):
            raise ValueError(f"Unknown sensor failure: {self.sensor_failure}")
        return self.delay.apply(observation)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        options = options or {}
        self._apply_curriculum()
        sample = self.randomizer.apply(
            self.np_random,
            self.domain_randomization_enabled,
            self.ood_randomization,
        )
        if self.forced_external_push_n is not None:
            sample = replace(sample, external_push_n=float(self.forced_external_push_n))
        if self.curriculum_enabled:
            from moonbot.learning.curriculum import STAGES

            stage = STAGES[self.curriculum_stage_index]
            if not stage.disturbances:
                sample = replace(sample, external_push_n=0.0)
            if self.curriculum_stage_index < 7:
                sample = replace(sample, observation_delay_steps=0, actuator_delay_steps=0)
        self.randomization_sample = sample
        terrain_family = str(options.get("terrain", self.terrain_family))
        difficulty = float(options.get("terrain_difficulty", self.terrain_difficulty))
        self.terrain.configure(TerrainSpecification(terrain_family, difficulty))
        manipulation = self.task == "mobile_manipulation"
        block_position = (
            np.array([0.90, 0.0, 0.04]) if manipulation else np.array([1.25, 0.0, 0.06])
        )
        self.backend.arm_controller.target = PANDA_HOME.copy()
        self.backend.arm_controller.kp = 300.0 if manipulation else 80.0
        self.backend.arm_controller.kd = 36.0 if manipulation else 6.0
        state = self.backend.reset(block_position=block_position)
        self.initial_block_position = state.block_position.copy()
        self.target_end_effector_orientation = state.end_effector_orientation.copy()
        self.target_heading = self._yaw(state)
        self.backend.set_base_fixed(self.task == "arm_reaching")
        self.pid.reset()
        if self.mpc is not None:
            self.mpc.reset()
        self.residual.reset()
        self.safety.reset()
        self.previous_action.fill(0.0)
        self.step_count = 0
        self.stage = 0
        self.manipulation_grasp_offset = np.array([0.0, 0.0, 0.015])
        self.manipulation_contact_steps = 0
        self.manipulation_joint_targets = {
            name: target.copy() for name, target in MANIPULATION_JOINT_TARGETS.items()
        }
        self.cumulative_energy = 0.0
        self.cumulative_slip = 0.0
        self.disturbance_applied = False
        self._task_targets(state)
        self.target_base_velocity = self._base_target(state)
        self.previous_waypoint_distance = float(
            np.linalg.norm(self.waypoint[:2] - state.position[:2])
        )
        initial = deployment_observation(
            state,
            self.target_base_velocity,
            self.target_end_effector,
            self.previous_action,
            self.arm_joint_ranges,
            task_stage=self.stage,
        )
        observation_delay = (
            sample.observation_delay_steps
            if self.fixed_observation_delay_steps is None
            else self.fixed_observation_delay_steps
        )
        self.delay = ObservationDelay(observation_delay)
        self.actuator_delay = ObservationDelay(sample.actuator_delay_steps)
        self.actuator_delay.reset(np.zeros(13, dtype=np.float32))
        self.external_push_step = (
            int(self.np_random.integers(10, max(11, self.maximum_steps - 10)))
            if sample.external_push_n > 0.0
            else -1
        )
        observation = self.delay.reset(initial)
        return observation, {
            "task": self.task,
            "terrain": terrain_family,
            "randomization": asdict(sample),
            "stage": self.stage,
        }

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float64)
        if action.shape != self.action_space.shape:
            raise ValueError(f"Expected action shape {self.action_space.shape}, got {action.shape}")
        action = np.clip(action, -1.0, 1.0)
        before = self.backend.state()
        invalid_initial_state = before.position[2] < 0.18
        stage_before = self.stage
        self._update_manipulation_stage(before)
        requested, gripper, controller_info = self._apply_action(before, action)
        state = before
        base_id = mujoco.mj_name2id(self.backend.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.backend.data.xfrc_applied[base_id] = 0.0
        disturbance_this_step = bool(
            self.randomization_sample is not None and self.step_count == self.external_push_step
        )
        if disturbance_this_step:
            angle = self.np_random.uniform(-np.pi, np.pi)
            force = self.randomization_sample.external_push_n
            self.backend.data.xfrc_applied[base_id, :2] = force * np.array(
                [np.cos(angle), np.sin(angle)]
            )
            self.disturbance_applied = True
        for _ in range(self.control_decimation):
            arm_torques = requested[6:13]
            if self.task == "mobile_manipulation" and self.control_mode != "direct":
                low_level_state = self.backend.state()
                arm_torques = (
                    300.0
                    * (self.backend.arm_controller.target - low_level_state.arm_positions)
                    - 36.0 * low_level_state.arm_velocities
                )
                joint_ids = np.array(
                    [
                        mujoco.mj_name2id(
                            self.backend.model,
                            mujoco.mjtObj.mjOBJ_JOINT,
                            f"joint{index}",
                        )
                        for index in range(1, 8)
                    ]
                )
                arm_torques += self.backend.data.qfrc_bias[
                    self.backend.model.jnt_dofadr[joint_ids]
                ]
                if self.control_mode == "residual":
                    arm_torques += 10.0 * action[6:13]
                limits = self.backend.model.actuator_ctrlrange[
                    self.backend.arm_controller.actuator_ids
                ]
                arm_torques = np.clip(arm_torques, limits[:, 0], limits[:, 1])
            state = self.backend.step(
                requested[:6],
                arm_torques,
                gripper_opening=gripper,
            )
        self.backend.data.xfrc_applied[base_id] = 0.0
        self.step_count += 1
        self.target_base_velocity = self._base_target(state)
        if self.task == "velocity_tracking":
            self.target_heading += self.target_base_velocity[1] * self.control_dt
        self._update_manipulation_stage(state)
        waypoint_distance = float(np.linalg.norm(self.waypoint[:2] - state.position[:2]))
        progress = self.previous_waypoint_distance - waypoint_distance
        self.previous_waypoint_distance = waypoint_distance
        velocity_error = np.array(
            [
                state.linear_velocity[0] - self.target_base_velocity[0],
                state.angular_velocity[2] - self.target_base_velocity[1],
            ]
        )
        ee_error = float(np.linalg.norm(state.end_effector_position - self.target_end_effector))
        ee_orientation_error = rotation_distance(
            self.target_end_effector_orientation,
            state.end_effector_orientation,
        )
        heading_error = self._heading_error(state)
        roll, pitch = quaternion_roll_pitch(state.orientation)
        slip = longitudinal_slip_ratio(
            state.wheel_velocities, float(state.linear_velocity[0]), 0.12
        )
        effort = float(np.mean(np.square(requested / np.maximum(np.abs(self.safety.upper), 1.0))))
        action_change = float(np.mean(np.square(action - self.previous_action)))
        collision = bool(self.backend.data.ncon > 40)
        success = self._success(state, waypoint_distance, ee_error)
        breakdown = task_reward(
            velocity_error=velocity_error,
            waypoint_progress=progress,
            end_effector_error=ee_error,
            roll=roll,
            pitch=pitch,
            slip_ratio=slip,
            squared_effort=effort,
            action_change=action_change,
            stability_margin=state.stability_margin,
            collision=collision,
            success=success,
            weights=self.reward_weights,
        )
        if self.task == "mobile_manipulation":
            stage_bonus = 5.0 * max(0, self.stage - stage_before)
            contact_bonus = 0.25 if self.backend.bilateral_block_contact() else 0.0
            shaping = stage_bonus + contact_bonus
            breakdown = replace(
                breakdown,
                total=breakdown.total + shaping,
                completion=breakdown.completion + shaping,
            )
        self.previous_action = action.copy()
        self.cumulative_energy += float(np.sum(np.abs(requested))) * self.control_dt
        self.cumulative_slip += slip
        failed = (
            invalid_initial_state
            or not np.all(np.isfinite(self.backend.data.qpos))
            or abs(roll) > np.deg2rad(40.0)
            or abs(pitch) > np.deg2rad(40.0)
            or state.position[2] < 0.18
        )
        if not np.all(np.isfinite(self.backend.data.qpos)):
            failure_category = "numerical"
        elif (
            state.position[2] < 0.18
            or abs(roll) > np.deg2rad(40.0)
            or abs(pitch) > np.deg2rad(40.0)
        ):
            failure_category = "tipover"
        elif collision:
            failure_category = "collision"
        else:
            failure_category = "none"
        terminated = bool(success or failed)
        truncated = self.step_count >= self.maximum_steps
        observation = self._observation(state)
        info: dict[str, Any] = {
            "is_success": success,
            "failure": failed,
            "stage": self.stage,
            "reward_terms": asdict(breakdown),
            "slip_ratio": slip,
            "stability_margin_m": state.stability_margin,
            "roll_rad": roll,
            "pitch_rad": pitch,
            "energy": self.cumulative_energy,
            "end_effector_error_m": ee_error,
            "end_effector_orientation_error_rad": ee_orientation_error,
            "position_error_m": waypoint_distance,
            "heading_error_rad": heading_error,
            "velocity_tracking_error": float(np.linalg.norm(velocity_error)),
            "peak_actuator_force": float(np.max(np.abs(state.actuator_forces))),
            "wheel_contact_impulse_ns": float(
                np.sum(state.wheel_contact_forces) * self.control_dt
            ),
            "collision": collision,
            "failure_category": failure_category,
            "disturbance_applied": disturbance_this_step,
            "external_push_n": (
                0.0
                if self.randomization_sample is None
                else self.randomization_sample.external_push_n
            ),
            "payload_displacement_m": float(
                np.linalg.norm(state.block_position - self.initial_block_position)
            ),
            "block_position_m": state.block_position.tolist(),
            "bilateral_block_contact": self.backend.bilateral_block_contact(),
            "block_ground_contact": self.backend.block_ground_contact(),
            "fingertip_forces_n": state.fingertip_forces.tolist(),
            "controller": controller_info,
            "grasp_attached": state.grasp_attached,
        }
        return observation, breakdown.total, terminated, truncated, info

    def _success(self, state: RoverState, waypoint_distance: float, ee_error: float) -> bool:
        if self.task == "velocity_tracking":
            return False
        if self.task in ("waypoint_navigation", "payload_transport"):
            return waypoint_distance < 0.25
        if self.task in ("arm_reaching", "mobile_tracking"):
            return ee_error < 0.035
        block_rotation = quaternion_to_rotation(state.block_orientation)
        return bool(
            self.stage == 7
            and np.linalg.norm(state.block_position[:2] - self.place_target[:2]) < 0.05
            and abs(state.block_position[2] - self.place_target[2]) < 0.01
            and block_rotation[2, 2] > 0.95
            and self.backend.block_ground_contact()
        )

    def render(self) -> np.ndarray | None:
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.backend.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.backend.data.qpos[:3] + np.array([0.4, 0.0, 0.35])
        camera.distance = 3.3
        camera.azimuth = 135.0
        camera.elevation = -20.0
        self._renderer.update_scene(self.backend.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
