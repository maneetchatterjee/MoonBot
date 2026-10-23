from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from moonbot.controllers.arm_pd import PANDA_HOME
from moonbot.controllers.pid_base import PIDAxis, SkidSteerPID
from moonbot.model.kinematics import (
    quaternion_to_rotation,
    solve_site_pose_multistart,
    solve_site_pose_near,
)
from moonbot.simulation.mujoco_backend import MuJoCoBackend, RoverState
from moonbot.simulation.terrain import TerrainManager, TerrainSpecification

JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
GRASP_ROTATION = np.diag([-1.0, 1.0, -1.0])
BLOCK_START = np.array([0.90, 0.0, 0.04])
PLACE_TARGET = np.array([1.05, 0.0, 0.04])
BASE_POSITION = np.array([0.30, 0.0, 0.417])
HOLD_OPENING = 0.0125
MOBILE_TRANSPORT_DISTANCE = 0.81
DRIVE_GOAL_MARGIN = 0.04


@dataclass(frozen=True)
class PickPlaceResult:
    passed: bool
    picked: bool
    transported: bool
    placed: bool
    final_block_position_m: list[float]
    placement_error_m: float
    maximum_block_height_m: float
    bilateral_contact_observed: bool
    bilateral_contact_time_s: float
    attachment_mode: str
    kinematic_attachment_used: bool
    stages: dict[str, bool]


@dataclass(frozen=True)
class MobilePickPlaceResult:
    passed: bool
    picked: bool
    transported: bool
    placed: bool
    rover_distance_m: float
    block_transport_distance_m: float
    final_block_position_m: list[float]
    target_position_m: list[float]
    placement_error_m: float
    maximum_block_height_m: float
    transport_contact_ratio: float
    maximum_grasp_slip_m: float
    wheel_rotation_rad: float
    maximum_suspension_travel_rad: float
    attachment_mode: str
    kinematic_attachment_used: bool
    stages: dict[str, bool]


class PickPlaceRecorder:
    def __init__(
        self,
        backend: MuJoCoBackend,
        output_path: str | Path,
        fps: int = 12,
        place_target_x: float = float(PLACE_TARGET[0]),
    ) -> None:
        self.backend = backend
        self.output_path = Path(output_path)
        self.fps = fps
        self.renderer = mujoco.Renderer(backend.model, height=720, width=960)
        self.camera = mujoco.MjvCamera()
        self.options = mujoco.MjvOption()
        self.frames: list[np.ndarray] = []
        self.next_frame_time = 0.0
        self.initial_base_x = float(backend.state().position[0])
        self.place_target_x = place_target_x

    def capture(self, stage: str, state: RoverState, *, force: bool = False) -> None:
        if not force and state.time + 1e-9 < self.next_frame_time:
            return
        manipulation_stage = stage not in {"reset", "settle", "complete"}
        if stage in {"rover_transport", "brake"}:
            self.camera.lookat[:] = np.array([0.95, 0.0, 0.43])
            self.camera.distance = 3.15
        elif manipulation_stage:
            self.camera.lookat[:] = 0.55 * state.block_position + 0.45 * state.position
            self.camera.lookat[2] = 0.42
            self.camera.distance = 2.25 if self.place_target_x > 1.3 else 1.85
        else:
            center_x = 0.5 * (self.initial_base_x + self.place_target_x)
            self.camera.lookat[:] = np.array([center_x, 0.0, 0.43])
            self.camera.distance = 3.0 if self.place_target_x > 1.3 else 2.75
        self.camera.azimuth = 132.0
        self.camera.elevation = -12.0
        self.renderer.update_scene(
            self.backend.data,
            camera=self.camera,
            scene_option=self.options,
        )
        image = Image.fromarray(self.renderer.render().copy())
        draw = ImageDraw.Draw(image)
        bilateral = self.backend.bilateral_block_contact()
        ground_supported = bool(state.block_position[2] <= 0.045)
        if bilateral:
            contact_text = "BILATERAL GRIP"
            contact_color = (92, 235, 168)
        elif ground_supported:
            contact_text = "GROUND SUPPORTED"
            contact_color = (255, 215, 128)
        else:
            contact_text = "open/free"
            contact_color = (220, 225, 230)
        draw.rounded_rectangle((14, 14, 425, 138), radius=8, fill=(12, 18, 24))
        draw.text((26, 23), f"Stage: {stage.replace('_', ' ').title()}", fill="white")
        draw.text(
            (26, 47),
            f"Object contact state: {contact_text}",
            fill=contact_color,
        )
        draw.text(
            (26, 69),
            "Kinematic attachment: OFF | ground pick/place",
            fill=(143, 214, 255),
        )
        draw.text(
            (26, 91),
            f"Rover travel: {state.position[0] - self.initial_base_x:+.2f} m",
            fill=(255, 215, 128),
        )
        draw.text(
            (26, 113),
            f"Fixed destination: x={self.place_target_x:.2f} m",
            fill=(176, 240, 190),
        )
        self.frames.append(np.asarray(image))
        self.next_frame_time = max(
            self.next_frame_time + 1.0 / self.fps,
            state.time + 1.0 / self.fps,
        )

    def close(self) -> None:
        import imageio.v3 as iio

        self.renderer.close()
        if not self.frames:
            raise RuntimeError("Pick-and-place recorder captured no frames")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(self.output_path, np.asarray(self.frames), fps=self.fps)


def _row(stage: str, state: RoverState, backend: MuJoCoBackend) -> dict[str, float | str]:
    return {
        "time_s": state.time,
        "stage": stage,
        "base_x_m": state.position[0],
        "end_effector_x_m": state.end_effector_position[0],
        "end_effector_y_m": state.end_effector_position[1],
        "end_effector_z_m": state.end_effector_position[2],
        "block_x_m": state.block_position[0],
        "block_y_m": state.block_position[1],
        "block_z_m": state.block_position[2],
        "left_fingertip_force_n": state.fingertip_forces[0],
        "right_fingertip_force_n": state.fingertip_forces[1],
        "bilateral_block_contact": float(backend.bilateral_block_contact()),
        "block_ground_contact": float(backend.block_ground_contact()),
        "grasp_attached": float(state.grasp_attached),
    }


def _arm_torque(backend: MuJoCoBackend, target: np.ndarray) -> np.ndarray:
    state = backend.state()
    torque = 300.0 * (target - state.arm_positions) - 36.0 * state.arm_velocities
    limits = backend.model.actuator_ctrlrange[backend.arm_controller.actuator_ids]
    return np.clip(torque, limits[:, 0], limits[:, 1])


def _observe(
    backend: MuJoCoBackend,
    state: RoverState,
    stage: str,
    step: int,
    rows: list[dict[str, float | str]],
    metrics: dict[str, float | bool],
    recorder: PickPlaceRecorder | None,
) -> None:
    bilateral = backend.bilateral_block_contact()
    metrics["maximum_height"] = max(float(metrics["maximum_height"]), state.block_position[2])
    metrics["bilateral_steps"] = float(metrics["bilateral_steps"]) + float(bilateral)
    metrics["bilateral_observed"] = bool(metrics["bilateral_observed"]) or bilateral
    metrics["attachment_seen"] = bool(metrics["attachment_seen"]) or state.grasp_attached
    if recorder is not None:
        recorder.capture(stage, state)
    if step % 10 == 0:
        rows.append(_row(stage, state, backend))


def _move_joint_target(
    backend: MuJoCoBackend,
    target: np.ndarray,
    duration_s: float,
    gripper_opening: float,
    stage: str,
    rows: list[dict[str, float | str]],
    metrics: dict[str, float | bool],
    recorder: PickPlaceRecorder | None,
) -> None:
    start = backend.state().arm_positions.copy()
    steps = round(duration_s / backend.timestep)
    for step in range(steps):
        phase = min(1.0, (step + 1) / max(1, int(0.8 * steps)))
        blend = phase * phase * (3.0 - 2.0 * phase)
        command = start + blend * (target - start)
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, command),
            gripper_opening=gripper_opening,
        )
        _observe(backend, state, stage, step, rows, metrics, recorder)


def _move_path(
    backend: MuJoCoBackend,
    targets: list[np.ndarray],
    duration_s: float,
    gripper_opening: float,
    stage: str,
    rows: list[dict[str, float | str]],
    metrics: dict[str, float | bool],
    recorder: PickPlaceRecorder | None,
) -> int:
    path = np.vstack((backend.state().arm_positions, *targets))
    steps = round(duration_s / backend.timestep)
    bilateral_steps = 0
    for step in range(steps):
        phase = (step + 1) / steps
        blend = phase * phase * (3.0 - 2.0 * phase)
        path_position = blend * (len(path) - 1)
        lower = min(int(path_position), len(path) - 2)
        fraction = path_position - lower
        command = (1.0 - fraction) * path[lower] + fraction * path[lower + 1]
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, command),
            gripper_opening=gripper_opening,
        )
        bilateral_steps += int(backend.bilateral_block_contact())
        _observe(backend, state, stage, step, rows, metrics, recorder)
    return bilateral_steps


def _pose_path(
    backend: MuJoCoBackend,
    positions: list[np.ndarray],
    initial_joints: np.ndarray,
) -> list[np.ndarray]:
    targets: list[np.ndarray] = []
    previous = initial_joints.copy()
    for position in positions:
        previous = solve_site_pose_near(
            backend.model,
            backend.data.qpos,
            "end_effector",
            JOINT_NAMES,
            position,
            GRASP_ROTATION,
            previous,
        )
        targets.append(previous)
    return targets


def run_pick_place(
    output_directory: str | Path,
    *,
    render: bool = False,
    video_output: str | Path | None = None,
    video_fps: int = 12,
) -> PickPlaceResult:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    backend = MuJoCoBackend()
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    state = backend.reset(base_position=BASE_POSITION, block_position=BLOCK_START)
    backend.set_base_fixed(True)
    backend.arm_controller.kp = 300.0
    backend.arm_controller.kd = 36.0
    recorder = (
        PickPlaceRecorder(backend, video_output or output / "pick_place.mp4", video_fps)
        if render
        else None
    )
    rows: list[dict[str, float | str]] = [_row("reset", state, backend)]
    metrics: dict[str, float | bool] = {
        "maximum_height": float(state.block_position[2]),
        "bilateral_steps": 0.0,
        "bilateral_observed": False,
        "attachment_seen": False,
    }
    stages: dict[str, bool] = {}
    if recorder is not None:
        recorder.capture("reset", state, force=True)

    for step in range(250):
        state = backend.step(np.zeros(6), gripper_opening=0.04)
        _observe(backend, state, "settle", step, rows, metrics, recorder)

    grasp = solve_site_pose_multistart(
        backend.model,
        backend.data.qpos,
        "end_effector",
        JOINT_NAMES,
        np.array([BLOCK_START[0], BLOCK_START[1], 0.055]),
        GRASP_ROTATION,
        PANDA_HOME,
    )
    pregrasp = solve_site_pose_near(
        backend.model,
        backend.data.qpos,
        "end_effector",
        JOINT_NAMES,
        np.array([BLOCK_START[0], BLOCK_START[1], 0.20]),
        GRASP_ROTATION,
        grasp,
    )
    _move_joint_target(backend, pregrasp, 3.0, 0.04, "pregrasp", rows, metrics, recorder)
    stages["pregrasp"] = bool(
        np.linalg.norm(backend.state().end_effector_position - np.array([0.90, 0.0, 0.20]))
        < 0.01
    )
    _move_joint_target(backend, grasp, 3.0, 0.04, "reach", rows, metrics, recorder)
    stages["reach"] = bool(
        np.linalg.norm(backend.state().end_effector_position - np.array([0.90, 0.0, 0.055]))
        < 0.01
    )

    grasp_contact = False
    for step in range(round(2.0 / backend.timestep)):
        close_target = max(0.0, 0.04 - 0.025 * step * backend.timestep)
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, grasp),
            gripper_opening=close_target,
        )
        _observe(backend, state, "close_gripper", step, rows, metrics, recorder)
        if backend.bilateral_block_contact():
            grasp_contact = True
            if recorder is not None:
                recorder.capture("bilateral_contact", state, force=True)
            break
    stages["bilateral_grasp"] = grasp_contact

    lift_positions = [
        np.array([0.90, 0.0, height]) for height in np.linspace(0.060, 0.315, 52)
    ]
    lift_path = _pose_path(backend, lift_positions, grasp)
    lift_contacts = _move_path(
        backend, lift_path, 1.2, HOLD_OPENING, "lift", rows, metrics, recorder
    )
    state = backend.state()
    stages["lift"] = bool(
        state.block_position[2] > 0.20
        and backend.bilateral_block_contact()
        and lift_contacts >= 0.95 * round(1.2 / backend.timestep)
    )

    transport_positions = [
        np.array([x_position, 0.0, state.end_effector_position[2]])
        for x_position in np.linspace(state.end_effector_position[0], PLACE_TARGET[0], 37)
    ]
    transport_path = _pose_path(backend, transport_positions, state.arm_positions)
    transport_contacts = _move_path(
        backend, transport_path, 1.0, HOLD_OPENING, "transport", rows, metrics, recorder
    )
    state = backend.state()
    transported = bool(
        abs(state.block_position[0] - BLOCK_START[0]) > 0.10
        and backend.bilateral_block_contact()
        and transport_contacts >= 0.95 * round(1.0 / backend.timestep)
    )
    stages["transport"] = transported

    grasp_offset = state.end_effector_position - state.block_position
    release_position = PLACE_TARGET + grasp_offset
    lower_positions = [
        np.array([release_position[0], release_position[1], height])
        for height in np.linspace(state.end_effector_position[2], release_position[2], 43)
    ]
    lower_path = _pose_path(backend, lower_positions, state.arm_positions)
    _move_path(
        backend,
        lower_path,
        0.8,
        HOLD_OPENING,
        "lower_to_ground",
        rows,
        metrics,
        recorder,
    )
    state = backend.state()
    stages["ground_lower"] = bool(state.block_position[2] <= 0.045)

    _move_joint_target(
        backend,
        state.arm_positions.copy(),
        0.8,
        0.04,
        "release",
        rows,
        metrics,
        recorder,
    )
    hold_target = backend.state().arm_positions.copy()
    for step in range(round(0.4 / backend.timestep)):
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, hold_target),
            gripper_opening=0.04,
        )
        _observe(backend, state, "released", step, rows, metrics, recorder)

    final_position = state.block_position.copy()
    placement_error = float(np.linalg.norm(final_position[:2] - PLACE_TARGET[:2]))
    block_rotation = quaternion_to_rotation(state.block_orientation)
    upright = bool(block_rotation[2, 2] > 0.95)
    placed = bool(
        placement_error < 0.04
        and abs(final_position[2] - PLACE_TARGET[2]) < 0.008
        and upright
    )
    stages["release_and_place"] = placed
    picked = bool(float(metrics["maximum_height"]) > 0.20)
    result = PickPlaceResult(
        passed=bool(all(stages.values()) and not bool(metrics["attachment_seen"])),
        picked=picked,
        transported=transported,
        placed=placed,
        final_block_position_m=final_position.tolist(),
        placement_error_m=placement_error,
        maximum_block_height_m=float(metrics["maximum_height"]),
        bilateral_contact_observed=bool(metrics["bilateral_observed"]),
        bilateral_contact_time_s=float(metrics["bilateral_steps"]) * backend.timestep,
        attachment_mode="friction_contact_only",
        kinematic_attachment_used=bool(metrics["attachment_seen"]),
        stages=stages,
    )
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(result.__dict__, stream, indent=2)
    if recorder is not None:
        recorder.capture("complete", state, force=True)
        recorder.close()
    return result


def _showcase_drive(
    backend: MuJoCoBackend,
    carry_target: np.ndarray,
    distance_m: float,
    rows: list[dict[str, float | str]],
    metrics: dict[str, float | bool],
    recorder: PickPlaceRecorder | None,
) -> tuple[RoverState, float, float, int, int]:
    controller = SkidSteerPID(
        linear=PIDAxis(kp=8.0, ki=0.5, kd=0.08, integral_limit=1.0),
        angular=PIDAxis(kp=5.0, ki=0.2, kd=0.05, integral_limit=0.5),
        max_torque=18.0,
        max_torque_rate=60.0,
    )
    state = backend.state()
    start_x = float(state.position[0])
    goal_x = start_x + distance_m + DRIVE_GOAL_MARGIN
    grasp_vector = state.end_effector_position - state.block_position
    maximum_slip = 0.0
    contact_steps = 0
    total_steps = 0
    wheel_torques = np.zeros(6)
    control_steps = max(1, round(0.01 / backend.timestep))
    backend.set_base_fixed(False)

    for step in range(round(12.0 / backend.timestep)):
        remaining = goal_x - float(state.position[0])
        if remaining < 0.012 and abs(float(state.linear_velocity[0])) < 0.035:
            break
        if step % control_steps == 0:
            target_speed = float(np.clip(0.65 * remaining, -0.06, 0.14))
            w, x, y, z = state.orientation
            yaw = float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
            wheel_torques = controller.command(
                target_speed,
                -1.2 * yaw,
                float(state.linear_velocity[0]),
                float(state.angular_velocity[2]),
                control_steps * backend.timestep,
            )
        state = backend.step(
            wheel_torques,
            arm_torques=_arm_torque(backend, carry_target),
            gripper_opening=HOLD_OPENING,
        )
        contact_steps += int(backend.bilateral_block_contact())
        total_steps += 1
        maximum_slip = max(
            maximum_slip,
            float(
                np.linalg.norm(
                    state.end_effector_position - state.block_position - grasp_vector
                )
            ),
        )
        _observe(backend, state, "rover_transport", step, rows, metrics, recorder)

    controller.reset()
    wheel_torques.fill(0.0)
    for step in range(round(1.2 / backend.timestep)):
        if step % control_steps == 0:
            wheel_torques = controller.command(
                0.0,
                0.0,
                float(state.linear_velocity[0]),
                float(state.angular_velocity[2]),
                control_steps * backend.timestep,
            )
        state = backend.step(
            wheel_torques,
            arm_torques=_arm_torque(backend, carry_target),
            gripper_opening=HOLD_OPENING,
        )
        contact_steps += int(backend.bilateral_block_contact())
        total_steps += 1
        maximum_slip = max(
            maximum_slip,
            float(
                np.linalg.norm(
                    state.end_effector_position - state.block_position - grasp_vector
                )
            ),
        )
        _observe(backend, state, "brake", step, rows, metrics, recorder)
    backend.set_base_fixed(True)
    return state, start_x, maximum_slip, contact_steps, total_steps


def run_mobile_pick_place_showcase(
    output_directory: str | Path,
    *,
    render: bool = True,
    video_output: str | Path | None = None,
    video_fps: int = 15,
    transport_distance_m: float = MOBILE_TRANSPORT_DISTANCE,
) -> MobilePickPlaceResult:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    backend = MuJoCoBackend()
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    state = backend.reset(base_position=BASE_POSITION, block_position=BLOCK_START)
    place_target = np.array(
        [BLOCK_START[0] + transport_distance_m, BLOCK_START[1], BLOCK_START[2]]
    )
    marker_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_SITE, "place_target")
    backend.model.site_pos[marker_id] = np.array([place_target[0], place_target[1], 0.0005])
    mujoco.mj_forward(backend.model, backend.data)
    backend.set_base_fixed(True)
    backend.arm_controller.kp = 300.0
    backend.arm_controller.kd = 36.0
    recorder = (
        PickPlaceRecorder(
            backend,
            video_output or output / "mobile_pick_place.mp4",
            video_fps,
            place_target_x=float(place_target[0]),
        )
        if render
        else None
    )
    rows: list[dict[str, float | str]] = [_row("reset", state, backend)]
    metrics: dict[str, float | bool] = {
        "maximum_height": float(state.block_position[2]),
        "bilateral_steps": 0.0,
        "bilateral_observed": False,
        "attachment_seen": False,
    }
    stages: dict[str, bool] = {}
    if recorder is not None:
        recorder.capture("reset", state, force=True)

    for step in range(250):
        state = backend.step(np.zeros(6), gripper_opening=0.04)
        _observe(backend, state, "settle", step, rows, metrics, recorder)

    grasp_position = np.array([BLOCK_START[0], BLOCK_START[1], 0.055])
    grasp = solve_site_pose_multistart(
        backend.model,
        backend.data.qpos,
        "end_effector",
        JOINT_NAMES,
        grasp_position,
        GRASP_ROTATION,
        PANDA_HOME,
    )
    pregrasp_position = grasp_position + np.array([0.0, 0.0, 0.145])
    pregrasp = solve_site_pose_near(
        backend.model,
        backend.data.qpos,
        "end_effector",
        JOINT_NAMES,
        pregrasp_position,
        GRASP_ROTATION,
        grasp,
    )
    _move_joint_target(backend, pregrasp, 3.0, 0.04, "pregrasp", rows, metrics, recorder)
    stages["pregrasp"] = bool(
        np.linalg.norm(backend.state().end_effector_position - pregrasp_position) < 0.01
    )
    _move_joint_target(backend, grasp, 3.0, 0.04, "reach", rows, metrics, recorder)
    stages["reach"] = bool(
        np.linalg.norm(backend.state().end_effector_position - grasp_position) < 0.01
    )

    grasp_contact = False
    for step in range(round(2.0 / backend.timestep)):
        close_target = max(0.0, 0.04 - 0.025 * step * backend.timestep)
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, grasp),
            gripper_opening=close_target,
        )
        _observe(backend, state, "close_gripper", step, rows, metrics, recorder)
        if backend.bilateral_block_contact():
            grasp_contact = True
            if recorder is not None:
                recorder.capture("bilateral_contact", state, force=True)
            break
    stages["bilateral_grasp"] = grasp_contact

    lift_positions = [
        np.array([BLOCK_START[0], BLOCK_START[1], height])
        for height in np.linspace(0.060, 0.315, 52)
    ]
    lift_path = _pose_path(backend, lift_positions, grasp)
    lift_contacts = _move_path(
        backend, lift_path, 1.4, HOLD_OPENING, "lift", rows, metrics, recorder
    )
    state = backend.state()
    stages["lift"] = bool(
        state.block_position[2] > 0.20
        and backend.bilateral_block_contact()
        and lift_contacts >= 0.98 * round(1.4 / backend.timestep)
    )

    carry_target = state.arm_positions.copy()
    wheel_start = state.wheel_positions.copy()
    suspension_start = state.suspension_positions.copy()
    block_before_drive = state.block_position.copy()
    state, drive_start_x, maximum_slip, contact_steps, drive_steps = _showcase_drive(
        backend,
        carry_target,
        transport_distance_m,
        rows,
        metrics,
        recorder,
    )
    rover_distance = float(state.position[0] - drive_start_x)
    block_transport_distance = float(state.block_position[0] - block_before_drive[0])
    contact_ratio = contact_steps / max(1, drive_steps)
    stages["rover_transport"] = bool(
        rover_distance > 0.70 * transport_distance_m
        and block_transport_distance > 0.70 * transport_distance_m
        and contact_ratio >= 0.98
        and maximum_slip < 0.015
        and backend.bilateral_block_contact()
    )

    grasp_offset = state.end_effector_position - state.block_position
    # Compensate the compliant arm/gripper deflection that appears when the block
    # first loads the ground while the wrist remains in its vertical grasp pose.
    release_position = place_target + grasp_offset + np.array([0.038, 0.0, 0.0])
    lower_positions = [
        np.array([release_position[0], release_position[1], height])
        for height in np.linspace(state.end_effector_position[2], release_position[2], 55)
    ]
    lower_path = _pose_path(backend, lower_positions, state.arm_positions)
    lower_contacts = _move_path(
        backend,
        lower_path,
        1.2,
        HOLD_OPENING,
        "lower_to_ground",
        rows,
        metrics,
        recorder,
    )
    _move_joint_target(
        backend,
        lower_path[-1],
        0.8,
        HOLD_OPENING,
        "placement_settle",
        rows,
        metrics,
        recorder,
    )
    state = backend.state()
    stages["ground_lower"] = bool(
        state.block_position[2] <= 0.045
        and lower_contacts >= 0.90 * round(1.2 / backend.timestep)
    )

    _move_joint_target(
        backend,
        state.arm_positions.copy(),
        0.9,
        0.04,
        "release",
        rows,
        metrics,
        recorder,
    )
    hold_target = backend.state().arm_positions.copy()
    for step in range(round(0.8 / backend.timestep)):
        state = backend.step(
            np.zeros(6),
            arm_torques=_arm_torque(backend, hold_target),
            gripper_opening=0.04,
        )
        _observe(backend, state, "released", step, rows, metrics, recorder)

    final_position = state.block_position.copy()
    placement_error = float(np.linalg.norm(final_position[:2] - place_target[:2]))
    upright = bool(quaternion_to_rotation(state.block_orientation)[2, 2] > 0.95)
    placed = bool(
        placement_error < 0.025
        and abs(final_position[2] - place_target[2]) < 0.008
        and upright
        and not backend.bilateral_block_contact()
    )
    stages["release_and_place"] = placed
    picked = bool(float(metrics["maximum_height"]) > 0.20)
    wheel_rotation = float(np.max(np.abs(state.wheel_positions - wheel_start)))
    suspension_travel = float(np.max(np.abs(state.suspension_positions - suspension_start)))
    result = MobilePickPlaceResult(
        passed=bool(all(stages.values()) and not bool(metrics["attachment_seen"])),
        picked=picked,
        transported=stages["rover_transport"],
        placed=placed,
        rover_distance_m=rover_distance,
        block_transport_distance_m=block_transport_distance,
        final_block_position_m=final_position.tolist(),
        target_position_m=place_target.tolist(),
        placement_error_m=placement_error,
        maximum_block_height_m=float(metrics["maximum_height"]),
        transport_contact_ratio=float(contact_ratio),
        maximum_grasp_slip_m=float(maximum_slip),
        wheel_rotation_rad=wheel_rotation,
        maximum_suspension_travel_rad=suspension_travel,
        attachment_mode="friction_contact_only",
        kinematic_attachment_used=bool(metrics["attachment_seen"]),
        stages=stages,
    )
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(result.__dict__, stream, indent=2)
    if recorder is not None:
        recorder.capture("complete", state, force=True)
        recorder.close()
    return result
