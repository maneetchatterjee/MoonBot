from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml

from moonbot.controllers.pid_base import PIDAxis, SkidSteerPID
from moonbot.evaluation.metrics import root_mean_square
from moonbot.model.parameters import DEFAULT_MODEL_PATH, PROJECT_ROOT
from moonbot.model.validation import validate_model
from moonbot.simulation.mujoco_backend import MuJoCoBackend
from moonbot.simulation.terrain import TerrainManager, TerrainSpecification


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, default=_json_default)
        stream.write("\n")


def _json_default(value: object) -> object:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def _validate(args: argparse.Namespace) -> int:
    report = validate_model(args.model)
    payload = report.to_dict()
    if args.output:
        _write_json(Path(args.output), payload)
    print(json.dumps(payload, indent=2))
    return 0 if report.passed else 1


def _load_pid(path: Path) -> SkidSteerPID:
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return SkidSteerPID(
        linear=PIDAxis(**config["linear"]),
        angular=PIDAxis(**config["angular"]),
        max_torque=float(config["maximum_wheel_torque"]),
        max_torque_rate=float(config["maximum_torque_rate"]),
    )


def _rollout(args: argparse.Namespace) -> int:
    if args.task != "velocity_tracking" or args.controller != "pid" or args.checkpoint:
        return _task_rollout(args)
    backend = MuJoCoBackend(args.model)
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    controller = _load_pid(PROJECT_ROOT / "configs" / "controller" / "pid.yaml")
    state = backend.reset()
    control_steps = max(1, round(0.01 / backend.timestep))
    rows: list[dict[str, float]] = []
    torques = np.zeros(6)
    while state.time < args.duration:
        if len(rows) % control_steps == 0:
            torques = controller.command(
                args.linear_velocity,
                args.angular_velocity,
                float(state.linear_velocity[0]),
                float(state.angular_velocity[2]),
                control_steps * backend.timestep,
            )
        state = backend.step(torques)
        rows.append(
            {
                "time_s": state.time,
                "x_m": float(state.position[0]),
                "y_m": float(state.position[1]),
                "z_m": float(state.position[2]),
                "linear_velocity_m_s": float(state.linear_velocity[0]),
                "angular_velocity_rad_s": float(state.angular_velocity[2]),
                "target_linear_velocity_m_s": args.linear_velocity,
                "target_angular_velocity_rad_s": args.angular_velocity,
                "mean_absolute_wheel_torque_nm": float(np.mean(np.abs(torques))),
            }
        )

    result_dir = Path(args.output)
    result_dir.mkdir(parents=True, exist_ok=True)
    with (result_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    warmup = min(len(rows) // 2, round(1.0 / backend.timestep))
    linear_errors = np.array(
        [row["linear_velocity_m_s"] - args.linear_velocity for row in rows[warmup:]]
    )
    summary = {
        "controller": "pid",
        "duration_s": args.duration,
        "target_linear_velocity_m_s": args.linear_velocity,
        "target_angular_velocity_rad_s": args.angular_velocity,
        "final_position_m": state.position.tolist(),
        "linear_velocity_rmse_m_s": root_mean_square(linear_errors),
        "simulation_steps": len(rows),
        "parameter_status": "provisional_composite_design",
    }
    _write_json(result_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2, default=_json_default))
    return 0


def _task_rollout(args: argparse.Namespace) -> int:
    import imageio.v3 as iio
    from stable_baselines3 import PPO

    from moonbot.environments.base import MoonBotEnv

    controller_modes = {
        "pid": ("pid", False),
        "mpc": ("mpc", False),
        "ppo": ("direct", False),
        "residual_ppo": ("residual", False),
        "residual_ppo_dr": ("residual", False),
        "residual_ppo_safe": ("residual", True),
    }
    control_mode, safety = controller_modes[args.controller]
    policy = None
    if args.controller not in ("pid", "mpc"):
        if not args.checkpoint:
            raise ValueError(f"{args.controller} requires --checkpoint")
        policy = PPO.load(args.checkpoint, device="cpu")
    env = MoonBotEnv(
        task=args.task,
        control_mode=control_mode,
        model_path=args.model,
        terrain=args.terrain,
        terrain_difficulty=args.terrain_difficulty,
        domain_randomization=args.domain_randomization,
        safety_filter=safety,
        forced_external_push_n=args.external_push_n,
        episode_seconds=args.duration,
        render_mode="rgb_array" if args.render else None,
    )
    observation, _ = env.reset(seed=args.seed)
    frames: list[np.ndarray] = []
    total_reward = 0.0
    final_info: dict[str, object] = {}
    steps = 0
    while True:
        action = (
            np.zeros(14, dtype=np.float32)
            if policy is None
            else policy.predict(observation, deterministic=True)[0]
        )
        observation, reward, terminated, truncated, final_info = env.step(action)
        total_reward += reward
        steps += 1
        if args.render and steps % 2 == 0:
            frame = env.render()
            if frame is not None:
                frames.append(frame)
        if terminated or truncated:
            break
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    summary = {
        "task": args.task,
        "controller": args.controller,
        "seed": args.seed,
        "steps": steps,
        "return": total_reward,
        "success": bool(final_info.get("is_success", False)),
        "final_info": final_info,
    }
    _write_json(output / "summary.json", summary)
    if frames:
        iio.imwrite(output / "rollout.mp4", np.asarray(frames), fps=12)
    env.close()
    print(json.dumps(summary, indent=2, default=_json_default))
    return 0


def _train(args: argparse.Namespace) -> int:
    from moonbot.learning.train import train_ppo

    checkpoint = train_ppo(
        args.config,
        args.output,
        total_timesteps=args.steps,
        seed=args.seed,
        ablation=args.ablation,
    )
    print(checkpoint.resolve())
    return 0


def _parse_checkpoints(values: list[str]) -> dict[str, str]:
    checkpoints: dict[str, str] = {}
    for value in values:
        controller, separator, path = value.partition("=")
        if not separator:
            raise ValueError("Checkpoint mappings must use controller=path")
        checkpoints[controller] = path
    return checkpoints


def _evaluate(args: argparse.Namespace) -> int:
    from moonbot.evaluation.benchmark import run_benchmark

    checkpoints = {args.controller: args.checkpoint} if args.checkpoint else {}
    _, summary = run_benchmark(
        [args.controller],
        args.scenario_manifest,
        args.output,
        checkpoints,
        args.episodes,
        args.maximum_steps,
        args.policy_seed,
        args.label,
    )
    print(summary.to_string(index=False))
    return 0


def _benchmark(args: argparse.Namespace) -> int:
    from moonbot.evaluation.benchmark import run_benchmark

    _, summary = run_benchmark(
        args.controllers,
        args.scenario_manifest,
        args.output,
        _parse_checkpoints(args.checkpoint),
        args.episodes,
        args.maximum_steps,
        args.policy_seed,
        args.label,
    )
    print(summary.to_string(index=False))
    return 0


def _aggregate(args: argparse.Namespace) -> int:
    from moonbot.evaluation.benchmark import combine_benchmark_results

    _, summary = combine_benchmark_results(args.inputs, args.output)
    print(summary.to_string(index=False))
    return 0


def _plot(args: argparse.Namespace) -> int:
    import pandas as pd

    from moonbot.evaluation.plots import create_benchmark_plots

    source = Path(args.results) / "episodes.parquet"
    paths = create_benchmark_plots(pd.read_parquet(source), Path(args.results) / "plots")
    print("\n".join(str(path.resolve()) for path in paths))
    return 0


def _plot_learning(args: argparse.Namespace) -> int:
    from moonbot.evaluation.plots import create_learning_curve

    path = create_learning_curve(args.inputs, args.output)
    print(path.resolve())
    return 0


def _pick_place(args: argparse.Namespace) -> int:
    from dataclasses import asdict

    from moonbot.evaluation.pick_place import run_pick_place

    result = run_pick_place(
        args.output,
        render=args.render,
        video_output=args.video_output,
        video_fps=args.fps,
    )
    print(json.dumps(asdict(result), indent=2))
    return 0 if result.passed else 1


def _mobile_pick_place(args: argparse.Namespace) -> int:
    from dataclasses import asdict

    from moonbot.evaluation.pick_place import run_mobile_pick_place_showcase

    result = run_mobile_pick_place_showcase(
        args.output,
        render=not args.no_render,
        video_output=args.video_output,
        video_fps=args.fps,
        transport_distance_m=args.distance,
    )
    print(json.dumps(asdict(result), indent=2))
    return 0 if result.passed else 1


def _render_model(args: argparse.Namespace) -> int:
    from PIL import Image

    backend = MuJoCoBackend(args.model)
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    backend.reset()
    display_pose = np.array([0.0, -0.45, 0.0, -1.55, 0.0, 1.15, 0.0])
    backend.arm_controller.target = display_pose.copy()
    for index, target in enumerate(display_pose, start=1):
        joint_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{index}")
        backend.data.qpos[backend.model.jnt_qposadr[joint_id]] = target
    mujoco.mj_forward(backend.model, backend.data)
    for _ in range(round(1.0 / backend.timestep)):
        backend.step(np.zeros(6), gripper_opening=0.035)
    camera = mujoco.MjvCamera()
    if args.view == "suspension":
        for name, value in (
            ("left_rocker_joint", 0.12),
            ("right_rocker_joint", -0.12),
            ("left_bogie_joint", 0.28),
        ):
            joint_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            backend.data.qpos[backend.model.jnt_qposadr[joint_id]] = value
        mujoco.mj_forward(backend.model, backend.data)
        camera.lookat[:] = np.array([0.0, -0.18, 0.30])
        camera.distance = 1.75
        camera.azimuth = 155.0
        camera.elevation = -12.0
    elif args.view == "gripper":
        end_effector_id = mujoco.mj_name2id(backend.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        camera.lookat[:] = backend.data.site_xpos[end_effector_id]
        camera.distance = 0.62
        camera.azimuth = 132.0
        camera.elevation = -12.0
    else:
        camera.lookat[:] = np.array([0.05, 0.0, 0.72])
        camera.distance = 3.45
        camera.azimuth = 132.0
        camera.elevation = -18.0
    with mujoco.Renderer(backend.model, height=args.height, width=args.width) as renderer:
        renderer.update_scene(backend.data, camera=camera)
        pixels = renderer.render()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(output)
    print(output.resolve())
    return 0


def _gripper_demo(args: argparse.Namespace) -> int:
    backend = MuJoCoBackend(args.model)
    TerrainManager(backend.model).configure(TerrainSpecification("flat"))
    initial = backend.reset().finger_positions
    phase_steps = round(args.phase_duration / backend.timestep)
    for _ in range(phase_steps):
        backend.step(np.zeros(6), gripper_opening=0.04)
    opened = backend.state().finger_positions
    for _ in range(phase_steps):
        backend.step(np.zeros(6), gripper_opening=0.0)
    closed = backend.state().finger_positions
    summary = {
        "initial_finger_positions_m": initial.tolist(),
        "opened_finger_positions_m": opened.tolist(),
        "closed_finger_positions_m": closed.tolist(),
        "phase_duration_s": args.phase_duration,
        "passed": bool(np.all(opened > 0.039) and np.all(closed < 0.001)),
    }
    _write_json(Path(args.output) / "summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m moonbot.cli")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate-model", help="Run mechanical model checks")
    validate.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    validate.add_argument("--output")
    validate.set_defaults(function=_validate)

    rollout = subparsers.add_parser("rollout", help="Run a task rollout")
    rollout.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    rollout.add_argument("--duration", type=float, default=5.0)
    rollout.add_argument("--linear-velocity", type=float, default=0.5)
    rollout.add_argument("--angular-velocity", type=float, default=0.0)
    rollout.add_argument("--output", default="results/pid_smoke")
    rollout.add_argument(
        "--task",
        choices=(
            "velocity_tracking",
            "waypoint_navigation",
            "arm_reaching",
            "mobile_tracking",
            "payload_transport",
            "mobile_manipulation",
        ),
        default="velocity_tracking",
    )
    rollout.add_argument(
        "--controller",
        choices=(
            "pid",
            "mpc",
            "ppo",
            "residual_ppo",
            "residual_ppo_dr",
            "residual_ppo_safe",
        ),
        default="pid",
    )
    rollout.add_argument("--checkpoint")
    rollout.add_argument("--terrain", default="flat")
    rollout.add_argument("--terrain-difficulty", type=float, default=0.0)
    rollout.add_argument("--domain-randomization", action="store_true")
    rollout.add_argument("--external-push-n", type=float)
    rollout.add_argument("--seed", type=int, default=0)
    rollout.add_argument("--render", action="store_true")
    rollout.set_defaults(function=_rollout)

    render = subparsers.add_parser("render-model", help="Render the assembled rover and arm")
    render.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    render.add_argument("--output", default="docs/images/moonbot_x.png")
    render.add_argument("--width", type=int, default=1280)
    render.add_argument("--height", type=int, default=720)
    render.add_argument("--view", choices=("overview", "suspension", "gripper"), default="overview")
    render.set_defaults(function=_render_model)

    gripper = subparsers.add_parser("gripper-demo", help="Verify full gripper travel")
    gripper.add_argument("--model", default=str(DEFAULT_MODEL_PATH))
    gripper.add_argument("--phase-duration", type=float, default=0.5)
    gripper.add_argument("--output", default="results/gripper_smoke")
    gripper.set_defaults(function=_gripper_demo)

    pick_place = subparsers.add_parser(
        "pick-place-demo", help="Run the staged drive, grasp, transport, and place pipeline"
    )
    pick_place.add_argument("--output", default="results/pick_place_smoke")
    pick_place.add_argument("--render", action="store_true")
    pick_place.add_argument("--video-output")
    pick_place.add_argument("--fps", type=int, default=12)
    pick_place.set_defaults(function=_pick_place)

    mobile_pick_place = subparsers.add_parser(
        "mobile-pick-place-showcase",
        help="Render a physical ground pickup, rover transport, and ground placement",
    )
    mobile_pick_place.add_argument("--output", default="results/mobile_pick_place_showcase")
    mobile_pick_place.add_argument("--video-output")
    mobile_pick_place.add_argument("--fps", type=int, default=15)
    mobile_pick_place.add_argument("--distance", type=float, default=0.81)
    mobile_pick_place.add_argument("--no-render", action="store_true")
    mobile_pick_place.set_defaults(function=_mobile_pick_place)

    train = subparsers.add_parser("train", help="Train PPO or residual PPO")
    train.add_argument("--config", default="configs/training/residual_ppo.yaml")
    train.add_argument("--output", default="results/training")
    train.add_argument("--steps", type=int)
    train.add_argument("--seed", type=int)
    train.add_argument(
        "--ablation",
        choices=(
            "none",
            "no_domain_randomization",
            "no_safety_filter",
            "no_stability_penalty",
            "no_slip_penalty",
            "no_stability_coordinator",
            "no_arm_base_coordination",
        ),
        default="none",
    )
    train.set_defaults(function=_train)

    evaluate = subparsers.add_parser("evaluate", help="Evaluate one controller")
    evaluate.add_argument("--controller", default="residual_ppo_safe")
    evaluate.add_argument("--checkpoint")
    evaluate.add_argument("--scenario-manifest", default="configs/evaluation/full_benchmark.yaml")
    evaluate.add_argument("--output", default="results/evaluation")
    evaluate.add_argument("--episodes", type=int)
    evaluate.add_argument("--maximum-steps", type=int)
    evaluate.add_argument("--policy-seed", type=int)
    evaluate.add_argument("--label")
    evaluate.set_defaults(function=_evaluate)

    benchmark = subparsers.add_parser("benchmark", help="Run paired controller scenarios")
    benchmark.add_argument("--controllers", nargs="+", default=["pid", "mpc"])
    benchmark.add_argument("--checkpoint", action="append", default=[])
    benchmark.add_argument("--scenario-manifest", default="configs/evaluation/full_benchmark.yaml")
    benchmark.add_argument("--output", default="results/full_benchmark")
    benchmark.add_argument("--episodes", type=int)
    benchmark.add_argument("--maximum-steps", type=int)
    benchmark.add_argument("--policy-seed", type=int)
    benchmark.add_argument("--label")
    benchmark.set_defaults(function=_benchmark)

    aggregate = subparsers.add_parser(
        "aggregate", help="Combine episode-level results across training seeds"
    )
    aggregate.add_argument("--inputs", nargs="+", required=True)
    aggregate.add_argument("--output", required=True)
    aggregate.set_defaults(function=_aggregate)

    plot = subparsers.add_parser("plot", help="Generate benchmark figures")
    plot.add_argument("--results", default="results/full_benchmark")
    plot.set_defaults(function=_plot)

    plot_learning = subparsers.add_parser(
        "plot-learning", help="Generate learning curves from SB3 progress logs"
    )
    plot_learning.add_argument("--inputs", nargs="+", required=True)
    plot_learning.add_argument("--output", required=True)
    plot_learning.set_defaults(function=_plot_learning)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.function(args))


if __name__ == "__main__":
    raise SystemExit(main())
