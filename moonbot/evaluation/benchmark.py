from __future__ import annotations

import json
import platform
import shutil
import subprocess
import sys
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from time import perf_counter
from typing import Any

import mujoco
import numpy as np
import pandas as pd
import torch
from stable_baselines3 import PPO

from moonbot.environments.base import MoonBotEnv
from moonbot.evaluation.metrics import paired_mean_difference
from moonbot.evaluation.scenarios import Scenario, load_manifest
from moonbot.evaluation.statistics import aggregate_episodes
from moonbot.model.parameters import PROJECT_ROOT

PAIRED_METRICS = (
    "success",
    "return",
    "position_rmse_m",
    "heading_rmse_rad",
    "end_effector_rmse_m",
    "mean_slip_ratio",
    "energy",
    "minimum_stability_margin_m",
    "collision_rate",
)


def environment_record() -> dict[str, Any]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "uncommitted"
    return {
        "git_commit": commit,
        "python": sys.version,
        "platform": platform.platform(),
        "mujoco": mujoco.__version__,
        "torch": torch.__version__,
        "device": "cuda" if torch.cuda.is_available() else "cpu",
        "timestamp_utc": datetime.now(UTC).isoformat(),
    }


def _controller_settings(controller: str) -> tuple[str, bool]:
    if controller == "pid":
        return "pid", False
    if controller == "mpc":
        return "mpc", False
    if controller == "ppo":
        return "direct", False
    if controller == "residual_ppo":
        return "residual", False
    if controller == "residual_ppo_dr":
        return "residual", False
    if controller == "residual_ppo_safe":
        return "residual", True
    raise ValueError(f"Unknown controller: {controller}")


def _load_policy(controller: str, checkpoint: str | Path | None) -> PPO | None:
    if controller in ("pid", "mpc"):
        return None
    if checkpoint is None:
        raise ValueError(f"Controller {controller} requires a checkpoint")
    return PPO.load(str(checkpoint), device="cpu")


def _paired_table(episodes: pd.DataFrame, baseline: str) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    baseline_rows = episodes[episodes["controller"] == baseline]
    for controller in sorted(set(episodes["controller"]) - {baseline}):
        controller_rows = episodes[episodes["controller"] == controller]
        for scenario in sorted(episodes["scenario"].unique()):
            first = controller_rows[controller_rows["scenario"] == scenario]
            second = baseline_rows[baseline_rows["scenario"] == scenario]
            if first.empty or second.empty:
                continue
            paired = first.merge(second, on=["scenario", "seed"], suffixes=("", "_baseline"))
            for metric in PAIRED_METRICS:
                values = paired[metric].to_numpy(dtype=np.float64)
                reference = paired[f"{metric}_baseline"].to_numpy(dtype=np.float64)
                finite = np.isfinite(values) & np.isfinite(reference)
                values = values[finite]
                reference = reference[finite]
                if len(values) == 0:
                    continue
                if len(values) >= 2:
                    mean, lower, upper = paired_mean_difference(values, reference, samples=2000)
                else:
                    mean = lower = upper = float(values[0] - reference[0])
                rows.append(
                    {
                        "controller": controller,
                        "baseline": baseline,
                        "scenario": scenario,
                        "metric": metric,
                        "mean_difference": mean,
                        "ci95_low": lower,
                        "ci95_high": upper,
                        "pairs": len(values),
                    }
                )
    return pd.DataFrame(rows)


def run_episode(
    controller: str,
    scenario: Scenario,
    seed: int,
    checkpoint: str | Path | None = None,
    maximum_steps: int | None = None,
    safety_event_sink: list[dict[str, Any]] | None = None,
    trajectory_sink: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    control_mode, safety = _controller_settings(controller)
    env = MoonBotEnv(
        task=scenario.task,
        control_mode=control_mode,
        terrain=scenario.terrain,
        terrain_difficulty=scenario.terrain_difficulty,
        domain_randomization=scenario.domain_randomization,
        safety_filter=safety,
        sensor_failure=scenario.sensor_failure,
        ood_randomization=scenario.ood_randomization,
        fixed_observation_delay_steps=scenario.observation_delay_steps,
        forced_external_push_n=scenario.external_push_n,
    )
    policy = _load_policy(controller, checkpoint)
    observation, _ = env.reset(seed=seed)
    returns = 0.0
    position_errors: list[float] = []
    heading_errors: list[float] = []
    ee_errors: list[float] = []
    ee_orientation_errors: list[float] = []
    slips: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    margins: list[float] = []
    latencies: list[float] = []
    mpc_solve_times: list[float] = []
    peak_actuator_force = 0.0
    contact_impulse = 0.0
    collision_steps = 0
    maximum_payload_displacement = 0.0
    safety_interventions = 0
    constraint_violations = 0
    mpc_fallbacks = 0
    success = False
    steps = 0
    disturbance_step: int | None = None
    recovery_time_s = float("nan")
    recovery_counter = 0
    episode_event_indices: list[int] = []
    while True:
        started = perf_counter()
        if policy is None:
            action = np.zeros(14, dtype=np.float32)
        else:
            action, _ = policy.predict(observation, deterministic=True)
        observation, reward, terminated, truncated, info = env.step(action)
        latencies.append(perf_counter() - started)
        returns += reward
        position_errors.append(float(info["position_error_m"]))
        heading_errors.append(float(info["heading_error_rad"]))
        ee_errors.append(float(info["end_effector_error_m"]))
        ee_orientation_errors.append(float(info["end_effector_orientation_error_rad"]))
        slips.append(float(info["slip_ratio"]))
        rolls.append(abs(float(info["roll_rad"])))
        pitches.append(abs(float(info["pitch_rad"])))
        margins.append(float(info["stability_margin_m"]))
        peak_actuator_force = max(peak_actuator_force, float(info["peak_actuator_force"]))
        contact_impulse += float(info["wheel_contact_impulse_ns"])
        collision_steps += int(info["collision"])
        maximum_payload_displacement = max(
            maximum_payload_displacement,
            float(info["payload_displacement_m"]),
        )
        controller_info = info["controller"]
        safety_info = controller_info.get("safety")
        if safety_info and safety_info["intervened"]:
            safety_interventions += 1
            if safety_event_sink is not None:
                episode_event_indices.append(len(safety_event_sink))
                safety_event_sink.append(
                    {
                        "controller": controller,
                        "scenario": scenario.identifier,
                        "seed": seed,
                        "step": steps,
                        "time_s": steps * env.control_dt,
                        "triggers": "|".join(safety_info["triggers"]),
                        "original_action": json.dumps(safety_info["original_action"]),
                        "filtered_action": json.dumps(safety_info["filtered_action"]),
                        "solve_time_s": float(safety_info["solve_time_s"]),
                        "status": str(safety_info["status"]),
                    }
                )
        mpc_info = controller_info.get("mpc")
        if mpc_info:
            mpc_solve_times.append(float(mpc_info["solve_time_s"]))
            if mpc_info["fallback_used"]:
                mpc_fallbacks += 1
        constraint_violations += int(
            bool(info["failure"]) or bool(safety_info and safety_info["triggers"])
        )
        if info["disturbance_applied"]:
            disturbance_step = steps
            recovery_counter = 0
        elif disturbance_step is not None and not np.isfinite(recovery_time_s):
            if float(info["velocity_tracking_error"]) < 0.15:
                recovery_counter += 1
                if recovery_counter >= 3:
                    recovery_time_s = (steps - disturbance_step) * env.control_dt
            else:
                recovery_counter = 0
        success = bool(info["is_success"])
        if trajectory_sink is not None:
            state = env.backend.state()
            trajectory_sink.append(
                {
                    "controller": controller,
                    "scenario": scenario.identifier,
                    "seed": seed,
                    "time_s": steps * env.control_dt,
                    "x_m": float(state.position[0]),
                    "y_m": float(state.position[1]),
                    "z_m": float(state.position[2]),
                    "yaw_rad": env._yaw(state),
                    "roll_rad": float(info["roll_rad"]),
                    "pitch_rad": float(info["pitch_rad"]),
                    "position_error_m": float(info["position_error_m"]),
                    "end_effector_error_m": float(info["end_effector_error_m"]),
                    "slip_ratio": float(info["slip_ratio"]),
                    "stability_margin_m": float(info["stability_margin_m"]),
                    "stage": int(info["stage"]),
                }
            )
        steps += 1
        if terminated or truncated or (maximum_steps is not None and steps >= maximum_steps):
            break
    if disturbance_step is not None and not np.isfinite(recovery_time_s):
        recovery_time_s = (steps - disturbance_step) * env.control_dt
    for event_index in episode_event_indices:
        safety_event_sink[event_index]["episode_success"] = success
    result = {
        "controller": controller,
        "scenario": scenario.identifier,
        "task": scenario.task,
        "terrain": scenario.terrain,
        "terrain_difficulty": scenario.terrain_difficulty,
        "domain_randomization": scenario.domain_randomization,
        "ood_randomization": scenario.ood_randomization,
        "sensor_failure": scenario.sensor_failure or "none",
        "external_push_n": scenario.external_push_n or 0.0,
        "seed": seed,
        "success": float(success),
        "return": float(returns),
        "duration_s": steps * env.control_dt,
        "position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "heading_rmse_rad": float(np.sqrt(np.mean(np.square(heading_errors)))),
        "end_effector_rmse_m": float(np.sqrt(np.mean(np.square(ee_errors)))),
        "end_effector_orientation_rmse_rad": float(
            np.sqrt(np.mean(np.square(ee_orientation_errors)))
        ),
        "mean_slip_ratio": float(np.mean(slips)),
        "energy": float(env.cumulative_energy),
        "peak_actuator_force": peak_actuator_force,
        "wheel_contact_impulse_ns": contact_impulse,
        "collision_rate": collision_steps / steps,
        "payload_displacement_m": maximum_payload_displacement,
        "peak_roll_rad": max(rolls),
        "peak_pitch_rad": max(pitches),
        "minimum_stability_margin_m": min(margins),
        "constraint_violation_rate": constraint_violations / steps,
        "safety_intervention_rate": safety_interventions / steps,
        "controller_latency_mean_s": float(np.mean(latencies)),
        "mpc_solve_time_mean_s": (
            float(np.mean(mpc_solve_times)) if mpc_solve_times else float("nan")
        ),
        "mpc_fallback_rate": mpc_fallbacks / steps,
        "recovery_time_s": recovery_time_s,
        "recovery_observed": float(
            disturbance_step is not None and recovery_counter >= 3
        ),
        "failure_category": "none" if success else info["failure_category"],
    }
    if not success and result["failure_category"] == "none":
        result["failure_category"] = "timeout"
    env.close()
    return result


def run_benchmark(
    controllers: list[str],
    manifest_path: str | Path,
    output_directory: str | Path,
    checkpoints: dict[str, str | Path] | None = None,
    episodes_override: int | None = None,
    maximum_steps: int | None = None,
    policy_seed: int | None = None,
    result_label: str | None = None,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    started_utc = datetime.now(UTC)
    manifest = load_manifest(manifest_path)
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    checkpoints = checkpoints or {}
    rows: list[dict[str, Any]] = []
    safety_events: list[dict[str, Any]] = []
    representative_trajectories: list[dict[str, Any]] = []
    episodes = episodes_override or manifest.episodes_per_scenario
    for controller in controllers:
        for scenario_index, scenario in enumerate(manifest.scenarios):
            for episode in range(episodes):
                seed = manifest.base_seed + scenario_index * 10_000 + episode
                row = run_episode(
                    controller,
                    scenario,
                    seed,
                    checkpoints.get(controller),
                    maximum_steps,
                    safety_events,
                    representative_trajectories if episode == 0 else None,
                )
                row["training_seed"] = policy_seed if policy_seed is not None else -1
                if result_label is not None:
                    row["controller"] = result_label
                rows.append(row)
    episode_frame = pd.DataFrame(rows)
    summary = aggregate_episodes(episode_frame)
    episode_frame.to_parquet(output / "episodes.parquet", index=False)
    pd.DataFrame(safety_events).to_parquet(
        output / "safety_interventions.parquet", index=False
    )
    pd.DataFrame(representative_trajectories).to_parquet(
        output / "representative_trajectories.parquet", index=False
    )
    summary.to_csv(output / "summary.csv", index=False)
    (
        episode_frame.groupby(["controller", "scenario", "failure_category"])
        .size()
        .rename("count")
        .reset_index()
        .to_csv(output / "failure_counts.csv", index=False)
    )
    baseline = controllers[0]
    _paired_table(episode_frame, baseline).to_csv(
        output / "paired_differences.csv", index=False
    )
    with (output / "environment.json").open("w", encoding="utf-8") as stream:
        json.dump(environment_record(), stream, indent=2)
    shutil.copy2(manifest.source, output / "manifest.yaml")
    with (output / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "name": manifest.name,
                "episodes_per_scenario": episodes,
                "base_seed": manifest.base_seed,
                "scenarios": [asdict(scenario) for scenario in manifest.scenarios],
            },
            stream,
            indent=2,
            default=str,
        )
    with (output / "run.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "controllers": controllers,
                "episodes_per_scenario": episodes,
                "scenarios": [asdict(scenario) for scenario in manifest.scenarios],
                "checkpoints": {key: str(value) for key, value in checkpoints.items()},
                "started_utc": started_utc.isoformat(),
                "ended_utc": datetime.now(UTC).isoformat(),
            },
            stream,
            indent=2,
        )
    return episode_frame, summary


def combine_benchmark_results(
    input_directories: list[str | Path], output_directory: str | Path
) -> tuple[pd.DataFrame, pd.DataFrame]:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    safety_frames = []
    trajectory_frames = []
    for directory in input_directories:
        directory = Path(directory)
        frame = pd.read_parquet(directory / "episodes.parquet")
        if "training_seed" not in frame:
            frame["training_seed"] = -1
        frames.append(frame)
        safety_path = directory / "safety_interventions.parquet"
        if safety_path.exists():
            safety_frames.append(pd.read_parquet(safety_path))
        trajectory_path = directory / "representative_trajectories.parquet"
        if trajectory_path.exists():
            trajectory_frames.append(pd.read_parquet(trajectory_path))
    episodes = pd.concat(frames, ignore_index=True)
    summary = aggregate_episodes(episodes)
    per_seed = aggregate_episodes(episodes, by_training_seed=True)
    episodes.to_parquet(output / "episodes.parquet", index=False)
    summary.to_csv(output / "summary.csv", index=False)
    per_seed.to_csv(output / "per_seed_summary.csv", index=False)
    baseline = "pid" if "pid" in set(episodes["controller"]) else str(
        episodes["controller"].iloc[0]
    )
    _paired_table(episodes, baseline).to_csv(
        output / "paired_differences.csv", index=False
    )
    if safety_frames:
        pd.concat(safety_frames, ignore_index=True).to_parquet(
            output / "safety_interventions.parquet", index=False
        )
    if trajectory_frames:
        pd.concat(trajectory_frames, ignore_index=True).to_parquet(
            output / "representative_trajectories.parquet", index=False
        )
    (
        episodes.groupby(["controller", "training_seed", "scenario", "failure_category"])
        .size()
        .rename("count")
        .reset_index()
        .to_csv(output / "failure_counts.csv", index=False)
    )
    with (output / "environment.json").open("w", encoding="utf-8") as stream:
        json.dump(environment_record(), stream, indent=2)
    return episodes, summary
