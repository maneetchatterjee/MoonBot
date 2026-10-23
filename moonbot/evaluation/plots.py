from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


def _save(figure: plt.Figure, path: Path, generated: list[Path]) -> None:
    figure.savefig(path, dpi=220)
    plt.close(figure)
    generated.append(path)


def _metric_bar(
    episodes: pd.DataFrame,
    metric: str,
    label: str,
    path: Path,
    generated: list[Path],
) -> None:
    figure, axis = plt.subplots(figsize=(8.0, 4.2), constrained_layout=True)
    sns.barplot(
        data=episodes,
        x="scenario",
        y=metric,
        hue="controller",
        ax=axis,
        errorbar=("ci", 95),
    )
    axis.set_ylabel(label)
    axis.set_xlabel("")
    axis.tick_params(axis="x", rotation=30)
    _save(figure, path, generated)


def _architecture_figure(path: Path, generated: list[Path]) -> None:
    figure, axis = plt.subplots(figsize=(9.0, 3.4), constrained_layout=True)
    axis.set_xlim(0, 10)
    axis.set_ylim(0, 4)
    axis.axis("off")
    boxes = (
        (0.3, 2.4, "Measured state\n+ target"),
        (2.2, 2.4, "PID / MPC base\n+ OSC arm"),
        (4.2, 2.4, "Nominal action"),
        (4.2, 0.7, "Residual PPO\ncorrection"),
        (6.2, 1.55, "Bounded sum +\nrate smoothing"),
        (8.1, 1.55, "OSQP safety\nprojection"),
    )
    for x, y, text in boxes:
        axis.text(
            x,
            y,
            text,
            ha="center",
            va="center",
            bbox={"boxstyle": "round,pad=0.45", "fc": "#e8f1fa", "ec": "#315a7d"},
        )
    arrows = (
        ((1.15, 2.4), (1.65, 2.4)),
        ((3.15, 2.4), (3.65, 2.4)),
        ((5.05, 2.4), (5.85, 1.9)),
        ((5.05, 0.9), (5.85, 1.45)),
        ((7.05, 1.55), (7.65, 1.55)),
        ((8.95, 1.55), (9.7, 1.55)),
    )
    for start, end in arrows:
        axis.annotate("", xy=end, xytext=start, arrowprops={"arrowstyle": "->", "lw": 1.5})
    axis.text(9.72, 1.55, "Robot", va="center", ha="left")
    axis.set_title("MoonBot-X whole-body control architecture")
    _save(figure, path, generated)


def _terrain_figure(path: Path, generated: list[Path]) -> None:
    x = np.linspace(0.0, 4.0, 200)
    profiles = {
        "flat": np.zeros_like(x),
        "ramp": np.clip(0.16 * (x - 0.8), 0.0, 0.35),
        "sine": 0.10 * np.sin(2.5 * x),
        "obstacles": np.where(((x > 1.0) & (x < 1.4)) | ((x > 2.5) & (x < 2.9)), 0.18, 0.0),
        "asymmetric": 0.07 * np.sin(3.0 * x) + 0.035 * np.sin(8.0 * x),
    }
    figure, axes = plt.subplots(1, len(profiles), figsize=(11.0, 2.6), sharey=True)
    for axis, (name, height) in zip(axes, profiles.items(), strict=True):
        axis.fill_between(x, -0.18, height, alpha=0.45)
        axis.plot(x, height, linewidth=1.5)
        axis.set_title(name.replace("_", " ").title())
        axis.set_xlabel("distance (m)")
        axis.set_xlim(0.0, 4.0)
    axes[0].set_ylabel("height (m)")
    figure.suptitle("Parameterized evaluation terrain families")
    _save(figure, path, generated)


def create_benchmark_plots(episodes: pd.DataFrame, output_directory: str | Path) -> list[Path]:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    sns.set_theme(style="whitegrid", context="paper")
    generated: list[Path] = []

    _metric_bar(episodes, "success", "Task success rate", output / "success.png", generated)

    figure, axes = plt.subplots(2, 2, figsize=(11.0, 7.0), constrained_layout=True)
    for axis, metric, label in (
        (axes[0, 0], "position_rmse_m", "Base position RMSE (m)"),
        (axes[0, 1], "heading_rmse_rad", "Base heading RMSE (rad)"),
        (axes[1, 0], "end_effector_rmse_m", "End-effector position RMSE (m)"),
        (
            axes[1, 1],
            "end_effector_orientation_rmse_rad",
            "End-effector orientation RMSE (rad)",
        ),
    ):
        sns.barplot(data=episodes, x="scenario", y=metric, hue="controller", ax=axis)
        axis.set_ylabel(label)
        axis.set_xlabel("")
        axis.tick_params(axis="x", rotation=30)
        if axis is not axes[0, 0] and axis.legend_ is not None:
            axis.legend_.remove()
    _save(figure, output / "tracking_errors.png", generated)

    figure, axes = plt.subplots(1, 2, figsize=(10.0, 4.0), constrained_layout=True)
    for axis, metric, label in (
        (axes[0], "mean_slip_ratio", "Mean wheel-slip ratio"),
        (axes[1], "energy", "Integrated absolute effort"),
    ):
        sns.barplot(data=episodes, x="scenario", y=metric, hue="controller", ax=axis)
        axis.set_ylabel(label)
        axis.set_xlabel("")
        axis.tick_params(axis="x", rotation=30)
    _save(figure, output / "slip_energy.png", generated)

    figure, axes = plt.subplots(1, 2, figsize=(10.0, 4.0), constrained_layout=True)
    for axis, metric, label in (
        (axes[0], "success", "Success rate"),
        (axes[1], "position_rmse_m", "Base position RMSE (m)"),
    ):
        sns.lineplot(
            data=episodes,
            x="terrain_difficulty",
            y=metric,
            hue="controller",
            marker="o",
            errorbar=("ci", 95),
            ax=axis,
        )
        axis.set_ylabel(label)
    _save(figure, output / "robustness_curves.png", generated)

    _metric_bar(
        episodes,
        "safety_intervention_rate",
        "Safety-filter intervention rate",
        output / "safety_interventions.png",
        generated,
    )
    _metric_bar(
        episodes,
        "controller_latency_mean_s",
        "Controller latency (s)",
        output / "controller_latency.png",
        generated,
    )
    _metric_bar(
        episodes,
        "minimum_stability_margin_m",
        "Minimum static-stability margin (m)",
        output / "stability_margin.png",
        generated,
    )

    failures = episodes[episodes["failure_category"] != "none"]
    figure, axis = plt.subplots(figsize=(8.0, 4.2), constrained_layout=True)
    if failures.empty:
        axis.text(0.5, 0.5, "No failures recorded", ha="center", va="center")
        axis.axis("off")
    else:
        counts = (
            failures.groupby(["controller", "failure_category"])
            .size()
            .rename("count")
            .reset_index()
        )
        sns.barplot(data=counts, x="controller", y="count", hue="failure_category", ax=axis)
        axis.tick_params(axis="x", rotation=25)
    axis.set_title("Episode failure categories")
    _save(figure, output / "failure_categories.png", generated)

    trajectory_path = output.parent / "representative_trajectories.parquet"
    if trajectory_path.exists():
        trajectories = pd.read_parquet(trajectory_path)
        figure, axis = plt.subplots(figsize=(7.0, 5.2), constrained_layout=True)
        sns.lineplot(
            data=trajectories,
            x="x_m",
            y="y_m",
            hue="controller",
            style="scenario",
            units="seed",
            estimator=None,
            ax=axis,
        )
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_title("Representative paired trajectories")
        _save(figure, output / "representative_trajectories.png", generated)

    _architecture_figure(output / "controller_architecture.png", generated)
    _terrain_figure(output / "task_environments.png", generated)
    return generated


def create_learning_curve(
    progress_files: list[str | Path], output_path: str | Path
) -> Path:
    rows = []
    for source in progress_files:
        path = Path(source)
        frame = pd.read_csv(path)
        if "time/total_timesteps" not in frame:
            continue
        frame = frame.assign(run=f"{path.parents[2].name}/{path.parents[1].name}")
        rows.append(frame)
    if not rows:
        raise ValueError("No valid Stable-Baselines3 progress CSV files were provided")
    data = pd.concat(rows, ignore_index=True)
    figure, axes = plt.subplots(1, 2, figsize=(10.0, 4.0), constrained_layout=True)
    for axis, metric, label in (
        (axes[0], "rollout/ep_rew_mean", "Mean episode return"),
        (axes[1], "rollout/success_rate", "Training success rate"),
    ):
        if metric in data:
            sns.lineplot(data=data, x="time/total_timesteps", y=metric, hue="run", ax=axis)
        axis.set_ylabel(label)
        axis.set_xlabel("Environment steps")
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=220)
    plt.close(figure)
    return destination
