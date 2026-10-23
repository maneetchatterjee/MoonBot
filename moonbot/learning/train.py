from __future__ import annotations

import json
import shutil
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import SubprocVecEnv

from moonbot.environments.base import MoonBotEnv
from moonbot.evaluation.benchmark import environment_record
from moonbot.learning.ablations import apply_ablation
from moonbot.learning.curriculum import Curriculum


def load_training_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Training configuration must be a mapping")
    return config


class CurriculumLoggingCallback(BaseCallback):
    def __init__(
        self,
        output_path: Path,
        enabled: bool,
        steps_per_stage: int,
        verbose: int = 0,
    ) -> None:
        super().__init__(verbose)
        self.output_path = output_path
        self.enabled = enabled
        self.curriculum = Curriculum(steps_per_stage)
        self.current_stage_index = -1
        self.history: list[dict[str, Any]] = []

    def _set_stage(self) -> None:
        stage_index, stage = self.curriculum.stage(self.num_timesteps)
        if stage_index == self.current_stage_index:
            return
        self.current_stage_index = stage_index
        if self.enabled:
            self.training_env.env_method("set_curriculum_stage", stage_index)
        self.history.append(
            {
                "timesteps": self.num_timesteps,
                "stage_index": stage_index,
                "stage": stage.name if self.enabled else "disabled",
                "task": stage.task if self.enabled else "configured_task",
                "terrain": stage.terrain if self.enabled else "configured_terrain",
                "updated_utc": datetime.now(UTC).isoformat(),
            }
        )
        with self.output_path.open("w", encoding="utf-8") as stream:
            json.dump(
                {
                    "enabled": self.enabled,
                    "steps_per_stage": self.curriculum.steps_per_stage,
                    "current_stage_index": self.current_stage_index,
                    "history": self.history,
                },
                stream,
                indent=2,
            )

    def _on_training_start(self) -> None:
        self._set_stage()

    def _on_step(self) -> bool:
        self._set_stage()
        return True


def train_ppo(
    config_path: str | Path,
    output_directory: str | Path,
    *,
    total_timesteps: int | None = None,
    seed: int | None = None,
    ablation: str | None = None,
) -> Path:
    source = Path(config_path).resolve()
    config = load_training_config(source)
    output = Path(output_directory)
    checkpoints = output / "checkpoints"
    logs = output / "logs"
    checkpoints.mkdir(parents=True, exist_ok=True)
    logs.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output / "config.yaml")
    with (output / "environment.json").open("w", encoding="utf-8") as stream:
        json.dump(environment_record(), stream, indent=2)

    run_seed = int(config["seed"] if seed is None else seed)
    algorithm = str(config["algorithm"])
    control_mode = "direct" if algorithm == "ppo" else "residual"
    environment_kwargs = apply_ablation(
        {
            "task": str(config["task"]),
            "control_mode": control_mode,
            "terrain": str(config.get("terrain", "flat")),
            "terrain_difficulty": float(config.get("terrain_difficulty", 0.0)),
            "domain_randomization": bool(config.get("domain_randomization", False)),
            "safety_filter": bool(config.get("safety_filter", False)),
            "curriculum_enabled": bool(config.get("curriculum", False)),
            "episode_seconds": float(config.get("episode_seconds", 12.0)),
        },
        ablation,
    )
    number_of_environments = int(config.get("parallel_environments", 1))
    vector_environment = make_vec_env(
        MoonBotEnv,
        n_envs=number_of_environments,
        seed=run_seed,
        env_kwargs=environment_kwargs,
        vec_env_cls=SubprocVecEnv if number_of_environments > 1 else None,
        vec_env_kwargs=({"start_method": "forkserver"} if number_of_environments > 1 else None),
    )
    model = PPO(
        str(config.get("policy", "MlpPolicy")),
        vector_environment,
        learning_rate=float(config["learning_rate"]),
        n_steps=int(config["n_steps"]),
        batch_size=int(config["batch_size"]),
        n_epochs=int(config["n_epochs"]),
        gamma=float(config["gamma"]),
        gae_lambda=float(config["gae_lambda"]),
        clip_range=float(config["clip_range"]),
        ent_coef=float(config["ent_coef"]),
        seed=run_seed,
        device=str(config.get("device", "auto")),
        verbose=1,
        tensorboard_log=str(logs / "tensorboard"),
    )
    model.set_logger(configure(str(logs), ["stdout", "csv", "tensorboard"]))
    checkpoint_callback = CheckpointCallback(
        save_freq=max(1, 100_000 // number_of_environments),
        save_path=str(checkpoints),
        name_prefix=f"{algorithm}_seed_{run_seed}",
    )
    curriculum_callback = CurriculumLoggingCallback(
        output / "curriculum.json",
        enabled=bool(config.get("curriculum", False)),
        steps_per_stage=int(config.get("curriculum_steps_per_stage", 100_000)),
    )
    steps = int(config["total_timesteps"] if total_timesteps is None else total_timesteps)
    started = datetime.now(UTC)
    model.learn(
        total_timesteps=steps,
        callback=[checkpoint_callback, curriculum_callback],
        progress_bar=False,
    )
    checkpoint = checkpoints / "final.zip"
    model.save(checkpoint)
    with (output / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "algorithm": algorithm,
                "task": config["task"],
                "seed": run_seed,
                "total_timesteps": steps,
                "checkpoint": str(checkpoint.resolve()),
                "started_utc": started.isoformat(),
                "ended_utc": datetime.now(UTC).isoformat(),
                "configuration": config,
                "effective_environment": environment_kwargs,
                "ablation": ablation or "none",
            },
            stream,
            indent=2,
        )
    vector_environment.close()
    return checkpoint
