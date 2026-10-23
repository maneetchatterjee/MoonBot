from __future__ import annotations

import numpy as np
import pandas as pd

from moonbot.evaluation.metrics import EPISODE_METRICS, bootstrap_mean_interval


def aggregate_episodes(
    episodes: pd.DataFrame,
    bootstrap_samples: int = 2000,
    *,
    by_training_seed: bool = False,
) -> pd.DataFrame:
    rows: list[dict[str, float | str | int]] = []
    group_columns = ["controller", "scenario"]
    if by_training_seed and "training_seed" in episodes:
        group_columns.insert(1, "training_seed")
    for keys, group in episodes.groupby(group_columns):
        if not isinstance(keys, tuple):
            keys = (keys,)
        controller = keys[0]
        scenario = keys[-1]
        row: dict[str, float | str | int] = {
            "controller": str(controller),
            "scenario": str(scenario),
            "episodes": len(group),
        }
        if by_training_seed and len(keys) == 3:
            row["training_seed"] = int(keys[1])
        for metric in EPISODE_METRICS:
            values = group[metric].to_numpy(dtype=np.float64)
            values = values[np.isfinite(values)]
            if len(values) == 0:
                row[f"{metric}_mean"] = float("nan")
                row[f"{metric}_std"] = float("nan")
                row[f"{metric}_ci95_low"] = float("nan")
                row[f"{metric}_ci95_high"] = float("nan")
                continue
            row[f"{metric}_mean"] = float(np.mean(values))
            row[f"{metric}_std"] = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
            if len(values) > 1:
                lower, upper = bootstrap_mean_interval(
                    values,
                    samples=bootstrap_samples,
                    seed=sum(ord(character) for character in f"{controller}:{scenario}:{metric}"),
                )
            else:
                lower = upper = float(values[0])
            row[f"{metric}_ci95_low"] = lower
            row[f"{metric}_ci95_high"] = upper
        rows.append(row)
    return pd.DataFrame(rows)
