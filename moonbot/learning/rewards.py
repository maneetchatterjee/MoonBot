from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RewardBreakdown:
    total: float
    tracking: float
    progress: float
    end_effector: float
    upright: float
    slip: float
    energy: float
    smoothness: float
    safety: float
    completion: float


def task_reward(
    *,
    velocity_error: np.ndarray,
    waypoint_progress: float,
    end_effector_error: float,
    roll: float,
    pitch: float,
    slip_ratio: float,
    squared_effort: float,
    action_change: float,
    stability_margin: float,
    collision: bool,
    success: bool,
    weights: dict[str, float],
) -> RewardBreakdown:
    tracking = -weights["velocity_tracking"] * float(np.dot(velocity_error, velocity_error))
    progress = weights["waypoint_progress"] * waypoint_progress
    end_effector = -weights["end_effector_tracking"] * end_effector_error
    upright = -weights["upright"] * (roll**2 + pitch**2)
    slip = -weights["slip"] * slip_ratio
    energy = -weights["energy"] * squared_effort
    smoothness = -weights["smoothness"] * action_change
    safety = weights["stability"] * min(stability_margin, 0.2)
    if collision:
        safety -= weights["collision"]
    completion = weights["completion"] if success else 0.0
    total = (
        tracking
        + progress
        + end_effector
        + upright
        + slip
        + energy
        + smoothness
        + safety
        + completion
    )
    return RewardBreakdown(
        float(total),
        float(tracking),
        float(progress),
        float(end_effector),
        float(upright),
        float(slip),
        float(energy),
        float(smoothness),
        float(safety),
        float(completion),
    )
