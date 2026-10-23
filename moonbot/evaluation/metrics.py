from __future__ import annotations

import numpy as np

EPISODE_METRICS = (
    "success",
    "return",
    "duration_s",
    "position_rmse_m",
    "heading_rmse_rad",
    "end_effector_rmse_m",
    "end_effector_orientation_rmse_rad",
    "mean_slip_ratio",
    "energy",
    "peak_actuator_force",
    "wheel_contact_impulse_ns",
    "collision_rate",
    "payload_displacement_m",
    "peak_roll_rad",
    "peak_pitch_rad",
    "minimum_stability_margin_m",
    "constraint_violation_rate",
    "safety_intervention_rate",
    "controller_latency_mean_s",
    "mpc_solve_time_mean_s",
    "mpc_fallback_rate",
    "recovery_time_s",
    "recovery_observed",
)


def root_mean_square(values: np.ndarray) -> float:
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        raise ValueError("Cannot calculate RMSE for an empty array")
    return float(np.sqrt(np.mean(np.square(array))))


def bootstrap_mean_interval(
    values: np.ndarray,
    confidence: float = 0.95,
    samples: int = 10_000,
    seed: int = 0,
) -> tuple[float, float]:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or array.size < 2:
        raise ValueError("Bootstrap intervals require at least two episode-level values")
    rng = np.random.default_rng(seed)
    resampled = rng.choice(array, size=(samples, array.size), replace=True).mean(axis=1)
    tail = (1.0 - confidence) / 2.0
    lower, upper = np.quantile(resampled, [tail, 1.0 - tail])
    return float(lower), float(upper)


def paired_mean_difference(
    first: np.ndarray,
    second: np.ndarray,
    samples: int = 10_000,
    seed: int = 0,
) -> tuple[float, float, float]:
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    if first.shape != second.shape or first.ndim != 1 or first.size < 2:
        raise ValueError("Paired comparisons require matching one-dimensional arrays")
    difference = first - second
    lower, upper = bootstrap_mean_interval(difference, samples=samples, seed=seed)
    return float(np.mean(difference)), lower, upper
