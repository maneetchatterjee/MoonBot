from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
import osqp
from scipy import sparse


@dataclass(frozen=True)
class SafetyFilterResult:
    action: np.ndarray
    intervened: bool
    triggers: tuple[str, ...]
    solve_time_s: float
    status: str


class ActionSafetyFilter:
    """QP projection for actuator/rate bounds plus documented stability heuristics."""

    def __init__(
        self,
        lower_bounds: np.ndarray,
        upper_bounds: np.ndarray,
        rate_limits: np.ndarray,
        attitude_limit_rad: float = np.deg2rad(22.0),
        minimum_stability_margin_m: float = 0.04,
    ) -> None:
        self.lower = np.asarray(lower_bounds, dtype=np.float64)
        self.upper = np.asarray(upper_bounds, dtype=np.float64)
        self.rate_limits = np.asarray(rate_limits, dtype=np.float64)
        if not (self.lower.shape == self.upper.shape == self.rate_limits.shape):
            raise ValueError("Safety filter limits must have identical shapes")
        self.attitude_limit_rad = attitude_limit_rad
        self.minimum_stability_margin_m = minimum_stability_margin_m
        self.previous = np.zeros_like(self.lower)

    def reset(self) -> None:
        self.previous.fill(0.0)

    def project(
        self,
        requested: np.ndarray,
        dt: float,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        stability_margin: float = 1.0,
        arm_positions: np.ndarray | None = None,
        arm_joint_ranges: np.ndarray | None = None,
    ) -> SafetyFilterResult:
        started = perf_counter()
        requested = np.asarray(requested, dtype=np.float64)
        if requested.shape != self.lower.shape:
            raise ValueError("Requested action has the wrong shape")
        triggers: list[str] = []
        lower = np.maximum(self.lower, self.previous - self.rate_limits * dt)
        upper = np.minimum(self.upper, self.previous + self.rate_limits * dt)
        if np.any(requested < self.lower) or np.any(requested > self.upper):
            triggers.append("actuator_limit")
        if np.any(requested < lower) or np.any(requested > upper):
            triggers.append("action_rate")

        attitude = max(abs(roll), abs(pitch))
        if attitude > 0.7 * self.attitude_limit_rad:
            scale = float(np.clip(1.0 - attitude / self.attitude_limit_rad, 0.1, 1.0))
            lower[:6] *= scale
            upper[:6] *= scale
            triggers.append("base_attitude_heuristic")
        if stability_margin < self.minimum_stability_margin_m:
            lower[:6] = np.maximum(lower[:6], -0.2 * np.abs(self.lower[:6]))
            upper[:6] = np.minimum(upper[:6], 0.2 * np.abs(self.upper[:6]))
            triggers.append("stability_margin_heuristic")
        if arm_positions is not None and arm_joint_ranges is not None:
            normalized_margin = np.minimum(
                arm_positions - arm_joint_ranges[:, 0],
                arm_joint_ranges[:, 1] - arm_positions,
            )
            near = normalized_margin < 0.08
            if np.any(near):
                arm_slice = slice(6, 13)
                lower[arm_slice][near] = np.maximum(lower[arm_slice][near], -2.0)
                upper[arm_slice][near] = np.minimum(upper[arm_slice][near], 2.0)
                triggers.append("joint_position_heuristic")

        inconsistent = lower > upper
        if np.any(inconsistent):
            safe_hold = np.clip(self.previous, self.lower, self.upper)
            lower[inconsistent] = safe_hold[inconsistent]
            upper[inconsistent] = safe_hold[inconsistent]
            triggers.append("constraint_intersection_hold")

        dimension = requested.size
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.eye(dimension, format="csc") * 2.0,
            q=-2.0 * requested,
            A=sparse.eye(dimension, format="csc"),
            l=lower,
            u=upper,
            verbose=False,
            polishing=False,
            warm_starting=True,
        )
        solution = solver.solve(raise_error=False)
        if solution.x is None or solution.info.status_val not in (1, 2):
            action = np.clip(requested, lower, upper)
            status = f"fallback_clip:{solution.info.status}"
            triggers.append("qp_failure")
        else:
            action = np.clip(np.asarray(solution.x, dtype=np.float64), lower, upper)
            status = solution.info.status
        intervened = not np.allclose(action, requested, atol=1e-7)
        self.previous = action.copy()
        return SafetyFilterResult(
            action=action,
            intervened=intervened,
            triggers=tuple(dict.fromkeys(triggers)),
            solve_time_s=perf_counter() - started,
            status=status,
        )
