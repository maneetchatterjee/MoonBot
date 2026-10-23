from __future__ import annotations

import numpy as np


class ResidualController:
    def __init__(
        self,
        residual_limits: np.ndarray,
        action_rate_limits: np.ndarray,
        smoothing: float = 0.25,
    ) -> None:
        self.residual_limits = np.asarray(residual_limits, dtype=np.float64)
        self.action_rate_limits = np.asarray(action_rate_limits, dtype=np.float64)
        if self.residual_limits.shape != self.action_rate_limits.shape:
            raise ValueError("Residual and rate limits must have identical shapes")
        self.smoothing = float(np.clip(smoothing, 0.0, 1.0))
        self.previous = np.zeros_like(self.residual_limits)

    def reset(self) -> None:
        self.previous.fill(0.0)

    def combine(
        self,
        nominal: np.ndarray,
        normalized_residual: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        nominal = np.asarray(nominal, dtype=np.float64)
        normalized = np.asarray(normalized_residual, dtype=np.float64)
        if nominal.shape != self.residual_limits.shape or normalized.shape != nominal.shape:
            raise ValueError("Nominal and residual actions do not match configured limits")
        target = np.clip(normalized, -1.0, 1.0) * self.residual_limits
        filtered = self.smoothing * target + (1.0 - self.smoothing) * self.previous
        maximum_change = self.action_rate_limits * dt
        residual = np.clip(
            filtered,
            self.previous - maximum_change,
            self.previous + maximum_change,
        )
        self.previous = residual
        return nominal + residual
