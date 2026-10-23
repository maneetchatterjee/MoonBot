from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StabilityDecision:
    scale: float
    triggers: tuple[str, ...]


class StabilityCoordinator:
    def __init__(
        self,
        attitude_limit_rad: float = np.deg2rad(18.0),
        minimum_margin_m: float = 0.08,
        maximum_slip: float = 0.55,
        minimum_contacts: int = 4,
    ) -> None:
        self.attitude_limit_rad = attitude_limit_rad
        self.minimum_margin_m = minimum_margin_m
        self.maximum_slip = maximum_slip
        self.minimum_contacts = minimum_contacts

    def evaluate(
        self,
        roll: float,
        pitch: float,
        stability_margin: float,
        slip: float,
        contacts: int,
    ) -> StabilityDecision:
        scales: list[float] = [1.0]
        triggers: list[str] = []
        attitude = max(abs(roll), abs(pitch))
        if attitude > 0.6 * self.attitude_limit_rad:
            scales.append(float(np.clip(1.0 - attitude / self.attitude_limit_rad, 0.15, 1.0)))
            triggers.append("attitude")
        if stability_margin < self.minimum_margin_m:
            scales.append(float(np.clip(stability_margin / self.minimum_margin_m, 0.1, 1.0)))
            triggers.append("stability_margin")
        if slip > self.maximum_slip:
            scales.append(float(np.clip(self.maximum_slip / max(slip, 1e-6), 0.2, 1.0)))
            triggers.append("wheel_slip")
        if contacts < self.minimum_contacts:
            scales.append(0.35)
            triggers.append("wheel_contact")
        return StabilityDecision(min(scales), tuple(triggers))
