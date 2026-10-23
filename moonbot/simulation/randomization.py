from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml


@dataclass(frozen=True)
class RandomizationSample:
    base_mass_scale: float
    arm_mass_scale: float
    payload_mass_scale: float
    friction_scale: float
    motor_strength_scale: float
    com_offset: np.ndarray
    observation_delay_steps: int
    actuator_delay_steps: int
    external_push_n: float


def load_randomization(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError("Randomization configuration must be a mapping")
    return result


class DomainRandomizer:
    def __init__(self, model: mujoco.MjModel, config: dict[str, Any]) -> None:
        self.model = model
        self.config = config
        self.nominal_mass = model.body_mass.copy()
        self.nominal_inertia = model.body_inertia.copy()
        self.nominal_ipos = model.body_ipos.copy()
        self.nominal_friction = model.geom_friction.copy()
        self.nominal_gear = model.actuator_gear.copy()
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.arm_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                for name in (*tuple(f"link{index}" for index in range(8)), "hand")
            ]
        )
        self.payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "grasp_block")

    @staticmethod
    def _uniform(rng: np.random.Generator, bounds: list[float]) -> float:
        return float(rng.uniform(float(bounds[0]), float(bounds[1])))

    def reset_nominal(self) -> None:
        self.model.body_mass[:] = self.nominal_mass
        self.model.body_inertia[:] = self.nominal_inertia
        self.model.body_ipos[:] = self.nominal_ipos
        self.model.geom_friction[:] = self.nominal_friction
        self.model.actuator_gear[:] = self.nominal_gear

    def apply(
        self,
        rng: np.random.Generator,
        enabled: bool = True,
        ood: bool = False,
    ) -> RandomizationSample:
        self.reset_nominal()
        if not enabled:
            return RandomizationSample(1.0, 1.0, 1.0, 1.0, 1.0, np.zeros(3), 0, 0, 0.0)
        parameters = {**self.config, **self.config.get("reserved_ood", {})} if ood else self.config
        base_scale = self._uniform(rng, parameters["base_mass_scale"])
        arm_scale = self._uniform(rng, parameters["arm_mass_scale"])
        payload_scale = self._uniform(rng, parameters["payload_mass_scale"])
        friction_scale = self._uniform(rng, parameters["friction_scale"])
        motor_scale = self._uniform(rng, parameters["motor_strength_scale"])
        offset_limit = np.asarray(self.config["com_offset_m"], dtype=np.float64)
        com_offset = rng.uniform(-offset_limit, offset_limit)
        delay_bounds = parameters["observation_delay_steps"]
        delay = int(rng.integers(int(delay_bounds[0]), int(delay_bounds[1]) + 1))
        actuator_delay_bounds = parameters["actuator_delay_steps"]
        actuator_delay = int(
            rng.integers(int(actuator_delay_bounds[0]), int(actuator_delay_bounds[1]) + 1)
        )
        push = self._uniform(rng, parameters["external_push_n"])

        self.model.body_mass[self.base_id] *= base_scale
        self.model.body_inertia[self.base_id] *= base_scale
        self.model.body_ipos[self.base_id] += com_offset
        self.model.body_mass[self.arm_ids] *= arm_scale
        self.model.body_inertia[self.arm_ids] *= arm_scale
        self.model.body_mass[self.payload_id] *= payload_scale
        self.model.body_inertia[self.payload_id] *= payload_scale
        self.model.geom_friction[:, 0] *= friction_scale
        for actuator_id in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or ""
            if "motor" in name:
                self.model.actuator_gear[actuator_id, 0] *= motor_scale
        mujoco.mj_setConst(self.model, mujoco.MjData(self.model))
        return RandomizationSample(
            base_scale,
            arm_scale,
            payload_scale,
            friction_scale,
            motor_scale,
            com_offset,
            delay,
            actuator_delay,
            push,
        )
