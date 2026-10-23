from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

TERRAIN_FAMILIES = ("flat", "ramp", "sine", "obstacles", "asymmetric", "low_friction")


@dataclass(frozen=True)
class TerrainSpecification:
    family: str = "flat"
    difficulty: float = 0.0
    friction: float = 1.2


class TerrainManager:
    def __init__(self, model: mujoco.MjModel, slots: int = 16) -> None:
        self.model = model
        self.geom_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"terrain_{index:02d}")
                for index in range(slots)
            ],
            dtype=int,
        )
        if np.any(self.geom_ids < 0):
            raise ValueError("Terrain slots are missing from the MJCF model")
        self.ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        self._nominal_ground_friction = model.geom_friction[self.ground_id].copy()
        self._compiled_positions = model.geom_pos[self.geom_ids].copy()
        self._compiled_sizes = model.geom_size[self.geom_ids].copy()

    def _activate(self, geom_ids: np.ndarray) -> None:
        self.model.geom_contype[geom_ids] = 1
        self.model.geom_conaffinity[geom_ids] = 1
        self.model.geom_rgba[geom_ids, 3] = 1.0

    def configure(self, specification: TerrainSpecification) -> None:
        if specification.family not in TERRAIN_FAMILIES:
            raise ValueError(f"Unknown terrain family: {specification.family}")
        difficulty = float(np.clip(specification.difficulty, 0.0, 1.0))
        self.model.geom_pos[self.geom_ids] = self._compiled_positions
        self.model.geom_quat[self.geom_ids] = np.array([1.0, 0.0, 0.0, 0.0])
        self.model.geom_size[self.geom_ids] = self._compiled_sizes
        self.model.geom_friction[self.geom_ids] = np.array([specification.friction, 0.01, 0.002])
        self.model.geom_contype[self.geom_ids] = 0
        self.model.geom_conaffinity[self.geom_ids] = 0
        self.model.geom_rgba[self.geom_ids, 3] = 0.0
        self.model.geom_friction[self.ground_id] = self._nominal_ground_friction

        if specification.family == "flat":
            return
        if specification.family == "low_friction":
            self.model.geom_friction[self.ground_id, 0] = max(0.15, 0.55 - 0.3 * difficulty)
            return
        if specification.family == "ramp":
            angle = np.deg2rad(4.0 + 8.0 * difficulty)
            length = 1.5
            geom_id = self.geom_ids[0]
            self._activate(self.geom_ids[:1])
            self.model.geom_size[geom_id] = np.array([length, 0.8, 0.035])
            self.model.geom_pos[geom_id] = np.array([2.5, 0.0, length * np.sin(angle)])
            self.model.geom_quat[geom_id] = np.array(
                [np.cos(angle / 2.0), 0.0, -np.sin(angle / 2.0), 0.0]
            )
            return

        count = len(self.geom_ids)
        self._activate(self.geom_ids)
        x_positions = 1.0 + 0.42 * np.arange(count)
        if specification.family == "sine":
            heights = 0.025 + (0.025 + 0.055 * difficulty) * (
                1.0 + np.sin(np.linspace(0.0, 3.5 * np.pi, count))
            )
            y_positions = np.zeros(count)
        elif specification.family == "asymmetric":
            heights = np.linspace(0.03, 0.05 + 0.13 * difficulty, count)
            y_positions = np.where(np.arange(count) % 2 == 0, 0.42, -0.42)
        else:
            heights = 0.035 + (0.04 + 0.10 * difficulty) * (
                0.25 + 0.75 * ((np.arange(count) * 7) % 11) / 10.0
            )
            y_positions = np.where(np.arange(count) % 3 == 0, 0.35, -0.25)
        for geom_id, x, y, height in zip(
            self.geom_ids, x_positions, y_positions, heights, strict=True
        ):
            self.model.geom_size[geom_id] = np.array([0.16, 0.32, height / 2.0])
            self.model.geom_pos[geom_id] = np.array([x, y, height / 2.0])
