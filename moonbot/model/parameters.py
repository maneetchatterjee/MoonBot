from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "assets" / "mjcf" / "moonbot_x.xml"
DEFAULT_ROBOT_CONFIG = PROJECT_ROOT / "configs" / "robot" / "moonbot_x.yaml"


@dataclass(frozen=True)
class RobotParameters:
    data: dict[str, Any]
    source: Path

    @property
    def timestep(self) -> float:
        return float(self.data["simulation"]["physics_timestep"])

    @property
    def wheel_radius(self) -> float:
        return float(self.data["wheels"]["radius"])

    @property
    def provisional(self) -> bool:
        return self.data["parameter_status"] != "hardware_validated"


def load_parameters(path: str | Path = DEFAULT_ROBOT_CONFIG) -> RobotParameters:
    source = Path(path).resolve()
    with source.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping in {source}")
    return RobotParameters(data=data, source=source)
