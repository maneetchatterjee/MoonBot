from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Scenario:
    identifier: str
    task: str
    terrain: str
    terrain_difficulty: float
    domain_randomization: bool
    sensor_failure: str | None
    ood_randomization: bool
    observation_delay_steps: int | None
    external_push_n: float | None


@dataclass(frozen=True)
class ScenarioManifest:
    name: str
    episodes_per_scenario: int
    base_seed: int
    scenarios: tuple[Scenario, ...]
    source: Path


def load_manifest(path: str | Path) -> ScenarioManifest:
    source = Path(path).resolve()
    with source.open(encoding="utf-8") as stream:
        data: dict[str, Any] = yaml.safe_load(stream)
    scenarios = tuple(
        Scenario(
            identifier=str(item["id"]),
            task=str(item["task"]),
            terrain=str(item["terrain"]),
            terrain_difficulty=float(item.get("terrain_difficulty", 0.0)),
            domain_randomization=bool(item.get("domain_randomization", False)),
            sensor_failure=item.get("sensor_failure"),
            ood_randomization=bool(item.get("ood_randomization", False)),
            observation_delay_steps=item.get("observation_delay_steps"),
            external_push_n=(
                None if item.get("external_push_n") is None else float(item["external_push_n"])
            ),
        )
        for item in data["scenarios"]
    )
    return ScenarioManifest(
        name=str(data["name"]),
        episodes_per_scenario=int(data["episodes_per_scenario"]),
        base_seed=int(data["base_seed"]),
        scenarios=scenarios,
        source=source,
    )
