from __future__ import annotations

from typing import Any

ABLATIONS: dict[str, dict[str, Any]] = {
    "none": {},
    "no_domain_randomization": {"domain_randomization": False},
    "no_safety_filter": {"safety_filter": False},
    "no_stability_penalty": {"reward_ablation": "stability"},
    "no_slip_penalty": {"reward_ablation": "slip"},
    "no_stability_coordinator": {"stability_coordinator": False},
    "no_arm_base_coordination": {"arm_base_coordination": False},
}


def apply_ablation(environment_kwargs: dict[str, Any], name: str | None) -> dict[str, Any]:
    if name is None or name == "none":
        return environment_kwargs
    if name not in ABLATIONS:
        raise ValueError(f"Unknown ablation: {name}")
    return {**environment_kwargs, **ABLATIONS[name]}
