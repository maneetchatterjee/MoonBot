from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CurriculumStage:
    name: str
    task: str
    terrain: str
    difficulty: float
    randomization: bool
    arm_motion: bool
    disturbances: bool


STAGES = (
    CurriculumStage("flat_nominal", "velocity_tracking", "flat", 0.0, False, False, False),
    CurriculumStage("random_commands", "velocity_tracking", "flat", 0.0, False, False, False),
    CurriculumStage("gentle_slopes", "waypoint_navigation", "ramp", 0.25, False, False, False),
    CurriculumStage("rough_terrain", "waypoint_navigation", "sine", 0.45, True, False, False),
    CurriculumStage("arm_during_motion", "mobile_tracking", "sine", 0.55, True, True, False),
    CurriculumStage("payload_variation", "payload_transport", "sine", 0.60, True, True, False),
    CurriculumStage("disturbances", "payload_transport", "ramp", 0.65, True, True, True),
    CurriculumStage("noise_and_delay", "mobile_tracking", "sine", 0.70, True, True, True),
    CurriculumStage("mobile_manipulation", "mobile_manipulation", "sine", 0.75, True, True, True),
)


class Curriculum:
    def __init__(self, steps_per_stage: int = 200_000) -> None:
        self.steps_per_stage = steps_per_stage

    def stage(self, total_steps: int) -> tuple[int, CurriculumStage]:
        index = min(total_steps // self.steps_per_stage, len(STAGES) - 1)
        return int(index), STAGES[index]
