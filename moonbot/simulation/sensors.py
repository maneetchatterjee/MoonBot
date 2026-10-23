from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SensorNoise:
    imu_gyro_std: float = 0.0
    imu_accel_std: float = 0.0
    encoder_std: float = 0.0
    velocity_std: float = 0.0


class ObservationDelay:
    def __init__(self, steps: int = 0) -> None:
        if steps < 0:
            raise ValueError("Delay must be nonnegative")
        self.steps = steps
        self._buffer: deque[np.ndarray] = deque(maxlen=steps + 1)

    def reset(self, observation: np.ndarray) -> np.ndarray:
        self._buffer.clear()
        for _ in range(self.steps + 1):
            self._buffer.append(np.asarray(observation, dtype=np.float32).copy())
        return self._buffer[0].copy()

    def apply(self, observation: np.ndarray) -> np.ndarray:
        self._buffer.append(np.asarray(observation, dtype=np.float32).copy())
        return self._buffer[0].copy()


def add_measurement_noise(
    values: np.ndarray,
    standard_deviation: float,
    rng: np.random.Generator,
) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if standard_deviation <= 0.0:
        return values.copy()
    return values + rng.normal(0.0, standard_deviation, size=values.shape)
