from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PIDAxis:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    integral: float = 0.0
    previous_error: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, error: float, dt: float, saturated: bool = False) -> float:
        if not saturated:
            self.integral = float(
                np.clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
            )
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class SkidSteerPID:
    def __init__(
        self,
        linear: PIDAxis,
        angular: PIDAxis,
        max_torque: float = 18.0,
        max_torque_rate: float = 120.0,
    ) -> None:
        self.linear = linear
        self.angular = angular
        self.max_torque = max_torque
        self.max_torque_rate = max_torque_rate
        self.previous = np.zeros(6, dtype=np.float64)

    def reset(self) -> None:
        self.linear.reset()
        self.angular.reset()
        self.previous.fill(0)

    def command(
        self,
        target_linear: float,
        target_angular: float,
        measured_linear: float,
        measured_angular: float,
        dt: float,
    ) -> np.ndarray:
        linear_effort = self.linear.update(target_linear - measured_linear, dt)
        angular_effort = self.angular.update(target_angular - measured_angular, dt)
        left = linear_effort - angular_effort
        right = linear_effort + angular_effort
        raw = np.array([left, left, left, right, right, right], dtype=np.float64)
        clipped = np.clip(raw, -self.max_torque, self.max_torque)
        maximum_change = self.max_torque_rate * dt
        output = np.clip(clipped, self.previous - maximum_change, self.previous + maximum_change)
        self.previous = output
        return output
