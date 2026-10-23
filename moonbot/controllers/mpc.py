from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import casadi as ca
import numpy as np

from moonbot.controllers.pid_base import SkidSteerPID


@dataclass(frozen=True)
class MPCResult:
    wheel_torques: np.ndarray
    solve_time_s: float
    objective: float
    feasible: bool
    fallback_used: bool
    status: str


class ReducedOrderMPC:
    def __init__(
        self,
        fallback: SkidSteerPID,
        horizon: int = 8,
        timestep: float = 0.05,
        maximum_acceleration: tuple[float, float] = (1.2, 1.8),
        maximum_velocity: tuple[float, float] = (1.5, 1.5),
        maximum_wheel_torque: float = 30.0,
        time_budget_s: float = 0.05,
    ) -> None:
        self.fallback = fallback
        self.horizon = horizon
        self.timestep = timestep
        self.maximum_acceleration = np.asarray(maximum_acceleration)
        self.maximum_velocity = np.asarray(maximum_velocity)
        self.maximum_wheel_torque = maximum_wheel_torque
        self.time_budget_s = time_budget_s
        self.previous_control = np.zeros(2)
        self.force_failure = False
        self._solver, self._lower, self._upper = self._build_solver()

    def _build_solver(self) -> tuple[ca.Function, np.ndarray, np.ndarray]:
        controls = ca.MX.sym("u", 2, self.horizon)
        states = ca.MX.sym("x", 2, self.horizon + 1)
        parameters = ca.MX.sym("p", 8)
        objective = 0
        constraints = [states[:, 0] - parameters[:2]]
        previous = parameters[4:6]
        stability_scale = parameters[6]
        slip = parameters[7]
        for step in range(self.horizon):
            next_state = states[:, step] + self.timestep * controls[:, step]
            constraints.append(states[:, step + 1] - next_state)
            tracking = states[:, step + 1] - parameters[2:4]
            change = controls[:, step] - previous
            objective += (
                16.0 * tracking[0] ** 2
                + 8.0 * tracking[1] ** 2
                + (0.25 + 0.8 * slip) * ca.sumsqr(controls[:, step])
                + 0.4 * ca.sumsqr(change)
                + 2.0 * (1.0 - stability_scale) * ca.sumsqr(controls[:, step])
            )
            previous = controls[:, step]
        decision = ca.vertcat(ca.reshape(states, -1, 1), ca.reshape(controls, -1, 1))
        problem = {
            "x": decision,
            "p": parameters,
            "f": objective,
            "g": ca.vertcat(*constraints),
        }
        solver = ca.nlpsol(
            "moonbot_mpc",
            "ipopt",
            problem,
            {
                "ipopt.print_level": 0,
                "ipopt.max_iter": 80,
                "print_time": False,
                "ipopt.sb": "yes",
            },
        )
        state_count = 2 * (self.horizon + 1)
        lower = np.concatenate(
            (
                np.tile(-self.maximum_velocity, self.horizon + 1),
                np.tile(-self.maximum_acceleration, self.horizon),
            )
        )
        upper = np.concatenate(
            (
                np.tile(self.maximum_velocity, self.horizon + 1),
                np.tile(self.maximum_acceleration, self.horizon),
            )
        )
        assert lower.size == state_count + 2 * self.horizon
        return solver, lower, upper

    def reset(self) -> None:
        self.previous_control.fill(0.0)
        self.fallback.reset()

    def command(
        self,
        target_linear: float,
        target_angular: float,
        measured_linear: float,
        measured_angular: float,
        stability_scale: float = 1.0,
        slip: float = 0.0,
    ) -> MPCResult:
        started = perf_counter()
        parameters = np.array(
            [
                measured_linear,
                measured_angular,
                target_linear,
                target_angular,
                *self.previous_control,
                stability_scale,
                slip,
            ]
        )
        status = "forced_failure" if self.force_failure else ""
        objective = np.nan
        feasible = False
        try:
            if self.force_failure:
                raise RuntimeError(status)
            initial_states = np.tile(parameters[:2], self.horizon + 1)
            initial_controls = np.tile(self.previous_control, self.horizon)
            result = self._solver(
                x0=np.concatenate((initial_states, initial_controls)),
                p=parameters,
                lbx=self._lower,
                ubx=self._upper,
                lbg=np.zeros(2 * (self.horizon + 1)),
                ubg=np.zeros(2 * (self.horizon + 1)),
            )
            stats = self._solver.stats()
            status = str(stats.get("return_status", "unknown"))
            feasible = bool(stats.get("success", False))
            decision = np.asarray(result["x"]).ravel()
            offset = 2 * (self.horizon + 1)
            control = decision[offset : offset + 2]
            objective = float(result["f"])
            solve_time = perf_counter() - started
            feasible = feasible and solve_time <= self.time_budget_s
            if not feasible:
                raise RuntimeError("MPC infeasible or exceeded time budget")
            self.previous_control = control
            left = 8.0 * control[0] - 5.0 * control[1]
            right = 8.0 * control[0] + 5.0 * control[1]
            torques = np.clip(
                np.array([left, left, left, right, right, right]),
                -self.maximum_wheel_torque,
                self.maximum_wheel_torque,
            )
            return MPCResult(torques, solve_time, objective, True, False, status)
        except (RuntimeError, ValueError) as error:
            solve_time = perf_counter() - started
            torques = self.fallback.command(
                target_linear,
                target_angular,
                measured_linear,
                measured_angular,
                self.timestep,
            )
            return MPCResult(
                torques,
                solve_time,
                objective,
                False,
                True,
                status or type(error).__name__,
            )
