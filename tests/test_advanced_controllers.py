import numpy as np

from moonbot.controllers.mpc import ReducedOrderMPC
from moonbot.controllers.pid_base import PIDAxis, SkidSteerPID
from moonbot.controllers.safety_filter import ActionSafetyFilter


def _pid() -> SkidSteerPID:
    return SkidSteerPID(
        PIDAxis(8.0, 1.0, 0.15, 3.0),
        PIDAxis(4.0, 0.5, 0.08, 2.0),
        max_torque=30.0,
        max_torque_rate=120.0,
    )


def test_safety_filter_enforces_actuator_and_rate_limits():
    lower = np.full(13, -10.0)
    upper = np.full(13, 10.0)
    safety = ActionSafetyFilter(lower, upper, np.full(13, 5.0))
    result = safety.project(np.full(13, 50.0), 0.1)
    assert result.intervened
    assert "actuator_limit" in result.triggers
    assert "action_rate" in result.triggers
    assert np.all(result.action <= 0.5 + 1e-6)


def test_mpc_produces_action_and_has_tested_pid_fallback():
    mpc = ReducedOrderMPC(_pid(), horizon=4, time_budget_s=1.0)
    result = mpc.command(0.4, 0.0, 0.0, 0.0)
    assert result.wheel_torques.shape == (6,)
    assert np.all(np.isfinite(result.wheel_torques))
    mpc.force_failure = True
    fallback = mpc.command(0.4, 0.0, 0.0, 0.0)
    assert fallback.fallback_used
    assert not fallback.feasible
    assert np.all(np.isfinite(fallback.wheel_torques))
