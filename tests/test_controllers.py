import numpy as np

from moonbot.controllers.pid_base import PIDAxis, SkidSteerPID


def make_controller() -> SkidSteerPID:
    return SkidSteerPID(
        PIDAxis(8.0, 1.0, 0.15, 3.0),
        PIDAxis(4.0, 0.5, 0.08, 2.0),
        max_torque=18.0,
        max_torque_rate=120.0,
    )


def test_pid_respects_torque_and_rate_limits():
    controller = make_controller()
    first = controller.command(10.0, 4.0, 0.0, 0.0, 0.01)
    second = controller.command(10.0, 4.0, 0.0, 0.0, 0.01)
    assert np.max(np.abs(first)) <= 1.2 + 1e-12
    assert np.max(np.abs(second - first)) <= 1.2 + 1e-12
    assert np.max(np.abs(second)) <= 18.0


def test_positive_yaw_command_is_differential():
    controller = make_controller()
    command = controller.command(0.0, 0.5, 0.0, 0.0, 0.01)
    assert np.all(command[:3] < command[3:])


def test_reset_clears_controller_state():
    controller = make_controller()
    controller.command(1.0, 1.0, 0.0, 0.0, 0.01)
    controller.reset()
    assert controller.linear.integral == 0.0
    assert controller.angular.integral == 0.0
    np.testing.assert_array_equal(controller.previous, np.zeros(6))
