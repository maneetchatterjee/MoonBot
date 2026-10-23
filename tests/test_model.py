import mujoco
import numpy as np
import pytest

from moonbot.model.validation import validate_model
from moonbot.simulation.mujoco_backend import MuJoCoBackend


def test_model_compiles_with_expected_dimensions(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    assert model.nq == 37
    assert model.nv == 35
    assert model.nu == 18
    assert model.neq == 3
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp_attachment") == -1
    )


def test_mechanical_validation_passes(model_path):
    report = validate_model(model_path)
    assert report.passed, report.to_dict()


def test_reset_is_deterministic(model_path):
    backend = MuJoCoBackend(model_path)
    first = backend.reset()
    first_qpos = backend.data.qpos.copy()
    second = backend.reset()
    np.testing.assert_array_equal(first_qpos, backend.data.qpos)
    np.testing.assert_array_equal(first.position, second.position)


def test_backend_rejects_wrong_wheel_action_shape(model_path):
    backend = MuJoCoBackend(model_path)
    backend.reset()
    try:
        backend.step(np.zeros(5))
    except ValueError as exc:
        assert "six wheel commands" in str(exc)
    else:
        raise AssertionError("Invalid action shape was accepted")


def test_backend_rejects_wrong_steering_action_shape(model_path):
    backend = MuJoCoBackend(model_path)
    backend.reset()
    try:
        backend.step(np.zeros(6), steering_angles=np.zeros(3))
    except ValueError as exc:
        assert "four steering commands" in str(exc)
    else:
        raise AssertionError("Invalid steering action shape was accepted")


def test_four_wheel_steering_tracks_position_commands(model_path):
    backend = MuJoCoBackend(model_path)
    backend.reset()
    target = np.array([0.2, 0.2, -0.2, -0.2])
    for _ in range(250):
        state = backend.step(np.zeros(6), steering_angles=target)
    np.testing.assert_allclose(state.steering_positions, target, atol=0.02)


def test_arm_hold_command_respects_actuator_limits(model_path):
    backend = MuJoCoBackend(model_path)
    backend.reset()
    command = backend.gravity_compensation()
    limits = backend.model.actuator_ctrlrange[backend.arm_controller.actuator_ids]
    assert np.all(command >= limits[:, 0])
    assert np.all(command <= limits[:, 1])


def test_state_exposes_rocker_bogie_arm_and_gripper(model_path):
    backend = MuJoCoBackend(model_path)
    state = backend.reset()
    assert state.wheel_positions.shape == (6,)
    assert state.suspension_positions.shape == (4,)
    assert state.steering_positions.shape == (4,)
    assert state.arm_positions.shape == (7,)
    assert state.finger_positions.shape == (2,)
    assert not state.grasp_attached
    assert state.block_position[2] == pytest.approx(0.04)


def test_parallel_gripper_opens_and_closes(model_path):
    backend = MuJoCoBackend(model_path)
    backend.reset()
    for _ in range(250):
        backend.step(np.zeros(6), gripper_opening=0.04)
    assert np.all(backend.state().finger_positions > 0.039)
    for _ in range(250):
        backend.step(np.zeros(6), gripper_opening=0.0)
    assert np.all(backend.state().finger_positions < 0.001)
