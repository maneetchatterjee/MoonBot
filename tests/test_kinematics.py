import mujoco
import numpy as np
import pytest

from moonbot.controllers.arm_pd import PANDA_HOME
from moonbot.model.kinematics import rotation_distance, solve_site_ik


def test_damped_ik_reduces_end_effector_position_error(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for index, target in enumerate(PANDA_HOME, start=1):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{index}")
        data.qpos[model.jnt_qposadr[joint_id]] = target
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    target_position = data.site_xpos[site_id] + np.array([0.04, -0.03, 0.03])
    initial_error = np.linalg.norm(target_position - data.site_xpos[site_id])
    _, final_error, _ = solve_site_ik(
        model,
        data,
        "end_effector",
        tuple(f"joint{index}" for index in range(1, 8)),
        target_position,
    )
    assert final_error < initial_error * 0.2


def test_rotation_distance_is_geodesic_angle():
    target = np.eye(3)
    quarter_turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert rotation_distance(target, target) == pytest.approx(0.0)
    assert rotation_distance(target, quarter_turn) == pytest.approx(np.pi / 2.0)
