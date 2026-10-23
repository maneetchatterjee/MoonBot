from dataclasses import replace

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from moonbot.environments.base import MoonBotEnv


def test_environment_is_gymnasium_compatible():
    env = MoonBotEnv(task="velocity_tracking", control_mode="residual", episode_seconds=0.2)
    check_env(env, skip_render_check=True)
    env.close()


def test_environment_reset_is_seed_reproducible():
    env = MoonBotEnv(
        task="velocity_tracking",
        control_mode="residual",
        domain_randomization=True,
        episode_seconds=0.2,
    )
    first, first_info = env.reset(seed=91)
    second, second_info = env.reset(seed=91)
    np.testing.assert_array_equal(first, second)
    assert first_info["randomization"]["base_mass_scale"] == pytest.approx(
        second_info["randomization"]["base_mass_scale"]
    )
    env.close()


def test_observation_action_reward_and_termination_contract():
    env = MoonBotEnv(task="arm_reaching", control_mode="residual", episode_seconds=0.12)
    observation, _ = env.reset(seed=4)
    assert env.observation_space.contains(observation)
    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)
    assert env.observation_space.contains(observation)
    assert np.isfinite(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert np.isfinite(info["end_effector_error_m"])
    with pytest.raises(ValueError, match="Expected action shape"):
        env.step(np.zeros(2))
    env.close()


def test_failure_termination_when_base_is_below_ground():
    env = MoonBotEnv(task="velocity_tracking", control_mode="pid", episode_seconds=0.2)
    env.reset(seed=5)
    env.backend.data.qpos[2] = 0.05
    observation, reward, terminated, _, info = env.step(np.zeros(14))
    assert np.all(np.isfinite(observation))
    assert np.isfinite(reward)
    assert terminated
    assert info["failure"]
    env.close()


def test_curriculum_stage_is_applied_on_reset():
    env = MoonBotEnv(
        task="velocity_tracking",
        control_mode="residual",
        domain_randomization=True,
        curriculum_enabled=True,
        episode_seconds=0.2,
    )
    env.set_curriculum_stage(6)
    _, info = env.reset(seed=17)
    assert env.task == "payload_transport"
    assert env.terrain_family == "ramp"
    assert env.terrain_difficulty == pytest.approx(0.65)
    assert info["randomization"]["external_push_n"] > 0.0
    env.close()


def test_mobile_manipulation_success_requires_physical_stage_seven_release(monkeypatch):
    env = MoonBotEnv(task="mobile_manipulation", control_mode="residual", episode_seconds=0.2)
    env.reset(seed=23)
    state = replace(
        env.backend.state(),
        block_position=env.place_target.copy(),
        block_orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    monkeypatch.setattr(env.backend, "block_ground_contact", lambda: True)

    for incomplete_stage in (5, 6):
        env.stage = incomplete_stage
        assert not env._success(state, waypoint_distance=0.0, ee_error=0.0)

    env.stage = 7
    assert env._success(state, waypoint_distance=0.0, ee_error=0.0)
    env.close()
