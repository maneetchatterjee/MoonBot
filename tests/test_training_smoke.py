from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from moonbot.environments.base import MoonBotEnv


def test_short_ppo_learning_smoke():
    env = MoonBotEnv(task="velocity_tracking", control_mode="residual", episode_seconds=0.4)
    model = PPO(
        "MlpPolicy",
        env,
        n_steps=16,
        batch_size=8,
        n_epochs=1,
        learning_rate=3e-4,
        device="cpu",
        seed=3,
        verbose=0,
    )
    model.learn(total_timesteps=32)
    observation, _ = env.reset(seed=3)
    action, _ = model.predict(observation, deterministic=True)
    assert env.action_space.contains(action)
    env.close()


def test_parallel_subprocess_environments_smoke():
    environments = make_vec_env(
        MoonBotEnv,
        n_envs=2,
        seed=17,
        env_kwargs={
            "task": "velocity_tracking",
            "control_mode": "residual",
            "episode_seconds": 0.2,
        },
        vec_env_cls=SubprocVecEnv,
        vec_env_kwargs={"start_method": "forkserver"},
    )
    observations = environments.reset()
    assert observations.shape == (2, 76)
    observations, rewards, _, _ = environments.step(
        [environments.action_space.sample() for _ in range(2)]
    )
    assert observations.shape == (2, 76)
    assert rewards.shape == (2,)
    environments.close()
