from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from envs.rover_locomotion_env import RoverLocomotionEnv

URDF_PATH = "urdf/rover.urdf"

def make_env():
    return RoverLocomotionEnv(URDF_PATH, gui=False)

env = DummyVecEnv([make_env])

model = PPO(
    "MlpPolicy",
    env,
    n_steps=2048,
    batch_size=128,
    learning_rate=3e-4,
    gamma=0.99,
    verbose=1
)

model.learn(total_timesteps=300_000)
model.save("results/checkpoints/ppo_rover_locomotion")
