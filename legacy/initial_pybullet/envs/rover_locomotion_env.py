import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from envs.reward_functions import locomotion_reward
from rl.domain_randomization import apply_domain_randomization

class RoverLocomotionEnv(gym.Env):
    metadata = {"render_modes": ["human"]}

    def __init__(self, urdf_path, gui=False):
        super().__init__()
        self.gui = gui
        self.urdf_path = urdf_path

        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.action_space = spaces.Box(
            low=-6.0, high=6.0, shape=(4,), dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(16,), dtype=np.float32
        )

        self.reset()

    def reset(self, seed=None):
        p.resetSimulation()
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(self.urdf_path, [0, 0, 0.25])

        apply_domain_randomization(self.robot)

        self.wheels = self._get_wheel_joints()
        self.steps = 0
        return self._get_obs(), {}

    def _get_wheel_joints(self):
        names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        ]
        joints = {}
        for i in range(p.getNumJoints(self.robot)):
            jname = p.getJointInfo(self.robot, i)[1].decode()
            if jname in names:
                joints[jname] = i
        return joints

    def _get_obs(self):
        lin_vel, ang_vel = p.getBaseVelocity(self.robot)
        pos, ori = p.getBasePositionAndOrientation(self.robot)
        roll, pitch, yaw = p.getEulerFromQuaternion(ori)
        wheel_states = [p.getJointState(self.robot, j)[0] for j in self.wheels.values()]

        return np.array(
            list(lin_vel) + list(ang_vel) + [roll, pitch, yaw] + wheel_states,
            dtype=np.float32
        )

    def step(self, action):
        for a, j in zip(action, self.wheels.values()):
            p.setJointMotorControl2(
                self.robot, j,
                p.VELOCITY_CONTROL,
                targetVelocity=float(a),
                force=50
            )

        p.stepSimulation()
        self.steps += 1

        obs = self._get_obs()
        reward = locomotion_reward(obs, action)
        terminated = self.steps >= 1500

        return obs, reward, terminated, False, {}

    def close(self):
        p.disconnect(self.cid)
