import numpy as np

def locomotion_reward(obs, action):
    forward_vel = obs[0]
    roll, pitch = obs[6], obs[7]

    energy_penalty = 0.0002 * np.sum(np.square(action))
    stability_penalty = abs(roll) + abs(pitch)

    return (
        1.5 * forward_vel
        - energy_penalty
        - 2.0 * stability_penalty
    )
