import pybullet as p
import random

def apply_domain_randomization(robot_id):
    mass_scale = random.uniform(0.8, 1.2)
    friction = random.uniform(0.6, 1.2)

    for i in range(p.getNumJoints(robot_id)):
        info = p.getDynamicsInfo(robot_id, i)
        p.changeDynamics(
            robot_id,
            i,
            mass=info[0] * mass_scale,
            lateralFriction=friction
        )
