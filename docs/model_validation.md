# Model validation

The canonical model is `assets/mjcf/moonbot_x.xml`. It is generated from `assets/mjcf/sawppy_base.xml` and the pinned Menagerie Panda source by `scripts/build_mobile_panda_model.py`. `configs/robot/moonbot_x.yaml` records dimensions, provenance, and assumptions.

## Compiled architecture

MuJoCo compiles the combined model to 28 bodies, 25 joints, 37 generalized positions, 35 generalized velocities, and 18 actuators. The actuators comprise six drive motors, four steering position servos, seven Panda joint torque motors, and one gripper-tendon position servo.

The rover has four passive suspension pivots and a coupled rocker differential. The four corner wheels steer; all six wheels drive. The Panda hierarchy includes links 0-7, the hand, two finger joints, imported inertial tensors, visual/collision meshes, and tendon-coupled finger motion.

## Automated checks

Run:

```bash
python scripts/build_mobile_panda_model.py
python -m moonbot.cli validate-model --output results/model_validation.json
```

The validator performs twenty checks covering compilation, positive masses and principal inertias, joint ranges, actuator mappings and limits, gear/dissipation sanity, sensor frames and dimensions, collision geometry and mesh scale, initial contacts, flat-ground equilibrium, wheel direction, arm kinematics, gravity-compensated hold, Panda topology, rocker-bogie topology, four-wheel-steering topology, suspension articulation, gripper travel, deterministic replay, and 2 ms versus 4 ms timestep sensitivity.

The recorded local run passed all checks. After two seconds of settling, chassis height was 0.4146 m, roll was effectively zero, and pitch was -0.176 degrees. A positive six-wheel command moved the rover 0.2235 m in positive x over 0.6 seconds. A 0.25 rad left-bogie rotation separated the middle and rear wheel heights by 0.1219 m. The Panda home shoulder-to-tool distance was 0.7277 m, and maximum joint drift under gravity-compensated holding was 0.000617 rad over 0.5 seconds. Settled poses at 2 ms and 4 ms physics steps differed by 0.00002 m and 0.0002 degrees.

The separate gripper smoke test moved both fingers from 40 mm to less than 0.007 mm in 0.5 seconds. The PID smoke rollout completed 2,501 physics steps and advanced 1.917 m without non-finite state.

## Evidence boundary

These checks establish internal simulator consistency and action/state wiring. They do not validate the rover's structural strength, motor sizing, tire model, mass distribution, Panda mounting safety, or manipulation performance against physical hardware.
