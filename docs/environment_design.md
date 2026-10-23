# Environment design

## Rates and interface

MuJoCo advances at 0.002 seconds. The Gymnasium policy interface defaults to 25 Hz through control decimation; the PID low-level design rate is 100 Hz. MPC horizon time and controller limits are configurable. Environments implement the current Gymnasium reset/step contract and pass its environment checker.

## State and observations

The backend exposes base pose/velocity, wheel and steering state, suspension angles, arm and finger state, end-effector pose/velocity, model center of mass, wheel contacts/forces, fingertip forces, IMU readings, block pose, actuator forces, grasp state, and an approximate static-stability margin.

The 76-value policy observation uses measurable quantities only. It combines base-frame gravity, IMU, wheel and arm encoders, desired motion, relative end-effector error, prior action, estimated base velocity, wheel contacts, gripper position, fingertip force, normalized manipulation stage, and eight terrain samples. It excludes world-frame block truth and exact simulator dynamics parameters. Domain-randomized training adds IMU/encoder noise and observation delay. IMU and wheel-encoder dropout modes are reserved for evaluation.

## Actions

Actions contain fourteen normalized values. The first six act on wheel commands, the next seven on the Panda joints, and the final value adjusts gripper opening. In residual mode, wheel/arm values are bounded corrections to nominal control. The safety layer subsequently projects the combined actuator vector.

## Tasks

- `velocity_tracking`: randomized linear/yaw targets on flat or randomized surfaces.
- `waypoint_navigation`: planar waypoint progress over selectable terrain.
- `arm_reaching`: Cartesian reaching with the base kinematically held.
- `mobile_tracking`: targets outside the stationary arm workspace.
- `payload_transport`: waypoint locomotion with payload randomization metrics.
- `mobile_manipulation`: staged approach, reach, bilateral contact grasp, lift, transport, and ground placement.

## Terrain and randomization

Compiled collision slots support ramps, sinusoidal box fields, discrete obstacles, asymmetric tracks, and low-friction ground. Randomization covers base/arm/payload mass, center-of-mass offset, friction, motor strength, observation and actuator delay, IMU/encoder noise, and external planar pushes. Training and harder reserved OOD ranges are separated in configuration.

## Reward and termination

Reward terms cover velocity tracking, waypoint progress, end-effector error, attitude, stability margin, slip, effort, action smoothness, collisions, and completion. Success thresholds and safety metrics are independent of reward. Episodes end on task success, non-finite state, excessive roll/pitch, chassis collapse, or time limit.

## Curriculum execution

The training callback actively schedules nine stages in every vector environment. Stage changes take effect on the next episode reset so task semantics never change midway through an episode. The sequence covers nominal/random-command velocity tracking, ramps, rough sine terrain, mobile arm tracking, payload variation, external disturbances, delayed/noisy sensing, and mobile manipulation. Discrete-obstacle and asymmetric terrain families remain excluded from training and are reserved for held-out evaluation.
