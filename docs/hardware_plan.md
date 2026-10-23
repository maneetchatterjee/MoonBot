# Hardware and sim-to-real plan

MoonBot-X is currently simulation-only. Hardware work should proceed only after rover parameters and the Panda mount are engineered.

## Required engineering

1. Replace assumed rover mass/inertia with CAD-derived values.
2. Select wheel, steering, gearbox, encoder, power, and braking hardware.
3. Analyze the arm adapter and rover frame for static, fatigue, shock, and tip-over loads.
4. Identify wheel friction, rolling resistance, drivetrain backlash, torque constants, and latency.
5. Add a ROS 2 Jazzy hardware interface, `ros2_control`, TF tree, IMU/encoder drivers, current estimation, watchdog, and hard emergency stop.
6. Validate timestamps, saturation, rate limits, and command loss before closed-loop testing.

## Staged validation

1. Wheels lifted; verify directions and emergency stop.
2. Low-speed flat driving and braking.
3. Arm-only motion at reduced speed.
4. Base/arm coordination with an empty gripper.
5. Small slopes and low obstacles.
6. Instrumented payload transport.
7. Contact-only grasp trials with force/current limits.
8. Full mobile manipulation.

Every stage should log commands, feedback, current/torque estimates, IMU, watchdog state, and emergency events. No simulation result in this repository is presented as hardware validation.
