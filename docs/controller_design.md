# Controller design

## PID and joint-space baseline

The base baseline uses separate linear-velocity and yaw-rate PID loops. Their efforts are mixed across left and right wheel groups, clipped to the modeled torque limits, and rate limited. Integral clamping prevents windup. Four steering targets are exposed independently; the current PID baseline holds them at zero and produces yaw through differential wheel torque.

The arm baseline uses seven-joint position/velocity feedback plus MuJoCo bias-force compensation. Commands are clipped to the Panda torque-motor ranges.

## Operational-space controller

The Cartesian controller computes the end-effector geometric Jacobian from MuJoCo and maps position/orientation feedback through its transpose. Damped pseudoinversion provides a nullspace projector. The nullspace term attracts the arm toward a nominal posture, damps joint motion, and adds a reciprocal joint-limit gradient. Arm torque saturation remains active.

## Stability coordinator

The coordinator reduces drive commands when roll/pitch approaches 18 degrees, the approximate center-of-mass projection approaches the support boundary, mean longitudinal slip exceeds its threshold, or fewer than four wheels report contact. This is a heuristic supervisor, not a formal stability proof.

## Reduced-order MPC

The MPC uses CasADi with IPOPT. Its state is planar linear and yaw velocity, and its controls are their accelerations over a configurable horizon. The objective penalizes tracking error, acceleration, control changes, estimated slip, and reduced stability scale. Velocity and acceleration bounds are enforced directly. The first acceleration is mapped to left/right wheel torques.

Every solve records duration, objective, feasibility, return status, and fallback use. Failure, infeasibility, forced test failure, or time-budget overrun activates the tested PID fallback. The smoke pipeline observed successful online solves; final latency distributions require the full server benchmark.

## Residual control

Residual PPO receives deployment-measurable observations and produces bounded corrections over the nominal six wheel and seven arm commands plus a gripper action. Residual magnitude, rate, and exponential smoothing are configurable. Pure PPO uses the same action dimension but directly commands normalized actuator effort.

## Safety filter

OSQP projects requested wheel/arm effort onto actuator and action-rate box constraints. These are genuine QP constraints. Roll/pitch scaling, static-margin scaling, and joint-proximity restrictions are conservative heuristics and are explicitly logged as such; they are not claimed as control-barrier-function guarantees. Every result includes the original request, projected action, solver status, triggers, solve time, and intervention flag.

## Contact-only manipulation

The deterministic demonstration fixes the rover after base positioning, approaches a ground-level block with full-pose IK, and closes the Panda gripper until bilateral collision-force contact is measured. A continuous joint-space path then lifts, transports, and lowers the object before the fingers open over the ground target. The block remains a free body throughout; the model contains no grasp weld or object-following code. The custom silicone jaw-liner geometry and friction are provisional simulation assumptions rather than hardware-validated grasp performance.

The separate mobile showcase uses the same physical grasp but unlocks the base after lift, applies closed-loop six-wheel torque control while the arm maintains its carry posture, brakes at a destination marker that is fixed from simulation reset, and parks the base only for lowering and release. Its validation records bilateral-contact ratio and end-effector-to-block drift during rover motion, rover and block displacement, wheel rotation, suspension travel, upright orientation, and final planar error.
