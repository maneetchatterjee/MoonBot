# CV-ready project points

## MoonBot-X — six-wheel mobile manipulation research platform

These points describe the verified simulation implementation. They intentionally do not claim
hardware validation, sim-to-real transfer, or successful learned pick-and-place.

1. **Architected a research-grade mobile-manipulation platform** by integrating a six-wheel,
   four-steer rocker-bogie rover with a 7-DOF Franka Emika Panda and tendon-coupled parallel
   gripper in MuJoCo. Modeled 23 mechanical joints, 18 actuator channels, floating-base dynamics,
   wheel/ground contact, fingertip sensing, IMU/encoder signals, and independently articulated
   rocker and bogie assemblies.

2. **Implemented a multi-layer whole-body control stack** comprising skid-steer velocity PID,
   gravity-compensated joint control, damped-Jacobian operational-space control, reduced-order
   CasADi/IPOPT MPC with measured solve status and PID fallback, residual PPO, and OSQP action
   projection for actuator and rate limits. Kept heuristic stability coordination explicitly
   separate from formal safety guarantees.

3. **Designed a deployable learning interface with a 76-dimensional observation and
   14-dimensional continuous action space.** Observations combine base-frame gravity, IMU,
   wheel/arm encoders, fingertip forces, contact state, previous action, task stage, desired
   motion, end-effector error, and terrain samples without exposing simulator-only object truth to
   the policy. Actions command six wheel efforts, seven arm residuals, and gripper opening.

4. **Engineered and physically validated a contact-only ground pick–transport–place pipeline.**
   The scripted controller picked a free 20 g block directly from the ground, retained bilateral
   finger contact throughout rover motion, drove the articulated base **0.808 m**, transported the
   block **0.795 m**, limited measured grasp drift to **1.30 mm**, and released it upright with
   **1.11 mm planar placement error**. The block remained a free MuJoCo body; no weld,
   object-following, or kinematic attachment was used.

5. **Built a configuration-driven robustness and evaluation framework** spanning 15 paired
   nominal, in-distribution, out-of-distribution, payload, disturbance, sensor-dropout, reaching,
   tracking, and manipulation scenarios. The protocol supports 100 episodes per scenario,
   five learned-policy seeds, four controller families, four ablations, episode-bootstrap 95%
   confidence intervals, paired differences, failure categories, controller latency, stability,
   slip, energy, contact impulse, and safety-intervention logs.

6. **Established reproducible engineering gates and failure analysis** with model-generation
   scripts, YAML experiment definitions, deterministic seeds, Parquet/CSV/TensorBoard artifacts,
   Docker and Slurm entry points, 20 mechanical model checks, and 27 automated tests. Diagnosed a
   stage-5 false-positive learned-policy completion after seven server checkpoints and halted the
   multi-day study; corrected success now requires stage-7 ground support, gripper release,
   upright orientation, and placement tolerance before any policy is counted as successful.

## Recommended four-bullet CV version

- Architected **MoonBot-X**, a MuJoCo mobile-manipulation platform combining a six-wheel,
  four-steer rocker-bogie rover with a 7-DOF Panda arm; modeled 23 joints, 18 actuator channels,
  floating-base/contact dynamics, IMU/encoders, and fingertip force sensing.
- Implemented PID, gravity-compensated and operational-space arm control, CasADi/IPOPT MPC with
  fallback, residual PPO with domain randomization, and OSQP actuator/rate projection through a
  **76-D observation / 14-D action** Gymnasium interface.
- Validated friction-only ground manipulation with **0.808 m rover travel, 0.795 m payload
  transport, 100% bilateral transport contact, 1.30 mm maximum grasp drift, and 1.11 mm placement
  error**, without welds or kinematic object attachment.
- Built a **15-scenario, five-seed** evaluation and ablation framework with paired episodes,
  bootstrap confidence intervals, failure analysis, and reproducible artifacts; caught and fixed
  a false-positive completion gate so learned success requires physical ground placement and
  released bilateral contact at stage 7.

## Claim boundary

MoonBot-X is currently simulation-only. The learned-policy study was stopped after its original
server code incorrectly treated stage 5 as completion. Existing learned checkpoints are retained
for diagnosis, not presented as successful end-to-end manipulation policies. Hardware system
identification, structural verification, ROS 2 deployment, and sim-to-real evaluation remain
future work.
