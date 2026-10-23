# MoonBot-X research report

## 1. Abstract

MoonBot-X studies robust whole-body control for a six-wheel rocker-bogie rover carrying a Franka Panda manipulator. The repository implements mechanically checked simulation, classical and predictive control, PPO and residual PPO, domain randomization, safety projection, paired evaluation, and headless evidence generation. Local smoke runs and a deterministic friction-contact mobile-manipulation demonstration verify execution. An initial learned-policy server study was halted after audit exposed a false-positive completion gate; no learned end-to-end manipulation result is claimed.

## 2. Motivation

Mobile manipulation on uneven terrain couples wheel slip, suspension motion, base attitude, arm reaction forces, payload uncertainty, and sensing delay. Comparing learning only against weak control obscures whether learned residuals actually compensate for model error. This project therefore uses progressively stronger, shared-task baselines.

## 3. Related work

The design is informed by rocker-bogie mobility, operational-space manipulation, reduced-order MPC, residual policy learning, domain randomization, and action projection. Upstream assets and licenses are recorded separately. A literature-quality related-work synthesis remains part of the paper-writing stage and should use primary papers.

## 4. Robot model

The platform combines a scaled Sawppy/Curio rover with an unscaled Menagerie Panda. Visual meshes are separated from simplified collision geometry. Six wheel, four steering, four passive suspension, seven arm, and two coupled finger joints are modeled.

## 5. Dynamics and task formulation

MuJoCo integrates the floating-base system at 500 Hz. Tasks progress from velocity tracking through rough-terrain navigation, reaching, mobile tracking, payload transport, and staged manipulation. Deployment observations exclude privileged state.

## 6. Baseline controllers

The classical baseline combines velocity PID, gravity-compensated joint control, operational-space control, and heuristic stability scaling. CasADi/IPOPT MPC optimizes reduced planar velocity dynamics and falls back to PID on failure or timeout.

## 7. Residual-learning method

PPO either commands normalized actuator effort directly or bounded residuals over nominal wheel/arm control. Residual magnitude, rate, and smoothing limits constrain the learned correction. Curriculum and randomization are configuration driven.

## 8. Safety constraints

OSQP enforces actuator and action-rate bounds. Attitude, static margin, and joint proximity modify those bounds heuristically. Solver status, trigger names, action changes, and solve time are logged. No unsupported formal CBF claim is made.

## 9. Experimental protocol

Final evaluation calls for five learned-policy seeds, at least 100 episodes per scenario, paired scenario seeds, nominal/ID/OOD/sensor-failure tests, episode-level metrics, bootstrap confidence intervals, and paired differences.

## 10. Results

Mechanical validation passes 20 checks. The deterministic contact-only mobile manipulation run picked a 20 g foam block from the ground, lifted it to 0.264 m, drove the rover 0.808 m, transported the block 0.795 m, and placed it upright with 1.11 mm planar error. Bilateral finger contact was retained for 100% of rover transport with 1.30 mm maximum measured grasp drift, and the model contains no grasp attachment constraint. Its visually inspected 25.1-second video shows approach, physical closure, lift, articulated rover motion, ground lowering, and release. Pure and residual PPO each completed a 256-step optimization smoke run and produced reloadable checkpoints. A 36-episode six-controller smoke benchmark completed. These results verify execution only; no learned-controller superiority claim is justified.

## 11. Ablations

The pipeline supports removal of domain randomization, safety filtering, stability/slip reward terms, stability coordination, and arm-base coordination. Delay, payload, and terrain extrapolation are represented as held-out scenarios. Seven initial one-million-step checkpoints were retained for diagnosis after the first server study was stopped; they are not valid evidence of learned mobile manipulation because the deployed revision used a permissive completion gate.

## 12. Failure analysis

Early integration found an unreachable ground-level manipulation target, an inconsistent safety-QP bound intersection, and unstable slip from short stock fingertip contacts. The final smoke task uses a 30-degree arm adapter, full-pose joint-margin-aware IK, physically modeled custom jaw liners, continuous contact-preserving trajectories, and a safe-hold rule for conflicting QP bounds. Audit of the first server study then found that its deployed revision marked stage 5 as success while the object was still held. One reported successful rollout ended after 99 steps with 0.0855 m payload displacement and 0.483 m base error. The corrected source requires stage 7, ground contact, upright placement, and released bilateral contact; an automated regression test prevents stage-5 or stage-6 success.

## 13. Limitations

Rover parameters are provisional, terrain/contact remain simplified, jaw friction and block material are assumed, and corrected learned-policy training/evaluation remain incomplete. The simulated contact grasp is not evidence of hardware force closure. Hardware and cross-simulator claims are absent.

## 14. Sim-to-real plan

The plan replaces assumptions with CAD and system identification, validates emergency behavior at low speed, introduces ROS 2 only as a deployment/logging layer, and advances through wheels-up, driving, arm-only, coordination, slope, payload, and grasp tests.

## 15. Conclusion

The repository provides an executable research pipeline with explicit physical success gates. The next evidence-producing step is a short corrected multi-seed manipulation gate; only after that gate passes should the predefined full GPU study be restarted and interpreted using episode-level outputs and failure videos.
