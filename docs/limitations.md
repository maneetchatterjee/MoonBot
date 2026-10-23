# Limitations and claim boundary

- The rover is a 2x geometry-scale, ballasted Sawppy/Curio derivative, not a stock hardware configuration.
- The arm is the MuJoCo Menagerie Panda rigid-body/mesh model with adapted torque actuation, not manufacturer-certified CAD or a safety model.
- Rover masses, inertias, friction, tire compliance, motor limits, steering limits, and sensors are provisional and uncalibrated.
- The rocker differential is a compliant equality rather than explicit mechanical linkages.
- The Panda adapter and frame have not been structurally analyzed for reaction loads, fatigue, payload, or impact.
- Terrain uses parameterized analytic boxes and a plane; deformable soil, wheel sinkage, and high-fidelity tire mechanics are absent.
- The manipulation smoke run is friction-contact-only, but it uses a light 20 g foam block, 56 mm custom silicone jaw liners, and an assumed friction coefficient of 2.0. These parameters require hardware identification, and the successful simulated grasp is not a hardware force-closure claim.
- Safety-QP actuator/rate bounds are enforced, but attitude, stability-margin, and joint-proximity rules are heuristic rather than formal CBF certificates.
- Local PPO and residual-PPO checkpoints are 256-step smoke artifacts and have no scientific performance significance.
- The 36-episode controller comparison is deliberately short and has zero task completions; it validates logging and pairing only.
- An initial GPU study produced seven one-million-step checkpoints, but its deployed revision incorrectly counted manipulation stage 5 as success. The study was stopped, and those checkpoints are diagnostic artifacts rather than evidence of released ground placement.
- The corrected source requires stage 7, upright ground placement, ground contact, and released bilateral finger contact. A short corrected multi-seed gate must pass before the five-seed, 100-episode study is restarted.
- No ROS 2 deployment, cross-simulator study, physical-hardware experiment, or sim-to-real result has been performed.
