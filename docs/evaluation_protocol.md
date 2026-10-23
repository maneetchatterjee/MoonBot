# Evaluation protocol

## Final design

The main study uses five training seeds for PPO variants and at least 100 episodes per scenario. Every controller receives identical scenario IDs and seeds. Policies are deterministic during evaluation. Scenario groups cover nominal, in-distribution randomization, held-out terrain, low friction, payload variation, disturbances, IMU dropout, encoder dropout, reaching, mobile tracking, and manipulation.

Controllers:

1. PID plus joint-space arm control.
2. MPC plus operational-space arm control.
3. End-to-end PPO.
4. Residual PPO.
5. Domain-randomized residual PPO.
6. Safety-filtered residual PPO where trained/evaluated separately.

## Saved evidence

Each row of `episodes.parquet` is one episode, never one physics timestep. Stored metrics include success, return, base position and heading RMSE, end-effector position and orientation RMSE, slip, integrated effort, peak actuator force, wheel-contact impulse, payload displacement, collision rate, roll/pitch peaks, minimum stability margin, violation/intervention rates, disturbance recovery time, controller latency, MPC solve time, and MPC fallback rate. `summary.csv` contains mean, sample standard deviation, and episode-bootstrap 95% intervals. `paired_differences.csv` compares controllers on identical scenario seeds. `failure_counts.csv` preserves timeout, collision, tip-over, and numerical categories.

`safety_interventions.parquet` logs every QP intervention with the trigger set, original and filtered actions, solve time/status, and eventual episode success. `representative_trajectories.parquet` stores one complete paired trajectory per controller/scenario for plots without treating timesteps as independent samples.

## Ablations

Training exposes removal of domain randomization, safety filtering, stability reward, slip reward, the stability coordinator, and arm-base coordination. Increased delay and payload/terrain extrapolation are evaluated through held-out manifests. Privileged-observation removal is not applicable because the current policy observation contains no privileged simulator state.

## Smoke versus final evidence

The local smoke benchmark uses one or two episodes and short horizons only to verify execution, checkpoint loading, paired seeds, storage, aggregation, confidence-interval code, and plotting. It must not be used to rank controllers. A first server study was stopped after detecting a stage-5 false-positive manipulation gate. Final tables may be produced only after the corrected stage-7 completion rule passes a short multi-seed gate and the full study is rerun.
