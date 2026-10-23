# Repository and source audit

## Initial state

The workspace audit on 2026-10-01 found no supplied MoonBot CAD, `.SLDASM`, `.SLDPRT`, URDF, MJCF, rover mesh, policy, controller, or evaluation artifact. Unrelated repositories were left unchanged, and `MoonBot-X` was created as a new project.

## Model selection

Several public rover sources were assessed. Curio's simulation representation of Sawppy was selected because it provided a six-wheel rocker-bogie topology, usable STL assemblies, and a permissive BSD 3-Clause license. The exact Franka arm was sourced from MuJoCo Menagerie's Apache-2.0 Panda model.

The rover meshes and Panda assets were copied into `assets/` so normal model generation, validation, tests, and rendering do not depend on the ignored `third_party/` checkout or network access. Source revisions and license copies are documented in `docs/external_assets.md`.

## Composition decision

Using stock hobby-scale Sawppy dimensions with an unscaled Panda would produce an implausible mass and geometry pairing. The rover visual geometry is therefore uniformly scaled by 2x and placed around a 45 kg ballasted chassis with a 2.5 kg arm adapter. Rover collision geometry, inertias, actuation, and limits remain research assumptions. The Panda geometry and inertial tensors are kept unscaled, but its actuation is changed to torque control.

The generated MJCF is therefore a reproducible composite research model, not a CAD-ready claim about either upstream robot.

## Development runtime

The final implementation was exercised through Ubuntu 24.04 WSL with `/home/manee/venv/bin/python` (Python 3.12.3). The environment contains MuJoCo 3.3.5, Gymnasium 1.2.3, Stable-Baselines3 2.7.1, PyTorch 2.8.0, and the numerical/plotting stack. CasADi 3.8.1, OSQP 1.1.3, and Ruff 0.16.10 were added because they were absent when the venv was audited. The Python version differs from the preferred 3.11 deployment image but remains within the declared support range.

The end-to-end smoke run produced model validation, contact-only ground pick/place evidence, pure and residual PPO checkpoints, paired controller episodes, Parquet/CSV summaries, plots, and an MP4. A later server study produced seven one-million-step checkpoints before audit found a stage-5 false-positive completion rule in the deployed revision. The study was stopped, the checkpoints were retained for diagnosis, and the current source now requires stage-7 physical release before manipulation success.

## Reproduction commands

```powershell
$env:PYTHONPATH='M:\Portfolio\MoonBot-X'
python scripts\build_mobile_panda_model.py
python -m moonbot.cli validate-model --output results\model_validation.json
python -m moonbot.cli rollout --duration 5 --linear-velocity 0.5 --output results\pid_smoke
python -m moonbot.cli gripper-demo --output results\gripper_smoke
pytest
ruff check .
bash scripts/run_headless_smoke.sh
```
