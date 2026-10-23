#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/manee/venv/bin/python}"
SMOKE_STEPS="${SMOKE_STEPS:-2048}"

cd "$ROOT"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

"$PYTHON_BIN" scripts/build_mobile_panda_model.py
"$PYTHON_BIN" -m moonbot.cli validate-model --output results/model_validation.json
"$PYTHON_BIN" -m pytest -q
"$PYTHON_BIN" -m ruff check .
"$PYTHON_BIN" -m moonbot.cli pick-place-demo \
  --output results/pick_place_smoke \
  --render \
  --video-output results/video_smoke/rollout.mp4
"$PYTHON_BIN" -m moonbot.cli train \
  --config configs/training/ppo_smoke.yaml \
  --steps "$SMOKE_STEPS" \
  --seed 11 \
  --output results/training_ppo_smoke
"$PYTHON_BIN" -m moonbot.cli train \
  --config configs/training/smoke.yaml \
  --steps "$SMOKE_STEPS" \
  --seed 7 \
  --output results/training_smoke
"$PYTHON_BIN" -m moonbot.cli benchmark \
  --controllers pid mpc ppo residual_ppo residual_ppo_dr residual_ppo_safe \
  --checkpoint ppo=results/training_ppo_smoke/checkpoints/final.zip \
  --checkpoint residual_ppo=results/training_smoke/checkpoints/final.zip \
  --checkpoint residual_ppo_dr=results/training_smoke/checkpoints/final.zip \
  --checkpoint residual_ppo_safe=results/training_smoke/checkpoints/final.zip \
  --scenario-manifest configs/evaluation/smoke.yaml \
  --episodes 2 \
  --maximum-steps 25 \
  --output results/full_pipeline_smoke
"$PYTHON_BIN" -m moonbot.cli plot --results results/full_pipeline_smoke
"$PYTHON_BIN" -m moonbot.cli aggregate \
  --inputs results/full_pipeline_smoke \
  --output results/aggregate_smoke
"$PYTHON_BIN" -m moonbot.cli plot-learning \
  --inputs results/training_ppo_smoke/logs/progress.csv \
    results/training_smoke/logs/progress.csv \
  --output results/full_pipeline_smoke/plots/learning_curves.png
