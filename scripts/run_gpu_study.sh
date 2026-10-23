#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"
FINAL_STEPS="${FINAL_STEPS:-1000000}"
SEEDS="${SEEDS:-1 2 3 4 5}"
RUN_ABLATIONS="${RUN_ABLATIONS:-1}"

cd "$ROOT"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

"$PYTHON_BIN" scripts/build_mobile_panda_model.py
"$PYTHON_BIN" -m moonbot.cli validate-model --output results/model_validation.json
"$PYTHON_BIN" -m pytest -q
"$PYTHON_BIN" -m ruff check .

for seed in $SEEDS; do
  "$PYTHON_BIN" -m moonbot.cli train \
    --config configs/training/ppo.yaml --steps "$FINAL_STEPS" --seed "$seed" \
    --output "results/final/ppo/seed_${seed}"
  "$PYTHON_BIN" -m moonbot.cli train \
    --config configs/training/residual_ppo_nominal.yaml --steps "$FINAL_STEPS" --seed "$seed" \
    --output "results/final/residual_ppo/seed_${seed}"
  "$PYTHON_BIN" -m moonbot.cli train \
    --config configs/training/residual_ppo_dr.yaml --steps "$FINAL_STEPS" --seed "$seed" \
    --output "results/final/residual_ppo_dr/seed_${seed}"
  "$PYTHON_BIN" -m moonbot.cli train \
    --config configs/training/residual_ppo.yaml --steps "$FINAL_STEPS" --seed "$seed" \
    --output "results/final/residual_ppo_safe/seed_${seed}"

  for controller in ppo residual_ppo residual_ppo_dr residual_ppo_safe; do
    "$PYTHON_BIN" -m moonbot.cli evaluate \
      --controller "$controller" \
      --checkpoint "results/final/${controller}/seed_${seed}/checkpoints/final.zip" \
      --policy-seed "$seed" \
      --scenario-manifest configs/evaluation/full_benchmark.yaml \
      --output "results/final/evaluation/${controller}_seed_${seed}"
  done

  if [[ "$RUN_ABLATIONS" == "1" ]]; then
    for ablation in no_stability_penalty no_slip_penalty no_stability_coordinator no_arm_base_coordination; do
      "$PYTHON_BIN" -m moonbot.cli train \
        --config configs/training/residual_ppo.yaml \
        --ablation "$ablation" \
        --steps "$FINAL_STEPS" \
        --seed "$seed" \
        --output "results/final/ablations/${ablation}/seed_${seed}"
      "$PYTHON_BIN" -m moonbot.cli evaluate \
        --controller residual_ppo_safe \
        --label "ablation_${ablation}" \
        --checkpoint "results/final/ablations/${ablation}/seed_${seed}/checkpoints/final.zip" \
        --policy-seed "$seed" \
        --scenario-manifest configs/evaluation/full_benchmark.yaml \
        --output "results/final/ablation_evaluation/${ablation}_seed_${seed}"
    done
  fi
done

"$PYTHON_BIN" -m moonbot.cli benchmark \
  --controllers pid mpc \
  --scenario-manifest configs/evaluation/full_benchmark.yaml \
  --output results/final/classical_benchmark

"$PYTHON_BIN" -m moonbot.cli aggregate \
  --inputs results/final/classical_benchmark results/final/evaluation/* \
  --output results/final/combined_benchmark
"$PYTHON_BIN" -m moonbot.cli plot --results results/final/combined_benchmark
"$PYTHON_BIN" -m moonbot.cli plot-learning \
  --inputs results/final/*/seed_*/logs/progress.csv \
  --output results/final/combined_benchmark/plots/learning_curves.png

for controller in pid mpc ppo residual_ppo residual_ppo_dr residual_ppo_safe; do
  checkpoint_args=()
  if [[ "$controller" != "pid" && "$controller" != "mpc" ]]; then
    checkpoint_args=(
      --checkpoint "results/final/${controller}/seed_1/checkpoints/final.zip"
    )
  fi
  "$PYTHON_BIN" -m moonbot.cli rollout \
    --task waypoint_navigation \
    --controller "$controller" \
    "${checkpoint_args[@]}" \
    --terrain asymmetric \
    --terrain-difficulty 0.9 \
    --domain-randomization \
    --external-push-n 120 \
    --seed 4242 \
    --duration 8 \
    --render \
    --output "results/final/comparison_videos/${controller}"
done

if [[ "$RUN_ABLATIONS" == "1" ]]; then
  "$PYTHON_BIN" -m moonbot.cli aggregate \
    --inputs results/final/ablation_evaluation/* \
    --output results/final/combined_ablations
  "$PYTHON_BIN" -m moonbot.cli plot --results results/final/combined_ablations
fi
