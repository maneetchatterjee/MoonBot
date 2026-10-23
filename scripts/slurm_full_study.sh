#!/usr/bin/env bash
#SBATCH --job-name=moonbot-x
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=48:00:00
#SBATCH --output=results/slurm-%j.out

set -euo pipefail
export MUJOCO_GL=egl
export PYTHON_BIN="${PYTHON_BIN:-python}"
bash scripts/run_gpu_study.sh
