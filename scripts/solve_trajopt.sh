#!/usr/bin/env bash
# Solve the 61-knot fixed-contact RISE trajectory (spec §32-§49, Gate 4).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
python python/trajopt/solve_rise.py "$@"
python python/trajopt/validate_solution.py
