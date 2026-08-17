#!/usr/bin/env bash
# Solve the rear-wheel equilibrium (spec §18-§22, Gate 2).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
python python/equilibrium/solve_rear_equilibrium.py "$@"
python python/equilibrium/validate_equilibrium.py
