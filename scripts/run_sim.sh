#!/usr/bin/env bash
# Launch the official unitree_mujoco simulator with Go2W (spec §7).
set -euo pipefail
cd "$(dirname "$0")/.."

for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel 2>/dev/null || true

export LD_LIBRARY_PATH="$PWD/third_party/install/lib:${CONDA_PREFIX:-}/lib:${LD_LIBRARY_PATH:-}"
exec ./third_party/unitree_mujoco/simulate/build/unitree_mujoco -r go2w -s scene.xml "$@"
