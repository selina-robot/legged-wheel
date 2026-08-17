#!/usr/bin/env bash
# Direct-spawn the robot at the rear-wheel equilibrium and run leg impedance +
# rear-wheel LQR balance (spec §23). Requires data/equilibrium/rear_equilibrium.yaml.
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
export LD_LIBRARY_PATH="$PWD/third_party/install/lib:${CONDA_PREFIX:-}/lib:${LD_LIBRARY_PATH:-}"
exec ./build/go2w_standup/go2w_standup_controller --mode balance "$@"
