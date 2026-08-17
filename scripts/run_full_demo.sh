#!/usr/bin/env bash
# Full FSM demo: reset -> PREPARE -> RISE -> CAPTURE -> BALANCE 20 s (spec §70).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
export LD_LIBRARY_PATH="$PWD/third_party/install/lib:${CONDA_PREFIX:-}/lib:${LD_LIBRARY_PATH:-}"
exec ./build/go2w_standup/go2w_standup_controller --mode full "$@"
