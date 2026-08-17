#!/usr/bin/env bash
# Gate 1: per-motor LowCmd/LowState smoke test (spec §12).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
export LD_LIBRARY_PATH="$PWD/third_party/install/lib:${CONDA_PREFIX:-}/lib:${LD_LIBRARY_PATH:-}"
mkdir -p artifacts/reports/model_audit
exec ./build/go2w_standup/lowlevel_io_smoke_test "$@"
