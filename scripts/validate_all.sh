#!/usr/bin/env bash
# Run all python + C++ tests (spec §74).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
python -m pytest tests/ -v
if [ -d build/go2w_standup ]; then
  (cd build/go2w_standup && ctest --output-on-failure)
fi
