#!/usr/bin/env bash
# Generate the diagnostics report (12 plots + metrics) for the latest run
# (spec §72-§73).
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
exec python python/analysis/make_report.py "$@"
