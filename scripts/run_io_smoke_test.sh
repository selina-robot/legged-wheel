#!/usr/bin/env bash
# Gate 1: per-motor LowCmd/LowState smoke test (spec §12).
# Runs against an already-running simulator (./scripts/run_sim.sh); the test
# drives the simulator's built-in BACKSPACE reset (mj_resetData) through
# tools/x11_sim_reset so each motor is pulsed during the spawn free fall.
set -euo pipefail
cd "$(dirname "$0")/.."
for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel
export LD_LIBRARY_PATH="$PWD/third_party/install/lib:${CONDA_PREFIX:-}/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:1}"
mkdir -p artifacts/reports/model_audit artifacts/logs

# Build the X11 reset helper if missing.
if [ ! -x build/x11_sim_reset ]; then
  cc -O2 tools/x11_sim_reset.c -o build/x11_sim_reset -lX11 -lXtst
fi

exec ./build/go2w_standup/lowlevel_io_smoke_test "$@"
