#!/usr/bin/env bash
# Build the go2w_standup controller (Release).
set -euo pipefail
cd "$(dirname "$0")/.."

for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel

export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc"
export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"
export CMAKE_PREFIX_PATH="$PWD/third_party/install:$CONDA_PREFIX:${CMAKE_PREFIX_PATH:-}"

cmake -S . -B build/go2w_standup -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/go2w_standup
echo "build OK"
