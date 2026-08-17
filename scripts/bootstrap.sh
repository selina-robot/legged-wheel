#!/usr/bin/env bash
# One-shot environment bootstrap (spec §5-§7). Idempotent.
set -euo pipefail
cd "$(dirname "$0")/.."

for c in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda "$HOME/miniforge3"; do
  [ -f "$c/etc/profile.d/conda.sh" ] && source "$c/etc/profile.d/conda.sh" && break
done
conda activate locowheel

# Conda gcc/g++ matches conda-forge libraries (boost 1.90 needs GLIBCXX_3.4.32).
export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc"
export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"

git submodule update --init --recursive

# MuJoCo 3.3.6 must live in ~/.mujoco (spec §7).
if [ ! -d "$HOME/.mujoco/mujoco-3.3.6" ]; then
  echo "ERROR: ~/.mujoco/mujoco-3.3.6 missing." >&2
  exit 1
fi
ln -sfn "$HOME/.mujoco/mujoco-3.3.6" third_party/unitree_mujoco/simulate/mujoco

# unitree_sdk2 -> third_party/install (no sudo, spec §6).
cmake -S third_party/unitree_sdk2 -B build/unitree_sdk2 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/third_party/install"
cmake --build build/unitree_sdk2
cmake --install build/unitree_sdk2

# Official C++ simulator (spec §7). Do not patch simulator logic.
# SPDLOG_FMT_EXTERNAL: conda-forge spdlog has no bundled fmt (build flag only).
# Only the `unitree_mujoco` target is built; `jstest` is an unused joystick tool.
# The simulator resolves config.yaml and unitree_robots relative to the
# executable location (simulate/build/unitree_mujoco -> proj_dir = simulate/).
cmake -S third_party/unitree_mujoco/simulate -B third_party/unitree_mujoco/simulate/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install;$CONDA_PREFIX" \
  -DCMAKE_CXX_FLAGS="-DSPDLOG_FMT_EXTERNAL -I$CONDA_PREFIX/include" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib"
cmake --build third_party/unitree_mujoco/simulate/build --target unitree_mujoco

echo "bootstrap OK"
