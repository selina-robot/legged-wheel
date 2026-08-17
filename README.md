# Go2W Rear-Wheel Stand-Up Controller

Architecture:
PREPARE → fixed-contact optimized RISE → LQR CAPTURE → rear-wheel BALANCE

Runtime:
500 Hz WBC / 500 Hz joint impedance / 500 Hz rear-wheel LQR

Simulation:
official Unitree Go2W MuJoCo model (`third_party/unitree_mujoco/unitree_robots/go2w/go2w.xml`)

## Quick start

```bash
conda activate locowheel
./scripts/bootstrap.sh
./scripts/build.sh
./scripts/run_full_demo.sh
```

## V1 target

- 20/20 nominal stand-up
- 20 s rear-wheel balance

实施规格与 Gate 定义见 `doc/Go2W_双轮站立_全新项目_Agent执行Plan.md`；工程约束见 `AGENTS.md`。
