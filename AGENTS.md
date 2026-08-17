# AGENTS.md — Go2W Rear-Wheel Stand-Up 工程约束

本项目为 Unitree Go2W 后轮双轮站立控制工程（V1）。实施规格见 `doc/Go2W_双轮站立_全新项目_Agent执行Plan.md`。以下约束对任何 coding agent 生效：

1. V1 only implements rear-wheel two-wheel stand.
2. Use official unitree_mujoco Go2W MJCF as the single model source of truth.
3. Do not create a new Go2W URDF/MJCF.
4. Do not replace MuJoCo before nominal Gate passes.
5. Do not implement a custom rigid-body dynamics library.
6. Do not implement a custom QP solver.
7. Use Pinocchio for dynamics.
8. Use CasADi+IPOPT for offline fixed-contact trajectory optimization.
9. Use ProxQP for runtime WBC.
10. Use a 61-knot, 1.50 s fixed contact schedule for V1.
11. Contact schedule is fixed; no contact-implicit optimization.
12. Finish and validate rear-wheel LQR balance before stand-up trajectory.
13. Terminal trajectory target must be the measured LQR capture basin.
14. Left-right symmetry is a hard constraint in V1.
15. Hip abduction is not a free trajectory optimization variable in V1.
16. All controller failures must be diagnosed from logs before changing architecture.
17. Change one tuning dimension at a time.
18. A Gate failure blocks the next phase.
19. Every experiment saves config, git hash, logs, metrics, and plots.
20. Do not update third-party dependencies after Gate 0 without an explicit documented reason.

## 构建与运行

- conda 环境：`locowheel`（`conda activate locowheel`）。
- 构建：`./scripts/bootstrap.sh`（一次）→ `./scripts/build.sh`。
- 运行 demo：`./scripts/run_full_demo.sh`。
- Gate 1 关节映射测试：先 `./scripts/run_sim.sh`（需 GUI），再 `./scripts/run_io_smoke_test.sh`。该测试在自由落体阶段逐电机施加 ±0.5 Nm 脉冲，并通过 `tools/x11_sim_reset.c`（XTest 向 MuJoCo 窗口发送 BACKSPACE）触发 `mj_resetData`，每电机两次 reset。
- 第三方依赖锁定在 `THIRD_PARTY.lock`，不要升级。

## Gate 顺序

Gate 0（环境/IO）→ Gate 1（joint map）→ Gate 2（equilibrium）→ Gate 3（LQR balance）→ Gate 4（trajopt）→ Playback Gate → WBC Gate → Gate 5（20/20 FSM）。任一 Gate 不过，禁止进入下一阶段。
