# Go2W 双轮站立控制工程：Agent 执行计划

> **用途**：这不是调研报告，而是一个全新工程的实施规格。  
> **执行对象**：coding agent / coder。  
> **目标**：基于开源组件，在 Unitree Go2W 官方 MuJoCo 模型中，从零建立一个可重复的“后轮双轮正立”控制工程。  
> **V1 最终架构固定为**：
>
> **四阶段 FSM + 61-knot fixed-contact trajectory optimization + 500 Hz WBC + 500 Hz joint impedance + 500 Hz rear-two-wheel LQR。**
>
> 第一版只做 **rear-wheel stand（后轮双轮正立）**。不要同时做前轮倒立。
>
> 本文档是 V1 的执行约束。Agent 不需要重新选择技术路线；只有某个 Gate 明确失败时，才允许按本文档指定的 fallback 处理。

---

# 0. Agent 执行契约

## 0.1 最终要交付的东西

工程最终必须做到：

```text
Go2W 四轮正常初始姿态
        ↓
PREPARE
        ↓
RISE
        ↓
CAPTURE
        ↓
BALANCE
        ↓
后轮双轮稳定站立 ≥ 20 s
```

在 nominal MuJoCo 仿真中：

```text
20/20 次完整执行成功
```

并且：

- 不依赖人工外力；
- 不依赖运行时手工拖动机器人；
- 不需要在线求 trajectory optimization；
- trajectory optimization 只离线运行；
- WBC 实时运行；
- LQR 实时运行；
- FSM 实时运行；
- 控制输出通过 Unitree `LowCmd` 语义组织；
- 保留以后切换到真实 Go2W 的 backend 接口。

---

## 0.2 V1 明确不做的事情

V1 不允许擅自增加以下工作：

```text
通用 locomotion
复杂地形
前轮倒立
动作风格优化
大范围初态恢复
全局自起身
多技能统一 policy
在线 MPC
contact-implicit optimization
同时更换 simulator
自研 rigid-body dynamics
自研 QP solver
自研 URDF/MJCF parser
```

V1 的目标只有一个：

> **把后轮双轮正立做成一个 deterministic、动力学可解释、可重复的闭环 demo。**

---

# 1. 固定技术栈

Agent 不再比较框架，直接采用下面组合。

## 1.1 Simulator / Robot model

使用：

```text
unitreerobotics/unitree_mujoco
```

仓库：

```text
https://github.com/unitreerobotics/unitree_mujoco
```

使用其中官方 Go2W：

```text
unitree_robots/go2w/go2w.xml
unitree_robots/go2w/scene.xml
```

**不要重新做 Go2W MJCF。**

**不要用自己以前的 Go2W 模型替代 V1 source of truth。**

`unitree_mujoco` 当前仓库已经包含：

```text
unitree_robots/go2w/
```

且 simulator 配置支持：

```text
robot: "go2w"
```

官方 simulator 同时提供 Unitree `LowCmd` / `LowState` 语义，因此它是本项目 V1 的仿真和未来 sim-to-real 接口基准。

---

## 1.2 Low-level interface

使用：

```text
unitreerobotics/unitree_sdk2
```

仓库：

```text
https://github.com/unitreerobotics/unitree_sdk2
```

固定使用已发布版本：

```text
v2.0.2
```

controller 必须围绕以下 motor command 语义设计：

```cpp
q
dq
kp
kd
tau
```

不要在业务代码中直接散落 DDS 读写。

建立统一：

```cpp
RobotState
MotorCommand
RobotBackend
```

后面 MuJoCo 和真机只切 backend/config，不切 controller。

---

## 1.3 Dynamics

使用：

```text
Pinocchio
```

仓库：

```text
https://github.com/stack-of-tasks/pinocchio
```

V1 使用：

```text
Pinocchio 4.0.x
```

用途：

```text
MJCF model parsing
forward kinematics
frame Jacobian
CoM
CRBA / mass matrix
nonlinear effects
RNEA
contact Jacobian
WBC dynamics
trajectory optimization dynamics
```

**禁止自己写刚体动力学。**

Pinocchio 直接加载与 MuJoCo 相同的：

```text
third_party/unitree_mujoco/unitree_robots/go2w/go2w.xml
```

不要维护第二套动力学参数。

---

## 1.4 Offline trajectory optimization

使用：

```text
CasADi + IPOPT
```

仓库：

```text
https://github.com/casadi/casadi
```

固定 stable：

```text
CasADi 3.7.2
```

用途：

```text
fixed contact schedule
direct transcription / multiple shooting
61 knots
nonlinear dynamics constraints
rolling constraints
friction constraints
torque limits
terminal capture constraints
```

需要验证：

```python
import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as ca
```

如果：

```python
import pinocchio.casadi
```

失败：

**Gate 0 不通过。**

先修好 Pinocchio 的 CasADi support，再继续。

不要换算法绕过这个问题。

---

## 1.5 WBC QP

使用：

```text
ProxSuite / ProxQP
```

仓库：

```text
https://github.com/Simple-Robotics/proxsuite
```

用途：

```text
500 Hz inverse-dynamics WBC QP
```

不要自己写 active-set / interior-point QP solver。

安装成功后冻结 resolved version 到：

```text
environment.lock.yml
THIRD_PARTY.lock
```

后续不升级。

---

## 1.6 LQR

不要引入大型控制框架。

Python 离线辨识和求增益使用：

```text
NumPy
SciPy
```

C++ runtime 只保存最终：

```text
A
B
K
x_eq
u_limit
```

运行：

```text
u = -K(x - x_eq)
```

---

## 1.7 只作为参考、不作为项目基座

可参考：

```text
https://github.com/unitreerobotics/unitree_rl_lab
```

尤其：

```text
deploy/robots/go2w/
source/.../assets/robots/unitree.py
```

用途仅限：

- Go2W SDK joint ordering 参考；
- Unitree 官方 FSM 工程组织参考；
- LowCmd / LowState wrapper 参考；
- `State_FixStand` 类似状态实现参考。

**不要 fork 整个训练工程作为本项目。**

这个项目不需要其训练部分。

---

# 2. 为什么工程这样拆

Agent 必须先理解控制器之间的职责，否则很容易把多个控制器调成互相打架。

最终控制链：

```text
                    ┌─────────────────────┐
                    │       FSM           │
                    │ phase/contact mode  │
                    └──────────┬──────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ trajectory player   │
                    │ q*,dq*,tau*,f*,...  │
                    └──────────┬──────────┘
                               │
                ┌──────────────┴──────────────┐
                ▼                             ▼
      ┌──────────────────┐          ┌──────────────────┐
      │       WBC        │          │ rear-wheel LQR   │
      │ inverse dynamics │          │ sagittal balance │
      └─────────┬────────┘          └─────────┬────────┘
                │                             │
                └──────────────┬──────────────┘
                               ▼
                    ┌─────────────────────┐
                    │ joint command layer │
                    │ q/dq/kp/kd/tau_ff  │
                    └──────────┬──────────┘
                               ▼
                    ┌─────────────────────┐
                    │ Unitree LowCmd      │
                    └──────────┬──────────┘
                               ▼
                         MuJoCo / Robot
```

责任必须严格划分。

## FSM

只负责：

```text
当前 phase
contact set
reference 切换
controller enable/disable
abort
```

FSM 不计算动力学。

## Trajectory optimizer

只负责离线产生：

```text
可行动作
q*
dq*
tau_ff*
contact schedule
contact force*
terminal state
```

## WBC

负责：

```text
实时修正 tracking error
满足刚体动力学
满足当前 contact mode
满足 friction / torque constraints
```

## Joint impedance

负责：

```text
局部关节 tracking
高频小误差抑制
```

## rear-two-wheel LQR

只负责最终双轮站立中：

```text
sagittal pitch
pitch rate
wheel displacement
forward velocity
```

BALANCE 阶段：

> **pitch 的主要闭环 authority 给 LQR。**

WBC 不要同时用一个极高权重 pitch task 跟 LQR 对抗。

---

# 3. 工程仓库结构

新建独立仓库：

```text
go2w_standup/
```

结构固定为：

```text
go2w_standup/
├── README.md
├── AGENTS.md
├── CMakeLists.txt
├── pyproject.toml
├── environment.yml
├── environment.lock.yml
├── THIRD_PARTY.lock
│
├── third_party/
│   ├── unitree_mujoco/          # git submodule
│   └── unitree_sdk2/            # git submodule, v2.0.2
│
├── config/
│   ├── robot.yaml
│   ├── sim.yaml
│   ├── control.yaml
│   ├── final_pose.yaml
│   ├── lqr.yaml
│   ├── trajopt.yaml
│   ├── wbc.yaml
│   ├── fsm.yaml
│   ├── logging.yaml
│   └── safety.yaml
│
├── include/go2w_standup/
│   ├── core/
│   │   ├── types.hpp
│   │   ├── joint_map.hpp
│   │   └── math_utils.hpp
│   ├── backend/
│   │   ├── robot_backend.hpp
│   │   └── unitree_lowlevel_backend.hpp
│   ├── estimation/
│   │   ├── state_estimator.hpp
│   │   └── contact_state_provider.hpp
│   ├── control/
│   │   ├── joint_impedance.hpp
│   │   ├── wheel_lqr.hpp
│   │   ├── wbc.hpp
│   │   └── command_mux.hpp
│   ├── trajectory/
│   │   ├── trajectory.hpp
│   │   └── trajectory_player.hpp
│   ├── fsm/
│   │   ├── fsm.hpp
│   │   ├── state.hpp
│   │   ├── prepare_state.hpp
│   │   ├── rise_state.hpp
│   │   ├── capture_state.hpp
│   │   ├── balance_state.hpp
│   │   └── passive_state.hpp
│   └── safety/
│       └── safety_supervisor.hpp
│
├── src/
│   ├── backend/
│   ├── estimation/
│   ├── control/
│   ├── trajectory/
│   ├── fsm/
│   ├── safety/
│   └── app/
│       ├── go2w_standup_controller.cpp
│       └── lowlevel_io_smoke_test.cpp
│
├── python/
│   ├── common/
│   │   ├── config.py
│   │   ├── model.py
│   │   └── io.py
│   ├── model/
│   │   ├── audit_mjcf.py
│   │   ├── compare_mujoco_pinocchio.py
│   │   ├── verify_joint_map.py
│   │   └── inspect_limits.py
│   ├── equilibrium/
│   │   ├── solve_rear_equilibrium.py
│   │   └── validate_equilibrium.py
│   ├── lqr/
│   │   ├── collect_identification_data.py
│   │   ├── fit_linear_model.py
│   │   ├── design_lqr.py
│   │   ├── scan_capture_basin.py
│   │   └── export_lqr.py
│   ├── trajopt/
│   │   ├── go2w_casadi_model.py
│   │   ├── contact_schedule.py
│   │   ├── build_ocp.py
│   │   ├── solve_rise.py
│   │   ├── validate_solution.py
│   │   └── export_trajectory.py
│   ├── analysis/
│   │   ├── plot_balance.py
│   │   ├── plot_trajopt.py
│   │   ├── plot_full_run.py
│   │   └── make_report.py
│   └── tools/
│       ├── run_batch.py
│       └── deterministic_replay.py
│
├── scripts/
│   ├── bootstrap.sh
│   ├── build.sh
│   ├── run_sim.sh
│   ├── run_io_smoke_test.sh
│   ├── run_balance.sh
│   ├── run_full_demo.sh
│   ├── solve_equilibrium.sh
│   ├── solve_trajopt.sh
│   ├── validate_all.sh
│   └── make_report.sh
│
├── data/
│   ├── equilibrium/
│   ├── identification/
│   ├── trajectories/
│   └── capture_basin/
│
├── tests/
│   ├── test_model_load.py
│   ├── test_joint_map.py
│   ├── test_dynamics_consistency.py
│   ├── test_equilibrium.py
│   ├── test_trajectory_schema.py
│   ├── test_lqr_config.cpp
│   ├── test_wbc_qp.cpp
│   ├── test_fsm.cpp
│   └── test_command_safety.cpp
│
└── artifacts/
    ├── logs/
    ├── plots/
    ├── videos/
    └── reports/
```

---

# 4. 第一个 commit：只搭工程，不写控制算法

Commit：

```text
chore: bootstrap Go2W standup project and pin open-source dependencies
```

执行：

```bash
mkdir go2w_standup
cd go2w_standup
git init

git submodule add \
  https://github.com/unitreerobotics/unitree_mujoco.git \
  third_party/unitree_mujoco

git submodule add \
  https://github.com/unitreerobotics/unitree_sdk2.git \
  third_party/unitree_sdk2

cd third_party/unitree_sdk2
git checkout v2.0.2
cd ../..

git submodule update --init --recursive
```

然后把：

```bash
git -C third_party/unitree_mujoco rev-parse HEAD
git -C third_party/unitree_sdk2 rev-parse HEAD
```

写入：

```text
THIRD_PARTY.lock
```

格式：

```yaml
unitree_mujoco:
  url: https://github.com/unitreerobotics/unitree_mujoco.git
  commit: <exact hash>

unitree_sdk2:
  url: https://github.com/unitreerobotics/unitree_sdk2.git
  tag: v2.0.2
  commit: <exact hash>
```

以后除非 Gate 明确要求，**不得更新 submodule**。

---

# 5. 环境

目标开发环境：

```text
Ubuntu 22.04 LTS
Python 3.10
C++17
CMake >= 3.22
Ninja
MuJoCo 3.3.6
```

Python 包：

```text
numpy
scipy
pandas
matplotlib
pyyaml
casadi==3.7.2
pinocchio 4.0.x
proxsuite
pytest
```

建议：

```bash
conda create -n go2w_standup python=3.10 -y
conda activate go2w_standup

conda install -c conda-forge \
  numpy scipy pandas matplotlib pyyaml \
  cmake ninja eigen boost \
  pinocchio=4.0.0 \
  proxsuite \
  -y

pip install casadi==3.7.2 pytest
```

然后：

```bash
python - <<'PY'
import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as ca
import proxsuite
print("pinocchio", pin.__version__)
print("casadi", ca.__version__)
print("proxsuite", proxsuite.__version__)
PY
```

必须成功。

把 resolved environment 导出：

```bash
conda env export --from-history > environment.yml
conda env export > environment.lock.yml
```

---

# 6. 构建 SDK2

不要 sudo 安装到系统。

本项目统一装到：

```text
third_party/install/
```

执行：

```bash
cmake \
  -S third_party/unitree_sdk2 \
  -B build/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PWD/third_party/install

cmake --build build/unitree_sdk2 -j
cmake --install build/unitree_sdk2
```

项目 shell 中设置：

```bash
export CMAKE_PREFIX_PATH=$PWD/third_party/install:$CONDA_PREFIX:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$PWD/third_party/install/lib:$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
```

---

# 7. 构建官方 unitree_mujoco

优先使用其 C++ simulator。

不要先用 Python simulator 做最终控制性能验证。

把 MuJoCo 3.3.6 放入：

```text
~/.mujoco/mujoco-3.3.6
```

按照 `unitree_mujoco` 官方结构完成链接。

然后：

```bash
cmake \
  -S third_party/unitree_mujoco/simulate \
  -B build/unitree_mujoco \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install;$CONDA_PREFIX"

cmake --build build/unitree_mujoco -j
```

运行：

```bash
build/unitree_mujoco/unitree_mujoco \
  -r go2w \
  -s scene.xml
```

如果当前 commit 的实际 CLI 输出目录不同，以其 README/CMake target 为准，但不要修改 simulator 逻辑来“适配”项目。

---

# 8. Gate 0：环境和官方 Go2W 必须完整跑通

Gate 0 必须完成：

```text
[ ] unitree_mujoco 启动 Go2W
[ ] 加载的模型来自 unitree_robots/go2w/go2w.xml
[ ] controller 能收到 LowState
[ ] controller 能发布 LowCmd
[ ] 16 个执行关节均能读取
[ ] 4 个 wheel joint 均能驱动
[ ] Pinocchio 能直接加载同一个 go2w.xml
[ ] pinocchio.casadi 可以 import
[ ] ProxQP C++ example 可以编译运行
```

Gate 0 没通过：

> **禁止写 trajectory optimizer、WBC、FSM。**

---

# 9. Canonical RobotState / MotorCommand

`include/go2w_standup/core/types.hpp`

定义：

```cpp
struct RobotState {
    double time_sec;

    Eigen::Quaterniond base_quat;
    Eigen::Vector3d imu_gyro;
    Eigen::Vector3d imu_accel;

    Eigen::Matrix<double, 16, 1> q;
    Eigen::Matrix<double, 16, 1> dq;
    Eigen::Matrix<double, 16, 1> tau_est;

    bool valid;
};

struct MotorCommand {
    Eigen::Matrix<double, 16, 1> q_des;
    Eigen::Matrix<double, 16, 1> dq_des;
    Eigen::Matrix<double, 16, 1> kp;
    Eigen::Matrix<double, 16, 1> kd;
    Eigen::Matrix<double, 16, 1> tau_ff;

    bool enable;
};
```

注意：

```text
MotorCommand.tau_ff
```

只表示 feedforward / WBC / LQR torque。

不要再在 backend 中自己计算一次：

```text
Kp * error + Kd * error
```

因为 `LowCmd` 已有独立：

```text
q/dq/kp/kd/tau
```

字段。

---

# 10. RobotBackend

接口：

```cpp
class RobotBackend {
public:
    virtual ~RobotBackend() = default;

    virtual bool initialize() = 0;
    virtual bool read(RobotState& state) = 0;
    virtual bool write(const MotorCommand& cmd) = 0;
    virtual void emergencyDamping() = 0;
};
```

实现：

```text
UnitreeLowLevelBackend
```

配置：

## simulation

```yaml
backend:
  type: unitree_lowlevel
  domain_id: 1
  interface: lo
```

## future hardware

```yaml
backend:
  type: unitree_lowlevel
  domain_id: 0
  interface: eth0
```

controller 上层不允许出现：

```cpp
if (mujoco) ...
if (real_robot) ...
```

---

# 11. 关节映射

官方 Go2W 是 16 个执行关节：

```text
4 × [hip, thigh, calf, wheel]
```

V1 的 expected SDK order 先按 Unitree 官方 Go2W deploy 配置初始化为：

```text
0  FR_hip
1  FR_thigh
2  FR_calf

3  FL_hip
4  FL_thigh
5  FL_calf

6  RR_hip
7  RR_thigh
8  RR_calf

9  RL_hip
10 RL_thigh
11 RL_calf

12 FR_wheel
13 FL_wheel
14 RR_wheel
15 RL_wheel
```

注意：

Unitree 的不同资产中，最后四个 wheel joint 可能命名为：

```text
*_foot_joint
```

而官方 MuJoCo MJCF 中使用：

```text
*_wheel_joint
```

本项目 runtime canonical name 一律使用：

```text
FR_wheel_joint
FL_wheel_joint
RR_wheel_joint
RL_wheel_joint
```

并由：

```text
config/robot.yaml
```

维护 SDK index 到 MJCF joint name 的显式映射。

**不要靠字符串排序隐式决定 motor index。**

---

# 12. Gate 1：关节映射必须逐个验证

实现：

```text
python/model/verify_joint_map.py
src/app/lowlevel_io_smoke_test.cpp
```

测试方法：

每次只给一个 motor：

```text
0.2–0.5 Nm
```

持续：

```text
100–200 ms
```

其他 motor：

```text
tau = 0
```

记录：

```text
q change
dq sign
MuJoCo joint movement
```

输出：

```text
artifacts/reports/model_audit/joint_map.csv
```

必须得到：

```text
sdk_index
canonical_name
mjcf_name
axis
positive_direction
wheel_forward_sign
```

尤其确定：

```yaml
wheel_forward_sign:
  FR: ...
  FL: ...
  RR: ...
  RL: ...
```

以后所有 LQR/WBC 先转换成 canonical “forward wheel torque”，最后由 mapping 转成 motor sign。

Gate 1：

```text
[ ] 16/16 joint identity verified
[ ] 16/16 sign verified
[ ] 4/4 forward wheel sign verified
```

少一个都不能继续。

---

# 13. Model audit

实现：

```text
python/model/audit_mjcf.py
```

从：

```text
third_party/unitree_mujoco/unitree_robots/go2w/go2w.xml
```

自动生成：

```text
artifacts/reports/model_audit/model.json
```

记录：

```text
joint names
joint axes
joint ranges
motor ctrlrange
body masses
body inertias
total mass
CoM
wheel radius
wheel axle direction
wheel link frames
ground friction
joint damping
joint armature
```

不要把这些参数手工复制到代码。

代码只从：

```text
config/robot.yaml
```

读取经 audit 固化的必要参数。

---

# 14. MuJoCo / Pinocchio 动力学一致性测试

实现：

```text
python/model/compare_mujoco_pinocchio.py
```

两边都加载同一个 MJCF。

随机生成：

```text
10 个合法、左右对称、无碰撞姿态
```

比较：

```text
total mass
CoM
gravity generalized torque
mass matrix selected blocks
RNEA torque
```

至少保证：

```text
joint order mapping 完全正确
gravity torque relative error < 3%
```

如果误差超过：

```text
3%
```

首先检查：

```text
joint mapping
free-flyer convention
quaternion order
wheel joint order
armature
gravity
```

不要通过“调 WBC 权重”绕过 model mismatch。

---

# 15. V1 控制频率固定

V1：

```yaml
rates:
  physics_hz: 1000
  command_hz: 500
  impedance_hz: 500
  wbc_hz: 500
  lqr_hz: 500
  fsm_hz: 500
  logger_hz: 500
```

对应：

```text
physics dt = 0.001 s
control dt = 0.002 s
```

如果当前 official simulator scene timestep 不是 0.001：

创建本项目自己的 scene wrapper：

```text
config/sim_scene_go2w.xml
```

只允许覆盖：

```text
timestep
ground
camera
sensor/logging
```

不要复制或修改 Go2W body/inertial/joint 参数。

---

# 16. V1 初始条件

固定 initial condition。

第一版不随机。

```yaml
initial:
  base_xy: [0.0, 0.0]
  yaw: 0.0
  linear_velocity: [0.0, 0.0, 0.0]
  angular_velocity: [0.0, 0.0, 0.0]
  joint_velocity: 0
  seed: 1
```

使用一个正常四轮支撑 crouched pose：

```text
q_init
```

存在：

```text
config/robot.yaml
```

所有 trajopt / playback / batch run 都使用完全相同的 `q_init`。

---

# 17. 最终两轮姿态必须先求出来

**不要先做站起轨迹。**

先解决：

> “如果 Go2W 已经处在后轮双轮姿态，我能不能让它稳定站住？”

这是本工程第一个控制目标。

---

# 18. Rear-wheel equilibrium optimization

实现：

```text
python/equilibrium/solve_rear_equilibrium.py
```

使用：

```text
Pinocchio + CasADi + IPOPT
```

目标：

求一个后轮支撑 equilibrium：

```text
q_eq
tau_eq
f_RR
f_RL
```

---

# 19. Equilibrium decision variables

变量：

```text
q
tau_leg
f_RR
f_RL
```

V1：

```text
front wheel torque = 0
rear wheel torque ≈ 0 at exact equilibrium
```

可先把 wheel angle 固定为 0，因为圆轮转角不影响刚体几何。

---

# 20. Equilibrium hard constraints

必须是 hard constraints。

## 20.1 Rear wheel ground geometry

对于后轮 wheel center：

```text
z_RR_center = wheel_radius
z_RL_center = wheel_radius
```

## 20.2 Front wheel clearance

```text
z_FR_center >= wheel_radius + 0.05 m
z_FL_center >= wheel_radius + 0.05 m
```

## 20.3 Roll / yaw

```text
base_roll = 0
base_yaw = 0
```

## 20.4 Left-right symmetry

hip abduction：

```text
固定为经过 joint-sign audit 后的对称 nominal 值
```

thigh/calf：

```text
FR = mirror(FL)
RR = mirror(RL)
```

不要让优化器自由产生 lateral splay。

## 20.5 Static rigid-body dynamics

```text
M(q) * 0 + h(q,0)
=
S^T * tau + J_RR^T f_RR + J_RL^T f_RL
```

## 20.6 Friction

```text
fz >= 0
|fx| <= mu * fz
|fy| <= mu * fz
```

V1 平面对称：

```text
fy = 0
```

## 20.7 Joint / torque limits

使用 MJCF 实际 limit。

求 equilibrium 时只允许：

```text
|tau| <= 0.70 * tau_limit
```

留 margin。

## 20.8 CoM over support axle

令：

```text
x_axle = 0.5 * (x_RR_center + x_RL_center)
```

约束：

```text
|x_com - x_axle| <= 0.005 m
```

这是 final balance working point。

---

# 21. Equilibrium cost

在满足上述 hard constraint 后，最小化：

```text
J =
  w_tau      * ||tau / tau_limit||²
+ w_joint    * ||q - q_nominal||²
+ w_margin   * joint_limit_barrier
+ w_height   * (base_height - desired_height)²
```

初始：

```yaml
equilibrium_weights:
  tau: 1.0
  joint: 0.1
  joint_limit_margin: 10.0
  height: 0.2
```

不要为“姿势好看”加入复杂 cost。

先求可行。

输出：

```text
data/equilibrium/rear_equilibrium.yaml
```

包含：

```yaml
base_pose:
joint_q:
joint_tau:
contact_forces:
com:
rear_axle:
solver_status:
constraint_residual:
```

---

# 22. Gate 2：Equilibrium

必须满足：

```text
IPOPT success
max dynamics residual < 1e-5 normalized
front wheel clearance >= 0.05 m
all joint limits satisfied
all |tau| <= 0.70 tau_limit
friction utilization <= 0.70
|x_com - x_axle| <= 5 mm
```

如果 Gate 2 不过：

只允许检查：

```text
joint limits
base pitch target
rear leg geometry
front clearance
model mapping
torque limits
```

不要开始站起轨迹。

---

# 23. 先做 final pose direct-spawn

写脚本：

```text
scripts/run_balance.sh
```

启动 simulator 时直接把 robot 放到：

```text
rear_equilibrium.yaml
```

姿态。

这时：

```text
leg joints = impedance hold
front wheels = airborne
rear wheels = torque controlled
```

---

# 24. Joint impedance 第一版

对 12 个 leg joints：

```text
q_des = q_eq
dq_des = 0
```

使用 LowCmd：

```text
kp
kd
tau_ff
```

初始 gain 不要凭感觉随便极高。

从：

```yaml
impedance:
  hip:
    kp: 30
    kd: 1.5
  thigh:
    kp: 35
    kd: 2.0
  calf:
    kp: 35
    kd: 2.0
```

开始。

这只是起始值。

允许小范围：

```text
Kp × [0.7, 1.0, 1.3]
Kd × [0.7, 1.0, 1.3]
```

禁止一次把 Kp 加到 100+ 来掩盖模型问题。

对于 wheel joints：

```text
kp = 0
```

BALANCE 中：

```text
dq_des = 0
kd = small damping or 0
tau_ff = LQR output
```

不要 wheel position hold。

---

# 25. LQR 状态定义固定

后轮两轮 final balance 的 canonical state：

\[
x =
\begin{bmatrix}
\theta-\theta_{eq} \\
\dot\theta \\
s-s_{eq} \\
\dot s
\end{bmatrix}
\]

其中：

```text
theta      = IMU body pitch
theta_dot  = IMU pitch rate
s          = rear wheel average displacement
s_dot      = rear wheel average linear velocity
```

定义：

```text
s = r * mean(canonical_forward_wheel_angle_RR,
             canonical_forward_wheel_angle_RL)
```

`wheel_forward_sign` 由 Gate 1 确定。

LQR 单输入：

```text
u = average rear wheel forward torque
```

最后：

```text
RR canonical wheel torque += u
RL canonical wheel torque += u
```

经过 sign mapping 后发到真实 motor index。

---

# 26. LQR 不手推简化模型，直接辨识实际 closed posture

实现：

```text
python/lqr/collect_identification_data.py
```

在：

```text
rear_equilibrium
+
固定 leg impedance
```

下收集数据。

输入 excitation：

```text
rear wheel average torque pulse
```

安全范围先用：

```text
±0.5 Nm
±1.0 Nm
±1.5 Nm
```

脉冲时长：

```text
20–80 ms
```

同时做小 initial pitch perturbation：

```text
±0.5°
±1°
±2°
```

收集：

```text
theta
theta_dot
s
s_dot
u
```

500 Hz。

---

# 27. Fit discrete linear model

实现：

```text
python/lqr/fit_linear_model.py
```

拟合：

\[
x_{k+1}=Ax_k+Bu_k
\]

使用 least squares。

分：

```text
train data
validation data
```

要求 validation one-step prediction：

```text
normalized RMSE < 10%
```

如果 >10%：

先检查：

```text
state sign
wheel sign
equilibrium
leg posture movement
time alignment
sample period
```

不要先调 Q/R。

---

# 28. LQR design

实现：

```text
python/lqr/design_lqr.py
```

状态先归一化：

```yaml
state_scale:
  pitch_rad: 0.1745        # 10 deg
  pitch_rate: 1.0
  wheel_displacement_m: 0.20
  wheel_velocity_mps: 1.0
```

归一化状态后初始：

```text
Q = diag([50, 5, 1, 2])
R = [0.2]
```

使用：

```python
scipy.linalg.solve_discrete_are
```

求 K。

wheel torque 第一版 saturation：

```text
0.60 * actual wheel torque limit
```

由 `robot.yaml` 自动读，不硬编码。

---

# 29. Balance controller runtime

`WheelLQR::update()`：

```cpp
LQRResult WheelLQR::update(const BalanceState& x) {
    Vector4d dx = normalize(x.value - x_eq_);
    double u = -(K_ * dx)(0);
    u = clamp(u, -u_limit_, u_limit_);
    return {u};
}
```

日志必须记录：

```text
pitch
pitch_rate
wheel_displacement
wheel_velocity
raw_u
clipped_u
```

---

# 30. Gate 3：双轮 Balance

在 direct-spawn final pose 下必须做到：

```text
nominal 20 s 不倒
```

然后测试：

```text
initial pitch ±1°
initial pitch ±3°
initial pitch ±5°
```

每档：

```text
20 episodes
```

Gate：

```text
±3° success >= 20/20
±5° success >= 19/20
20 s hold
wheel travel <= 0.5 m
continuous torque saturation < 100 ms
leg joint limit violation = 0
```

Gate 3 不过：

只允许检查：

```text
wheel sign
pitch sign
A/B identification
Q/R
equilibrium COM-to-axle
leg impedance
actuator saturation
sample time
```

**禁止开始 trajectory optimization。**

---

# 31. Capture basin

Balance 通过后，实现：

```text
python/lqr/scan_capture_basin.py
```

状态扫描：

```text
pitch error
pitch rate
wheel velocity
```

固定 wheel displacement：

```text
0
```

建议初始 grid：

```text
pitch error:
-12° ... +12°

pitch rate:
-2.0 ... +2.0 rad/s

wheel velocity:
-1.5 ... +1.5 m/s
```

每个初态只启用：

```text
leg impedance + LQR
```

运行：

```text
5 s
```

分类：

```text
RECOVERED
FELL
SATURATED
TIMEOUT
```

输出：

```text
data/capture_basin/capture_basin.csv
data/capture_basin/capture_gate.yaml
```

`capture_gate.yaml` 必须选择：

> 已验证成功区域内部的保守区域。

不要把理论 upright pose 当成 capture condition。

---

# 32. Trajectory optimization 的目标

现在才做从：

```text
four-wheel q_init
```

到：

```text
LQR capture basin
```

的动作。

注意：

> optimizer 的 terminal target 不是“看起来竖直”，而是“进入已经验证过的 LQR capture basin”。

---

# 33. RISE trajectory fixed setup

第一版固定：

```yaml
trajopt:
  duration: 1.50
  knots: 61
```

因此：

```text
dt = 1.50 / 60 = 0.025 s
```

不要第一版优化 duration。

不要第一版优化 knot count。

---

# 34. Fixed contact schedule

trajectory optimizer 中 contact schedule 预先给定。

不做 contact-implicit。

61 knot 第一版：

```text
k = 0 ... 18
    4-wheel contact
    RR RL FR FL

k = 19 ... 60
    rear-wheel contact only
    RR RL
```

即：

```text
release time ≈ 0.45 s
```

允许后续只扫：

```text
release_knot = 14 ... 24
```

但是 V1 第一次求解：

```text
release_knot = 18
```

固定。

---

# 35. Trajopt decision variables

Pinocchio floating base：

```text
q_k
v_k
```

每 knot：

```text
q_k
v_k
a_k
tau_k
f_contact_k
```

其中：

```text
tau_k ∈ R^16
```

接触力：

4-contact phase：

```text
f_FR
f_FL
f_RR
f_RL
```

2-contact phase：

```text
f_RR
f_RL
```

每个：

```text
R^3
```

---

# 36. Direct transcription dynamics

每 knot：

\[
M(q_k)a_k+h(q_k,v_k)
=
S^T\tau_k + J_c(q_k)^T f_k
\]

作为 hard equality。

velocity integration：

\[
v_{k+1}=v_k+a_k\Delta t
\]

configuration 使用 Pinocchio manifold integration：

\[
\hat q_{k+1}
=
integrate(q_k,\ v_k\Delta t+\frac12 a_k\Delta t^2)
\]

约束：

\[
difference(\hat q_{k+1},q_{k+1})=0
\]

不要直接对 quaternion 做普通加法。

---

# 37. Wheel rolling constraint

不要把 wheel 当 fixed foot。

平地 V1 对每个 support wheel 使用 wheel-center rolling formulation。

对于 wheel center：

```text
z_center = wheel_radius
```

velocity：

```text
v_center_normal = 0
v_center_lateral = 0
v_center_longitudinal = r * omega_wheel_canonical
```

即：

```text
no penetration
no lateral slip
pure longitudinal rolling
```

在 trajectory optimization 中作为 hard constraint。

不要强行：

```text
v_contact_xyz = 0
```

否则会错误锁死轮子。

---

# 38. Lifted front wheel constraints

k >= release_knot 后：

```text
FR
FL
```

不再创建 contact force。

同时 hard constrain：

```text
z_FR_center >= wheel_radius + clearance(t)
z_FL_center >= wheel_radius + clearance(t)
```

clearance ramp：

```text
release 后 2–3 knots：
>= r + 0.005

随后：
>= r + 0.02
```

不要一离地就硬要求 5 cm，避免 NLP 不必要困难。

---

# 39. Planar / symmetry hard constraints

V1 强制接近 sagittal motion。

## hip abduction

四个 hip abduction joint：

```text
固定在 q_init / symmetric nominal
```

trajectory optimizer 不优化。

## left-right sagittal symmetry

根据 joint mapping/sign：

```text
FR thigh == mirror(FL thigh)
FR calf  == mirror(FL calf)

RR thigh == mirror(RL thigh)
RR calf  == mirror(RL calf)
```

## wheels

```text
FR canonical wheel angle == FL canonical wheel angle
RR canonical wheel angle == RL canonical wheel angle
```

## base

```text
base_y = 0
base_roll = 0
base_yaw = 0
v_y = 0
roll_rate = 0
yaw_rate = 0
```

## contact force

```text
f_y = 0
left/right f_x equal
left/right f_z equal
```

这样 V1 从优化层面就不允许左右叉开。

---

# 40. Friction hard constraints

每个 support wheel：

```text
fz >= fz_min
fz <= fz_max

|fx| <= mu_eff * fz
|fy| <= mu_eff * fz
```

V1：

```text
mu_eff = 0.8 * model_mu
```

给 robustness margin。

正常 phase：

```text
fz_min = 0
```

PRELOAD 的最初若干 knots 可设置极小正值避免接触数值漂移：

```text
fz_min = 5 N
```

不要人工设很大的前轮最小力。

---

# 41. Torque / velocity hard constraints

使用 model limit。

trajectory optimization 用更保守：

```text
|tau| <= 0.80 * tau_limit
```

joint velocity：

```text
|dq| <= 0.80 * dq_limit
```

wheel speed 同样受实际 limit。

---

# 42. Joint limit margin

不是仅：

```text
q_min <= q <= q_max
```

而是：

```text
q_min + margin <= q <= q_max - margin
```

初始 margin：

```text
hip:   5 deg
thigh: 5 deg
calf:  5 deg
```

wheel continuous joint 不设置 angle limit。

---

# 43. Terminal constraint：进入 capture gate

最后 knot：

```text
front wheels airborne
rear wheels supporting
```

并要求：

```text
pitch
pitch_rate
rear wheel velocity
```

落在：

```text
capture_gate.yaml
```

的保守范围内。

wheel displacement 可以作为 soft cost，不一定硬限制为 0。

另外：

```text
q_leg_final
```

靠近：

```text
rear_equilibrium.yaml
```

---

# 44. Trajopt cost

所有 residual 先归一化，再加权。

建议 residual scale：

```yaml
scale:
  joint_rad: 0.5
  joint_vel: 2.0
  base_pos_m: 0.10
  base_angle_rad: 0.20
  base_vel: 1.0
  torque: use_tau_limit
  force: total_mass_times_g
  accel: 20.0
```

总 cost：

\[
J =
J_{terminal}
+J_{posture}
+J_{\tau}
+J_f
+J_a
+J_{\Delta \tau}
+J_{\Delta f}
\]

初始 dimensionless weights：

```yaml
cost:
  terminal_capture: 1000.0
  final_leg_pose: 200.0
  base_progress: 10.0
  joint_posture: 1.0
  torque: 0.02
  contact_force: 0.01
  acceleration: 0.02
  torque_smoothness: 0.5
  force_smoothness: 0.5
```

不要第一版加十几个 aesthetic cost。

---

# 45. Front-leg push 的处理

不靠“奖励”让前腿学会推。

优化器中前轮在：

```text
k = 0 ... 18
```

有真实 contact force decision variable。

动力学和 terminal capture condition 会要求它贡献必要 impulse。

另外记录：

\[
J_{front}
=
\sum_{k=0}^{18}
(f_{FR,z}+f_{FL,z})\Delta t
\]

和 pitch angular impulse。

如果最终 solution 前轮几乎没力但仍可行：

先检查它是否真的通过其他合法方式完成动作。

只有出现明显非目标机制时，才允许增加：

```text
front_force_reference
```

作为 soft cost。

不要一开始硬规定一个大前腿力。

---

# 46. Warm start

这个全新工程不依赖外部动作参考。

第一版 warm start 由以下三个姿态插值产生：

```text
q_init
q_mid
q_eq
```

其中：

```text
q_mid
```

由简单 IK / joint interpolation 构造：

```text
body pitch 朝 q_eq 方向移动约 50%
rear legs 逐渐伸展
front legs 保持地面可接触
```

warm start 只用于 IPOPT initial guess。

它不需要动力学可行。

但：

```text
solver final output
```

必须满足全部 dynamics constraint。

---

# 47. IPOPT 设置

`config/trajopt.yaml`：

```yaml
solver:
  name: ipopt
  max_iter: 3000
  tol: 1.0e-6
  acceptable_tol: 1.0e-4
  print_level: 5

  linear_solver: mumps

  warm_start: true
```

先用 MUMPS。

不要在 solver 未验证前安装多个商业/外部 linear solver。

---

# 48. Trajectory optimizer 输出

输出：

```text
data/trajectories/rear_rise_v001/
```

包含：

```text
metadata.yaml
trajectory.csv
contacts.csv
forces.csv
solver_stats.yaml
```

`trajectory.csv` 至少：

```text
time

base px py pz
base qw qx qy qz

base vx vy vz
base wx wy wz

16 joint q
16 joint dq
16 tau_ff
```

`contacts.csv`：

```text
time
FR
FL
RR
RL
```

`forces.csv`：

```text
time
FR_fx FR_fy FR_fz
FL_fx FL_fy FL_fz
RR_fx RR_fy RR_fz
RL_fx RL_fy RL_fz
```

---

# 49. Gate 4：trajectory solution

必须运行：

```text
python/trajopt/validate_solution.py
```

检查：

```text
max rigid-body dynamics residual
integration residual
rolling residual
friction utilization
joint limit margin
torque utilization
velocity utilization
front wheel clearance
terminal capture state
left-right symmetry
```

Gate：

```text
IPOPT status = success / acceptable
max normalized dynamics residual <= 1e-4
rolling residual <= 1e-4 normalized
max torque utilization <= 0.80
max friction utilization <= 0.80
joint limit violation = 0
front-wheel penetration after release = 0
terminal state inside conservative capture gate
```

Gate 4 不过：

只允许调：

```text
warm start
release_knot
duration [1.3, 1.5, 1.7]
solver scaling
equilibrium target
```

一次只改变一个因素。

不要改变控制架构。

---

# 50. Trajectory playback

写：

```text
Trajectory
TrajectoryPlayer
```

从离线文件加载。

runtime 500 Hz，使用 cubic Hermite interpolation：

输入 knot：

```text
q*
dq*
tau_ff*
```

500 Hz 查询：

```text
q_ref(t)
dq_ref(t)
tau_ff(t)
```

`tau_ff` 可线性插值。

contacts：

```text
piecewise constant
```

---

# 51. 第一轮 tracking：先不启用 WBC

这是一个 debug gate，不是最终架构。

第一次验证 trajectory 时：

```text
leg:
  q_des = q_ref
  dq_des = dq_ref
  kp/kd = impedance
  tau_ff = trajopt tau_ff

wheel:
  kp = 0
  tau_ff = trajopt wheel torque
```

运行：

```text
PREPARE
+
RISE playback
```

先看 trajectory 本身是否能把机器人送入 capture basin。

这一步禁止加入 WBC，是为了区分：

```text
trajectory failure
```

和：

```text
WBC failure
```

---

# 52. Playback Gate

固定 initial state。

执行 10 次。

由于仿真 deterministic：

```text
10/10 应该一致
```

要求至少：

```text
front wheels 正常 lift off
robot 不 lateral diverge
rear wheels 不严重 slip
terminal state 能进入 capture basin
```

如果不进入：

比较：

```text
desired vs actual q
desired vs actual base pitch
tau_ff saturation
front contact forces
rear contact forces
rolling error
```

允许：

```text
小幅 impedance gain sweep
```

但不要用超大 Kp 修复明显动力学错误。

---

# 53. WBC formulation

现在实现最终 WBC。

文件：

```text
include/.../control/wbc.hpp
src/control/wbc.cpp
```

500 Hz。

QP decision：

\[
z =
\begin{bmatrix}
\ddot q \\
f_c \\
\tau
\end{bmatrix}
\]

其中：

```text
ddq: floating-base generalized acceleration
f_c: current support wheel forces
tau: 16 actuator torques
```

---

# 54. WBC equality constraints

## 54.1 Rigid-body dynamics

\[
M(q)\ddot q+h(q,\dot q)
=
S^T\tau+J_c^T f_c
\]

hard equality。

## 54.2 Rolling contact acceleration

每个 support wheel：

```text
normal acceleration = stabilization term
lateral acceleration = stabilization term
longitudinal rolling acceleration relation = stabilization term
```

不要使用 3D fixed-foot acceleration constraint。

使用 Baumgarte-style small stabilization：

```text
a_ref = -Kp_c * position_error - Kd_c * velocity_error
```

但 rolling longitudinal 仍满足 wheel speed coupling。

---

# 55. WBC inequality constraints

hard：

```text
tau_min <= tau <= tau_max
fz >= 0
|fx| <= mu_eff * fz
|fy| <= mu_eff * fz
ddq bounds
```

使用和 trajopt 一样的：

```text
mu_eff
```

---

# 56. WBC tracking tasks

RISE 阶段 soft tasks：

```text
base orientation tracking
base angular velocity tracking
CoM tracking
leg joint posture tracking
leg joint velocity tracking
contact force tracking
tau_ff tracking
```

但因为 V1 强制左右对称，可保持配置简单。

建议优先级/权重：

```yaml
wbc:
  base_orientation: 100
  base_angular_velocity: 20
  com_position: 30
  com_velocity: 10
  joint_posture: 20
  joint_velocity: 5
  contact_force_ref: 1
  torque_ref: 0.5
  ddq_regularization: 0.01
  force_regularization: 0.001
  tau_regularization: 0.001
```

这是起始值。

不要同时 sweep 全部权重。

---

# 57. WBC reference acceleration

对于 task：

\[
a^*
=
a_{ff}
+
K_p e
+
K_d \dot e
\]

例如 joint：

```text
ddq_ref
+ Kp_joint_task(q_ref-q)
+ Kd_joint_task(dq_ref-dq)
```

WBC task gain 和 motor impedance gain 是两层不同的 gain。

二者都不要设极高。

---

# 58. WBC 在各 FSM phase 的职责

## PREPARE

contact：

```text
4 wheels
```

WBC：

```text
optional
```

第一版 PREPARE 可以直接 impedance interpolation，不强制 WBC。

## RISE

contact set 跟 trajectory file。

WBC：

```text
ON
```

## CAPTURE

rear contact only。

WBC：

```text
ON
```

reference 从 trajectory terminal blend 到 equilibrium。

LQR：

```text
ramp ON
```

## BALANCE

rear contact only。

WBC：

```text
ON
```

但：

```text
sagittal pitch task weight ≈ 0 or very small
```

LQR：

```text
full authority on rear wheel average torque
```

---

# 59. WBC + LQR torque arbitration

BALANCE 时：

先由 WBC 求：

```text
tau_wbc
```

然后：

```text
u_lqr
```

只加到 support rear wheels 的 canonical average torque。

示意：

```text
tau_RR_forward = tau_wbc_RR_forward + u_lqr + u_yaw_diff
tau_RL_forward = tau_wbc_RL_forward + u_lqr - u_yaw_diff
```

V1：

```text
u_yaw_diff = 0
```

因为 planar symmetry 已保证 yaw ≈ 0。

后续若必要，再加小 yaw PD。

不要在 V1 同时开发 yaw wheel controller。

---

# 60. ProxQP runtime 要求

WBC QP：

```text
warm start from previous solution
```

记录：

```text
solve_time_us
iterations
status
primal residual
dual residual
```

500 Hz：

```text
budget = 2 ms
```

Gate：

```text
p99 solve time < 1.5 ms
```

如果 >1.5 ms：

先：

```text
reuse matrices
avoid heap allocation
use sparse/dense appropriate backend
warm start
release build
```

不要第一反应降控制频率。

---

# 61. CommandMux

实现唯一 command composition point：

```text
src/control/command_mux.cpp
```

不允许 FSM state 各自直接写 LowCmd。

输入：

```text
trajectory reference
WBC tau
LQR tau
phase
safety state
```

输出唯一：

```text
MotorCommand
```

规则：

## leg joints

```text
q_des  = reference q
dq_des = reference dq
kp/kd  = phase gains
tau_ff = WBC/output feedforward
```

## wheel joints during RISE

```text
kp = 0
q_des ignored
tau_ff = WBC wheel tau
```

## support rear wheel during BALANCE

```text
kp = 0
tau_ff = WBC + LQR
```

## airborne front wheel during BALANCE

```text
q_des / dq_des = safe tucked pose
small kp/kd
tau_ff = posture feedforward if needed
```

---

# 62. SafetySupervisor

每个 500 Hz cycle 在 command send 前运行。

abort condition：

```text
invalid state
state timeout
NaN
joint outside hard limit
predicted torque > hard limit
base roll > 20 deg
unexpected yaw > 25 deg
base height unsafe
front/rear body collision
QP infeasible consecutive > threshold
LQR persistent saturation
```

abort 后：

```text
FSM -> PASSIVE/DAMPING
```

不要继续执行轨迹。

---

# 63. FSM 固定为 5 个代码状态

代码上保留：

```text
PASSIVE
PREPARE
RISE
CAPTURE
BALANCE
```

用户所说“四阶段”是：

```text
PREPARE
RISE
CAPTURE
BALANCE
```

`PASSIVE` 是安全状态，不算动作阶段。

---

# 64. PREPARE

目的：

```text
稳定进入 trajectory optimizer 的 q_init
```

contact：

```text
4 wheels
```

轮子：

```text
不主动跑
```

实现：

```text
0.3–0.5 s quintic interpolation
```

从当前合法四轮姿态到：

```text
trajopt q_init
```

退出条件：

```text
joint RMS error < 0.03 rad
body roll < 3 deg
body yaw < 3 deg
all 4 contacts valid
state stable 100 ms
```

timeout：

```text
1.0 s
```

失败：

```text
PASSIVE
```

---

# 65. RISE

目的：

```text
执行离线动力学可行起身轨迹
```

source：

```text
rear_rise_vxxx/trajectory.csv
```

controller：

```text
TrajectoryPlayer + WBC + impedance
```

contact set：

```text
由 trajectory contacts.csv
```

退出条件不是纯 timer。

要求：

```text
trajectory time >= terminal
AND
front contacts released
AND
state inside capture_gate
```

如果轨迹结束但未进入 capture gate：

允许短暂：

```text
max 150 ms
```

terminal hold。

仍未进入：

```text
PASSIVE
```

不要硬切 BALANCE。

---

# 66. CAPTURE

目的：

```text
平滑把 sagittal authority 从 trajectory/WBC 转给 LQR
```

时长：

```text
0.15–0.30 s
```

默认：

```text
0.20 s
```

blend：

\[
\alpha(t)
=
smoothstep(0\rightarrow1)
\]

rear wheel：

```text
tau =
tau_WBC
+
alpha * tau_LQR
```

同时：

```text
trajectory terminal leg reference
→
rear equilibrium leg reference
```

WBC pitch task weight：

```text
从 RISE 权重
→
BALANCE 小权重
```

退出：

```text
alpha = 1
AND
state remains inside balance safe region for 100 ms
```

否则：

```text
PASSIVE
```

---

# 67. BALANCE

contact：

```text
RR + RL
```

目标：

```text
rear_equilibrium
```

leg：

```text
WBC + impedance
```

rear wheels：

```text
LQR average torque
+
WBC residual/contact torque
```

front legs：

```text
固定 safe airborne pose
```

退出：

```text
用户停止
或 safety abort
```

---

# 68. Contact transition

trajectory 内 release knot 前后必须做 force ramp。

在：

```text
release_knot - 3
...
release_knot
```

给前轮 force reference 从当前：

```text
→ 0
```

WBC contact set 真正从 4C 切到 2C 时：

必须满足至少一个：

```text
front planned Fz 已接近 0
front actual Fz 已接近 0
front wheel normal relative velocity >= safe lift-off condition
```

不要在前轮还有大 normal force 时直接删 contact constraint。

---

# 69. WBC Gate

单独 playback：

```text
same trajectory
```

比较：

```text
feedforward+impedance
```

与：

```text
WBC+impedance
```

要求 WBC：

```text
QP infeasible = 0
p99 solve < 1.5 ms
joint tracking RMS 不恶化
base pitch tracking RMS 不恶化
friction utilization 不增加到 >0.9
torque clipping 不增加
```

否则 WBC 不进入 full FSM。

---

# 70. Gate 5：完整 FSM nominal

运行：

```bash
./scripts/run_full_demo.sh
```

一轮：

```text
reset
PREPARE
RISE
CAPTURE
BALANCE 20 s
```

成功条件：

```text
达到 BALANCE
保持 20 s
无 safety abort
无 joint limit violation
无 QP infeasible
无 persistent saturation
front wheels 不触地
```

连续：

```text
20/20
```

才通过。

---

# 71. 日志 schema

每个 run 写：

```text
artifacts/logs/<timestamp>/
```

包含：

```text
config_snapshot.yaml
git_hash.txt
third_party.lock
run.csv
events.csv
metrics.json
```

`run.csv` 500 Hz 至少记录：

```text
time
fsm_state

base quaternion
roll pitch yaw
imu gyro

16 q
16 dq

16 q_ref
16 dq_ref

16 tau_ff
16 tau_cmd_est
16 tau_limit_ratio

4 contact state
4 contact fx/fy/fz

CoM xyz

wheel canonical angle
wheel canonical velocity

lqr x[4]
lqr raw u
lqr clipped u

wbc status
wbc solve time
wbc primal residual
wbc dual residual

trajectory time
trajectory contact mode
```

---

# 72. 必做 plots

`python/analysis/make_report.py` 自动生成：

```text
01_fsm.png
02_pitch.png
03_pitch_rate.png
04_joint_tracking.png
05_torque.png
06_contact_force.png
07_friction_utilization.png
08_wheel_speed.png
09_lqr_state.png
10_lqr_input.png
11_wbc_solve_time.png
12_com_rear_axle.png
```

不要依靠“看视频感觉”。

---

# 73. metrics.json

每轮自动算：

```json
{
  "success": true,
  "balance_duration_s": 20.0,
  "max_roll_deg": 0,
  "max_yaw_deg": 0,
  "max_pitch_error_deg": 0,
  "joint_rms_error_rad": 0,
  "max_torque_utilization": 0,
  "max_friction_utilization": 0,
  "qp_infeasible_count": 0,
  "wbc_p99_ms": 0,
  "lqr_saturation_ms": 0,
  "front_contact_after_capture": false
}
```

---

# 74. 自动测试

## test_model_load.py

Pass：

```text
Go2W MJCF loads in MuJoCo
Go2W MJCF loads in Pinocchio
16 actuators identified
4 wheels identified
```

## test_joint_map.py

Pass：

```text
16 unique mapping
no duplicate
expected SDK index sequence
```

## test_dynamics_consistency.py

Pass：

```text
gravity torque relative error < 3%
```

## test_equilibrium.py

Pass：

```text
all Gate 2 constraints
```

## test_trajectory_schema.py

Pass：

```text
all required fields
monotonic time
61 knots
no NaN
```

## test_lqr_config.cpp

Pass：

```text
K dimension correct
sign smoke test
saturation
```

## test_wbc_qp.cpp

在 equilibrium state：

```text
QP feasible
```

并：

```text
dynamics residual < tolerance
```

## test_fsm.cpp

测试所有合法 transition：

```text
PASSIVE -> PREPARE
PREPARE -> RISE
RISE -> CAPTURE
CAPTURE -> BALANCE
ANY -> PASSIVE
```

非法 transition 必须拒绝。

---

# 75. 定向故障诊断

Agent 不允许“看到动作不好就整体调参”。

按下面诊断。

---

## 75.1 左右腿叉开

先检查：

```text
hip abduction 是否在 trajopt hard-fixed
left/right symmetry constraint 是否生效
joint sign mapping 是否正确
WBC roll/yaw task 是否异常
左右 contact force 是否对称
```

V1 中只要 trajopt symmetry hard constraint 正确：

> 优化器本身不应该产生左右 splay。

如果实际 tracking splay：

优先定位：

```text
mapping
WBC
motor sign
```

而不是改 trajectory cost。

---

## 75.2 后腿过度弯曲

看：

```text
q_ref 本身是否弯
还是 q_ref 正常、tracking 后变弯
```

若 q_ref 弯：

检查：

```text
equilibrium pose
joint margin
terminal leg pose weight
torque limit
COM-to-axle constraint
```

若 q_ref 正常但 tracking 弯：

检查：

```text
tau saturation
calf motor limit
WBC q task
impedance gain
model mismatch
```

不要先把 calf Kp 翻倍。

---

## 75.3 前腿没有有效推力

查看：

```text
planned front Fz
actual front Fz
front impulse
pitch angular acceleration
```

如果 planned front Fz 就接近 0：

这是 optimizer mechanism。

检查：

```text
release schedule
terminal capture target
warm start
front geometry
```

如果 planned 有力、actual 没有：

是 tracking/contact 问题。

检查：

```text
front wheel height
contact mode switch
tau saturation
wheel rolling constraint
WBC force tracking
```

---

## 75.4 前轮释放时弹跳

检查：

```text
release 前 planned Fz
release 前 actual Fz
normal velocity
contact constraint 删除时间
```

修复优先：

```text
force ramp
contact switch gate
trajectory release knot
```

不是加 damping 掩盖。

---

## 75.5 后轮打滑

看：

```text
|fx| / (mu*fz)
rolling velocity residual
wheel torque saturation
```

如果 optimizer 本身：

```text
friction utilization >0.8
```

trajectory Gate 就应该失败。

如果 optimizer <0.8，但 sim 打滑：

检查：

```text
MuJoCo friction
wheel radius
contact geometry
WBC wheel sign
tracking overshoot
```

---

## 75.6 进入 CAPTURE 立刻倒

优先检查：

```text
terminal state 是否真的在 capture_gate
LQR state sign
blend 是否平滑
WBC pitch 是否与 LQR 对打
rear wheel torque sign
```

不要先改 stand-up trajectory。

---

# 76. 实施 commit 顺序

严格按顺序。

## Commit 1

```text
chore: bootstrap project and pin Unitree dependencies
```

## Commit 2

```text
feat(model): add canonical Go2W model audit and joint mapping
```

## Commit 3

```text
feat(io): add Unitree LowCmd/LowState backend and smoke tests
```

## Commit 4

```text
test(model): add MuJoCo-Pinocchio dynamics consistency checks
```

## Commit 5

```text
feat(balance): solve rear-wheel equilibrium pose
```

## Commit 6

```text
feat(balance): add rear-wheel plant identification and LQR
```

## Commit 7

```text
test(balance): add capture-basin scan and balance regression
```

## Commit 8

```text
feat(trajopt): add fixed-contact 61-knot rise optimizer
```

## Commit 9

```text
test(trajopt): add dynamics/contact/torque feasibility validation
```

## Commit 10

```text
feat(trajectory): add deterministic trajectory playback
```

## Commit 11

```text
feat(wbc): add Pinocchio-ProxQP inverse-dynamics WBC
```

## Commit 12

```text
feat(fsm): add prepare-rise-capture-balance state machine
```

## Commit 13

```text
test(system): add 20-run nominal standup regression
```

## Commit 14

```text
feat(report): add automatic controller diagnostics report
```

---

# 77. Phase 0 — Bootstrap

实现：

```text
repo
submodules
environment
CMake
Python package
scripts
```

命令：

```bash
./scripts/bootstrap.sh
./scripts/build.sh
```

输出：

```text
build success
python imports success
```

Gate：

```text
Gate 0
```

未通过不要继续。

---

# 78. Phase 1 — Model + IO

实现：

```text
RobotState
MotorCommand
RobotBackend
joint mapping
model audit
smoke test
```

命令：

```bash
./scripts/run_sim.sh
./scripts/run_io_smoke_test.sh
pytest tests/test_model_load.py
pytest tests/test_joint_map.py
pytest tests/test_dynamics_consistency.py
```

Gate：

```text
Gate 1
```

---

# 79. Phase 2 — Final equilibrium

实现：

```text
solve_rear_equilibrium.py
validate_equilibrium.py
```

命令：

```bash
./scripts/solve_equilibrium.sh
```

输出：

```text
data/equilibrium/rear_equilibrium.yaml
```

Gate：

```text
Gate 2
```

---

# 80. Phase 3 — Two-wheel LQR

实现：

```text
direct-spawn equilibrium
identification
fit A/B
LQR
capture basin
C++ runtime
```

命令：

```bash
python python/lqr/collect_identification_data.py
python python/lqr/fit_linear_model.py
python python/lqr/design_lqr.py
python python/lqr/scan_capture_basin.py
./scripts/run_balance.sh
```

Gate：

```text
Gate 3
```

---

# 81. Phase 4 — Fixed-contact trajectory optimization

实现：

```text
CasADi Pinocchio dynamics
fixed contact schedule
61 knot OCP
validation
export
```

命令：

```bash
./scripts/solve_trajopt.sh
```

输出：

```text
data/trajectories/rear_rise_v001/
```

Gate：

```text
Gate 4
```

---

# 82. Phase 5 — Feedforward playback

实现：

```text
Trajectory
TrajectoryPlayer
joint impedance
```

先不开 WBC。

命令：

```bash
./scripts/run_full_demo.sh --stop-after rise --no-wbc
```

要求：

```text
能稳定接近 capture basin
```

失败只排：

```text
trajectory
tau_ff
impedance
contact timing
model
```

---

# 83. Phase 6 — WBC

实现：

```text
Pinocchio model
contact Jacobian
rolling constraints
inverse dynamics QP
ProxQP
```

先 standalone：

```text
equilibrium
```

再：

```text
trajectory playback
```

不要一写完就接 FSM。

通过 WBC Gate 后再继续。

---

# 84. Phase 7 — FSM integration

实现：

```text
PASSIVE
PREPARE
RISE
CAPTURE
BALANCE
```

加入：

```text
SafetySupervisor
CommandMux
logging
```

命令：

```bash
./scripts/run_full_demo.sh
```

先成功 1 次。

然后：

```text
5/5
10/10
20/20
```

---

# 85. Phase 8 — Nominal regression

只在 full system 通过后做。

固定：

```text
same model
same seed
same config
```

20 次。

CI / local test：

```bash
python python/tools/run_batch.py \
  --episodes 20 \
  --config config/control.yaml
```

自动失败条件：

```text
success rate < 100%
QP infeasible > 0
joint violation > 0
NaN
persistent torque clipping
```

---

# 86. Phase 9 — 小范围鲁棒性

这不是 V1 nominal Gate 的前置条件。

完成 nominal 后才跑：

```text
initial pitch ±3°
mass ±5%
CoM x ±5 mm
friction ±10%
motor strength ±5%
control latency 0–2 ms
```

一次只 sweep 一个维度。

目标：

```text
success >= 90%
```

如果失败：

定位具体 sensitivity。

不要立即引入新的控制框架。

---

# 87. 最快看到结果的执行顺序

如果 Agent 想最快证明路线正确：

## 第 1 步

官方 Go2W MuJoCo 跑起来。

## 第 2 步

求 rear equilibrium。

## 第 3 步

直接 spawn 到 equilibrium。

## 第 4 步

把 LQR 做到：

```text
±5° recovery
```

到这里已经验证：

> final two-wheel stand 是闭环可控的。

## 第 5 步

61-knot fixed-contact OCP。

## 第 6 步

先：

```text
tau_ff + joint impedance
```

播放。

## 第 7 步

加 WBC。

## 第 8 步

连接 FSM CAPTURE。

不要反过来先写完整 FSM/WBC/优化器，再第一次运行。

---

# 88. AGENTS.md 必须写入的约束

Agent 在项目初始化时，把以下内容复制到：

```text
AGENTS.md
```

```text
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
```

---

# 89. Definition of Done

## Infrastructure

- [ ] 新 repo 独立创建
- [ ] `unitree_mujoco` submodule pinned
- [ ] `unitree_sdk2 v2.0.2` pinned
- [ ] environment lock 保存
- [ ] C++ Release build
- [ ] Python package 可复现

## Model

- [ ] official `go2w.xml` 为唯一 source of truth
- [ ] 16 joint identity verified
- [ ] 16 joint sign verified
- [ ] 4 wheel forward sign verified
- [ ] MuJoCo/Pinocchio gravity torque error <3%
- [ ] wheel radius / torque / friction 从模型读取

## Balance

- [ ] rear equilibrium solved
- [ ] front wheel clearance >=5 cm
- [ ] CoM aligned rear axle
- [ ] direct-spawn 20 s balance
- [ ] ±3° 20/20
- [ ] ±5° >=19/20
- [ ] capture basin saved

## Trajectory optimization

- [ ] 61 knots
- [ ] 1.50 s
- [ ] fixed release knot
- [ ] fixed contact schedule
- [ ] rolling constraints
- [ ] friction constraints
- [ ] torque constraints
- [ ] joint-limit margin
- [ ] left-right symmetry hard constraints
- [ ] terminal inside capture basin
- [ ] all validation passed

## Tracking

- [ ] feedforward+impedance playback tested
- [ ] WBC QP feasible
- [ ] WBC p99 <1.5 ms
- [ ] no persistent torque saturation
- [ ] no lateral splay
- [ ] clean front-wheel lift-off

## FSM

- [ ] PASSIVE
- [ ] PREPARE
- [ ] RISE
- [ ] CAPTURE
- [ ] BALANCE
- [ ] abort works
- [ ] capture blend works

## Final

- [ ] 四轮初态自动进入站起
- [ ] 双轮 balance >=20s
- [ ] nominal 20/20
- [ ] 自动 report
- [ ] 一条命令可复现 demo

最终命令：

```bash
./scripts/run_full_demo.sh
```

---

# 90. Agent 遇到问题时的决策树

```text
官方 Go2W 都跑不起来？
│
├─ 是 → 只修环境 / submodule / SDK2 / MuJoCo
│       不写控制器
│
└─ 否
    │
    ├─ joint mapping / dynamics 不一致？
    │   ├─ 是 → 修 mapping/model convention
    │   └─ 否
    │       │
    │       ├─ direct-spawn 双轮平衡不了？
    │       │   ├─ 是 → 只修 equilibrium / LQR / sign / gains
    │       │   └─ 否
    │       │       │
    │       │       ├─ trajectory optimizer 不可行？
    │       │       │   ├─ 是 → warm-start / release knot / duration / scaling
    │       │       │   └─ 否
    │       │       │       │
    │       │       │       ├─ tau_ff+impedance 跟不住？
    │       │       │       │   ├─ 是 → model / contact / saturation / gains
    │       │       │       │   └─ 否
    │       │       │       │       │
    │       │       │       │       ├─ WBC 加入后变差？
    │       │       │       │       │   ├─ 是 → QP/task/contact formulation
    │       │       │       │       │   └─ 否
    │       │       │       │       │       │
    │       │       │       │       │       ├─ CAPTURE 失败？
    │       │       │       │       │       │   ├─ 是 → terminal/capture gate/blend
    │       │       │       │       │       │   └─ 否 → 做 20/20 regression
```

---

# 91. 验证过的开源落点

以下是本计划要求 Agent 优先复用的项目。

## Unitree MuJoCo

```text
https://github.com/unitreerobotics/unitree_mujoco
```

直接复用：

```text
Go2W MJCF
MuJoCo simulator
LowCmd/LowState bridge
sim-to-real low-level semantics
```

## Unitree SDK2

```text
https://github.com/unitreerobotics/unitree_sdk2
```

直接复用：

```text
DDS
LowCmd
LowState
motor command semantics
```

其官方 Go2 low-level example 使用：

```text
dt = 0.002 s
```

且直接设置：

```text
q
dq
kp
kd
tau
```

这与本项目 500 Hz command layer 对齐。

## Unitree RL Lab

```text
https://github.com/unitreerobotics/unitree_rl_lab
```

只参考：

```text
Go2W joint SDK order
Go2W deploy FSM organization
```

不要引入训练栈。

## Pinocchio

```text
https://github.com/stack-of-tasks/pinocchio
```

直接复用：

```text
MJCF parser
rigid-body dynamics
contact Jacobians
CasADi scalar support
```

## CasADi

```text
https://github.com/casadi/casadi
```

直接复用：

```text
NLP
automatic differentiation
IPOPT interface
```

## ProxSuite

```text
https://github.com/Simple-Robotics/proxsuite
```

直接复用：

```text
ProxQP
warm-started runtime QP
```

---

# 92. 项目完成时 README 首页只需要展示

```text
Go2W Rear-Wheel Stand-Up Controller
===================================

Architecture:
PREPARE
→ fixed-contact optimized RISE
→ LQR CAPTURE
→ rear-wheel BALANCE

Runtime:
500 Hz WBC
500 Hz joint impedance
500 Hz rear-wheel LQR

Simulation:
official Unitree Go2W MuJoCo model

Quick start:
./scripts/bootstrap.sh
./scripts/build.sh
./scripts/run_full_demo.sh

V1 result:
20/20 nominal stand-up
20 s rear-wheel balance
```

---

# 93. 最终执行原则

这个项目的实施顺序不能变成：

```text
先把所有东西写完
→
最后统一调参
```

正确顺序固定为：

```text
官方模型/IO
→
模型一致性
→
final equilibrium
→
direct-spawn LQR balance
→
capture basin
→
fixed-contact trajectory optimization
→
feedforward playback
→
WBC
→
FSM capture
→
20/20 regression
```

每一步都能单独证明或否定一个模块。

只要严格按照这个顺序执行，Agent 遇到失败时必须知道问题属于：

```text
model
balance
trajectory
tracking
WBC
capture
```

中的哪一层，而不是无边界地修改整个系统。

**V1 完成之前，不改变这套架构。**
