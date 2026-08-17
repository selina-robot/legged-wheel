"""Closed-posture rear-wheel balance plant (spec §23-§26).

Headless MuJoCo simulation of the Go2W at the rear-wheel equilibrium with
  * 12 leg joints under joint impedance hold (q_des = q_eq, dq_des = 0,
    kp/kd from config/control.yaml, tau_ff = tau_eq feedforward),
  * front wheels free (kp=0, kd=0, tau=0; never position hold, spec §24),
  * rear wheels torque-controlled by the balance input u (average rear wheel
    forward torque; identical motor torque on both since
    wheel_forward_sign = +1, Gate 1).

This mirrors the C++ runtime (src/control/joint_impedance.cpp +
src/control/wheel_lqr.cpp). Used for identification data collection, LQR
validation, Gate 3 regression, and capture-basin scanning. NOT the final
performance evidence (spec §7): Gate 3 nominal is re-run on the official
C++ simulator chain.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_control_config, load_robot_config, repo_path  # noqa: E402
from common.model import (  # noqa: E402
    actuator_order_names,
    joint_order_names,
    load_mujoco,
    permutation,
)

EQ_YAML = repo_path("data/equilibrium/rear_equilibrium.yaml")

# Gate 3 thresholds (spec §30)
FALL_PITCH_ERR_RAD = 0.35     # |theta - theta_eq| beyond this = fell
WHEEL_TRAVEL_MAX_M = 0.5      # wheel travel limit
SAT_CONTINUOUS_MAX_S = 0.1    # continuous saturation limit


class BalancePlant:
    """One closed-loop instance: model + data + controller state."""

    def __init__(self, lqr_cfg: dict | None = None, eq_path: str | None = None):
        self.mjm, self.mjd = load_mujoco(with_scene=True)
        self.dt = float(self.mjm.opt.timestep)  # 0.002 s (scene); control == physics rate
        self.eq_yaml = Path(eq_path) if eq_path else EQ_YAML

        robot_cfg = load_robot_config()
        ctrl_cfg = load_control_config()
        self.wheel_radius = float(robot_cfg["model_params"]["wheel_radius"])
        self.wheel_tau_limit = float(robot_cfg["model_params"]["torque_limit"]["wheel"])
        self.wheel_sign = {leg: float(robot_cfg["wheel_forward_sign"][leg])
                           for leg in ("FR", "FL", "RR", "RL")}
        assert all(s == 1.0 for s in self.wheel_sign.values())

        imp = ctrl_cfg["impedance"]

        def gain(name, key):
            t = ("hip" if "hip" in name else "thigh" if "thigh" in name
                 else "calf" if "calf" in name else "wheel")
            return 0.0 if t == "wheel" else float(imp[t][key])

        eq = yaml.safe_load(open(self.eq_yaml))
        sdk_names = eq["joint_q"]["sdk_order"]
        q_sdk = np.array(eq["joint_q"]["values_sdk"])
        tau_sdk = np.array(eq["joint_tau"]["values_sdk"])
        mj_joints = joint_order_names(self.mjm)
        act_joints = actuator_order_names(self.mjm)
        # joint-space (declaration order) and actuator-order conversions
        perm_sdk_to_joint = permutation(sdk_names, mj_joints)
        perm_sdk_to_act = permutation(sdk_names, act_joints)
        self.q_eq_joint = q_sdk[perm_sdk_to_joint]
        self.tau_eq_joint = tau_sdk[perm_sdk_to_joint]
        # leg impedance gains in joint/declaration order (wheels: zero)
        self.kp_joint = np.array([gain(n, "kp") for n in mj_joints])
        self.kd_joint = np.array([gain(n, "kd") for n in mj_joints])
        self.perm_joint_to_act = permutation(mj_joints, act_joints)
        self.tau_eq_act = tau_sdk[perm_sdk_to_act]
        self.q_eq_act = self.q_eq_joint[self.perm_joint_to_act]
        self.is_wheel_act = np.array(["wheel" in n for n in act_joints])
        self.is_leg_act = ~self.is_wheel_act
        self.eq_base_z = float(eq["base_pose"]["z"])

        # LQR state anchors
        bp = eq["base_pose"]
        self.theta_eq = float(bp["pitch"])
        self.s_eq = 0.0  # s is measured relative to the spawn wheel angles

        # joint indices in qpos/qvel
        self.joint_qadr = np.array([
            self.mjm.jnt_qposadr[mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in mj_joints
        ])
        self.joint_vadr = np.array([
            self.mjm.jnt_dofadr[mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in mj_joints
        ])
        self.rr_act = act_joints.index("RR_wheel_joint")
        self.rl_act = act_joints.index("RL_wheel_joint")
        self.rr_j = mj_joints.index("RR_wheel_joint")
        self.rl_j = mj_joints.index("RL_wheel_joint")

        # LQR (filled by set_lqr from config/lqr.yaml)
        self.K = None
        self.u_limit = None
        if lqr_cfg is not None:
            self.set_lqr(lqr_cfg)

    def set_lqr(self, lqr_cfg: dict):
        self.K = np.array(lqr_cfg["K"], dtype=float).reshape(1, 4)
        x_eq = np.array(lqr_cfg["x_eq"], dtype=float)
        assert np.isclose(x_eq[1], 0.0) and np.isclose(x_eq[3], 0.0)
        self.theta_eq = float(x_eq[0])
        self.s_eq = float(x_eq[2])
        self.u_limit = float(lqr_cfg["u_limit"])

    # ---- state ----
    def base_pitch(self):
        # ZYX pitch via atan2(-R[2,0], R[0,0]): smooth through +/-90 deg,
        # unlike asin(2(wy-zx)) which folds exactly at the equilibrium
        # pitch (-90 deg, base vertical). Diagnosed from identification data
        # (pitch derivative sign flipped at the asin fold).
        w, x, y, z = self.mjd.qpos[3:7]
        r20 = 2 * (x * z - w * y)   # R[2,0]
        r00 = 1 - 2 * (y * y + z * z)  # R[0,0]
        return float(np.arctan2(-r20, r00))

    def state(self):
        """Canonical LQR state x (spec §25): [theta, theta_dot, s, s_dot]."""
        theta = self.base_pitch()
        theta_dot = float(self.mjd.qvel[4])  # base angular velocity, local y
        q = self.mjd.qpos[self.joint_qadr]
        dq = self.mjd.qvel[self.joint_vadr]
        s = self.wheel_radius * 0.5 * (q[self.rr_j] + q[self.rl_j])
        s_dot = self.wheel_radius * 0.5 * (dq[self.rr_j] + dq[self.rl_j])
        return np.array([theta, theta_dot, s, s_dot])

    def lqr_error(self):
        x = self.state()
        return x - np.array([self.theta_eq, 0.0, self.s_eq, 0.0])

    # ---- control ----
    def lqr_u(self):
        """u = -K (x - x_eq), saturated at u_limit (spec §28/§29)."""
        dx = self.lqr_error()
        raw = float(-(self.K @ dx)[0])
        return raw, float(np.clip(raw, -self.u_limit, self.u_limit))

    def step(self, u_wheel: float | None):
        """One control==physics step. u_wheel=None means no LQR (free wheels)."""
        d = self.mjd
        q = d.qpos[self.joint_qadr]
        dq = d.qvel[self.joint_vadr]
        # leg impedance hold with equilibrium feedforward (spec §24), computed
        # in joint/declaration order, then permuted to actuator order
        tau_joint = (self.kp_joint * (self.q_eq_joint - q)
                     - self.kd_joint * dq + self.tau_eq_joint)
        d.ctrl[:] = 0.0
        d.ctrl[:] = tau_joint[self.perm_joint_to_act]
        if u_wheel is not None:
            d.ctrl[self.rr_act] = u_wheel  # sign = +1 both (Gate 1)
            d.ctrl[self.rl_act] = u_wheel
        mujoco.mj_step(self.mjm, d)

    # ---- episode ----
    def reset(self, pitch_offset_rad=0.0, wheel_vel=0.0, pitch_rate=0.0):
        """Direct-spawn at the equilibrium, optionally perturbed."""
        eq_qpos = np.zeros(self.mjm.nq)
        pitch = self.theta_eq + pitch_offset_rad
        eq_qpos[:3] = [0.0, 0.0, self.eq_base_z]
        eq_qpos[3:7] = [np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0]
        eq_qpos[self.joint_qadr] = self.q_eq_joint
        self.mjd.qpos[:] = eq_qpos
        self.mjd.qvel[:] = 0.0
        if pitch_rate != 0.0:
            self.mjd.qvel[4] = pitch_rate
        if wheel_vel != 0.0:
            dq = wheel_vel / self.wheel_radius
            self.mjd.qvel[self.joint_vadr[self.rr_j]] = dq
            self.mjd.qvel[self.joint_vadr[self.rl_j]] = dq
        mujoco.mj_forward(self.mjm, self.mjd)

    def is_fallen(self):
        x = self.state()
        return (abs(x[0] - self.theta_eq) > FALL_PITCH_ERR_RAD
                or self.mjd.qpos[2] < 0.25)


def run_episode(plant: BalancePlant, duration_s: float, use_lqr: bool = True,
                u_override=None, log_rate_hz: int = 500,
                limit_ranges: dict | None = None):
    """Run one closed-loop episode; returns a list of row dicts at log_rate_hz.

    u_override: callable(t_sec) -> u, replaces the LQR output (identification).
    limit_ranges: optional dict joint name -> (lo, hi) (declaration order
    names); if given, out-of-range joint positions are counted online and
    reported as rows[-1]["limit_violations"] (spec §30: 0 allowed).
    """
    rows = []
    n_steps = int(duration_s / plant.dt)
    log_every = max(1, int(1.0 / (log_rate_hz * plant.dt)))
    sat_start = None
    sat_continuous_max = 0.0
    s0 = None
    limit_violations = 0
    limit_idx = []
    if limit_ranges is not None:
        from common.model import joint_order_names
        names = joint_order_names(plant.mjm)
        limit_idx = [(i, *limit_ranges[n]) for i, n in enumerate(names)
                     if n in limit_ranges]
    for k in range(n_steps):
        t = k * plant.dt
        x = plant.state()  # pre-step state: pairs (x_k, u_k) -> next row's x
        if s0 is None:
            s0 = x[2]
        raw_u, u = (0.0, 0.0)
        if use_lqr:
            raw_u, u = plant.lqr_u()
        if u_override is not None:
            raw_u = u_override(t)
            u = float(np.clip(raw_u, -plant.u_limit, plant.u_limit)
                      if plant.u_limit else raw_u)
        if abs(u - raw_u) > 1e-12 and raw_u != 0.0:
            sat_start = t if sat_start is None else sat_start
            sat_continuous_max = max(sat_continuous_max, t - sat_start)
        else:
            sat_start = None
        if k % log_every == 0:
            rows.append({
                "t": t, "pitch": x[0], "pitch_rate": x[1],
                "wheel_disp": x[2], "wheel_vel": x[3],
                "raw_u": raw_u, "u": u,
                "pitch_err": x[0] - plant.theta_eq,
                "pitch_rate_abs": abs(x[1]),
                "wheel_travel": x[2] - s0,
            })
        plant.step(u if (use_lqr or u_override is not None) else None)
        if limit_idx:
            q = plant.mjd.qpos[plant.joint_qadr]
            for i, lo, hi in limit_idx:
                if q[i] < lo - 1e-6 or q[i] > hi + 1e-6:
                    limit_violations += 1
        if plant.is_fallen():
            break
    for r in rows:
        r["sat_continuous_max"] = sat_continuous_max
        r["limit_violations"] = limit_violations
    return rows
