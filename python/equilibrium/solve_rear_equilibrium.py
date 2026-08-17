#!/usr/bin/env python3
"""Rear-wheel equilibrium optimization (spec §18-§22, Gate 2).

Finds q_eq, tau_eq, f_RR, f_RL for the rear-wheel two-wheel stand using
Pinocchio (CasADi scalar) + CasADi + IPOPT (MUMPS).

Formulation note: spec §19 lists tau_leg as decision variables, but the 16
joint rows of the statics equation determine tau uniquely from (q, f), so
tau is derived algebraically (tau = g_joints - J_joints^T f) and its bounds
are imposed as inequality constraints. This keeps the NLP well-posed (the
base rows of statics are the only non-trivial equilibrium equations).
Left-right symmetry (spec §20.4) is imposed by construction: one shared
(pitch-only) base, mirrored leg joints, and a single shared rear-axle force
f = f_RR = f_RL.

Hard constraints (spec §20):
  20.1  z of each rear wheel center == wheel_radius (symmetric -> one row)
  20.2  z of each front wheel center >= wheel_radius + 0.05
  20.3  base roll = yaw = 0 (exact, via pitch-only base parameterization)
  20.4  hip abduction fixed to hip_abduction_nominal; thigh/calf mirrored
  20.5  statics: g(q) = S^T tau + J_RR^T f_RR + J_RL^T f_RL
        (point forces at the wheel-bottom contact points)
  20.6  fz >= 0, |fx| <= mu*fz, fy = 0
  20.7  |tau| <= 0.70 * tau_limit (MJCF ctrlrange from the model audit)
  20.8  |x_com - x_axle| <= 0.005 m

Wheel angles are fixed to 0 (a circular wheel's angle does not change rigid
geometry, spec §19); front wheel torques are exactly 0 (their statics row is
identically zero: the wheel CG lies on the axle).

Cost (spec §21, initial weights; feasibility first):
  1.0*||tau/tau_limit||^2 + 0.1*||q-q_nominal||^2 + 10.0*joint_limit_barrier
  + 0.2*(base_height - 0.40)^2

Output: data/equilibrium/rear_equilibrium.yaml

Usage: python python/equilibrium/solve_rear_equilibrium.py
"""
import sys
from pathlib import Path

import casadi as ca
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, load_yaml, repo_path  # noqa: E402
from common.model import load_pinocchio, pin_joint_order_names  # noqa: E402

OUT_YAML = repo_path("data/equilibrium/rear_equilibrium.yaml")

W_TAU = 1.0
W_JOINT = 0.1
W_MARGIN = 10.0
W_HEIGHT = 0.2
DESIRED_BASE_HEIGHT = 0.40  # soft preference only (w_height is small)

TAU_UTIL = 0.70  # spec §20.7


def skew(v):
    return ca.vertcat(
        ca.horzcat(0, -v[2], v[1]),
        ca.horzcat(v[2], 0, -v[0]),
        ca.horzcat(-v[1], v[0], 0),
    )


def main(pitch_min=-1.57, out_yaml=OUT_YAML) -> int:
    robot_cfg = load_robot_config()
    trajopt_cfg = load_yaml("config/trajopt.yaml")
    mp = robot_cfg["model_params"]
    wheel_radius = float(mp["wheel_radius"])
    mu = float(mp["contact_friction"]["tangential"])
    hip_nom = float(robot_cfg["hip_abduction_nominal"])
    tau_lim_type = mp["torque_limit"]
    mg = float(mp["total_mass"]) * 9.81

    model, _ = load_pinocchio()
    pin_names = pin_joint_order_names(model)  # declaration order FL/FR/RL/RR
    jidx = {n: i for i, n in enumerate(pin_names)}
    name_to_type = {
        robot_cfg["joint_map"][i]["name"]: robot_cfg["joint_map"][i]["type"]
        for i in range(16)
    }
    tau_limit = np.array([float(tau_lim_type[name_to_type[n]]) for n in pin_names])
    jrange = {}
    for n in pin_names:
        t = name_to_type[n]
        if t == "hip":
            jrange[n] = mp["joint_range"]["hip"]
        elif t == "thigh":
            jrange[n] = mp["joint_range"]["thigh_front" if n.startswith("F") else "thigh_back"]
        elif t == "calf":
            jrange[n] = mp["joint_range"]["calf"]
        else:
            jrange[n] = None

    # ---- decision variables ----
    bx = ca.SX.sym("bx")          # base x
    bz = ca.SX.sym("bz")          # base z
    th = ca.SX.sym("theta")       # base pitch about +y (roll=yaw=0 exact)
    thf = ca.SX.sym("thigh_front")  # shared by FR/FL
    caf = ca.SX.sym("calf_front")
    thr = ca.SX.sym("thigh_rear")   # shared by RR/RL
    car = ca.SX.sym("calf_rear")
    fxn = ca.SX.sym("fxn")        # shared rear-axle contact force, in units of m*g
    fzn = ca.SX.sym("fzn")
    fx = fxn * mg
    fz = fzn * mg
    x = ca.vertcat(bx, bz, th, thf, caf, thr, car, fxn, fzn)

    # ---- assemble q (pin free-flyer: x y z qx qy qz qw), declaration order
    q_leg = {}
    for leg, sgn in (("FR", 1.0), ("RR", 1.0), ("FL", -1.0), ("RL", -1.0)):
        q_leg[leg] = {
            "hip": sgn * hip_nom,
            "thigh": thf if leg.startswith("F") else thr,
            "calf": caf if leg.startswith("F") else car,
            "wheel": 0.0,
        }
    q_list = [bx, 0.0, bz, 0.0, ca.sin(th / 2), 0.0, ca.cos(th / 2)]
    for n in pin_names:
        leg = n.split("_")[0]
        typ = n[len(leg) + 1:].split("_")[0]
        q_list.append(q_leg[leg][typ])
    q = ca.vertcat(*q_list)

    # ---- model evaluation ----
    cmodel = cpin.Model(model)
    cdata = cmodel.createData()
    cpin.framesForwardKinematics(cmodel, cdata, q)
    g = cpin.computeGeneralizedGravity(cmodel, cdata, q)  # nv = 22
    com = cpin.centerOfMass(cmodel, cdata, q)

    wc = {}   # wheel center world position
    Jc = {}   # 3xnv point Jacobian at the wheel-bottom contact point
    for leg in ("FR", "FL", "RR", "RL"):
        fid = cmodel.getFrameId(f"{leg}_wheel_link")
        o = cdata.oMf[fid].translation
        wc[leg] = o
        Jf = cpin.computeFrameJacobian(cmodel, cdata, q, fid, pin.LOCAL_WORLD_ALIGNED)
        d = ca.vertcat(0.0, 0.0, -wheel_radius)  # contact - center, world frame
        # pinocchio frame Jacobian rows: 0:3 linear, 3:6 angular
        Jc[leg] = Jf[0:3, :] - skew(d) @ Jf[3:6, :]

    f3 = ca.vertcat(fx, 0.0, fz)  # 20.6: fy = 0
    Jrear = Jc["RR"] + Jc["RL"]   # f_RR == f_RL == f, so J^T f_RR + J^T f_RL

    # derived leg/wheel torques from the 16 joint columns of the statics
    tau_derived = g[6:] - Jrear[:, 6:].T @ f3  # 16 joint rows

    # ---- equality constraints ----
    g_eq = []
    # 20.1 rear wheel ground geometry (one row per rear wheel; by symmetry
    # they are identical, but write both as the spec lists them)
    for leg in ("RR", "RL"):
        g_eq.append(wc[leg][2] - wheel_radius)
    # 20.5 statics: the 6 base rows (the 16 joint rows define tau_derived).
    # Normalized for IPOPT conditioning: force rows / (m g), torque rows /
    # (m g * 0.3 m).
    stat_base = g[0:6] - Jrear[:, 0:6].T @ f3
    row_scale = ca.vertcat(ca.SX.ones(3) / mg, ca.SX.ones(3) / (mg * 0.3))
    g_eq.append(ca.times(stat_base, row_scale))
    # 20.6 symmetry is by construction (single shared f); nothing to add.

    # ---- inequality constraints ----
    g_in = []
    lbg_in, ubg_in = [], []
    # 20.8 CoM over support axle
    x_axle = 0.5 * (wc["RR"][0] + wc["RL"][0])
    g_in.append(com[0] - x_axle)
    lbg_in.append(-0.005); ubg_in.append(0.005)
    # 20.2 front wheel clearance
    for leg in ("FR", "FL"):
        g_in.append(wc[leg][2] - wheel_radius)
        lbg_in.append(0.05); ubg_in.append(ca.inf)
    # 20.6 friction cone
    g_in.append(fz);                 lbg_in.append(0.0); ubg_in.append(ca.inf)
    g_in.append(mu * fz - ca.fabs(fx)); lbg_in.append(0.0); ubg_in.append(ca.inf)
    # 20.7 torque limits on the derived torques (legs; rear wheels bounded too)
    for i, n in enumerate(pin_names):
        g_in.append(TAU_UTIL * tau_limit[i] - ca.fabs(tau_derived[i]))
        lbg_in.append(0.0); ubg_in.append(ca.inf)

    # ---- variable bounds ----
    lbx = [-1.0, 0.15, pitch_min]  # bx, bz, th (pitch: down to near-vertical)
    ubx = [1.0, 0.9, 0.2]
    for name in ("FR_thigh_joint", "FR_calf_joint", "RR_thigh_joint", "RR_calf_joint"):
        lo, hi = jrange[name]
        lbx.append(lo + 0.02)
        ubx.append(hi - 0.02)
    lbx += [-mu, 0.0]
    ubx += [mu, 1.0]

    # ---- cost (spec §21) ----
    cost = 0.0
    for i, n in enumerate(pin_names):
        cost = cost + W_TAU * (tau_derived[i] / tau_limit[i]) ** 2
    for var, nom in ((thf, 0.9), (caf, -1.8), (thr, 0.9), (car, -1.8)):
        cost = cost + W_JOINT * (var - nom) ** 2
    for name, var in (("FR_thigh_joint", thf), ("FR_calf_joint", caf),
                      ("RR_thigh_joint", thr), ("RR_calf_joint", car)):
        lo, hi = jrange[name]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        cost = cost + W_MARGIN * ((var - mid) / half) ** 4
    cost = cost + W_HEIGHT * (bz - DESIRED_BASE_HEIGHT) ** 2

    nlp = {"x": x, "f": cost, "g": ca.vertcat(ca.vertcat(*g_eq), ca.vertcat(*g_in))}
    n_eq = sum(e.shape[0] for e in g_eq)
    opts = {
        "ipopt.max_iter": int(trajopt_cfg["solver"]["max_iter"]),
        "ipopt.tol": float(trajopt_cfg["solver"]["tol"]),
        "ipopt.acceptable_tol": float(trajopt_cfg["solver"]["acceptable_tol"]),
        "ipopt.print_level": int(trajopt_cfg["solver"]["print_level"]),
        "ipopt.linear_solver": str(trajopt_cfg["solver"]["linear_solver"]),
        "print_time": 0,
    }
    solver = ca.nlpsol("equilibrium", "ipopt", nlp, opts)

    x0 = np.array([-0.10, 0.50, -0.55, 0.6, -1.5, 1.9, -2.4, 0.0, 0.5])
    lbg = [0.0] * n_eq + lbg_in
    ubg = [0.0] * n_eq + ubg_in
    sol = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
    xs = np.asarray(sol["x"]).ravel()
    stats = solver.stats()

    bx_, bz_, th_, thf_, caf_, thr_, car_ = xs[:7]
    fx_, fz_ = xs[7] * mg, xs[8] * mg  # denormalize (solver works in m*g)

    # ---- recompute everything with float pinocchio ----
    q_np = np.zeros(model.nq)
    q_np[:7] = [bx_, 0.0, bz_, 0.0, np.sin(th_ / 2), 0.0, np.cos(th_ / 2)]
    q_joint_pin = np.zeros(16)
    for i, n in enumerate(pin_names):
        leg = n.split("_")[0]
        typ = n[len(leg) + 1:].split("_")[0]
        val = {"hip": (1.0 if leg in ("FR", "RR") else -1.0) * hip_nom,
               "thigh": thf_ if leg.startswith("F") else thr_,
               "calf": caf_ if leg.startswith("F") else car_,
               "wheel": 0.0}[typ]
        q_joint_pin[i] = val
        q_np[model.joints[model.getJointId(n)].idx_q] = val

    fmodel, fdata = load_pinocchio()
    pin.forwardKinematics(fmodel, fdata, q_np)
    pin.updateFramePlacements(fmodel, fdata)
    g_f = pin.computeGeneralizedGravity(fmodel, fdata, q_np)
    com_f = pin.centerOfMass(fmodel, fdata, q_np)
    wc_f, Jc_f = {}, {}
    for leg in ("FR", "FL", "RR", "RL"):
        fid = fmodel.getFrameId(f"{leg}_wheel_link")
        o = np.asarray(fdata.oMf[fid].translation)
        wc_f[leg] = o
        d = np.array([0.0, 0.0, -wheel_radius])
        Jf = pin.computeFrameJacobian(fmodel, fdata, q_np, fid, pin.LOCAL_WORLD_ALIGNED)
        # pinocchio frame Jacobian rows: 0:3 linear, 3:6 angular
        Jc_f[leg] = Jf[0:3, :] - _skew_np(d) @ Jf[3:6, :]
    f3_np = np.array([fx_, 0.0, fz_])
    tau_np = g_f[6:] - (Jc_f["RR"][:, 6:] + Jc_f["RL"][:, 6:]).T @ f3_np
    resid = g_f - np.concatenate([np.zeros(6), tau_np]) \
        - Jc_f["RR"].T @ f3_np - Jc_f["RL"].T @ f3_np

    scale = np.concatenate([np.full(3, mg), np.full(3, mg * 0.3), tau_limit])
    max_resid = float(np.max(np.abs(resid)))
    max_resid_norm = float(np.max(np.abs(resid / scale)))

    sdk_names = [robot_cfg["joint_map"][i]["name"] for i in range(16)]
    pin_of = {n: i for i, n in enumerate(pin_names)}
    q_sdk = [float(q_joint_pin[pin_of[n]]) for n in sdk_names]
    tau_sdk = [float(tau_np[pin_of[n]]) for n in sdk_names]

    out = {
        "base_pose": {"x": float(bx_), "y": 0.0, "z": float(bz_),
                      "roll": 0.0, "pitch": float(th_), "yaw": 0.0},
        "joint_q": {"sdk_order": sdk_names, "values_sdk": q_sdk},
        "joint_tau": {"sdk_order": sdk_names, "values_sdk": tau_sdk},
        "contact_forces": {
            "RR": [float(fx_), 0.0, float(fz_)],
            "RL": [float(fx_), 0.0, float(fz_)],
        },
        "com": [float(v) for v in com_f],
        "rear_axle": {"x": float(0.5 * (wc_f["RR"][0] + wc_f["RL"][0]))},
        "wheel_centers": {leg: [float(v) for v in wc_f[leg]] for leg in wc_f},
        "solver_status": {
            "success": bool(stats.get("success", False)),
            "return_status": str(stats.get("return_status", "")),
            "iterations": int(stats.get("iter_count", -1)),
        },
        "constraint_residual": {
            "max_abs": max_resid,
            "max_normalized": max_resid_norm,
        },
        "units": {"q": "rad", "tau": "Nm", "force": "N"},
    }
    out_yaml = Path(out_yaml)
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    with open(out_yaml, "w") as fp:
        yaml.safe_dump(out, fp, sort_keys=False)
    print(f"[equilibrium] wrote {out_yaml}")
    print(f"[equilibrium] solver: {out['solver_status']}")
    print(f"[equilibrium] base: x={bx_:.4f} z={bz_:.4f} pitch={th_:.4f}")
    print(f"[equilibrium] legs: front thigh/calf=({thf_:.3f},{caf_:.3f}) "
          f"rear thigh/calf=({thr_:.3f},{car_:.3f})")
    print(f"[equilibrium] force per rear wheel: fx={fx_:.3f} fz={fz_:.3f} "
          f"(2*fz={2*fz_:.2f} vs mg={mg:.2f})")
    print(f"[equilibrium] residual: max_abs={max_resid:.2e} max_norm={max_resid_norm:.2e}")
    return 0 if stats.get("success", False) else 1


def _skew_np(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--pitch-min", type=float, default=-1.57,
                    help="lower bound on base pitch (rad); the balance "
                         "working point (default -1.57 = vertical)")
    ap.add_argument("--out", default=str(OUT_YAML))
    args = ap.parse_args()
    sys.exit(main(pitch_min=args.pitch_min, out_yaml=repo_path(args.out)
                  if not Path(args.out).is_absolute() else Path(args.out)))
