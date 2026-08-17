#!/usr/bin/env python3
"""Independent Gate 2 validation (spec §22) of data/equilibrium/rear_equilibrium.yaml.

Recomputes everything with float Pinocchio (not the CasADi path) and checks:
  [ ] IPOPT success
  [ ] max dynamics residual < 1e-5 normalized
        (base force rows / (m g), base torque rows / (m g * 0.3 m),
         joint rows / tau_limit)
  [ ] front wheel clearance >= 0.05 m
  [ ] all joint limits satisfied
  [ ] all |tau| <= 0.70 * tau_limit
  [ ] friction utilization <= 0.70 (|fx| / (mu * fz), fz > 0)
  [ ] |x_com - x_axle| <= 5 mm

Plus a MuJoCo cross-check: at q_eq with zero velocity, the scene's contacts
must show up on the rear wheels only, with the sum of normal forces ~= m g;
and a 0.3 s free simulation with constant tau_eq feedforward must keep the
base pitch within 0.05 rad.

Usage: python python/equilibrium/validate_equilibrium.py
Exit 0 iff every Gate 2 criterion passes.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, repo_path  # noqa: E402
from common.model import (  # noqa: E402
    actuator_order_names,
    joint_order_names,
    load_mujoco,
    load_pinocchio,
    permutation,
    pin_joint_order_names,
)

EQ_YAML = repo_path("data/equilibrium/rear_equilibrium.yaml")

TAU_UTIL = 0.70          # spec §20.7
FRONT_CLEARANCE = 0.05   # spec §20.2
COM_AXLE_TOL = 0.005     # spec §20.8
FRICTION_UTIL = 0.70     # spec §22
RESID_NORM_MAX = 1e-5    # spec §22


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def main() -> int:
    eq = yaml.safe_load(open(EQ_YAML))
    robot_cfg = load_robot_config()
    mp = robot_cfg["model_params"]
    wheel_radius = float(mp["wheel_radius"])
    mu = float(mp["contact_friction"]["tangential"])
    mg = float(mp["total_mass"]) * 9.81
    tau_lim_type = mp["torque_limit"]
    name_to_type = {
        robot_cfg["joint_map"][i]["name"]: robot_cfg["joint_map"][i]["type"]
        for i in range(16)
    }

    sdk_names = eq["joint_q"]["sdk_order"]
    q_sdk = np.array(eq["joint_q"]["values_sdk"])
    tau_sdk = np.array(eq["joint_tau"]["values_sdk"])

    model, data = load_pinocchio()
    pin_names = pin_joint_order_names(model)
    perm = permutation(sdk_names, pin_names)  # pin[i] = sdk[perm[i]]
    q_joint = q_sdk[perm]
    tau = tau_sdk[perm]

    bp = eq["base_pose"]
    q = np.zeros(model.nq)
    q[:3] = [bp["x"], bp["y"], bp["z"]]
    q[3:7] = [0.0, np.sin(bp["pitch"] / 2), 0.0, np.cos(bp["pitch"] / 2)]  # xyzw
    for i, n in enumerate(pin_names):
        q[model.joints[model.getJointId(n)].idx_q] = q_joint[i]

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    g = pin.computeGeneralizedGravity(model, data, q)
    com = pin.centerOfMass(model, data, q)

    wc, Jc = {}, {}
    for leg in ("FR", "FL", "RR", "RL"):
        fid = model.getFrameId(f"{leg}_wheel_link")
        o = np.asarray(data.oMf[fid].translation)
        wc[leg] = o
        d = np.array([0.0, 0.0, -wheel_radius])
        Jf = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        # pinocchio frame Jacobian rows: 0:3 linear, 3:6 angular
        Jc[leg] = Jf[0:3, :] - skew(d) @ Jf[3:6, :]

    resid = g - np.concatenate([np.zeros(6), tau])
    for leg in ("RR", "RL"):
        f = np.array(eq["contact_forces"][leg])
        resid -= Jc[leg].T @ f

    scale = np.concatenate([
        np.full(3, mg), np.full(3, mg * 0.3),
        np.array([float(tau_lim_type[name_to_type[n]]) for n in pin_names]),
    ])
    max_resid = float(np.max(np.abs(resid)))
    max_resid_norm = float(np.max(np.abs(resid / scale)))

    fr = {leg: np.array(eq["contact_forces"][leg]) for leg in ("RR", "RL")}
    x_axle = 0.5 * (wc["RR"][0] + wc["RL"][0])

    checks = []

    def check(name, ok, detail):
        checks.append((name, bool(ok), detail))

    check("IPOPT success", eq["solver_status"]["success"],
          eq["solver_status"]["return_status"])
    check("max dynamics residual < 1e-5 normalized",
          max_resid_norm < RESID_NORM_MAX,
          f"max_abs={max_resid:.3e} max_norm={max_resid_norm:.3e}")
    clr = min(wc["FR"][2], wc["FL"][2]) - wheel_radius
    check("front wheel clearance >= 0.05 m", clr >= FRONT_CLEARANCE,
          f"clearance={clr:.4f} m")
    lim_ok = True
    lim_detail = []
    for i, n in enumerate(pin_names):
        t = name_to_type[n]
        if t == "wheel":
            continue
        lo, hi = mp["joint_range"]["hip"] if t == "hip" else \
            mp["joint_range"]["thigh_front" if n.startswith("F") else "thigh_back"] \
            if t == "thigh" else mp["joint_range"]["calf"]
        if not (lo - 1e-9 <= q_joint[i] <= hi + 1e-9):
            lim_ok = False
            lim_detail.append(f"{n}={q_joint[i]:.3f} not in [{lo},{hi}]")
    check("all joint limits satisfied", lim_ok, "; ".join(lim_detail) or "ok")
    tau_util = max(
        abs(tau[i]) / float(tau_lim_type[name_to_type[pin_names[i]]])
        for i in range(16)
    )
    check("all |tau| <= 0.70 * tau_limit", tau_util <= TAU_UTIL + 1e-9,
          f"max utilization={tau_util:.3f}")
    fric_util = max(abs(f[0]) / (mu * f[2]) for f in fr.values())
    check("friction utilization <= 0.70", fric_util <= FRICTION_UTIL + 1e-9,
          f"utilization={fric_util:.3f}")
    com_axle = abs(com[0] - x_axle)
    check("|x_com - x_axle| <= 5 mm", com_axle <= COM_AXLE_TOL + 1e-12,
          f"|dx|={com_axle * 1e3:.3f} mm")
    fz_sum = sum(f[2] for f in fr.values())
    check("sum fz ~= m g", abs(fz_sum - mg) / mg < 1e-3,
          f"2*fz={fz_sum:.3f} vs mg={mg:.3f}")

    # --- MuJoCo cross-check ---
    mujoco_ok, mujoco_detail = mujoco_crosscheck(
        model, q, tau, sdk_names, mg, wheel_radius
    )
    check("MuJoCo: rear-only contact, sum normal ~= m g", mujoco_ok,
          mujoco_detail)

    print("==== Gate 2 validation ====")
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    all_ok = all(ok for _, ok, _ in checks)
    print("GATE 2 " + ("PASS" if all_ok else "FAIL"))
    return 0 if all_ok else 1


def mujoco_crosscheck(model, q_pin, tau_pin, sdk_names, mg, wheel_radius):
    """Set q_eq in the MuJoCo scene with zero velocity and constant tau_eq
    feedforward. After a 100 ms settle (the compliant mesh contact needs a
    few ms to load), the rear wheels must carry ~m g with the front wheels
    untouched; over a further 0.3 s the base pitch must drift < 0.05 rad."""
    mjm, mjd = load_mujoco(with_scene=True)
    pin_names = joint_order_names(mjm)   # mujoco qpos order == declaration
    act_names = actuator_order_names(mjm)

    # q_pin (pin free-flyer xyzw quat, declaration-order joints) -> qpos
    qpos = np.zeros(mjm.nq)
    qpos[:3] = q_pin[:3]
    quat_xyzw = q_pin[3:7]
    qpos[3:7] = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
    for n in pin_names:
        jid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, n)
        qpos[mjm.jnt_qposadr[jid]] = q_pin[model.joints[model.getJointId(n)].idx_q]

    # tau in actuator order for ctrl
    perm = permutation(pin_names, act_names)  # act[i] = pin[perm[i]]
    ctrl = tau_pin[perm]

    def quat_pitch(wxyz):
        w, x, y, z = wxyz
        return np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))

    mjd.qpos[:] = qpos
    mjd.qvel[:] = 0.0
    mjd.ctrl[:] = ctrl

    def contact_summary():
        wheel_body = {
            leg: mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_wheel_link")
            for leg in ("FR", "FL", "RR", "RL")
        }
        normal = {leg: 0.0 for leg in wheel_body}
        for c in range(mjd.ncon):
            frc = np.zeros(6)
            mujoco.mj_contactForce(mjm, mjd, c, frc)
            b1 = mjm.geom_bodyid[mjd.contact[c].geom1]
            b2 = mjm.geom_bodyid[mjd.contact[c].geom2]
            for leg, bid in wheel_body.items():
                if b1 == bid or b2 == bid:
                    normal[leg] += frc[0]  # contact frame: normal first
        return normal

    mujoco.mj_forward(mjm, mjd)
    n0 = contact_summary()
    front0 = n0["FR"] + n0["FL"]
    if front0 > 0.01 * mg:
        return False, f"front wheels in contact at t=0 ({front0:.2f} N)"

    # settle the compliant contact under the equilibrium feedforward
    for _ in range(int(0.1 / mjm.opt.timestep)):
        mujoco.mj_step(mjm, mjd)
    n1 = contact_summary()
    rear_fz = n1["RR"] + n1["RL"]
    front_fz = n1["FR"] + n1["FL"]
    pitch0 = quat_pitch(mjd.qpos[3:7])

    for _ in range(int(0.3 / mjm.opt.timestep)):
        mujoco.mj_step(mjm, mjd)
    pitch1 = quat_pitch(mjd.qpos[3:7])
    n2 = contact_summary()

    detail = (f"rear normal={rear_fz:.2f} N (mg={mg:.2f}), "
              f"front={front_fz:.3f} N (ncon={mjd.ncon}); "
              f"pitch drift over 0.3 s: {pitch1 - pitch0:+.4f} rad; "
              f"late rear normal={n2['RR'] + n2['RL']:.2f} N")
    ok = (abs(rear_fz - mg) / mg < 0.01 and front_fz < 0.01 * mg
          and abs(pitch1 - pitch0) < 0.05)
    return ok, detail


if __name__ == "__main__":
    sys.exit(main())
