#!/usr/bin/env python3
"""MuJoCo vs Pinocchio dynamics consistency (spec §14).

Both sides load the same official go2w.xml. We generate 10 legal, left-right
symmetric, collision-free poses and compare:

  total mass, CoM, gravity generalized torque, mass-matrix blocks, RNEA torque

Gravity torque relative error must be < 3% (hard Gate criterion). If any
error is large, the suspects are (in order): joint mapping, free-flyer
convention, quaternion order (MuJoCo wxyz vs Pinocchio xyzw storage),
wheel joint order, armature, gravity. Never tune anything else to hide a
model mismatch.

Also cross-checks that config/robot.yaml's joint_map order equals the MJCF
actuator order (the §14 "joint order mapping" item).

Usage: python python/model/compare_mujoco_pinocchio.py
Exit 0 iff all criteria pass.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config  # noqa: E402
from common.model import (  # noqa: E402
    actuator_order_names,
    joint_order_names,
    load_mujoco,
    load_pinocchio,
    mujoco_qpos_to_pin,
    mujoco_qvel_to_pin,
    pin_joint_order_names,
)

GRAVITY_RTOL = 0.03  # spec §14 hard criterion
TIGHT_RTOL = 1e-6    # same model: everything else should match to ~machine eps


def generate_poses(mjm, mjd, n=10, seed=1):
    """10 legal, left-right symmetric, collision-free poses (MuJoCo qpos).

    Left-right symmetry about the sagittal plane: FL mirrors FR and RL
    mirrors RR, i.e. hip abduction flips sign, thigh/calf/wheel are equal.
    The base gets a small random orientation (to exercise the free-flyer /
    quaternion conventions) and sits at spawn height, so only self-contacts
    can occur.
    """
    rng = np.random.default_rng(seed)
    names = joint_order_names(mjm)
    adr = {}
    for name in names:
        jid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, name)
        adr[name] = mjm.jnt_qposadr[jid]

    poses = []
    while len(poses) < n:
        q = np.array(mjm.qpos0)
        q[2] = 0.6  # spawn height: wheels clear of the floor
        rpy = rng.uniform(-0.3, 0.3, 3)
        cr, cp, cy = np.cos(rpy / 2)
        sr, sp, sy = np.sin(rpy / 2)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        q[3:7] = [qw, qx, qy, qz]

        def setj(name, val):
            q[adr[name]] = val

        # sample right side, mirror to the left
        hip_f = rng.uniform(-0.6, 0.6)
        hip_r = rng.uniform(-0.6, 0.6)
        th_f = rng.uniform(-1.0, 2.5)
        ca_f = rng.uniform(-2.5, -1.0)
        th_r = rng.uniform(0.0, 3.5)
        ca_r = rng.uniform(-2.5, -1.0)
        wh_f = rng.uniform(-3.0, 3.0)
        wh_r = rng.uniform(-3.0, 3.0)
        for leg, hip, th, ca, wh in (
            ("FR", hip_f, th_f, ca_f, wh_f),
            ("RR", hip_r, th_r, ca_r, wh_r),
        ):
            setj(f"{leg}_hip_joint", hip)
            setj(f"{leg}_thigh_joint", th)
            setj(f"{leg}_calf_joint", ca)
            setj(f"{leg}_wheel_joint", wh)
        for leg, hip, th, ca, wh in (
            ("FL", -hip_f, th_f, ca_f, wh_f),
            ("RL", -hip_r, th_r, ca_r, wh_r),
        ):
            setj(f"{leg}_hip_joint", hip)
            setj(f"{leg}_thigh_joint", th)
            setj(f"{leg}_calf_joint", ca)
            setj(f"{leg}_wheel_joint", wh)

        mjd.qpos[:] = q
        mjd.qvel[:] = 0.0
        mujoco.mj_forward(mjm, mjd)
        if mjd.ncon == 0:
            poses.append(q)
    return poses


def compare(num_poses=10, seed=1, verbose=True):
    mjm, mjd = load_mujoco()
    model, data = load_pinocchio()
    robot_cfg = load_robot_config()

    # --- joint order sanity: MuJoCo qpos order == Pinocchio q order, and
    # robot.yaml's SDK order == MJCF actuator order ---
    mj_joints = joint_order_names(mjm)
    pin_joints = pin_joint_order_names(model)
    assert mj_joints == pin_joints, (mj_joints, pin_joints)
    act_joints = actuator_order_names(mjm)
    sdk_names = [robot_cfg["joint_map"][i]["name"] for i in range(16)]
    assert act_joints == sdk_names, (act_joints, sdk_names)

    # gravity vector must agree
    assert np.allclose(mjm.opt.gravity[:3], [0, 0, -9.81])
    assert np.allclose(model.gravity.linear, [0, 0, -9.81]), model.gravity.linear

    # armature: MuJoCo dof_armature vs Pinocchio model.armature
    arm_mj = np.asarray(mjm.dof_armature)
    arm_pin = np.asarray(model.armature)
    assert np.allclose(arm_mj, arm_pin), (arm_mj, arm_pin)

    results = {
        "total_mass_mj": float(np.sum(mjm.body_mass)),
        "total_mass_pin": float(pin.computeTotalMass(model)),
        "gravity_rel_err": [],
        "com_abs_err_mm": [],
        "mass_matrix_rel_err": [],
        "mass_matrix_base_block_rel_err": [],
        "rne_inertial_rel_err": [],
        "rne_bias_rel_err": [],
        "rne_full_rel_err": [],
    }

    rng = np.random.default_rng(seed + 100)
    for q_mj in generate_poses(mjm, mjd, num_poses, seed):
        q_pin = mujoco_qpos_to_pin(q_mj)

        # Free-flyer convention bridge: MuJoCo expresses the base linear
        # part (velocity, acceleration, generalized force) in the GLOBAL
        # frame, Pinocchio in the BODY-LOCAL frame; angular parts are local
        # on both sides. R rotates body-local -> global.
        w, x, y, z = q_mj[3:7]
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        T = np.eye(mjm.nv)
        T[:3, :3] = R  # maps pin base tangent/force -> mujoco convention

        # --- CoM ---
        mjd.qpos[:] = q_mj
        mjd.qvel[:] = 0.0
        mujoco.mj_forward(mjm, mjd)
        com_mj = np.asarray(mjd.subtree_com[0])
        com_pin = pin.centerOfMass(model, data, q_pin)
        results["com_abs_err_mm"].append(1e3 * np.linalg.norm(com_mj - com_pin))

        # --- gravity generalized torque (v = 0) ---
        g_mj = np.asarray(mjd.qfrc_bias).copy()
        g_pin = pin.computeGeneralizedGravity(model, data, q_pin)
        g_mj_local = T.T @ g_mj
        results["gravity_rel_err"].append(
            np.linalg.norm(g_mj_local - g_pin) / np.linalg.norm(g_mj_local)
        )

        # --- mass matrix (same change of basis on rows and columns) ---
        M_mj = np.zeros((mjm.nv, mjm.nv))
        mujoco.mj_fullM(mjm, M_mj, mjd.qM)
        M_pin = pin.crba(model, data, q_pin)
        M_pin = 0.5 * (M_pin + M_pin.T)
        M_mj_local = T.T @ M_mj @ T
        results["mass_matrix_rel_err"].append(
            np.linalg.norm(M_mj_local - M_pin) / np.linalg.norm(M_mj_local)
        )
        base_mj = M_mj_local[:6, :6]
        base_pin = M_pin[:6, :6]
        results["mass_matrix_base_block_rel_err"].append(
            np.linalg.norm(base_mj - base_pin) / np.linalg.norm(base_mj)
        )

        # --- RNEA with random velocity/acceleration ---
        # Sample on the Pinocchio side and forward-map to MuJoCo
        # coordinates: v_mj = T v_pin; a_mj = T a_pin + [R(omega x v_lin); 0]
        # (the base linear tangent is p_dot in MuJoCo but v_local in
        # Pinocchio, so accelerations pick up the frame-rotation term).
        # Note: mj_rne's inertial term double-counts the joint armature
        # (residual == armature * qacc on joint dofs, zero on base dofs;
        # measured and documented). Hence the physically meaningful checks
        # are the inertial part and the bias part separately; the raw
        # full-call difference is only reported.
        v_pin = rng.uniform(-1.0, 1.0, mjm.nv)
        v_pin[18:] = rng.uniform(-15.0, 15.0, 4)  # wheels spin fast
        a_pin = rng.uniform(-3.0, 3.0, mjm.nv)
        corr = np.zeros(mjm.nv)
        corr[:3] = R @ np.cross(v_pin[3:6], v_pin[:3])
        v_mj = T @ v_pin
        a_mj = T @ a_pin + corr

        mjd.qvel[:] = v_mj
        mujoco.mj_forward(mjm, mjd)
        # mj_rne consumes d.qacc, so set it only after mj_forward (which
        # overwrites qacc) has computed the velocity-dependent quantities.
        mjd.qacc[:] = a_mj
        tau_mj = np.zeros(mjm.nv)
        mujoco.mj_rne(mjm, mjd, 1, tau_mj)
        mjd.qacc[:] = 0.0
        bias_mj = np.zeros(mjm.nv)
        mujoco.mj_rne(mjm, mjd, 0, bias_mj)

        tau_pin = pin.rnea(model, data, q_pin, v_pin, a_pin)
        bias_pin = pin.rnea(model, data, q_pin, v_pin, np.zeros(mjm.nv))

        # bias comparison: C_pin == T^T (C_mj + M_mj * corr)
        results["rne_bias_rel_err"].append(
            np.linalg.norm(bias_pin - T.T @ (bias_mj + M_mj @ corr))
            / np.linalg.norm(bias_pin)
        )
        # inertial comparison: M_pin a_pin == T^T M_mj T a_pin
        inertial_pin = tau_pin - bias_pin
        inertial_mj_local = T.T @ (M_mj @ (T @ a_pin))
        results["rne_inertial_rel_err"].append(
            np.linalg.norm(inertial_mj_local - inertial_pin)
            / np.linalg.norm(inertial_pin)
        )
        # raw full-call difference (includes the armature quirk)
        tau_mj_local = T.T @ tau_mj
        results["rne_full_rel_err"].append(
            np.linalg.norm(tau_mj_local - tau_pin) / np.linalg.norm(tau_pin)
        )

    summary = {
        "total_mass_mj": results["total_mass_mj"],
        "total_mass_pin": results["total_mass_pin"],
        "total_mass_abs_err": abs(
            results["total_mass_mj"] - results["total_mass_pin"]
        ),
        "gravity_rel_err_max": float(np.max(results["gravity_rel_err"])),
        "gravity_rel_err_all": results["gravity_rel_err"],
        "com_abs_err_mm_max": float(np.max(results["com_abs_err_mm"])),
        "mass_matrix_rel_err_max": float(np.max(results["mass_matrix_rel_err"])),
        "mass_matrix_base_block_rel_err_max": float(
            np.max(results["mass_matrix_base_block_rel_err"])
        ),
        "rne_inertial_rel_err_max": float(np.max(results["rne_inertial_rel_err"])),
        "rne_bias_rel_err_max": float(np.max(results["rne_bias_rel_err"])),
        "rne_full_rel_err_max": float(np.max(results["rne_full_rel_err"])),
        "num_poses": num_poses,
    }

    if verbose:
        print(f"poses: {num_poses} (legal, left-right symmetric, collision-free)")
        print(f"total mass:        mujoco {summary['total_mass_mj']:.6f}  "
              f"pinocchio {summary['total_mass_pin']:.6f}  "
              f"|diff| {summary['total_mass_abs_err']:.2e}")
        print(f"CoM abs err max:   {summary['com_abs_err_mm_max']:.3e} mm")
        print(f"mass matrix rel err max: {summary['mass_matrix_rel_err_max']:.3e}")
        print(f"  base 6x6 block rel err max: {summary['mass_matrix_base_block_rel_err_max']:.3e}")
        print(f"RNEA inertial part rel err max: {summary['rne_inertial_rel_err_max']:.3e}")
        print(f"RNEA bias part rel err max:     {summary['rne_bias_rel_err_max']:.3e}")
        print(f"RNEA full-call rel err max:     {summary['rne_full_rel_err_max']:.3e} "
              "(dominated by the mj_rne armature bookkeeping quirk, see code comment)")
        print("gravity torque rel err per pose: "
              + " ".join(f"{e:.2e}" for e in results["gravity_rel_err"]))
        print(f"gravity torque rel err max: {summary['gravity_rel_err_max']:.3e} "
              f"(criterion < {GRAVITY_RTOL})")

    summary["pass"] = bool(
        summary["gravity_rel_err_max"] < GRAVITY_RTOL
        and summary["total_mass_abs_err"] < 1e-6
        and summary["com_abs_err_mm_max"] < 1e-3
        and summary["mass_matrix_rel_err_max"] < TIGHT_RTOL
        and summary["rne_inertial_rel_err_max"] < TIGHT_RTOL
        and summary["rne_bias_rel_err_max"] < TIGHT_RTOL
        and summary["rne_full_rel_err_max"] < 0.01  # armature quirk bound
    )
    return summary


if __name__ == "__main__":
    s = compare()
    print("compare_mujoco_pinocchio: " + ("PASS" if s["pass"] else "FAIL"))
    sys.exit(0 if s["pass"] else 1)
