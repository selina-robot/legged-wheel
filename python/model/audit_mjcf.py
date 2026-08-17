#!/usr/bin/env python3
"""Model audit (spec §13): extract every model parameter of the official
Go2W MJCF into artifacts/reports/model_audit/model.json, and freeze the
parameters the code actually needs into config/robot.yaml's `model_params`
section (replacing the placeholder).

Nothing here is hand-copied into code; runtime code reads audited values
from config/robot.yaml only.

Usage: python python/model/audit_mjcf.py
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, repo_path  # noqa: E402
from common.io import write_json  # noqa: E402
from common.model import (  # noqa: E402
    actuator_order_names,
    joint_order_names,
    load_mujoco,
    permutation,
    MJCF_REL,
    SCENE_REL,
)

OUT_JSON = "artifacts/reports/model_audit/model.json"
ROBOT_YAML = repo_path("config/robot.yaml")


def audit():
    mjm, mjd = load_mujoco()
    mjm_scene, mjd_scene = load_mujoco(with_scene=True)
    robot_cfg = load_robot_config()

    joint_names = joint_order_names(mjm)      # declaration order (FL/FR/RL/RR)
    act_names = actuator_order_names(mjm)     # actuator/SDK order (FR/FL/RR/RL)

    # --- joints (declaration order) ---
    joints = []
    for j in range(mjm.njnt):
        if mjm.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
        dof = mjm.jnt_dofadr[j]
        joints.append(
            {
                "name": name,
                "body": mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, mjm.jnt_bodyid[j]),
                "axis": np.asarray(mjm.jnt_axis[j]).tolist(),
                "range": np.asarray(mjm.jnt_range[j]).tolist(),
                "limited": bool(mjm.jnt_limited[j]),
                "damping": float(mjm.dof_damping[dof]),
                "armature": float(mjm.dof_armature[dof]),
                "frictionloss": float(mjm.dof_frictionloss[dof]),
            }
        )

    # --- actuators (SDK order) ---
    actuators = []
    for a in range(mjm.nu):
        jid = mjm.actuator_trnid[a, 0]
        actuators.append(
            {
                "name": mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, a),
                "joint": mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, jid),
                "ctrlrange": np.asarray(mjm.actuator_ctrlrange[a]).tolist(),
                "gear": float(mjm.actuator_gear[a, 0]),
            }
        )

    # --- bodies ---
    bodies = []
    for b in range(1, mjm.nbody):
        bodies.append(
            {
                "name": mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, b),
                "mass": float(mjm.body_mass[b]),
                "ipos": np.asarray(mjm.body_ipos[b]).tolist(),
                "iquat": np.asarray(mjm.body_iquat[b]).tolist(),
                "inertia": np.asarray(mjm.body_inertia[b]).tolist(),
            }
        )
    total_mass = float(np.sum(mjm.body_mass))

    # --- CoM at qpos0 and at the nominal crouch q_init ---
    def com_at(qpos):
        mjd.qpos[:] = qpos
        mjd.qvel[:] = 0.0
        mujoco.mj_forward(mjm, mjd)
        return np.asarray(mjd.subtree_com[0]).tolist()

    com_qpos0 = com_at(mjm.qpos0)

    # q_init is stored in SDK order; map to MuJoCo qpos (declaration order).
    sdk_names = [robot_cfg["joint_map"][i]["name"] for i in range(16)]
    perm_sdk_to_joint = permutation(sdk_names, joint_names)  # joint[i] = sdk[perm[i]]
    q_init_sdk = np.array(robot_cfg["q_init"], dtype=float)
    q_init_joint = q_init_sdk[perm_sdk_to_joint]
    qpos_init = np.array(mjm.qpos0)
    qpos_init[2] = 0.323  # FK-verified base height for this crouch
    for i, name in enumerate(joint_names):
        jid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_init[mjm.jnt_qposadr[jid]] = q_init_joint[i]
    com_q_init = com_at(qpos_init)

    # --- wheels: radius from the mesh, axle direction, link frames ---
    wheels = {}
    for name in joint_names:
        if "wheel" not in name:
            continue
        jid = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, name)
        bid = mjm.jnt_bodyid[jid]
        body_name = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, bid)
        # geom of this body using the wheel mesh
        radius = None
        for g in range(mjm.ngeom):
            if mjm.geom_bodyid[g] == bid and mjm.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = mjm.geom_dataid[g]
                v0, nv = mjm.mesh_vertadr[mid], mjm.mesh_vertnum[mid]
                verts = np.asarray(mjm.mesh_vert[v0 : v0 + nv]).reshape(nv, 3)
                # axle in geom frame: R_geom^T * axle_link (joint frame ==
                # body frame here since the joint pos is 0 and axis local)
                axle_link = np.asarray(mjm.jnt_axis[jid])
                qw, qx, qy, qz = np.asarray(mjm.geom_quat[g])
                Rg = np.array(
                    [
                        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
                    ]
                )
                axle_geom = Rg.T @ axle_link
                r = np.linalg.norm(
                    verts - np.outer(verts @ axle_geom, axle_geom), axis=1
                ).max()
                radius = float(r)
        wheels[body_name] = {
            "joint": name,
            "radius_from_mesh": radius,
            "axle_local": np.asarray(mjm.jnt_axis[jid]).tolist(),
            "body_pos_offset": np.asarray(mjm.body_pos[bid]).tolist(),
            "body_quat_offset": np.asarray(mjm.body_quat[bid]).tolist(),
        }
    radii = [w["radius_from_mesh"] for w in wheels.values()]
    wheel_radius = float(np.mean(radii))
    if max(radii) - min(radii) > 1e-6:
        raise RuntimeError(f"wheel radii disagree: {radii}")

    # --- friction: all contact geoms on the wheel bodies + scene floor ---
    # The wheel body carries TWO contact geoms over the same mesh: a
    # frictionless condim-1 shell (class "collision") and the frictional
    # condim-6 "foot" class geom. Report both; the effective wheel-ground
    # contact is their combination.
    wheel_contact_geoms = []
    for g in range(mjm.ngeom):
        bid = mjm.geom_bodyid[g]
        bname = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, bid)
        if bname and "wheel" in bname and mjm.geom_contype[g] > 0:
            wheel_contact_geoms.append(
                {
                    "geom_id": int(g),
                    "body": bname,
                    "friction": np.asarray(mjm.geom_friction[g]).tolist(),
                    "condim": int(mjm.geom_condim[g]),
                }
            )
    # per-class values are identical across wheels; verify
    fr_set = {tuple(np.round(w["friction"], 8)) for w in wheel_contact_geoms}
    assert len(fr_set) == 2, f"unexpected wheel contact geom set: {fr_set}"
    foot = max(wheel_contact_geoms, key=lambda w: w["condim"])  # condim 6 = foot class
    shell = min(wheel_contact_geoms, key=lambda w: w["condim"])  # condim 1 shell
    floor_friction = None
    floor_condim = None
    floor_id = mujoco.mj_name2id(mjm_scene, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if floor_id >= 0:
        floor_friction = np.asarray(mjm_scene.geom_friction[floor_id]).tolist()
        floor_condim = int(mjm_scene.geom_condim[floor_id])

    report = {
        "source": MJCF_REL,
        "scene": SCENE_REL,
        "joint_order_mujoco_pinocchio": joint_names,
        "actuator_order_sdk": act_names,
        "joints": joints,
        "actuators": actuators,
        "bodies": bodies,
        "total_mass": total_mass,
        "com_qpos0": com_qpos0,
        "com_q_init": com_q_init,
        "wheels": wheels,
        "wheel_radius": wheel_radius,
        "friction": {
            "wheel_geoms": wheel_contact_geoms,
            "floor_geom": {"friction": floor_friction, "condim": floor_condim},
            "combine_rule": "element-wise maximum (MuJoCo default for equal priority)",
        },
        "joint_damping": sorted(set(j["damping"] for j in joints)),
        "joint_armature": sorted(set(j["armature"] for j in joints)),
        "joint_frictionloss": sorted(set(j["frictionloss"] for j in joints)),
        "gravity": np.asarray(mjm.opt.gravity).tolist(),
    }
    write_json(OUT_JSON, report)
    print(f"[audit] wrote {OUT_JSON}")

    _update_robot_yaml(report)


def _update_robot_yaml(report):
    """Freeze the parameters runtime code needs into config/robot.yaml's
    model_params section (replaces the placeholder line)."""
    ranges = {j["name"]: j["range"] for j in report["joints"]}
    trq = {a["joint"]: abs(a["ctrlrange"][1]) for a in report["actuators"]}
    foot = max(report["friction"]["wheel_geoms"], key=lambda w: w["condim"])

    def fmt(x):
        return f"{x:.6g}"

    block = f"""model_params:
  # Frozen by python/model/audit_mjcf.py from the official MJCF.
  total_mass: {fmt(report['total_mass'])}
  wheel_radius: {fmt(report['wheel_radius'])}
  joint_range:
    hip: {ranges['FR_hip_joint']}
    thigh_front: {ranges['FR_thigh_joint']}
    thigh_back: {ranges['RR_thigh_joint']}
    calf: {ranges['FR_calf_joint']}
    wheel: null  # unlimited
  torque_limit:
    hip: {fmt(trq['FR_hip_joint'])}
    thigh: {fmt(trq['FR_thigh_joint'])}
    calf: {fmt(trq['FR_calf_joint'])}
    wheel: {fmt(trq['FR_wheel_joint'])}
  dq_limit: null  # not specified in the MJCF
  contact_friction:  # wheel "foot" class geom; the wheel also carries a
    # frictionless condim-1 shell, and the floor uses MuJoCo defaults.
    tangential: {fmt(foot['friction'][0])}
    torsional: {fmt(foot['friction'][1])}
    rolling: {fmt(foot['friction'][2])}
    condim: {foot['condim']}
  joint_damping: {fmt(report['joint_damping'][0])}
  joint_armature: {fmt(report['joint_armature'][0])}
  joint_frictionloss: {fmt(report['joint_frictionloss'][0])}"""

    text = ROBOT_YAML.read_text()
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if l.startswith("model_params:"))
    # the placeholder is a single line ("model_params: {}"); if a previous
    # audit block exists, drop through to the next top-level key.
    end = start + 1
    while end < len(lines) and (lines[end].startswith("  ") or not lines[end].strip()):
        end += 1
    new_text = "\n".join(lines[:start]) + "\n" + block + "\n" + "\n".join(lines[end:])
    if not new_text.endswith("\n"):
        new_text += "\n"
    ROBOT_YAML.write_text(new_text)
    print(f"[audit] froze model_params in {ROBOT_YAML}")


if __name__ == "__main__":
    audit()
