"""Model loading and joint-order helpers for the official Go2W MJCF.

The single model source of truth is third_party/unitree_mujoco Go2W MJCF
(AGENTS.md). Two orderings matter:

- joint/declaration order (MuJoCo qpos/qvel, Pinocchio q/v): FL, FR, RL, RR
  per leg, i.e. the worldbody order in go2w.xml.
- actuator/SDK order (LowCmd/LowState, actuators section): FR, FL, RR, RL.

All conversions go through explicit name-based permutation built from
config/robot.yaml / the loaded models. Never sort names alphabetically.
"""
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from .config import repo_path

MJCF_REL = "third_party/unitree_mujoco/unitree_robots/go2w/go2w.xml"
SCENE_REL = "third_party/unitree_mujoco/unitree_robots/go2w/scene.xml"

NUM_JOINTS = 16
LEGS = ("FR", "FL", "RR", "RL")


def mjcf_path() -> Path:
    return repo_path(MJCF_REL)


def load_mujoco(with_scene: bool = False) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(repo_path(SCENE_REL if with_scene else MJCF_REL)))
    return model, mujoco.MjData(model)


def load_pinocchio() -> tuple[pin.Model, pin.Data]:
    model = pin.buildModelFromMJCF(str(mjcf_path()))
    return model, model.createData()


def joint_order_names(mjm: mujoco.MjModel) -> list[str]:
    """Hinge joint names in MuJoCo qpos order (declaration order)."""
    names = []
    for j in range(mjm.njnt):
        if mjm.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            names.append(mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j))
    return names


def actuator_order_names(mjm: mujoco.MjModel) -> list[str]:
    """Joint names in actuator order (== SDK LowCmd order for Go2W)."""
    names = []
    for a in range(mjm.nu):
        jid = mjm.actuator_trnid[a, 0]
        names.append(mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, jid))
    return names


def pin_joint_order_names(model: pin.Model) -> list[str]:
    """Joint names in Pinocchio q order (excluding universe and the free
    flyer, whose name ends with '_free')."""
    return [n for n in model.names if n != "universe" and not n.endswith("_free")]


def permutation(src_names: list[str], dst_names: list[str]) -> np.ndarray:
    """perm such that vec_dst[i] = vec_src[perm[i]] (by name)."""
    assert sorted(src_names) == sorted(dst_names), (src_names, dst_names)
    index = {n: i for i, n in enumerate(src_names)}
    return np.array([index[n] for n in dst_names])


def mujoco_qpos_to_pin(qpos: np.ndarray) -> np.ndarray:
    """MuJoCo free-joint qpos (x y z qw qx qy qz) -> Pinocchio free-flyer
    (x y z qx qy qz qw). Joint components keep the same order."""
    q = np.asarray(qpos, dtype=float).copy()
    quat = q[3:7].copy()
    q[3:7] = [quat[1], quat[2], quat[3], quat[0]]
    return q


def mujoco_qvel_to_pin(qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
    """MuJoCo free-joint qvel (global-frame linear, body-local angular) ->
    Pinocchio free-flyer velocity (body-local linear, body-local angular).

    Verified empirically against mj_step/pin.integrate (Gate-1 era tests).
    """
    q = np.asarray(qpos, dtype=float)
    v = np.asarray(qvel, dtype=float)
    qw, qx, qy, qz = q[3:7]
    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    out = np.asarray(v).copy()
    out[0:3] = R.T @ v[0:3]
    out[3:6] = v[3:6]
    return out


def mujoco_qacc_to_pin(qpos: np.ndarray, qvel: np.ndarray, qacc: np.ndarray) -> np.ndarray:
    """MuJoCo qacc -> Pinocchio tangent acceleration.

    The base linear tangent is p_dot (world frame) in MuJoCo but v_local in
    Pinocchio, so accelerations carry a frame-rotation correction:
        a_pin_lin = R^T a_mj_lin - omega_local x (R^T p_dot)
    """
    a_mj = np.asarray(qacc, dtype=float)
    v_pin = mujoco_qvel_to_pin(qpos, qvel)
    omega = v_pin[3:6]
    out = a_mj.copy()
    q = np.asarray(qpos, dtype=float)
    qw, qx, qy, qz = q[3:7]
    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    out[0:3] = R.T @ a_mj[0:3] - np.cross(omega, v_pin[0:3])
    out[3:6] = a_mj[3:6]
    return out
