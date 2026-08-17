"""Gate-1 model loading test (spec §74): the official Go2W MJCF must load in
both MuJoCo and Pinocchio, with 16 actuators and 4 wheels identified."""
import mujoco
import pinocchio as pin

from common.model import (
    actuator_order_names,
    joint_order_names,
    load_mujoco,
    load_pinocchio,
    pin_joint_order_names,
)


def test_mjcf_loads_in_mujoco():
    mjm, _ = load_mujoco()
    assert mjm.nq == 23 and mjm.nv == 22


def test_mjcf_loads_in_pinocchio():
    model, _ = load_pinocchio()
    assert model.nq == 23 and model.nv == 22


def test_sixteen_actuators_identified():
    mjm, _ = load_mujoco()
    act = actuator_order_names(mjm)
    assert len(act) == 16
    # expected SDK order (spec §11)
    assert act == [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "FR_wheel_joint", "FL_wheel_joint", "RR_wheel_joint", "RL_wheel_joint",
    ]


def test_four_wheels_identified():
    mjm, _ = load_mujoco()
    joints = joint_order_names(mjm)
    wheels = [n for n in joints if "wheel" in n]
    assert len(wheels) == 4
    assert set(wheels) == {
        "FR_wheel_joint", "FL_wheel_joint", "RR_wheel_joint", "RL_wheel_joint"
    }


def test_mujoco_and_pinocchio_joint_order_agree():
    mjm, _ = load_mujoco()
    model, _ = load_pinocchio()
    assert joint_order_names(mjm) == pin_joint_order_names(model)
