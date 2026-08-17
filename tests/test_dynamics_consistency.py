"""Dynamics consistency test (spec §74): gravity torque relative error
between MuJoCo and Pinocchio must be < 3% over the 10 reference poses.

The remaining quantities (total mass, CoM, mass matrix, RNEA parts) must
match to machine precision: both sides load the same MJCF, so any real
discrepancy is a mapping/convention bug, not physics.
"""
from model.compare_mujoco_pinocchio import compare

SUMMARY = compare(num_poses=10, seed=1, verbose=True)


def test_gravity_torque_relative_error_below_3_percent():
    assert SUMMARY["gravity_rel_err_max"] < 0.03, SUMMARY["gravity_rel_err_all"]


def test_total_mass_matches():
    assert SUMMARY["total_mass_abs_err"] < 1e-6


def test_com_matches():
    assert SUMMARY["com_abs_err_mm_max"] < 1e-3


def test_mass_matrix_matches():
    assert SUMMARY["mass_matrix_rel_err_max"] < 1e-6


def test_rnea_parts_match():
    # inertial and bias parts must match to machine precision once the
    # free-flyer frame correction is applied (see compare script comments)
    assert SUMMARY["rne_inertial_rel_err_max"] < 1e-6
    assert SUMMARY["rne_bias_rel_err_max"] < 1e-6
