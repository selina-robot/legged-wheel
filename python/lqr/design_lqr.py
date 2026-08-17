#!/usr/bin/env python3
"""LQR design (spec §28): normalize the identified model by state_scale,
solve the discrete ARE, export A/B/K/x_eq/u_limit to config/lqr.yaml via
python/lqr/export_lqr.py.

Usage: python python/lqr/design_lqr.py
"""
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, load_yaml, repo_path  # noqa: E402
from lqr.export_lqr import export_lqr  # noqa: E402

NPZ = repo_path("data/identification/linear_model.npz")


def main() -> int:
    data = np.load(NPZ)
    A, B = data["A"], data["B"]
    theta_eq = float(data["theta_eq"])

    lqr_cfg = load_yaml("config/lqr.yaml")
    scale = np.array([
        lqr_cfg["state_scale"]["pitch_rad"],
        lqr_cfg["state_scale"]["pitch_rate"],
        lqr_cfg["state_scale"]["wheel_displacement_m"],
        lqr_cfg["state_scale"]["wheel_velocity_mps"],
    ])
    Q = np.diag(np.array(lqr_cfg["Q"], dtype=float))
    R = np.array(lqr_cfg["R"], dtype=float).reshape(1, 1)

    # normalize: x_n = x / scale -> A_n = S^-1 A S, B_n = S^-1 B
    S = np.diag(scale)
    A_n = np.linalg.solve(S, A) @ S
    B_n = np.linalg.solve(S, B)

    K_n = solve_discrete_are(A_n, B_n, Q, R)
    # DARE returns P; K = (R + B^T P B)^-1 B^T P A
    K_n = np.linalg.solve(R + B_n.T @ K_n @ B_n, B_n.T @ K_n @ A_n)
    K_phys = K_n @ np.linalg.inv(S)  # u = -K_n x_n = -(K_n S^-1) x

    # closed-loop sanity: eigenvalues of A - B K must be inside the unit circle
    eigs = np.linalg.eigvals(A - B @ K_phys)
    max_eig = float(np.max(np.abs(eigs)))

    # saturation: torque_saturation_scale * wheel torque limit (from
    # robot.yaml model_params, never hard-coded)
    robot_cfg = load_robot_config()
    wheel_limit = float(robot_cfg["model_params"]["torque_limit"]["wheel"])
    u_limit = float(lqr_cfg["torque_saturation_scale"]) * wheel_limit

    x_eq = [theta_eq, 0.0, 0.0, 0.0]
    export_lqr(A=A.tolist(), B=[[v] for v in B.ravel()], K=K_phys.tolist(),
               x_eq=x_eq, u_limit=u_limit)
    print(f"[design] K (physical) = {np.round(K_phys, 4)}")
    print(f"[design] closed-loop |eig| max = {max_eig:.4f} (must be < 1)")
    print(f"[design] u_limit = {u_limit:.2f} Nm "
          f"(0.60 x {wheel_limit:.2f} from robot.yaml)")
    print("[design] exported to config/lqr.yaml")
    return 0 if max_eig < 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
