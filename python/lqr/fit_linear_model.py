#!/usr/bin/env python3
"""Fit the discrete linear balance model (spec §27): x_{k+1} = A x_k + B u_k
by least squares over the identification data, with a train/validation split.

Criterion: validation one-step prediction normalized RMSE < 10% per state
component (normalized by config/lqr.yaml state_scale). If it fails, check
state sign / wheel sign / equilibrium / leg posture movement / time
alignment / sample period first (never tune Q/R to compensate).

Output: data/identification/linear_model.npz (A, B, x_eq, dt, metrics)

Usage: python python/lqr/fit_linear_model.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_yaml, repo_path  # noqa: E402

IN_CSV = repo_path("data/identification/identification_data.csv")
OUT_NPZ = repo_path("data/identification/linear_model.npz")
OUT_METRICS = repo_path("data/identification/fit_metrics.json")


def main() -> int:
    df = pd.read_csv(IN_CSV)
    lqr_cfg = load_yaml("config/lqr.yaml")
    scale = np.array([
        lqr_cfg["state_scale"]["pitch_rad"],
        lqr_cfg["state_scale"]["pitch_rate"],
        lqr_cfg["state_scale"]["wheel_displacement_m"],
        lqr_cfg["state_scale"]["wheel_velocity_mps"],
    ])
    theta_eq = float(yaml.safe_load(open(
        repo_path("data/equilibrium/rear_equilibrium.yaml")))["base_pose"]["pitch"])

    # one-step transitions within each episode: (x_k, u_k) -> x_{k+1}
    x_cols = ["pitch", "pitch_rate", "wheel_disp", "wheel_vel"]
    xs, us, ys = [], [], []
    dt = None
    for ep, g in df.groupby("episode", sort=False):
        g = g.reset_index(drop=True)
        if len(g) < 3:
            continue
        dt = float(np.median(np.diff(g["t"])))
        X = g[x_cols].to_numpy()
        X[:, 0] -= theta_eq
        U = g["u"].to_numpy()
        for k in range(len(g) - 1):
            xs.append(np.concatenate([X[k], [U[k]]]))
            ys.append(X[k + 1])
    X5 = np.array(xs)
    Y = np.array(ys)
    split = df.drop_duplicates("episode")[["episode", "split"]]
    n_val_eps = (split["split"] == "validation").sum()

    # rebuild the split mask per transition (episode of the transition)
    trans_split = []
    for ep, g in df.groupby("episode", sort=False):
        s = g["split"].iloc[0]
        trans_split += [s] * (len(g) - 1)
    trans_split = np.array(trans_split)

    tr = trans_split == "train"
    va = trans_split == "validation"

    # least squares per state row: Y = X5 @ W, W = [A B].T
    W, *_ = np.linalg.lstsq(X5[tr], Y[tr], rcond=None)
    A = W[:4, :].T
    B = W[4:, :].T

    # Physical structure constraint (diagnosed in Gate-3 debugging): on flat
    # ground with round wheels, the pitch dynamics are invariant to the wheel
    # displacement s. The unconstrained fit puts small spurious couplings in
    # A[1,2] (s -> pitch_rate) and A[3,0] (pitch -> wheel_vel); an LQR
    # designed on them closes a slow, divergent theta<->s loop on the real
    # plant. Zero them and re-fit the affected rows by least squares.
    for row, zero_cols in ((1, [2]), (3, [0])):
        keep = [c for c in range(5) if c not in zero_cols]
        w, *_ = np.linalg.lstsq(X5[tr][:, keep], Y[tr][:, row], rcond=None)
        A[row, :] = 0.0
        B[row, :] = 0.0
        for c, k in zip(keep, w):
            if c < 4:
                A[row, c] = k
            else:
                B[row, 0] = k

    # validation one-step prediction
    pred = X5[va] @ W
    err = (pred - Y[va]) / scale
    rmse = np.sqrt((err**2).mean(axis=0))
    rmse_max = float(rmse.max())

    metrics = {
        "dt": dt,
        "n_train": int(tr.sum()),
        "n_validation": int(va.sum()),
        "validation_episodes": int(n_val_eps),
        "validation_one_step_rmse_normalized": {
            c: float(e) for c, e in zip(x_cols, rmse)
        },
        "rmse_max": rmse_max,
        "pass": bool(rmse_max < 0.10),
        "A": A.tolist(),
        "B": B.ravel().tolist(),
    }
    np.savez(OUT_NPZ, A=A, B=B, dt=dt, theta_eq=theta_eq, scale=scale)
    OUT_METRICS.write_text(json.dumps(metrics, indent=2) + "\n")

    print(f"[fit] dt={dt*1000:.2f} ms, train={tr.sum()} val={va.sum()} transitions")
    print("[fit] A =")
    print(np.array2string(A, precision=4, suppress_small=True))
    print(f"[fit] B = {B.ravel()}")
    print(f"[fit] validation one-step normalized RMSE per component: "
          f"{dict(zip(x_cols, np.round(rmse, 4)))}")
    print(f"[fit] max RMSE {rmse_max:.4f} (< 0.10 required) -> "
          f"{'PASS' if rmse_max < 0.10 else 'FAIL'}")
    return 0 if rmse_max < 0.10 else 1


if __name__ == "__main__":
    sys.exit(main())
