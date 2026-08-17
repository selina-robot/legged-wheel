#!/usr/bin/env python3
"""Collect LQR identification data (spec §26).

At the rear-wheel equilibrium with fixed leg impedance, excite the average
rear-wheel torque with pulses of +-{0.5, 1.0, 1.5} Nm for {20, 50, 80} ms,
optionally from an initial pitch perturbation of +-{0.5, 1, 2} deg, and log
theta, theta_dot, s, s_dot, u at 500 Hz.

Output: data/identification/identification_data.csv
  (columns: episode, split, t, pitch, pitch_rate, wheel_disp, wheel_vel, u;
   states are logged pre-step so (row k, row k+1) is a one-step transition
   with u of row k)

Usage: python python/lqr/collect_identification_data.py
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lqr.balance_plant import BalancePlant, run_episode  # noqa: E402

AMPS_NM = [0.5, 1.0, 1.5]          # spec §26
DURATIONS_MS = [20, 50, 80]        # within 20-80 ms
PITCH_PERT_DEG = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]
PULSE_DELAY_S = 0.05               # settle before the pulse
EPISODE_MAX_S = 0.6

OUT_CSV = "data/identification/identification_data.csv"


def main():
    plant = BalancePlant()
    plant.u_limit = 15.0  # identification excitation must not clip

    all_rows = []
    episode = 0
    for pert_deg in PITCH_PERT_DEG:
        for amp in AMPS_NM:
            for sign in (+1, -1):
                for dur_ms in DURATIONS_MS:
                    dur = dur_ms / 1000.0
                    plant.reset(pitch_offset_rad=np.deg2rad(pert_deg))
                    u_fun = (lambda t, a=sign * amp, d=dur:
                             a if PULSE_DELAY_S <= t < PULSE_DELAY_S + d else 0.0)
                    rows = run_episode(plant, EPISODE_MAX_S, use_lqr=False,
                                       u_override=u_fun, log_rate_hz=500)
                    # train/validation split by episode index (interleaved)
                    split = "validation" if episode % 5 == 4 else "train"
                    for r in rows:
                        r["episode"] = episode
                        r["split"] = split
                        r["amp"] = sign * amp
                        r["dur_ms"] = dur_ms
                        r["pitch_pert_deg"] = pert_deg
                    all_rows.extend(rows)
                    episode += 1

    df = pd.DataFrame(all_rows)
    out = Path(OUT_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"[collect] wrote {out}: {len(df)} rows, {episode} episodes, "
          f"episodes that fell early: {df.groupby('episode').size().lt(200).sum()}")


if __name__ == "__main__":
    main()
