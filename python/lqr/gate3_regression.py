#!/usr/bin/env python3
"""Gate 3 regression (spec §30): nominal 20 s hold + initial pitch offsets.

Per level (1, 3, 5 deg), 20 episodes with the initial pitch offset sampled
uniformly in [-level, +level]. Success per episode: 20 s without falling,
wheel travel <= 0.5 m, continuous saturation < 100 ms, no leg joint limit
violation (checked at every step against the audited ranges).

Gate: 3deg >= 20/20, 5deg >= 19/20 (spec §30).

Output: data/identification/../capture_basin/../gate3/gate3_report.json +
per-episode logs under artifacts/logs/gate3/.

Usage: python python/lqr/gate3_regression.py
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, repo_path  # noqa: E402
from lqr.balance_plant import (  # noqa: E402
    BalancePlant,
    run_episode,
    WHEEL_TRAVEL_MAX_M,
    SAT_CONTINUOUS_MAX_S,
)

LEVELS_DEG = [1.0, 3.0, 5.0]
EPISODES = 20
HOLD_S = 20.0

OUT_JSON = repo_path("data/capture_basin/gate3_report.json")
LOG_DIR = repo_path("artifacts/logs/gate3")


def main() -> int:
    robot_cfg = load_robot_config()
    cfg = yaml.safe_load(open(repo_path("config/lqr.yaml")))
    jr = robot_cfg["model_params"]["joint_range"]
    leg_limits = {}
    for i in range(12):
        name = robot_cfg["joint_map"][i]["name"]
        t = robot_cfg["joint_map"][i]["type"]
        leg_limits[name] = (jr["hip"] if t == "hip" else
                            jr["thigh_front" if name.startswith("F") else "thigh_back"]
                            if t == "thigh" else jr["calf"])

    report = {}
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    for level in LEVELS_DEG:
        ok_count = 0
        details = []
        for ep in range(EPISODES):
            # deterministic but varying offsets in [-level, +level]
            frac = (ep + 0.5) / EPISODES  # in (0,1)
            off_deg = level * (2 * frac - 1)
            p = BalancePlant()
            p.set_lqr(cfg)
            p.reset(pitch_offset_rad=np.deg2rad(off_deg))
            rows = run_episode(p, HOLD_S, use_lqr=True,
                               limit_ranges=leg_limits)
            last = rows[-1]
            fell = p.is_fallen()
            travel = abs(last["wheel_travel"])
            sat = last["sat_continuous_max"]
            limit_viol = last["limit_violations"]

            success = (not fell) and travel <= WHEEL_TRAVEL_MAX_M \
                and sat < SAT_CONTINUOUS_MAX_S and limit_viol == 0
            ok_count += int(success)
            details.append({
                "episode": ep, "offset_deg": off_deg, "success": success,
                "t_end": last["t"], "wheel_travel": travel,
                "sat_continuous_max": sat, "limit_violations": limit_viol,
            })
            pd.DataFrame(rows).to_csv(
                LOG_DIR / f"level{int(level)}_ep{ep:02d}.csv", index=False)

        report[f"{level}deg"] = {
            "success": ok_count, "episodes": EPISODES, "details": details,
        }
        print(f"[gate3] +/-{level} deg: {ok_count}/{EPISODES}")

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(report, indent=2) + "\n")

    nominal = report["1.0deg"]["success"]
    mid = report["3.0deg"]["success"]
    far = report["5.0deg"]["success"]
    gate = mid >= 20 and far >= 19
    print(f"[gate3] summary: 1deg {nominal}/20, 3deg {mid}/20, 5deg {far}/20 "
          f"-> {'PASS' if gate else 'FAIL'}")
    print(f"[gate3] wrote {OUT_JSON}")
    return 0 if gate else 1


if __name__ == "__main__":
    sys.exit(main())
