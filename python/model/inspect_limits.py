#!/usr/bin/env python3
"""Print/cross-check joint limits and actuator ctrlranges (for trajopt/WBC).

Reads the audited model.json (spec §13) and config/robot.yaml, and prints one
table in SDK order: range, torque limit, margin of q_init to the range.

Usage: python python/model/inspect_limits.py [--check]
  --check: exit nonzero if q_init violates any range or the audit is stale.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, repo_path  # noqa: E402

AUDIT_JSON = repo_path("artifacts/reports/model_audit/model.json")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    if not AUDIT_JSON.exists():
        print("model.json missing; run python/model/audit_mjcf.py first")
        return 1
    audit = json.load(open(AUDIT_JSON))
    robot_cfg = load_robot_config()

    rng = {j["name"]: (j["range"], j["limited"]) for j in audit["joints"]}
    trq = {a["joint"]: abs(a["ctrlrange"][1]) for a in audit["actuators"]}
    q_init = np.array(robot_cfg["q_init"], dtype=float)

    print(f"{'idx':>3} {'joint':<16} {'range':>24} {'|tau|max':>8} {'q_init':>8} {'margin':>8}")
    ok = True
    for i in range(16):
        name = robot_cfg["joint_map"][i]["name"]
        (lo, hi), limited = rng[name]
        tmax = trq[name]
        q0 = q_init[i]
        if limited:
            margin = min(q0 - lo, hi - q0)
            viol = not (lo - 1e-9 <= q0 <= hi + 1e-9)
            ok &= not viol
            print(f"{i:>3} {name:<16} [{lo:>9.4f},{hi:>9.4f}] {tmax:>8.2f} {q0:>8.3f} {margin:>8.3f}"
                  + ("  <-- OUT OF RANGE" if viol else ""))
        else:
            print(f"{i:>3} {name:<16} {'(unlimited)':>24} {tmax:>8.2f} {q0:>8.3f} {'--':>8}")

    # audit freshness: robot.yaml model_params must match the audit
    mp = robot_cfg.get("model_params") or {}
    for key, val in (("total_mass", audit["total_mass"]), ("wheel_radius", audit["wheel_radius"])):
        if not np.isclose(mp.get(key, np.nan), val, rtol=1e-6):
            print(f"stale model_params.{key}: yaml={mp.get(key)} audit={val}")
            ok = False
    print("inspect_limits: OK" if ok else "inspect_limits: FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
