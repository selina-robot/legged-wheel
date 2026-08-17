#!/usr/bin/env python3
"""Re-runnable Gate 1 joint-map verification (spec §12, §74).

Cross-checks three sources against each other:
  1. config/robot.yaml            (explicit SDK index -> joint name mapping)
  2. the MJCF actuator order      (what the sim bridge actually drives)
  3. artifacts/reports/model_audit/joint_map.csv  (Gate 1 smoke test output)

Also checks axes against the audited model.json when present.

Usage: python python/model/verify_joint_map.py
Exit 0 iff everything agrees; the Gate-1 measured columns
(positive_direction, wheel_forward_sign) are reported but only checked for
presence/validity (they were validated by the smoke test itself).
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.config import load_robot_config, repo_path  # noqa: E402
from common.model import actuator_order_names, load_mujoco  # noqa: E402

CSV_PATH = repo_path("artifacts/reports/model_audit/joint_map.csv")
AUDIT_JSON = repo_path("artifacts/reports/model_audit/model.json")

EXPECTED_HEADER = [
    "sdk_index", "canonical_name", "mjcf_name", "axis",
    "positive_direction", "wheel_forward_sign",
]


def fail(msg):
    print(f"[verify_joint_map] FAIL: {msg}")
    sys.exit(1)


def main():
    robot_cfg = load_robot_config()
    jm = robot_cfg["joint_map"]
    sdk_names = [jm[i]["name"] for i in range(16)]

    # --- robot.yaml vs MJCF actuator order ---
    mjm, _ = load_mujoco()
    act = actuator_order_names(mjm)
    if act != sdk_names:
        fail(f"robot.yaml SDK order {sdk_names} != MJCF actuator order {act}")
    print("[verify_joint_map] robot.yaml joint_map == MJCF actuator order: OK")

    # --- axes vs audit ---
    if AUDIT_JSON.exists():
        import json
        audit = json.load(open(AUDIT_JSON))
        audit_axes = {j["name"]: j["axis"] for j in audit["joints"]}
        for i in range(16):
            yaml_axis = [float(x) for x in jm[i]["axis"].split()]
            mjcf_axis = audit_axes[jm[i]["name"]]
            if not all(abs(a - b) < 1e-12 for a, b in zip(yaml_axis, mjcf_axis)):
                fail(f"axis mismatch at sdk {i}: yaml {yaml_axis} vs mjcf {mjcf_axis}")
        print("[verify_joint_map] joint axes match audited MJCF: OK")
    else:
        print("[verify_joint_map] WARN: model.json missing, axis check skipped")

    # --- Gate 1 CSV ---
    if not CSV_PATH.exists():
        print(f"[verify_joint_map] {CSV_PATH} missing; run Gate 1 smoke test first")
        fail("joint_map.csv missing")
    with open(CSV_PATH) as f:
        rows = list(csv.reader(f))
    if rows[0] != EXPECTED_HEADER:
        fail(f"CSV header {rows[0]} != expected {EXPECTED_HEADER}")
    if len(rows) != 17:
        fail(f"expected 16 data rows, got {len(rows) - 1}")

    wheel_signs = {}
    for row in rows[1:]:
        idx, name, mjcf, axis, pos_dir, wfwd = int(row[0]), row[1], row[2], row[3], row[4], row[5]
        if name != sdk_names[idx] or mjcf != sdk_names[idx]:
            fail(f"CSV row {idx}: name {name}/{mjcf} != robot.yaml {sdk_names[idx]}")
        if int(pos_dir) not in (1, -1):
            fail(f"CSV row {idx}: positive_direction={pos_dir} undetermined")
        if "wheel" in name:
            if int(wfwd) not in (1, -1):
                fail(f"CSV row {idx} ({name}): wheel_forward_sign undetermined")
            leg = name.split("_")[0]
            wheel_signs[leg] = int(wfwd)
            if int(wfwd) != robot_cfg["wheel_forward_sign"][leg]:
                fail(f"{name}: CSV wfs {wfwd} != robot.yaml {robot_cfg['wheel_forward_sign'][leg]}")

    print(f"[verify_joint_map] joint_map.csv consistent with robot.yaml: OK")
    print(f"[verify_joint_map] measured wheel_forward_sign: {wheel_signs}")
    print("[verify_joint_map] PASS")


if __name__ == "__main__":
    main()
