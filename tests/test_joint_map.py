# Gate 1 config test (spec §74): the explicit joint map in config/robot.yaml
# must define 16 unique joints, no duplicates, in the expected SDK index
# sequence (spec §11).
from pathlib import Path

import yaml

ROBOT_YAML = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"

# Expected SDK order (spec §11): FR/FL/RR/RL x hip/thigh/calf, then wheels.
EXPECTED_ORDER = [
    (0, "FR_hip_joint", "FR", "hip"),
    (1, "FR_thigh_joint", "FR", "thigh"),
    (2, "FR_calf_joint", "FR", "calf"),
    (3, "FL_hip_joint", "FL", "hip"),
    (4, "FL_thigh_joint", "FL", "thigh"),
    (5, "FL_calf_joint", "FL", "calf"),
    (6, "RR_hip_joint", "RR", "hip"),
    (7, "RR_thigh_joint", "RR", "thigh"),
    (8, "RR_calf_joint", "RR", "calf"),
    (9, "RL_hip_joint", "RL", "hip"),
    (10, "RL_thigh_joint", "RL", "thigh"),
    (11, "RL_calf_joint", "RL", "calf"),
    (12, "FR_wheel_joint", "FR", "wheel"),
    (13, "FL_wheel_joint", "FL", "wheel"),
    (14, "RR_wheel_joint", "RR", "wheel"),
    (15, "RL_wheel_joint", "RL", "wheel"),
]


def load_cfg():
    with open(ROBOT_YAML) as f:
        return yaml.safe_load(f)


def test_sixteen_unique_mapping():
    jm = load_cfg()["joint_map"]
    assert len(jm) == 16, f"expected 16 entries, got {len(jm)}"
    indices = sorted(int(k) for k in jm)
    assert indices == list(range(16)), f"SDK indices must be 0..15: {indices}"
    names = [v["name"] for _, v in sorted(jm.items())]
    assert len(set(names)) == 16, "canonical names must be unique"


def test_no_duplicate_legs_types():
    jm = load_cfg()["joint_map"]
    combos = [(v["leg"], v["type"]) for _, v in sorted(jm.items())]
    assert len(set(combos)) == 16, "duplicate (leg, type) combination"


def test_expected_sdk_index_sequence():
    jm = load_cfg()["joint_map"]
    for idx, name, leg, jtype in EXPECTED_ORDER:
        entry = jm[idx]
        assert entry["name"] == name, f"sdk {idx}: {entry['name']} != {name}"
        assert entry["leg"] == leg
        assert entry["type"] == jtype


def test_wheel_forward_sign_is_signed():
    wfs = load_cfg()["wheel_forward_sign"]
    assert set(wfs) == {"FR", "FL", "RR", "RL"}
    for leg, s in wfs.items():
        assert s in (1, -1), f"wheel_forward_sign.{leg} must be +/-1, got {s}"


def test_joint_map_csv_consistent_if_present():
    """If Gate 1 smoke test already produced joint_map.csv, it must agree
    with config/robot.yaml (names, order, and wheel signs)."""
    csv_path = (
        Path(__file__).resolve().parents[1]
        / "artifacts" / "reports" / "model_audit" / "joint_map.csv"
    )
    if not csv_path.exists():
        return  # CSV is produced by the smoke test, not a config precondition
    cfg = load_cfg()
    jm = cfg["joint_map"]
    lines = csv_path.read_text().strip().splitlines()
    assert lines[0].split(",") == [
        "sdk_index", "canonical_name", "mjcf_name", "axis",
        "positive_direction", "wheel_forward_sign",
    ]
    assert len(lines) == 17, f"expected 16 data rows, got {len(lines) - 1}"
    for row in lines[1:]:
        idx_s, name, mjcf, axis, pos_dir, wfwd = row.split(",")
        idx = int(idx_s)
        assert jm[idx]["name"] == name
        if jm[idx]["type"] == "wheel":
            leg = jm[idx]["leg"]
            assert int(wfwd) == cfg["wheel_forward_sign"][leg], (
                f"{name}: CSV wheel_forward_sign {wfwd} disagrees with "
                f"robot.yaml {cfg['wheel_forward_sign'][leg]}; update "
                "config/robot.yaml with the verified value"
            )
