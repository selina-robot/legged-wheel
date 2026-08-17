"""YAML config loading with repo-root path resolution.

All python tools run from the repo root; this module centralizes where the
config files live so the rest never hard-codes paths.
"""
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"


def repo_path(rel: str) -> Path:
    """Absolute path inside the repository."""
    return REPO_ROOT / rel


def load_yaml(path) -> dict:
    path = Path(path)
    if not path.is_absolute():
        path = REPO_ROOT / path
    with open(path) as f:
        return yaml.safe_load(f)


def load_robot_config() -> dict:
    """config/robot.yaml: joint map, wheel signs, audited model params."""
    return load_yaml(CONFIG_DIR / "robot.yaml")


def load_sim_config() -> dict:
    return load_yaml(CONFIG_DIR / "sim.yaml")


def load_control_config() -> dict:
    return load_yaml(CONFIG_DIR / "control.yaml")


def sdk_order_names(robot_cfg: dict) -> list[str]:
    """The 16 canonical joint names in SDK/actuator order (0..15)."""
    jm = robot_cfg["joint_map"]
    return [jm[i]["name"] for i in range(16)]
