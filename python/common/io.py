"""Small IO helpers for python tooling (artifacts output)."""
import json
from pathlib import Path

import numpy as np

from .config import repo_path


def ensure_dir(path) -> Path:
    p = Path(path)
    if not p.is_absolute():
        p = repo_path(str(p))
    p.mkdir(parents=True, exist_ok=True)
    return p


class _Encoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        return super().default(o)


def write_json(rel_path: str, payload) -> Path:
    p = repo_path(rel_path)
    ensure_dir(p.parent)
    with open(p, "w") as f:
        json.dump(payload, f, indent=2, cls=_Encoder)
        f.write("\n")
    return p
