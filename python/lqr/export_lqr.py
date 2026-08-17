"""Export the identified model and LQR gain into config/lqr.yaml.

Replaces the A/B/K/x_eq/u_limit entries in place (comments preserved); all
other content of the file is untouched.

Called by design_lqr.py; also usable standalone:
  python -m lqr.export_lqr  (re-exports from data/identification/linear_model.npz
  with the current lqr.yaml weights via design_lqr)
"""
from pathlib import Path

from common.config import repo_path

LQR_YAML = repo_path("config/lqr.yaml")


def export_lqr(A, B, K, x_eq, u_limit):
    def fmt_mat(M):
        return "\n" + "\n".join(
            "  - [" + ", ".join(f"{v:.8g}" for v in row) + "]" for row in M)

    def fmt_vec(v):
        return "[" + ", ".join(f"{x:.8g}" for x in v) + "]"

    text = LQR_YAML.read_text()
    lines = text.splitlines()
    repl = {
        "A:": "A:" + fmt_mat(A),
        "B:": "B:" + fmt_mat(B),
        "K:": "K:" + fmt_mat(K),
        "x_eq:": "x_eq: " + fmt_vec(x_eq),
        "u_limit:": f"u_limit: {u_limit:.8g}",
    }
    out = []
    skip = False
    for line in lines:
        top_level = line and not line.startswith((" ", "-"))
        key = line.split()[0] if line.split() else None
        if top_level and key in repl:
            out.append(repl[key])
            skip = True   # drop the old value's continuation lines
            continue
        if skip and line.startswith("  -"):
            continue
        skip = False
        out.append(line)
    LQR_YAML.write_text("\n".join(out) + "\n")
