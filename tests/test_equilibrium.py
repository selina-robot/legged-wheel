"""Gate 2 test (spec §74): the rear-wheel equilibrium must exist and pass
every Gate 2 criterion (spec §22).

The test re-solves the equilibrium (IPOPT, a few seconds) and then runs the
independent validator, which includes the MuJoCo statics cross-check.
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT / "python" / "equilibrium"))

import solve_rear_equilibrium  # noqa: E402
import validate_equilibrium  # noqa: E402


def test_rear_equilibrium_gate2():
    assert solve_rear_equilibrium.main() == 0, "IPOPT did not succeed"
    assert validate_equilibrium.main() == 0, "Gate 2 criteria failed"
