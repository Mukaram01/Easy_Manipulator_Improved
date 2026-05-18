from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_generated_draft_fixture_passes_cell_definition_validator() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    validator = repo_root / "scripts" / "validate_cell_definition.py"
    draft = repo_root / "tests" / "fixtures" / "cell_definition_ur5_suction_conveyor_placeholder.yaml"

    result = subprocess.run(
        [sys.executable, str(validator), str(draft)],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, (
        "Expected generated draft shape to validate as cell_definition/v1.\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
