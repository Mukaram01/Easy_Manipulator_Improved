from __future__ import annotations

import subprocess
import sys
from pathlib import Path


SCRIPT = Path("scripts/validate_scene_builder_full_contract.py")


def test_runner_exists_and_references_required_validators():
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    for token in [
        "validate_scene_builder_mainline_flow.py",
        "validate_scene_builder_ui_acceptance.py",
        "validate_scene3d_mesh_preview_contract.py",
        "validate_scene_builder_canvas_generated_parity.py",
    ]:
        assert token in text


def test_runner_reports_combined_outcomes(tmp_path: Path):
    pass_script = tmp_path / "pass_validator.py"
    fail_script = tmp_path / "fail_validator.py"
    pass_script.write_text("print('PASS validator ran')\n", encoding="utf-8")
    fail_script.write_text("import sys\nprint('FAIL validator ran')\nsys.exit(2)\n", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--validators",
            str(pass_script),
            str(fail_script),
        ],
        cwd=Path.cwd(),
        capture_output=True,
        text=True,
    )

    out = result.stdout
    assert result.returncode != 0
    assert "PASS validator ran" in out
    assert "FAIL validator ran" in out
    assert "Overall: FAIL" in out
