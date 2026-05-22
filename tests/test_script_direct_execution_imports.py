from __future__ import annotations

import subprocess
from pathlib import Path

SCRIPTS = [
    "run_workcell_builder_scene3d_gui_smoke.py",
    "run_workcell_studio_scene_readiness_gate.py",
    "run_workcell_studio_local_validation.py",
    "validate_scene3d_runtime_acceptance.py",
    "check_scene3d_canvas_contract.py",
]


def test_scripts_support_direct_execution_help_without_import_errors():
    repo_root = Path(__file__).resolve().parents[1]
    for script in SCRIPTS:
        proc = subprocess.run(
            ["python3", str(repo_root / "scripts" / script), "--help"],
            cwd=repo_root,
            text=True,
            capture_output=True,
        )
        assert proc.returncode == 0, f"{script} failed: stdout={proc.stdout}\nstderr={proc.stderr}"
        combined = f"{proc.stdout}\n{proc.stderr}"
        assert "No module named 'scripts.workcell_studio_path_resolver'" not in combined
        assert "ModuleNotFoundError" not in combined


def test_runtime_scripts_do_not_hardcode_workspace_paths():
    repo_root = Path(__file__).resolve().parents[1]
    forbidden = ["/root/workcell_ws", "~/workcell_ws"]
    for script in SCRIPTS:
        text = (repo_root / "scripts" / script).read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in text, f"{script} contains forbidden path token: {token}"
