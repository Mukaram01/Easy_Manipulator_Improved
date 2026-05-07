from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def test_generate_studio_pack_created(tmp_path: Path) -> None:
    scene_src = Path("scenes/ur5_2f_builder_pick_place_demo")
    scene = tmp_path / "scene"
    shutil.copytree(scene_src, scene)

    out = tmp_path / "studio_out"
    run = subprocess.run([
        sys.executable,
        "scripts/workcell_studio.py",
        "import-builder-scene",
        "--scene-package",
        str(scene),
        "--output-dir",
        str(out),
        "--validate",
        "--project-name",
        "studio_pack_demo",
    ], capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stdout + run.stderr

    summary = json.loads((out / "workcell_studio_import_summary.json").read_text(encoding="utf-8"))
    pack = Path(summary["preview_bundle_dir"])
    assert pack.exists()

    expected = {
        "cell_definition.yaml",
        "environment_layout.yaml",
        "selected_assets.json",
        "builder_export_summary.json",
        "compatibility_report.json",
        "readiness_summary.md",
        "environment_preview.svg",
        "environment_preview.html",
        "generated_launch_commands.md",
    }
    assert expected.issubset({p.name for p in pack.iterdir()})

    selected_assets = json.loads((pack / "selected_assets.json").read_text(encoding="utf-8"))
    assert "custom_stls" in selected_assets

    readiness = (pack / "readiness_summary.md").read_text(encoding="utf-8")
    assert "offline/fake hardware first" in readiness.lower()
    assert "moveit services were executed" in readiness.lower()

    compatibility = json.loads((pack / "compatibility_report.json").read_text(encoding="utf-8"))
    assert compatibility["fake_hardware_default"] is True
