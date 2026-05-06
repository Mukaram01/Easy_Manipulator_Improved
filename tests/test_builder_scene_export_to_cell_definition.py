from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_scene(root: Path, robot: str, ee: str, metadata: str) -> None:
    (root / "package.xml").write_text("<package/>", encoding="utf-8")
    (root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (root / "environment.yaml").write_text(
        f"robot: {{name: {robot}}}\nend_effector: {{name: {ee}}}\nobjects:\n  box1:\n    filepath: meshes/box1.stl\n    dimensions: [0.2, 0.2, 0.1]\n",
        encoding="utf-8",
    )
    (root / "workcell_builder_metadata.yaml").write_text(metadata, encoding="utf-8")


def test_export_script_generates_files_and_validation_payloads() -> None:
    with tempfile.TemporaryDirectory() as d:
        root = Path(d) / "scene"
        root.mkdir()
        _write_scene(
            root,
            "ur5",
            "robotiq_2f",
            '{"robot":{"capability_id":"ur5"},"end_effector":{"capability_id":"robotiq_2f_85","family":"finger"},"grasp_strategy":{"strategy_id":"finger_pinch_basic"}}',
        )
        subprocess.run(
            ["python3", str(REPO_ROOT / "scripts/export_builder_scene_to_cell_definition.py"), str(root), "--output-dir", str(root / "generated"), "--validate"],
            check=True,
        )
        assert (root / "generated/cell_definition.yaml").is_file()
        assert (root / "generated/environment_layout.yaml").is_file()
        summary = json.loads((root / "generated/builder_export_summary.json").read_text(encoding="utf-8"))
        assert summary["generated_by"] == "workcell_builder"
        assert summary["validation"]["cell_definition"]["result"] in {"PASS", "WARN", "FAIL"}
        assert summary["validation"]["environment_layout"]["result"] in {"PASS", "WARN"}


def test_builder_validator_warns_for_legacy_scene_without_exports() -> None:
    with tempfile.TemporaryDirectory() as d:
        root = Path(d) / "scene"
        root.mkdir()
        _write_scene(root, "ur5", "suction", "{}")
        out = subprocess.run(
            ["python3", str(REPO_ROOT / "scripts/validate_builder_generated_scene.py"), str(root), "--json"],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(out.stdout)
        assert payload["ok"] is True
        assert any("legacy scenes" in w for w in payload["warnings"])


def test_preview_only_metadata_emits_warning() -> None:
    with tempfile.TemporaryDirectory() as d:
        root = Path(d) / "scene"
        root.mkdir()
        _write_scene(
            root,
            "generic delta",
            "suction",
            '{"robot":{"capability_id":"generic_delta_900","preview_only":true},"end_effector":{"capability_id":"onrobot_airpick_style","family":"suction"},"grasp_strategy":{"strategy_id":"suction_top_basic"}}',
        )
        subprocess.run(["python3", str(REPO_ROOT / "scripts/export_builder_scene_to_cell_definition.py"), str(root), "--output-dir", str(root / "generated")], check=True)
        summary = json.loads((root / "generated/builder_export_summary.json").read_text(encoding="utf-8"))
        assert any("preview_only" in w for w in summary["warnings"])
