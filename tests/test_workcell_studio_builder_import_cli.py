from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CLI = REPO_ROOT / "scripts" / "workcell_studio.py"


def _write_scene(root: Path, metadata: str = "{}") -> None:
    (root / "package.xml").write_text("<package/>", encoding="utf-8")
    (root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (root / "environment.yaml").write_text(
        "robot: {name: ur5}\nend_effector: {name: robotiq_2f}\nobjects:\n  box1:\n    filepath: meshes/box1.stl\n    dimensions: [0.2, 0.2, 0.1]\n",
        encoding="utf-8",
    )
    (root / "workcell_builder_metadata.yaml").write_text(metadata, encoding="utf-8")


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["python3", str(CLI), *args], capture_output=True, text=True, check=False)


def test_import_reuses_existing_exports_and_writes_summaries() -> None:
    with tempfile.TemporaryDirectory() as d:
        scene = Path(d) / "scene"
        scene.mkdir()
        _write_scene(scene)
        generated = scene / "generated"
        generated.mkdir()
        (generated / "cell_definition.yaml").write_text((REPO_ROOT / "tests/fixtures/cell_definition_pick_place.yaml").read_text(encoding="utf-8"), encoding="utf-8")
        (generated / "environment_layout.yaml").write_text((REPO_ROOT / "tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml").read_text(encoding="utf-8"), encoding="utf-8")

        out = Path(d) / "out"
        proc = _run("import-builder-scene", "--scene-package", str(scene), "--output-dir", str(out), "--project-name", "demo", "--validate", "--generate-project")
        assert proc.returncode == 0, proc.stdout + proc.stderr

        summary = json.loads((out / "workcell_studio_import_summary.json").read_text(encoding="utf-8"))
        assert (out / "workcell_studio_import_summary.md").is_file()
        assert summary["validation"]["builder_scene"]
        assert summary["validation"]["cell_definition"]
        assert summary["validation"]["environment_layout"]
        assert summary["generated_project_path"]


def test_import_auto_exports_missing_generated_files() -> None:
    with tempfile.TemporaryDirectory() as d:
        scene = Path(d) / "scene"
        scene.mkdir()
        _write_scene(scene)
        out = Path(d) / "out"

        proc = _run("import-builder-scene", "--scene-package", str(scene), "--output-dir", str(out), "--project-name", "demo", "--validate")
        assert proc.returncode == 0, proc.stdout + proc.stderr
        assert (scene / "generated" / "cell_definition.yaml").is_file()
        assert (scene / "generated" / "environment_layout.yaml").is_file()
        summary = json.loads((out / "workcell_studio_import_summary.json").read_text(encoding="utf-8"))
        assert summary["export_status"] == "generated"


def test_preview_only_readiness_is_not_hard_failure() -> None:
    with tempfile.TemporaryDirectory() as d:
        scene = Path(d) / "scene"
        scene.mkdir()
        _write_scene(scene, '{"preview_only":true,"fake_hardware_ready":true}')
        out = Path(d) / "out"
        proc = _run("import-builder-scene", "--scene-package", str(scene), "--output-dir", str(out), "--project-name", "demo", "--validate")
        assert proc.returncode == 0, proc.stdout + proc.stderr
        summary = json.loads((out / "workcell_studio_import_summary.json").read_text(encoding="utf-8"))
        assert summary["safety_status"]["runtime_status"] in {"preview_only", "fake_hardware_ready", "runtime_ready"}


def test_missing_scene_package_fails() -> None:
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "out"
        proc = _run("import-builder-scene", "--scene-package", str(Path(d) / "missing"), "--output-dir", str(out), "--project-name", "demo", "--validate")
        assert proc.returncode != 0
        assert "does not exist" in (proc.stdout + proc.stderr)
