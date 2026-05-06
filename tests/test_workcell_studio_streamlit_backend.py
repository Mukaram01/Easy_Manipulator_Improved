from __future__ import annotations

import tempfile
from pathlib import Path

from tools.workcell_studio_streamlit import backend


def _write_scene(root: Path, metadata: str = "{}") -> None:
    (root / "package.xml").write_text("<package/>", encoding="utf-8")
    (root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (root / "environment.yaml").write_text(
        "robot: {name: ur5}\nend_effector: {name: robotiq_2f}\nobjects:\n  box1:\n    filepath: meshes/box1.stl\n    dimensions: [0.2, 0.2, 0.1]\n",
        encoding="utf-8",
    )
    (root / "workcell_builder_metadata.yaml").write_text(metadata, encoding="utf-8")


def test_catalog_loading_discovers_expected_entries() -> None:
    caps = backend.load_capability_catalog()
    grasps = backend.load_grasp_strategy_catalog()

    robot_ids = {item.get("id") for item in caps["robots"]}
    ee_ids = {item.get("id") for item in caps["end_effectors"]}
    sensor_ids = {item.get("id") for item in caps["sensors"]}
    assert "ur5" in robot_ids
    assert any(token in (item or "") for item in ee_ids for token in {"robotiq", "suction", "vacuum"})
    assert any("realsense" in (item or "") for item in sensor_ids)
    assert len(grasps) >= 1


def test_validate_cell_definition_wrapper() -> None:
    fixture = backend.repo_root() / "tests" / "fixtures" / "cell_definition_pick_place.yaml"
    result = backend.validate_cell_definition(fixture)
    assert result["returncode"] == 0
    assert result["json"]["result"] in {"PASS", "WARN"}


def test_validate_environment_layout_wrapper() -> None:
    fixture = backend.repo_root() / "tests" / "fixtures" / "environment_layouts" / "ur5_table_bins_existing_assets.layout.yaml"
    result = backend.validate_environment_layout(fixture)
    assert result["returncode"] == 0
    assert result["json"]["result"] in {"PASS", "WARN"}


def test_import_builder_scene_wrapper_returns_summary_paths() -> None:
    with tempfile.TemporaryDirectory() as d:
        scene = Path(d) / "scene"
        scene.mkdir()
        _write_scene(scene)
        generated = scene / "generated"
        generated.mkdir()
        generated_cell = backend.repo_root() / "tests" / "fixtures" / "cell_definition_pick_place.yaml"
        generated_layout = backend.repo_root() / "tests" / "fixtures" / "environment_layouts" / "ur5_table_bins_existing_assets.layout.yaml"
        (generated / "cell_definition.yaml").write_text(generated_cell.read_text(encoding="utf-8"), encoding="utf-8")
        (generated / "environment_layout.yaml").write_text(generated_layout.read_text(encoding="utf-8"), encoding="utf-8")
        out = Path(d) / "out"

        result = backend.import_builder_scene(scene, out, "demo")
        assert result["returncode"] == 0
        summary = result["summary"]
        assert summary["summary_json_path"]
        assert summary["summary_markdown_path"]


def test_missing_input_handling_returns_structured_error() -> None:
    missing_cell = backend.validate_cell_definition("/tmp/definitely_missing_cell_definition.yaml")
    assert not missing_cell["ok"]
    assert "Missing" in missing_cell["error"]

    missing_scene = backend.import_builder_scene("/tmp/definitely_missing_scene", "/tmp/out", "demo")
    assert not missing_scene["ok"]
    assert "does not exist" in missing_scene["error"]


def test_static_preview_backend_helpers() -> None:
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "preview"
        fixture = backend.repo_root() / "tests" / "fixtures" / "cell_definition_pick_place.yaml"
        result = backend.generate_static_preview(fixture, out, "Backend Preview")
        assert result["returncode"] == 0
        summary = backend.load_static_preview_summary(out)
        assert summary.get("title") == "Backend Preview"
