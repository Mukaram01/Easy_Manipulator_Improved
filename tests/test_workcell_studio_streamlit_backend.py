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


def test_create_cell_backend_helpers() -> None:
    choices = backend.resolve_catalog_choices()
    assert "robots" in choices and choices["robots"]
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "cell"
        result = backend.create_cell(
            cell_id="backend_cell",
            robot="ur5",
            end_effector="robotiq_2f",
            sensor="intel_realsense_d435i",
            task="task_magnetic_pick_place",
            grasp_strategy="finger_pinch_basic",
            output_dir=out,
            validate=True,
            preview=False,
            generate_bundle=False,
            force=True,
        )
        assert result["returncode"] == 0
        summary = backend.load_create_cell_summary(out)
        assert summary["summary_json_path"]


def test_builder_task_intent_helpers_roundtrip() -> None:
    with tempfile.TemporaryDirectory() as d:
        scene = Path(d)/"scene"; scene.mkdir(parents=True, exist_ok=True)
        payload = backend.default_builder_task_intent("scene")
        path = scene/"generated"/"workcell_builder_task_intent.yaml"
        backend.save_builder_task_intent(path, payload)
        loaded = backend.load_builder_task_intent(path)
        assert loaded.get("schema") == "workcell_builder_task_intent/v1"
        assert backend.find_builder_task_intent(scene).endswith("workcell_builder_task_intent.yaml")
        result = backend.validate_builder_task_intent(path)
        assert result["returncode"] in {0,1}


def test_task_flow_backend_helpers():
    intent = backend.repo_root() / 'tests' / 'fixtures' / 'builder_task_intent_valid.yaml'
    result = backend.summarize_task_flow(task_intent_path=intent)
    assert result['returncode'] in {0,1}
    with tempfile.TemporaryDirectory() as d:
        out=Path(d)/'preview'
        cell=backend.repo_root()/'tests'/'fixtures'/'cell_definition_pick_place.yaml'
        pr=backend.generate_static_preview_with_task_flow(cell,out,'x',task_intent_path=intent)
        assert pr['returncode']==0


def test_backend_prepare_and_validate_rviz_plan_preview_session(tmp_path):
    from tools.workcell_studio_streamlit import backend
    recipe = tmp_path/"recipe.yaml"
    recipe.write_text((Path(__file__).resolve().parents[1]/"tests/fixtures/task_recipes/valid_pick_place.yaml").read_text(encoding="utf-8"), encoding="utf-8")
    req = tmp_path/"offline.yaml"
    gen = backend.run_command(["python3", str(backend.repo_root()/"scripts"/"generate_offline_plan_preview_request.py"), "--task-recipe", str(recipe), "--output", str(req), "--allow-incomplete", "--json"], cwd=backend.repo_root())
    assert gen["returncode"] == 0
    out = tmp_path/"session"
    prep = backend.prepare_rviz_plan_preview("scenes/ur5_2f_test", req, out)
    assert prep["ok"]
    payload = backend.load_rviz_plan_preview_session(out)
    assert payload.get("schema") == "rviz_moveit_plan_preview_session/v1"
    text = backend.read_suggested_commands(out)
    assert "use_fake_hardware:=false" not in text
    val = backend.validate_rviz_plan_preview_session(out/"rviz_moveit_plan_preview_session.json")
    assert val["ok"]

def test_backend_smoke_launch_helpers(tmp_path):
    session = tmp_path / "session.json"
    session.write_text('{"schema":"rviz_moveit_plan_preview_session/v1","source":{"scene_package":"scenes/ur5_2f_test"},"session":{"launch_allowed":false,"generated_commands_only":true},"rviz_moveit":{"suggested_launch":{"command":"python3 -c \\"print(1)\\" use_fake_hardware:=true"}},"safety":{"motion_started":false,"moveit_service_called":false,"ros_launch_started":false,"runtime_io_applied":false,"fake_hardware_required":true}}', encoding="utf-8")
    out = tmp_path / "smoke"
    run = backend.smoke_launch_preview(session, out, execute=False, timeout_s=2)
    assert run["ok"]
    report = backend.load_smoke_launch_report(out)
    assert report.get("schema") == "fake_hardware_smoke_launch_report/v1"
    val = backend.validate_smoke_launch_report(out / "fake_hardware_smoke_launch_report.json")
    assert val["ok"]
