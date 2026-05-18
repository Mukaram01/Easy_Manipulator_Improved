import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "validate_scene_builder_canvas_generated_parity.py"
LEGACY_SCRIPT = REPO_ROOT / "scripts" / "validate_scene_builder_canvas_to_generated_parity.py"
FIXTURE_STL = REPO_ROOT / "tests" / "fixtures" / "meshes" / "tiny_ascii_cube.stl"


def _run_parity_script(args: list[str] | None = None) -> tuple[dict, subprocess.CompletedProcess[str]]:
    args = args or ["--json"]
    result = subprocess.run([sys.executable, str(SCRIPT), *args], cwd=REPO_ROOT, check=False, capture_output=True, text=True)
    return json.loads(result.stdout), result


def test_parity_script_exists_and_executes() -> None:
    assert SCRIPT.is_file()
    report, result = _run_parity_script()
    assert result.returncode in {0, 1}
    assert report["status"] in {"PASS", "WARN", "FAIL"}


def test_report_contains_required_fields_and_fake_hardware_token() -> None:
    report, _ = _run_parity_script()
    required = {
        "status", "source_layout_path", "generated_package_path", "scene_builder_parity_action_present",
        "parity_status_labels_present", "parity_report_filename_contract", "parity_report_required_fields",
        "layout_asset_ids", "generated_asset_ids", "layout_asset_id_set", "generated_asset_id_set",
        "transform_mismatches", "mesh_reference_mismatches", "unsupported_assets", "warning_count",
        "mismatch_count", "blocker_count", "fake_hardware_default", "fake_hardware_token_preserved",
        "unsupported_asset_warning_contract", "transform_mismatch_section", "mesh_reference_mismatch_section",
        "warnings", "errors",
    }
    assert required.issubset(report.keys())
    assert report["fake_hardware_token_preserved"] is True
    assert set(report["parity_report_required_fields"]) == required


def test_cli_scene_dir_json_output_file_and_stdout_json(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene"
    (scene_dir / "layout").mkdir(parents=True)
    (scene_dir / "generated").mkdir(parents=True)
    (scene_dir / "urdf").mkdir(parents=True)
    (scene_dir / "layout" / "workcell_studio_layout.yaml").write_text("assets: [{id: a1, pose: {xyz: [0,0,0], rpy: [0,0,0]}, mesh: m1.stl}]\n", encoding="utf-8")
    (scene_dir / "generated" / "generated_workcell_summary.json").write_text('{"assets": [{"id": "a1", "pose": {"xyz": [0,0,0], "rpy": [0,0,0]}, "mesh": "m1.stl"}]}', encoding="utf-8")

    out_path = tmp_path / "reports" / "parity.json"
    result = subprocess.run([sys.executable, str(SCRIPT), str(scene_dir), "--json", "--output", str(out_path)], cwd=REPO_ROOT, check=False, capture_output=True, text=True)
    assert result.returncode == 0
    stdout_report = json.loads(result.stdout)
    assert out_path.is_file()
    file_report = json.loads(out_path.read_text(encoding="utf-8"))
    assert stdout_report == file_report


def test_missing_optional_files_warn_not_crash(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene"
    scene_dir.mkdir()
    report, result = _run_parity_script([str(scene_dir), "--json"])
    assert result.returncode in {0, 1}
    assert isinstance(report["warnings"], list)
    assert any("optional scene artifact missing" in w for w in report["warnings"])


def test_malformed_scene_file_reports_clear_error(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene"
    (scene_dir / "layout").mkdir(parents=True)
    (scene_dir / "layout" / "workcell_studio_layout.yaml").write_text("assets: [oops", encoding="utf-8")
    report, result = _run_parity_script([str(scene_dir), "--json"])
    assert result.returncode == 1
    assert any("failed to parse" in err for err in report["errors"])


def test_small_fixture_ascii_stl_is_available_for_deterministic_runs() -> None:
    assert FIXTURE_STL.is_file()


def test_canonical_validator_is_the_only_contract_target() -> None:
    script_files = sorted(path.name for path in (REPO_ROOT / "scripts").glob("validate_scene_builder_canvas*generated_parity.py"))
    assert script_files == ["validate_scene_builder_canvas_generated_parity.py", "validate_scene_builder_canvas_to_generated_parity.py"]


def test_legacy_wrapper_prints_deprecation_and_delegates_to_canonical() -> None:
    result = subprocess.run([sys.executable, str(LEGACY_SCRIPT), "--json"], cwd=REPO_ROOT, check=False, capture_output=True, text=True)
    assert result.returncode in {0, 1}
    assert "DEPRECATED:" in result.stderr
    report = json.loads(result.stdout)
    assert report["status"] in {"PASS", "WARN", "FAIL"}
