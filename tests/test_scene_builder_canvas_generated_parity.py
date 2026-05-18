import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "validate_scene_builder_canvas_generated_parity.py"
FIXTURE_STL = REPO_ROOT / "tests" / "fixtures" / "meshes" / "tiny_ascii_cube.stl"


def _run_parity_script() -> dict:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--json"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(result.stdout)


def test_parity_script_exists_and_executes() -> None:
    assert SCRIPT.is_file()
    report = _run_parity_script()
    assert report["status"] in {"PASS", "FAIL"}


def test_report_contains_required_fields_and_fake_hardware_token() -> None:
    report = _run_parity_script()
    required = {
        "status",
        "source_layout_path",
        "generated_package_path",
        "scene_builder_parity_action_present",
        "parity_status_labels_present",
        "parity_report_filename_contract",
        "parity_report_required_fields",
        "layout_asset_ids",
        "generated_asset_ids",
        "layout_asset_id_set",
        "generated_asset_id_set",
        "transform_mismatches",
        "mesh_reference_mismatches",
        "unsupported_assets",
        "warning_count",
        "mismatch_count",
        "blocker_count",
        "fake_hardware_default",
        "fake_hardware_token_preserved",
        "unsupported_asset_warning_contract",
        "transform_mismatch_section",
        "mesh_reference_mismatch_section",
        "warnings",
        "errors",
    }
    assert required.issubset(report.keys())
    assert report["fake_hardware_token_preserved"] is True
    assert isinstance(report["source_layout_path"], str)
    assert isinstance(report["generated_package_path"], str)
    assert isinstance(report["layout_asset_ids"], list)
    assert isinstance(report["generated_asset_ids"], list)
    assert isinstance(report["layout_asset_id_set"], list)
    assert isinstance(report["generated_asset_id_set"], list)
    assert isinstance(report["transform_mismatches"], list)
    assert isinstance(report["mesh_reference_mismatches"], list)
    assert isinstance(report["unsupported_assets"], list)
    assert isinstance(report["warning_count"], int)
    assert isinstance(report["mismatch_count"], int)
    assert isinstance(report["blocker_count"], int)
    assert isinstance(report["fake_hardware_default"], bool)
    assert set(report["parity_report_required_fields"]) == required


def test_unsupported_assets_warning_and_mismatch_sections_present_and_used() -> None:
    report = _run_parity_script()
    assert report["unsupported_asset_warning_contract"] is True
    assert report["transform_mismatch_section"] is True
    assert report["mesh_reference_mismatch_section"] is True
    assert isinstance(report["warnings"], list)
    assert report["warning_count"] == len(report["warnings"]) + len(report["unsupported_assets"])
    assert report["mismatch_count"] == len(report["transform_mismatches"]) + len(report["mesh_reference_mismatches"])
    assert report["blocker_count"] == len(report["errors"])


def test_small_fixture_ascii_stl_is_available_for_deterministic_runs() -> None:
    assert FIXTURE_STL.is_file()
    text = FIXTURE_STL.read_text(encoding="utf-8")
    assert text.startswith("solid tiny_ascii_cube")
    assert "facet normal" in text
