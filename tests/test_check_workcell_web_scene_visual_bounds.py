import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_workcell_web_scene_visual_bounds.py"

spec = importlib.util.spec_from_file_location("check_workcell_web_scene_visual_bounds", SCRIPT)
checker = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(checker)


def _valid_payload():
    return {
        "schema_version": "workcell_studio_web_scene/v1",
        "metadata": {"visual_bounds_contract": {"status": "passed"}},
        "robots": [
            {
                "id": "ur5_base_link",
                "category": "robot",
                "mesh_contract_category": "robot_link",
                "mesh_load_required": True,
                "mesh_uri": "assets/ur5/base.dae",
                "expected_dimensions_m": [0.2, 0.2, 0.1],
                "allow_mesh_unit_autoscale": False,
            }
        ],
        "tools": [],
        "assets": [
            {
                "id": "workbench",
                "role": "support_surface",
                "mesh_contract_category": "table",
                "mesh_uri": "assets/table.stl",
                "expected_dimensions_m": [1.2, 0.8, 0.08],
                "allow_mesh_unit_autoscale": True,
            }
        ],
        "sensors": [
            {
                "id": "realsense_camera",
                "role": "camera",
                "mesh_contract_category": "camera",
                "mesh_uri": "assets/realsense.stl",
                "expected_dimensions_m": [0.08, 0.08, 0.06],
                "allow_mesh_unit_autoscale": True,
            }
        ],
        "zones": [],
    }


def test_checker_accepts_visual_bounds_contract_payload():
    summary, errors = checker.check(_valid_payload())
    assert errors == []
    assert summary["contract_status"] == "passed"
    assert summary["table_item_count"] == 1
    assert summary["camera_item_count"] == 1
    assert summary["core_mesh_item_count"] == 3


def test_checker_rejects_visual_bounds_contract_failed_status_with_blockers():
    payload = _valid_payload()
    payload["metadata"]["visual_bounds_contract"] = {
        "status": "failed",
        "camera_framing_blockers": [
            {"id": "workbench", "category": "table", "reason": "oversized_item_can_break_camera_framing"}
        ],
    }

    summary, errors = checker.check(payload)

    assert summary["contract_status"] == "failed"
    assert any("metadata.visual_bounds_contract.status must be pass or passed" in error for error in errors)


def test_checker_cli_reports_camera_framing_blocker_as_violation(tmp_path):
    web_scene = tmp_path / "scene.web_scene.json"
    payload = _valid_payload()
    payload["metadata"]["visual_bounds_contract"] = {
        "status": "passed",
        "camera_framing_blockers": [
            {
                "id": "helper_bounds_box",
                "category": "bounds_box",
                "reason": "helper_overlay_would_dominate_product_view",
            }
        ],
    }
    web_scene.write_text(json.dumps(payload), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(web_scene), "--json"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 1
    summary = json.loads(result.stdout)
    assert summary["contract_status"] == "failed"
    assert any(
        "helper_bounds_box" in error
        and "bounds_box" in error
        and "helper_overlay_would_dominate_product_view" in error
        for error in summary["violations"]
    )

def test_checker_rejects_missing_dimensions_mesh_contract_and_robot_autoscale():
    payload = _valid_payload()
    del payload["assets"][0]["expected_dimensions_m"]
    del payload["sensors"][0]["mesh_contract_category"]
    payload["robots"][0]["allow_mesh_unit_autoscale"] = True
    payload["robots"][0]["mesh_unit_correction"] = {"scale": 0.001}

    summary, errors = checker.check(payload)

    assert summary["contract_status"] == "failed"
    assert any("table/workbench item but lacks expected_dimensions_m" in error for error in errors)
    assert any("required/core mesh item but lacks mesh_contract_category" in error for error in errors)
    assert any("allow_mesh_unit_autoscale: true" in error for error in errors)
    assert any("contains mesh_unit_correction" in error for error in errors)


def test_checker_does_not_treat_helper_zone_label_as_physical_table():
    payload = _valid_payload()
    payload["zones"].append(
        {
            "id": "safety_zone_keepout",
            "display_name": "Robot/Table Keepout Boundary",
            "role": "keepout",
            "category": "safety_zone",
            "dimensions": [1.6, 1.2, 0.02],
        }
    )

    summary, errors = checker.check(payload)

    assert errors == []
    assert summary["contract_status"] == "passed"
    assert summary["table_item_count"] == 1
    assert summary["core_mesh_item_count"] == 3


def test_checker_cli_returns_json_summary_and_nonzero_for_violations(tmp_path):
    web_scene = tmp_path / "scene.web_scene.json"
    payload = _valid_payload()
    payload["assets"][0]["expected_dimensions_m"] = [1200, 0.8, 0.08]
    web_scene.write_text(json.dumps(payload), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(web_scene), "--json"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 1
    summary = json.loads(result.stdout)
    assert summary["contract_status"] == "failed"
    assert any("obviously impossible expected_dimensions_m.x" in error for error in summary["violations"])
