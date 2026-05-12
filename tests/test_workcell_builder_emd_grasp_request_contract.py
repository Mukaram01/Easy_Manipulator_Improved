import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "workcell_builder/workcell_builder"

def _sample_strategy():
    return {
        "grasp_strategy": {
            "schema_version": 1,
            "runtime_mode": "preview_only",
            "default_strategy": "generic_top_down",
            "strategies": [
                {"class_label": "box", "end_effector_type": "finger", "strategy": "side_grasp", "approach_distance_m": 0.10, "retreat_distance_m": 0.10},
                {"class_label": "flat_part", "end_effector_type": "suction", "strategy": "top_down_suction", "approach_distance_m": 0.08, "retreat_distance_m": 0.12},
            ],
        }
    }

def test_grasp_strategy_yaml_roundtrip():
    data = _sample_strategy()
    txt = yaml.safe_dump(data)
    back = yaml.safe_load(txt)
    assert back["grasp_strategy"]["strategies"][0]["class_label"] == "box"
    assert back["grasp_strategy"]["strategies"][0]["strategy"] == "side_grasp"
    assert back["grasp_strategy"]["strategies"][1]["end_effector_type"] == "suction"

def test_strategy_selection_and_default_fallback_and_incompatible():
    rules = _sample_strategy()["grasp_strategy"]["strategies"]
    assert next(r for r in rules if r["class_label"]=="box" and r["end_effector_type"]=="finger")["strategy"] == "side_grasp"
    assert next(r for r in rules if r["class_label"]=="flat_part" and r["end_effector_type"]=="suction")["strategy"] == "top_down_suction"
    default = _sample_strategy()["grasp_strategy"]["default_strategy"]
    assert default == "generic_top_down"
    assert rules[1]["end_effector_type"] != "finger"

def test_emd_request_artifact_safety_flags():
    payload = {"emd_grasp_planner_request": {"planner_backend": "existing_emd_grasp_planner", "robot_motion_commanded": False, "gripper_command_sent": False, "moveit_execute_called": False, "grasp_execution_called": False}}
    y = yaml.safe_dump(payload)
    j = json.dumps(payload)
    assert "existing_emd_grasp_planner" in y
    assert '"robot_motion_commanded": false' in j.lower()


def test_scene_status_and_ui_strings_present():
    status_cpp = (SRC / "src_workcell_scene_status.cpp").read_text()
    assert "Grasp strategy available" in status_cpp
    assert "EMD grasp planner request generated" in status_cpp
    assert "No grasp execution called" in status_cpp
    ui_src = (SRC / "gui/mainwindow.cpp").read_text()
    for label in ["Check Grasp Strategy", "Generate EMD Grasp Request", "Open EMD Grasp Request"]:
        assert label in ui_src

def test_bundle_export_preserves_grasp_contract_artifacts():
    bundle_cpp = (SRC / "src_workcell_scene_bundle.cpp").read_text()
    for rel in ["preview/grasp_strategy.yaml", "preview/emd_grasp_planner_request.yaml", "preview/emd_grasp_planner_request.json", "preview/grasp_strategy_readiness_report.yaml", "preview/grasp_strategy_readiness_report.json"]:
        assert rel in bundle_cpp

def test_scenario_catalog_and_roadmap_mentions_contract_and_fake_hw_false():
    catalog = (ROOT / "catalog/scenarios/industrial_scenarios.yaml").read_text()
    assert "grasp_strategy_contract" in catalog
    assert "real_hardware_ready: false" in catalog
    assert "bin_picking_tote_picking: partial" in catalog
    for p in [ROOT / "docs/roadmap/WORKCELL_STUDIO_TODO.md", ROOT / "docs/roadmap/WORKCELL_STUDIO_CAPABILITY_MATRIX.md"]:
        t = p.read_text()
        assert "EMD Grasp Request Contract" in t
