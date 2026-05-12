from pathlib import Path
import subprocess
import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "catalog/scenarios/industrial_scenarios.yaml"
TODO_DOC = ROOT / "docs/roadmap/WORKCELL_STUDIO_TODO.md"
CHECKER = ROOT / "scripts/check_workcell_scenario_catalog.py"


def _load_catalog():
    return yaml.safe_load(CATALOG.read_text())["scenarios"]


def test_checker_runs_cleanly():
    result = subprocess.run(
        ["python3", str(CHECKER)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr


def test_required_scenarios_exist():
    ids = {s["id"] for s in _load_catalog()}
    required = {
        "static_table_pick_place",
        "conveyor_upstream_detection_downstream_pick",
        "conveyor_sorting_by_class",
        "bin_tote_picking",
        "kitting_tray_loading",
        "machine_tending_load_unload",
        "inspection_and_reject",
        "palletizing_depalletizing_light",
        "fixture_loading_assembly_assist",
        "multi_bin_sorting_cell",
        "dual_camera_workcell",
        "mobile_portable_demo_cell",
    }
    assert required.issubset(ids)


def test_conveyor_and_static_exist():
    ids = {s["id"] for s in _load_catalog()}
    assert "conveyor_upstream_detection_downstream_pick" in ids
    assert "static_table_pick_place" in ids


def test_live_epd_feed_present_as_required_or_missing():
    scenarios = _load_catalog()
    assert any(
        s.get("live_epd_feed_required") or "live_epd_feed" in s.get("missing_capabilities", [])
        for s in scenarios
    )


def test_task_intent_preview_supported_for_previewable_scenarios():
    scenarios = _load_catalog()
    previewish = {"supported_preview", "partial_preview", "needs_live_epd"}
    for s in scenarios:
        if s["current_status"] in previewish:
            assert isinstance(s["task_intent_preview_supported"], bool)


def test_real_hardware_ready_false_by_default():
    for s in _load_catalog():
        assert s["real_hardware_ready"] is False


def test_todo_has_all_priority_sections():
    text = TODO_DOC.read_text()
    for sec in ["## P0", "## P1", "## P2", "## P3", "## P4"]:
        assert sec in text
