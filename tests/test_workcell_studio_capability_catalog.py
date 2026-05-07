from pathlib import Path

from scripts.workcell_builder_capability_catalog import load_workcell_capability_catalog


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_shipping_catalog_loads_offline_and_groups() -> None:
    payload = load_workcell_capability_catalog(REPO_ROOT / "catalog" / "capabilities")
    assert payload["robots"]
    assert payload["end_effectors"]
    assert payload["sensors"]
    assert payload["environment_assets"]
    assert payload["task_templates"]


def test_required_minimum_capabilities_exist() -> None:
    payload = load_workcell_capability_catalog(REPO_ROOT / "catalog" / "capabilities")
    ids = {
        "robots": {x["capability_id"] for x in payload["robots"]},
        "end_effectors": {x["capability_id"] for x in payload["end_effectors"]},
        "sensors": {x["capability_id"] for x in payload["sensors"]},
        "environment_assets": {x["capability_id"] for x in payload["environment_assets"]},
        "task_templates": {x["capability_id"] for x in payload["task_templates"]},
    }
    assert {"ur5", "generic_delta_900", "generic_gantry_xyz"}.issubset(ids["robots"])
    assert {"robotiq_2f_85", "onrobot_airpick_style", "generic_vacuum_array"}.issubset(ids["end_effectors"])
    assert "intel_realsense_d435i" in ids["sensors"]
    assert {"table_standard_1200", "bin_blue_large", "conveyor_2m", "camera_mount_overhead_a"}.issubset(ids["environment_assets"])
    assert {"pick_place", "sort_by_colour", "inspection_routing", "machine_tending"}.issubset(ids["task_templates"])
