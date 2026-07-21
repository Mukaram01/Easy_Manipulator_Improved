import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/check_workcell_web_scene_visual_bounds.py"

spec = importlib.util.spec_from_file_location(
    "check_workcell_web_scene_visual_bounds_editable_source_layer", SCRIPT
)
checker = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(checker)


def test_editable_source_layer_does_not_make_an_object_a_table():
    payload = {
        "metadata": {"visual_bounds_contract": {"status": "passed"}},
        "robots": [],
        "tools": [],
        "assets": [
            {
                "id": "commissioning_object",
                "source_layer": "editable_authored_physical",
                "active_visual_source": "cell_definition.yaml",
                "dimensions": [0.05, 0.05, 0.05],
            },
            {
                "id": "workbench",
                "role": "support_surface",
                "mesh_contract_category": "table",
                "expected_dimensions_m": [1.2, 0.8, 0.08],
            },
        ],
        "sensors": [
            {
                "id": "realsense_camera",
                "mesh_contract_category": "camera",
                "expected_dimensions_m": [0.08, 0.08, 0.06],
            }
        ],
        "zones": [],
    }

    summary, errors = checker.check(payload)

    assert errors == []
    assert summary["contract_status"] == "passed"
    assert summary["table_item_count"] == 1
    assert checker._category("assets", payload["assets"][0]) == "object"
