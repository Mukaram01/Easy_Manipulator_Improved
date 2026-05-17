from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_cell_definition_task_zone_keys_exist_in_scene_yaml_templates():
    text = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    for needle in [
        "pick_zone: {source: default_from_environment}",
        "place_zone: {target: default_from_environment}",
        "ensure_ws_key(\"pick_zone\"",
        "ensure_ws_key(\"place_zone\"",
    ]:
        assert needle in text
