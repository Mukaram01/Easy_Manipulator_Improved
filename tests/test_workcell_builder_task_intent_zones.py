from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_task_intent_zone_binding_actions_and_blockers_present():
    text = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    for needle in [
        "Use Selected Zone as Pick Zone",
        "Use Selected Zone as Place Zone",
        "pick source missing",
        "place target missing",
        "selected_pick_zone_id",
        "selected_place_zone_id",
    ]:
        assert needle in text
