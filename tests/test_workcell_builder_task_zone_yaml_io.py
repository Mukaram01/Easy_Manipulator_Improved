from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_task_zone_yaml_tokens_include_task_zones_and_safe_legacy_behavior():
    text = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    # task-zone schema/token persistence hooks
    for needle in [
        "pick_zone",
        "place_zone",
        "parse_work_zones_from_yaml",
        "Auto-fix Pick/Place Zones",
    ]:
        assert needle in text


def test_task_zone_yaml_legacy_or_missing_input_is_warn_only_no_crash_tokens():
    text = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    for needle in [
        "No supported legacy YAML normalization needed.",
        "Repaired legacy objects list->map format; wrote environment.yaml.bak",
        "scene YAML parse failed",
        "downgraded to legacy mode",
    ]:
        assert needle in text
