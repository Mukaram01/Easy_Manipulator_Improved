from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_task_zone_preview_launch_is_offline_only_no_controller_moveit_or_hardware_startup():
    text = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    for needle in [
        "Offline/fake-hardware layout preview only",
        "no MoveIt planning and no robot motion commanded",
        "No real hardware enabled",
    ]:
        assert needle in text


def test_conveyor_task_zone_preview_reports_no_runtime_controller_actions():
    text = (ROOT / "workcell_builder/workcell_builder/src_conveyor_pick_preview.cpp").read_text(encoding="utf-8")
    for needle in [
        "INFO: preview_only",
        "INFO: no robot motion commanded",
        "INFO: no real conveyor commanded",
    ]:
        assert needle in text
