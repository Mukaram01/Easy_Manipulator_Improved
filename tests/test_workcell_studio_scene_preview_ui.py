from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_ui_labels_and_launch_command_present():
    cpp = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()
    for label in [
        "Existing Scenes",
        "Open in Scene Builder",
        "Open Preview",
        "Open Smoke Report",
        "Copy Launch Command",
        "No robot motion commanded",
        "use_fake_hardware:=true",
    ]:
        assert label in cpp


def test_malformed_metadata_handled_as_warning_not_crash():
    src = (ROOT / "workcell_builder/workcell_builder/src_workcell_studio_scene_browser.cpp").read_text()
    assert "Could not parse environment.yaml" in src
