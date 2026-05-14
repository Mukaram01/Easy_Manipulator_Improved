from pathlib import Path


def test_critical_buttons_wired_or_safe_fallback():
    text = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    wiring_tokens = [
        "run_layout_merge_for_selected_scene", "save_layout_changes", "run_preview_build",
        "run_fake_hardware_preview", "stop_preview_process", "open_layout_merge_report",
        'open_selected_scene_artifact("run_acceptance")', 'open_selected_scene_artifact("demo_dashboard")',
        "show_not_wired_message",
    ]
    for token in wiring_tokens:
        assert token in text
