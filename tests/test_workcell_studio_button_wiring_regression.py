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


def test_top_bar_uses_named_page_enum_and_no_invalid_indexes():
    cpp = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    header = Path("workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")
    assert "enum class StudioPage" in header
    assert "show_studio_page(StudioPage::DashboardPage)" in cpp
    assert "show_studio_page(StudioPage::SceneBuilderPage)" in cpp
    assert "show_studio_page(StudioPage::ExistingScenesPage)" in cpp
    assert "show_studio_page(StudioPage::ValidationPage)" in cpp
    assert "show_studio_page(StudioPage::PlanSimulatePage)" in cpp
    assert "show_studio_page(StudioPage::ExportPage)" in cpp
    assert "setCurrentRow(9)" not in cpp
    assert "setCurrentRow(10)" not in cpp
