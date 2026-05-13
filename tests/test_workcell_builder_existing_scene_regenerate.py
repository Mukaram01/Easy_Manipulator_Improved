from pathlib import Path

GUI_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
STATUS_CPP = Path('workcell_builder/workcell_builder/src_workcell_scene_status.cpp').read_text(encoding='utf-8')


def test_regenerate_action_exists_for_existing_scenes_and_has_non_dead_end_guidance():
    assert 'on_generate_full_scene_package_start_clicked' in GUI_CPP
    assert 'on_generate_files_clicked' in GUI_CPP
    assert 'Next recommended action: regenerate full scene package' in GUI_CPP


def test_lifecycle_includes_generated_package_ready_path():
    assert 'GENERATED_PACKAGE_READY' in STATUS_CPP
    assert 'Generated Package Ready' in STATUS_CPP
