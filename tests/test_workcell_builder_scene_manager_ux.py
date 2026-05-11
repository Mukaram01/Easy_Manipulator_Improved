from pathlib import Path


def test_scene_root_discovery_prefers_emd_scenes_folder():
    cpp = Path('workcell_builder/workcell_builder/src_scene_select_paths.cpp').read_text(encoding='utf-8')
    assert 'cwd / "src" / "easy_manipulation_deployment" / "scenes"' in cpp
    assert 'candidates.push_back(cwd / "src" / "easy_manipulation_deployment")' in cpp


def test_scene_status_labels_are_exposed_in_ui_listing():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for label in [
        'VALID', 'INCOMPLETE', 'SCAFFOLD_ONLY',
        'MISSING_ENVIRONMENT_YAML', 'MISSING_ROBOT', 'MISSING_MOVEIT_CONFIG'
    ]:
        assert f'"{label}"' in cpp
    assert 'name + " [" + scene_status_label(status) + "]"' in cpp


def test_generate_guidance_includes_fake_hardware_launch_and_warning():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'colcon build --symlink-install --packages-select' in cpp
    assert 'ros2 launch ' in cpp
    assert 'use_fake_hardware:=true' in cpp
    assert 'Real hardware mode requires explicit validation and use_fake_hardware:=false.' in cpp


def test_unknown_description_lookup_text_not_reintroduced():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'unknown_description' not in cpp
    assert 'unknown_moveit_config' not in cpp
