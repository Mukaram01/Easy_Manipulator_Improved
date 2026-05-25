from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/src_workcell_studio_template_instantiator.cpp').read_text(encoding='utf-8')
MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_generated_package_contains_scene_xacro_contract():
    assert 'urdf" / "scene.urdf.xacro' in CPP
    assert 'robot_visual_placeholder_due_to_missing_robot_xacro' in CPP


def test_generated_launch_is_not_placeholder_and_has_rsp():
    assert '# generated launch placeholder' not in CPP
    for token in ['DeclareLaunchArgument(\'use_fake_hardware\'', 'DeclareLaunchArgument(\'launch_rviz\'', 'robot_state_publisher', 'rviz2']:
        assert token in CPP


def test_readiness_report_flags_and_blockers_present():
    for token in [
        'scene_package_readiness.json',
        'placeholder_launch_only',
        'rviz_truth_preview_ready',
        'robot_visual_mode',
        'missing robot xacro',
        'missing robot package',
        'missing end-effector xacro',
        'placeholder visual only',
    ]:
        assert token in CPP


def test_existing_readiness_path_still_checks_scene_xacro():
    assert 'const bool scene_xacro_ready = has("urdf/scene.urdf.xacro") || s.has_scene_urdf_xacro;' in MAIN
