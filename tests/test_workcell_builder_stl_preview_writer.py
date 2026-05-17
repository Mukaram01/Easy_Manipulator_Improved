from pathlib import Path


def test_preview_writer_files_and_markers_exist():
    h = Path('workcell_builder/workcell_builder/include/placed_object_preview_writer.hpp').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
    for marker in [
        'PlacedObjectPreviewWriter',
        'placed_objects_preview.yaml',
        'placed_objects_preview.urdf.xacro',
        'preview_scene.launch.py',
        'README_PREVIEW.md',
        '/tmp/workcell_builder_preview',
        '<joint name=',
        'type=\\"fixed\\"',
        '<mesh filename=',
        'origin xyz=',
        'rpy=',
    ]:
        assert marker in h or marker in cpp


def test_invalid_mesh_warnings_and_safety_markers():
    cpp = Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
    for marker in [
        'mesh path is empty',
        'mesh extension should be .stl/.dae/.obj',
        'absolute external path is discouraged',
        'mesh file does not exist on disk',
        'Visual-only offline preview',
        'No MoveIt, controllers, trajectories, or real robot motion',
    ]:
        assert marker in cpp


def test_preview_launch_is_visual_only():
    cpp = Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8').lower()
    assert 'robot_state_publisher' in cpp
    assert 'joint_state_publisher' in cpp
    assert 'rviz2' in cpp
    for forbidden in ['move_group', 'controller_manager', 'followjointtrajectory', 'execute_trajectory', 'real_hardware']:
        assert forbidden not in cpp
