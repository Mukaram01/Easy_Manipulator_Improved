from pathlib import Path


IMPORTER_CPP = Path('workcell_builder/workcell_builder/gui/rviz_pose_feedback_importer.cpp').read_text(encoding='utf-8')
DIALOG_CPP = Path('workcell_builder/workcell_builder/gui/object_placement_dialog.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = Path('workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
PREVIEW_NODE = Path('scripts/workcell_builder_interactive_preview_node.py').read_text(encoding='utf-8')


def test_valid_feedback_parses_markers_exist():
    for marker in [
        'YAML::Load(input)',
        'root.IsMap()',
        'out.entries.push_back(entry)',
        'entry.valid = entry.errors.empty()',
    ]:
        assert marker in IMPORTER_CPP


def test_safe_for_robot_motion_true_rejected():
    for marker in [
        'Rejected feedback: safe_for_robot_motion must be explicitly false.',
        'Rejected feedback: safe_for_robot_motion must be false.',
    ]:
        assert marker in IMPORTER_CPP


def test_wrong_source_rejected():
    assert 'Rejected feedback: source must be rviz_interactive_marker_preview.' in IMPORTER_CPP


def test_missing_objects_list_safe_handling():
    assert 'Missing or invalid objects list.' in IMPORTER_CPP


def test_malformed_yaml_safe_handling():
    for marker in ['Malformed YAML: ', 'Malformed YAML: root is not a map.']:
        assert marker in IMPORTER_CPP


def test_missing_xyz_rpy_safe_handling():
    for marker in [
        'Missing or non-finite x.',
        'Missing or non-finite y.',
        'Missing or non-finite z.',
        'Missing or non-finite roll.',
        'Missing or non-finite pitch.',
        'Missing or non-finite yaw.',
    ]:
        assert marker in IMPORTER_CPP


def test_nan_inf_rejected():
    assert 'std::isfinite(out)' in IMPORTER_CPP


def test_unknown_object_warning_behavior():
    assert 'Unknown object in feedback' in DIALOG_CPP


def test_matched_object_updates_pose_only_name_source_mesh_unchanged():
    for marker in [
        'if (entry.valid && entry.name == obj.name)',
        'obj.x = entry.x;',
        'obj.y = entry.y;',
        'obj.z = entry.z;',
        'obj.roll = entry.roll;',
        'obj.pitch = entry.pitch;',
        'obj.yaw = entry.yaw;',
    ]:
        assert marker in DIALOG_CPP
    feedback_apply_block = DIALOG_CPP.split('if (entry.valid && entry.name == obj.name)')[1].split('model_ = ObjectPlacementModel();')[0]
    for forbidden in [
        'obj.name =',
        'obj.source_type =',
        'obj.mesh_path =',
    ]:
        assert forbidden not in feedback_apply_block


def test_examples_contain_safe_for_robot_motion_false():
    assert 'safe_for_robot_motion: false' in PREVIEW_CPP
    assert 'safe_for_robot_motion' in PREVIEW_NODE and 'False' in PREVIEW_NODE
