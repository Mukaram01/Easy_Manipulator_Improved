from pathlib import Path


def test_task_grasp_ui_fields_exist():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    for needle in [
        'Task &amp; Grasp Strategy', 'Task Type', 'Pick Source', 'Place Target', 'Grasp Strategy',
        'Orientation Mode', 'Approach Distance (m)', 'Retreat Distance (m)', 'Release Strategy',
        'Validate Task', 'Reset to Tool Defaults', 'finger_top', 'suction_top', 'open_gripper', 'vacuum_off'
    ]:
        assert needle in ui


def test_task_recipe_persistence_and_safety_strings_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for needle in [
        'config" / "task_recipe.yaml', 'schema_version: workcell_task/v1', 'fake_hardware_first: true',
        'motion_command_sent: false', 'runtime_execution_enabled: false',
        'Task recipe: OK', 'Grasp strategy: OK', 'Pick source: OK', 'Place target: OK',
        'Task/grasp recipe generated for offline/fake-hardware planning only. No robot motion was commanded.',
        'Task: ', 'Grasp: ', 'Pick source: ', 'Place target: '
    ]:
        assert needle in cpp


def test_no_runtime_execution_or_moveit_service_calls_added():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8').lower()
    assert 'moveit_msgs/srv/getmotionplan' not in cpp
    assert 'motion_plan' not in cpp or 'preview' in cpp


def test_readiness_and_summary_task_preview_strings_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for needle in ['task_recipe.yaml', 'Task recipe generated: OK', 'Task plan dry-run preview:', 'workcell_studio_summary.json', 'workcell_studio_summary.md']:
        assert needle in cpp
