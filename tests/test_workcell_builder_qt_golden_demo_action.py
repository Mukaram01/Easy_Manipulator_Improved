from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_golden_demo_action_creates_complete_builder_state(tmp_path):
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Create Golden UR5 + Robotiq 2F Cell' in ui
    out = wf.create_golden_demo_cell('golden_ur5_2f_cell', tmp_path)
    assert out['ok']
    state = out['state']
    assert state['selected']['robot'] == 'ur5'
    assert state['selected']['tool'] == 'robotiq_2f'
    assert len(state['current_cell_assets']) >= 7
    scene = Path(out['scene_dir'])
    assert (scene / 'environment.yaml').exists()
    assert (scene / 'scene_manifest.yaml').exists()
    assert 'colcon build --symlink-install --packages-select golden_ur5_2f_cell' in out['build_command']
    assert 'ros2 launch golden_ur5_2f_cell demo.launch.py use_fake_hardware:=true' == out['launch_command']
