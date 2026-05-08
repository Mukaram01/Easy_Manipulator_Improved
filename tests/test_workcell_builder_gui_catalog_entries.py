from pathlib import Path
from scripts.workcell_builder_gui_catalog import generate

def _names(group):
    return {x.get('display_name','').lower() for x in group}

def test_expected_entries_present():
    data=generate(Path('.').resolve())
    robots=_names(data['robots'])
    assert any('ur5' in n for n in robots)
    assert any('ur3' in n for n in robots)
    assert any('ur10' in n for n in robots)
    assert any('fanuc' in n for n in robots)
    assert any('panda' in n for n in robots)
    grippers=_names(data['grippers'])|_names(data['tools'])
    assert any('robotiq' in n and ('85' in n or '2f' in n) for n in grippers)
    assert any('robotiq' in n and '3f' in n for n in grippers)
    assert any('suction' in n for n in grippers)
    assert any('airpick' in n for n in grippers)
    env=_names(data['tables'])|_names(data['workbenches'])|_names(data['objects'])|_names(data['conveyors'])
    assert any('table' in n for n in env)
    assert any('bench' in n for n in env)
    assert any('cube' in n or 'bin' in n for n in env)
    sensors=_names(data['sensors'])|_names(data['cameras'])
    assert any('d435' in n or 'realsense' in n for n in sensors)
