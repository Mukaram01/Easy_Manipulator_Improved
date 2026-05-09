from pathlib import Path


def test_repair_helper_for_missing_environment_yaml_exists():
    content = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'ensure_minimal_environment_yaml' in content
    assert 'Repair Missing environment.yaml applied at:' in content
    assert 'refresh_scene_manifest_if_missing(entry.path(), scene_name);' in content
