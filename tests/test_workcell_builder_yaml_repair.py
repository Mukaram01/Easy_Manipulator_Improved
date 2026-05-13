from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_yaml_repair_hook_and_backup_present():
    assert 'repair_scene_yaml_file' in CPP
    assert 'environment.yaml.bak' in CPP
    assert 'objects' in CPP and 'IsSequence' in CPP

def test_repair_non_overwrite_on_failure_and_dedup_message_present():
    assert 'YAML parse failure' in CPP
    assert 'if (last_status_message_ == QString::fromStdString(message))' in CPP
