from pathlib import Path


def test_offline_smoke_model_fields_and_categories_exist():
    h = Path('workcell_builder/workcell_builder/include/offline_smoke_check_model.hpp').read_text()
    for token in [
        'OfflineSmokeCheckResult', 'status', 'scene_name', 'scene_dir', 'checks', 'blockers', 'warnings',
        'generated_artifacts', 'next_action', 'safety_flags'
    ]:
        assert token in h
    for cat in ['scene_yaml', 'task_grasp', 'package_files', 'preview_artifacts', 'safety_flags', 'build_launch_commands']:
        assert cat in Path('workcell_builder/workcell_builder/src_offline_smoke_check_model.cpp').read_text()
