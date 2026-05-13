from pathlib import Path
PYF = Path('scripts/create_or_update_builder_task_intent.py').read_text(encoding='utf-8')

def test_task_recipe_generation_tokens():
    for t in ['pick', 'source', 'place', 'target', 'approach_distance_m', 'retreat_distance_m', 'place_clearance_m', 'fake_hardware_first', 'runtime_execution_enabled', 'motion_command_sent']:
        assert t in PYF
