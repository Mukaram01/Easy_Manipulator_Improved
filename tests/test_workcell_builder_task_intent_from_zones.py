from pathlib import Path
PYF = Path('scripts/create_or_update_builder_task_intent.py').read_text()

def test_task_intent_includes_pick_place_and_clearance_fields():
    for token in ['pick', 'source', 'place', 'target', 'strategy_ref', 'approach_axis', 'retreat_axis', 'place_clearance_m']:
        assert token in PYF
