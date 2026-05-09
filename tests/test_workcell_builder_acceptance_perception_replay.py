from pathlib import Path

def test_acceptance_script_mentions_publish_perception_replay():
    t=Path('scripts/workcell_builder_acceptance_check.py').read_text(encoding='utf-8')
    assert 'publish_perception_replay:=true' in t
