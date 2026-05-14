from pathlib import Path

DEMO_PY = Path('scripts/workcell_studio_demo_mode.py').read_text(encoding='utf-8')

def test_demo_report_contains_layout_merge_status():
    for token in ['Layout Merge', 'Saved Layout Timestamp', 'Merge Timestamp', 'Layout Applied:', 'Stale:', 'workcell_studio_layout_merge.py']:
        assert token in DEMO_PY
