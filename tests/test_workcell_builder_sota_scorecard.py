from pathlib import Path


def test_sota_scorecard_has_levels_and_level3_baseline():
    doc = Path('docs/manuals/WORKCELL_BUILDER_SOTA_ACCEPTANCE.md').read_text(encoding='utf-8')
    for level in ['Level 0','Level 1','Level 2','Level 3','Level 4','Level 5']:
        assert level in doc
    assert 'Level 3' in doc and 'required builder baseline' in doc
