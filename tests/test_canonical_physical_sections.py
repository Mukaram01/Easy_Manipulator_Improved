import sys
from pathlib import Path
import yaml

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from workcell_scene_sections import physical_sections


def test_canonical_scenes_resolve_physical_owners():
    for name in ('ur5_2f_test', 'suction_test'):
        data = yaml.safe_load((ROOT / 'scenes' / name / 'environment.yaml').read_text())
        sections = physical_sections(data)
        assert sections['assets']
        assert sections['support_surfaces']
        assert all(item['id'] for rows in sections.values() for item in rows)


def test_nested_sections_override_root_without_merging_duplicates():
    assert physical_sections(dict(assets=[{'id': 'legacy'}], sensors=[{'id': 'camera'}],
                                  environment=dict(assets=[]))) == dict(
        assets=[], sensors=[{'id': 'camera'}], support_surfaces=[])
