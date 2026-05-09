import json
from pathlib import Path

def test_detected_objects_fixture_schema():
    d=json.loads(Path('tests/fixtures/detected_objects/detected_objects_v1_sample.yaml').read_text())
    assert d['schema_version']=='detected_objects/v1'
    assert d['objects'] and d['objects'][0]['pose']['xyz']
