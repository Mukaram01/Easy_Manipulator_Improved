from pathlib import Path
from scripts.capability_registry import load_structured_data

def test_manifest_loads_and_defaults():
    p=Path('workcell_studio_catalog/asset_manifest.yaml')
    doc,_=load_structured_data(p)
    assert doc['rules']['fake_hardware_default'] is True
    assert doc['package_exposure']['mode']=='single_source'
