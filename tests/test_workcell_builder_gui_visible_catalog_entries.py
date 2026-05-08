import json
from pathlib import Path

EXPECTED_ANY = [
    ['ur3'],['ur5'],['ur10'],['fanuc'],['panda'],
    ['robotiq 85','robotiq_2f','2f'],['robotiq 3f','robotiq_3f'],['single suction','suction'],['airpick4','onrobot'],
    ['table'],['workbench'],['cube placeholder','cube_placeholder'],['bin placeholder','bin_placeholder'],['conveyor placeholder','conveyor_placeholder'],['d435i','realsense']
]

def test_catalog_entries_visible():
    payload = json.loads(Path('workcell_studio_catalog/generated/workcell_builder_gui_catalog.json').read_text())
    hay=[]
    for arr in payload.values():
        for item in arr:
            hay.append((item.get('display_name','')+' ' +item.get('id','')).lower())
    text='\n'.join(hay)
    for choices in EXPECTED_ANY:
        assert any(c in text for c in choices), f"Missing expected catalog variant: {choices}"
