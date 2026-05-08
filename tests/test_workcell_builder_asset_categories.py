import json
from pathlib import Path

def test_categories_present():
    payload=json.loads(Path('workcell_studio_catalog/generated/workcell_builder_gui_catalog.json').read_text())
    cats=set(payload['asset_categories'])
    expected={'Robots','Grippers & Tools','Cameras & Sensors','Tables & Workbenches','Bins & Totes','Conveyors','Fixtures & Jigs','Machines','Safety','Objects','Custom STL','Preview-only Assets'}
    assert expected.issubset(cats)
