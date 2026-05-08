import json
from pathlib import Path

def test_recommended_starter_assets_present():
    payload=json.loads(Path('workcell_studio_catalog/generated/workcell_builder_gui_catalog.json').read_text())
    starter=payload['recommended_starter_assets']
    for item in ['UR5','Robotiq 2F','Workbench/Table','Cube Small','Small Bin','RealSense D435i visual asset']:
        assert item in starter
