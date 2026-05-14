from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_asset_catalog_panel_and_categories_present():
    for t in ['Scene Hierarchy','Asset Catalog','Robots','End Effectors','Cameras','Tables','Conveyors','Bins','Fixtures','Objects / STLs','Pick/Place Zones','Custom / Imported']:
        assert t in CPP

def test_asset_catalog_action_tokens_present():
    for t in ['Add to Canvas','Open Asset Folder','Copy Asset Path']:
        assert t in CPP
