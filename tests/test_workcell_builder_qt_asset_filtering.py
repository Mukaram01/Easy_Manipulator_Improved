from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_asset_category_controls_exist_and_filter_changes_assets():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'name="asset_category_tree"' in ui
    robots = wf.list_catalog_assets_by_category('Robots')
    grippers = wf.list_catalog_assets_by_category('Grippers & Tools')
    assert robots and grippers
    assert set(robots) != set(grippers)
