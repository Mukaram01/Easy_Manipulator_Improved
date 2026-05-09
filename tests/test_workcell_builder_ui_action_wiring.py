from pathlib import Path


def test_major_buttons_have_object_names_or_actions_or_disabled_tooltips():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    required_any = [
        ('New Cell',),('Open Cell',),('Save Cell',),('Browse Scenes Folder',),('Refresh Scenes',),
        ('Generate YAML files for scene','Generate Canonical Files'),('Generate files from YAML','Generate Files from YAML'),
        ('Generate Studio/Readiness Pack','Generate Studio Pack'),('Validate Cell',),('Refresh Asset Catalog',),
        ('Add selected asset','Add as Pick Object'),('Duplicate selected asset','Duplicate Selected Asset'),
        ('Remove selected asset','Remove Selected Asset'),('Clear assets','Clear Cell Assets'),('Import Custom STL',),
        ('Show/Export Preview','Open Preview','Export Layout Preview'),('Show Launch Command','Copy Fake-Hardware Launch Command','Copy Safe Simulation Launch Command')
    ]
    for options in required_any:
        assert any(opt in ui for opt in options), options
    assert 'name=""' not in ui
