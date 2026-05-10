from pathlib import Path
from workcell_builder.workcell_builder.scene_asset_library import discover_scenes

def test_scene_discovery_and_status(tmp_path: Path):
    root=tmp_path/'src'/'scenes'; root.mkdir(parents=True)
    ready=root/'ready'; (ready/'launch').mkdir(parents=True); (ready/'environment.yaml').write_text('a: 1'); (ready/'launch'/'demo.launch.py').write_text('')
    yaml_only=root/'yaml_only'; yaml_only.mkdir(); (yaml_only/'environment.yaml').write_text('a: 1')
    repair=root/'repair'; repair.mkdir(); (repair/'package.xml').write_text('<p/>')
    rows={r.name:r.status for r in discover_scenes(root)}
    assert rows['ready']=='READY'
    assert rows['yaml_only']=='YAML_ONLY'
    assert rows['repair']=='NEEDS_REPAIR'
