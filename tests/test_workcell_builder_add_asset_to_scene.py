from pathlib import Path
from workcell_builder.workcell_builder.scene_asset_library import add_environment_asset_to_scene

def test_add_asset_updates_environment_yaml_without_erasing(tmp_path: Path):
    scene=tmp_path/'scene_a'; scene.mkdir()
    (scene/'environment.yaml').write_text('existing: true\n')
    env=add_environment_asset_to_scene(scene,'table_description',name='table_1',xyz=[0,0,0],rpy=[0,0,0],scale=[1,1,1],collision_enabled=True)
    text=env.read_text()
    assert 'existing: true' in text
    assert 'environment_objects:' in text
    assert 'table_1' in text
