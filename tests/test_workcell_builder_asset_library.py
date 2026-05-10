from pathlib import Path
from workcell_builder.workcell_builder.scene_asset_library import discover_assets

def test_asset_discovery_categories(tmp_path: Path):
    assets=tmp_path/'src'/'assets'
    for cat,name in [('robots','ur5_description'),('end_effectors','rg2_description'),('environment','table_description')]:
        d=assets/cat/name; (d/'urdf').mkdir(parents=True); (d/'meshes').mkdir(); (d/'package.xml').write_text('')
    result=discover_assets(assets)
    assert result['Robots'][0]['package']=='ur5_description'
    assert result['End Effectors'][0]['package']=='rg2_description'
    assert result['Environment Objects'][0]['package']=='table_description'
