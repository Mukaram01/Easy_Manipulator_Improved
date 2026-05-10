from pathlib import Path
from workcell_builder.workcell_builder.scene_asset_library import import_stl_as_environment_asset,sanitize_asset_name

def test_stl_import_and_duplicate_name(tmp_path: Path):
    stl=tmp_path/'Box Demo.STL'; stl.write_text('solid box')
    assets=tmp_path/'src'/'assets'
    first=import_stl_as_environment_asset(stl,assets)
    second=import_stl_as_environment_asset(stl,assets)
    assert first!=second
    for p in [first,second]:
        assert (p/'package.xml').exists()
        assert (p/'CMakeLists.txt').exists()
        assert list((p/'urdf').glob('*.xacro'))
        assert list((p/'meshes'/'visual').glob('*.stl'))
        assert list((p/'meshes'/'collision').glob('*.stl'))

def test_asset_name_sanitize():
    assert sanitize_asset_name('My Box-1')=='my_box_1'
