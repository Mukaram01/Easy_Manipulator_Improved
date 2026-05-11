from pathlib import Path
import json

ASSETS=["table_small","table_large","workbench","bin_small","bin_large","tray","tote_box","conveyor_placeholder","fixture_plate","pallet","pedestal","camera_stand","safety_fence_panel","robot_base_plate","calibration_cube","pick_box","cylinder_object"]

def test_curated_assets_exist_with_stl_and_urdf_and_no_symlinks_or_huge_files():
    root=Path('workcell_builder/workcell_builder/assets/environment')
    assert root.exists()
    for aid in ASSETS:
        base=root/f'{aid}_description'
        stl=base/'meshes'/f'{aid}.stl'
        urdf=base/'urdf'/f'{aid}.urdf.xacro'
        assert stl.exists() and urdf.exists()
        assert not stl.is_symlink() and not urdf.is_symlink()
        assert stl.stat().st_size < 2*1024*1024

def test_environment_assets_catalog_and_categories_and_paths():
    arr=json.loads(Path('workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json').read_text())
    ids={x['asset_id'] for x in arr}
    assert set(ASSETS).issubset(ids)
    for item in arr:
        assert not str(item['mesh_path']).startswith('/')
        assert item['category'] in {"Tables / Workbenches","Bins / Trays / Totes","Conveyors","Fixtures","Safety / Fencing","Camera Mounts","Robot Bases","Pick Objects"}

def test_no_forbidden_tokens():
    txt='\n'.join(Path(p).read_text(encoding='utf-8',errors='ignore').lower() for p in ['scripts/validate_workcell_asset_catalog.py','scripts/generate_golden_workcell_demo.py'])
    for forbidden in ['pyyaml','streamlit','getmotionplan','execute_trajectory','/plan_kinematic_path','real_hardware_enabled: true']:
        assert forbidden not in txt
