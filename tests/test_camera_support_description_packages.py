from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENV = ROOT / 'assets' / 'environment'

ASSETS = [
    ('tslot_camera_frame_description', 'tslot_camera_frame'),
    ('vslot_camera_frame_description', 'vslot_camera_frame'),
    ('pipe_camera_stand_description', 'pipe_camera_stand'),
    ('flat_plate_camera_bracket_description', 'flat_plate_camera_bracket'),
    ('overhead_camera_gantry_description', 'overhead_camera_gantry'),
    ('table_clamp_camera_mount_description', 'table_clamp_camera_mount'),
]


def test_package_structure():
    for package_name, asset_name in ASSETS:
        package_dir = ENV / package_name
        assert (package_dir / 'CMakeLists.txt').exists()
        assert (package_dir / 'package.xml').exists()
        assert (package_dir / 'launch' / f'view_{asset_name}.launch.py').exists()
        assert (package_dir / 'urdf' / f'{asset_name}.urdf.xacro').exists()
        assert (package_dir / 'urdf' / f'test_{asset_name}.urdf.xacro').exists()
        assert (package_dir / 'rviz' / f'view_{asset_name}.rviz').exists()
        assert (package_dir / 'meshes' / 'visual' / f'{asset_name}.stl').exists()
        assert (package_dir / 'meshes' / 'collision' / f'{asset_name}.stl').exists()


def test_cmake_install_pattern():
    required = 'install(DIRECTORY launch meshes rviz urdf\n  DESTINATION share/${PROJECT_NAME})'
    for package_name, _ in ASSETS:
        content = (ENV / package_name / 'CMakeLists.txt').read_text(encoding='utf-8')
        assert required in content


def test_urdf_mesh_and_camera_mount_references():
    for package_name, asset_name in ASSETS:
        urdf = (ENV / package_name / 'urdf' / f'{asset_name}.urdf.xacro').read_text(encoding='utf-8')
        assert f'package://{package_name}/meshes/visual/' in urdf
        assert f'package://{package_name}/meshes/collision/' in urdf
        assert ('camera_mount_link' in urdf) or ('camera_plate_link' in urdf)


def test_workcell_builder_yaml_wrappers_exist():
    for package_name, asset_name in ASSETS:
        yaml_path = ENV / package_name / f'{asset_name}.yaml'
        assert yaml_path.exists()


def test_bundle_includes_camera_support_package_name():
    content = (ROOT / 'workcell_builder' / 'workcell_builder' / 'src_workcell_scene_bundle.cpp').read_text(encoding='utf-8')
    assert 'assets/environment/' in content
    assert 'object_name + "_description"' in content
