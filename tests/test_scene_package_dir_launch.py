"""The qualified Stage-A launch can run on a Builder-saved cell's own generated files."""
import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LAUNCH = REPO / 'scenes/ur5_2f_test/launch/demo.launch.py'
REQUIRED = ('environment.yaml', 'cell_definition.yaml', 'urdf/scene.urdf.xacro', 'urdf/arm_hand.srdf.xacro',
            'config/ros2_controllers.yaml', 'config/moveit_collision_objects.yaml')


def load():
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('qualified_demo_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def package(tmp_path, skip=()):
    for rel in REQUIRED:
        if rel not in skip:
            (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
            (tmp_path / rel).write_text('x')
    return tmp_path


def test_a_bound_cell_package_replaces_only_the_scene_package_share(tmp_path, monkeypatch):
    launch = load()
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda name: f'/installed/{name}')
    assert launch._bind_scene_package_dir(str(package(tmp_path))) == str(tmp_path.resolve())
    assert launch._share('ur5_2f_test') == str(tmp_path.resolve())
    assert launch._share('robotiq_85_description') == '/installed/robotiq_85_description'   # robot/tool still installed
    assert launch._bind_scene_package_dir('') is None
    assert launch._share('ur5_2f_test') == '/installed/ur5_2f_test'                      # default: reference package


def test_an_ungenerated_cell_is_refused_before_anything_launches(tmp_path):
    launch = load()
    with pytest.raises(RuntimeError, match='urdf/arm_hand.srdf.xacro'):
        launch._bind_scene_package_dir(str(package(tmp_path, skip=('urdf/arm_hand.srdf.xacro',))))
    assert launch._SCENE_PACKAGE_DIR is None


def test_every_scene_package_read_goes_through_the_binding():
    source = LAUNCH.read_text()
    body = source[source.index('def _bind_scene_package_dir'):]
    assert 'get_package_share_directory(' not in body                                    # no read bypasses it
    setup = source[source.index('def _launch_setup'):]
    assert setup.index('_bind_scene_package_dir(LaunchConfiguration("scene_package_dir")') < setup.index('execution_backend')
    assert 'DeclareLaunchArgument("scene_package_dir", default_value="")' in source
