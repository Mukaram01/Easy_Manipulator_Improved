import importlib.util
import types
import sys
from pathlib import Path
import pytest

# Stub out ROS-specific modules so the launch file can be imported without ROS installed.
launch_mod = types.ModuleType('launch')
launch_mod.LaunchDescription = object
sys.modules.setdefault('launch', launch_mod)
launch_ros_mod = types.ModuleType('launch_ros')
actions_mod = types.ModuleType('launch_ros.actions')
actions_mod.Node = object
launch_ros_mod.actions = actions_mod
sys.modules.setdefault('launch_ros', launch_ros_mod)
sys.modules.setdefault('launch_ros.actions', actions_mod)
aip_mod = types.ModuleType('ament_index_python')
packages_mod = types.ModuleType('ament_index_python.packages')
packages_mod.get_package_share_directory = lambda pkg: str(Path(pkg))
aip_mod.packages = packages_mod
sys.modules.setdefault('ament_index_python', aip_mod)
sys.modules.setdefault('ament_index_python.packages', packages_mod)

# Skip the entire module if xacro is not available, mirroring the behaviour of the launch file.
pytest.importorskip("xacro")

module_path = Path(__file__).resolve().parent / 'launch' / 'demo.launch.py'
spec = importlib.util.spec_from_file_location('demo_launch', module_path)
if spec is None or spec.loader is None:  # pragma: no cover - defensive programming
    raise ImportError(f'Cannot load module from {module_path}')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_load_yaml_requires_existing_file(tmp_path, monkeypatch):
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(tmp_path))
    with pytest.raises(FileNotFoundError):
        demo.load_yaml('pkg', 'missing.yaml')


def test_load_yaml_returns_none_on_parse_error(tmp_path, monkeypatch):
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    bad_yaml = pkg_dir / 'bad.yaml'
    bad_yaml.write_text(': - invalid')
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    assert demo.load_yaml('pkg', bad_yaml.name) is None


def test_load_yaml_handles_missing_package(monkeypatch):
    monkeypatch.setattr(
        demo,
        'get_package_share_directory',
        lambda pkg: (_ for _ in ()).throw(EnvironmentError('missing package')),
    )
    assert demo.load_yaml('missing_pkg', 'config.yaml') is None


def test_load_yaml_expands_user_paths(tmp_path, monkeypatch):
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    yaml_file = pkg_dir / 'data.yaml'
    yaml_file.write_text('value: 42')
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(tmp_path))
    data = demo.load_yaml('pkg', '~/pkg/data.yaml')
    assert data['value'] == 42
