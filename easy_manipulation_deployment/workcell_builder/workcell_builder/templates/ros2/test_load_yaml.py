import importlib.util
import types
import sys
from pathlib import Path
import pytest

# Stub out ROS-specific modules to allow importing demo.launch without ROS installed
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

# ``demo.launch.py`` depends on xacro and yaml during import. Skip this test
# module entirely when those dependencies are not available.
pytest.importorskip("xacro")
pytest.importorskip("yaml")

module_path = Path(__file__).resolve().parent / 'launch' / 'demo.launch.py'
spec = importlib.util.spec_from_file_location('demo_launch', module_path)
if spec is None or spec.loader is None:
    raise ImportError(f"Cannot load module from {module_path}")
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_load_yaml_handles_missing_package(monkeypatch):
    """load_yaml should return None when the package cannot be found."""
    monkeypatch.setattr(
        demo,
        'get_package_share_directory',
        lambda pkg: (_ for _ in ()).throw(Exception('missing package')),
    )
    assert demo.load_yaml('missing', 'config.yaml') is None


def test_load_yaml_reads_file(tmp_path, monkeypatch):
    """load_yaml should return parsed YAML content when available."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    yaml_file = pkg_dir / 'config.yaml'
    yaml_file.write_text('value: 1')
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    assert demo.load_yaml('pkg', yaml_file.name) == {'value': 1}


def test_load_yaml_missing_file(tmp_path, monkeypatch):
    """A missing YAML file should raise FileNotFoundError."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    with pytest.raises(FileNotFoundError):
        demo.load_yaml('pkg', 'missing.yaml')


def test_load_yaml_returns_none_on_parse_error(tmp_path, monkeypatch):
    """Invalid YAML content should cause load_yaml to return None."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    bad_yaml = pkg_dir / 'bad.yaml'
    # Missing closing bracket results in a YAML parsing error
    bad_yaml.write_text('list: [1, 2')
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    assert demo.load_yaml('pkg', bad_yaml.name) is None
