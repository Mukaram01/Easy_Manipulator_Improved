import importlib.util
import types
import sys
from pathlib import Path
import os
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

# Skip this test if required dependencies are not available.  The
# ``demo.launch.py`` module imports ``xacro`` and ``yaml`` during its
# initialisation.  When those packages are missing the import would raise
# ``ModuleNotFoundError`` at collection time which results in a hard
# failure instead of the test being reported as skipped.  ``importorskip``
# gracefully skips the entire test module when either dependency is not
# installed.
pytest.importorskip("xacro")
pytest.importorskip("yaml")

module_path = Path(__file__).resolve().parent / 'launch' / 'demo.launch.py'
spec = importlib.util.spec_from_file_location('demo_launch', module_path)
# ``spec_from_file_location`` may return ``None`` if the module cannot be
# loaded (e.g. the file is missing).  Guard against this and provide a clear
# error instead of raising ``AttributeError`` when accessing ``spec.loader``
# below.  Likewise, ``spec.loader`` itself can legitimately be ``None``
# depending on the import system.  In both cases we raise ``ImportError`` with
# context so failures are easier to diagnose.
if spec is None or spec.loader is None:
    raise ImportError(f"Cannot load module from {module_path}")

demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)

def test_to_urdf_requires_existing_file(tmp_path):
    """to_urdf should raise ``FileNotFoundError`` if the xacro file is missing."""
    missing = tmp_path / "missing.xacro"
    with pytest.raises(FileNotFoundError):
        demo.to_urdf(missing)


def test_to_urdf_rejects_non_xacro_file(tmp_path):
    """to_urdf should reject source files that do not end with ``.xacro``."""
    non_xacro = tmp_path / "robot.urdf"
    non_xacro.write_text("<robot name='test'></robot>")
    with pytest.raises(ValueError, match="xacro_path must point to a .xacro file"):
        demo.to_urdf(non_xacro)

def test_to_urdf_creates_urdf_file(tmp_path):
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    urdf_path = demo.to_urdf(str(xacro_file))
    assert urdf_path.endswith('.urdf')
    assert os.path.exists(urdf_path)
    with open(urdf_path) as f:
        content = f.read()
    assert '<robot' in content
    os.remove(urdf_path)


def test_to_urdf_respects_output_path(tmp_path):
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    custom_path = tmp_path / 'custom'
    result = demo.to_urdf(str(xacro_file), str(custom_path))
    expected = custom_path.with_suffix('.urdf')
    assert result == str(expected)
    assert expected.exists()
    with open(expected) as f:
        assert '<robot' in f.read()


def test_to_urdf_allows_filename_without_directory(tmp_path, monkeypatch):
    """Ensure to_urdf works when the output path has no directory component."""
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    # Change to the temporary directory so that a bare filename is valid
    monkeypatch.chdir(tmp_path)
    result = demo.to_urdf(str(xacro_file), 'out')
    expected = Path('out.urdf')
    assert result == str(expected)
    assert expected.exists()
    assert '<robot' in expected.read_text()

def test_to_urdf_rejects_empty_output_path(tmp_path):
    """to_urdf should raise a clear error when given an empty output path."""
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    with pytest.raises(ValueError, match="urdf_path must not be empty"):
        demo.to_urdf(str(xacro_file), '')


def test_to_urdf_rejects_directory_only_path(tmp_path):
    """to_urdf should error when output path points to a directory."""
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    with pytest.raises(ValueError, match="urdf_path must not be empty"):
        demo.to_urdf(str(xacro_file), '.')


def test_to_urdf_rejects_path_with_trailing_separator(tmp_path):
    """Ensure directories specified with a trailing slash are rejected."""
    xacro_file = tmp_path / 'robot.xacro'
    xacro_file.write_text("<robot name='test'></robot>")
    dir_path = tmp_path / 'outdir'
    dir_path.mkdir()
    with pytest.raises(ValueError, match="urdf_path must not be empty"):
        demo.to_urdf(str(xacro_file), str(dir_path) + os.path.sep)


def test_to_urdf_expands_user_paths(tmp_path, monkeypatch):
    """Paths containing ``~`` should resolve to the user's home directory."""
    # Use a temporary directory as the fake home so that ``~/`` points to a
    # location we control.
    monkeypatch.setenv("HOME", str(tmp_path))
    xacro_file = tmp_path / "robot.xacro"
    xacro_file.write_text("<robot name='test'></robot>")
    result = demo.to_urdf("~/robot.xacro", "~/out")
    expected = tmp_path / "out.urdf"
    assert result == str(expected)
    assert expected.exists()

def test_load_file_does_not_write_urdf_next_to_xacro(tmp_path):
    """``load_file`` should place generated URDFs in a temporary location."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    xacro_file = pkg_dir / 'robot.urdf.xacro'
    xacro_file.write_text("<robot name='test'></robot>")

    content = demo.load_file(str(pkg_dir), xacro_file.name)
    assert '<robot' in content
    # ``load_file`` should not leave any URDF files in the package directory
    assert list(pkg_dir.glob('*.urdf')) == []


def test_load_file_avoids_double_extension(tmp_path, monkeypatch):
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    xacro_file = pkg_dir / 'robot.urdf.xacro'
    xacro_file.write_text("<robot name='test'></robot>")

    captured = {}
    orig_to_urdf = demo.to_urdf

    def capture(xacro_path, urdf_path=None):
        path = orig_to_urdf(xacro_path, urdf_path)
        captured['path'] = path
        return path

    monkeypatch.setattr(demo, 'to_urdf', capture)

    content = demo.load_file(str(pkg_dir), xacro_file.name)
    assert '<robot' in content
    assert captured['path'].endswith('.urdf')
    assert not captured['path'].endswith('.urdf.urdf')


def test_load_file_handles_missing_package(monkeypatch):
    """``load_file`` should return ``None`` when the package is not found."""
    monkeypatch.setattr(
        demo,
        'get_package_share_directory',
        lambda pkg: (_ for _ in ()).throw(EnvironmentError('missing package')),
    )
    assert demo.load_file('does_not_exist', 'file.urdf.xacro') is None


def test_load_file_returns_none_for_missing_file(tmp_path, monkeypatch):
    """``load_file`` should return ``None`` when the target file is missing."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    assert demo.load_file('pkg', 'missing.urdf.xacro') is None


def test_load_file_returns_none_on_xacro_error(tmp_path, monkeypatch):
    """Invalid xacro files should cause ``load_file`` to return ``None``."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    bad_xacro = pkg_dir / 'bad.urdf.xacro'
    bad_xacro.write_text('<robot name="test">')  # malformed XML (no closing tag)
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))
    assert demo.load_file('pkg', bad_xacro.name) is None


def test_load_file_reads_plain_urdf(tmp_path, monkeypatch):
    """``load_file`` should read existing URDF files without invoking ``to_urdf``."""
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    urdf_file = pkg_dir / 'robot.urdf'
    urdf_file.write_text('<robot name="test"/>')

    # ``to_urdf`` should not be called when the input is already a URDF file.
    monkeypatch.setattr(demo, 'to_urdf', lambda *a, **kw: (_ for _ in ()).throw(AssertionError('to_urdf called')))
    monkeypatch.setattr(demo, 'get_package_share_directory', lambda pkg: str(pkg_dir))

    assert demo.load_file('pkg', urdf_file.name) == '<robot name="test"/>'


def test_load_yaml_requires_existing_file(tmp_path, monkeypatch):
    """``load_yaml`` should raise ``FileNotFoundError`` for missing files."""
    monkeypatch.setattr(
        demo, 'get_package_share_directory', lambda pkg: str(tmp_path)
    )
    with pytest.raises(FileNotFoundError):
        demo.load_yaml('pkg', 'missing.yaml')
