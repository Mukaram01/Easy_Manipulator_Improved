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

# Skip this test if PyYAML is not available. The demo.launch.py module
# imports ``yaml`` when it is executed below, and without PyYAML installed
# the import would raise ``ModuleNotFoundError`` during test collection.
pytest.importorskip("yaml")

module_path = Path(__file__).resolve().parent / 'launch' / 'demo.launch.py'
spec = importlib.util.spec_from_file_location('demo_launch', module_path)
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)

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
