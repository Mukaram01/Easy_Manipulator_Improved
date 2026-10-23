"""A camera stand carries the camera; the Web3D export keeps it a physical
asset instead of a second (dropped) camera sensor."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from export_workcell_studio_web_scene_impl import _section_from_item  # noqa: E402


def test_camera_stand_is_not_a_sensor():
    stand = {'id': 'camera_01', 'display_name': 'Camera Stand', 'category': 'environment_object', 'role': 'fixture',
             'mesh_path': 'assets/imported/catalog/72f14e9dadbcf7e2.stl'}
    assert _section_from_item(stand) != 'sensors'
    for name in ('pipe_camera_stand', 'flat_plate_camera_bracket', 'overhead_camera_gantry', 'tslot_camera_frame'):
        assert _section_from_item({'id': name, 'display_name': name, 'category': 'environment_object'}) != 'sensors'


def test_cameras_stay_sensors():
    assert _section_from_item({'id': 'realsense_overhead', 'display_name': 'Fixed RealSense D435i'}) == 'sensors'
    assert _section_from_item({'id': 'cam', 'role': 'camera', 'display_name': 'Camera stand camera'}) == 'sensors'
    assert _section_from_item({'id': 'camera_01', 'display_name': 'Overhead camera'}) == 'sensors'
