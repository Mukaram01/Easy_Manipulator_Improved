from pathlib import Path
from scripts.capability_registry import load_structured_data


def _load(path):
    return load_structured_data(Path(path))[0]

def test_priority_entries_exist_and_point_to_existing_assets():
    ur5=_load('catalog/capabilities/robots/robot_ur5.yaml')['robot']
    assert Path(ur5['urdf_or_xacro']).exists()
    assert ur5['driver_status']=='optional_later'
    ee2f=_load('catalog/capabilities/end_effectors/ee_robotiq_2f.yaml')['end_effector']
    assert Path(ee2f['urdf_or_xacro']).exists()
    air=_load('catalog/capabilities/end_effectors/ee_airpick_suction.yaml')['end_effector']
    assert Path(air['urdf_or_xacro']).exists()
    d435=_load('catalog/capabilities/sensors/sensor_realsense_d435i.yaml')['sensor']
    assert Path(d435['urdf_or_xacro']).exists()
    table=_load('catalog/capabilities/environment_assets/asset_table.yaml')['asset']
    assert Path(table['urdf_or_xacro']).exists()
