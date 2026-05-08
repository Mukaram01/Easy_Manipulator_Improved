from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]

def load(path):
    return yaml.safe_load((ROOT/path).read_text(encoding="utf-8"))

def test_catalog_entries_exist():
    assert load("catalog/capabilities/robots/robot_ur5.yaml")["robot"]["id"]=="ur5"
    assert load("catalog/capabilities/end_effectors/ee_robotiq_2f.yaml")["end_effector"]["id"]=="robotiq_2f_85"
    assert load("catalog/capabilities/environment_assets/asset_table.yaml")["asset"]["id"]
    assert load("catalog/capabilities/environment_assets/asset_bin.yaml")["asset"]["id"]

def test_ur5_packages_exist():
    r=load("catalog/capabilities/robots/robot_ur5.yaml")["robot"]
    assert (ROOT/r["urdf_or_xacro"]).exists()
    assert r["moveit_config_package"]=="ur5_moveit_config"
