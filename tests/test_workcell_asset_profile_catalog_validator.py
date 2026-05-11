from pathlib import Path
import json


def test_validator_exists_and_markers_and_no_yaml():
    p = Path('scripts/validate_workcell_asset_catalog.py')
    t = p.read_text(encoding='utf-8')
    assert p.exists()
    for m in ['WORKCELL_ASSET_CATALOG: PASS', 'WORKCELL_ASSET_CATALOG: WARN', 'WORKCELL_ASSET_CATALOG: FAIL']:
        assert m in t
    assert 'import yaml' not in t.lower() and 'pyyaml' not in t.lower()


def test_profiles_required_fields_and_ids():
    robot = json.loads(Path('workcell_builder/workcell_builder/config/compatibility_profiles/robots/ur5.json').read_text())
    for f in ['planning_group', 'default_tool_mount_link', 'approximate_reach_radius_m', 'base_exclusion_radius_m']:
        assert f in robot
    tool = json.loads(Path('workcell_builder/workcell_builder/config/compatibility_profiles/tools/robotiq_2f85.json').read_text())
    for f in ['tcp_frame', 'grasp_strategy_default', 'release_strategy_default']:
        assert f in tool
    air = json.loads(Path('workcell_builder/workcell_builder/config/compatibility_profiles/tools/onrobot_airpick.json').read_text())
    assert air['tool_type'] == 'suction' and air['requires_io'] is True
    cam = json.loads(Path('workcell_builder/workcell_builder/config/camera_profiles/realsense_d435i.json').read_text())
    for f in ['rgb_topic', 'depth_topic', 'camera_info_topic', 'pointcloud_topic', 'camera_id', 'frame_id', 'optical_frame_id']:
        assert f in cam


def test_healthcheck_and_golden_reference_catalog_and_forbidden_markers():
    hc = Path('scripts/validate_workcell_builder_healthcheck.py').read_text(encoding='utf-8')
    gd = Path('scripts/validate_golden_workcell_demo.py').read_text(encoding='utf-8')
    assert 'validate_workcell_asset_catalog.py' in hc
    for n in ['duplicate', 'invalid compatibility status', 'missing required field']:
        assert n in Path('scripts/validate_workcell_asset_catalog.py').read_text(encoding='utf-8').lower()
    assert 'ur5' in gd and 'robotiq_2f85' in gd and 'realsense_d435i' in gd
    all_txt = '\n'.join(Path(p).read_text(encoding='utf-8', errors='ignore').lower() for p in [
        'scripts/validate_workcell_asset_catalog.py',
        'scripts/validate_golden_workcell_demo.py'])
    for forbidden in ['getmotionplan', 'execute_trajectory', '/plan_kinematic_path', 'followjointtrajectory', 'import yaml', 'pyyaml']:
        assert forbidden not in all_txt
