from __future__ import annotations

import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
TPL = ROOT / 'workcell_builder' / 'workcell_builder' / 'templates' / 'scenarios' / 'conveyor_sorting_live_epd_preview'


def test_template_existence_and_required_files():
    assert TPL.exists()
    required = [
        'scenario.yaml', 'environment.yaml', 'demo.launch.py',
        'preview/live_epd_detection_snapshot.json',
        'preview/live_epd_detection_mapping.yaml',
        'preview/conveyor_pick_preview.yaml',
        'preview/class_routing_table.yaml',
        'preview/class_routing_result.yaml',
        'preview/task_intent_preview.yaml',
        'preview/planning_readiness_report.yaml',
        'preview/dry_run_planning_request.yaml',
        'preview/grasp_strategy.yaml',
        'preview/emd_grasp_planner_request.yaml',
        'preview/scenario_readiness_report.yaml',
    ]
    for rel in required:
        assert (TPL / rel).exists(), rel


def test_scene_structure_work_zones_routing_and_conveyor_flow():
    scenario = yaml.safe_load((TPL / 'scenario.yaml').read_text())
    env = yaml.safe_load((TPL / 'environment.yaml').read_text())

    assert scenario['scene_name'] == 'conveyor_sorting_live_epd_preview'
    for zone in ['detection_zone_1', 'pick_zone_1', 'place_zone_box', 'place_zone_bottle', 'reject_zone']:
        assert zone in scenario['work_zones']

    assert scenario['class_routing']['box'] == 'place_zone_box'
    assert scenario['class_routing']['bottle'] == 'place_zone_bottle'
    assert scenario['class_routing']['unknown'] == 'reject_zone'

    flow = scenario['conveyor_flows'][0]
    assert flow['name'] == 'conveyor_flow_1'
    assert flow['from_zone'] == 'detection_zone_1'
    assert flow['to_zone'] == 'pick_zone_1'
    assert flow['estimated_distance_m'] / flow['speed_mps'] > 0

    assert env['scene_name'] == 'conveyor_sorting_live_epd_preview'
    assert any(z['name'] == 'detection_zone_1' for z in env['work_zones'])


def test_readiness_and_safety_contract_and_waiting_epd():
    readiness = yaml.safe_load((TPL / 'preview' / 'scenario_readiness_report.yaml').read_text())['scenario_readiness']
    assert readiness['fake_hardware_preview_supported'] is True
    assert readiness['live_epd_bridge_supported'] is True
    assert readiness['robot_motion_commanded'] is False
    assert readiness['moveit_execute_called'] is False
    assert readiness['gripper_command_sent'] is False
    assert readiness['conveyor_command_sent'] is False
    assert readiness['real_hardware_ready'] is False
    assert readiness['live_epd_bridge_status'] == 'waiting_for_live_epd_snapshot'

    snap = json.loads((TPL / 'preview' / 'live_epd_detection_snapshot.json').read_text())
    assert snap['status'] == 'waiting_for_live_epd_snapshot'
