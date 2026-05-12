from __future__ import annotations

import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
TPL = ROOT / 'workcell_builder' / 'workcell_builder' / 'templates' / 'scenarios' / 'conveyor_sorting_live_epd_preview'


def test_generate_template_has_environment_and_scenario_files():
    assert (TPL / 'environment.yaml').exists()
    assert (TPL / 'scenario.yaml').exists()


def test_preview_artifacts_exist():
    required = [
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


def test_component_and_routing_contracts_and_safety_flags():
    scenario = yaml.safe_load((TPL / 'scenario.yaml').read_text())
    readiness = yaml.safe_load((TPL / 'preview' / 'scenario_readiness_report.yaml').read_text())['scenario_readiness']

    assert scenario['robot']
    assert scenario['end_effector']
    assert scenario['conveyor']
    assert scenario['camera']
    assert scenario['camera_mount']

    for zone in ['detection_zone_1', 'pick_zone_1', 'place_zone_box', 'place_zone_bottle', 'reject_zone']:
        assert zone in scenario['work_zones']

    assert scenario['class_routing']['box'] == 'place_zone_box'
    assert scenario['class_routing']['bottle'] == 'place_zone_bottle'
    assert scenario['class_routing']['unknown'] == 'reject_zone'

    assert (TPL / 'preview' / 'task_intent_preview.yaml').exists()
    assert (TPL / 'preview' / 'planning_readiness_report.yaml').exists()
    assert (TPL / 'preview' / 'emd_grasp_planner_request.yaml').exists()

    assert readiness['real_hardware_ready'] is False
    assert readiness['robot_motion_commanded'] is False
    assert readiness['gripper_command_sent'] is False
    assert readiness['conveyor_command_sent'] is False

    snap = json.loads((TPL / 'preview' / 'live_epd_detection_snapshot.json').read_text())
    assert snap['status'] in {'waiting_for_live_epd_snapshot', 'sample_epd_snapshot'}
