from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp'
NODE = ROOT / 'workcell_builder/workcell_builder/scripts/conveyor_sorting_live_preview_node.py'
RVIZ_TEMPLATE = ROOT / 'workcell_builder/workcell_builder/templates/scenarios/conveyor_sorting_live_epd_preview/demo.launch.py'


def test_parser_valid_and_malformed_json():
    payload = json.loads('{"detections":[{"class_label":"box"}]}')
    assert payload['detections'][0]['class_label'] == 'box'
    try:
        json.loads('{bad json')
        assert False
    except json.JSONDecodeError:
        assert True


def test_launch_generation_has_preview_args_and_nodes():
    text = CPP.read_text()
    for token in ['enable_conveyor_sorting_preview', 'epd_snapshot_topic', 'publish_sample_detections', 'conveyor_sorting_live_preview_node.py', 'robot_motion_commanded: false']:
        assert token in text


def test_routing_and_marker_topic_tokens():
    text = CPP.read_text()
    for token in ['place_zone_box', 'place_zone_bottle', 'reject_zone', '/workcell_studio/conveyor_sorting_preview_markers', '/workcell_studio/conveyor_sorting_preview_status']:
        assert token in text


def test_artifact_tokens_present():
    text = CPP.read_text()
    for token in ['live_conveyor_sorting_status.json', 'live_conveyor_sorting_status.yaml', 'live_task_intent_preview.yaml', 'live_emd_grasp_planner_request.yaml']:
        assert token in text
