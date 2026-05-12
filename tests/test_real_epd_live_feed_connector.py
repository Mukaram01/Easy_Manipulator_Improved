from pathlib import Path
import importlib.util
import json
import time

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'workcell_builder/workcell_builder/scripts/epd_to_workcell_snapshot_node.py'
CPP_CONSOLE = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.cpp'
WIZARD_CPP = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp'
LAUNCH_TEMPLATE = ROOT / 'workcell_builder/workcell_builder/templates/scenarios/conveyor_sorting_live_epd_preview/demo.launch.py'
NODE_PREVIEW = ROOT / 'workcell_builder/workcell_builder/scripts/conveyor_sorting_live_preview_node.py'

spec = importlib.util.spec_from_file_location('epd_connector', SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_epd_message_adapter_conversion_preserves_fields():
    payload = {
        'objects': [
            {'object_id': 'obj_1', 'label': 'box', 'confidence': 0.95, 'centroid': [0.1, 0.0, 0.7], 'world_position': [0.6, 0.0, 0.8], 'bbox_px': [20, 30, 40, 50], 'has_cloud': True}
        ]
    }
    snap = mod.build_snapshot_from_epd_payload(payload, 'realsense_d435i_1', 'camera_color_optical_frame', 'detection_zone_1')
    d = snap['detections'][0]
    assert d['class_label'] == 'box'
    assert d['estimated_xyz_camera'] == [0.1, 0.0, 0.7]
    assert d['estimated_xyz_world'] == [0.6, 0.0, 0.8]
    assert d['zone_hint'] == 'detection_zone_1'


def test_snapshot_schema_required_fields_and_missing_optional_safe():
    snap = mod.build_snapshot_from_epd_payload({'objects': [{'label': 'unknown'}]}, 'cam', 'frame', 'zone')
    for key in ['schema_version', 'source', 'runtime_mode', 'camera', 'camera_frame', 'timestamp_sec', 'detections']:
        assert key in snap
    assert snap['detections'][0]['bbox_px'] is None


def test_status_semantics_tokens_present():
    text = SCRIPT.read_text(encoding='utf-8')
    for token in ['epd_connected', 'last_msg_age_s', 'last_error', '/workcell_studio/epd_connector_status', 'stale_timeout_s']:
        assert token in text


def test_run_console_ui_epd_controls_tokens_present():
    text = CPP_CONSOLE.read_text(encoding='utf-8')
    for token in ['Real EPD Feed', 'Copy EPD Connector Command', 'Start EPD Connector', 'Stop EPD Connector', 'Refresh EPD Status', '/easy_perception_deployment/epd_localize_output', '/workcell_studio/epd_detection_snapshot_json']:
        assert token in text


def test_launch_config_tokens_and_safety_tokens_present():
    text = LAUNCH_TEMPLATE.read_text(encoding='utf-8') + CPP_CONSOLE.read_text(encoding='utf-8')
    for token in ['enable_epd_connector', 'localization_topic', 'tracking_topic', 'output_snapshot_topic', 'publish_sample_detections:=true', 'use_fake_hardware:=true', 'real_hardware_ready must remain false']:
        assert token in text


def test_live_preview_runtime_source_agnostic_snapshot_consumer():
    text = NODE_PREVIEW.read_text(encoding='utf-8')
    assert "payload.get('detections', [])" in text


def test_artifact_tokens_exist_for_connector_and_preview_outputs():
    wizard_text = WIZARD_CPP.read_text(encoding='utf-8')
    preview_text = NODE_PREVIEW.read_text(encoding='utf-8')
    combined = wizard_text + preview_text
    for token in ['preview/live_epd_detection_snapshot.json', 'preview/live_epd_detection_mapping.yaml', 'live_conveyor_sorting_status.json', 'live_task_intent_preview.yaml', 'live_emd_grasp_planner_request.yaml', 'robot_motion_commanded', 'gripper_command_sent', 'conveyor_command_sent']:
        assert token.split('/')[-1] in combined
