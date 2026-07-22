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


def _snap(scene='scene1', camera='cam1', ts='2026-07-22T00:00:00Z', oid='o1'):
    return {'schema_version':'workcell_perception_snapshot/v1','scene_id':scene,'camera_id':camera,'timestamp':ts,'frame_id':'camera_frame','objects':[{'object_id':oid,'label':'part','confidence':0.9,'centroid':[0.1,0.2,0.3]}]}


def test_disabled_mode_is_clean_without_subscription_intent():
    adapter = mod.PerceptionSourceAdapter({'status':'NOT_APPLICABLE','scene_id':'scene1','camera':{'camera_id':'cam1'}})
    assert adapter.mode == 'disabled'
    assert adapter.status()['state'] == 'DISABLED'
    assert adapter.status()['object_count'] == 0
    assert adapter.accept_live_payload(_snap()) is None


def test_deterministic_replay_no_loop_by_default(tmp_path):
    replay = tmp_path / 'snapshots.jsonl'
    replay.write_text(json.dumps(_snap(ts='2026-07-22T00:00:00Z', oid='o1'))+'\n'+json.dumps(_snap(ts='2026-07-22T00:00:01Z', oid='o2'))+'\n', encoding='utf-8')
    adapter = mod.PerceptionSourceAdapter({'mode':'replay','scene_id':'scene1','camera':{'camera_id':'cam1'},'replay':{'path':str(replay),'rate_hz':5}})
    assert adapter.next_replay_snapshot()['objects'][0]['object_id'] == 'o1'
    assert adapter.next_replay_snapshot()['objects'][0]['object_id'] == 'o2'
    assert adapter.next_replay_snapshot() is None
    assert adapter.status()['state'] == 'STALE'
    assert adapter.status()['reason'] == 'replay exhausted'


def test_live_waiting_ready_stale_without_repeated_warning():
    adapter = mod.PerceptionSourceAdapter({'mode':'live','scene_id':'scene1','camera':{'camera_id':'cam1','frame_id':'camera_frame'},'freshness_timeout_s':0.01})
    assert adapter.status()['state'] == 'WAITING'
    assert adapter.accept_live_payload(_snap()) is not None
    assert adapter.status()['state'] == 'READY'
    adapter.refresh_staleness(now=adapter.last_wall_time + 1.0)
    first = adapter.status()
    adapter.refresh_staleness(now=adapter.last_wall_time + 2.0)
    second = adapter.status()
    assert first['state'] == second['state'] == 'STALE'
    assert first['reason'] == second['reason'] == 'freshness timeout exceeded'


def test_scene_camera_mismatch_rejected_and_source_switch_clears():
    adapter = mod.PerceptionSourceAdapter({'mode':'live','scene_id':'scene1','camera':{'camera_id':'cam1'}})
    assert adapter.accept_live_payload(_snap(scene='wrong', camera='cam1')) is None
    assert adapter.status()['state'] == 'FAILED'
    adapter.configure({'mode':'disabled','scene_id':'scene2','camera':{'camera_id':'cam2'}})
    assert adapter.status()['state'] == 'DISABLED'
    assert adapter.status()['object_count'] == 0


def test_malformed_replay_fails_precisely(tmp_path):
    replay = tmp_path / 'bad.json'
    bad = _snap(); bad['objects'].append(dict(bad['objects'][0]))
    replay.write_text(json.dumps([bad]), encoding='utf-8')
    adapter = mod.PerceptionSourceAdapter({'mode':'replay','scene_id':'scene1','camera':{'camera_id':'cam1'},'replay':{'path':str(replay)}})
    status = adapter.status()
    assert status['state'] == 'FAILED'
    assert 'duplicate object id' in status['reason']


def test_no_motion_or_epd_processing_added_to_connector():
    text = SCRIPT.read_text(encoding='utf-8')
    forbidden = ['move_group', 'FollowJointTrajectory', 'create_client', 'create_service', 'send_goal', 'robot_motion']
    assert not any(token in text for token in forbidden)
