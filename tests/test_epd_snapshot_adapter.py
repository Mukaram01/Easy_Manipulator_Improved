import json, subprocess, sys
from pathlib import Path

def test_adapter_generates_payload(tmp_path):
    profile=tmp_path/'perception_profile.yaml'
    detected=tmp_path/'detected.yaml'
    profile.write_text(json.dumps({'scene_id':'scene1','perception':{'provider':'epd','camera':{'camera_id':'cam1','frame_id':'camera_frame'}}}))
    detected.write_text(json.dumps({'schema_version':'detected_objects/v1','scene_id':'scene1','camera_id':'cam1','timestamp':'2026-07-22T00:00:00Z','frame_id':'camera_frame','objects':[{'id':'o1','label':'cube','confidence':0.9,'pose':{'frame_id':'camera_frame','xyz':[1,2,3],'orientation_xyzw':[0,0,0,1]}}]}))
    out=tmp_path/'bridge.json'; summary=tmp_path/'summary.json'
    p=subprocess.run([sys.executable,'scripts/epd_snapshot_adapter.py','--profile',str(profile),'--input',str(detected),'--output',str(out),'--summary',str(summary)],capture_output=True,text=True)
    assert p.returncode==0
    payload=json.loads(out.read_text())
    assert payload['dry_run_only'] is True
    assert payload['runtime_execution']['auto_execute'] is False



def _load_adapter():
    import importlib.util
    spec = importlib.util.spec_from_file_location("epd_snapshot_adapter", "scripts/epd_snapshot_adapter.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _load_generator():
    import importlib.util
    spec = importlib.util.spec_from_file_location("generate_workcell_from_cell_definition", "scripts/generate_workcell_from_cell_definition.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_valid_localization_and_tracking_snapshots():
    mod = _load_adapter()
    snap = {"schema_version":"workcell_perception_snapshot/v1","scene_id":"s1","camera_id":"cam1","timestamp":"2026-07-22T00:00:00Z","frame_id":"camera_color_optical_frame","objects":[{"object_id":"obj1","track_id":"trk1","label":"part","confidence":0.7,"pose":{"frame_id":"camera_color_optical_frame","position":[0.1,0.2,0.3],"orientation_xyzw":[0,0,0,1]},"attributes":{"colour":"blue"}}]}
    assert mod.validate_normalized_snapshot(snap, expected_scene_id="s1", expected_camera_id="cam1") == []
    snap["objects"] = [{"track_id":"trk2","label":"part","confidence":1.0,"centroid":[0.1,0.2,0.3]}]
    assert mod.validate_normalized_snapshot(snap, expected_scene_id="s1", expected_camera_id="cam1") == []


def test_malformed_snapshot_rejection_and_scene_camera_mismatch():
    mod = _load_adapter()
    snap = {"schema_version":"workcell_perception_snapshot/v1","scene_id":"wrong","camera_id":"wrong_cam","timestamp":"now","frame_id":"camera","objects":[{"object_id":"dup","label":"part","confidence":1.2,"pose":{"frame_id":"camera","position":[0,0,float('nan')],"orientation_xyzw":[0,0,0,2]}},{"object_id":"dup","label":"part","confidence":0.5,"centroid":[0,0,0]}]}
    errors = mod.validate_normalized_snapshot(snap, expected_scene_id="scene", expected_camera_id="cam")
    assert any("scene_id mismatch" in e for e in errors)
    assert any("camera_id mismatch" in e for e in errors)
    assert any("confidence" in e for e in errors)
    assert any("non-finite" in e for e in errors)
    assert any("normalized" in e for e in errors)
    assert any("duplicate" in e for e in errors)


def test_deterministic_adapter_config_generation_and_disabled_scene():
    gen = _load_generator()
    cell = {"cell":{"id":"suction_test"},"camera":{"enabled":True,"camera_id":"cam","frame_id":"camera_frame"},"perception":{"enabled":True,"epd_input":{"topic":"/epd","message_type":"std_msgs/msg/String"},"required_object_classes":["part"],"confidence_threshold":0.6},"task":{"id":"pick","source_object":"part_a"}}
    first = gen._build_perception_adapter_config(cell, {"id":"pick"}, [])
    second = gen._build_perception_adapter_config(cell, {"id":"pick"}, [])
    assert first == second
    assert first["status"] == "READY"
    assert first["normalized_output_contract"]["schema_version"] == "workcell_perception_snapshot/v1"
    disabled = gen._build_perception_adapter_config({"cell":{"id":"plain"},"camera":{"enabled":False}}, {}, [])
    assert disabled["status"] == "NOT_APPLICABLE"


def test_incomplete_perception_backed_scene_rejected():
    gen = _load_generator()
    cell = {"cell":{"id":"bad"},"camera":{"enabled":True,"camera_id":"UNKNOWN_CAMERA","frame_id":"UNKNOWN_FRAME"},"perception":{"enabled":True}}
    try:
        gen._build_perception_adapter_config(cell, {}, [])
    except ValueError as exc:
        assert "Invalid perception-backed scene metadata" in str(exc)
    else:
        raise AssertionError("expected invalid perception scene to fail")
