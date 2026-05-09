from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf

def test_generate_perception_replay_preview(tmp_path):
    (tmp_path/'perception_profile.yaml').write_text('{"perception": {"object_mapping": {"place_target_ref":"bin_1"}}}')
    (tmp_path/'sample_detected_objects.yaml').write_text('{"schema_version":"detected_objects/v1","objects":[{"id":"o1","label":"cube","confidence":0.9,"pose":{"xyz":[0,0,0]}}]}')
    r=wf.generate_perception_replay_preview(tmp_path)
    assert r['status'] in {'PERCEPTION_REPLAY_READY','PERCEPTION_REPLAY_WARN'}
    assert (tmp_path/'runtime_bridge_payload.preview.json').exists()
