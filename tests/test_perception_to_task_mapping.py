import json, subprocess, sys

def test_mapping_nearest_pick_area(tmp_path):
    profile=tmp_path/'p.json'; det=tmp_path/'d.json'; env=tmp_path/'environment_layout.yaml'
    profile.write_text(json.dumps({'perception':{'camera':{'frame_id':'camera_color_optical_frame'},'object_mapping':{'selection_policy':'nearest_pick_area','pick_area_ref':'pick_area_1','place_target_ref':'bin_1','confidence_threshold':0.5}}}))
    env.write_text(json.dumps({'assets':[{'id':'pick_area_1','pose':{'xyz':[0,0,0]}}]}))
    det.write_text(json.dumps({'schema_version':'detected_objects/v1','frame_id':'camera_color_optical_frame','objects':[{'id':'far','label':'cube','confidence':0.9,'pose':{'xyz':[2,0,0]}},{'id':'near','label':'cube','confidence':0.9,'pose':{'xyz':[0.1,0,0]}}]}))
    out=tmp_path/'out.json'; s=tmp_path/'s.json'; ss=tmp_path/'sel.json'; m=tmp_path/'m.json'
    p=subprocess.run([sys.executable,'scripts/epd_snapshot_adapter.py','--profile',str(profile),'--input',str(det),'--environment',str(env),'--output',str(out),'--summary',str(s),'--selected-summary',str(ss),'--markers',str(m)],capture_output=True,text=True)
    assert p.returncode==0
    assert json.loads(ss.read_text())['selected_object']['id']=='near'
