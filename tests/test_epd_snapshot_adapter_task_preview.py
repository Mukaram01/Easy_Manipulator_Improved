import json, subprocess, sys

def test_preview_outputs_generated(tmp_path):
    profile=tmp_path/'p.json'; det=tmp_path/'d.json'; out=tmp_path/'o.json'; s=tmp_path/'s.json'; ss=tmp_path/'ss.json'; m=tmp_path/'m.json'
    profile.write_text(json.dumps({'perception':{'object_mapping':{'place_target_ref':'bin_1'}}}))
    det.write_text(json.dumps({'schema_version':'detected_objects/v1','objects':[{'id':'o1','label':'cube','confidence':0.1,'pose':{'xyz':[0,0,0]},'dimensions_xyz':[1,1,1]}]}))
    p=subprocess.run([sys.executable,'scripts/epd_snapshot_adapter.py','--profile',str(profile),'--input',str(det),'--output',str(out),'--summary',str(s),'--selected-summary',str(ss),'--markers',str(m)],capture_output=True,text=True)
    assert p.returncode==0 and out.exists() and s.exists() and ss.exists() and m.exists()
