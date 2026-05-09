import json, subprocess, sys

def test_markers_include_arrow_and_selected(tmp_path):
    pth=tmp_path/'p.json'; d=tmp_path/'d.json'; out=tmp_path/'o.json'; s=tmp_path/'s.json'; m=tmp_path/'m.json'; ss=tmp_path/'ss.json'
    pth.write_text(json.dumps({'perception':{'object_mapping':{'place_target_ref':'bin_1'}}}))
    d.write_text(json.dumps({'schema_version':'detected_objects/v1','objects':[{'id':'o1','label':'cube','confidence':0.9,'pose':{'xyz':[0,0,0]}}]}))
    subprocess.check_call([sys.executable,'scripts/epd_snapshot_adapter.py','--profile',str(pth),'--input',str(d),'--output',str(out),'--summary',str(s),'--markers',str(m),'--selected-summary',str(ss)])
    data=json.loads(m.read_text())
    assert data['selected_target_id']=='o1'
    assert 'pick_to_place_arrow' in data
