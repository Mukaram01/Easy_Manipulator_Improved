import json, subprocess, sys
from pathlib import Path

def test_adapter_generates_payload(tmp_path):
    profile=tmp_path/'perception_profile.yaml'
    detected=tmp_path/'detected.yaml'
    profile.write_text(json.dumps({'perception':{'provider':'epd'}}))
    detected.write_text(json.dumps({'schema_version':'detected_objects/v1','objects':[{'id':'o1','label':'cube','confidence':0.9,'pose':{'xyz':[1,2,3],'rpy':[0,0,0]}}]}))
    out=tmp_path/'bridge.json'; summary=tmp_path/'summary.json'
    p=subprocess.run([sys.executable,'scripts/epd_snapshot_adapter.py','--profile',str(profile),'--input',str(detected),'--output',str(out),'--summary',str(summary)],capture_output=True,text=True)
    assert p.returncode==0
    payload=json.loads(out.read_text())
    assert payload['dry_run_only'] is True
    assert payload['runtime_execution']['auto_execute'] is False
