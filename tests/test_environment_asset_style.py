import json, subprocess, tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def run_checker(strict=True):
    with tempfile.NamedTemporaryFile(suffix='.json',delete=False) as f:
        out=f.name
    cmd=['python3','scripts/check_environment_asset_style.py','--root','assets/environment','--json-out',out]
    if strict: cmd.append('--strict')
    cp=subprocess.run(cmd,cwd=ROOT,check=not strict and False)
    data=json.loads(Path(out).read_text())
    return cp.returncode,data

def _row(data,name):
    return next(r for r in data['results'] if r['name']==name)

def test_realsense_passes():
    code,data=run_checker(strict=False)
    row=_row(data,'realsense2_description')
    assert not row['errors']

def test_camera_support_packages_pass():
    _,data=run_checker(strict=False)
    for n in ['tslot_camera_frame_description','vslot_camera_frame_description','pipe_camera_stand_description','flat_plate_camera_bracket_description','overhead_camera_gantry_description','table_clamp_camera_mount_description']:
        assert not _row(data,n)['errors']

def test_standardized_environment_assets_have_required_layout():
    _,data=run_checker(strict=False)
    for n in ['table_description','workbench_description']:
        assert not _row(data,n)['errors']
