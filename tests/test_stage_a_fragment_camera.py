import copy,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_fragment_camera as camera

def separated():
    ids=np.zeros((256,256),np.uint32);ids[80:120,30:70]=11;ids[80:120,160:200]=22
    r=dict(camera_profile='separated_workpieces',acquisition=dict(camera_profile='separated_workpieces'),
        inventory=[dict(id=11),dict(id=22)])
    r['separation_witness']=camera.separation(ids,[11,22])
    return r,ids

def test_separated_view_requires_measured_pixels_and_gap():
    r,ids=separated();camera.validate_profile(r,ids)
    assert r['separation_witness']['blank_columns']==90
    for key,value in [('blank_columns',12),('pixel_counts',{})]:
        bad=copy.deepcopy(r);bad['separation_witness'][key]=value
        with pytest.raises(ValueError):camera.validate_profile(bad,ids)
    ids[:]=0;ids[80:120,30:70]=11;ids[80:120,75:115]=22
    r['separation_witness']=camera.separation(ids,[11,22])
    with pytest.raises(ValueError):camera.validate_profile(r,ids)

def test_overlap_keeps_actual_near_far_requirement():
    r,ids=separated();r['camera_profile']='authored_oblique_overlap_fixture';r['acquisition']['camera_profile']=r['camera_profile']
    with pytest.raises(ValueError):camera.validate_profile(r,ids)
    r.update(occlusion_verified=True,occlusion_witness=dict(x=35,y=85,near_id=11,far_id=22,captured_id=11))
    camera.validate_profile(r,ids)
    r['occlusion_witness']['captured_id']=22
    with pytest.raises(ValueError):camera.validate_profile(r,ids)

@pytest.mark.parametrize('kind',['unknown','stale','small','unexpected'])
def test_profile_rejects_incomplete_or_ambiguous_evidence(kind):
    r,ids=separated()
    if kind=='unknown':r['camera_profile']='unknown'
    if kind=='stale':r['acquisition']['camera_profile']='authored_oblique_overlap_fixture'
    if kind=='small':ids[80:110,30:70]=0;r['separation_witness']=camera.separation(ids,[11,22])
    if kind=='unexpected':ids[0,0]=99
    with pytest.raises(ValueError):camera.validate_profile(r,ids)


def test_authored_side_on_design_preserves_geometry_and_fits_both_boxes():
    from test_stage_a_gazebo_fragment_runner import world
    import xml.etree.ElementTree as ET
    root=world();before=ET.tostring(root);d=camera.design(root)
    assert ET.tostring(root)==before
    assert d['position']==[.4,-.357,.0125] and d['look_at']==[.4,-.217,.0125]
    assert d['projected_gap_pixels']>12 and len(d['workpieces'])==2
    for row in d['workpieces']:
        x0,y0,x1,y1=row['projected_bounds'];assert 0<x0<x1<256 and 0<y0<y1<256
