"""CPU contract tests; native graphics runs are explicit and separately bounded."""
import copy
import hashlib
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_fragment_identity as api


def capture():
    rgb=np.zeros((4,4,3),np.uint8);ids=np.zeros((4,4),np.uint32)
    rgb[1,1]=[255,0,0];ids[1,1]=16777217
    rgb[2,2]=[0,255,0];ids[2,2]=4000000022
    r=dict(session='test',frame=1,width=4,height=4,shared_scene_pass=True,
        rgb_format='RGBA8_UNORM',id_format='R32_UINT',samples=1,
        opaque=True,blending=False,depth_test=True,depth_write=True,inventory_complete=True,
        inventory=[dict(id=16777217,native_entity='a'),dict(id=4000000022,native_entity='b')],
        rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
        id_sha256=hashlib.sha256(ids.astype('<u4').tobytes()).hexdigest())
    return r,rgb,ids


def test_distinct_large_integer_ids():
    r,rgb,ids=capture();result=api.validate_fragment_capture(r,rgb,ids,'test',1)
    assert result['ids']==[16777217,4000000022]
    assert result['contact_authority'] is False


def test_preserved_manual_gpu_capture_actual_bytes():
    import gzip
    import json
    p=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/same_fragment/accepted_manual'
    r=json.loads(gzip.decompress((p/'capture.json.gz').read_bytes()))
    acceptance=json.loads((p/'acceptance.json').read_text())
    rgb_bytes=gzip.decompress((p/'capture.json.rgb8.gz').read_bytes())
    id_bytes=gzip.decompress((p/'capture.json.ids.u32.gz').read_bytes())
    rgb=np.frombuffer(rgb_bytes,dtype=np.uint8).reshape(r['height'],r['width'],3)
    ids=np.frombuffer(id_bytes,dtype='<u4').reshape(r['height'],r['width'])
    # These hashes attest retained readback bytes, not a new graphics run.
    r['rgb_sha256']=acceptance['rgb_sha256'];r['id_sha256']=acceptance['id_sha256']
    api.validate_fragment_capture(r,rgb,ids,r['session'],r['frame'])
    assert dict(zip(*np.unique(ids,return_counts=True)))=={0:40612,16777217:21756,4000000022:3168}
    assert ids[128,128]==16777217 and np.any(rgb) and np.ptp(rgb)>0
    assert [a['format'] for a in r['actual_attachments']]==[32856,33334]
    assert r['render_pass_count']==1 and r['post_pass_depth_test'] and r['post_pass_depth_write']


@pytest.mark.parametrize('change',[
    lambda r:r.update(session='stale'),lambda r:r.update(frame=0),
    lambda r:r.update(inventory_complete=False),lambda r:r.update(shared_scene_pass=False),lambda r:r.update(id_format='RGBA8_UNORM'),
    lambda r:r.update(samples=4),lambda r:r.update(opaque=False),
    lambda r:r.update(blending=True),lambda r:r.update(depth_test=False),
    lambda r:r.update(inventory=[dict(id=1,native_entity='a')]),
    lambda r:r['inventory'].append(copy.deepcopy(r['inventory'][0])),
    lambda r:r['inventory'][1].update(native_entity='a'),
    lambda r:r.update(rgb_sha256='changed'),lambda r:r.update(id_sha256='changed'),
])
def test_invalid_capture_rejected(change):
    r,rgb,ids=capture();change(r)
    with pytest.raises(ValueError):api.validate_fragment_capture(r,rgb,ids,'test',1)


def test_unique_mask_identity_without_pose_export():
    r,rgb,ids=capture();mask=ids==16777217
    result=api.mask_identity(r,rgb,ids,mask,'test',1,r['rgb_sha256'])
    assert result==16777217


@pytest.mark.parametrize('kind',['mixed','background','empty','wrong_shape','rgb_changed','invalid_id','float_id'])
def test_ambiguous_or_changed_mask_rejected(kind):
    r,rgb,ids=capture();mask=ids==16777217
    if kind=='mixed':mask=ids!=0
    if kind=='background':mask[0,0]=True
    if kind=='empty':mask[:]=False
    if kind=='wrong_shape':mask=mask[:2]
    if kind=='rgb_changed':rgb[0,0]=1
    if kind=='invalid_id':ids[1,1]=4294967295
    if kind=='float_id':ids=ids.astype(float)
    with pytest.raises(ValueError):api.mask_identity(r,rgb,ids,mask,'test',1,r['rgb_sha256'])


def test_mrt_target_reservation_immediately_precedes_target_creation():
    # Ogre2.2 requires reserve before addTargetPass; exercising it graphically
    # would abort, so guard the fixture's explicit construction order statically.
    import re
    source=(Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd/fragment_mrt.hh').read_text()
    assert re.search(r'nd->setNumTargetPass\(1\);\s*auto target=nd->addTargetPass\("fragment_mrt"\);',source)


def test_native_scene_graph_preparation_precedes_manual_workspace_update():
    import re
    source=(Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd/fragment_mrt.hh').read_text()
    pattern=r'sm->updateSceneGraph\(\);\s*workspace->_beginUpdate\(true\);workspace->_update\(\);'
    assert re.search(pattern,source)
    assert not re.search(pattern,source.replace('sm->updateSceneGraph();',''))
    assert not re.search(pattern,source.replace('sm->updateSceneGraph();','').replace('workspace->_update();','workspace->_update();sm->updateSceneGraph();'))
