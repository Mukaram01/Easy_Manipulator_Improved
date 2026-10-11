"""TEST FIXTURES ONLY: these identities never qualify genuine EPD masks."""
import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import stage_a_paired_reference as paired


def fixture():
    state = dict(step=2, stamp_ns=2000000, update_epoch=2, renderutil_stamp_ns=2000000,
                 inventory={'visuals':[6,10], 'geometry':[.025,.025,.025]},
                 joins=[{'renderer_id':65517,'collision_id':11}, {'renderer_id':65523,'collision_id':7}],
                 transforms=[[1,0,0,.35],[1,0,0,.45]], view=list(np.eye(4).ravel()),
                 projection=list(np.eye(4).ravel()), camera={'aspect':1.,'near':.01,'fovy':1.0471975803375244})
    images=[np.zeros((256,256),dtype='<u4'),np.zeros((512,512),dtype='<u4')]
    for a in images: a[1,1]=65517;a[2,2]=65523
    r=dict(schema='workcell_paired_native/v1', acquisition_order=[256,512], separate_render_passes=2,
           frozen=[copy.deepcopy(state) for _ in range(3)], optics=[paired.optics(n,state['camera']['fovy']) for n in (256,512)],
           passes=[dict(width=n,height=n,render_pass_count=1,id_format='R32_UINT',depth_test=True,depth_write=True) for n in (256,512)],
           rasters=[{'sha256':paired.digest(a.tobytes())} for a in images])
    for n,p in zip((256,512),r['passes']):
        p.update(actual_viewport=[0,0,n,n],post_pass_depth_test=True,post_pass_depth_write=True,
                 actual_attachments=[dict(format=f,width=n,height=n) for f in (32856,33334)])
    r['pixel_rays']=[]
    for n in (256,512):
        rays=[];o=paired.optics(n,state['camera']['fovy'])
        for v,u in ((0,0),(n//2-1,n//2-1),(n-1,n-1),(105,37)):
            d=np.array([(u-o['cx'])/o['fx'],-(v-o['cy'])/o['fy'],-1.])
            rays.append(dict(yx=[v,u],direction=(d/np.linalg.norm(d)).tolist(),origin=(d*.01).tolist()))
        r['pixel_rays'].append(rays)
    return r,images


def test_matching_native_pair_accepts():
    r,images=fixture();paired.validate_pair(r,*images)


@pytest.mark.parametrize('bad', ['resolution','pixel_centre','geometry','identity','stale','hash','update','projection'])
def test_bad_pair_fails_closed(bad):
    r,images=fixture()
    if bad=='resolution': images[1]=images[1][::2,::2]
    if bad=='pixel_centre': r['optics'][1]['cx']+=.5
    if bad=='geometry': r['frozen'][1]['inventory']['geometry'][0]=.03
    if bad=='identity': images[1][images[1]==65523]=0
    if bad=='stale': r['frozen'][2]['stamp_ns']-=1
    if bad=='hash': r['rasters'][1]['sha256']='0'*64
    if bad=='update': r['frozen'][1]['update_epoch']+=1
    if bad=='projection': r['frozen'][1]['projection'][0]+=1
    with pytest.raises(ValueError): paired.validate_pair(r,*images)


def test_pixel_centres_preserve_optics_without_resizing():
    a,b=[paired.optics(n,1.0471975803375244) for n in (256,512)]
    for u in (0,21,127,255):
        assert ((2*u+.5)-b['cx'])/b['fx']==pytest.approx((u-a['cx'])/a['fx'],abs=1e-15)
    assert a['cx']==127.5 and b['cx']==255.5


def test_historical_comparison_requires_exact_bytes_and_scene():
    a=np.zeros((256,256),dtype='<u4'); rgb=bytes(256*256*3)
    witness={'geometry':[.025]*3,'camera':[.4,-.357,.0125]}
    assert paired.comparable(rgb,a,rgb,a,witness,witness)
    assert not paired.comparable(bytes([1])+rgb[1:],a,rgb,a,witness,witness)
    assert not paired.comparable(rgb,a,rgb,a,witness,{'geometry':[.03]*3})


def test_spill_metrics_single_pixel_and_proxy_disagreement():
    ids=np.zeros((512,512),dtype='<u4');ids[100:102,100:102]=65517
    mask=ids==65517;mask[102,101]=True
    m=paired.metrics(mask,ids,65517)
    assert (m['background'],m['missing_object'],m['intersection'],m['union'])==(1,0,4,5)
    assert m['iou']==.8 and m['spill_distance_max']==1


def test_ray_evidence_rejects_half_pixel_shift():
    camera=dict(fovy=1.0471975803375244,near=.01)
    o=paired.optics(512,camera['fovy']);u,v=37,105
    d=np.array([(u-o['cx'])/o['fx'],-(v-o['cy'])/o['fy'],-1.]);d/=np.linalg.norm(d)
    ray={'yx':[v,u],'direction':d.tolist(),'origin':(d*(.01/-d[2])).tolist()}
    paired.validate_rays([ray],np.eye(4),camera,512)
    bad=copy.deepcopy(ray);bad['yx'][1]+=.5
    with pytest.raises(ValueError):paired.validate_rays([bad],np.eye(4),camera,512)


@pytest.mark.parametrize('variable',['LD_LIBRARY_PATH','LD_PRELOAD','LD_AUDIT'])
def test_changed_runtime_environment_fails_before_capture(monkeypatch,variable):
    import stage_a_paired_capture as capture
    pinned=capture.runtime_environment()
    monkeypatch.setenv(variable,'/changed/runtime')
    with pytest.raises(ValueError):capture.check_environment(pinned)


def test_changed_preflight_file_hash_rejects(tmp_path):
    import stage_a_paired_capture as capture
    f=tmp_path/'scene';f.write_bytes(b'original');hashes={str(f):paired.digest(f.read_bytes())}
    capture.check_hashes(hashes);f.write_bytes(b'changed')
    with pytest.raises(ValueError):capture.check_hashes(hashes)


@pytest.mark.parametrize('bad',['missing_rays','shifted_viewport','depth_disabled','wrong_attachment'])
def test_native_pass_evidence_rejects(bad):
    r,images=fixture()
    if bad=='missing_rays':r['pixel_rays']=[]
    if bad=='shifted_viewport':r['passes'][1]['actual_viewport']=[1,0,512,512]
    if bad=='depth_disabled':r['passes'][1]['post_pass_depth_test']=False
    if bad=='wrong_attachment':r['passes'][1]['actual_attachments'][1]['width']=256
    with pytest.raises(ValueError):paired.validate_pair(r,*images)


def test_float32_ray_replay_is_exact_and_rejects_changed_direction():
    r,_=fixture();rays=r['pixel_rays'][0];replay=[dict(resolution=256,**copy.deepcopy(ray)) for ray in rays]
    # A native f32 ray must match its independent arithmetic replay bit for bit.
    rays[1]['direction'][0]+=3.65e-6;replay[1]['direction']=copy.deepcopy(rays[1]['direction'])
    paired.validate_rays(rays,np.eye(4),r['frozen'][0]['camera'],256,replay)
    rays[1]['direction'][0]+=1e-7
    with pytest.raises(ValueError):paired.validate_rays(rays,np.eye(4),r['frozen'][0]['camera'],256,replay)


BUNDLE=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/same_fragment/paired_native_reference'
BUNDLE_SHA='927cabf64d305c73bbfd48226c009cebb3454ce62d0ef20b3ef69dc197d1a264'


def test_genuine_retained_native_pair_and_spill_replay(tmp_path):
    import json
    import stage_a_paired_capture as capture
    artifacts=paired.verified_bundle(BUNDLE,BUNDLE_SHA)
    for name,raw in artifacts.items():
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
    actual=capture.analyse(tmp_path)
    expected=json.loads(artifacts['diagnostic.json'])
    assert actual==expected
    assert actual['historical_comparison']=='EXACT_COMPARABILITY_ESTABLISHED'
    assert actual['classification']=='MODEL OUTPUT' and actual['strict_accepted_associations']==0
    assert actual['renderer_resolution_disagreement']==376
    assert [d['raw_native']['background'] for d in actual['detections']]==[456,635,41]
    assert [d['roi_native']['background'] for d in actual['detections']]==[303,564,41]
    assert [d['raw_native']['missing_object'] for d in actual['detections']]==[75,0,8329]
    assert [d['crop_foreground_removed'] for d in actual['detections']]==[153,71,0]
    assert all(d['crop_foreground_added']==d['crop_background_added']==d['raw_native']['cross_object']==0 for d in actual['detections'])
    assert all(len(d['raw_native']['background_yx'])==d['raw_native']['background'] for d in actual['detections'])


@pytest.mark.parametrize('name',['capture.json.rgb8','capture.json.ids.u32','capture.json.native512.ids.u32',
                                  'paired.checked.json','offline_ray_replay.json'])
def test_modified_retained_artifact_rejects(tmp_path,name):
    import gzip
    import shutil
    shutil.copytree(BUNDLE,tmp_path/'bundle');folder=tmp_path/'bundle'
    path=folder/(name+'.gz');raw=gzip.decompress(path.read_bytes());path.write_bytes(gzip.compress(bytes([raw[0]^1])+raw[1:],mtime=0))
    with pytest.raises(ValueError,match='changed paired artifact'):paired.verified_bundle(folder,BUNDLE_SHA)


def test_modified_manifest_rejects(tmp_path):
    import shutil
    shutil.copytree(BUNDLE,tmp_path/'bundle');folder=tmp_path/'bundle'
    p=folder/'sha256.json';p.write_bytes(p.read_bytes()+b' ')
    with pytest.raises(ValueError,match='changed paired manifest'):paired.verified_bundle(folder,BUNDLE_SHA)
