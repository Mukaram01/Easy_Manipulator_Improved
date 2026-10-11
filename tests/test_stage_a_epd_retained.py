import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_epd_retained as adapter
from test_stage_a_live_fragment_identity import live

def test_transform_is_explicit_and_rejects_channel_resize_or_padding_changes():
    p=adapter.preprocessing()
    adapter.check_preprocessing(p)
    for key,value in [('input_order','BGR'),('epd_order','RGB'),('resize','NEAREST'),('padding',1),('scale',1)]:
        bad=copy.deepcopy(p);bad[key]=value
        with pytest.raises(ValueError):adapter.check_preprocessing(bad)

def test_mask_mapping_preserves_all_roi_support_without_ids():
    mask=np.zeros((4,4),np.float32);mask[0,1]=.9;mask[3,3]=.8
    mapped=adapter.map_mask(mask,[2,4,6,8])
    assert mapped.shape==(256,256) and mapped.dtype==np.bool_
    assert mapped[2,1] and mapped[3,2] and mapped.sum()==2
    with pytest.raises(ValueError):adapter.map_mask(mask,[2,4,5,8])
    mask[0,0]=np.nan
    with pytest.raises(ValueError):adapter.map_mask(mask,[2,4,6,8])

def test_zero_detections_and_duplicate_associations_fail_closed():
    o,r,rgb,ids=live()
    assert adapter.associate(o,r,rgb,ids,[],[])['detections']==[]
    # Masks remain genuine inputs to this function; synthetic masks are tests only.
    det=dict(class_index=1,label='cube',confidence=.9,bbox=[0,0,2,2])
    mask=np.zeros(ids.shape,dtype=bool);mask[1,1]=True
    out=adapter.associate(o,r,rgb,ids,[det,det],[mask,mask])
    assert all(d['association']=='REJECTED' for d in out['detections'])

@pytest.mark.parametrize('kind',['rgb_hash','id_hash','stale','duplicate','missing','scene'])
def test_metadata_and_image_rejections(kind):
    o,r,rgb,ids=live()
    if kind=='rgb_hash':r['rgb_sha256']='wrong'
    if kind=='id_hash':r['id_sha256']='wrong'
    if kind=='stale':r['acquisition']['session']='stale'
    if kind=='duplicate':r['draw_bindings'][1]['item_id']=r['draw_bindings'][0]['item_id']
    if kind=='missing':o['shapes'].pop()
    if kind=='scene':r['renderer']['scene_id']+=1
    with pytest.raises(ValueError):adapter.associate(o,r,rgb,ids,[],[])

@pytest.mark.parametrize('kind',['mixed','background','unknown'])
def test_masks_reject_unresolved_identity(kind):
    o,r,rgb,ids=live();m=np.zeros(ids.shape,bool)
    if kind=='mixed':m[1,1]=m[2,2]=True
    if kind=='background':m[0,0]=True
    if kind=='unknown':ids=ids.copy();ids[1,1]=42;m[1,1]=True
    det=dict(class_index=1,label='cube',confidence=.9,bbox=[0,0,8,8])
    if kind=='unknown':
        with pytest.raises(ValueError):adapter.associate(o,r,rgb,ids,[det],[m])
    else:assert adapter.associate(o,r,rgb,ids,[det],[m])['accepted']==0


def test_retained_actual_capture_hashes_and_alignment():
    folder=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/same_fragment/material_binding/attempt'
    o,r,rgb,ids,binding=adapter.load_capture(folder)
    assert rgb.shape==(256,256,3) and ids.dtype==np.uint32
    assert binding['visual_collision_draw_binding']=='PASS'


def test_actual_rgb_bgr_preparation_and_pixel_cell_inverse():
    rgb=np.zeros((256,256,3),np.uint8);rgb[:]=[10,30,90]
    assert np.all(adapter.prepared_bgr(rgb)==[90,30,10])
    roi=np.zeros((1,1),np.float32);roi[0,0]=1
    # All four 512 centres within original cell (20,30) map to that same cell.
    for y in (40,41):
        for x in (60,61):
            m=adapter.map_mask(roi,[x,y,x+1,y+1]);assert m[20,30] and m.sum()==1
