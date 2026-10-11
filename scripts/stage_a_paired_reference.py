#!/usr/bin/env python3
"""Offline native-resolution reference checks. Never modifies/qualifies EPD masks."""
import hashlib
import math
import cv2
import numpy as np


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def optics(n, fovy):
    # Coordinates are array indices: centre of sample i is i, edge is i-.5.
    focal = n / (2 * math.tan(fovy / 2))
    return dict(width=n, height=n, fx=focal, fy=focal, cx=(n-1)/2, cy=(n-1)/2,
                pixel_centre='index_plus_half_in_edge_coordinates', aspect=1., fovy=fovy)


def validate_pair(report, ids256, ids512):
    require(report['schema']=='workcell_paired_native/v1', 'wrong paired schema')
    require(report['acquisition_order']==[256,512] and report['separate_render_passes']==2, 'wrong acquisition order/pass count')
    states=report['frozen']
    require(len(states)==3 and states[0]==states[1]==states[2], 'scene/camera/acquisition changed between passes')
    state=states[0]
    require(state['step']==2 and state['stamp_ns']==state['renderutil_stamp_ns']==2000000 and state['update_epoch']==2, 'stale acquisition')
    joins=state['joins']
    expected={x['renderer_id'] for x in joins}
    require(len(joins)==len(expected)==2 and expected=={65517,65523} and len({x['collision_id'] for x in joins})==2, 'missing/duplicate identity join')
    require(len(report['optics'])==len(report['passes'])==len(report['rasters'])==2, 'incomplete pass inventory')
    for i,(n,ids) in enumerate(((256,ids256),(512,ids512))):
        require(ids.shape==(n,n) and ids.dtype==np.dtype('<u4'), 'wrong native resolution/type')
        require(set(np.unique(ids))=={0,*expected}, 'missing/unexpected native identity')
        require(digest(ids.tobytes())==report['rasters'][i]['sha256'], 'modified native raster')
        require(report['optics'][i]==optics(n,state['camera']['fovy']), 'shifted pixel centres or camera optics')
        p=report['passes'][i]
        require(p['width']==p['height']==n and p['render_pass_count']==1 and p['id_format']=='R32_UINT' and p['depth_test'] is True and p['depth_write'] is True, 'invalid depth-tested native pass')
        require(p['actual_viewport']==[0,0,n,n] and p['post_pass_depth_test'] is True and p['post_pass_depth_write'] is True,'shifted viewport/disabled depth')
        require(len(p['actual_attachments'])==2,'missing typed MRT attachment')
        for attachment,format in zip(p['actual_attachments'],(32856,33334)):
            require(attachment['format']==format and attachment['width']==attachment['height']==n,'incorrect native MRT attachment')
    require(len(report['pixel_rays'])==2,'missing resolution ray inventory')
    for n,rays in zip((256,512),report['pixel_rays']):
        require([r['yx'] for r in rays]==[[0,0],[n//2-1,n//2-1],[n-1,n-1],[105,37]],'missing/shifted ray samples')
        validate_rays(rays,state['view'],state['camera'],n,report.get('arithmetic_replay',{}).get(str(n)))
    return True


def comparable(rgb, ids, historical_rgb, historical_ids, scene, historical_scene):
    return rgb==historical_rgb and np.array_equal(ids,historical_ids) and scene==historical_scene


def metrics(mask, ids, identity):
    require(mask.shape==ids.shape==(512,512) and mask.dtype==np.bool_, 'mask/reference mismatch')
    obj=ids==identity
    background=mask & (ids==0)
    cross=mask & (ids!=0) & ~obj
    intersection=int(np.count_nonzero(mask & obj));union=int(np.count_nonzero(mask | obj))
    distance=cv2.distanceTransform((~obj).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    kernel=np.ones((3,3),np.uint8)
    mb=mask & ~cv2.erode(mask.astype(np.uint8),kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
    ob=obj & ~cv2.erode(obj.astype(np.uint8),kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
    md=cv2.distanceTransform((~mb).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    od=cv2.distanceTransform((~ob).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    errors=np.concatenate((od[mb],md[ob]))
    samples=distance[background]
    # Connected components are diagnostics only. No mask edit affects association.
    count,_,stats,_=cv2.connectedComponentsWithStats(background.astype(np.uint8),8)
    return dict(foreground=int(mask.sum()),object_pixels=int(obj.sum()),intersection=intersection,union=union,
        background=int(background.sum()),cross_object=int(cross.sum()),missing_object=int((obj & ~mask).sum()),
        iou=intersection/union if union else None,spill_distance_max=float(samples.max()) if samples.size else 0.,
        spill_distance_mean=float(samples.mean()) if samples.size else 0.,
        spill_distance_gt_sqrt2=int((samples>math.sqrt(2)+1e-6).sum()),
        background_components=stats[1:,cv2.CC_STAT_AREA].tolist(),
        boundary_error_max=float(errors.max()) if errors.size else None,
        boundary_error_mean=float(errors.mean()) if errors.size else None,
        background_yx=np.argwhere(background).tolist())


def validate_rays(rays,view,camera,n,exact_replay=None):
    o=optics(n,camera['fovy']);inverse=np.linalg.inv(np.asarray(view).reshape(4,4))
    require(bool(rays),'missing actual camera rays')
    if exact_replay is not None:
        require(len(exact_replay)==len(rays),'incomplete arithmetic replay')
    for index,ray in enumerate(rays):
        v,u=ray['yx']
        require(type(u) is int and type(v) is int and 0<=u<n and 0<=v<n,'shifted/out-of-range pixel centre')
        local=np.array([(u-o['cx'])/o['fx'],-(v-o['cy'])/o['fy'],-1.])
        world=inverse[:3,:3]@local;world/=np.linalg.norm(world)
        origin=inverse[:3,:3]@(local*camera['near'])+inverse[:3,3]
        require(np.allclose(ray['origin'],origin,rtol=0,atol=2e-6),'actual camera pixel origin/optics mismatch')
        if exact_replay is None:
            require(np.allclose(ray['direction'],world,rtol=0,atol=2e-6),'actual camera pixel direction/optics mismatch')
        else:
            expected=exact_replay[index]
            require(expected['resolution']==n and expected['yx']==ray['yx'],'changed arithmetic replay sample')
            for key in ('origin','direction'):
                require(np.asarray(ray[key],dtype='<f4').tobytes()==np.asarray(expected[key],dtype='<f4').tobytes(),'float32 camera arithmetic differs')


def verified_bundle(folder,expected_manifest):
    """An external pinned manifest is mandatory before trusting retained bytes."""
    import gzip
    import json
    from pathlib import Path
    folder=Path(folder)
    require(digest((folder/'sha256.json').read_bytes())==expected_manifest,'changed paired manifest')
    manifest=json.loads((folder/'sha256.json').read_text());artifacts={}
    for name,expected in manifest.items():
        require(not Path(name).is_absolute() and '..' not in Path(name).parts,'invalid artifact path')
        raw=gzip.decompress((folder/(name+'.gz')).read_bytes())
        require(digest(raw)==expected,'changed paired artifact: '+name)
        artifacts[name]=raw
    inventory=json.loads(artifacts['retention_inventory.json'])
    pre=json.loads(artifacts['paired_preflight.json'])
    for path,name in inventory['executed_sources'].items():
        require(digest(artifacts[name])==pre['dependencies'][path],'changed executed source: '+path)
    require(digest(artifacts['executed_binary.bin'])==json.loads(artifacts['preflight.json'])['binary_sha256'],'changed native executable')
    return artifacts
