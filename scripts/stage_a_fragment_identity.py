"""Same-fragment fixture checks; renderer-local identity only, never contact authority.

No masks are generated here. Callers must preserve genuine EPD masks and verify
its preprocessing/capture provenance before using mask_identity in a live path.
"""
import hashlib
import numpy as np
from stage_a_physics_contact import require


def validate_fragment_capture(report,rgb,ids,session,frame):
    require(bool(session) and report.get('session')==session and report.get('frame')==frame,'stale capture')
    shape=(report.get('height'),report.get('width'))
    require(rgb.dtype==np.uint8 and rgb.shape==(*shape,3) and
            ids.dtype==np.uint32 and ids.shape==shape,'unsupported image buffers')
    require(report.get('shared_scene_pass') is True and report.get('samples')==1 and
            report.get('opaque') is True and report.get('blending') is False and
            report.get('depth_test') is True and report.get('depth_write') is True,
            'unsupported fragment visibility contract')
    require(report.get('rgb_format')=='RGBA8_UNORM' and report.get('id_format')=='R32_UINT','unsupported attachments')
    require(report.get('inventory_complete') is True,'incomplete visual inventory')
    inventory=report.get('inventory',[])
    values=[v.get('id') for v in inventory];entities=[v.get('native_entity') for v in inventory]
    require(values and all(type(v) is int and 0<v<4294967295 for v in values) and
            len(set(values))==len(values) and all(isinstance(v,str) and v for v in entities) and
            len(set(entities))==len(entities),'invalid or duplicate visual inventory')
    require(set(np.unique(ids))=={0,*values},'missing, invalid or unexpected visible IDs')
    require(report.get('rgb_sha256')==hashlib.sha256(rgb.tobytes()).hexdigest() and
            report.get('id_sha256')==hashlib.sha256(ids.astype('<u4').tobytes()).hexdigest(),
            'changed RGB/ID correspondence')
    return dict(ids=sorted(values),contact_authority=False,scope='renderer-local fixture only; physics mapping unknown')


def mask_identity(report,rgb,ids,mask,session,frame,epd_rgb_sha256):
    validate_fragment_capture(report,rgb,ids,session,frame)
    require(epd_rgb_sha256==report['rgb_sha256'],'EPD RGB image mismatch')
    require(isinstance(mask,np.ndarray) and mask.dtype==np.bool_ and mask.shape==ids.shape and np.any(mask),'invalid mask')
    values=np.unique(ids[mask])
    require(len(values)==1 and int(values[0]) not in (0,4294967295),'mixed, missing or ambiguous mask ID')
    return int(values[0])
