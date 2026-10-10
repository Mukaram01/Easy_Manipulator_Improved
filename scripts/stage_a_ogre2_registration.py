#!/usr/bin/env python3
"""Exact projection-only diagnostic for the opt-in native Ogre 2 image probe.

This does NOT bound rasterization, shader depth linearization or physical-state
alignment. It cannot export a qualified total registration bound or authority.
"""
from fractions import Fraction as F
import numpy as np
from stage_a_physics_contact import require,outward


def analyse(report,rgb,depth,labels):
    require(all(report.get(k)==512 for k in ('width','height','depth_width','depth_height','segmentation_width','segmentation_height')),'inconsistent image resolution')
    require(report.get('depth_channels')==1 and report.get('depth_format')=='FLOAT32' and
        report.get('segmentation_channels')==3 and report.get('segmentation_format')=='R8G8B8','unsupported image format')
    require(report.get('depth_frames')==report.get('segmentation_frames')==report.get('shared_render_batches')==3,'incomplete render batches')
    require(rgb.dtype==np.uint8 and rgb.shape==(512,512,3) and
        depth.dtype==np.float32 and depth.shape==(512,512) and labels.dtype==np.uint8 and labels.shape==rgb.shape,'invalid image buffers')
    require(int(rgb.min())!=int(rgb.max()),'blank RGB')
    require(np.all(labels[:,:,0]==labels[:,:,1]) and np.all(labels[:,:,1]==labels[:,:,2]),'invalid semantic-label encoding')
    require(set(np.unique(labels[:,:,0]))=={0,11,22},'missing or unsupported segmentation identities')
    for label in (11,22):
        values=depth[labels[:,:,0]==label]
        require(np.any(np.isfinite(values)&(values>=.02)&(values<=5.)),'no finite visible object depth')
    matrices=report.get('native_camera_matrices',[])
    require(len(matrices)==3 and all(m.get('attached_camera_count')==1 for m in matrices),'actual attached-camera inventory required')
    for m in matrices:
        for key in ('projection','view'):
            a=np.asarray(m[key]);require(a.shape==(4,4) and np.all(np.isfinite(a)),'finite 4x4 native matrix required')
    require(all(m['view']==matrices[0]['view'] for m in matrices),'different camera views')
    projections=[[[F(v) for v in row] for row in m['projection']] for m in matrices]
    for p in projections:
        require(p[0][0]>0 and p[1][1]>0 and p[0][1:]==[0,0,0] and
            [p[1][0],*p[1][2:]]==[0,0,0] and p[3]==[0,0,-1,0], 'unsupported off-axis projection')
    # Same view and centered perspective: u = W/2 * (1 + f*x/(-z)).
    # Within the union of image frusta, |x/z| <= 1/min(f). Thus this
    # exact rational expression bounds coordinate disagreement from the
    # recorded projection coefficients, before rasterization/shader errors.
    bound=max(F(256)*(max(p[axis][axis] for p in projections)-min(p[axis][axis] for p in projections))/
        min(p[axis][axis] for p in projections) for axis in (0,1))
    return dict(decision='BLOCKED',image_production='PASS',
        scope='static native fixture; exact recorded-matrix coordinate discrepancy only',
        projection_only_registration_upper_px=outward(bound,True),
        native_intrinsics=[dict(fx=float(p[0][0]*256),fy=float(p[1][1]*256),cx=256,cy=256) for p in projections],
        total_registration_bound_px=None,contact_authority=False,execution_goals=0,
        outstanding_proof='acquisition-time native projection provenance, shader/rasterization registration and depth-linearization enclosure; then Gazebo/DART state alignment')
