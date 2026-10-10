#!/usr/bin/env python3
"""Exact projection-only diagnostic for the opt-in native Ogre 2 image probe.

This does NOT bound rasterization, shader depth linearization or physical-state
alignment. It cannot export a qualified total registration bound or authority.
"""
from fractions import Fraction as F
import numpy as np
from stage_a_physics_contact import require,outward


def validate_acquisitions(report):
    """Validate CPU camera callback witnesses, never GPU pass/visibility authority."""
    events=report.get('acquisition_events',[]);session=report.get('native_session_id')
    require(isinstance(session,str) and bool(session) and bool(events),'missing acquisition witnesses')
    for i,e in enumerate(events):
        require(e.get('sequence')==i and e.get('session')==session,'stale or unordered acquisition event')
    cursor=0;identities={};stable={};viewport_known=True
    def take(batch,camera,phase):
        nonlocal cursor
        require(cursor<len(events),'incomplete acquisition events')
        e=events[cursor];cursor+=1
        require((e.get('batch'),e.get('camera'),e.get('phase'))==(batch,camera,phase),'incorrect render/readback ordering')
        return e
    for batch in (1,2,3):
        take(batch,-1,'scene_pre_render')
        for camera in range(3):
            take(batch,camera,'render_begin');passes=0
            while cursor<len(events) and events[cursor].get('phase')=='scene_pre':
                for phase in ('scene_pre','scene_post'):
                    state=take(batch,camera,phase).get('state',{})
                    require(state.get('attached_camera_count')==1 and state.get('attached_matches') is True,'wrong attached camera')
                    identity=(state.get('camera_id'),state.get('name'))
                    require(identity[0] is not None and isinstance(identity[1],str) and bool(identity[1]),'missing camera identity')
                    require(identity==identities.setdefault(camera,identity),'changed camera identity')
                    require(state.get('configured_width')==state.get('configured_height')==512,'incorrect acquisition image dimensions')
                    reference=report['native_camera_matrices'][camera]
                    for key in ('view','projection'):
                        require(state.get(key)==reference[key],'acquisition versus final camera disagreement')
                    for key in ('view','projection','rs_projection'):
                        a=np.asarray(state.get(key));require(a.shape==(4,4) and np.all(np.isfinite(a)),'invalid acquisition matrix')
                    require(np.isfinite(state.get('near',np.nan)) and np.isfinite(state.get('far',np.nan)) and 0<state['near']<state['far'],'invalid native clipping')
                    ab=np.asarray(state.get('projection_ab'));require(ab.shape==(2,) and np.all(np.isfinite(ab)),'missing native depth projection parameters')
                    require(state['rs_projection'][:2]==state['projection'][:2],'unsupported render-system XY projection conversion')
                    if camera==1:
                        conversion=state.get('depth_conversion',{})
                        for material in ('DepthCamera','DepthCameraFinal'):
                            resource=conversion.get(material,{})
                            require(resource.get('present') is True and bool(resource.get('fragment_program')) and bool(resource.get('source_file')),'missing depth shader resource')
                            for uniform in (('near','far','projectionParams') if material=='DepthCamera' else ('near','far')):
                                values=np.asarray(resource.get('uniforms',{}).get(uniform),dtype=float)
                                require(values.shape==((2,) if uniform=='projectionParams' else (1,)) and np.all(np.isfinite(values)),'invalid depth conversion uniform')
                    signature={k:state[k] for k in ('camera_id','name','view','projection','rs_projection','near','far','projection_ab')}
                    if camera==1:signature['depth_conversion']=state['depth_conversion']
                    require(signature==stable.setdefault(camera,signature),'unstable native camera state')
                    vp=state.get('viewport')
                    if vp is None:viewport_known=False
                    else:require(vp==[0,0,512,512],'unsupported native viewport')
                passes+=1
            require(passes>0,'missing native render callbacks')
            take(batch,camera,'render_end')
        for camera in range(3):take(batch,camera,'readback')
        take(batch,-1,'scene_post_render')
    require(cursor==len(events) and len({v[0] for v in identities.values()})==len({v[1] for v in identities.values()})==3,'ambiguous camera inventory')
    return dict(camera_callback_state='PASS_STATIC_CPU_WITNESS',
        viewport_state='LAST_VIEWPORT_RECORDED' if viewport_known else 'UNKNOWN_AT_SOME_CALLBACKS')


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
    acquisition=validate_acquisitions(report)
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
    return dict(**acquisition,qualified_pixel_count=0,excluded_pixel_count=512*512,
        qualified_pixel_registration_error_px=None,qualified_depth_error_m=None,
        pixel_exclusion_reason='active compositor uniforms, raster coverage, visibility and GPU arithmetic unqualified',
        binary32_to_binary64_matrix_conversion_error=0,
        decision='BLOCKED',image_production='PASS',
        scope='static native fixture; exact recorded-matrix coordinate discrepancy only',
        projection_only_registration_upper_px=outward(bound,True),
        native_intrinsics=[dict(fx=float(p[0][0]*256),fy=float(p[1][1]*256),cx=256,cy=256) for p in projections],
        total_registration_bound_px=None,contact_authority=False,execution_goals=0,
        outstanding_proof='active compositor-pass shader/uniform and rasterization/precision enclosure; then Gazebo/DART state alignment')
