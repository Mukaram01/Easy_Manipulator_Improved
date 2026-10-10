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


def compositor_authority(report):
    """Post-pass GL observations cannot assert draw-time bindings or a GPU bound.

    Always emit an explicit empty qualified mask; no caller-supplied PASS or
    numeric precision field can upgrade this diagnostic to contact authority.
    """
    malformed=not isinstance(report,dict)
    if malformed:report={}
    reasons={'DRAW_TIME_BINDING_AND_GPU_ARITHMETIC_UNQUALIFIED',
        'RASTER_COVERAGE_VISIBILITY_AND_TEXTURE_UNIT_MAPPING_UNQUALIFIED'}
    if malformed:reasons.add('CAPTURE_SCHEMA_INVALID')
    events=report.get('compositor_events',[])
    if not isinstance(events,list):reasons.add('CAPTURE_SCHEMA_INVALID');events=[]
    if not events:reasons.add('MISSING_COMPOSITOR_EVENTS')
    observed=0
    acquisitions=report.get('acquisition_events',[])
    if not isinstance(acquisitions,list) or any(not isinstance(a,dict) for a in acquisitions):
        acquisitions=[];reasons.add('CAPTURE_SCHEMA_INVALID')
    for index,e in enumerate(events):
        if not isinstance(e,dict):reasons.add('CAPTURE_SCHEMA_INVALID');continue
        if e.get('sequence')!=index:reasons.add('INCOMPLETE_COMPOSITOR_SEQUENCE')
        boundary=e.get('acquisition_sequence',-1)
        begins=[a.get('sequence',-1) for a in acquisitions if a.get('phase')=='render_begin' and (a.get('batch'),a.get('camera'))==(e.get('batch'),e.get('camera'))]
        ends=[a.get('sequence',-1) for a in acquisitions if a.get('phase')=='render_end' and (a.get('batch'),a.get('camera'))==(e.get('batch'),e.get('camera'))]
        reads=[a.get('sequence',-1) for a in acquisitions if a.get('phase')=='readback' and (a.get('batch'),a.get('camera'))==(e.get('batch'),e.get('camera'))]
        if not(len(begins)==len(ends)==len(reads)==1 and begins[0]<boundary<=ends[0]<reads[0]):
            reasons.add('COMPOSITOR_READBACK_ASSOCIATION_INVALID')
        if e.get('session')!=report.get('native_session_id') or e.get('batch') not in (1,2,3):
            reasons.add('STALE_COMPOSITOR_STATE')
        if not e.get('pass_id') or e.get('phase')!='pass_post':reasons.add('INCOMPLETE_PASS_IDENTITY')
        gl=e.get('gl',{})
        if not isinstance(gl,dict):reasons.add('CAPTURE_SCHEMA_INVALID');continue
        if gl.get('status')!='POST_EXECUTION_STATE':reasons.add('UNSUPPORTED_OR_MISSING_GL_STATE')
        if not gl.get('fragment_program'):reasons.add('MISSING_ACTIVE_FRAGMENT_PROGRAM')
        else:observed+=1
        if e.get('pass_type') in ('scene','quad') and gl.get('viewport')!=[0,0,512,512]:reasons.add('UNSUPPORTED_GPU_VIEWPORT')
        if gl.get('samples') not in (0,1):reasons.add('UNSUPPORTED_MULTISAMPLING')
        if gl.get('gl_error_before') or gl.get('gl_error_after'):reasons.add('GL_QUERY_ERROR')
        # The current diagnostic never asserts program correspondence from a
        # matching CPU name or from an opaque program-binary blob.
        if gl.get('draw_time_binding')!='UNKNOWN':reasons.add('UNSUPPORTED_AUTHORITY_CLAIM')
        for name,values in gl.get('uniforms',{}).items():
            if name in ('near','far','projectionParams'):
                try:finite=np.all(np.isfinite(np.asarray(values,dtype=float)))
                except (ValueError,TypeError):finite=False
                if not finite:reasons.add('NONFINITE_DEPTH_CONVERSION')
        depth=gl.get('depth_attachment',{})
        if depth.get('format') not in (None,36012):reasons.add('UNSUPPORTED_DEPTH_ATTACHMENT')
        cpu=e.get('pass_resource',{}).get('uniforms',{})
        for name in ('near','far','projectionParams'):
            if name in cpu and name in gl.get('uniforms',{}) and cpu[name]!=gl['uniforms'][name]:
                reasons.add('CPU_GL_UNIFORM_DISAGREEMENT')
    return dict(decision='BLOCKED',qualified_pixel_count=0,excluded_pixel_count=512*512,
        qualified_pixel_mask=np.zeros((512,512),dtype=np.uint8),
        shader_depth_error_m=None,total_registration_bound_px=None,
        post_pass_fragment_observations=observed,failure_reasons=sorted(reasons),
        contact_authority=False,execution_goals=0)


if __name__=='__main__':
    import argparse,json,hashlib
    from pathlib import Path
    parser=argparse.ArgumentParser(description='Fail-closed native image/compositor diagnostic; never grants contact authority')
    parser.add_argument('capture_report',type=Path);parser.add_argument('new_result',type=Path)
    args=parser.parse_args()
    mask_path=Path(str(args.new_result)+'.mask.uint8')
    if args.new_result.exists() or mask_path.exists():parser.error('result and mask paths must be new')
    camera={};report={};input_error=None
    try:
        report=json.loads(args.capture_report.read_text())
        raw=lambda suffix,dtype,shape:np.fromfile(str(args.capture_report)+suffix,dtype).reshape(shape)
        camera=analyse(report,raw('.rgb8',np.uint8,(512,512,3)),raw('.depth.f32',np.float32,(512,512)),raw('.labels.rgb8',np.uint8,(512,512,3)))
    except (OSError,ValueError,KeyError,TypeError,AttributeError) as error:
        input_error=str(error)
    try:result=compositor_authority(report)
    except (ValueError,TypeError,KeyError,AttributeError,IndexError) as error:
        result=compositor_authority({});result['failure_reasons'].append('CAPTURE_SCHEMA_INVALID');input_error=str(error)
    mask=result.pop('qualified_pixel_mask')
    if input_error:
        result['failure_reasons'].append('CAPTURE_INPUT_INVALID');result['input_error']=input_error
    result['camera']=camera
    result['schema']='workcell_ogre2_registration/v2'
    mask.tofile(mask_path)
    result['qualified_mask']=dict(path=str(mask_path),format='uint8',width=512,height=512,
        sha256=hashlib.sha256(mask.tobytes()).hexdigest())
    args.new_result.write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(2)
