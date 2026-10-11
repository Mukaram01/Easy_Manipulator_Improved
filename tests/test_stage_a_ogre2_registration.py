"""Projection-only arithmetic is deliberately insufficient for contact authority."""
import copy
from fractions import Fraction as F
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_ogre2_registration as r


def fixture():
    p=[[2.,0.,0.,0.],[0.,2.,0.,0.],[0.,0.,-1.,-.04],[0.,0.,-1.,0.]]
    v=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    m=dict(attached_camera_count=1,projection=p,view=v)
    report=dict(width=512,height=512,depth_width=512,depth_height=512,segmentation_width=512,segmentation_height=512,
        depth_channels=1,depth_format='FLOAT32',segmentation_channels=3,segmentation_format='R8G8B8',
        depth_frames=3,segmentation_frames=3,shared_render_batches=3,native_camera_matrices=[copy.deepcopy(m) for _ in range(3)])
    rgb=np.zeros((512,512,3),dtype=np.uint8);rgb[100:120,100:120]=255
    depth=np.ones((512,512),dtype=np.float32)
    labels=np.zeros_like(rgb);labels[100:120,100:120]=11;labels[100:120,140:160]=22
    return report,rgb,depth,labels


def test_projection_difference_uses_exact_outward_bound_and_no_authority():
    a,rgb,depth,labels=acquisition_fixture();a['native_camera_matrices'][1]['projection'][0][0]=1.9999998807907104
    for e in a['acquisition_events']:
        if e['camera']==1 and e['state'] is not None:
            e['state']['projection'][0][0]=1.9999998807907104
            e['state']['rs_projection'][0][0]=1.9999998807907104
    result=r.analyse(a,rgb,depth,labels)
    expected=256*abs(F(2)-F(1.9999998807907104))/F(1.9999998807907104)
    assert F(result['projection_only_registration_upper_px'])>=expected
    assert not result['contact_authority'] and result['total_registration_bound_px'] is None
    assert result['decision']=='BLOCKED'


@pytest.mark.parametrize('case',['blank','infinite','missing_labels','wrong_resolution','wrong_view','nonfinite','wrong_batch','wrong_format'])
def test_invalid_or_uncertifiable_images_rejected(case):
    a,rgb,depth,labels=acquisition_fixture()
    if case=='blank':rgb[:]=0
    if case=='infinite':depth[:]=np.inf
    if case=='missing_labels':labels[:]=0
    if case=='wrong_resolution':a['depth_width']=511
    if case=='wrong_view':a['native_camera_matrices'][1]['view'][0][3]=.2
    if case=='nonfinite':a['native_camera_matrices'][1]['projection'][0][0]=float('nan')
    if case=='wrong_batch':a['segmentation_frames']=2
    if case=='wrong_format':a['segmentation_format']='RGB_FLOAT32'
    with pytest.raises(ValueError):r.analyse(a,rgb,depth,labels)


def acquisition_fixture():
    a,rgb,depth,labels=fixture();a['native_session_id']='test-session'
    events=[]
    def event(batch,camera,phase,state=None):
        events.append(dict(sequence=len(events),session=a['native_session_id'],batch=batch,camera=camera,phase=phase,state=state))
    for batch in (1,2,3):
        event(batch,-1,'scene_pre_render')
        for camera in range(3):
            state=dict(a['native_camera_matrices'][camera],camera_id=camera+10,name=f'camera{camera}',
                rs_projection=a['native_camera_matrices'][camera]['projection'],near=.02,far=5.,
                projection_ab=[1.,-.02],attached_matches=True,configured_width=512,configured_height=512,viewport=[0,0,512,512],viewport_source='getLastViewport')
            if camera==1:
                state['depth_conversion']={'DepthCamera':dict(present=True,fragment_program='DepthCameraFS_GLSL',source_file='depth_camera_fs.glsl',uniforms={'projectionParams':[1.,-.004],'near':[.02],'far':[5.]}),'DepthCameraFinal':dict(present=True,fragment_program='DepthCameraFinalFS_GLSL',source_file='depth_camera_final_fs.glsl',uniforms={'near':[.02],'far':[5.]})}
            event(batch,camera,'render_begin');event(batch,camera,'scene_pre',copy.deepcopy(state))
            event(batch,camera,'scene_post',copy.deepcopy(state));event(batch,camera,'render_end')
        for camera in range(3):event(batch,camera,'readback')
        event(batch,-1,'scene_post_render')
    a['acquisition_events']=events
    return a,rgb,depth,labels


def test_acquisition_contract_keeps_unknown_shader_and_all_pixels_excluded():
    result=r.analyse(*acquisition_fixture())
    assert result['camera_callback_state']=='PASS_STATIC_CPU_WITNESS'
    assert result['qualified_pixel_count']==0
    assert result['qualified_depth_error_m'] is None
    assert result['total_registration_bound_px'] is None


@pytest.mark.parametrize('case',['missing','stale','session','identity','offset','projection','viewport','order','readback'])
def test_acquisition_witness_rejects_inconsistent_or_missing_state(case):
    a,rgb,depth,labels=acquisition_fixture()
    pre=next(e for e in a['acquisition_events'] if e['phase']=='scene_pre')
    if case=='missing':a.pop('acquisition_events')
    if case=='stale':pre['batch']=0
    if case=='session':pre['session']='other'
    if case=='identity':pre['state']['camera_id']=999
    if case=='offset':pre['state']['view'][0][3]=.01
    if case=='projection':pre['state']['projection'][0][0]=3.
    if case=='viewport':pre['state']['viewport']=[0,0,511,512]
    if case=='order':a['acquisition_events'][2]['sequence']=44
    if case=='readback':a['acquisition_events']=[e for e in a['acquisition_events'] if not(e['phase']=='readback' and e['batch']==3 and e['camera']==1)]
    with pytest.raises(ValueError):r.analyse(a,rgb,depth,labels)


@pytest.mark.parametrize('case',['occlusion','touching','subpixel_edge','discontinuity','shifted_depth','shifted_labels'])
def test_unqualified_visibility_and_raster_regions_never_admit_pixels(case):
    a,rgb,depth,labels=acquisition_fixture()
    if case=='occlusion':labels[105:115,105:115]=22
    if case=='touching':labels[100:120,120:140]=22
    if case=='subpixel_edge':rgb[99,100]=128
    if case=='discontinuity':depth[110:115,100:120]=2.
    if case=='shifted_depth':
        depth[:]=np.inf;depth[100:120,100:120]=.9;depth[100:120,140:160]=1.1
        depth=np.roll(depth,4,axis=1)
    if case=='shifted_labels':labels=np.roll(labels,4,axis=1)
    result=r.analyse(a,rgb,depth,labels)
    assert result['qualified_pixel_count']==0 and not result['contact_authority']


@pytest.mark.parametrize('case',['missing_depth_material','nonfinite_uniform','changing_uniform','missing_attached_match','wrong_dimensions','rs_xy_mismatch'])
def test_incomplete_native_configuration_rejected(case):
    a,rgb,depth,labels=acquisition_fixture()
    e=next(e for e in a['acquisition_events'] if e['phase']=='scene_pre' and e['camera']==1)
    state=e['state']
    if case=='missing_depth_material':state.pop('depth_conversion')
    if case=='nonfinite_uniform':state['depth_conversion']['DepthCamera']['uniforms']['projectionParams'][0]=float('nan')
    if case=='changing_uniform':state['depth_conversion']['DepthCamera']['uniforms']['near']=[.03]
    if case=='missing_attached_match':state.pop('attached_matches')
    if case=='wrong_dimensions':state['configured_width']=511
    if case=='rs_xy_mismatch':state['rs_projection']=copy.deepcopy(state['rs_projection']);state['rs_projection'][0][0]=3.
    with pytest.raises(ValueError):r.analyse(a,rgb,depth,labels)


def test_recorded_native_acquisition_passes_cpu_contract_without_pixel_authority():
    import gzip,json
    root=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/ogre2_acquisition'
    a=json.loads(gzip.decompress((root/'acquisition_readback.json.gz').read_bytes()))
    rgb=np.frombuffer(gzip.decompress((root/'rgb8.gz').read_bytes()),np.uint8).reshape(512,512,3)
    depth=np.frombuffer(gzip.decompress((root/'depth.f32.gz').read_bytes()),np.float32).reshape(512,512)
    labels=np.frombuffer(gzip.decompress((root/'labels.rgb8.gz').read_bytes()),np.uint8).reshape(512,512,3)
    result=r.analyse(a,rgb,depth,labels)
    assert result['camera_callback_state']=='PASS_STATIC_CPU_WITNESS'
    assert result['qualified_pixel_count']==0 and result['decision']=='BLOCKED'
    assert result['projection_only_registration_upper_px']<.00003524


def test_recorded_blank_svga_report_rejected_as_blank_not_as_valid_capture():
    import json
    root=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/ogre2_image_production'
    a=json.loads((root/'svga_readback.json').read_text())
    _,rgb,depth,labels=acquisition_fixture();rgb[:]=0;depth[:]=np.inf;labels[:]=0
    with pytest.raises(ValueError,match='blank RGB'):r.analyse(a,rgb,depth,labels)
