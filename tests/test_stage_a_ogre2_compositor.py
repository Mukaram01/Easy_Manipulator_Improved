"""Compositor readback remains bounded diagnostics until draw/raster proof exists."""
import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage_a_ogre2_registration import compositor_authority


def witness():
    return dict(session='fixture',batch=3,camera=1,phase='pass_post',pass_id='pass1',pass_type='quad',
        acquisition_sequence=10,gl=dict(status='POST_EXECUTION_STATE',fragment_program=7,
        viewport=[0,0,512,512],samples=0,depth_attachment=dict(format=36012,width=512,height=512),
        uniforms={'near':[.02],'far':[5.],'projectionParams':[.1,.2]},
        draw_time_binding='UNKNOWN',executable_provenance='UNKNOWN'))


def test_post_execution_binding_cannot_authorise_draw_samples():
    result=compositor_authority(dict(native_session_id='fixture',compositor_events=[witness()]))
    assert result['decision']=='BLOCKED'
    assert result['qualified_pixel_count']==0
    assert result['qualified_pixel_mask'].shape==(512,512)
    assert not np.any(result['qualified_pixel_mask'])
    assert result['shader_depth_error_m'] is None


@pytest.mark.parametrize('case',['wrong_shader','wrong_uniform','wrong_format','stale','projection','conversion','edge','occlusion','shifted_labels','incomplete','nonfinite','unsupported'])
def test_unknown_or_changed_gpu_state_never_admits_pixels(case):
    e=witness()
    if case=='wrong_shader':e['gl']['fragment_program']=99
    if case=='wrong_uniform':e['gl']['uniforms']['near']=[.03]
    if case=='wrong_format':e['gl']['depth_attachment']['format']=0
    if case=='stale':e['batch']=2
    if case=='projection':e['gl']['viewport']=[0,0,511,512]
    if case=='conversion':e['gl']['uniforms']['projectionParams']=[.1,.3]
    if case=='edge':e['gl']['subpixel_bits']=None
    if case=='occlusion':e['gl']['visibility']='UNKNOWN'
    if case=='shifted_labels':e['gl']['segmentation_alignment']='UNKNOWN'
    if case=='incomplete':e.pop('pass_id')
    if case=='nonfinite':e['gl']['uniforms']['near']=[float('nan')]
    if case=='unsupported':e['gl']['status']='UNSUPPORTED_GL'
    result=compositor_authority(dict(native_session_id='fixture',compositor_events=[e]))
    assert result['decision']=='BLOCKED' and not np.any(result['qualified_pixel_mask'])
    assert result['failure_reasons']


def test_uniform_disagreement_is_reported_without_residual_tolerance():
    e=witness();e['pass_resource']={'uniforms':{'near':[.03]}}
    result=compositor_authority(dict(native_session_id='fixture',compositor_events=[e]))
    assert 'CPU_GL_UNIFORM_DISAGREEMENT' in result['failure_reasons']


def test_post_pass_state_must_be_inside_matching_camera_render_call():
    e=witness()
    report=dict(native_session_id='fixture',compositor_events=[e],acquisition_events=[
        dict(sequence=0,batch=3,camera=1,phase='render_begin'),
        dict(sequence=1,batch=3,camera=1,phase='render_end'),
        dict(sequence=2,batch=3,camera=1,phase='readback')])
    result=compositor_authority(report)
    assert 'COMPOSITOR_READBACK_ASSOCIATION_INVALID' in result['failure_reasons']


def test_cli_missing_capture_writes_explicit_empty_mask_and_failure(tmp_path):
    import subprocess,json
    script=Path(__file__).resolve().parents[1]/'scripts/stage_a_ogre2_registration.py'
    output=tmp_path/'decision.json'
    result=subprocess.run([sys.executable,str(script),str(tmp_path/'absent.json'),str(output)],capture_output=True,text=True)
    assert result.returncode==2
    record=json.loads(output.read_text())
    assert record['decision']=='BLOCKED' and record['qualified_pixel_count']==0
    assert 'CAPTURE_INPUT_INVALID' in record['failure_reasons']
    assert (tmp_path/'decision.json.mask.uint8').read_bytes()==bytes(512*512)


@pytest.mark.parametrize('malformed',['null','[]','{"compositor_events":[null]}','{"compositor_events":42}','{"compositor_events":[{"gl":{"uniforms":null}}]}'])
def test_malformed_capture_schema_emits_blocked_result(tmp_path,malformed):
    import subprocess,json
    script=Path(__file__).resolve().parents[1]/'scripts/stage_a_ogre2_registration.py'
    source=tmp_path/'capture.json';source.write_text(malformed);output=tmp_path/'decision.json'
    result=subprocess.run([sys.executable,str(script),str(source),str(output)],capture_output=True,text=True)
    assert result.returncode==2
    record=json.loads(output.read_text())
    assert record['decision']=='BLOCKED' and record['failure_reasons']
    assert (tmp_path/'decision.json.mask.uint8').read_bytes()==bytes(512*512)


def test_recorded_compositor_is_bound_to_render_calls_but_admits_no_pixels():
    import gzip,json,hashlib
    root=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/ogre2_compositor'
    report=json.loads(gzip.decompress((root/'compositor_readback.json.gz').read_bytes()))
    result=compositor_authority(report)
    assert len(report['compositor_events'])==29
    assert 'COMPOSITOR_READBACK_ASSOCIATION_INVALID' not in result['failure_reasons']
    assert 'CPU_GL_UNIFORM_DISAGREEMENT' not in result['failure_reasons']
    assert result['decision']=='BLOCKED' and not np.any(result['qualified_pixel_mask'])
    provenance=json.loads((root/'provenance.json').read_text())
    for binary in provenance['gl_program_binaries'].values():
        data=gzip.decompress((root/binary['file']).read_bytes())
        assert hashlib.sha256(data).hexdigest()==binary['sha256']
    assert gzip.decompress((root/'qualified_mask.uint8.gz').read_bytes())==bytes(512*512)
