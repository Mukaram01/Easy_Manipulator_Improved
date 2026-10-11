"""Synthetic arithmetic only; no ONNX evaluator, ORT session or genuine detection creation."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_onnx_mask_graph as graph


@pytest.mark.parametrize('source,shape,expected',[
    ([[0.,1.]],(1,4),[[0.,.25,.75,1.]]),
    ([[0.,1.]],(1,1),[[.5]]),
    ([[0.,1.,0.]],(1,6),[[0.,.25,.75,.75,.25,0.]]),
    ([[0.,0.],[0.,1.]],(1,1),[[.25]]),
])
def test_actual_half_pixel_linear_coordinates(source,shape,expected):
    np.testing.assert_array_equal(graph.resize_linear_half_pixel(np.array(source,dtype=np.float32),shape),np.array(expected,dtype=np.float32))


def test_fpn_asymmetric_nearest_floor():
    actual=graph.resize_asymmetric_nearest(np.array([[10.,20.]],dtype=np.float32),(1,5))
    np.testing.assert_array_equal(actual,[[10.,10.,10.,20.,20.]])


def test_resize_threshold_does_not_create_positive_without_positive_contributor():
    src=np.array([[0.,.5],[.25,.5]],dtype=np.float32)
    out=graph.resize_linear_half_pixel(src,(7,9))
    assert np.all(out<=.5) and np.all(out>=0) and not (out>.5).any()


def test_half_pixel_and_asymmetric_can_move_a_synthetic_threshold_boundary():
    src=np.array([[0.,.6]],dtype=np.float32)
    out=graph.resize_linear_half_pixel(src,(1,4))
    assert (out>.5).tolist()==[[False,False,False,True]]
    # Linear asymmetric samples x=[0,.5,1,1.5], clamped to the input edge.
    asymmetric=np.array([[0.,.3,.6,.6]],dtype=np.float32)
    assert (asymmetric>.5).tolist()==[[False,False,True,True]]


def test_expansion_truncation_and_inclusive_dimensions():
    geom=graph.paste_geometry(np.array([0.,0.,28.,28.],dtype=np.float32))
    assert geom['integer_box']==[-1,-1,29,29]
    assert geom['resize_hw']==[31,31]
    assert geom['destination_xyxy']==[0,0,30,30]
    assert geom['source_xyxy']==[1,1,31,31]
    assert graph.truncate_box(np.array([-1.9,-.1,2.9,3.1],dtype=np.float32)).tolist()==[-1,0,2,3]


def test_degenerate_dimensions_use_max_one_and_half_pixel_midpoint():
    geom=graph.paste_geometry(np.array([5.,6.,5.,6.],dtype=np.float32))
    assert geom['resize_hw']==[1,1]
    assert graph.source_coordinate(5,5,1)==14.5


def test_paste_clips_without_wrapping_or_shifting_source_samples():
    mask=np.ones((28,28),dtype=np.float32)
    actual,geom=graph.synthetic_paste(mask,np.array([-2.,-2.,2.,2.],dtype=np.float32))
    assert actual.shape==(512,512) and actual.dtype==np.float32
    assert geom['integer_box']==[-2,-2,2,2]
    assert geom['source_xyxy']==[2,2,5,5]
    assert (actual[:3,:3]>.5).all() and not actual[3:,:].any() and not actual[:,3:].any()


@pytest.mark.parametrize('source,shape',[(np.zeros((2,2),dtype=np.float32),(0,2)),(np.array([[np.nan]],dtype=np.float32),(1,1))])
def test_malformed_synthetic_inputs_reject(source,shape):
    with pytest.raises(ValueError):graph.resize_linear_half_pixel(source,shape)


@pytest.mark.parametrize('x,y,expected',[(.49,5.,.49),(.5,.5,.25),(14.5,14.5,1.),(28.75,5.,.25),(-.25,4.,0.),(29.25,4.,0.)])
def test_zero_pad_probability_upper_bound(x,y,expected):
    assert graph.padding_probability_upper_bound(x,y)==pytest.approx(expected)


def test_padding_bound_limits_any_synthetic_interior():
    padded=np.pad(np.ones((28,28),dtype=np.float32),1)
    resized=graph.resize_linear_half_pixel(padded,(96,106))
    y=graph.source_coordinate(np.arange(96),0,96)[:,None]
    x=graph.source_coordinate(np.arange(106),0,106)[None,:]
    np.testing.assert_allclose(resized,graph.padding_probability_upper_bound(x,y),rtol=0,atol=3e-8)


def test_roi_align_output_half_pixel_uses_no_negative_half_offset():
    # Bin 0 of a 14-bin ROI with two samples: start + (.25,.75)*bin_width.
    assert graph.roi_align_sample(4.,32.,.25,0,0)==pytest.approx(1.125)
    assert graph.roi_align_sample(4.,32.,.25,0,1)==pytest.approx(1.375)
    # A hypothetical half_pixel variant subtracts .5, unlike the actual graph.
    assert graph.roi_align_sample(4.,32.,.25,0,0)-.5==pytest.approx(.625)


def test_image_resize_centres_512_to_800():
    assert graph.source_coordinate(0,0,800,512)==pytest.approx(-.18)
    assert graph.source_coordinate(799,0,800,512)==pytest.approx(511.18)
    # Inverse index relation preserving centre coordinates, not index doubling.
    assert (graph.source_coordinate(317,0,800,512)+.5)*800/512-.5==pytest.approx(317.)


FROZEN=graph.PAIR.parent/'onnx_mask_static'
FROZEN_HASHES={'graph.json':'f800744cf08a03c68149eead69b660916ca850371a207da8224df96af5f197e3',
               'coordinates.json':'14ef25012c0ea829bfee6a7b69ac81c2852060be14557414608ca4006846e616'}


def frozen(name):
    import gzip,json
    raw=gzip.decompress((FROZEN/(name+'.gz')).read_bytes())
    assert graph.sha(raw)==FROZEN_HASHES[name]
    return json.loads(raw)


def test_changed_model_rejects_before_parser_import(monkeypatch):
    import builtins
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name=='onnx':raise AssertionError('must reject model hash before parsing')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    with pytest.raises(ValueError,match='changed pinned ONNX model'):graph.static_trace(b'changed model')


def test_frozen_graph_has_actual_resize_pool_and_loop_contract():
    trace=frozen('graph.json');nodes={n['name']:n for n in trace['nodes']}
    assert trace['model_sha256']==graph.MODEL_SHA and trace['model_bytes']==176310020
    assert trace['model_node_count']==2411 and trace['selected_node_count']==745
    assert trace['derived_dimensions']['selected_class']==['N',1,28,28]
    body=nodes['Loop_3207']['attributes']['body'];assert len(body['nodes'])==176
    resize=next(n for n in body['nodes'] if n['name']=='Resize_3263')
    assert resize['attributes']==dict(mode='linear',coordinate_transformation_mode='half_pixel',nearest_mode='floor',cubic_coeff_a=-.75)
    assert resize['inputs']==['mask','','','3846']
    assert nodes['/network/roi_heads/mask_roi_pool/RoiAlign']['attributes']['coordinate_transformation_mode']=='output_half_pixel'
    assert nodes['/network/Unsqueeze_8']['outputs']==['masks']
    assert trace['execution']==dict(onnx_runtime_calls=0,model_evaluations=0,simulator_runs=0,motion_goals=0)


def test_retained_coordinate_analysis_reproduces_without_evaluator(monkeypatch):
    import builtins
    import stage_a_epd_spill as old
    import stage_a_epd_origin as origin
    import stage_a_paired_reference as paired
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name.startswith(('onnxruntime','onnx.reference')):raise AssertionError('forbidden evaluator import')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    retained=old.load_retained();origin.analyse(origin.FROZEN)
    artifacts=paired.verified_bundle(graph.PAIR,graph.PAIR_SHA)
    import json
    actual=graph.inspect_coordinates(retained,artifacts,json.loads(artifacts['diagnostic.json']))
    assert actual==frozen('coordinates.json')
    assert actual['strict_accepted_associations']==0 and actual['classification'].startswith('UNKNOWN')
    first=actual['detections'][0]['coordinates']['missing_object']
    assert first['padding_bound_at_most_half_count']==72
    assert first['padding_bound_at_most_half_max']==pytest.approx(.49056603773584906)
    third=actual['detections'][2]['coordinates']['missing_object']
    assert third['outside_derived_paste_window']==8297
    assert len(third['padded_source_xy'])==32  # No fictitious source for outside zeros.


def test_actual_model_static_parse_reproduces_trace_without_evaluator(monkeypatch):
    import builtins
    model=Path('/home/user/stage_a_cube_model/cube_maskrcnn.onnx')
    if not model.exists():pytest.skip('optional local hash-pinned model; frozen graph remains tested')
    pytest.importorskip('onnx')
    original=builtins.__import__
    def guarded(name,*args,**kwargs):
        if name.startswith(('onnxruntime','onnx.reference')):raise AssertionError('forbidden evaluator import')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded)
    actual=graph.static_trace(model.read_bytes());expected=frozen('graph.json')
    # Schema prose may change between parser versions; node protobuf facts must not.
    for key in ('nodes','initializers','inputs','outputs','derived_dimensions','execution'):
        assert actual[key]==expected[key]


def test_static_bundle_artifact_and_source_hashes():
    import json,gzip
    manifest=json.loads((FROZEN/'sha256.json').read_text())
    for name,digest in manifest.items():
        assert graph.sha(gzip.decompress((FROZEN/(name+'.gz')).read_bytes()))==digest
    for name,digest in FROZEN_HASHES.items():assert manifest[name]==digest
    provenance=json.loads(gzip.decompress((FROZEN/'provenance.json.gz').read_bytes()))
    for name,digest in provenance['source_sha256'].items():assert graph.sha((graph.ROOT/name).read_bytes())==digest
