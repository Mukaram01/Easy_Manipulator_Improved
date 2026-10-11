"""Diagnostic-only fixtures. Tests never create an ORT session or run a model."""
import sys
from pathlib import Path
import numpy as np
import pytest
try:
    import onnx
except ImportError:
    onnx=None
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_onnx_head_observe as observation


def tiny_model():
    if onnx is None:pytest.skip('optional ONNX protobuf package for structural fixtures')
    h=onnx.helper
    # Minimal structural fixture, not an executed model or genuine detection.
    nodes=[h.make_node('Identity',['images'],[observation.TARGET],name='/network/roi_heads/Slice_12'),
           h.make_node('Identity',[observation.TARGET],['masks'],name='fixture')]
    return h.make_model(h.make_graph(nodes,'fixture',[h.make_tensor_value_info('images',1,['N',1,28,28])],
        [h.make_tensor_value_info(n,1,None) for n in ('boxes','labels','scores','masks')]),opset_imports=[h.make_opsetid('',17)])


def test_output_only_transform_preserves_every_original_field():
    original=tiny_model();diag=observation.expose(original)
    observation.validate_transform(original,diag)
    assert [v.name for v in diag.graph.output]==['boxes','labels','scores','masks',observation.TARGET]
    assert diag.graph.output[-1].type.tensor_type.elem_type==1
    dims=diag.graph.output[-1].type.tensor_type.shape.dim
    assert [d.dim_param if d.dim_param else d.dim_value for d in dims]==['N',1,28,28]
    assert len(original.graph.output)==4


@pytest.mark.parametrize('kind',['node','initializer','output_order','target','shape','dtype'])
def test_changed_computation_or_observation_contract_rejects(kind):
    original=tiny_model();diag=observation.expose(original)
    if kind=='node':diag.graph.node[0].op_type='Relu'
    if kind=='initializer':diag.graph.initializer.append(onnx.numpy_helper.from_array(np.array([1],dtype=np.float32),name='new'))
    if kind=='output_order':diag.graph.output[0].name='scores'
    if kind=='target':diag.graph.output[-1].name='wrong'
    if kind=='shape':diag.graph.output[-1].type.tensor_type.shape.dim[2].dim_value=30
    if kind=='dtype':diag.graph.output[-1].type.tensor_type.elem_type=11
    with pytest.raises(ValueError):observation.validate_transform(original,diag)


def test_class_selection_keeps_detection_order_and_exact_probability_bytes():
    a=np.zeros((3,2,28,28),dtype=np.float32)
    a[0,1]=.25;a[1,0]=.75;a[2,1]=np.nextafter(np.float32(.5),np.float32(1))
    labels=np.array([1,0,1],dtype=np.int64)
    selected=observation.select_class_fixture(a,labels)
    assert selected.shape==(3,1,28,28) and selected.nbytes==9408
    assert selected[:,0,0,0].tolist()==[.25,.75,float(np.nextafter(np.float32(.5),np.float32(1)))]
    assert a[1,0].tobytes()==selected[1,0].tobytes()


def test_bilinear_contributors_and_padding_have_literal_weights():
    head=np.ones((28,28),dtype=np.float32)
    c=observation.contributors(head,.25,1.5)
    assert c['value']==.25
    assert c['weights']==[.375,.125,.375,.125]
    assert c['probabilities']==[0.,1.,0.,1.]
    assert c['head_yx']==[None,[0,0],None,[1,0]]
    assert c['nearest_probability']==0.


def test_interpolation_can_cross_threshold_with_low_nearest_sample():
    head=np.zeros((28,28),dtype=np.float32);head[0,0]=.4;head[0,1]=1.
    c=observation.contributors(head,1.25,1.)
    assert c['nearest_probability']==pytest.approx(.4)
    assert c['value']==pytest.approx(.55)


def test_exported_output_byte_difference_is_noncomparable():
    before=[np.array([0.],dtype=np.float32).tobytes()]*4
    after=before.copy();after[0]=np.array([-0.],dtype=np.float32).tobytes()
    assert observation.comparability(before,before)['comparable']
    assert not observation.comparability(before,after)['comparable']


def test_exclusive_claim_survives_and_rejects_second_execution(tmp_path):
    observation.claim_once(tmp_path)
    with pytest.raises(FileExistsError):observation.claim_once(tmp_path)


def test_offline_reconstruction_literal_constant_mask_and_zero_border():
    import stage_a_onnx_head_analysis as analysis
    head=np.ones((28,28),dtype=np.float32)
    result,geometry=analysis.reconstruct(head,np.array([10.,10.,38.,38.],dtype=np.float32))
    assert geometry['integer_box']==[9,9,39,39]
    assert result[24,24]==1. and result[9,9]==0.
    assert result.shape==(512,512) and result.dtype==np.float32


def test_nearest_counterfactual_uses_half_pixel_with_ties_to_lower_sample():
    import stage_a_onnx_head_analysis as analysis
    source=np.array([[0.,1.]],dtype=np.float32)
    np.testing.assert_array_equal(analysis.nearest_reference(source,(1,4)),[[0.,0.,1.,1.]])
    np.testing.assert_array_equal(analysis.nearest_reference(source,(1,1)),[[0.]])


FROZEN=observation.graph.PAIR.parent/'onnx_head_observation'
MANIFEST_SHA='d57a634d504f1de5114fdd4f43452d88fb45de5c298dea0bf622f8126f5eed40'


def test_genuine_frozen_observation_and_arithmetic_replay():
    import gzip,json
    import stage_a_onnx_head_analysis as analysis
    actual=analysis.analyse(FROZEN,MANIFEST_SHA)
    expected=json.loads(gzip.decompress((FROZEN/'analysis.json.gz').read_bytes()))
    assert actual==expected and actual['comparison']['comparable'] and actual['strict_accepted_associations']==0
    assert actual['classification'].startswith('UNKNOWN')
    assert actual['observation_shape']==[3,1,28,28] and actual['inference_calls']==1
    for d,bg,roi,missing,added,removed in zip(actual['detections'][:2],[456,635],[303,564],[75,0],[6,166],[35,48]):
        assert (d['raw_native']['background'],d['roi_native']['background'],d['raw_native']['missing_object'])==(bg,roi,missing)
        assert d['reconstruction']['foreground_decision_differences']==0
        assert d['reconstruction']['max_absolute_error']<1e-6
        assert d['actual_minus_nearest_counterfactual']['actual_added_background']==added
        assert d['actual_minus_nearest_counterfactual']['actual_removed_background']==removed
    first=actual['detections'][0]
    assert first['coordinates']['missing_object']['padding_bound_at_most_half']==72
    assert first['actual_minus_edge_pad_counterfactual']['actual_removed_object']==75


def unpack(folder):
    import gzip,json
    manifest=json.loads((FROZEN/'sha256.json').read_text())
    for name in manifest:
        path=folder/name;path.parent.mkdir(exist_ok=True,parents=True)
        path.write_bytes(gzip.decompress((FROZEN/(name+'.gz')).read_bytes()))


@pytest.mark.parametrize('kind',['runtime','count','dtype','order'])
def test_altered_observation_contract_rejects_before_attribution(tmp_path,kind):
    import json
    import stage_a_onnx_head_analysis as analysis
    unpack(tmp_path)
    name={'runtime':'loaded_library_sha256.json','count':'process_exit.json','dtype':'output/outputs.json','order':'output/outputs.json'}[kind]
    path=tmp_path/name;obj=json.loads(path.read_text())
    if kind=='runtime':obj[next(iter(obj))]='0'*64
    if kind=='count':obj['process_launches']=2
    if kind=='dtype':obj[4]['onnx_element_type']=11
    if kind=='order':obj[4]['name']='wrong'
    path.write_text(json.dumps(obj))
    with pytest.raises(ValueError):analysis.analyse(tmp_path)


def test_changed_original_output_returns_noncomparable_without_attribution(tmp_path):
    import stage_a_onnx_head_analysis as analysis
    unpack(tmp_path);path=tmp_path/'output/outputs_0.bin'
    boxes=np.frombuffer(path.read_bytes(),'<f4').copy();boxes[0]=np.nextafter(boxes[0],np.float32(1000))
    path.write_bytes(boxes.tobytes())
    result=analysis.analyse(tmp_path)
    assert result['classification']=='NON-COMPARABLE' and not result['attribution_performed']
    assert not result['comparison']['comparable'] and 'detections' not in result


def test_frozen_modified_head_bytes_reject(tmp_path):
    import shutil,gzip
    import stage_a_onnx_head_analysis as analysis
    shutil.copytree(FROZEN,tmp_path/'bundle')
    path=tmp_path/'bundle/output/outputs_4.bin.gz';raw=bytearray(gzip.decompress(path.read_bytes()));raw[0]^=1
    path.write_bytes(gzip.compress(raw,mtime=0))
    with pytest.raises(ValueError,match='changed observation artifact'):analysis.analyse(tmp_path/'bundle',MANIFEST_SHA)


def test_actual_diagnostic_model_recreates_pinned_identity_without_execution():
    import gzip,json
    if onnx is None or not observation.MODEL.exists():pytest.skip('optional local pinned model/parser; structural fixtures run when parser available')
    original=onnx.load_model_from_string(observation.MODEL.read_bytes())
    transformed=observation.expose(original);onnx.checker.check_model(transformed)
    contract=json.loads(gzip.decompress((FROZEN/'transformation.json.gz').read_bytes()))
    assert observation.sha(transformed.SerializeToString())==contract['diagnostic_model_sha256']
    assert transformed.graph.output[-1].SerializeToString()==gzip.decompress((FROZEN/'appended_output.pb.gz').read_bytes())
