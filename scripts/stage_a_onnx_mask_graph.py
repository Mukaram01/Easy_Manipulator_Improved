#!/usr/bin/env python3
"""Static ONNX protobuf inspection and synthetic coordinate arithmetic only.

Never creates an evaluator/session, executes a model node, or resamples genuine
masks. Synthetic helpers describe ONNX conventions, not bit-exact ORT kernels.
"""
import argparse
import gzip
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
MODEL_SHA='21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2'
PAIR=ROOT/'evidence/stage_a2_geometry/physics_owner/same_fragment/paired_native_reference'
PAIR_SHA='927cabf64d305c73bbfd48226c009cebb3454ce62d0ef20b3ef69dc197d1a264'


def require(ok,reason):
    if not ok:raise ValueError(reason)


def sha(raw):return hashlib.sha256(raw).hexdigest()


def checked_source(source,shape):
    require(source.ndim==2 and source.dtype==np.float32 and all(source.shape) and np.isfinite(source).all(),'invalid synthetic source')
    require(len(shape)==2 and all(type(n) is int and 0<n<=1024 for n in shape),'invalid/bounded synthetic output size')


def source_coordinate(index,origin,length,input_size=30):
    """half_pixel index in the padded source, mathematically (not an ORT run)."""
    return (np.asarray(index)-origin+.5)*input_size/length-.5


def roi_align_sample(start,end,scale,bin_index,sample_index):
    """Actual output_half_pixel, 14 bins, sampling_ratio=2 (one axis)."""
    require(0<=bin_index<14 and sample_index in (0,1),'invalid synthetic ROI sample')
    return start*scale+(bin_index+(sample_index+.5)/2)*max((end-start)*scale,1)/14


def padding_probability_upper_bound(x,y):
    """Bilinear mass on interior indices 1..28; zero border indices 0,29.

    A sigmoid interior is bounded by 1. This is an analytic upper bound,
    conditional on the derived paste geometry, not an observation of its tensor.
    """
    def mass(s):
        s=np.asarray(s)
        return np.clip(np.minimum(s,29-s),0,1)
    return mass(x)*mass(y)


def resize_linear_half_pixel(source,shape):
    checked_source(source,shape)
    h,w=shape;ih,iw=source.shape
    y=np.clip(source_coordinate(np.arange(h),0,h,ih),0,ih-1)
    x=np.clip(source_coordinate(np.arange(w),0,w,iw),0,iw-1)
    y0=np.floor(y).astype(int);x0=np.floor(x).astype(int)
    y1=np.minimum(y0+1,ih-1);x1=np.minimum(x0+1,iw-1)
    wy=(y-y0)[:,None];wx=(x-x0)[None,:]
    top=(1-wx)*source[y0[:,None],x0]+wx*source[y0[:,None],x1]
    bottom=(1-wx)*source[y1[:,None],x0]+wx*source[y1[:,None],x1]
    return ((1-wy)*top+wy*bottom).astype(np.float32)


def resize_asymmetric_nearest(source,shape):
    checked_source(source,shape)
    h,w=shape;ih,iw=source.shape
    y=np.floor(np.arange(h)*ih/h).astype(int);x=np.floor(np.arange(w)*iw/w).astype(int)
    return source[y[:,None],x]


def truncate_box(box):
    require(box.shape==(4,) and box.dtype==np.float32 and np.isfinite(box).all(),'invalid float32 box')
    return box.astype(np.int64) # ONNX Cast to INT64 truncates toward zero.


def paste_geometry(box):
    truncate_box(box)
    half=(box[2:]-box[:2])*np.float32(.5)
    centre=(box[2:]+box[:2])*np.float32(.5)
    expanded_half=half*np.float32(np.float32(30)/np.float32(28))
    expanded=np.concatenate((centre-expanded_half,centre+expanded_half))
    b=truncate_box(expanded);x0,y0,xmax,ymax=map(int,b)
    w=max(xmax-x0+1,1);h=max(ymax-y0+1,1)
    dx0,dy0=max(x0,0),max(y0,0);dx1,dy1=min(xmax+1,512),min(ymax+1,512)
    require(0<=dx0<=dx1<=512 and 0<=dy0<=dy1<=512,'unsupported wholly outside/inverted box')
    return dict(expanded_float=expanded.tolist(),integer_box=b.tolist(),resize_hw=[h,w],
                destination_xyxy=[dx0,dy0,dx1,dy1],source_xyxy=[dx0-x0,dy0-y0,dx1-x0,dy1-y0])


def synthetic_paste(probabilities,box):
    require(probabilities.shape==(28,28) and probabilities.dtype==np.float32 and
            np.isfinite(probabilities).all() and np.all((probabilities>=0)&(probabilities<=1)),'invalid synthetic probabilities')
    g=paste_geometry(box);resized=resize_linear_half_pixel(np.pad(probabilities,1),tuple(g['resize_hw']))
    x0,y0,x1,y1=g['destination_xyxy'];sx0,sy0,sx1,sy1=g['source_xyxy']
    out=np.zeros((512,512),dtype=np.float32);out[y0:y1,x0:x1]=resized[sy0:sy1,sx0:sx1]
    return out,g


def verified_inputs(model_path):
    import stage_a_epd_spill as old
    import stage_a_epd_origin as origin
    import stage_a_paired_reference as reference
    # All previous input/source/session/provenance checks run before coordinates
    # are interpreted. Existing validators only read/decompress evidence.
    retained=old.load_retained();origin.analyse(origin.FROZEN)
    artifacts=reference.verified_bundle(PAIR,PAIR_SHA)
    prior=json.loads(artifacts['diagnostic.json'])
    require(prior['historical_comparison']=='EXACT_COMPARABILITY_ESTABLISHED' and
            prior['strict_accepted_associations']==0,'historical/native comparison not established')
    pre=json.loads(gzip.decompress((old.RETAINED/'epd/preflight.json.gz').read_bytes()))
    raw=Path(model_path).read_bytes()
    require(sha(raw)==pre['model_sha256']==MODEL_SHA,'changed pinned ONNX model')
    return raw,retained,artifacts,prior


def static_trace(raw):
    """Parse/check protobuf and operator schemas only; never evaluate any node."""
    require(sha(raw)==MODEL_SHA,'changed pinned ONNX model')
    import onnx
    m=onnx.load_model_from_string(raw)
    require([(o.domain,o.version) for o in m.opset_import]==[('',17)],'changed model opset')
    require(all(t.data_location!=onnx.TensorProto.EXTERNAL for t in m.graph.initializer),'unexpected external weights')
    onnx.checker.check_model(m)
    producers={o:n for n in m.graph.node for o in n.output}
    names={n.name:n for n in m.graph.node};selected=set()
    cut={'/network/rpn/Gather_79_output_0','/network/roi_heads/Cast_output_0'}
    def visit(tensor):
        n=producers.get(tensor)
        if n is None or n.name in selected:return
        selected.add(n.name)
        if n.op_type in ('Conv','ConvTranspose','Gemm','RoiAlign') or any(o in cut for o in n.output):return
        for i in n.input:visit(i)
        for a in n.attribute:
            if a.type==onnx.AttributeProto.GRAPH:
                local={o for node in a.g.node for o in node.output}|{i.name for i in a.g.input}
                for node in a.g.node:
                    for i in node.input:
                        if i not in local:visit(i)
    start=next(i for i,n in enumerate(m.graph.node) if n.name=='/network/roi_heads/Sigmoid')
    seeds=[n for n in m.graph.node if n.name.startswith(('/network/roi_heads/mask_roi_pool/',
           '/network/roi_heads/mask_head/','/network/roi_heads/mask_predictor/')) or n.op_type=='Resize']
    seeds+=list(m.graph.node[start:])
    for n in seeds:
        for o in n.output:visit(o)
    require(len(selected)<1200,'unexpected graph trace scope')
    def value(v):
        if isinstance(v,bytes):return v.decode('utf8')
        if isinstance(v,onnx.TensorProto):
            a=onnx.numpy_helper.to_array(v)
            return dict(dtype=onnx.TensorProto.DataType.Name(v.data_type),shape=list(a.shape),
                        values=a.tolist() if a.size<=256 else None,sha256=sha(v.SerializeToString()))
        if isinstance(v,onnx.GraphProto):return graph_record(v)
        return v
    def node_record(n):
        return dict(name=n.name,op=n.op_type,inputs=list(n.input),outputs=list(n.output),
                    attributes={a.name:value(onnx.helper.get_attribute_value(a)) for a in n.attribute})
    def shape_record(v):
        return dict(name=v.name,dtype=onnx.TensorProto.DataType.Name(v.type.tensor_type.elem_type),
                    shape=[d.dim_value if d.HasField('dim_value') else d.dim_param or '?' for d in v.type.tensor_type.shape.dim])
    def graph_record(g):
        return dict(name=g.name,inputs=[shape_record(i) for i in g.input],outputs=[shape_record(o) for o in g.output],
                    nodes=[node_record(n) for n in g.node])
    nodes=[node_record(n) for n in m.graph.node if n.name in selected]
    used={i for n in m.graph.node if n.name in selected for i in n.input}
    weights=[dict(name=t.name,dtype=onnx.TensorProto.DataType.Name(t.data_type),shape=list(t.dims),
                  sha256=sha(t.SerializeToString())) for t in m.graph.initializer if t.name in used]
    schemas={}
    for op in ('Resize','RoiAlign','Cast','Slice','Pad','Loop'):
        s=onnx.defs.get_schema(op,17,'')
        schemas[op]=dict(since_version=s.since_version,doc=s.doc,
                        attributes={k:dict(description=a.description,default=value(onnx.helper.get_attribute_value(a.default_value)) if a.default_value.name else None) for k,a in s.attributes.items()})
    body=next(a.g for a in names['Loop_3207'].attribute if a.name=='body')
    require(len(body.node)==176 and producers['masks'].name=='/network/Unsqueeze_8','changed mask assembly')
    resize=next(n for n in body.node if n.name=='Resize_3263')
    attrs={a.name:value(onnx.helper.get_attribute_value(a)) for a in resize.attribute}
    require(attrs==dict(coordinate_transformation_mode='half_pixel',cubic_coeff_a=-.75,mode='linear',nearest_mode='floor') and list(resize.input)==['mask','','','3846'],'changed paste Resize contract')
    # Values below are derived from pool/kernel/weight metadata, not observed tensors.
    return dict(schema='workcell_onnx_mask_static/v1',model_sha256=MODEL_SHA,model_bytes=len(raw),opset=17,
        onnx_parser_version=onnx.__version__,model_node_count=len(m.graph.node),selected_node_count=len(nodes),
        inputs=[shape_record(i) for i in m.graph.input],outputs=[shape_record(o) for o in m.graph.output],
        nodes=nodes,initializers=weights,operator_schemas=schemas,
        dimensions_status='STATIC_DERIVED; N dynamic (previous actual output N=3); no internal tensor observed',
        derived_dimensions=dict(pool=['N',256,14,14],four_conv_relu=['N',256,14,14],
            transpose_conv_relu=['N',256,28,28],logits_sigmoid=['N',2,28,28],
            selected_class=['N',1,28,28],padded=['N',1,30,30],loop_image=[512,512],assembled=['N',1,512,512]),
        stop_boundaries=sorted(cut),execution=dict(onnx_runtime_calls=0,model_evaluations=0,simulator_runs=0,motion_goals=0))


def inspect_coordinates(retained,artifacts,prior):
    import stage_a_epd_origin as origin
    from stage_a_paired_reference import metrics
    _,_,_,_,dets,rois,_=retained
    ids=np.frombuffer(artifacts['capture.json.native512.ids.u32'],dtype='<u4').reshape(512,512)
    def tensor(name,shape):
        return np.frombuffer(gzip.decompress((origin.FROZEN/('output/'+name+'.gz')).read_bytes()),dtype='<f4').reshape(shape)
    boxes=tensor('session_outputs_after_Run_0.bin',(3,4))
    masks=tensor('session_outputs_after_Run_3.bin',(3,1,512,512))
    rows=[]
    for i,identity in enumerate((65517,65523,65517)):
        raw=masks[i,0];g=paste_geometry(boxes[i]);mask=raw>.5
        roi=np.zeros((512,512),dtype=np.float32);x0,y0,x1,y1=dets[i]['bbox'];roi[y0:y1,x0:x1]=rois[i]
        measured=metrics(mask,ids,identity);cropped=metrics(roi>.5,ids,identity)
        require(measured==prior['detections'][i]['raw_native'] and cropped==prior['detections'][i]['roi_native'],'changed native spill measurements')
        gx0,gy0,gx1,gy1=g['destination_xyxy'];window=np.zeros((512,512),dtype=bool);window[gy0:gy1,gx0:gx1]=True
        require(not np.any(raw[~window]!=0),'raw probabilities outside graph-derived paste window')
        groups={}
        for name,selection in dict(raw_background=mask&(ids==0),roi_background=(roi>.5)&(ids==0),missing_object=(ids==identity)&~mask).items():
            yx=np.argwhere(selection);x=yx[:,1];y=yx[:,0];h,w=g['resize_hw'];bx,by,_,_=g['integer_box']
            inside=window[selection]
            sx=source_coordinate(x[inside],bx,w);sy=source_coordinate(y[inside],by,h)
            lowx=np.floor(sx).astype(int);lowy=np.floor(sy).astype(int)
            pad_touched=(lowx<1)|(lowx+1>28)|(lowy<1)|(lowy+1>28)
            values=raw[selection]
            upper=padding_probability_upper_bound(sx,sy)
            require(np.all(values[inside]<=upper+2e-6),'probability exceeds graph-derived zero-padding bound')
            # This is an inverse coordinate relation, not recovered head values.
            groups[name]=dict(count=len(yx),yx=yx.tolist(),probability_min=float(values.min()) if len(values) else None,
                probability_max=float(values.max()) if len(values) else None,
                outside_epd_roi=int(((x<x0)|(x>=x1)|(y<y0)|(y>=y1)).sum()),
                outside_derived_paste_window=int((~inside).sum()),
                contributor_footprints_touch_zero_pad=int(pad_touched.sum()),
                inside_paste_yx=yx[inside].tolist(),
                padded_source_xy=np.column_stack((sx,sy)).tolist(),
                padded_source_range_xy=[float(sx.min()),float(sy.min()),float(sx.max()),float(sy.max())] if len(sx) else None,
                padding_bound_at_most_half_count=int((upper<=.5).sum()),
                padding_bound_at_most_half_yx=yx[inside][upper<=.5].tolist(),
                padding_bound_min=float(upper.min()) if len(upper) else None,
                padding_bound_at_most_half_max=float(upper[upper<=.5].max()) if np.any(upper<=.5) else None)
        rows.append(dict(index=i,identity=identity,confidence=dets[i]['confidence'],original_float_box=boxes[i].tolist(),
            geometry_status='DERIVED from retained boxes and exact graph operations; internal tensors unobserved',geometry=g,
            roi_xyxy=dets[i]['bbox'],raw_background=measured['background'],roi_background=cropped['background'],
            missing_object=measured['missing_object'],raw_zero_outside_paste_window=True,coordinates=groups))
    return dict(schema='workcell_onnx_mask_coordinates/v1',detections=rows,
        classification='UNKNOWN: learned mask-head versus graph-internal resizing/pasting',
        established='MODEL OUTPUT at exported ONNX boundary; independent native 512 reference',
        strict_accepted_associations=0,thresholds=dict(confidence=.8,mask=.5),
        paired_manifest_sha256=PAIR_SHA,model_sha256=MODEL_SHA,
        coordinate_status='Inverse coordinate formula only; no latent probabilities reconstructed; no genuine mask resized or altered',
        next_observation=dict(tensor='/network/roi_heads/Slice_12_output_0',stage='post-sigmoid selected class, before /network/Pad',
            static_shape=['N',1,28,28],previous_N=3,expected_bytes_if_N3=9408,
            purpose='Observe selected head probabilities, retain unchanged boxes/labels/raw outputs and compare exact raw reproduction before per-pixel attribution.',
            status='PROPOSED ONLY; requires separate authorization; no inference in this task'))


def analyse(model_path):
    raw,retained,artifacts,prior=verified_inputs(model_path)
    trace=static_trace(raw);coordinates=inspect_coordinates(retained,artifacts,prior)
    require(sha(Path(model_path).read_bytes())==MODEL_SHA,'model changed during static inspection')
    return trace,coordinates


def provenance():
    import onnx
    import stage_a_epd_origin as origin
    import stage_a_epd_spill as old
    sources=['scripts/stage_a_onnx_mask_graph.py','tests/test_stage_a_onnx_mask_graph.py',
             'scripts/stage_a_epd_spill.py','scripts/stage_a_epd_origin.py',
             'scripts/stage_a_epd_retained.py','scripts/stage_a_paired_reference.py']
    modules=[Path(onnx.__file__),Path(np.__file__),Path(sys.executable).resolve()]
    modules+=list(Path(onnx.__file__).parent.glob('onnx_cpp2py_export*.so'))
    return dict(baseline='cd1e372e1b5d2bff8a5dec0ab2eb32de4c512896',
        source_sha256={p:sha((ROOT/p).read_bytes()) for p in sources},
        parser_runtime_sha256={str(p):sha(p.read_bytes()) for p in modules},
        python_version=sys.version,onnx_version=onnx.__version__,numpy_version=np.__version__,
        model_sha256=MODEL_SHA,previous_manifests=dict(separated_workpieces=old.MANIFEST_HASHES,
            instrumented_inference=origin.MANIFEST_SHA256,paired_native_reference=PAIR_SHA),
        activities=dict(static_protobuf_parse=True,onnx_checker=True,retained_bytes_read=True,
            synthetic_arithmetic=True,model_evaluations=0,onnx_runtime_calls=0,gazebo_runs=0,motion_goals=0))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--model',type=Path,default=Path('/home/user/stage_a_cube_model/cube_maskrcnn.onnx'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    trace,coordinates=analyse(a.model)
    a.output.mkdir() # Never overwrite retained evidence.
    manifest={}
    for name,obj in (('graph.json',trace),('coordinates.json',coordinates),('provenance.json',provenance())):
        (a.output/name).write_text(json.dumps(obj,indent=2,sort_keys=True)+'\n')
        manifest[name]=sha((a.output/name).read_bytes())
    (a.output/'sha256.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(model_sha256=MODEL_SHA,nodes=trace['selected_node_count'],classification=coordinates['classification'],
        raw_background=[d['raw_background'] for d in coordinates['detections']],roi_background=[d['roi_background'] for d in coordinates['detections']],
        missing_object=[d['missing_object'] for d in coordinates['detections']]),indent=2))
