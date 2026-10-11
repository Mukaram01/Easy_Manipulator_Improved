#!/usr/bin/env python3
"""Offline analysis of the sole head observation. Never creates a model session."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
import stage_a_onnx_head_observe as observation
import stage_a_onnx_mask_graph as graph
import stage_a_epd_origin as origin
import stage_a_epd_spill as old
import stage_a_paired_reference as paired


def nearest_reference(source,shape):
    h,w=shape;ih,iw=source.shape
    y=np.clip(np.ceil(graph.source_coordinate(np.arange(h),0,h,ih)-.5).astype(int),0,ih-1)
    x=np.clip(np.ceil(graph.source_coordinate(np.arange(w),0,w,iw)-.5).astype(int),0,iw-1)
    return source[y[:,None],x]


def reconstruct(head,box,variant='actual_linear_zero_pad'):
    """Analysis arrays only: never input to association or genuine-mask replacement.

    Mathematical float64 indices/weights, final float32 rounding. Values are
    compared to observed ORT, NOT assumed bit-exact with its float32 kernel.
    """
    graph.require(head.shape==(28,28) and head.dtype==np.float32 and np.isfinite(head).all() and
                  np.all((head>=0)&(head<=1)),'invalid observed probabilities')
    g=graph.paste_geometry(box);shape=tuple(g['resize_hw'])
    graph.require(variant in ('actual_linear_zero_pad','counterfactual_nearest','counterfactual_edge_pad'),'unknown reconstruction')
    pad=np.pad(head,1,mode='edge') if variant=='counterfactual_edge_pad' else np.pad(head,1)
    resized=nearest_reference(pad,shape) if variant=='counterfactual_nearest' else graph.resize_linear_half_pixel(pad,shape)
    x0,y0,x1,y1=g['destination_xyxy'];sx0,sy0,sx1,sy1=g['source_xyxy']
    result=np.zeros((512,512),np.float32);result[y0:y1,x0:x1]=resized[sy0:sy1,sx0:sx1]
    return result,g


def analyse(folder,manifest_sha=None):
    folder=Path(folder)
    manifest=None
    if (folder/'sha256.json').exists():
        graph.require(manifest_sha is not None and graph.sha((folder/'sha256.json').read_bytes())==manifest_sha,'changed/unpinned observation manifest')
        manifest=json.loads((folder/'sha256.json').read_text())
    def read(name):
        data=gzip.decompress((folder/(name+'.gz')).read_bytes()) if manifest is not None else (folder/name).read_bytes()
        if manifest is not None:graph.require(graph.sha(data)==manifest[name],'changed observation artifact: '+name)
        return data
    if manifest is not None:
        for name in manifest:read(name)
    retained=old.load_retained();origin.analyse(origin.FROZEN)
    native=paired.verified_bundle(graph.PAIR,graph.PAIR_SHA);prior=json.loads(native['diagnostic.json'])
    graph.require(prior['historical_comparison']=='EXACT_COMPARABILITY_ESTABLISHED','native reference not comparable')
    for n,h in observation.STATIC_SHA.items():
        graph.require(graph.sha(gzip.decompress((observation.STATIC/(n+'.gz')).read_bytes()))==h,'changed static graph evidence')
    pre=json.loads(read('preflight.json'));post=json.loads(read('after.json'));status=json.loads(read('process_exit.json'))
    graph.require(pre['model_sha256']==graph.MODEL_SHA==post['model_sha256'] and
                  pre['diagnostic_model_sha256']==post['diagnostic_model_sha256'] and
                  pre['input_sha256']==observation.INPUT_SHA==post['input_sha256']==graph.sha(read('input.f32')),'changed observation model/input attestation')
    historical=json.loads(gzip.decompress((origin.FROZEN/'preflight.json.gz').read_bytes()))
    allowed=set(historical['historical_loaded_library_sha256'].values())
    graph.require(set(pre['allowed_historical_library_sha256'])==allowed and
                  all(h in allowed for h in json.loads(read('loaded_library_sha256.json')).values()),'changed observation runtime')
    graph.require(post['external_source_sha256']==pre['external_source_sha256']==historical['external_source_sha256'] and
                  post['external_git_status']==pre['external_git_status']==historical['external_git_status'],'changed external EPD')
    graph.require(status['exit_code']==0 and not status['timeout'] and status['process_launches']==1 and
                  status['maximum_inference_calls']==1 and status['gazebo_runs']==status['motion_goals']==0,'failed or repeated experiment')
    graph.require(json.loads(read('execution_claim.json'))==dict(maximum_runs=1,retries=False,gazebo_runs=0,motion_goals=0) and
                  json.loads(read('output/run_entered.json'))==dict(session_run_calls_entered=1,maximum=1) and
                  json.loads(read('output/run_completed.json'))==dict(session_run_calls_completed=1),'invalid run count')
    session=json.loads(read('output/session.json'))
    graph.require(session==dict(ort_version='1.16.3',backend='CPU',intra_op_threads=2,inter_op_threads=1,optimization='ORT_ENABLE_ALL',execution='ORT_SEQUENTIAL'),'changed runtime configuration')
    if manifest is not None:
        for p,h in pre['source_artifact_tool_sha256'].items():
            snapshot='source/'+Path(p).name
            if snapshot in manifest:graph.require(graph.sha(read(snapshot))==h,'changed executed-source snapshot')
        graph.require(graph.sha(read('observe'))==pre['source_artifact_tool_sha256'][pre['binary']],'changed executed binary')
    rows=json.loads(read('output/outputs.json'))
    expected=[([3,4],1,48),([3],7,24),([3],1,12),([3,1,512,512],1,3145728),([3,1,28,28],1,9408)]
    graph.require(len(rows)==5,'wrong output count')
    outputs=[]
    for i,(r,(shape,dtype,size)) in enumerate(zip(rows,expected)):
        data=read('output/'+r['file'])
        graph.require(r['index']==i and r['name']==observation.NAMES[i] and r['shape']==shape and
                      r['onnx_element_type']==dtype and r['bytes']==len(data)==size,'changed observed tensor metadata')
        outputs.append(data)
    before=[gzip.decompress((origin.FROZEN/f'output/session_outputs_after_Run_{i}.bin.gz').read_bytes()) for i in range(4)]
    comparison=observation.comparability(before,outputs[:4])
    if not comparison['comparable']:
        return dict(classification='NON-COMPARABLE',comparison=comparison,attribution_performed=False)
    boxes=np.frombuffer(outputs[0],'<f4').reshape(3,4);labels=np.frombuffer(outputs[1],'<i8');scores=np.frombuffer(outputs[2],'<f4')
    raw=np.frombuffer(outputs[3],'<f4').reshape(3,1,512,512);heads=np.frombuffer(outputs[4],'<f4').reshape(3,1,28,28)
    graph.require(labels.tolist()==[1,1,1] and np.isfinite(heads).all() and np.all((heads>=0)&(heads<=1)),'invalid selected mask/class order')
    ids=np.frombuffer(native['capture.json.native512.ids.u32'],'<u4').reshape(512,512)
    report=[]
    for i,identity in enumerate((65517,65523,65517)):
        head=heads[i,0];genuine=raw[i,0];mask=genuine>.5
        reconstructed,g=reconstruct(head,boxes[i]);nearest,_=reconstruct(head,boxes[i],'counterfactual_nearest');edge,_=reconstruct(head,boxes[i],'counterfactual_edge_pad')
        error=np.abs(reconstructed.astype(np.float64)-genuine.astype(np.float64));margin=float(np.min(np.abs(genuine.astype(float)-.5)))
        threshold_diff=int(np.count_nonzero((reconstructed>.5)!=mask))
        measured=paired.metrics(mask,ids,identity)
        graph.require(measured==prior['detections'][i]['raw_native'],'changed native spill counts')
        # Numerical reconstruction must preserve all binary decisions before contributor attribution.
        graph.require(threshold_diff==0 and float(error.max())<margin,'unresolved downstream numerical threshold ambiguity')
        bx,by,_,_=g['integer_box'];h,w=g['resize_hw'];groups={}
        for name,selection in dict(background=mask&(ids==0),missing_object=(ids==identity)&~mask).items():
            yx=np.argwhere(selection);inside=(yx[:,1]>=g['destination_xyxy'][0])&(yx[:,1]<g['destination_xyxy'][2])&(yx[:,0]>=g['destination_xyxy'][1])&(yx[:,0]<g['destination_xyxy'][3])
            details=[]
            for y,x in yx[inside]:
                sx=float(graph.source_coordinate(x,bx,w));sy=float(graph.source_coordinate(y,by,h))
                c=observation.contributors(head,sx,sy)
                c.update(yx=[int(y),int(x)],padded_source_xy=[sx,sy],observed_probability=float(genuine[y,x]),
                         padding_upper_bound=float(graph.padding_probability_upper_bound(sx,sy)),
                         centre_outside_original_float_box=bool(not (boxes[i,0]<=x+.5<boxes[i,2] and boxes[i,1]<=y+.5<boxes[i,3])))
                details.append(c)
            groups[name]=dict(count=len(yx),outside_paste_window=int((~inside).sum()),coordinates=details,
                centres_outside_original_float_box=sum(c['centre_outside_original_float_box'] for c in details),
                nearest_at_most_half=sum(c['nearest_probability']<=.5 for c in details),
                all_positive_weight_contributors_above_half=sum(all(p>.5 for p,w in zip(c['probabilities'],c['weights']) if w>0) for c in details),
                padding_bound_at_most_half=sum(c['padding_upper_bound']<=.5 for c in details))
        yy,xx=np.indices((28,28));px=bx+(xx+1.5)*w/30-.5;py=by+(yy+1.5)*h/30-.5
        ix=np.floor(px+.5).astype(int);iy=np.floor(py+.5).astype(int)
        mapped_bg=(ids[iy,ix]==0);positive=head>.5
        grid=[dict(head_yx=[int(y),int(x)],probability=float(head[y,x]),image_continuous_index_xy=[float(px[y,x]),float(py[y,x])],
                   nearest_native_yx=[int(iy[y,x]),int(ix[y,x])]) for y,x in np.argwhere(mapped_bg&positive)]
        def delta(counterfactual):
            cf=counterfactual>.5;added=mask&~cf;removed=cf&~mask
            return dict(actual_added_foreground=int(added.sum()),actual_removed_foreground=int(removed.sum()),
                actual_added_background=int((added&(ids==0)).sum()),actual_removed_background=int((removed&(ids==0)).sum()),
                actual_removed_object=int((removed&(ids==identity)).sum()))
        unique_positive_contributors={tuple(yx) for c in groups['background']['coordinates'] for yx,p,w in zip(c['head_yx'],c['probabilities'],c['weights']) if yx is not None and p>.5 and w>0}
        report.append(dict(index=i,identity=identity,confidence=float(scores[i]),raw_native=measured,roi_native=prior['detections'][i]['roi_native'],
            original_float_box=boxes[i].tolist(),unique_above_half_head_samples_contributing_to_background=len(unique_positive_contributors),
            observed_head_sha256=graph.sha(head.tobytes()),head_foreground_count=int(positive.sum()),head_min=float(head.min()),head_max=float(head.max()),
            geometry=g,reconstruction=dict(arithmetic='float64 mathematical indices/weights, float32 final output; NOT bit-exact ORT',
                values_changed=int(np.count_nonzero(reconstructed.view('u4')!=genuine.view('u4'))),max_absolute_error=float(error.max()),
                mean_absolute_error=float(error.mean()),foreground_decision_differences=threshold_diff,min_distance_from_half=margin),
            actual_minus_nearest_counterfactual=delta(nearest),actual_minus_edge_pad_counterfactual=delta(edge),
            mapped_head_centres_on_native_background=grid,coordinates=groups))
    return dict(classification='UNKNOWN: exclusive learned-mask error versus box-placement error; measured mechanisms described separately',
        observed_mechanisms='High selected-head probabilities map to background samples; bilinear-versus-nearest support differs; zero padding suppresses foreground.',
        comparison=comparison,attribution_performed=True,detections=report,strict_accepted_associations=0,
        observation_tensor=observation.TARGET,observation_shape=[3,1,28,28],observation_sha256=graph.sha(outputs[4]),
        native_reference_sha256=graph.sha(native['capture.json.native512.ids.u32']),
        counterfactual_status='Offline arithmetic diagnostics only, never genuine replacement masks or acceptance; nearest uses same half_pixel coordinates and ties to lower index; edge-pad changes border only at fixed geometry.',
        spatial_uncertainty='Head-centre references sample nearest native raster cells, not independent continuous 28x28 labels. Learned head and learned box errors cannot be uniquely separated.',
        inference_calls=1,gazebo_runs=0,motion_goals=0)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--manifest-sha');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=analyse(a.folder,a.manifest_sha)
    observation.write_json(a.output,result)
    print(json.dumps(dict(classification=result['classification'],comparable=result['comparison']['comparable'],
        reconstruction=[r['reconstruction'] for r in result.get('detections',[])]),indent=2))
