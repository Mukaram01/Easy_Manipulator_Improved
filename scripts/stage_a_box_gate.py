#!/usr/bin/env python3
"""Offline box-placement gate. No session, capture, production mask or acceptance writes."""
import argparse
import csv
import gzip
import itertools
import json
from pathlib import Path
import cv2
import numpy as np
import stage_a_onnx_head_analysis as analysis
import stage_a_onnx_mask_graph as graph
import stage_a_paired_reference as paired

OBSERVATION=graph.PAIR.parent/'onnx_head_observation'
OBSERVATION_SHA='d57a634d504f1de5114fdd4f43452d88fb45de5c298dea0bf622f8126f5eed40'


def load_inputs(folder=OBSERVATION):
    folder=Path(folder)
    baseline=analysis.analyse(folder,OBSERVATION_SHA) # verifies all historical bundles/provenance
    graph.require(baseline['comparison']['comparable'],'noncomparable baseline')
    def tensor(index,shape):
        return np.frombuffer(gzip.decompress((folder/f'output/outputs_{index}.bin.gz').read_bytes()),'<f4').reshape(shape)
    native=paired.verified_bundle(graph.PAIR,graph.PAIR_SHA)
    return dict(baseline=baseline,boxes=tensor(0,(3,4)),heads=tensor(4,(3,1,28,28)),
                raw=tensor(3,(3,1,512,512)),
                ids=np.frombuffer(native['capture.json.native512.ids.u32'],'<u4').reshape(512,512))


def candidates(box):
    """Scale about original centre, then translate; float32 graph inputs."""
    graph.truncate_box(box)
    centre=(box[:2].astype(float)+box[2:])/2
    half=(box[2:].astype(float)-box[:2])/2
    for dx,dy,sx,sy in itertools.product(range(-6,7,2),range(-6,7,2),(.9,.95,1.,1.05,1.1),(.9,.95,1.,1.05,1.1)):
        control=(dx,dy,sx,sy)==(0,0,1.,1.)
        offset=np.array([dx,dy]);extent=half*np.array([sx,sy])
        value=box.copy() if control else np.concatenate((centre+offset-extent,centre+offset+extent)).astype(np.float32)
        yield dict(parameters=[dx,dy,sx,sy],box_xyxy=value.tolist(),control=control)


def measure(mask,ids,identity):
    """Exact counts and deterministic float64 nearest pixel-centre distances."""
    graph.require(mask.shape==ids.shape==(512,512) and mask.dtype==np.bool_,'mask/reference mismatch')
    obj=ids==identity;bg=mask&(ids==0);cross=mask&(ids!=0)&~obj
    intersection=int((mask&obj).sum());union=int((mask|obj).sum())
    def boundary(a):
        # Measurement only: never return an eroded candidate.
        return np.argwhere(a&~cv2.erode(a.astype(np.uint8),np.ones((3,3),np.uint8),
            borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool))
    ob=boundary(obj);mb=boundary(mask)
    graph.require(len(ob)>0 and len(mb)>0,'empty metric boundary')
    def distances(points,queries):
        raster=np.ones(mask.shape,np.uint8);raster[points[:,0],points[:,1]]=0
        approximate=cv2.distanceTransform(raster,cv2.DIST_L2,cv2.DIST_MASK_PRECISE).astype(np.float64)
        # Pixel centres have integer coordinates, so squared Euclidean distances
        # are integers. Canonicalize float32 sqrt roundoff before float64 sqrt.
        squared=approximate*approximate
        graph.require(np.max(np.abs(squared-np.rint(squared)))<.25,'ambiguous integer squared distance')
        return np.sqrt(np.rint(squared[queries[:,0],queries[:,1]]))
    errors=np.concatenate((distances(ob,mb),distances(mb,ob)))
    spill=distances(ob,np.argwhere(bg))
    return dict(foreground=int(mask.sum()),object_pixels=int(obj.sum()),intersection=intersection,union=union,
        background=int(bg.sum()),cross_object=int(cross.sum()),missing_object=int((obj&~mask).sum()),
        iou=intersection/union,spill_distance_max=float(spill.max()) if spill.size else 0.,
        spill_distance_mean=float(spill.mean()) if spill.size else 0.,
        spill_distance_gt_sqrt2=int((spill>np.sqrt(2)+1e-6).sum()),
        boundary_error_max=float(errors.max()),boundary_error_mean=float(errors.mean()))


def cropped_mask(mask,box):
    """Existing EPD truncation/clipping ROI, diagnostic only."""
    x0,y0,x1,y1=np.clip(np.asarray(box).astype(np.int64),0,512)
    out=np.zeros_like(mask);out[y0:y1,x0:x1]=mask[y0:y1,x0:x1]
    return out


def pareto_indices(rows):
    """Minimize background spill and missing pixels; keep every tied geometry."""
    values=np.array([(r['background'],r['missing_object']) for r in rows])
    return [i for i,v in enumerate(values) if not np.any(np.all(values<=v,axis=1)&np.any(values<v,axis=1))]


def run():
    cv2.setNumThreads(1)
    data=load_inputs();results=[]
    for i,identity in enumerate((65517,65523)):
        rows=[]
        for index,row in enumerate(candidates(data['boxes'][i])):
            probability,geometry=analysis.reconstruct(data['heads'][i,0],np.array(row['box_xyxy'],np.float32))
            mask=probability>.5
            if row['control']:
                graph.require(np.array_equal(mask,data['raw'][i,0]>.5),'control changed foreground')
            row.update(candidate_index=index,geometry=geometry,metrics=measure(mask,data['ids'],identity),
                cropped_metrics=measure(cropped_mask(mask,row['box_xyxy']),data['ids'],identity))
            rows.append(row)
        front=pareto_indices([r['metrics'] for r in rows])
        best=max(range(len(rows)),key=lambda n:(rows[n]['metrics']['iou'],-n))
        full=[r for r in rows if r['metrics']['missing_object']==0]
        zero=[r for r in rows if r['metrics']['background']==0 and r['metrics']['cross_object']==0]
        cropped_front=pareto_indices([r['cropped_metrics'] for r in rows])
        cropped_best=max(range(len(rows)),key=lambda n:(rows[n]['cropped_metrics']['iou'],-n))
        chosen=rows[best]
        mask=analysis.reconstruct(data['heads'][i,0],np.array(chosen['box_xyxy'],np.float32))[0]>.5
        results.append(dict(index=i,identity=identity,original_box_xyxy=data['boxes'][i].tolist(),
            head_sha256=graph.sha(data['heads'][i,0].tobytes()),head_above_half=int((data['heads'][i,0]>.5).sum()),
            candidates=rows,pareto_candidate_indices=front,best_iou_candidate_index=best,
            cropped_pareto_candidate_indices=cropped_front,cropped_best_iou_candidate_index=cropped_best,
            cropped_minimum_background_at_full_coverage=min((r['cropped_metrics']['background'] for r in rows if r['cropped_metrics']['missing_object']==0),default=None),
            cropped_minimum_missing_at_zero_spill=min((r['cropped_metrics']['missing_object'] for r in rows if r['cropped_metrics']['background']==r['cropped_metrics']['cross_object']==0),default=None),
            cropped_exact_candidates=[r['candidate_index'] for r in rows if r['cropped_metrics']['background']==r['cropped_metrics']['cross_object']==r['cropped_metrics']['missing_object']==0],
            minimum_background=min(r['metrics']['background'] for r in rows),
            minimum_background_at_full_coverage=min((r['metrics']['background'] for r in full),default=None),
            minimum_missing_at_zero_spill=min((r['metrics']['missing_object'] for r in zero),default=None),
            exact_candidates=[r['candidate_index'] for r in zero if r['metrics']['missing_object']==0],
            best_iou_residual_yx=dict(background=np.argwhere(mask&(data['ids']==0)).tolist(),
                missing_object=np.argwhere(~mask&(data['ids']==identity)).tolist(),
                cross_object=np.argwhere(mask&(data['ids']!=0)&(data['ids']!=identity)).tolist())))
    promising=all(r['exact_candidates'] or r['cropped_exact_candidates'] for r in results)
    return dict(schema='stage_a2_box_gate/v1',baseline_commit='6106fe83',
        decision='BOX_GEOMETRY_CANDIDATE — REQUIRES INDEPENDENT VALIDATION' if promising else 'MODEL_IMPROVEMENT_REQUIRED_FOR_TESTED_SEARCH',
        scope='Tested grid only, not a global impossibility proof. Per-detection oracle evaluation; no runtime correction.',
        placement='float64 centre/extent scaling and translation, cast once to float32; original box exact control; graph float32 expansion30/28, truncation, +1 sizes, zero pad, linear half_pixel, >0.5',
        pareto_objectives=['background','missing_object'],candidate_count_per_detection=1225,
        observation_manifest_sha256=OBSERVATION_SHA,native_manifest_sha256=graph.PAIR_SHA,
        baseline=[{k:r[k] for k in ('identity','raw_native','roi_native','original_float_box')} for r in data['baseline']['detections']],
        strict_original_accepted_associations=data['baseline']['strict_accepted_associations'],
        activities=dict(new_inference_calls=0,new_captures=0,motion_goals=0,production_changes=0),detections=results)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=run()
    a.output.mkdir() # fail rather than overwrite any retained artifact
    raw=(json.dumps(result,sort_keys=True,indent=2)+'\n').encode()
    (a.output/'results.json.gz').write_bytes(gzip.compress(raw,mtime=0))
    (a.output/'sha256.json').write_text(json.dumps({'results.json':graph.sha(raw)},sort_keys=True,indent=2)+'\n')
    with (a.output/'pareto.csv').open('x',newline='') as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(['path','identity','candidate_index','dx','dy','sx','sy','x0','y0','x1','y1','background','missing_object','iou','boundary_mean_px','boundary_max_px','expanded_integer_xyxy'])
        for d in result['detections']:
            for path,key,metric in [('raw','pareto_candidate_indices','metrics'),('cropped','cropped_pareto_candidate_indices','cropped_metrics')]:
                for n in d[key]:
                    c=d['candidates'][n];m=c[metric]
                    w.writerow([path,d['identity'],n,*c['parameters'],*c['box_xyxy'],m['background'],m['missing_object'],m['iou'],m['boundary_error_mean'],m['boundary_error_max'],json.dumps(c['geometry']['integer_box'])])
    manifest=json.loads((a.output/'sha256.json').read_text());manifest['pareto.csv']=graph.sha((a.output/'pareto.csv').read_bytes())
    (a.output/'sha256.json').write_text(json.dumps(manifest,sort_keys=True,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('detections','baseline')},indent=2))
    for d in result['detections']:
        print(json.dumps({k:v for k,v in d.items() if k not in ('candidates','best_iou_residual_yx')},indent=2))
