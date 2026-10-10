#!/usr/bin/env python3
"""Offline RGB-only external EPD caller and strict simulation identity consumer."""
import argparse
import collections
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import cv2
from stage_a_fragment_identity import validate_live_fragment_capture,mask_identity

def sha(data):return hashlib.sha256(data).hexdigest()
def require(ok,reason):
    if not ok:raise ValueError(reason)

def preprocessing():
    return dict(input_order='RGB',epd_order='BGR',source_size=[256,256],epd_size=[512,512],
        resize='INTER_LINEAR',scale=2,padding=0,pixel_centres='u512=2*u256+0.5',
        external_epd=dict(resize=[512,512],interpolation='INTER_LINEAR',channel_conversion='BGR2RGB',
            tensor='CHW_float32_RGB_div255',full_image_masks=True,roi='clipped_integer_bbox_512'),
        inverse_mask='union_2x2_samples_no_ID_trimming',mask_threshold=0.5,confidence_threshold=0.8)

def check_preprocessing(p):require(p==preprocessing(),'wrong preprocessing transform/channel order')

def prepared_bgr(rgb):
    require(rgb.dtype==np.uint8 and rgb.shape==(256,256,3),'incorrect RGB dimensions/order contract')
    return cv2.cvtColor(cv2.resize(rgb,(512,512),interpolation=cv2.INTER_LINEAR),cv2.COLOR_RGB2BGR)

def map_mask(roi,box):
    require(len(box)==4 and all(type(x) is int for x in box),'invalid bbox')
    x0,y0,x1,y1=box
    require(0<=x0<x1<=512 and 0<=y0<y1<=512 and roi.dtype==np.float32 and
        roi.shape==(y1-y0,x1-x0) and np.all(np.isfinite(roi)) and np.all((roi>=0)&(roi<=1)),'invalid ROI mask geometry/values')
    whole=np.zeros((512,512),dtype=bool);whole[y0:y1,x0:x1]=roi>0.5
    # Each 512 sample centre lies in exactly one original pixel cell.
    # Preserve the union of all foreground samples, including boundary support.
    return whole.reshape(256,2,256,2).any(axis=(1,3))

def load_capture(folder):
    folder=Path(folder);manifest=json.loads((folder/'raw_sha256.json').read_text())
    def raw(name):
        p=folder/name;b=p.read_bytes() if p.exists() else gzip.decompress((folder/(name+'.gz')).read_bytes())
        require(sha(b)==manifest[name],'wrong artifact hash: '+name);return b
    # Verify the entire retained acquisition, not just the two image files.
    for name in manifest:raw(name)
    report=json.loads(raw('capture.checked.json'));native=json.loads(raw('capture.json'))
    normalized=copy.deepcopy(native);normalized['renderer']['scene_fingerprint']=report['renderer']['scene_fingerprint']
    normalized['rgb_sha256']=report['rgb_sha256'];normalized['id_sha256']=report['id_sha256']
    require(normalized==report,'substituted checked capture')
    owners=[json.loads(line) for line in raw('owner.jsonl').splitlines() if line.strip()]
    require(len(owners)==1,'missing/duplicate owner record');owner=owners[0]
    pre=json.loads(raw('preflight.json'));result=json.loads(raw('result.json'))
    require(result['decision']=='PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY' and result['exit_code']==0 and
        result['child_exit']=={'exit_code':0} and result['executed_binary_sha256']==pre['binary_sha256'] and
        pre['session']==report['session']==owner['session'] and result['qualified_owner_mapped'] is True and
        result['loaded_library_sha256'][pre['owner']]==pre['owner_sha256'],'inconsistent owner/capture provenance')
    require(sys.byteorder=='little','unsupported endian')
    rgb=np.frombuffer(raw('capture.json.rgb8'),dtype=np.uint8).reshape(256,256,3)
    ids=np.frombuffer(raw('capture.json.ids.u32'),dtype='<u4').reshape(256,256)
    binding=validate_live_fragment_capture(owner,report,rgb,ids)
    return owner,report,rgb,ids,binding

def associate(owner,report,rgb,ids,detections,masks):
    require(len(detections)==len(masks),'incomplete detection/mask inventory')
    binding=validate_live_fragment_capture(owner,report,rgb,ids)
    mappings={m['renderer_id']:m for m in binding['mappings']}
    output=[]
    for det,mask in zip(detections,masks):
        row=copy.deepcopy(det);row['association']='REJECTED'
        try:
            require(np.isfinite(det['confidence']) and .8<=det['confidence']<=1,'below original confidence threshold or invalid confidence')
            require(type(det['class_index']) is int and det['class_index']>0 and bool(det['label']),'invalid class')
            identity=mask_identity(report,rgb,ids,mask,report['session'],report['frame'],report['rgb_sha256'])
            require(identity in mappings,'unknown renderer identity')
            row['mapping']=copy.deepcopy(mappings[identity]);row['association']='ACCEPTED'
        except ValueError as e:row['reason']=str(e)
        output.append(row)
    counts=collections.Counter(r['mapping']['collision_id'] for r in output if r['association']=='ACCEPTED')
    for r in output:
        if r['association']=='ACCEPTED' and counts[r['mapping']['collision_id']]!=1:
            r['association']='REJECTED';r['reason']='duplicate/ambiguous detection association'
    return dict(detections=output,accepted=sum(r['association']=='ACCEPTED' for r in output),
        timing_authority='BLOCKED',physical_penetration='BLOCKED',contact_authority=False,
        extraction_authority='BLOCKED',execution_goals=0,rgbd_localization='NOT_CLAIMED_NO_DEPTH_OR_CALIBRATION')

def run(args):
    owner,report,rgb,ids,binding=load_capture(args.capture)
    p=preprocessing();check_preprocessing(p)
    model=Path(args.model);labels=Path(args.labels);binary=Path(args.binary)
    pinned={'model':'21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2',
        'labels':'adeeb37af7c067d456ea0fb2978c9bc4a242bea4d1fbb5a7b53072d486d7e113'}
    for key,path in [('model',model),('labels',labels)]:require(sha(path.read_bytes())==pinned[key],'changed EPD '+key)
    names=labels.read_text().splitlines();require(names==['__background__','cube'],'unexpected labels')
    expected_bgr=prepared_bgr(rgb)
    args.output.mkdir() # exclusive, no overwriting or repeated inference
    rgb_path=args.output/'input.rgb8';rgb_path.write_bytes(rgb.tobytes())
    pre=dict(preprocessing=p,model=str(model),labels=str(labels),model_sha256=pinned['model'],labels_sha256=pinned['labels'],
        binary_sha256=sha(binary.read_bytes()),rgb_sha256=report['rgb_sha256'],id_sha256=report['id_sha256'],
        session=report['session'],frame=report['frame'],acquisition=report['acquisition'],execution_backend='cpu',timeout_seconds=120)
    root=Path(__file__).resolve().parents[1]
    epd=Path('/home/user/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment')
    pre['source_sha256']={str(f):sha(f.read_bytes()) for f in [Path(__file__),root/'scripts/stage_a_rgbd/epd_retained.cpp',
        epd/'src/p3_ort_base.cpp',epd/'src/ort_base.cpp',epd/'include/ort_cpp_lib/p3_ort_base.hpp']}
    pre['prepared_bgr_sha256']=sha(expected_bgr.tobytes());pre['python_opencv_version']=cv2.__version__
    (args.output/'preflight.json').write_text(json.dumps(pre,indent=2)+'\n')
    (args.output/'execution_claim.json').write_text(json.dumps(dict(inference_calls=1,simulator_runs=0)))
    with (args.output/'stdout.log').open('wb') as stdout,(args.output/'stderr.log').open('wb') as stderr:
        proc=subprocess.run(['timeout','120s',str(binary),str(model),str(labels),str(rgb_path),str(args.output)],stdout=stdout,stderr=stderr,env={**os.environ,'EPD_EXECUTION_BACKEND':'cpu'})
    result=dict(exit_code=proc.returncode,preprocessing=p)
    if proc.returncode:
        result.update(decision='BLOCKED',reason='external EPD process failed; no retry')
    else:
        native=json.loads((args.output/'inference.json').read_text());masks=[]
        require(sha((args.output/'epd_input.bgr8').read_bytes())==pre['prepared_bgr_sha256'],'incorrect RGB/BGR or resize bytes')
        result['loaded_library_sha256']={p:sha(Path(p).read_bytes()) for p in native.get('loaded_library_paths',[]) if Path(p).is_file()}

        for d in native['detections']:
            require(d['label']==names[d['class_index']],'EPD label mismatch')
            x0,y0,x1,y1=d['bbox'];raw=(args.output/d['mask_file']).read_bytes()
            d['mask_sha256']=sha(raw)
            roi=np.frombuffer(raw,dtype='<f4').reshape(y1-y0,x1-x0)
            mapped=map_mask(roi,d['bbox']);masks.append(mapped)
            name=d['mask_file']+'.original.bool8';(args.output/name).write_bytes(mapped.astype(np.uint8).tobytes());d['mapped_mask_file']=name
        result.update(associate(owner,report,rgb,ids,native['detections'],masks))
        result['decision']='PASS_IDENTITY_ONLY' if result['accepted'] else 'BLOCKED_NO_QUALIFIED_EPD_ASSOCIATION'
        result['inference_seconds']=native['inference_seconds'];result['genuine_detection_count']=len(native['detections'])
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--capture',type=Path,required=True);a.add_argument('--model',required=True)
    a.add_argument('--labels',required=True);a.add_argument('--binary',required=True);a.add_argument('--output',type=Path,required=True)
    print(json.dumps(run(a.parse_args()),indent=2))
