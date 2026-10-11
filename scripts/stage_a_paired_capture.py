#!/usr/bin/env python3
"""Opt-in one-shot paired native MRT experiment; no EPD, ROS or motion calls."""
import argparse
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import stage_a_gazebo_fragment as original
import stage_a_epd_spill as historical
import stage_a_epd_origin as origin
import stage_a_paired_reference as reference

ROOT=Path(__file__).resolve().parents[1]
SOURCES=['scripts/stage_a_paired_capture.py','scripts/stage_a_paired_reference.py','scripts/stage_a_epd_spill.py','scripts/stage_a_epd_origin.py','scripts/stage_a_epd_retained.py',
         *[str(p.relative_to(ROOT)) for p in sorted((ROOT/'scripts/stage_a_rgbd/paired_native').glob('*')) if p.is_file()]]


def runtime_environment():
    return {k:v for k,v in os.environ.items() if any(x in k for x in ('IGN','GZ','OGRE','LIBGL','DISPLAY','LD_','QT_','PATH','MESA','EGL','GBM','__GL'))}


def check_environment(pinned):
    reference.require(runtime_environment()==pinned,'changed preflight runtime environment')


def check_hashes(hashes):
    for path,expected in hashes.items():
        reference.require(original.sha(path)==expected,'changed preflight dependency: '+path)


def stable_scene(owner,capture):
    # Process pointers/session IDs deliberately differ between independent runs.
    shapes=[{k:v for k,v in s.items() if k!='shape_node_identity'} for s in owner['shapes']]
    joins=[{k:v for k,v in s.items() if k!='shape_node_identity'} for s in capture['draw_bindings']]
    return dict(shapes=shapes,joins=joins,inventory=capture['render_inventory'],
                camera=capture['camera_configuration'],materials=[{k:v for k,v in m.items() if k not in ('datablock_identity','datablock_name')} for m in capture['materials']],
                step=capture['acquisition']['step'],stamp_ns=capture['acquisition']['stamp_ns'])


def prepare_pair(world,binary,owner,output):
    historical.load_retained();origin.analyse(origin.FROZEN)
    reference.require(original.sha(world)=='1ac9bf3adea8f732607c84c80ab6160bc7f40ab32ab6ce2743f40cf9631f0884','changed original scene input')
    pre=original.prepare(world,binary,owner,output,'separated_workpieces')
    original.validate_prepared_world(pre)
    # Resolve actual ELF dependencies without executing the capture binary.
    ldd=subprocess.run(['ldd',str(binary)],capture_output=True,text=True,check=True).stdout
    dependencies={str(Path(binary).resolve().with_name('stage_a_paired_ray_replay')),str(Path(binary).resolve()),str(Path(owner).resolve()),str(Path('/usr/bin/c++').resolve()),str(Path('/usr/bin/cmake').resolve())}
    for line in ldd.splitlines():
        fields=line.split()
        for field in fields:
            if field.startswith('/') and Path(field).is_file(): dependencies.add(str(Path(field).resolve()))
    old=json.loads(gzip.decompress((historical.RETAINED/'capture/result.json.gz').read_bytes()))
    dependencies.update(p for p in old['loaded_library_sha256'] if Path(p).is_file())
    dependencies.update(str(ROOT/n) for n in SOURCES)
    dependencies.update(str(ROOT/n) for n in pre['source_sha256'])
    dependencies.add(str(Path(sys.executable).resolve()))
    for module in list(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name and Path(name).is_file() and Path(name).suffix in ('.py','.so'):dependencies.add(str(Path(name).resolve()))
    # Compiler-emitted dependency inventory pins every included project header.
    dependency_file=Path('/tmp/stage_a2_paired_native_build/CMakeFiles/stage_a_paired_native.dir/capture.cpp.o.d')
    for token in dependency_file.read_text().replace('\\\n',' ').split():
        if token.startswith('/') and Path(token).is_file():dependencies.add(str(Path(token).resolve()))
    for p in (Path('/tmp/stage_a2_render_source/Ogre2Scene.cc'),Path('/tmp/stage_a2_paired_native_build/CMakeFiles/stage_a_paired_native.dir/flags.make'),Path('/tmp/stage_a2_paired_native_build/CMakeFiles/stage_a_paired_native.dir/link.txt')):
        reference.require(p.is_file(),'missing build/lifecycle provenance');dependencies.add(str(p))
    extra=dict(schema='workcell_paired_preflight/v1',maximum_runs=1,retry_allowed=False,
               dependencies={p:original.sha(p) for p in sorted(dependencies)},ldd=ldd,
               environment=runtime_environment(),
               historical_rgb_sha256='92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793',
               historical_id_sha256='75246c2b20a8a0bcbbef866b6ade995535de6fa8f7be24f46bcb1d8197681a19')
    check_hashes(extra['dependencies'])
    (Path(output)/'paired_preflight.json').write_text(json.dumps(extra,indent=2)+'\n')
    return pre


def run_pair(output):
    output=Path(output);extra=json.loads((output/'paired_preflight.json').read_text())
    reference.require(extra['maximum_runs']==1 and extra['retry_allowed'] is False,'wrong one-shot budget')
    check_hashes(extra['dependencies'])
    check_environment(extra['environment'])
    historical.load_retained();origin.analyse(origin.FROZEN)
    # Existing runner owns the exclusive claim, timeout, debugger, qualified owner
    # checks and original 256 validator. There is exactly one invocation, no retry.
    result=original.run(output)
    reference.require(result['decision']=='PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY', 'bounded capture failed: '+result.get('failure_reason',''))
    check_pair_offline(output)
    check_hashes(extra['dependencies'])
    return analyse(output)


def attach_arithmetic_replay(output,pair):
    output=Path(output)
    path=output/'offline_ray_replay.json'
    if not path.exists():
        pre=json.loads((output/'preflight.json').read_text())
        binary=Path(pre['binary']).with_name('stage_a_paired_ray_replay')
        # Pure float32 matrix arithmetic, no renderer/server construction.
        process=subprocess.run([str(binary),str(output/'capture.checked.json')],capture_output=True,check=True,timeout=10)
        path.write_bytes(process.stdout)
    replay=json.loads(path.read_text())
    reference.require(replay['bitwise_exact'] is True and len(replay['rays'])==8,'camera float32 replay failed')
    pair['arithmetic_replay']={str(n):[r for r in replay['rays'] if r['resolution']==n] for n in (256,512)}


def check_pair_offline(output):
    output=Path(output);capture=json.loads((output/'capture.checked.json').read_text());pair=capture['paired_capture']
    ids=[np.frombuffer((output/name).read_bytes(),dtype='<u4').reshape(n,n) for name,n in
         (('capture.json.ids.u32',256),('capture.json.native512.ids.u32',512))]
    pair['optics']=[reference.optics(n,pair['frozen'][0]['camera']['fovy']) for n in (256,512)]
    pair['passes']=[{k:p[k] for k in ('width','height','render_pass_count','id_format','depth_test','depth_write','actual_viewport','actual_attachments','post_pass_depth_test','post_pass_depth_write')} for p in (capture,pair['native512'])]
    pair['rasters']=[dict(sha256=reference.digest(a.tobytes())) for a in ids]
    attach_arithmetic_replay(output,pair)
    reference.validate_pair(pair,*ids)
    result=json.loads((output/'result.json').read_text());pre=json.loads((output/'paired_preflight.json').read_text())
    reference.require(result['exit_code']==0 and result['child_exit']=={'exit_code':0} and result['decision']=='PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY','failed native acquisition')
    reference.require(all(h in set(pre['dependencies'].values()) for h in result['loaded_library_sha256'].values()),'unpreflighted native DSO')
    reference.require(pair['no_scene_or_physics_update_between_acquisitions'] is True,'unexpected update')
    reference.require(len(pair['acquisition_wall_ns'])==3 and pair['acquisition_wall_ns']==sorted(pair['acquisition_wall_ns']),'stale acquisition order')
    pair['frozen_fingerprints']=[reference.digest(json.dumps(s,sort_keys=True,separators=(',',':')).encode()) for s in pair['frozen']]
    (output/'paired.checked.json').write_text(json.dumps(pair,indent=2)+'\n')
    return pair


def analyse(output):
    output=Path(output)
    pair=check_pair_offline(output)
    ids=np.frombuffer((output/'capture.json.ids.u32').read_bytes(),dtype='<u4').reshape(256,256)
    native=np.frombuffer((output/'capture.json.native512.ids.u32').read_bytes(),dtype='<u4').reshape(512,512)
    reference.validate_pair(pair,ids,native)
    old_owner,old_cap,old_rgb,old_ids,dets,rois,_=historical.load_retained()
    origin.analyse(origin.FROZEN) # verifies full raw-model experiment provenance
    owner=json.loads((output/'owner.jsonl').read_text())
    cap=json.loads((output/'capture.checked.json').read_text())
    rgb=(output/'capture.json.rgb8').read_bytes()
    scene,old_scene=stable_scene(owner,cap),stable_scene(old_owner,old_cap)
    matched=reference.comparable(rgb,ids,old_rgb.tobytes(),old_ids,scene,old_scene)
    proxy=ids.repeat(2,0).repeat(2,1)
    report=dict(schema='workcell_native_spill_diagnostic/v1',classification='UNKNOWN',
        historical_comparison='EXACT_COMPARABILITY_ESTABLISHED' if matched else 'BLOCKED',
        rgb_exact=rgb==old_rgb.tobytes(),id_exact=np.array_equal(ids,old_ids),scene_camera_exact=scene==old_scene,
        scene_difference_keys=[k for k in scene if scene[k]!=old_scene[k]],
        scene_fingerprints=[reference.digest(json.dumps(s,sort_keys=True,separators=(',',':')).encode()) for s in (scene,old_scene)],
        native_counts=historical.counts(native),proxy_counts=historical.counts(proxy),
        renderer_resolution_disagreement=int(np.count_nonzero(native!=proxy)),
        renderer_resolution_disagreement_yx=np.argwhere(native!=proxy).tolist(),
        source_hashes=json.loads((output/'paired_preflight.json').read_text())['dependencies'],
        input_hashes={n:original.sha(output/n) for n in ('capture.json.rgb8','capture.json.ids.u32','capture.json.native512.ids.u32')},
        strict_accepted_associations=0,execution_goals=0,contact_authority=False,
        uncertainty='Native raster is independently rendered at 512 from the same two-cube disposable unlit depth-tested scene. No physical/contact/extraction qualification.')
    if matched:
        raw=np.frombuffer(gzip.decompress((origin.FROZEN/'output/session_outputs_after_Run_3.bin.gz').read_bytes()),dtype='<f4').reshape(3,1,512,512)
        rows=[]
        for i,(identity,det,roi) in enumerate(zip((65517,65523,65517),dets,rois)):
            embedded=np.zeros((512,512),dtype=np.float32);x0,y0,x1,y1=det['bbox'];embedded[y0:y1,x0:x1]=roi
            model=raw[i,0]>.5;cropped=embedded>.5
            rows.append(dict(index=i,identity=identity,confidence=det['confidence'],below_confidence_gate=det['confidence']<.8,
                raw_native=reference.metrics(model,native,identity),roi_native=reference.metrics(cropped,native,identity),
                raw_proxy=reference.metrics(model,proxy,identity),roi_proxy=reference.metrics(cropped,proxy,identity),
                crop_foreground_added=int((cropped & ~model).sum()),crop_foreground_removed=int((model & ~cropped).sum()),
                crop_background_added=int((cropped & ~model & (native==0)).sum()),
                proxy_background_reclassified_object=int((model & (proxy==0) & (native==identity)).sum()),
                proxy_object_reclassified_background=int((model & (proxy==identity) & (native==0)).sum())))
        report['detections']=rows
        report['classification']=origin.classify(sum(x['raw_native']['background'] for x in rows),sum(x['crop_background_added'] for x in rows))
    report['next_single_action']='Review the native 512 spill coordinates and exported model mask generation offline; no further capture or inference authorized.' if matched else 'Resolve the reported historical scene/camera or byte mismatch offline before attempting historical 512 attribution.'
    (output/'diagnostic.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--world',type=Path);p.add_argument('--binary',type=Path);p.add_argument('--owner',type=Path)
    p.add_argument('--run-prepared',action='store_true');p.add_argument('--analyse-only',action='store_true');a=p.parse_args()
    if a.analyse_only: result=analyse(a.output)
    elif a.run_prepared: result=run_pair(a.output)
    else:
        p.error('preflight needs world/binary/owner') if not all((a.world,a.binary,a.owner)) else None
        result=prepare_pair(a.world,a.binary,a.owner,a.output)
    print(json.dumps({k:result[k] for k in ('historical_comparison','classification','rgb_exact','id_exact','scene_camera_exact','scene_difference_keys','renderer_resolution_disagreement') if k in result},indent=2))
