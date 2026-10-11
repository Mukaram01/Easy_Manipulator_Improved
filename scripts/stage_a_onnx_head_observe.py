#!/usr/bin/env python3
"""Isolated output-only ONNX observation; prepare is offline, execute is one-shot.

No production model/source writes. Tests and offline analysis never run a model.
"""
import argparse
import copy
import datetime
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import stage_a_onnx_mask_graph as graph
import stage_a_epd_origin as origin
import stage_a_epd_spill as old
import stage_a_epd_instrument as instrument

ROOT=graph.ROOT
MODEL=Path('/home/user/stage_a_cube_model/cube_maskrcnn.onnx')
TARGET='/network/roi_heads/Slice_12_output_0'
NAMES=['boxes','labels','scores','masks',TARGET]
VENDOR=Path('/home/user/epd_ros2_ws/install/epd_onnxruntime_vendor/opt/epd_onnxruntime_vendor')
JSONCPP=Path('/home/user/epd_ros2_ws/build/jsoncpp_vendor/jsoncpp_install')
INPUT_SHA='4dbd106f22bf0f7c77305e9a426afbf0102a91effe62c4442ccb95e22586d2fe'
STATIC=graph.PAIR.parent/'onnx_mask_static'
STATIC_SHA={'graph.json':'f800744cf08a03c68149eead69b660916ca850371a207da8224df96af5f197e3',
            'coordinates.json':'14ef25012c0ea829bfee6a7b69ac81c2852060be14557414608ca4006846e616'}
require=graph.require
sha=graph.sha


def write_json(path,value):
    with Path(path).open('x') as f:f.write(json.dumps(value,indent=2,sort_keys=True)+'\n')


def validate_transform(original,diagnostic):
    import onnx
    expected=onnx.helper.make_tensor_value_info(TARGET,onnx.TensorProto.FLOAT,['N',1,28,28])
    require(len(original.graph.output)==4 and [v.name for v in original.graph.output]==NAMES[:4],'unexpected original outputs')
    require(len(diagnostic.graph.output)==5 and diagnostic.graph.output[-1]==expected,'changed observation contract')
    stripped=copy.deepcopy(diagnostic);del stripped.graph.output[-1]
    require(stripped.SerializeToString()==original.SerializeToString(),'changed original computation/model fields')
    producers=[n for n in original.graph.node if TARGET in n.output]
    require(len(producers)==1 and producers[0].name=='/network/roi_heads/Slice_12','ambiguous/wrong observation tensor')
    return dict(change='append one graph output only; stripping it restores original protobuf exactly',
                original_computation_sha256=sha(original.SerializeToString()),
                output_names=NAMES,observation_shape=['N',1,28,28],observation_dtype='FLOAT',expected_N3_bytes=9408)


def expose(original):
    import onnx
    diagnostic=copy.deepcopy(original)
    diagnostic.graph.output.append(onnx.helper.make_tensor_value_info(TARGET,onnx.TensorProto.FLOAT,['N',1,28,28]))
    validate_transform(original,diagnostic)
    return diagnostic


def select_class_fixture(probabilities,labels):
    require(probabilities.ndim==4 and probabilities.shape[1:]==(2,28,28) and probabilities.dtype==np.float32,'invalid fixture head')
    require(labels.shape==(len(probabilities),) and labels.dtype==np.int64 and np.all((labels>=0)&(labels<2)),'invalid fixture classes')
    return probabilities.reshape(-1,784)[np.arange(len(labels))*2+labels].reshape(-1,1,28,28).copy()


def contributors(head,sx,sy):
    require(head.shape==(28,28) and head.dtype==np.float32 and np.isfinite(head).all(),'invalid observed head')
    sx=float(np.clip(sx,0,29));sy=float(np.clip(sy,0,29))
    x0=int(np.floor(sx));y0=int(np.floor(sy));x1=min(x0+1,29);y1=min(y0+1,29)
    wx=sx-x0;wy=sy-y0
    coords=[(y0,x0),(y0,x1),(y1,x0),(y1,x1)]
    weights=[(1-wy)*(1-wx),(1-wy)*wx,wy*(1-wx),wy*wx]
    def p(y,x):return float(head[y-1,x-1]) if 1<=x<=28 and 1<=y<=28 else 0.
    probabilities=[p(y,x) for y,x in coords]
    nx=int(np.ceil(sx-.5));ny=int(np.ceil(sy-.5)) # nearest, tie to lower index; diagnostic only
    return dict(weights=weights,probabilities=probabilities,
        head_yx=[[y-1,x-1] if 1<=x<=28 and 1<=y<=28 else None for y,x in coords],
        value=sum(w*p for w,p in zip(weights,probabilities)),nearest_probability=p(ny,nx))


def comparability(before,after):
    require(len(before)==len(after)==4,'wrong original output inventory')
    rows=[dict(name=n,identical=a==b,historical_sha256=sha(a),observed_sha256=sha(b)) for n,a,b in zip(NAMES,before,after)]
    return dict(comparable=all(r['identical'] for r in rows),outputs=rows)


def claim_once(folder):
    write_json(Path(folder)/'execution_claim.json',dict(maximum_runs=1,retries=False,gazebo_runs=0,motion_goals=0))


def previous():
    raw,retained,artifacts,prior=graph.verified_inputs(MODEL)
    for n,h in STATIC_SHA.items():require(sha(gzip.decompress((STATIC/(n+'.gz')).read_bytes()))==h,'changed static evidence')
    pre=json.loads(gzip.decompress((origin.FROZEN/'preflight.json.gz').read_bytes()))
    for name,digest in pre['historical_loaded_library_sha256'].items():
        require(sha(Path(name).read_bytes())==digest,'changed historical runtime: '+name)
    require(instrument.external_sources()==pre['external_source_sha256'],'changed external EPD source')
    require(subprocess.check_output(['git','-C',str(instrument.EPD),'status','--porcelain=v1'],text=True)==pre['external_git_status'],'changed external EPD status')
    return raw,retained,artifacts,prior,pre


def prepare(folder):
    import onnx
    folder=Path(folder).resolve();folder.mkdir()
    raw,_,_,_,historical=previous()
    original=onnx.load_model_from_string(raw);diagnostic=expose(original)
    onnx.checker.check_model(diagnostic)
    data=diagnostic.SerializeToString();(folder/'diagnostic.onnx').write_bytes(data)
    transform=validate_transform(original,onnx.load_model_from_string(data))
    transform.update(original_model_sha256=sha(raw),diagnostic_model_sha256=sha(data),diagnostic_bytes=len(data))
    write_json(folder/'transformation.json',transform)
    (folder/'appended_output.pb').write_bytes(diagnostic.graph.output[-1].SerializeToString())
    source=ROOT/'scripts/stage_a_rgbd/onnx_mask_observe.cpp';binary=folder/'observe'
    args=['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(source),'-I'+str(VENDOR/'include'),
          '-I'+str(JSONCPP/'include'),'-L'+str(VENDOR/'lib'),'-Wl,-rpath,'+str(VENDOR/'lib'),
          '-Wl,-rpath,'+str(JSONCPP/'lib'),'-lonnxruntime',str(JSONCPP/'lib/libjsoncpp.so.1.9.5'),'-o',str(binary)]
    result=subprocess.run(args,capture_output=True,timeout=90)
    (folder/'build.stdout').write_bytes(result.stdout);(folder/'build.stderr').write_bytes(result.stderr)
    require(result.returncode==0,'diagnostic build failed')
    synthetic=folder/'synthetic';synthetic.mkdir()
    result=subprocess.run([str(binary),'--synthetic',str(synthetic)],capture_output=True,timeout=15)
    (folder/'synthetic.stdout').write_bytes(result.stdout);(folder/'synthetic.stderr').write_bytes(result.stderr)
    require(result.returncode==0,'typed synthetic extraction failed')
    expected=np.array([-0.,.5,np.nextafter(np.float32(.5),np.float32(1))],dtype='<f4').tobytes()
    require((synthetic/'outputs_0.bin').read_bytes()==expected and
            (synthetic/'outputs_1.bin').read_bytes()==np.array([1,0,4294967297],dtype='<i8').tobytes(),'synthetic extraction changed values/type/order')
    inp=gzip.decompress((origin.FROZEN/'output/session_inputs_before_Run_0.bin.gz').read_bytes())
    require(sha(inp)==INPUT_SHA and len(inp)==3145728,'changed historical preprocessed input')
    (folder/'input.f32').write_bytes(inp)
    dependencies=instrument.dependencies(binary)
    allowed=set(historical['historical_loaded_library_sha256'].values())
    require(all(h in allowed for h in dependencies.values()),'diagnostic runtime differs from historical loader closure')
    require(not os.environ.get('LD_PRELOAD'),'unaccounted preload')
    files=[MODEL,MODEL.with_name('labels.txt'),source,Path(__file__),ROOT/'tests/test_stage_a_onnx_head_observe.py',
           binary,folder/'diagnostic.onnx',folder/'input.f32',folder/'transformation.json',folder/'appended_output.pb',
           Path('/usr/bin/c++').resolve(),*sorted((VENDOR/'include').glob('onnxruntime*api.h')),
           VENDOR/'include/onnxruntime_cxx_inline.h',VENDOR/'include/onnxruntime_float16.h',
           *sorted((JSONCPP/'include/json').glob('*.h')),*sorted(synthetic.iterdir())]
    snapshots=folder/'source';snapshots.mkdir()
    for p in (source,Path(__file__),ROOT/'tests/test_stage_a_onnx_head_observe.py'):
        (snapshots/p.name).write_bytes(p.read_bytes())
    files+=list(snapshots.iterdir())
    write_json(folder/'preflight.json',dict(baseline='918659a0',model_sha256=graph.MODEL_SHA,input_sha256=INPUT_SHA,
        diagnostic_model_sha256=sha(data),binary=str(binary),source_artifact_tool_sha256={str(p):sha(p.read_bytes()) for p in files},
        dependencies=dependencies,allowed_historical_library_sha256=sorted(allowed),external_source_sha256=historical['external_source_sha256'],
        external_git_status=historical['external_git_status'],build_command=args,build_exit_code=0,synthetic_exit_code=0,
        session=dict(backend='CPU',intra_op_threads=2,inter_op_threads=1,execution='ORT_SEQUENTIAL',optimization='ORT_ENABLE_ALL'),
        python=sys.version,onnx_version=onnx.__version__,environment={k:os.environ.get(k) for k in ('LD_LIBRARY_PATH','LD_PRELOAD')},
        inference_calls_so_far=0,maximum_inference_calls=1,timeout_seconds=120))
    return transform


def execute(folder):
    folder=Path(folder).resolve();pre=json.loads((folder/'preflight.json').read_text());previous()
    require(pre['build_exit_code']==pre['synthetic_exit_code']==0,'readiness gate failed')
    for p,h in {**pre['source_artifact_tool_sha256'],**pre['dependencies']}.items():
        require(sha(Path(p).read_bytes())==h,'preflight changed: '+p)
    require(instrument.dependencies(Path(pre['binary']))==pre['dependencies'],'changed loader resolution')
    require(all(os.environ.get(k)==v for k,v in pre['environment'].items()),'changed loader environment')
    output=folder/'output';output.mkdir();claim_once(folder)
    args=[pre['binary'],str(folder/'diagnostic.onnx'),str(folder/'input.f32'),str(output)]
    start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (folder/'stdout.log').open('xb') as out,(folder/'stderr.log').open('xb') as err:
        try:
            result=subprocess.run(args,stdout=out,stderr=err,timeout=120)
            status=dict(exit_code=result.returncode,timeout=False)
        except subprocess.TimeoutExpired:status=dict(exit_code=None,timeout=True)
    status.update(command=args,started_utc=start,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  process_launches=1,maximum_inference_calls=1,gazebo_runs=0,motion_goals=0)
    write_json(folder/'process_exit.json',status)
    # No retry regardless of status. Hashes and checkout attestation after execution.
    loaded={}
    for name in ('maps_before_run.txt','maps_after_run.txt'):
        if (output/name).exists():
            for line in (output/name).read_text().splitlines():
                p=line.split()[-1]
                if p.startswith('/') and ('.so' in p or 'ld-linux' in p):loaded[p]=sha(Path(p).read_bytes())
    write_json(folder/'loaded_library_sha256.json',loaded)
    post=dict(model_sha256=sha(MODEL.read_bytes()),diagnostic_model_sha256=sha((folder/'diagnostic.onnx').read_bytes()),
              input_sha256=sha((folder/'input.f32').read_bytes()),external_source_sha256=instrument.external_sources(),
              external_git_status=subprocess.check_output(['git','-C',str(instrument.EPD),'status','--porcelain=v1'],text=True))
    write_json(folder/'after.json',post)
    return status


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['prepare','execute']);p.add_argument('folder',type=Path);a=p.parse_args()
    print(json.dumps(prepare(a.folder) if a.mode=='prepare' else execute(a.folder),indent=2))
