#!/usr/bin/env python3
"""Build/prove the pinned bridge executable shutdown fix; never modify /opt."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

SOURCE_REPOSITORY='https://github.com/ros2-gbp/ros_ign-release.git'
SOURCE_COMMIT='90cdc5361059a6f949bc004658e7363df33bcffe'
SOURCE_TAG='release/humble/ros_gz_bridge/0.244.26-1'
UPSTREAM_COMMIT='63793185bbbe58732b6eac8613937dc1bcb6ea2c'
SOURCE_MAIN_SHA256='92c215bec11c013d403ae037b48b3aa670ca7e7aacc1d4904683e41ab017760d'


def run(command,**kwargs):
    return subprocess.run([str(x) for x in command],check=True,**kwargs)


def output(command):
    return subprocess.check_output([str(x) for x in command]).decode().strip()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def apply_reviewed_patch(source,patch):
    diff=subprocess.check_output(['git','-C',str(source),'diff','HEAD','--binary'])
    if not diff:
        run(['git','-C',source,'apply','--check',patch]);run(['git','-C',source,'apply',patch])
        diff=subprocess.check_output(['git','-C',str(source),'diff','HEAD','--binary'])
    if diff!=patch.read_bytes() or output(['git','-C',source,'ls-files','--others','--exclude-standard']):
        raise RuntimeError('overlay source contains changes beyond the reviewed patch')
    return diff


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/home/ubuntu/workcell_ws/ros_gz_bridge_shutdown_overlay'))
    parser.add_argument('--valgrind',type=Path,default=shutil.which('valgrind'))
    parser.add_argument('--valgrind-lib',type=Path)
    parser.add_argument('--baseline-results',type=Path,
                        help='reuse preserved canonical baseline SIGABRT proof for these exact binaries')
    args=parser.parse_args()
    if not args.valgrind or not args.valgrind.is_file():parser.error('provide installed or locally extracted --valgrind')
    if args.valgrind_lib:os.environ['VALGRIND_LIB']=str(args.valgrind_lib.resolve())
    repo=Path(__file__).resolve().parents[1];root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    manifest_path=root/'provenance.json';manifest_path.unlink(missing_ok=True)
    package='ros-humble-ros-gz-bridge';version=output(['dpkg-query','-W','-f=${Version}',package])
    if not version.startswith('0.244.26-1'):raise RuntimeError('installed bridge version differs from pinned source')
    source=root/'src/ros_gz_bridge';source.parent.mkdir(exist_ok=True)
    if not source.exists():run(['git','clone','--depth','1','--branch',SOURCE_TAG,SOURCE_REPOSITORY,source])
    if output(['git','-C',source,'rev-parse','HEAD'])!=SOURCE_COMMIT:raise RuntimeError('source revision differs')
    original=subprocess.check_output(['git','-C',str(source),'show','HEAD:src/parameter_bridge.cpp'])
    if hashlib.sha256(original).hexdigest()!=SOURCE_MAIN_SHA256:raise RuntimeError('source main differs')
    header=source/'include/ros_gz_bridge/ros_gz_bridge.hpp'
    installed_header=Path('/opt/ros/humble/include/ros_gz_bridge/ros_gz_bridge/ros_gz_bridge.hpp')
    if header.read_bytes()!=installed_header.read_bytes():raise RuntimeError('installed bridge API differs from pinned source')
    patch=repo/'patches/ros_gz_bridge_humble_shutdown.patch'
    diff=apply_reviewed_patch(source,patch)
    proof=repo/'scripts/bridge_shutdown_repro';build=root/'build';install=root/'install'
    run(['cmake','-S',proof,'-B',build,'-DBRIDGE_SOURCE='+str(source),
         '-Dros_gz_bridge_DIR=/opt/ros/humble/share/ros_gz_bridge/cmake',
         '-DCMAKE_BUILD_TYPE=RelWithDebInfo','-DCMAKE_INSTALL_PREFIX='+str(install),'-DPython3_EXECUTABLE=/usr/bin/python3'])
    run(['cmake','--build',build,'--parallel','2']);run(['cmake','--install',build])
    binary=install/'lib/ros_gz_bridge/parameter_bridge';library=Path('/opt/ros/humble/lib/libros_gz_bridge.so').resolve()
    baseline=Path('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge').resolve()
    evidence=root/'evidence'/datetime.datetime.now(datetime.timezone.utc).strftime('run-%Y%m%d-%H%M%S')
    evidence.mkdir(parents=True)
    (evidence/'source.patch').write_bytes(diff)
    common=['/usr/bin/python3',proof/'verify.py','--publisher',build/'bridge_traffic','--library',library]
    if args.baseline_results:
        prior=json.loads(args.baseline_results.read_text())
        if (not prior.get('expected_abort') or not prior.get('all_success') or
                prior.get('binary')!=str(baseline) or prior.get('binary_sha256')!=sha(baseline) or
                prior.get('library')!=str(library) or prior.get('library_sha256')!=sha(library) or
                not any(row.get('passed') and row.get('returncodes')==[0,-6]
                        for row in prior.get('results',[]))):
            raise RuntimeError('prior baseline proof does not match current installed binaries')
        shutil.copytree(args.baseline_results.resolve().parent,evidence/'baseline')
    else:
        run([*common,'--binary',baseline,'--cycles','20','--expect-abort','--output',evidence/'baseline'])
    run([*common,'--binary',binary,'--cycles','20','--output',evidence/'native'])
    run([*common,'--binary',binary,'--cycles','5','--valgrind',args.valgrind,'--output',evidence/'memcheck'])
    def results(name):
        path=evidence/name/'results.json';record=json.loads(path.read_text())
        if not record['all_success']:raise RuntimeError(name+' proof failed')
        return dict(cycles=record['completed'],passed=record['passed'],errors=record['errors'],
                    results_path=str(path),results_sha256=sha(path))
    baseline_proof=json.loads((evidence/'baseline/results.json').read_text())
    manifest=dict(schema='workcell_ros_gz_bridge_shutdown/v1',
        source=dict(repository=SOURCE_REPOSITORY,commit=SOURCE_COMMIT,tag=SOURCE_TAG,version='0.244.26',
                    upstream_commit=UPSTREAM_COMMIT,pristine_main_sha256=SOURCE_MAIN_SHA256,
                    patched_main_sha256=sha(source/'src/parameter_bridge.cpp'),installed_header_sha256=sha(installed_header)),
        patch=dict(path=str(patch),sha256=sha(patch)),
        executable=dict(path=str(binary.resolve()),sha256=sha(binary)),library=dict(path=str(library),sha256=sha(library)),
        baseline=dict(package=package,version=version,executable=str(baseline),sha256=sha(baseline),
                      results_path=str(evidence/'baseline/results.json'),results_sha256=sha(evidence/'baseline/results.json')),
        reproduction=dict(baseline_returncode=baseline_proof['results'][-1]['returncodes'][1],
                          native=results('native'),memcheck=results('memcheck')),
        build_script_sha256=sha(__file__),proof_script_sha256=sha(proof/'verify.py'),
        publisher_source_sha256=sha(proof/'publisher.cpp'),configuration_sha256=sha(proof/'bridges.yaml'))
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    (root/'setup.bash').write_text('export ROS_GZ_BRIDGE_SHUTDOWN_OVERLAY='+shlex.quote(str(root))+'\n')
    print('Qualified bridge shutdown overlay: '+str(manifest_path))


if __name__=='__main__':main()
