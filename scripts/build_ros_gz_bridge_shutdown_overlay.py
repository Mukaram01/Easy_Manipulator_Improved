#!/usr/bin/env python3
"""Build/prove the pinned bridge executable shutdown fix; never modify /opt."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from bridge_shutdown_repro.qualification import (LEGACY_METHOD, LIFETIME_METHOD, OBSERVED_METHOD,
    identity, invalidate_publication, publish, validate_lifetime, validate_observed)

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


def argument_parser():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='overlay root (a new directory per qualification)')
    parser.add_argument('--build-only',action='store_true',
                        help='build the patched bridge and stop: nothing is proved or published (observation input)')
    parser.add_argument('--valgrind',type=Path,default=shutil.which('valgrind'))
    parser.add_argument('--valgrind-lib',type=Path)
    parser.add_argument('--baseline-results',type=Path,
                        help='reuse preserved canonical baseline SIGABRT proof for these exact binaries')
    parser.add_argument('--qualification-method',choices=(LEGACY_METHOD,LIFETIME_METHOD,OBSERVED_METHOD),default=LEGACY_METHOD)
    parser.add_argument('--lifetime-evidence',type=Path,help='source-bound lifetime or natural shutdown-order evidence record')
    parser.add_argument('--mechanism-review',type=Path,help='independent review record bound to the exact evidence and implementation')
    return parser


def validate_arguments(parser,args):
    if args.build_only and (args.qualification_method!=LEGACY_METHOD or args.lifetime_evidence or
                            args.mechanism_review or args.baseline_results):
        parser.error('--build-only builds the observation input; it takes no qualification records')
    if args.qualification_method in (LIFETIME_METHOD,OBSERVED_METHOD):
        for variable in ('LD_PRELOAD','LD_AUDIT','VALGRIND_LIB','VALGRIND_OPTS'):
            if os.environ.get(variable):
                parser.error('alternative qualification rejects ambient '+variable)
        if not args.lifetime_evidence or not args.mechanism_review:
            parser.error('alternative qualification requires --lifetime-evidence and --mechanism-review')
        if args.baseline_results:
            parser.error('installed crash results cannot stand in for separately bound alternative evidence')
        if args.valgrind_lib or not args.valgrind or args.valgrind.resolve()!=Path('/usr/bin/valgrind').resolve():
            parser.error('alternative qualification requires /usr/bin/valgrind without an alternate library')
    elif args.lifetime_evidence or args.mechanism_review:
        parser.error('alternative records require an explicit alternative --qualification-method')
    if not args.valgrind or not args.valgrind.is_file():parser.error('provide installed or locally extracted --valgrind')


def main():
    parser=argument_parser();args=parser.parse_args();validate_arguments(parser,args)
    if args.valgrind_lib:os.environ['VALGRIND_LIB']=str(args.valgrind_lib.resolve())
    repo=Path(__file__).resolve().parents[1];root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    manifest_path=root/'provenance.json'
    evidence=root/'evidence'/datetime.datetime.now(datetime.timezone.utc).strftime('run-%Y%m%d-%H%M%S-%f')
    evidence.mkdir(parents=True,exist_ok=False)
    for name in ('provenance.json','setup.bash'):
        if (root/name).is_file():shutil.copy2(root/name,evidence/('previous-'+name))
    invalidate_publication(root)
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
    if args.build_only:
        print('Built unqualified bridge for observation (not published): '+str(binary.resolve()));return
    baseline=Path('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge').resolve()
    (evidence/'source.patch').write_bytes(diff)
    common=['/usr/bin/python3',proof/'verify.py','--publisher',build/'bridge_traffic','--library',library]
    if args.qualification_method in (LIFETIME_METHOD,OBSERVED_METHOD):
        pass  # The separate source-bound record is checked before any runtime cycles.
    elif args.baseline_results:
        prior=json.loads(args.baseline_results.read_text())
        if (not prior.get('expected_abort') or not prior.get('all_success') or
                prior.get('binary')!=str(baseline) or prior.get('binary_sha256')!=sha(baseline) or
                prior.get('library')!=str(library) or prior.get('library_sha256')!=sha(library) or
                not any(row.get('passed') and row.get('returncodes')==[0,-6]
                        for row in prior.get('results',[]))):
            raise RuntimeError('prior baseline proof does not match current installed binaries')
        import importlib.util
        spec=importlib.util.spec_from_file_location('bridge_proof',proof/'verify.py')
        verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
        if not any(row.get('passed') and verifier.baseline_lifetime_abort(
                row.get('returncodes'),
                (args.baseline_results.parent/f"{row['cycle']:02}-bridge.log").read_text())
                for row in prior.get('results', [])):
            raise RuntimeError('prior baseline lacks the preserved shutdown-corruption signature')
        shutil.copytree(args.baseline_results.resolve().parent,evidence/'baseline')
    else:
        run([*common,'--binary',baseline,'--cycles','20','--expect-abort','--output',evidence/'baseline'])
    manifest=dict(schema='workcell_ros_gz_bridge_shutdown/v1',
        source=dict(repository=SOURCE_REPOSITORY,commit=SOURCE_COMMIT,tag=SOURCE_TAG,version='0.244.26',
                    upstream_commit=UPSTREAM_COMMIT,pristine_main_sha256=SOURCE_MAIN_SHA256,
                    patched_main_sha256=sha(source/'src/parameter_bridge.cpp'),installed_header_sha256=sha(installed_header)),
        patch=dict(path=str(patch),sha256=sha(patch)),
        executable=dict(path=str(binary.resolve()),sha256=sha(binary)),library=dict(path=str(library),sha256=sha(library)),
        baseline=dict(package=package,version=version,executable=str(baseline),sha256=sha(baseline)),
        reproduction={},qualification=dict(method=args.qualification_method),
        runtime_environment={key:os.environ.get(key,'') for key in
            ('RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY','LD_PRELOAD','LD_AUDIT','VALGRIND_LIB','VALGRIND_OPTS')},
        build_script_sha256=sha(__file__),proof_script_sha256=sha(proof/'verify.py'),
        shutdown_context_test=dict(source_sha256=sha(proof/'shutdown_context_test.cpp'),
            log_path=str(evidence/'shutdown-context-test.log'),
            log_sha256=None),
        publisher_source_sha256=sha(proof/'publisher.cpp'),configuration_sha256=sha(proof/'bridges.yaml'))
    if args.qualification_method in (LIFETIME_METHOD,OBSERVED_METHOD):
        lifetime=json.loads(args.lifetime_evidence.read_text())
        manifest['qualification'].update(evidence=identity(args.lifetime_evidence),
            review=identity(args.mechanism_review),runtime_dependencies=lifetime['runtime']['dependencies'])
        validate_alternative=validate_observed if args.qualification_method==OBSERVED_METHOD else validate_lifetime
        validate_alternative(manifest,include_runtime=False)
    else:
        baseline_path=evidence/'baseline/results.json'
        baseline_proof=json.loads(baseline_path.read_text())
        manifest['baseline'].update(results_path=str(baseline_path),results_sha256=sha(baseline_path))
        manifest['reproduction']['baseline_returncode']=baseline_proof['results'][-1]['returncodes'][1]
    with (evidence/'shutdown-context-test.log').open('w') as log:
        run([build/'shutdown_context_test'], stdout=log, stderr=subprocess.STDOUT,
            env=dict(os.environ, ROS_DOMAIN_ID='198', ROS_LOCALHOST_ONLY='1'))
    manifest['shutdown_context_test']['log_sha256']=sha(evidence/'shutdown-context-test.log')
    manifest['runtime_started_ns']=time.time_ns()
    run([*common,'--binary',binary,'--cycles','20','--output',evidence/'native'])
    run([*common,'--binary',binary,'--cycles','5','--valgrind',args.valgrind,'--output',evidence/'memcheck'])
    for name in ('native','memcheck'):
        path=evidence/name/'results.json';record=json.loads(path.read_text())
        manifest['reproduction'][name]=dict(cycles=record['completed'],passed=record['passed'],errors=record['errors'],
            results_path=str(path),results_sha256=sha(path),
            artifacts=[identity(p) for p in sorted(path.parent.iterdir()) if p.name!='results.json'])
    publish(root,manifest,patch)
    print('Qualified bridge shutdown overlay: '+str(manifest_path))


if __name__=='__main__':main()
