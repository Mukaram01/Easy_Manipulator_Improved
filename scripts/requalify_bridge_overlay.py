#!/usr/bin/env python3
"""Bounded local requalification of the ros_gz_bridge shutdown overlay (observed-shutdown-order/v1).

Needed when the reviewed admission implementation itself changes (its bytes are bound by the
independent review), never merely because a checkout moved: repository files are admitted by
checkout-relative name and bytes (qualification.bound_identity). Each step reuses the reviewed
tools and writes only inside a NEW overlay root; a historical overlay or its evidence is never touched.

  1. build    build_ros_gz_bridge_shutdown_overlay.py --build-only (pinned source + reviewed patch)
  2. observe  observe.py, 5 natural shutdowns each of the installed and the patched bridge
  3. assemble the observed evidence record from those batches and pinned dependency sources,
              then the reviewer packet (exact implementation identities to review)
  4. review   performed by an independent reviewer, who writes qualification/mechanism-review.json
  5. qualify  build_ros_gz_bridge_shutdown_overlay.py --qualification-method observed-shutdown-order/v1
              (fresh shutdown-context test, 20 native + 5 Memcheck cycles, then atomic publication)

Every step fails closed; ``status`` reports which records exist and whether the publication admits.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
from bridge_shutdown_repro import qualification as q  # noqa: E402

OPT_BRIDGE = Path('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge')
OPT_LIBRARY = Path('/opt/ros/humble/lib/libros_gz_bridge.so')
# Pinned upstream sources of the shutdown path the review inspects (name: (checkout, files)).
DEPENDENCY_SOURCES = {
    'rclcpp': ('rclcpp', ['rclcpp/src/rclcpp/context.cpp', 'rclcpp/src/rclcpp/utilities.cpp',
                          'rclcpp/src/rclcpp/signal_handler.cpp']),
    'rcl': ('rcl', ['rcl/src/rcl/init.c', 'rcl/src/rcl/logging.c']),
    'rcutils': ('rcutils', ['src/logging.c']),
    'rcl_logging_spdlog': ('rcl_logging', ['rcl_logging_spdlog/src/rcl_logging_spdlog.cpp']),
    'spdlog': ('spdlog', ['include/spdlog/details/registry-inl.h'])}
SOURCE_TAGS = {'rclcpp': ('https://github.com/ros2/rclcpp.git', '16.0.21'),
               'rcl': ('https://github.com/ros2/rcl.git', '5.3.13'),
               'rcutils': ('https://github.com/ros2/rcutils.git', '5.1.10'),
               'rcl_logging': ('https://github.com/ros2/rcl_logging.git', '2.3.2'),
               'spdlog': ('https://github.com/gabime/spdlog.git', 'v1.9.2')}


def paths(root):
    root = Path(root).resolve()
    record = root / 'qualification'
    return dict(root=root, record=record, baseline=record / 'observations/baseline',
                patched=record / 'observations/patched', evidence=record / 'observed-evidence.json',
                packet=record / 'review-packet.json', review=record / 'mechanism-review.json',
                binary=root / 'install/lib/ros_gz_bridge/parameter_bridge', publisher=root / 'build/bridge_traffic')


def runtime_env():
    env = dict(os.environ, RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOCALHOST_ONLY='1')
    for key in ('LD_PRELOAD', 'LD_AUDIT', 'VALGRIND_LIB', 'VALGRIND_OPTS'):
        env.pop(key, None)
    return env


def build(args):
    p = paths(args.root)
    if (p['root'] / 'provenance.json').exists():
        raise SystemExit('refusing to rebuild a published overlay root; requalify into a new root')
    if args.source_from:
        source = p['root'] / 'src/ros_gz_bridge'
        if not source.exists():
            source.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(args.source_from, source, symlinks=True)   # the build verifies commit, main and patch
    subprocess.run([sys.executable, SCRIPTS / 'build_ros_gz_bridge_shutdown_overlay.py', '--output', p['root'],
                    '--build-only'], check=True, env=runtime_env())


def observe(args):
    p = paths(args.root)
    for name, binary in (('baseline', OPT_BRIDGE.resolve()), ('patched', p['binary'])):
        subprocess.run([sys.executable, SCRIPTS / 'bridge_shutdown_repro/observe.py', '--binary', binary,
                        '--publisher', p['publisher'], '--output', p[name], '--cycles', '5'],
                       check=True, env=runtime_env())


def _batch(folder):
    return dict(experiment=q.identity(folder / 'experiment.json'), results=q.identity(folder / 'results.json'),
                artifacts=[q.identity(item) for item in sorted(folder.glob('[0-9][0-9]/*'))])


def assemble(args):
    p = paths(args.root)
    sources_root = Path(args.sources).resolve()
    for name, (url, tag) in SOURCE_TAGS.items():
        target = sources_root / name
        if not target.exists():
            subprocess.run(['git', 'clone', '--depth', '1', '--branch', tag, url, str(target)], check=True)
    baseline, patched = _batch(p['baseline']), _batch(p['patched'])
    rows = json.loads(Path(patched['results']['path']).read_text())
    dependencies = [item for item in rows[0]['loaded_identities'] if Path(item['path']).name != OPT_LIBRARY.name]
    dependency_records = {}
    for name, (checkout, files) in DEPENDENCY_SOURCES.items():
        source = sources_root / checkout
        dependency_records[name] = dict(
            revision=subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
            files=[q.identity(source / item) for item in files])
    main_source = p['root'] / 'src/ros_gz_bridge/src/parameter_bridge.cpp'
    from build_ros_gz_bridge_shutdown_overlay import SOURCE_COMMIT
    evidence = dict(schema='workcell_bridge_observed_shutdown/v1', method=q.OBSERVED_METHOD, producer=args.producer,
                    source=dict(bridge_commit=SOURCE_COMMIT, files=[q.identity(main_source)], dependencies=dependency_records),
                    patch=q.identity(REPO / 'patches/ros_gz_bridge_humble_shutdown.patch'),
                    runtime=dict(executable=q.identity(p['binary']), library=q.identity(OPT_LIBRARY),
                                 source_main=q.identity(main_source), dependencies=dependencies),
                    observations=dict(baseline=baseline, patched=patched))
    manifest = dict(executable=evidence['runtime']['executable'], library=evidence['runtime']['library'],
                    baseline=dict(executable=str(OPT_BRIDGE.resolve()), sha256=q.digest(OPT_BRIDGE.resolve())),
                    runtime_environment=dict(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOCALHOST_ONLY='1',
                                             LD_PRELOAD='', LD_AUDIT='', VALGRIND_LIB='', VALGRIND_OPTS=''))
    seen, implementation = set(), []
    for name, corrected in (('baseline', False), ('patched', True)):
        implementation += q._observed_batch(evidence['observations'][name], manifest, dependencies, corrected, seen)
    p['evidence'].write_text(json.dumps(evidence, indent=2) + '\n')
    # Exactly what validate_observed requires the review to bind (evidence, sources, admission code).
    implementation = [evidence['patch'], *evidence['source']['files'],
                      *[item for record in dependency_records.values() for item in record['files']],
                      *implementation, *[q.identity(REPO / name) for name in q.REPOSITORY_FILES if name != 'patches/ros_gz_bridge_humble_shutdown.patch']]
    unique = list({q.bound_identity(item): item for item in implementation}.values())
    p['packet'].write_text(json.dumps(dict(
        schema='workcell_bridge_review_packet/v1', evidence=q.identity(p['evidence']),
        review_schema='workcell_bridge_observed_shutdown_review/v1', implementation=unique,
        required_assessment=['causal_adequacy', 'scheduling', 'correction', 'runtime_isolation', 'claim_limits',
                             'uninstall_join'],
        instructions='An independent reviewer (not the producer) inspects the evidence and implementation and writes '
                     'mechanism-review.json with disposition, reviewer, reviewed_at_ns, evidence_sha256, report '
                     '(file identity, >=200 chars), assessment, validation and this implementation list.'),
        indent=2) + '\n')
    print('observed batches admitted; evidence', p['evidence'], 'review packet', p['packet'])


def qualify(args):
    p = paths(args.root)
    subprocess.run([sys.executable, SCRIPTS / 'build_ros_gz_bridge_shutdown_overlay.py', '--output', p['root'],
                    '--qualification-method', q.OBSERVED_METHOD, '--lifetime-evidence', p['evidence'],
                    '--mechanism-review', p['review']], check=True, env=runtime_env())


def status(args):
    p = paths(args.root)
    report = {name: p[name].exists() for name in ('binary', 'baseline', 'patched', 'evidence', 'packet', 'review')}
    try:
        manifest = json.loads((p['root'] / 'provenance.json').read_text())
        q.validate_observed(manifest)
        report['published'] = 'ADMITTED'
    except FileNotFoundError:
        report['published'] = 'NOT_PUBLISHED'
    except ValueError as exc:
        report['published'] = 'REJECTED: ' + str(exc)
    print(json.dumps(report, indent=2))
    return 0 if report['published'] == 'ADMITTED' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='step', required=True)
    for name in ('build', 'observe', 'assemble', 'qualify', 'status'):
        step = sub.add_parser(name)
        step.add_argument('--root', type=Path, required=True, help='new overlay root')
        if name == 'build':
            step.add_argument('--source-from', type=Path, help='existing pinned ros_gz_bridge checkout to copy (else cloned)')
        if name == 'assemble':
            step.add_argument('--sources', type=Path, required=True, help='pinned dependency source checkouts')
            step.add_argument('--producer', required=True, help='who produced the evidence (the reviewer must differ)')
    args = parser.parse_args(argv)
    return globals()[args.step](args) or 0


if __name__ == '__main__':
    sys.exit(main())
