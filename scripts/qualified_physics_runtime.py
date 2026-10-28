#!/usr/bin/env python3
"""The workspace's qualified physics runtime: registered once from qualification evidence, then discovered.

``register`` takes the MoveIt teardown overlay and the native physics overlay that a passing Stage-A
acceptance run actually used (its final report and full-cycle receipt), and a published bridge
shutdown overlay, validates each with its own admission check, and writes the generated record
``<workspace>/workcell_qualified_runtime.json`` (identities and hashes; nobody edits it).

``env`` (what the Workcell Builder runs) re-validates that record against the files on disk and prints
the shell environment for the qualified runner; any changed, missing or unqualified component fails
closed with the reason and the command that requalifies or re-registers it. ``check`` reports it as JSON.
``moveit-env`` (fake-hardware planning) prints only the registered MoveIt teardown overlay, still bound by
its manifest hash; with no record it prints nothing and the planning runner reports unpatched MoveIt.
The workspace is this checkout's colcon workspace (``<repo>/../..``); no user name or path is assumed.
"""
import argparse
import hashlib
import json
import os
import shlex
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
WORKSPACE = REPO.parents[1]
RECORD_NAME = 'workcell_qualified_runtime.json'
SCHEMA = 'workcell_qualified_runtime/v1'
sys.path.insert(0, str(SCRIPTS))


class RuntimeUnavailable(RuntimeError):
    """The workspace has no (or no longer a) qualified physics runtime."""


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _bridge(root):
    from simulator_backend import read_bridge_overlay
    root = Path(root).resolve()
    data = read_bridge_overlay(root / 'provenance.json', REPO / 'patches/ros_gz_bridge_humble_shutdown.patch')
    if not (root / 'setup.bash').is_file():
        raise RuntimeUnavailable(f'bridge overlay {root} has no published setup.bash')
    return dict(root=str(root), manifest_sha256=_sha256(root / 'provenance.json'),
                executable_sha256=data['executable']['sha256'])


def _physics(root, receipt_sha256=None):
    from simulator_backend import simulator_physics_contract
    previous = os.environ.get('WORKCELL_SIMULATOR_PHYSICS_ROOT')
    os.environ['WORKCELL_SIMULATOR_PHYSICS_ROOT'] = str(root)
    try:
        contract = simulator_physics_contract()
    finally:
        if previous is None:
            os.environ.pop('WORKCELL_SIMULATOR_PHYSICS_ROOT')
        else:
            os.environ['WORKCELL_SIMULATOR_PHYSICS_ROOT'] = previous
    if receipt_sha256 is not None and contract['receipt_sha256'] != receipt_sha256:
        raise RuntimeUnavailable(f'physics overlay {root} receipt differs from the qualified run')
    return dict(root=contract['root'], receipt_sha256=contract['receipt_sha256'])


def _moveit(manifest_path, manifest_sha256=None):
    manifest_path = Path(manifest_path).resolve()
    if manifest_sha256 is not None and _sha256(manifest_path) != manifest_sha256:
        raise RuntimeUnavailable(f'MoveIt overlay provenance {manifest_path} changed since qualification')
    root = manifest_path.parent
    setup = root / 'install/setup.bash'
    if not setup.is_file():
        raise RuntimeUnavailable(f'MoveIt overlay {root} has no install/setup.bash')
    return dict(root=str(root), manifest=str(manifest_path), manifest_sha256=_sha256(manifest_path))


def _acceptance(directory):
    directory = Path(directory).resolve()
    report = json.loads((directory / 'stage-a1-final-report.json').read_text())
    if report.get('status') != 'PASS' or report.get('stage_a_acceptance') is not True:
        raise RuntimeUnavailable(f'{directory} is not a passing Stage-A acceptance run')
    receipt = json.loads((directory / '06-full-cycle/runtime/receipt.json').read_text())
    return directory, report, receipt


def register(args):
    workspace = Path(args.workspace).resolve()
    directory, report, receipt = _acceptance(args.from_acceptance)
    physics = _physics(receipt['native_physics']['root'], receipt['native_physics']['receipt_sha256'])
    moveit = _moveit(report['preflight']['commissioning_build']['moveit_overlay']['manifest_path'])
    bridge = _bridge(args.bridge_overlay)
    ros_setup = Path(f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash")
    record = dict(schema=SCHEMA, workspace=str(workspace), ros_setup=str(ros_setup),
                  rmw_implementation='rmw_fastrtps_cpp', workspace_setup=str(workspace / 'install/setup.bash'),
                  stage_a_acceptance=dict(evidence=str(directory), report_sha256=_sha256(directory / 'stage-a1-final-report.json'),
                                          repo_head=report['preflight']['repo_head']),
                  moveit_overlay=moveit, bridge_overlay=bridge, physics_overlay=physics)
    _validate(record)
    path = workspace / RECORD_NAME
    path.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(dict(status='REGISTERED', record=str(path)), indent=2))
    return 0


def _validate(record):
    if record.get('schema') != SCHEMA:
        raise RuntimeUnavailable('unknown qualified runtime record schema')
    for key in ('ros_setup', 'workspace_setup'):
        if not Path(record[key]).is_file():
            raise RuntimeUnavailable(f'{key} {record[key]} is missing (build the workspace with colcon first)')
    evidence = Path(record['stage_a_acceptance']['evidence'])
    if _sha256(evidence / 'stage-a1-final-report.json') != record['stage_a_acceptance']['report_sha256']:
        raise RuntimeUnavailable('the Stage-A acceptance report changed since registration')
    _acceptance(evidence)
    _moveit(record['moveit_overlay']['manifest'], record['moveit_overlay']['manifest_sha256'])
    _physics(record['physics_overlay']['root'], record['physics_overlay']['receipt_sha256'])
    bridge = _bridge(record['bridge_overlay']['root'])
    if bridge['manifest_sha256'] != record['bridge_overlay']['manifest_sha256']:
        raise RuntimeUnavailable('the bridge overlay publication changed since registration; register again')
    return record


def load(workspace=WORKSPACE):
    path = Path(workspace).resolve() / RECORD_NAME
    if not path.is_file():
        raise RuntimeUnavailable(
            f'no qualified physics runtime is registered for workspace {Path(workspace).resolve()}: run '
            f'scripts/qualified_physics_runtime.py register --from-acceptance <Stage-A evidence> --bridge-overlay <overlay>')
    try:
        return _validate(json.loads(path.read_text()))
    except (OSError, KeyError, ValueError, TypeError) as exc:
        raise RuntimeUnavailable(f'qualified physics runtime of {Path(workspace).resolve()} rejected: {exc}') from exc


def shell_environment(record):
    q = shlex.quote
    physics = record['physics_overlay']['root']
    return '\n'.join([
        f"source {q(record['ros_setup'])}",
        f"export RMW_IMPLEMENTATION={q(record['rmw_implementation'])}",
        f"source {q(record['workspace_setup'])}",
        f"source {q(str(Path(record['moveit_overlay']['root']) / 'install/setup.bash'))}",
        f"source {q(str(Path(record['bridge_overlay']['root']) / 'setup.bash'))}",
        f"export WORKCELL_SIMULATOR_PHYSICS_ROOT={q(physics)}",
        f"export LD_LIBRARY_PATH={q(physics + '/lib')}\"${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}\"",
        f"export WORKCELL_STAGE_A_PRIOR_EVIDENCE={q(record['stage_a_acceptance']['evidence'])}",
    ]) + '\n'


def moveit_environment(workspace=WORKSPACE):
    """The qualified MoveIt teardown overlay alone, for fake-hardware planning (no physics prerequisites).

    No registered record selects installed MoveIt; a registered overlay that changed fails closed. The
    planning runner still verifies the binaries the owned MoveGroup actually maps."""
    path = Path(workspace).resolve() / RECORD_NAME
    if not path.is_file():
        return ''
    try:
        record = json.loads(path.read_text())
        if record.get('schema') != SCHEMA:
            raise RuntimeUnavailable('unknown qualified runtime record schema')
        moveit = _moveit(record['moveit_overlay']['manifest'], record['moveit_overlay']['manifest_sha256'])
    except (OSError, KeyError, ValueError, TypeError) as exc:
        raise RuntimeUnavailable(f'qualified MoveIt overlay of {Path(workspace).resolve()} rejected: {exc}') from exc
    return f"source {shlex.quote(str(Path(moveit['root']) / 'install/local_setup.bash'))}\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    reg = sub.add_parser('register')
    reg.add_argument('--from-acceptance', type=Path, required=True, help='passing Stage-A acceptance evidence directory')
    reg.add_argument('--bridge-overlay', type=Path, required=True, help='published bridge shutdown overlay root')
    for name in ('env', 'check', 'moveit-env'):
        sub.add_parser(name)
    for command in sub.choices.values():
        command.add_argument('--workspace', type=Path, default=WORKSPACE)
    args = parser.parse_args(argv)
    try:
        if args.command == 'register':
            return register(args)
        if args.command == 'moveit-env':
            sys.stdout.write(moveit_environment(args.workspace))
            return 0
        record = load(args.workspace)
    except (RuntimeUnavailable, RuntimeError, OSError, KeyError, ValueError) as exc:
        print(('echo ' + shlex.quote('QUALIFIED_RUNTIME_UNAVAILABLE: ' + str(exc)) + ' >&2; false')
              if args.command in ('env', 'moveit-env') else json.dumps(dict(status='UNAVAILABLE', reason=str(exc))))
        return 2
    sys.stdout.write(shell_environment(record) if args.command == 'env' else json.dumps(dict(status='READY', **record), indent=2) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
