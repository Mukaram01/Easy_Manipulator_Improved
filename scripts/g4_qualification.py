#!/usr/bin/env python3
"""G4 qualification judge (offline; issues and admits nothing; needs ROS 2 sourced for the numeric tracking).

  judge session|stop|cancel RUN     one trial run's verdict (g4_session_evidence / g4_stop_after_cycle_evidence /
                                    g4_cancel_evidence), numeric tracking bound to this checkout's g4_tracking_evidence
  readiness --cell CELL --session RUN (x3) --stop RUN --cancel RUN [--other RUN ...]
                                    g4_qualification_readiness of the exact cell against this workspace's native build,
                                    runtime and rendered-EPD perception, at this checkout's planner and G4 control identity

Production trial roots (every listed run lies under one; every authorised trial there must be listed; unreadable trial
evidence there blocks): <workspace>/log/qualification/g4/{qualification,g4_e1}. Broad scan: every authorised trial with
a runner report anywhere under <workspace>/log/qualification must be listed too; records without one (the deliberately
forged negative-test probes) are reported as ignored. Exit 0 only for a PASS / QUALIFIED_READY verdict."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

import physics_scope_qualification as scope  # noqa: E402
from g4_tracking_evidence import session_tracking  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--workspace', type=Path, default=SCRIPTS.parents[2], help='the colcon workspace (cad_ws)')
    commands = parser.add_subparsers(dest='command', required=True)
    judge = commands.add_parser('judge')
    judge.add_argument('kind', choices=['session', 'stop', 'cancel'])
    judge.add_argument('run', type=Path)
    ready = commands.add_parser('readiness')
    ready.add_argument('--cell', type=Path, required=True)
    ready.add_argument('--session', type=Path, action='append', default=[])
    ready.add_argument('--stop', type=Path)
    ready.add_argument('--cancel', type=Path)
    ready.add_argument('--other', type=Path, action='append', default=[])
    args = parser.parse_args(argv)
    workspace, evidence = args.workspace.resolve(), args.workspace.resolve().parent / 'log/qualification'
    if args.command == 'judge':
        try:                     # a verdict comes only from the very planner and G4 control code the trial bound
            judging = (scope.planner_identity(), scope.g4_control_identity())
        except scope.PhysicsScopeRefused as exc:
            judging = None
            print(f'the judging checkout has no committed planner or G4 control identity: {exc}')
        if args.kind == 'session':
            found = scope.g4_session_evidence(args.run, tracking=session_tracking)
            passed = found['kind'] == 'STRICT_PASS'
        elif args.kind == 'stop':
            found = scope.g4_stop_after_cycle_evidence(args.run, tracking=session_tracking)
            passed = found['accepted']
        else:
            found = scope.g4_cancel_evidence(args.run)
            passed = found['accepted']
        if judging is None or (found.get('planner'), found.get('control')) != judging:
            passed = False
            print(f"the trial bound planner/G4 control {found.get('planner')} / {found.get('control')}; this checkout is "
                  f'{judging}: not its verifier')
        print(json.dumps({k: v for k, v in found.items() if k not in ('binding', 'facts')}, indent=2, default=str))
        print(f'\nG4 {args.kind.upper()} VERDICT:', 'PASS' if passed else 'FAIL')
        return 0 if passed else 1
    import yaml
    intent = yaml.safe_load((args.cell / 'config/workcell_builder_task_intent.yaml').read_text())
    result = scope.g4_qualification_readiness(
        args.cell, args.session,
        scope.native_identity(workspace / 'install/workcell_builder/share/workcell_builder/commission_execute_build.json'),
        scope.runtime_identity(workspace), scope.g4_perception(workspace, intent), stop_trial=args.stop,
        cancel_trial=args.cancel, trial_roots=[evidence / 'g4/qualification', evidence / 'g4/g4_e1'],
        scan_roots=[evidence], other_runs=args.other, tracking=session_tracking)
    print(json.dumps(result, indent=2, default=str))
    print('\nG4 QUALIFICATION READINESS:', result['status'])
    return 0 if result['status'] == 'QUALIFIED_READY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
