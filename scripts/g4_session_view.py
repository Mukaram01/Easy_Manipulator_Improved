#!/usr/bin/env python3
"""Operator view of one G4 physics sorting session (run_g4_sorting.py output), read from its evidence files only.

Workcell Builder (Plan / Simulate) and the Operator HMI show what this prints, while the session runs and after it
ends. Nothing is derived beyond the files:
  objects   the sorting ledger's own state (sorting_session.json): EPD's bound class, status, reason, pose;
  cycles    per cycle, the rendered frame, EPD's classes bound to the measured objects (classify/observations.yaml)
            and the executor's plan-only resolve (resolve/summary.json); the runner's report adds each cycle's
            authorisation decision and outcome once the session has ended;
  counts    the ledger's counts (a plan-only cycle is never an attempt or a count);
  motion    granted cycle authorisations, executor runs and the controller goals in the launch log, never assumed.

usage: g4_session_view.py <run dir>
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

SCHEMA = 'workcell_g4_session_view/v1'
REPORT = 'g4-sorting-report.json'


def _json(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _yaml(path: Path) -> dict[str, Any] | None:
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return None
    return data if isinstance(data, dict) else None


def _frame(directory: Path) -> dict[str, Any] | None:
    frame = _yaml(directory / 'frame/frame.yaml')
    if frame is None:
        return None
    source = frame.get('source') or {}
    return {key: source.get(key) for key in ('producer', 'provenance', 'camera_id', 'capture_wall_ns', 'world_revision')}


def _classified(directory: Path) -> list[dict[str, Any]] | None:
    observations = _yaml(directory / 'classify/observations.yaml')
    if observations is None:
        return None
    return [{'object_id': o.get('object_id'), 'class_id': o.get('class_id'), 'confidence': o.get('confidence'),
             'xyz': (o.get('pose') or {}).get('xyz'), 'dimensions': o.get('dimensions')}
            for o in observations.get('objects') or []]


def _plan(directory: Path) -> dict[str, Any] | None:
    """The executor's plan-only resolve of this cycle's target, as its summary records it."""
    summary = _json(directory / 'resolve/summary.json')
    if summary is None:
        return {'state': 'PLANNING'} if (directory / 'resolve').is_dir() else None
    resolution = (summary.get('task_intent_resolution') or {}).get('place_resolution') or {}
    route = resolution.get('route') or {}
    return {'state': 'PLANNED' if summary.get('full_cycle_plan_success') else 'REFUSED',
            'result': summary.get('result'), 'target': summary.get('batch_target_object_id') or summary.get('selected_object_id'),
            'class_id': route.get('class_id'), 'rule_id': route.get('rule_id'), 'destination': route.get('region_ref'),
            'bin': route.get('asset_ref'), 'place_xyz': (route.get('world_pose') or {}).get('xyz_m'),
            'grasp': (summary.get('selected_extraction_intent') or {}).get('candidate_id'),
            'stages': [p.get('stage') for p in summary.get('plan_metadata') or []],
            'failure': summary.get('failure'), 'execution_attempted': summary.get('execution_attempted')}


def _cycle(directory: Path, recorded: dict[str, Any] | None) -> dict[str, Any]:
    steps = (recorded or {}).get('steps') or {}
    authorise = steps.get('authorise')
    return {'cycle': int(directory.name), 'frame': _frame(directory), 'objects': _classified(directory),
            'plan': _plan(directory),
            'authorisation': None if authorise is None else {'granted': bool(authorise.get('granted')),
                                                              'refusal': authorise.get('refusal')},
            'executed': (directory / 'execute/summary.json').is_file(),
            'outcome': (recorded or {}).get('outcome')}


def view(run: Path) -> dict[str, Any]:
    from physics_scope_qualification import G4_GOAL_LOG
    run = Path(run)
    report = _json(run / REPORT)
    session = _json(run / 'session/g4-session.json') or {}
    ledger = _json(run / 'sorting_session.json') or {}
    recorded = {int(c['cycle']): c for c in (report or {}).get('cycles') or [] if isinstance(c, dict) and 'cycle' in c}
    cycles = [_cycle(d, recorded.get(int(d.name))) for d in sorted((run / 'session/cycles').glob('[0-9][0-9][0-9]'))]
    try:
        goals = len(G4_GOAL_LOG.findall((run / 'session/launch.log').read_text(errors='replace')))
    except OSError:
        goals = 0
    shutdown = (report or {}).get('shutdown') or {}
    planned = {}                                                     # object id -> its latest plan-only resolve
    for c in cycles:
        if c['plan'] and c['plan'].get('target'):
            planned[c['plan']['target']] = {'cycle': c['cycle'], **{key: c['plan'].get(key) for key in (
                'state', 'destination', 'bin', 'grasp', 'place_xyz')}}
    return {
        'schema': SCHEMA, 'run': str(run), 'finished': report is not None,
        'session_state': session.get('state'),                       # G4_PLAN_ONLY or G4_AUTHORISED_TRIAL
        'scope': (report or {}).get('scope'),
        'status': (report or {}).get('status'), 'result': (report or {}).get('result'),
        'stop': (report or {}).get('stop'), 'failure': (report or {}).get('failure'),
        'duration_s': (report or {}).get('duration_s'),
        'shutdown': None if report is None else {'clean': shutdown.get('clean'),
                                                 'surviving': len(shutdown.get('surviving_owned_processes') or [])},
        'ledger': {key: ledger.get(key) for key in ('state', 'state_reason', 'fault', 'counts')},
        'objects': [{**{key: o.get(key) for key in ('id', 'object_id', 'class_id', 'confidence', 'status', 'reason',
                                                    'destination', 'pose', 'dimensions')}, 'plan': planned.get(o.get('id'))}
                    for o in ledger.get('objects') or []],
        'cycles': cycles,
        'motion': {'authorised_cycles': sum(1 for _ in run.glob('session/cycles/*/g4-cycle.json')),
                   'executed_cycles': sum(1 for c in cycles if c['executed']), 'controller_goals': goals},
    }


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    print(json.dumps(view(Path(args[0])), default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
