"""Continuous class-routed sorting: lifecycle, object ledger and per-cycle reconciliation (R2.4).

The sorting engine only decides *which* observed object is next and keeps the truth about every object
between cycles. Each cycle is still one run of the existing certified executor (selection, routing, grasp,
collision and placement are the resolver's and MoveIt's): the session passes it the reconciled world, the
one target and the objects already occupying destinations, and counts an object only from that run's
state-verified execution evidence.

Lifecycle: IDLE -> READY -> RUNNING <-> PAUSING/PAUSED, -> STOPPING -> STOPPED; any active state -> FAULT.
Pause and stop take effect at a cycle boundary (robot home, nothing attached); a software stop is never an
emergency stop. Observation sources:

* ``batch_replay`` - one replayed frame is consumed once; later cycles observe the world reconciled from
  execution evidence (completed objects at their verified placed pose), never the old frame again.
* ``continuous`` - every cycle ingests a new live frame (camera or simulator measurement); a frame that
  still shows a processed object at its pick place is rejected as stale instead of re-picking it.
"""
from __future__ import annotations

import copy
import json
import math
import os
import time
from pathlib import Path

STATES = ('IDLE', 'READY', 'RUNNING', 'PAUSING', 'PAUSED', 'STOPPING', 'STOPPED', 'FAULT')
TRANSITIONS = {
    'IDLE': {'READY', 'STOPPING', 'FAULT'},
    'READY': {'RUNNING', 'STOPPING', 'FAULT'},
    'RUNNING': {'PAUSING', 'PAUSED', 'STOPPING', 'FAULT'},
    'PAUSING': {'PAUSED', 'RUNNING', 'STOPPING', 'FAULT'},
    'PAUSED': {'RUNNING', 'STOPPING', 'FAULT'},
    'STOPPING': {'STOPPED', 'FAULT'},
    'STOPPED': set(),
    'FAULT': set(),
}
# Object outcomes. Only COMPLETED is a production count; REJECTED is a failed object.
PENDING, COMPLETED, REJECTED = 'PENDING', 'COMPLETED', 'REJECTED'
SKIPPED_UNROUTED, SKIPPED_DISABLED, NOT_ELIGIBLE = 'SKIPPED_UNROUTED', 'SKIPPED_DISABLED', 'NOT_ELIGIBLE'
WAITING_FULL, REMOVED = 'WAITING_DESTINATION_FULL', 'REMOVED'
# Resolver/planner refusals that another attempt on an unchanged world cannot change.
DETERMINISTIC_REFUSALS = ('PLACE_OBJECT_DOES_NOT_FIT', 'DESTINATION_CONTAINMENT', 'GRASP_CANDIDATE_INVALID',
                          'STRATEGY_TOOL_INCOMPATIBLE', 'REQUIRED_CAPABILITY_UNAVAILABLE', 'TARGET_NOT_ELIGIBLE',
                          'TASK_CONSTRAINT_UNSATISFIED', 'DESTINATION_FULL')
COMPLETION_FLAGS = ('full_cycle_prevalidated', 'prevalidation_left_live_scene_unchanged', 'execution_attempted',
                    'full_cycle_execution_success', 'attach_verified', 'detach_verified', 'home_verified',
                    'destination_verified', 'physical_destination_verified', 'final_collision_valid',
                    'baseline_acm_restored')
EXECUTE_STAGES = ['EXECUTE_APPROACH', 'EXECUTE_GRASP', 'EXECUTE_CLOSE_GRIPPER', 'EXECUTE_LIFT', 'EXECUTE_TRANSFER',
                  'EXECUTE_PLACE', 'EXECUTE_OPEN_GRIPPER', 'EXECUTE_RETREAT', 'EXECUTE_HOME']


class SortingFault(RuntimeError):
    """The session cannot continue safely; the code names why."""

    def __init__(self, code, reason):
        super().__init__(f'{code}: {reason}')
        self.code, self.reason = code, reason


def _rpy_from_quaternion(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def failure_code(text):
    """The leading CODE_NAME of an executor failure text, else ''."""
    head = (text or '').split(':', 1)[0].strip()
    return head if head and head.replace('_', '').isalnum() and head.upper() == head else ''


def verify_completed_cycle(summary, target_id, route_class, new_statuses):
    """Problems with a cycle's evidence of a completed, state-verified pick and place (empty: completed).

    Counts only what the run proved: all nine stages planned and executed with MoveIt success, controller
    terminal success for every new goal, exclusive mock hardware, the target detached once in the world,
    nothing attached, the final pose contained in the routed destination, home reached.
    """
    problems = []
    if summary.get('result') != 'PASS':
        problems.append(f"result {summary.get('result')!r}: {summary.get('failure', '')}")
    problems += [f'{flag} not proven' for flag in COMPLETION_FLAGS if summary.get(flag) is not True]
    if summary.get('selected_object_id') != target_id:
        problems.append(f"executor selected {summary.get('selected_object_id')!r}, not the session target {target_id!r}")
    results = summary.get('execution_results') or []
    if [r.get('stage') for r in results] != EXECUTE_STAGES or any(
            r.get('code') != 1 or r.get('action_status') != 4 for r in results):
        problems.append('not all nine execution stages succeeded')
    if not new_statuses or any(status != 4 for status in new_statuses):
        problems.append(f'controller terminal success missing: {new_statuses}')
    guard = summary.get('fake_hardware_guard') or {}
    if guard.get('real_hardware') is not False or set(guard.get('hardware_classes') or []) != {'mock_components/GenericSystem'}:
        problems.append('exclusive mock hardware not proven')
    final = summary.get('final_planning_scene') or {}
    if (final.get('world_ids') or []).count(target_id) != 1 or final.get('attached_ids'):
        problems.append('target not uniquely detached in the world')
    state = summary.get('final_object_state') or {}
    if state.get('id', target_id) != target_id or len(state.get('pose') or []) != 7:
        problems.append('final object state missing')
    route = ((summary.get('task_intent_resolution') or {}).get('place_resolution') or {}).get('route') or {}
    if route.get('class_id') != route_class:
        problems.append(f"resolved route class {route.get('class_id')!r} differs from the object's class {route_class!r}")
    return problems


class SortingSession:
    def __init__(self, intent, environment, *, session_id, mode='batch_replay', scene=None, clock=time.time):
        from task_intent_v2 import class_routing_rules, routing_job_policy
        if class_routing_rules(intent) is None:
            raise SortingFault('SORTING_REQUIRES_CLASS_ROUTING', 'the saved task has no class routing table')
        if mode not in ('batch_replay', 'continuous'):
            raise SortingFault('SORTING_MODE_UNSUPPORTED', mode)
        self.intent, self.environment, self.session_id, self.mode, self.scene = intent, environment, session_id, mode, scene
        self.policy = routing_job_policy(intent)
        self.clock = clock
        self.state, self.state_reason = 'IDLE', 'created'
        self.objects = {}        # runtime id -> ledger entry
        self.cycles = []
        self.events = []
        self.fault = None
        self.active = None       # {'object': id, 'class_id', 'destination', 'cycle'}
        self.stage = None
        self.frames = []         # provenance of every ingested frame
        self.control_seq = 0

    # -------------------------------------------------------------- lifecycle
    def transition(self, state, reason=''):
        if state not in TRANSITIONS[self.state]:
            raise SortingFault('SORTING_TRANSITION_INVALID', f'{self.state} -> {state}')
        self.events.append({'time': self.clock(), 'from': self.state, 'to': state, 'reason': reason})
        self.state, self.state_reason = state, reason

    def fail(self, code, reason):
        self.fault = {'code': code, 'reason': reason, 'state': self.state, 'active': copy.deepcopy(self.active)}
        if self.state not in ('STOPPED', 'FAULT'):
            self.events.append({'time': self.clock(), 'from': self.state, 'to': 'FAULT', 'reason': f'{code}: {reason}'})
            self.state, self.state_reason = 'FAULT', code

    def command(self, command, *, cycle_active):
        """Operator command: pause, resume or stop. Returns what happened (applied at the safe boundary)."""
        if command == 'pause':
            if self.state == 'RUNNING':
                self.transition('PAUSING' if cycle_active else 'PAUSED', 'operator pause')
                return 'pausing at the cycle boundary' if cycle_active else 'paused'
        elif command == 'resume':
            if self.state in ('PAUSED', 'PAUSING'):
                self.transition('RUNNING', 'operator resume')
                return 'running'
        elif command == 'stop':
            if self.state in ('IDLE', 'READY', 'RUNNING', 'PAUSING', 'PAUSED'):
                self.transition('STOPPING', 'operator stop (software stop at the cycle boundary, not an emergency stop)')
                return 'stopping at the cycle boundary'
        else:
            return f'unknown command {command!r} refused'
        return f'{command} ignored in {self.state}'

    def poll_control(self, path, *, cycle_active):
        """Apply new commands of the control file ({"seq": n, "command": ...}) in order."""
        try:
            data = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return []
        commands = data.get('commands') if isinstance(data, dict) else None
        applied = []
        for item in commands or []:
            if not isinstance(item, dict) or not isinstance(item.get('seq'), int) or item['seq'] <= self.control_seq:
                continue
            self.control_seq = item['seq']
            applied.append({'seq': item['seq'], 'command': item.get('command'),
                            'outcome': self.command(item.get('command'), cycle_active=cycle_active)})
        return applied

    def boundary(self):
        """At a cycle boundary a pending pause becomes PAUSED and a stop STOPPED."""
        if self.state == 'PAUSING':
            self.transition('PAUSED', 'paused at the cycle boundary')
        elif self.state == 'STOPPING':
            self.transition('STOPPED', self.state_reason or 'stopped')

    # -------------------------------------------------------------- observations
    def ingest_frame(self, snapshot, observations, *, source_sha256):
        """Record a frame's objects. A frame showing a processed object back at its pick place is stale."""
        from runtime_pick_inputs import contained, zone
        import perceived_object_grasp_plan as geometry
        if self.mode == 'batch_replay' and self.frames:
            raise SortingFault('BATCH_FRAME_ALREADY_CONSUMED',
                               'a replayed batch frame is consumed once; later cycles use the reconciled world')
        region = zone({'environment': self.environment}, self.intent['pick']['selection']['zone_ref'])
        seen = set()
        for item in observations:
            seen.add(item['id'])
            known = self.objects.get(item['id'])
            if known is None:
                self.objects[item['id']] = {
                    'id': item['id'], 'object_id': item.get('object_id'), 'class_id': item.get('class_id'),
                    'confidence': item.get('confidence'), 'dimensions': list(item['dimensions']),
                    'initial_pose': list(item['pose']), 'pose': list(item['pose']), 'status': PENDING,
                    'attempts': [], 'destination': None, 'first_frame': len(self.frames)}
                continue
            if known['status'] == COMPLETED and contained(item, region, geometry):
                raise SortingFault('PROCESSED_OBJECT_STILL_OBSERVED',
                                   f"{item['id']} was placed in {known['destination']} but this frame still shows it in "
                                   'the pick zone; the frame is stale or the placement did not happen')
            if known['class_id'] != item.get('class_id'):
                raise SortingFault('OBJECT_IDENTITY_CONFLICT',
                                   f"{item['id']} changed class {known['class_id']!r} -> {item.get('class_id')!r}")
            known['pose'] = list(item['pose'])
        for oid, known in self.objects.items():
            if self.mode == 'continuous' and oid not in seen and known['status'] in (PENDING, WAITING_FULL):
                known['status'] = REMOVED
        self.frames.append({'source_sha256': source_sha256, 'objects': len(observations), 'time': self.clock(),
                            'source': copy.deepcopy(snapshot.get('source'))})

    def world_snapshot(self, base_snapshot):
        """detected_objects/v1 of the reconciled world for the next cycle (batch replay), with its provenance."""
        if not self.objects:
            raise SortingFault('SORTING_NO_OBJECTS', 'no frame has been ingested')
        objects = []
        for entry in sorted(self.objects.values(), key=lambda e: e['id']):
            if entry['status'] == REMOVED:
                continue
            pose = entry['pose']
            objects.append({'object_id': entry['object_id'], 'class_id': entry['class_id'],
                            'confidence': entry['confidence'], 'age_seconds': 0.0,
                            'pose': {'frame_id': 'world', 'xyz': list(pose[:3]), 'rpy': _rpy_from_quaternion(pose[3:])},
                            'dimensions': list(entry['dimensions'])})
        source = copy.deepcopy(base_snapshot.get('source') or {})
        source['mode'] = 'replayed_snapshot'
        source['sorting_session'] = {
            'session_id': self.session_id, 'cycle': len(self.cycles) + 1,
            'reconciled_from': 'the initial replayed frame and each completed cycle\'s verified final object state',
            'completed_object_ids': sorted(e['id'] for e in self.objects.values() if e['status'] == COMPLETED),
            'frames': copy.deepcopy(self.frames)}
        return {'schema_version': 'detected_objects/v1', 'source': source, 'objects': objects}

    def current_observations(self, now):
        """The ledger as normalized observations (resolver input) at time `now`."""
        return [{'id': e['id'], 'object_id': e['object_id'], 'class_id': e['class_id'], 'confidence': e['confidence'],
                 'timestamp': now, 'frame_id': 'world', 'shape': 'BOX', 'pose': list(e['pose']),
                 'dimensions': list(e['dimensions'])}
                for e in self.objects.values() if e['status'] != REMOVED]

    # -------------------------------------------------------------- selection
    def next_target(self, now=None):
        """The next object to pick, deterministically, or None when nothing more can be sorted now.

        Uses the resolver's own eligibility, routes, priorities and order. Objects of an unrouted or disabled
        class are skipped per the unrouted policy (stop_session faults); an object that cannot fit its route is
        rejected before planning; one whose destination has no free place waits. Completed and rejected
        objects are never selected again.
        """
        from runtime_pick_inputs import contained, zone
        import perceived_object_grasp_plan as geometry
        from task_intent_resolver import (_routed_placement_fit, class_routes, explain_observations,
                                          select_observations)
        now = self.clock() if now is None else now
        observations = self.current_observations(now)
        region = zone({'environment': self.environment}, self.intent['pick']['selection']['zone_ref'])
        for decision in explain_observations(self.intent, self.environment, observations, now):
            entry = self.objects[decision['id']]
            if entry['status'] in (COMPLETED, REJECTED):
                continue
            in_zone = contained(next(o for o in observations if o['id'] == entry['id']), region, geometry)
            if decision['reason_code'] in ('CLASS_NOT_ROUTED', 'CLASS_ROUTE_DISABLED') and in_zone:
                if decision['reason_code'] == 'CLASS_NOT_ROUTED' and self.policy['unrouted_policy'] == 'stop_session':
                    raise SortingFault('UNROUTED_OBJECT_PRESENT',
                                       f"{entry['id']} (class {entry['class_id']!r}) has no routing rule and the "
                                       'unrouted policy is stop_session')
                entry['status'] = SKIPPED_UNROUTED if decision['reason_code'] == 'CLASS_NOT_ROUTED' else SKIPPED_DISABLED
                entry['reason'] = decision['reason']
            elif not decision['accepted'] and entry['status'] == PENDING:
                entry['status'], entry['reason'] = NOT_ELIGIBLE, f"{decision['reason_code']}: {decision['reason']}"
            elif decision['accepted'] and entry['status'] == NOT_ELIGIBLE:
                entry['status'] = PENDING        # e.g. it entered the pick zone in a later live frame
        routes = class_routes(self.intent, self.environment)
        clearance = self.intent['place']['placement']['clearance_m']
        occupied = self.occupied()
        for item in select_observations(self.intent, self.environment, observations, now):
            entry = self.objects[item['id']]
            if entry['status'] not in (PENDING, WAITING_FULL):
                continue
            fitting, unfit, places, full = _routed_placement_fit([item], routes, clearance, None, self.environment,
                                                                 [o for o in occupied if o['id'] != item['id']])
            if item['id'] in full:
                entry['status'], entry['reason'] = WAITING_FULL, unfit[item['id']][0]
                continue
            if not fitting:
                self._reject(entry, 'PLACE_OBJECT_DOES_NOT_FIT', unfit[item['id']][0], planned=False)
                continue
            entry['status'] = PENDING
            route = routes[item['class_id']]
            return {'id': item['id'], 'class_id': item['class_id'], 'rule_id': route['rule_id'],
                    'destination': route['region_ref'], 'asset': route['asset_ref'],
                    'attempt': len(entry['attempts']) + 1}
        return None

    def occupied(self):
        """Every object physically in the cell, as the resolver's occupancy input."""
        return [{'id': e['id'], 'pose': list(e['pose']), 'dimensions': list(e['dimensions'])}
                for e in self.objects.values() if e['status'] != REMOVED]

    # -------------------------------------------------------------- outcomes
    def begin_cycle(self, target):
        self.active = dict(target, cycle=len(self.cycles) + 1, started=self.clock())
        self.stage = None

    def _reject(self, entry, code, reason, *, planned):
        entry['status'], entry['reason'] = REJECTED, f'{code}: {reason}'
        entry['rejected_before_planning'] = not planned

    def record_cycle(self, summary, returncode, new_statuses, evidence_dir=None):
        """Classify the active cycle from its executor evidence; returns the outcome. Faults raise."""
        target = self.active
        entry = self.objects[target['id']]
        attempt = {'cycle': target['cycle'], 'attempt': target['attempt'], 'returncode': returncode,
                   'evidence': str(evidence_dir) if evidence_dir else None,
                   'failure': summary.get('failure'), 'failed_stage': summary.get('failed_stage'),
                   'execution_attempted': summary.get('execution_attempted'),
                   'duration_s': self.clock() - target['started']}
        entry['attempts'].append(attempt)
        outcome = None
        if summary.get('execution_attempted') is True:
            problems = verify_completed_cycle(summary, target['id'], entry['class_id'], new_statuses)
            if problems:
                # Motion started and the end state is not proven: the world is uncertain, never guessed.
                self.active = None
                raise SortingFault('CYCLE_EXECUTION_UNVERIFIED', f"{target['id']}: " + '; '.join(problems))
            final = summary['final_object_state']
            place = summary['task_intent_resolution']['place_resolution']
            entry.update(status=COMPLETED, pose=list(final['pose']), destination=place['route']['region_ref'],
                         completed_cycle=target['cycle'], placement_error_m=summary.get('placement_error_m'))
            attempt['outcome'] = outcome = COMPLETED
        else:
            if returncode == 0:
                self.active = None
                raise SortingFault('CYCLE_RESULT_INCONSISTENT', f"{target['id']}: executor exited 0 without execution")
            code = failure_code(summary.get('failure'))
            deterministic = any(code.startswith(prefix) for prefix in DETERMINISTIC_REFUSALS)
            if deterministic or len(entry['attempts']) >= self.policy['max_attempts_per_object']:
                self._reject(entry, code or 'CYCLE_FAILED', summary.get('failure') or 'executor failed', planned=True)
                attempt['outcome'] = outcome = REJECTED
            else:
                attempt['outcome'] = outcome = 'RETRY'
        self.cycles.append({'cycle': target['cycle'], 'object': target['id'], 'class_id': entry['class_id'],
                            'destination': target['destination'], 'outcome': outcome,
                            'failure': summary.get('failure'), 'evidence': attempt['evidence'],
                            'duration_s': attempt['duration_s']})
        self.active, self.stage = None, None
        return outcome

    # -------------------------------------------------------------- reporting
    def counts(self):
        statuses = [e['status'] for e in self.objects.values()]
        per_class, per_destination = {}, {}
        for e in self.objects.values():
            row = per_class.setdefault(e['class_id'], {'completed': 0, 'failed': 0, 'skipped': 0, 'waiting': 0, 'pending': 0})
            key = {COMPLETED: 'completed', REJECTED: 'failed', SKIPPED_UNROUTED: 'skipped', SKIPPED_DISABLED: 'skipped',
                   WAITING_FULL: 'waiting', PENDING: 'pending'}.get(e['status'])
            if key:
                row[key] += 1
            if e['status'] == COMPLETED:
                per_destination[e['destination']] = per_destination.get(e['destination'], 0) + 1
        return {'completed': statuses.count(COMPLETED), 'failed': statuses.count(REJECTED),
                'skipped': statuses.count(SKIPPED_UNROUTED) + statuses.count(SKIPPED_DISABLED),
                'waiting_destination_full': statuses.count(WAITING_FULL), 'pending': statuses.count(PENDING),
                'not_eligible': statuses.count(NOT_ELIGIBLE), 'removed': statuses.count(REMOVED),
                'cycles': len(self.cycles), 'per_class': per_class, 'per_destination': per_destination}

    def status(self):
        return {'schema': 'workcell_sorting_session/v1', 'session_id': self.session_id, 'mode': self.mode,
                'state': self.state, 'state_reason': self.state_reason, 'fault': copy.deepcopy(self.fault),
                'active': copy.deepcopy(self.active), 'stage': self.stage, 'counts': self.counts(),
                'unrouted_policy': self.policy['unrouted_policy'],
                'max_attempts_per_object': self.policy['max_attempts_per_object'],
                'execution': 'MoveIt fake hardware (mock controllers): planning and kinematic execution only; '
                             'no physics, contact or retention evidence; real robot locked',
                'stop_semantics': 'Pause and Stop are software requests honoured at the cycle boundary; not an emergency stop'}

    def report(self):
        return dict(self.status(), objects=sorted(copy.deepcopy(list(self.objects.values())), key=lambda e: e['id']),
                    cycles=copy.deepcopy(self.cycles), events=copy.deepcopy(self.events), frames=copy.deepcopy(self.frames))


def write_json_atomic(path, payload):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n')
    os.replace(tmp, path)
