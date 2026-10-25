#!/usr/bin/env python3
"""Measured guards used only by the existing executor's explicit commissioning mode.

No trajectory producer, simulator mutation, or autonomous recovery lives here.
"""
import copy
import ctypes
import hashlib
import json
import math
from pathlib import Path
import time
import threading
import measurement_clock
import queue
from enum import Enum
import xml.etree.ElementTree as ET
from perceived_object_grasp_plan import compose_pose,inverse_pose,quaternion_from_rpy,rotate_vector


def angle(a,b):
    return 2*math.acos(min(1.,abs(sum(x*y for x,y in zip(a[3:],b[3:])))))


def box_motion_bound(pose,reference,size):
    """Bound displacement of every point of a rigid BOX by its eight corners.

    The displacement is affine in body coordinates and its norm is convex,
    so its maximum over the box occurs at a vertex. Include a conservative
    floating-point allowance for normalization, rotation and subtraction.
    """
    values=(*pose,*reference,*size)
    if len(pose)!=7 or len(reference)!=7 or len(size)!=3 or not all(math.isfinite(v) for v in values) or min(size)<=0:
        raise RuntimeError('invalid BOX motion geometry')
    def quaternion(p):
        # Match the measured-pose domain even for direct geometry checks.
        if abs(sum(v*v for v in p[3:])-1.)>.00001:
            raise RuntimeError('invalid BOX motion orientation')
        norm=math.hypot(*p[3:])
        return [v/norm for v in p[3:]]
    q=quaternion(pose);r=quaternion(reference)
    translation=[a-b for a,b in zip(pose[:3],reference[:3])]
    maximum=0.
    for x in (-size[0]/2,size[0]/2):
        for y in (-size[1]/2,size[1]/2):
            for z in (-size[2]/2,size[2]/2):
                a=rotate_vector(q,[x,y,z]);b=rotate_vector(r,[x,y,z])
                maximum=max(maximum,math.hypot(*(math.fsum((t,u,-v)) for t,u,v in zip(translation,a,b))))
    # CPython 3.10 binary64 / nearest-even: with u=2**-53 and
    # M=max(1,abs(inputs)), near-unit normalization, both Hamilton-product
    # rotations, translation, fsum (including double rounding) and hypot
    # have combined forward error <128*u*M. Corner norm <=M;
    # 256*ulp(M)>=256*u*M. nextafter covers the final addition rounding.
    # https://docs.python.org/3.10/library/math.html (hypot and fsum).
    rounding=256*math.ulp(max(1.,*(abs(v) for v in values)))
    return math.nextafter(maximum+rounding,math.inf)


class StaleMeasurement(RuntimeError):
    """Structurally valid telemetry that is too old to use."""


def validate_sample(s,receipt,now,previous=None):
    """now is measurement_clock seconds (CLOCK_MONOTONIC_RAW), never wall time.

    Age and ordering use the simulator's monotonic_raw_ns (see measurement_clock):
    a stepped wall clock made a 1 ms-old sample stale mid-approach (LENLEG
    2026-10-23); a backward step would make a stale one look fresh. wall_ns
    stays required evidence."""
    if (s.get('run_id')!=receipt['run_id'] or s.get('pid')!=receipt['pid'] or s.get('error') or
        type(s.get('wall_ns')) is not int or s['wall_ns']<=0 or
        type(s.get('monotonic_raw_ns')) is not int or s['monotonic_raw_ns']<=0 or now<s['monotonic_raw_ns']/1e9 or
        not s.get('poses') or not s.get('joints') or
        not s.get('collisions') or not isinstance(s.get('contacts'),list) or
        not isinstance(s.get('sim_ns'),int) or type(s.get('iteration')) is not int or (previous is not None and s['iteration']!=previous+1)):
        raise RuntimeError('SIMULATOR_TELEMETRY_INVALID: stale, missing, contradictory or skipped measurement')
    for p in s['poses'].values():
        if len(p)!=7 or not all(math.isfinite(x) for x in p) or abs(sum(x*x for x in p[3:])-1)>.00001:
            raise RuntimeError('invalid measured pose')
    if any(len(v)!=2 or not all(math.isfinite(x) for x in v) for v in s['joints'].values()):
        raise RuntimeError('invalid measured joint')
    # Historical geometry-only fixtures do not declare a measurement source.
    # Live acquisition below requires the declaration, so it cannot take this path.
    if 'measurement_pose_source' in receipt:
        from simulator_backend import PHYSICS_POSE_METHOD
        source=s.get('pose_source');entities=s.get('pose_entities')
        if (receipt['measurement_pose_source']!=PHYSICS_POSE_METHOD or not isinstance(source,dict) or
            source.get('method')!=PHYSICS_POSE_METHOD or source.get('frame')!='world' or
            source.get('read_only') is not True or not isinstance(source.get('gazebo_version'),str) or
            not source['gazebo_version'].strip() or
            any(type(source.get(query)) is not int or source[query]!=s[field]
                for query,field in [('query_iteration','iteration'),('query_sim_ns','sim_ns')])):
            raise RuntimeError('authoritative physics pose source missing, stale or mismatched')
        if not isinstance(entities,dict) or set(entities)!=set(s['poses']):
            raise RuntimeError('authoritative physics pose entity coverage mismatch')
        for name,entity in entities.items():
            if (not isinstance(name,str) or not name.startswith(receipt['world']+'::') or
                not isinstance(entity,dict) or entity.get('kind') not in ('model','link') or
                any(type(entity.get(key)) is not int or entity[key]<=0
                    for key in ('entity','link_entity','query_entity')) or
                (entity['kind']=='link' and entity['entity']!=entity['link_entity'])):
                raise RuntimeError('authoritative physics pose entity identity mismatch')
        if len({e['entity'] for e in entities.values()})!=len(entities):
            raise RuntimeError('authoritative physics pose entity identity is ambiguous')
        links={e['entity']:(name,e) for name,e in entities.items() if e['kind']=='link'}
        queries={e['query_entity'] for _,e in links.values()}
        if len(queries)!=len(links) or queries & {e['entity'] for e in entities.values()}:
            raise RuntimeError('authoritative physics query entity identity is ambiguous')
        for name,entity in entities.items():
            if entity['kind']=='model':
                link=links.get(entity['link_entity'])
                if (link is None or not link[0].startswith(name+'::') or
                    link[1]['query_entity']!=entity['query_entity']):
                    raise RuntimeError('authoritative physics model/link identity mismatch')
    # Check age last: an aged message with another defect must still latch that
    # defect. A structurally valid rejected pre-motion sample may be replaced
    # by a later fresh sample, never re-stamped or used for any guard.
    if now-s['monotonic_raw_ns']/1e9>.25:
        raise StaleMeasurement('SIMULATOR_TELEMETRY_INVALID: stale measurement')


# The certificate's support contact tolerance (support_contact_policy.hpp
# support_contact_tolerance_m): the deepest certified support penetration.
SUPPORT_CONTACT_TOLERANCE_M=1e-4


class Separation:
    def __init__(self,origin,floor):
        self.origin=origin;self.floor=floor;self.expired=False;self.height=0.
    def check(self,pose,bottom,support_contact):
        if self.expired:
            if support_contact:raise RuntimeError('support exception expired: recontact')
            return
        motion=[a-b for a,b in zip(pose[:3],self.origin[:3])]
        # Physics settles by ~1e-8 m while the part is held at rest, so descent is
        # measured against the running maximum height and bounded by the certified
        # support tolerance; the bound never accumulates sample to sample.
        if (math.hypot(*motion[:2])>.0025 or motion[2]<self.height-SUPPORT_CONTACT_TOLERANCE_M or motion[2]>.01 or angle(pose,self.origin)>.01):
            raise RuntimeError('initial support separation leaves certified corridor')
        self.height=max(self.height,motion[2])
        if not support_contact and bottom>self.floor+.0001:self.expired=True


class GraspRetentionLoss(RuntimeError):
    """Physical grasp contract violation, distinct from transport/planning failures."""
    def __init__(self, missing_contacts, translation_error_m, rotation_error_rad):
        super().__init__('measured grasp retention lost: contact/slip')
        self.details=dict(missing_contacts=sorted(missing_contacts),
            translation_error_m=translation_error_m,rotation_error_rad=rotation_error_rad)
        self.sample=None


class RecoveryState(str, Enum):
    CANCEL_AND_STOP='CANCEL_AND_STOP'
    FAILURE_RECONCILE='FAILURE_RECONCILE'
    QUALIFY_RETREAT='QUALIFY_RETREAT'
    BLOCKED='RECOVERY_BLOCKED'


class RetentionRecovery:
    """Terminal recovery boundary; no motion or retry can be authorized here.

    A continuation needs a qualified contact-exit policy and a new capture/
    resolution epoch. Historical summary records are evidence, never plan inputs.
    """
    def __init__(self, cycle, summary, failure, *, during_action):
        self.state=RecoveryState.CANCEL_AND_STOP
        self.during_action=during_action
        # Revoke execution before cancellation or any fallible scene operation.
        cycle.clear()
        cycle.update(full_cycle_prevalidated=False,invalidated_by='GRASP_RETENTION_LOSS')
        summary['full_cycle_prevalidated']=False
        self.evidence=dict(state=self.state.value,transitions=[self.state.value],
            reason='GRASP_RETENTION_LOSS',details=copy.deepcopy(failure.details),
            rejected_sample=copy.deepcopy(failure.sample),requires_owned_cancel=during_action,
            invalidated=['candidate','grasp_transform','trajectories','motion_start_state',
                         'observation','resolution'],retry_count=0,retry_limit=1,
            previous_attempt={key:copy.deepcopy(summary.get(key)) for key in (
                'selected_object_id','selected_grasp_index','selected_extraction_intent',
                'resolution_sha256','observation_sha256','execution_attempt')})
        summary['grasp_recovery']=self.evidence

    def _transition(self, expected, target):
        if self.state!=expected:raise RuntimeError('RECOVERY_STATE_ORDER: '+self.state.value)
        self.state=target;self.evidence['state']=target.value
        self.evidence['transitions'].append(target.value)

    def confirm_stop(self, sample, cancellation_confirmed):
        if self.during_action and not cancellation_confirmed:
            raise RuntimeError('CANCELLATION_UNCONFIRMED: recovery requires owned goal terminal and stop')
        if not sample:raise RuntimeError('RECOVERY_STOP_UNCONFIRMED')
        failed=self.evidence['rejected_sample']
        if failed and (any(sample.get(k)!=failed.get(k) for k in ('run_id','pid')) or
                       any(sample[k]<=failed[k] for k in ('iteration','sim_ns','monotonic_raw_ns'))):
            raise RuntimeError('RECOVERY_STOP_PRECEDES_LOSS')
        self._transition(RecoveryState.CANCEL_AND_STOP,RecoveryState.FAILURE_RECONCILE)
        self.evidence['stopped_sample']={k:sample[k] for k in ('iteration','sim_ns','wall_ns','monotonic_raw_ns')}

    def reconciled(self, evidence):
        if evidence['attached_ids'] or not evidence['acm_restored'] or not evidence['measured_geometry_matches']:
            raise RuntimeError('RECOVERY_RECONCILIATION_UNCONFIRMED')
        self._transition(RecoveryState.FAILURE_RECONCILE,RecoveryState.QUALIFY_RETREAT)
        self.evidence['reconciliation']=copy.deepcopy(evidence)

    def qualify_retreat(self, valid, physical_contacts):
        if self.state!=RecoveryState.QUALIFY_RETREAT:raise RuntimeError('RECOVERY_STATE_ORDER: '+self.state.value)
        self.evidence['retreat_qualification']=dict(current_state_valid=bool(valid),
            physical_robot_contacts=copy.deepcopy(physical_contacts),qualified=False,
            reason='No qualified generic withdrawal policy from a detached contact state')
        code=('RECOVERY_START_STATE_IN_CONTACT' if physical_contacts else
              'RECOVERY_CURRENT_STATE_INVALID' if not valid else 'RECOVERY_RETREAT_POLICY_UNQUALIFIED')
        self.block(code)

    def block(self, code, detail=None):
        if self.state==RecoveryState.BLOCKED:return
        self._transition(self.state,RecoveryState.BLOCKED)
        self.evidence.update(failure_code=code,failure_detail=detail)


# Hold reserve past the measured contact closure (servo target 0.1 x reserve x
# 100 Hz). Chosen experimentally; see grasp_hold evidence in each executor summary.
GRASP_HOLD_RESERVE_RAD = 0.002


def grasp_hold_target(measured, reserve, certified_close, open_position):
    """Hold command for a closed grasp: measured contact closure plus a reserve.

    The CLOSE command (certified first contact + one closure step) leaves the
    stalled finger servo a standing ~0.01 rad error; at the servo's 0.1 rad/s
    target it advanced 0.1 mrad whenever the pad manifold regenerated and jolted
    the part about the pinch axis (LENLEG 2026-10-24). The hold never exceeds the
    certified closure, so no penetration or collision beyond the plan is commanded.
    """
    values=(measured,reserve,certified_close,open_position)
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
        raise RuntimeError('GRASP_HOLD_INPUT_INVALID')
    if reserve<=0 or measured<=open_position+.001 or certified_close<measured:
        raise RuntimeError('GRASP_HOLD_REQUIRES_MEASURED_CLOSURE_WITHIN_CERTIFIED_RANGE')
    return min(measured+reserve,certified_close)


def grasp_load_evidence(sample, leader, touch_links, target_model):
    """Measured finger positions, pad contacts and joint loads of one sample.

    Loads come from the simulator's transmitted joint wrenches: the fingertip
    joint force is the pad's contact load, axis torques are actuator/constraint
    efforts. Evidence only; retention acceptance stays with the contact guard.
    """
    joints={k.split('::')[-1]:v for k,v in sample.get('joints',{}).items()}
    wrenches={k.split('::')[-1]:v for k,v in (sample.get('joint_wrenches') or {}).items()}
    gripper={n:v for n,v in joints.items() if n.startswith('gripper_')}
    if leader not in gripper or not wrenches:
        raise RuntimeError('GRASP_LOAD_EVIDENCE_MISSING')
    pads={}
    for contact in sample.get('contacts',[]):
        bodies=(contact['a'],contact['b'])
        if not any(f'::{target_model}::' in body for body in bodies):continue
        for link in touch_links:
            if any(f'::{link}::' in body for body in bodies):
                pads[link]=pads.get(link,0)+len(contact['points'])
    return dict(iteration=sample['iteration'],sim_ns=sample['sim_ns'],monotonic_raw_ns=sample['monotonic_raw_ns'],
        leader_position_rad=gripper[leader][0],leader_velocity_rad_s=gripper[leader][1],
        joint_positions_rad={n:v[0] for n,v in sorted(gripper.items())},
        pad_contact_points={link:pads.get(link,0) for link in sorted(touch_links)},
        fingertip_joint_force_n={n:math.sqrt(sum(x*x for x in w[:3])) for n,w in sorted(wrenches.items())
                                 if n.startswith('gripper_') and n.endswith('finger_tip_joint')},
        joint_axis_torque_nm={n:w[6] for n,w in sorted(wrenches.items()) if n.startswith('gripper_')})


RETENTION_TRANSLATION_LIMIT_M=.002
RETENTION_ROTATION_LIMIT_RAD=.01
RETENTION_POLICIES=('strict','record')


class HeldObject:
    """Measured part-in-tool reference and the Stage-A retention limits.

    ``strict`` (qualification) raises on any exceedance. ``record`` (functional
    validation) keeps the unchanged limits but records translation/rotation
    exceedances as task-quality findings and continues; lost finger contact is a
    physical drop and still raises under either policy.
    """
    def __init__(self,tool,obj,fingers,required,closure,open_position,policy='strict'):
        self.required=set(required)
        if len(self.required)<2 or not self.required.issubset(fingers) or closure<=open_position+.001:
            raise RuntimeError('closure lacks measured opposing finger/object contact')
        if policy not in RETENTION_POLICIES:raise RuntimeError('unknown retention policy')
        self.relative=compose_pose(inverse_pose(tool),obj);self.policy=policy
        self.findings=[];self.peak=dict(translation_m=0.,rotation_rad=0.,translation_phase=None,rotation_phase=None)
    def check(self,tool,obj,fingers,phase=None,sample=None):
        actual=compose_pose(inverse_pose(tool),obj)
        translation=math.dist(actual[:3],self.relative[:3]);rotation=angle(actual,self.relative)
        if translation>self.peak['translation_m']:self.peak.update(translation_m=translation,translation_phase=phase)
        if rotation>self.peak['rotation_rad']:self.peak.update(rotation_m=None,rotation_rad=rotation,rotation_phase=phase)
        missing=self.required-set(fingers)
        exceeded=translation>RETENTION_TRANSLATION_LIMIT_M or rotation>RETENTION_ROTATION_LIMIT_RAD
        if missing or (exceeded and self.policy=='strict'):
            raise GraspRetentionLoss(missing,translation,rotation)
        if exceeded:
            kind='translation' if translation>RETENTION_TRANSLATION_LIMIT_M else 'rotation'
            if not any(f['kind']==kind and f['phase']==phase for f in self.findings):
                self.findings.append(dict(kind=kind,phase=phase,translation_error_m=translation,rotation_error_rad=rotation,
                    limit=RETENTION_TRANSLATION_LIMIT_M if kind=='translation' else RETENTION_ROTATION_LIMIT_RAD,
                    sim_ns=(sample or {}).get('sim_ns'),iteration=(sample or {}).get('iteration')))
    def evidence(self):
        peak={k:v for k,v in self.peak.items() if k!='rotation_m'}
        return dict(policy=self.policy,limits=dict(translation_m=RETENTION_TRANSLATION_LIMIT_M,rotation_rad=RETENTION_ROTATION_LIMIT_RAD),
                    peak=peak,findings=copy.deepcopy(self.findings),strict_compliant=not self.findings)


def cancel_owned(cancel,terminal,stopped,revoke,reconcile):
    """Revocation is unconditional; reconciliation requires confirmed stationary state."""
    try:
        accepted=cancel();ended=terminal();stationary=stopped()
        if not (accepted and ended and stationary):raise RuntimeError('CANCELLATION_UNCONFIRMED: terminal state or measured stop missing')
    finally:revoke()
    reconcile()


def _joint_measurement(sample,joints,names):
    if not names or not set(names).issubset(joints):
        raise RuntimeError('missing commanded joint measurement')
    selected={name:list(joints[name]) for name in sorted(names)}
    if any(len(value)!=2 or not all(math.isfinite(x) for x in value) for value in selected.values()):
        raise RuntimeError('invalid commanded joint measurement')
    return dict(iteration=sample['iteration'],sim_ns=sample['sim_ns'],wall_ns=sample['wall_ns'],
                monotonic_raw_ns=sample['monotonic_raw_ns'],joints=selected)


class CancellationMotion:
    """Evidence of actual commanded-arm movement after owned goal acceptance."""
    def __init__(self,goal_uuid,sample,joints,names,accepted_monotonic_raw_ns=None):
        self.names=set(names)
        self.accepted_monotonic_raw_ns=sample['monotonic_raw_ns'] if accepted_monotonic_raw_ns is None else accepted_monotonic_raw_ns
        self.evidence=dict(goal_uuid=goal_uuid,accepted=_joint_measurement(sample,joints,self.names),movement=None)
    def observe(self,sample,joints):
        measured=_joint_measurement(sample,joints,self.names)
        start=self.evidence['accepted']
        if (measured['iteration']<=start['iteration'] or measured['sim_ns']<=start['sim_ns'] or
                measured['monotonic_raw_ns']<=self.accepted_monotonic_raw_ns):return False
        deltas={name:abs(value[0]-start['joints'][name][0]) for name,value in measured['joints'].items()}
        # The existing 1 mrad state comparison and 2 mrad/s stop thresholds
        # distinguish motion from settled numerical jitter; neither is relaxed.
        if any(deltas[name]>.001 and abs(value[1])>.002 for name,value in measured['joints'].items()):
            measured.update(max_displacement_rad=max(deltas.values()),max_velocity_rad_s=max(abs(v[1]) for v in measured['joints'].values()))
            if self.evidence['movement'] is None:self.evidence['movement']=measured
        return self.evidence['movement'] is not None


def cancel_response_matches(owned_uuid,response):
    return bool(response is not None and response.return_code==0 and any(
        list(info.goal_id.uuid)==list(owned_uuid) for info in response.goals_canceling))


class StopWindow:
    """Retain every complete sample in the existing 300 ms stationary window."""
    def __init__(self,names):
        self.names=set(names);self.previous=None;self.samples=[];self._evidence=None
    def observe(self,sample,joints):
        measured=_joint_measurement(sample,joints,self.names)
        if self.previous is not None and measured['iteration']!=self.previous+1:
            raise RuntimeError('stop measurement gap')
        self.previous=measured['iteration']
        if max(abs(v[1]) for v in measured['joints'].values())>=.002:
            self.samples=[];self._evidence=None;return False
        self.samples.append(measured)
        duration=measured['sim_ns']-self.samples[0]['sim_ns']
        if duration<300000000:return False
        self._evidence=dict(duration_sim_ns=duration,samples=self.samples,
            max_velocity_rad_s=max(abs(v[1]) for item in self.samples for v in item['joints'].values()))
        return True
    @property
    def evidence(self):return copy.deepcopy(self._evidence)


QUALIFIED_CAPABILITY_SHA256='9f750e46a438d4b415afb07d3d3b77ee66f636fd3beedb3ec8b3e5d90d8d0489'


def _metric(values):
    values=sorted(float(v) for v in values)
    if not values:return dict(count=0,p50=None,p95=None,p99=None,max=None)
    def percentile(q):
        return values[min(len(values)-1,max(0,math.ceil(q*len(values))-1))]
    return dict(count=len(values),p50=percentile(.50),p95=percentile(.95),
                p99=percentile(.99),max=values[-1])


def telemetry_metrics(timing,armed_monotonic_raw_ns=None):
    """Summarize the existing authoritative physics stream without changing it.

    Every interval is CLOCK_MONOTONIC_RAW: a wall-clock step between source and
    receive stamps would otherwise read as delivery latency."""
    receive=[e for e in timing if e.get('event')=='receive' and e.get('iteration') is not None
             and not e.get('error') and (armed_monotonic_raw_ns is None or e.get('receive_monotonic_raw_ns',0)>=armed_monotonic_raw_ns)]
    fresh=[e for e in timing if e.get('event')=='fresh' and e.get('iteration') is not None
           and not e.get('error') and (armed_monotonic_raw_ns is None or e.get('read_monotonic_raw_ns',0)>=armed_monotonic_raw_ns)]
    if len(receive)<2 or not fresh:raise RuntimeError('insufficient motion telemetry timing evidence')
    if any(type(e.get(k)) is not int for e in receive for k in ('source_monotonic_raw_ns','receive_monotonic_raw_ns')):
        raise RuntimeError('motion telemetry timing lacks monotonic stamps')
    receive.sort(key=lambda e:e['receive_monotonic_raw_ns'])
    if any(b['iteration']!=a['iteration']+1 for a,b in zip(receive,receive[1:])):
        raise RuntimeError('telemetry timing contains an iteration gap')
    source_intervals=[(b['source_monotonic_raw_ns']-a['source_monotonic_raw_ns'])/1e6 for a,b in zip(receive,receive[1:])]
    receive_intervals=[(b['receive_monotonic_raw_ns']-a['receive_monotonic_raw_ns'])/1e6 for a,b in zip(receive,receive[1:])]
    delivery=[(e['receive_monotonic_raw_ns']-e['source_monotonic_raw_ns'])/1e6 for e in receive]
    publish=[(e['publish_monotonic_raw_ns']-e['source_monotonic_raw_ns'])/1e6 for e in receive
             if e.get('publish_monotonic_raw_ns') is not None]
    callback=[e['callback_ns']/1e6 for e in receive if e.get('callback_ns') is not None]
    serialization=[e['source_serialization_ns']/1e6 for e in receive
                   if e.get('source_serialization_ns') is not None]
    ages=[e['age_ns']/1e6 for e in fresh if e.get('age_ns') is not None]
    lock_wait=[e['lock_wait_ns']/1e6 for e in fresh if e.get('lock_wait_ns') is not None]
    if any(v<0 for v in delivery+publish+ages):
        raise RuntimeError('telemetry monotonic-clock ordering is contradictory')
    sim_span=receive[-1].get('sim_ns',0)-receive[0].get('sim_ns',0)
    wall_span=receive[-1]['source_monotonic_raw_ns']-receive[0]['source_monotonic_raw_ns']
    result=dict(samples=len(receive),clock=measurement_clock.CLOCK_NAME,
        source_interval_ms=_metric(source_intervals),
        receive_interval_ms=_metric(receive_intervals),
        source_to_publish_ms=_metric(publish),
        source_to_receive_ms=_metric(delivery),
        callback_ms=_metric(callback),
        source_serialization_ms=_metric(serialization),
        fresh_age_ms=_metric(ages),
        lock_wait_ms=_metric(lock_wait),
        max_fresh_age_ms=max(ages),
        max_delivery_ms=max(delivery),
        real_time_factor=(sim_span/wall_span if wall_span>0 else None),
        first_iteration=receive[0]['iteration'],last_iteration=receive[-1]['iteration'])
    if result['max_fresh_age_ms']>=250.0:
        raise RuntimeError(f"motion telemetry exceeded unchanged 250 ms freshness guard: {result['max_fresh_age_ms']:.3f} ms")
    if result['max_delivery_ms']>=250.0:
        raise RuntimeError(f"motion telemetry delivery exceeded unchanged 250 ms freshness guard: {result['max_delivery_ms']:.3f} ms")
    return result


def require_cancellation_acceptance(summary):
    validate_controller_cancellation(summary.get('controller_cancellation',{}))
    reconciliation=summary.get('measured_reconciliation',{})
    recovery=summary.get('recovery_scene',{})
    if (not all(summary.get(k) for k in ('cancellation_confirmed','cancellation_accepted',
            'cancellation_movement_verified','motion_stop_verified')) or
        summary.get('interrupted_action_terminal_status')!=5 or
        not summary.get('owned_execution_goal',{}).get('accepted') or
        not summary.get('cancellation_motion',{}).get('movement') or
        summary.get('stopped_window',{}).get('duration_sim_ns',0)<300000000 or
        summary.get('recovery_inspection_failure') or not reconciliation.get('acm_restored') or
        not reconciliation.get('measured_geometry_matches') or
        reconciliation.get('held') or reconciliation.get('attached_ids') or
        not recovery.get('contact_acm_restored') or recovery.get('attached_ids')):
        raise RuntimeError('CANCELLATION_TRIAL_UNVERIFIED: movement, owned cancellation, stop or alive cleanup missing')


class ProcessStallWatch:
    """Diagnostic only. Samples this process's kernel scheduler, page-fault and
    swap counters so a freshness failure can be attributed to CPU starvation,
    major faults (swap-in) or blocking instead of guessed. It never touches a
    measurement, a guard or a threshold; its records are kept beside the
    acquisition timing evidence."""
    FILES=dict(vmstat=('pswpin','pswpout','pgmajfault','allocstall_movable','allocstall_normal'),
               status=('VmRSS','VmSwap','Threads'))
    def __init__(self,period_s=.02,heartbeat_s=.25,stall_ms=20.):
        import gc
        self.period=period_s;self.heartbeat=heartbeat_s;self.stall_ms=stall_ms
        self.events=[];self.stop_event=threading.Event()
        self.gc_started=None;self._gc=self._gc_callback;gc.callbacks.append(self._gc)
        self.thread=threading.Thread(target=self._run,name='process_stall_watch',daemon=True)
    def start(self):self.thread.start()
    def stop(self):
        import gc
        self.stop_event.set();self.thread.join(timeout=2)
        if self._gc in gc.callbacks:gc.callbacks.remove(self._gc)
    def _gc_callback(self,phase,info):
        if phase=='start':self.gc_started=time.monotonic_ns();return
        if self.gc_started is None:return
        duration=time.monotonic_ns()-self.gc_started;self.gc_started=None
        if info.get('generation')==2 or duration>=5000000:
            self.events.append(dict(event='gc',wall_ns=time.time_ns(),generation=info.get('generation'),
                collected=info.get('collected'),duration_ns=duration))
    @staticmethod
    def _read(path):
        try:
            with open(path) as f:return f.read()
        except OSError:return ''
    def sample(self,lateness_ms,reason):
        names={t.native_id:t.name for t in threading.enumerate() if t.native_id is not None}
        threads={}
        for tid in _task_ids():
            stat=self._read(f'/proc/self/task/{tid}/stat');sched=self._read(f'/proc/self/task/{tid}/schedstat').split()
            if not stat or len(sched)<3:continue
            rest=stat.rsplit(')',1)[1].split()
            threads[tid]=dict(name=names.get(tid),state=rest[0],minflt=int(rest[7]),majflt=int(rest[9]),
                utime=int(rest[11]),stime=int(rest[12]),run_ns=int(sched[0]),runqueue_wait_ns=int(sched[1]),slices=int(sched[2]))
        vm={};system={}
        for line in self._read('/proc/self/status').splitlines():
            key=line.split(':',1)[0]
            if key in self.FILES['status']:vm[key]=int(line.split()[1])
        for line in self._read('/proc/vmstat').splitlines():
            key,_,value=line.partition(' ')
            if key in self.FILES['vmstat']:system[key]=int(value)
        for resource in ('cpu','memory','io'):
            for line in self._read(f'/proc/pressure/{resource}').splitlines():
                kind,_,rest=line.partition(' ')
                total=next((f.split('=')[1] for f in rest.split() if f.startswith('total=')),None)
                if total is not None:system[f'psi_{resource}_{kind}_total_us']=int(total)
        system['loadavg']=self._read('/proc/loadavg').split()[:3]
        self.events.append(dict(event='process_watch',wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),
            reason=reason,lateness_ms=lateness_ms,threads=threads,vm=vm,system=system))
    def _sample(self,lateness_ms,reason):
        try:self.sample(lateness_ms,reason)
        except Exception as exc:
            self.events.append(dict(event='process_watch',wall_ns=time.time_ns(),reason=reason,lateness_ms=lateness_ms,error=str(exc)))
    def _run(self):
        self._sample(0.,'start');last_heartbeat=time.monotonic()
        while not self.stop_event.is_set():
            expected=time.monotonic()+self.period
            self.stop_event.wait(self.period)
            lateness_ms=(time.monotonic()-expected)*1e3
            if lateness_ms>=self.stall_ms:self._sample(lateness_ms,'late');last_heartbeat=time.monotonic()
            elif time.monotonic()-last_heartbeat>=self.heartbeat:self._sample(lateness_ms,'heartbeat');last_heartbeat=time.monotonic()
        self._sample(0.,'stop')


GIL_WATCHDOG_ENV='WORKCELL_GIL_WATCHDOG_MS'
GIL_PROBE_LIB_ENV='WORKCELL_GIL_PROBE_LIB'
GIL_PROBE_MS_ENV='WORKCELL_GIL_PROBE_MS'
GIL_PROBE_NATIVE_PC_ENV='WORKCELL_GIL_PROBE_NATIVE_PC'


class GilProbe:
    """Diagnostic only, opt-in (WORKCELL_GIL_PROBE_LIB + WORKCELL_GIL_PROBE_MS): loads the
    native gil_probe (scripts/gil_probe.c, built by the runner). Its own pthread reads only
    the static _PyRuntime GIL fields, never an interpreter object, and records every hold
    of at least the threshold; while a hold lasts it reads the holder's /proc CPU and
    syscall state and, with WORKCELL_GIL_PROBE_NATIVE_PC=1, samples its native PC.

    Attribution is done under the GIL with supported interfaces only:
      * every Python thread registers itself (thread-state pointer and never-reused id,
        kernel tid, GIL switch number): the constructing thread at start(), later threads
        through a one-shot threading.setprofile hook.
      * a sampler thread asks for the GIL every SAMPLER_PERIOD_S, so a hold this long is
        contended. When it waited at least a quarter of the threshold it takes one native
        snapshot (gil_probe_snapshot: GC off, GIL kept) of every thread's Python frames
        and of the live thread-state -> (ident, id) map, with the switch number at both
        ends. A hold is 'verified' only when the live id at the holder's address is the id
        of the registration in force at the hold's switch; its frames are 'exact' only
        when no thread ran during the snapshot and at most one between release and it.
    Every unavailable, ambiguous or inconclusive observation is labelled as such. It
    never touches a measurement, a guard or a threshold."""
    SAMPLER_PERIOD_S=.02
    FRAME_DEPTH=24
    LOOP_SLOW_NS=20_000_000
    MAX_SNAPSHOTS=2000
    COUNTERS=('holds','lates','unmapped_holds','proc_read_failures','pc_sent','pc_received','pc_lost',
              'pc_send_failures','pc_skipped_in_syscall','pc_handler_lost','brief_holds')
    def __init__(self,library,path,threshold_ms,native_pc=False):
        import ctypes,sys
        threshold_ms=float(threshold_ms)
        if not math.isfinite(threshold_ms) or not 20<=threshold_ms<=1000:
            raise RuntimeError('GIL probe threshold must be within [20, 1000] ms')
        import platform
        if sys.version_info[:2]!=(3,10):
            raise RuntimeError('GIL probe reads the CPython 3.10 runtime layout only')
        self.library=Path(library);self.library_sha256=hashlib.sha256(self.library.read_bytes()).hexdigest()
        # CDLL releases the GIL around start/stop; PyDLL keeps it for register and snapshot.
        self.lib=ctypes.CDLL(str(self.library));self.pylib=ctypes.PyDLL(str(self.library))
        self.path=Path(path);self.threshold_ms=threshold_ms;self.native_pc=bool(native_pc)
        # A const char[] in the library: read the bytes in place (it is not a char* pointer).
        built=ctypes.string_at(ctypes.addressof(ctypes.c_char.in_dll(self.lib,'gil_probe_python_version'))).decode()
        if built!=platform.python_version():
            raise RuntimeError(f'GIL probe was compiled for Python {built!r}, not {platform.python_version()}')
        self.python_version=built
        self.lib.gil_probe_start.argtypes=[ctypes.c_char_p,ctypes.c_longlong,ctypes.c_int];self.lib.gil_probe_start.restype=ctypes.c_int
        self.lib.gil_probe_stop.restype=ctypes.c_longlong
        self.lib.gil_probe_counters.argtypes=[ctypes.POINTER(ctypes.c_longlong),ctypes.c_int];self.lib.gil_probe_counters.restype=ctypes.c_int
        self.lib.gil_probe_pc_signal.restype=ctypes.c_int
        self.pylib.gil_probe_register.argtypes=[ctypes.c_int,ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_ulonglong),
                                                ctypes.POINTER(ctypes.c_ulong)]
        self.pylib.gil_probe_register.restype=ctypes.c_int
        self.pylib.gil_probe_snapshot.argtypes=[ctypes.py_object,ctypes.c_int];self.pylib.gil_probe_snapshot.restype=ctypes.c_int
        self.events=[];self.snapshots=[];self.started=False;self.names={};self.registry={}
        self.registration_failures=[];self.unregistered_at_start=[];self.hook_installed=False
        self.sampler_wakes=0;self.snapshot_failures=0;self.snapshots_dropped=0
        self.sampler_stop=threading.Event();self.sampler=None
    @classmethod
    def from_environment(cls,path):
        import os
        library=os.environ.get(GIL_PROBE_LIB_ENV);threshold=os.environ.get(GIL_PROBE_MS_ENV)
        if not library and not threshold:return None
        if not library or not threshold:raise RuntimeError('GIL probe needs both its library and its threshold')
        return cls(library,path,threshold,native_pc=os.environ.get(GIL_PROBE_NATIVE_PC_ENV)=='1')
    @property
    def loop_slow_ns(self):return self.LOOP_SLOW_NS if self.started else 0
    def register_current(self):
        import ctypes
        t=threading.current_thread();tid=threading.get_native_id()
        tstate,tstate_id,switch=ctypes.c_ulong(),ctypes.c_ulonglong(),ctypes.c_ulong()
        rc=self.pylib.gil_probe_register(tid,ctypes.byref(tstate),ctypes.byref(tstate_id),ctypes.byref(switch))
        if rc:self.registration_failures.append(dict(thread=t.name,rc=rc));return
        # A thread state's address and a pthread ident are both reused after a thread exits:
        # keep every registration with its never-reused thread-state id and switch number.
        self.registry.setdefault(tstate.value,[]).append(dict(ident=t.ident,native_id=tid,name=t.name,
            tstate_id=tstate_id.value,switch_number=switch.value,registered_wall_ns=time.time_ns()))
    def _thread_hook(self,frame,event,arg):
        import sys
        sys.setprofile(None)                  # one-shot: this thread is never profiled again
        try:self.register_current()
        except Exception as exc:self.registration_failures.append(dict(thread=threading.current_thread().name,error=str(exc)))
    def start(self):
        rc=self.lib.gil_probe_start(str(self.path).encode(),int(self.threshold_ms*1e6),int(self.native_pc))
        if rc:raise RuntimeError(f'GIL probe failed to start ({rc})')
        self.started=True
        import atexit
        atexit.register(self.stop)            # never left polling into interpreter finalization
        if threading.getprofile() is None:threading.setprofile(self._thread_hook);self.hook_installed=True
        self.register_current()
        self.unregistered_at_start=sorted(t.name for t in threading.enumerate() if t is not threading.current_thread())
        self.capture_names()
        self.sampler=threading.Thread(target=self._sample,name='gil_frame_sampler',daemon=True);self.sampler.start()
    def capture_names(self):
        # close() joins the acquisition and writer threads before stop(): keep their names now.
        self.names.update((t.ident,t.name) for t in threading.enumerate())
    def _sample(self):
        # The wait is measured from the end of the previous iteration, so the GIL being kept
        # from this thread anywhere in its loop (not only inside sleep) is seen.
        # A quarter of the threshold: with the 20 ms period, every hold of at least half the
        # threshold (brief holds included) keeps this thread waiting at least that long.
        period_ns=int(self.SAMPLER_PERIOD_S*1e9);snapshot_ns=self.threshold_ms*2.5e5
        last=time.monotonic_ns()
        while not self.sampler_stop.is_set():
            time.sleep(self.SAMPLER_PERIOD_S);got=time.monotonic_ns()
            self.sampler_wakes+=1;wait_ns=got-last-period_ns
            if wait_ns<snapshot_ns:last=time.monotonic_ns();continue
            if len(self.snapshots)>=self.MAX_SNAPSHOTS:self.snapshots_dropped+=1;last=time.monotonic_ns();continue
            out=[]
            try:self.pylib.gil_probe_snapshot(out,self.FRAME_DEPTH)
            except Exception as exc:
                self.snapshot_failures+=1
                self.snapshots.append(dict(event='gil_frame_snapshot_failed',wall_ns=time.time_ns(),wait_ms=wait_ns/1e6,error=str(exc)))
                last=time.monotonic_ns();continue
            switch_start,switch_end,wall_ns,monotonic_ns,threads,tstates=out[0]
            names={str(t.ident):t.name for t in threading.enumerate()}   # idents are reused: names as of now
            # asked: the sampler's last moment with the GIL; acquired: its first one again.
            self.snapshots.append(dict(event='gil_frame_snapshot',wall_ns=wall_ns,monotonic_ns=monotonic_ns,
                asked_monotonic_ns=last,acquired_monotonic_ns=got,wait_ms=wait_ns/1e6,
                switch_number=switch_start,switch_number_end=switch_end,
                threads={str(ident):[dict(file=f,function=n,firstlineno=fl,lasti=la,line=ln) for f,n,fl,la,ln in stack]
                         for ident,stack in threads.items()},
                tstates={str(tstate):[ident,tstate_id] for tstate,(ident,tstate_id) in tstates.items()},names=names))
            last=time.monotonic_ns()
    def note_slow(self,timing,call,started_ns,ended_ns):
        """Executor loop timing (diagnostic): one record per call that took at least LOOP_SLOW_NS."""
        timing.append(dict(event='loop_slow',call=call,thread=threading.current_thread().name,
            start_wall_ns=time.time_ns()-(time.monotonic_ns()-started_ns),duration_ns=ended_ns-started_ns))
    def stop(self):
        import ctypes
        if not self.started:return
        sampler_alive=bool(self.sampler and self.sampler.is_alive())
        self.sampler_stop.set()
        if self.sampler is not None:self.sampler.join(timeout=2)
        if self.hook_installed and threading.getprofile()==self._thread_hook:threading.setprofile(None)
        holds=self.lib.gil_probe_stop();self.started=False
        values=(ctypes.c_longlong*len(self.COUNTERS))();self.lib.gil_probe_counters(values,len(values))
        counters=dict(zip(self.COUNTERS,values))
        maps=self.path.with_suffix('.maps')
        try:maps.write_text(Path('/proc/self/maps').read_text())
        except OSError:maps=None
        self.capture_names()
        self.events.extend(self.parse(self.path.read_text(errors='replace'),self.registry,self.names,self.snapshots))
        self.events.extend(self.snapshots)
        self.events.append(dict(event='gil_probe_summary',wall_ns=time.time_ns(),threshold_ms=self.threshold_ms,
            holds=holds,**{k:v for k,v in counters.items() if k!='holds'},file=self.path.name,maps_file=maps.name if maps else None,
            library=str(self.library),library_sha256=self.library_sha256,python_version=self.python_version,
            native_pc=self.native_pc,pc_signal=self.lib.gil_probe_pc_signal() or None,
            sampler_period_ms=self.SAMPLER_PERIOD_S*1e3,sampler_wakes=self.sampler_wakes,
            sampler_alive_before_stop=sampler_alive,snapshots=sum(1 for s in self.snapshots if s['event']=='gil_frame_snapshot'),
            snapshot_failures=self.snapshot_failures,snapshots_dropped=self.snapshots_dropped,
            registered_threads=sorted({e['name'] for v in self.registry.values() for e in v}),
            registration_failures=self.registration_failures,unregistered_at_start=self.unregistered_at_start,
            thread_hook=self.hook_installed))
    @staticmethod
    def parse(text,registry,names,snapshots=()):
        holds=[];lates=[];detail={}
        snapshots=sorted((s for s in snapshots if s.get('event')=='gil_frame_snapshot'),key=lambda s:s['wall_ns'])
        for line in text.splitlines():
            fields=line.split()
            if fields[:1]==['hold'] and len(fields)==8:
                holds.append(dict(event='gil_probe_hold',start_wall_ns=int(fields[1]),end_wall_ns=int(fields[2]),
                    hold_ms=float(fields[3]),tstate=int(fields[4],0),switch_number=int(fields[5]),native_id=int(fields[6]),
                    max_poll_gap_ms=float(fields[7])))
            elif fields[:1]==['brief'] and len(fields)==7:     # >= threshold/2: identity and frames only
                holds.append(dict(event='gil_probe_brief_hold',start_wall_ns=int(fields[1]),end_wall_ns=int(fields[2]),
                    hold_ms=float(fields[3]),tstate=int(fields[4],0),switch_number=int(fields[5]),native_id=-1,
                    max_poll_gap_ms=float(fields[6])))
            elif fields[:1]==['open'] and len(fields)==7:      # still held when the probe stopped
                holds.append(dict(event='gil_probe_open_hold',start_wall_ns=int(fields[1]),end_wall_ns=int(fields[2]),
                    hold_ms=float(fields[3]),tstate=int(fields[4],0),switch_number=int(fields[5]),native_id=int(fields[6]),
                    max_poll_gap_ms=None))
            elif fields[:1]==['late'] and len(fields)==4:
                lates.append(dict(event='gil_probe_late',start_wall_ns=int(fields[1]),end_wall_ns=int(fields[2]),late_ms=float(fields[3])))
            elif fields[:1] in (['cpu'],['sys'],['pc']) and len(fields)>=3:
                detail.setdefault(int(fields[1]),[]).append(fields)
        for hold in holds:
            hold['overlaps_probe_late']=any(l['start_wall_ns']<hold['end_wall_ns'] and l['end_wall_ns']>hold['start_wall_ns'] for l in lates)
            # The snapshot of the sampler that was already waiting when the hold ended. The probe
            # sees the release one poll (~1.1 ms) late; no snapshot can fall inside a hold (it needs
            # the GIL) and one taken before the hold is at least the threshold earlier.
            snap=next((s for s in snapshots if s['wall_ns']>=hold['end_wall_ns']-5_000_000 and
                       s['wall_ns']-(s['monotonic_ns']-s['asked_monotonic_ns'])<=hold['end_wall_ns']),None)
            live=snap['tstates'].get(str(hold['tstate'])) if snap else None
            # The registration in force at the hold's switch: a registration made during the hold
            # has the hold's switch number (only the holder runs Python then); a later thread
            # reusing the address registers at a higher one.
            entry=max((e for e in registry.get(hold['tstate'],()) if e['switch_number']<=hold['switch_number']),
                      key=lambda e:(e['switch_number'],e['registered_wall_ns']),default=None)
            ident=None;name=None
            if live is not None and entry is not None:
                if live[1]==entry['tstate_id']:ident,name,attribution=live[0],entry['name'],'verified'
                else:attribution='address reused, ambiguous'
            elif live is not None:
                # a thread state created but not yet started carries its creator's ident
                if sum(1 for v in snap['tstates'].values() if v[0]==live[0])>1:attribution='ambiguous (thread starting)'
                else:ident,name,attribution=live[0],snap.get('names',{}).get(str(live[0])),'snapshot only'
            elif entry is not None:name,attribution=entry['name'],'registered, unconfirmed'
            else:attribution='unattributed'
            hold.update(thread_id=ident,thread=name,attribution=attribution,registration=entry)
            # /proc and PC data were read for the latest registration's tid at the threshold crossing
            if hold['native_id']<=0:proc='none'
            elif attribution=='verified' and entry['native_id']==hold['native_id']:proc='holder'
            elif attribution=='registered, unconfirmed' and entry['native_id']==hold['native_id']:proc='registered thread, unconfirmed'
            else:proc='not attributable to the holder'
            hold['holder_proc_data']=proc
            records=detail.get(hold['start_wall_ns'],[])
            cpu={f[3]:f[4:] for f in records if f[0]=='cpu'}
            if len(cpu.get('begin',[]))==8 and len(cpu.get('end',[]))==8:
                b,e=cpu['begin'],cpu['end']
                hold['holder_cpu']=dict(state_begin=b[0],state_end=e[0],utime_ticks=int(e[1])-int(b[1]),stime_ticks=int(e[2])-int(b[2]),
                    minflt=int(e[3])-int(b[3]),majflt=int(e[4])-int(b[4]),on_cpu_ms=(int(e[5])-int(b[5]))/1e6,
                    runqueue_ms=(int(e[6])-int(b[6]))/1e6,window='threshold crossing to end')
            else:hold['holder_cpu']=None
            sys_samples=[' '.join(f[3:]) for f in records if f[0]=='sys']
            hold['holder_syscall']=dict(samples=len(sys_samples),running=sum(1 for s in sys_samples if s=='running'),
                unavailable=sum(1 for s in sys_samples if s=='unavailable'),
                in_syscall=sorted({s.split()[0] for s in sys_samples if s not in ('running','unavailable')}))
            hold['native_pc']=[dict(sent_wall_ns=int(f[2]),delivery_us=int(f[3]),tid=int(f[4]),rip=int(f[5],0),rsp=int(f[6],0),
                stack_bytes=int(f[7]),candidates=[int(a,0) for a in f[9:9+int(f[8])]]) for f in records if f[0]=='pc' and len(f)>=9]
            # The holder's frames at the release: exact when no thread ran during the snapshot and at
            # most one other thread ran between the release and it.
            if snap is None:hold.update(release_frames=None,release_frames_status='no snapshot')
            elif ident is None:hold.update(release_frames=None,release_frames_status='holder not attributable')
            else:
                frames=snap['threads'].get(str(ident))
                after=snap['switch_number']-hold['switch_number']
                steady=snap.get('switch_number_end',snap['switch_number'])==snap['switch_number']
                hold.update(release_frames=frames,
                    release_frames_status='holder has no Python frame' if frames is None else
                        ('exact' if steady and 1<=after<=2 else 'holder may have run again'),
                    release_snapshot_switches_after=after,release_snapshot_delay_ms=(snap['wall_ns']-hold['end_wall_ns'])/1e6)
        return holds+lates


class TimedRLock:
    """Reentrant lock that keeps per-thread wait/hold totals per wall-clock second.

    Diagnostic only: it behaves exactly like threading.RLock for `with` use and
    records, for the outermost acquisition of each thread, how long it waited and
    how long it held the lock, so the evidence names which thread blocks telemetry
    acquisition. Totals are aggregated (no per-acquisition records)."""
    def __init__(self):
        self._lock=threading.RLock();self._local=threading.local();self.stats={}
    def __enter__(self):
        waiting=time.monotonic_ns();self._lock.acquire();entered=time.monotonic_ns()
        depth=getattr(self._local,'depth',0);self._local.depth=depth+1
        if depth==0:self._local.entered=entered;self._local.wait=entered-waiting
        return self
    def __exit__(self,*exc):
        depth=self._local.depth-1;self._local.depth=depth
        try:
            if depth==0:
                hold=time.monotonic_ns()-self._local.entered;wait=self._local.wait
                key=(threading.current_thread().name,time.time_ns()//1_000_000_000)
                entry=self.stats.get(key)
                if entry is None:entry=self.stats[key]=[0,0,0,0,0]
                entry[0]+=1;entry[1]+=wait;entry[2]=max(entry[2],wait);entry[3]+=hold;entry[4]=max(entry[4],hold)
        finally:
            self._lock.release()
        return False
    def held_by_current_thread(self):
        return getattr(self._local,'depth',0)>0
    def profile(self):
        with self:
            rows=[dict(thread=name,second=second,count=c,wait_ms=w/1e6,wait_max_ms=wm/1e6,hold_ms=h/1e6,hold_max_ms=hm/1e6)
                  for (name,second),(c,w,wm,h,hm) in sorted(self.stats.items(),key=lambda item:(item[0][1],item[0][0]))]
        return dict(event='lock_profile',wall_ns=time.time_ns(),rows=rows)


class GilHoldWatch:
    """Diagnostic only, opt-in (WORKCELL_GIL_WATCHDOG_MS). Names the code that holds
    the interpreter lock during a stall. A heartbeat thread re-arms CPython's
    faulthandler watchdog every period; the watchdog is a native thread that needs no
    GIL, so if the heartbeat cannot run for about threshold - period (one thread is
    inside a call that keeps the GIL, a garbage collection, or the process is starved
    of CPU) it writes every thread's Python stack to the dump file. The dump does not
    mark the GIL holder; read it with the gc and receive events of the same timing
    file. The heartbeat records each overrun so dumps can be matched to that evidence.
    It never touches a measurement, a guard or a threshold."""
    PERIOD_S=.025
    def __init__(self,path,threshold_ms):
        threshold_ms=float(threshold_ms)
        if not math.isfinite(threshold_ms) or not 50<=threshold_ms<=1000:
            raise RuntimeError('GIL watchdog threshold must be within [50, 1000] ms')
        self.path=Path(path);self.threshold=threshold_ms/1e3;self.events=[];self.file=None
        self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self._run,name='gil_hold_watch',daemon=True)
    @classmethod
    def from_environment(cls,path):
        import os
        value=os.environ.get(GIL_WATCHDOG_ENV)
        return cls(path,value) if value else None
    def start(self):
        self.file=self.path.open('w');self.thread.start()
    def _run(self):
        import faulthandler
        overruns=0;last=time.monotonic()
        try:
            while not self.stop_event.is_set():
                faulthandler.dump_traceback_later(self.threshold,repeat=False,file=self.file)
                self.stop_event.wait(self.PERIOD_S)
                now=time.monotonic();gap=now-last;last=now
                if gap>self.threshold:
                    overruns+=1
                    self.events.append(dict(event='gil_hold',wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),
                        gap_ms=gap*1e3,overrun=overruns))
        finally:
            faulthandler.cancel_dump_traceback_later()
    def stop(self):
        import faulthandler
        self.stop_event.set();self.thread.join(timeout=2)
        # The timer is process-global: cancel it here too, so no late dump can reach a
        # reused file descriptor; keep the file open while the heartbeat could re-arm.
        faulthandler.cancel_dump_traceback_later()
        if self.file is None:return
        if self.thread.is_alive():
            self.events.append(dict(event='gil_watch_summary',wall_ns=time.time_ns(),thread_alive=True,
                dump_file=self.path.name));return
        self.file.close()
        dumps=self.path.read_text(errors='replace').count('Timeout (')
        self.events.append(dict(event='gil_watch_summary',wall_ns=time.time_ns(),threshold_ms=self.threshold*1e3,
            period_ms=self.PERIOD_S*1e3,overruns=sum(1 for e in self.events if e['event']=='gil_hold'),dumps=dumps,
            dump_file=self.path.name))


def _task_ids():
    import os
    try:return [int(t) for t in os.listdir('/proc/self/task')]
    except OSError:return []


class Measurements:
    def __init__(self,node,receipt,log):
        import yaml
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
        from simulator_backend import verify_receipt_process,PHYSICS_POSE_METHOD
        self.receipt_path=Path(receipt);self.receipt=verify_receipt_process(receipt)
        if self.receipt.get('measurement_pose_source')!=PHYSICS_POSE_METHOD:
            raise RuntimeError('authoritative physics pose source required before acquisition')
        from rclpy.node import Node
        self.node=Node('simulator_measurement_acquisition',context=node.context)
        self.lock=TimedRLock();self.latest=None;self.pending=[];self.error=None;self.previous=None
        self.timing=[];self.armed_wall_ns=None;self.armed_monotonic_raw_ns=None;self.log_path=Path(log)
        self.log=Path(log).open('w');self.armed=False
        self.log_queue=queue.Queue(maxsize=10000);self.raw={}
        self.sub=self.node.create_subscription(String,f"/world/{self.receipt['world']}/workcell_measurements",self.update,
            QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST))
        self.stop_guard=self.node.create_guard_condition(lambda:None);self.stopping=False
        self.loader=yaml.CSafeLoader
        self.robot=ET.parse(self.receipt_path.parent/'robot.urdf').getroot()
        self.parents={j.find('child').get('link'):j for j in self.robot.findall('joint')}
        # Validated (and the native probe started) before any thread starts, so a bad
        # value or a failed start cannot strand them.
        self.gil_watch=GilHoldWatch.from_environment(self.log_path.with_suffix('.gil-holds.txt'))
        self.gil_probe=GilProbe.from_environment(self.log_path.with_suffix('.gil-probe.txt'))
        if self.gil_probe is not None:self.gil_probe.start()
        self.writer=threading.Thread(target=self._write,name='measurement_evidence');self.writer.start()
        self.acquisition=threading.Thread(target=self._acquire,name='measurement_acquisition');self.acquisition.start()
        self.watch=ProcessStallWatch();self.watch.start()
        if self.gil_watch is not None:self.gil_watch.start()
        if self.gil_probe is not None:self.gil_probe.capture_names()
    def _acquire(self):
        # One wait set built once, every available message taken per wake. The
        # executor-based spin rebuilt its wait set per message, which measured
        # 1.25 ms of CPU per 2 ms sample (0.44 ms of it the callback) and left
        # the 500 Hz stream no headroom against consumer or evidence work.
        # Every taken message still goes through the unchanged update() path.
        from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
        from std_msgs.msg import String
        context=self.node.context
        try:
            timer=self.loop_timer()     # GIL probe only: which native call of this loop was slow
            with self.sub.handle,self.stop_guard.handle,context.handle:
                wait_set=_rclpy.WaitSet(1,1,0,0,0,0,context.handle)
                while not self.stopping and context.ok():
                    started=time.monotonic_ns() if timer else 0
                    wait_set.clear_entities()
                    wait_set.add_subscription(self.sub.handle);wait_set.add_guard_condition(self.stop_guard.handle)
                    if timer:rebuilt=time.monotonic_ns();timer('wait_set_rebuild',started,rebuilt)
                    wait_set.wait(100000000)
                    if timer:timer('wait_set_wait',rebuilt,time.monotonic_ns())
                    if self.stopping or not context.ok():break
                    if not wait_set.is_ready('subscription',0):continue
                    while not self.stopping:
                        started=time.monotonic_ns() if timer else 0
                        taken=self.sub.handle.take_message(String,False)
                        if timer:timer('take_message',started,time.monotonic_ns())
                        if taken is None:break
                        self.update(taken[0])
        except Exception as exc:
            with self.lock:
                if self.error is None:self.error=RuntimeError(f'measurement acquisition stopped: {exc}')
        finally:
            if not self.stopping:
                with self.lock:
                    if self.error is None:self.error=RuntimeError('measurement acquisition stopped before close')
    def update(self,msg):
        received=time.time_ns();started=time.monotonic_ns();received_raw=measurement_clock.now_ns();s={};sample_error=None
        # Parsing touches no shared state; keep it out of the lock that fresh(),
        # drain() and the evidence writer contend for. A parse failure is raised
        # inside the lock exactly where json.loads used to raise it.
        try:parsed=json.loads(msg.data);parse_error=None
        except Exception as exc:parsed=None;parse_error=exc
        waiting=time.monotonic_ns();parse_ns=waiting-started
        with self.lock:
            lock_wait_ns=time.monotonic_ns()-waiting
            try:
                if parse_error is not None:raise parse_error
                s=parsed
                if self.latest and s.get('sim_ns',0)<=self.latest['sim_ns']:raise RuntimeError('simulation time stopped/reversed')
                validate_sample(s,self.receipt,measurement_clock.now_ns()/1e9,self.previous if self.armed else None)
                self.previous=s['iteration'];self.latest=s
                if self.armed:
                    if len(self.pending)>=10000:raise RuntimeError('measurement consumer queue overflow')
                    self.pending.append(s)
                    # The simulator's own serialization is the evidence text; keep
                    # it so the writer never re-serializes the parsed sample.
                    if len(self.raw)>=10000:del self.raw[next(iter(self.raw))]
                    self.raw[s['iteration']]=msg.data
            except StaleMeasurement as exc:
                sample_error=exc
                if self.armed and self.error is None:self.error=exc
            except Exception as exc:
                sample_error=exc
                if self.error is None:self.error=exc
            self.timing.append(dict(event='receive',iteration=s.get('iteration'),sim_ns=s.get('sim_ns'),
                source_wall_ns=s.get('wall_ns'),publish_wall_ns=s.get('publish_wall_ns'),
                source_monotonic_raw_ns=s.get('monotonic_raw_ns'),publish_monotonic_raw_ns=s.get('publish_monotonic_raw_ns'),
                source_serialization_ns=s.get('serialization_ns'),receive_wall_ns=received,
                receive_monotonic_raw_ns=received_raw,
                callback_ns=time.monotonic_ns()-started,parse_ns=parse_ns,lock_wait_ns=lock_wait_ns,queue_depth=len(self.pending),
                error=str(self.error or sample_error) if self.error or sample_error else None))
    def loop_timer(self):
        """GIL probe only: a recorder of loop calls that took at least its LOOP_SLOW_NS, else None."""
        probe=getattr(self,'gil_probe',None)
        slow=probe.loop_slow_ns if probe is not None else 0
        if not slow:return None
        def record(call,started_ns,ended_ns):
            if ended_ns-started_ns<slow:return
            # list.append is atomic under the GIL: no lock, so this never waits on an exception path
            try:probe.note_slow(self.timing,call,started_ns,ended_ns)
            except Exception:pass              # a diagnostic record never fails the caller
        return record
    def fresh(self):
        # Error, latest sample and read time are taken together under the lock;
        # the unchanged validation of that same immutable sample at that same
        # time then runs outside it (inside it when the caller already holds it).
        waiting=time.monotonic_ns()
        with self.lock:
            entered=time.monotonic_ns();now_ns=time.time_ns();read_raw=measurement_clock.now_ns();error=self.error;latest=self.latest
            if error:
                self.timing.append(dict(event='fresh',iteration=None,read_wall_ns=now_ns,read_monotonic_raw_ns=read_raw,
                    lock_wait_ns=entered-waiting,error=str(error)));raise error
            if latest is None:
                self.timing.append(dict(event='fresh',iteration=None,read_wall_ns=now_ns,read_monotonic_raw_ns=read_raw,
                    lock_wait_ns=entered-waiting,error='no simulator measurement'))
                raise RuntimeError('no simulator measurement')
        stamp=latest.get('monotonic_raw_ns')
        age_ns=read_raw-stamp if type(stamp) is int else None
        try:validate_sample(latest,self.receipt,read_raw/1e9)
        except Exception as exc:
            with self.lock:
                self.timing.append(dict(event='fresh',iteration=latest.get('iteration'),read_wall_ns=now_ns,
                    read_monotonic_raw_ns=read_raw,age_ns=age_ns,lock_wait_ns=entered-waiting,error=str(exc)))
            raise
        with self.lock:
            self.timing.append(dict(event='fresh',iteration=latest['iteration'],read_wall_ns=now_ns,
                read_monotonic_raw_ns=read_raw,age_ns=age_ns,lock_wait_ns=entered-waiting,error=None))
        return latest
    def drain(self):
        self.fresh()
        with self.lock:
            # An error latched after that read still prevents consumption.
            if self.error:
                self.timing.append(dict(event='fresh',iteration=None,read_wall_ns=time.time_ns(),
                    lock_wait_ns=0,error=str(self.error)));raise self.error
            pending,self.pending=self.pending,[]
            return pending
    def arm(self):
        from simulator_backend import verify_receipt_process
        verify_receipt_process(self.receipt_path)
        publishers=self.node.get_publishers_info_by_topic(f"/world/{self.receipt['world']}/workcell_measurements")
        validate_publisher(publishers)
        with self.lock:
            self.fresh();self.armed_wall_ns=time.time_ns();self.armed_monotonic_raw_ns=measurement_clock.now_ns()
            self.armed=True;self.pending=[]
    def metrics(self):
        return telemetry_metrics(self.timing,self.armed_monotonic_raw_ns)
    def frame(self,s,link):
        prefix=self.receipt['world']+'::'+self.receipt['model']+'::'
        transform=[0,0,0,0,0,0,1]
        while prefix+link not in s['poses']:
            j=self.parents.get(link)
            if j is None or j.get('type')!='fixed':raise RuntimeError('measured frame missing: '+link)
            o=j.find('origin');xyz=[float(x) for x in o.get('xyz','0 0 0').split()] if o is not None else [0,0,0]
            rpy=[float(x) for x in o.get('rpy','0 0 0').split()] if o is not None else [0,0,0]
            transform=compose_pose(xyz+quaternion_from_rpy(rpy),transform);link=j.find('parent').get('link')
        return compose_pose(s['poses'][prefix+link],transform)
    def object_pose(self,s,name):
        # Telemetry JSON writes exact values as integers (an upright resting object's quaternion w is 1);
        # ROS pose fields accept only float.
        return [float(v) for v in s['poses'][self.receipt['world']+'::'+name]]
    def joints(self,s):
        prefix=self.receipt['world']+'::'+self.receipt['model']+'::'
        return {k[len(prefix):]:v for k,v in s['joints'].items() if k.startswith(prefix)}
    def record(self,phase,s):
        # Queue immutable received samples. Never serialize on acquisition or on
        # the action/cancel callback path. Overflow latches a failure, not loss.
        wall=time.time_ns()
        try:self.log_queue.put_nowait((phase,s,wall))
        except queue.Full:
            with self.lock:
                if self.error is None:self.error=RuntimeError('measurement evidence queue overflow')
            raise self.error
    def _write(self):
        while True:
            item=self.log_queue.get()
            if item is None:return
            phase,s,wall=item;begin=time.monotonic_ns()
            try:
                with self.lock:text=self.raw.pop(s['iteration'],None)
                if text is None or len(text.splitlines())!=1:text=json.dumps(s)
                self.log.write('{"phase": '+json.dumps(phase)+', "measurement": '+text+'}\n')
            except Exception as exc:
                with self.lock:
                    if self.error is None:self.error=exc
            with self.lock:
                self.timing.append(dict(event='consume_log',iteration=s['iteration'],consume_wall_ns=wall,
                    logging_ns=time.monotonic_ns()-begin,queue_depth=self.log_queue.qsize()))
    def close(self):
        self.stopping=True;self.stop_guard.trigger();self.acquisition.join()
        self.node.destroy_node()
        self.log_queue.put(None);self.writer.join();self.log.close()
        self.watch.stop();self.timing.extend(self.watch.events)
        if self.gil_watch is not None:self.gil_watch.stop();self.timing.extend(self.gil_watch.events)
        if self.gil_probe is not None:self.gil_probe.stop();self.timing.extend(self.gil_probe.events)
        if isinstance(self.lock,TimedRLock):self.timing.append(self.lock.profile())
        self.log_path.with_suffix('.timing.json').write_text(json.dumps(self.timing))


DESTINATION_PHASES=('place','opening','released')


class ContactGuard:
    def __init__(self,measurements,object_id,dimensions,touch_links,support,baseline,tool):
        from ament_index_python.packages import get_package_prefix
        library=ctypes.CDLL(str(Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'))
        self.predicate=library.workcell_support_contact_valid
        self.predicate.argtypes=[ctypes.c_char_p]*4+[ctypes.c_double,ctypes.POINTER(ctypes.c_double)];self.predicate.restype=ctypes.c_bool
        self.m=measurements;self.object=object_id;self.name=object_id.removeprefix('runtime::');self.dimensions=dimensions
        self.touch=set(touch_links);self.support=support;self.tool=tool;self.held=None;self.separation=None
        self.phase='approach';self.last=None;self.min_clearance=math.inf;self.validate_current=None
        self.allowed={(a,b) for i,a in enumerate(baseline.entry_names) for j,b in enumerate(baseline.entry_names) if baseline.entry_values[i].enabled[j]}
        # Touch links against the bound support are judged here by measured points on the
        # certified floor, never by a planning-scene allowance: refuse an ACM that would
        # let `(a,b) in self.allowed` admit a finger/support contact.
        # The destination floor (receipt binding) is admitted the same way during place/opening/released.
        binding=(getattr(measurements,'receipt',None) or {}).get('destination_geometry_binding')
        self.destination=(dict(object_id=object_id,support_id=binding['object_id'],floor_z=float(binding['floor_z']))
                          if binding and binding.get('object_id') and binding.get('floor_z') is not None else None)
        for bound in (support,self.destination):
            if not bound:continue
            exposed=sorted(t for t in self.touch if (t,bound['support_id']) in self.allowed or (bound['support_id'],t) in self.allowed)
            if exposed:raise RuntimeError('SUPPORT_CONTACT_POLICY: touch links ACM-allowed against the bound support: '+', '.join(exposed))
        self.fingers=set();self.release_samples=[];self.held_samples=0;self.held_start_sim_ns=None;self.held_proof=None
        self.pile_objects={};self.pile_collisions={};self.pile_certificate=None
        self.pile_expired=set();self.pile_origin=None;self.pile_height=0.;self.planning_attached=False
        self.ownership='UNHELD';self.release_goal=None;self.release_candidate=None;self.release_transition=None
        self.library=library;self.pile_admitting=False;self.pile_live_sequence=False;self.pile_last_iteration=None
        # The end-effector capability's hold profile decides which measured contact
        # links a verified hold must keep (set by the executor; None keeps every touch link).
        self.hold_profile=None
        # 'strict' (qualification) or 'record' (functional validation); limits are shared.
        self.retention_policy='strict';self.retention_record=None

    def bind_pile(self,objects,binding):
        """Bind perceived BOX geometry to the receipt's immutable physical world."""
        world=ET.parse(self.m.receipt_path.parent/'world.sdf').getroot().find('world')
        if world is None or world.get('name')!=self.m.receipt['world']:
            raise RuntimeError('pile world identity mismatch')
        if not binding.get('resolution_sha256') or not binding.get('execution_attempt'):
            raise RuntimeError('pile execution binding missing')
        self.pile_binding=copy.deepcopy(binding)
        for obj in objects:
            name=obj['object_id'];oid=obj['id']
            if oid!='runtime::'+name or oid in self.pile_objects or '::' in name:
                raise RuntimeError('ambiguous pile object identity')
            models=world.findall(f"model[@name='{name}']")
            if len(models)!=1 or models[0].findtext('static','false').lower() not in ('false','0'):
                raise RuntimeError('unknown/non-dynamic pile model '+name)
            links=models[0].findall('link')
            collisions=links[0].findall('collision') if len(links)==1 else []
            if len(collisions)!=1 or obj['shape']!='BOX':raise RuntimeError('unsupported pile geometry '+name)
            link=links[0];collision=collisions[0]
            for element in (link,collision):
                pose=element.find('pose')
                if pose is not None and (pose.attrib or any(float(x)!=0. for x in pose.text.split())):
                    raise RuntimeError('unsupported pile collision frame '+name)
            size=[float(x) for x in collision.findtext('geometry/box/size','').split()]
            if len(size)!=3 or size!=obj['dimensions'] or any(not math.isfinite(x) or x<=0 for x in size):
                raise RuntimeError('physical/perceived pile geometry mismatch '+name)
            scoped='::'.join((self.m.receipt['world'],name,link.get('name'),collision.get('name')))
            self.pile_collisions[scoped]=oid
            self.pile_objects[oid]=dict(name=name,size=size)
        if self.object not in self.pile_objects:raise RuntimeError('selected pile identity missing')
        self.pile_geometry_fn=self.library.workcell_measured_pile_contact
        pointer=ctypes.POINTER(ctypes.c_double)
        self.pile_geometry_fn.argtypes=[pointer]*5+[ctypes.c_size_t,pointer]
        self.pile_geometry_fn.restype=ctypes.c_bool
        self.pile_predicate=self.library.workcell_pile_contact_valid
        self.pile_predicate.argtypes=[ctypes.c_char_p]*4+[pointer]
        self.pile_predicate.restype=ctypes.c_bool

    def pile_geometry(self,s,neighbor,points):
        a=self.pile_objects[self.object];b=self.pile_objects[neighbor]
        def array(values):return (ctypes.c_double*len(values))(*values)
        output=(ctypes.c_double*8)()
        valid=self.pile_geometry_fn(array(a['size']),array(self.m.object_pose(s,a['name'])),
            array(b['size']),array(self.m.object_pose(s,b['name'])),
            array([x for point in points for x in point]),len(points),output)
        return bool(valid),dict(depth_m=output[0],separation_m=output[1],normal=list(output[2:5]),point=list(output[5:8]))

    def pile_contacts(self,s):
        result={}
        for c in s['contacts']:
            if c['a'] not in s['collisions'] or c['b'] not in s['collisions']:
                raise RuntimeError('unknown measured collision identity')
            a,b=self.identity(c['a']),self.identity(c['b'])
            if self.object not in (a,b):continue
            neighbor=b if a==self.object else a
            if neighbor in self.touch or (self.support and neighbor==self.support['support_id']):continue
            if self.destination and neighbor==self.destination['support_id'] and self.phase in DESTINATION_PHASES:continue
            if neighbor not in self.pile_objects:raise RuntimeError('uncertified target contact identity '+neighbor)
            if neighbor in result:raise RuntimeError('ambiguous duplicate pile contact '+neighbor)
            if not c.get('points') or any(len(p)!=3 or not all(math.isfinite(x) for x in p) for p in c['points']):
                raise RuntimeError('incomplete measured pile contact: '+neighbor)
            result[neighbor]=c
        return result

    def _pile_rejection(self,s,exc):
        if self.pile_certificate is not None:
            if not self.pile_certificate['rejected'] or self.pile_certificate['rejected'][-1]!=str(exc):
                self.pile_certificate['rejected'].append(str(exc))
            if 'rejected_sample' not in self.pile_certificate:
                self.pile_certificate['rejected_sample']=copy.deepcopy(s)

    def _admit_pile_contact(self,s,neighbor,c):
        valid,geometry=self.pile_geometry(s,neighbor,c['points'])
        entry=dict(neighbor=neighbor,physical_pair=[c['a'],c['b']],points=copy.deepcopy(c['points']),
            target_pose=list(self.m.object_pose(s,self.name)),
            neighbor_pose=list(self.m.object_pose(s,self.pile_objects[neighbor]['name'])),geometry=geometry)
        if not valid:raise RuntimeError('measured pile geometry/depth rejected: '+neighbor)
        self.pile_certificate['contacts'].append(entry)
        self.pile_certificate['certified_set'].append(neighbor)
        self.pile_certificate['certified_set'].sort()
        self.pile_certificate['admissions'].append(dict(copy.deepcopy(entry),iteration=s['iteration'],
            sim_ns=s['sim_ns'],wall_ns=s['wall_ns'],monotonic_raw_ns=s['monotonic_raw_ns'],
            freshness_ms=(measurement_clock.now_ns()-s['monotonic_raw_ns'])/1e6))

    def certify_pile(self,s):
        # Direct certification stays a frozen, one-sample operation. The live
        # post-close path explicitly opens admission below, before the existing
        # stationary stop proof; predicted planning contacts are never authority.
        evidence=dict(target=self.object,physical_target=self.m.receipt['world']+'::'+self.name,
            binding=copy.deepcopy(self.pile_binding),run_id=s.get('run_id'),iteration=s.get('iteration'),
            sim_ns=s.get('sim_ns'),wall_ns=s.get('wall_ns'),monotonic_raw_ns=s.get('monotonic_raw_ns'),
            certified_set=[],contacts=[],rejected=[],
            admissions=[],active_set=[],frozen=True,checked_samples=0,expired_pairs=[])
        self.pile_certificate=evidence;self.pile_expired=set();self.pile_origin=None
        self.pile_admitting=False;self.pile_live_sequence=False;self.pile_last_iteration=s.get('iteration')
        try:
            validate_sample(s,self.m.receipt,measurement_clock.now_ns()/1e9)
            admitted=self.pile_binding.get('close_goal_terminal_monotonic_raw_ns')
            accepted=self.pile_binding.get('close_goal_accepted_monotonic_raw_ns')
            if (not self.pile_binding.get('close_goal_uuid') or type(admitted) is not int or
                type(accepted) is not int or accepted>admitted or s['monotonic_raw_ns']<admitted):
                raise RuntimeError('pile admission requires measured state after successful owned close')
            evidence['freshness_ms']=(measurement_clock.now_ns()-s['monotonic_raw_ns'])/1e6
            for neighbor,c in sorted(self.pile_contacts(s).items()):self._admit_pile_contact(s,neighbor,c)
            evidence['active_set']=list(evidence['certified_set'])
            evidence['initial_bottom_m']=self.bottom(self.m.object_pose(s,self.name))
        except Exception as exc:
            self._pile_rejection(s,exc);evidence['certified_set']=[];evidence['active_set']=[]
            raise
        return evidence

    def begin_pile_admission(self):
        if self.pile_certificate is not None or self.held:
            raise RuntimeError('pile admission already initialized')
        admitted=self.pile_binding.get('close_goal_terminal_monotonic_raw_ns',0)
        deadline=time.monotonic()+.25
        while self.m.fresh()['monotonic_raw_ns']<admitted:
            if time.monotonic()>=deadline:raise RuntimeError('post-close physical measurement not received')
            time.sleep(.001)
        # Consume buffered pre-admission measurements in the original unheld
        # phase. Only a fresh post-close sample can open the live certificate.
        with self.m.lock:
            for pending in self.m.drain():self.check(pending)
            s=self.m.fresh();self.check(s);self.certify_pile(s)
            self.pile_admitting=True;self.pile_live_sequence=True
            self.pile_certificate['frozen']=False
        return self.pile_certificate

    def begin_pile_separation(self,s):
        if self.pile_certificate is None:raise RuntimeError('missing measured pile certificate')
        self.pile_origin=list(self.m.object_pose(s,self.name));self.pile_height=0.
        self.pile_lift_start_sim_ns=s['sim_ns']

    def check_pile(self,s):
        try:self._check_pile(s)
        except Exception as exc:
            self._pile_rejection(s,exc)
            raise

    def _check_pile(self,s):
        certificate=self.pile_certificate
        if certificate is None or certificate['rejected']:raise RuntimeError('missing valid pile certificate')
        if s['run_id']!=certificate['run_id'] or s['iteration']<certificate['iteration']:
            raise RuntimeError('pile certificate measurement binding changed')
        if self.pile_live_sequence:
            previous=self.pile_last_iteration if s['iteration']!=self.pile_last_iteration else None
            validate_sample(s,self.m.receipt,measurement_clock.now_ns()/1e9,previous)
        contacts=self.pile_contacts(s);certified=set(certificate['certified_set'])
        recontact=set(contacts)&self.pile_expired
        if recontact:raise RuntimeError('pile contact recontact after separation: '+','.join(sorted(recontact)))
        new=set(contacts)-certified
        if new and not self.pile_admitting:
            raise RuntimeError('new uncertified pile contact: '+','.join(sorted(new)))
        for neighbor in sorted(new):self._admit_pile_contact(s,neighbor,contacts[neighbor])
        certified=set(certificate['certified_set'])
        active=certified-self.pile_expired
        lifting=self.phase=='lift' and self.pile_origin is not None and s['sim_ns']>=self.pile_lift_start_sim_ns
        if lifting and active:
            obj=self.m.object_pose(s,self.name);motion=[a-b for a,b in zip(obj[:3],self.pile_origin[:3])]
            if math.hypot(*motion[:2])>.0025 or motion[2]<self.pile_height-1e-9 or motion[2]>.01 or angle(obj,self.pile_origin)>.01:
                raise RuntimeError('initial pile separation leaves certified corridor')
            self.pile_height=motion[2]
        if self.phase in ('transfer','place','opening','released') and active:
            raise RuntimeError('initial pile contacts did not separate before transfer')
        for entry in certificate['contacts']:
            neighbor=entry['neighbor']
            if neighbor not in active:continue
            pose=self.m.object_pose(s,self.pile_objects[neighbor]['name'])
            if box_motion_bound(pose,entry['neighbor_pose'],self.pile_objects[neighbor]['size'])>.0001:
                raise RuntimeError('certified pile neighbor geometry moved '+neighbor)
            valid,geometry=self.pile_geometry(s,neighbor,contacts.get(neighbor,{}).get('points',[]))
            if neighbor in contacts:
                if not valid:raise RuntimeError('measured pile geometry/depth rejected: '+neighbor)
            elif math.isfinite(geometry['separation_m']) and geometry['separation_m']>.0001:
                self.pile_expired.add(neighbor)
                certificate['expired_pairs'].append(dict(neighbor=neighbor,iteration=s['iteration'],sim_ns=s['sim_ns'],geometry=geometry))
            elif not math.isfinite(geometry['depth_m']) or geometry['depth_m']>.0001:
                raise RuntimeError('incomplete/overdeep pile separation geometry '+neighbor)
        certificate['active_set']=sorted(certified-self.pile_expired)
        if s['iteration']!=self.pile_last_iteration:certificate['checked_samples']+=1
        self.pile_last_iteration=s['iteration']

    def identity(self,collision):
        fields=collision.split('::')
        if len(fields)<4 or fields[0]!=self.m.receipt['world']:raise RuntimeError('unknown collision identity '+collision)
        model,link=fields[1:3]
        if 'runtime::'+model in self.pile_objects:
            if collision not in self.pile_collisions:raise RuntimeError('unknown pile collision identity '+collision)
            return self.pile_collisions[collision]
        # The bound support's physics model carries the planning identity recorded in the
        # receipt (sparse world: model pick_support, collision object support_surface_table).
        # Only the exact bound link/collision maps; other geometry of that model stays unmapped.
        for key in ('support_geometry_binding','destination_geometry_binding'):
            binding=self.m.receipt.get(key)
            if (binding and binding.get('object_id') and len(fields)==4 and
                    fields[1:4]==[binding.get('model'),binding.get('link'),binding.get('collision')]):
                return binding['object_id']
        return link if model==self.m.receipt['model'] else ('runtime::'+model if model==self.name else 'workcell::'+model)
    def bottom(self,p):
        return min(p[2]+rotate_vector(p[3:],[x*self.dimensions[0]/2,y*self.dimensions[1]/2,z*self.dimensions[2]/2])[2] for x in (-1,1) for y in (-1,1) for z in (-1,1))
    def check(self,s):
        try:self._check(s)
        except Exception as exc:
            if isinstance(exc,GraspRetentionLoss):exc.sample=copy.deepcopy(s)
            self._pile_rejection(s,exc)
            raise

    def _check(self,s):
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool);fingers=set();support_contact=False
        robot_prefix=self.m.receipt['world']+'::'+self.m.receipt['model']+'::'
        # Detect actual fingertip loss before applying the carried pile rule to
        # this same sample. An accepted OPEN by itself never creates this boundary.
        for contact in s['contacts']:
            a,b=self.identity(contact['a']),self.identity(contact['b'])
            if self.object in (a,b) and {a,b}-{self.object} <= self.touch:
                fingers.update({a,b}-{self.object})
        self._observe_release(s,obj,tool,fingers)
        physically_free=(self.release_candidate is not None and self.ownership=='RELEASING') or self.ownership in ('FREE_SETTLING','RELEASE_CONFIRMED')
        if physically_free:
            previous=self.pile_last_iteration if self.pile_live_sequence and s['iteration']!=self.pile_last_iteration else None
            validate_sample(s,self.m.receipt,measurement_clock.now_ns()/1e9,previous)
            self.pile_last_iteration=s['iteration']
        elif self.held or self.pile_admitting:self.check_pile(s)
        for contact in s['contacts']:
            if not contact.get('points') or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in contact['points']):
                raise RuntimeError('incomplete measured contact')
            a,b=self.identity(contact['a']),self.identity(contact['b']);pair={a,b}
            selected=self.object in pair
            robot=contact['a'].startswith(robot_prefix) or contact['b'].startswith(robot_prefix)
            if selected and physically_free and robot:
                raise RuntimeError('released target recontacted robot/tool: '+a+' / '+b)
            if selected and physically_free and not robot:continue
            if not robot and not (selected and self.held):continue
            if selected and pair-{self.object} <= self.touch and self.phase!='approach':
                fingers.update(pair-{self.object});continue
            if robot and not selected and (a,b) in self.allowed:continue
            if selected and self.held and pair-{self.object} <= set(self.pile_certificate['certified_set'])-self.pile_expired:
                continue
            if selected and self.held and self.support and pair=={self.object,self.support['support_id']}:
                # Physical identities and positions must agree with the certified
                # floor. Normals/depth are checked independently by measured_fcl.
                if any(abs(p[2]-self.support['floor_z'])>.0001 for p in contact['points']):
                    raise RuntimeError('physical support point is not on certified floor')
                support_contact=True;continue
            if (selected and self.held and self.destination and self.phase in DESTINATION_PHASES and
                    pair=={self.object,self.destination['support_id']}):
                # The placed part may rest on the bound destination floor only (same floor-height rule).
                if any(abs(p[2]-self.destination['floor_z'])>.0001 for p in contact['points']):
                    raise RuntimeError('physical destination point is not on the bound floor')
                continue
            raise RuntimeError('unpermitted physical contact: '+a+' / '+b)
        self.fingers=fingers
        if self.held and self.phase not in ('opening','released'):
            self.held.check(tool,obj,fingers,phase=self.phase,sample=s);self.held_samples+=1
            bottom=self.bottom(obj)
            if self.separation:self.separation.check(obj,bottom,support_contact)
            self.min_clearance=min(self.min_clearance,bottom-self.support['floor_z']) if self.support else math.inf
        # Real coupling is measured independently of MoveIt's derived mimics.
        joints=self.m.joints(s)
        for j in self.m.robot.findall('joint'):
            mimic=j.find('mimic')
            if mimic is None:continue
            actual=joints[j.get('name')][0];parent=joints[mimic.get('joint')][0]
            if abs(actual-parent*float(mimic.get('multiplier','1')))>.02:raise RuntimeError('measured mimic coupling error')
        self.last=s
        if self.phase=='released':self.release_samples.append(s)
        self.m.record(self.phase,s)
    def establish(self):
        if not self.pile_admitting:
            raise RuntimeError('live post-close pile admission must precede retention')
        # Finish all provisional checks before freezing. No stale point is
        # recertified, and no pair can be added after this held-state boundary.
        with self.m.lock:
            for pending in self.m.drain():self.check(pending)
            s=self.m.fresh();self.check(s)
            self.pile_admitting=False
            certificate=self.pile_certificate
            certificate['certified_set']=list(certificate['active_set'])
            certificate['frozen']=True
            certificate['freeze']=dict(iteration=s['iteration'],sim_ns=s['sim_ns'],wall_ns=s['wall_ns'],
                monotonic_raw_ns=s['monotonic_raw_ns'],freshness_ms=(measurement_clock.now_ns()-s['monotonic_raw_ns'])/1e6)
            certificate['initial_bottom_m']=self.bottom(self.m.object_pose(s,self.name))
        joints=self.m.joints(s)
        leaders=[j.get('name') for c in self.m.robot.findall('ros2_control') for j in c.findall('joint') if j.find('command_interface') is not None and j.get('name') not in self.arm_names]
        if len(leaders)!=1:raise RuntimeError('unsupported gripper command topology')
        self.leader=leaders[0]
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool)
        if self.ownership!='UNHELD':raise RuntimeError('physical ownership cannot regress to carried')
        required=self.hold_profile.required_contact_links(self.touch,self.fingers) if self.hold_profile else self.touch
        self.held=HeldObject(tool,obj,self.fingers,required,joints[self.leader][0],self.open_position,self.retention_policy)
        self.retention_record=self.held
        self.ownership='CARRIED'
        self.held_start_sim_ns=s['sim_ns'];self.held_samples=1
        self.held_proof=dict(run_id=s['run_id'],target=self.object,
            binding=copy.deepcopy(certificate['binding']),start_iteration=s['iteration'],
            start_sim_ns=s['sim_ns'],start_wall_ns=s['wall_ns'],start_monotonic_raw_ns=s['monotonic_raw_ns'])
        if self.support:self.separation=Separation(obj,self.support['floor_z'])
        return self.held.relative
    def begin_release(self,goal,held_sample):
        if self.ownership!='CARRIED' or self.held is None or self.phase!='opening':
            raise RuntimeError('physical ownership cannot regress or open without a carried target')
        binding=self.pile_binding
        if (not goal.get('accepted') or not isinstance(goal.get('uuid'),str) or len(goal['uuid'])!=32 or
            goal.get('stage') not in ('COMMISSION_RELEASE','EXECUTE_OPEN_GRIPPER') or
            goal.get('run_id')!=self.m.receipt['run_id'] or goal.get('target')!=self.object or
            goal.get('resolution_sha256')!=binding['resolution_sha256'] or
            goal.get('execution_attempt')!=binding['execution_attempt'] or
            goal.get('selected_grasp_index')!=binding.get('selected_grasp_index',0) or
            not isinstance(goal.get('wall_ns'),int) or type(goal.get('monotonic_raw_ns')) is not int):
            raise RuntimeError('owned OPEN goal does not match current grasp attempt')
        validate_sample(held_sample,self.m.receipt,measurement_clock.now_ns()/1e9)
        if held_sample['monotonic_raw_ns']>=goal['monotonic_raw_ns']:
            raise RuntimeError('owned OPEN goal lacks a preceding held measurement')
        self.release_goal=copy.deepcopy(goal)
        self.release_goal.update(terminal_wall_ns=None,terminal_monotonic_raw_ns=None)
        self.release_candidate=None;self.release_transition=None
        self.release_start_joint=self.m.joints(held_sample)[self.leader][0]
        self.ownership='RELEASING'

    def _observe_release(self,s,obj,tool,fingers):
        if self.ownership!='RELEASING' or self.release_candidate is not None:return
        goal=self.release_goal
        if s.get('run_id')!=goal['run_id'] or s.get('monotonic_raw_ns',0)<=goal['monotonic_raw_ns']:
            return
        if self.held.required.intersection(fingers):return
        joint=self.m.joints(s)[self.leader][0]
        relative=compose_pose(inverse_pose(tool),obj)
        displacement=math.dist(relative[:3],self.held.relative[:3])
        if self.release_start_joint-joint<=.001 or displacement<=.000001:return
        validate_sample(s,self.m.receipt,measurement_clock.now_ns()/1e9,self.pile_last_iteration if self.pile_live_sequence and s['iteration']!=self.pile_last_iteration else None)
        self.release_candidate=dict(iteration=s['iteration'],sim_ns=s['sim_ns'],wall_ns=s['wall_ns'],
            monotonic_raw_ns=s['monotonic_raw_ns'],
            run_id=s['run_id'],target=self.object,gripper_position_rad=joint,
            relative_translation_m=displacement,required_finger_contacts_absent=True,
            pose_source=copy.deepcopy(s.get('pose_source')))

    def terminal_release(self,uuid,status,moveit_code,terminal_wall_ns,terminal_monotonic_raw_ns):
        if self.ownership!='RELEASING' or not self.release_goal or uuid!=self.release_goal['uuid']:
            raise RuntimeError('owned OPEN terminal goal mismatch')
        if (status!=4 or moveit_code!=1 or type(terminal_monotonic_raw_ns) is not int or
                terminal_monotonic_raw_ns<self.release_goal['monotonic_raw_ns']):
            raise RuntimeError('owned OPEN did not reach successful terminal state')
        self.release_goal.update(terminal_status=status,moveit_code=moveit_code,
                                 terminal_wall_ns=terminal_wall_ns,terminal_monotonic_raw_ns=terminal_monotonic_raw_ns)

    def release_ready(self):
        if self.ownership!='RELEASING':
            raise RuntimeError('physical release has not been measured')
        goal=self.release_goal
        if goal.get('terminal_status')!=4 or goal.get('moveit_code')!=1 or not goal.get('terminal_monotonic_raw_ns'):
            raise RuntimeError('owned OPEN successful terminal state missing')
        s=self.checked_current()
        if self.release_candidate is None:
            raise RuntimeError('physical release has not been measured')
        if s['monotonic_raw_ns']<goal['terminal_monotonic_raw_ns']:
            raise RuntimeError('post-terminal measured release sample missing')
        if s['run_id']!=goal['run_id'] or s['sim_ns']<self.release_candidate['sim_ns'] or self.fingers:
            raise RuntimeError('physical release current measurement changed')
        joint=self.m.joints(s)[self.leader][0]
        if abs(joint-self.open_position)>.01:
            raise RuntimeError('physical release gripper did not open')
        return s

    def finish_release(self,detachment):
        if self.ownership!='RELEASING' or self.release_candidate is None:
            raise RuntimeError('physical release has not been measured')
        goal=self.release_goal
        if goal.get('terminal_status')!=4 or goal.get('moveit_code')!=1 or not goal.get('terminal_monotonic_raw_ns'):
            raise RuntimeError('owned OPEN successful terminal state missing')
        if (detachment.get('held') is not False or detachment.get('attached_ids') or
            detachment.get('acm_restored') is not True or detachment.get('measured_geometry_matches') is not True):
            raise RuntimeError('physical release cannot retain a planning attachment')
        if detachment.get('run_id') not in (None,goal['run_id']) or detachment.get('target') not in (None,self.object):
            raise RuntimeError('physical release planning attachment identity changed')
        s=self.release_ready()
        evidence=dict(run_id=s['run_id'],target=self.object,binding=copy.deepcopy(self.pile_binding),
            goal=copy.deepcopy(goal),separation=copy.deepcopy(self.release_candidate),
            detachment=copy.deepcopy(detachment),transition_sim_ns=s['sim_ns'],
            transition_iteration=s['iteration'],state='FREE_SETTLING')
        self.release_transition=copy.deepcopy(evidence)
        self.ownership='FREE_SETTLING';self.held=None;self.separation=None
        self.planning_attached=False;self.phase='released'
        return evidence

    def confirm_settled_release(self,settling):
        if self.ownership!='FREE_SETTLING' or not self.release_transition:
            raise RuntimeError('free settling transition missing')
        evidence=dict(copy.deepcopy(self.release_transition),**settling,state='RELEASE_CONFIRMED')
        self.ownership='RELEASE_CONFIRMED'
        return evidence

    def checked_current(self):
        # Keep latest sampling and queue consumption in one acquisition lock so
        # a newer snapshot cannot skip unvalidated sequential physical samples.
        with self.m.lock:
            for pending in self.m.drain():self.check(pending)
            s=self.m.fresh();self.check(s)
            return s

    def retention_evidence(self,min_duration_ns=1000000000):
        if not self.held or self.held_start_sim_ns is None:raise RuntimeError('physical retention was not established')
        proof=self.held_proof;certificate=self.pile_certificate
        if (not proof or not certificate or certificate.get('frozen') is not True or
            proof['run_id']!=self.m.receipt['run_id'] or proof['run_id']!=certificate['run_id'] or
            proof['target']!=self.object or proof['target']!=certificate['target'] or
            proof['binding']!=self.pile_binding or proof['binding']!=certificate['binding']):
            raise RuntimeError('physical retention attempt binding changed')
        s=self.checked_current()
        duration=s['sim_ns']-self.held_start_sim_ns
        if duration<min_duration_ns:raise RuntimeError('physical retention duration is too short')
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool)
        actual=compose_pose(inverse_pose(tool),obj)
        translation=math.dist(actual[:3],self.held.relative[:3]);rotation=angle(actual,self.held.relative)
        if translation>.002 or rotation>.01:raise RuntimeError('physical retention relative slip exceeds limit')
        joints=self.m.joints(s)
        return dict(copy.deepcopy(proof),duration_sim_ns=duration,samples=self.held_samples,
            contact_links=sorted(self.fingers),required_contact_links=sorted(self.held.required),
            relative_translation_m=translation,relative_rotation_rad=rotation,
            closure_position_rad=joints[self.leader][0],open_position_rad=self.open_position,
            end_iteration=s['iteration'],end_sim_ns=s['sim_ns'],end_wall_ns=s['wall_ns'],
            final_iteration=s['iteration'],final_sim_ns=s['sim_ns'])
    def drain(self):
        self.m.fresh()
        pending=self.m.drain()
        for sample in pending:self.check(sample)
        if self.validate_current:self.validate_current()


def measured_attachment(original,contract,measurements,sample):
    from perceived_object_grasp_execute import attachment_diff
    obj=copy.deepcopy(original)
    relative=compose_pose(inverse_pose(measurements.frame(sample,contract['grasp_frame'])),measurements.object_pose(sample,obj.id.removeprefix('runtime::')))
    obj.header.frame_id=contract['grasp_frame']
    (obj.pose.position.x,obj.pose.position.y,obj.pose.position.z,obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w)=relative
    # MoveIt consumes the world object while applying attached ADD, before
    # processing world updates. An explicit REMOVE would then reject the diff.
    return attachment_diff(obj,contract['grasp_frame'],contract['allowed_touch_links'],remove_world=False)


def require_trial_evidence(path,current_capability):
    if path is None:raise RuntimeError('full cycle requires cancellation, motion telemetry, retention and contact-release evidence')
    records=json.loads(Path(path).read_text())
    expected={'CANCELLATION_TRIAL_PASS','MOTION_TELEMETRY_PASS','STATIONARY_RETENTION_PASS','CONTACT_RELEASE_PASS'}
    if len(records)!=4 or {r.get('result') for r in records}!=expected:
        raise RuntimeError('commissioning prerequisite trials have not all passed')
    capability={r.get('commissioning_capability',{}).get('sha256') for r in records}
    if len(capability)!=1 or None in capability:
        raise RuntimeError('commissioning prerequisite evidence uses inconsistent capability binaries')
    capability_sha=next(iter(capability))
    if current_capability.get('sha256')!=capability_sha:
        raise RuntimeError('current commissioning binary differs from freshly qualified evidence')
    overlays=[((r.get('commissioning_capability',{}).get('moveit_overlay') or {}).get('library') or {}).get('sha256')
              for r in records]
    if any(not isinstance(sha,str) or not sha.strip() for sha in overlays) or len(set(overlays))!=1:
        raise RuntimeError('commissioning prerequisite evidence uses missing or inconsistent MoveIt overlay binaries')
    overlay_sha=overlays[0]
    current_overlay=((current_capability.get('moveit_overlay') or {}).get('library') or {}).get('sha256')
    if current_overlay!=overlay_sha:
        raise RuntimeError('current MoveIt overlay binary differs from freshly qualified evidence')
    executables=[(((r.get('commissioning_capability',{}).get('moveit_overlay') or {}).get('move_group') or {}).get('executable') or {}).get('sha256')
                 for r in records]
    if any(not isinstance(sha,str) or not sha.strip() for sha in executables) or len(set(executables))!=1:
        raise RuntimeError('commissioning prerequisite evidence uses missing or inconsistent MoveGroup executable binaries')
    move_group_sha=executables[0]
    current_executable=(((current_capability.get('moveit_overlay') or {}).get('move_group') or {}).get('executable') or {}).get('sha256')
    if current_executable!=move_group_sha:
        raise RuntimeError('current MoveGroup executable differs from freshly qualified evidence')
    for r in records:
        backend=r.get('motion_backend_identity') or r.get('backend_identity',{})
        if backend.get('backend')!='simulator':raise RuntimeError('commissioning evidence backend identity unproven')
        reconciliation=r.get('measured_reconciliation',{})
        if not reconciliation.get('acm_restored') or reconciliation.get('attached_ids'):
            raise RuntimeError('commissioning evidence restoration unproven')
    cancellation=next(r for r in records if r['result']=='CANCELLATION_TRIAL_PASS')
    require_cancellation_acceptance(cancellation)
    telemetry=next(r for r in records if r['result']=='MOTION_TELEMETRY_PASS').get('motion_telemetry',{})
    if telemetry.get('max_fresh_age_ms',math.inf)>=250 or telemetry.get('max_delivery_ms',math.inf)>=250:
        raise RuntimeError('motion telemetry prerequisite does not satisfy unchanged 250 ms guard')
    retention=next(r for r in records if r['result']=='STATIONARY_RETENTION_PASS').get('stationary_retention',{})
    if retention.get('duration_sim_ns',0)<1000000000 or retention.get('samples',0)<2:
        raise RuntimeError('stationary physical retention prerequisite missing')
    contact=next(r for r in records if r['result']=='CONTACT_RELEASE_PASS')
    if contact.get('verified_lift_clearance_m',0)<.01 or not contact.get('release_evidence',{}).get('settled'):
        raise RuntimeError('physical lift/release prerequisite missing')
    return dict(capability_sha256=capability_sha,moveit_overlay_sha256=overlay_sha,
        move_group_sha256=move_group_sha,results=sorted(expected))


def verify_release(guard,held_sample):
    s=guard.checked_current()
    p=guard.m.object_pose(s,guard.name);old=guard.m.object_pose(held_sample,guard.name)
    joint=guard.m.joints(s)[guard.leader]
    if guard.fingers or abs(joint[0]-guard.open_position)>.01 or old[2]-p[2]<.01:
        raise RuntimeError('physical release/fall not measured')
    # Last one second of measured samples must settle; no stale pose restoration.
    recent=[x for x in guard.release_samples if s['sim_ns']-x['sim_ns']<=1000000000]
    if len(recent)<2 or recent[-1]['sim_ns']-recent[0]['sim_ns']<900000000:
        raise RuntimeError('insufficient release settling measurements')
    if any(math.dist(guard.m.object_pose(x,guard.name)[:3],p[:3])>.001 for x in recent):
        raise RuntimeError('released object has not resettled')
    return guard.confirm_settled_release(dict(open_position_rad=joint[0],fall_m=old[2]-p[2],
        final_pose=p,settled=True,sim_ns=s['sim_ns']))


def verify_settled_release(guard,held_sample):
    """Prove opening + loss of finger contact + one-second physical settling."""
    s=guard.checked_current()
    p=guard.m.object_pose(s,guard.name);old=guard.m.object_pose(held_sample,guard.name)
    joint=guard.m.joints(s)[guard.leader]
    if guard.fingers or abs(joint[0]-guard.open_position)>.01:
        raise RuntimeError('physical release not measured: gripper remains closed or in finger contact')
    recent=[x for x in guard.release_samples if s['sim_ns']-x['sim_ns']<=1000000000]
    if len(recent)<2 or recent[-1]['sim_ns']-recent[0]['sim_ns']<900000000:
        raise RuntimeError('insufficient release settling measurements')
    if any(math.dist(guard.m.object_pose(x,guard.name)[:3],p[:3])>.001 for x in recent):
        raise RuntimeError('released object has not resettled')
    return guard.confirm_settled_release(dict(open_position_rad=joint[0],vertical_change_m=old[2]-p[2],
        final_pose=p,settled=True,sim_ns=s['sim_ns'],samples=len(recent)))


def validate_measured_contacts(response,support,expired,predicate=None,pile_guard=None,destination=None):
    """Use actual MoveIt contact geometry; never synthesize missing physics normals."""
    if response.valid:return
    if not response.contacts:
        raise RuntimeError('measured carried/robot collision or invalid state')
    if (support or destination) and predicate is None:
        from ament_index_python.packages import get_package_prefix
        lib=ctypes.CDLL(str(Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'))
        predicate=lib.workcell_support_contact_valid
        predicate.argtypes=[ctypes.c_char_p]*4+[ctypes.c_double,ctypes.POINTER(ctypes.c_double)];predicate.restype=ctypes.c_bool
    for c in response.contacts:
        types={c.contact_body_1:c.body_type_1,c.contact_body_2:c.body_type_2}
        point=[c.position.x,c.position.y,c.position.z,c.normal.x,c.normal.y,c.normal.z,c.depth]
        if pile_guard and pile_guard.pile_certificate:
            target=pile_guard.object
            neighbor=next(iter(set(types)-{target}),None)
            allowed=set(pile_guard.pile_certificate['certified_set'])-pile_guard.pile_expired
            if (types.get(target)==2 and types.get(neighbor)==1 and neighbor in allowed and
                pile_guard.pile_predicate(target.encode(),neighbor.encode(),c.contact_body_1.encode(),
                    c.contact_body_2.encode(),(ctypes.c_double*7)(*point))):continue
        if destination and types.get(destination['object_id'])==2 and types.get(destination['support_id'])==1:
            # Carried part resting on the bound destination floor (place/opening/released only).
            if predicate(destination['object_id'].encode(),destination['support_id'].encode(),c.contact_body_1.encode(),
                         c.contact_body_2.encode(),destination['floor_z'],(ctypes.c_double*7)(*point)):continue
            raise RuntimeError('measured FCL destination contact rejected: pair/normal/depth/position')
        if not support or expired:
            raise RuntimeError('measured carried/robot collision or invalid state')
        if types.get(support['object_id'])!=2 or types.get(support['support_id'])!=1:
            raise RuntimeError('measured support body types contradict carried state')
        if not predicate(support['object_id'].encode(),support['support_id'].encode(),c.contact_body_1.encode(),c.contact_body_2.encode(),support['floor_z'],(ctypes.c_double*7)(*point)):
            raise RuntimeError('measured FCL support contact rejected: pair/normal/depth/position')


def validate_publisher(publishers):
    if len(publishers)!=1 or publishers[0].node_name!='ros_gz_bridge' or publishers[0].node_namespace!='/':
        raise RuntimeError('ambiguous measurement publisher')


def validate_controller_cancellation(controllers):
    if not controllers or any(not c.get('executing_before_cancel') or not c.get('uuid') or
            c.get('status')!=5 or c.get('result_wall_ns') is None for c in controllers.values()):
        raise RuntimeError('controller interruption unverified: exact active goal must finish CANCELED')


class ControllerCancellationAudit:
    """Read-only status/result observer of the controllers already owned by TEM."""
    def __init__(self,node,controllers,joint_names):
        from action_msgs.msg import GoalStatusArray
        from control_msgs.action import FollowJointTrajectory
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.node=node;self.states={};self.baseline={};self.results={};self.events=[];self.subs=[];self.clients={}
        self.service_type=FollowJointTrajectory.Impl.GetResultService
        requested=set(joint_names);covered=set()
        for c in controllers:
            owned={x.rsplit('/',1)[0] for x in c.claimed_interfaces}
            if c.state!='active' or not requested.intersection(owned):continue
            if covered.intersection(owned):raise RuntimeError('ambiguous controller ownership')
            covered.update(owned&requested);name=c.name;self.states[name]={}
            action='/'+name.strip('/')+'/follow_joint_trajectory'
            self.subs.append(node.create_subscription(GoalStatusArray,action+'/_action/status',
                lambda msg,n=name:self.update(n,msg),QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            self.clients[name]=node.create_client(self.service_type,action+'/_action/get_result')
        if covered!=requested:raise RuntimeError('controller audit missing requested joint ownership')
        if not all(c.wait_for_service(timeout_sec=3) for c in self.clients.values()):raise RuntimeError('controller result service unavailable')
    def update(self,name,msg):
        wall=time.time_ns();raw=measurement_clock.now_ns()
        for s in msg.status_list:
            uid=bytes(s.goal_info.goal_id.uuid).hex()
            self.states[name][uid]=dict(status=s.status,uuid=list(s.goal_info.goal_id.uuid))
            self.events.append(dict(controller=name,uuid=uid,status=s.status,receive_wall_ns=wall,
                receive_monotonic_raw_ns=raw,
                accepted_sim_ns=s.goal_info.stamp.sec*1000000000+s.goal_info.stamp.nanosec))
    def arm(self):
        self.baseline={n:set(states) for n,states in self.states.items()}
        if any(s['status'] in (1,2,3) for states in self.states.values() for s in states.values()):
            raise RuntimeError('controller already executing before owned goal')
    def before_cancel(self):
        for name,states in self.states.items():
            current=set(states)-self.baseline[name]
            if len(current)!=1:raise RuntimeError('ambiguous/missing active controller goal')
            uid=current.pop();s=states[uid]
            self.results[name]=dict(uuid=uid,executing_before_cancel=s['status']==2,status_before_cancel=s['status'])
        if not all(c['executing_before_cancel'] for c in self.results.values()):raise RuntimeError('controller completed before cancel')
    def collect(self):
        import rclpy
        for name,e in self.results.items():
            req=self.service_type.Request();req.goal_id.uuid=list(bytes.fromhex(e['uuid']))
            future=self.clients[name].call_async(req)
            rclpy.spin_until_future_complete(self.node,future,timeout_sec=3)
            if not future.done() or future.result() is None:raise RuntimeError('controller terminal result missing')
            result=future.result();e.update(status=result.status,error_code=result.result.error_code,
                error_string=result.result.error_string,result_wall_ns=time.time_ns(),
                result_monotonic_raw_ns=measurement_clock.now_ns())
            terminal=next((x for x in self.events if x['uuid']==e['uuid'] and x['status'] in (4,5,6)),None)
            if terminal:e.update(terminal_status_receive_wall_ns=terminal['receive_wall_ns'],
                                 terminal_status_receive_monotonic_raw_ns=terminal['receive_monotonic_raw_ns'])
        validate_controller_cancellation(self.results)
        return self.results


def cancellation_metrics(summary,measurements,samples):
    # Executor and simulator stamps are compared on CLOCK_MONOTONIC_RAW only.
    request=summary['cancellation_request']['monotonic_raw_ns'];anchor=summary['cancel_measurement']
    names=set(summary['cancellation_motion']['accepted']['joints'])
    origin=measurements.joints(anchor);previous=origin;travel={n:0. for n in names};extent={n:0. for n in names}
    for sample in samples:
        joints=measurements.joints(sample)
        for n in names:
            travel[n]+=abs(joints[n][0]-previous[n][0]);extent[n]=max(extent[n],abs(joints[n][0]-origin[n][0]))
        previous=joints
    # Search all retained post-request samples, including the controller-result
    # wait, for the earliest subsequently sustained stationary sequence. The
    # separately accepted stop window still begins after terminal confirmation.
    earliest=StopWindow(set(origin));first=summary['stopped_window']['samples'][0]
    for sample in samples:
        if sample['monotonic_raw_ns']<request:continue
        if earliest.observe(sample,measurements.joints(sample)):
            first=earliest.samples[0];break
    return dict(cancel_to_measured_stop_ms=(first['monotonic_raw_ns']-request)/1e6,
        cancel_to_controller_result_ms={n:(c.get('terminal_status_receive_monotonic_raw_ns',c['result_monotonic_raw_ns'])-request)/1e6
            for n,c in summary.get('controller_cancellation',{}).items()},
        cancel_snapshot_age_ms=(request-anchor['monotonic_raw_ns'])/1e6,
        additional_joint_travel_rad=travel,max_joint_displacement_after_cancel_rad=extent,
        measurement_samples=len(samples),stop_window_confirmed_wall_ns=time.time_ns())
