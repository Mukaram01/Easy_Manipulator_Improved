#!/usr/bin/env python3
"""Measured guards used only by the existing executor's explicit commissioning mode.

No trajectory producer, simulator mutation, or autonomous recovery lives here.
"""
import copy
import ctypes
import json
import math
from pathlib import Path
import time
import threading
import queue
import xml.etree.ElementTree as ET
from perceived_object_grasp_plan import compose_pose,inverse_pose,quaternion_from_rpy,rotate_vector


def angle(a,b):
    return 2*math.acos(min(1.,abs(sum(x*y for x,y in zip(a[3:],b[3:])))))


def validate_sample(s,receipt,now,previous=None):
    if (s.get('run_id')!=receipt['run_id'] or s.get('pid')!=receipt['pid'] or s.get('error') or
        not 0<=now-s.get('wall_ns',0)/1e9<=.25 or not s.get('poses') or not s.get('joints') or
        not s.get('collisions') or not isinstance(s.get('contacts'),list) or
        not isinstance(s.get('sim_ns'),int) or (previous is not None and s['iteration']!=previous+1)):
        raise RuntimeError('SIMULATOR_TELEMETRY_INVALID: stale, missing, contradictory or skipped measurement')
    for p in s['poses'].values():
        if len(p)!=7 or not all(math.isfinite(x) for x in p) or abs(sum(x*x for x in p[3:])-1)>.00001:
            raise RuntimeError('invalid measured pose')
    if any(len(v)!=2 or not all(math.isfinite(x) for x in v) for v in s['joints'].values()):
        raise RuntimeError('invalid measured joint')


class Separation:
    def __init__(self,origin,floor):
        self.origin=origin;self.floor=floor;self.expired=False;self.height=0.
    def check(self,pose,bottom,support_contact):
        if self.expired:
            if support_contact:raise RuntimeError('support exception expired: recontact')
            return
        motion=[a-b for a,b in zip(pose[:3],self.origin[:3])]
        if (math.hypot(*motion[:2])>.0025 or motion[2]<self.height-1e-9 or motion[2]>.01 or angle(pose,self.origin)>.01):
            raise RuntimeError('initial support separation leaves certified corridor')
        self.height=motion[2]
        if not support_contact and bottom>self.floor+.0001:self.expired=True


class HeldObject:
    def __init__(self,tool,obj,fingers,required,closure,open_position):
        self.required=set(required)
        if len(self.required)<2 or not self.required.issubset(fingers) or closure<=open_position+.001:
            raise RuntimeError('closure lacks measured opposing finger/object contact')
        self.relative=compose_pose(inverse_pose(tool),obj)
    def check(self,tool,obj,fingers):
        actual=compose_pose(inverse_pose(tool),obj)
        if (not self.required.issubset(fingers) or math.dist(actual[:3],self.relative[:3])>.002 or angle(actual,self.relative)>.01):
            raise RuntimeError('measured grasp retention lost: contact/slip')


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
    return dict(iteration=sample['iteration'],sim_ns=sample['sim_ns'],wall_ns=sample['wall_ns'],joints=selected)


class CancellationMotion:
    """Evidence of actual commanded-arm movement after owned goal acceptance."""
    def __init__(self,goal_uuid,sample,joints,names,accepted_wall_ns=None):
        self.names=set(names)
        self.accepted_wall_ns=sample['wall_ns'] if accepted_wall_ns is None else accepted_wall_ns
        self.evidence=dict(goal_uuid=goal_uuid,accepted=_joint_measurement(sample,joints,self.names),movement=None)
    def observe(self,sample,joints):
        measured=_joint_measurement(sample,joints,self.names)
        start=self.evidence['accepted']
        if (measured['iteration']<=start['iteration'] or measured['sim_ns']<=start['sim_ns'] or
                measured['wall_ns']<=self.accepted_wall_ns):return False
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


def telemetry_metrics(timing,armed_wall_ns=None):
    """Summarize the existing authoritative physics stream without changing it."""
    receive=[e for e in timing if e.get('event')=='receive' and e.get('iteration') is not None
             and not e.get('error') and (armed_wall_ns is None or e.get('receive_wall_ns',0)>=armed_wall_ns)]
    fresh=[e for e in timing if e.get('event')=='fresh' and e.get('iteration') is not None
           and not e.get('error') and (armed_wall_ns is None or e.get('read_wall_ns',0)>=armed_wall_ns)]
    if len(receive)<2 or not fresh:raise RuntimeError('insufficient motion telemetry timing evidence')
    receive.sort(key=lambda e:e['receive_wall_ns'])
    if any(b['iteration']!=a['iteration']+1 for a,b in zip(receive,receive[1:])):
        raise RuntimeError('telemetry timing contains an iteration gap')
    source_intervals=[(b['source_wall_ns']-a['source_wall_ns'])/1e6 for a,b in zip(receive,receive[1:])]
    receive_intervals=[(b['receive_wall_ns']-a['receive_wall_ns'])/1e6 for a,b in zip(receive,receive[1:])]
    delivery=[(e['receive_wall_ns']-e['source_wall_ns'])/1e6 for e in receive]
    publish=[(e['publish_wall_ns']-e['source_wall_ns'])/1e6 for e in receive
             if e.get('publish_wall_ns') is not None]
    callback=[e['callback_ns']/1e6 for e in receive if e.get('callback_ns') is not None]
    serialization=[e['source_serialization_ns']/1e6 for e in receive
                   if e.get('source_serialization_ns') is not None]
    ages=[e['age_ns']/1e6 for e in fresh if e.get('age_ns') is not None]
    lock_wait=[e['lock_wait_ns']/1e6 for e in fresh if e.get('lock_wait_ns') is not None]
    if any(v<0 for v in delivery+publish+ages):
        raise RuntimeError('telemetry wall-clock ordering is contradictory')
    sim_span=receive[-1].get('sim_ns',0)-receive[0].get('sim_ns',0)
    wall_span=receive[-1]['source_wall_ns']-receive[0]['source_wall_ns']
    result=dict(samples=len(receive),
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


class Measurements:
    def __init__(self,node,receipt,log):
        import yaml
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
        from simulator_backend import verify_receipt_process
        self.receipt_path=Path(receipt);self.receipt=verify_receipt_process(receipt)
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        self.node=Node('simulator_measurement_acquisition',context=node.context)
        self.executor=SingleThreadedExecutor(context=node.context);self.executor.add_node(self.node)
        self.lock=threading.RLock();self.latest=None;self.pending=[];self.error=None;self.previous=None
        self.timing=[];self.armed_wall_ns=None;self.log_path=Path(log)
        self.log=Path(log).open('w');self.armed=False
        self.log_queue=queue.Queue(maxsize=10000)
        self.sub=self.node.create_subscription(String,f"/world/{self.receipt['world']}/workcell_measurements",self.update,
            QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST))
        self.loader=yaml.CSafeLoader
        self.robot=ET.parse(self.receipt_path.parent/'robot.urdf').getroot()
        self.parents={j.find('child').get('link'):j for j in self.robot.findall('joint')}
        self.writer=threading.Thread(target=self._write,name='measurement_evidence');self.writer.start()
        self.acquisition=threading.Thread(target=self.executor.spin,name='measurement_acquisition');self.acquisition.start()
    def update(self,msg):
        received=time.time_ns();started=time.monotonic_ns();s={}
        with self.lock:
            try:
                s=json.loads(msg.data)
                validate_sample(s,self.receipt,time.time(),self.previous if self.armed else None)
                if self.latest and s['sim_ns']<=self.latest['sim_ns']:raise RuntimeError('simulation time stopped/reversed')
                self.previous=s['iteration'];self.latest=s
                if self.armed:
                    if len(self.pending)>=10000:raise RuntimeError('measurement consumer queue overflow')
                    self.pending.append(s)
            except Exception as exc:
                if self.error is None:self.error=exc
            self.timing.append(dict(event='receive',iteration=s.get('iteration'),sim_ns=s.get('sim_ns'),
                source_wall_ns=s.get('wall_ns'),publish_wall_ns=s.get('publish_wall_ns'),
                source_serialization_ns=s.get('serialization_ns'),receive_wall_ns=received,
                callback_ns=time.monotonic_ns()-started,queue_depth=len(self.pending),
                error=str(self.error) if self.error else None))
    def fresh(self):
        waiting=time.monotonic_ns()
        with self.lock:
            entered=time.monotonic_ns();now_ns=time.time_ns()
            if self.error:
                self.timing.append(dict(event='fresh',iteration=None,read_wall_ns=now_ns,
                    lock_wait_ns=entered-waiting,error=str(self.error)));raise self.error
            if self.latest is None:
                self.timing.append(dict(event='fresh',iteration=None,read_wall_ns=now_ns,
                    lock_wait_ns=entered-waiting,error='no simulator measurement'))
                raise RuntimeError('no simulator measurement')
            age_ns=now_ns-self.latest['wall_ns']
            try:validate_sample(self.latest,self.receipt,now_ns/1e9)
            except Exception as exc:
                self.timing.append(dict(event='fresh',iteration=self.latest.get('iteration'),read_wall_ns=now_ns,
                    age_ns=age_ns,lock_wait_ns=entered-waiting,error=str(exc)));raise
            self.timing.append(dict(event='fresh',iteration=self.latest['iteration'],read_wall_ns=now_ns,
                age_ns=age_ns,lock_wait_ns=entered-waiting,error=None))
            return self.latest
    def drain(self):
        with self.lock:
            self.fresh()
            pending,self.pending=self.pending,[]
            return pending
    def arm(self):
        from simulator_backend import verify_receipt_process
        verify_receipt_process(self.receipt_path)
        publishers=self.node.get_publishers_info_by_topic(f"/world/{self.receipt['world']}/workcell_measurements")
        validate_publisher(publishers)
        with self.lock:
            self.fresh();self.armed_wall_ns=time.time_ns();self.armed=True;self.pending=[]
    def metrics(self):
        return telemetry_metrics(self.timing,self.armed_wall_ns)
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
    def object_pose(self,s,name):return s['poses'][self.receipt['world']+'::'+name]
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
            try:self.log.write(json.dumps(dict(phase=phase,measurement=s))+'\n')
            except Exception as exc:
                with self.lock:
                    if self.error is None:self.error=exc
            with self.lock:
                self.timing.append(dict(event='consume_log',iteration=s['iteration'],consume_wall_ns=wall,
                    logging_ns=time.monotonic_ns()-begin,queue_depth=self.log_queue.qsize()))
    def close(self):
        self.executor.shutdown();self.acquisition.join()
        self.node.destroy_node()
        self.log_queue.put(None);self.writer.join();self.log.close()
        self.log_path.with_suffix('.timing.json').write_text(json.dumps(self.timing))


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
        self.fingers=set();self.release_samples=[];self.held_samples=0;self.held_start_sim_ns=None
        self.pile_objects={};self.pile_collisions={};self.pile_certificate=None
        self.pile_expired=set();self.pile_origin=None;self.pile_height=0.;self.planning_attached=False
        self.library=library

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
            if neighbor not in self.pile_objects:raise RuntimeError('uncertified target contact identity '+neighbor)
            if neighbor in result:raise RuntimeError('ambiguous duplicate pile contact '+neighbor)
            if not c.get('points') or any(len(p)!=3 or not all(math.isfinite(x) for x in p) for p in c['points']):
                raise RuntimeError('incomplete measured pile contact')
            result[neighbor]=c
        return result

    def certify_pile(self,s):
        # Admission occurs only on fresh post-close measurements. A resolution's
        # predicted contact set is never reused as physical execution authority.
        evidence=dict(target=self.object,physical_target=self.m.receipt['world']+'::'+self.name,
            binding=copy.deepcopy(self.pile_binding),run_id=s.get('run_id'),iteration=s.get('iteration'),
            sim_ns=s.get('sim_ns'),wall_ns=s.get('wall_ns'),certified_set=[],contacts=[],rejected=[])
        self.pile_certificate=evidence;self.pile_expired=set();self.pile_origin=None
        try:
            validate_sample(s,self.m.receipt,time.time())
            admitted=self.pile_binding.get('close_goal_terminal_wall_ns')
            accepted=self.pile_binding.get('close_goal_accepted_wall_ns')
            if (not self.pile_binding.get('close_goal_uuid') or not isinstance(admitted,int) or
                not isinstance(accepted,int) or accepted>admitted or s['wall_ns']<admitted):
                raise RuntimeError('pile admission requires measured state after successful owned close')
            evidence['freshness_ms']=(time.time_ns()-s['wall_ns'])/1e6
            for neighbor,c in sorted(self.pile_contacts(s).items()):
                valid,geometry=self.pile_geometry(s,neighbor,c['points'])
                entry=dict(neighbor=neighbor,physical_pair=[c['a'],c['b']],points=c['points'],
                    target_pose=self.m.object_pose(s,self.name),
                    neighbor_pose=self.m.object_pose(s,self.pile_objects[neighbor]['name']),geometry=geometry)
                evidence['contacts'].append(entry)
                if not valid:raise RuntimeError('measured pile geometry/depth rejected: '+neighbor)
                evidence['certified_set'].append(neighbor)
            evidence['initial_bottom_m']=self.bottom(self.m.object_pose(s,self.name))
            evidence['checked_samples']=0;evidence['expired_pairs']=[]
        except Exception as exc:
            evidence['rejected'].append(str(exc));evidence['certified_set']=[]
            raise
        return evidence

    def begin_pile_separation(self,s):
        if self.pile_certificate is None:raise RuntimeError('missing measured pile certificate')
        self.pile_origin=list(self.m.object_pose(s,self.name));self.pile_height=0.
        self.pile_lift_start_sim_ns=s['sim_ns']

    def check_pile(self,s):
        certificate=self.pile_certificate
        if certificate is None or certificate['rejected']:raise RuntimeError('missing valid pile certificate')
        if s['run_id']!=certificate['run_id'] or s['iteration']<certificate['iteration']:
            raise RuntimeError('pile certificate measurement binding changed')
        contacts=self.pile_contacts(s);certified=set(certificate['certified_set'])
        if set(contacts)-certified:raise RuntimeError('new uncertified pile contact')
        if set(contacts)&self.pile_expired:raise RuntimeError('pile contact recontact after separation')
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
            radius=math.sqrt(sum(x*x for x in self.pile_objects[neighbor]['size']))/2
            if math.dist(pose[:3],entry['neighbor_pose'][:3])+radius*angle(pose,entry['neighbor_pose'])>.0001:
                raise RuntimeError('certified pile neighbor geometry moved '+neighbor)
            valid,geometry=self.pile_geometry(s,neighbor,contacts.get(neighbor,{}).get('points',[]))
            if neighbor in contacts:
                if not valid:raise RuntimeError('measured pile geometry/depth rejected: '+neighbor)
            elif math.isfinite(geometry['separation_m']) and geometry['separation_m']>.0001:
                self.pile_expired.add(neighbor)
                certificate['expired_pairs'].append(dict(neighbor=neighbor,iteration=s['iteration'],sim_ns=s['sim_ns'],geometry=geometry))
            elif not math.isfinite(geometry['depth_m']) or geometry['depth_m']>.0001:
                raise RuntimeError('incomplete/overdeep pile separation geometry '+neighbor)
        certificate['checked_samples']+=1

    def identity(self,collision):
        fields=collision.split('::')
        if len(fields)<4 or fields[0]!=self.m.receipt['world']:raise RuntimeError('unknown collision identity '+collision)
        model,link=fields[1:3]
        if 'runtime::'+model in self.pile_objects:
            if collision not in self.pile_collisions:raise RuntimeError('unknown pile collision identity '+collision)
            return self.pile_collisions[collision]
        return link if model==self.m.receipt['model'] else ('runtime::'+model if model==self.name else 'workcell::'+model)
    def bottom(self,p):
        return min(p[2]+rotate_vector(p[3:],[x*self.dimensions[0]/2,y*self.dimensions[1]/2,z*self.dimensions[2]/2])[2] for x in (-1,1) for y in (-1,1) for z in (-1,1))
    def check(self,s):
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool);fingers=set();support_contact=False
        if self.held:self.check_pile(s)
        robot_prefix=self.m.receipt['world']+'::'+self.m.receipt['model']+'::'
        for contact in s['contacts']:
            if not contact.get('points') or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in contact['points']):
                raise RuntimeError('incomplete measured contact')
            a,b=self.identity(contact['a']),self.identity(contact['b']);pair={a,b}
            selected=self.object in pair
            robot=contact['a'].startswith(robot_prefix) or contact['b'].startswith(robot_prefix)
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
            raise RuntimeError('unpermitted physical contact: '+a+' / '+b)
        self.fingers=fingers
        if self.held and self.phase not in ('opening','released'):
            self.held.check(tool,obj,fingers);self.held_samples+=1
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
        admitted=self.pile_binding.get('close_goal_terminal_wall_ns',0)
        deadline=time.monotonic()+.25
        while self.m.fresh()['wall_ns']<admitted:
            if time.monotonic()>=deadline:raise RuntimeError('post-close physical measurement not received')
            time.sleep(.001)
        # Consume pre-admission samples under their original (not-held) phase;
        # lock acquisition briefly so none can later masquerade as held data.
        with self.m.lock:
            for pending in self.m.drain():self.check(pending)
            s=self.m.fresh();self.check(s)
            self.certify_pile(s)
        joints=self.m.joints(s)
        leaders=[j.get('name') for c in self.m.robot.findall('ros2_control') for j in c.findall('joint') if j.find('command_interface') is not None and j.get('name') not in self.arm_names]
        if len(leaders)!=1:raise RuntimeError('unsupported gripper command topology')
        self.leader=leaders[0]
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool)
        self.held=HeldObject(tool,obj,self.fingers,self.touch,joints[self.leader][0],self.open_position)
        self.held_start_sim_ns=s['sim_ns'];self.held_samples=1
        if self.support:self.separation=Separation(obj,self.support['floor_z'])
        return self.held.relative
    def retention_evidence(self,min_duration_ns=1000000000):
        if not self.held or self.held_start_sim_ns is None:raise RuntimeError('physical retention was not established')
        s=self.m.fresh();self.check(s)
        duration=s['sim_ns']-self.held_start_sim_ns
        if duration<min_duration_ns:raise RuntimeError('physical retention duration is too short')
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool)
        actual=compose_pose(inverse_pose(tool),obj)
        translation=math.dist(actual[:3],self.held.relative[:3]);rotation=angle(actual,self.held.relative)
        if translation>.002 or rotation>.01:raise RuntimeError('physical retention relative slip exceeds limit')
        joints=self.m.joints(s)
        return dict(duration_sim_ns=duration,samples=self.held_samples,
            contact_links=sorted(self.fingers),required_contact_links=sorted(self.held.required),
            relative_translation_m=translation,relative_rotation_rad=rotation,
            closure_position_rad=joints[self.leader][0],open_position_rad=self.open_position,
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
    return attachment_diff(obj,contract['grasp_frame'],contract['allowed_touch_links'])


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
    s=guard.m.fresh();guard.check(s)
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
    return dict(open_position_rad=joint[0],fall_m=old[2]-p[2],final_pose=p,settled=True,sim_ns=s['sim_ns'])


def verify_settled_release(guard,held_sample):
    """Prove opening + loss of finger contact + one-second physical settling."""
    s=guard.m.fresh();guard.check(s)
    p=guard.m.object_pose(s,guard.name);old=guard.m.object_pose(held_sample,guard.name)
    joint=guard.m.joints(s)[guard.leader]
    if guard.fingers or abs(joint[0]-guard.open_position)>.01:
        raise RuntimeError('physical release not measured: gripper remains closed or in finger contact')
    recent=[x for x in guard.release_samples if s['sim_ns']-x['sim_ns']<=1000000000]
    if len(recent)<2 or recent[-1]['sim_ns']-recent[0]['sim_ns']<900000000:
        raise RuntimeError('insufficient release settling measurements')
    if any(math.dist(guard.m.object_pose(x,guard.name)[:3],p[:3])>.001 for x in recent):
        raise RuntimeError('released object has not resettled')
    return dict(open_position_rad=joint[0],vertical_change_m=old[2]-p[2],
        final_pose=p,settled=True,sim_ns=s['sim_ns'],samples=len(recent))


def validate_measured_contacts(response,support,expired,predicate=None,pile_guard=None):
    """Use actual MoveIt contact geometry; never synthesize missing physics normals."""
    if response.valid:return
    if not response.contacts:
        raise RuntimeError('measured carried/robot collision or invalid state')
    if support and predicate is None:
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
        wall=time.time_ns()
        for s in msg.status_list:
            uid=bytes(s.goal_info.goal_id.uuid).hex()
            self.states[name][uid]=dict(status=s.status,uuid=list(s.goal_info.goal_id.uuid))
            self.events.append(dict(controller=name,uuid=uid,status=s.status,receive_wall_ns=wall,
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
                error_string=result.result.error_string,result_wall_ns=time.time_ns())
            terminal=next((x for x in self.events if x['uuid']==e['uuid'] and x['status'] in (4,5,6)),None)
            if terminal:e['terminal_status_receive_wall_ns']=terminal['receive_wall_ns']
        validate_controller_cancellation(self.results)
        return self.results


def cancellation_metrics(summary,measurements,samples):
    request=summary['cancellation_request']['wall_ns'];anchor=summary['cancel_measurement']
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
        if sample['wall_ns']<request:continue
        if earliest.observe(sample,measurements.joints(sample)):
            first=earliest.samples[0];break
    return dict(cancel_to_measured_stop_ms=(first['wall_ns']-request)/1e6,
        cancel_to_controller_result_ms={n:(c.get('terminal_status_receive_wall_ns',c['result_wall_ns'])-request)/1e6
            for n,c in summary.get('controller_cancellation',{}).items()},
        cancel_snapshot_age_ms=(request-anchor['wall_ns'])/1e6,
        additional_joint_travel_rad=travel,max_joint_displacement_after_cancel_rad=extent,
        measurement_samples=len(samples),stop_window_confirmed_wall_ns=time.time_ns())
