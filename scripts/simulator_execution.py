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
        self.timing=[];self.log_path=Path(log)
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
            self.timing.append(dict(event='receive',iteration=s.get('iteration'),source_wall_ns=s.get('wall_ns'),
                publish_wall_ns=s.get('publish_wall_ns'),source_serialization_ns=s.get('serialization_ns'),
                receive_wall_ns=received,callback_ns=time.monotonic_ns()-started,queue_depth=len(self.pending),
                error=str(self.error) if self.error else None))
    def fresh(self):
        with self.lock:
            if self.error:raise self.error
            if self.latest is None:raise RuntimeError('no simulator measurement')
            validate_sample(self.latest,self.receipt,time.time())
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
            self.fresh();self.armed=True;self.pending=[]
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
        self.fingers=set();self.release_samples=[]
    def identity(self,collision):
        fields=collision.split('::')
        if len(fields)<4 or fields[0]!=self.m.receipt['world']:raise RuntimeError('unknown collision identity '+collision)
        model,link=fields[1:3]
        return link if model==self.m.receipt['model'] else ('runtime::'+model if model==self.name else 'workcell::'+model)
    def bottom(self,p):
        return min(p[2]+rotate_vector(p[3:],[x*self.dimensions[0]/2,y*self.dimensions[1]/2,z*self.dimensions[2]/2])[2] for x in (-1,1) for y in (-1,1) for z in (-1,1))
    def check(self,s):
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool);fingers=set();support_contact=False
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
            if selected and self.held and self.support and pair=={self.object,self.support['support_id']}:
                # Physical identities and positions must agree with the certified
                # floor. Normals/depth are checked independently by measured_fcl.
                if any(abs(p[2]-self.support['floor_z'])>.0001 for p in contact['points']):
                    raise RuntimeError('physical support point is not on certified floor')
                support_contact=True;continue
            raise RuntimeError('unpermitted physical contact: '+a+' / '+b)
        self.fingers=fingers
        if self.held and self.phase not in ('opening','released'):
            self.held.check(tool,obj,fingers)
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
        s=self.m.fresh();self.check(s)
        joints=self.m.joints(s)
        leaders=[j.get('name') for c in self.m.robot.findall('ros2_control') for j in c.findall('joint') if j.find('command_interface') is not None and j.get('name') not in self.arm_names]
        if len(leaders)!=1:raise RuntimeError('unsupported gripper command topology')
        self.leader=leaders[0]
        obj=self.m.object_pose(s,self.name);tool=self.m.frame(s,self.tool)
        self.held=HeldObject(tool,obj,self.fingers,self.touch,joints[self.leader][0],self.open_position)
        if self.support:self.separation=Separation(obj,self.support['floor_z'])
        return self.held.relative
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


def require_trial_evidence(path,identity):
    if path is None:raise RuntimeError('full cycle requires measured contact-release and cancellation acceptance')
    records=json.loads(Path(path).read_text())
    if len(records)!=2 or {r.get('result') for r in records}!={'COMMISSION_TRIAL_PASS','CANCELLATION_TRIAL_PASS'}:
        raise RuntimeError('commissioning trials have not passed')
    for r in records:
        if not r.get('measured_reconciliation',{}).get('acm_restored') or r.get('backend_identity',{}).get('backend')!='simulator':
            raise RuntimeError('commissioning restoration/identity unproven')
    # Until the full measured final-state gate is installed, evidence never
    # authorizes ordinary execution by omission.
    raise RuntimeError('full-cycle final measured acceptance not yet commissioned')


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


def validate_measured_contacts(response,support,expired,predicate=None):
    """Use actual MoveIt contact geometry; never synthesize missing physics normals."""
    if response.valid:return
    if not support or expired or not response.contacts:
        raise RuntimeError('measured carried/robot collision or invalid state')
    if predicate is None:
        from ament_index_python.packages import get_package_prefix
        lib=ctypes.CDLL(str(Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'))
        predicate=lib.workcell_support_contact_valid
        predicate.argtypes=[ctypes.c_char_p]*4+[ctypes.c_double,ctypes.POINTER(ctypes.c_double)];predicate.restype=ctypes.c_bool
    for c in response.contacts:
        types={c.contact_body_1:c.body_type_1,c.contact_body_2:c.body_type_2}
        if types.get(support['object_id'])!=2 or types.get(support['support_id'])!=1:
            raise RuntimeError('measured support body types contradict carried state')
        point=[c.position.x,c.position.y,c.position.z,c.normal.x,c.normal.y,c.normal.z,c.depth]
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
