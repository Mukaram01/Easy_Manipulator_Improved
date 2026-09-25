#!/usr/bin/env python3
"""Local Fortress identity and derived control model for the existing executor.

A receipt locates a live process; it is NOT authorization. Every consumer checks
/proc identity and loaded libraries, domain/partition, immutable model inputs,
world service response and the live controller topology independently.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
import traceback
import xml.etree.ElementTree as ET

SIM_CLASSES = {'ign_ros2_control/IgnitionSystem', 'gz_ros2_control/GazeboSimSystem'}
SUPPORTED_COMMISSION_MOVEIT_VERSIONS = {'2.5.9', '2.5.10'}
PHYSICS_POSE_METHOD = 'physics_link_frame_data_at_offset'
DEFAULT_CONTROL_CONTRACT = {
    'hardware_class': 'ign_ros2_control/IgnitionSystem',
    'plugin_name': 'ign_ros2_control::IgnitionROS2ControlPlugin',
    'plugin_library': '/opt/ros/humble/lib/libign_ros2_control-system.so',
}


def digest(data):
    return hashlib.sha256(data.encode() if isinstance(data,str) else data).hexdigest()


def validate_identity(e):
    def reject(reason):
        raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: ' + reason)
    for key in ('process_verified','world_verified','description_verified','use_sim_time'):
        if e.get(key) is not True:reject(key)
    if e.get('backend')!='simulator' or e.get('manager_count')!=1:reject('ambiguous backend/controller_manager')
    names=e.get('component_names',[]); expected=e.get('expected_components',[])
    if not expected or sorted(names)!=sorted(expected) or len(set(names))!=len(names):reject('component ownership')
    classes=e.get('component_classes',[])
    if len(classes)!=len(names) or any(c and c not in SIM_CLASSES for c in classes):reject('unknown/contradictory hardware class')
    if len(e.get('component_states',[]))!=len(names) or any(s!=3 for s in e['component_states']):reject('inactive hardware')
    if not e.get('expected_controllers') or any(e.get('controllers',{}).get(c)!='active' for c in e['expected_controllers']):reject('controllers not active')
    if not e.get('expected_commands') or sorted(e.get('claimed_commands',[]))!=sorted(e['expected_commands']):reject('command ownership')
    if e.get('conflicting_processes')!=[]:reject('conflicting control processes')
    return dict(e, real_hardware=False, accepted_reason='verified local Fortress process, simulator model, world and exclusive controller topology')


def select_simulator_control_contract(prefixes):
    """Select one installed Fortress/gz ros2_control contract by real library presence."""
    unique=[]
    for prefix in prefixes:
        path=Path(prefix)
        if path not in unique:unique.append(path)
    candidates=[
        ('libign_ros2_control-system.so','ign_ros2_control/IgnitionSystem',
         'ign_ros2_control::IgnitionROS2ControlPlugin'),
        ('libgz_ros2_control-system.so','gz_ros2_control/GazeboSimSystem',
         'gz_ros2_control::GazeboSimROS2ControlPlugin'),
    ]
    for library_name,hardware_class,plugin_name in candidates:
        for prefix in unique:
            library=prefix/'lib'/library_name
            if library.is_file():
                return dict(hardware_class=hardware_class,plugin_name=plugin_name,
                            plugin_library=str(library.resolve()))
    raise RuntimeError('no reviewed Fortress/gz ros2_control system plugin library is installed')


def simulator_control_contract():
    from ament_index_python.packages import get_package_prefix
    prefixes=[]
    for package in ('gz_ros2_control','ign_ros2_control'):
        try:prefixes.append(Path(get_package_prefix(package)))
        except Exception:pass
    return select_simulator_control_contract(prefixes)


def simulator_description(xml, controllers, publisher, control_contract=None):
    """Derive runtime interfaces from source URDF; followers never accept commands."""
    root=ET.fromstring(xml); controls=root.findall('ros2_control')
    if not controls:raise ValueError('no hardware components')
    control=control_contract or DEFAULT_CONTROL_CONTRACT
    if control.get('hardware_class') not in SIM_CLASSES:
        raise ValueError('unreviewed simulator hardware class')
    library=Path(control.get('plugin_library',''))
    if control_contract is not None and not library.is_file():
        raise ValueError('selected simulator control plugin library is missing')
    owner={}
    for c in controls:
        plugin=c.find('hardware/plugin')
        if plugin is None or plugin.text!='mock_components/GenericSystem':raise ValueError('simulator derivation requires exclusively known mock source')
        plugin.text=control['hardware_class']
        for gpio in c.findall('gpio'):c.remove(gpio)
        for j in c.findall('joint'):
            if j.get('name') in owner:raise ValueError('duplicate joint ownership')
            owner[j.get('name')]=c
            for command in j.findall('command_interface'):
                if command.get('name')!='position':j.remove(command)
    for joint in root.findall('joint'):
        mimic=joint.find('mimic')
        if mimic is None:continue
        parent=mimic.get('joint'); name=joint.get('name')
        if parent not in owner or float(mimic.get('offset','0'))!=0:raise ValueError('unsupported mimic parent/offset')
        multiplier=float(mimic.get('multiplier','1'))
        if not math.isfinite(multiplier):raise ValueError('nonfinite mimic multiplier')
        c=owner[parent]
        j=c.find(f"joint[@name='{name}']")
        if j is not None:c.remove(j)
        elif name in owner:raise ValueError('mimic owned by different component')
        j=ET.SubElement(c,'joint',name=name)
        ET.SubElement(j,'param',name='mimic').text=parent
        ET.SubElement(j,'param',name='multiplier').text=mimic.get('multiplier','1')
        for interface in ('position','velocity'):ET.SubElement(j,'state_interface',name=interface)
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if 'mimic_joint_plugin' in plugin.get('filename',''):gazebo.remove(plugin)
            else:raise ValueError('unreviewed source Gazebo plugin')
    gazebo=ET.SubElement(root,'gazebo')
    plugin=ET.SubElement(gazebo,'plugin',filename=control['plugin_library'],name=control['plugin_name'])
    for key,value in [('robot_param','robot_description'),('robot_param_node',publisher),('parameters',controllers)]:ET.SubElement(plugin,key).text=value
    from ament_index_python.packages import get_package_share_directory
    for mesh in root.findall('.//mesh'):
        uri=mesh.get('filename','')
        if uri.startswith('package://'):
            package,relative=uri[10:].split('/',1)
            mesh.set('filename','file://'+str(Path(get_package_share_directory(package))/relative))
    return ET.tostring(root,encoding='unicode')


def telemetry_world(xml, library, run_id):
    root=ET.fromstring(xml); world=root.find('world')
    if world is None:raise ValueError('one world required')
    plugin=ET.SubElement(world,'plugin',filename=str(library),name='workcell::SimulatorMeasurements')
    ET.SubElement(plugin,'run_id').text=run_id
    ET.SubElement(plugin,'topic').text=f"/world/{world.get('name')}/workcell_measurements"
    return ET.tostring(root,encoding='unicode')


def bind_support_geometry(xml, manifest_path):
    """Use the same authored collision box as MoveIt for the Stage-A support."""
    import yaml
    root=ET.fromstring(xml)
    models=root.findall("world/model[@name='pick_support']")
    if not models:return xml,None
    if len(models)!=1 or manifest_path is None:
        raise ValueError('support requires one model and a collision manifest')
    try:
        path=Path(manifest_path).resolve();raw=path.read_bytes()
        manifest=yaml.safe_load(raw)
        if (not isinstance(manifest,dict) or not isinstance(manifest.get('objects'),list) or
                any(not isinstance(item,dict) for item in manifest['objects'])):
            raise ValueError('invalid support manifest objects')
        supports=[item for item in manifest['objects'] if item.get('semantic_role')=='support_surface']
        if len(supports)!=1:raise ValueError('ambiguous support objects')
        item=supports[0];geometry=item['collision_geometry'];pose=item['pose']
        if (item['frame_id']!='world' or item['operation']!='ADD' or
                geometry['type']!='box' or not isinstance(item['id'],str) or not item['id']):
            raise ValueError('unsupported support identity/frame/geometry')
        def vector(values,size):
            if len(values)!=size:raise ValueError('support vector dimension')
            result=[float(v) for v in values]
            if not all(math.isfinite(v) for v in result):raise ValueError('nonfinite support geometry')
            return result
        dimensions=vector(geometry['dimensions_m'],3);xyz=vector(pose['xyz'],3)
        quaternion=vector(pose['quaternion_xyzw'],4)
        if min(dimensions)<=0 or abs(sum(v*v for v in quaternion)-1.)>1e-9:
            raise ValueError('invalid support dimensions/quaternion')
        norm=math.sqrt(sum(v*v for v in quaternion))
        x,y,z,w=[v/norm for v in quaternion]
        r00,r10=1-2*(y*y+z*z),2*(x*y+w*z)
        r01,r11=2*(x*y-w*z),1-2*(x*x+z*z)
        sin_pitch=2*(w*y-z*x);cos_pitch=math.hypot(r00,r10)
        pitch=math.atan2(sin_pitch,cos_pitch)
        if cos_pitch<1e-12:
            roll=0.;yaw=math.atan2(-r01,r11)
        else:
            roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
            if cos_pitch<1e-4:
                # Near gimbal lock, recover yaw from the well-conditioned second
                # matrix column and chosen roll, not two tiny first-column terms.
                a=sin_pitch*math.sin(roll);b=math.cos(roll)
                yaw=math.atan2(a*r11-b*r01,a*r01+b*r11)
            else:yaw=math.atan2(r10,r00)
        rpy=[roll,pitch,yaw]
        cr,sr=math.cos(roll/2),math.sin(roll/2)
        cp,sp=math.cos(pitch/2),math.sin(pitch/2)
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        reconstructed=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
                       cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
        if min(sum((a-b)**2 for a,b in zip(reconstructed,quaternion)),
               sum((a+b)**2 for a,b in zip(reconstructed,quaternion)))>1e-18:
            raise ValueError('support quaternion/Euler roundtrip mismatch')
        model=models[0];links=model.findall('link')
        if (model.findtext('static')!='true' or model.get('placement_frame') or model.findall('joint') or
                len(links)!=1 or links[0].get('name')!='support_link'):
            raise ValueError('unsupported support model/link')
        link=links[0];collisions=link.findall('collision');visuals=link.findall('visual')
        if (len(collisions)!=1 or collisions[0].get('name')!='support_collision' or
                len(visuals)!=1 or visuals[0].get('name')!='support_visual'):
            raise ValueError('ambiguous support collision/visual')
        def set_pose(element,values):
            for old in element.findall('pose'):element.remove(old)
            ET.SubElement(element,'pose').text=' '.join(format(v,'.17g') for v in values)
        set_pose(model,xyz+rpy);set_pose(link,[0.]*6)
        for shape in collisions+visuals:
            set_pose(shape,[0.]*6)
            for old in shape.findall('geometry'):shape.remove(old)
            box=ET.SubElement(ET.SubElement(shape,'geometry'),'box')
            ET.SubElement(box,'size').text=' '.join(format(v,'.17g') for v in dimensions)
        binding=dict(manifest_path=str(path),manifest_sha256=digest(raw),object_id=item['id'],
                     model='pick_support',link='support_link',collision='support_collision',
                     dimensions_m=dimensions,pose_xyz=xyz,quaternion_xyzw=quaternion,pose_rpy=rpy)
        return ET.tostring(root,encoding='unicode'),binding
    except (OSError,KeyError,TypeError,ValueError,yaml.YAMLError) as exc:
        raise ValueError('support geometry binding rejected: '+str(exc)) from exc


def prepare(xml, controllers, world_path, publisher, output, *, collision_manifest_path=None):
    import yaml
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    world_xml,support_binding=bind_support_geometry(Path(world_path).read_text(),collision_manifest_path)
    world=ET.fromstring(world_xml).find('world')
    if world is None:raise ValueError('one explicit simulator world required')
    plugins=world.findall('.//plugin')
    if any(any(s in p.get('filename','').lower() for s in ('ros2_control','attach','magnet','suction')) for p in plugins):raise ValueError('world already controls/attaches objects')
    config=yaml.safe_load(Path(controllers).read_text())
    params=config['controller_manager']['ros__parameters'];params['use_sim_time']=True
    params['physical_joint_states']={'type':'joint_state_broadcaster/JointStateBroadcaster'}
    config['physical_joint_states']={'ros__parameters':{'use_local_topics':True}}
    controller_names=[k for k,v in params.items() if isinstance(v,dict) and v.get('type')=='joint_trajectory_controller/JointTrajectoryController']
    joints=[j for n in controller_names for j in config[n]['ros__parameters']['joints']]
    config['joint_state_broadcaster']={'ros__parameters':{'joints':joints,'interfaces':['position','velocity']}}
    controller_path=output/'controllers.yaml';controller_path.write_text(yaml.safe_dump(config))
    control=simulator_control_contract()
    description=simulator_description(xml,str(controller_path),publisher,control)
    (output/'robot.urdf').write_text(description)
    # A pristine initial world, with the robot inserted only after physics steps.
    from ament_index_python.packages import get_package_prefix
    library=Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_simulator_measurements.so'
    if not library.is_file():raise RuntimeError('build affected workcell_simulator_measurements target before simulator launch')
    run_id=uuid.uuid4().hex
    (output/'world.sdf').write_text(telemetry_world(world_xml,library,run_id))
    spec=dict(world=world.get('name'),model='workcell_robot',output=str(output),
              run_id=run_id,telemetry_library=str(library),telemetry_sha256=digest(library.read_bytes()),
              control_plugin_library=control['plugin_library'],
              control_plugin_sha256=digest(Path(control['plugin_library']).read_bytes()),
              control_plugin_name=control['plugin_name'],hardware_class=control['hardware_class'],
              domain=os.environ.get('ROS_DOMAIN_ID','0'),partition=os.environ.get('IGN_PARTITION',''),
              description_sha256=digest(description),world_sha256=digest((output/'world.sdf').read_bytes()),
              controllers_sha256=digest(controller_path.read_bytes()),expected_controllers=controller_names,
              support_geometry_binding=support_binding,measurement_pose_source=PHYSICS_POSE_METHOD)
    if not spec['partition'] or spec['domain']=='0':raise ValueError('simulator requires explicit isolated ROS_DOMAIN_ID and IGN_PARTITION')
    bridges=[]
    for topic,ros,gz,lazy in [('/clock','rosgraph_msgs/msg/Clock','ignition.msgs.Clock',False),
        (f"/world/{spec['world']}/pose/info",'tf2_msgs/msg/TFMessage','ignition.msgs.Pose_V',False),
        (f"/world/{spec['world']}/workcell_measurements",'std_msgs/msg/String','ignition.msgs.StringMsg',True)]:
        bridges.append(dict(ros_topic_name=topic,gz_topic_name=topic,ros_type_name=ros,gz_type_name=gz,direction='GZ_TO_ROS',lazy=lazy,publisher_queue=1000,subscriber_queue=1000))
    (output/'bridges.yaml').write_text(yaml.safe_dump(bridges))
    (output/'spec.json').write_text(json.dumps(spec,indent=2))
    return description,str(controller_path),spec


def process_info(pid):
    p=Path('/proc')/str(pid)
    return dict(start_ticks=p.joinpath('stat').read_text().rsplit(')',1)[1].split()[19],
                command=p.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode(),
                environ=dict(item.split('=',1) for item in p.joinpath('environ').read_bytes().decode().split('\0') if '=' in item),
                libraries=p.joinpath('maps').read_text())


def run_ign(args,timeout=15):
    return subprocess.check_output(['ign',*args],text=True,stderr=subprocess.STDOUT,timeout=timeout)


def world_contains_model(world,model,timeout=5):
    try:
        scene=run_ign(['service','-s',f"/world/{world}/scene/info",
            '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene',
            '--timeout',str(int(timeout*1000)),'--req',''],timeout+2)
    except (subprocess.CalledProcessError,subprocess.TimeoutExpired):
        return False
    return f'name: "{model}"' in scene


def wait_world_model(world,model,timeout=10):
    """Wait for SceneBroadcaster readback after an accepted/asynchronous create."""
    deadline=time.monotonic()+timeout
    checks=0
    while time.monotonic()<deadline:
        checks+=1
        if world_contains_model(world,model,min(2,max(.2,deadline-time.monotonic()))):
            return dict(found=True,checks=checks,wait_wall_ns=time.time_ns())
        time.sleep(.05)
    return dict(found=False,checks=checks,wait_wall_ns=time.time_ns())


def create_model(out,spec,max_attempts=3):
    """Create with bounded retries; never duplicate an accepted or observed model."""
    service=f"/world/{spec['world']}/create"
    request=f'sdf_filename: "{out / "robot.urdf"}" name: "{spec["model"]}"'
    attempts=[]
    for attempt in range(1,max_attempts+1):
        started=time.time_ns()
        try:
            response=run_ign(['service','-s',service,
                '--reqtype','ignition.msgs.EntityFactory',
                '--reptype','ignition.msgs.Boolean',
                '--timeout','10000','--req',request],12)
            record=dict(attempt=attempt,start_wall_ns=started,end_wall_ns=time.time_ns(),
                        response=response,timed_out=False)
            attempts.append(record)
            if 'data: true' in response:
                readback=wait_world_model(spec['world'],spec['model'],10)
                record['readback']=readback
                if not readback['found']:
                    raise RuntimeError('simulator create acknowledged but model never appeared in authoritative scene readback')
                return dict(attempts=attempts,recovered_by_scene_readback=False)
            readback=wait_world_model(spec['world'],spec['model'],2)
            record['readback']=readback
            if readback['found']:
                return dict(attempts=attempts,recovered_by_scene_readback=True)
            if 'data: false' in response:
                raise RuntimeError('simulator model creation rejected: '+repr(response))
        except subprocess.TimeoutExpired as exc:
            record=dict(attempt=attempt,start_wall_ns=started,end_wall_ns=time.time_ns(),
                        response='Service call timed out',timed_out=True)
            attempts.append(record)
            readback=wait_world_model(spec['world'],spec['model'],3)
            record['readback']=readback
            if readback['found']:
                return dict(attempts=attempts,recovered_by_scene_readback=True)
            if attempt==max_attempts:
                raise RuntimeError('simulator model creation timed out after delayed readback proved the model absent') from exc
            time.sleep(.5)
        except subprocess.CalledProcessError as exc:
            record=dict(attempt=attempt,start_wall_ns=started,end_wall_ns=time.time_ns(),
                        response=exc.output or str(exc),timed_out=False)
            attempts.append(record)
            readback=wait_world_model(spec['world'],spec['model'],2)
            record['readback']=readback
            if readback['found']:
                return dict(attempts=attempts,recovered_by_scene_readback=True)
            if attempt==max_attempts:
                raise RuntimeError('simulator model creation command failed and delayed readback proves model absent') from exc
            time.sleep(.5)
    raise RuntimeError('simulator model creation exhausted bounded attempts')


def verify_receipt_process(receipt):
    receipt=Path(receipt);r=json.loads(receipt.read_text());out=receipt.parent
    if r['domain']!=os.environ.get('ROS_DOMAIN_ID','0') or r['partition']!=os.environ.get('IGN_PARTITION',''):
        raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: domain/partition mismatch')
    process=process_info(r['pid'])
    if r.get('telemetry_library') and (digest(Path(r['telemetry_library']).read_bytes())!=r['telemetry_sha256'] or str(Path(r['telemetry_library']).resolve()) not in process['libraries']):
        raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: measurement binary changed')
    control_library=Path(r.get('control_plugin_library',''))
    if (not control_library.is_file() or
            digest(control_library.read_bytes())!=r.get('control_plugin_sha256') or
            str(control_library.resolve()) not in process['libraries']):
        raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: ros2_control simulator plugin changed')
    expected_files=[('world.sdf','world_sha256'),('controllers.yaml','controllers_sha256'),('robot.urdf','description_sha256')]
    if any(digest((out/f).read_bytes())!=r[key] for f,key in expected_files):
        raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: runtime input changed')
    verified=(process['start_ticks']==r['start_ticks'] and 'gazebo -s -r ' in process['command']
        and str(out/'world.sdf') in process['command']
        and 'libignition-gazebo6' in process['libraries']
        and process['environ'].get('ROS_DOMAIN_ID')==r['domain']
        and process['environ'].get('IGN_PARTITION')==r['partition'])
    if not verified:raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: live process mismatch')
    return r


def live_identity(receipt, description, components, controllers, interfaces, nodes, use_sim_time):
    receipt=Path(receipt);r=verify_receipt_process(receipt)
    e=dict(backend='simulator',use_sim_time=use_sim_time,process_verified=False,world_verified=False,
           description_verified=False,manager_count=sum(n=='controller_manager' for n,ns in nodes),
           component_names=[c.name for c in components],component_classes=[c.class_type for c in components],
           component_states=[c.state.id for c in components],controllers={c.name:c.state for c in controllers},
           expected_controllers=r['expected_controllers'],claimed_commands=[i.name for i in interfaces if i.is_claimed],
           conflicting_processes=[],pid=r['pid'],world=r['world'],partition=r['partition'],domain=r['domain'])
    e['process_verified']=True
    e['receipt_sha256']=digest(receipt.read_bytes())
    root=ET.fromstring(description);controls=root.findall('ros2_control')
    e['description_verified']=(digest(description)==r['description_sha256'] and bool(controls)
        and r.get('hardware_class') in SIM_CLASSES
        and all(c.findtext('hardware/plugin')==r.get('hardware_class') for c in controls))
    e['expected_components']=[c.get('name') for c in controls]
    e['expected_commands']=[j.get('name')+'/'+i.get('name') for c in controls for j in c.findall('joint') for i in j.findall('command_interface')]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name)==r['pid']:continue
        try:
            cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()
            if not any(x in cmd.split(' ', 1)[0] for x in ('ros2_control_node','ur_ros2_control_node','ur_robot_driver')):
                continue
            env=dict(item.split('=',1) for item in (p/'environ').read_bytes().decode().split('\0') if '=' in item)
            if env.get('ROS_DOMAIN_ID')!=r['domain']:continue
            # The exclusively simulator-owned manager is loaded into the verified
            # Gazebo process. A second manager/driver is a contradiction, not a fallback.
            e['conflicting_processes'].append(int(p.name))
        except (FileNotFoundError,ProcessLookupError):continue
        except PermissionError:
            if p.stat().st_uid==os.getuid():raise RuntimeError('SIMULATOR_IDENTITY_REJECTED: cannot inspect peer process')
    scene=run_ign(['service','-s',f"/world/{r['world']}/scene/info",'--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene','--timeout','5000','--req',''])
    e['world_verified']=f'name: "{r["model"]}"' in scene
    return validate_identity(e)


def serve(output):
    out=Path(output);spec=json.loads((out/'spec.json').read_text())
    server=subprocess.Popen(['ruby','/usr/bin/ign','gazebo','-s','-r',str(out/'world.sdf'),'-v','3'])
    def stop(signum,frame):
        if server.poll() is None:server.send_signal(signal.SIGINT)
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            if server.poll() is not None:raise RuntimeError('Fortress exited before physics startup')
            try:
                stats=run_ign(['topic','-e','-n','1','-t',f"/world/{spec['world']}/stats"],min(10,max(.1,deadline-time.monotonic())))
            except subprocess.TimeoutExpired:
                continue
            count=re.search(r'iterations: (\d+)',stats)
            if count and int(count[1])>1:break
        else:raise RuntimeError('physics initialization timed out')
        spawn=create_model(out,spec)
        (out/'spawn-response.json').write_text(json.dumps(spawn,indent=2))
        record=dict(spec,pid=server.pid,start_ticks=process_info(server.pid)['start_ticks'],
                    spawn_after_iteration=int(count[1]),spawn_attempts=len(spawn['attempts']),
                    spawn_recovered_by_scene_readback=spawn['recovered_by_scene_readback'])
        (out/'receipt.json').write_text(json.dumps(record,indent=2))
        print('Simulator receipt:',out/'receipt.json',flush=True)
        return server.wait()
    finally:
        if server.poll() is None:
            server.send_signal(signal.SIGINT)
            try:server.wait(timeout=10)
            except subprocess.TimeoutExpired:server.kill();server.wait()


if __name__=='__main__':
    try:
        raise SystemExit(serve(sys.argv[1]))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        out=Path(sys.argv[1]) if len(sys.argv)>1 else None
        if out is not None:
            try:
                out.mkdir(parents=True,exist_ok=True)
                (out/'startup-failure.json').write_text(json.dumps({
                    'type':type(exc).__name__,'message':str(exc),
                    'traceback':traceback.format_exc(),'wall_ns':time.time_ns()
                },indent=2)+'\n')
            except Exception:
                pass
        raise



MOVEIT_TEARDOWN_SOURCE_COMMIT='c62753946ae3629a8cb745767844f7e69ca51489'
MOVE_GROUP_TEARDOWN_SOURCE_COMMIT='66d37b40594e2b0ce8e8bd407122d20791d8c3b5'


def read_moveit_overlay(manifest_path,patch_path,move_group_patch_path):
    """Verify the one reviewed TEM dependency build, not arbitrary overlay claims."""
    try:
        path=Path(manifest_path);data=json.loads(path.read_text())
        if (data['schema']!='workcell_moveit_teardown_overlay/v1' or
            data['source']['version']!='2.5.10' or data['source']['commit']!=MOVEIT_TEARDOWN_SOURCE_COMMIT):
            raise ValueError('unreviewed source/version')
        if digest(Path(patch_path).read_bytes())!=data['patch']['sha256']:
            raise ValueError('patch differs from reviewed repository patch')
        lib=Path(data['library']['path']).resolve()
        if (str(lib).startswith('/opt/') or
            lib.name!='libmoveit_trajectory_execution_manager.so.2.5.10' or
            digest(lib.read_bytes())!=data['library']['sha256']):
            raise ValueError('patched library path/hash mismatch')
        proof=data['reproduction']
        if proof['baseline_returncode']!=-11 or proof['cycles']<20 or proof['passed']!=proof['cycles']:
            raise ValueError('twenty consecutive clean dependency cycles required')
        if (data['baseline']['package']!='ros-humble-moveit-ros-planning' or
            not data['baseline']['version'].startswith('2.5.10-')):
            raise ValueError('unreviewed installed baseline')
        group=data['move_group']
        if group['source']['version']!='2.5.10' or group['source']['commit']!=MOVE_GROUP_TEARDOWN_SOURCE_COMMIT:
            raise ValueError('unreviewed MoveGroup source/version')
        if digest(Path(move_group_patch_path).read_bytes())!=group['patch']['sha256']:
            raise ValueError('MoveGroup patch differs from reviewed repository patch')
        executable=Path(group['executable']['path']).resolve()
        if (str(executable).startswith('/opt/') or
            not str(executable).endswith('/lib/moveit_ros_move_group/move_group') or
            digest(executable.read_bytes())!=group['executable']['sha256']):
            raise ValueError('patched MoveGroup executable path/hash mismatch')
        proof=group['reproduction']
        if proof['baseline_returncode']!=-11 or proof['cycles']<20 or proof['passed']!=proof['cycles']:
            raise ValueError('twenty consecutive clean MoveGroup cycles required')
        if (group['baseline']['package']!='ros-humble-moveit-ros-move-group' or
            not group['baseline']['version'].startswith('2.5.10-')):
            raise ValueError('unreviewed installed MoveGroup baseline')
        return dict(data,manifest_path=str(path.resolve()),manifest_sha256=digest(path.read_bytes()))
    except (KeyError,ValueError,TypeError,OSError) as exc:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: '+str(exc)) from exc


def active_moveit_overlay():
    """Check package discovery and the loader's actual resolution before launch."""
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix('moveit_ros_planning'))
    patch=Path(__file__).resolve().parents[1]/'patches/moveit_humble_tem_teardown.patch'
    data=read_moveit_overlay(prefix/'share/moveit_ros_planning/workcell_teardown_overlay.json',patch,
        patch.parent/'moveit_humble_capability_teardown.patch')
    library=Path(data['library']['path']).resolve()
    if (prefix/'lib/libmoveit_trajectory_execution_manager.so').resolve()!=library:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: package prefix selects a different TEM library')
    installed=subprocess.check_output(['dpkg-query','-W','-f='+chr(36)+'{Version}',data['baseline']['package']],text=True).strip()
    if installed!=data['baseline']['version']:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: installed baseline changed; rebuild/requalify overlay')
    executable=Path(get_package_prefix('moveit_ros_move_group'))/'lib/moveit_ros_move_group/move_group'
    if executable.resolve()!=Path(data['move_group']['executable']['path']).resolve():
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: package discovery selects unpatched MoveGroup')
    group_baseline=data['move_group']['baseline']
    installed_group=subprocess.check_output(['dpkg-query','-W','-f='+chr(36)+'{Version}',group_baseline['package']],text=True).strip()
    if installed_group!=group_baseline['version']:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: installed MoveGroup baseline changed; rebuild/requalify overlay')
    linked=subprocess.check_output(['ldd',str(executable)],text=True)
    resolved=re.findall(r'libmoveit_trajectory_execution_manager\.so[^\s]*\s+=>\s+(\S+)',linked)
    if len(resolved)!=1 or Path(resolved[0]).resolve()!=library:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: dynamic loader still selects unpatched MoveIt; source overlay')
    return dict(data,ldd_library=str(Path(resolved[0]).resolve()))


def verify_moveit_overlay_maps(data,maps):
    """Require the verified inode/path to be mapped; a matching version is insufficient."""
    loaded={}
    for field,item,marker in [('library',data['library'],'/libmoveit_trajectory_execution_manager.so'),
            ('executable',data['move_group']['executable'],'/moveit_ros_move_group/move_group')]:
        path=Path(item['path']).resolve();stat=path.stat()
        fields=[line.split() for line in maps.splitlines() if marker in line]
        if (not fields or digest(path.read_bytes())!=item['sha256'] or
            any(len(f)!=6 or f[-1]!=str(path) or int(f[4])!=stat.st_ino for f in fields)):
            raise RuntimeError('MOVEIT_OVERLAY_REJECTED: move_group did not map the qualified '+field+' inode/path/hash')
        loaded['loaded_'+field]=str(path);loaded['loaded_'+field+'_inode']=stat.st_ino
    return dict(data,**loaded)


def moveit_overlay_identity(data):
    return (data.get('library',{}).get('sha256'),
        data.get('move_group',{}).get('executable',{}).get('sha256'))


def live_moveit_overlay(domain,partition,expected_identity):
    data=active_moveit_overlay()
    if moveit_overlay_identity(data)!=expected_identity:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: dependency differs from runner preflight')
    matches=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            info=process_info(int(proc.name))
            if '/moveit_ros_move_group/move_group ' not in info['command'] or info['environ'].get('ROS_DOMAIN_ID')!=str(domain):continue
            if info['environ'].get('IGN_PARTITION')!=partition:
                raise RuntimeError('MOVEIT_OVERLAY_REJECTED: ambiguous MoveIt partition')
            matches.append(dict(verify_moveit_overlay_maps(data,info['libraries']),pid=int(proc.name),start_ticks=info['start_ticks']))
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    if len(matches)!=1:raise RuntimeError('MOVEIT_OVERLAY_REJECTED: exactly one live MoveIt process required')
    return matches[0]


def commissioning_capability_identity(node,receipt,enabled,capabilities,disabled):
    """Fail closed unless the explicit, sole action server maps the pinned build."""
    from ament_index_python.packages import get_package_prefix
    r=verify_receipt_process(receipt)
    if not enabled or capabilities.split()!=['workcell/CommissionExecuteTrajectory'] or 'move_group/MoveGroupExecuteTrajectoryAction' not in disabled.split():
        raise RuntimeError('corrected simulator-only ExecuteTrajectory capability is not selected')
    pubs=node.get_publishers_info_by_topic('/execute_trajectory/_action/status')
    if len(pubs)!=1 or pubs[0].node_name!='move_group' or pubs[0].node_namespace!='/':
        raise RuntimeError('ExecuteTrajectory must have exactly one authoritative move_group server')
    prefix=Path(get_package_prefix('workcell_builder'))
    manifest=json.loads((prefix/'share/workcell_builder/commission_execute_build.json').read_text())
    if manifest['moveit_version'] not in SUPPORTED_COMMISSION_MOVEIT_VERSIONS:
        raise RuntimeError('unqualified MoveIt capability version')
    lib=Path(manifest['library']).resolve();stat=lib.stat()
    if digest(lib.read_bytes())!=manifest['sha256']:raise RuntimeError('commission capability differs from tracked build')
    for path,expected in manifest['sources'].items():
        if digest(Path(path).read_bytes())!=expected:raise RuntimeError('commission capability source changed since build: '+path)
    overlay=active_moveit_overlay()
    if moveit_overlay_identity(manifest.get('moveit_overlay',{}))!=moveit_overlay_identity(overlay):
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: rebuild commissioning capability against qualified overlay')
    matches=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            info=process_info(int(proc.name))
            if '/moveit_ros_move_group/move_group ' not in info['command'] or info['environ'].get('ROS_DOMAIN_ID')!=r['domain']:continue
            if info['environ'].get('IGN_PARTITION')!=r['partition']:raise RuntimeError('MoveIt simulator partition mismatch')
            mapped=[line.split() for line in info['libraries'].splitlines() if str(lib) in line]
            if not mapped or any(fields[-1]!=str(lib) or int(fields[4])!=stat.st_ino for fields in mapped):
                raise RuntimeError('MoveIt has not loaded the current commissioning binary')
            dependency=verify_moveit_overlay_maps(overlay,info['libraries'])
            matches.append(dict(pid=int(proc.name),start_ticks=info['start_ticks'],library=str(lib),
                sha256=manifest['sha256'],moveit_overlay=dependency))
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    if len(matches)!=1:raise RuntimeError('one identity-bound MoveIt process required')
    return dict(**matches[0],moveit_version=manifest['moveit_version'],sources=manifest['sources'],action_servers=1,
        receipt_sha256=digest(Path(receipt).read_bytes()))


BRIDGE_SHUTDOWN_SOURCE_COMMIT='90cdc5361059a6f949bc004658e7363df33bcffe'


def read_bridge_overlay(manifest_path,patch_path):
    """Require the source-matched bridge shutdown fix and its traffic proof."""
    try:
        path=Path(manifest_path);data=json.loads(path.read_text())
        if (data['schema']!='workcell_ros_gz_bridge_shutdown/v1' or
            data['source']['version']!='0.244.26' or data['source']['commit']!=BRIDGE_SHUTDOWN_SOURCE_COMMIT):
            raise ValueError('unreviewed bridge source/version')
        if digest(Path(patch_path).read_bytes())!=data['patch']['sha256']:
            raise ValueError('bridge patch differs from reviewed repository patch')
        executable=Path(data['executable']['path']).resolve()
        if str(executable).startswith('/opt/') or not str(executable).endswith('/lib/ros_gz_bridge/parameter_bridge'):
            raise ValueError('patched bridge executable must be outside /opt')
        for field in ('executable','library'):
            if digest(Path(data[field]['path']).read_bytes())!=data[field]['sha256']:
                raise ValueError('bridge '+field+' hash mismatch')
        if Path(data['library']['path']).name!='libros_gz_bridge.so':
            raise ValueError('unexpected bridge library')
        if (data['baseline']['package']!='ros-humble-ros-gz-bridge' or
            not data['baseline']['version'].startswith('0.244.26-')):
            raise ValueError('unreviewed installed bridge baseline')
        proof=data['reproduction']
        if proof['baseline_returncode']!=-6:raise ValueError('bridge abort baseline proof missing')
        for name,minimum in (('native',20),('memcheck',1)):
            result=proof[name]
            if (type(result['cycles']) is not int or result['cycles']<minimum or
                result['passed']!=result['cycles'] or (name=='memcheck' and result['errors']!=0)):
                raise ValueError('clean bridge '+name+' repetitions required')
            if digest(Path(result['results_path']).read_bytes())!=result['results_sha256']:
                raise ValueError('bridge '+name+' proof hash mismatch')
        return dict(data,manifest_path=str(path.resolve()),manifest_sha256=digest(path.read_bytes()))
    except (KeyError,ValueError,TypeError,OSError) as exc:
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: '+str(exc)) from exc


def active_bridge_overlay():
    root=os.environ.get('ROS_GZ_BRIDGE_SHUTDOWN_OVERLAY')
    if not root:raise RuntimeError('BRIDGE_OVERLAY_REJECTED: source the qualified bridge shutdown overlay')
    patch=Path(__file__).resolve().parents[1]/'patches/ros_gz_bridge_humble_shutdown.patch'
    data=read_bridge_overlay(Path(root)/'provenance.json',patch)
    installed=subprocess.check_output(['dpkg-query','-W','-f='+chr(36)+'{Version}',data['baseline']['package']],text=True).strip()
    if installed!=data['baseline']['version']:
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: installed bridge changed; rebuild/requalify overlay')
    linked=subprocess.check_output(['ldd',data['executable']['path']],text=True)
    libraries=re.findall(r'libros_gz_bridge\.so\s+=>\s+(\S+)',linked)
    if len(libraries)!=1 or Path(libraries[0]).resolve()!=Path(data['library']['path']).resolve():
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: patched executable selects a different bridge library')
    return data


def bridge_executable(commissioning):
    return active_bridge_overlay()['executable']['path'] if commissioning else 'parameter_bridge'


def bridge_overlay_identity(data):
    return (data.get('executable',{}).get('sha256'),data.get('library',{}).get('sha256'))


def verify_bridge_overlay_maps(data,maps):
    loaded={}
    for field,marker in (('executable','/ros_gz_bridge/parameter_bridge'),('library','/libros_gz_bridge.so')):
        path=Path(data[field]['path']).resolve();stat=path.stat()
        fields=[line.split() for line in maps.splitlines() if marker in line]
        if (not fields or digest(path.read_bytes())!=data[field]['sha256'] or
            any(len(f)!=6 or f[-1]!=str(path) or int(f[4])!=stat.st_ino for f in fields)):
            raise RuntimeError('BRIDGE_OVERLAY_REJECTED: live bridge differs from qualified '+field+' inode/path/hash')
        loaded['loaded_'+field]=str(path);loaded['loaded_'+field+'_inode']=stat.st_ino
    return dict(data,**loaded)


def live_bridge_overlay(domain,partition,expected_identity):
    data=active_bridge_overlay()
    if bridge_overlay_identity(data)!=expected_identity:
        raise RuntimeError('BRIDGE_OVERLAY_REJECTED: dependency differs from runner preflight')
    matches=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            info=process_info(int(proc.name))
            if '/ros_gz_bridge/parameter_bridge ' not in info['command'] or info['environ'].get('ROS_DOMAIN_ID')!=str(domain):continue
            if info['environ'].get('IGN_PARTITION')!=partition:
                raise RuntimeError('BRIDGE_OVERLAY_REJECTED: ambiguous bridge partition')
            if (proc/'exe').resolve()!=Path(data['executable']['path']).resolve():
                raise RuntimeError('BRIDGE_OVERLAY_REJECTED: live executable differs from qualified bridge')
            matches.append(dict(verify_bridge_overlay_maps(data,info['libraries']),pid=int(proc.name),start_ticks=info['start_ticks']))
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    if len(matches)!=1:raise RuntimeError('BRIDGE_OVERLAY_REJECTED: exactly one qualified live bridge required')
    return matches[0]
