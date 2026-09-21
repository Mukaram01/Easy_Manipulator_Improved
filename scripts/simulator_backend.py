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


def prepare(xml, controllers, world_path, publisher, output):
    import yaml
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    world=ET.parse(world_path).getroot().find('world')
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
    (output/'world.sdf').write_text(telemetry_world(Path(world_path).read_text(),library,run_id))
    spec=dict(world=world.get('name'),model='workcell_robot',output=str(output),
              run_id=run_id,telemetry_library=str(library),telemetry_sha256=digest(library.read_bytes()),
              control_plugin_library=control['plugin_library'],
              control_plugin_sha256=digest(Path(control['plugin_library']).read_bytes()),
              control_plugin_name=control['plugin_name'],hardware_class=control['hardware_class'],
              domain=os.environ.get('ROS_DOMAIN_ID','0'),partition=os.environ.get('IGN_PARTITION',''),
              description_sha256=digest(description),world_sha256=digest((output/'world.sdf').read_bytes()),
              controllers_sha256=digest(controller_path.read_bytes()),expected_controllers=controller_names)
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
        spawn_started=time.time_ns()
        result=run_ign(['service','-s',f"/world/{spec['world']}/create",'--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','10000','--req',f'sdf_filename: "{out / "robot.urdf"}" name: "{spec["model"]}"'])
        (out/'spawn-response.json').write_text(json.dumps(dict(start_wall_ns=spawn_started,end_wall_ns=time.time_ns(),response=result),indent=2))
        if 'data: true' not in result:raise RuntimeError('simulator model creation failed: '+repr(result))
        record=dict(spec,pid=server.pid,start_ticks=process_info(server.pid)['start_ticks'],spawn_after_iteration=int(count[1]))
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
            matches.append(dict(pid=int(proc.name),start_ticks=info['start_ticks'],library=str(lib),sha256=manifest['sha256']))
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    if len(matches)!=1:raise RuntimeError('one identity-bound MoveIt process required')
    return dict(**matches[0],moveit_version=manifest['moveit_version'],sources=manifest['sources'],action_servers=1,
        receipt_sha256=digest(Path(receipt).read_bytes()))
