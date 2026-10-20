"""Fail-closed identity and correct derived coupling, independent of a running ROS graph."""
import copy
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from simulator_backend import (validate_identity, simulator_description,
    select_simulator_control_contract, SUPPORTED_COMMISSION_MOVEIT_VERSIONS)


def evidence():
    return dict(process_verified=True, world_verified=True, description_verified=True,
                manager_count=1, component_names=['arm','hand'],
                expected_components=['arm','hand'], component_classes=['',''],
                component_states=[3,3], controllers={'arm_controller':'active','hand_controller':'active'},
                expected_controllers=['arm_controller','hand_controller'],
                claimed_commands=['a/position','g/position'], expected_commands=['a/position','g/position'],
                conflicting_processes=[], use_sim_time=True, backend='simulator')


def test_positive_simulator_identity_requires_all_independent_evidence():
    assert validate_identity(evidence())['backend']=='simulator'

@pytest.mark.parametrize('key,value', [
 ('process_verified',False),('world_verified',False),('description_verified',False),
 ('manager_count',0),('manager_count',2),('component_classes',['','RealSystem']),
 ('component_classes',['','mock_components/GenericSystem']),('component_classes',['','unknown']),
 ('component_names',['arm']),('component_states',[3,2]),('controllers',{'arm_controller':'inactive'}),
 ('claimed_commands',['a/position','g/position','f/position']),('conflicting_processes',[99]),
 ('use_sim_time',False),('backend','fake')])
def test_missing_unknown_or_contradictory_identity_rejected(key,value):
    e=evidence();e[key]=value
    with pytest.raises(RuntimeError):validate_identity(e)


def test_simulator_derives_followers_and_leaves_input_unchanged(tmp_path):
    xml='''<robot name="r"><joint name="f"><mimic joint="g" multiplier="-1"/></joint>
    <ros2_control name="hand" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="g"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/></joint></ros2_control></robot>'''
    result=simulator_description(xml, str(tmp_path/'controllers.yaml'),'rsp')
    import xml.etree.ElementTree as ET
    root=ET.fromstring(result);j=root.find("ros2_control/joint[@name='f']")
    assert not j.findall('command_interface')
    assert j.find("param[@name='multiplier']").text=='-1'
    assert len(root.findall("ros2_control/joint[@name='f']"))==1
    assert 'mock_components/GenericSystem' in xml
    assert 'ign_ros2_control/IgnitionSystem' in result

@pytest.mark.parametrize('replacement',['RealSystem','unknown','ign_ros2_control/IgnitionSystem'])
def test_derivation_requires_known_mock_source(replacement,tmp_path):
    xml=f'<robot><ros2_control><hardware><plugin>{replacement}</plugin></hardware></ros2_control></robot>'
    with pytest.raises(ValueError):simulator_description(xml,str(tmp_path/'c.yaml'),'rsp')


def test_settling_rejects_motion_between_equal_endpoints():
    from simulator_observations import settling_error
    samples=[{'poses':{'part':[0.,0.,0.,0.,0.,0.,1.]}} for _ in range(3)]
    assert settling_error(samples)==0
    samples[1]={'poses':{'part':[0.,0.,.01,0.,0.,0.,1.]}}
    assert settling_error(samples)==pytest.approx(.01)
    samples[1]['poses']['part'][2]=float('nan')
    assert settling_error(samples)==float('inf')


def test_receipt_process_rejects_wrong_domain_before_acquisition(tmp_path,monkeypatch):
    from simulator_backend import verify_receipt_process
    import json
    path=tmp_path/'receipt.json'
    path.write_text(json.dumps({'domain':'199','partition':'owned'}))
    monkeypatch.setenv('ROS_DOMAIN_ID','198');monkeypatch.setenv('IGN_PARTITION','owned')
    with pytest.raises(RuntimeError,match='domain/partition'):
        verify_receipt_process(path)


def test_snapshot_cannot_cross_simulator_runs():
    from simulator_observations import verify_snapshot_binding
    with pytest.raises(RuntimeError,match='observation'):
        verify_snapshot_binding({'simulator_receipt_sha256':'old'},'new')
    verify_snapshot_binding({'simulator_receipt_sha256':'same'},'same')


def test_telemetry_world_is_receipt_bound_and_does_not_change_geometry():
    from simulator_backend import telemetry_world
    import xml.etree.ElementTree as ET
    original='<sdf version="1.8"><world name="cell"><model name="part"><link name="body"><collision name="shape"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>'
    result=ET.fromstring(telemetry_world(original,'/verified/plugin.so','nonce'))
    assert ET.tostring(result.find('.//model'))==ET.tostring(ET.fromstring(original).find('.//model'))
    plugin=result.find('.//plugin')
    assert plugin.get('filename')=='/verified/plugin.so'
    assert plugin.findtext('run_id')=='nonce'
    assert plugin.findtext('topic')=='/world/cell/workcell_measurements'


def test_commissioning_moveit_versions_are_explicitly_bounded():
    assert SUPPORTED_COMMISSION_MOVEIT_VERSIONS == {'2.5.9','2.5.10'}
    assert '2.5.11' not in SUPPORTED_COMMISSION_MOVEIT_VERSIONS


def test_control_contract_prefers_fortress_legacy_library_when_available(tmp_path):
    lib=tmp_path/'lib';lib.mkdir()
    (lib/'libgz_ros2_control-system.so').write_bytes(b'gz')
    (lib/'libign_ros2_control-system.so').write_bytes(b'ign')
    contract=select_simulator_control_contract([tmp_path])
    assert contract['hardware_class']=='ign_ros2_control/IgnitionSystem'
    assert contract['plugin_name']=='ign_ros2_control::IgnitionROS2ControlPlugin'
    assert contract['plugin_library'].endswith('libign_ros2_control-system.so')


def test_control_contract_falls_back_to_gz_library(tmp_path):
    lib=tmp_path/'lib';lib.mkdir()
    (lib/'libgz_ros2_control-system.so').write_bytes(b'gz')
    contract=select_simulator_control_contract([tmp_path])
    assert contract['hardware_class']=='gz_ros2_control/GazeboSimSystem'
    assert contract['plugin_name']=='gz_ros2_control::GazeboSimROS2ControlPlugin'


def test_simulator_description_uses_selected_control_contract(tmp_path):
    library=tmp_path/'libign_ros2_control-system.so';library.write_bytes(b'x')
    xml='<robot name="r"><ros2_control name="arm" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware><joint name="j"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/></joint></ros2_control></robot>'
    contract={'hardware_class':'ign_ros2_control/IgnitionSystem',
              'plugin_name':'ign_ros2_control::IgnitionROS2ControlPlugin',
              'plugin_library':str(library)}
    result=simulator_description(xml,str(tmp_path/'c.yaml'),'rsp',contract)
    assert '<plugin>ign_ros2_control/IgnitionSystem</plugin>' in result
    assert str(library) in result
    assert 'ign_ros2_control::IgnitionROS2ControlPlugin' in result




def test_create_model_retries_only_after_scene_readback_proves_absent(tmp_path,monkeypatch):
    import simulator_backend, subprocess
    (tmp_path/'robot.urdf').write_text('<robot/>')
    calls=[]
    def fake(args,timeout=15):
        calls.append(list(args))
        if args[0]=='service' and '/world/a0/create' in args:
            creates=sum('/create' in item for call in calls for item in call)
            if creates==1:
                raise subprocess.TimeoutExpired(args,timeout)
            return 'data: true\n'
        if args[0]=='service' and '/world/a0/scene/info' in args:
            return ('model { name: "workcell_robot" }\n' if sum('/world/a0/create' in call for call in calls)>=2 else 'name: "part_00"\n')
        raise AssertionError(args)
    monkeypatch.setattr(simulator_backend,'run_ign',fake)
    monkeypatch.setattr(simulator_backend.time,'sleep',lambda _:None)
    result=simulator_backend.create_model(tmp_path,{'world':'a0','model':'workcell_robot'})
    assert len(result['attempts'])==2
    assert result['recovered_by_scene_readback'] is False


def test_create_model_accepts_lost_response_only_when_scene_proves_spawn(tmp_path,monkeypatch):
    import simulator_backend, subprocess
    (tmp_path/'robot.urdf').write_text('<robot/>')
    create_calls=0
    def fake(args,timeout=15):
        nonlocal create_calls
        if args[0]=='service' and '/world/a0/create' in args:
            create_calls+=1
            raise subprocess.TimeoutExpired(args,timeout)
        if args[0]=='service' and '/world/a0/scene/info' in args:
            return 'model { name: "workcell_robot" }\n'
        raise AssertionError(args)
    monkeypatch.setattr(simulator_backend,'run_ign',fake)
    result=simulator_backend.create_model(tmp_path,{'world':'a0','model':'workcell_robot'})
    assert create_calls==1
    assert result['recovered_by_scene_readback'] is True


def test_wait_world_model_tolerates_scene_broadcast_delay(monkeypatch):
    import simulator_backend
    answers=iter([False,False,True])
    monkeypatch.setattr(simulator_backend,'world_contains_model',
                        lambda *args,**kwargs: next(answers))
    monkeypatch.setattr(simulator_backend.time,'sleep',lambda _:None)
    ticks=iter([0.,.01,.02,.03,.04])
    monkeypatch.setattr(simulator_backend.time,'monotonic',lambda:next(ticks,.04))
    result=simulator_backend.wait_world_model('a0','workcell_robot',1.0)
    assert result['found'] is True
    assert result['checks']==3


def test_create_model_data_true_waits_for_authoritative_scene_without_duplicate(tmp_path,monkeypatch):
    import simulator_backend
    (tmp_path/'robot.urdf').write_text('<robot/>')
    create_calls=0
    def fake_ign(args,timeout=15):
        nonlocal create_calls
        if '/world/a0/create' in args:
            create_calls+=1
            return 'data: true\n'
        raise AssertionError(args)
    monkeypatch.setattr(simulator_backend,'run_ign',fake_ign)
    monkeypatch.setattr(simulator_backend,'wait_world_model',
                        lambda *args,**kwargs:{'found':True,'checks':4,'wait_wall_ns':123})
    result=simulator_backend.create_model(tmp_path,{'world':'a0','model':'workcell_robot'})
    assert create_calls==1
    assert result['attempts'][0]['readback']['checks']==4
    assert result['recovered_by_scene_readback'] is False


def test_refresh_snapshot_geometry_renews_time_without_changing_bound_geometry():
    from simulator_observations import refresh_snapshot_geometry
    base=dict(schema_version='detected_objects/v1',simulator_receipt_sha256='same',
        source='old',objects=[dict(object_id='part_00',class_id='part',confidence=None,timestamp=10.,
        pose=dict(frame_id='world',xyz=[.4,-.2,.0125],rpy=[0.,0.,.2]),dimensions=[.025]*3)])
    fresh=copy.deepcopy(base)
    fresh['objects'][0]['timestamp']=200.
    fresh['objects'][0]['pose']['xyz'][2]-=1e-6
    rebound,evidence=refresh_snapshot_geometry(base,fresh)
    assert rebound['objects'][0]['timestamp']==200.
    assert rebound['objects'][0]['pose']==base['objects'][0]['pose']
    assert evidence['max_position_delta_m']>0


def test_refresh_snapshot_geometry_rejects_real_motion():
    from simulator_observations import refresh_snapshot_geometry
    base=dict(schema_version='detected_objects/v1',simulator_receipt_sha256='same',
        objects=[dict(object_id='part_00',class_id='part',confidence=None,timestamp=10.,
        pose=dict(frame_id='world',xyz=[.4,-.2,.0125],rpy=[0.,0.,0.]),dimensions=[.025]*3)])
    fresh=copy.deepcopy(base);fresh['objects'][0]['pose']['xyz'][0]+=.001
    with pytest.raises(RuntimeError,match='geometry changed'):
        refresh_snapshot_geometry(base,fresh)


def overlay_fixture(tmp_path):
    import json
    from simulator_backend import digest
    patch=tmp_path/'fix.patch';patch.write_text('reviewed destructor ordering')
    library=tmp_path/'libmoveit_trajectory_execution_manager.so.2.5.10'
    library.write_bytes(b'patched TEM')
    group_patch=tmp_path/'moveit_humble_capability_teardown.patch';group_patch.write_text('retain capability loader past main node')
    executable=tmp_path/'lib/moveit_ros_move_group/move_group';executable.parent.mkdir(parents=True);executable.write_bytes(b'patched MoveGroup')
    capabilities=tmp_path/'lib/libmoveit_move_group_default_capabilities.so.2.5.10'
    capabilities.write_bytes(b'patched active-result capability')
    manifest=tmp_path/'provenance.json'
    data=dict(schema='workcell_moveit_teardown_overlay/v1',
        source=dict(version='2.5.10',commit='c62753946ae3629a8cb745767844f7e69ca51489'),
        patch=dict(path=str(patch),sha256=digest(patch.read_bytes())),
        library=dict(path=str(library),sha256=digest(library.read_bytes())),
        baseline=dict(package='ros-humble-moveit-ros-planning',version='2.5.10-1jammy.test'),
        reproduction=dict(baseline_returncode=-11,cycles=20,passed=20),
        move_group=dict(source=dict(version='2.5.10',commit='66d37b40594e2b0ce8e8bd407122d20791d8c3b5'),
            patch=dict(path=str(group_patch),sha256=digest(group_patch.read_bytes())),
            executable=dict(path=str(executable),sha256=digest(executable.read_bytes())),
            capabilities=dict(path=str(capabilities),sha256=digest(capabilities.read_bytes())),
            baseline=dict(package='ros-humble-moveit-ros-move-group',version='2.5.10-1jammy.test'),
            reproduction=dict(baseline_returncode=-11,cycles=20,passed=20)))
    from test_moveit_overlay_publication import add_active_proof
    add_active_proof(tmp_path,data)
    manifest.write_text(json.dumps(data))
    return manifest,patch,library,data


def test_moveit_overlay_provenance_requires_current_patch_binary_and_twenty_cycles(tmp_path):
    import json
    from simulator_backend import read_moveit_overlay
    manifest,patch,library,data=overlay_fixture(tmp_path)
    assert read_moveit_overlay(manifest,patch,patch.parent/"moveit_humble_capability_teardown.patch")['library']['path']==str(library)
    for field,value in [('patch',dict(data['patch'],sha256='stale')),
                        ('library',dict(data['library'],sha256='stale')),
                        ('source',dict(data['source'],version='2.5.9')),
                        ('source',dict(data['source'],commit='b'*40)),
                        ('reproduction',dict(data['reproduction'],passed=19)),
                        ('reproduction',dict(data['reproduction'],baseline_returncode=0))]:
        changed=copy.deepcopy(data);changed[field]=value;manifest.write_text(json.dumps(changed))
        with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):read_moveit_overlay(manifest,patch,patch.parent/"moveit_humble_capability_teardown.patch")
    for field,value in [('patch',dict(data['move_group']['patch'],sha256='stale')),
            ('executable',dict(data['move_group']['executable'],sha256='stale')),
            ('capabilities',dict(data['move_group']['capabilities'],sha256='stale')),
            ('source',dict(data['move_group']['source'],commit='b'*40)),
            ('reproduction',dict(data['move_group']['reproduction'],passed=19))]:
        changed=copy.deepcopy(data);changed['move_group'][field]=value;manifest.write_text(json.dumps(changed))
        with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):read_moveit_overlay(manifest,patch,patch.parent/'moveit_humble_capability_teardown.patch')
    manifest.write_text(json.dumps(data));library.write_bytes(b'original TEM replaced overlay')
    with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):read_moveit_overlay(manifest,patch,patch.parent/"moveit_humble_capability_teardown.patch")


def test_moveit_process_maps_must_match_patched_path_inode_and_hash(tmp_path):
    from simulator_backend import read_moveit_overlay, verify_moveit_overlay_maps
    manifest,patch,library,_=overlay_fixture(tmp_path)
    data=read_moveit_overlay(manifest,patch,patch.parent/"moveit_humble_capability_teardown.patch")
    executable=Path(data['move_group']['executable']['path'])
    mapping=f'1000-2000 r-xp 0 00:01 {library.stat().st_ino} {library}\n'
    mapping+=f'3000-4000 r-xp 0 00:01 {executable.stat().st_ino} {executable}\n'
    capabilities=Path(data['move_group']['capabilities']['path'])
    mapping+=f'5000-6000 r-xp 0 00:01 {capabilities.stat().st_ino} {capabilities}\n'
    assert verify_moveit_overlay_maps(data,mapping)['loaded_library']==str(library)
    for changed in ['',mapping.rstrip()+' (deleted)\n',mapping.replace(str(library),'/opt/ros/humble/lib/'+library.name),
                    mapping.replace(str(library.stat().st_ino),'0'),
                    mapping.replace(str(executable),'/opt/ros/humble/lib/moveit_ros_move_group/move_group'),
                    mapping.replace(str(executable.stat().st_ino),'0'),
                    mapping.replace(str(capabilities),'/opt/ros/humble/lib/'+capabilities.name),
                    mapping.replace(str(capabilities.stat().st_ino),'0')]:
        with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):verify_moveit_overlay_maps(data,changed)


def bridge_overlay_fixture(tmp_path):
    import json
    from simulator_backend import digest
    patch=tmp_path/'bridge.patch';patch.write_text('shutdown before main returns')
    exe=tmp_path/'lib/ros_gz_bridge/parameter_bridge';exe.parent.mkdir(parents=True);exe.write_bytes(b'patched bridge')
    library=tmp_path/'libros_gz_bridge.so';library.write_bytes(b'installed bridge library')
    proof=tmp_path/'results.json';proof.write_text('qualified results')
    result=dict(cycles=20,passed=20,errors=0,results_path=str(proof),results_sha256=digest(proof.read_bytes()))
    data=dict(schema='workcell_ros_gz_bridge_shutdown/v1',
        source=dict(version='0.244.26',commit='90cdc5361059a6f949bc004658e7363df33bcffe'),
        patch=dict(path=str(patch),sha256=digest(patch.read_bytes())),
        executable=dict(path=str(exe),sha256=digest(exe.read_bytes())),
        library=dict(path=str(library),sha256=digest(library.read_bytes())),
        baseline=dict(package='ros-humble-ros-gz-bridge',version='0.244.26-1jammy.test'),
        reproduction=dict(baseline_returncode=-6,native=result,memcheck=dict(result,cycles=5,passed=5)))
    manifest=tmp_path/'provenance.json';manifest.write_text(json.dumps(data))
    return manifest,patch,data


def test_bridge_overlay_requires_qualified_source_binary_patch_and_clean_proof(tmp_path):
    import json
    from simulator_backend import read_bridge_overlay
    manifest,patch,data=bridge_overlay_fixture(tmp_path)
    assert read_bridge_overlay(manifest,patch)['executable']==data['executable']
    for path,value in [('source.commit','wrong'),('source.version','0.244.25'),
                       ('patch.sha256','wrong'),('executable.sha256','wrong'),('library.sha256','wrong'),
                       ('reproduction.baseline_returncode',0),('reproduction.native.cycles',19),
                       ('reproduction.native.passed',19),('reproduction.memcheck.errors',1),
                       ('reproduction.memcheck.cycles',0),('reproduction.memcheck.results_sha256','wrong')]:
        changed=copy.deepcopy(data);part=changed;keys=path.split('.')
        for key in keys[:-1]:part=part[key]
        part[keys[-1]]=value;manifest.write_text(json.dumps(changed))
        with pytest.raises(RuntimeError,match='BRIDGE_OVERLAY_REJECTED'):
            read_bridge_overlay(manifest,patch)


def test_bridge_live_maps_require_exact_qualified_executable_and_library_inode(tmp_path):
    from simulator_backend import verify_bridge_overlay_maps
    _,_,data=bridge_overlay_fixture(tmp_path)
    exe=Path(data['executable']['path']);lib=Path(data['library']['path'])
    mapping=f'1000-2000 r-xp 0 00:01 {exe.stat().st_ino} {exe}\n'
    mapping+=f'3000-4000 r-xp 0 00:01 {lib.stat().st_ino} {lib}\n'
    assert verify_bridge_overlay_maps(data,mapping)['loaded_executable']==str(exe)
    for changed in ['',mapping.replace(str(exe),'/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge'),
                    mapping.replace(str(lib.stat().st_ino),'0'),mapping.rstrip()+' (deleted)\n']:
        with pytest.raises(RuntimeError,match='BRIDGE_OVERLAY_REJECTED'):
            verify_bridge_overlay_maps(data,changed)


def test_bridge_overlay_only_required_for_explicit_commissioning(monkeypatch):
    import simulator_backend as backend
    monkeypatch.setattr(backend,'active_bridge_overlay',lambda: {'executable':{'path':'/qualified/parameter_bridge'}})
    assert backend.bridge_executable(False)=='parameter_bridge'
    assert backend.bridge_executable(True)=='/qualified/parameter_bridge'
    def missing():raise RuntimeError('BRIDGE_OVERLAY_REJECTED: missing')
    monkeypatch.setattr(backend,'active_bridge_overlay',missing)
    assert backend.bridge_executable(False)=='parameter_bridge'
    with pytest.raises(RuntimeError,match='missing'):backend.bridge_executable(True)


def support_prepare_inputs(tmp_path, monkeypatch):
    import json, yaml, simulator_backend as backend
    import ament_index_python.packages
    world=tmp_path/'input.sdf'
    world.write_text('''<sdf version="1.8"><world name="cell"><model name="pick_support"><static>true</static><link name="support_link"><collision name="support_collision"><geometry><plane><normal>0 0 1</normal><size>2 2</size></plane></geometry><surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision><visual name="support_visual"><pose>9 9 9 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry><material><ambient>0.2 0.3 0.4 1</ambient></material></visual></link></model><model name="bin"><static>true</static></model></world></sdf>''')
    item={'id':'workcell::authored_table','source_item_id':'authored_table','semantic_role':'support_surface','frame_id':'world','operation':'ADD',
          'pose':{'xyz':[.3,-.4,.2],'quaternion_xyzw':[0.,0.,2**-.5,2**-.5]},
          'collision_geometry':{'type':'box','dimensions_m':[.9,.6,.4]}}
    manifest=tmp_path/'manifest.yaml';manifest.write_text(yaml.safe_dump({'objects':[item]}))
    controllers=tmp_path/'controllers.yaml';controllers.write_text(yaml.safe_dump({'controller_manager':{'ros__parameters':{'arm':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},'arm':{'ros__parameters':{'joints':['j']}}}))
    lib=tmp_path/'lib';lib.mkdir();(lib/'libworkcell_simulator_measurements.so').write_bytes(b'telemetry')
    control=lib/'control.so';control.write_bytes(b'control')
    monkeypatch.setattr(ament_index_python.packages,'get_package_prefix',lambda _:str(tmp_path))
    monkeypatch.setattr(backend,'simulator_control_contract',lambda:dict(backend.DEFAULT_CONTROL_CONTRACT,plugin_library=str(control)))
    monkeypatch.setattr(backend,'simulator_description',lambda *args:'<robot/>')
    monkeypatch.setenv('ROS_DOMAIN_ID','199');monkeypatch.setenv('IGN_PARTITION','test_support_parity')
    return world,manifest,controllers,item


def test_prepare_binds_moved_rotated_support_to_manifest_and_preserves_identity(tmp_path,monkeypatch):
    import json, math, xml.etree.ElementTree as ET
    from simulator_backend import prepare,digest
    world,manifest,controllers,item=support_prepare_inputs(tmp_path,monkeypatch)
    original=ET.parse(world).getroot();output=tmp_path/'runtime'
    _,_,spec=prepare('<robot/>',controllers,world,'rsp',output,collision_manifest_path=manifest)
    root=ET.parse(output/'world.sdf').getroot();model=root.find("world/model[@name='pick_support']")
    pose=[float(v) for v in model.findtext('pose').split()]
    assert pose==pytest.approx([.3,-.4,.2,0,0,math.pi/2])
    for kind,name in [('collision','support_collision'),('visual','support_visual')]:
        shape=model.find(f"link[@name='support_link']/{kind}[@name='{name}']")
        assert [float(v) for v in shape.findtext('geometry/box/size').split()]==pytest.approx([.9,.6,.4])
        assert shape.find('geometry/plane') is None
        assert [float(v) for v in shape.findtext('pose').split()]==[0.]*6
    assert model.findtext('.//surface/friction/ode/mu')=='0.8'
    assert model.findtext('.//visual/material/ambient')=='0.2 0.3 0.4 1'
    assert ET.tostring(root.find("world/model[@name='bin']"))==ET.tostring(original.find("world/model[@name='bin']"))
    assert spec['measurement_pose_source']=='physics_link_frame_data_at_offset'
    binding=spec['support_geometry_binding']
    assert binding['manifest_path']==str(manifest.resolve())
    assert binding['manifest_sha256']==digest(manifest.read_bytes())
    assert binding['object_id']==item['id']
    assert binding['dimensions_m']==item['collision_geometry']['dimensions_m']
    assert binding['pose_xyz']==item['pose']['xyz']
    assert binding['quaternion_xyzw']==item['pose']['quaternion_xyzw']
    assert json.loads((output/'spec.json').read_text())['support_geometry_binding']==binding


@pytest.mark.parametrize('invalid',['missing_manifest','missing_support','ambiguous','non_world','mesh','zero_size','nan_size','nan_pose','bad_quaternion','remove','duplicate_model','malformed_manifest','placement_frame'])
def test_prepare_rejects_unbound_or_unsupported_support(tmp_path,monkeypatch,invalid):
    import yaml
    from simulator_backend import prepare
    world,manifest,controllers,item=support_prepare_inputs(tmp_path,monkeypatch)
    objects=[item]
    if invalid=='missing_manifest':manifest=None
    elif invalid=='missing_support':objects=[]
    elif invalid=='ambiguous':objects.append(copy.deepcopy(item))
    elif invalid=='non_world':item['frame_id']='map'
    elif invalid=='mesh':item['collision_geometry']['type']='mesh'
    elif invalid=='zero_size':item['collision_geometry']['dimensions_m'][0]=0.
    elif invalid=='nan_size':item['collision_geometry']['dimensions_m'][0]=float('nan')
    elif invalid=='nan_pose':item['pose']['xyz'][0]=float('nan')
    elif invalid=='bad_quaternion':item['pose']['quaternion_xyzw']=[0.,0.,0.,0.]
    elif invalid=='remove':item['operation']='REMOVE'
    elif invalid=='duplicate_model':world.write_text(world.read_text().replace('</world>','<model name="pick_support"/></world>'))
    elif invalid=='placement_frame':world.write_text(world.read_text().replace('<model name="pick_support">','<model name="pick_support" placement_frame="other">'))
    if manifest:manifest.write_text(yaml.safe_dump({'objects':objects}))
    if invalid=='malformed_manifest':manifest.write_text('[]')
    with pytest.raises(ValueError,match='support'):
        prepare('<robot/>',controllers,world,'rsp',tmp_path/'runtime',collision_manifest_path=manifest)
    assert not (tmp_path/'runtime/world.sdf').exists()


def test_canonical_curated_launch_passes_shared_manifest_only_in_simulator_branch():
    import ast, importlib.util
    repo=Path(__file__).parents[1];launch=repo/'scenes/ur5_2f_test/launch/demo.launch.py'
    tree=ast.parse(launch.read_text())
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and
           isinstance(n.func.value,ast.Name) and n.func.value.id=='simulator_backend' and n.func.attr=='prepare']
    assert len(calls)==1
    keyword=next(k for k in calls[0].keywords if k.arg=='collision_manifest_path')
    assert isinstance(keyword.value,ast.Name) and keyword.value.id=='collision_manifest_path'
    branches=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="backend == 'simulator'"]
    assert any(calls[0] in list(ast.walk(branch)) for branch in branches)
    spec=importlib.util.spec_from_file_location('parity_generator',repo/'scripts/generate_workcell_from_cell_definition.py')
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    assert not generator._is_existing_package_generator_owned_output(Path('launch/demo.launch.py'))
    assert not generator._profile_review_launch_owned(launch.parents[1],'ur5_2f_test','world')


@pytest.mark.parametrize('pitch_offset',[0.,1e-9,-1e-9,1e-6,-1e-6])
@pytest.mark.parametrize('pitch_sign',[-1.,1.])
def test_support_binding_preserves_pitched_orientation_at_euler_singularities(tmp_path,monkeypatch,pitch_offset,pitch_sign):
    import math, yaml, xml.etree.ElementTree as ET
    from simulator_backend import bind_support_geometry
    def quaternion(roll,pitch,yaw):
        cr,sr=math.cos(roll/2),math.sin(roll/2)
        cp,sp=math.cos(pitch/2),math.sin(pitch/2)
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
    world,manifest,_,item=support_prepare_inputs(tmp_path,monkeypatch)
    expected=quaternion(.3,pitch_sign*math.pi/2+pitch_offset,.7)
    item['pose']['quaternion_xyzw']=expected
    manifest.write_text(yaml.safe_dump({'objects':[item]}))
    xml,_=bind_support_geometry(world.read_text(),manifest)
    emitted=[float(v) for v in ET.fromstring(xml).findtext("world/model[@name='pick_support']/pose").split()]
    actual=quaternion(*emitted[3:])
    assert min(sum((a-b)**2 for a,b in zip(actual,expected)),sum((a+b)**2 for a,b in zip(actual,expected)))**.5 < 1e-12


def test_moveit_admission_requires_capabilities_and_real_active_shutdown(tmp_path):
    import json
    from simulator_backend import read_moveit_overlay
    manifest,patch,library,data=overlay_fixture(tmp_path)
    for field in ('capabilities','active_shutdown'):
        changed=copy.deepcopy(data)
        changed['move_group'].pop(field,None)
        manifest.write_text(json.dumps(changed))
        with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):
            read_moveit_overlay(manifest,patch,patch.parent/'moveit_humble_capability_teardown.patch')


def test_moveit_identity_includes_default_capabilities_library():
    from simulator_backend import moveit_overlay_identity
    old={'library':{'sha256':'tem'},'move_group':{'executable':{'sha256':'exe'},'capabilities':{'sha256':'old'}}}
    new=copy.deepcopy(old);new['move_group']['capabilities']['sha256']='new'
    assert moveit_overlay_identity(old)!=moveit_overlay_identity(new)


def test_moveit_admission_checks_real_active_proof_contents(tmp_path):
    import json
    from simulator_backend import read_moveit_overlay, digest
    manifest,patch,_,data=overlay_fixture(tmp_path)
    path=Path(data['move_group']['active_shutdown']['results_path'])
    original=json.loads(path.read_text())
    changes=[dict(completed=4),dict(method='synthetic_hold/v1')]
    for key,value in [('pending_at_signal',False),('returncode',-6),('production_certificate_after_signal',False),('forced_cleanup',True)]:
        rows=copy.deepcopy(original['results']);rows[0][key]=value;changes.append(dict(results=rows))
    for fields in changes:
        changed=dict(original,**fields);path.write_text(json.dumps(changed))
        data['move_group']['active_shutdown']['results_sha256']=digest(path.read_bytes())
        manifest.write_text(json.dumps(data))
        with pytest.raises(RuntimeError,match='MOVEIT_OVERLAY'):
            read_moveit_overlay(manifest,patch,patch.parent/'moveit_humble_capability_teardown.patch')


def simulator_shutdown_process(tmp_path, mode):
    """Exercise serve's real signals/reaping; replace only external Gazebo startup."""
    import json
    import subprocess
    child=tmp_path/'child.py'
    child.write_text('''import signal,sys,time
from pathlib import Path
root=Path(sys.argv[1]);mode=sys.argv[2]
def stop(signum,frame):
    with (root/'interrupts').open('a') as stream:stream.write('SIGINT\\n')
    if mode=='ignore':return
    time.sleep(.5)
    (root/'finished').write_text('terminal')
    raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
(root/'ready').write_text('ready')
while True:time.sleep(.05)
''')
    wrapper=tmp_path/'wrapper.py'
    wrapper.write_text('''import sys,time
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import simulator_backend as backend
root=Path(sys.argv[2]);mode=sys.argv[3]
Original=backend.subprocess.Popen
class Child(Original):
    def wait(self,timeout=None):
        if mode=='ignore' and timeout==10:timeout=.15
        return super().wait(timeout=timeout)
backend.subprocess.Popen=lambda *a,**kw:Child([sys.executable,str(root/'child.py'),str(root),mode])
def stats(*a,**kw):
    deadline=time.monotonic()+3
    while not (root/'ready').exists():
        if time.monotonic()>deadline:raise RuntimeError('child did not become ready')
        time.sleep(.01)
    return 'iterations: 2'
backend.run_ign=stats
backend.create_model=lambda *a:dict(attempts=[{}],recovered_by_scene_readback=False)
try:raise SystemExit(backend.serve(root))
except KeyboardInterrupt:pass
''')
    (tmp_path/'spec.json').write_text(json.dumps(dict(world='unit',model='unit')))
    return subprocess.Popen([sys.executable,str(wrapper),str(Path(__file__).parents[1]/'scripts'),str(tmp_path),mode],
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)


def wait_shutdown_file(path, process):
    import time
    deadline=time.monotonic()+4
    while not path.exists():
        assert process.poll() is None,process.communicate()
        assert time.monotonic()<deadline,str(path)
        time.sleep(.01)


def test_simulator_repeated_stop_cannot_interrupt_child_reaping(tmp_path):
    """A second launch signal must not leave a live simulator behind exit zero."""
    import os
    import signal
    import time
    process=simulator_shutdown_process(tmp_path,'normal')
    try:
        wait_shutdown_file(tmp_path/'receipt.json',process)
        process.send_signal(signal.SIGINT)
        wait_shutdown_file(tmp_path/'interrupts',process)
        process.send_signal(signal.SIGTERM)
        time.sleep(.1)
        assert process.poll() is None,'wrapper returned before its child reached terminal'
        assert process.wait(timeout=3)==0,process.communicate()
        assert (tmp_path/'finished').read_text()=='terminal'
        assert (tmp_path/'interrupts').read_text().splitlines()==['SIGINT']
    finally:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.communicate(timeout=3)


def test_simulator_internal_forced_kill_is_a_failure(tmp_path):
    import os
    import signal
    process=simulator_shutdown_process(tmp_path,'ignore')
    try:
        wait_shutdown_file(tmp_path/'receipt.json',process)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=3)!=0,'forced simulator termination was reported as clean'
        assert not (tmp_path/'finished').exists()
    finally:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.communicate(timeout=3)

# Verifier/model conversion units only. These samples never count as physical evidence.
@pytest.mark.parametrize('ratio',[1.,-1.])
@pytest.mark.parametrize('joint_type',['revolute','continuous'])
def test_native_description_preserves_authored_mimic_relation(tmp_path,monkeypatch,ratio,joint_type):
    import simulator_backend as backend
    import xml.etree.ElementTree as ET
    (tmp_path/'robot.urdf').write_text(f'<robot><joint name="driver" type="revolute"/><joint name="follower" type="{joint_type}"><mimic joint="driver" multiplier="{ratio}"/></joint></robot>')
    converted='<sdf><model><joint name="driver"><axis/></joint><joint name="follower"><axis/></joint></model></sdf>'
    monkeypatch.setattr(backend.subprocess,'run',lambda *args,**kw:NS(stdout=converted,stderr=''))
    relations=backend.native_simulator_description(tmp_path)
    root=ET.parse(tmp_path/'robot.sdf').getroot();mimic=root.find("model/joint[@name='follower']/axis/mimic")
    assert relations==[dict(follower='follower',driver='driver',multiplier=ratio)]
    assert mimic.get('joint')=='driver' and float(mimic.findtext('multiplier'))==ratio
    assert mimic.findtext('offset')=='0' and mimic.findtext('reference')=='0'

@pytest.mark.parametrize('driver,ratio,offset',[('missing',1.,0.),('driver',.5,0.),('driver',1.,.01),('driver',float('nan'),0.)])
def test_native_description_rejects_unsupported_relations_before_publication(tmp_path,monkeypatch,driver,ratio,offset):
    import simulator_backend as backend
    (tmp_path/'robot.urdf').write_text(f'<robot><joint name="driver" type="revolute"/><joint name="follower" type="revolute"><mimic joint="{driver}" multiplier="{ratio}" offset="{offset}"/></joint></robot>')
    converted='<sdf><model><joint name="driver"><axis/></joint><joint name="follower"><axis/></joint></model></sdf>'
    monkeypatch.setattr(backend.subprocess,'run',lambda *args,**kw:NS(stdout=converted,stderr=''))
    with pytest.raises(ValueError,match='authored flat'):backend.native_simulator_description(tmp_path)
    assert not (tmp_path/'robot.sdf').exists()


def test_native_create_uses_bound_sdf_without_changing_spawn_ownership(tmp_path,monkeypatch):
    import simulator_backend as backend
    calls=[]
    monkeypatch.setattr(backend,'run_ign',lambda args,*a,**kw:calls.append(args) or 'data: true')
    monkeypatch.setattr(backend,'wait_world_model',lambda *a,**kw:dict(found=True))
    result=backend.create_model(tmp_path,dict(world='world',model='robot',native_physics={'bound':True}))
    assert 'robot.sdf' in calls[0][-1] and 'robot.urdf' not in calls[0][-1]
    assert len(result['attempts'])==1 and result['attempts'][0]['readback']['found'] is True

@pytest.mark.parametrize('changed',['upstream_source','control_build'])
def test_native_contract_rejects_changed_build_inputs(tmp_path,monkeypatch,changed):
    import json
    import simulator_backend as backend
    import build_simulator_physics_overlay as builder
    proof=Path(backend.__file__).parent/'simulator_physics_repro'
    (tmp_path/'lib').mkdir()
    artifacts={}
    for name in builder.LIBRARIES:
        p=tmp_path/'lib'/name;p.write_bytes(b'verifier fixture, not a runtime library')
        artifacts[name]=dict(path=str(p),sha256=backend.digest(p.read_bytes()))
    sources={name:dict(url=url,archive_sha256=sha,patch_sha256=backend.digest((proof/builder.PATCHES[name]).read_bytes())) for name,(url,sha) in builder.SOURCES.items()}
    data=dict(schema='workcell_simulator_physics_build/v1',build_script_sha256=backend.digest(Path(builder.__file__).read_bytes()),sources=sources,artifacts=artifacts,control_build_sha256=backend.digest((proof/'control.CMakeLists.txt').read_bytes()))
    if changed=='upstream_source':data['sources']['physics']['archive_sha256']='0'*64
    else:data['control_build_sha256']='0'*64
    (tmp_path/'artifacts.json').write_text(json.dumps(data))
    monkeypatch.setenv('WORKCELL_SIMULATOR_PHYSICS_ROOT',str(tmp_path))
    with pytest.raises(RuntimeError,match='SIMULATOR_IDENTITY_REJECTED'):backend.simulator_physics_contract()


def test_native_builder_rejects_existing_build_cache_before_mutation(tmp_path):
    import subprocess
    import build_simulator_physics_overlay as builder
    output=tmp_path/'output';cache=output/'build/bullet/CMakeCache.txt'
    cache.parent.mkdir(parents=True);cache.write_text('USE_DOUBLE_PRECISION:BOOL=ON\n')
    invalid=tmp_path/'invalid-source';invalid.write_bytes(b'invalid pinned source')
    result=subprocess.run([sys.executable,'-B',builder.__file__,'--output',str(output),'--physics-source',str(invalid)],capture_output=True,text=True,timeout=10)
    assert result.returncode!=0 and 'new or empty output directory' in result.stderr
    assert cache.read_text()=='USE_DOUBLE_PRECISION:BOOL=ON\n'
    assert not (output/'build.log').exists()


def test_selected_native_physics_rejects_live_stock_receipt(tmp_path,monkeypatch):
    import json
    import simulator_backend as backend
    control=tmp_path/'stock-control.so';control.write_bytes(b'unit fixture')
    data=dict(domain='226',partition='unit-partition',pid=123,start_ticks=456,
              control_plugin_library=str(control),control_plugin_sha256=backend.digest(control.read_bytes()))
    for name,key in [('world.sdf','world_sha256'),('controllers.yaml','controllers_sha256'),('robot.urdf','description_sha256')]:
        p=tmp_path/name;p.write_bytes(b'unit fixture');data[key]=backend.digest(p.read_bytes())
    receipt=tmp_path/'receipt.json';receipt.write_text(json.dumps(data))
    monkeypatch.setenv('ROS_DOMAIN_ID',data['domain']);monkeypatch.setenv('IGN_PARTITION',data['partition'])
    monkeypatch.setattr(backend,'process_info',lambda pid:dict(start_ticks=456,command='gazebo -s -r '+str(tmp_path/'world.sdf'),libraries=str(control)+'\nlibignition-gazebo6',environ={'ROS_DOMAIN_ID':data['domain'],'IGN_PARTITION':data['partition']}))
    monkeypatch.setattr(backend,'simulator_physics_contract',lambda:dict(selected_native=True))
    with pytest.raises(RuntimeError,match='SIMULATOR_IDENTITY_REJECTED'):backend.verify_receipt_process(receipt)


def _boundary_fixture(tmp_path,monkeypatch,**overrides):
    """Bound snapshot, normalized objects and a fake live acquisition for the execution boundary."""
    import math
    from types import SimpleNamespace as N
    import simulator_observations
    receipt=tmp_path/'receipt.json';receipt.write_text('{"world":"a0","run_id":"r"}')
    world=tmp_path/'world.sdf';world.write_text('<sdf/>')
    size=overrides.get('world_dimensions',[.025,.025,.025])
    monkeypatch.setattr(simulator_observations,'box_models',lambda path:({'world':'a0'},overrides.get('world_models',{'part_04':size})))
    from simulator_backend import digest
    bound=dict(schema_version='detected_objects/v1',source='old',simulator_receipt_sha256=overrides.get('bound_receipt',digest(receipt.read_bytes())),
        objects=[dict(object_id='part_04',class_id=overrides.get('bound_class','part'),confidence=None,timestamp=1000.,
            pose=dict(frame_id='world',xyz=[.4,-.2,.0125],rpy=[0.,0.,.1]),dimensions=[.025]*3)])
    objects=[dict(id='runtime::part_04',object_id='part_04',timestamp=1000.)]
    yaw=.1+overrides.get('yaw_delta',0.);q=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]
    pose=[.4+overrides.get('dx',0.),-.2,.0125+overrides.get('dz',0.)]+q
    models=overrides.get('world_models',{'part_04':size})
    sample=dict(iteration=5,sim_ns=5000000,wall_ns=overrides.get('wall_ns',2_000_000_000_000),run_id='r',
        poses={'a0::'+name:pose for name in models},joints={'a0::robot::shoulder_pan_joint':[overrides.get('pan',1.57),0.],'a0::robot::elbow_joint':[1.83,0.]})
    def fresh():
        if overrides.get('stale'):raise RuntimeError('SIMULATOR_TELEMETRY_INVALID: stale measurement')
        return sample
    m=N(fresh=fresh,receipt_path=receipt,receipt={'world':'a0','model':'robot','run_id':'r'},
        object_pose=lambda s,name:s['poses']['a0::'+name],
        joints=lambda s:{k.split('::',2)[2]:v for k,v in s['joints'].items()})
    start=dict(names=['shoulder_pan_joint','elbow_joint'],positions=[1.57,1.83])
    return bound,objects,m,start,sample


def test_execution_boundary_refresh_renews_only_measured_timestamps(tmp_path,monkeypatch):
    from simulator_observations import refresh_at_execution_boundary
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch,dz=-1e-6,pan=1.572)
    rebound,evidence=refresh_at_execution_boundary(bound,objects,m,start)
    assert objects[0]['timestamp']==sample['wall_ns']/1e9==rebound['objects'][0]['timestamp']
    assert rebound['objects'][0]['pose']==bound['objects'][0]['pose']   # bound geometry kept, not re-measured
    assert evidence['iteration']==5 and evidence['run_id']=='r' and evidence['geometry']['max_position_delta_m']>0
    assert evidence['start_state_max_deviation_rad']==pytest.approx(.002)
    assert evidence['original_timestamps']=={'part_04':1000.}
    # The renewed stamp is the sample's own measurement time, not the wall clock.
    import time;assert objects[0]['timestamp']==2000. and abs(2000.-time.time())>1e6


@pytest.mark.parametrize('change,match',[
    (dict(dx=2e-5),'geometry changed'),(dict(yaw_delta=1e-3),'geometry changed'),
    (dict(world_dimensions=[.025,.025,.03]),'geometry changed'),
    (dict(world_models={'part_05':[.025]*3}),'identity changed'),(dict(bound_class='bolt'),None),
    (dict(bound_receipt='other-run'),'crossed simulator runs'),(dict(pan=1.58),'start state diverged'),
    (dict(stale=True),'stale measurement')])
def test_execution_boundary_refresh_rejects_changes(tmp_path,monkeypatch,change,match):
    from simulator_observations import refresh_at_execution_boundary
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch,**change)
    if change.get('bound_class'):
        # No class is measured in simulation (capture also stamps it from --class-id),
        # so production carries the bound class by identity. This exercises the
        # library contract that a differing class is still rejected.
        import simulator_observations
        original=simulator_observations.measured_snapshot
        def mismatched(b,mm,sample=None):
            snap,s=original(b,mm,sample);snap['objects'][0]['class_id']='part';return snap,s
        monkeypatch.setattr(simulator_observations,'measured_snapshot',mismatched);match='class changed'
    with pytest.raises(RuntimeError,match=match):
        refresh_at_execution_boundary(bound,objects,m,start)
    assert objects[0]['timestamp']==1000.   # rejection never renews


def test_execution_boundary_refresh_uses_real_world_geometry_gate(tmp_path,monkeypatch):
    """Without the box_models monkeypatch: dimensions come from the launched world and a
    changed world.sdf (sha mismatch with the receipt) fails closed."""
    import json
    from simulator_backend import digest
    from simulator_observations import refresh_at_execution_boundary
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch)
    monkeypatch.undo()
    world='<sdf><world name="a0"><model name="part_04"><link name="l"><collision name="c"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model></world></sdf>'
    (tmp_path/'world.sdf').write_text(world)
    (tmp_path/'receipt.json').write_text(json.dumps(dict(world='a0',run_id='r',world_sha256=digest(world.encode()))))
    bound['simulator_receipt_sha256']=digest((tmp_path/'receipt.json').read_bytes())
    rebound,evidence=refresh_at_execution_boundary(bound,objects,m,start)
    assert objects[0]['timestamp']==2000.
    (tmp_path/'world.sdf').write_text(world.replace('0.025 0.025 0.025','0.025 0.025 0.030'))
    objects[0]['timestamp']=1000.
    with pytest.raises(ValueError,match='world changed'):refresh_at_execution_boundary(bound,objects,m,start)
    assert objects[0]['timestamp']==1000.


def test_execution_boundary_refresh_rejects_missing_or_nonfinite_joints(tmp_path,monkeypatch):
    from simulator_observations import refresh_at_execution_boundary
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch)
    with pytest.raises(RuntimeError,match='start state diverged'):
        refresh_at_execution_boundary(bound,objects,m,dict(names=['wrist_9_joint'],positions=[0.]))
    sample['joints']['a0::robot::shoulder_pan_joint']=[float('nan'),0.]
    with pytest.raises(RuntimeError,match='start state diverged'):refresh_at_execution_boundary(bound,objects,m,start)
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch)
    objects[0]['object_id']='part_09'
    with pytest.raises(RuntimeError,match='do not match the refreshed observation'):refresh_at_execution_boundary(bound,objects,m,start)


def test_execution_boundary_refresh_requires_every_world_object_measured(tmp_path,monkeypatch):
    from simulator_observations import refresh_at_execution_boundary
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch)
    del sample['poses']['a0::part_04']
    with pytest.raises(RuntimeError,match='measured pose missing'):
        refresh_at_execution_boundary(bound,objects,m,start)
    bound,objects,m,start,sample=_boundary_fixture(tmp_path,monkeypatch)
    with pytest.raises(RuntimeError,match='start is incomplete'):
        refresh_at_execution_boundary(bound,objects,m,dict(names=[],positions=[]))


def destination_prepare_inputs(tmp_path,monkeypatch,floor_top=.1):
    import yaml
    world,manifest,controllers,item=support_prepare_inputs(tmp_path,monkeypatch)
    xml=world.read_text()
    bin_model=('<model name="target_bin_physics"><static>true</static><link name="bin_link">'
               '<collision name="floor"><pose>0.45 0.22 %.17g 0 0 0</pose><geometry><box><size>0.26 0.14 0.01</size></box></geometry></collision>'
               '<collision name="wall_x_minus"><pose>0.325 0.22 0.145 0 0 0</pose><geometry><box><size>0.01 0.14 0.10</size></box></geometry></collision>'
               '</link></model>'%(floor_top-.005))
    world.write_text(xml.replace('</world>',bin_model+'</world>'))
    bin_item={'id':'workcell::target_bin_default','source_item_id':'target_bin_default','semantic_role':'target_bin','frame_id':'world','operation':'ADD',
              'pose':{'xyz':[.45,.22,.1],'quaternion_xyzw':[0.,0.,0.,1.]},'source_geometry':{'type':'mesh'}}
    manifest.write_text(yaml.safe_dump({'objects':[item,bin_item]}))
    return world,manifest,controllers,bin_item


def test_prepare_binds_destination_floor_to_authored_bin_origin(tmp_path,monkeypatch):
    import json
    from simulator_backend import prepare,digest
    world,manifest,controllers,item=destination_prepare_inputs(tmp_path,monkeypatch)
    output=tmp_path/'runtime'
    _,_,spec=prepare('<robot/>',controllers,world,'rsp',output,collision_manifest_path=manifest)
    binding=spec['destination_geometry_binding']
    assert binding['object_id']=='workcell::target_bin_default'
    assert (binding['model'],binding['link'],binding['collision'])==('target_bin_physics','bin_link','floor')
    assert binding['floor_z']==pytest.approx(.1) and binding['floor_xyz'][:2]==pytest.approx([.45,.22])
    assert binding['manifest_sha256']==digest(manifest.read_bytes())
    assert json.loads((output/'spec.json').read_text())['destination_geometry_binding']==binding


def test_prepare_rejects_physics_bin_floor_below_authored_destination(tmp_path,monkeypatch):
    from simulator_backend import prepare
    world,manifest,controllers,_=destination_prepare_inputs(tmp_path,monkeypatch,floor_top=.095)  # stage_a0_sparse before the fix
    with pytest.raises(ValueError,match='destination geometry binding rejected.*0.095000.*0.100000'):
        prepare('<robot/>',controllers,world,'rsp',tmp_path/'runtime',collision_manifest_path=manifest)


def test_prepare_without_bin_model_has_no_destination_binding(tmp_path,monkeypatch):
    from simulator_backend import prepare
    world,manifest,controllers,_=support_prepare_inputs(tmp_path,monkeypatch)
    _,_,spec=prepare('<robot/>',controllers,world,'rsp',tmp_path/'runtime',collision_manifest_path=manifest)
    assert spec['destination_geometry_binding'] is None


def test_sparse_world_bin_floor_sits_at_the_authored_destination_origin():
    import xml.etree.ElementTree as ET, yaml
    from pathlib import Path
    from simulator_backend import bind_destination_geometry
    repo=Path(__file__).parents[1]
    binding=bind_destination_geometry((repo/'scenes/ur5_2f_test/worlds/stage_a0_sparse.sdf').read_text(),
                                      repo/'scenes/ur5_2f_test/config/moveit_collision_objects.yaml')
    assert binding['floor_z']==pytest.approx(.1) and binding['object_id']=='workcell::target_bin_default'
