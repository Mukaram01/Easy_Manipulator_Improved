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
        if args[0]=='service' and '/create' in args:
            creates=sum('/create' in item for call in calls for item in call)
            if creates==1:
                raise subprocess.TimeoutExpired(args,timeout)
            return 'data: true\n'
        if args[0]=='service' and '/scene/info' in args:
            return 'name: "part_00"\n'
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
        if args[0]=='service' and '/create' in args:
            create_calls+=1
            raise subprocess.TimeoutExpired(args,timeout)
        if args[0]=='service' and '/scene/info' in args:
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
        if '/create' in args:
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
