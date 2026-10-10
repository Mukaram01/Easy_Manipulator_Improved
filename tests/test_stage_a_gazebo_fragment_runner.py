"""Disposable-world preflight only. Never starts Gazebo or a renderer."""
import sys
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_gazebo_fragment as runner


def world():
    root=ET.fromstring('<sdf version="1.8"><world name="w"><plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/></world></sdf>')
    for name,x in [('part_00',.35),('part_01',.45)]:
        model=ET.SubElement(root.find('world'),'model',name=name);ET.SubElement(model,'pose').text=f'{x} -.217 .0125 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        for kind in ('visual','collision'):
            row=ET.SubElement(link,kind,name=kind);ET.SubElement(ET.SubElement(ET.SubElement(row,'geometry'),'box'),'size').text='.025 .025 .025'
        material=ET.SubElement(link.find('visual'),'material');ET.SubElement(material,'diffuse').text='1 0 0 1'
    return root


def test_disposable_world_preserves_original_cube_geometry_and_uses_one_owner():
    source=world();ET.SubElement(source.find('world'),'light',name='sun',type='directional');before=ET.tostring(source)
    result=runner.disposable_world(source,'/owner.so','session','/owner.jsonl')
    assert ET.tostring(source)==before
    w=result.find('world');assert len(w.findall('plugin'))==1
    assert not w.findall('light')
    assert w.find('plugin').get('name')=='ignition::gazebo::systems::WorkcellOwnerPhysics'
    assert w.findtext('plugin/owner_record_steps')=='2'
    assert [ET.tostring(m) for m in w.findall('model')]==[ET.tostring(m) for m in source.find('world').findall('model')]


@pytest.mark.parametrize('kind',['duplicate_model','missing_model','multiple_collision','mesh','transparent','extra_visual'])
def test_preflight_rejects_unsupported_scene_before_any_process(kind):
    root=world();w=root.find('world');m=w.find('model');link=m.find('link')
    if kind=='duplicate_model':w.append(ET.fromstring(ET.tostring(m)))
    if kind=='missing_model':w.remove(m)
    if kind=='multiple_collision':link.append(ET.fromstring(ET.tostring(link.find('collision'))))
    if kind=='mesh':link.find('visual/geometry/box').tag='mesh'
    if kind=='transparent':ET.SubElement(link.find('visual'),'transparency').text='.5'
    if kind=='extra_visual':link.append(ET.fromstring(ET.tostring(link.find('visual'))))
    with pytest.raises(ValueError):runner.disposable_world(root,'/owner.so','s','/owner.jsonl')


def test_existing_execution_claim_prevents_a_second_process(tmp_path):
    pre=dict(session='cpu-only')
    for name in ('binary','owner','world'):
        path=tmp_path/name;path.write_text('inert CPU test input')
        pre[name]=str(path);pre[name+'_sha256']=runner.sha(path)
    (tmp_path/'preflight.json').write_text(json.dumps(pre))
    (tmp_path/'execution_claim.json').write_text('{"maximum_runs":1}')
    with pytest.raises(FileExistsError):runner.run(tmp_path)


def qualified():
    name='/qualified/owner.so';digest='a'*64
    pre={'pre_run_sha256':{name:digest}}
    report=dict(decision='PASS_SYSTEMLOADER_ONLY',filename=name,loaded=True,
        instance_class='ignition::gazebo::v6::systems::WorkcellOwnerPhysics',
        registered_classes=['ignition::gazebo::v6::systems::WorkcellOwnerPhysics'],
        requested_class='ignition::gazebo::systems::WorkcellOwnerPhysics',
        system=True,configure=True,update=True,pre_update=False,post_update=False,
        gpu_device_fds=[],physics_steps=0)
    return pre,report,{'returncode':0},{name:digest},digest


def test_repaired_owner_requires_actual_loader_and_loaded_hash_proof():
    runner.validate_owner_qualification(*qualified())


@pytest.mark.parametrize('kind',['stale_elf','wrong_class','duplicate','failed_exit','unloaded','loaded_hash','wrong_alias'])
def test_repaired_owner_provenance_rejects_substitution(kind):
    pre,r,exit,loaded,digest=qualified()
    if kind=='stale_elf':digest='b'*64
    if kind=='wrong_class':r['instance_class']='Physics'
    if kind=='duplicate':r['registered_classes'].append('Physics')
    if kind=='failed_exit':exit['returncode']=-4
    if kind=='unloaded':r['loaded']=False
    if kind=='loaded_hash':loaded[r['filename']]='c'*64
    if kind=='wrong_alias':r['requested_class']='ignition::gazebo::systems::Physics'
    with pytest.raises(ValueError):runner.validate_owner_qualification(pre,r,exit,loaded,digest)


@pytest.mark.parametrize('kind',['extra_plugin','wrong_session','wrong_trace','wrong_owner','sensor','extra_model','light'])
def test_prepared_world_rejects_changed_runtime_configuration(tmp_path,kind):
    root=runner.disposable_world(world(),'/owner.so','s',tmp_path/'owner.jsonl');w=root.find('world')
    pre=dict(world=str(tmp_path/'world.sdf'),owner='/owner.so',session='s',output=str(tmp_path))
    if kind=='extra_plugin':ET.SubElement(w.find('model'),'plugin',name='controller')
    if kind=='wrong_session':w.find('plugin/owner_session').text='old'
    if kind=='wrong_trace':w.find('plugin/owner_output').text='/old/owner.jsonl'
    if kind=='wrong_owner':w.find('plugin').set('name','ignition::gazebo::systems::Physics')
    if kind=='sensor':ET.SubElement(w.find('model/link'),'sensor')
    if kind=='extra_model':ET.SubElement(w,'model')
    if kind=='light':ET.SubElement(w,'light',name='unexpected',type='directional')
    ET.ElementTree(root).write(pre['world'])
    with pytest.raises(ValueError):runner.validate_prepared_world(pre)


def test_cpu_debugger_contract_stops_fatal_signal_without_retry(tmp_path):
    script=runner.debugger_script(tmp_path)
    assert script.splitlines().count('run')==1 and 'continue' not in script.replace('continued_after_signal','')
    assert 'thread apply all bt full' in script and 'gdb.SignalEvent' in script
    assert "if not (out/'child_exit.json').exists()" in script
    p=tmp_path/'stderr';p.write_text('noise\n[workcell-lifecycle] mrt_draw_begin\ncrash\n')
    assert runner.last_lifecycle(p)=='[workcell-lifecycle] mrt_draw_begin'


def test_fatal_child_without_final_report_still_retains_exit_and_marker(tmp_path,monkeypatch):
    from types import SimpleNamespace
    pre=dict(session='cpu',output=str(tmp_path),owner_qualification_sha256={},source_sha256={})
    for name in ('binary','owner','world','debugger'):
        p=tmp_path/name;p.write_text('CPU inert fixture')
        pre[name]=str(p);pre[name+'_sha256']=runner.sha(p)
    (tmp_path/'preflight.json').write_text(json.dumps(pre))
    monkeypatch.setattr(runner,'owner_provenance',lambda *a:{})
    monkeypatch.setattr(runner,'validate_prepared_world',lambda *a:None)
    calls=[]
    def child(*args,**kwargs):
        calls.append(args)
        kwargs['stderr'].write(b'[workcell-lifecycle] server_teardown_begin\n')
        kwargs['stderr'].flush()
        (tmp_path/'child_exit.json').write_text(json.dumps(dict(signal='SIGILL',continued_after_signal=False)))
        return SimpleNamespace(returncode=255)
    monkeypatch.setattr(runner.subprocess,'run',child)
    result=runner.run(tmp_path)
    assert len(calls)==1 and result['decision']=='BLOCKED'
    assert result['failure_reason']=='native fatal signal SIGILL'
    assert 'server_teardown_begin' in result['last_lifecycle_marker']
    assert json.loads((tmp_path/'process_exit.json').read_text())['child']['signal']=='SIGILL'
    with pytest.raises(FileExistsError):runner.run(tmp_path)
    assert len(calls)==1
