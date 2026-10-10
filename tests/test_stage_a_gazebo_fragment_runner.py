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
    source=world();before=ET.tostring(source)
    result=runner.disposable_world(source,'/owner.so','session','/owner.jsonl')
    assert ET.tostring(source)==before
    w=result.find('world');assert len(w.findall('plugin'))==1
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
