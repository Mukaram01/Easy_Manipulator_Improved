"""Opt-in native capability checks; no Gazebo server or image capture."""
import json
import os
from pathlib import Path
import subprocess
import pytest


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    source=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd'
    build=tmp_path_factory.mktemp('render_probe_build')
    if subprocess.run(['pkg-config','--exists','ignition-rendering6','ignition-utils1','jsoncpp'],check=False).returncode:
        pytest.skip('Fortress rendering development libraries unavailable')
    subprocess.run(['cmake','-S',str(source),'-B',str(build),'-DSTAGE_A_RENDER_PROBE_ONLY=ON'],check=True,capture_output=True)
    subprocess.run(['cmake','--build',str(build),'--target','stage_a_render_identity_probe','-j1'],check=True,capture_output=True)
    return build/'stage_a_render_identity_probe'


def run(probe,engine,output):
    result=subprocess.run([str(probe),engine,str(output)],capture_output=True,text=True,timeout=30)
    return result,json.loads(output.read_text())


def test_unknown_engine_fails_closed(probe,tmp_path):
    result,report=run(probe,'stage_a_unknown_engine',tmp_path/'unknown.json')
    assert result.returncode==2 and report['reason']=='RENDER_ENGINE_UNAVAILABLE'
    assert not report['contact_authority'] and not report['execution_goals']


def test_existing_ogre_backend_cannot_supply_identity_witness(probe,tmp_path):
    if not os.environ.get('DISPLAY'):pytest.skip('requires disposable graphics context')
    result,report=run(probe,'ogre',tmp_path/'ogre.json')
    assert result.returncode==2
    assert report['engine_name']=='ogre'
    assert report['reason']=='SEGMENTATION_CAMERA_UNSUPPORTED'
    assert not report['segmentation_camera_created']
    assert report['decision']=='BLOCKED'
    assert not report['contact_authority'] and not report['moveit_started']
    assert any('rendering6-ogre.so' in p for p in report['loaded_library_paths'])
