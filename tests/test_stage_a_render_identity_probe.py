"""Opt-in native capability checks; no Gazebo server or EPD capture."""
import json
import os
from pathlib import Path
import subprocess
import pytest


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    source=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd'
    build=tmp_path_factory.mktemp('render_probe_build')
    if subprocess.run(['pkg-config','--exists','ignition-rendering6','ignition-utils1','jsoncpp','ignition-rendering6-ogre2','OGRE-Next'],check=False).returncode:
        pytest.skip('Fortress rendering development libraries unavailable')
    subprocess.run(['cmake','-S',str(source),'-B',str(build),'-DSTAGE_A_RENDER_PROBE_ONLY=ON'],check=True,capture_output=True)
    subprocess.run(['cmake','--build',str(build),'--target','stage_a_render_identity_probe','stage_a_ogre2_image_probe','-j1'],check=True,capture_output=True)
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


def test_ogre2_produces_nonempty_images(probe,tmp_path):
    if not os.environ.get('DISPLAY'):pytest.skip('requires disposable graphics context')
    output=tmp_path/'images.json'
    environment=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1')
    result=subprocess.run([str(probe.with_name('stage_a_ogre2_image_probe')),'ogre2',str(output),'--images'],env=environment,capture_output=True,text=True,timeout=90)
    assert output.exists(),result.stderr
    report=json.loads(output.read_text())
    assert result.returncode==0,report
    assert report['image_production']=='PASS'
    assert report['pixel_registration'].startswith(('BLOCKED','UNQUALIFIED'))
    assert len(report['native_camera_matrices'])==3
    assert report['rgb_nonblank'] and report['finite_depth_pixels']>0
    assert report['segmentation_labels']==[11,22]
    assert report['width']==report['height']==512
    assert not report['contact_authority']
    assert report['decision']=='BLOCKED'


def test_ogre_next_target_rejects_ogre1_before_loading_it(probe,tmp_path):
    binary=probe.with_name('stage_a_ogre2_image_probe');output=tmp_path/'wrong_engine.json'
    result=subprocess.run([str(binary),'ogre',str(output),'--images'],capture_output=True,text=True,timeout=5)
    assert result.returncode==2
    report=json.loads(output.read_text())
    assert report['reason']=='RENDER_ENGINE_PROFILE_UNSUPPORTED'
    assert not report['contact_authority']
