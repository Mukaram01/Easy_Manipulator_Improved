import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stage_a_rgbd_world import rgbd_world

WORLD = Path(__file__).resolve().parents[1] / 'scenes/ur5_2f_test/worlds/stage_a0.sdf'

def test_disabled_world_is_byte_identical():
    text = WORLD.read_text()
    assert rgbd_world(text) == text

def test_optional_camera_preserves_physics_and_models():
    text = WORLD.read_text()
    before = ET.fromstring(text).find('world')
    after = ET.fromstring(rgbd_world(text, [0.4, -0.217, 0.614, 0, 1.5707963267948966, 0])).find('world')
    assert ET.tostring(before.find('physics')) == ET.tostring(after.find('physics'))
    assert [ET.tostring(m) for m in before.findall('model')] == [ET.tostring(m) for m in after.findall('model')[:-1]]
    sensor = after.find("model[@name='stage_a_camera']/link/sensor")
    assert sensor.get('type') == 'rgbd_camera'
    assert sensor.findtext('camera/optical_frame_id') == 'stage_a_camera_optical_frame'
    assert sensor.findtext('camera/image/width') == '512'

@pytest.mark.parametrize('pose', [[0]*5, [0]*5+[float('nan')]])
def test_invalid_pose_rejected(pose):
    with pytest.raises(ValueError):
        rgbd_world(WORLD.read_text(), pose)

from stage_a_rgbd_snapshot import transform_snapshot
from epd_snapshot_adapter import validate_normalized_snapshot

def capture_snapshot():
    return {'schema_version':'workcell_perception_snapshot/v1', 'scene_id':'ur5_2f_test',
            'camera_id':'stage_a_camera','timestamp':100,'frame_id':'stage_a_camera_optical_frame',
            'objects':[{'object_id':'epd_100_0','label':'cube','confidence':0.9,'centroid':[0,0,0.5]}],
            'source':{'camera_pose_source':'live_scene_info','camera_is_static':True, 'camera_definition_source':'live_generate_world_sdf',
                      'camera_world_pose':[0.4,-0.2,0.6,0,2**-0.5,0,2**-0.5]}}

def test_verified_downward_camera_transform():
    result=transform_snapshot(capture_snapshot(),[0.4,-0.2,0.6,0,1.5707963267948966,0])
    assert result['objects'][0]['centroid']==pytest.approx([0.4,-0.2,0.1])
    assert result['frame_id']=='world'
    assert validate_normalized_snapshot(result)==[]
    assert 'dimensions_xyz' not in result['objects'][0]

@pytest.mark.parametrize('change', ['missing','mismatch','nonfinite','dynamic'])
def test_unverified_transform_rejected(change):
    snapshot=capture_snapshot()
    if change=='missing':snapshot['source'].pop('camera_world_pose')
    if change=='mismatch':snapshot['source']['camera_world_pose'][0]=10
    if change=='nonfinite':snapshot['source']['camera_world_pose'][0]=float('nan')
    if change=='dynamic':snapshot['source']['camera_is_static']=False
    with pytest.raises(ValueError):transform_snapshot(snapshot,[0.4,-0.2,0.6,0,1.5707963267948966,0])


def test_pose_bearing_snapshot_cannot_retain_optical_pose_in_world():
    snapshot=capture_snapshot()
    snapshot['objects'][0]['pose']={'position':[0,0,0.5], 'orientation_xyzw':[0,0,0,1]}
    with pytest.raises(ValueError, match='centroid-only'):
        transform_snapshot(snapshot,[0.4,-0.2,0.6,0,1.5707963267948966,0])
