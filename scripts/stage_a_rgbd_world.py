#!/usr/bin/env python3
"""Optional sensor derivation of the canonical Stage-A world; never starts a robot."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def rgbd_world(xml, camera_pose=None, render_engine="ogre"):
    if camera_pose is None:
        return xml
    if len(camera_pose) != 6 or not all(math.isfinite(v) for v in camera_pose):
        raise ValueError('camera pose requires six finite xyz/rpy values')
    if render_engine not in ('ogre', 'ogre2'):
        raise ValueError('unsupported render engine')
    root = ET.fromstring(xml)
    world = root.find('world')
    if world is None or world.find("model[@name='stage_a_camera']") is not None:
        raise ValueError('one world without a stage_a_camera required')
    if world.find("plugin[@name='ignition::gazebo::systems::Sensors']") is None:
        plugin = ET.SubElement(world, 'plugin', filename='ignition-gazebo-sensors-system',
                               name='ignition::gazebo::systems::Sensors')
        ET.SubElement(plugin, 'render_engine').text = render_engine
    camera = ET.fromstring('''<model name="stage_a_camera"><static>true</static><pose/>
      <link name="camera_link"><sensor name="rgbd" type="rgbd_camera">
        <always_on>true</always_on><update_rate>2</update_rate><topic>/stage_a/camera</topic>
        <camera><optical_frame_id>stage_a_camera_optical_frame</optical_frame_id>
          <horizontal_fov>1.047</horizontal_fov>
          <image><width>512</width><height>512</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>5</far></clip>
        </camera>
      </sensor></link></model>''')
    camera.find('pose').text = ' '.join(str(v) for v in camera_pose)
    world.append(camera)
    return ET.tostring(root, encoding='unicode')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--render-engine', choices=['ogre', 'ogre2'], default='ogre')
    p.add_argument('--camera-pose', type=float, nargs=6, metavar=('X','Y','Z','R','P','YAW'))
    a = p.parse_args()
    with a.output.open('x') as output:
        output.write(rgbd_world(a.world.read_text(), a.camera_pose, a.render_engine))

if __name__ == '__main__':
    main()
