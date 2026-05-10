from pathlib import Path
import re
import xml.etree.ElementTree as ET


def test_demo_launch_does_not_emit_world_with_trailing_space():
    launch_template = Path(
        'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py'
    ).read_text(encoding='utf-8')
    assert "'world '" not in launch_template
    assert '"world "' not in launch_template


def test_generated_srdf_template_avoids_duplicate_manipulator_group_definition():
    parser = Path('workcell_builder/workcell_builder/include/armhand_xacro_parser.h').read_text(encoding='utf-8')
    assert '<group name=\"manipulator\">' not in parser


def test_robotiq_gripper_base_link_has_collision_geometry():
    gripper_xacro = Path(
        'assets/end_effectors/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro'
    ).read_text(encoding='utf-8')

    link_start = gripper_xacro.find('<link name="${prefix}gripper_base_link">')
    assert link_start >= 0, 'gripper_base_link link block not found'
    link_end = gripper_xacro.find('</link>', link_start)
    assert link_end > link_start, 'gripper_base_link link block not closed'
    link_xml = f'<root>{gripper_xacro[link_start:link_end + len("</link>")]}</root>'
    root = ET.fromstring(link_xml)
    link = root.find('link')
    assert link is not None
    collision = link.find('collision')
    assert collision is not None, 'gripper_base_link is missing collision geometry'
    assert collision.find('geometry/mesh') is not None
