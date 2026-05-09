from pathlib import Path
import ast
import xml.etree.ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = REPO_ROOT / 'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py'


def _load_joint_helpers():
    source = TEMPLATE.read_text(encoding='utf-8')
    module = ast.parse(source)
    names = {
        'UR_ARM_JOINTS', '_collect_movable_urdf_joints', '_derive_chain_joints_from_urdf', '_extract_controller_joints'
    }
    selected = [n for n in module.body if isinstance(n, (ast.FunctionDef, ast.Assign)) and getattr(n, 'name', None) in names or isinstance(n, ast.Assign)]
    ns = {'ET': ET}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(TEMPLATE), 'exec'), ns)
    return ns['_extract_controller_joints']


def test_extract_controller_joints_supports_chain_and_ur_fallback():
    extractor = _load_joint_helpers()
    urdf = '<robot><joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="l1"/></joint><joint name="shoulder_lift_joint" type="revolute"><parent link="l1"/><child link="l2"/></joint><joint name="elbow_joint" type="revolute"><parent link="l2"/><child link="l3"/></joint><joint name="wrist_1_joint" type="revolute"><parent link="l3"/><child link="l4"/></joint><joint name="wrist_2_joint" type="revolute"><parent link="l4"/><child link="l5"/></joint><joint name="wrist_3_joint" type="revolute"><parent link="l5"/><child link="tool0"/></joint></robot>'
    srdf_chain = '<robot><group name="arm"><chain base_link="base_link" tip_link="tool0"/></group></robot>'
    group, joints = extractor(srdf_chain, urdf)
    assert group in {'arm', 'manipulator'}
    assert joints == [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]


def test_armhand_generator_injects_explicit_ur5_manipulator_joints():
    content = (REPO_ROOT / 'workcell_builder/workcell_builder/include/armhand_xacro_parser.h').read_text(encoding='utf-8')
    assert 'if (robot.name == "ur5")' in content
    assert 'MyFile << "  <group name=\\"manipulator\\">\\n";' in content
    for joint in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']:
        assert f'MyFile << "    <joint name=\\"{joint}\\"/>\\n";' in content
