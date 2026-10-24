"""Primitive simulator contact geometry derived from the tool's own collision meshes."""
import copy
import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
import simulation_collision as sc  # noqa: E402

TIP_MESH = Path('/home/ros2/workcell_ws/install/robotiq_85_description/share/robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.stl')
RECORD = yaml.safe_load((REPO/'catalog/capabilities/end_effectors/ee_robotiq_2f.yaml').read_text())['end_effector']


def stl(path, triangles):
    data = bytearray(80) + struct.pack('<I', len(triangles))
    for tri in triangles:
        data += struct.pack('<12fH', 0, 0, 0, *np.asarray(tri, dtype=float).ravel(), 0)
    path.write_bytes(bytes(data))
    return path


# An L-shaped (concave) pad: the bounding box fills the notch.
L_SHAPE = [[[0, 0, 0], [.04, 0, 0], [.04, .01, 0]], [[0, 0, 0], [.01, .03, .02], [0, .03, .02]],
           [[.04, .01, .02], [.01, .01, 0], [.01, .03, .02]]]


def description(mesh, origin='<origin xyz="0.1 0 0" rpy="0 0 1.5707963267948966"/>', extra=''):
    return (f'<robot name="r"><link name="tip">{extra}<collision>{origin}<geometry><mesh filename="file://{mesh}" scale="1 1 1"/>'
            '</geometry></collision></link><link name="other"><collision><geometry><mesh filename="file:///x.stl"/></geometry>'
            '</collision></link></robot>')


def test_the_box_encloses_the_mesh_lies_on_its_extreme_planes_and_keeps_the_collision_rotation(tmp_path):
    mesh = stl(tmp_path/'l.stl', L_SHAPE)
    bound, binding = sc.bind_mesh_bounding_boxes(description(mesh), ['tip'], evidence='test')
    collision = ET.fromstring(bound).find("link[@name='tip']/collision")
    assert collision.find('geometry/mesh') is None
    assert [float(v) for v in collision.find('geometry/box').get('size').split()] == pytest.approx([.04, .03, .02])
    origin = collision.find('origin')
    # centre (0.02, 0.015, 0.01) rotated 90 deg about z, then offset by xyz
    assert [float(v) for v in origin.get('xyz').split()] == pytest.approx([.1-.015, .02, .01])
    assert [float(v) for v in origin.get('rpy').split()] == pytest.approx([0, 0, math.pi/2])
    record, = binding['links']
    assert record['triangles'] == 3 and record['mesh_sha256'] and record['boxes'][0]['box_centre_m'] == pytest.approx([.02, .015, .01])
    assert ET.fromstring(bound).find("link[@name='other']/collision/geometry/mesh") is not None     # unlisted links untouched
    assert binding['mode'] == 'mesh_bounding_box' and binding['evidence'] == 'test'


@pytest.mark.skipif(not TIP_MESH.is_file(), reason='robotiq_85_description not installed')
def test_the_2f85_slab_boxes_keep_the_real_pad_face_and_set_the_rest_back():
    block = RECORD['simulator_collision']
    bound, binding = sc.bind_mesh_bounding_boxes(description(TIP_MESH, origin=''), ['tip'],
                                                 axis=block['axis'], cuts=tuple(block['cuts_m']))
    record, = binding['links']
    assert record['triangles'] == 2436 and binding['mode'] == 'mesh_slab_bounding_boxes'
    proximal, distal = record['boxes']
    face = lambda box: box['box_centre_m'][1] + box['box_size_m'][1]/2
    # The recorded pad contacts all lie on tip-frame y = 0.0078 m (the real pad face, x 7.0-44.5 mm).
    assert face(distal) == pytest.approx(.0078, abs=1e-5)
    assert face(proximal) == pytest.approx(.0060, abs=1e-5)                    # 1.8 mm behind the pad plane
    lo = distal['box_centre_m'][0] - distal['box_size_m'][0]/2
    assert 0.004 < lo < 0.007 and distal['box_size_m'][2] == pytest.approx(.022, abs=1e-6)   # pad face, not the whole tip
    collisions = ET.fromstring(bound).findall("link[@name='tip']/collision")
    assert len(collisions) == 2 and all(c.find('geometry/box') is not None for c in collisions)


def test_slab_boxes_enclose_every_triangle_and_fail_closed_otherwise():
    tris = np.array(L_SHAPE, dtype=float)
    boxes = sc.slab_boxes(tris, 'x', (0.02,))
    assert len(boxes) == 2
    for tri in tris:
        assert any(((tri >= lo - 1e-12) & (tri <= hi + 1e-12)).all() for lo, hi in boxes)
    assert len(sc.slab_boxes(tris)) == 1 and len(sc.slab_boxes(tris, 'x', (5.0,))) == 1     # empty slabs dropped


@pytest.mark.parametrize('xml,message', [
    ('<robot><link name="tip"/></robot>', 'exactly one collision'),
    ('<robot><link name="tip"><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>', 'not a mesh'),
    ('<robot><link name="other"/></robot>', 'exactly one collision'),
])
def test_unbindable_links_fail_closed(xml, message):
    with pytest.raises(sc.SimulationCollisionError, match=message):
        sc.bind_mesh_bounding_boxes(xml, ['tip'])


def test_two_collisions_or_a_bad_mesh_fail_closed(tmp_path):
    mesh = stl(tmp_path/'l.stl', L_SHAPE)
    twice = description(mesh, extra=f'<collision><geometry><mesh filename="file://{mesh}"/></geometry></collision>')
    with pytest.raises(sc.SimulationCollisionError, match='exactly one collision'):
        sc.bind_mesh_bounding_boxes(twice, ['tip'])
    (tmp_path/'ascii.stl').write_text('solid x\nendsolid x\n')
    with pytest.raises(sc.SimulationCollisionError, match='binary STL'):
        sc.bind_mesh_bounding_boxes(description(tmp_path/'ascii.stl'), ['tip'])
    with pytest.raises(sc.SimulationCollisionError, match='mesh scale'):
        sc.bind_mesh_bounding_boxes(description(mesh).replace('scale="1 1 1"', 'scale="1 0 1"'), ['tip'])


def test_the_capability_block_is_validated_and_absent_means_unchanged():
    assert sc.block_errors(RECORD) == []
    assert RECORD['simulator_collision']['links'] == ['gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link']
    for change, message in ((lambda b: b.update(mode='convex_hull'), 'mode'), (lambda b: b.update(links=[]), 'links'),
                            (lambda b: b.pop('evidence'), 'evidence'), (lambda b: b.update(axis='w'), 'axis'),
                            (lambda b: b.update(cuts_m=[.02, .01]), 'cuts_m'), (lambda b: b.update(cuts_m=[]), 'cuts_m'),
                            (lambda b: b.update(mode='mesh_bounding_box'), 'no axis or cuts_m')):
        bad = copy.deepcopy(RECORD); change(bad['simulator_collision'])
        assert any(message in e for e in sc.block_errors(bad))
        with pytest.raises(sc.SimulationCollisionError):
            sc.bind_from_capability('<robot/>', bad)
    unchanged = '<robot name="r"/>'
    assert sc.bind_from_capability(unchanged, {'family': 'finger_gripper'}) == (unchanged, None)


def test_the_launch_binds_primitive_pads_in_the_simulator_branch_only_before_derivation():
    launch = (REPO/'scenes/ur5_2f_test/launch/demo.launch.py').read_text()
    simulator = launch[launch.index('if backend == "simulator":'):launch.index('ExecuteProcess(cmd=[sys.executable, simulator_backend.__file__')]
    assert simulator.index('end_effector_hold.bind_source_description(') < \
        simulator.index('simulation_collision.bind_from_capability(') < simulator.index('simulator_backend.prepare(') < \
        simulator.index("simulator_spec['simulator_collision_primitives'] = simulation_collision.augment_spec(")
    assert launch.count('simulation_collision.') == simulator.count('simulation_collision.')       # never on the fake path
    assert 'simulation_collision' not in (REPO/'scripts/simulator_backend.py').read_text()          # evidence-bound module untouched
