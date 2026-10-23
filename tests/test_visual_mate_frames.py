"""Visual mate selection (R3.2): the resolver publishes datum world frames for
viewport picking, and the seated proposal goes through the existing edit path."""
import copy
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import asset_mounting as m  # noqa: E402
from test_asset_datum_mating import cell, by_id, datum_world  # noqa: E402

ZONE = {'id': 'drop_zone', 'category': 'zone', 'type': 'place_zone', 'pose_xyz': [0.45, 0.22, 0.11], 'pose_rpy': [0., 0., 0.]}


def test_frames_are_resolver_world_frames_for_physical_assets_only():
    rows = cell() + [copy.deepcopy(ZONE)]
    frames = m.datum_frames(rows)
    assert 'drop_zone' not in frames  # task zones are not mounting targets
    bottom = frames['bin']['datums']['bottom']
    assert bottom['datum_type'] == 'plane' and 'reviewed catalog datum (bin_standard)' in bottom['source']
    expected = datum_world(rows, 'bin', 'origin')
    assert frames['bin']['world']['xyz'] == pytest.approx(expected[0])
    assert bottom['xyz'] == pytest.approx([0.45, 0.22, 0.0])
    assert bottom['normal'] == pytest.approx([0., 0., -1.], abs=1e-12)
    surface = frames['table']['datums']['surface']
    assert surface['source'] == 'reviewed support surface (surface_z_m)'
    assert surface['xyz'] == pytest.approx([0.71, -0.2, -0.0009]) and surface['normal'] == pytest.approx([0., 0., 1.])
    assert set(frames['stand']['datums']) >= {'origin', 'top', 'base', 'mast_axis'}
    assert set(frames['camera']['datums']) == {'origin'}


def test_frames_follow_a_mounted_parent_and_rotation():
    rows = m.edit(cell(), 'bin', parent_object='table', parent_link='surface', child_datum='bottom', alignment='flush',
                  relative_pose={'xyz': [-0.26, 0.42, 0.], 'rpy': [0., 0., 0.]})
    rows = m.edit(rows, 'table', world_pose={'xyz': [1., 0., 0.], 'rpy': [0., 0., math.pi / 2]})
    frames = m.datum_frames(rows)
    bottom, top = frames['bin']['datums']['bottom'], frames['table']['datums']['surface']
    # Flush: the bin bottom lies in the table top plane with opposed normals.
    gap = sum((a - b) * n for a, b, n in zip(bottom['xyz'], top['xyz'], top['normal']))
    assert gap == pytest.approx(0., abs=1e-12)
    assert bottom['normal'] == pytest.approx([-v for v in top['normal']], abs=1e-12)
    assert bottom['x_axis'] == pytest.approx(top['x_axis'], abs=1e-12)


def test_frames_refuse_stale_world_poses():
    rows = m.edit(cell(), 'bin', parent_object='table', parent_link='surface', child_datum='bottom', alignment='flush',
                  relative_pose={'xyz': [-0.26, 0.42, 0.], 'rpy': [0., 0., 0.]})
    by_id(rows)['bin']['pose_xyz'][2] += 0.01
    with pytest.raises(ValueError, match='stale world pose'):
        m.datum_frames(rows)


def test_seated_proposal_measures_then_seats_through_edit():
    """The GUI measures the current offset (edit without relative_pose keeps the
    world pose), keeps X, Y and yaw, and seats with zero gap and tilt."""
    rows = cell()
    by_id(rows)['bin']['pose_rpy'] = [0., 0., 0.3]
    measured = m.edit(rows, 'bin', parent_object='table', parent_link='surface', child_datum='bottom', alignment='flush')
    origin = by_id(measured)['bin']['origin']
    assert origin['z'] == pytest.approx(0.0009)  # the bin is 0.9 mm above the table top
    assert origin['roll'] == pytest.approx(0., abs=1e-12) and origin['pitch'] == pytest.approx(0., abs=1e-12)
    seated = m.edit(rows, 'bin', parent_object='table', parent_link='surface', child_datum='bottom', alignment='flush',
                    relative_pose={'xyz': [origin['x'], origin['y'], 0.], 'rpy': [0., 0., origin['yaw']]})
    b = by_id(seated)['bin']
    assert b['pose_xyz'] == pytest.approx([0.45, 0.22, 0.0991])  # only the gap closes
    assert b['pose_rpy'] == pytest.approx([0., 0., 0.3])
    assert by_id(rows)['bin']['pose_xyz'] == [0.45, 0.22, 0.1]  # dry runs never mutate the input


def test_cli_datum_frames_operation():
    request = {'operation': 'datum_frames', 'items': cell()}
    result = subprocess.run([sys.executable, str(ROOT / 'scripts' / 'asset_mounting.py')],
                            input=json.dumps(request), capture_output=True, text=True, check=True)
    response = json.loads(result.stdout)
    assert response['ok'] is True
    assert response['result']['table']['datums']['surface']['datum_type'] == 'plane'
