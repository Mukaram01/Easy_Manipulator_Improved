"""R2.5B G0/G1: a two-bin cell derives two hollow physics bins, each bound to its own planning bin; one-bin worlds,
receipts and wall identities are unchanged; motion in a multi-destination world is refused."""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import destination_binding as db  # noqa: E402
import simulator_world as sw  # noqa: E402

pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')


@pytest.fixture(scope='module')
def cells(tmp_path_factory):
    from r4_1b_cells import author_cell_a
    from r25b_cells import author_two_bin_cell
    root = tmp_path_factory.mktemp('r25b')
    a, rc_a = author_two_bin_cell(root, 'two_bin_a', destination='a')
    b, rc_b = author_two_bin_cell(root, 'two_bin_b', destination='b')
    assert rc_a == rc_b == 0
    return {'one_bin': author_cell_a(root)[0], 'a': a, 'b': b}


def bin_models(text):
    return {m.get('name'): m for m in ET.fromstring(text).find('world').findall('model') if 'target_bin' in m.get('name')}


def spec_for(tmp_path, text):
    out = tmp_path / 'sim'
    out.mkdir()
    (out / 'world.sdf').write_text(text)
    (out / 'spec.json').write_text('{}')
    return out


def test_one_bin_cell_world_and_binding_are_unchanged(cells, tmp_path):
    text, report = sw.derive_world(cells['one_bin'])
    assert sw.derive_world(cells['one_bin'], physical_destinations='all')[0] == text   # one bin: the same world
    assert 'destinations' not in report and 'physical_destinations' not in report
    binding = db.augment_spec(spec_for(tmp_path, text), cells['one_bin'] / 'config/moveit_collision_objects.yaml', True)
    spec = json.loads((tmp_path / 'sim/spec.json').read_text())
    assert spec == {'destination_geometry_binding': binding}                           # no multi key
    assert set(binding) == {'manifest_path', 'manifest_sha256', 'object_id', 'floor_contract', 'model', 'link',
                            'collision', 'floor_z', 'floor_xyz', 'floor_size_m', 'pose_xyz', 'walls'}
    assert db.wall_object_ids(binding) == [db.WALL_OBJECT_PREFIX + w['collision'] for w in binding['walls']]


def test_two_bin_cell_derives_two_hollow_physics_bins_identically_for_either_task(cells):
    task_text, task_report = sw.derive_world(cells['a'])
    assert set(bin_models(task_text)) == {sw.DESTINATION_MODEL}                       # qualified world: task bin only
    text_a, report_a = sw.derive_world(cells['a'], physical_destinations='all')
    text_b, report_b = sw.derive_world(cells['b'], physical_destinations='all')
    assert text_a == text_b and report_a['world_sha256'] == report_b['world_sha256']   # one world for both tasks
    assert report_a['destination']['asset_ref'] == 'target_bin_default'
    assert report_b['destination']['asset_ref'] == 'target_bin_b'
    models = bin_models(text_a)
    assert set(models) == {'target_bin_physics', 'target_bin_physics_2'}
    by_asset = {d['asset_ref']: d for d in report_a['destinations']}
    assert {d['model'] for d in by_asset.values()} == set(models)
    for asset, d in by_asset.items():
        names = {c.get('name') for c in models[d['model']].iter('collision')}
        assert names == {'floor', 'wall_x_minus', 'wall_x_plus', 'wall_y_minus', 'wall_y_plus'}   # hollow
        assert models[d['model']].findtext('static') == 'true'
    a, b = by_asset['target_bin_default'], by_asset['target_bin_b']
    assert a['interior_world']['x'][0] - b['interior_world']['x'][1] > 2 * a['proxy_wall_thickness_m']   # apart
    assert report_a['planning_only_objects'] == []                                       # both bins are physical


def test_each_physics_bin_binds_its_own_planning_bin_and_floor(cells, tmp_path):
    text, report = sw.derive_world(cells['b'], physical_destinations='all')
    manifest = cells['b'] / 'config/moveit_collision_objects.yaml'
    assert db.augment_spec(spec_for(tmp_path, text), manifest, True) is None            # no single binding
    spec = json.loads((tmp_path / 'sim/spec.json').read_text())
    bindings = spec['destination_geometry_bindings']
    assert spec['destination_geometry_binding'] is None
    by_asset = {d['asset_ref']: d for d in report['destinations']}
    assert {b['source_item_id'] for b in bindings} == set(by_asset)
    for b in bindings:
        d = by_asset[b['source_item_id']]
        assert b['model'] == d['model'] and b['object_id'] == d['object_id']
        assert b['floor_z'] == pytest.approx(d['floor_top_z'], abs=1e-9)
        assert b['floor_contract'] == 'planning_mesh_floor'
    ids = [i for b in bindings for i in db.wall_object_ids(b)]
    assert len(ids) == len(set(ids)) == 8
    assert db.select_destination_binding(bindings, 'target_bin_b')['model'] == by_asset['target_bin_b']['model']
    with pytest.raises(ValueError, match='0 physics bin bindings'):
        db.select_destination_binding(bindings, 'nowhere')
    with pytest.raises(ValueError, match='several bins need derived cell floors'):
        db.augment_spec(tmp_path / 'sim', manifest, False)


def test_overlapping_bins_are_refused():
    def bin_(asset, x):
        return dict(asset_ref=asset, proxy_wall_thickness_m=0.01, interior_world=dict(x=list(x), y=[0.1, 0.3]))
    sw._require_separate_bins([bin_('a', (0.0, 0.29)), bin_('b', (0.31, 0.6))])
    with pytest.raises(sw.SimulatorWorldError, match='overlap'):
        sw._require_separate_bins([bin_('a', (0.0, 0.30)), bin_('b', (0.31, 0.6))])


def test_multi_destination_world_arms_no_physical_guard():
    from simulator_execution import ContactGuard

    class Measurements:
        receipt = {'destination_geometry_bindings': [{}, {}], 'destination_geometry_binding': None}
    with pytest.raises(RuntimeError, match='MULTI_DESTINATION_PHYSICS_UNQUALIFIED'):
        ContactGuard(Measurements(), 'runtime::part', [0.03] * 3, [], None, None, 'tool0')


@pytest.mark.parametrize('cell,asset', [('a', 'target_bin_default'), ('b', 'target_bin_b')])
def test_a_scoped_guard_admits_only_its_own_bin_floor(cells, tmp_path, cell, asset):
    """G3A/G3B: the guard armed for one bin maps only that bin's exact floor collision to it; its walls and every
    collision of the other bin stay unpermitted physical obstacles, and a substituted binding is refused."""
    from simulator_execution import ContactGuard
    text, _ = sw.derive_world(cells[cell], physical_destinations='all')
    db.augment_spec(spec_for(tmp_path, text), cells[cell] / 'config/moveit_collision_objects.yaml', True)
    bindings = json.loads((tmp_path / 'sim/spec.json').read_text())['destination_geometry_bindings']
    selected = db.select_destination_binding(bindings, asset)
    other = next(b for b in bindings if b is not selected)

    class Measurements:
        receipt = {'world': 'a0', 'model': 'workcell_robot', 'destination_geometry_bindings': bindings,
                   'destination_geometry_binding': None}

    class Baseline:
        entry_names, entry_values = [], []
    guard = ContactGuard(Measurements(), 'runtime::part_04', [0.025] * 3, [], None, Baseline(), 'tool0',
                         destination_binding=selected)
    assert guard.destination == dict(object_id='runtime::part_04', support_id='workcell::' + asset,
                                     floor_z=float(selected['floor_z']))
    assert guard.identity(f"a0::{selected['model']}::bin_link::floor") == 'workcell::' + asset
    assert guard.identity(f"a0::{selected['model']}::bin_link::wall_x_plus") == 'workcell::' + selected['model']
    assert guard.identity(f"a0::{other['model']}::bin_link::floor") == 'workcell::' + other['model']
    # A binding that is not one of the launched bins (or none) arms nothing. Which launched bin is armed is decided
    # by the verified scope: the executor passes only the scope-verified bin (test below).
    for substitute in (dict(selected, floor_z=0.5), dict(selected, collision='wall_x_plus'), None):
        with pytest.raises(RuntimeError, match='MULTI_DESTINATION_PHYSICS_UNQUALIFIED'):
            ContactGuard(Measurements(), 'runtime::part_04', [0.025] * 3, [], None, Baseline(), 'tool0',
                         destination_binding=substitute)

    class OneBin:
        receipt = {'world': 'a0', 'model': 'workcell_robot', 'destination_geometry_binding': selected}
    with pytest.raises(RuntimeError, match='single-destination world binds its destination floor in the receipt'):
        ContactGuard(OneBin(), 'runtime::part_04', [0.025] * 3, [], None, Baseline(), 'tool0', destination_binding=selected)


def test_executor_refuses_motion_in_a_multi_destination_world():
    source = (ROOT / 'scripts/perceived_object_grasp_execute.py').read_text()
    block = source[source.index("destination_bindings = receipt_record.get('destination_geometry_bindings')"):]
    block = block[:block.index('elif destination_binding is not None:')]
    assert block.index('if args.start:') < block.index('MULTI_DESTINATION_PHYSICS_UNQUALIFIED') < block.index('apply(')
    # Only an authorised full-cycle with a verified scope passes the refusal; nothing else in the block moves.
    assert "args.physics_scope is None or args.simulator_commission != 'full-cycle'" in block
    assert block.index('verify_trial(') < block.index('scoped_destination = selected') < block.index('apply(')
    assert "scope_record['binding']['destination']['model'] != selected['model']" in block
    assert source.count('destination_binding=scoped_destination') == 1   # the guard arms the scope-verified bin only
    assert block.index('verify_trial(') > block.index("args.retention_policy != 'strict'")   # strict retention only
    assert 'verify_qualified_scope' not in source        # the executor has no qualified-scope admission path


@pytest.mark.parametrize('extra', [[], ['--mode', 'targeted', '--gate', 'full-cycle', '--prior-evidence', 'x'],
                                   ['--mode', 'full'],
                                   ['--mode', 'targeted', '--gate', 'resolve', '--authorise-physics-trial'],
                                   ['--mode', 'targeted', '--gate', 'full-cycle', '--authorise-physics-trial'],
                                   ['--mode', 'full', '--authorise-physics-trial', '--prior-evidence', 'x']])
def test_runner_allows_all_physical_destinations_for_plan_only_resolve_only(tmp_path, extra):
    import run_stage_a1_finish
    argv = ['--output', str(tmp_path / 'out'), '--physical-destinations', 'all', '--scene-package', str(tmp_path), *extra]
    with pytest.raises(SystemExit) as exc:
        run_stage_a1_finish.main(argv)
    assert exc.value.code == 2 and not (tmp_path / 'out').exists()


def test_runner_authorises_a_trial_for_the_task_world_only(tmp_path):
    import run_stage_a1_finish
    argv = ['--output', str(tmp_path / 'out'), '--scene-package', str(tmp_path), '--mode', 'targeted', '--gate',
            'full-cycle', '--prior-evidence', 'x', '--authorise-physics-trial']
    with pytest.raises(SystemExit) as exc:          # a one-bin (task) world has nothing to authorise
        run_stage_a1_finish.main(argv)
    assert exc.value.code == 2 and not (tmp_path / 'out').exists()
