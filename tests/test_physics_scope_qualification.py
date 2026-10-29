"""G3A: a workcell_physics_scope/v1 trial record admits one full-cycle into exactly the bound bin of the exact cell,
world, robot, runtime and evidence it was authorised for; every changed, missing or substituted binding refuses."""
import copy
import hashlib
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import destination_binding as db  # noqa: E402
import physics_scope_qualification as psq  # noqa: E402
import simulator_world as sw  # noqa: E402

pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')

NATIVE = {'sha256': 'commission', 'support_sha256': 'support', 'telemetry_sha256': 'telemetry', 'moveit_version': '2.5.10',
          'sources': {'/ws/src_support_contact_adapter.cpp': 'adapter'}}
RUNTIME = {'record_sha256': 'record', 'stage_a_evidence': '/evidence/stage_a', 'stage_a_report_sha256': 'report',
           'stage_a_repo_head': 'head'}


@pytest.fixture(scope='module')
def cells(tmp_path_factory):
    from r25b_cells import author_two_bin_cell
    root = tmp_path_factory.mktemp('g3a')
    a, rc_a = author_two_bin_cell(root, 'two_bin_a', destination='a')
    b, rc_b = author_two_bin_cell(root, 'two_bin_b', destination='b')
    assert rc_a == rc_b == 0
    return {'a': a, 'b': b}


def launch(tmp_path, cell, edit_world=None, occupant=None):
    """A session as the qualified launch leaves it: launched world, receipt with every bin bound, measured parts."""
    text, report = sw.derive_world(cell, physical_destinations='all')
    runtime = tmp_path / 'runtime'
    runtime.mkdir(parents=True)
    (runtime / 'world.sdf').write_text(edit_world(text) if edit_world else text)
    (runtime / 'spec.json').write_text('{}')
    db.augment_spec(runtime, cell / 'config/moveit_collision_objects.yaml', True)
    receipt = json.loads((runtime / 'spec.json').read_text())
    receipt.update(world='a0', run_id='run-1', world_sha256=hashlib.sha256((runtime / 'world.sdf').read_bytes()).hexdigest(),
                   support_geometry_binding=dict(model='pick_support', link='support_link', collision='support_collision',
                                                 object_id='workcell::support_surface_table', manifest_path='/session/x'),
                   expected_controllers=['ur5_arm_controller', 'ur5_gripper_controller'],
                   end_effector_hold={'capability_id': 'robotiq_2f_85', 'sha256': 'hold'},
                   native_mimics=[{'follower': 'f2', 'driver': 'f1', 'multiplier': 1.0}],
                   **{k: 'id-' + k for k in psq.ROBOT_RECEIPT_KEYS
                      if k not in ('expected_controllers', 'end_effector_hold', 'native_mimics')})
    (runtime / 'receipt.json').write_text(json.dumps(receipt))
    parts = [dict(object_id=o['object_id'], dimensions=o['dimensions_m'],
                  pose=dict(frame_id='world', xyz=o['spawn_pose'][:3], rpy=o['spawn_pose'][3:])) for o in report['objects']]
    if occupant:
        parts.append(dict(object_id='occupant', dimensions=[0.025] * 3, pose=dict(frame_id='world', xyz=occupant, rpy=[0, 0, 0])))
    (tmp_path / 'parts.yaml').write_text(yaml.safe_dump({'objects': parts}))
    (tmp_path / 'commission-evidence.json').write_text('[{"result": "CANCELLATION_TRIAL_PASS"}]')
    return tmp_path


def binding(session, cell, native=NATIVE, runtime=RUNTIME, resolution='resolution'):
    return psq.scope_binding(cell_dir=cell, receipt_path=session / 'runtime/receipt.json',
                             observations_path=session / 'parts.yaml', native=native, runtime=runtime,
                             commission_evidence_path=session / 'commission-evidence.json', resolution_sha256=resolution)


def rewrite(path, edit):
    """An edited record with both of its digests recomputed (what a forger would do)."""
    record = json.loads(path.read_text())
    edit(record)
    record['binding_sha256'] = hashlib.sha256(psq._canonical(record['binding'])).hexdigest()
    data = (json.dumps(record) + '\n').encode()
    path.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


BINS = [('a', 'target_bin_default'), ('b', 'target_bin_b')]      # the same world; each cell's task selects its bin


@pytest.mark.parametrize('cell,asset', BINS)
def test_authorised_trial_verifies_against_the_same_recomputed_binding(cells, tmp_path, cell, asset):
    session = launch(tmp_path, cells[cell])
    bound = binding(session, cells[cell])
    sha = psq.authorise_trial(tmp_path / 'scope.json', bound)
    record = psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells[cell]))
    assert record['state'] == 'AUTHORISED_TRIAL' and record['operation'] == psq.TRIAL_OPERATION
    assert record['binding']['destination']['asset'] == asset
    assert record['binding']['destination']['model'] == db.select_destination_binding(
        json.loads((session / 'runtime/receipt.json').read_text())['destination_geometry_bindings'], asset)['model']
    assert len(record['binding']['destinations']) == 2                                   # the other bin is bound too
    assert all('manifest_path' not in d for d in record['binding']['destinations'])     # content bound, not paths


@pytest.mark.parametrize('cell,asset', BINS)
def test_a_trial_for_one_bin_never_admits_the_other(cells, tmp_path, cell, asset):
    other_cell = 'a' if cell == 'b' else 'b'
    session = launch(tmp_path, cells[cell])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells[cell]))
    # The same world with the task pointed at the other bin: the destination (and the cell) no longer match.
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: .*destination'):
        psq.verify_trial(tmp_path / 'scope.json', sha,
                         binding(launch(tmp_path / other_cell, cells[other_cell]), cells[other_cell]))
    other = next(d for d in binding(session, cells[cell])['destinations'] if d['source_item_id'] != asset)
    forged = rewrite(tmp_path / 'scope.json', lambda r: r['binding']['destination'].update(
        model=other['model'], object_id=other['object_id'], floor_z=other['floor_z']))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: destination'):
        psq.verify_trial(tmp_path / 'scope.json', forged, binding(session, cells[cell]))


def test_changed_bin_geometry_refuses(cells, tmp_path):
    session = launch(tmp_path / 'one', cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    receipt = json.loads((session / 'runtime/receipt.json').read_text())
    receipt['destination_geometry_bindings'][0]['floor_z'] += 0.001
    (session / 'runtime/receipt.json').write_text(json.dumps(receipt))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b']))
    # A launched bin that is not the cell's derived bin is refused before any record is written.
    moved = launch(tmp_path / 'two', cells['b'], edit_world=lambda text: text.replace(
        '<size>0.3200500719225441', '<size>0.3300500719225441', 1))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_WORLD_MISMATCH'):
        binding(moved, cells['b'])


def test_source_build_or_runtime_change_refuses(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    rebuilt = copy.deepcopy(NATIVE)
    rebuilt['sources']['/ws/src_support_contact_adapter.cpp'] = 'repaired-adapter'
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: native'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b'], native=rebuilt))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: runtime'):
        psq.verify_trial(tmp_path / 'scope.json', sha,
                         binding(session, cells['b'], runtime=dict(RUNTIME, stage_a_report_sha256='other')))
    # The native identity itself is proved against the bytes on disk.
    libs = {name: tmp_path / f'{name}.so' for name in ('library', 'support_library', 'telemetry_library')}
    for path in libs.values():
        path.write_bytes(path.name.encode())
    source = tmp_path / 'src_support_contact_adapter.cpp'
    source.write_text('qualified')
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps(dict({k: str(v) for k, v in libs.items()}, sha256=digest(libs['library']),
        support_sha256=digest(libs['support_library']), telemetry_sha256=digest(libs['telemetry_library']),
        moveit_version='2.5.10', sources={str(source): digest(source)})))
    assert psq.native_identity(manifest)['support_sha256'] == digest(libs['support_library'])
    libs['support_library'].write_bytes(b'rebuilt support library')          # commission library unchanged
    with pytest.raises(psq.PhysicsScopeRefused, match='support_library differs'):
        psq.native_identity(manifest)
    libs['support_library'].write_bytes(b'support_library.so')
    source.write_text('edited')
    with pytest.raises(psq.PhysicsScopeRefused, match='bound source changed'):
        psq.native_identity(manifest)


def test_missing_or_changed_evidence_refuses(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    (session / 'commission-evidence.json').write_text('[]')
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: evidence'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b']))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_INCOMPLETE: plan resolution'):
        binding(session, cells['b'], resolution=None)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_RUNTIME_UNQUALIFIED'):
        psq.runtime_identity(tmp_path / 'no_registered_runtime')


def test_record_integrity_state_and_operation(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    current = binding(session, cells['b'])
    path = tmp_path / 'scope.json'
    sha = psq.authorise_trial(path, current)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_RECORD_CHANGED'):
        psq.verify_trial(path, '0' * 64, current)
    record = json.loads(path.read_text())
    record['binding']['destination']['floor_z'] = 0.5                     # edited without the binding digest
    path.write_text(json.dumps(record))
    with pytest.raises(psq.PhysicsScopeRefused, match='binding digest mismatch'):
        psq.verify_trial(path, hashlib.sha256(path.read_bytes()).hexdigest(), current)
    for edit, match in ((lambda r: r.update(state='QUALIFIED'), "scope state 'QUALIFIED'"),
                        (lambda r: r['operation'].update(cycles=2), 'another operation'),
                        (lambda r: r['operation'].update(real_hardware=True), 'another operation'),
                        (lambda r: r.update(schema='workcell_physics_scope/v0'), 'unknown scope schema')):
        psq.authorise_trial(path, current)
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            psq.verify_trial(path, rewrite(path, edit), current)
    assert psq.verify_trial(path, psq.authorise_trial(path, current), current)


def test_an_occupied_destination_or_a_single_bin_world_is_unsupported(cells, tmp_path):
    receipt = launch(tmp_path / 'clear', cells['b']) / 'runtime/receipt.json'
    floor = db.select_destination_binding(json.loads(receipt.read_text())['destination_geometry_bindings'],
                                          'target_bin_b')['floor_xyz']
    occupied = launch(tmp_path / 'occupied', cells['b'], occupant=[floor[0], floor[1], 0.02])
    with pytest.raises(psq.PhysicsScopeRefused, match="PHYSICS_SCOPE_DESTINATION_OCCUPIED: \\['occupant'\\]"):
        binding(occupied, cells['b'])
    single = launch(tmp_path / 'single', cells['b'])
    data = json.loads((single / 'runtime/receipt.json').read_text())
    data.update(destination_geometry_binding=data['destination_geometry_bindings'][0], destination_geometry_bindings=None)
    (single / 'runtime/receipt.json').write_text(json.dumps(data))
    with pytest.raises(psq.PhysicsScopeRefused, match='does not bind a multi-destination world'):
        binding(single, cells['b'])
