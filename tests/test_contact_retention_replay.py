"""Faithful replays of recorded Stage-A transfers that lost pad contact.

The replay (scripts/simulator_physics_repro/mechanism_replay.cpp) runs the session's own gripper, the
overlay's contact detector and coupling rows (taken from the repo's physics patch), the gz position hold
and the UR5 tracking the recorded joint states. With DART's default contact compliance it reproduces the
recorded pad dropout exactly; with the overlay's workcell contact compliance (CFM 1e-4) both pads stay
loaded through the same motions, including a 3.1 s transfer.
"""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PROOF = REPO/'scripts/simulator_physics_repro'
PATCH = PROOF/'passive-joint-coupling.patch'
WS = Path('/home/ros2/workcell_ws')
SHORT = WS/'stage_a1_targeted_native_pads_fc1_20261025_000126/06-full-cycle'       # dropout 0.4 s into transfer
LONG = WS/'stage_a1_targeted_contact_layer_seat0_fc1_20261025_010938/06-full-cycle'  # 3.1 s transfer recorded

pytestmark = pytest.mark.skipif(shutil.which('pkg-config') is None or
                                subprocess.run(['pkg-config', '--exists', 'dart']).returncode != 0,
                                reason='DART development files absent')


def overlay_contact_cfm():
    match = re.search(r'^\+\s*dart::constraint::ContactConstraint::setConstraintForceMixing\(([0-9.e-]+)\);', PATCH.read_text(), re.M)
    assert match, 'workcell contact compliance missing from the physics patch'
    return match.group(1)


def patch_new_files(names):
    """Contents of files the patch creates (--- /dev/null sections)."""
    files, current = {}, None
    for line in PATCH.read_text().splitlines():
        match = re.match(r'\+\+\+ b/dartsim/src/(\S+)', line)
        if match:
            current = match.group(1) if match.group(1) in names else None
            if current:
                files[current] = []
        elif current and line.startswith('+'):
            files[current].append(line[1:])
        elif current and not line.startswith('@@'):
            current = None
    return {name: '\n'.join(lines) + '\n' for name, lines in files.items()}


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    work = tmp_path_factory.mktemp('replay')
    for name, text in patch_new_files({'JointCouplingConstraint.hpp', 'TouchingBulletContacts.hpp'}).items():
        (work/name).write_text(text)
    flags = subprocess.run(['pkg-config', '--cflags', '--libs', 'dart'], capture_output=True, text=True, check=True).stdout.split()
    subprocess.run(['g++', '-O2', f'-I{work}', '-I/usr/include/bullet', str(PROOF/'mechanism_replay.cpp'),
                    '-o', str(work/'replay'), *flags, '-std=c++17', '-ldart-collision-bullet', '-ldart-collision-ode', '-lode',
                    '-lassimp', '-lBulletCollision', '-lLinearMath'], check=True, capture_output=True, timeout=600)
    return work/'replay'


def replay(binary, session, first, last, tmp_path, *options):
    if not (session/'full-cycle/summary.measurements.jsonl').is_file():
        pytest.skip('recorded session absent')
    subprocess.run([sys.executable, str(PROOF/'mechanism_replay_inputs.py'), str(session), str(first), str(last), str(tmp_path)],
                   check=True, capture_output=True, timeout=300)
    out = subprocess.run([str(binary), str(tmp_path/'mechanism.txt'), str(tmp_path/'replay.txt'), f"arm={tmp_path/'arm.txt'}", *options],
                         check=True, capture_output=True, text=True, timeout=300).stdout
    line = [l for l in out.splitlines() if l.startswith('RESULT ')][-1]
    return {k: v for k, v in (field.split('=', 1) for field in line.split()[1:])}


def test_default_contact_compliance_reproduces_the_recorded_pad_dropout(binary, tmp_path):
    result = replay(binary, SHORT, 176000, 183400, tmp_path)
    recorded = json.loads((SHORT/'full-cycle/summary.json').read_text())['grasp_recovery']['rejected_sample']
    assert int(result['first_dropout_it']) == recorded['iteration'] == 183296
    assert float(result['peak_rotation_rad']) == pytest.approx(0.000597, abs=2e-6)          # recorded 0.000597
    assert float(result['min_tip_force_n']) < 0.5


@pytest.mark.parametrize('session,first,last', [(SHORT, 176000, 183400), (LONG, 123950, 135100)])
def test_workcell_contact_compliance_keeps_both_pads_loaded_through_the_recorded_transfers(binary, tmp_path, session, first, last):
    rigid = replay(binary, session, first, last, tmp_path)
    compliant = replay(binary, session, first, last, tmp_path, f'cfm={overlay_contact_cfm()}')
    assert int(rigid['dropout_samples']) > 0
    assert int(compliant['dropout_samples']) == 0 and int(compliant['partial_manifold_samples']) == 0
    assert float(compliant['peak_rotation_rad']) < 0.0002                                   # limit 0.0100
    assert float(compliant['min_tip_force_n']) > 18.0                                       # 20 N hold retained


def test_a_stationary_hold_command_is_not_a_correction(binary, tmp_path):
    # Holding the closure position without the closing reserve loses the pads during the long transfer.
    result = replay(binary, LONG, 123950, 135100, tmp_path, f'cfm={overlay_contact_cfm()}', 'hold=stationary')
    assert int(result['dropout_samples']) > 0
