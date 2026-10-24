"""Faithful replay of the recorded Stage-A transfer that lost pad contact (iteration 183296).

The replay (scripts/simulator_physics_repro/mechanism_replay.cpp) runs the session's own gripper, the
overlay's contact detector and coupling rows (taken from the repo's physics patch), the gz position hold
and the UR5 tracking the recorded joint states. Rigid pads must reproduce the recorded dropout exactly;
the capability's compliant pad layer must hold both pads for the same motion.
"""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
PROOF = REPO/'scripts/simulator_physics_repro'
SESSION = Path('/home/ros2/workcell_ws/stage_a1_targeted_native_pads_fc1_20261025_000126/06-full-cycle')
LAYER = yaml.safe_load((REPO/'catalog/capabilities/end_effectors/ee_robotiq_2f.yaml').read_text())[
    'end_effector']['simulator_collision']['contact_surface_layer_m']

pytestmark = pytest.mark.skipif(not (SESSION/'full-cycle/summary.measurements.jsonl').is_file() or
                                shutil.which('pkg-config') is None or
                                subprocess.run(['pkg-config', '--exists', 'dart']).returncode != 0,
                                reason='recorded session or DART development files absent')


def patch_new_files(patch, names):
    """Contents of files the patch creates (--- /dev/null sections)."""
    files, current = {}, None
    for line in patch.read_text().splitlines():
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
def replay(tmp_path_factory):
    work = tmp_path_factory.mktemp('replay')
    for name, text in patch_new_files(PROOF/'passive-joint-coupling.patch',
                                      {'JointCouplingConstraint.hpp', 'TouchingBulletContacts.hpp'}).items():
        (work/name).write_text(text)
    flags = subprocess.run(['pkg-config', '--cflags', '--libs', 'dart'], capture_output=True, text=True, check=True).stdout.split()
    subprocess.run(['g++', '-O2', f'-I{work}', '-I/usr/include/bullet', str(PROOF/'mechanism_replay.cpp'),
                    '-o', str(work/'replay'), *flags, '-std=c++17', '-ldart-collision-bullet', '-ldart-collision-ode', '-lode', '-lassimp',
                    '-lBulletCollision', '-lLinearMath'], check=True, capture_output=True, timeout=600)
    subprocess.run([sys.executable, str(PROOF/'mechanism_replay_inputs.py'), str(SESSION), '176000', '183400', str(work)],
                   check=True, capture_output=True, timeout=300)

    def run(*options):
        out = subprocess.run([str(work/'replay'), str(work/'mechanism.txt'), str(work/'replay.txt'), f"arm={work/'arm.txt'}", *options],
                             check=True, capture_output=True, text=True, timeout=300).stdout
        line = [l for l in out.splitlines() if l.startswith('RESULT ')][-1]
        return {k: v for k, v in (field.split('=', 1) for field in line.split()[1:])}
    return run


def test_rigid_pads_reproduce_the_recorded_pad_dropout(replay):
    result = replay()
    recorded = json.loads((SESSION/'full-cycle/summary.json').read_text())['grasp_recovery']['rejected_sample']
    assert float(result['pad_layer']) == 0 and float(result['part_layer']) == 0.0001       # as the session authored
    assert int(result['first_dropout_it']) == recorded['iteration'] == 183296
    assert float(result['peak_rotation_rad']) == pytest.approx(0.000597, abs=2e-6)          # recorded 0.000597
    assert float(result['min_tip_force_n']) < 0.5


def test_compliant_pad_layer_keeps_both_pads_loaded_through_the_same_transfer(replay):
    result = replay(f'pad_layer={LAYER}')
    assert int(result['dropout_samples']) == 0 and int(result['partial_manifold_samples']) == 0
    assert float(result['peak_rotation_rad']) < 0.0002                                      # limit 0.0100
    assert float(result['min_tip_force_n']) > 18.0                                          # 20 N hold retained


def test_a_stationary_hold_command_is_not_a_correction(replay):
    # Holding the closure position without the closing reserve releases the grip once the pads settle.
    result = replay(f'pad_layer={LAYER}', 'hold=stationary')
    assert int(result['dropout_samples']) > 0
