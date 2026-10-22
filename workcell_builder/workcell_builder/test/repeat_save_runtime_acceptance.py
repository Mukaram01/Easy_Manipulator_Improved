"""Exercise real Inspector/Apply/Save and process reopen on disposable scene copies.

Source ROS Humble, then run from the repository root with --binary and --output.
No generation, planning, execution, or canonical-scene writes are requested.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(binary, output, backend, axis):
    repo = Path(__file__).resolve().parents[3]
    source = repo / 'scenes/ur5_2f_test'
    original_hashes = {p.relative_to(source): digest(p) for p in source.rglob('*') if p.is_file()}
    output.mkdir(parents=True, exist_ok=False)
    scene = output / 'scenes/ur5_2f_test'
    shutil.copytree(source, scene, ignore=shutil.ignore_patterns(
        'generated', 'preview_launch', 'smoke', 'validation', '*.bak.yaml', '__pycache__'))
    # Give the fixture its own repository/server identity and generated caches.
    shutil.copytree(repo / 'scripts', output / 'scripts', ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(repo / 'catalog', output / 'catalog')
    (output / 'assets').symlink_to(repo / 'assets', target_is_directory=True)
    (output / 'workcell_builder').symlink_to(repo / 'workcell_builder', target_is_directory=True)
    shutil.copytree(repo / 'workcell_studio_web', output / 'workcell_studio_web',
                    ignore=shutil.ignore_patterns('node_modules', '__pycache__'))
    workspace = output / 'workspace'
    (workspace / 'src').mkdir(parents=True)
    (workspace / 'src/easy_manipulation_deployment').symlink_to(output, target_is_directory=True)
    (workspace / 'src/assets').symlink_to(repo / 'assets', target_is_directory=True)
    (workspace / 'src/scenes').symlink_to(output / 'scenes', target_is_directory=True)
    authored = {p.relative_to(scene): p.read_bytes() for p in scene.rglob('*.yaml')}
    env = os.environ.copy()
    env.update(XDG_CONFIG_HOME=str(output / 'settings'),
               WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND=backend,
               QTWEBENGINE_DISABLE_SANDBOX='1')
    command = [str(binary), '--repeat-save-acceptance', '--scene-path', str(scene),
               '--smoke-output', str(output), '--workspace', str(workspace), '--axis', axis]
    process_failures = []
    try:
        for reopen in (False, True):
            with (output / ('reopen.log' if reopen else 'gui.log')).open('w') as log:
                result = subprocess.run(command + (['--reopen'] if reopen else []),
                                        cwd=output, env=env, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=120)
            report = json.loads((output / ('reopen.json' if reopen else 'repeat-save.json')).read_text())
            assert report['result'] == 'PASS', (
                f"{'reopen' if reopen else 'authoring'} exit={result.returncode}, "
                f"assertions={report['result']}, failure={report.get('failure', 'none')}")
            if result.returncode:
                # Still inspect disk and reopen a proven save, but never hide a shutdown failure.
                process_failures.append(f"{'reopen' if reopen else 'authoring'} exit={result.returncode}")
        coordinate = 1 if axis == 'y' else 0
        expected = yaml.safe_load(authored[Path('environment.yaml')])
        for container in (expected, expected['environment']):
            for collection in ('assets', 'task_zones'):
                for item in container.get(collection, []):
                    if item['id'] in ('target_bin_default', 'default_drop_zone', 'place_zone_default'):
                        item['pose_xyz'][coordinate] = .24 if axis == 'y' else .47
        # Save already reconciles this compatibility camera mirror with physical truth.
        camera = next(a for a in expected['environment']['assets'] if a['id'] == 'realsense_overhead')
        expected['camera']['pose'] = camera['pose_xyz'] + camera['pose_rpy']
        assert yaml.safe_load((scene / 'environment.yaml').read_text()) == expected
        expected_layout = yaml.safe_load(authored[Path('layout/workcell_studio_layout.yaml')])
        for item in expected_layout['items']:
            if item['id'] in ('target_bin_default', 'place_zone_default'):
                item['pose']['xyz'][coordinate] = .24 if axis == 'y' else .47
        assert yaml.safe_load((scene / 'layout/workcell_studio_layout.yaml').read_text()) == expected_layout
        for relative, before in authored.items():
            if relative not in (Path('environment.yaml'), Path('layout/workcell_studio_layout.yaml')):
                assert (scene / relative).read_bytes() == before, f'unexpected authored change: {relative}'
        assert not process_failures, '; '.join(process_failures)
        print(f'PASS {backend} {axis}: two saves, exact disk state, rejected invalid edit, process reopen')
    finally:
        assert original_hashes == {p.relative_to(source): digest(p) for p in source.rglob('*') if p.is_file()}, 'canonical fixture changed'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backend', choices=('native_scene3d', 'embedded_web3d'), default='embedded_web3d')
    parser.add_argument('--axis', choices=('x', 'y'), default='y')
    args = parser.parse_args()
    run(args.binary.resolve(), args.output.resolve(), args.backend, args.axis)
