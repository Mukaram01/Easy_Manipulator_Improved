#!/usr/bin/env python3
"""Build pinned Fortress physics repairs. A build receipt grants no qualification.

No qualified provenance.json or setup.bash is published by this builder.
Stage-A must prove these exact libraries before qualification is published.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request

SOURCES = {
    'physics': ('https://codeload.github.com/gazebosim/gz-physics/tar.gz/refs/tags/ignition-physics5_5.4.0',
                '8a05acbc6855472ae34fdaaaa96d5c3d9e8e5a8518036d044d6adfe29e715a73'),
    'bullet': ('https://archive.ubuntu.com/ubuntu/pool/universe/b/bullet/bullet_3.06+dfsg.orig.tar.xz',
               '1178bd4cb5c8f25478d3745e56528725bad5478719441de5a12d77f8fe1e5648'),
    'control': ('https://raw.githubusercontent.com/ros-controls/gz_ros2_control/0.7.21/gz_ros2_control/src/gz_ros2_control_plugin.cpp',
                'c537fdfeffe5e0ad6dadf14547de892a4b50c78706b4971724985ef9b9c76a18'),
}
PATCHES = {'physics': 'passive-joint-coupling.patch', 'bullet': 'bullet-finite-edge.patch',
           'control': 'control-teardown.patch'}
LIBRARIES = ('libignition-physics5-dartsim-plugin.so', 'libign_ros2_control-system.so',
             'libBulletCollision.so.3.05', 'libLinearMath.so.3.05')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(command, log):
    subprocess.run([str(x) for x in command], check=True, stdout=log,
                   stderr=subprocess.STDOUT, timeout=3600)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    for name in SOURCES:
        parser.add_argument('--' + name + '-source', type=Path,
                            help='cached pinned archive, or original control cpp')
    args = parser.parse_args()
    root = args.output.resolve()
    if root.exists() and any(root.iterdir()):
        raise RuntimeError('use a new or empty output directory; preserve existing build evidence')
    build_script_sha256 = sha(__file__)
    root.mkdir(parents=True, exist_ok=True)
    if any((root / name).exists() for name in ('provenance.json', 'setup.bash')):
        raise RuntimeError('use a new output directory; preserve existing qualified publication')
    proof = Path(__file__).resolve().parent / 'simulator_physics_repro'
    (root / 'lib').mkdir(exist_ok=True)
    inputs = {}
    with (root / 'build.log').open('w') as log:
        for name, (url, expected) in SOURCES.items():
            cached = getattr(args, name + '_source')
            if cached is None:
                cached = root / (name + ('.cpp' if name == 'control' else '.tar'))
                if not cached.exists():
                    with urllib.request.urlopen(url, timeout=60) as response, cached.open('wb') as output:
                        shutil.copyfileobj(response, output)
            if sha(cached) != expected:
                raise RuntimeError('pinned upstream source identity differs: ' + name)
            source = root / 'sources' / name
            source.mkdir(parents=True, exist_ok=False)
            if name == 'control':
                (source / 'gz_ros2_control/src').mkdir(parents=True)
                shutil.copy2(cached, source / 'gz_ros2_control/src/gz_ros2_control_plugin.cpp')
            else:
                run(['tar', '-xf', cached, '-C', source, '--strip-components=1'], log)
            patch = proof / PATCHES[name]
            subprocess.run(['patch', '--batch', '--forward', '-p1', '-i', str(patch)],
                           cwd=source, check=True, stdout=log, stderr=subprocess.STDOUT)
            if name == 'control':
                shutil.copy2(proof / 'control.CMakeLists.txt', source / 'CMakeLists.txt')
            inputs[name] = dict(url=url, archive_sha256=expected, patch_sha256=sha(patch),
                               files={str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()})
        builds = {}
        for name in SOURCES:
            source, build = root / 'sources' / name, root / 'build' / name
            options = ['-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DBUILD_TESTING=OFF',
                       '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g',
                       '-DCMAKE_C_FLAGS_RELWITHDEBINFO=-O2 -g']
            if name == 'bullet':
                options += ['-DBUILD_SHARED_LIBS=ON', '-DBUILD_BULLET2_DEMOS=OFF',
                            '-DBUILD_CPU_DEMOS=OFF', '-DBUILD_OPENGL3_DEMOS=OFF',
                            '-DBUILD_UNIT_TESTS=OFF', '-DBUILD_EXTRAS=OFF']
            configure = ['cmake', '-S', source, '-B', build, *options]
            targets = {'physics': ['ignition-physics5-dartsim-plugin'],
                       'control': ['ign_ros2_control-system'],
                       'bullet': ['BulletCollision', 'LinearMath']}[name]
            compile_command = ['cmake', '--build', build, '--target', *targets, '--parallel', '2']
            run(configure, log)
            run(compile_command, log)
            builds[name] = dict(configure=[str(x) for x in configure],
                                compile=[str(x) for x in compile_command],
                                cache_sha256=sha(build / 'CMakeCache.txt'))
        binaries = [root / 'build/physics/lib' / LIBRARIES[0], root / 'build/control' / LIBRARIES[1],
                    root / 'build/bullet/src/BulletCollision' / LIBRARIES[2],
                    root / 'build/bullet/src/LinearMath' / LIBRARIES[3]]
        artifacts = {}
        for name, binary in zip(LIBRARIES, binaries):
            destination = root / 'lib' / name
            shutil.copy2(binary.resolve(), destination)
            if sha(destination) != sha(binary):
                raise RuntimeError('copied artifact identity differs')
            artifacts[name] = dict(path=str(destination), sha256=sha(destination))
    receipt = dict(schema='workcell_simulator_physics_build/v1', qualified=False,
                   build_script_sha256=build_script_sha256, sources=inputs, artifacts=artifacts,
                   control_build_sha256=sha(proof / 'control.CMakeLists.txt'), builds=builds,
                   build_log_sha256=sha(root / 'build.log'))
    (root / 'artifacts.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Pinned physics build receipt: ' + str(root / 'artifacts.json'))


if __name__ == '__main__':
    main()
