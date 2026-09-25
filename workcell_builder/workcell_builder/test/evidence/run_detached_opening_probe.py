#!/usr/bin/env python3
"""Offline only: compile against the existing test target and replay saved geometry.

Requires the installed ROS/MoveIt overlays sourced, a completed passing test
qualification gate, and a fixture containing scene.cdr, robot.urdf, robot.srdf,
leader.txt. No ROS node, transport, recovery authority, or execution is created.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('fixture', type=Path)
parser.add_argument('build', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
output = args.output.resolve()
output.mkdir(parents=True, exist_ok=True)
build = args.build.resolve()
source = Path(__file__).with_name('detached_opening_probe.cpp').resolve()
flags = dict(line.split(' = ', 1) for line in
             (build / 'CMakeFiles/workcell_support_contact_test.dir/flags.make').read_text().splitlines()
             if ' = ' in line)
obj = output / 'detached_opening_probe.o'
compile_command = ['/usr/bin/c++']
for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
    compile_command.extend(shlex.split(flags[key]))
compile_command.extend(['-c', str(source), '-o', str(obj)])
link = shlex.split((build / 'CMakeFiles/workcell_support_contact_test.dir/link.txt').read_text())
executable = output / 'detached_opening_probe'
link[link.index('-o') + 1] = str(executable)
for i, part in enumerate(link):
    if part.endswith('/test/support_contact_policy_test.cpp.o'):
        link[i] = str(obj)
(output / 'build-command.json').write_text(json.dumps({'compile': compile_command, 'link': link}, indent=2) + '\n')
subprocess.run(compile_command, cwd=build, check=True)
subprocess.run(link, cwd=build, check=True)
env = dict(os.environ, WORKCELL_DETACHED_FIXTURE=str(args.fixture.resolve()), WORKCELL_DETACHED_RECORDS=str(output))
with (output / 'historical.txt').open('w') as log:
    subprocess.run([str(executable), '--gtest_filter=WithdrawalContinuousGate.*',
                    '--gtest_output=xml:' + str(output / 'historical.xml')],
                   env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
print((output / 'historical.txt').read_text())
