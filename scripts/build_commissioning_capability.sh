#!/usr/bin/env bash
# Scoped existing-workspace overlay; never writes /opt or builds UI/dependencies.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
workspace=$(cd "$repo/../.." && pwd)
build="$workspace/build/workcell_builder"
prefix="$workspace/install/workcell_builder"
test -d "$build" && test -d "$prefix"
cmake -S "$repo/workcell_builder/workcell_builder" -B "$build" \
  -DWORKCELL_BUILD_COMMISSIONING_CAPABILITY=ON \
  -DWORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK=ON

if ! cmake --build "$build" --target help | grep -q 'workcell_simulator_measurements'; then
  echo "ERROR: workcell_simulator_measurements target is unavailable." >&2
  echo "Fortress/ignition-gazebo6 development files must be discoverable by CMake." >&2
  exit 41
fi

cmake --build "$build" --target \
  workcell_commission_execute \
  workcell_execute_action_test \
  workcell_simulator_measurements \
  -j2
python3 - "$repo" "$build" "$prefix" <<'PY'
from pathlib import Path
import sys,json,hashlib
repo,build,prefix=map(Path,sys.argv[1:])
links={prefix/'lib/libworkcell_commission_execute.so':build/'libworkcell_commission_execute.so',
 prefix/'share/workcell_builder/execute_trajectory_plugins.xml':repo/'workcell_builder/workcell_builder/execute_trajectory_plugins.xml',
 prefix/'share/ament_index/resource_index/moveit_ros_move_group__pluginlib__plugin/workcell_builder':build/'ament_cmake_index/share/ament_index/resource_index/moveit_ros_move_group__pluginlib__plugin/workcell_builder'}
for target,source in links.items():
 assert source.is_file(),source
 target.parent.mkdir(parents=True,exist_ok=True)
 if target.is_symlink() and target.resolve()==source.resolve():continue
 if target.exists() or target.is_symlink():raise RuntimeError(f'refusing to replace unrelated {target}')
 target.symlink_to(source)
sources=[repo/'workcell_builder/workcell_builder'/name for name in (
 'commission_execute_server.hpp','controller_terminal_audit.hpp','src_commission_execute_capability.cpp','execute_trajectory_plugins.xml','CMakeLists.txt')]
library=build/'libworkcell_commission_execute.so'
manifest=dict(moveit_version='2.5.9',library=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
 sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(prefix/'share/workcell_builder/commission_execute_build.json').write_text(json.dumps(manifest,indent=2))
PY
