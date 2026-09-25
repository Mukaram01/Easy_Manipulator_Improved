#!/usr/bin/env bash
# Scoped existing-workspace overlay; never writes /opt or builds UI/dependencies.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
workspace=$(cd "$repo/../.." && pwd)
build="$workspace/build/workcell_builder"
prefix="$workspace/install/workcell_builder"
test -d "$build" && test -d "$prefix"
# Resolve the qualified package through the caller's overlay, before CMake can
# reuse an old /opt package directory from its cache.
moveit_prefix=$(python3 - "$repo" <<'PYPREFIX'
from pathlib import Path
import sys
sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from simulator_backend import active_moveit_overlay
print(Path(active_moveit_overlay()['library']['path']).parents[1])
PYPREFIX
)
moveit_group_prefix=$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("moveit_ros_move_group"))')
cmake -S "$repo/workcell_builder/workcell_builder" -B "$build" \
  -Dmoveit_ros_move_group_DIR="$moveit_group_prefix/share/moveit_ros_move_group/cmake" \
  -Dmoveit_ros_planning_DIR="$moveit_prefix/share/moveit_ros_planning/cmake" \
  -DWORKCELL_BUILD_COMMISSIONING_CAPABILITY=ON \
  -DWORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK=ON

targets=$(cmake --build "$build" --target help)
if ! grep -q 'workcell_simulator_measurements' <<<"$targets"; then
  echo "ERROR: workcell_simulator_measurements target is unavailable." >&2
  echo "Fortress/ignition-gazebo6 development files must be discoverable by CMake." >&2
  exit 41
fi

cmake --build "$build" --target \
  workcell_commission_execute \
  workcell_execute_action_test \
  workcell_support_contact \
  workcell_support_contact_test \
  workcell_simulator_measurements \
  -j2
python3 - "$repo" "$build" "$prefix" <<'PY'
from pathlib import Path
import sys,json,hashlib
repo,build,prefix=map(Path,sys.argv[1:])
sys.path.insert(0,str(repo/"scripts"))
from simulator_backend import active_moveit_overlay
links={prefix/'lib/libworkcell_commission_execute.so':build/'libworkcell_commission_execute.so',
 prefix/'lib/libworkcell_simulator_measurements.so':build/'libworkcell_simulator_measurements.so',
 prefix/'share/workcell_builder/execute_trajectory_plugins.xml':repo/'workcell_builder/workcell_builder/execute_trajectory_plugins.xml',
 prefix/'share/ament_index/resource_index/moveit_ros_move_group__pluginlib__plugin/workcell_builder':build/'ament_cmake_index/share/ament_index/resource_index/moveit_ros_move_group__pluginlib__plugin/workcell_builder'}
support_links={
 prefix/'lib/libworkcell_support_contact.so':build/'libworkcell_support_contact.so',
 prefix/'share/workcell_builder/support_contact_plugins.xml':repo/'workcell_builder/workcell_builder/support_contact_plugins.xml',
 prefix/'share/ament_index/resource_index/moveit_core__pluginlib__plugin/workcell_builder':
   build/'ament_cmake_index/share/ament_index/resource_index/moveit_core__pluginlib__plugin/workcell_builder',
}
for target,source in links.items():
 assert source.is_file(),source
 target.parent.mkdir(parents=True,exist_ok=True)
 if target.is_symlink() and target.resolve()==source.resolve():continue
 if target.exists() or target.is_symlink():raise RuntimeError(f'refusing to replace unrelated {target}')
 target.symlink_to(source)
# These three paths are owned generated install artifacts for the planning
# adapter. Refresh them explicitly because this scoped build intentionally does
# not run a full workspace install.
for target,source in support_links.items():
 assert source.is_file(),source
 target.parent.mkdir(parents=True,exist_ok=True)
 if target.is_symlink() and target.resolve()==source.resolve():continue
 if target.exists() or target.is_symlink():target.unlink()
 target.symlink_to(source)
sources=[repo/'workcell_builder/workcell_builder'/name for name in (
 'commission_execute_server.hpp','controller_terminal_audit.hpp','src_commission_execute_capability.cpp',
 'execute_trajectory_plugins.xml','src_support_contact_adapter.cpp','support_contact_policy.hpp',
 'support_contact_plugins.xml','CMakeLists.txt')]
library=build/'libworkcell_commission_execute.so'
version_file=build/'workcell_commission_moveit_version.txt'
if not version_file.is_file():raise RuntimeError('commissioning MoveIt version receipt missing')
moveit_version=version_file.read_text().strip()
if moveit_version not in {'2.5.9','2.5.10'}:raise RuntimeError('unreviewed MoveIt version: '+moveit_version)
telemetry=build/'libworkcell_simulator_measurements.so'
support=build/'libworkcell_support_contact.so'
manifest=dict(moveit_overlay=active_moveit_overlay(),moveit_version=moveit_version,library=str(library),sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
 telemetry_library=str(telemetry),telemetry_sha256=hashlib.sha256(telemetry.read_bytes()).hexdigest(),
 support_library=str(support),support_sha256=hashlib.sha256(support.read_bytes()).hexdigest(),
 sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
(prefix/'share/workcell_builder/commission_execute_build.json').write_text(json.dumps(manifest,indent=2))
PY
