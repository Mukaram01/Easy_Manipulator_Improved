#!/usr/bin/env bash
# Build only the matching Humble MoveIt planning package outside /opt, then
# prove its real TEM's owned shutdown before publishing a runtime receipt.
set -eo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
overlay=${1:-$(cd "$repo/../.." && pwd)/moveit_teardown_overlay}
release_tag=release/humble/moveit_ros_planning/2.5.10-1
release_commit=c62753946ae3629a8cb745767844f7e69ca51489
debian_tag=debian/ros-humble-moveit-ros-planning_2.5.10-1_jammy
debian_commit=4e3d4445099fccf3d3732e893bf461e71e86522d
source_dir="$overlay/src/moveit_ros_planning"
patch="$repo/patches/moveit_humble_tem_teardown.patch"
mkdir -p "$overlay"
overlay=$(cd "$overlay" && pwd)
evidence="$overlay/evidence/run-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$overlay/evidence"
mkdir "$evidence"
# The ABI-preserving package override is built against only the installed Humble.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
baseline_version=$(dpkg-query -W -f='${Version}' ros-humble-moveit-ros-planning)
case "$baseline_version" in 2.5.10-1jammy.*) ;; *) echo "Unreviewed MoveIt baseline: $baseline_version" >&2; exit 2;; esac
if [ ! -d "$source_dir/.git" ]; then
  git clone --depth 1 --branch "$release_tag" https://github.com/ros2-gbp/moveit2-release.git "$source_dir" > "$evidence/source-fetch.log" 2>&1
fi
test "$(git -C "$source_dir" rev-parse HEAD)" = "$release_commit"
git -C "$source_dir" fetch --depth 1 origin "refs/tags/$debian_tag" > "$evidence/debian-fetch.log" 2>&1
test "$(git -C "$source_dir" rev-parse FETCH_HEAD)" = "$debian_commit"
git -C "$source_dir" diff --name-only HEAD FETCH_HEAD > "$evidence/debian-diff-files.txt"
if rg -v '^debian/' "$evidence/debian-diff-files.txt"; then echo 'Unexpected Debian source patch' >&2; exit 3; fi
cmp "$source_dir/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h" /opt/ros/humble/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h
if [ -f "$overlay/provenance.json" ]; then
  cp "$overlay/provenance.json" "$evidence/previous-provenance.json"
  rm "$overlay/provenance.json"
fi
cmake -S "$repo/scripts/moveit_teardown_repro" -B "$overlay/repro_build" > "$evidence/repro-configure.log" 2>&1
cmake --build "$overlay/repro_build" -j2 > "$evidence/repro-build.log" 2>&1
binary="$overlay/repro_build/workcell_tem_teardown_repro"
baseline=/opt/ros/humble/lib/libmoveit_trajectory_execution_manager.so.2.5.10
python3 "$repo/scripts/moveit_teardown_repro/verify.py" --binary "$binary" --library "$baseline" --output "$evidence/baseline" --cycles 1 --expect-crash
ROS_DOMAIN_ID=193 gdb -batch -ex run -ex 'thread apply all bt' -ex 'info sharedlibrary' -ex 'x/8i $pc-12' --args "$binary" --self-shutdown > "$evidence/baseline-gdb.log" 2>&1
if ! git -C "$source_dir" diff --quiet; then
  git -C "$source_dir" diff --binary > "$evidence/current-source.patch"
  cmp "$evidence/current-source.patch" "$patch"
else
  git -C "$source_dir" apply --check "$patch"
  git -C "$source_dir" apply "$patch"
fi
test -z "$(git -C "$source_dir" ls-files --others --exclude-standard)"
(cd "$overlay" && MAKEFLAGS='-j2 -l2' colcon build --packages-select moveit_ros_planning --allow-overriding moveit_ros_planning --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF) > "$evidence/build.log" 2>&1
source "$overlay/install/local_setup.bash"
library="$overlay/install/moveit_ros_planning/lib/libmoveit_trajectory_execution_manager.so.2.5.10"
python3 "$repo/scripts/moveit_teardown_repro/verify.py" --binary "$binary" --library "$library" --output "$evidence/patched" --cycles 20
ROS_DOMAIN_ID=193 gdb -batch -ex 'set breakpoint pending on' -ex 'break trajectory_execution_manager::TrajectoryExecutionManager::~TrajectoryExecutionManager()' -ex run -ex 'info sharedlibrary' -ex continue --args "$binary" --self-shutdown > "$evidence/patched-gdb.log" 2>&1
ldd /opt/ros/humble/lib/moveit_ros_move_group/move_group > "$evidence/move-group-ldd.log"
python3 - "$repo" "$overlay" "$evidence" "$baseline_version" <<'PY'
from pathlib import Path
import hashlib,json,subprocess,sys,urllib.request
repo,overlay,evidence=map(Path,sys.argv[1:4]);baseline_version=sys.argv[4]
source=overlay/'src/moveit_ros_planning'
commit='c62753946ae3629a8cb745767844f7e69ca51489'
upstream='c283a36186a6f7a5985360e6674bf8fd0790e485'
# Compare the unpatched released TEM implementation against the pinned upstream tag.
relative='trajectory_execution_manager/src/trajectory_execution_manager.cpp'
released=subprocess.check_output(['git','show',f'HEAD:{relative}'],cwd=source)
url=f'https://raw.githubusercontent.com/moveit/moveit2/{upstream}/moveit_ros/planning/{relative}'
upstream_source=urllib.request.urlopen(url,timeout=30).read()
assert released==upstream_source,'release TEM differs from pinned upstream source'
(evidence/'upstream-tem.cpp').write_bytes(upstream_source)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
library=(overlay/'install/moveit_ros_planning/lib/libmoveit_trajectory_execution_manager.so.2.5.10').resolve()
baseline=Path('/opt/ros/humble/lib/libmoveit_trajectory_execution_manager.so.2.5.10')
results=json.loads((evidence/'patched/results.json').read_text())
before=json.loads((evidence/'baseline/results.json').read_text())
assert results['passed'] and results['completed']>=20 and not results['expected_crash']
assert before['passed'] and before['results'][0]['returncode']==-11
assert str(library) in (evidence/'move-group-ldd.log').read_text()
manifest=dict(schema='workcell_moveit_teardown_overlay/v1',
 source=dict(version='2.5.10',commit=commit,release_commit=commit,upstream_commit=upstream,
  upstream_url='https://github.com/ros2-gbp/moveit2-release.git',release_tag='release/humble/moveit_ros_planning/2.5.10-1',
  debian_commit='4e3d4445099fccf3d3732e893bf461e71e86522d',
  package_source_sha256=hashlib.sha256(subprocess.check_output(['git','archive','HEAD'],cwd=source)).hexdigest()),
 patch=dict(path=str(repo/'patches/moveit_humble_tem_teardown.patch'),sha256=sha(repo/'patches/moveit_humble_tem_teardown.patch')),
 library=dict(path=str(library),sha256=sha(library),soname=library.name),
 baseline=dict(package='ros-humble-moveit-ros-planning',version=baseline_version,library_path=str(baseline),library_sha256=sha(baseline)),
 reproduction=dict(baseline_returncode=-11,cycles=results['completed'],passed=sum(row['passed'] for row in results['results']),all_success=True,results_path=str(evidence/'patched/results.json')),
 build=dict(command='MAKEFLAGS="-j2 -l2" colcon build --packages-select moveit_ros_planning --allow-overriding moveit_ros_planning --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF',log=str(evidence/'build.log')))
(overlay/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
link=overlay/'install/moveit_ros_planning/share/moveit_ros_planning/workcell_teardown_overlay.json'
if link.is_symlink():link.unlink()
assert not link.exists(),'refusing to replace unrelated provenance file'
link.symlink_to(overlay/'provenance.json')
print(json.dumps(manifest,indent=2))
PY
