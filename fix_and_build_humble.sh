#!/usr/bin/env bash
set -euo pipefail
# Bootstrap + build script for ROS 2 Humble/Jazzy workspaces.
# Usage: ./fix_and_build_humble.sh [--profile minimal|full] [--with-tesseract-qt] [--legacy-workarounds]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws

PROFILE="minimal"
ENABLE_LEGACY_WORKAROUNDS=0
WITH_TESSERACT_QT=0

usage() {
  cat <<'EOF'
Usage: ./fix_and_build_humble.sh [options]

Options:
  --profile minimal|full      Select bootstrap profile (default: minimal)
  --with-tesseract-qt         Opt in to building optional Studio/Qt widgets in full profile
  --legacy-workarounds        Enable legacy Humble patch/ignore behavior (opt-in)
  -h, --help                  Show this help message

Profiles:
  minimal   Headless-friendly path that uses released apt packages and skips
            importing the tesseract/trajopt source overlays.
  full      Imports tesseract.repos overlays for planning/dev workflows.
            By default this skips optional tesseract_qt; pass
            --with-tesseract-qt to build Studio/Qt widgets.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --with-tesseract-qt)
      WITH_TESSERACT_QT=1
      shift
      ;;
    --legacy-workarounds)
      ENABLE_LEGACY_WORKAROUNDS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "$PROFILE" != "minimal" && "$PROFILE" != "full" ]]; then
  echo "Invalid --profile value: '$PROFILE' (expected: minimal or full)" >&2
  exit 1
fi

APT_GET_CMD="apt-get"
if command -v sudo >/dev/null 2>&1; then
  APT_GET_CMD="sudo apt-get"
fi

# 0) Reset overlays and source only the intended base underlay (Humble)
reset_colcon_environment() {
  local ros_setup="/opt/ros/humble/setup.bash"
  local local_setup="$PWD/install/local_setup.bash"

  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

  if [[ ! -f "$ros_setup" ]]; then
    echo "Expected ROS setup is missing: $ros_setup" >&2
    exit 1
  fi

  set +u
  : "${AMENT_TRACE_SETUP_FILES:=}"
  # shellcheck source=/dev/null
  source "$ros_setup"
  if [[ -f "$local_setup" ]]; then
    # shellcheck source=/dev/null
    source "$local_setup"
  fi
  set -u
}

ROS_DISTRO=humble
reset_colcon_environment

# Ensure required tools are available (install common ones if missing)
MISSING_APT_PACKAGES=()
INSTALL_ROSDEP=0

for cmd in git colcon rosdep vcs; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    case "$cmd" in
      git)
        MISSING_APT_PACKAGES+=(git)
        ;;
      colcon)
        MISSING_COLCON=1
        ;;
      rosdep)
        MISSING_APT_PACKAGES+=(python3-rosdep)
        INSTALL_ROSDEP=1
        ;;
      vcs)
        MISSING_APT_PACKAGES+=(python3-vcstool)
        ;;
      *)
        echo "Required tool '$cmd' is not installed." >&2
        exit 1
        ;;
    esac
  fi
done

if [[ ${#MISSING_APT_PACKAGES[@]} -gt 0 ]]; then
  ${APT_GET_CMD} update -y
  ${APT_GET_CMD} install -y "${MISSING_APT_PACKAGES[@]}" >/dev/null 2>&1
fi

if [[ ${MISSING_COLCON:-0} -eq 1 ]]; then
  python3 -m pip install -U colcon-common-extensions >/dev/null 2>&1 || {
    echo "Failed to install colcon" >&2
    exit 1
  }
fi

if [[ $INSTALL_ROSDEP -eq 1 ]]; then
  rosdep init >/dev/null 2>&1 || true
fi

ensure_pyyaml() {
  if python3 - <<'PY'
import yaml  # noqa: F401
PY
  then
    return
  fi

  echo "Installing python3-yaml for repos processing"
  ${APT_GET_CMD} install -y python3-yaml >/dev/null 2>&1
}

load_ruckig_baseline() {
  local repos_file="$SCRIPT_DIR/dependencies/emd_epd_ws.repos"

  ensure_pyyaml

  mapfile -t _ruckig_meta < <(REPOS_FILE="$repos_file" python3 - <<'PY'
import os
import sys

import yaml

repos_file = os.environ["REPOS_FILE"]
with open(repos_file, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

ruckig = ((data.get("repositories") or {}).get("ruckig") or {})
version = (ruckig.get("version") or "").strip()
url = (ruckig.get("url") or "").strip()

if not version or not url:
    sys.exit("dependencies/emd_epd_ws.repos is missing the pinned ruckig source entry")

print(version)
print(url)
PY
  )

  if [[ ${#_ruckig_meta[@]} -lt 2 ]]; then
    echo "Failed to load the documented ruckig baseline from $repos_file" >&2
    exit 1
  fi

  RUCKIG_PINNED_REVISION="${_ruckig_meta[0]}"
  RUCKIG_REMOTE_URL="${_ruckig_meta[1]}"
}

check_ruckig_preflight() {
  local repo_dir="$PWD/src/ruckig"

  if [[ ! -d "$repo_dir/.git" ]]; then
    return
  fi

  local head_revision
  head_revision=$(git -C "$repo_dir" rev-parse HEAD)

  if [[ "$head_revision" == "$RUCKIG_PINNED_REVISION" ]]; then
    return
  fi

  if git -C "$repo_dir" merge-base --is-ancestor "$RUCKIG_PINNED_REVISION" "$head_revision"; then
    cat >&2 <<EOF
Unsupported ruckig checkout detected at src/ruckig.
  checked out: $head_revision
  supported:   $RUCKIG_PINNED_REVISION ($RUCKIG_REMOTE_URL)

This workspace is documented and CI-tested for Ubuntu 22.04 + ROS 2 Humble + GCC 11
with the pre-<format> ruckig line. A newer ruckig revision that includes <format>
(or otherwise requires std::format/C++20) is not supported on the default Jammy/Humble
libstdc++ baseline.

Fix it by re-importing the pinned manifest entry from dependencies/emd_epd_ws.repos /
tesseract.repos, or remove src/ruckig and rely on the distro libruckig-dev package.
EOF
    exit 1
  fi
}

load_ruckig_baseline

ensure_cereal_cmake_config() {
  local cereal_config="/usr/lib/x86_64-linux-gnu/cmake/cereal/cerealConfig.cmake"
  local cereal_source="/usr/share/cmake/cereal"

  if [[ -f "$cereal_config" || ! -d "$cereal_source" ]]; then
    return
  fi

  echo "Fixing cereal CMake package path"
  if command -v sudo >/dev/null 2>&1; then
    sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
    sudo ln -sf "$cereal_source" /usr/lib/x86_64-linux-gnu/cmake/cereal
  else
    mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
    ln -sf "$cereal_source" /usr/lib/x86_64-linux-gnu/cmake/cereal
  fi
}

apply_trajopt_ifopt_patch() {
  local planners_dir="$PWD/src/tesseract_planning/tesseract_motion_planners"
  local cmake_file="$planners_dir/CMakeLists.txt"

  if [[ ! -d "$planners_dir" ]]; then
    return
  fi

  if [[ -d "$planners_dir/trajopt_ifopt" ]]; then
    echo "Removing incompatible trajopt_ifopt planner from workspace"
    rm -rf "$planners_dir/trajopt_ifopt"
  fi

  if [[ -f "$cmake_file" ]]; then
    CMAKE_FILE="$cmake_file" python3 - <<'PY'
import os
from pathlib import Path

cmake_file = Path(os.environ["CMAKE_FILE"])
lines = cmake_file.read_text().splitlines()
new_lines = []
changed = False

for line in lines:
    stripped = line.lstrip()
    if "add_subdirectory(trajopt_ifopt)" in line and not stripped.startswith("#"):
        new_lines.append(f"# {line}")
        changed = True
        continue
    if (
        "list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)" in line
        and not stripped.startswith("#")
    ):
        new_lines.append(f"# {line}")
        changed = True
        continue
    new_lines.append(line)

if changed:
    cmake_file.write_text("\n".join(new_lines) + "\n")
PY
  fi
}

apply_legacy_ignore_workarounds() {
  local ignore_paths=(
    "$PWD/src/tesseract_qt/COLCON_IGNORE"
    "$PWD/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE"
    "$PWD/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE"
    "$PWD/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE"
    "$PWD/src/tesseract_planning/tesseract_examples/COLCON_IGNORE"
  )

  for marker in "${ignore_paths[@]}"; do
    if [[ -d "$(dirname "$marker")" ]]; then
      echo "Applying legacy ignore marker: $marker"
      touch "$marker"
    fi
  done
}

# Install core system dependencies (Boost + TinyXML2) using the shared helper so
# all setup paths stay in sync. The helper also reinstalls Boost headers when
# container images omit them even though dpkg claims the packages are present,
# preventing the "Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)"
# failure in trajopt_common.
"${SCRIPT_DIR}/scripts/ensure_rosdep_overrides.sh" taskflow
"${SCRIPT_DIR}/scripts/install_system_deps.sh"
ensure_cereal_cmake_config

# Point CMake at the OSQP package configuration so find_package(osqp) succeeds
# even on environments where the default prefix search path is trimmed down.
if [[ -z ${OSQP_DIR:-} ]]; then
  for candidate in /usr/lib/x86_64-linux-gnu/cmake/osqp /usr/lib/cmake/osqp; do
    if [[ -f "${candidate}/osqpConfig.cmake" ]]; then
      export OSQP_DIR="$candidate"
      export CMAKE_PREFIX_PATH="${candidate}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
      break
    fi
  done
fi

# 0.5) Install missing build tools and dependencies
if [ ! -f "/opt/ros/${ROS_DISTRO}/share/ament_cmake/package.xml" ]; then
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y "ros-${ROS_DISTRO}-ament-cmake"
  else
    apt-get update -y
    apt-get install -y "ros-${ROS_DISTRO}-ament-cmake"
  fi
fi

REPO_FILE="$SCRIPT_DIR/tesseract.repos"
if [[ "$PROFILE" == "full" && -f "$REPO_FILE" ]]; then
  echo "Profile 'full': importing source overlays from tesseract.repos"
  ensure_pyyaml

  mapfile -t MISSING_REPOS < <(REPOS_FILE="$REPO_FILE" SRC="$PWD/src" python3 - <<'PY'
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required to process tesseract.repos")

repos_file = os.environ["REPOS_FILE"]
src = os.environ["SRC"]

with open(repos_file, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

for name in (data.get("repositories") or {}):
    if not os.path.isdir(os.path.join(src, name)):
        print(name)
PY
  )

  if [[ ${#MISSING_REPOS[@]} -gt 0 ]]; then
    FILTERED_MISSING=()
    for repo in "${MISSING_REPOS[@]}"; do
      if [[ $repo == "trajopt_ifopt" ]]; then
        if apt-cache show "ros-$ROS_DISTRO-trajopt-ifopt" >/dev/null 2>&1; then
          echo "Installing ros-$ROS_DISTRO-trajopt-ifopt from apt instead of cloning"
          ${APT_GET_CMD} install -y "ros-$ROS_DISTRO-trajopt-ifopt" >/dev/null 2>&1
          continue
        else
          echo "ros-$ROS_DISTRO-trajopt-ifopt not available via apt; will attempt to clone" >&2
        fi
      fi
      FILTERED_MISSING+=("$repo")
    done

    if [[ ${#FILTERED_MISSING[@]} -gt 0 ]]; then
      FILTERED_REPOS=$(mktemp)
      REPOS_FILE="$REPO_FILE" MISSING="${FILTERED_MISSING[*]}" python3 - <<'PY' >"$FILTERED_REPOS"
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required to process the repos file")

repos_file = os.environ["REPOS_FILE"]
missing = {item for item in os.environ.get("MISSING", "").split() if item}

with open(repos_file, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

repos = data.get("repositories") or {}
selected = {name: repos[name] for name in repos if name in missing}

yaml.safe_dump({"repositories": selected}, sys.stdout)
PY
      vcs import --recursive src < "$FILTERED_REPOS"
      rm -f "$FILTERED_REPOS"
    fi
  fi

  for overlay in tesseract trajopt; do
    if [ -d "$SCRIPT_DIR/$overlay" ]; then
      mkdir -p "src/$overlay"
      cp -a "$SCRIPT_DIR/$overlay/." "src/$overlay/"
      # Immediately rename ignore markers shipped with overlays so colcon can discover the packages
      while IFS= read -r -d '' marker; do
        echo "Renaming ignore marker $marker"
        mv -f "$marker" "$marker.repo"
      done < <(find "src/$overlay" -maxdepth 2 \( -name 'COLCON_IGNORE' -o -name 'AMENT_IGNORE' \) -print0)
    fi
  done
fi

# Repair the workspace layout (symlinks, overlay locations, and missing upstream
# dependencies) so subsequent colcon builds can locate generated CMake package
# files such as tesseract_motion_planners_coreConfig.cmake. The helper now
# fails fast if workbench_description is still missing, which aborts this build
# before rosdep update/install can proceed with a broken workspace layout.
WS=~/workcell_ws "$SCRIPT_DIR/scripts/fix_workspace_layout.sh"
check_ruckig_preflight
if [[ "$PROFILE" == "full" && $ENABLE_LEGACY_WORKAROUNDS -eq 1 ]]; then
  echo "Applying legacy trajopt_ifopt and COLCON_IGNORE workarounds"
  apply_trajopt_ifopt_patch
  apply_legacy_ignore_workarounds
fi

ROSDEP_OVERRIDES="$SCRIPT_DIR/scripts/rosdep_overrides.yaml"
if [[ -f "$ROSDEP_OVERRIDES" ]]; then
  export ROSDEP_ADDITIONAL_SOURCES_PATHS="$ROSDEP_OVERRIDES${ROSDEP_ADDITIONAL_SOURCES_PATHS:+:$ROSDEP_ADDITIONAL_SOURCES_PATHS}"
  echo "Using rosdep overrides from $ROSDEP_OVERRIDES" >&2
fi

rosdep update

# Ensure trajopt_common declares all dependencies it links against. Some upstream
# snapshots omit find_package() calls for tinyxml2, Boost graph, and the KDL
# state solver, which results in CMake configure errors such as missing
# tinyxml2::tinyxml2 or tesseract::tesseract_state_solver_kdl targets.
TRAJOPT_COMMON_CMAKE=$(find "$PWD/src" -path "*/trajopt_common/CMakeLists.txt" -print -quit 2>/dev/null || true)
if [[ -n "$TRAJOPT_COMMON_CMAKE" && -f "$TRAJOPT_COMMON_CMAKE" ]]; then
  echo "Ensuring required find_package() entries exist in $TRAJOPT_COMMON_CMAKE"
  TRAJOPT_COMMON_CMAKE="$TRAJOPT_COMMON_CMAKE" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TRAJOPT_COMMON_CMAKE"])
lines = cmake.read_text().splitlines()


def ensure_find_package(statement: str) -> None:
    if any(statement in line for line in lines):
        return
    try:
        insert_at = next(i for i, line in enumerate(lines) if line.strip().startswith("find_package"))
    except StopIteration:
        try:
            insert_at = next(i for i, line in enumerate(lines) if line.strip().startswith("project")) + 1
        except StopIteration:
            insert_at = 0
    lines.insert(insert_at, statement)


ensure_find_package("find_package(tinyxml2 CONFIG REQUIRED)")
ensure_find_package("find_package(Boost COMPONENTS graph REQUIRED)")
ensure_find_package("find_package(tesseract_state_solver COMPONENTS kdl REQUIRED)")

cmake.write_text("\n".join(lines) + "\n")
PY
fi

# Ensure the tesseract_plugins package can locate the collision backends it
# links against.  Some snapshots omit the necessary find_package() calls,
# which leaves the tesseract::tesseract_collision_* targets undefined and
# causes CMake configuration to fail before colcon can build anything else.
TESSERACT_PLUGINS_CMAKE=$(find "$PWD/src" -path "*/tesseract_plugins/CMakeLists.txt" -print -quit 2>/dev/null || true)
if [[ -n "$TESSERACT_PLUGINS_CMAKE" && -f "$TESSERACT_PLUGINS_CMAKE" ]]; then
  echo "Ensuring tesseract_plugins declares collision backend dependencies"
  TESSERACT_PLUGINS_CMAKE="$TESSERACT_PLUGINS_CMAKE" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TESSERACT_PLUGINS_CMAKE"])
lines = cmake.read_text().splitlines()


def insert_find_package(statement: str) -> None:
    """Add a find_package() statement if it is missing."""

    if any(statement in line for line in lines):
        return

    try:
        insert_at = next(
            i for i, line in enumerate(lines) if line.strip().startswith("find_package")
        )
    except StopIteration:
        try:
            insert_at = next(
                i for i, line in enumerate(lines) if line.strip().startswith("project")
            ) + 1
        except StopIteration:
            insert_at = 0

    lines.insert(insert_at, statement)


def upsert_bullet_find_package(required: bool) -> None:
    """Insert or upgrade the Bullet find_package() directive."""

    required_stmt = "find_package(tesseract_collision_bullet CONFIG REQUIRED)"
    quiet_stmt = "find_package(tesseract_collision_bullet CONFIG QUIET)"
    desired_stmt = required_stmt if required else quiet_stmt

    for idx, line in enumerate(lines):
        if line.strip().startswith("find_package(tesseract_collision_bullet"):
            lines[idx] = desired_stmt
            return

    insert_find_package(desired_stmt)


insert_find_package("find_package(tesseract_collision CONFIG REQUIRED)")
insert_find_package("find_package(tesseract_collision_fcl CONFIG QUIET)")

has_bullet_plugin = any("tesseract_collision_bullet" in line for line in lines)
bullet_guarded = any(
    "tesseract_collision_bullet_FOUND" in line
    or "ENABLE_BULLET" in line.upper()
    or ("if(" in line and "BULLET" in line.upper())
    for line in lines
)

upsert_bullet_find_package(required=has_bullet_plugin and not bullet_guarded)

cmake.write_text("\n".join(lines) + "\n")
PY
fi

# Remove any duplicate ROS packages before resolving dependencies to prevent
# rosdep and colcon build failures. Keep the first occurrence of a package
# name and discard subsequent duplicates.
declare -A pkg_seen
while read -r name path _; do
  if [[ -n "${pkg_seen[$name]:-}" ]]; then
    echo "Removing duplicate package '$name' from $path (keeping ${pkg_seen[$name]})"
    rm -rf "$path"
  else
    pkg_seen[$name]="$path"
  fi
done < <(colcon list --base-paths src)

# Skip rosdep keys for packages that are either intentionally source-only on
# Humble/Jammy or are supplied by the imported overlay workspaces. This avoids
# asking rosdep/apt for binary packages when the source checkout is the intended
# provider and also covers keys that still look unresolved to rosdep even though
# colcon can build them from the overlay (for example tesseract_visualization
# from the tesseract_qt source tree).
mapfile -t WORKSPACE_PACKAGES < <(colcon list --base-paths src --names-only)
declare -A WORKSPACE_PRESENT
for pkg in "${WORKSPACE_PACKAGES[@]}"; do
  WORKSPACE_PRESENT["$pkg"]=1
done

# Humble/Jammy full-profile overlays intentionally provide the planning stack
# from source. Keep this list aligned with the rosdep keys used by the upstream
# Tesseract/TrajOpt manifests so manual rosdep retries can reproduce the same
# bootstrap behavior.
SOURCE_OVERLAY_SKIP_KEYS=(
  boost_plugin_loader
  descartes_light
  opw_kinematics
  qt_advanced_docking
  taskflow
  tesseract
  tesseract_collision
  tesseract_command_language
  tesseract_common
  tesseract_environment
  tesseract_kinematics
  tesseract_motion_planners
  tesseract_motion_planners_core
  tesseract_motion_planners_simple
  tesseract_msgs
  tesseract_process_managers
  tesseract_rosutils
  tesseract_task_composer
  tesseract_visualization
  trajopt
  trajopt_ifopt
  trajopt_sco
  trajopt_sqp
)

# Some keys still appear in Humble dependency resolution but do not have a
# supported binary bootstrap path for this workflow. Treat them as source-only.
SOURCE_ONLY_SKIP_KEYS=(
  qt_advanced_docking
  taskflow
  tesseract_visualization
)

add_skip_key() {
  local key="$1"
  local existing
  for existing in "${SKIP_KEYS[@]:-}"; do
    if [[ "$existing" == "$key" ]]; then
      return
    fi
  done
  SKIP_KEYS+=("$key")
}

SKIP_KEYS=()
if [[ "$PROFILE" == "minimal" ]]; then
  SKIP_KEYS=(
    qt_advanced_docking
    taskflow
    tesseract_environment
    tesseract_motion_planners
    tesseract_motion_planners_core
    tesseract_motion_planners_simple
    tesseract_task_composer
    tesseract_visualization
    trajopt
    trajopt_ifopt
    trajopt_sco
    trajopt_sqp
  )
else
  for key in "${SOURCE_OVERLAY_SKIP_KEYS[@]}"; do
    if [[ -n ${WORKSPACE_PRESENT[$key]:-} ]]; then
      add_skip_key "$key"
      continue
    fi

    case "$key" in
      tesseract_visualization)
        if [[ -d "$PWD/src/tesseract_qt" ]]; then
          add_skip_key "$key"
        fi
        ;;
      qt_advanced_docking|taskflow)
        if [[ -d "$PWD/src/tesseract_qt" || -d "$PWD/src/tesseract_planning" ]]; then
          add_skip_key "$key"
        fi
        ;;
    esac
  done
fi

if [[ "$PROFILE" == "full" ]]; then
  for key in "${SOURCE_ONLY_SKIP_KEYS[@]}"; do
    add_skip_key "$key"
  done
fi

if [[ ${#SKIP_KEYS[@]} -gt 0 ]]; then
  printf "rosdep preflight (%s profile): skipping keys: %s\n" \
    "$PROFILE" "$(IFS=', '; echo "${SKIP_KEYS[*]}")"
else
  printf "rosdep preflight (%s profile): no profile-specific skip keys\n" "$PROFILE"
fi

SKIP_KEYS_ARG=$(IFS=","; echo "${SKIP_KEYS[*]}")
if [[ -n "$SKIP_KEYS_ARG" ]]; then
  echo "rosdep skip-keys: $SKIP_KEYS_ARG"
fi

rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}" \
  --skip-keys "$SKIP_KEYS_ARG"

# Consolidate CMake arguments to enforce C++17 and position independent code so
# static libraries can link cleanly into shared objects.
CMAKE_ARGS=(
  -DCMAKE_CXX_STANDARD=17
  -DCMAKE_CXX_STANDARD_REQUIRED=ON
  -DCMAKE_CXX_EXTENSIONS=OFF
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

# 2) Clean fully (start from scratch)
rm -rf build install log
find src \( -name build -o -name install -o -name log \) -print0 | xargs -0 -r rm -rf

FULL_BUILD_ARGS=()
if [[ "$PROFILE" == "full" && "$WITH_TESSERACT_QT" -eq 0 ]]; then
  echo "Full profile: skipping optional tesseract_qt (use --with-tesseract-qt to enable Studio/Qt widgets)"
  FULL_BUILD_ARGS+=(--packages-skip tesseract_qt)
fi

if [[ "$PROFILE" == "full" ]]; then
  # 1) Ensure boost_plugin_loader exists
  if [ ! -d src/boost_plugin_loader ]; then
    git -C src clone https://github.com/tesseract-robotics/boost_plugin_loader.git
  fi

  # 3) Stage 1: infrastructure vendors first
  reset_colcon_environment
  colcon build --symlink-install --packages-select \
    ros_industrial_cmake_boilerplate eigen boost_plugin_loader

  # 4) Stage 2: up to tesseract/tesseract_msgs with C++17
  reset_colcon_environment
  colcon build --symlink-install --packages-up-to tesseract tesseract_msgs \
    --cmake-args "${CMAKE_ARGS[@]}"

  # 4.5) Stage 2.5: ensure tesseract_state_solver is installed before downstream packages.
  # Build its dependency chain as well so the workspace has the expected environment
  # hooks available when colcon sources it during later stages.
  reset_colcon_environment
  colcon build --symlink-install --packages-up-to tesseract_state_solver \
    --cmake-args "${CMAKE_ARGS[@]}"

  # 5) Stage 3: full workspace
  reset_colcon_environment
  colcon build --symlink-install "${FULL_BUILD_ARGS[@]}" \
    --cmake-args "${CMAKE_ARGS[@]}"
else
  echo "Profile 'minimal': skipping source overlays and running a single workspace build"
  reset_colcon_environment
  colcon build --symlink-install --cmake-args "${CMAKE_ARGS[@]}"
fi
