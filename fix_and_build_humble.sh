#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws

APT_GET_CMD="apt-get"
if command -v sudo >/dev/null 2>&1; then
  APT_GET_CMD="sudo apt-get"
fi

# 0) Source underlays (support Humble or Jazzy)
default_rosdistro=""
for d in jazzy humble; do
  if [ -f "/opt/ros/${d}/setup.bash" ]; then
    default_rosdistro=${d}
    break
  fi
done
ROS_DISTRO=${ROS_DISTRO:-${default_rosdistro}}
[ -n "${ROS_DISTRO:-}" ] || { echo "ROS 2 distro not found (expected jazzy or humble)"; exit 1; }
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
if [ -f ~/ws_moveit2/install/setup.bash ]; then
  # shellcheck source=/dev/null
  source ~/ws_moveit2/install/setup.bash
fi

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

# Install core system dependencies (Boost + TinyXML2) using the shared helper so
# all setup paths stay in sync. The helper also reinstalls Boost headers when
# container images omit them even though dpkg claims the packages are present,
# preventing the "Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)"
# failure in trajopt_common.
"${SCRIPT_DIR}/scripts/install_system_deps.sh"

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
if [ -f "$REPO_FILE" ]; then
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

# Skip keys for packages supplied by unreleased overlays shipped with this repo.
mapfile -t WORKSPACE_PACKAGES < <(colcon list --base-paths src --names-only)
declare -A WORKSPACE_PRESENT
for pkg in "${WORKSPACE_PACKAGES[@]}"; do
  WORKSPACE_PRESENT["$pkg"]=1
done

OVERLAY_SKIP_CANDIDATES=(
  tesseract
  tesseract_process_planners
  trajopt_ifopt
  trajopt_sqp
  trajopt
  jsoncpp
  message_generation
)
SKIP_KEYS=()
for key in "${OVERLAY_SKIP_CANDIDATES[@]}"; do
  if [[ -n ${WORKSPACE_PRESENT[$key]:-} ]]; then
    SKIP_KEYS+=("$key")
  fi
done
SKIP_KEYS_ARG=$(IFS=","; echo "${SKIP_KEYS[*]}")

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

# 1) Ensure boost_plugin_loader exists
if [ ! -d src/boost_plugin_loader ]; then
  git -C src clone https://github.com/tesseract-robotics/boost_plugin_loader.git
fi

# 2) Clean fully (start from scratch)
rm -rf build install log
find src \( -name build -o -name install -o -name log \) -print0 | xargs -0 -r rm -rf

# 3) Stage 1: infrastructure vendors first
colcon build --symlink-install --packages-select \
  ros_industrial_cmake_boilerplate eigen boost_plugin_loader

# 4) Stage 2: up to tesseract_common/tesseract_msgs with C++17
colcon build --symlink-install --packages-up-to tesseract_common tesseract_msgs \
  --cmake-args "${CMAKE_ARGS[@]}"

# 4.5) Stage 2.5: ensure tesseract_state_solver is installed before downstream packages.
# Build its dependency chain as well so the workspace has the expected environment
# hooks available when colcon sources it during later stages.
colcon build --symlink-install --packages-up-to tesseract_state_solver \
  --cmake-args "${CMAKE_ARGS[@]}"

# 5) Stage 3: full workspace
colcon build --symlink-install \
  --cmake-args "${CMAKE_ARGS[@]}"
