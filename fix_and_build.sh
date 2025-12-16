#!/usr/bin/env bash
# Helper to repair and rebuild the easy_manipulation_deployment workspace.
#
# Optional environment toggles:
#   SKIP_TESTS=1           Disable Tesseract tests/examples by appending
#                          -DTESSERACT_ENABLE_TESTING=OFF and
#                          -DTESSERACT_ENABLE_EXAMPLES=OFF to CMAKE_ARGS.
#   LIGHTWEIGHT_PLANNERS=1 Disable heavyweight planners by appending
#                          -DTESSERACT_BUILD_OMPL=OFF
#                          -DTESSERACT_BUILD_DESCARTES=OFF
#                          -DTESSERACT_BUILD_TRAJOPT=OFF
#                          -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF to CMAKE_ARGS.
set -euo pipefail

WS=${WS:-$HOME/workcell_ws}
SRC="$WS/src"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SRC"

# Ensure we are being run from the repository inside the workspace. If the
# repository already lives inside a non-default workspace (e.g., after cloning
# into ~/custom_ws/src), infer WS/SRC from its location instead of forcing the
# hard-coded ~/workcell_ws layout.
if [[ "$REPO_DIR" != "$SRC/easy_manipulation_deployment" ]]; then
  if [[ "$REPO_DIR" =~ ^(.+)/src/easy_manipulation_deployment$ ]]; then
    WS="${BASH_REMATCH[1]}"
    SRC="$WS/src"
    echo "Detected workspace at $WS based on repository location" >&2
  else
    echo "Please run this script from ~/workcell_ws/src/easy_manipulation_deployment or set WS to your workspace root" >&2
    exit 1
  fi
fi

# Select ROS distribution: prefer Humble then Jazzy. If ROS_DISTRO is already
# set, verify the corresponding setup script exists before continuing.
if [[ -n ${ROS_DISTRO:-} ]]; then
  if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "ROS_DISTRO is set to '$ROS_DISTRO', but /opt/ros/$ROS_DISTRO/setup.bash is missing" >&2
    exit 1
  fi
else
  for d in humble jazzy; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then
      ROS_DISTRO=$d
      break
    fi
  done
fi

if [[ -z ${ROS_DISTRO:-} ]]; then
  echo "No supported ROS distro found (need humble or jazzy)" >&2
  exit 1
fi
echo "Using ROS_DISTRO=$ROS_DISTRO (script validated on ROS 2 Humble / Ubuntu 22.04)" >&2

# shellcheck source=/dev/null
# Disable nounset while sourcing ROS setup, as it references
# environment variables that may not be defined.
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
export LANG=C.UTF-8 LC_ALL=C.UTF-8

# Ensure required tools. Prefer password-less sudo when available, otherwise fall
# back to raw apt-get (useful in containerized environments running as root).
APT_GET="apt-get"
if command -v sudo >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    APT_GET="sudo apt-get"
  elif [[ $EUID -ne 0 ]]; then
    echo "sudo is present but requires a password; prompting for credentials" >&2
    APT_GET="sudo apt-get"
  fi
elif [[ $EUID -ne 0 ]]; then
  echo "This script needs root privileges to install dependencies; please run with sudo" >&2
  exit 1
fi

apt_get_retry() {
  local attempts=5
  local delay=5
  local output
  local status

  for ((attempt = 1; attempt <= attempts; attempt++)); do
    set +e
    output=$($APT_GET "$@" 2>&1)
    status=$?
    set -e

    if [[ $status -eq 0 ]]; then
      printf '%s\n' "$output"
      return 0
    fi

    printf '%s\n' "$output" >&2

    if [[ $output == *"Could not get lock"* || $output == *"Unable to lock"* ]]; then
      if (( attempt < attempts )); then
        echo "Encountered apt lock; retrying in ${delay}s (attempt ${attempt}/${attempts})" >&2
        sleep "$delay"
        continue
      fi
    fi

    return "$status"
  done
}

ensure_pyyaml() {
  if python3 - <<'PY'
import yaml  # noqa: F401
PY
  then
    return
  fi

  echo "Installing python3-yaml for repos processing"
  apt_get_retry install -y python3-yaml
}

MISSING_APT_PACKAGES=()
INSTALL_ROSDEP=0

for cmd in git colcon rosdep vcs; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing tool detected: $cmd"
    case "$cmd" in
      git) MISSING_APT_PACKAGES+=(git);;
      colcon) MISSING_COLCON=1;;
      rosdep)
        MISSING_APT_PACKAGES+=(python3-rosdep)
        INSTALL_ROSDEP=1
        ;;
      vcs) MISSING_APT_PACKAGES+=(python3-vcstool);;
    esac
  fi
done

if [[ ${#MISSING_APT_PACKAGES[@]} -gt 0 ]]; then
  apt_get_retry update -y
  apt_get_retry install -y "${MISSING_APT_PACKAGES[@]}"
fi

if [[ ${MISSING_COLCON:-0} -eq 1 ]]; then
  python3 -m pip install -U colcon-common-extensions
fi

if [[ $INSTALL_ROSDEP -eq 1 ]]; then
  rosdep init || true
fi

# Ensure core system dependencies (Boost + TinyXML2) are present.  Calling the
# shared helper keeps the dependency list in one place so it stays in sync with
# the README instructions and avoids repeated Boost discovery failures in
# trajopt_common.
"$REPO_DIR/scripts/install_system_deps.sh"

# Import any missing repositories listed in tesseract.repos.  Relying solely on
# the presence of $SRC/tesseract is insufficient when the workspace already
# contains a partial overlay copy (e.g., tesseract without tesseract_plugins).
if [[ -f "$REPO_DIR/tesseract.repos" ]]; then
  ensure_pyyaml

  mapfile -t MISSING_REPOS < <(REPOS_FILE="$REPO_DIR/tesseract.repos" SRC="$SRC" python3 - <<'PY'
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
    # Attempt to satisfy trajopt_ifopt from binary packages if the source
    # repository is unavailable publicly. This avoids interactive GitHub
    # authentication prompts when the upstream repository is private.
    FILTERED_MISSING=()
    for repo in "${MISSING_REPOS[@]}"; do
      if [[ $repo == "trajopt_ifopt" ]]; then
        if apt-cache show "ros-$ROS_DISTRO-trajopt-ifopt" >/dev/null 2>&1; then
          echo "Installing ros-$ROS_DISTRO-trajopt-ifopt from apt instead of cloning"
          apt_get_retry install -y "ros-$ROS_DISTRO-trajopt-ifopt"
          continue
        else
          echo "ros-$ROS_DISTRO-trajopt-ifopt not available via apt; will attempt to clone" >&2
        fi
      fi
      FILTERED_MISSING+=("$repo")
    done

    if [[ ${#FILTERED_MISSING[@]} -eq 0 ]]; then
      MISSING_REPOS=()
    else
      MISSING_REPOS=("${FILTERED_MISSING[@]}")
    fi
  fi

  if [[ ${#MISSING_REPOS[@]} -gt 0 ]]; then
    echo "Importing missing repositories: ${MISSING_REPOS[*]}"
    FILTERED_REPOS=$(mktemp)
    REPOS_FILE="$REPO_DIR/tesseract.repos" MISSING="${MISSING_REPOS[*]}" python3 - <<'PY' >"$FILTERED_REPOS"
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required to process the repos file")

repos_file = os.environ["REPOS_FILE"]
missing = {item for item in os.environ.get("MISSING", "").split() if item}
token = os.environ.get("GITHUB_TOKEN_FOR_IMPORT") or os.environ.get("GITHUB_TOKEN")


def with_token(url: str) -> str:
    """Inject a GitHub token into the URL to allow non-interactive clones."""

    if not token or "github.com" not in url or "@" in url:
        return url

    prefix = "https://"
    if url.startswith(prefix):
        return f"{prefix}{token}@{url[len(prefix):]}"
    return url

with open(repos_file, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

repos = data.get("repositories") or {}
selected = {name: repos[name] for name in repos if name in missing}

if token:
    for repo in selected.values():
        if isinstance(repo, dict) and isinstance(repo.get("url"), str):
            repo["url"] = with_token(repo["url"])

yaml.safe_dump({"repositories": selected}, sys.stdout)
PY
    if ! GIT_TERMINAL_PROMPT=${GIT_TERMINAL_PROMPT:-0} vcs import --recursive "$SRC" < "$FILTERED_REPOS"; then
      echo "Failed to import repositories non-interactively. If access requires authentication, rerun with GIT_TERMINAL_PROMPT=1." >&2
      rm -f "$FILTERED_REPOS"
      exit 1
    fi
    rm -f "$FILTERED_REPOS"
  fi
fi

# Copy overlays from repo checkout if they exist
# Copy overlays from repo checkout if they exist and immediately reveal packages
HIDDEN_OVERLAY_PREFIXES=("$SRC/tesseract_ros2")
should_skip_marker() {
  local marker=$1
  for prefix in "${HIDDEN_OVERLAY_PREFIXES[@]}"; do
    if [[ $marker == "$prefix"* ]]; then
      return 0
    fi
  done
  return 1
}

for overlay in tesseract trajopt; do
  if [[ -d "$REPO_DIR/$overlay" ]]; then
    mkdir -p "$SRC/$overlay"
    cp -a "$REPO_DIR/$overlay/." "$SRC/$overlay/"
    # Rename ignore markers that may ship with upstream overlays so colcon sees the packages
    while IFS= read -r -d '' marker; do
      if should_skip_marker "$marker"; then
        continue
      fi
      echo "Renaming ignore marker $marker"
      mv -f "$marker" "$marker.repo"
    done < <(find "$SRC/$overlay" -maxdepth 2 \( -name 'COLCON_IGNORE' -o -name 'AMENT_IGNORE' \) -print0)
  fi
done

# Ensure the workspace trajopt checkout tracks a stable branch instead of a detached commit
TRAJOPT_WS_DIR="$SRC/trajopt"
if [[ -d "$TRAJOPT_WS_DIR/.git" ]]; then
  current_ref=$(git -C "$TRAJOPT_WS_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)
  if [[ "$current_ref" == "HEAD" ]]; then
    if git -C "$TRAJOPT_WS_DIR" show-ref --verify --quiet refs/heads/main || \
       git -C "$TRAJOPT_WS_DIR" show-ref --verify --quiet refs/remotes/origin/main; then
      git -C "$TRAJOPT_WS_DIR" checkout --quiet main 2>/dev/null || \
        git -C "$TRAJOPT_WS_DIR" checkout --quiet origin/main
      echo "Switched trajopt to the main branch to avoid detached HEAD builds" >&2
    else
      echo "trajopt repository is detached and no main branch was found; please check out a stable tag manually." >&2
    fi
  fi
fi

# Ensure workspace layout is sanitized (handles stray trajopt sources)
WS="$WS" "$REPO_DIR/scripts/fix_workspace_layout.sh"

# Ensure boost_plugin_loader exists
if [[ ! -d "$SRC/boost_plugin_loader" ]]; then
  if ! GIT_TERMINAL_PROMPT=${GIT_TERMINAL_PROMPT:-0} git clone https://github.com/tesseract-robotics/boost_plugin_loader.git "$SRC/boost_plugin_loader"; then
    echo "Failed to clone boost_plugin_loader. Verify GitHub access or rerun with GIT_TERMINAL_PROMPT=1 to allow credentials prompts." >&2
    exit 1
  fi
fi

# Reveal any hidden Tesseract packages that may remain from upstream checkouts
find "$SRC" -path "$SRC/tesseract*" \( -name COLCON_IGNORE -o -name AMENT_IGNORE \) -print | while read -r f; do
  if should_skip_marker "$f"; then
    continue
  fi
  echo "Renaming ignore marker $f"
  mv -f "$f" "$f.repo"
done

# Remove duplicate package names
declare -A seen
while read -r name path _; do
  if [[ -n ${seen[$name]:-} && ${seen[$name]} != "$path" ]]; then
    echo "Removing duplicate package $name from $path (keeping ${seen[$name]})"
    rm -rf "$path"
  else
    seen[$name]="$path"
  fi
done < <(colcon list --base-paths "$SRC")

# Verify required packages are visible
if ! colcon list --base-paths "$SRC" | grep -E '^(tesseract_common|tesseract_msgs)\b' >/dev/null; then
  echo "tesseract_common or tesseract_msgs not visible to colcon" >&2
  exit 1
fi

# Install dependencies. Pass the explicit package paths reported by colcon to avoid
# rosdep scanning duplicate overlays (e.g., both the repo checkout and the copied
# overlay of trajopt_sco) which would otherwise trigger "Multiple packages found"
# errors.
mapfile -t ROSDEP_PATHS < <(colcon list --base-paths "$SRC" --paths-only)
if [[ ${#ROSDEP_PATHS[@]} -eq 0 ]]; then
  echo "No packages discovered for rosdep installation" >&2
  exit 1
fi

ROSDEP_OVERRIDES="$REPO_DIR/scripts/rosdep_overrides.yaml"
if [[ -f "$ROSDEP_OVERRIDES" ]]; then
  export ROSDEP_ADDITIONAL_SOURCES_PATHS="$ROSDEP_OVERRIDES${ROSDEP_ADDITIONAL_SOURCES_PATHS:+:$ROSDEP_ADDITIONAL_SOURCES_PATHS}"
  echo "Using rosdep overrides from $ROSDEP_OVERRIDES" >&2
fi

rosdep update
# Some snapshots reference rosdep keys that are missing or incorrect for this
# workspace (for example, "rviz" under ROS 2).  Skip those keys and install the
# ones we still need manually so rosdep can proceed without hard errors.  Also
# skip packages provided by the repo's overlays so rosdep does not try to
# resolve them from the system.
mapfile -t WORKSPACE_PACKAGES < <(colcon list --base-paths "$SRC" --names-only)
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
OVERLAY_SKIP_KEYS=()
for key in "${OVERLAY_SKIP_CANDIDATES[@]}"; do
  if [[ -n ${WORKSPACE_PRESENT[$key]:-} ]]; then
    OVERLAY_SKIP_KEYS+=("$key")
  fi
done
SKIP_KEYS=(
  catkin
  rviz
  roslib
)
ALL_SKIP_KEYS=("${OVERLAY_SKIP_KEYS[@]}" "${SKIP_KEYS[@]}")
SKIP_KEYS_ARG=$(IFS=","; echo "${ALL_SKIP_KEYS[*]}")

rosdep install --from-paths "${ROSDEP_PATHS[@]}" --ignore-src -yr --rosdistro "$ROS_DISTRO" \
  --skip-keys "$SKIP_KEYS_ARG"

BOOST_DEPS=(
  libboost-dev
  libboost-program-options-dev
  libboost-stacktrace-dev
  libboost-serialization-dev
  libboost-graph-dev
)
TINYXML2_DEPS=(
  libtinyxml2-dev
)
CEREAL_DEPS=(
  libcereal-dev
)

# Install Boost development packages explicitly because rosdep may skip or fail
# to resolve them in some environments.  Using apt-get directly ensures the
# headers are present before CMake runs instead of relying on a potentially
# stale dpkg check.
echo "Ensuring required Boost development packages are installed" >&2
apt_get_retry update -y
apt_get_retry install -y "${BOOST_DEPS[@]}" "${TINYXML2_DEPS[@]}" "${CEREAL_DEPS[@]}"

# Double-check the Boost headers are discoverable before running CMake.  Both
# rosdep and the manual install above should have pulled them in, but explicitly
# failing fast here provides a clearer error than a later "Could NOT find
# Boost" message when the required components are missing.
if [[ ! -f /usr/include/boost/program_options/options_description.hpp || ! -f /usr/include/boost/serialization/serialization.hpp || \
      ! -f /usr/include/boost/stacktrace.hpp || ! -f /usr/include/boost/graph/adjacency_list.hpp ]]; then
  echo "Boost headers for required components are missing even after installing ${BOOST_DEPS[*]}." >&2
  echo "Please ensure the listed Boost dev packages (including Boost.Graph) are available and rerun this script." >&2
  exit 1
fi

# Ensure cereal headers are available for the planners that serialize profiles.
if [[ ! -f /usr/include/cereal/cereal.hpp ]]; then
  echo "cereal headers are missing even after installing ${CEREAL_DEPS[*]}." >&2
  echo "Please ensure libcereal-dev is available for your distribution and rerun this script." >&2
  exit 1
fi

# Ensure tinyxml2 provides a CMake package configuration for trajopt_sco to
# consume.  Ubuntu Noble's package switched to a lower-case filename
# (tinyxml2-config.cmake), while Jammy shipped tinyxml2Config.cmake.  Accept
# either to keep the check working across both releases instead of forcing the
# older path and failing even when the dev package is installed.
TINYXML2_CONFIG_DIR=/usr/lib/x86_64-linux-gnu/cmake/tinyxml2
shopt -s nullglob
TINYXML2_CONFIGS=(
  "$TINYXML2_CONFIG_DIR"/tinyxml2Config.cmake
  "$TINYXML2_CONFIG_DIR"/tinyxml2-config.cmake
)
shopt -u nullglob

if [[ ${#TINYXML2_CONFIGS[@]} -eq 0 ]]; then
  echo "tinyxml2 CMake package config not found; ensure ${TINYXML2_DEPS[*]} is available." >&2
  exit 1
fi

# Enforce C++17 and position independent code so static libraries can be linked
# into shared objects without -fPIC relocation errors.
export AMENT_CMAKE_CXX_STANDARD=17
CMAKE_ARGS=(
  -DCMAKE_CXX_STANDARD=17
  -DCMAKE_CXX_STANDARD_REQUIRED=ON
  -DCMAKE_CXX_EXTENSIONS=OFF
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  # Force CMake to use the system cereal package instead of FetchContent to avoid network delays during configure.
  -DTESSERACT_DISABLE_CEREAL_FETCH=ON
)

CEREAL_CMAKE_DIRS=(
  /usr/lib/cmake/cereal
  /usr/lib/"${DEB_HOST_MULTIARCH:-x86_64-linux-gnu}"/cmake/cereal
)
for dir in "${CEREAL_CMAKE_DIRS[@]}"; do
  if [[ -f "$dir/cerealConfig.cmake" ]]; then
    CMAKE_ARGS+=("-Dcereal_DIR=$dir")
    break
  fi
done
if [[ ${SKIP_TESTS:-0} -eq 1 ]]; then
  CMAKE_ARGS+=(
    -DTESSERACT_ENABLE_TESTING=OFF
    -DTESSERACT_ENABLE_EXAMPLES=OFF
  )
fi

if [[ ${LIGHTWEIGHT_PLANNERS:-0} -eq 1 ]]; then
  CMAKE_ARGS+=(
    -DTESSERACT_BUILD_OMPL=OFF
    -DTESSERACT_BUILD_DESCARTES=OFF
    -DTESSERACT_BUILD_TRAJOPT=OFF
    -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF
  )
fi

COLCON_BASE_ARGS=(
  --symlink-install
  --merge-install
  --allow-overriding
  tesseract_motion_planners
)

# Minimal include fix for profile_dictionary
# Use find's -print -quit to grab the first match and avoid non-zero exit when not found
PDH=$(find "$SRC" -path "*/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h" -print -quit 2>/dev/null)
if [[ -f "$PDH" ]] && grep -q '<shared_mutex>' "$PDH" && ! grep -q '<mutex>' "$PDH"; then
  echo "Patching missing <mutex> include in $PDH"
  sed -i '/<shared_mutex>/a #include <mutex>' "$PDH"
fi

# Ensure tesseract_command_language can locate the helper macros exported by
# tesseract_common. Some environments fail to load the CONFIG_EXTRAS provided by
# tesseract_common automatically, which leaves tesseract_variables() undefined
# and causes CMake to abort early. Inject a small fallback before the first call
# to tesseract_variables() so the build can continue.
TCL_CMAKE=$(find "$SRC" -path "*/tesseract_command_language/CMakeLists.txt" -print -quit 2>/dev/null || true)
if [[ -n "$TCL_CMAKE" && -f "$TCL_CMAKE" ]]; then
  echo "Adding tesseract_common fallback include to $TCL_CMAKE"
  TCL_CMAKE="$TCL_CMAKE" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TCL_CMAKE"])
lines = cmake.read_text().splitlines()

begin_marker = "# BEGIN tesseract_common fallback"
end_marker = "# END tesseract_common fallback"
sentinel = "  message(FATAL_ERROR \"tesseract_variables() is unavailable; ensure tesseract_common is discoverable\")"


def find_index(predicate):
    for idx, line in enumerate(lines):
        if predicate(line):
            return idx
    return None


def remove_existing_block():
    """Strip any previously injected guard to avoid malformed nesting."""

    if begin_marker in lines:
        start = lines.index(begin_marker)
        try:
            end = lines.index(end_marker, start) + 1
        except ValueError:
            end = start
            while end < len(lines) and lines[end].strip():
                end += 1
            if end < len(lines):
                end += 1
        del lines[start:end]
        return

    sentinel_idx = find_index(lambda l: "tesseract_variables() is unavailable" in l)
    if sentinel_idx is not None:
        start = sentinel_idx
        while start > 0 and lines[start - 1].strip():
            start -= 1
        end = sentinel_idx + 1
        while end < len(lines) and lines[end].strip():
            end += 1
        if end < len(lines):
            end += 1
        del lines[start:end]


guard = [
    begin_marker,
    "if(NOT DEFINED tesseract_common_DIR)",
    "  find_package(tesseract_common QUIET CONFIG)",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    "  if(DEFINED tesseract_common_DIR AND EXISTS \"${tesseract_common_DIR}/tesseract_common_module_path.cmake\")",
    "    include(\"${tesseract_common_DIR}/tesseract_common_module_path.cmake\")",
    "  endif()",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    "  find_path(_tesseract_common_src cmake/tesseract_macros.cmake",
    "    HINTS",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../../tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../easy_manipulation_deployment/tesseract/tesseract_common\"",
    "      \"${CMAKE_CURRENT_LIST_DIR}/../../../easy_manipulation_deployment/tesseract/tesseract_common\"",
    "    PATH_SUFFIXES share/tesseract_common",
    "  )",
    "  if(NOT _tesseract_common_src STREQUAL \"_tesseract_common_src-NOTFOUND\")",
    "    list(APPEND CMAKE_MODULE_PATH \"${_tesseract_common_src}/cmake\")",
    "    if(EXISTS \"${_tesseract_common_src}/cmake/tesseract_macros.cmake\")",
    "      include(\"${_tesseract_common_src}/cmake/tesseract_macros.cmake\")",
    "    endif()",
    "  endif()",
    "endif()",
    "if(NOT COMMAND tesseract_variables)",
    sentinel,
    "endif()",
    end_marker,
    "",
]


remove_existing_block()

insert_at = find_index(lambda l: "tesseract_variables()" in l)
if insert_at is not None:
    lines[insert_at:insert_at] = guard


cmake.write_text("\n".join(lines) + "\n")
PY

  # Ensure the tesseract_plugins package can locate the collision backends it
  # links against.  Some snapshots omit the necessary find_package() calls,
  # which leaves the tesseract::tesseract_collision_* targets undefined and
  # causes CMake configuration to fail before colcon can build anything else.
  if TESSERACT_PLUGINS_CMAKE=$(find "$SRC" -path "*/tesseract_plugins/CMakeLists.txt" -print -quit 2>/dev/null); then
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


  echo "Ensuring tesseract_command_language declares tinyxml2 and Boost stacktrace dependencies"
  TCL_CMAKE="$TCL_CMAKE" python3 - <<'PY'
import os
from pathlib import Path

cmake = Path(os.environ["TCL_CMAKE"])
lines = cmake.read_text().splitlines()


def ensure_find_package(statement: str) -> None:
    """Insert a find_package() statement if it is missing."""

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


ensure_find_package("find_package(tinyxml2 CONFIG QUIET)")


def ensure_boost_stacktrace_guard() -> None:
    """Prefer Boost stacktrace_backtrace but fall back to basic or noop."""

    begin_marker = "# BEGIN boost stacktrace fallback"
    end_marker = "# END boost stacktrace fallback"

    if begin_marker in lines:
        start = lines.index(begin_marker)
        try:
            end = lines.index(end_marker, start) + 1
        except ValueError:
            end = start + 1
        del lines[start:end]

    guard = [
        begin_marker,
        "option(TESSERACT_FORCE_BOOST_STACKTRACE \"Force Boost stacktrace search even if headers are missing\" OFF)",
        "if(NOT TARGET Boost::stacktrace_backtrace AND NOT TARGET Boost::stacktrace_basic AND NOT TARGET Boost::stacktrace_noop)",
        "  find_path(BOOST_STACKTRACE_HEADER boost/stacktrace.hpp)",
        "  if(BOOST_STACKTRACE_HEADER STREQUAL \"BOOST_STACKTRACE_HEADER-NOTFOUND\")",
        "    if(TESSERACT_FORCE_BOOST_STACKTRACE)",
        "      find_package(Boost COMPONENTS stacktrace_backtrace stacktrace_basic stacktrace_noop QUIET)",
        "    else()",
        "      message(WARNING \"Boost stacktrace headers not found; using noop interface to avoid BoostDetectToolset\")",
        "      add_library(Boost::stacktrace_noop INTERFACE IMPORTED)",
        "      target_compile_definitions(Boost::stacktrace_noop INTERFACE BOOST_STACKTRACE_USE_NOOP)",
        "    endif()",
        "  else()",
        "    find_package(Boost COMPONENTS stacktrace_backtrace stacktrace_basic stacktrace_noop QUIET)",
        "    if(NOT TARGET Boost::stacktrace_backtrace AND NOT TARGET Boost::stacktrace_basic AND NOT TARGET Boost::stacktrace_noop)",
        "      add_library(Boost::stacktrace_noop INTERFACE IMPORTED)",
        "      target_compile_definitions(Boost::stacktrace_noop INTERFACE BOOST_STACKTRACE_USE_NOOP)",
        "      target_include_directories(Boost::stacktrace_noop INTERFACE \"${BOOST_STACKTRACE_HEADER}/..\")",
        "    endif()",
        "  endif()",
        "endif()",
        end_marker,
        "",
    ]

    try:
        insert_at = next(
            i
            for i, line in enumerate(lines)
            if line.strip().startswith("find_package")
        )
    except StopIteration:
        insert_at = 0

    lines[insert_at:insert_at] = guard


ensure_boost_stacktrace_guard()

cmake.write_text("\n".join(lines) + "\n")
PY
fi

  # Ensure trajopt_common declares all dependencies it links against. Some upstream
  # snapshots omit the KDL state solver find_package() call, which results in
  # CMake configure errors when the solver target is unavailable. TinyXML2 and
  # Boost are now handled by system dependency setup earlier in the script.
  TRAJOPT_COMMON_CMAKE=$(find "$SRC" -path "*/trajopt_common/CMakeLists.txt" -print -quit 2>/dev/null || true)
  if [[ -n "$TRAJOPT_COMMON_CMAKE" && -f "$TRAJOPT_COMMON_CMAKE" ]]; then
    if [[ "$TRAJOPT_COMMON_CMAKE" == "$REPO_DIR"/* ]]; then
      echo "Skipping modification of tracked file $TRAJOPT_COMMON_CMAKE"
    else
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
        insert_at = next(
            i for i, line in enumerate(lines) if line.strip().startswith("find_package")
        )
    except StopIteration:
        try:
            insert_at = (
                next(i for i, line in enumerate(lines) if line.strip().startswith("project"))
                + 1
            )
        except StopIteration:
            insert_at = 0
    lines.insert(insert_at, statement)


ensure_find_package("find_package(tesseract_state_solver COMPONENTS kdl REQUIRED)")

cmake.write_text("\n".join(lines) + "\n")
PY
    fi
  fi

  # Ensure the collision coefficient maps use a stable hash implementation for
  # LinkNamesPair. Older snapshots relied on std::hash<std::pair<>> which is
  # undefined and fails to compile on libstdc++11.
  TRAJOPT_COLLISION_TYPES=$(find "$SRC" -path "*/trajopt_common/include/trajopt_common/collision_types.h" -print -quit 2>/dev/null || true)
  TRAJOPT_COLLISION_TYPES_SRC=$(find "$SRC" -path "*/trajopt_common/src/collision_types.cpp" -print -quit 2>/dev/null || true)
  if [[ -n "$TRAJOPT_COLLISION_TYPES" && -f "$TRAJOPT_COLLISION_TYPES" && -n "$TRAJOPT_COLLISION_TYPES_SRC" && -f "$TRAJOPT_COLLISION_TYPES_SRC" ]]; then
    echo "Enforcing PairHash usage in trajopt_common collision coefficient maps"
    TRAJOPT_COLLISION_TYPES="$TRAJOPT_COLLISION_TYPES" TRAJOPT_COLLISION_TYPES_SRC="$TRAJOPT_COLLISION_TYPES_SRC" python3 - <<'PY'
import os
from pathlib import Path

header = Path(os.environ["TRAJOPT_COLLISION_TYPES"])
source = Path(os.environ["TRAJOPT_COLLISION_TYPES_SRC"])

header_replacements = {
    "std::unordered_map<tesseract_common::LinkNamesPair, double>":
        "std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash>",
}

source_replacements = {
    "const std::unordered_map<tesseract_common::LinkNamesPair, double>& CollisionCoeffData::getCollisionCoeffPairData() const":
        "const std::unordered_map<tesseract_common::LinkNamesPair, double, tesseract_common::PairHash>& CollisionCoeffData::getCollisionCoeffPairData() const",
}


def apply(text: str, mapping: dict) -> str:
    for old, new in mapping.items():
        text = text.replace(old, new)
    return text


header_text = header.read_text()
updated_header = apply(header_text, header_replacements)
if updated_header != header_text:
    header.write_text(updated_header)

source_text = source.read_text()
updated_source = apply(source_text, source_replacements)
if updated_source != source_text:
    source.write_text(updated_source)
PY
  fi

cd "$WS"

# If a previous build used the default isolated layout, colcon will refuse to
# reuse the existing install directory when invoked with --merge-install.
# Detect this mismatch early and remove the stale install tree so the merge
# layout requested below can be honored without manual cleanup.
LAYOUT_FILE="$WS/install/.colcon_install_layout"
if [[ -f "$LAYOUT_FILE" ]]; then
  layout=$(<"$LAYOUT_FILE")
  if [[ "$layout" != "merged" ]]; then
    echo "Removing install directory created with '$layout' layout to allow --merge-install" >&2
    rm -rf "$WS/install"
  fi
fi

source_install() {
  if [[ -f install/setup.bash ]]; then
    set +u
    : "${COLCON_TRACE:=}"
    source install/setup.bash
    # Ensure plain CMake packages that ship Colcon environment hooks properly
    # extend CMAKE_PREFIX_PATH even when sourced multiple times.  Some of the
    # vendored Tesseract packages only provide a hook script and do not rely on
    # ament to evaluate it, so manually source any available hooks here.
    if ! declare -F _colcon_prepend_unique_value >/dev/null; then
      _colcon_prepend_unique_value() {
        local listname="$1"
        local value="$2"
        local existing
        existing="$(eval "printf '%s' \"\${$listname:-}\"")"
        if [[ -n "$existing" ]]; then
          case ":$existing:" in
            *":$value:"*) return 0 ;;
          esac
        fi
        if [[ -n "$existing" ]]; then
          eval "export ${listname}=\"$value:$existing\""
        else
          eval "export ${listname}=\"$value\""
        fi
      }
    fi
    if compgen -G "install/*/share/*/hook/cmake_prefix_path.sh" >/dev/null; then
      # shellcheck disable=SC1090
      for hook in install/*/share/*/hook/cmake_prefix_path.sh; do
        source "$hook"
      done
    fi
    set -u
  fi
}

ensure_tesseract_collision_core_config() {
  local package_prefix="$WS/install/tesseract_collision"
  local config_dir="$package_prefix/lib/cmake/tesseract_collision_core"
  local config_file="$config_dir/tesseract_collision_coreConfig.cmake"

  if [[ -f "$config_file" ]] || [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local core_lib=""
  local candidate
  for candidate in \
    "$package_prefix/lib/libtesseract_collision_core.so" \
    "$package_prefix/lib/libtesseract_collision_core.dylib" \
    "$package_prefix/lib/tesseract_collision_core.lib" \
    "$package_prefix/bin/tesseract_collision_core.dll"; do
    if [[ -f "$candidate" ]]; then
      core_lib="$candidate"
      break
    fi
  done

  if [[ -z "$core_lib" ]]; then
    echo "tesseract_collision_core library not found under $package_prefix; cannot synthesize CMake config" >&2
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_collision"
  fi

  mkdir -p "$config_dir"

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_collision_core when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(console_bridge)
find_dependency(tesseract_geometry)
find_dependency(tesseract_common)
find_dependency(yaml-cpp)
find_dependency(octomap)
find_dependency(Boost COMPONENTS filesystem system program_options serialization)
if(NOT TARGET tesseract_collision_core)
  add_library(tesseract_collision_core SHARED IMPORTED)
  set_target_properties(tesseract_collision_core PROPERTIES
    IMPORTED_LOCATION "${core_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}")
endif()
if(NOT TARGET tesseract::tesseract_collision_core)
  add_library(tesseract::tesseract_collision_core ALIAS tesseract_collision_core)
endif()
set(tesseract_collision_core_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_collision/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_collision/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_collision/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

path = os.environ["PACKAGE_XML"]
try:
    root = ET.parse(path).getroot()
except Exception:
    print("0.0.0")
else:
    text = root.findtext("version")
    print(text.strip() if text else "0.0.0")
PY
)"
    version="${version:-0.0.0}"
  fi

  local version_file="$config_dir/tesseract_collision_coreConfigVersion.cmake"
  local version_script
  version_script=$(mktemp)
  cat >"$version_script" <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P "$version_script"
  rm -f "$version_script"

  echo "Synthesized missing tesseract_collision_core package configuration at $config_dir"
}

ensure_tesseract_collision_bullet_config() {
  local package_prefix="$WS/install/tesseract_collision"
  local config_dir="$package_prefix/lib/cmake/tesseract_collision_bullet"
  local config_file="$config_dir/tesseract_collision_bulletConfig.cmake"

  if [[ -f "$config_file" ]] || [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local bullet_lib=""
  local candidate
  for candidate in \
    "$package_prefix/lib/libtesseract_collision_bullet.so" \
    "$package_prefix/lib/libtesseract_collision_bullet.dylib" \
    "$package_prefix/lib/tesseract_collision_bullet.lib" \
    "$package_prefix/bin/tesseract_collision_bullet.dll"; do
    if [[ -f "$candidate" ]]; then
      bullet_lib="$candidate"
      break
    fi
  done

  if [[ -z "$bullet_lib" ]]; then
    echo "tesseract_collision_bullet library not found under $package_prefix; cannot synthesize CMake config" >&2
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_collision"
  fi

  mkdir -p "$config_dir"

  local extras_source="$REPO_DIR/tesseract/tesseract_collision/bullet/cmake/bullet-extras.cmake"
  if [[ -f "$extras_source" ]]; then
    cp -f "$extras_source" "$config_dir/bullet-extras.cmake"
  fi

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_collision_bullet when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(tesseract_collision COMPONENTS core)
if(EXISTS "\${CMAKE_CURRENT_LIST_DIR}/bullet-extras.cmake")
  include("\${CMAKE_CURRENT_LIST_DIR}/bullet-extras.cmake")
endif()
if(NOT TARGET tesseract_collision_bullet)
  add_library(tesseract_collision_bullet SHARED IMPORTED)
  set_target_properties(tesseract_collision_bullet PROPERTIES
    IMPORTED_LOCATION "${bullet_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_collision_core")
endif()
if(NOT TARGET tesseract::tesseract_collision_bullet)
  add_library(tesseract::tesseract_collision_bullet ALIAS tesseract_collision_bullet)
endif()
set(tesseract_collision_bullet_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_collision/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_collision/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_collision/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

path = os.environ["PACKAGE_XML"]
try:
    root = ET.parse(path).getroot()
except Exception:
    print("0.0.0")
else:
    text = root.findtext("version")
    print(text.strip() if text else "0.0.0")
PY
)"
    version="${version:-0.0.0}"
  fi

  local version_file="$config_dir/tesseract_collision_bulletConfigVersion.cmake"
  local version_script
  version_script=$(mktemp)
  cat >"$version_script" <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P "$version_script"
  rm -f "$version_script"

  echo "Synthesized missing tesseract_collision_bullet package configuration at $config_dir"
}

# Generate a minimal CMake package configuration for tesseract_state_solver when
# the upstream build fails to install one.  Downstream packages like
# tesseract_kinematics rely on tesseract_state_solverConfig.cmake to discover the
# core, KDL, and OFKT targets.
ensure_tesseract_state_solver_config() {
  local package_prefix="$WS/install/tesseract_state_solver"
  local config_dir="$package_prefix/lib/cmake/tesseract_state_solver"
  local config_file="$config_dir/tesseract_state_solverConfig.cmake"

  if [[ -f "$config_file" ]]; then
    local needs_regeneration=false

    # Some upstream builds generate a config that tries to create alias targets
    # without first declaring the underlying imported libraries. Detect the
    # missing definitions so we can replace the broken config with a synthesized
    # one that defines the core, KDL, and OFKT targets before aliasing them.
    if ! grep -Eq "add_library\\(tesseract_state_solver_core" "$config_file"; then
      needs_regeneration=true
    fi
    if ! grep -Eq "add_library\\(tesseract_state_solver_kdl" "$config_file"; then
      needs_regeneration=true
    fi
    if ! grep -Eq "add_library\\(tesseract_state_solver_ofkt" "$config_file"; then
      needs_regeneration=true
    fi

    if [[ "$needs_regeneration" == false ]]; then
      return
    fi

    echo "Replacing invalid tesseract_state_solverConfig.cmake at $config_file"
  fi

  if [[ ! -d "$package_prefix" ]]; then
    return
  fi

  local include_dir="$package_prefix/include"
  if [[ ! -d "$include_dir" ]]; then
    include_dir="$package_prefix/include/tesseract_state_solver"
  fi

  local kdl_lib="" ofkt_lib="" core_link
  for candidate in \
    "$package_prefix/lib/libtesseract_state_solver_kdl.so" \
    "$package_prefix/lib/libtesseract_state_solver_kdl.dylib" \
    "$package_prefix/lib/tesseract_state_solver_kdl.lib" \
    "$package_prefix/bin/tesseract_state_solver_kdl.dll"; do
    if [[ -f "$candidate" ]]; then
      kdl_lib="$candidate"
      break
    fi
  done

  for candidate in \
    "$package_prefix/lib/libtesseract_state_solver_ofkt.so" \
    "$package_prefix/lib/libtesseract_state_solver_ofkt.dylib" \
    "$package_prefix/lib/tesseract_state_solver_ofkt.lib" \
    "$package_prefix/bin/tesseract_state_solver_ofkt.dll"; do
    if [[ -f "$candidate" ]]; then
      ofkt_lib="$candidate"
      break
    fi
  done

  core_link="Eigen3::Eigen;tesseract::tesseract_common;tesseract::tesseract_scene_graph;console_bridge::console_bridge"

  mkdir -p "$config_dir"

  cat >"$config_file" <<EOF
# Generated by fix_and_build.sh to provide a minimal CMake package for
# tesseract_state_solver when the upstream build does not install one.
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(tesseract_scene_graph)
find_dependency(tesseract_common)
find_dependency(console_bridge)
find_dependency(orocos_kdl)

if(NOT TARGET tesseract_state_solver_core)
  add_library(tesseract_state_solver_core INTERFACE IMPORTED)
  set_target_properties(tesseract_state_solver_core PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "${core_link}")
endif()
if(NOT TARGET tesseract::tesseract_state_solver_core)
  add_library(tesseract::tesseract_state_solver_core ALIAS tesseract_state_solver_core)
endif()

if("${kdl_lib}" AND NOT TARGET tesseract_state_solver_kdl)
  add_library(tesseract_state_solver_kdl SHARED IMPORTED)
  set_target_properties(tesseract_state_solver_kdl PROPERTIES
    IMPORTED_LOCATION "${kdl_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_state_solver_core;orocos-kdl")
endif()
if(TARGET tesseract_state_solver_kdl AND NOT TARGET tesseract::tesseract_state_solver_kdl)
  add_library(tesseract::tesseract_state_solver_kdl ALIAS tesseract_state_solver_kdl)
endif()

if("${ofkt_lib}" AND NOT TARGET tesseract_state_solver_ofkt)
  add_library(tesseract_state_solver_ofkt SHARED IMPORTED)
  set_target_properties(tesseract_state_solver_ofkt PROPERTIES
    IMPORTED_LOCATION "${ofkt_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    INTERFACE_LINK_LIBRARIES "tesseract::tesseract_state_solver_core")
endif()
if(TARGET tesseract_state_solver_ofkt AND NOT TARGET tesseract::tesseract_state_solver_ofkt)
  add_library(tesseract::tesseract_state_solver_ofkt ALIAS tesseract_state_solver_ofkt)
endif()

set(tesseract_state_solver_FOUND TRUE)
EOF

  local package_xml="$package_prefix/share/tesseract_state_solver/package.xml"
  if [[ ! -f "$package_xml" && -f "$REPO_DIR/tesseract/tesseract_state_solver/package.xml" ]]; then
    package_xml="$REPO_DIR/tesseract/tesseract_state_solver/package.xml"
  fi

  local version="0.0.0"
  if [[ -f "$package_xml" ]]; then
    version="$(PACKAGE_XML="$package_xml" python3 - <<'PY'
import os
import xml.etree.ElementTree as ET

path = os.environ["PACKAGE_XML"]
try:
    root = ET.parse(path).getroot()
except Exception:
    print("0.0.0")
else:
    text = root.findtext("version")
    print(text.strip() if text else "0.0.0")
PY
)"
    version="${version:-0.0.0}"
  fi

  local version_file="$config_dir/tesseract_state_solverConfigVersion.cmake"
  local version_script
  version_script=$(mktemp)
  cat >"$version_script" <<'EOF'
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${OUTPUT_FILE}" VERSION "${PACKAGE_VERSION}" COMPATIBILITY SameMajorVersion)
EOF
  cmake -DPACKAGE_VERSION="$version" -DOUTPUT_FILE="$version_file" -P "$version_script"
  rm -f "$version_script"

  echo "Synthesized missing tesseract_state_solver package configuration at $config_dir"
}

# Build boost_plugin_loader first
colcon build "${COLCON_BASE_ARGS[@]}" --packages-select boost_plugin_loader --cmake-args "${CMAKE_ARGS[@]}"
source_install
find install -name 'tesseract_commonConfig.cmake' || true

# Build up to tesseract_common and tesseract_msgs
colcon build "${COLCON_BASE_ARGS[@]}" --packages-up-to tesseract_common tesseract_msgs --cmake-args "${CMAKE_ARGS[@]}"
source_install
find install -name 'tesseract_commonConfig.cmake'

# Build and expose tesseract_state_solver early so downstream packages like
# tesseract_kinematics can reliably locate its exported CMake package. Build
# its full dependency set instead of only the package itself to guarantee the
# package.sh environment hooks exist when colcon tries to source them.
colcon build "${COLCON_BASE_ARGS[@]}" --packages-up-to tesseract_state_solver --cmake-args "${CMAKE_ARGS[@]}"
source_install
ensure_tesseract_state_solver_config

# Build up to trajopt_sco
colcon build "${COLCON_BASE_ARGS[@]}" --packages-up-to trajopt_sco --cmake-args "${CMAKE_ARGS[@]}"
source_install
ensure_tesseract_collision_core_config
ensure_tesseract_collision_bullet_config
ensure_tesseract_state_solver_config
find install -name 'tesseract_commonConfig.cmake'

# Build the whole workspace so every package is compiled at least once. Allow
# tesseract_collision to rebuild and regenerate the synthesized configs
# immediately afterwards to keep downstream packages satisfied.
COLCON_BUILD_ARGS=("${COLCON_BASE_ARGS[@]}" --cmake-args "${CMAKE_ARGS[@]}")
echo "Starting final full colcon build (this can sit on tesseract_motion_planners for a while; that's normal)"
colcon build "${COLCON_BUILD_ARGS[@]}"
source_install
ensure_tesseract_collision_core_config
ensure_tesseract_collision_bullet_config
ensure_tesseract_state_solver_config
find install -name 'tesseract_commonConfig.cmake'

# Print overlay information
colcon list --paths-only | sort
printf '%s\n' ${CMAKE_PREFIX_PATH//:/\n} | head -n 20

echo "Build completed successfully"
