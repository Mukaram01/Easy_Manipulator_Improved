#!/usr/bin/env bash
set -euo pipefail

WS=${WS:-$HOME/workcell_ws}
SRC="$WS/src"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$SRC"

# Ensure we are being run from the repository inside the workspace
if [[ "$REPO_DIR" != "$SRC/easy_manipulation_deployment" ]]; then
  echo "Please run this script from ~/workcell_ws/src/easy_manipulation_deployment" >&2
  exit 1
fi

# Select ROS distribution: prefer Humble then Jazzy
for d in humble jazzy; do
  if [[ -f "/opt/ros/$d/setup.bash" ]]; then
    ROS_DISTRO=${ROS_DISTRO:-$d}
    break
  fi
done

if [[ -z ${ROS_DISTRO:-} ]]; then
  echo "No supported ROS distro found (need humble or jazzy)" >&2
  exit 1
fi

# shellcheck source=/dev/null
# Disable nounset while sourcing ROS setup, as it references
# environment variables that may not be defined.
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
export LANG=C.UTF-8 LC_ALL=C.UTF-8

# Ensure required tools
APT_GET="sudo apt-get"
command -v sudo >/dev/null 2>&1 || APT_GET="apt-get"

for cmd in git colcon rosdep vcs; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Installing missing tool: $cmd"
    $APT_GET update -y
    case "$cmd" in
      git) $APT_GET install -y git;;
      colcon) python3 -m pip install -U colcon-common-extensions;;
      rosdep) $APT_GET install -y python3-rosdep && rosdep init || true;;
      vcs) $APT_GET install -y python3-vcstool;;
    esac
  fi
done

# Import external repositories if tesseract not yet present
if [[ -f "$REPO_DIR/tesseract.repos" ]] && [[ ! -d "$SRC/tesseract" ]]; then
  vcs import --recursive "$SRC" < "$REPO_DIR/tesseract.repos"
fi

# Copy overlays from repo checkout if they exist
for overlay in tesseract trajopt; do
  if [[ -d "$REPO_DIR/$overlay" ]]; then
    mkdir -p "$SRC/$overlay"
    cp -a "$REPO_DIR/$overlay/." "$SRC/$overlay/"
  fi
done

# Ensure workspace layout is sanitized (handles stray trajopt sources)
WS="$WS" "$REPO_DIR/scripts/fix_workspace_layout.sh"

# Ensure boost_plugin_loader exists
if [[ ! -d "$SRC/boost_plugin_loader" ]]; then
  git clone https://github.com/tesseract-robotics/boost_plugin_loader.git "$SRC/boost_plugin_loader"
fi

# Reveal any hidden Tesseract packages
find "$SRC" -path "$SRC/tesseract*" \( -name COLCON_IGNORE -o -name AMENT_IGNORE \) -print | while read -r f; do
  echo "Renaming ignore marker $f"
  mv "$f" "$f.bak"
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

# Install dependencies
rosdep update
rosdep install --from-paths "$SRC" --ignore-src -yr --rosdistro "$ROS_DISTRO" \
  --skip-keys "tesseract tesseract_process_planners trajopt_ifopt trajopt_sqp"

# Enforce C++17
export AMENT_CMAKE_CXX_STANDARD=17
CMAKE_ARGS=( -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF )

# Minimal include fix for profile_dictionary
# Use find's -print -quit to grab the first match and avoid non-zero exit when not found
PDH=$(find "$SRC" -path "*/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h" -print -quit 2>/dev/null)
if [[ -f "$PDH" ]] && grep -q '<shared_mutex>' "$PDH" && ! grep -q '<mutex>' "$PDH"; then
  echo "Patching missing <mutex> include in $PDH"
  sed -i '/<shared_mutex>/a #include <mutex>' "$PDH"
fi

cd "$WS"

# Build boost_plugin_loader first
colcon build --symlink-install --packages-select boost_plugin_loader --cmake-args "${CMAKE_ARGS[@]}"
source install/setup.bash
find install -name 'tesseract_commonConfig.cmake' || true

# Build up to tesseract_common and tesseract_msgs
colcon build --symlink-install --packages-up-to tesseract_common tesseract_msgs --cmake-args "${CMAKE_ARGS[@]}"
source install/setup.bash
find install -name 'tesseract_commonConfig.cmake'

# Build up to trajopt_sco
colcon build --symlink-install --packages-up-to trajopt_sco --cmake-args "${CMAKE_ARGS[@]}"
source install/setup.bash
find install -name 'tesseract_commonConfig.cmake'

# Build the whole workspace
colcon build --symlink-install --cmake-args "${CMAKE_ARGS[@]}"
source install/setup.bash
find install -name 'tesseract_commonConfig.cmake'

# Print overlay information
colcon list --paths-only | sort
printf '%s\n' ${CMAKE_PREFIX_PATH//:/\n} | head -n 20

echo "Build completed successfully"
