#!/usr/bin/env bash
set -euo pipefail

# Workspace setup
WS=~/workcell_ws
SRC=$WS/src
mkdir -p "$SRC"

# 0) Environment bootstrap
# Support either Humble or Jazzy depending on what is available or requested
default_rosdistro=""
for d in jazzy humble; do
  if [ -f "/opt/ros/${d}/setup.bash" ]; then
    default_rosdistro=${d}
    break
  fi
done

ROS_DISTRO=${ROS_DISTRO:-${default_rosdistro}}
[ -n "${ROS_DISTRO:-}" ] || { echo "ROS 2 distro not found (expected jazzy or humble)"; exit 1; }
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] || { echo "ROS ${ROS_DISTRO} not found"; exit 1; }
set +u; : "${AMENT_TRACE_SETUP_FILES:=}"; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
export LANG=C.UTF-8 LC_ALL=C.UTF-8
case "${ROS_DISTRO}" in
  humble|jazzy) ;;
  *) echo "Wrong ROS distro: ${ROS_DISTRO}"; exit 1 ;;
esac

# Ensure required tools are available
for cmd in git colcon rosdep; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Required tool '$cmd' is not installed." >&2
    exit 1
  fi
done

# Install missing build tools like ament_cmake if absent
if [ ! -f "/opt/ros/${ROS_DISTRO}/share/ament_cmake/package.xml" ]; then
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y "ros-${ROS_DISTRO}-ament-cmake"
  else
    apt-get update -y
    apt-get install -y "ros-${ROS_DISTRO}-ament-cmake"
  fi
fi

# Install missing dependencies
rosdep update

# Remove any duplicate ROS packages within the workspace before resolving
# dependencies to avoid rosdep and colcon errors. Keep the first occurrence of
# a package name and discard subsequent duplicates.
declare -A pkg_seen
while read -r name path _; do
  if [[ -n "${pkg_seen[$name]:-}" ]]; then
    echo "Removing duplicate package '$name' from $path (keeping ${pkg_seen[$name]})"
    rm -rf "$path"
  else
    pkg_seen[$name]="$path"
  fi
done < <(colcon list --base-paths "$SRC")

rosdep install --from-paths "$SRC" --ignore-src -yr --rosdistro "${ROS_DISTRO}" \
  --skip-keys "tesseract tesseract_process_planners"

# C++17 everywhere
export AMENT_CMAKE_CXX_STANDARD=17
CMAKE_STD_ARGS=(-DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF)

# 1) Ensure boost_plugin_loader and build it first
# Ensure the boost_plugin_loader from tesseract-robotics is available
if [ ! -d "$SRC/boost_plugin_loader" ]; then
  git clone https://github.com/tesseract-robotics/boost_plugin_loader.git "$SRC/boost_plugin_loader"
fi
cd "$WS"
colcon build --symlink-install --packages-select boost_plugin_loader

# 2) Patch Tesseract's missing <mutex> include
PDH="$SRC/tesseract_planning/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h"
if [ -f "$PDH" ] && ! grep -qE '^\s*#\s*include\s*<mutex>' "$PDH"; then
  sed -i '/#include <shared_mutex>/a #include <mutex>' "$PDH"
fi

# 3) Enforce C++17 blocks in any CMakeLists that lack one
find "$SRC" -name CMakeLists.txt -print0 | while IFS= read -r -d '' f; do
  sed -i 's/\(CMAKE_CXX_STANDARD *\)[0-9][0-9]*/\117/g' "$f" || true
  if ! grep -q 'CMAKE_CXX_STANDARD' "$f"; then
    awk 'BEGIN{done=0}/^project\(/ && !done {print $0 RS "if(NOT CMAKE_CXX_STANDARD)\n  set(CMAKE_CXX_STANDARD 17)\nendif()\nset(CMAKE_CXX_STANDARD_REQUIRED ON)\nset(CMAKE_CXX_EXTENSIONS OFF)\n"; done=1; next} {print}' "$f" > "$f.tmp" && mv "$f.tmp" "$f"
  fi
 done

# 4) Fix trajopt_sco to depend on rclcpp
CMAKEL="$SRC/trajopt/trajopt_sco/CMakeLists.txt"
PKGXML="$SRC/trajopt/trajopt_sco/package.xml"

if [ -f "$CMAKEL" ]; then
  grep -q 'find_package( *rclcpp' "$CMAKEL" || sed -i '/project(/a find_package(rclcpp REQUIRED)' "$CMAKEL"

  if grep -q 'ament_target_dependencies( *trajopt_sco' "$CMAKEL"; then
    sed -i '0,/ament_target_dependencies( *trajopt_sco/s//ament_target_dependencies(trajopt_sco\n  rclcpp\n/' "$CMAKEL"
  fi

  if ! grep -q 'rclcpp::rclcpp' "$CMAKEL"; then
    cat >> "$CMAKEL" <<'EOC'

# Ensure rclcpp include/link for trajopt_sco targets (if not already added)
get_property(_targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
foreach(t IN LISTS _targets)
  if(NOT t MATCHES "^gtest_|^gmock_")
    get_target_property(_type ${t} TYPE)
    if(_type STREQUAL "EXECUTABLE" OR _type STREQUAL "SHARED_LIBRARY" OR _type STREQUAL "STATIC_LIBRARY")
      target_link_libraries(${t} PUBLIC rclcpp::rclcpp)
    endif()
  endif()
endforeach()
EOC
  fi
fi

if [ -f "$PKGXML" ] && ! grep -q '<depend>rclcpp</depend>' "$PKGXML"; then
  sed -i 's@</package>@  <depend>rclcpp</depend>\n</package>@' "$PKGXML"
fi

# 5) Satisfy jsoncpp system dependency (optional)
if command -v sudo >/dev/null 2>&1; then
  sudo apt-get update -y || true
  sudo apt-get install -y libjsoncpp-dev || true
fi

# 6) Build in sequence
cd "$WS"
colcon build --symlink-install --packages-select \
  ros_industrial_cmake_boilerplate eigen boost_plugin_loader

colcon build --symlink-install --packages-up-to tesseract_common tesseract_msgs \
  --cmake-args "${CMAKE_STD_ARGS[@]}"

colcon build --symlink-install --packages-up-to trajopt_sco \
  --cmake-args "${CMAKE_STD_ARGS[@]}"

colcon build --symlink-install --cmake-args "${CMAKE_STD_ARGS[@]}"
