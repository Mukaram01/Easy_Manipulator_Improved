#!/usr/bin/env bash
set -euo pipefail

WS=~/workcell_ws
SRC="$WS/src"
LOG="$WS/fix_build.log"
touch "$LOG"

say() { echo -e "[FIX] $*" | tee -a "$LOG"; }
have() { command -v "$1" >/dev/null 2>&1; }

# --- Detect capabilities ---
SUDO_OK=0
APT_OK=0
INET_OK=0

if have sudo && sudo -n true 2>/dev/null; then SUDO_OK=1; fi
if have apt-get; then APT_OK=1; fi
if ping -c1 -W2 8.8.8.8 >/dev/null 2>&1 || curl -I https://github.com >/dev/null 2>&1; then INET_OK=1; fi

say "Capabilities: sudo=$SUDO_OK apt=$APT_OK internet=$INET_OK"

# Ensure we start from a clean ROS environment
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX CFLAGS CXXFLAGS

# --- Ensure ROS Humble environment ---
if [ ! -f /opt/ros/humble/setup.bash ]; then
  say "ROS Humble not found at /opt/ros/humble."
  if [ "$SUDO_OK" = "1" ] && [ "$APT_OK" = "1" ] && [ "$INET_OK" = "1" ]; then
    say "Installing ROS Humble (desktop) ..."
    sudo apt-get update -y
    sudo apt-get install -y curl gnupg lsb-release
    sudo bash -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
    sudo apt-get update -y
    sudo apt-get install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions
  else
    say "Cannot install ROS Humble automatically. Please install it, then rerun this script."
    exit 1
  fi
fi

# Source Humble
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export LANG=C.UTF-8 LC_ALL=C.UTF-8
say "ROS_DISTRO=$ROS_DISTRO (expect 'humble')"

# Force C++17 for all builds
export AMENT_CMAKE_CXX_STANDARD=17
export CXXFLAGS="-std=gnu++17"

# --- Ensure rosdep/colcon available ---
if ! have rosdep && [ "$SUDO_OK" = "1" ] && [ "$APT_OK" = "1" ] && [ "$INET_OK" = "1" ]; then
  say "Installing rosdep & colcon ..."
  sudo apt-get install -y python3-rosdep python3-colcon-common-extensions
fi
if have rosdep; then
  say "Initializing rosdep (idempotent) ..."
  sudo rosdep init 2>/dev/null || true
  rosdep update || true
else
  say "rosdep not available; will skip dependency installation."
fi

# --- Ensure boost_plugin_loader present (apt or vendor) ---
BPL_HEADER="/opt/ros/humble/include/boost_plugin_loader/fwd.h"
BPL_PKG="$SRC/boost_plugin_loader"

if [ -f "$BPL_HEADER" ]; then
  say "boost_plugin_loader found in /opt/ros/humble."
elif [ -d "$BPL_PKG" ]; then
  say "boost_plugin_loader vendored in workspace."
elif [ "$SUDO_OK" = "1" ] && [ "$APT_OK" = "1" ] && [ "$INET_OK" = "1" ]; then
  say "Attempting apt install of ros-humble-boost-plugin-loader ..."
  sudo apt-get update -y || true
  sudo apt-get install -y ros-humble-boost-plugin-loader || true
  if [ ! -f "$BPL_HEADER" ]; then
    say "Apt did not provide headers; vendoring from source ..."
    mkdir -p "$SRC"
    git clone https://github.com/ros-industrial/boost_plugin_loader.git "$BPL_PKG"
  fi
elif [ "$INET_OK" = "1" ]; then
  say "Vendoring boost_plugin_loader (no sudo) ..."
  mkdir -p "$SRC"
  git clone https://github.com/ros-industrial/boost_plugin_loader.git "$BPL_PKG" || {
    say "Failed to clone boost_plugin_loader (no internet or blocked)."
  }
else
  say "No way to obtain boost_plugin_loader automatically (no apt, no internet)."
fi

# --- Patch known missing <mutex> in tesseract_command_language ---
PDH="$SRC/tesseract_planning/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h"
if [ -f "$PDH" ]; then
  if ! grep -q '^#include <mutex>' "$PDH"; then
    say "Patching <mutex> into profile_dictionary.h ..."
    sed -i '/#include <shared_mutex>/a #include <mutex>' "$PDH" || true
  fi
fi

# --- Patch trajopt_sco to depend on rclcpp ---
TSC_CMAKE="$SRC/trajopt/trajopt_sco/CMakeLists.txt"
TSC_PKGXML="$SRC/trajopt/trajopt_sco/package.xml"

# Some versions of trajopt_sco do not declare a dependency on rclcpp even
# though headers from rclcpp are included via ros_node_base.h.  When that
# happens the include path to rclcpp is missing and compilation fails with
# "fatal error: rclcpp/rclcpp.hpp: No such file or directory".  The block
# below patches both the CMakeLists.txt and package.xml in-place to make the
# dependency explicit.
if [ -f "$TSC_CMAKE" ] && ! grep -q "find_package(rclcpp" "$TSC_CMAKE"; then
  say "Patching trajopt_sco CMakeLists.txt for rclcpp ..."
  # Insert the find_package call after the OpenMP find_package if present
  perl -0 -i -pe 's/find_package\s*\(\s*OpenMP[^\n]*\)\n/&find_package(rclcpp REQUIRED)\n/' "$TSC_CMAKE" || true
  # Ensure the target links against rclcpp so that its include directories
  # are propagated to the build
  perl -0 -i -pe 's/target_link_libraries\s*\(\s*\$\{PROJECT_NAME\}[^\n]*\n)/$&  rclcpp::rclcpp\n/' "$TSC_CMAKE" || true
fi

if [ -f "$TSC_PKGXML" ] && ! grep -q '<depend>rclcpp</depend>' "$TSC_PKGXML"; then
  say "Adding rclcpp dependency to trajopt_sco package.xml ..."
  perl -0 -i -pe 's/(<depend>[^<]+<\/depend>)/\1\n  <depend>rclcpp<\/depend>/' "$TSC_PKGXML" || true
fi

# --- Try rosdep to install remaining system deps ---
if have rosdep; then
  say "Running rosdep install ..."
  rosdep install --from-paths "$SRC" --ignore-src -yr --rosdistro "${ROS_DISTRO:-humble}" || true
fi

# --- Clean & build minimal set with verbose compile lines ---
say "Cleaning build/install/log ..."
rm -rf "$WS/build" "$WS/install" "$WS/log" || true
mkdir -p "$WS"
cd "$WS"

say "Building minimal set: tesseract_common tesseract_msgs ..."
colcon build --symlink-install \
  --packages-select tesseract_common tesseract_msgs \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -DCMAKE_CXX_EXTENSIONS=OFF \
    -DAMENT_CMAKE_CXX_STANDARD=17 || {
      say "Minimal build failed — showing last 200 lines of log:"
      tail -n 200 "$WS/log/latest_build/*/stdout_stderr.log" 2>/dev/null || true
      exit 2
    }

# --- Verify C++ standard used ---
COMPILE_JSON="$WS/build/tesseract_msgs/compile_commands.json"
if [ -f "$COMPILE_JSON" ] && grep -q -- "-std=gnu++11" "$COMPILE_JSON"; then
  say "Detected gnu++11 compilation; attempting to fix ..."
  grep -RIn -- "-std=gnu++11" "$WS/build" || true
  grep -RIn --include=CMakeLists.txt "CMAKE_CXX_STANDARD" "$SRC" || true
  grep -RIl --include=CMakeLists.txt "CMAKE_CXX_STANDARD *1[014]" "$SRC" | \
    xargs -r sed -i 's/CMAKE_CXX_STANDARD *[0-9][0-9]*/CMAKE_CXX_STANDARD 17/g'
  say "Rebuilding minimal set after fixes ..."
  rm -rf "$WS/build" "$WS/install" "$WS/log" || true
  colcon build --symlink-install \
    --packages-select tesseract_common tesseract_msgs \
    --event-handlers console_direct+ \
    --cmake-args \
      -DCMAKE_VERBOSE_MAKEFILE=ON \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_CXX_STANDARD_REQUIRED=ON \
      -DCMAKE_CXX_EXTENSIONS=OFF \
      -DAMENT_CMAKE_CXX_STANDARD=17 || {
        say "Minimal rebuild failed — showing last 200 lines of log:"
        tail -n 200 "$WS/log/latest_build/*/stdout_stderr.log" 2>/dev/null || true
        exit 2
      }
  if [ -f "$COMPILE_JSON" ] && grep -q -- "-std=gnu++11" "$COMPILE_JSON"; then
    say "ERROR: tesseract_msgs still compiling with -std=gnu++11"
    grep -n -- "-std=gnu++11" "$COMPILE_JSON" | tee -a "$LOG"
    exit 2
  fi
fi

# --- Full workspace build ---
say "Minimal set built. Building full workspace ..."
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_CXX_STANDARD_REQUIRED=ON \
    -DCMAKE_CXX_EXTENSIONS=OFF \
    -DAMENT_CMAKE_CXX_STANDARD=17 || {
      say "Full build failed — showing last 200 lines of log:"
      tail -n 200 "$WS/log/latest_build/*/stdout_stderr.log" 2>/dev/null || true
      exit 3
    }

say "✅ Build completed successfully."
