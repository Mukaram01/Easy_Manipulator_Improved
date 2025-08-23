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

# --- Enforce C++17 across all packages ---
say "Enforcing C++17 in all packages ..."
find "$SRC" -name CMakeLists.txt -print0 | while IFS= read -r -d '' f; do
  # Bump any existing standard to 17
  sed -i 's/\(CMAKE_CXX_STANDARD *\)\([0-9][0-9]*\)/\117/g' "$f" || true
  # Insert standard block after project() if not present
  if ! grep -q "CMAKE_CXX_STANDARD" "$f"; then
    awk '
      BEGIN{done=0}
      /^project\(/ && !done {
        print $0 RS "if(NOT CMAKE_CXX_STANDARD)\n  set(CMAKE_CXX_STANDARD 17)\nendif()\nset(CMAKE_CXX_STANDARD_REQUIRED ON)\nset(CMAKE_CXX_EXTENSIONS OFF)\n"
        done=1; next
      }
      {print}
    ' "$f" > "$f.tmp" && mv "$f.tmp" "$f"
  fi

done

# --- Patch known missing <mutex> in tesseract_command_language ---
PDH="$SRC/tesseract_planning/tesseract_command_language/include/tesseract_command_language/profile_dictionary.h"
if [ -f "$PDH" ]; then
  if ! grep -q '^#include <mutex>' "$PDH"; then
    say "Patching <mutex> into profile_dictionary.h ..."
    sed -i '/#include <shared_mutex>/a #include <mutex>' "$PDH" || true
  fi
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
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_CXX_STANDARD=17 || {
    say "Minimal build failed — showing last 200 lines of log:"
    tail -n 200 "$WS/log/latest_build/*/stdout_stderr.log" 2>/dev/null || true
    exit 2
  }

# --- Full workspace build ---
say "Minimal set built. Building full workspace ..."
colcon build --symlink-install --cmake-args -DCMAKE_CXX_STANDARD=17 || {
  say "Full build failed — showing last 200 lines of log:"
  tail -n 200 "$WS/log/latest_build/*/stdout_stderr.log" 2>/dev/null || true
  exit 3
}

say "✅ Build completed successfully."
