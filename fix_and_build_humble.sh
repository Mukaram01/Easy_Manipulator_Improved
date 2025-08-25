#!/usr/bin/env bash
set -euo pipefail
cd ~/workcell_ws

# 0) Source underlays
source /opt/ros/humble/setup.bash
if [ -f ~/ws_moveit2/install/setup.bash ]; then source ~/ws_moveit2/install/setup.bash; fi

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

rosdep update
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}" \
  --skip-keys "tesseract tesseract_process_planners"

# 1) Ensure boost_plugin_loader exists
if [ ! -d src/boost_plugin_loader ]; then
  git -C src clone https://github.com/tesseract-robotics/boost_plugin_loader.git
fi

# 2) Clean fully (start from scratch)
rm -rf build install log
find src -name build -o -name install -o -name log | xargs -r rm -rf

# 3) Stage 1: infrastructure vendors first
colcon build --symlink-install --packages-select \
  ros_industrial_cmake_boilerplate eigen boost_plugin_loader

# 4) Stage 2: up to tesseract_common/tesseract_msgs with C++17
colcon build --symlink-install --packages-up-to tesseract_common tesseract_msgs \
  --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF

# 5) Stage 3: full workspace
colcon build --symlink-install \
  --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF
