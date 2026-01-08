# Contributing Guide

Thanks for contributing to Easy Manipulation Deployment! This guide captures the
supported ROS 2 distributions, expected workspace layout, setup steps, and the
build/test flow used in CI and helper scripts.

## Supported ROS 2 distros

- **Humble (Ubuntu 22.04)**: **required** and CI-validated.
- **Jazzy (Ubuntu 24.04)**: experimental (no CI coverage).

## Workspace layout

Use a ROS 2 workspace that mirrors the layout in the README and helper scripts:

```
~/workcell_ws/
  src/
    easy_manipulation_deployment/   # this repo
    assets/                         # moved or symlinked from this repo
    scenes/                         # moved or symlinked from this repo
```

Clone this repository into `~/workcell_ws/src` using the name
`easy_manipulation_deployment`. Assets and scenes can be moved or symlinked to
`~/workcell_ws/src` as shown below.

## Local setup (mirrors README quick install)

The commands below mirror the README quick install flow and the helper scripts in
`scripts/`.

### 0) Install apt prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-vcstool \
  python3-colcon-common-extensions \
  ros-humble-moveit \
  ros-humble-moveit-visual-tools \
  ros-humble-xacro
```

### 1) Create workspace + clone

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src

git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
```

### 2) Assets and scenes placement

Either move the directories into `~/workcell_ws/src`:

```bash
cd ~/workcell_ws/src
mv easy_manipulation_deployment/assets .
mv easy_manipulation_deployment/scenes .
```

Or create symlinks:

```bash
ln -s ~/workcell_ws/src/easy_manipulation_deployment/assets ~/workcell_ws/src/assets
ln -s ~/workcell_ws/src/easy_manipulation_deployment/scenes ~/workcell_ws/src/scenes
```

### 3) Import dependencies

```bash
cd ~/workcell_ws/src
vcs import < easy_manipulation_deployment/tesseract.repos
```

### 4) Apply Humble fixes/skips (from README + scripts)

```bash
cd ~/workcell_ws
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install system deps
~/workcell_ws/src/easy_manipulation_deployment/scripts/install_system_deps.sh

# Fix cereal CMake path (common on Jammy)
sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
sudo ln -sf /usr/share/cmake/cereal /usr/lib/x86_64-linux-gnu/cmake/cereal

# Skip packages that cause distro mismatches
touch ~/workcell_ws/src/tesseract_qt/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_planning/tesseract_examples/COLCON_IGNORE

# If you see trajopt_ifopt build errors, remove the incompatible planner
rm -rf ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/
sed -i 's/add_subdirectory(trajopt_ifopt)/#add_subdirectory(trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
sed -i 's/list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/#list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
```

### 5) Build workspace

```bash
cd ~/workcell_ws
rosdep update
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

rm -rf build install log
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### Optional: helper script

You can also run the helper script that automates the setup/build steps:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build.sh
```

## Build and test

From the workspace root (`~/workcell_ws`):

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

### Packages to skip (if needed)

Some GUI-related packages are optional or mismatch the distro. Use
`--packages-skip` or `COLCON_IGNORE` files when needed:

- `tesseract_qt`
- `tesseract_ros2/tesseract_rviz`
- `tesseract_ros2/tesseract_ros_examples`
- `tesseract_ros2/tesseract_planning_server`
- `tesseract_planning/tesseract_examples`

Example:

```bash
colcon build --packages-skip tesseract_qt tesseract_rviz tesseract_ros_examples
```

## Code style

This repository follows ROS 2/ament conventions. When contributing:

- Keep formatting aligned with existing files.
- Run the same build/tests described above.
- If you touch `ros2_control` related logic, you can run the check script:

```bash
./scripts/check_ros2_control_joint_state_fix.sh
```

If additional lint/format commands are added to `scripts/` in the future, prefer
using those over ad-hoc formatting to stay consistent with CI expectations.
