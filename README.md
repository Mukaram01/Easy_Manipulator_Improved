# Easy Manipulation Deployment

<img src="./images/emd_logo.png" width="30%" height="30%">

[![Documentation Status](https://readthedocs.org/projects/easy-manipulation-deployment-docs/badge/?version=latest)](https://easy-manipulation-deployment-docs.readthedocs.io/en/latest/?badge=latest)
[![Humble CI](https://github.com/Mukaram01/Easy_Manipulator_Improved/actions/workflows/humble-ci.yml/badge.svg)](https://github.com/Mukaram01/Easy_Manipulator_Improved/actions/workflows/humble-ci.yml)
[![License](https://img.shields.io/github/license/ros-industrial/easy_manipulation_deployment.svg)](https://github.com/ros-industrial/easy_manipulation_deployment/blob/master/LICENSE)

A ROS 2 package providing a modular, easy-to-deploy manipulation pipeline that integrates perception elements to enable end-to-end pick-and-place tasks.

<img src="./images/grasp_planner.gif" width="20%" height="20%"> <img src="./images/grasp_execution.gif" width="20%" height="20%">

This package was tested with [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment), but any perception system publishing compatible ROS 2 messages will work.

---

## Supported Platforms

| Platform | ROS 2 | Status |
|----------|-------|--------|
| Ubuntu 22.04 | Humble | ✅ Tested (see Version Notes) |
| Ubuntu 24.04 | Jazzy | 🔧 Experimental (see Version Notes) |

## Version Notes

- **Humble** is the tested, CI-validated target.
- **Jazzy** is experimental and does not have CI coverage unless added.

---

## CI verified on Humble

CI runs on Ubuntu 22.04 with ROS 2 Humble.

To run the same checks locally:

```bash
rosdep install --from-paths src --ignore-src -yr --rosdistro humble
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Some GUI-related packages are optional in CI. You can skip them locally with
`--packages-skip` (e.g., `colcon build --packages-skip <pkg1> <pkg2>`) or by
adding `COLCON_IGNORE` files in the package directories.

---

## Quick Installation (Ubuntu 22.04 + ROS 2 Humble)

### Prerequisites

- ROS 2 Humble installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- CI uses Ubuntu 22.04 + ROS 2 Humble via `ros-tooling/setup-ros`

### Recommended bootstrap (single command)

The canonical entrypoint is the Humble bootstrap script. It installs system
dependencies, applies the Cereal/Boost fixes, removes the incompatible
`trajopt_ifopt` planner, and builds the workspace end-to-end.

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src

# Clone the repo into the intended layout.
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment

cd easy_manipulation_deployment
./fix_and_build_humble.sh
```

### Manual Installation (advanced / troubleshooting)

#### 0) Install apt packages (matching CI)

```bash
sudo apt-get update
sudo apt-get install -y \
  libpcap-dev \
  libpng-dev \
  python3-vcstool \
  python3-colcon-common-extensions \
  ros-humble-moveit \
  ros-humble-moveit-visual-tools \
  ros-humble-xacro
```

#### 1) Recreate the workspace the “original” way

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src

# clone your repo INTO src as easy_manipulation_deployment (this is the intended layout)
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
```

#### 2) Put assets/scenes where the README expects them but if you dont want to move them see 2.a.

```bash
cd ~/workcell_ws/src
mv easy_manipulation_deployment/assets .
mv easy_manipulation_deployment/scenes .
```

#### 2.a) Create symlinks where the build expects them

```bash
ln -s ~/workcell_ws/src/easy_manipulation_deployment/assets ~/workcell_ws/src/assets
ln -s ~/workcell_ws/src/easy_manipulation_deployment/scenes ~/workcell_ws/src/scenes
```

**Asset sourcing note:** Workcell Builder first looks for assets inside the workcell's
`src/assets` directory (robots, end effectors, environment). If those subfolders are empty,
it copies the packaged defaults from `share/workcell_builder/assets` so new workcells still
start with the bundled assets when running from an installed build.

#### 3) Import dependencies into `~/workcell_ws/src`

```bash
cd ~/workcell_ws/src
vcs import < easy_manipulation_deployment/tesseract.repos
```

#### 4) Apply the Humble “known fixes / skips” (matches README notes)

> **Note:** If you ran `./fix_and_build_humble.sh`, these fixes have already
> been applied and you can skip this section.

```bash
cd ~/workcell_ws
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install deps script if you have it
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

#### 5) Now rosdep + build

```bash
cd ~/workcell_ws
rosdep update
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

rm -rf build install log
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

#### After this, your repo commands are back to normal

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
git pull
```

### Helper Script (preferred)

If you already have the repository cloned, you can run the canonical Humble
bootstrap script directly:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh
```

---

## Components

### 1. Grasp Planner

An algorithm-based grasp planner for 3D space. Supports multifinger parallel grippers and suction cup arrays.

| Two Finger | Three Finger | Single Suction | 2x2 Suction Array |
|:----------:|:------------:|:--------------:|:-----------------:|
| <img src="./images/two_finger.png" width="80"> | <img src="./images/three_finger.png" width="80"> | <img src="./images/single_suction.png" width="80"> | <img src="./images/2x2_array.png" width="80"> |

**Quick Start:**
```bash
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_planner grasp_planner_3f_launch.py
```

**EPD timeout parameter:**
The grasp planner can monitor Easy Perception Deployment (EPD) message activity. Configure the
timeout in `easy_manipulation_deployment/emd_demo_nodes/run_grasp_planner/config/*.yaml` via
`easy_perception_deployment.epd_msg_timeout_s` to warn and re-trigger the EPD pipeline when
messages stall.

### 2. Grasp Execution

MoveIt2-based grasp execution with real-time dynamic safety components.

**Quick Start:**
```bash
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py
```

### 3. Workcell Builder

GUI-based tool for generating robotic workcell simulations. The supported workflow targets
**ROS 2 Humble** (ament + MoveIt 2).

**Quick Start:**
```bash
source ~/workcell_ws/install/setup.bash
workcell_builder
```
 
Example ROS 2 (Humble) launch templates live under `workcell_builder/examples/ros2/`.

**Required external robot descriptions (ROS 2 Humble):**
* `fanuc`: `moveit_resources_fanuc_description` (or `fanuc_description` if you have it installed).
* `panda_robot`: `moveit_resources_panda_description`.

---

## Running Demo Scenes

```bash
source ~/workcell_ws/install/setup.bash

# UR5 with three-finger gripper
ros2 launch ur5_3f_test demo.launch.py

# UR5 with two-finger gripper
ros2 launch ur5_2f_test demo.launch.py

# Custom scene (if you created one with workcell_builder)
ros2 launch new_scene demo.launch.py
```

---

## Troubleshooting

### Optional manual patches

The following steps are optional and only needed if you hit the corresponding build errors.

<details>
<summary><b>Skip incompatible packages</b></summary>

```bash
touch ~/workcell_ws/src/tesseract_qt/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_planning/tesseract_examples/COLCON_IGNORE
```
</details>

> **Caution:** The next step removes the `trajopt_ifopt` planner, which disables the `trajopt_ifopt` and related IFOPT-based planners. If you need to revert, use `git checkout` to restore the folder and `CMakeLists.txt` changes, or re-import the package from its source.

<details>
<summary><b>Remove incompatible trajopt_ifopt planner</b></summary>

```bash
rm -rf ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/
sed -i 's/add_subdirectory(trajopt_ifopt)/#add_subdirectory(trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
sed -i 's/list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/#list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
```
</details>

### Common Build Errors

<details>
<summary><b>Boost missing components</b></summary>

```
Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)
```

**Fix:**
```bash
sudo apt install -y libboost-dev libboost-graph-dev libboost-program-options-dev \
    libboost-serialization-dev libboost-stacktrace-dev
rm -rf build/ install/ log/
colcon build --symlink-install --parallel-workers 2
```
</details>

<details>
<summary><b>Cereal not found</b></summary>

```
Could not find a package configuration file provided by "cereal"
```

**Fix:**
```bash
sudo apt install -y libcereal-dev
sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
sudo ln -sf /usr/share/cmake/cereal /usr/lib/x86_64-linux-gnu/cmake/cereal
```
</details>

<details>
<summary><b>Boost serialization library_version_type error</b></summary>

```
'library_version_type' is not a member of 'boost::serialization'
```

**Fix:**
```bash
sudo sed -i '/#include <boost\/serialization\/item_version_type.hpp>/a #include <boost/serialization/library_version_type.hpp>' \
    /usr/include/boost/serialization/unordered_collections_load_imp.hpp
```
</details>

<details>
<summary><b>Duplicate package errors from rosdep</b></summary>

```
Multiple packages found with the same name "tesseract_common"
```

**Fix:** Remove duplicate packages or run `./scripts/fix_workspace_layout.sh`
</details>

---

## Checks

Run the ros2_control joint state fix verification:

```bash
./scripts/check_ros2_control_joint_state_fix.sh
```

---

## Documentation

- 📖 [Full Documentation](https://easy-manipulation-deployment-docs.readthedocs.io/)
- 📚 [API Documentation](https://tanjpg.github.io/emd_docs/html/index.html)

---

## Architecture

This package uses the following external dependencies (fetched via `tesseract.repos`):

| Package | Description |
|---------|-------------|
| [tesseract](https://github.com/tesseract-robotics/tesseract) | Motion planning framework |
| [tesseract_planning](https://github.com/tesseract-robotics/tesseract_planning) | Planning algorithms |
| [trajopt](https://github.com/tesseract-robotics/trajopt) | Trajectory optimization |
| [tesseract_ros2](https://github.com/tesseract-robotics/tesseract_ros2) | ROS 2 integration |
| [boost_plugin_loader](https://github.com/tesseract-robotics/boost_plugin_loader) | Plugin system |

---

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
