# Easy Manipulation Deployment

<img src="./images/emd_logo.png" width="30%" height="30%">

[![Documentation Status](https://readthedocs.org/projects/easy-manipulation-deployment-docs/badge/?version=latest)](https://easy-manipulation-deployment-docs.readthedocs.io/en/latest/?badge=latest)
[![License](https://img.shields.io/github/license/ros-industrial/easy_manipulation_deployment.svg)](https://github.com/ros-industrial/easy_manipulation_deployment/blob/master/LICENSE)

A ROS 2 package providing a modular, easy-to-deploy manipulation pipeline that integrates perception elements to enable end-to-end pick-and-place tasks.

<img src="./images/grasp_planner.gif" width="20%" height="20%"> <img src="./images/grasp_execution.gif" width="20%" height="20%">

This package was tested with [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment), but any perception system publishing compatible ROS 2 messages will work.

---

## Supported Platforms

| Platform | ROS 2 | Status |
|----------|-------|--------|
| Ubuntu 22.04 | Humble | ✅ Tested |
| Ubuntu 24.04 | Jazzy | 🔧 Experimental |

---

## Quick Installation (Ubuntu 22.04 + ROS 2 Humble)

### Prerequisites

- ROS 2 Humble installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- [vcstool](https://github.com/dirk-thomas/vcstool): `sudo apt install python3-vcstool`
- colcon: `sudo apt install python3-colcon-common-extensions`

### Step-by-Step Installation

```bash
# 1. Create workspace and clone repository
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment

mv easy_manipulation_deployment/assets/ .
mv easy_manipulation_deployment/scenes/ .

# 2. Import dependencies
vcs import < easy_manipulation_deployment/tesseract.repos

# 3. Setup environment
cd ~/workcell_ws
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# 4. Install system dependencies
~/workcell_ws/src/easy_manipulation_deployment/scripts/install_system_deps.sh
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

# 5. Apply Ubuntu 22.04 fixes
# Fix Boost serialization header bug
sudo sed -i '/#include <boost\/serialization\/item_version_type.hpp>/a #include <boost/serialization/library_version_type.hpp>' \
    /usr/include/boost/serialization/unordered_collections_load_imp.hpp

# Fix cereal CMake path
sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
sudo ln -sf /usr/share/cmake/cereal /usr/lib/x86_64-linux-gnu/cmake/cereal

# 6. Skip incompatible packages
touch ~/workcell_ws/src/tesseract_qt/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_planning/tesseract_examples/COLCON_IGNORE

# 7. Remove incompatible trajopt_ifopt planner
rm -rf ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/
sed -i 's/add_subdirectory(trajopt_ifopt)/#add_subdirectory(trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
sed -i 's/list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/#list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/' \
    ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt

# 8. Build (use parallel-workers 2 to reduce memory usage)
colcon build --symlink-install --parallel-workers 2

# 9. Source the workspace
source install/setup.bash
```

### Alternative: Use Helper Script

After cloning, you can use the automated build script:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build.sh
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

### 2. Grasp Execution

MoveIt2-based grasp execution with real-time dynamic safety components.

**Quick Start:**
```bash
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py
```

### 3. Workcell Builder

GUI-based tool for generating robotic workcell simulations.

**Quick Start:**
```bash
source ~/workcell_ws/install/setup.bash
workcell_builder
```

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
