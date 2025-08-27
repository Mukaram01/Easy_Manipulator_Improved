# Easy Manipulation Deployment 

<img src="./images/emd_logo.png"  width="30%" height="30%">
<br>

[![Documentation Status](https://readthedocs.org/projects/easy-manipulation-deployment-docs/badge/?version=latest)](https://easy-manipulation-deployment-docs.readthedocs.io/en/latest/?badge=latest)
[![License](https://img.shields.io/github/license/ros-industrial/easy_manipulation_deployment.svg)](https://github.com/ros-industrial/easy_manipulation_deployment/blob/master/LICENSE)

### This ROS 2 package provides a modular, easy-to-deploy manipulation pipeline that integrates perception elements to enable an end-to-end pick-and-place task
<br>

<img src="./images/grasp_planner.gif"  width="20%" height="20%"> <img src="./images/grasp_execution.gif"  width="20%" height="20%">

This package was tested with the [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment) ROS 2 package, but any perception system publishing the same ROS 2 message on the expected topic can work with this package as well.

It is recommended to run this package on **ROS 2 Jazzy** (Ubuntu 24.04) or **ROS 2 Humble** (Ubuntu 22.04).
For installation guidance, see the ROS 2 [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) or [Humble](https://docs.ros.org/en/humble/Installation.html) documentation.

---
## Prerequisites

- [vcstool](https://github.com/dirk-thomas/vcstool) for fetching dependency repositories
- `rosdep` for installing package dependencies
- A ROS 2 environment sourced (Jazzy or Humble)

---
## Build on Jazzy or Humble

```
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
vcs import < easy_manipulation_deployment/tesseract.repos
mv easy_manipulation_deployment/assets/ .
mv easy_manipulation_deployment/scenes/ .
mv easy_manipulation_deployment/easy_manipulation_deployment/workcell_builder ./easy_manipulation_deployment
cd ~/workcell_ws
export ROS_DISTRO=humble  # or jazzy
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"
source ~/ws_moveit2/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

A convenience script `fix_and_build.sh` is provided in this repository. It automatically
detects whether Humble or Jazzy is installed. After cloning into `~/workcell_ws`, run:

```
cd easy_manipulation_deployment
./fix_and_build.sh
```

This script ensures required tools such as `ament_cmake` are installed via `rosdep` before building.

---
## Full Documentation/Wiki

[Check out the Full Documentation here](https://easy-manipulation-deployment-docs.readthedocs.io/)

[Check out the API Documentation here](https://tanjpg.github.io/emd_docs/html/index.html)

---
## Components

### 1) Grasp Planner

An algorithmic based grasp planner that plans grasps in 3D space. Highly configurable and currently supports multifinger parallel grippers and suction cup arrays.

Two Finger Gripper

<img src="./images/two_finger.png"  width="10%" height="10%"> 

Three Finger Gripper

<img src="./images/three_finger.png"  width="10%" height="10%"> 

Single Suction Cup

<img src="./images/single_suction.png"  width="10%" height="10%"> 

2x2 Suction Array

<img src="./images/2x2_array.png"  width="10%" height="10%">

Quick start:

```
colcon build --symlink-install
source install/setup.bash
ros2 launch run_grasp_planner grasp_planner_3f_launch.py
```

### 2) Grasp Execution

A Moveit2 Based Grasp Execution package that incorporates real time dynamic safety components

Quick start:

```
colcon build --symlink-install
source install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py
```

### 3) Workcell Builder

A GUI based solution for ease of generation of robotic workcell simulations

---
## Running demo scenes

After building the workspace with `colcon build`, source the generated setup file and
launch one of the included demo scenes.  For example, to launch the UR5 with the
three finger gripper:

```
colcon build --symlink-install
source install/setup.bash
ros2 launch ur5_3f_test demo.launch.py
```

The grasp execution example can be started in a similar manner:

```
ros2 launch run_grasp_execution grasp_execution.launch.py
```

To launch both the grasp planner and execution together, use the provided demo script:

```
./scripts/grasp_demo.sh
```

---
## Acknowledgements

We would like to acknowledge the Singapore government for their vision and support to start this ambitious research and development project, "Accelerating Open Source Technologies for Cross Domain Adoption through the Robot Operating System". The project is supported by Singapore National Robotics Programme (NRP).

Any opinions, findings and conclusions or recommendations expressed in this material are those of the author(s) and do not reflect the views of the NR2PO.

