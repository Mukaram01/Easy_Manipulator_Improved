# Easy Manipulation Deployment 

<img src="./images/emd_logo.png"  width="30%" height="30%">
<br>

[![Documentation Status](https://readthedocs.org/projects/easy-manipulation-deployment-docs/badge/?version=latest)](https://easy-manipulation-deployment-docs.readthedocs.io/en/latest/?badge=latest)
[![License](https://img.shields.io/github/license/ros-industrial/easy_manipulation_deployment.svg)](https://github.com/ros-industrial/easy_manipulation_deployment/blob/master/LICENSE)

This ROS 2 package provides a modular, easy-to-deploy manipulation pipeline that integrates perception elements to enable an end-to-end pick-and-place task.
<br>

<img src="./images/grasp_planner.gif"  width="20%" height="20%"> <img src="./images/grasp_execution.gif"  width="20%" height="20%">

This package was tested with the [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment) ROS 2 package, but any perception system publishing the same ROS 2 message on the expected topic can work with this package as well.

It is recommended to run this package on **ROS 2 Jazzy** (Ubuntu 24.04) or **ROS 2 Humble** (Ubuntu 22.04).
For installation guidance, see the ROS 2 [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) or [Humble](https://docs.ros.org/en/humble/Installation.html) documentation.

> **Important:** Run `./easy_manipulation_deployment/scripts/install_system_deps.sh`
> immediately after cloning (before your first `colcon build`). This installs the
> Boost graph/program_options/serialization headers that `trajopt_common`
> requires and the `cereal` headers needed by `tesseract_motion_planners`, so you
> do not hit configure errors such as
> `Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)` or
> `Could not find a package configuration file provided by "cereal"` on minimal
> environments.

---
## Prerequisites

- [vcstool](https://github.com/dirk-thomas/vcstool) for fetching dependency repositories
- `rosdep` for installing package dependencies
- `colcon` build tool
- A ROS 2 environment sourced (Jazzy or Humble)
- The bundled Tesseract overlays now export TinyXML2 via the `tinyxml2_vendor`
  package, so no extra system packages are required to satisfy
  `find_package(TinyXML2)` calls during the build.
- The `cereal` headers are required by `tesseract_motion_planners`; the helper
  script installs the `libcereal-dev` package to provide them.
- The TrajOpt OSQP solver backend is required by
  `tesseract_motion_planners/trajopt`; install the `libosqp-dev` package so
  `find_package(osqp)` can locate the CMake config without additional setup.
- If you plan to build manually (without the helper scripts below), run
  `./easy_manipulation_deployment/scripts/install_system_deps.sh` after cloning
  to install the TinyXML2 and Boost development packages that
  `trajopt_common` requires.

Tested platform: **Ubuntu 22.04 with ROS 2 Humble**. Install the system packages
below before running `colcon build` to avoid repeated fetch/fallback loops in
`tesseract_motion_planners` and related dependencies:

```
sudo apt update && sudo apt install -y libcereal-dev libjsoncpp-dev libomp-dev libosqp-dev ros-humble-roslib
```

---
## Build on Jazzy or Humble

```
## Installation (ROS2 Humble on Ubuntu 22.04)

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
vcs import < easy_manipulation_deployment/tesseract.repos

cd ~/workcell_ws
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install system dependencies
~/workcell_ws/src/easy_manipulation_deployment/scripts/install_system_deps.sh
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"

# Fix Boost serialization header bug (Ubuntu 22.04)
sudo sed -i '/#include <boost\/serialization\/item_version_type.hpp>/a #include <boost/serialization/library_version_type.hpp>' /usr/include/boost/serialization/unordered_collections_load_imp.hpp

# Fix cereal CMake path
sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
sudo ln -sf /usr/share/cmake/cereal /usr/lib/x86_64-linux-gnu/cmake/cereal

# Skip incompatible packages
touch ~/workcell_ws/src/tesseract_qt/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE
touch ~/workcell_ws/src/tesseract_planning/tesseract_examples/COLCON_IGNORE

# Remove incompatible trajopt_ifopt planner
rm -rf ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/trajopt_ifopt/
sed -i 's/add_subdirectory(trajopt_ifopt)/#add_subdirectory(trajopt_ifopt)/' ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt
sed -i 's/list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/#list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)/' ~/workcell_ws/src/tesseract_planning/tesseract_motion_planners/CMakeLists.txt

# Build
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

The helper scripts register `scripts/rosdep_overrides.yaml` automatically so
rosdep can resolve the Humble-specific keys for `cereal`, `openmp`, `roslib`,
and the various Tesseract packages without downloading sources during build
configuration. They also install the OSQP development package and export
`OSQP_DIR` when its CMake config is detected so `find_package(osqp)` resolves
cleanly during the TrajOpt planner build.

> **Tip:** Run `./easy_manipulation_deployment/scripts/fix_workspace_layout.sh`
> after importing the repositories and before calling `rosdep install`. The
> script relocates the bundled Tesseract and TrajOpt overlays so only one copy of
> each package is visible. This prevents `rosdep` from aborting with duplicate
> package errors such as `tesseract_common` or `trajopt_sco` while still keeping
> the patched overlays in use.

The `tesseract.repos` file now also fetches the
[`boost_plugin_loader`](https://github.com/tesseract-robotics/boost_plugin_loader)
dependency, which provides headers such as `boost_plugin_loader/fwd.h`
required by `tesseract_common`.

The workspace overlay also includes the upstream
[`tesseract_plugins`](https://github.com/tesseract-robotics/tesseract_plugins)
repository, so ensure you re-run `vcs import < tesseract.repos` to pull the
plugin definitions before building.

The helper scripts check out the bundled `trajopt` repository on its `main`
branch when a detached HEAD is detected, keeping repeated builds aligned to a
stable upstream reference instead of a transient commit hash.

### Troubleshooting missing `tesseract_motion_planners_coreConfig.cmake`

If CMake reports

```
Could not find a package configuration file provided by "tesseract_motion_planners_core" with any of the following names:

  tesseract_motion_planners_coreConfig.cmake
  tesseract_motion_planners_core-config.cmake
```

the bundled Tesseract and TrajOpt overlays may be hidden or mis-symlinked after
importing the repositories. Re-run `./scripts/fix_workspace_layout.sh` from the
repository root to restore the intended overlay layout, rebuild the planner
package, and retry the composer so the core package exports its config file into
the workspace `CMAKE_PREFIX_PATH`:

```
./scripts/fix_workspace_layout.sh
colcon build --packages-select tesseract_motion_planners
colcon test --packages-select tesseract_task_composer
```

Re-sourcing the workspace after that rebuild ensures `find_package` can locate
`tesseract_motion_planners_core` on subsequent builds.

A convenience script `fix_and_build.sh` is provided in this repository. It automatically
detects whether Humble or Jazzy is installed and installs the Boost/TinyXML2 development
packages via `scripts/install_system_deps.sh`. After cloning into `~/workcell_ws`, run:

```
cd easy_manipulation_deployment
./fix_and_build.sh
```

This helper also runs `scripts/fix_workspace_layout.sh` internally, so it repairs
the overlay symlinks and dependency declarations required for
`tesseract_motion_planners_core` to be discoverable before invoking `colcon`.
The final `colcon build` stage can take several minutes when compiling
`tesseract_motion_planners`; seeing repeated "Processing" lines for that package
is expected while it finishes.
Set `SKIP_TESTS=1` to disable building Tesseract tests and examples, or
`LIGHTWEIGHT_PLANNERS=1` to skip heavyweight planners (OMPL, Descartes, Trajopt,
Trajopt IFOPT). These toggles append the corresponding CMake flags to every
`colcon build` invocation within the script.
This script fixes a common issue where a `COLCON_IGNORE` file in the bundled
Tesseract sources hid packages such as `tesseract_common`, causing
`colcon build --packages-up-to tesseract_common` to fail.  The script renames
any ignore markers under `src/tesseract*`, ensures `boost_plugin_loader` is
present, installs dependencies with `rosdep`, and builds the workspace in a
safe order.  It is idempotent and can be re-run at any time.

### Troubleshooting Boost-related configure errors

If `colcon build` fails in `trajopt_common` with an error such as:

```
Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)
```

install the missing Boost development packages before re-running the build:

```
sudo apt-get update && sudo apt-get install -y \
  libboost-dev libboost-graph-dev libboost-program-options-dev libboost-serialization-dev
```

Alternatively, re-run `./fix_and_build.sh`, which automatically installs these
packages when they are absent. If you previously attempted a build without
them, clean any cached configuration before retrying:

```
rm -rf build/trajopt_common install/trajopt_common log/trajopt_common
```

When building outside the helper scripts (for example, directly invoking
`colcon build` in an existing workspace), manually run
`./easy_manipulation_deployment/scripts/install_system_deps.sh` first and
remove any cached `trajopt_common` build or install folders. This ensures the
required Boost graph/program_options/serialization headers are present before
the configure step that otherwise emits the `Could NOT find Boost (missing:
Boost_INCLUDE_DIR graph)` error.

If CMake instead reports a missing `stacktrace_backtrace` component while
configuring `tesseract_command_language`:

```
Could NOT find Boost (missing: Boost_INCLUDE_DIR stacktrace_backtrace)
```

install the Boost stacktrace development package and retry the build:

```
sudo apt-get update && sudo apt-get install -y libboost-stacktrace-dev
```

Running `./fix_and_build.sh` or `scripts/install_system_deps.sh` also installs
this dependency automatically. If the error persists after installing the
package, clear any cached build artifacts for `tesseract_command_language`
before retrying so CMake re-detects the newly available stacktrace component:

```
rm -rf build/tesseract_command_language install/tesseract_command_language \
  log/tesseract_command_language
```

If the build fails in `tesseract_motion_planners` with an error like:

```
By not providing "Findcereal.cmake" in CMAKE_MODULE_PATH this project has asked CMake to find a package configuration file provided by "cereal", but CMake did not find one.
```

install the Cereal headers (or rerun `./easy_manipulation_deployment/scripts/install_system_deps.sh`, which installs them automatically) and clear any cached artifacts for the package before retrying:

```
sudo apt-get update && sudo apt-get install -y libcereal-dev
rm -rf build/tesseract_motion_planners install/tesseract_motion_planners log/tesseract_motion_planners
```

Similarly, if a configure step fails with a TinyXML2 error such as:

```
Could not find a package configuration file provided by "tinyxml2" with any
of the following names:

  tinyxml2Config.cmake
  tinyxml2-config.cmake
```

install the TinyXML2 development package (or rerun
`./easy_manipulation_deployment/scripts/install_system_deps.sh`, which handles
this automatically) and retry the build after clearing any cached artifacts for
the affected package:

```
sudo apt-get update && sudo apt-get install -y libtinyxml2-dev
rm -rf build/tesseract_command_language install/tesseract_command_language \
  log/tesseract_command_language
```

Run `scripts/fix_workspace_layout.sh` once to disable duplicate third-party
packages (Tesseract and TrajOpt) and set up the necessary symlinks. BPMPD
support is optional and can be toggled with `-DHAVE_BPMPD=ON/OFF` when building.

---
## Full Documentation/Wiki

[Check out the Full Documentation here](https://easy-manipulation-deployment-docs.readthedocs.io/)

[Check out the API Documentation here](https://tanjpg.github.io/emd_docs/html/index.html)

---
## Components

### 1) Grasp Planner

An algorithm-based grasp planner that plans grasps in 3D space. It is highly configurable and currently supports multifinger parallel grippers and suction cup arrays.

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

Before running your first `colcon build` (here or in any other section), make
sure `./easy_manipulation_deployment/scripts/install_system_deps.sh` has been
executed after cloning. It installs the Boost graph/program_options/
serialization headers that `trajopt_common` checks for during configure.

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

