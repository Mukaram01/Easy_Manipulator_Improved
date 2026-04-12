# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **Thread-safety in `MoveitReplannerContext::run()`** — the planning mutex is
  now released before the long `solve()` call by working on a lightweight
  `PlanningScene::diff()` snapshot.  This allows concurrent `update()` calls to
  keep the parent scene current while planning is in progress
  (`emd_dynamic_safety/src/moveit/replanner_moveit.cpp`).
- **Multi-axis joint handling in `MoveitReplannerContext::run()`** — replaced
  the broken single-element `{value}` initializer with a per-variable position
  index that correctly handles joints whose `getVariableCount() > 1`
  (`emd_dynamic_safety/src/moveit/replanner_moveit.cpp`).
- **Multi-axis joint handling in `MoveitReplannerContext::update()`** — use
  `setVariablePosition` (per-variable update matching JointState message layout)
  instead of `setJointPositions` with a single-element list
  (`emd_dynamic_safety/src/moveit/replanner_moveit.cpp`).
- **Multi-axis joint handling in `MoveitCppGraspExecution::move_to()`** — same
  per-variable fix; JointState names are variable names so `setVariablePosition`
  is the correct API
  (`emd_grasp_execution/src/moveit2/moveit_cpp_if.cpp`).
- **Trajectory ending condition in `NextPointPublisher`** — on completion the
  publisher now clamps to the exact final waypoint state and logs the event once,
  replacing the previous undefined behaviour
  (`emd_dynamic_safety/src/next_point_publisher.cpp`).
- **Stale TODO in `DefaultExecutor`** — removed the outdated "fix doesn't finish
  in time" TODO and documented that `TIMED_OUT` intentionally maps to success
  after `stopExecution()` has been called
  (`emd_grasp_execution/src/moveit2/executor/default_executor.cpp`).
- **`load_end_effectors()` missing error handling** — each end-effector is now
  loaded inside a try/catch for `ParameterNotDeclaredException` and
  `std::invalid_argument`; a misconfigured entry is logged and skipped instead of
  crashing the planning node.  Unknown end-effector types are also logged
  (`emd_grasp_planner/src/grasp_scene.cpp`).
- **Hardcoded `"base_link"` TF frame** — the robot base frame used by the TF
  message filter and world-collision transform lookups is now read from the
  `camera_parameters.robot_base_frame` ROS parameter (default `"base_link"` in
  all demo config files), making the planner portable across workcells with
  non-standard base frame names
  (`emd_grasp_planner/src/grasp_scene.cpp`,
  `emd_demo_nodes/run_grasp_planner/config/params*.yaml`).

### Added
- **Configurable release pose** in grasp execution demo — two new ROS parameters
  (`release_x_offset`, default `-0.3`; `release_use_grasp_z`, default `true`)
  replace the previously hardcoded values
  (`emd_demo_nodes/run_grasp_execution/src/demo_node.cpp` and
  `emd_demo_nodes/run_grasp_execution/config/grasp_execution.yaml`).
- **ROS 2 Jazzy CI workflow** (`.github/workflows/jazzy-ci.yml`) — mirrors the
  existing Humble CI on Ubuntu 24.04 + ROS 2 Jazzy.
- **Dependency caching in CI** — both the Humble and Jazzy workflows now cache
  the rosdep database and colcon build artefacts keyed on `package.xml` /
  `CMakeLists.txt` hashes, significantly reducing repeat-run build times.

### Changed
- **Named constant for depth scale factor** — replaced the `0.001F` magic
  number in the depth-to-metres conversion with
  `static constexpr float kDepthMmToMeters = 0.001F`
  (`emd_grasp_planner/src/epd_detection_adapter.cpp`).
- **C-style headers replaced** — `<stdlib.h>` and `<math.h>` replaced with
  `<cstdlib>` and `<cmath>` in `grasp_object.hpp` and `pcl_functions.hpp`.
- **Dead code removed** — removed the unused `get_camera_position()` declaration
  and definition, stale commented-out constructor assignments in `grasp_object.cpp`,
  the stale `// setup(topic_name);` constructor comment in `grasp_scene.hpp`, the
  empty `/*! \brief */` Doxygen comment in the EPD section, and the superseded
  commented-out install/export block in `emd_grasp_planner/CMakeLists.txt`.
- **`org_cloud` field documented** — replaced empty `/*! \brief  */` with a
  descriptive comment in `grasp_scene.hpp`.

## [0.1.0] - 2026-01-08
### Added
- Baseline repository with Easy Manipulator assets, deployment tooling, and documentation.

