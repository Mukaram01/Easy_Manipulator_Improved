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

## [0.1.0] - 2026-01-08
### Added
- Baseline repository with Easy Manipulator assets, deployment tooling, and documentation.

