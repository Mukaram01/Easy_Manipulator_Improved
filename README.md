# Easy Manipulation Deployment

Modular ROS 2 manipulation packages for grasp planning, grasp execution, and workcell deployment.

<img src="./images/emd_logo.png" width="30%" height="30%">

[![Documentation Status](https://readthedocs.org/projects/easy-manipulation-deployment-docs/badge/?version=latest)](https://easy-manipulation-deployment-docs.readthedocs.io/en/latest/?badge=latest)
[![Humble CI](https://github.com/Mukaram01/Easy_Manipulator_Improved/actions/workflows/humble-ci.yml/badge.svg)](https://github.com/Mukaram01/Easy_Manipulator_Improved/actions/workflows/humble-ci.yml)
[![License](https://img.shields.io/github/license/ros-industrial/easy_manipulation_deployment.svg)](https://github.com/ros-industrial/easy_manipulation_deployment/blob/master/LICENSE)

<img src="./images/grasp_planner.gif" width="20%" height="20%"> <img src="./images/grasp_execution.gif" width="20%" height="20%">

This package was tested with [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment), but any perception system publishing compatible ROS 2 messages will work.

## Contents

- [Supported platform](#supported-platform)
- [Quick start (recommended path)](#quick-start-recommended-path)
- [Step-by-step install](#step-by-step-install)
- [First run / verification](#first-run--verification)
- [Fake hardware vs real hardware](#fake-hardware-vs-real-hardware)
- [Common scenarios](#common-scenarios)
- [Troubleshooting](#troubleshooting)
- [Version notes / advanced compatibility](#version-notes--advanced-compatibility)

## Supported platform

- **Supported/tested:** Ubuntu 22.04 + ROS 2 Humble
- Other platforms and experimental combinations are listed later in [Version notes / advanced compatibility](#version-notes--advanced-compatibility)

## Quick start (recommended path)

If you want the shortest known-good flow on Ubuntu 22.04 + ROS 2 Humble:

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
mv easy_manipulation_deployment/assets/ .
mv easy_manipulation_deployment/scenes/ .
cd ~/workcell_ws

# One-time rosdep override registration
echo "yaml file://$HOME/workcell_ws/src/easy_manipulation_deployment/scripts/rosdep_overrides.yaml" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-easy-manipulator-overrides.list >/dev/null

# Repeat before each build in source-checkout workflows
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh

# Build
eval "$(./src/easy_manipulation_deployment/scripts/ensure_taskflow_cmake_package.sh --export)"
colcon build --parallel-workers 2
source install/setup.bash
```

Then verify:

```bash
ros2 pkg prefix emd_msgs
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
```

---

## Step-by-step install

Use this for the complete, supported, non-GUI Humble path.

1. Install system dependencies.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake curl git libpcap-dev libpng-dev \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-moveit ros-humble-moveit-visual-tools \
  ros-humble-ur-description ros-humble-xacro
```

2. Initialize `rosdep` if needed, then refresh it.

```bash
sudo rosdep init || true
rosdep update
```

3. Create the workspace and clone this repo into the expected path.

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
mv easy_manipulation_deployment/assets/ .
mv easy_manipulation_deployment/scenes/ .
```

4. Import the source dependencies.

```bash
cd ~/workcell_ws
vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos
```

5. Register the repo-local `rosdep` overrides.

```bash
echo "yaml file://$HOME/workcell_ws/src/easy_manipulation_deployment/scripts/rosdep_overrides.yaml" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-easy-manipulator-overrides.list >/dev/null
rosdep update
rosdep resolve cereal
```

6. Expose the repository asset packages into `src/` **before every `colcon build` when this repository is checked out as a workspace source tree**.

```bash
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
test -L src/workbench_description -o -d src/workbench_description
```

7. Run an APT preflight check before `rosdep` so you know whether binary Tesseract/QtADS providers are available.

```bash
./src/easy_manipulation_deployment/scripts/preflight_tesseract_apt.sh --ros-distro humble
```

- If the check passes, `apt-cache policy` can discover `ros-humble-tesseract-*` packages and QtADS-related providers.
- If the check reports missing packages, use the supported source-overlay fallback route (already included by `vcs import` from `dependencies/emd_epd_ws.repos`), and let the helper script inject additional rosdep `--skip-keys` automatically.

8. Install package dependencies from the workspace source tree.

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "taskflow osqp-eigen tesseract_environment tesseract_motion_planners tesseract_motion_planners_core tesseract_motion_planners_simple tesseract_task_composer trajopt trajopt_ifopt trajopt_sco trajopt_sqp"
```

For scripted workflows, `./scripts/fix_and_build.sh` now runs the same preflight automatically before its rosdep phase and appends fallback skip-keys (for example `qt_advanced_docking`, `tesseract_visualization`) when binary packages are unavailable.

9. Build and source the workspace. The layout helper is a required pre-build step for source checkouts because it exposes hidden asset packages from `assets/` into `src/`.

```bash
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
eval "$(./src/easy_manipulation_deployment/scripts/ensure_taskflow_cmake_package.sh --export)"
colcon build --parallel-workers 2
source install/setup.bash
```

## Straightforward manual setup (no repo helper scripts)

If you prefer not to use `fix_workspace_layout.sh` or other repository helper scripts, use the commands below.

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
mv easy_manipulation_deployment/assets/ .
mv easy_manipulation_deployment/scenes/ .
cd ~/workcell_ws
vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos

source /opt/ros/humble/setup.bash

echo "yaml file://$HOME/workcell_ws/src/easy_manipulation_deployment/scripts/rosdep_overrides.yaml" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-easy-manipulator-overrides.list >/dev/null
rosdep update

# Optional preflight visibility check (same logic used by scripted builds)
./src/easy_manipulation_deployment/scripts/preflight_tesseract_apt.sh --ros-distro humble

# Expose repo asset packages manually (equivalent to layout helper behavior).
# IMPORTANT: create links in src/ to actual package directories that contain package.xml.
ln -sfn easy_manipulation_deployment/assets/environment/table_description src/table_description
ln -sfn easy_manipulation_deployment/assets/environment/workbench_description src/workbench_description
ln -sfn easy_manipulation_deployment/assets/robots/universal_robot/ur5_moveit_config src/ur5_moveit_config
ln -sfn easy_manipulation_deployment/assets/end_effectors/robotiq_85_gripper/robotiq_85_moveit_config src/robotiq_85_moveit_config
ln -sfn easy_manipulation_deployment/assets/end_effectors/robotiq_85_gripper/robotiq_85_description src/robotiq_85_description
ln -sfn easy_manipulation_deployment/assets/end_effectors/single_suction_gripper/single_suction_moveit_config src/single_suction_moveit_config
ln -sfn easy_manipulation_deployment/assets/end_effectors/single_suction_gripper/single_suction_description src/single_suction_description

# Optional GUI packages are disabled in this simple/headless path
mkdir -p src/tesseract_qt src/qtadvanceddocking src/ruckig
touch src/tesseract_qt/COLCON_IGNORE src/qtadvanceddocking/COLCON_IGNORE src/ruckig/COLCON_IGNORE

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "tesseract_visualization taskflow osqp-eigen tesseract_environment tesseract_motion_planners tesseract_motion_planners_core tesseract_motion_planners_simple tesseract_task_composer trajopt trajopt_ifopt trajopt_sco trajopt_sqp"

colcon build --symlink-install --parallel-workers 2 \
  --packages-skip tesseract_qt QtADS tesseract_rviz tesseract_planning_server
source install/setup.bash
```

If scene/robot packages are still unresolved in an existing workspace, relink them to your actual checkout location. For example:

```bash
cd ~/workcell_ws
rm -f src/table_description src/workbench_description src/ur5_moveit_config
ln -s ~/workcell_ws/src/easy_manipulation_deployment/assets/environment/table_description src/table_description
ln -s ~/workcell_ws/src/easy_manipulation_deployment/assets/environment/workbench_description src/workbench_description
ln -s ~/workcell_ws/src/easy_manipulation_deployment/assets/robots/universal_robot/ur5_moveit_config src/ur5_moveit_config
test -e src/table_description && test -e src/workbench_description && test -e src/ur5_moveit_config && echo OK_scene_assets
```

## Build profiles (simple)

- **Headless / simplest (recommended):**

  ```bash
  cd ~/workcell_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --parallel-workers 2 \
    --packages-skip tesseract_qt QtADS tesseract_rviz tesseract_planning_server
  ```

- **GUI / Studio enabled (advanced):**
  - Remove `COLCON_IGNORE` from `src/tesseract_qt` and `src/qtadvanceddocking`.
  - Install GUI dependencies (Qt + ADS).
  - Build without skipping `tesseract_qt` / `QtADS` / `tesseract_rviz`.


## Skip list for missing Tesseract Qt packages (simple)

From your build log, the failure is caused by `tesseract_rviz` requiring `tesseract_qt`.

Use this minimal skip list:

- `tesseract_rviz` (required if `tesseract_qt` is not installed)

Optional defensive skips (only if those packages are present in your workspace and you want to force a headless build):

- `tesseract_qt`
- `QtADS` (sometimes appears as checkout folder `qtadvanceddocking`)

### 1) Easiest one-liner (`--packages-skip`)

```bash
colcon build --symlink-install --packages-skip tesseract_rviz
```

Headless defensive variant:

```bash
colcon build --symlink-install --packages-skip tesseract_rviz tesseract_qt QtADS qtadvanceddocking
```

### 2) Persistent skip with `COLCON_IGNORE`

```bash
touch src/tesseract_rviz/COLCON_IGNORE
colcon build --symlink-install
```

Headless defensive variant:

```bash
touch src/tesseract_rviz/COLCON_IGNORE src/tesseract_qt/COLCON_IGNORE src/qtadvanceddocking/COLCON_IGNORE
colcon build --symlink-install
```

### 3) Remove the persistent ignore later

```bash
rm -f src/tesseract_rviz/COLCON_IGNORE src/tesseract_qt/COLCON_IGNORE src/qtadvanceddocking/COLCON_IGNORE
```

## First run / verification

Source the workspace in each new shell:

```bash
source ~/workcell_ws/install/setup.bash
```

Quick verification options:

```bash
ros2 pkg prefix emd_msgs
```

```bash
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
ros2 launch suction_test demo.launch.py
```

Run the validator before `suction_test` so missing `single_suction_description` / `single_suction_moveit_config` packages are reported with explicit remediation via `fix_workspace_layout.sh`.

If you only want to confirm the workspace built successfully, `source install/setup.bash` plus a successful `ros2 pkg prefix emd_msgs` is the safest quick check.

## Fake hardware vs real hardware

All scene launch files default to **simulated (fake) hardware** so you can develop, test, and visualise motions without a physical robot.

| Mode | `use_fake_hardware` | Hardware interface used |
|------|-------------------|------------------------|
| Fake / simulated (default) | `true` | `fake_components/GenericSystem` via ros2_control |
| Real robot | `false` | Physical driver (e.g. `ur_robot_driver`) |

### Default: fake hardware (no robot required)

```bash
# All scene launch files use use_fake_hardware:=true by default.
# The argument can be omitted – both lines below are equivalent:
ros2 launch ur5_2f_test demo.launch.py
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true
```

The same applies to every other scene package:

```bash
ros2 launch ur3_suction_test  demo.launch.py
ros2 launch ur5_3f_test       demo.launch.py
ros2 launch ur5_airpick4_test demo.launch.py
ros2 launch ur10_2f_test      demo.launch.py
ros2 launch suction_test      demo.launch.py
```

### Launch **any** scene package (including Workcell Builder generated scenes)

If a scene package contains `launch/demo.launch.py`, you can launch it with the same pattern:

```bash
ros2 launch <scene_package> demo.launch.py [use_fake_hardware:=true|false]
```

Examples:

```bash
# Existing scene package from this repo (fake hardware)
ros2 launch ur5_3f_test demo.launch.py use_fake_hardware:=true

# Existing scene package on a real robot
ros2 launch ur5_3f_test demo.launch.py use_fake_hardware:=false

# New scene generated with Workcell Builder
ros2 launch my_generated_scene demo.launch.py use_fake_hardware:=true
ros2 launch my_generated_scene demo.launch.py use_fake_hardware:=false
```

Tip: you can verify a scene package exists before launch:

```bash
ros2 pkg prefix <scene_package>
```

### Switching to a real UR robot

1. **Install and start `ur_robot_driver`** (ships separately from this repository):

   ```bash
   sudo apt install -y ros-humble-ur
   ```

2. **Launch the UR driver** with your robot's IP address *before* starting the scene launch:

   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py \
     ur_type:=ur5 \
     robot_ip:=192.168.1.102
   ```

   Replace `ur5` with your robot model (`ur3`, `ur10`, …) and `192.168.1.102` with the actual IP shown on the teach pendant.

3. **Launch the scene** with `use_fake_hardware:=false`:

   ```bash
   ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=false
   ```

   The scene URDF skips the simulated `fake_components/GenericSystem` block when this flag is `false`, so ros2_control picks up the real hardware interface provided by the driver.

> **Safety reminder:** always verify E-stop and speed limits on the teach pendant before enabling real-hardware mode.

## Common scenarios

### Build with GUI / Studio tools

The default supported path is headless/non-GUI. Only opt in if you need Tesseract Studio / Qt widgets.

Install the extra Qt/GUI system packages first:

```bash
sudo apt install -y \
  qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5svg5-dev
```

For the default manual/headless path, keep the GUI packages disabled so a plain `colcon build` does not try to compile `tesseract_qt` or Qt ADS. The checkout folder is `qtadvanceddocking`, but colcon may discover that package as `QtADS` in the workspace.

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --symlink-install --parallel-workers 2
```

If you want an explicit non-GUI full-workspace example that matches the Humble helper-script defaults from the logs, skip the GUI packages by their discovered names (with the checkout folder as an extra fallback) or use the helper script without `--with-gui`:

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --symlink-install --parallel-workers 2 \
  --packages-skip tesseract_qt QtADS qtadvanceddocking
```

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full
```

`fix_workspace_layout.sh` creates `COLCON_IGNORE` markers for `src/tesseract_qt` and `src/qtadvanceddocking` unless you explicitly opt into GUI support. If you prefer to manage that manually, either leave those markers in place or pass `--packages-skip tesseract_qt QtADS` to `colcon build` (optionally also adding `qtadvanceddocking` as a compatibility fallback for folder-based troubleshooting notes). To opt into the GUI-enabled path instead, use `./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh --with-gui` for the manual flow or `./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full --with-gui` for the helper-script flow.

For an explicit GUI-enabled manual build, remove those markers by opting into GUI mode when syncing the workspace layout:

```bash
cd ~/workcell_ws
vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh --with-gui
colcon build --symlink-install --parallel-workers 2
```

If you use the helper script instead, the equivalent GUI-enabled path is:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full --with-gui
```

`tesseract_qt` and the Qt ADS checkout (`qtadvanceddocking`, package name `QtADS`) remain optional in this repository. When skipping them manually with colcon, use the discovered package name `QtADS` and optionally also `qtadvanceddocking` as a compatibility fallback.

### Clean rebuild

```bash
cd ~/workcell_ws
rm -rf build install log
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --parallel-workers 2
```

Helper-script equivalent:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --clean
```

### Helper-script phase model (opt-in mutation)

`fix_and_build_humble.sh` now separates execution into explicit phases:

- `--check-prereqs`: read-only validation.
- `--install-prereqs`: mutating apt/pip installation.
- `--build`: workspace build only (`colcon build`).

The script also supports `--dry-run` and writes a machine-readable summary to
`<workspace>/fix_and_build_summary.json` (or prints JSON in dry-run mode).

**Local development (copy/paste):**

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh \
  --workspace ~/workcell_ws \
  --check-prereqs \
  --install-prereqs \
  --build \
  --profile full
```

**Locked-down enterprise host (no machine mutation):**

```bash
cd /opt/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh \
  --workspace /opt/workcell_ws \
  --check-prereqs \
  --dry-run
```

**CI runner (pre-provisioned image):**

```bash
./fix_and_build_humble.sh \
  --workspace "$GITHUB_WORKSPACE" \
  --check-prereqs \
  --build \
  --profile minimal \
  --clean
```

### Rebuild only one package

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --packages-select emd_grasp_planner --parallel-workers 2
```

For dependency-first troubleshooting of the planning stack:

```bash
cd ~/workcell_ws
colcon build --packages-select tesseract_geometry 2>&1 | tee /tmp/colcon_tesseract_geometry.log
colcon build --packages-select tesseract_collision 2>&1 | tee /tmp/colcon_tesseract_collision.log
rg -n 'IMPORTED_LOCATION not set for imported target "octomap"' /tmp/colcon_tesseract_geometry.log /tmp/colcon_tesseract_collision.log
```

### Dynamic safety tests

Dynamic safety is part of the grasp execution workflow.

```bash
source ~/workcell_ws/install/setup.bash
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_3f_test
```

If you are validating packages only, run the workspace tests after a build:

```bash
cd ~/workcell_ws
colcon test
colcon test-result --verbose
```

### Using `--symlink-install` if desired

The main supported quick-start path uses a plain build:

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --parallel-workers 2
```

If you are actively developing packages and want editable install behavior, use:

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --symlink-install --parallel-workers 2
```

Some helper scripts and source-overlay workflows still use `--symlink-install`, especially for full-profile development workspaces.


## Dependency manifests

- Canonical manifest: `dependencies/emd_epd_ws.repos`.
- Legacy compatibility manifest: `tesseract.repos` (root-level).
- Canonical import command:

```bash
cd ~/workcell_ws
vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos
```

Repository scripts now resolve manifests in this order: canonical first, then legacy with a warning so existing workspaces keep working during migration.

## Troubleshooting

### Old ROS-installed Ruckig headers vs source-built Ruckig on Humble

If the compiler picks `/opt/ros/humble/include/ruckig` but the linker picks a newer workspace-built `ruckig`, you can get linker errors mentioning older symbols such as `PositionStep1` or `VelocityStep2`.

Recommended recovery:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
cd ~/workcell_ws
rm -rf build install log
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --parallel-workers 2
```

Do **not** remove `ros-humble-ruckig` just to build this repository; that can remove unrelated MoveIt packages.

### Missing `trajoptConfig.cmake` during configure

`scripts/fix_workspace_layout.sh` does **not** generate synthetic TrajOpt package config files. The supported path is to make sure the real TrajOpt packages are discoverable in your workspace (or installed in an underlay), then rerun the layout helper and rebuild.

Recommended recovery:

```bash
cd ~/workcell_ws
vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
colcon build --packages-up-to trajopt_sco
```

If `trajoptConfig.cmake` is still missing, verify `src/trajopt`, `src/trajopt_common`, and `src/trajopt_sco` exist and that your active underlay/overlay environment is sourced correctly before running a full build.

### Optional GUI package issues

If `tesseract_qt` fails to link because the ADS target name differs on your system, apply the included patch and rebuild:

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/apply_upstream_patches.sh
colcon build --symlink-install --parallel-workers 2
```

Qt ADS (`qtadvanceddocking`) may also require Qt private headers on some systems. If the build cannot find `qpa/qplatformnativeinterface.h`, install the distro package that provides the Qt private development headers for your Qt version before rebuilding.

If you do not need Studio / Qt widgets, keep the default non-GUI path or temporarily skip the GUI-facing packages:

```bash
for marker in \
  ~/workcell_ws/src/tesseract_qt/COLCON_IGNORE \
  ~/workcell_ws/src/tesseract_ros2/tesseract_rviz/COLCON_IGNORE \
  ~/workcell_ws/src/tesseract_ros2/tesseract_ros_examples/COLCON_IGNORE \
  ~/workcell_ws/src/tesseract_ros2/tesseract_planning_server/COLCON_IGNORE \
  ~/workcell_ws/src/tesseract_planning/tesseract_examples/COLCON_IGNORE; do
  [ -d "$(dirname "$marker")" ] && touch "$marker"
done
```

### `emd_dynamic_safety` / MoveIt / Ruckig mismatch on Humble

On Humble, test-only or overlay builds can expose a mismatch between the MoveIt-provided Ruckig headers and a newer source-built Ruckig library. Start from a clean shell, source only `/opt/ros/humble/setup.bash`, and rebuild the workspace before rerunning dynamic safety or grasp execution tests.

If you need the full source-overlay workflow, prefer the repository helper so its preflight checks run before building:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full --clean
```

### Clean shell / clean workspace recovery

If the workspace starts failing after switching branches, changing overlays, or reusing old build outputs, reset to a clean state:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
cd ~/workcell_ws
rm -rf build install log
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "taskflow osqp-eigen tesseract_environment tesseract_motion_planners tesseract_motion_planners_core tesseract_motion_planners_simple tesseract_task_composer trajopt trajopt_ifopt trajopt_sco trajopt_sqp"
colcon build --parallel-workers 2
```

### `workbench_description` missing during `rosdep install`

If you see `Cannot locate rosdep definition for [workbench_description]`, the layout helper was not run before `rosdep install`.

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
test -L src/workbench_description -o -d src/workbench_description
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

### Duplicate package discovery errors

If `rosdep` or `colcon` reports duplicate packages in an older workspace, rerun the layout helper or remove the stale duplicate tree.

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
```

### Common dependency fixes

Boost missing components:

```bash
sudo apt install -y libboost-dev libboost-graph-dev libboost-program-options-dev \
  libboost-serialization-dev libboost-stacktrace-dev
cd ~/workcell_ws
rm -rf build install log
colcon build --symlink-install --parallel-workers 2
```

Cereal not found:

```bash
sudo apt install -y libcereal-dev
sudo mkdir -p /usr/lib/x86_64-linux-gnu/cmake/
sudo ln -sf /usr/share/cmake/cereal /usr/lib/x86_64-linux-gnu/cmake/cereal
```

Boost serialization `library_version_type` error:

```bash
sudo sed -i '/#include <boost\/serialization\/item_version_type.hpp>/a #include <boost/serialization/library_version_type.hpp>' \
  /usr/include/boost/serialization/unordered_collections_load_imp.hpp
```

## Version notes / advanced compatibility

- **Supported baseline:** Ubuntu 22.04 + ROS 2 Humble + a C++17/libstdc++ baseline.
- **Experimental:** Ubuntu 24.04 + ROS 2 Jazzy is not the main supported path for this repository.
- `scripts/lib/build.sh` intentionally keeps `-DCMAKE_CXX_STANDARD=17` for the supported baseline.
- `dependencies/emd_epd_ws.repos` (canonical manifest) pins `ruckig` to `v0.15.3` (`37b6e7a`) so source builds stay on the pre-`std::format` line that still works with the Humble/Jammy toolchain. The legacy root-level `tesseract.repos` is retained as a compatibility fallback in scripts.
- `fix_and_build_humble.sh` supports explicit `--check-prereqs`, `--install-prereqs`, and `--build` phases for deterministic workflows.
- If you move to a newer `ruckig` release that requires `std::format`, you are also moving beyond the documented Humble/Jammy support envelope.

### Full-profile and helper-script notes

The repository helper remains the canonical bootstrap for full planning/development workspaces:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
./fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full
```

Use it when you explicitly need the Tesseract/TrajOpt planning overlays from source, overlay-aware `rosdep` skip keys, or the repo's additional preflight checks.

### Optional GUI details

- `tesseract_qt` and Qt ADS are optional. The repository checkout is `qtadvanceddocking`, while colcon may discover the package as `QtADS`; use `--packages-skip tesseract_qt QtADS` (optionally also `qtadvanceddocking`) for manual skips.
- The default install path stays headless/non-GUI, and `scripts/fix_workspace_layout.sh` keeps those packages ignored unless you opt in.
- Use `scripts/fix_workspace_layout.sh --with-gui` for the manual GUI-enabled path.
- Use `fix_and_build_humble.sh --workspace ~/workcell_ws --check-prereqs --build --profile full --with-gui` for the helper-script GUI-enabled path.
- Install the extra Qt development packages before GUI builds.
- If Studio builds fail on Qt ADS targets, use `scripts/apply_upstream_patches.sh` and rebuild.
- If Qt ADS cannot find `qpa/qplatformnativeinterface.h`, install the Qt private-header development package that matches your distro Qt version.

### Workspace layout notes

This repository is intended to live inside a larger ROS 2 workspace, typically as:

```text
~/workcell_ws/src/easy_manipulation_deployment
```

The layout helper exposes repository asset packages into `src/` so `rosdep` and `colcon` can discover packages such as `workbench_description`, `ur5_moveit_config`, `robotiq_85_moveit_config`, and the robot description packages without manually moving directories. Run it before every `colcon build` in a source checkout workflow.

Recommended layout:

```text
~/workcell_ws/src/
└── easy_manipulation_deployment/
    ├── assets/
    ├── scenes/
    ├── CMakeLists.txt
    └── ...
```

Legacy compatibility symlinks can still exist in older workspaces, but the repository tree above is the source of truth. Stale or dangling top-level links such as `~/workcell_ws/assets` or `~/workcell_ws/scenes` can cause the GUI to fail before it falls back to `src/`.

If Workcell Builder exits with the exact pattern `Failed to inspect directory '<workspace>/assets': No such file or directory`, treat that as a workspace-layout issue instead of a missing package. Quick remediation checklist:

- inspect whether `<workspace>/assets` is a broken symlink,
- remove stale top-level links if the real layout now lives under `<workspace>/src`,
- rerun `./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh`,
- rebuild/source the workspace if required.

Use `./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh` as the supported verification step after cleanup.

For `suction_test` verification and troubleshooting specifically, always run the validator first. The validator now explicitly reports `single_suction_description` and `single_suction_moveit_config` in missing/index-fail output, then points to `fix_workspace_layout.sh` for remediation when they are hidden in `assets/`.

## Components

### 1. Grasp Planner

An algorithm-based grasp planner for 3D space. Supports multifinger parallel grippers and suction cup arrays.

| Two Finger | Three Finger | Single Suction | 2x2 Suction Array |
|:----------:|:------------:|:--------------:|:-----------------:|
| <img src="./images/two_finger.png" width="80"> | <img src="./images/three_finger.png" width="80"> | <img src="./images/single_suction.png" width="80"> | <img src="./images/2x2_array.png" width="80"> |

```bash
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_planner grasp_planner_3f_launch.py
```

The grasp planner can monitor Easy Perception Deployment (EPD) message activity. Configure the timeout in `easy_manipulation_deployment/emd_demo_nodes/run_grasp_planner/config/*.yaml` via `easy_perception_deployment.epd_msg_timeout_s`.

### 2. Grasp Execution

MoveIt2-based grasp execution with real-time dynamic safety components.

```bash
source ~/workcell_ws/install/setup.bash
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_3f_test
# or, after generating and rebuilding your own scene package (generated scene packages still depend on the exposed asset packages above):
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=my_generated_scene
```

### Fake hardware workflow with `grasp_execution.launch.py` and grasp planner launchers

If you want to run the full demo stack without a physical robot, start the scene in fake-hardware mode first, then launch the planner and execution nodes against the same scene package.

```bash
# Terminal 1: start the scene with fake ros2_control hardware
source ~/workcell_ws/install/setup.bash
ros2 launch ur5_3f_test demo.launch.py use_fake_hardware:=true

# Terminal 2: start the grasp planner
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_planner grasp_planner_3f_launch.py

# Terminal 3: start grasp execution for the same scene package
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_3f_test
```

Notes:
- `grasp_execution.launch.py` selects the robot/workcell through `scene_package:=...`; keep this value aligned with the scene you launched in Terminal 1.
- The fake-vs-real hardware switch is controlled by each scene launcher (`demo.launch.py`) via `use_fake_hardware:=true|false`.
- The same pattern works for generated scenes, for example `scene_package:=my_generated_scene`.

### 3. Workcell Builder

GUI-based tool for generating robotic workcell simulations on ROS 2 Humble.

```bash
source ~/workcell_ws/install/setup.bash
workcell_builder
```

Scene root resolution precedence:

1. `--scene-root <path>`
2. `WORKCELL_BUILDER_SCENE_ROOT`
3. `$PWD`
4. `$PWD/..`
5. `$PWD/src/easy_manipulation_deployment`
6. `$PWD/src`

For deterministic behavior in multi-workspace setups, explicitly select the repository scene root:

```bash
workcell_builder --scene-root ~/workcell_ws/src/easy_manipulation_deployment
```

If you keep more than one workspace on disk, prefer the explicit `--scene-root` launch above even after running `./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh`.

```bash
export WORKCELL_BUILDER_SCENE_ROOT=~/workcell_ws/src/easy_manipulation_deployment
workcell_builder
```

Required external robot descriptions on ROS 2 Humble:

- `fanuc`: `moveit_resources_fanuc_description` or `fanuc_description`
- `panda_robot`: `moveit_resources_panda_description`

## Running demo scenes

Before launching `ur5_*` demo scenes, verify that `ur5_moveit_config` resolves from the active workspace via `ament_index`. This catches the common case where asset packages are still hidden by `COLCON_IGNORE` markers because the workspace was built before running `fix_workspace_layout.sh`.

```bash
source ~/workcell_ws/install/setup.bash
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
ros2 pkg prefix ur5_moveit_config
ros2 launch ur5_3f_test demo.launch.py
ros2 launch ur5_2f_test demo.launch.py
# after generating and rebuilding your own package (scene packages still depend on the asset packages above):
ros2 launch my_generated_scene demo.launch.py
```

For scripted/CI workflows where you want one reusable launcher command for **any** scene:

```bash
SCENE_PKG=my_generated_scene
USE_FAKE=true   # set to false for real hardware

ros2 pkg prefix "${SCENE_PKG}"
ros2 launch "${SCENE_PKG}" demo.launch.py use_fake_hardware:="${USE_FAKE}"
```

## Checks

```bash
./scripts/check_ros2_control_joint_state_fix.sh
```

## Documentation

- 📖 [Full Documentation](https://easy-manipulation-deployment-docs.readthedocs.io/)
- 📚 [API Documentation](https://tanjpg.github.io/emd_docs/html/index.html)

## Architecture

This package uses the following external dependencies (fetched via `dependencies/emd_epd_ws.repos` by default):

| Package | Description |
|---------|-------------|
| [tesseract](https://github.com/tesseract-robotics/tesseract) | Motion planning framework |
| [tesseract_planning](https://github.com/tesseract-robotics/tesseract_planning) | Planning algorithms |
| [trajopt](https://github.com/tesseract-robotics/trajopt) | Trajectory optimization |
| [tesseract_ros2](https://github.com/tesseract-robotics/tesseract_ros2) | ROS 2 integration |
| [boost_plugin_loader](https://github.com/tesseract-robotics/boost_plugin_loader) | Plugin system |

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
