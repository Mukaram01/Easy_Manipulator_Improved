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
## Quick start (Ubuntu 22.04 + ROS 2 Humble)

This repository is intended to live inside a larger ROS 2 workspace (for example, `~/emd_epd_ws`) alongside upstream source dependencies (`tesseract`, `tesseract_planning`, `trajopt`, `tesseract_ros2`, `tesseract_qt`, and related libraries imported by `vcs`).

```bash
# 1) System dependencies (Ubuntu 22.04 / ROS 2 Humble)
sudo apt update
sudo apt install -y \
  build-essential cmake git curl \
  python3-vcstool python3-colcon-common-extensions python3-rosdep \
  qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5svg5-dev \
  ros-humble-moveit ros-humble-moveit-visual-tools ros-humble-xacro ros-humble-ur-description

# If this is your first ROS 2 machine:
sudo rosdep init || true
rosdep update

# 2) Create workspace
mkdir -p ~/emd_epd_ws/src
cd ~/emd_epd_ws/src

# 3) Clone this repo into src/ using the workspace-facing name used below
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment

# 4) Import upstream source dependencies
cd ~/emd_epd_ws
vcs import src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos

# 5) Install package dependencies from source tree
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Note: this repository now ships ignore markers on compatibility asset mirrors
# to prevent duplicate-package discovery errors in fresh clones.

# 6) Build
colcon build --symlink-install

# 7) Source + run demo
source install/setup.bash
ros2 launch suction_test demo.launch.py
```

> Tip: after syncing and validating a workspace, you can lock exact revisions with `vcs export --exact src > emd_epd_ws_exact.repos`.

### If Studio build fails

If `tesseract_qt` Studio fails to link against Qt ADS targets, apply the included upstream patch and rebuild:

```bash
cd ~/emd_epd_ws
./src/easy_manipulation_deployment/scripts/apply_upstream_patches.sh
colcon build --symlink-install
```

## Known issues / Troubleshooting

- **Qt ADS target mismatch in `tesseract_qt` Studio (common on Ubuntu 22.04/Jammy):** some installations export `ads::qtadvanceddocking-qt5` instead of `ads::qtadvanceddocking`. This repository includes a patch script that applies `patches/tesseract_qt_qtads_target_fix.patch` so Studio chooses whichever exported ADS target exists.
- **Patch application command:** run `./src/easy_manipulation_deployment/scripts/apply_upstream_patches.sh` from your workspace root, then rebuild with `colcon build --symlink-install`.
- **Device/group permission changes:** if you add your user to groups such as `video`, `plugdev`, or `input`, log out and back in before retrying tools that require those permissions.

### Dependency-first build flow (catch `tesseract_geometry`/OctoMap issues early)

If the workspace fails during global `colcon build`, validate the dependency layer first and only then build everything.

```bash
cd ~/emd_epd_ws

# 1) Build the package that exports tesseract geometry config first.
colcon build --symlink-install --packages-select tesseract_geometry \
  2>&1 | tee /tmp/colcon_tesseract_geometry.log

# 2) Build one OctoMap-consuming layer next to validate imported target exports.
colcon build --symlink-install --packages-select tesseract_collision \
  2>&1 | tee /tmp/colcon_tesseract_collision.log

# 3) Immediately fail-fast on the known OctoMap import error signature.
if rg -n 'IMPORTED_LOCATION not set for imported target "octomap"' \
  /tmp/colcon_tesseract_geometry.log /tmp/colcon_tesseract_collision.log; then
  echo "OctoMap import/export issue detected. Fix dependency exports before full build."
  return 1 2>/dev/null || exit 1
fi

# 4) Only run the full workspace build after dependency checks pass.
colcon build --symlink-install
```

Minimal invocation sequence:

1. `colcon build --packages-select tesseract_geometry`
2. `colcon build --packages-select tesseract_collision`
3. `rg 'IMPORTED_LOCATION not set for imported target "octomap"' /tmp/colcon_tesseract_*.log`
4. `colcon build --symlink-install`


### Known benign warnings

During source builds, warnings from third-party packages such as `qpoases` and
`octomap-distribution` can appear noisy but are often harmless.

Use this quick rule:
- If build output only shows `warning:` diagnostics and the package finishes, treat it as benign.
- If colcon stops, ignore preceding warning spam and start with the first `error:` line.

The project keeps strict warnings (`-Wall -Wextra -Wpedantic`) for repository-owned
code, while warning suppressions are scoped only to external imported targets.

---

## Universal Robots Description (ur_description)

- Expected source: [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`ur_description` v2.x, ROS 2).
- Install via apt when using ROS 2 binaries: `ros-${ROS_DISTRO}-ur-description`.
- Scenes use `ur_macro.xacro` with `ur_type` configs and no longer require per-robot xacros.
- Legacy per-robot xacros remain supported when present.

Example validation command:

```bash
ros2 run xacro xacro src/scenes/suction_test/urdf/scene.urdf.xacro ur_type:=ur5 > /tmp/scene.urdf
```

Troubleshooting:
- If a ROS 1 `ur_description` is on your `ROS_PACKAGE_PATH`, `ur.urdf.xacro` will fail with `Undefined substitution argument name`. Remove the ROS 1 package and install the ROS 2 `ur_description` v2.x instead.

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
dependencies, applies the Cereal/Boost fixes, and builds the workspace.

By default, it uses the **minimal** profile for headless/runtime deployments,
which does **not** import the full Tesseract/TrajOpt source overlays.

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src

# Clone the repo into the intended layout.
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment

cd easy_manipulation_deployment
./fix_and_build_humble.sh
```

Use the **full** profile when you explicitly need planning/dev overlays:

```bash
./fix_and_build_humble.sh --profile full
```

Enable legacy workaround behavior (optional, explicit opt-in):

```bash
./fix_and_build_humble.sh --profile full --legacy-workarounds
```

### Deployment profile matrix

| Profile | Intended use | Overlay import (`tesseract.repos`) | Legacy ignores/patches |
|---------|--------------|--------------------------------------|-------------------------|
| `minimal` (default) | Runtime/headless deployment | ❌ No | ❌ No |
| `full` | Planning + development workspace | ✅ Yes | ❌ No (unless `--legacy-workarounds`) |

### Package matrix

| Scope | Typical packages |
|-------|------------------|
| Minimal runtime | `easy_manipulation_deployment`, `emd_msgs`, `workcell_builder`, scene/demo packages, ROS binary dependencies installed via `rosdep` |
| Full planning/dev | Minimal runtime packages **plus** source overlays from `tesseract.repos` (`tesseract`, `tesseract_planning`, `trajopt`, `tesseract_ros2`, `boost_plugin_loader`) |

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
  ros-humble-ur-description \
  ros-humble-xacro
```

#### 1) Recreate the workspace the “original” way

```bash
mkdir -p ~/workcell_ws/src
cd ~/workcell_ws/src

# clone your repo INTO src as easy_manipulation_deployment (this is the intended layout)
git clone https://github.com/Mukaram01/Easy_Manipulator_Improved.git easy_manipulation_deployment
```

#### 2) Use a single canonical location for `assets` and `scenes`

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
# Keep both directories in the repository root as the source of truth.
# Do not keep separate copied directories in ~/workcell_ws/src.
test -d assets && test -d scenes
```

If you are migrating an older workspace, do **not** run:

```bash
mv easy_manipulation_deployment/easy_manipulation_deployment/workcell_builder ./easy_manipulation_deployment
```

That path never existed in this repository. The ROS package lives at:

```text
easy_manipulation_deployment/workcell_builder/workcell_builder
```

The recommended fix is to let the repository helper expose the correct package
layout for you:

```bash
cd ~/workcell_ws
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
```

That helper creates `src/workcell_builder` plus symlinks for the hidden asset
packages (for example `workbench_description`, `ur5_moveit_config`,
`robotiq_85_moveit_config`, and related robot/environment packages) so
`rosdep` and `colcon` can discover them without manually moving directories.

Resulting layout (recommended):

```text
~/workcell_ws/src/
└── easy_manipulation_deployment/
    ├── assets/
    ├── scenes/
    ├── CMakeLists.txt
    └── ...
```

#### 2.a) Migration from older dual-path setups

Use this once if your workspace still has duplicate directories in both
`~/workcell_ws/src/` and `~/workcell_ws/src/easy_manipulation_deployment/`.

1. Detect existing duplicates.
2. Choose `easy_manipulation_deployment/{assets,scenes}` as authoritative.
3. Remove stale duplicate copies in `~/workcell_ws/src` safely.
4. Recreate top-level symlinks only if a legacy tool still requires them.

```bash
cd ~/workcell_ws/src

# 1) Detect duplicates and show their type (directory/symlink/missing)
for p in assets scenes; do
  printf "%-8s src/%-6s: " "$p" "$p"
  [ -e "$p" ] && stat -c '%F -> %N' "$p" || echo "missing"
  printf "%-8s repo/%-6s: " "$p" "$p"
  [ -e "easy_manipulation_deployment/$p" ] && stat -c '%F -> %N' "easy_manipulation_deployment/$p" || echo "missing"
done

# 2) Ensure canonical directories exist in the repo path
test -d easy_manipulation_deployment/assets
test -d easy_manipulation_deployment/scenes

# 3) Remove stale top-level copies ONLY if they are real directories (not symlinks)
for p in assets scenes; do
  if [ -d "$p" ] && [ ! -L "$p" ]; then
    rm -rf "$p"
  fi
done

# 4) Optional: recreate legacy compatibility symlinks (only if needed)
# ln -s easy_manipulation_deployment/assets assets
# ln -s easy_manipulation_deployment/scenes scenes
```

If you must keep legacy symlinks, this is the exact expected tree:

```text
~/workcell_ws/src/
├── assets -> easy_manipulation_deployment/assets
├── scenes -> easy_manipulation_deployment/scenes
└── easy_manipulation_deployment/
    ├── assets/
    ├── scenes/
    ├── CMakeLists.txt
    └── ...
```

**Asset sourcing note:** `share/workcell_builder/assets` is the canonical default-asset
source. Runtime/workspace paths such as `src/assets` exist only as writable mirrors for user
customization; Workcell Builder populates them from the packaged defaults when needed. The repo
also keeps compatibility asset trees for older workflows, and CI now enforces that any tracked
duplicate tree matches the canonical package asset tree by path list and checksum.

#### 3) Import full planning overlays (optional)

Only needed for a full planning/development workspace:

```bash
cd ~/workcell_ws/src
vcs import < easy_manipulation_deployment/tesseract.repos
```

If you want the repo-pinned dependency set used by the older EMD/EPD workspace
instructions instead, the correct file is:

```bash
cd ~/workcell_ws
vcs import src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos
```

#### 4) Preferred package/version guidance (before legacy workarounds)

Prefer these first:

- Use the latest `main` branch of this repository.
- Keep to Ubuntu 22.04 + ROS 2 Humble for the tested path.
- Run `./fix_and_build_humble.sh` (minimal) for runtime deployment.
- Use `./fix_and_build_humble.sh --profile full` only when you need overlay sources.

#### 5) Legacy fallback workaround (explicit, only if needed)

> **Note:** If you ran `./fix_and_build_humble.sh`, these fixes have already
> been applied only when you opt in with `--legacy-workarounds`.

```bash
cd ~/workcell_ws
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Recreate the expected package layout first so rosdep sees workbench_description
# and the robot/end-effector config packages hidden under assets/.
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh

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

#### 6) Now rosdep + build

```bash
cd ~/workcell_ws
rosdep update
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}" \
  --skip-keys "tesseract_motion_planners"

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

**Scene root resolution precedence (startup):**
1. `--scene-root <path>` (or `--scene-root=<path>`) CLI flag.
2. `WORKCELL_BUILDER_SCENE_ROOT` environment variable.
3. Current working directory (`$PWD`).
4. Parent of current working directory (`$PWD/..`).
5. `$PWD/src/easy_manipulation_deployment`.
6. `$PWD/src`.

The selected workcell root and all rejected candidates are logged at startup. If multiple valid
scene directories exist, Workcell Builder warns and keeps the highest-priority candidate.

For deterministic behavior in multi-workspace setups, explicitly set one override:
```bash
# CLI override (highest priority)
workcell_builder --scene-root ~/workcell_ws/src/easy_manipulation_deployment

# or environment override
export WORKCELL_BUILDER_SCENE_ROOT=~/workcell_ws/src/easy_manipulation_deployment
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

### Known Issues

#### ONNX Runtime build failures on ROS 2 Humble (Ubuntu 22.04)

On ROS 2 Humble/Jammy, older vendorized ONNX Runtime drops can fail during C/C++
compilation with messages similar to:

```
error: exponent has no digits
```

This is a known compatibility issue in legacy packaging paths and should be
handled in the following order:

1. **Prefer the native APT package first** (`libonnxruntime-dev`) when it is
   available for your platform and compatible with your ROS/Tesseract stack.
2. **Otherwise prefer a newer ONNX Runtime release** (for example via an updated
   ROS vendor package) that is known to support Ubuntu 22.04/Humble better than
   legacy vendor drops.

If you must stay on a vendor package path, expect to apply Jammy-specific
patching and environment cleanup (for example locale normalization and compiler
compatibility fixes) when hitting parser/compiler edge-case errors.

**Why we document this explicitly:** maintainers repeatedly lose time by
defaulting back to older vendor snapshots that compile unreliably on Jammy,
while current system packages or newer ONNX Runtime releases are generally more
stable and require less local patch maintenance.

### Optional manual patches

The following steps are optional and only needed if you hit the corresponding build errors.

<details>
<summary><b>Legacy fallback: skip incompatible packages</b></summary>

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
<summary><b>Legacy fallback: remove incompatible trajopt_ifopt planner</b></summary>

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

**Fix:** Fresh clones should avoid this automatically via ignore markers in `assets/`. If you still see it in an older checkout, run `./scripts/fix_workspace_layout.sh` or remove one duplicate tree.
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
