# Workcell Operator & Commissioning Manual

This manual is written for engineers who are comfortable with Linux/ROS 2 but are new to this repository. It focuses on copy-paste operational steps for building, validating, launching, troubleshooting, and extending scenes without changing grasp runtime behavior.

---

## A) Purpose of the platform

Easy_Manipulator_Improved is an **internal engineering platform** for:

- generating robotic workcell scene packages,
- validating scene contracts before runtime,
- simulating grasp planning/execution,
- commissioning robotic cells with repeatable launch paths.

The intended product is the **final physical robotic cell deployment** (robot, tooling, integration, commissioning package). The software repository supports engineering delivery; it is not the primary sellable product by itself.

---

## B) System overview

### Major layers

- **Scene packages** (`scenes/<scene_name>`): define robot/environment/metadata and launch config per cell.
- **workcell_builder**: generates scene package artifacts for new cells.
- **`scene_manifest.yaml` / `workcell.yaml`**: normalized scene contract consumed by validation and runtime loaders.
- **Grasp planner**: perception-driven grasp candidate generation and filtering.
- **Grasp execution**: motion planning, pick/place/release state flow.
- **MoveIt / ros2_control**: planning pipelines, kinematics, controllers, trajectories.
- **Perception / EPD**: object localization and scene input for grasp planning.

### Simple architecture diagram

```text
workcell_builder (optional generation step)
                |
                v
   scene package + scene_manifest.yaml/workcell.yaml
                |
                v
      validate_scene_contract.py / check_all_scenes.sh
                |
                v
        run_grasp_planner  <---- Perception / EPD input
                |
                v
        run_grasp_execution ----> MoveIt + ros2_control
                |
                v
           Simulated/commissioned robotic cell behavior
```

[Future screenshot: repository tree showing scenes and manual files]

---

## C) Workspace setup

Use a clean shell and source ROS + workspace overlays in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash
```

Build all packages (symlink install, conservative worker count):

```bash
colcon build --symlink-install --parallel-workers 2
```

Targeted build examples:

```bash
colcon build --symlink-install --packages-select run_grasp_execution
colcon build --symlink-install --packages-select run_grasp_planner run_grasp_execution
```

---

## D) Validate the workspace

Recommended validation-only preflight gate:

```bash
./scripts/preflight_workcell.sh
```

Launch smoke preflight (validation + bounded headless launch readiness checks):

```bash
./scripts/preflight_workcell.sh --with-smoke
```

Individual commands:

```bash
./scripts/validate_scene_contract.py ur5_2f_test
./scripts/check_all_scenes.sh
./scripts/generate_scene_validation_report.py
./scripts/check_scene_self_tests.sh
./scripts/generate_scene_self_test_report.py
./scripts/check_task_recipes.sh
./scripts/generate_task_recipe_report.py
./scripts/smoke_launch_scenes.sh ur5_2f_test
./scripts/smoke_launch_scenes.sh
./scripts/generate_smoke_launch_report.py
```

When to use each tool:

- `validate_scene_contract.py`: deep check for one specific scene (best for fixing failures).
- `check_all_scenes.sh`: quick fleet check across known scene packages.
- `generate_scene_validation_report.py`: writes markdown summary for handoff/review.
- `preflight_workcell.sh`: operator command that runs checks + report together.

Interpretation:

- **PASS**: required contract fields are present and valid.
- **WARN**: contract passes, but non-blocking metadata issue exists (for example `scene.name` mismatch).
- **FAIL**: required fields/types or declared file paths are invalid.
- **SKIP**: scene package is not currently discoverable in your sourced environment.

Smoke preflight intent:

- Validation preflight checks scene manifests/files only.
- Smoke launch preflight proves `run_grasp_execution` can start to readiness markers for each scene.
- Smoke launch preflight does **not** test full grasp execution quality, EPD perception behavior, camera hardware, or physical robot behavior.
- Use full planner/perception/robot integration tests after smoke preflight passes.
- Scene self-test metadata checks are deterministic/offline metadata validation only; they do not launch the robot.
- Task recipe metadata checks are deterministic/offline contract validation only; they do not launch the robot.

> Generated scenes from `workcell_builder` must pass this same validation contract before they are treated as runnable scenes.

[Future screenshot: terminal output for successful validation]

---

## E) Launch known-good scene

Known-good execution launch command:

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_test
```

Typical success indicators in logs:

- generated workcell context is printed/loaded,
- required controllers are loaded/spawned,
- planning group initialized,
- grasp planning/execution succeeded,
- release step succeeded,
- return-to-home step succeeded.

[Future screenshot: RViz showing loaded ur5_2f_test scene]

---

## F) Run grasp planner with perception

Open a second terminal (with ROS/workspace sourced) and run:

```bash
ros2 launch run_grasp_planner grasp_planner_launch.py
```

Expected EPD/perception-adjacent logs often include:

- `EPD Workflow Enabled`
- `EPD Localization Enabled`
- `Perception input received`
- `EPD detected N objects`
- `Grasp Planning complete`

Important interpretation:

- `No grasp tasks generated` usually indicates candidate filtering, collision rejection, or perception input quality issues.
- It does **not automatically mean launch failure**.

[Future screenshot: grasp candidate in RViz]

---

## G) Scene contract explained

Complete example (`scene_manifest.yaml`):

```yaml
scene:
  name: ur5_2f_test

robot:
  model: ur5
  planning_group: manipulator
  base_frame: base_link
  ee_link: gripper_base_link
  home_named_target: home

end_effector:
  type: finger
  brand: robotiq_85_gripper
  grasp_frame: gripper_base_link
  allowed_touch_links:
    - gripper_base_link
    - gripper_finger1_knuckle_link
    - gripper_finger2_knuckle_link
    - gripper_finger1_finger_link
    - gripper_finger2_finger_link
    - gripper_finger1_inner_knuckle_link
    - gripper_finger2_inner_knuckle_link
    - gripper_finger1_finger_tip_link
    - gripper_finger2_finger_tip_link

environment:
  support_surface_link: table

perception:
  input_frame_options:
    - world
    - base_link

home_return:
  safe_joint_state: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
```

Field intent:

- `scene.name`: canonical scene identifier.
- `robot.model`: robot family/model used by launch config.
- `robot.planning_group`: MoveIt group used for planning (must exist in SRDF).
- `robot.base_frame`: expected base frame for scene transforms.
- `robot.ee_link`: runtime end-effector link used by planners/execution.
- `end_effector.type`: end-effector class (e.g., finger/suction).
- `end_effector.brand`: normalized end-effector metadata identifier.
- `end_effector.grasp_frame`: frame used to apply grasp poses.
- `end_effector.allowed_touch_links`: links allowed to contact grasped objects.
- `environment.support_surface_link`: table/surface link used for support assumptions.
- `perception.input_frame_options`: accepted perception frame options.
- `home_return.safe_joint_state`: explicit safe home fallback joint values.

Validation rule of thumb: if required fields are missing/empty or wrong type, validation should fail **before runtime launch**.

---

## H) Workcell Builder generated scenes

Generated scenes should emit all required artifacts:

- scene package,
- launch file,
- `environment.yaml` (if used by the scene),
- `scene_manifest.yaml` or `workcell.yaml`,
- `self_test` commissioning metadata block,
- `task_recipe` job metadata block,
- URDF links that actually exist,
- a valid MoveIt planning group,
- valid gripper metadata.

Reference placeholder template: `workcell_builder/workcell_builder/templates/ros2/scene_manifest_contract_template.yaml`.

### Generated scene checklist

- [ ] Package builds with `colcon build --symlink-install --packages-select <scene_pkg>`
- [ ] `scene_manifest.yaml` or `workcell.yaml` exists in package share
- [ ] `./scripts/validate_scene_contract.py <scene_pkg>` returns PASS
- [ ] `robot.planning_group` is valid in SRDF
- [ ] `robot.ee_link` exists in URDF
- [ ] `end_effector.grasp_frame` exists and is consistent
- [ ] `end_effector.allowed_touch_links` are non-empty and valid link names
- [ ] `self_test` block validates for deterministic commissioning metadata
- [ ] `task_recipe` block validates (or is intentionally disabled with WARN)

---

## Scene self-test object

`self_test` defines a deterministic commissioning object pose per scene. It is intended for repeatable offline checks and future one-click offline pick/place simulation workflows.

Important scope:

- it **does not** launch the robot,
- it **does not** require EPD/camera input,
- it **does not** require RealSense, RViz, or physical hardware,
- it is conservative commissioning metadata that should be tuned per physical cell.

Example:

```yaml
self_test:
  enabled: true
  object:
    id: commissioning_box
    shape: box
    dimensions: [0.05, 0.05, 0.05]
    frame_id: world
    pose_xyz: [0.45, 0.0, 0.08]
    pose_rpy: [0.0, 0.0, 0.0]
  expected:
    min_grasp_candidates: 1
    allow_simulated_execution: true
```

Quick commands:

```bash
./scripts/check_scene_self_tests.sh
./scripts/generate_scene_self_test_report.py
./scripts/preflight_workcell.sh
```
- [ ] launch file starts without package-not-found errors

---

## Task recipes

`task_recipe` describes **what job the cell should perform**. This avoids hard-coding every scene to simple pick/place and creates a stable contract for future UI/runtime layers.

Representative job families:

- colour sorting
- shape sorting
- garbage sorting
- reject handling
- inspection routing
- palletising
- binning
- generic pick/process/place workflows

Current scope (this PR):

- validates and reports task recipe metadata,
- does not yet convert recipes into runtime execution behavior,
- does not require EPD, RealSense, RViz, or physical hardware.

Future scope:

- offline simulated jobs generated from recipes,
- UI-driven workflow selection and execution using recipe contracts.

Example:

```yaml
task_recipe:
  id: colour_sort_demo
  name: Colour Sort Demo
  type: sort
  enabled: true
  inputs:
    perception_source: epd
    required_attributes:
      - class
      - colour
      - shape
  pick:
    object_source: perception
    grasp_strategy: auto
    allowed_grasp_methods:
      - finger
  decision_rules:
    - id: red_to_bin_a
      when:
        attribute: colour
        equals: red
      destination: bin_a
    - id: default_reject
      when:
        default: true
      destination: reject_bin
  destinations:
    - id: bin_a
      frame_id: world
      pose_xyz: [0.35, -0.35, 0.12]
      pose_rpy: [0.0, 0.0, 0.0]
      action: place
    - id: reject_bin
      frame_id: world
      pose_xyz: [0.20, 0.0, 0.12]
      pose_rpy: [0.0, 0.0, 0.0]
      action: reject
  expected:
    allow_offline_validation: true
    min_valid_destinations: 1
```

Commands:

```bash
./scripts/check_task_recipes.sh
./scripts/generate_task_recipe_report.py
./scripts/preflight_workcell.sh
```

---

## I) Troubleshooting guide

### `parameter_value_from failed for home_return.safe_joint_state`

- Cause: missing/invalid `home_return.safe_joint_state` and no usable fallback.
- Fix: provide a valid numeric list, or ensure `robot.home_named_target` exists and resolves.

### `No kinematics plugins defined`

- Cause: missing/incomplete MoveIt kinematics config.
- Fix: check scene MoveIt config package and `kinematics.yaml` mapping for planning group.

### `All Possible Grasp sample collides with something in the scene`

- Cause: candidates filtered by collision checker.
- Fix: verify object pose, support surface dimensions, allowed touch links, and end-effector frame alignment.

### `IK solution in collision`

- Cause: solver found kinematics but colliding state.
- Fix: inspect candidate pose/orientation and nearby collision geometry.

### `forearm_link/table_/upper_arm_link collisions`

- Cause: conservative collision pairs or poor approach path with current setup.
- Fix: verify scene alignment, table pose, and start/home state consistency.

### Release planning fails due to octomap/fingertip contact

- Cause: conservative collision checking during release phase.
- Fix: verify release waypoint clearance, perception map freshness, and object detachment timing.

### `gripper controller not spawned`

- Cause: ros2_control controller configuration or launch ordering issue.
- Fix: inspect controller YAML and launch logs for spawn/load failures.

### Missing `grasp_frame`

- Cause: missing `end_effector.grasp_frame` in manifest.
- Fix: add non-empty valid frame string and re-run validation.

### Missing `ee_link`

- Cause: missing `robot.ee_link` in manifest.
- Fix: add non-empty valid link and re-run validation.

### `package not found`

- Cause: workspace not sourced or package not built/installed.
- Fix: source `/opt/ros/humble/setup.bash` then `~/workcell_ws/install/setup.bash`, rebuild if needed.

### `SKIP` in `check_all_scenes.sh`

- Cause: scene package not discoverable in current shell.
- Fix: source the correct workspace overlay and ensure scene package is installed.

### PyYAML missing / fallback parser in use

- Cause: `python3-yaml` is not installed in the current environment.
- Expected behavior: validation still runs with the built-in fallback parser and prints a parser note.
- Fix (optional): install `python3-yaml` for full YAML syntax support, or simplify manifest syntax to contract-friendly YAML.

### RViz `get_planning_scene` warning when demo node dies

- Cause: expected warning after planning scene service provider exits.
- Fix: treat as secondary symptom; diagnose primary node crash earlier in logs.

---

## J) Manual test checklist

- [ ] source ROS (`source /opt/ros/humble/setup.bash`)
- [ ] source workspace (`source ~/workcell_ws/install/setup.bash`)
- [ ] build (`colcon build --symlink-install --parallel-workers 2`)
- [ ] run preflight gate (`./scripts/preflight_workcell.sh`)
- [ ] validate one scene (`./scripts/validate_scene_contract.py ur5_2f_test`)
- [ ] launch `ur5_2f_test`
- [ ] run planner
- [ ] confirm successful grasp
- [ ] confirm release
- [ ] confirm home return

---

## K) Future screenshots/pictures

Use this folder for future manual imagery:

- `docs/manuals/assets/images/`

Suggested future inserts:

- `[Future screenshot: RViz showing loaded ur5_2f_test scene]`
- `[Future screenshot: terminal output for successful validation]`
- `[Future screenshot: grasp candidate in RViz]`

No image files are required for this PR.
