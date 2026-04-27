# run_grasp_execution

## ROS 2 Humble operator runbook (build/test/manual smoke)

Run the following steps in order on Ubuntu 22.04 + ROS 2 Humble.

1. Source ROS and your workspace overlay:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash
```

2. Validate workspace assets used by scene packages:

```bash
./src/easy_manipulation_deployment/scripts/validate_workspace_assets.sh
```

3. Build the package:

```bash
colcon build --symlink-install --packages-select run_grasp_execution
```

4. Run launch tests:

```bash
colcon test --packages-select run_grasp_execution --ctest-args -R test_grasp_execution_launch
```

5. Manual smoke launch (choose one supported scene package below):

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_3f_test
```

> The helper test module `test/test_grasp_execution_launch_helpers.py` is **not** a ROS launch entrypoint.

## Supported `scene_package` examples and launch commands

`grasp_execution.launch.py` supports packaged examples and generated scenes through `scene_package:=...`.

### Packaged examples

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_test
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_3f_test
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_airpick4_test
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=suction_test
```

### Generated scenes from Workcell Builder

After generating a scene with Workcell Builder and rebuilding/sourcing your workspace, launch with:

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=<your_generated_scene_package>
```

## Workcell Builder scene metadata requirements (`environment.yaml`)

`run_grasp_execution` reads `<scene_package>/environment.yaml` and derives planner/workcell end-effector context.

### Required schema expectations

- `environment.yaml` root must be a YAML mapping (dictionary).
- `end_effector` should be a YAML mapping when present.
- To avoid fallback classification, provide at least:
  - `end_effector.name`
  - `end_effector.brand`
  - optional `end_effector.ee_type`
  - optional `end_effector.attributes.fingers` (recommended for 2F/3F disambiguation)
  - optional frame hints: `robot_link`, `grasp_frame`, `tcp_link`, `physical_ee_link`, `base_link`, `link`, and/or `links`

### Generated workcell context constraints

Derived frames must exist in the generated `urdf/scene.urdf.xacro` link set:

- derived `moveit_link` (`tool0` by default), and
- derived `grasp_frame`.

If derived frames are missing from URDF links, launch falls back to arm-only context (`ee='ur_tool0'`, `link='tool0'`, `grasp_frame='tool0'`).

## Expected logs

### Workcell context generation (normal)

Expect an info log similar to:

- `Generated workcell context for scene '<scene_package>': ee=<ee_id> brand=<ee_id> moveit_link=<ee_link> grasp_frame=<ee_grasp_frame> (file='<tmp_yaml>')`

### Fallback warnings (metadata/frame normalization)

You may see warnings like:

- `Falling back to arm-only workcell end effector classification ... using fallback link/frame 'tool0' and planner ee id 'ur_tool0'.`
- `Scene metadata declares Robotiq 3F, but the URDF does not expose 3F links; falling back to arm-only workcell context ...`
- `Derived workcell frames are not present in the scene URDF links (...); falling back to arm-only context link/frame 'tool0'.`

## Troubleshooting

### 1) Missing `environment.yaml`

Symptom:

- Launch fails with `Failed while parsing scene metadata for grasp execution launch ... file='environment.yaml'`.

Actions:

1. Confirm the package exists in your sourced overlay.
2. Verify `<scene_package>/environment.yaml` exists and is valid YAML.
3. Rebuild and re-source:

```bash
colcon build --symlink-install --packages-select <scene_package>
source ~/workcell_ws/install/setup.bash
```

### 2) Invalid `grasp_frame`

Symptom:

- Candidate warning: `grasp frame conversion unavailable ... Falling back to legacy behaviour.`
- Or frame-normalization warning that derived workcell frames are not in URDF links.

Actions:

1. Ensure `end_effector.grasp_frame` (or fallback frame fields) names a real URDF link.
2. Ensure `end_effector.robot_link` is valid for MoveIt IK target link behavior.
3. Regenerate scene, rebuild, and relaunch.

### 3) Gripper controller skipped

Symptom:

- `Skipping ur5_gripper_controller spawner: ... dummy gripper driver remains responsible for gripper actions.`

Actions:

1. Check scene metadata maps to the expected gripper family (2F/3F/suction).
2. Verify gripper joints exist in robot description and command/state interfaces.
3. Verify `ur5_ros_controllers.yaml` gripper joint list matches URDF joint names.

### 4) `safe_joint_state` skipped

Symptom:

- `[HomeReturn] home_return.use_safe_intermediate=true but home_return.safe_joint_state is empty. Skipping safe intermediate and continuing to home.`
- `[HomeReturn] home_return.safe_joint_state has <N> values, expected <DOF> ... Skipping safe intermediate and continuing to home.`

Actions:

1. Set `home_return.safe_joint_state` as a `double_array`.
2. Ensure array length equals manipulator DOF.
3. Keep `home_return.use_safe_intermediate=true` only when a valid state is configured.

### 5) Start-state collision

Symptom:

- Planning fails/retries with start-state collision issues.

Actions:

1. Confirm current robot state is collision-free in RViz/scene setup.
2. Remove penetrations with table/object/fixtures before planning.
3. Re-run after updating start positions and scene geometry.

Note: launch includes MoveIt request adapter `default_planner_request_adapters/FixStartStateCollision`, but persistent collisions still require correcting scene/state inputs.

## Grasp candidate precheck collision filtering

- Grasp precheck now allows expected gripper fingertip contact with perceived target occupancy during IK collision checks.
- This filtering is scoped only to grasp candidate precheck, and does **not** change global collision behavior used for planning/execution.
- Default precheck parameters:
  - `grasp_precheck_allowed_touch_links: [gripper_finger1_finger_tip_link, gripper_finger2_finger_tip_link]`
  - `grasp_precheck_allowed_collision_ids: [<octomap>]`
- The current `target_id` (and attached form `#<target_id>`) is always added dynamically for the candidate being checked.
- Arm/support collisions (for example `forearm_link<->table_`, `upper_arm_link<->table_`, or arm links vs octomap) remain invalid and still reject the candidate.

### Troubleshooting (precheck)

- If a candidate logs `allowed expected grasp contact`, fingertip contact against `<octomap>`/target is treated as expected during precheck.
- Candidates are still rejected when any non-allowed contact exists (for example arm/support collisions like `forearm_link<->table_`).
