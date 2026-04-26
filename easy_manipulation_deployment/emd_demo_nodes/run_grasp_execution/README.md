# run_grasp_execution

## Launching and launch tests

- Use `ros2 launch run_grasp_execution grasp_execution.launch.py` for normal runtime launch.
- Run launch tests with `colcon test --packages-select run_grasp_execution --ctest-args -R test_grasp_execution_launch`.

The helper test module in `test/test_grasp_execution_launch_helpers.py` is **not** a ROS launch entrypoint.

## Grasp candidate precheck collision filtering

- Grasp precheck now allows expected gripper fingertip contact with the perceived target occupancy during IK collision checks.
- This filtering is scoped only to grasp candidate precheck, and does **not** change global collision behavior used for planning/execution.
- Default precheck parameters:
  - `grasp_precheck_allowed_touch_links: [gripper_finger1_finger_tip_link, gripper_finger2_finger_tip_link]`
  - `grasp_precheck_allowed_collision_ids: [<octomap>]`
- The current `target_id` (and attached form `#<target_id>`) is always added dynamically for the candidate being checked.
- Arm/support collisions (for example `forearm_link<->table_`, `upper_arm_link<->table_`, or arm links vs octomap) remain invalid and still reject the candidate.
