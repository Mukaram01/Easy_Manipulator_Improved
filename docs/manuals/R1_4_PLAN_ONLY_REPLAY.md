# R1.4: headless plan-only replay commissioning

This check runs the existing perceived-object grasp planner and executor against
`ur5_2f_test`. It plans approach, contact, closure, attached-object lift, transfer,
place, opening, retreat and home. It does **not** execute any trajectory.

`detected_objects/v1` is the common input: stable object ID/class, world-frame
XYZ/RPY pose, positive box dimensions, confidence and acquisition timestamp.
`--replay` only stamps a deterministic observation once at acquisition; both
replay and timestamped live-format data then use the same normalization, target
filter, geometry-derived grasp candidates, executor and MoveIt planning path.
The cup is selected explicitly by the commissioning task. The bottle stays a
collision obstacle. The scene's authored bottle task is not overwritten.

The normalizer currently requires world-frame boxes; a future live adapter must
supply that contract. This is not live EPD or physical-grasp acceptance.

## Build and run

From a ROS 2 Humble workspace containing this checkout and the normal EMD
build dependencies:

```bash
source /opt/ros/humble/setup.bash
# On this workstation, use the clean Humble MoveIt underlay used for compilation.
source /home/user/ws_moveit2/install/local_setup.bash
colcon build --symlink-install --packages-select \
  workcell_builder ur5_2f_test emd_grasp_planner emd_grasp_execution \
  run_grasp_planner run_grasp_execution run_waypoint_execution \
  --allow-overriding ur5_2f_test
source install/setup.bash
cd src/easy_manipulation_deployment
python3 -m pytest -q tests/test_runtime_pick_inputs.py \
  tests/test_transactional_pick_cycle.py tests/test_perceived_object_grasp_execute.py \
  tests/test_perceived_object_grasp_plan.py tests/test_r14_plan_only_acceptance.py
timeout --signal=TERM --kill-after=5s 160s \
  python3 scripts/run_r14_plan_only_acceptance.py \
  --output-dir /tmp/r14-plan-only --timeout 120 --domain-id 179
```

Use an unused ROS domain. The runner fixes launch arguments to
`use_fake_hardware:=true allow_trajectory_execution:=false launch_rviz:=false`
and never passes the executor's `--start` flag. It checks the installed scene
against this checkout and records its hashes. It waits for the existing scene
loader, runs the executor with a finite budget, and stops its owned process
groups. A forced shutdown or remaining process fails acceptance.

`executor.json` contains the actual candidate/stage results, fake-hardware
proof and unchanged-live-scene check. `acceptance.json` checks all nine
`PREPLAN_*` results, records execution-action observations and shutdown status.
A PASS requires `full_cycle_prevalidated=true`, `execution_attempted=false`,
no trajectory execution action goals and `shutdown_clean=true`. Startup alone
cannot pass. Planning uses private predicted robot/attachment/world states;
real collision geometry and the baseline ACM remain enforced. Contact allowance
is restricted to the selected object and configured fingertips.

For a fresh build without the original workspace overlay, also build the
repository's `emd_msgs`, `ur_description`, `ur5_moveit_config`,
`robotiq_85_description`, `robotiq_85_moveit_config`, `realsense2_description`,
`workbench_description` and `sorting_bin_description` packages. Source only
Humble, the same MoveIt underlay used at build time, and that clean install's
`local_setup.bash` for runtime reproduction. Build `emd_waypoint_execution` too
when isolating the EMD demo dependencies.

Reverting this PR removes the replay adapter/full-cycle preplanning changes;
it does not alter authored scene transforms or hardware configuration.

## Workstation dependency evidence

The isolated runtime uses the clean MoveIt 2.5.9 checkout at
`/home/user/ws_moveit2` (commit `53979331b`), then the clean Workcell install.
The first `/opt/ros/humble`-only runtime planned all stages but segfaulted in
`rclcpp::CallbackGroup` during MoveIt teardown. It is **not** accepted as clean
shutdown. The runner rejects crashed child processes even when none remain.
Using the consistent clean MoveIt underlay passed planning and graceful shutdown;
no MoveIt sources or binaries were changed by this PR.
