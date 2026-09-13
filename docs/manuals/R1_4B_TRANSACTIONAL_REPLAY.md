# R1.4B — full-cycle replay prevalidation and target retry

This supersedes the sequential-planning behavior described in the original R1.4 replay note. The same executor, generated scene, normalized inputs, robot/gripper model, MoveIt backend, home and destination contracts are reused. No new planning server, GUI, simulator or EPD adapter is introduced.

## Planning transaction

After validating and inserting all observations, capture the complete live PlanningScene and robot state. Require canonical home, an open gripper, no attachment, and exclusively mock hardware. There is no automatic home motion before candidate acceptance.

Enumerate eligible targets by descending confidence, then stable runtime ID. Enumerate grasps in the existing order (3, 0, 1, 2, 4, 5, 6, 7). Choose the first target/grasp that passes every critical stage. Every rejected attempt records object ID, grasp index, failed stage and reason. Non-target objects never enter this loop, and all rejected targets remain world obstacles.

Every motion request uses the existing `/move_action` (`moveit_msgs/action/MoveGroup`) with `planning_options.plan_only=true`, `replan=false`, and a request-local `planning_scene_diff`. Humble's MoveGroup implementation clones its planning scene before applying this diff. Predicted robot states, target-contact ACM entries, attachments and placed geometry are never applied to the live scene during prevalidation.

The candidate sequence is:

1. Approach from the captured initial state.
2. Grasp/contact, then planned gripper closing to the first geometrically valid fingertip contact.
3. Private attachment: remove the target from the private world's object list and put its geometry in the predicted robot state, expressed relative to the attachment link using predicted FK.
4. Lift, transfer and place, all with that attachment and normal environment/distractor collision rules.
5. Gripper opening; private detachment at predicted achieved FK.
6. Retreat with the placed object in the world, then return home.

Each segment starts at the previous trajectory endpoint. Mimic joints are derived from the runtime URDF. Seeded IK proposes joint goals only; it does not establish collision feasibility. Every accepted trajectory must come from a successful MoveIt private-scene plan, with a nonempty trajectory and a start state matching the predicted state. Start-state adapter displacement is rejected.

Contact and retreat chain short (at most 5 mm) MoveIt-planned moves. FK checks at every returned trajectory point enforce a narrow vertical corridor (2.5 mm lateral, 1 mm vertical endpoint margin, quaternion distance 0.005). The cached subtrajectories are concatenated with monotonic timestamps and stationary waypoint boundaries. Collision validation remains MoveIt's responsibility; this is a segmented path, not a claim of mathematically exact Cartesian interpolation between all samples.

Only after all nine motion segments succeed is `full_cycle_prevalidated` set. The live robot/scene must still match the captured snapshot. A plan-only invocation then returns `PLAN_ONLY` without any ExecuteTrajectory goal. No trajectory payload is written to configuration or authored YAML.

## Execution and divergence

`--start` plus full-cycle prevalidation are both mandatory. Execute the cached trajectories; there is no sequential replanning or motion retry after execution starts. Before and after each stage, compare robot joints (5 mrad tolerance), world geometry, attachment identity/transform/touch links, ACM, padding/scaling and collision map against the expected state. Target geometry permits at most 3 mm positional and 0.005 quaternion-distance deviation; non-target box geometry uses a 1e-7 tolerance. Meshes are compared as ROS fields, not serialized CDR bytes, whose padding is not scene state.

Use the existing real-scene attachment/detachment helpers only at their execution stages. Verify actual fingertip contacts before attaching. Detach at achieved FK, not an idealized goal. On failure, report the precise stage, stop the sequence, restore the baseline contact ACM where possible, and preserve held/placed objects for explicit recovery. Accepted-action timeouts request cancellation and report confirmation status. There is no automatic recovery motion.

The in-memory cycle contains selected object/grasp, ordered steps with expected before/after scenes and cached RobotTrajectory messages. JSON reports contain compact per-stage result codes, point counts, planning time, attached/world IDs, candidate failures, stage timestamps and execution results. They omit trajectory arrays and mesh payloads.

## Acceptance

Terminal 1:

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=164
ros2 launch ur5_2f_test demo.launch.py \
  use_fake_hardware:=true allow_trajectory_execution:=true launch_rviz:=false
```

After controllers and scene loading are ready, terminal 2:

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=164
cd src/easy_manipulation_deployment
python3 scripts/perceived_object_grasp_execute.py \
  --task-request config/runtime/r1_4b_task.yaml \
  --detections config/runtime/r1_4b_retry_replay.yaml --replay --start \
  --summary-output /tmp/r1_4b_acceptance.json
```

Omit `--start` for the complete planning-only check. Restart the fake scene before each invocation: existing runtime IDs deliberately block a new observation from overwriting a placed/held object. The new fixture contains two cups and one bottle. Its higher-confidence cup is obstructed by the existing bin; the lower-confidence cup is feasible. IDs are fixture data only. The 180-second age policy is explicit for this static replay search; it is not a live-perception freshness recommendation.

## Scope limits

This proves feasibility against a captured fake scene, not physical grasp quality or a distributed atomic transaction. Environment/joint correspondence is checked at stage boundaries, not by a new continuous scene-change monitor. Planning remains finite-budget and OMPL trajectories need not be identical between runs; candidate ordering is deterministic. Only the existing world-frame BOX/Robotiq baseline is covered. Runtime stream/TF adapters, concurrent-task arbitration, operator controls and automatic recovery/reset remain out of scope. A subsequent invocation requires explicit scene reset. Existing scene-authoring/bin-zone alignment is unchanged.

## Observed validation — 2026-09-12

`scenes/ur5_2f_test/acceptance/r1_4b_transactional_runtime.json` records the successful headless fake-hardware run and source hashes. Eight grasps for the first cup were rejected at `PREPLAN_APPROACH`; the second cup's preferred grasp passed all nine motion stages. Private lift/transfer/place requests carried exactly the selected attachment and excluded it from world objects. `CANDIDATE_READY` preceded the first execution stage, and the live scene/joints were verified unchanged after prevalidation.

All cached motion actions then returned MoveIt/controller execution success, including close/open, retreat and home. Independent final inspection verified unchanged unpicked cup/bottle geometry, zero attachments, home within tolerance and approximately 0.066 mm placement error. The separate full planning-only run returned `PLAN_ONLY`, with no execution and unchanged live scene; it preceded the mesh-comparison correction noted in the evidence. The isolated launch was stopped after verification.

Focused tests: 68 passed across `test_transactional_pick_cycle.py`, `test_runtime_pick_inputs.py`, `test_perceived_object_grasp_execute.py`, and `test_perceived_object_grasp_plan.py`. Transfer/place/retreat failure-injection tests exercise candidate rejection and exact failure propagation; runtime evidence establishes the successful full MoveIt transaction. Byte-compilation and `git diff --check` passed. No colcon rebuild, broad suite, GUI or physical-hardware test was run.
