# R1.4 replay pick/place execution

Current executor behavior is documented in [R1.4B transactional replay](R1_4B_TRANSACTIONAL_REPLAY.md). The sequential-planning limitations below describe the original R1.4 implementation and evidence.

This extends `scripts/perceived_object_grasp_execute.py`; it does not create an operator GUI or change authored/generated scene geometry. Use the current generated `ur5_2f_test` package with its existing MoveIt scene loader, controllers, home state, grasp/TCP metadata and destination zone. Replay is a one-shot runtime observation, independent of the EPD bridge.

## Contracts

Task JSON/YAML contains `action: pick_and_place`, arbitrary `target_class`, `source_zone`, `destination_zone`, positive `max_age_seconds` (default 2), and `min_confidence` in [0,1] (default 0). Both zone references resolve against generated `cell_definition.yaml`. The task is an explicit runtime request; it does not rewrite the authored task's target class.

Inputs use the existing `schema_version: detected_objects/v1` envelope with a nonempty `objects` list. Its strict execution profile requires, per object:

- unique stable string `object_id`, nonempty string `class_id`, confidence in [0,1];
- `pose: {frame_id: world, xyz: [x,y,z], rpy: [roll,pitch,yaw]}` in metres/radians;
- positive `dimensions: [x,y,z]` or `{x: ..., y: ..., z: ...}` describing conservative local box bounds supplied by perception/model/fixture;
- finite `timestamp` in Unix seconds; optional opaque `grasp` metadata.

Other frames are rejected until an upstream adapter supplies a valid world transform. Invalid geometry anywhere aborts acquisition rather than silently dropping an obstacle. Stale/low-confidence/out-of-zone/non-target observations remain obstacles but are ineligible. Containment checks the entire oriented box against the oriented source zone. Internal IDs are `runtime::` plus URL-escaped detection IDs, and poses become XYZ + quaternion XYZW. IDs/classes are never inferred from object geometry.

`--replay` accepts only `source.mode: replayed_snapshot`; it stamps a new observation once on acquisition using optional `age_seconds` (default 0). Without that flag timestamps are never refreshed. A future live adapter can produce the same snapshot contract without changes to the executor. No live EPD integration is included.

## Commands

In terminal 1, from the workspace containing the built packages:

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=164
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true allow_trajectory_execution:=true launch_rviz:=true
```

After controllers and the planning-scene loader are ready, in terminal 2:

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=164
cd src/easy_manipulation_deployment
python3 scripts/perceived_object_grasp_execute.py \
  --scene-package ur5_2f_test \
  --task-request config/runtime/r1_4_task.yaml \
  --detections config/runtime/r1_4_replay.yaml --replay --start \
  --summary-output /tmp/r1_4_acceptance.json
```

Omit `--start` for approach/contact planning only. `PLAN_ONLY` is not a full-cycle plan or execution success. Launch itself sends no motion. Both MoveIt parameters and controller-manager hardware classes must prove exclusively `mock_components/GenericSystem` before any task execution. The baseline requires no existing attachment and an open Robotiq gripper. The fixture requires no dimension entry.

Stop/relaunch this isolated fake scene before another observation/run: duplicate runtime IDs fail closed, preserving placed/held object state for recovery. Do not run an EPD publisher concurrently. A post-attachment failure retains the attachment; it does not delete or teleport the held object.

## Planning and status

Stage events stream as JSON to stdout; the final JSON contains `failed_stage`, exact reason, candidate contact fractions/diagnostics, actual execution results, and attachment verification. The executor requires complete Cartesian contact before approach execution, plans gripper closure using the existing `gripper` group, checks measured fingertip-only contact, attaches with MoveIt's native world consumption, plans the held-object lift/transfer/place, plans opening, detaches at achieved FK, retreats and returns to canonical home.

Contact allowances are restricted to the selected object and the generated allowed fingertip links, then restored. Table, bin, distractor, palm and arm collisions remain active. Collision-disabled IK is used only to diagnose rejected contact goals and is never sent for execution.

The initial gripper implementation is explicitly the supported Robotiq 2F-85 baseline: `gripper_finger1_joint`, group `gripper`, range 0–0.804 rad. It searches for the first allowed fingertip contact in 80 increments and requires a MoveIt plan before sending the command. This is geometric fake-hardware contact, not force/physics simulation or a physical grasp guarantee.

Limitations: only world-frame box bounds; one selected target per request; no alternate-object retry if all candidates for that target fail; transfer/place/retreat are planned sequentially, not proven for every candidate before the first motion. A late failure stops the sequence and is not reported as success. There is no operator action server, concurrent-task arbiter or live stream adapter yet.

## Observed evidence (2026-09-12)

`scenes/ur5_2f_test/acceptance/r1_4_replay_runtime.json` records a headless Humble/MoveIt run on isolated domain 164, with both hardware components verified as mock. The complete sequence returned `PASS`: approach, contact, close, attachment, held-object lift, transfer, place, open, detachment, retreat and home. The achieved object centre was [0.449612, 0.220413, 0.129355] m versus zone goal [0.45, 0.22, 0.13] m. A separate final scene query confirmed unchanged bottle pose/dimensions, table/bin presence, one placed cup and no attachments. The launch was stopped afterwards.

Earlier fixture placement at [0.32, -0.12, 0.051] m failed all contact candidates against the bin and sent no trajectory. Moving only the runtime observations within the existing zone resolved that obstruction. The first execution then exposed redundant world removal during attachment; using the existing attach-only helper resolved it. No collision geometry or broad collision allowances were changed.

This is evidence for the deterministic replay fixture and current dirty-worktree generated scene (hashes are recorded), not live perception, physical grasp reliability, GUI acceptance, or full preplanning of the entire cycle. The current generated destination region is used as-is; this work does not reconcile pre-existing authored bin/zone edits. Runtime emitted warnings about unavailable FIFO scheduling and unspecified acceleration limits using MoveIt's default. No physical readiness claim is made.

Focused validation: 55 tests passed across `test_runtime_pick_inputs.py`, `test_perceived_object_grasp_execute.py`, and `test_perceived_object_grasp_plan.py`; Python byte-compilation and `git diff --check` passed. No colcon rebuild or GUI test was run; Python support scripts were exercised directly against the existing built scene/MoveIt packages.
