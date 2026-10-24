# Feature 4A — EPD-independent multi-object commissioning queue

## Purpose

Workcell Builder's existing **Task → Object input** adds **Preview multi-object
task queue (no motion)**. A beginner can review a saved task against a
previously recorded, explicit `detected_objects/v1` commissioning fixture:
which objects match the target class/colour, confidence, freshness and
full oriented pick-zone containment; which non-target objects remain obstacles;
and the deterministic processing order and saved destination for eligible items.

This uses the **existing** TaskIntent v2 file, physical scene and shared
`runtime_pick_inputs.normalize` + `task_intent_resolver.select_observations`.
No third authoring GUI, EPD dependency, camera launch, ROS startup, new
geometry authority or silent replay file is introduced.

The preview is a **read-only eligibility queue**. It is not a MoveIt plan,
does not test robot reachability or collision-free trajectories, does not
assert that the simulator is connected, does not command a gripper, and
does not run physics-backed lift/transfer/release. These require independent
acceptance, including the still-blocked Stage-A grasp-retention work.

## Operation

1. Open a saved Workcell Studio cell, select the pick class and zone, physical
   destination, and grasp strategy/policy.
2. In **Task → Object input**, select **Off — commissioning objects** or
   **Recorded / replayed observations**. Set **Observation snapshot** to
   an *existing* `detected_objects/v1` commissioning recording, containing
   explicit world-frame object poses and strictly positive measured/known
   dimensions. The recording must declare `source.mode: replayed_snapshot`.
   No workpiece box will be generated from a scene asset.
3. Click **Save task**. Then click **Preview multi-object task queue (no motion)**.
4. Review the ordered candidate list, destination region, and reasons other
   objects were excluded. Every observed object remains in the report for
   later collision checks; the preview does not yet apply those objects to
   MoveIt.

A saved task with **Live** perception is refused. A missing recorded input is
not replaced by the repository's historical fallback fixture. Recording
observations are restamped only by the existing **explicit replay acquisition**
function and labelled `explicit_commissioning_replay`; no result is represented
as live perception or current Gazebo physics.

## Headless CLI

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 scripts/simulation_batch_preview.py \
  --scene scenes/ur5_2f_test --limit 20
```

This requires an explicit saved TaskIntent v2 replay binding in that scene.
It outputs a JSON report on stdout; no files are written.

Reports distinguish `ELIGIBILITY_ONLY` from `BLOCKED`, include the saved
TaskIntent normalized SHA-256 and raw input-file SHA-256, keep a deterministic
`queue` ordered by the shared resolver, and report all `observed_objects`.
Each queued object is `ELIGIBLE_NOT_PLANNED`; every object has
`planning_scene_verified: false`. `moveit_planning_attempted`,
`physical_grasp_verified`, `simulator_live_connected`, `epd_used` and
`motion_commanded` are all false.

Invalid schema, missing file, malformed frame or dimensions, duplicate IDs,
future source timestamps, unsupported/unsafe task mode, missing validated task,
and unsafe real-hardware selections are blockers. Queue length must be 1–100;
a limit truncates only the **planned order**, not the observation/obstacle
report. A task with no eligible objects returns `BLOCKED` without inventing
a target. The preview never marks the original task saved or ready for motion.

## Validation and follow-on

Focused tests:

```bash
python3 -m pytest -q tests/test_simulation_batch_preview.py
# On Ubuntu ROS Humble workstation:
colcon build --symlink-install --packages-select workcell_builder
# Run the existing task_intent_editor_test target for GUI gating.
```

**Workstation status:** Tests were authored but have not yet been executed
in the GitHub-only development session. No ROS, Gazebo, MoveIt or Qt runtime
acceptance is claimed.

**Next practical implementation after this slice:** feed the same task queue
with genuine, timestamped **simulator-owned object poses** using the
independently developed Stage-A simulator observation adapter, then bind each
candidate to fresh measured obstacles and the existing collision-aware
MoveIt **plan-only** cycle evaluator, invalidating downstream queue entries
whenever the simulated world changes. Only after true Stage-A physics
acceptance may an Operator HMI offer real simulation-execution controls.
