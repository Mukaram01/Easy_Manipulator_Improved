# Feature 3B2 — Capture selected live EPD observations into a saved TaskIntent draft

## Goal and authority

Workcell Builder's existing **Task → Object input** can now request one
bounded, no-motion EPD localization/tracking capture from the selected
`simulated_rgbd` or `realsense_d435i` source.

The source remains the existing saved
`config/workcell_builder_task_intent.yaml`; the configured optical frame,
camera identity and EPD topic/message type remain from `environment.yaml`.
There is **no third GUI**, no duplicate detector, no default camera topic,
no separate object selection policy and no fake geometry.

The Task tab adds **Capture live EPD objects (no robot motion)**. It is disabled
until Live mode and a camera are selected **and saved**. This does not launch
Gazebo, a D435i driver, EPD, MoveIt or a robot; those upstream components
must already be operational.

A valid capture creates a **new file** under
`<scene>/captures/epd/live_*/qualified_detected_objects.yaml`. Existing
replays, prior captures, authored geometry and task files are not overwritten.
The UI adopts the candidate path into an **unsaved** TaskIntent draft;
review the observations, then explicitly **Save Task**. The existing
`planning_input` and TaskIntent v2 selection/zone rules still decide
eligibility. A successful snapshot does **not** enable motion.

The generated capture result retains:

- the original EPD acquisition stamp and class/track identity, without
  retiming or reviving frozen/replayed input;
- EPD-reported centroid/pose and **observed positive dimensions**, never
  scene-derived workpiece boxes;
- a verified historical TF transformation into `world`, or a hard blocker;
- the selected camera-source binding plus
  `camera_source_verification: configured_ros_topic_only_not_hardware_attested`.

The source label describes the **selected ROS transport**, not cryptographic
sensor/hardware provenance. ROS image topics and EPD headers by themselves
cannot prove the publisher is a genuine physical D435i. No commissioning or
physical grasp claim is made.

## Blocking rules

Captures block without modifying the task if any of the following apply:
missing saved Live source; missing authored EPD topic or unsupported EPD
message type; EPD unavailable; no matching target; no actual acquisition stamp;
missing or mismatched optical frame; TF not verified at the source stamp;
missing or nonpositive observed dimensions for **any** object; future/stale
observation; missing configured positive freshness limit; invalid pick zone,
class or grasp input. Existing non-target objects remain eligible as collision
obstacles and are not filtered from the snapshot.

`perception.replay_path` is reused for both replay and live filenames for
backward compatibility; the authoritative mode remains `perception.mode`.
The new helper refuses to reclassify replay input as live.

**Simulation time limitation:** Gazebo may publish stamps in ROS simulated
clock time rather than Unix wall time. Current `planning_input` evaluates
freshness with Unix wall time, so simulated-clock snapshots can be blocked
even when camera TF is correct. Do not rewrite the stamp or weaken freshness
to force acceptance. Qualifying an explicit shared ROS-time authority is a
subsequent feature, not a waiver in this PR.

## Headless operation

With ROS Humble sourced and upstream camera + EPD already running:

```bash
python3 scripts/capture_selected_live_epd.py \
  --scene scenes/ur5_2f_test \
  --timeout 10
```

The saved scene must have Live perception with one of the explicit camera
sources and a configured EPD localization/tracking topic. The helper prints
one JSON status. Success returns
`CAPTURE_QUALIFIED_FOR_TASK_INPUT` with `capture_path`. Failure returns
`BLOCKED` with an actionable message. Both report
`planning_ready: false` and `robot_motion_commanded: false`. Actual
MoveIt full-cycle planning remains a separate existing action.

## Tests, evidence, remaining work

Pure-Python regression tests:
`python3 -m pytest -q tests/test_capture_selected_live_epd.py`
plus `tests/test_task_intent_authoring.py` and
`tests/test_workcell_camera_probe.py`.
Existing Qt TaskIntentEditor test now covers the new button's saved/unsaved
enablement.

No ROS/Qt workstation acceptance has yet been performed for this branch.
Before merge: test both simulated and D435i ROS topics, EPD localization
stamp, camera-to-world TF at the original stamp, observed geometry, no input
fallback, class filtering and Save/reopen, then MoveIt **plan-only** separately.
Gazebo simulation-clock interoperability, EPD segmentation/geometry
qualification and simulator physical-grasp retention remain independent
blockers. Do not mark them complete based on this code.
