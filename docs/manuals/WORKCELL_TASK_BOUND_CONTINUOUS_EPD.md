# Feature 3B3 — Task-bound continuous EPD -> MoveIt PlanningScene

## Product change

The existing production-oriented
`scripts/epd_dynamic_planning_scene_node.py` already has continuous ROS
subscription, live observation handling, collision-object ADD/update/remove,
TF, ownership suppression, freshness timeouts and PlanningScene verification.
**No replacement detector or second continuous bridge is created.**

This slice adds an **opt-in saved Workcell Studio TaskIntent binding** to that
existing bridge. The bridge receives the EPD topic, message type, optical frame,
camera identity, pick class, pick zone, freshness/confidence rules and
planning frame from the selected scene, **not from legacy fixture defaults**.
Its preflight refuses mismatched generated tasks, stale class/zone/target
bindings, wrong camera identity/frame/topic, missing scene metadata or a task
that enables real hardware. It also blocks duplicate/out-of-order, wrong-frame,
missing-stamp and stale EPD messages against the receiving node's ROS clock.

The node still does **not command robot motion**, launch cameras, start EPD,
start controllers or start MoveIt. It only applies and verifies dynamic MoveIt
world collision objects to an **already running** ROS graph, when explicitly
started.

## Preflight, then explicit monitor start

With the ROS Humble environment sourced and the scene saved/generated:

```bash
python3 scripts/epd_dynamic_planning_scene_node.py \
  --task-intent-scene /absolute/path/to/new_cell \
  --preflight-only
```

A successful preflight reports `PREFLIGHT_CONFIGURED_ONLY`,
`source_identity_attested: false`, `planning_ready: false` and
`robot_motion_commanded: false`. This tests configuration parity only. It
does **not** require or start ROS, EPD, a camera or a robot.

After independently establishing **fake robot hardware and plan-only MoveIt**
on an isolated/tested ROS domain, and separately verifying camera/EPD and TF:

```bash
python3 scripts/epd_dynamic_planning_scene_node.py \
  --task-intent-scene /absolute/path/to/new_cell \
  --summary-output /tmp/workcell_live_scene_summary.json \
  --snapshot-output /tmp/workcell_live_scene_snapshot.json
```

The runtime will subscribe to the **authored EPD topic and message type**.
The selected camera binding is from TaskIntent v2; the received ROS source is
checked against the authored optical frame. Topic/frame agreement is *not*
independent hardware attestation; no claim is made that a real Intel D435i was
identified or that a Gazebo RGB-D sensor was commissioned.

The live stream writes a diagnostic summary and normalized eligible-target
snapshot. For task-bound mode, any correctly measured non-target class remains
a **collision obstacle in MoveIt**, never a pick candidate: requesting cups
must not make a detected bottle disappear from the PlanningScene. A missing
observed shape/dimensions or failed collision scene update blocks planning
readiness; no dimensions or object poses are invented.

## Staleness and safety policy

- ROS sensor headers must contain the **authored optical frame** and a
  positive native acquisition stamp, newer than the last applied stamp.
- Freshness is assessed in the node's **ROS clock** against the saved task
  `max_age_seconds`; future/stale frames block before object application.
  If Gazebo uses `/clock`, explicitly configure `use_sim_time` consistently
  across sensor, TF, EPD and bridge nodes. No stamp rewriting.
- Generated `cell_definition.yaml` and saved TaskIntent must agree on object
  class, confidence, max age, pick region and placement region. Run Generate
  after editing task/camera configuration; mismatches block.
- Task-bound source requires `real_hardware_enabled: false` and
  `require_fake_hardware: true` in saved TaskIntent. **The bridge does not
  independently attest the actual ROS controller hardware.** The operator
  must verify mock/fake controllers before running the monitor.
- Manual/legacy non-task-bound CLI behaviour is preserved for existing
  scoped tests; its fixture defaults are **not** used in task-bound mode.
- Summary `READY` means a current observed candidate was applied and
  verified in MoveIt. It does **not** mean a grasp plan succeeded, collision
  accuracy was validated or a physical pick cycle passed. The summary's
  `planning_ready` remains false.

## Tests and limitations

Focused tests:

```bash
python3 -m pytest -q \
  tests/test_continuous_epd_task_binding.py \
  tests/test_epd_dynamic_planning_scene_node.py \
  tests/test_capture_selected_live_epd.py
```

**No actual ROS/Qt/Gazebo/D435i workstation run has occurred for this branch.**
The current implementation only adds task-bound continuous use of an existing
bridge. It does not include a new GUI start/stop panel, automatic system bring-up,
full-cycle MoveIt planning, source authentication, EPD segmentation improvement,
physics grasp retention or robot actuation. Stage-A and EPD qualification gates
remain open. Do not merge or advertise physical readiness without actual test
and trace evidence.
