# Workcell Builder Class Routing

Class labels come from EPD snapshot/live bridge metadata. Routing maps class_label to place_zone and destination for preview-only task intent and planning readiness artifacts. Configure unknown/default route to reject bin. Safety constraints: no robot motion, no MoveIt execution, no gripper command, no conveyor command, no real hardware command.

## Saved-task class routing (TaskIntent v2, R2.4)

A saved Pick & Place task can send each observed class to an authored placement region. Any number of classes and regions is supported, and many classes may share a region. The existing resolver, MoveIt planning and executor are used; there is no separate planner.

### Authoring
In **Task > Where > Route by class**, edit the routing table:
- **Add rule / Delete rule:** add or remove rows.
- **Columns:** Object class, Destination region, Enabled, Priority.
  - The class column offers the labels of the workspace's registered EPD runtime (`workcell_perception_runtime.json` → `epd.labels`).
  - The destination column offers the cell's own placement regions.
- **Unrouted objects:** what happens to a class without an enabled rule.
  - *Leave in place* (not picked, kept as an obstacle);
  - *Stop the sorting session*.
- **Attempts per object:** 1–5.
- **No rows** means direct pick and place into the Placement region.

The saved task then carries:

```yaml
routing:
  mode: class
  unrouted_policy: leave_in_place        # or stop_session
  max_attempts_per_object: 2
  rules:
  - {id: route_1, when: {attribute: class, equals: cup}, destination: default_drop_zone, priority: 10}
  - {id: route_2, when: {attribute: class, equals: bottle}, destination: default_drop_zone}
  - {id: route_3, when: {attribute: class, equals: mouse}, destination: place_zone_b_left}
  - {id: route_4, when: {attribute: class, equals: remote}, destination: place_zone_b_right, enabled: false}
```

These are blocking task errors:
- **Rules:**
  - two rules for one class (`ROUTING_CLASS_AMBIGUOUS`);
  - a rule without a destination (`ROUTING_DESTINATION_MISSING`);
  - a default rule, or any condition other than `attribute: class` (`ROUTING_CONDITION_UNSUPPORTED`);
  - no enabled rule (`ROUTING_RULES_REQUIRED`).
- **Policies:**
  - a missing unrouted policy (`ROUTING_UNROUTED_POLICY_REQUIRED`);
  - an attempts value outside 1–5 (`ROUTING_ATTEMPTS_INVALID`).
- **Conflicts with the rest of the task:**
  - a set **Object class** (`ROUTING_CLASS_FILTER_CONFLICT`). The routes decide what is picked.
  - a routed class that is also excluded (`ROUTING_CLASS_EXCLUDED`);
  - a placement policy other than AUTO (`ROUTING_PLACEMENT_POLICY`);
  - a **Placement region** that is not the destination of an enabled rule (`ROUTING_PRIMARY_DESTINATION`);
  - a saved multi-object sequence (`ROUTING_SEQUENCE_UNSUPPORTED`).
- **Destinations and classes:**
  - every rule's region, including disabled rules, must resolve inside its physical target, and that target must have enabled collision geometry (`TASK_ROUTE_DESTINATION_INVALID`);
  - a class outside the registered EPD vocabulary (`ROUTING_CLASS_NOT_IN_VOCABULARY`). Without a registered EPD runtime, the vocabulary is reported as unknown, never guessed.

### What planning does
- **Selection.** Only observations of a class with an enabled rule are pick candidates. Higher priority is picked first, then the resolver's existing confidence, height and identity order. The class is the one EPD emitted in the observation, never one derived from authored geometry.
  - A class without a rule is `CLASS_NOT_ROUTED`; a class whose rule is disabled is `CLASS_ROUTE_DISABLED`.
  - Either way the object is not picked and stays in the planning scene as an obstacle.
- **Fit.** Each object is checked against its own class's destination before planning. An object that does not fit is refused (`PLACE_OBJECT_DOES_NOT_FIT`). It is never sent to another bin.
- **Occupancy.** In a sorting cycle the resolver also receives every other object in the cell.
  - Places in the routed region are packed from the region corner around objects already there, and up to three are planned.
  - A region with no free place is `DESTINATION_FULL`.
  - The considered objects are part of the resolution identity. Revalidation refuses a place that became occupied (`DESTINATION_OCCUPIED`).
- **Route identity.** The resolution records the selected route and place (`place_resolution.route`) and all routes (`place_resolution.routes`). A handoff whose destination is not the object's route is refused (`TASK_ROUTE_CHANGED`).
- **Generate.** Generate exports each routed region once, and the enabled routed classes as the perception adapter's required classes.

## Sorting sessions (R2.4)
A sorting session sorts the saved cell continuously, one object per cycle, on **MoveIt fake hardware**.
- **From Workcell Builder:** Plan / Simulate → **Start Sorting (Fake Hardware)**, **Pause**, **Resume**, **Stop at Cycle End**.
- **From the Operator HMI:** the **Sorting · fake hardware** card has **Start Sorting**, **Pause**, **Resume** and **Stop**. Workcell Builder owns and runs the session.

### Lifecycle
`IDLE → READY → RUNNING ⇄ PAUSING/PAUSED → STOPPING → STOPPED`; any active state → `FAULT`.
- **Safe boundary.** Pause and Stop take effect at the next cycle boundary: robot home, nothing held. They are software requests, **not an emergency stop**.
- **Hard cancel.** The existing **Stop Simulation** cancels immediately through the supervised cleanup. The session then ends in FAULT (`SESSION_INTERRUPTED`), because a cancelled cycle's world is not verified.

### Each cycle
1. **Observe:**
   - *Batch replay:* the replayed frame is consumed once. Later cycles observe the world reconciled from execution evidence: completed objects at their verified placed pose. The old frame is never read again.
   - *Continuous:* each cycle ingests a new live frame. A frame that still shows a processed object at its pick place is rejected as stale (`PROCESSED_OBJECT_STILL_OBSERVED`), never re-picked.
2. **Select** one object deterministically, using the resolver's eligibility, routes, priority and order.
   - Unrouted and disabled classes follow the unrouted policy.
   - An object that cannot fit its route is rejected before planning.
   - An object whose destination is full waits.
3. **Resolve, plan and execute** that object only. This is one run of the existing executor (`--sorting-cycle`) with every other object as an obstacle and the destinations' occupancy.
4. **Count** the object COMPLETED only from that run's state-verified evidence: all nine stages planned and executed with MoveIt success, controller terminal success, exclusive mock hardware, the object detached once in the world, its final pose contained in the routed destination, and home reached.
5. **Retry or reject:**
   - A failure before motion is retried up to the attempts limit.
   - Deterministic refusals (fit, aperture, tool) are rejected at once.
   - Unverified motion faults the session (`CYCLE_EXECUTION_UNVERIFIED`).
6. **Continue** until no further object can be sorted (`BATCH_COMPLETE`), or the operator stops.

### Counts and evidence
- **Counts:** completed, failed, skipped (unrouted or disabled), waiting (destination full) and pending, per class and per destination.
- **Shown in:** both the Plan / Simulate monitor and the HMI.
- **Evidence:** the session report (`sorting_session.json`) and every cycle's observations and executor evidence are under `log/studio_sorting/<session>/`.

### Simulator-rendered EPD observations (R2.5A)
A routed fake-hardware task can be observed by its own sorting session instead of a saved batch. In **Task > Perception**, choose **Live EPD** with the camera **Simulated RGB-D (Gazebo + EPD)** and leave the observation snapshot empty. The cell needs authored physical test objects (box bodies with declared visual meshes, class `unclassified`) and an enabled cell camera.

**Each frame.** Before the first cycle, after every cycle and after a pause, the session observes again (`perception_replay.py session-frame`):
1. **Render.** The cell camera renders the session's current world: the authored objects at rest, then each completed object at its state-verified final pose in its bin.
   - The render-only world (`simulator_world.derive_render_world`) has static models, every routed destination container and no robot.
   - It is renamed, so no physics entry point can run it.
2. **Classify.** EPD's production node classifies the rendered frames (replay mode, CPU).
3. **Bind.** Each EPD world centroid binds to the one object box that contains it.
   - An object takes EPD's class and confidence only from exactly one detection; otherwise it is `unperceived`.
   - Detections matching no object, or several, are reported and never used.
   - Identity, pose and size are always the world's.

The frame is labelled `SIMULATOR_RENDERED_SYNTHETIC`. It carries its real capture time and the world revision it shows, and the cycle's executor receives it unchanged.

**What the session accepts:**
- **World revision.** A frame must name the session's current world revision and reproduce every known object (`FRAME_WORLD_MISMATCH`).
- **Unclassified objects.** An object EPD does not classify in the current frame is not eligible and stays an obstacle. It is never subject to the unrouted policy, and it becomes eligible if a later frame classifies it.
- **Changing classes.** An object whose class changes between frames is `CLASS_UNSTABLE` and is never picked.
- **Finished objects.** A completed or rejected object is never re-decided by the class EPD now reports for it.

**Freshness.** Rendered frames have their own producer, `simulator_render`. It is admitted only for the simulated camera on fake hardware.
- **Ceiling:** 300 s from capture to execution, covering render, CPU inference and the certified search.
- **Timestamps:** never renewed.
- **At the execution boundary:** the executor verifies that the live PlanningScene still reproduces the rendered world, then applies the ceiling to the true capture time.
- **Live sources unchanged:** a live capture keeps the 2 s live ceiling, and a RealSense task never admits this producer.

**Limits.** These frames are synthetic renderings of the fake-hardware world.
- **No real camera:** this is not a real-camera observation, and the robot is not rendered.
- **Positions:** objects move only through verified fake-hardware cycles.

### Fake-hardware observation refresh
For a replayed batch (not a rendered frame): a cycle's certified search outlives the 2 s replay age limit. On fake hardware only, observation time is renewed at the execution boundary when the live PlanningScene reproduces every observed identity and geometry exactly (`fake_world_refresh`). The PlanningScene is the fake backend's whole world. The unchanged age limit is then applied; any difference is `SORTING_WORLD_CHANGED`.

### Physics
A sorting session or a routed task is refused in Gazebo:
- `TASK_ROUTING_PHYSICS_UNQUALIFIED` or `SORTING_PHYSICS_UNQUALIFIED`;
- the reason comes from `simulator_world.physics_task_qualification`.

The qualified Stage-A world covers one object, one destination and one cycle. That scope is fixed in code: no catalog or other editable file can widen it, because the catalog is not bound to qualification evidence. A future multi-destination or continuous-sorting qualification must provide a scope verified against its own bound contact, retention and placement evidence; no such verifier exists yet, so routed and multi-destination physics fail closed. The sorting engine needs no change when one is added.

**Physics sorting ledger (G4, offline; motion not admitted).**
- **The switch:** `SortingSession(..., backend='simulator', physical_destinations=, part_dimensions=)` counts a cycle only from contact-backed evidence.
- **The checks:** `verify_physical_cycle`, which is the per-cycle judgement that qualified G3C (`physics_scope_qualification.physical_cycle_problems`), plus three session checks:
  - the session's target;
  - the route class;
  - the part measured, detached and reconciled in the scene.
- **Measured reconciliation:** a completed object is reconciled at its measured Gazebo final pose (`measured: true`). Occupancy, and so the next cycle's place, uses that pose.
- **No substitution:** fake-hardware evidence never counts on the physics backend, and physics evidence never counts on the fake one.
- **Still refused:** a sorting session or routed task through the Builder or HMI (`SORTING_PHYSICS_UNQUALIFIED`), and any G4 motion outside a CLI bounded trial (below). G3C qualifies one full-cycle per exact cell, not continuous sorting.

**G4 single-launch sorting runner (`scripts/run_g4_sorting.py`: plan-only, or a CLI-only bounded trial).**
- **The cell:** a class-routed task saved with the simulator backend and the simulated RGB-D camera (Task Editor: perception live EPD, simulated camera, routing). Its source type is `rendered_perception_in_simulation`: classes come from EPD on rendered frames, bound to the objects Gazebo measures (the Stage-B binding, its replay ceiling unchanged). Any other cell is refused (`G4_CELL_UNSUPPORTED`).
- **One launch:** Gazebo Fortress + MoveIt with every authored bin as a hollow physics bin (`physical_destinations='all'`) for the whole session.
- **Each cycle, in order:**
  1. **Observe:** measure every object; the measured world must be the session's verified world (1e-5 m/rad, the refresh tolerance), otherwise `G4_WORLD_CHANGED`;
  2. **Classify:** render exactly that world with the cell camera, classify it with EPD (`perception_replay.py session-frame`), bind the classes to the measured objects (`simulator_observations.py --perception`); a frame of another world revision is refused (`FRAME_WORLD_MISMATCH`);
  3. **Select:** the ledger (`backend='simulator'`) picks the next routed object; unclassified, unrouted, completed and removed objects are never targets and stay obstacles;
  4. **Plan:** a padded plan-only resolve of that target only (`--target-object-id`), every other object and both bins as obstacles, every other measured object as destination occupancy (the next place moves aside from an object already in the bin); the plan must be the target's own route (`G4_PLAN_MISMATCH`). A refused plan is a motionless ledger attempt (rejected per the job policy; the object stays an obstacle) and the session observes again;
  5. **Validate:** re-measure (geometry unchanged since classification); the routed bin must hold exactly what the ledger expects (`G4_OCCUPANCY_MISMATCH`);
  6. **Authorise:** `g4_authorise_cycle` under the session record;
  7. **Execute:** the executor runs exactly the authorised plan (`--g4-session/--g4-cycle/--g4-plan`, below);
  8. **Verify:** `assert_gate` and `g4_verify_cycle` (the executor summary file), before the ledger may count it;
  9. **Reconcile:** a fresh Gazebo measurement must show every other object unmoved (1e-5) and the placed part within 1 mm of the executor's final measurement; only then the ledger counts the cycle, at the re-measured pose (an unreconciled cycle faults the session uncounted); then Observe again.
- **Plan-only (default):** the session record is `G4_PLAN_ONLY`, so step 6 refuses (`G4_SCOPE_PLAN_ONLY`) and nothing moves. `--plan-every-target` observes again and plans each further routed object once in the same launch; a plan-only cycle is never a ledger attempt or a count; a refused plan is a motionless ledger attempt and the session observes again.
- **Bounded trial (CLI only, never Builder/HMI):** `--authorise-g4-trial --prior-evidence <the registered Stage-A acceptance> --max-cycles 1..2`. The runner writes `G4_AUTHORISED_TRIAL` (bound to this runner process) and authorises at most two cycles in this one launch. Any refused plan, refused or failed execution, executor timeout, unverifiable or unreconciled result, world change found by Observe or Reconcile, or occupancy mismatch ends the session as a FAULT (exit 5); a failed or timed-out helper step (measure, render, plan-only resolve, the Validate refresh, which is where a world change between classification and validation surfaces) ends it BLOCKED (exit 1). Nothing is retried. Evidence is labelled `G4_TRIAL_NOT_QUALIFIED`.
- **Software:** every G4 session, plan-only included, needs unmodified planner and G4 control sources. The runner takes the G4 control identity of the code it loaded at start and refuses when HEAD no longer holds it (`G4_SOFTWARE_CHANGED`).
- **Stopping:** SIGUSR1 to the runner process latches stop at cycle end (G5B): a running cycle finishes under every guard and is verified; a latch before authorisation or execution ends STOPPED, never FAULT. SIGTERM/SIGINT cancel and are forwarded to the running executor (G5A). Control-file pause/stop apply at cycle boundaries. Exit codes: 0 PASS, 1 BLOCKED, 3 CANCELLED, 4 STOPPED, 5 FAULT; a failed shutdown never hides a FAULT or CANCELLED. The G5 stop/cancel primitives are reused unchanged (mechanism identity equal to G3C's); the G4 runner's own stop boundaries and exit mapping are new and have no G4 acceptance yet.
- **Evidence:** `g4-sorting-report.json` (every cycle's steps), `sorting_session.json` (the ledger), `session/g4-session.json`, and per cycle `session/cycles/NNN/` (measurement, world, rendered frame with EPD's output, bound observations, resolve summary, validation).

**G4 continuous-sorting authority (`physics_scope_qualification`, `workcell_g4_sorting_scope/v1`, offline).**
- **Session record:** `g4_session_binding` binds one exact routed multi-object cell: its files, the derived world (every object the qualified body), every bin binding, the support, robot identities, routing rules and job policy, the simulator-rendered EPD source (producer, source type, EPD runtime record, frame age limit), safeguards (contact guard, strict retention, pads, stop/cancel), stop conditions, native/runtime/planner identity, the G4 control identity (git blobs of `G4_CONTROL_SOURCES`: this authority, the runner, the ledger and the G5 runner primitives; outside the G3C identity), `max_cycles` (at most one per object) and the session receipt.
- **States:** `g4_open_session` writes `G4_PLAN_ONLY`. `g4_authorise_trial` (CLI runner only) writes `G4_AUTHORISED_TRIAL` once: `G4_TRIAL_OPERATION` fixed in code (at most 2 cycles, `qualified: False`), the same-build Stage-A prerequisite evidence, and the issuing runner process (pid, kernel start time, `run_g4_sorting.py --authorise-g4-trial` in its command line; only that process may write it). This is process lineage on one host, not authentication. Every use recomputes the binding from the current files and checks the live issuer: a record copied, hand-written or replayed elsewhere is refused (`G4_SCOPE_ISSUER_MISMATCH`, `G4_SCOPE_MISMATCH: <key>`). `G4_AUTHORISED_SESSION` (continuous production) would need a G4 acceptance verifier; none issues it, so a record claiming that state is refused (`G4_SCOPE_NOT_QUALIFIED`).
- **Per cycle:** `g4_authorise_cycle` re-reads the rendered frame (hash, age 0..300 s), the validated observations (this session's receipt, classes bound to that frame), the plan-only summary (target, route, routed bin, physics pads, resolution) and the measured occupancy of the routed bin, and re-verifies the previous cycle (`g4_verify_cycle`, same session). It refuses: plan-only sessions, a latched stop, the session limit, a broken chain, stale or foreign observations, a class/destination pair no enabled rule routes, an unknown target, a plan that is not the target's, an occupancy mismatch, and a planned grasp without contact evidence on the qualified body (`G4_GRASP_UNQUALIFIED`: only `top_2f`, the grasp of every counted G3C trial). A cycle number is issued at most once (`G4_CYCLE_ALREADY_ISSUED`; the marker holds the record digest), so a failed cycle blocks every later one.
- **Executor admission:** with `--g4-*` the executor accepts only one strict simulator full-cycle of `--target-object-id` (never `--resolve-task`, `--replay` or `--physics-scope`). Before planning, `g4_admit_execution` verifies the session (recomputed from the current files, `max_cycles` read from the record under the fixed limit of 2; live issuer), the exact record and its issued digest, the predecessor's consumption, the grasp and route, the target, unchanged observations, plan and frame, and the routed bin; the authorised resolution is consumed unchanged: no new candidate or place search; MoveIt re-plans the bound candidate's motion in a revalidate pass and those trajectories execute; a diverging result is refused (`TASK_RESOLUTION_DIVERGED`, a resolver BLOCKED code, or the final `G4_PLAN_DIVERGED` check). Without an admitted multi-destination trial cycle the run refuses (`G4_ADMISSION_MISSING`). At the motion boundary the session is re-verified (binding and live issuer), the session and cycle record digests and the frame re-checked, and the cycle consumed once (`G4_CYCLE_REPLAYED`); the plan, observations, issued marker, chain, grasp and route are checked at admission. The contact guard is armed on the routed bin's bound floor; the G5 controller audit, hold proof and transfer-cancel verdict stay active.
- **Counting:** `g4_verify_cycle` accepts a cycle only from the G3C per-cycle judgement (`physical_cycle_problems`) on the record's routed bin, every bottom corner on that floor, the authorised target, grasp and resolution, measured, detached and reconciled, executed under this record and session.
- **Qualification contract (`G4_QUALIFICATION_CONTRACT`; offline, recomputed from raw evidence; issues and admits nothing — `G4_AUTHORISED_SESSION` stays refused, continuous production is G6):** `g4_qualification_readiness` is QUALIFIED_READY only for 3 STRICT_PASS two-cycle sessions + one G4-S + one G4-C of the exact cell, all at one planner identity and one G4 control identity, the workspace's native build, runtime and rendered-EPD perception, one shared binding (geometry, robot, routing, perception, safeguards, stop conditions) and five distinct sessions.
  - **Session (`g4_session_evidence`):** `g4_trial_evidence` (the run-level judge: STRICT_PASS, INCOMPLETE, CANCELLED, PRE_MOTION_FAILURE, FAILED, NOT_A_TRIAL) plus, from raw files: the reporting runner issued the record and its commit and control code never changed; the registry holds exactly the issued/consumed markers; the authorised frame, observations and plan are the kept files; previous reconcile < measure < render < authorise < consume < first motion < reconcile; the frame within its age; the target classified as routed in its own frame; nothing moved between cycles; the authorised occupancy is the measured content of the routed bin and the placed footprint overlaps no occupant; bystanders unmoved (the runner's own world metric) and the placed part where the executor measured it; zero unpermitted contacts; all 9 motion stages successful; every controller goal inside a cycle (7 arm goals each); no `[ERROR]` line except one late re-sent detach per cycle after its home stage; no motion in a plan-only resolve; numeric tracking (below). "Nothing moved" is cross-checked against the runner's execute step, executor stage events and controller goals.
  - **G4-S (`g4_stop_after_cycle_evidence`):** SIGUSR1 latched during cycle 1's `EXECUTE_TRANSFER`, never forwarded or superseded; cycle 1 strict and counted; STOPPED before any cycle 2 exists; clean shutdown.
  - **G4-C (`g4_cancel_evidence`):** SIGTERM during cycle 1's transfer, forwarded (not killing) to the executor; the executor summary bound to the authorised cycle (record and session digests, target, resolution, Stage-A prerequisites); `transfer_cancellation_acceptance` recomputed and equal to the recorded verdict (controller goal cancelled, measured stop, part still held, attachment kept, permissions revoked, recovery required, no motion after the cancel); zero unpermitted contacts; exactly the cycle's four arm goals and its gripper close, no `[ERROR]` line; authorised and consumed once and never counted; nothing further authorised; clean shutdown. G5A/G5B (G3C) are compatibility evidence only, never a substitute.
  - **Completeness:** every listed run lies under a production trial root and every authorised trial of the cell and planner identity there must be listed (unreadable evidence blocks); so must every runner session (`<run>/session/g4-session.json` with its launch log or cycle registry, even without a report: a killed runner) anywhere under the broad scan root; other records there (the deliberately forged negative-test probes) are reported as ignored. A session claiming plan-only is believed only without any trial sign (trial operation, issuer, evidence scope, cycle registry, controller goals). A run whose identity cannot be recovered and a FAILED exact-identity run block. A cancelled exact-identity run is judged where it was cancelled: before its cycle moved (reliability, or incomplete after verified cycles; contradicted by motion evidence: blocking), during a transfer (the G4-C judge of that cycle: accepted → incomplete, else blocking), during any other motion stage (blocking: G5A defines the transfer cancel only). CLI: `scripts/g4_qualification.py judge session|stop|cancel RUN` (a verdict only from the very planner and G4 control identity the trial bound, both committed and clean) and `readiness --cell … --session … --stop … --cancel … [--other …]` (roots fixed in code).
- **Numeric tracking (`scripts/g4_tracking_evidence.py`, required by the verifier, bound to this checkout):** the Cartesian deviation of the receipt-bound simulated fingertip boxes (Gazebo poses in the executor's own measurement stream, every sample, coverage of each controller goal checked) from the certified JTC spline of each executed arm stage (the trial's own `result 0` certificate, chained back from the executed home goal, unambiguous, point count equal to the executed plan, the certificate's own gripper state), path-space, max over every box corner. Free stages must stay within the planning pads' tracking allowance (`PHYSICS_PAD_ALLOWANCE_M`: 15 mm approach/transfer/home, 1 mm retreat); contact stages are reported and bound by the contact guard and strict retention. A cancelled cycle's stages are not tracking-checked (its own G5A acceptance and the contact guard judge it). Receipt, URDF and stream are bound to the trial record (session receipt digest, robot description digest, the executor's measurement run). The JTC has no configured path/goal tolerances, so no abort ever proved tracking; this is the tracking evidence. A trial refuses to start without its own absolute, new planning trace (`WORKCELL_PLANNING_TRACE_DIR`).
- **G3C stays narrow:** the G3C trial scope refuses routed or multi-object cells.
- **Envelope limits (plan-only sessions, 2026-10-30):** the qualified body (a 25 mm cube) carries one EPD class, "cup" (a mug visual filling the cube, camera ~0.3 m); no second class fits it. The qualified top grasp turns in world quarter-turns, so a part yawed more than 15 deg from a face-aligned pose has no qualified grasp (refused; an object-aligned pinch may plan but is never authorised). An object authored inside a bin is refused by the physics world derivation (objects start on the support).

**Multi-bin physics world, plan-only (R2.5B G0–G2).**
- **Deriving it:** `simulator_world.derive_world(scene, physical_destinations='all')` makes every authored target bin on the task's support a hollow static physics bin (`target_bin_physics`, `target_bin_physics_2`, … in asset-id order). Every task configuration of a cell therefore derives the same world.
- **Overlap:** overlapping bins are refused.
- **Binding:** the simulator launch binds each physics bin to the planning bin whose measured mesh floor lies under it (`destination_geometry_bindings`).
- **Planning:** the executor adds every bin's physics walls as planning obstacles and seats the placement on the task's own bin floor (a targeted cycle: its routed bin's floor).
- **Motion refused:** in such a world the executor refuses motion (`MULTI_DESTINATION_PHYSICS_UNQUALIFIED`) and the contact guard does not arm.
- **Runner:** it accepts `--physical-destinations all` only for `--mode targeted --gate resolve`. A class-routed task may derive this world too (plan-only; the G4 runner).
- **One-bin cells:** worlds, receipts and wall ids are unchanged.

**One authorised trial into one bin (G3A, `workcell_physics_scope/v1`).**
- **Default:** a multi-bin world stays plan-only.
- **The only exception:** the CLI qualification runner may authorise one guarded full-cycle into the task's bin:
  `run_stage_a1_finish.py --scene-package <cell> --mode targeted --gate full-cycle --physical-destinations all --authorise-physics-trial --prior-evidence <registered Stage-A acceptance>`.
- **When it is written:** the runner writes an `AUTHORISED_TRIAL` record after the session's plan-only resolve and revalidate pass and the geometry is re-measured.
- **What the record binds:**
  - the cell's authored and generated files;
  - the derived world (every launched bin and object model must equal it);
  - every bin binding and the selected bin;
  - the support binding;
  - the robot, URDF, controller and hold-profile identities;
  - the native libraries and bound sources, proved on disk;
  - the registered runtime and its Stage-A evidence;
  - the prerequisite-trial evidence;
  - the revalidated plan;
  - the session receipt;
  - an empty destination.
- **What it allows:** fixed in code as one full-cycle, one object, one destination, with no routing, no sorting and no real hardware.
- **Executor:** recomputes every binding before planning. It arms the contact guard on that bin's floor only, so every other bin and the selected bin's walls stay unpermitted contacts. It checks the record again before reporting a physical result.
- **Refusal:** any missing, changed or substituted binding, an occupied bin, or an unsupported state.
- **Earlier success:** not required. The prerequisite Stage-A trials are reused, from the same build, through `require_trial_evidence`.
- **Evidence label:** `PHYSICS_SCOPE_TRIAL_NOT_QUALIFIED`.
- **Not yet available:** a qualified scope that binds a strict trial PASS (and would admit the Builder or HMI) is not produced or accepted. The Builder and HMI never authorise a trial.

### Limits
- **Fake hardware only:** planning and kinematic execution on mock controllers. There is no contact, retention or settling evidence.
- **Continuous mode:** its only frame source is the session's own simulator-rendered EPD frame (R2.5A). There is no live camera source, so a live continuous session is still refused (`CONTINUOUS_SOURCE_UNAVAILABLE`).
- **Conveyors:** no conveyor tracking or production batching.

## Physics sorting, plan only, from Workcell Builder and the Operator HMI (G6)
The G4 runner's plan-only session is the Builder's and the HMI's sorting session for a physics cell. Nothing in this path can move the robot.

**Which cells.** A class-routed task saved with the physics simulator backend and live EPD on the simulated RGB-D camera, with no saved capture (Task Editor: Perception *Live*, camera *Simulated RGB-D (Gazebo + EPD)*, routing table, Run *Qualified physics simulation*). The cell must be saved, generated and validated.

**Starting it.**
- **Workcell Builder:** Plan / Simulate → **Start Sorting · Plan Only (Physics)**; **Pause**, **Resume** and **Stop at Cycle End** work as for fake-hardware sessions.
- **Operator HMI:** the **Sorting session** card → **Start Sorting · Plan Only**. The confirmation states the backend and that the robot never moves. Workcell Builder decides, owns and runs the session.

**What runs.** `run_g4_sorting.py --scene-package <cell> --output <log/studio_sorting/<id>> --domain <220..232> --plan-every-target --stream-status --control-file <id>.control.json`, in the workspace's qualified runtime environment.
- The command never carries `--authorise-g4-trial`, `--prior-evidence` or `--max-cycles`, so the session record is `G4_PLAN_ONLY`.
- Every cycle: measure the world in Gazebo, render it with the cell camera, classify the frame with EPD, select the next routed object, plan it with MoveIt (padded, plan-only resolve). The authorisation is then refused (`G4_SCOPE_PLAN_ONLY`), and the next routed object is planned.
- The session ends `BATCH_COMPLETE` when every routed object has a plan, or `STOPPED` at a cycle boundary after Stop. **Stop Simulation** cancels it (supervised cleanup).

**What is shown, and from where.**
- **Source:** `scripts/g4_session_view.py` reads the session's own evidence files and nothing else:
  - the ledger (`sorting_session.json`);
  - each cycle's rendered frame, EPD classes bound to the measured objects, and the plan-only resolve summary;
  - the runner report at the end.
- **Plan / Simulate monitor:** every object with its EPD class, confidence and decision (planned → region, grasp, cycle, *not executed*), and the evidence of motion:
  - granted cycle authorisations;
  - executed cycles;
  - controller goals in the launch log.
- **HMI:** the *Detected objects* table and the top view show the session's objects (Gazebo poses, EPD classes, plans). The *Sorting session* card lists each cycle's EPD result, plan and refused authorisation.
- **Counts:** the ledger's own. A plan is never counted as a sorted object: *completed* stays 0.
- **Viewers:** **Open RViz** and **Open Gazebo** attach to the session's receipt-bound simulator and MoveIt (read-only).

**Capabilities (Plan / Simulate "What this cell can do" and the HMI Production card).** Each row is the Builder's own gate for the selected cell:

| Capability | Status for a physics sorting cell |
|---|---|
| Physics simulation, one pick-and-place cycle | BLOCKED (`TASK_ROUTING_PHYSICS_UNQUALIFIED`); QUALIFIED only for a Stage-A scope cell |
| Physics sorting, plan only | EXPERIMENTAL (available) |
| Physics sorting with robot motion, bounded (Gazebo) | QUALIFIED only for a cell byte-identical to a G4-qualified cell (below); otherwise LOCKED with the reason |
| Continuous production sorting | LOCKED: G4 qualifies bounded sessions only |
| Sorting on MoveIt fake hardware | UNAVAILABLE (the task selects the physics simulator) |
| Real robot | UNAVAILABLE |

**Task Editor.** A session-observed task is *Authoring valid*. Plan Saved Task still needs a recorded input, and the editor says so instead of marking the task blocked.

## Bounded Gazebo sorting with robot motion (G6 admission)
A G4-qualified cell may run a **bounded** sorting session in which the simulated robot really picks, transports and places in Gazebo. It reuses the unchanged G4 runner, its trial authority, executor, ledger, EPD, MoveIt, controllers and stop/cancel mechanisms. Continuous production and the real robot stay unavailable.

**Admission (all must hold, or motion is refused):**
- **A recorded qualification:** the workspace holds a G4 readiness record, `log/qualification/g4/qualification/g4_readiness_runs.json`, written by the campaign's `write_readiness_record.py` with status `QUALIFIED_READY`.
- **The same cell, byte for byte:** the selected cell's seven qualified files (`physics_scope_qualification.QUALIFIED_CELL_FILES`) are byte-identical to the qualified cell's. Any edit or regeneration of them locks motion.
- **A fresh verdict before motion:** at Start, the launched shell first recomputes `g4_qualification.py readiness` for this exact cell from the raw evidence (identities, native build, runtime, perception, completeness of every trial). This takes several minutes. Only exit 0 (`QUALIFIED_READY`) `exec`s the runner; otherwise nothing starts, and the verdict (`<id>.admission.json`) is shown.
- **Per cycle:** the runner's own guards still decide every cycle: fresh rendered frame, EPD class, routing, certified plan, authorisation, contact guard, verification and reconciliation. A stale frame, an unsafe plan or missing evidence refuses or faults the cycle.

**Starting it.**
- **Workcell Builder:** Plan / Simulate → **Start Sorting · Gazebo Motion (2 cycles)** (confirmation required).
- **Operator HMI:** **Sorting session** → **Start Sorting · Gazebo Motion (2 cycles)** (confirmation required; the request carries `mode: bounded_motion, cycles: 2`, and the Builder re-checks admission).
- **The command:** `g4_qualification.py readiness … && exec run_g4_sorting.py --scene-package <cell> --output <log/studio_sorting/<id>> --domain <220..232> --stream-status --authorise-g4-trial --prior-evidence "$WORKCELL_STAGE_A_PRIOR_EVIDENCE" --max-cycles 2`, with `WORKCELL_PLANNING_TRACE_DIR=<id>.trace` (the certified trajectories).

**Stopping it.**
- **Stop / Stop at Cycle End:** SIGUSR1 to the runner, the G4-S mechanism. The running cycle finishes under every guard and is verified; no further cycle starts. It is not an emergency stop. Before the runner has started (admission still running), Stop cancels the start instead.
- **Cancel Simulation / Stop Simulation:** SIGTERM, the G4-C mechanism. It is forwarded to the executor, which cancels the motion under supervision. The cycle is never counted.
- There is no pause in a bounded session.

**What is shown.**
- **Source:** the same `g4_session_view.py` evidence view as plan-only sessions.
- **Objects:** an object reads **COMPLETED → destination · verified** only when the ledger counted it, after the runner verified and reconciled its cycle.
- **Cycles:** per cycle, the HMI shows the EPD result, the plan, the granted authorisation, *executed* and the outcome.
- **Motion evidence:** granted authorisations, executed cycles and controller goals.
- **Where outputs go:** `cad_ws/log/studio_sorting/`, outside the qualification evidence roots. They are operating runs, not qualification trials.
