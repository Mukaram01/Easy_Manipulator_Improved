# Beginner-created cell task readiness — Feature 2

Stacked on Feature 1 (`db4dfaa2`, draft PR #3177). EMD only.

The existing TaskIntent v2 editor now configures off/replay/live normalized
snapshot input, with a file chooser. It retains grasp/place policies and exposes
approach, lift/retreat and placement distances alongside beginner controls.
Normal Save Layout saves dirty task edits. Source/zone bindings preserve
`detected_objects/v1` in perception mode. Reviewed equipment metadata survives
switching from off to replay. Missing, malformed, stale or ineligible input blocks
Plan/Checks; it remains possible to save a draft and generate diagnostic physical
scene output. Collision-aware planning still owns reachability and feasibility.

In-place generation refreshes task identity in the scene manifest and its existing
workcell mirror, preserving curated runtime metadata. Disabled recipes and direct
placement requests do not require invented fallback routes.

## Actual workstation acceptance — Ubuntu 22.04, ROS 2 Humble

**PASS for task-plan readiness. No physical pick/place execution claim.**

- Real desktop Qt New Cell Wizard created `task_readiness_acceptance`, UR5+2F.
- Actual mounting dialog mounted bin → table `surface`, preserving world pose.
- Existing Task controls selected class `cup`, pick zone `pick_zone_main`, physical
  target `target_bin_default`, region `default_drop_zone`, AUTO grasp/place,
  approach 0.12 m, lift 0.15 m, clearance 0.001 m and explicit 300 s observation age.
- Invalid EXACT local X=10 m was saved unchanged and blocked. Missing input was
  saved as a draft and blocked. Normal Save persisted the final valid task.
- Separate Builder process reopened identical TaskIntent and physical IDs/poses;
  actual GUI Generate, Validate and Plan Saved Task actions completed.
- First complete plan resolved the task; final run consumed and revalidated the
  same resolution. Both used fake-hardware RViz/MoveIt, sent zero execution action
  goals, left the live planning scene unchanged during prevalidation, and shut
  down owned processes cleanly. Runtime records contain every planned segment.
- Ten consumers agree: resolver, cell, scene manifest, workcell mirror, recipe,
  compatibility recipe, offline preview, validator, Product View and Plan.
- Destination is world `[0.45, 0.22, 0.11]`, relative to mounted bin
  `[0.45, 0.22, 0.1]`; task/physical frame identity is retained.
- Humble Builder and generated scene builds passed. 78 focused Python tests passed.
  Full Qt editor suite: 16 passed, one opt-in retained-workspace test skipped;
  the actual GUI acceptance above covers that retained workflow. Five affected
  Qt cases passed again after final guard changes. Independent review found no
  remaining blocking defect; whitespace checks passed.
- Canonical `ur5_2f_test` commissioning plan-only regression passed with the existing
  explicit commissioning task request/replay; its authored files were unchanged.

Evidence: [GUI authoring/negative checks](newcell-task-readiness-v1/authoring.json),
[reopen and real plan result](newcell-task-readiness-v1/reopen.json),
[planning screenshot](newcell-task-readiness-v1/planned_scene.png),
[consumer identity parity](newcell-task-readiness-v1/consumer-parity.json),
[canonical commissioning plan](newcell-task-readiness-v1/canonical-plan.json).
Full runtime logs remain at the `runtime_evidence` path in the reopen record;
source commit/dirty status in raw runtime records is retained without restamping.

## Reproduce

Build/source Humble dependencies and `workcell_builder`. From this checkout,
point `$WS/src/easy_manipulation_deployment` at it and use a NEW scene name:

```bash
workcell_builder --newcell-task-acceptance --workspace "$WS" \
  --scene my_task_cell --observations "$PWD/config/runtime/r1_4_replay.yaml" \
  --smoke-output "$EVIDENCE"
workcell_builder --newcell-task-acceptance --workspace "$WS" \
  --scene my_task_cell --observations "$PWD/config/runtime/r1_4_replay.yaml" \
  --smoke-output "$EVIDENCE" --reopen
```

This extends the existing Qt acceptance driver and existing plan-only supervisor;
it does not add another editor, task model, resolver or planner. End users use
Task controls and the observation file chooser; no YAML/ROS commands are needed.

## Limits

Acceptance reused the unchanged reviewed `config/runtime/r1_4_replay.yaml`
commissioning dataset: deterministic conservative bounds, **not a camera
recording**. No perception or collision geometry was invented for this feature.
Live mode consumes a current normalized adapter snapshot without replay stamping;
EPD/camera capture was not tested or modified. Final runtime used installed
CycloneDDS, matching Feature 1's workstation setup.

Generated readiness retains honest in-place/self-test-skipped warnings, with no
missing-recipe/fallback warning or task blocker after resolution. Plan-only
success does not prove object retention, physical release or real execution.
Two legacy `bin_red` authoring-flow fixtures fail unchanged on Feature 1's base
(`test_authoring_flow`, `test_list_and_wrappers`); they were not repaired or hidden.
Full test/build logs are `/tmp/workcell-task-readiness-*`. No EPD or Stage-A source
was modified. Real hardware remains locked; no automatic execution is enabled.
