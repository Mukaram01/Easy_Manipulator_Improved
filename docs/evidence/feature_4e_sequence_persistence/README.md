# Feature 4E — persistent pick sequences: runtime evidence (2026-10-17)

**Status: CONFIRMED (plan-only, fake hardware).** Nothing was executed. These
results do not cover simulated execution, physical contact or grasp retention.

## Environment

The environment is the same as in `docs/evidence/feature_4d_sequence_placement`:

- Docker `moveit/moveit2:humble-release` (ROS 2 Humble with MoveIt 2) and Ubuntu
  Qt WebEngine.
- Xvfb with Mesa software GL.
- `colcon build --symlink-install --packages-up-to ur5_2f_test`.
- One owned fake-hardware `ur5_2f_test` launch for each check.

Each scenario ran the real Builder GUI as two separate processes, so every
"reopen" run is a fresh application start:

`workcell_builder --multiobject-plan-acceptance --workspace /ws --observations
tests/fixtures/multiobject_plan_check_ur5_2f_test.yaml --sequence-order <ids>
--sequence-places <id=place-k|auto,…> [--reopen] [--expect-sequence <status>]`

- **Author run:**
  1. Open `ur5_2f_test`.
  2. In the Task tab, choose recorded observations and class `cup`, then Save
     and Generate.
  3. Click **Preview**. In **Pick sequence**, tick the objects, drag them into
     order, and choose each place in **Place for selected object**.
  4. Save, which writes the `sequence` block into
     `config/workcell_builder_task_intent.yaml`.
  5. Generate again, then click **Check full sequence**.
- **Reopen run:**
  1. Start the app and open the scene.
  2. Without a Preview, check that the list shows the same ticked objects, in
     the same order, with the same places.
  3. Click **Check full sequence**, which plans the saved and generated task.

## Results

| Scenario | Run | Task tab after Save or reopen | Full-sequence check |
|---|---|---|---|
| A, manual places | author | 1. cup-01 → Place 2 of 2 — far end; 2. cup-02 → Place 1 of 2 — near end; cup-03 unticked | **SEQUENCE_PLANNED**: cup-01 at (+0.062, 0), cup-02 at (−0.062, 0), both "(your chosen place)", 118 s |
| A | reopen | identical order and places, Ready, not dirty | **SEQUENCE_PLANNED**, same places, 50 s |
| B, Automatic with a reserved place | author | 1. cup-01 → Automatic (next free place); 2. cup-02 → Place 1 of 2 — near end | **SEQUENCE_PLANNED**: cup-01 Automatic at (+0.006, 0), keeping clear of the near end reserved for the later cup-02; cup-02 at (−0.062, 0), 113 s |
| B | reopen | identical | **SEQUENCE_PLANNED**, same places, 70 s |
| C1, recording changed (cup-02 recorded 2 mm away, same ID) | reopen | ⚠ both rows BLOCKED: `SEQUENCE_SOURCE_CHANGED`; saved places still shown | **Refused before MoveIt**: `SEQUENCE_BLOCKED`, `moveit_planning_attempted: false` |
| C2, destination shortened from 0.20 m to 0.16 m | reopen | ⚠ cup-02 NEEDS REVIEW: `PLACE_OUTSIDE_REGION`; its place is not changed | **Refused before MoveIt**: `SEQUENCE_NEEDS_REVIEW` |
| E, three cups, all Automatic | author | ⚠ cup-03 BLOCKED: `INSUFFICIENT_CAPACITY` (0.060 × 0.060 m with 0.005 m clearance, after 2 objects) | **Refused before MoveIt**: `SEQUENCE_BLOCKED` |
| D, compatibility: the 4B per-object check, which reuses the R1.4 single-object supervisor | author | — | CHECKED: cup-01 and cup-02 PLANNABLE; cup-03 COLLISION @ PREPLAN_GRASP (bottles), as before |

Times are for the whole run: GUI start, Save/Generate and the launch.

- **Clean shutdown.** Every planning launch reported `shutdown_clean: true`.
  After the series, no ROS, MoveIt, plan-check or Builder processes were left
  (`pgrep` empty).
- **Temporary edits restored.** The temporary recording and environment edits
  for C1 and C2 were restored byte-for-byte (`cmp`).

**Why two places, not three.** Places keep clearance plus the planner's 3 mm
placement tolerance from the walls and from each other. With that, the 0.20 m
region holds two 0.06 m cups (the 4D "Place 3" spot is now "Place 2 of 2 — far
end"). A third cup is reported as `INSUFFICIENT_CAPACITY`, and clearance is
never relaxed.

**Why the margin is needed.** During development, a chosen place sitting
exactly at the wall-clearance limit planned in one run and failed the
planner's own `DESTINATION_CONTAINMENT` check in the next. The planned
placement had landed within tolerance, but outward. The margin removes that
failure by construction.

## Tests

- **Python, on Humble.** The suites in `tests/`: `test_physical_destination`,
  `test_perceived_object_grasp_execute`, `test_simulation_batch_plan_check`,
  `test_simulation_batch_preview`, `test_full_cycle_preplanner` and
  `test_task_intent_{authoring,resolver,v2}`. **208 passed** (193 at the #3188
  head).
- **Wider R1.x and task-intent suites.** `test_builder_task_intent*`,
  `test_r14_plan_only_acceptance`, `test_r1*_*`, `test_workcell_builder_task_*`,
  `test_task_*` and `test_runtime_pick_inputs*`, with the tkinter-only
  `test_task_flow_preview` excluded: 170 passed and 11 failed. The 11 failures
  are identical at the #3188 head, so none is new.
- **Qt.** `workcell_task_intent_editor_test` **22/22**, including:
  - `SavedSequenceReopensEditsRevertsAndSaves`: reopen without a Preview,
    choose a place, Revert, drag, Save, reopen.
  - `SequenceProblemsAreShownAndNeverSilentlyRebound`: conflict, and a changed
    recording.
  - `TaskWithoutSequenceLoadsWithEmptyList`.

## Known defect: shutdown crash (investigated, not fixed)

- **Reproduced.** In a bounded loop of 12 open → verify → close runs with no
  planning, the Builder crashed while closing in **2 of 12** runs (exit 139),
  after writing `result: PASS`. It also happened in 1 of the 8 scenario runs
  above (D, exit 245).
- **Backtraces.** Both core dumps show glibc detecting heap corruption
  (`malloc_consolidate`/`unlink_chunk`) while `MainWindow::~MainWindow` deletes
  its children, at different free sites:
  - `QGraphicsViewPrivate::~QGraphicsViewPrivate`;
  - `QProcess::~QProcess` inside `OwnedProductViewServer::~OwnedProductViewServer`.
- **What that means.** The corrupting write happened earlier and is not in
  these traces.
- **AddressSanitizer.** A `workcell_builder` build with AddressSanitizer
  completed 6 of 6 lifecycle runs with no report. At the observed rate that is
  inconclusive (about a 1-in-3 chance of six clean runs).
- **Next step.** Longer AddressSanitizer runs, or valgrind on the teardown
  alone. The crash does not affect saved files or planning results.
