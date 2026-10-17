# Sequence placement allocation — runtime evidence (2026-10-17)

**Status: CONFIRMED (plan-only, fake hardware).** Nothing was executed. These
results do not cover physical contact or grasp retention.

## Environment

- Docker `moveit/moveit2:humble-release` (ROS 2 Humble with MoveIt 2), plus
  Ubuntu `qtwebengine5-dev`. Xvfb with Mesa software GL; WebGL ran with
  `--ignore-gpu-blacklist`.
- `colcon build --symlink-install --packages-up-to ur5_2f_test`, which builds
  `workcell_builder` and the scene packages. The build passed.
- Each run was one owned fake-hardware `ur5_2f_test` launch (`use_fake_hardware`,
  mock components, trajectory execution disabled). The launch was started by the
  existing R1.4 supervisor from the Task tab.

## Beginner workflow driven through the real GUI

`workcell_builder --multiobject-plan-acceptance --workspace /ws --observations
tests/fixtures/multiobject_plan_check_ur5_2f_test.yaml --smoke-output <dir>
--sequence-order <ids>` [`--reopen`]

The run opens `ur5_2f_test` in the Builder. In the Task tab it selects
recorded observations and class `cup`, and sets AUTO grasp, AUTO placement,
5 mm clearance and a maximum age of 300 s. It then clicks **Save** and
**Generate**, which waits until Product View live authoring unlocks. Finally it
clicks **Check full sequence (no motion)** and reads the result shown in the
Task tab. `--reopen` restarts the app, checks that the saved Task fields reopen
identically and not dirty, and checks the sequence again.

## Measured results

| Run | Order | Sequence status | Per-object result (target-local x, y in m) | Wall time* |
|---|---|---|---|---|
| `before_pair` (4C scripts) | cup-01, cup-02 | SEQUENCE_BLOCKED | cup-01 PLANNABLE; **cup-02 COLLISION @ PREPLAN_PLACE** with cup-01 | 122 s |
| `pair` | cup-01, cup-02 | **SEQUENCE_PLANNED** | cup-01 (−0.065, 0); cup-02 (0.000, 0) | 75 s |
| `rev` | cup-02, cup-01 | **SEQUENCE_PLANNED** | cup-02 (−0.065, 0); cup-01 (0.000, 0) | 100 s |
| `reopen` | cup-01, cup-02 | **SEQUENCE_PLANNED** | same places as `pair`; the Task reopened identically | 110 s |
| `rep1`, `rep2` | cup-01, cup-02, cup-03 | SEQUENCE_BLOCKED | cup-01 and cup-02 as in `pair`; cup-03 MISSING_DESTINATION (`DESTINATION_FULL`) | 92 s, 83 s |
| `c3first` | cup-03, cup-01, cup-02 | SEQUENCE_BLOCKED | cup-03 COLLISION @ PREPLAN_GRASP (bottle-n, bottle-s), one place tried; cup-01 (−0.065, 0); cup-02 (0.000, 0) | 105 s |

\*Wall time covers the whole run: GUI start, Save, Generate, the private
launch and planning.

In every run, `execution_attempted` and `motion_commanded` were false and the
supervisor reported `shutdown_clean: true`.

The repeated runs chose the same places. In one earlier `c3first` run, the
(0.000, 0) place for cup-02 failed MoveIt planning, and the next allocated
place, (0.020, −0.010), planned. Sampling-based planning can vary between runs,
which is why up to three places are tried for each object.

The drop region (`default_drop_zone`, 0.20 × 0.10 m) fits three 0.06 m cups
with 5 mm clearance only if every placement is exact. The planner accepts up
to 3 mm of placement error, so after two cups there is no room left for a third.
cup-03 is blocked anyway: its grasp collides with the bottles around it.

## Known defect seen during these runs (not fixed here)

In 3 of 10 plain runs the Builder process crashed during shutdown, after the
harness had written `result: PASS` and after the checker had shut down cleanly
(exit codes 250, 5 and 245; messages `corrupted double-linked list` and
`Segmentation fault`). It did not happen in 3 runs under gdb, so there is no
backtrace yet. The crash happens while the Qt application is closing. It does
not affect planning results.
