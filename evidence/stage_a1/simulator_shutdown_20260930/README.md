# Simulator wrapper shutdown evidence — 2026-09-30

**CONFIRMED:** the wrapper retains and reaps its Gazebo child through repeated
shutdown signals, and reports forced termination as failure. Two focused
regressions changed from failing to passing; all 69 backend tests passed.
Three fresh trials with real Gazebo and four active simulator controllers
completed without forced cleanup or surviving simulator processes.

**LIMIT:** the original long-running Gazebo stall was not reproduced. This
bundle establishes the wrapper correction and its bounded native trials.
Stage-A remains blocked: the preceding Resolve search exhausted its budget,
its teardown was unclean, and no motion or later gate passed.

## Failed Stage-A run

The preserved run is
`/home/user/workcell_ws/stage-a1-closeout-20260930-active-plan`, at clean commit
`84ca9a47f1779d8fe0dd56f7b39a8d38f5b0cb05`. Source, build and qualified artifact
identities remained unchanged through terminal capture. Resolve failed with
`SEARCH_BUDGET_EXHAUSTED` after its 900-second search allowance.

The launch supervisor was PID **73904**, its `python3-3` simulator wrapper was
PID **73922**, and the actual Gazebo server was PID **73970**. The receipt and
live process maps identify that server; the wrapper's exit status does not
represent the grandchild's terminal state.

| Evidence time (Unix seconds) | Observation |
| --- | --- |
| 1790805404.133410735 | MoveGroup records the cleanup SIGINT. Launch also sends SIGINT to the simulator wrapper. |
| 1790805404.633248774–1790805408.637907045 | Process samples retain launch 73904, wrapper 73922 and Gazebo 73970. |
| 5 seconds after launch's first SIGINT | Launch explicitly escalates the wrapper to SIGTERM; the wrapper then reports exit 0. |
| 1790805409.638198381 | Launch and wrapper are absent. Gazebo 73970 remains live, reparented to PID 1170. |
| Through 1790805418.647631639 | The same Gazebo process remains asleep. |
| 1790805419.648662423 | No sampled owned process remains. The final supervisor report records SIGINT, SIGTERM and SIGKILL. |

Every logged launch child returned 0, but the supervisor correctly marked
shutdown **unclean** because cleanup reached SIGKILL. The sole remaining owned
Gazebo process is the inferred SIGKILL recipient; no per-recipient signal audit
was collected. [shutdown-timeline.json](shutdown-timeline.json) preserves the
exact samples and this attribution limit.

The old signal handler forwarded SIGINT and raised `KeyboardInterrupt`. Its
`finally` block forwarded another SIGINT and waited for the child. A later
SIGTERM could invoke the same handler during that wait, interrupt cleanup and
reach the top-level `KeyboardInterrupt` handler, which exited 0. The wrapper
could therefore abandon a live Gazebo process. The reason Gazebo initially
failed to exit promptly remains unresolved; duplicate signals alone were not
proved to cause that stall.

## Correction and verification

The corrected wrapper sends one SIGINT, keeps ownership while later shutdown
signals arrive, reaps the child before returning, and returns its actual
terminal code. The existing 10-second cleanup bound is unchanged. If that
bound requires SIGKILL, the wrapper raises a failure. Previous signal handlers
are restored after cleanup.

The two regressions exercise real subprocess signals and reaping, with the
external Gazebo startup replaced by a controlled child. They cover repeated
signals and forced termination hidden as exit 0. These are unit evidence.

The separate native trials started actual Gazebo, robot-state-publisher and
four simulator controllers: `joint_state_broadcaster`, `ur5_arm_controller`,
`ur5_gripper_controller` and `physical_joint_states`. The model used
`ign_ros2_control/IgnitionSystem`, and process maps contained the native control
plugin. No bridge, MoveGroup, planning goal or motion goal was started in these
three trials. Each wrapper received SIGINT followed by SIGTERM.

| Cycle | Wrapper / Gazebo PIDs | Observed shutdown | Wrapper / RSP exits | Forced cleanup / live server |
| --- | --- | --- | --- | --- |
| 1 | 78239 / 78252 | 0.275241 s | 0 / 0 | None / none |
| 2 | 78464 / 78477 | 0.489750 s | 0 / 0 | None / none |
| 3 | 78687 / 78700 | 0.375193 s | 0 / 0 | None / none |

The implementation identities are:

- `scripts/simulator_backend.py`: `255ff862959c5f81b837e99349000970775c46d6c2221f1c665d44e642d7f8c7`
- `tests/test_simulator_backend.py`: `8d12d9361f7e578385ec9ab8f99b3b4ef15ca6560aaf127577266fa07d3b1036`

## Evidence and limits

- [lifecycle-result.json](lifecycle-result.json): byte-identical diagnostic
  report with implementation, test, probe, model, log and process-map hashes.
- [native-results.json](native-results.json): byte-identical three-cycle results.
- [source-manifest.json](source-manifest.json): original paths and hashes for
  the copied files, failed-run logs, terminal reports and process samples.

Full local diagnostics remain at
`/home/user/workcell_ws/diagnostics/stage-a1-closeout-20260930-simulator-shutdown`.
Large logs, models and maps are referenced by hash rather than copied here.
Rejected setup attempts remain there and are excluded from the successful
trials. A short full idle baseline also exited normally; it did not reproduce
the long-running stall.

Changing the backend and test file invalidates the earlier bridge review's
whole-file identities. A fresh independent review and 20 native plus 5 Memcheck
cycles are required before the next full-chain launch. This bundle does not
claim that separate bridge qualification or any Stage-A gate has completed.
