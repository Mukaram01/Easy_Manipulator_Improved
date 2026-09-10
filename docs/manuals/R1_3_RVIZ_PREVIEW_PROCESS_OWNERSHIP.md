# R1.3 RViz preview process ownership

Scope: Workcell Studio `Plan / Simulate` fake-hardware preview lifecycle, first accepted on `ur5_2f_test`.

## Reproduced workstation failure

On 2026-09-10 the canonical fake-hardware launch successfully started RViz, MoveIt, `ros2_control_node`, `robot_state_publisher`, static TF and the Workcell Studio layout mesh helper. Clicking **Stop RViz** changed the UI to `PREVIEW_STOPPED`, but those runtime processes remained alive. The tracked `ros2 launch`/shell process had exited while its descendants were re-parented and continued running in the inherited process group.

Sending SIGINT to the six exact surviving preview processes shut them down cleanly. This proved that the failure was ownership/lifecycle handling rather than a ROS node refusing graceful shutdown.

## Ownership contract

The actual RViz/MoveIt fake-hardware launch is now wrapped by `scripts/workcell_preview_process_group.py`.

The supervisor:

- remains the single process tracked by the existing Qt `QProcess`;
- starts `ros2 launch ... use_fake_hardware:=true launch_rviz:=true` in a new POSIX session/process group using `start_new_session=True`;
- verifies that the child PID, PGID and SID are identical and different from the supervisor/Workcell Studio process group before accepting ownership;
- never uses process-name matching, `pkill`, `killall`, or another broad system-wide stop;
- on a stop request, signals only the owned process group with SIGINT, then SIGTERM, then SIGKILL if required;
- waits until the owned group no longer exists before the supervisor exits;
- treats a launch leader that exits while descendants remain as an error, cleans the owned group, then exits non-zero.

The existing `MainWindow::stop_preview_process()` still calls `QProcess::terminate()` first. On Linux this asks the tracked supervisor to stop. Because the supervisor does not exit until its isolated launch group is gone, `handle_preview_finished()` can no longer publish `PREVIEW_STOPPED` while RViz/MoveIt/ros2_control descendants are still alive. The existing three-second Qt hard-kill remains a final supervisor watchdog; the supervisor's own escalation window completes before that timeout.

`MainWindow::closeEvent()` already uses the same stop path, so close-while-running receives the same descendant cleanup semantics.

## Safety preserved

The launch still requires `use_fake_hardware:=true` and `launch_rviz:=true`, keeps the real-hardware deny list, retains the duplicate `controller_manager/ros2_control_node` guard, and does not alter collision/ACM policy, scene geometry, EPD or operator-HMI behavior.

## Regression coverage

`tests/test_rviz_preview_process_group_lifecycle.py` is a real POSIX process-tree test rather than a source-token-only assertion. It proves:

- the preview child receives a session/process group isolated from the test/Studio process group;
- terminating the supervisor causes group SIGINT and removes descendants before the supervisor returns;
- stubborn descendants that ignore SIGINT and SIGTERM are removed by the SIGKILL fallback;
- a leader exiting while a descendant remains is cleaned and reported as failure;
- a second preview can start after the first is fully stopped;
- an unrelated process in another process group is left untouched.

The existing static RViz safety assertions remain as wiring/guard checks, but no longer serve as the process-tree proof.

## Remaining workstation acceptance

After merging this fix, repeat:

`Plan / Simulate -> Build & Run RViz -> Stop RViz -> verify RViz and all owned ROS descendants disappear -> Build & Run RViz again -> Stop RViz -> return to Scene Builder`

Then complete the persistence round trip:

`edit -> Save Layout -> Generate -> Validate -> Plan / Simulate -> fake-hardware preview`.
