# Active planning shutdown qualification — 2026-09-30

**CONFIRMED:** the new MoveIt installation passed 20 TEM shutdowns, 20 idle
MoveGroup shutdowns, and five owned SIGINTs during the genuine saved UR5
PLAN_ONLY request. Each active cycle completed the production certificate
(1,647 inspected intervals) after the process's actual rclcpp signal-handler
record and exited 0. No cycle needed forced cleanup or left a new core or live
owned process. Planning and trajectory execution stayed separate: no execution
goal or hardware driver was started.

The genuine baseline is the Stage-A Resolve SIGABRT in
`stage-a1-closeout-20260930-hybrid-lift/01-resolve/launch.log`. The guard handles
terminal-result publication after ROS context shutdown while keeping
valid-context exceptions visible. Six portable compiled callback cases and a
signal-observation regression are unit evidence; the attached active results
come from the installed native MoveGroup and production Workcell adapter.

The external overlay is
`/home/user/workcell_ws/moveit_teardown_overlay_active_plan_20260930`.
TEM was copied unchanged from the preserved September 27 qualified install;
only MoveGroup was rebuilt from its pinned release with the extended patch.
The original overlays and provenance remain unchanged. Standard colcon setup
hooks were generated using the workstation's installed colcon modules.

[summary.json](summary.json) binds source/build identities, all three native
proofs, current MoveIt/bridge admission, and the full 45-cycle mapping audit
(195 shared-library identities). [active-results.json](active-results.json)
contains the five real active cycles. Full logs, maps, archived adapter/probe,
and earlier rejected qualification attempts stay in the overlay's `evidence/`
directory. They are listed honestly in the summary and were never published
as passing proof. These checks qualify dependency lifecycle cleanup; the
Stage-A simulator gates have their own live acceptance evidence.

Use the source sequence recorded in the summary, starting from ROS, followed
by the new overlay's `install/local_setup.bash`, the workspace's
`install/local_setup.bash`, and the qualified bridge's `setup.bash`.
