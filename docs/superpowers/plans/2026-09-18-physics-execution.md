# Physics execution commissioning plan

Goal: one contact-only UR5/Robotiq pick/place through the existing authoritative path.
Baseline: 7a5cd224; user A0.5 specification is the acceptance contract. No environment rebuild/repair; no commit before acceptance.

- [x] Map authoritative Generate/freshness, backend identity and support/runtime contracts (a05 evidence/contracts.md).
- [x] Add focused backend evidence tests; implement explicit simulator verification without changing fake behavior.
- [x] Add reusable simulator model/startup integration to canonical launch, preserving state-only coupled followers and post-physics spawn.
- [x] Save and Generate a current task using existing authoring/export path; capture freshly settled physical observations; resolve with existing ranking/preplanner.
- [ ] Commission measured execution/support transition only if the physical/live planning scene can remain consistent under the existing scoped collision policy. Add negative tests before enabling it.
- [ ] Execute at most one pick/place cycle and measure contact, retention, separation, release and restoration, or stop at a proved contract blocker.
- [x] Run focused verification and report evidence; commit/PR only after full physical acceptance.

Files: scripts/simulator_backend.py (identity and simulator observation contract); scripts/perceived_object_grasp_execute.py (explicit selection, existing default fake path preserved); scenes/ur5_2f_test/launch/demo.launch.py (simulator startup integration); focused tests; existing CMake script install list if new module is integrated. No edits to full_cycle_preplanner or global collision permissions unless a demonstrated defect requires them.

Outcome: BLOCKED at measured support/contact execution. No physical cycle and no commit/PR. Full-cycle planning passed; runtime acceptance remains unchecked. Evidence: /home/user/workcell_ws/a05-evidence-20260918/REPORT.md. Live evidence justified bounded quaternion-equivalence and authored destination-orientation fixes in addition to the initially mapped files.

Continuation (same branch/worktree): implement the existing missing contract. The executor now has an explicit simulator commissioning selector for cancellation/contact-release, measured-state collision queries using the shared C++ floor predicate, real physics contact points, grasp/slip checks and measured cancellation/reconciliation. Ordinary simulator execution remains blocked. Full-cycle admission still requires physical commissioning. No source changes are committed pending acceptance.

The Fortress contact-message adapter omits normal/depth; its surface-event callback is unavailable with the installed physics plugin. The final read-only telemetry adapter therefore reports actual physics collision IDs/points plus poses/joints, and measured MoveIt FCL state queries provide the separate collision normal/depth policy evidence. No surface parameters, contact geometry or tolerances are changed. Evidence is in `/home/user/workcell_ws/a05-evidence-20260918/continuation/`.

Continuation result: 118 focused tests pass and live physics contact/pose/joint telemetry is confirmed. Resolve again planned nine stages, but both execution-time selected-approach revalidations timed out (-6), including the explicit 10-second segment budget. No execution goal was sent. Cancellation and contact-release physical acceptance remain unchecked; final live scene/ACM/state were captured before shutdown. See continuation/REPORT.md.

2026-09-18 bounded revalidation/cancellation continuation: paired effective requests proved fresh IK selected different arm branches for the same candidate. The shared resolution now binds the successful approach IK seed to model/group/TCP/target; current-state collision planning still runs for every stage. Three consecutive nine-stage revalidations passed without motion. The guarded moving cancellation trial proved accepted ownership and physical movement, but installed MoveIt2.5.9 ExecuteTrajectory deferred cancellation until approach completion and returned SUCCEEDED, not CANCELED. Sustained stop telemetry also failed freshness. Final services-alive audit retained baseline ACM/no attachments/valid collision state and a fresh stationary snapshot; no contact/full-cycle execution or automatic recovery home/open occurred. STATUS remains BLOCKED; all work uncommitted. Detailed evidence: /home/user/workcell_ws/a05-evidence-20260918/revalidation/REPORT.md. Next: correct/qualify execution action cancellation before further motion.


### 2026-09-18 cancellation qualification continuation

Preserved the successful shared IK binding and prior three-pass planning result.
Added the opt-in MoveIt2.5.9 cancellation capability, actual ROS action tests,
responsive measured telemetry acquisition/queued evidence writing, and exact
controller-result/stop acceptance checks. One moving cancellation passed with
301ms consecutive measured stop and alive scene/ACM reconciliation. No contact,
release, transfer or home was executed. Final source review added a read-only
immutable controller-result audit to avoid the upstream handle's late-cancel
status-cache race; the final build passes10 action tests but was not moved.
Final capability physical qualification therefore remains pending. No further
motion was sent; all changes are uncommitted. See
`~/workcell_ws/a05-evidence-20260918/cancellation-qualification/REPORT.md`.
Physical pick/place remains UNCOMMISSIONED; ordinary/hardware locks remain.
