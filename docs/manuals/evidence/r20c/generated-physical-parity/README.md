# R2.0c generated physical review parity — 2026-09-16

Classification **B: generated contract defect**, not a validator defect.
The fresh profile supplied a real model, but inherited the generator's legacy
marker-only `demo.launch.py`. It never expanded/published robot_description;
strict runtime/Product View comparison therefore correctly refused to proceed.
A healthy preview mesh index did not establish a runtime launch contract.

Exact original records (the report contains strings, not structured rule IDs):

- Severity warning: `both layout sources exist; using preferred layout file
  .../layout/workcell_studio_layout.yaml and treating legacy .../environment_layout.yaml
  as secondary`. Rule/id, expected, actual, recommendation: not recorded.
  Affected paths: the two layouts. This is truthful in both generation phases;
  the compatibility layout persists after generation. It is not a readiness failure.
- Severity blocker: `Final runtime/Product View transform parity unavailable:
  Cannot resolve the runtime launch xacro request`. Rule/id, expected, actual,
  recommendation: not recorded. Affected contract: `launch/demo.launch.py`.
  Traced expectation: the runtime model expansion consumed by robot_state_publisher.
  Actual: only layout markers. Remedy: generate the missing physical review launch.

The existing package generator now connects a selected physical profile's authored
Xacro to robot_state_publisher and joint_state_publisher, using the model's authored
home state. Existing canonical layout mesh publication and RViz review are retained.
RViz uses Humble Topic/QoS configuration to receive retained fixture markers.
The launch rejects non-fake mode before expanding the model or creating nodes;
it starts no controllers, drivers, task executor or MoveIt pipeline.
This closes physical review handoff, not R2.0d/e task-runtime parity.

Regeneration upgrades only exact generator-produced launch templates on physical
profile cells. Curated/edited launches remain intact. Existing package metadata
is retained with only missing publisher dependencies appended. There is no
acceptance-ID special case, new geometry, hand-edited generated scene repair,
validator change or alternate Product View path.

Evidence:

- Reproduced failing real-Humble profile test before fix (`red.log`).
- User fresh cell: pre- and post-generation **0 mismatches / 0 blockers / 1 warning**.
  Strict final parity rerun after real Web3D handoff also passes.
- Canonical source had a stale generated bin pose (0.94,-0.28,0.1 versus authored
  0.45,0.22,0.1). The unchanged validator detected it. Regenerated an isolated copy
  through the existing generator: **0 mismatches / 0 blockers / 0 warnings**.
  Canonical authored/runtime source was not modified.
- Real generated ROS package colcon build PASS. Review launch started RSP, JSP,
  existing layout marker node and RViz in isolated ROS domain 87. Captured robot
  description, exact authored home joint values, workbench/bin mesh markers.
  Stopped the bounded launch with SIGINT after capture; log includes JSP shutdown
  KeyboardInterrupt, not a startup failure. No hardware/controller process started.
- 7 profile/safety/ownership tests PASS; 16 existing parity tests PASS (22 combined
  before adding the seventh profile test). Real mismatch detection remains covered.
- Existing R2.0c focused tests **116 PASS** (76 Python,15 editor,9 model,16 wizard).
- Humble workcell_builder and affected targets build PASS.
- Fresh Web3D handoff **2/2 READY**, both identities match. `git diff --check` PASS.
- User environment/layout/TaskIntent/Xacro hashes unchanged; `.worktrees/` untouched.

Main commands after sourcing Humble + workspace install (repo root):

```bash
python3 -m pytest -q tests/test_new_cell_physical_profile.py tests/test_scene_builder_canvas_generated_parity.py
python3 scripts/generate_workcell_from_cell_definition.py scenes/r20c_home_ui_acceptance_v2_20260916/cell_definition.yaml --output-dir scenes --package-name r20c_home_ui_acceptance_v2_20260916 --existing-package-dir scenes/r20c_home_ui_acceptance_v2_20260916 --workspace-root /home/user/workcell_ws
python3 scripts/validate_scene_builder_canvas_generated_parity.py scenes/r20c_home_ui_acceptance_v2_20260916 --mode post_generation --json
```

## Resume manual acceptance

```bash
bash /tmp/r20c-production-acceptance-v2/launch.sh
```

1. Home → reopen `r20c_home_ui_acceptance_v2_20260916`; do not recreate it.
2. Generate Scene Package → Validate / Canvas–RViz parity. Expect 0 mismatches,
   0 blockers, the single dual-layout warning above, and Web3D ready.
3. Continue saved task policy/binding checks, Save → Home → reopen, then close
   Studio and run the same launch command again to verify restart/reopen persistence.
4. Validate an invalid EXACT placement (`10,0,0` local XYZ): visibly BLOCKED,
   unchanged EXACT/request, no fallback. Discard that invalid edit and reopen saved state.
5. Send the printed session directories, policy/blocker screenshots, and PASS or
   failed step. The launcher captures authored files and normalized TaskIntent hash
   on exit; Codex will compare them. No manual file repairs are required.

PR #3170 remains draft; R2.0c remains partial until manual persistence/hash/EXACT
acceptance succeeds. Rollback by reverting this scoped change; preserve user cells.
