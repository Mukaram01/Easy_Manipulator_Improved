# R2.0b E1 top extraction — real fake plan-only evidence

**CONFIRMED:** the extracted shared preplanner preserves the canonical top full-cycle planning behavior on source commit `baed0e486cec5fc927b80ef784d50eaf9c59fc44`.

`acceptance.json` is an unchanged copy of the actual acceptance runner output. `source_manifest.json` is an unchanged copy of the installed/source six-module hash verification. Historical paths and source provenance inside them describe the run and have not been rewritten to this evidence location.

## Result and scope

- Acceptance PASS in 17.013868 seconds; source tracked files clean at run time.
- Canonical `ur5_2f_test` selected `runtime::sample-cup`, grasp index 3 (`top_2f::003`), first candidate attempt. Executor reported eight generated top candidates.
- All nine MoveIt motion plans succeeded; eleven stage events include private ATTACH and DETACH. Candidate checks include destination containment. Grasp and retreat retain validated vertical Cartesian corridors.
- Live PlanningScene unchanged after prevalidation; exclusively mock hardware; trajectory execution disabled; no execution action goals or terminal statuses.
- Executor and launch exited 0; clean shutdown, no crashes or remaining owned process groups.
- Focused Python regressions: 138 passed in 1.86s, no skips or warnings.
- This proves legacy top plan-only extraction. Side/pinch, v2 resolver and post-extraction actual execution are outside this evidence. Sampling-based trajectories/timings are not expected to be byte-identical between runs.

## Exact commands and environment

Working directory for every command:

```text
/home/user/workcell_ws/src/easy_manipulation_deployment/.worktrees/r20b-sdd
```

Build, using the existing workspace install only as dependency underlay:

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/install/setup.bash
colcon --log-base .superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/build-log build --base-paths workcell_builder/workcell_builder scenes/ur5_2f_test --build-base .superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/build --install-base .superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/install --symlink-install --packages-select workcell_builder ur5_2f_test --allow-overriding workcell_builder ur5_2f_test --cmake-args -DBUILD_TESTING=OFF
```

Build exit 0: `workcell_builder` finished in 4m11s, `ur5_2f_test` in 2.23s; two packages finished in 4m15s. Both installed package prefixes resolve inside this worktree overlay. All six installed scripts were byte-compared against the source, then SHA-256 recorded in `source_manifest.json`; the runner additionally checked executor and canonical scene hashes.

Acceptance (no `--execute`, no `--start`):

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/install/setup.bash
source .superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/install/setup.bash
python3 scripts/run_r14_plan_only_acceptance.py --output-dir .superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/top-parity-runtime --timeout 240 --domain-id 187 --stream-status
```

Focused final verification:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/test_full_cycle_preplanner.py tests/test_grasp_strategy_candidates.py tests/test_physical_destination.py tests/test_transactional_pick_cycle.py tests/test_perceived_object_grasp_execute.py tests/test_task_intent_v2.py tests/test_r14_plan_only_acceptance.py
```

## Build stderr and limits

The build succeeded but its output was not pristine. Exact initial npm stderr:

```text
npm ERR! code ELSPROBLEMS
npm ERR! missing: @esbuild/linux-x64@0.20.2, required by workcell-studio-web-viewer@0.1.0
npm ERR! missing: esbuild@0.20.2, required by workcell-studio-web-viewer@0.1.0
npm ERR! missing: three@0.160.0, required by workcell-studio-web-viewer@0.1.0
npm ERR! missing: urdf-loader@0.13.0, required by workcell-studio-web-viewer@0.1.0

npm ERR! A complete log of this run can be found in:
npm ERR!     /home/user/.npm/_logs/2026-09-16T05_59_13_118Z-debug-0.log
```

The existing build then ran its own `npm ci`: `added 4 packages, and audited 5 packages in 2s`, reporting `1 moderate severity vulnerability`; the current viewer bundle required no rebuild. Existing AutoUic duplicate names `copy_build_command_button` and `copy_launch_command_button` also produced warnings. These observations do not establish GUI/npm health. No GUI, npm dependency, or audit-fix changes were made in E1.

Large raw scene/log artifacts remain local under `.superpowers/sdd/2026-09-15-r20b-shared-readiness-planner-extraction/top-parity-runtime/`; they are intentionally omitted from this compact tracked package. The acceptance JSON preserves successful checks, stage evidence, provenance, action monitoring and process cleanup.

## Copied artifact hashes

- `acceptance.json`: `078824472e74b6688cec1dfe73e425730074109644d5556ce3a9c120a9b39824`
- `source_manifest.json`: `5001181216ca01118b9b28ae88eefc9bced68765fb0b4de9565153c42a188228`
