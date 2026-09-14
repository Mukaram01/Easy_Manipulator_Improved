# R1.6 same-session two-cycle closure

**CONFIRMED on 2026-09-14:** the existing Workcell Builder
`--full-cycle-acceptance` completed both real fake-hardware cycles in one Builder
process, PID **11171**. The acceptance implementation was not changed.
Camera/EPD was not used; fake hardware only. This uses the deterministic R1.5
replay input, not calibrated live perception or physical robot commissioning.

Base: PR #3165, `989b48ea0b0cc68ae8d436f83f72d0ecc4fcec89`.
Platform: Ubuntu 22.04.5, ROS 2 Humble, existing MoveIt underlay
`53979331bcaa472ddd3a3fc895907172df01374d`.

## First failure and fix

The unchanged-main acceptance passed cycle 1, then failed at phase 5 with
`Generate did not propagate authored drop zone`. Inspector → Apply → Save
persisted `place_zone_default.x = 0.46` in the authoritative layout. However,
`workcell_studio_layout_merge.py --save-authored` matched physical records only
by `id`. The runtime-bound `default_drop_zone` has
`layout_item_ref: place_zone_default`, so it retained `x = 0.45` while the
existing direct-ID record changed. Generate correctly projected that stale
authored environment into the runtime handoff.

The only production change matches existing records by either `id` or
`layout_item_ref`, updates all matching projections, and preserves their semantic
IDs, task references and other metadata. Existing top-level mirrors follow their
respective semantic IDs. An alias-only record no longer causes an extra direct-ID
record to be created. No scene-specific mapping, executor, controller, collision,
ACM, perception or acceptance-runner change was needed.

Two regression cases reproduce the bug against unchanged main and pass with the
fix. They cover alias-only and existing direct-ID records, XYZ/RPY persistence,
unchanged unrelated records, semantic metadata, mirrors, repeated-save stability,
and the generated environment projection.

## Verified result

The [machine-readable record](evidence/r16_two_cycle_acceptance.json) includes the
failed first attempt, original passing Studio acceptance, both complete runtime
acceptance reports, disk readbacks, process cleanup and regression results.

| Evidence | Cycle 1 | Cycle 2 |
| --- | --- | --- |
| Builder PID | 11171 | 11171 |
| Result | PASS | PASS |
| Consumed destination, metres | `[0.45, 0.22, 0.13]` | `[0.46, 0.22, 0.13]` |
| Placement error | 0.05948 mm | 0.02910 mm |
| Home / detach / final collision / baseline ACM | All verified | All verified |
| Shutdown / residual owned runtime groups | Clean / zero | Clean / zero |

Each cycle recorded nine successful plans and nine successful ExecuteTrajectory
results: approach, grasp, close gripper, lift, transfer, place, open gripper,
retreat and home. The observed nine MoveIt, seven arm-controller and two
gripper-controller goals all reached terminal status 4 (SUCCEEDED).
Attachment was verified; the final scene contained the cup once in the world,
no attached objects, and an unchanged bottle distractor. Both hardware components
reported `mock_components/GenericSystem`.

Between cycles, the same Builder returned to Scene Builder and used the real
Inspector Apply and Save controls. Disk readback verified `0.46` in the layout
and in both authored zone records. Generate/Validate propagated it to the runtime
handoff, and cycle 2's executor reported that new destination.

After cycle 2, the acceptance used Inspector → Apply → Save → Generate → Validate
to restore `[0.45, 0.22, 0.13]`. Independent disk reads verified the restored
layout, authoritative environment and generated cell. Builder exited with code 0;
its Product View server (PID 11268) also exited. The recorded runtime process
groups were checked after exit and none remained. Generated report and YAML
serialization churn was archived and then reverted; the PR carries no scene edits.

## Commands and results

Initial tree was clean. `git switch main` and `git pull --ff-only` reported main
already current; branch `codex/r17-r16-two-cycle-closure` was created.
Commands below ran from the indicated directories. Evidence outputs were new.

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder ur5_2f_test --parallel-workers 2

# Subsequent build and runtime commands also sourced the proven MoveIt underlay.
source /home/user/ws_moveit2/install/local_setup.bash
colcon build --symlink-install --packages-select workcell_builder ur5_2f_test --parallel-workers 2
source /home/user/workcell_ws/install/local_setup.bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment

# First attempt: cycle 1 PASS, then the authored destination mapping failure.
ros2 run workcell_builder workcell_builder --workspace /home/user/workcell_ws \
  --full-cycle-acceptance --smoke-output /home/user/workcell_ws/r17-evidence/two-cycle-1

# After the focused fix and rebuild: complete same-session PASS.
ros2 run workcell_builder workcell_builder --workspace /home/user/workcell_ws \
  --full-cycle-acceptance --smoke-output /home/user/workcell_ws/r17-evidence/two-cycle-2

python3 scripts/run_r14_plan_only_acceptance.py --timeout 120 --domain-id 196 \
  --output-dir /home/user/workcell_ws/r17-evidence/r14
python3 scripts/run_r14_plan_only_acceptance.py --execute --timeout 900 --domain-id 197 \
  --output-dir /home/user/workcell_ws/r17-evidence/r15

python3 -m pytest -q tests/test_runtime_pick_inputs.py \
  tests/test_transactional_pick_cycle.py tests/test_perceived_object_grasp_execute.py \
  tests/test_perceived_object_grasp_plan.py tests/test_r14_plan_only_acceptance.py \
  tests/test_workcell_studio_layout_merge.py tests/test_r11_canonical_readiness.py \
  tests/test_workcell_studio_live_inspector_pose_editing.py \
  tests/test_rviz_preview_process_group_lifecycle.py

QT_QPA_PLATFORM=offscreen ctest --test-dir /home/user/workcell_ws/build/workcell_builder \
  -R 'workcell_(rviz_preview_metadata_command|layout_serialization_contract|studio_layout_editor|preview_process_state)_test' \
  --output-on-failure
git diff --check
git status --short
```

Both relevant builds passed (two packages). One intermediate build attempt found
the archived failed scene as a duplicate package; adding
`/home/user/workcell_ws/r17-evidence/COLCON_IGNORE` excluded the evidence directory,
and the same build command passed. Final focused tests: **115 passed**.
Qt targets: **4 passed**. R1.4: **PASS**, nine plans, zero execution goals, clean
shutdown. Standalone R1.5: **PASS**, canonical destination, all state/action guards,
clean shutdown. Existing acceptance checks were reapplied to the saved runtime
JSON, not inferred from exit codes alone.

An additional earlier test command included
`tests/test_workcell_studio_canvas_layout_persistence.py` alongside the readiness,
live Inspector and process-group tests: **20 passed, 2 failed**. Its
`test_pose_and_preview_only_fields_written` and
`test_remove_action_is_layout_instance_only_and_non_destructive` require obsolete
literal strings (`pose["x"]` and the old `environment_layout.yaml` remove prompt)
that are also absent on unchanged main. These pre-existing source-text failures
were not changed or counted as passing. The final focused suite above excludes
that unrelated legacy file.

Raw logs, screenshots, original JSON and failed/restored scene snapshots are
preserved at `/home/user/workcell_ws/r17-evidence`. Cycle screenshots are
`two-cycle-2/run_1.png` and `two-cycle-2/run_2.png`. Full runtime evidence is also
copied into `run_1/` and `run_2/` there. The committed JSON records file hashes and
original runtime paths; `source_dirty: true` honestly reflects the tested fix and
run-generated scene changes on the recorded base commit.

## Limits and rollback

This closes uninterrupted deterministic replay acceptance for `ur5_2f_test` only.
It does not close calibrated live perception, other scene/modality acceptance,
or the pre-existing MoveIt cancellation limitation documented in
[R1.5](R1_5_FAKE_HARDWARE_EXECUTION.md). No camera or EPD was started or debugged.
Fake-hardware defaults and real-execution locks remain unchanged.

Reverting the merge-helper change restores the old ID-only save mapping and
reintroduces the demonstrated stale runtime destination. No runtime or underlay
rollback is required.
