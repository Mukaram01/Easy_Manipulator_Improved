# Destination-bin repeat-save regression (2026-10-22)

M1 acceptance: two valid bin edits in one editor session must preserve the
destination's target-local placement, survive process reopen, and reject a real
out-of-bounds region edit without changing the prior committed files.

## Cause proved before the production change

Baseline: `1c23a07913ed0125e0eb52efe25fa271e9540043`, PR #3201 (Draft).
Both a disposable Python projection trace and the actual display-enabled Qt
Inspector → Apply → Save path reproduced the failure. Canonical fixtures were
not edited.

`target_bin_default` owns reviewed usable geometry. `default_drop_zone` and its
`place_zone_default` compatibility projection both retain
`layout_item_ref: place_zone_default` and `target_ref: target_bin_default`.
Their `placement_local`, not a cached editor world pose, defines their relation.

All poses below are world XYZ in metres, with zero RPY (identity rotations).
Target-local region XYZ starts at `[0, 0, 0.01]` (stored Z has ordinary binary
floating-point rounding). Region dimensions are `[0.20, 0.10, 0.12]`; usable
dimensions are `[0.24, 0.12, 0.14]`, centred at local `[0, 0, 0.01]`.

| Boundary | Bin world XYZ | Environment zone world XYZ | Retained Qt zone XYZ | Local Y |
|---|---|---|---|---|
| Before Save 1 | .45, .22, .10 | .45, .22, .11 | .45, .22, .11 | 0 |
| After Save 1 | .45, .23, .10 | .45, .23, .11 | .45, .22, .11 | 0 |
| Before Save 2 | requested .45, .24, .10 | previous .45, .23, .11 | .45, .22, .11 | 0 on disk |
| Failed projection | .45, .24, .10 | overwritten .45, .22, .11 | .45, .22, .11 | −.02 inferred |

The merge sees the stale zone pose differ from the previous environment and
mistakes it for an independent zone edit. Its inferred local Y extent is
`.02 + .05 = .07`, exceeding usable half-width `.06`. Containment correctly
rejects that inferred geometry. The requested bin-only motion is valid: keeping
local Y zero gives extent `.05`, within `.06`.

The X variant is also defective: two +.01 edits can succeed while silently
changing local X to −.02, exactly at the X margin. Tests therefore check local
relationships and entire persisted documents, not only Save's return status.

This is stale retained editor state causing a derived zone to become an
apparent independent edit (B/C). No usable bounds, dimensions, destination IDs,
task references, or local-placement serialization drift caused the rejection.
MainWindow rollback restored the previous layout byte-for-byte.

## Change and verification

After successful physical projection, MainWindow reads committed destination
poses/dimensions into its retained preview and canvas, refreshes hierarchy, then
captures the session. Web3D keeps its existing dependent/baseline rebasing: no
post-save bridge command overwrites newer browser edits. Physical containment
and the merge helper are unchanged. Explicit invalid region edits still fail.

The opt-in acceptance runner copies `ur5_2f_test`, scripts, and web resources
into an isolated output root; it uses the real window and buttons, requires the
requested backend, runs a second process for reopen, compares complete authored
YAML and task/config files, verifies retained rejected edits, and hashes the
canonical source before/after. Existing Save reconciles the top-level camera
compatibility pose with its already-authored physical pose; this pre-existing
mirror update is included explicitly in the expected document.

```bash
source /opt/ros/humble/setup.bash
python3 workcell_builder/workcell_builder/test/repeat_save_runtime_acceptance.py \
  --binary /home/user/wcb-gui-build/workcell_builder \
  --output /tmp/repeat-save-y --backend native_scene3d --axis y
```

Use a fresh output directory for every run; `--axis x` checks the second axis.
No generation, planning, execution, Stage-A, EPD or Gazebo work is requested.

Display-enabled Ubuntu 22.04 / ROS Humble results:

- Native X: **PASS**, bin and zone X `.45 → .46 → .47`; local X stays zero.
- Native Y: **PASS**, bin and zone Y `.22 → .23 → .24`; local Y stays zero.
- Both native runs: complete document comparisons, retained invalid edit,
  byte-identical failed-save rollback, separate-process reopen, and clean exit.
- Embedded Web3D Y: both saves, invalid rejection/rollback, retained edit, and
  separate-process reopen assertions **PASS**; exact layout/environment and
  unchanged task/config metadata also verified after reopen. The authoring
  process aborted during shutdown with `corrupted double-linked list` after
  writing its passing assertion report. The independent reopen process exited
  zero. The runner rejects the nonzero authoring exit; this is **not** a fully
  passing embedded-process acceptance. Shutdown remains a scoped-out blocker,
  with no attribution to baseline or this fix established.
- Reviewed `workcell_builder` build: **PASS**; focused independent review found
  no remaining actionable issues after removing the post-save browser sync.

Local evidence is under `/tmp/wcb-repeat-save-evidence/`: `red-y` captures the
original failure; `final-native-x2`, `final-native-y`, and `final-web-y2` contain
real GUI reports/logs and screenshots. No source scene or canonical fixture was
changed. Only the existing editor/layout projection and isolated test/evidence
files are in the patch.

## Existing failures outside the fix

The focused physical-destination/merge/acceptance tests pass (42 tests), as do
the layout serialization, layout editor and semantic-role C++ suites (26 tests).
Additional checks exposed these failures in unchanged code/tests; they are not
fixed or hidden by this change:

- `test_fallback_blocks_only_on_missing_required_authoring_files_with_exact_names`
  expects the removed `environment_layout.yaml` fallback source token.
- `test_required_cross_scene_roundtrip_acceptance_passes_and_reports_machine_readable_results`
  fails its existing temporary-scene generation acceptance.
- `SaveProjectionFailure.FallsBackToStdoutThenToAGenericReason` has message
  formatting expectations that disagree with the unchanged reason formatter.
- `WorkcellSceneIdentity.HomeReadinessUsesCurrentCanonicalAcceptanceNotLegacySmokeExistence`
  and `WorkcellSceneIdentity.CanonicalScenesUseTheSameReadyContractAsHome` expect
  readiness/acceptance fingerprints not satisfied by the existing fixtures.

The relevant formatter, scene browser, merge, physical-destination and
cross-scene scripts are byte-identical to the starting commit. No readiness
claim is inferred from these checks.
