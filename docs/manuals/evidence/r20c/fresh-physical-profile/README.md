# R2.0c fresh physical profile handoff — 2026-09-16

Status: production fix automated checks PASS; full user acceptance PENDING.
PR #3170 stays draft. No real motion, EPD implementation changes, physics or R2.0d/e.

Root cause: the New Cell wizard published UI metadata and TaskIntent but no
physical authored environment/model. Product View correctly required a real
Xacro expansion. The failed user scene had no `urdf/scene.urdf.xacro`; extractor
exit 2 hid that underlying error. This was not a viewport discovery/lifecycle bug.

The existing generated-scene opening contract needs an initial physical package.
The wizard now materializes the selected reviewed profile into the new scene's
authored environment/layout/equipment Xacro, exports the existing cell definition,
and invokes the existing full initial package generator inside atomic staging.
Create and Open publishes only after strict real-Xacro extraction succeeds.
Generate Scene Package retains its meaning: regenerate derived state after edits;
it is not a prerequisite terminal repair for the initial Product View.

`ur5_2f_workbench` references the reviewed physical sections and equipment model;
it does not copy the canonical task, scene identity, manifests or caches.
The same workbench/bin/camera meshes, transforms and R1.9 placement region are
used. The fresh wizard authors its own TaskIntent. No new geometry or region was
invented. Modified profile rows fail explicitly rather than being silently ignored.

The exporter preserves v2 metadata without inventing a resolved grasp. The legacy
v1 recipe renderer/converter cannot honor v2 policies, so its v2 output is disabled
with an explicit blocker and no fallback/rules; stale recipe previews and runtime
command suggestions are suppressed. The shared resolver/preplanner is unchanged.
This is a safety boundary, not completion of runtime parity. v1 conversion tests
remain green. Published operational references use the destination path, not a
removed temporary generation directory.

Verification (Humble sourced, real X display for GUI checks):

- Existing focused tests: 76 Python + 15 editor + 9 model + 16 wizard = 116 PASS
  (the original 115 plus the prior wizard layout regression).
- 11 targeted tests PASS: five profile contracts, three legacy converter tests,
  and three strict extractor diagnostic/real-expansion tests. Real profile test
  creates a new physical package, verifies identity/meshes/physical equivalence,
  renames it to published destination, regenerates without authored changes,
  runs generated-scene Validate, and rejects outside-region EXACT.
- Humble workcell_builder and all three affected Qt test targets build PASS.
- Actual displayed Studio Setup → Home New Cell → Review → Create and Open used
  `r20c_profile_gui_final_20260916`; Web3D loaded the new cell's reviewed physical
  meshes. No scene files were manually repaired.
- Canonical displayed editor Save/destroy/reopen and repeated canonical/fresh
  Web3D handoff results are recorded alongside this file.
- `git diff --check` PASS. `.worktrees/` untouched; user failed scene authored
  environment and TaskIntent SHA256 still match reproduction.json.

An exploratory full extractor test run was stopped: it includes repository-wide
cache regeneration and assumptions about old committed fallback artifacts. Its
cache refresh changed the failed scene's derived mesh diagnostic index (not its
authored files). Final validation uses the three scoped extractor tests above.
Earlier extra legacy export/generation fixture tests failed on missing required
TaskIntent/physical bounds; these are not treated as passing acceptance evidence.

Commands (repo root, after sourcing Humble and workspace install):

```bash
export PYTHONPATH="scripts:.:${PYTHONPATH}"
python3 -m pytest -q tests/test_task_intent_authoring.py tests/test_task_intent_v2.py tests/test_builder_task_intent.py tests/test_physical_destination.py tests/test_task_intent_resolver.py
python3 -m pytest -q tests/test_new_cell_physical_profile.py tests/test_builder_task_intent_to_task_recipe.py tests/test_scene_urdf_visual_mesh_index.py::test_require_xacro_strict_nonzero_on_simulated_xacro_failure tests/test_scene_urdf_visual_mesh_index.py::test_main_records_successful_real_xacro_expansion tests/test_scene_urdf_visual_mesh_index.py::test_main_records_real_xacro_failure_command_and_output
cmake --build /home/user/workcell_ws/build/workcell_builder --target workcell_builder workcell_task_intent_editor_test workcell_task_intent_model_test workcell_new_cell_wizard_test -j2
```

The full fresh-cell manual save/home/reopen/restart/policy workflow remains unproven.
Use [MANUAL_ACCEPTANCE.md](MANUAL_ACCEPTANCE.md). Visibility alone does not close R2.0c.
Rollback: revert this scoped handoff change; leave authored user cells untouched.
