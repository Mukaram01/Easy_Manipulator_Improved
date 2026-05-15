# Workcell Studio New Cell from Scratch

This guide consolidates New Cell flow behavior, action mapping, audits, and recovery patterns.

## Button and action map
Choose Workspace → New Cell → New Scene → Use Recommended Layout → Add to Canvas → Save Layout → Remove Selected Layout Item → Validate Layout → Generate/Update Task Intent → Open Task File → Copy Task Summary → Generate Scene Package → Refresh Existing Scenes → Run Offline Validation → Generate Readiness Pack → Open Readiness Dashboard → Open Plan & Simulate → Open RViz2 / MoveIt → Run Fake-Hardware Simulation → Stop Simulation → Copy Launch Command.

## Point 2: File-output Audit
Expected files:
- `environment_layout.yaml`
- `config/workcell_builder_task_intent.yaml`
- `package.xml`
- `CMakeLists.txt`
- `launch/demo.launch.py`

## Point 3: State-transition Audit
`NO_WORKSPACE` → `WORKSPACE_READY` → `CELL_DRAFT_CREATED` → `LAYOUT_CREATED` → `LAYOUT_SAVED` → `TASK_INTENT_CREATED` → `SCENE_PACKAGE_GENERATED` → `FILE_OUTPUTS_CHECKED` → `VALIDATION_READY` → `VALIDATION_PASSED` → `PLAN_SIMULATE_READY`.

## Point 4: Error-message Audit
Check expected error surfaces for missing workspace/src/assets, malformed YAML, and missing launch/package build artifacts.

## Point 5: Real Build/Run Audit
Run build smoke and launch contract checks before releasing generated scene packages.

## Point 6: Regression Audit
Run regression scripts to confirm no workflow regressions in New Cell and Plan & Simulate paths.

## Common recovery
- Re-select workspace if missing.
- Save layout before task intent generation.
- Regenerate package if launch artifacts are missing.
- Re-run validation when YAML or manifest content changes.

## Acceptance command
```bash
python3 scripts/generate_scratch_cell_acceptance.py --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
```

Audit outcomes use PASS, WARNINGS, and BLOCKED statuses.
