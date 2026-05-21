# Workcell Studio Scene Reproducibility

Use `python3 scripts/validate_all_workcell_studio_scenes.py` to generate
`build/workcell_studio/all_scene_reproducibility_report.json`.

## Status taxonomy

- **PASS**: Required scene metadata/layout files exist and parse; preview/generation metadata is complete for this contract.
- **WARN**: Scene is usable but reproducibility metadata has gaps (for example optional artifacts or runtime-intent metadata missing).
- **FAIL**: Blocking contract issue (missing required file, unreadable YAML/mesh index, or empty renderable mesh contract).
- **SKIP**: Reserved for explicit exclusion flows.

> PASS means the scene contract is reproducible in Workcell Studio metadata terms.
> It does **not** mean real-hardware safety certification, MoveIt execution approval, or grasp safety validation.

## How to fix WARN scenes

1. Open the JSON report and inspect `warning_groups` for each scene.
2. Fix warnings by category:
   - `metadata`: fill or correct scene/source-of-truth descriptors.
   - `preview`: correct layout metadata that affects preview readiness.
   - `generation`: add or regenerate generated artifacts when available.
   - `launch_simulation`: capture launch/simulation warning details when applicable.
   - `runtime_smoke`: document or provide smoke-intent metadata (without claiming unvalidated runtime readiness).
3. Re-run validator and confirm warnings are specific and non-empty.
4. Keep runtime honesty: if smoke launch or MoveIt execution is unvalidated, do not mark runtime-ready.

## Regenerating scene metadata artifacts

Generate readiness metadata for all supported scenes:

```bash
python3 scripts/generate_workcell_studio_scene_artifacts.py --all --overwrite
```

Per-scene outputs:
- `generated/environment_assets.yaml`
- `layout/workcell_studio_layout.generated.yaml`
- `config/task_recipe.yaml`
- `config/workcell_builder_task_intent.yaml`

These generated files are **readiness metadata** for simulation-first workflows. They keep editable layout (`layout/workcell_studio_layout.yaml`) separate from generated/locked preview metadata, and they keep task intent defaults in simulation-safe mode (`use_fake_hardware: true`, `execution_mode: simulation_preview`).

> Important: generated metadata improves reproducibility and audit visibility only. It does **not** prove MoveIt launch success, grasp planning/execution validity, or real hardware readiness.
