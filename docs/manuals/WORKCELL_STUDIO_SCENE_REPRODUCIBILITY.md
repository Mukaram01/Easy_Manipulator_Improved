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
