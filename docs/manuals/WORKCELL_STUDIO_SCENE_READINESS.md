# Workcell Studio Scene Readiness Matrix

Use the scene readiness matrix when you need a catalog-backed view of whether
supported Workcell Studio scenes have enough evidence to proceed through the
safe simulation-first readiness path.

The matrix is broader than a visual screenshot audit. Scene3D visual-quality
screenshots are only one readiness category. The matrix also checks package
structure, generated artifacts, schema validation, manifest references,
fake-hardware launch derivation, and whether ROS-dependent checks were skipped
or evaluated in the current environment.

## Run the readiness matrix

Run the matrix from the repository root:

```bash
python3 scripts/run_workcell_studio_scene_readiness_matrix.py --supported-scenes scenes/supported_scenes.yaml --output-dir build/workcell_studio_scene_readiness
```

The command reads `scenes/supported_scenes.yaml` as the supported-scene catalog
and writes the readiness report artifacts under
`build/workcell_studio_scene_readiness`.

## Output files

The readiness matrix writes these files:

- `build/workcell_studio_scene_readiness/scene_readiness_summary.json`
- `build/workcell_studio_scene_readiness/scene_readiness_summary.md`

Use the JSON output for CI, dashboards, and machine-readable regression checks.
Use the Markdown output for quick human triage in PRs, demo-readiness reviews,
and scene owner handoffs.

## Status definitions

The matrix uses these status values:

- **PASS**: required evidence is present and valid.
- **FAIL**: required evidence exists but is invalid or internally inconsistent.
- **BLOCKED**: readiness cannot be evaluated because required inputs, generated
  artifacts, ROS environment, or visual evidence are missing.

`BLOCKED` is intentionally distinct from `FAIL`: it means Workcell Studio cannot
make a trustworthy readiness judgment yet because a prerequisite is absent or the
current environment cannot evaluate it. Do not hide a broken supported scene by
removing it from the supported-scene catalog; keep the blocker visible until the
missing input or environment is restored.

## Readiness categories

The readiness matrix combines several categories so that a scene is not treated
as ready based on one signal alone:

- package structure expected by the supported-scene catalog;
- required authoring files and generated artifacts;
- `cell_definition.yaml` schema validation;
- `scene_manifest.yaml` references and internal consistency;
- mesh, screenshot, and visual evidence needed for Scene3D quality review;
- fake-hardware RViz/MoveIt launch derivation from catalog/package metadata;
- ROS skip/evaluation state, including whether ROS-dependent checks were
  actually evaluated or explicitly skipped because the environment was missing.

Scene3D screenshots help demonstrate visual quality and layout fidelity, but
they do not prove that the package contract, generated artifacts, manifest
references, or fake-hardware launch path are ready.

## Recommended triage loop

After the first failing scene, fix issues in dependency order instead of jumping
straight to RViz/MoveIt launch debugging:

1. Fix missing authoring and generated files first.
2. Validate `cell_definition.yaml` and correct schema or consistency errors.
3. Fix `scene_manifest.yaml` references so the package index points at real,
   current artifacts.
4. Regenerate mesh, screenshot, and other visual evidence required for the
   Scene3D visual-quality category.
5. Only then evaluate fake-hardware RViz/MoveIt readiness.

This order keeps the earliest source-of-truth blockers visible and prevents ROS
launch failures from masking simpler missing-file, schema, or manifest-reference
problems.

## Safety notes

The readiness matrix is a simulation-first evidence report. Fake hardware
remains the default readiness path, and the matrix does not invoke real robot
motion. Any later real-hardware commissioning work must remain behind explicit
guarded flags, preflight checks, and clear operator warnings.
