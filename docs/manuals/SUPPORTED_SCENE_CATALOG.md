# Workcell Studio Supported Scene Catalog Contract

`scenes/supported_scenes.yaml` is the single source of truth for scene packages that Workcell Studio treats as supported reproducibility targets.

All all-scenes reporting and readiness scripts must consume this catalog instead of hardcoding scene names. The catalog exists so each supported scene can answer, in one machine-readable place:

- what scene/package name Workcell Studio supports;
- which authoring and generated files are required;
- which command validates the generated scene contract;
- which package must build;
- which fake-hardware RViz/MoveIt launch command demonstrates the safe simulation path;
- whether the scene is supported, experimental, disabled, or blocked;
- what known blocker currently prevents full readiness, if any.

## Required fields per scene

Every `scenes[]` entry must define these fields:

| Field | Purpose |
| --- | --- |
| `scene_name` | Directory/name of the supported scene, for example `ur5_2f_test`. |
| `package_name` | ROS 2 package name expected in `package.xml`. |
| `scene_path` | Repository-relative path to the scene directory. |
| `support_level` | `supported` for normal all-scenes checks; `experimental` only for opt-in checks. |
| `status` | Current catalog status such as `supported`, `blocked`, or `disabled`. |
| `known_blocker` | Concrete blocker text, or an empty string when no blocker is known. |
| `authoring_files` | Source-of-truth authoring/editor files that must exist. |
| `generated_files` | Generated scene/package files that must exist. |
| `validation_command` | Explicit scene validation command. It must name the scene. |
| `build_package_name` | Package passed to `colcon build --packages-select`. |
| `build_command` | Explicit package build command. |
| `fake_hardware_launch_command` | Explicit safe launch command. It must include `use_fake_hardware:=true`. |

## Local validation workflow

Run these commands from the repository root to check the catalog-backed local readiness path:

```bash
python3 scripts/validate_supported_scenes_readiness.py --repo-root . --workspace-root . --skip-build --skip-launch-smoke --json
```

```bash
python3 scripts/validate_all_workcell_studio_scenes.py
```

```bash
scripts/check_all_scenes.sh
```

The readiness command above intentionally skips ROS package builds and launch smoke tests for fast local checks. Full fake-hardware launch validation must still keep fake hardware explicit with `use_fake_hardware:=true`; do not make real hardware the default path.

## Status semantics

- `support_level: supported` and `status: supported` are the default quality-gate targets for supported-scene readiness.
- `status: blocked` remains in the catalog for scenes that are intended to be supported but currently cannot pass readiness; every blocked scene must include a concrete `known_blocker`.
- `support_level: experimental` is opt-in and is not part of the default supported-scene gate.
- `disabled` scenes are skipped and must not be used to hide broken supported scenes.

## Required readiness JSON fields

Readiness JSON emitted by catalog-backed validation must include these fields for each scene result:

| Field | Purpose |
| --- | --- |
| `scene_name` | Scene/package name reported from the catalog entry. |
| `status` | Catalog status used by the readiness gate, such as `supported`, `blocked`, or `disabled`. |
| `support_level` | Support tier used to decide default versus opt-in validation scope. |
| `blocker` | Normalized blocker text for readiness output; it should reflect `known_blocker` when the scene is blocked. |
| `missing_authoring_files` | Required authoring/editor files from the catalog that were not found. |
| `missing_generated_files` | Required generated scene/package files from the catalog that were not found. |
| `validation_result` | Result from the explicit scene validation command. |
| `build_command` | Package build command associated with the scene. |
| `fake_hardware_launch_command` | Safe simulation launch command; it must keep `use_fake_hardware:=true` explicit. |

## Update rules for future Codex tasks

When adding, removing, renaming, or intentionally disabling a supported scene:

1. Update `scenes/supported_scenes.yaml` in the same PR as the scene change.
2. Keep `scene_name`, `package_name`, and `build_package_name` aligned unless the difference is intentional and documented in `known_blocker`.
3. Add all required authoring and generated files to `authoring_files` and `generated_files` instead of relying on script-local defaults.
4. Use `support_level: experimental` only for scenes that should not yet be part of the default supported-scene gate.
5. Use `status: blocked` plus a concrete `known_blocker` for scenes that are intended to be supported but currently cannot validate, build, or fake-launch.
6. Never remove a broken scene from the catalog just to make all-scenes reporting pass unless the product decision is to stop supporting that scene.
7. Preserve fake-hardware-first safety: `fake_hardware_launch_command` must use `use_fake_hardware:=true`; real hardware must not become the default.

## Catalog-backed reporting commands

Default supported-scene readiness:

```bash
python3 scripts/validate_supported_scenes_readiness.py --repo-root . --workspace-root . --skip-build --skip-launch-smoke --json
```

All-scenes reproducibility report:

```bash
python3 scripts/validate_all_workcell_studio_scenes.py
```

ROS package scene-contract loop:

```bash
scripts/check_all_scenes.sh
```

For full build and fake-hardware launch validation, omit `--skip-build` and `--skip-launch-smoke` only in an environment with ROS 2 Humble sourced and the workspace ready.
