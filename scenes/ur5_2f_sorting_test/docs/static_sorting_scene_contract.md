# Static Sorting Scene Contract Validator

This validator performs **offline-only** checks that a static sorting scene package is structurally valid and ready for dry-run/runtime testing handoff.

## What it checks
- Manifest existence/readability and required sections (`objects`, `destinations`, `routing`)
- Target field completeness (`id`, `frame_id`, `destination`, and pick/grasp hint support)
- Duplicate target/destination IDs and missing destination references
- Routing consistency between manifest and generated payload
- Payload generator readiness (status PASS, expected target count)
- Per-target payload contract fields (`object_id`, `target_pose.frame_id`, `target_shape`, grasp methods/poses, positive top-grasp local Z, destination fields)
- Destination world-frame enforcement (`destination_pose.frame_id == world`)
- Sequence runner dry-run report generation for `--all`
- No runtime send is attempted by validation

## Usage
### Dry contract validation
```bash
ros2 run ur5_2f_sorting_test validate_static_sorting_scene_contract --print-summary
```

### JSON validation
```bash
ros2 run ur5_2f_sorting_test validate_static_sorting_scene_contract --json
```

### Strict validation (warnings fail)
```bash
ros2 run ur5_2f_sorting_test validate_static_sorting_scene_contract --strict --print-summary
```

## Why this is a template
`ur5_2f_sorting_test` now has a scene contract gate that can be copied to future sorting/inspection/machine-tending scene packages. New scenes can prove manifest/payload/routing/dry-run readiness before runtime integration.
