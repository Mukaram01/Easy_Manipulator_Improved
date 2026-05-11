# Workcell Scene Template Library

This library adds ready-made scene templates for Workcell Studio so users can start from a practical cell instead of a blank scene.

## Included templates
- UR5 Pick and Place Cell
- UR5 Sorting Cell
- Camera Inspection Cell
- Conveyor Pick Placeholder Cell
- Palletizing Placeholder Cell

Templates are stored in `workcell_builder/workcell_builder/config/scene_templates/scene_templates.json` and instantiate `workcell_scene/v1` scenes using curated asset IDs.

## Create a scene from template
Use:
`python3 scripts/generate_workcell_scene_from_template.py --template ur5_pick_place_cell --scene-name my_scene --output-dir /tmp/workcell_template_scene --validate --print-summary`

## Generated artifacts
Template generation emits:
- `environment.yaml` (schema-compatible `workcell_scene/v1`)
- `config/task_recipe.yaml`
- `workcell_template_summary.json` and `workcell_template_summary.md`
- `workcell_studio_summary.json` and `workcell_studio_summary.md`
- `preview/workcell_preview.svg` and `preview/workcell_preview.html`

Preview artifacts are deterministic and include scene name, robot/tool, placed object names, camera marker, and a fake-hardware-first note.

## Validation, smoke, and bundle round-trip
Template scenes should pass:
- scene schema validation (`scripts/validate_workcell_scene.py`)
- fake-hardware static smoke (`scripts/run_workcell_fake_hardware_smoke.py --skip-launch`)
- export/import bundle flow (`export_workcell_scene_bundle.py`, `validate_workcell_scene_bundle.py`, `import_workcell_scene_bundle.py`)

## Safety
Template generation is offline-only and preserves:
- `fake_hardware_first: true`
- `real_hardware_enabled: false`
- `runtime_execution_enabled: false`
- `motion_command_sent: false`
- `moveit_plan_service_called: false`

Golden Demo remains separate developer/test tooling.

## Troubleshooting common validation warnings
- If workspace warnings appear, ensure `workspace.bounds` includes `x_min/x_max/y_min/y_max/z_min/z_max`.
- If camera warnings appear, ensure camera fields include `enabled`, `camera_id`, `frame_id`, and six-number `pose`.
- If compatibility warnings appear, set `compatibility.status` to a concrete status (for templates: `COMPATIBLE`).
- If smoke warns about launch guidance, ensure `workcell_studio_summary` includes `demo.launch.py use_fake_hardware:=true launch_rviz:=false`.
