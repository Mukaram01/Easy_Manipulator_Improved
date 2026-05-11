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

Outputs:
- `environment.yaml`
- `config/task_recipe.yaml`
- summary json/md
- validation and safe fake-hardware references

## Safety
Template generation is offline-only and preserves:
- `fake_hardware_first: true`
- `real_hardware_enabled: false`
- `runtime_execution_enabled: false`
- `motion_command_sent: false`
- `moveit_plan_service_called: false`

Golden Demo remains separate developer/test tooling.
