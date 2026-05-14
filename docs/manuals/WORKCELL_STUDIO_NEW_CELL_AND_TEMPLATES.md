# Workcell Studio: New Cell + Templates

New Cell now uses the existing `workcell_builder` scene generation path (no Streamlit replacement, no EPD merge).

## Templates
- Pick and Place Cell
- Conveyor Sorting Cell
- Camera Inspection Cell
- Machine Tending Placeholder (PREVIEW_ONLY)
- Bin Picking Placeholder (PREVIEW_ONLY)
- Palletizing Placeholder (PREVIEW_ONLY)

Each template is safety-first and shows status (launch-ready vs preview-only).

## Supported robot/tool combinations
- UR5 + Robotiq 2F
- UR5 + suction
- UR10 + Robotiq 2F (when assets/config are present)
- Delta/cartesian + suction placeholders are explicitly PREVIEW_ONLY

## Recommended layout
Use **Use Recommended Layout** to write/update practical layout metadata for the selected scene/template using the existing scene metadata files.

## Output
Generated scenes are written under the selected scenes root and include scaffold files such as:
- `environment.yaml`
- `scene_manifest.yaml` (when supported)
- `config/workcell_builder_task_intent.yaml`
- `config/task_recipe.yaml`

## Gripper default mount RPY
Default for new/generated template flows:
- `-1.5708 -1.5708 0`

This remains user-editable.

## Safety note
No robot motion is commanded automatically.
- `fake_hardware_first: true`
- `runtime_execution_enabled: false`
- `motion_command_sent: false`

EPD remains separate/external to Workcell Studio.
