# Offline Task Execution Plan: generated_ur5_suction_sorting_from_wizard

- Scene: `generated_ur5_suction_sorting_from_wizard`
- Task recipe id: `ur5_suction_sorting_demo_task`
- Task type: `sort`
- Object id: `commissioning_object`
- Object attributes: `class=unknown, colour=unknown, material=unknown, shape=box`
- Matched rule id: `fallback`
- Destination id: `unknown_reject_bin`
- Destination pose: `frame=world, xyz=[0.3, 0.14999999999999997, 0.1], rpy=[0.0, 0.0, 0.0]`
- Destination action: `place`
- End effector: `type=suction, brand=generic`
- Motion context: `planning_group=manipulator, base_frame=world, moveit_link=tool0, grasp_frame=suction_tip`

## Ordered execution steps

| # | step id | step type | action | description | offline/runtime status |
|---|---|---|---|---|---|
| 1 | `acquire_object` | `perception_or_self_test` | `resolve_object` | Use self_test object for offline commissioning. | `offline-only` |
| 2 | `select_grasp_strategy` | `grasp_selection` | `select_strategy` | Select grasp strategy for end effector. | `offline-only` |
| 3 | `move_to_pre_grasp` | `motion_plan` | `plan_pre_grasp` | Plan safe approach to pre-grasp pose. | `runtime-target` |
| 4 | `move_to_grasp` | `motion_plan` | `plan_grasp_approach` | Plan final approach to grasp pose. | `runtime-target` |
| 5 | `close_end_effector` | `end_effector_action` | `activate_suction` | Engage the selected end effector. | `runtime-target` |
| 6 | `attach_object` | `planning_scene_update` | `attach_collision_object` | Attach object to end-effector link. | `runtime-target` |
| 7 | `move_to_pre_place` | `motion_plan` | `plan_pre_place` | Move toward destination approach pose. | `runtime-target` |
| 8 | `move_to_place` | `motion_plan` | `plan_place` | Move to destination pose. | `runtime-target` |
| 9 | `release_object` | `end_effector_action` | `deactivate_suction` | Release object at destination. | `runtime-target` |
| 10 | `detach_object` | `planning_scene_update` | `detach_collision_object` | Detach object from end-effector link. | `runtime-target` |
| 11 | `retreat` | `motion_plan` | `plan_retreat` | Retreat from placement pose. | `runtime-target` |
| 12 | `return_home` | `motion_plan` | `plan_return_home` | Return to configured safe home state. | `runtime-target` |

## Notes

- PyYAML not available: using built-in fallback parser.
- PyYAML not available: using built-in fallback parser.
- Dry-run resolved a matching rule, destination, and action.
