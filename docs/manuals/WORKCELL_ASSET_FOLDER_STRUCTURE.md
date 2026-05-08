# Workcell Asset Folder Structure

Recommended folder organization:

- `assets/robots/{universal_robot,fanuc,panda_robot,placeholders}`
- `assets/end_effectors/{robotiq_85_gripper,robotiq_3f_gripper,onrobot_airpick4,single_suction_gripper,placeholders}`
- `assets/environment/{tables,workbenches,bins,conveyors,fixtures,machines,safety,cameras,placeholders}`
- `assets/objects/{primitives,boxes,cylinders,demo_parts,custom}`
- `assets/sensors/{realsense,placeholders}`

## Rules
- Keep existing ROS package paths stable.
- Add category aliases in catalog metadata when paths differ from user-facing category names.
- Mark preview-only assets explicitly with `preview_only: true` and clear notes.
- Use lightweight local placeholder meshes under `meshes/` when runtime packages are not available.
