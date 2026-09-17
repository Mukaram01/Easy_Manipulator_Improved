# A0.3 — full-path grasp admissibility and initial support contact

Scope: MoveIt **plan-only**. No physics, perception integration, robot execution,
physical grasp claim or later Stage-A work. The preserved A0 ground-truth snapshot
is a commissioning replay input only.

## Authority and correction

Ranking already proposes candidates to the existing resolver/selection adapter
and `full_cycle_preplanner`. The A0.2 prototype bypassed that authority. The
production correction adds straight-lift corridor validation, collision-aware IK
when the private and live scenes match, and structured rejection diagnostics.
Ranking, TaskIntent AUTO/PREFERRED/EXACT and observation poses remain unchanged.

MoveIt previously classified approximately 14.4 nm of settled-object/floor overlap
as a collision at the first carried-object state. Its start-state adapter moved
the planned start, causing the straight-lift corridor check to reject the path.

`workcell/InitialSupportContact` is a request adapter in the existing MoveIt
pipeline. It uses MoveIt's per-contact conditional ACM API in a **private scene**,
not a second planner or a Boolean object/bin collision exemption.

- Fixed inclusive `support_contact_tolerance_m = 1e-4` (0.1 mm), an engineering
  tolerance, not a constant fitted to the observed overlap.
- Only an identified BOX observation / declared support or reviewed placement
  asset pair qualifies. Reviewed XY footprint and height exclude lip locations.
- The floor is independently certified from the actual collision geometry's
  lowest upward surface beneath the whole object footprint. The conservative
  `usable_placement` box is not treated as a physical floor.
- All FCL contacts are checked: depth, body types, pair identity, upward normal
  and floor height. Fingers, walls/lips, other parts and unrelated obstacles keep
  their normal collision rules, even below the numerical tolerance.
- Only the first short straight-lift request may activate the adapter. Every
  stored waypoint after the original start is strictly collision-valid. Ordered
  intermediate checks permit only initial separation; recontact fails. Sampling
  uses a conservative carried-point travel bound (including mimic joints) tied
  to one quarter tolerance, 25 micrometres. These are sampled MoveIt checks, not
  a claim of continuous collision detection.
- The adapter validates bounds, constraints, unchanged start and the lift corridor
  independently. No adapter-added indexes or global/live ACM mutations are used.
- All subsequent lift/transfer/place requests retain the original strict ACM.
- Support-transition plans cannot be executed by `--start`; execution remains
  uncommissioned. Existing canonical fake-hardware paths without this transition
  retain their existing behavior.

The supported numerical-contact case is currently horizontal, unrotated support and BOX
observations, matching the existing runtime. Tilted/ambiguous/unsupported support
geometry fails closed. This does not broaden robot/tool or task-strategy support.

## Reproduction

Build `workcell_builder` in the existing Humble workspace. Launch the canonical
scene with fake hardware, RViz off and execution disabled, on an unused ROS domain:

```bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false
```

Use the existing commissioning replay interface. `settled-parts.yaml` contains the
unaltered ten A0 poses. Its confidence threshold identifies the same two candidate
parts; the other eight remain collision obstacles. The recipe below prepares a
fresh temporary fixture from the canonical scene; it is not production logic.
Set `EMD_REPO` to this checkout and `A03_REPLAY` to a new temporary directory.

```python
import os, shutil, time, yaml, sys
from pathlib import Path
repo, output = Path(os.environ['EMD_REPO']), Path(os.environ['A03_REPLAY'])
output.mkdir(exist_ok=False)
scene = output / 'scene'; scene.mkdir()
for name in ('environment.yaml', 'cell_definition.yaml', 'config', 'layout'):
    src = repo / 'scenes/ur5_2f_test' / name
    (shutil.copytree if src.is_dir() else shutil.copy2)(src, scene / name)
cell = yaml.safe_load((scene / 'cell_definition.yaml').read_text())
cell.pop('builder_task_intent', None)  # Existing legacy commissioning adapter.
cell['task']['grasp']['approach_distance_m'] = .20
cell['task']['grasp']['retreat_distance_m'] = .20
(scene / 'cell_definition.yaml').write_text(yaml.safe_dump(cell))
evidence = repo / 'docs/manuals/evidence/a03'
snapshot = yaml.safe_load((evidence / 'settled-parts.yaml').read_text())
task = yaml.safe_load((evidence / 'task.yaml').read_text())
sys.path.insert(0, str(repo / 'scripts'))
import runtime_pick_inputs as inputs, perceived_object_grasp_plan as geometry
objects = inputs.normalize(inputs.replay_snapshot(snapshot, time.time()), time.time(), geometry)
obstacles = [o for o in objects if o['confidence'] < task['min_confidence']]
import rclpy
from moveit_msgs.srv import ApplyPlanningScene
rclpy.init(); node = rclpy.create_node('a03_replay_obstacles')
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
assert client.wait_for_service(timeout_sec=20)
future = client.call_async(ApplyPlanningScene.Request(scene=inputs.scene_diff(obstacles)))
rclpy.spin_until_future_complete(node, future, timeout_sec=10)
assert future.done() and future.result().success
node.destroy_node(); rclpy.shutdown()
snapshot['objects'] = [o for o in snapshot['objects'] if o['confidence'] >= task['min_confidence']]
(output / 'detections.yaml').write_text(yaml.safe_dump(snapshot))
```

```bash
python3 "$EMD_REPO/scripts/perceived_object_grasp_execute.py" \
  --scene-package "$A03_REPLAY/scene" \
  --task-request "$EMD_REPO/docs/manuals/evidence/a03/task.yaml" \
  --detections "$A03_REPLAY/detections.yaml" --replay --timeout 600 \
  --summary-output "$A03_REPLAY/result.json"
```

No `--start` is supplied. All scene inputs and outputs must belong to the isolated
replay domain. Original A0/A0.1/A0.2 temporary harnesses remain preserved.

Validation and actual rejection/selection evidence are in `acceptance.json`.
A successful motion plan is not evidence of physical contact retention.

Implementation references: [Humble FCL conditional-contact checks](https://github.com/moveit/moveit2/blob/humble/moveit_core/collision_detection_fcl/src/collision_common.cpp)
and [Humble planning-pipeline path validation](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp).
The adapter independently qualifies the initial state; it does not rely on the
pipeline's permissive start-index behavior as evidence of valid contact.
