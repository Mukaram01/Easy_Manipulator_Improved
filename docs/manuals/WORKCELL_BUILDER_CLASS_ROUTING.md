# Workcell Builder Class Routing

Class labels come from EPD snapshot/live bridge metadata. Routing maps class_label to place_zone and destination for preview-only task intent and planning readiness artifacts. Configure unknown/default route to reject bin. Safety constraints: no robot motion, no MoveIt execution, no gripper command, no conveyor command, no real hardware command.

## Saved-task class routing (TaskIntent v2, R2.4A)

A saved Pick & Place task can send each observed class to its own authored placement region. The existing resolver, MoveIt planning and executor are used; there is no separate sorting engine.

### Authoring
In **Task > Where > Route by class**, type `class=region` pairs, for example `cup=default_drop_zone, bottle=place_zone_b`. Clear the field to return to direct pick and place.

The saved task then carries:

```yaml
routing:
  mode: class
  rules:
  - {id: route_1_cup, when: {attribute: class, equals: cup}, destination: default_drop_zone}
  - {id: route_2_bottle, when: {attribute: class, equals: bottle}, destination: place_zone_b}
```

These are blocking task errors:
- **Rules:**
  - two rules for one class (`ROUTING_CLASS_AMBIGUOUS`);
  - a rule without a destination (`ROUTING_DESTINATION_MISSING`);
  - a default rule, or any condition other than `attribute: class` (`ROUTING_CONDITION_UNSUPPORTED`).
- **Conflicts with the rest of the task:**
  - a set **Object class** (`ROUTING_CLASS_FILTER_CONFLICT`). The routes decide what is picked.
  - a routed class that is also excluded (`ROUTING_CLASS_EXCLUDED`);
  - a placement policy other than AUTO (`ROUTING_PLACEMENT_POLICY`);
  - a **Placement region** that is not one of the routed regions (`ROUTING_PRIMARY_DESTINATION`);
  - a saved multi-object sequence (`ROUTING_SEQUENCE_UNSUPPORTED`).
- **Destinations:** every routed region must resolve inside its physical target, and that target must have enabled collision geometry (`TASK_ROUTE_DESTINATION_INVALID`).

### What planning does
- **Selection.** Only observations of a routed class are pick candidates. The class is the one EPD emitted in the observation, never one derived from authored geometry. Any other class is `CLASS_NOT_ROUTED`: it is not picked and stays in the planning scene as an obstacle.
- **Fit.** Each object is checked against its own class's destination before planning. An object that does not fit its bin is refused (`PLACE_OBJECT_DOES_NOT_FIT`). It is never sent to another bin.
- **Planning.** Each candidate is planned only to its own route's destination. The resolution records the selected route in `place_resolution.route` (rule, class, region, target, world pose) and all validated routes in `place_resolution.routes`.
- **Revalidation.** Revalidation and execution use exactly the resolved destination. A handoff whose destination is not the object's route is refused (`TASK_ROUTE_CHANGED`).
- **Generate.** Generate exports every routed destination, and the routed classes as the perception adapter's required classes.

### Limits
- **Physics.** Class routing is qualified for **fake-hardware planning only** (Plan Saved Task). The qualified Gazebo physics world binds one destination, so a routed task is refused there (`TASK_ROUTING_PHYSICS_UNQUALIFIED`). Physical sorting needs a separately qualified multi-destination world.
- **One object per run.** One planned cycle sends one object to its bin. It is not continuous multi-object sorting.
