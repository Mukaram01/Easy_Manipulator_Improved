# Workcell Builder Class Routing

Class labels come from EPD snapshot/live bridge metadata. Routing maps class_label to place_zone and destination for preview-only task intent and planning readiness artifacts. Configure unknown/default route to reject bin. Safety constraints: no robot motion, no MoveIt execution, no gripper command, no conveyor command, no real hardware command.

## Saved-task class routing (TaskIntent v2, R2.4)

A saved Pick & Place task can send each observed class to an authored placement region. Any number of classes and regions is supported, and many classes may share a region. The existing resolver, MoveIt planning and executor are used; there is no separate planner.

### Authoring
In **Task > Where > Route by class**, edit the routing table:
- **Add rule / Delete rule:** add or remove rows.
- **Columns:** Object class, Destination region, Enabled, Priority.
  - The class column offers the labels of the workspace's registered EPD runtime (`workcell_perception_runtime.json` → `epd.labels`).
  - The destination column offers the cell's own placement regions.
- **Unrouted objects:** what happens to a class without an enabled rule.
  - *Leave in place* (not picked, kept as an obstacle);
  - *Stop the sorting session*.
- **Attempts per object:** 1–5.
- **No rows** means direct pick and place into the Placement region.

The saved task then carries:

```yaml
routing:
  mode: class
  unrouted_policy: leave_in_place        # or stop_session
  max_attempts_per_object: 2
  rules:
  - {id: route_1, when: {attribute: class, equals: cup}, destination: default_drop_zone, priority: 10}
  - {id: route_2, when: {attribute: class, equals: bottle}, destination: default_drop_zone}
  - {id: route_3, when: {attribute: class, equals: mouse}, destination: place_zone_b_left}
  - {id: route_4, when: {attribute: class, equals: remote}, destination: place_zone_b_right, enabled: false}
```

These are blocking task errors:
- **Rules:**
  - two rules for one class (`ROUTING_CLASS_AMBIGUOUS`);
  - a rule without a destination (`ROUTING_DESTINATION_MISSING`);
  - a default rule, or any condition other than `attribute: class` (`ROUTING_CONDITION_UNSUPPORTED`);
  - no enabled rule (`ROUTING_RULES_REQUIRED`).
- **Policies:**
  - a missing unrouted policy (`ROUTING_UNROUTED_POLICY_REQUIRED`);
  - an attempts value outside 1–5 (`ROUTING_ATTEMPTS_INVALID`).
- **Conflicts with the rest of the task:**
  - a set **Object class** (`ROUTING_CLASS_FILTER_CONFLICT`). The routes decide what is picked.
  - a routed class that is also excluded (`ROUTING_CLASS_EXCLUDED`);
  - a placement policy other than AUTO (`ROUTING_PLACEMENT_POLICY`);
  - a **Placement region** that is not the destination of an enabled rule (`ROUTING_PRIMARY_DESTINATION`);
  - a saved multi-object sequence (`ROUTING_SEQUENCE_UNSUPPORTED`).
- **Destinations and classes:**
  - every rule's region, including disabled rules, must resolve inside its physical target, and that target must have enabled collision geometry (`TASK_ROUTE_DESTINATION_INVALID`);
  - a class outside the registered EPD vocabulary (`ROUTING_CLASS_NOT_IN_VOCABULARY`). Without a registered EPD runtime, the vocabulary is reported as unknown, never guessed.

### What planning does
- **Selection.** Only observations of a class with an enabled rule are pick candidates. Higher priority is picked first, then the resolver's existing confidence, height and identity order. The class is the one EPD emitted in the observation, never one derived from authored geometry.
  - A class without a rule is `CLASS_NOT_ROUTED`; a class whose rule is disabled is `CLASS_ROUTE_DISABLED`.
  - Either way the object is not picked and stays in the planning scene as an obstacle.
- **Fit.** Each object is checked against its own class's destination before planning. An object that does not fit is refused (`PLACE_OBJECT_DOES_NOT_FIT`). It is never sent to another bin.
- **Occupancy.** In a sorting cycle the resolver also receives every other object in the cell.
  - Places in the routed region are packed from the region corner around objects already there, and up to three are planned.
  - A region with no free place is `DESTINATION_FULL`.
  - The considered objects are part of the resolution identity. Revalidation refuses a place that became occupied (`DESTINATION_OCCUPIED`).
- **Route identity.** The resolution records the selected route and place (`place_resolution.route`) and all routes (`place_resolution.routes`). A handoff whose destination is not the object's route is refused (`TASK_ROUTE_CHANGED`).
- **Generate.** Generate exports each routed region once, and the enabled routed classes as the perception adapter's required classes.

## Sorting sessions (R2.4)
A sorting session sorts the saved cell continuously, one object per cycle, on **MoveIt fake hardware**.
- **From Workcell Builder:** Plan / Simulate → **Start Sorting (Fake Hardware)**, **Pause**, **Resume**, **Stop at Cycle End**.
- **From the Operator HMI:** the **Sorting · fake hardware** card has **Start Sorting**, **Pause**, **Resume** and **Stop**. Workcell Builder owns and runs the session.

### Lifecycle
`IDLE → READY → RUNNING ⇄ PAUSING/PAUSED → STOPPING → STOPPED`; any active state → `FAULT`.
- **Safe boundary.** Pause and Stop take effect at the next cycle boundary: robot home, nothing held. They are software requests, **not an emergency stop**.
- **Hard cancel.** The existing **Stop Simulation** cancels immediately through the supervised cleanup. The session then ends in FAULT (`SESSION_INTERRUPTED`), because a cancelled cycle's world is not verified.

### Each cycle
1. **Observe:**
   - *Batch replay:* the replayed frame is consumed once. Later cycles observe the world reconciled from execution evidence: completed objects at their verified placed pose. The old frame is never read again.
   - *Continuous:* each cycle ingests a new live frame. A frame that still shows a processed object at its pick place is rejected as stale (`PROCESSED_OBJECT_STILL_OBSERVED`), never re-picked.
2. **Select** one object deterministically, using the resolver's eligibility, routes, priority and order.
   - Unrouted and disabled classes follow the unrouted policy.
   - An object that cannot fit its route is rejected before planning.
   - An object whose destination is full waits.
3. **Resolve, plan and execute** that object only. This is one run of the existing executor (`--sorting-cycle`) with every other object as an obstacle and the destinations' occupancy.
4. **Count** the object COMPLETED only from that run's state-verified evidence: all nine stages planned and executed with MoveIt success, controller terminal success, exclusive mock hardware, the object detached once in the world, its final pose contained in the routed destination, and home reached.
5. **Retry or reject:**
   - A failure before motion is retried up to the attempts limit.
   - Deterministic refusals (fit, aperture, tool) are rejected at once.
   - Unverified motion faults the session (`CYCLE_EXECUTION_UNVERIFIED`).
6. **Continue** until no further object can be sorted (`BATCH_COMPLETE`), or the operator stops.

### Counts and evidence
- **Counts:** completed, failed, skipped (unrouted or disabled), waiting (destination full) and pending, per class and per destination.
- **Shown in:** both the Plan / Simulate monitor and the HMI.
- **Evidence:** the session report (`sorting_session.json`) and every cycle's observations and executor evidence are under `log/studio_sorting/<session>/`.

### Fake-hardware observation refresh
A cycle's certified search outlives the 2 s replay age limit. On fake hardware only, observation time is renewed at the execution boundary when the live PlanningScene reproduces every observed identity and geometry exactly (`fake_world_refresh`). The PlanningScene is the fake backend's whole world. The unchanged age limit is then applied; any difference is `SORTING_WORLD_CHANGED`.

### Physics
A sorting session or a routed task is refused in Gazebo:
- `TASK_ROUTING_PHYSICS_UNQUALIFIED` or `SORTING_PHYSICS_UNQUALIFIED`;
- the reason comes from `simulator_world.physics_task_qualification`.

The qualified Stage-A world covers one object, one destination and one cycle. A future multi-destination or continuous-sorting qualification declares a wider `task_scope` in the qualified catalog, with its own contact, retention and placement evidence. The sorting engine needs no change.

### Limits
- **Fake hardware only:** planning and kinematic execution on mock controllers. There is no contact, retention or settling evidence.
- **Continuous mode:** implemented and unit-tested, but no live frame source is wired into the runner. The runner refuses continuous mode (`CONTINUOUS_SOURCE_UNAVAILABLE`).
- **Conveyors:** no conveyor tracking or production batching.
