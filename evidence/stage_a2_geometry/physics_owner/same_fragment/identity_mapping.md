# Read-only Gazebo identity chain — runtime image binding BLOCKED

The successful manual RGB/R32_UINT capture is retained in `accepted_manual/`.
Its fixture-local IDs are **not** Gazebo entity or physics collision IDs.

## Implemented authority sources

- `physics_owner/identity_inventory.hh` enumerates every World, Model, Link,
  Visual and Collision component from the ECM passed to the existing
  `OwnerRecord` callback. Parent IDs are full-width Gazebo entity IDs. Entities
  missing parents, geometry or material data remain in the inventory.
- `owner_private.inc` records this topology in the same session/iteration/time
  record as the existing authoritative `entityCollisionMap` and live DART
  ShapeNode bindings. No second backend, reconstruction or pose export is added.
- `renderer_identity.hh` accepts the live Gazebo `SceneManager` and uses its
  supported `VisualById(Entity)` lookup. It checks actual parent pointers,
  reads `Visual::Id()` without narrowing Gazebo IDs, and rejects untracked
  geometry-bearing renderer visuals. It neither renders nor infers timing.
- `stage_a_fragment_identity.bind_visual_collisions` checks the complete
  inventories and produces a deterministic identity join. It requires exactly
  one visual and one collision per link, unique native uint32 IDs and ShapeNodes,
  supported opaque BOX geometry, exact world/session/step/time context and a
  topology/geometry/ShapeNode-lifetime fingerprint. Names, ordering, proximity
  and dynamic poses are not used.

The public map implementation is in the matching
[Fortress 6.18.0 SceneManager source](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/rendering/SceneManager.cc).
The adapter compiles against the installed headers and library. Gazebo's
integer `gazebo-entity` render userdata is deliberately not used as a full-width
entity map.

## Qualification boundary

Passing the join proves **metadata consistency only**. Renderer context is
provided by its caller; copying that context does not establish render/physics
alignment. The output explicitly retains `mrt_binding=BLOCKED_NOT_WITNESSED`
and `contact_authority=false`.

The existing successful MRT executable constructs its own Ogre v1 prefab
entities and assigns constant `entityId` uniforms. It does not consume the live
Gazebo `RenderUtil::SceneManager()` or its geometry objects. A native uint32 value
coinciding with a Gazebo renderer ID would therefore prove nothing.

**Exact remaining interface:** an acquisition hook owning the live Gazebo
`RenderUtil::SceneManager()` must bind each mapped visual's actual native ID to
the ID output of that visual's same RGB geometry draw, retaining the complete
inventory and independently established owner/render context. The new lookup
adapter is not that draw/capture hook.

No Gazebo identity experiment or EPD inference is run while this image binding
is absent. No historical penetration evidence is imported into a new capture.
EPD poses/envelopes, the 0.1 mm limit and all contact/extraction gates are intact.

## Focused reproduction (CPU / build only)

```sh
python3 -m pytest -q tests/test_stage_a_fragment_identity.py tests/test_stage_a_visual_collision_identity.py
cmake -S scripts/stage_a_rgbd/physics_owner -B /tmp/stage_a2_owner_build
cmake --build /tmp/stage_a2_owner_build --target owner_identity_inventory_test workcell_owner_physics -j1
ctest --test-dir /tmp/stage_a2_owner_build -R '^owner_(bindings|identity_inventory)$' --output-on-failure
```

Python regressions include actual retained GPU buffer verification, two-object
joins with entity IDs above 32 bits and shuffled inventories, duplicate/missing
IDs, multiple visuals sharing a collision, ambiguous ownership, missing
ShapeNodes, unsupported geometry/materials, scene/session/step changes and
renderer-parent disagreement. C++ integration uses a real ECM but no renderer
scene or physics run; the null-scene adapter check must fail closed. Neither
synthetic test records nor compilation establish live identity association.
