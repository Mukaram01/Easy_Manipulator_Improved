# Lossless inventory comparison — CONFIRMED; acquisition BLOCKED

Baseline `0e38ee51`. Exactly **one** separately recorded disposable Gazebo run.
Both complete inventories, per-field numeric representations and authoritative
ECM component/parent diagnostics are retained in `capture.json.gz` and
`comparison.json`. Historical evidence and the physics-owner ELF are unchanged.

## Minimal integration

`inventory_compare.hh` replaces raw JsonCpp equality at the live inventory join
with a schema-aware comparison indexed by exact `Json::UInt64` entity IDs.
Positive signed and unsigned integers compare by exact value without conversion
to floating point. Negative, floating-point or missing entity IDs are invalid.
Geometry values compare exactly; there is no new tolerance or rounding budget.
Arrays of entity rows are compared by ID, preserving missing/extra entities and
all original snapshots; duplicates and ambiguous parent/ownership roles reject.
Unknown fields, unsupported geometry/materials and incomplete schemas reject.
Representation-only differences are reported separately from changed values.

`inventory_ecm_diagnostics.hh` records actual component type IDs/names and the
parent chain for every inventory entity. Diagnostic entity names are never used
for ownership. It does not delete, reclassify into a collision owner, or exempt
any entity. Both inventories are retained before validation can throw.
The existing SceneManager mapping, two-cube inventory count, native Item,
attachment, depth, visibility, timing and contact gates remain in place.
No change to owner stepping, DART maps, collision geometry or planning geometry.
The opt-in runner now pins the two new live-source headers in its preflight.

## Focused tests and builds

- A CPU red regression first reproduced `a != parse(write(a))` for UInt64 IDs,
  then failed the new comparison assertion before implementation (`red.log.gz`).
- Pure native comparison test PASS: exact JSON round trip, signed/unsigned
  equivalence, adjacent IDs at the full uint64 maximum, floating/negative ID
  rejection, missing/extra entities, exact geometry-field differences, duplicate
  IDs, ambiguous visual/collision ownership, nonfinite/unsupported geometry and
  retention/rejection of the third visual.
- Existing native ECM inventory test extended with an actual Light entity and
  child Visual, validating authoritative parent classification and continued
  rejection. Both owner native tests PASS.
- **91 focused Python tests PASS; three native test targets PASS** (one comparator
  and two owner tests). Live capture and affected native test builds PASS.
- These CPU tests grant no rendering/contact authority. No broad suite or
  unrelated rebuild; installed libraries and protected Stage-A1 untouched.

## One actual runtime result

```sh
python3 scripts/stage_a_gazebo_fragment.py \
  --world /tmp/workcell_stage_a2_separated/physical_world.sdf \
  --binary /tmp/stage_a2_live_fragment_build/stage_a_gazebo_fragment_capture \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --output /tmp/stage_a2_inventory_join_20261010_0e38ee51
python3 scripts/stage_a_gazebo_fragment.py \
  --output /tmp/stage_a2_inventory_join_20261010_0e38ee51 --run-prepared
```

Preflight passed: one qualified owner, two byte-preserved original cube models,
fresh output/execution claim/session, no robot/controller/sensor interfaces.
`LIBGL_ALWAYS_SOFTWARE=1`, unique transport partition, GDB and `timeout 60s` were
used; no second launch. Executed ELF SHA256:
`1dcd68314933e346c1d67b5c09e50fad3a42cb7967ca680c5f1096ee317ffd79`.
Owner SHA256 remained
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`,
and was witnessed mapped. Source, debugger, input/world and loaded-library hashes
are retained in the preflight/result artifacts.

- Child and supervisor exited **2**, normally; no SIGILL/SIGSEGV. Historical crash
  cause remains unproven; no claim of universal crash freedom.
- Complete owner step **2**, timestamp **2,000,000 ns**, world entity **1**.
- **Raw JSON equality false; schema-aware value equivalence true.**
- **Zero value differences, zero missing/extra entities.** Exactly **20** positive
  integer representation differences: IDs and parents for world 1, models 4/8,
  links 5/9, visuals 6/10/13 and collisions 7/11. Owner JSON parses as signed
  integer (`intValue=1`), live ECM extraction uses unsigned (`uintValue=2`). All
  values agree exactly. `comparison.json` lists every full-width ID/path/value.
- The old mismatch was therefore numeric representation, not scene mutation, in
  this tested run. No tolerance or entity exclusion was needed to establish it.

## Visual 13: authoritative classification, still rejected

Actual component/parent chain: **13 → 12 → 1**.

| Entity | Authoritative components / role |
|---|---|
| 13 | Visual, ParentEntity=12, Pose, CastShadows, Transparency, Name; **no Geometry, Collision, Link or Model** |
| 12 | **Light**, LightType=`directional`, ParentEntity=1; no Link, Model or Collision |
| 1 | World, plus actual physics/render configuration components |

Names `sunVisual` / `sun` are diagnostic corroboration only. Classification is
based on components and parent IDs: visual child of an authoritative directional
Light entity without Geometry. It is **not** a physical cube visual with a unique
link/collision owner. Nothing establishes a collision identity for it.

Both inventories include visual 13 unchanged. The exact rejected schema fields,
on **each** side, are:

- `/visuals/13/geometry`: `UNSUPPORTED_OR_MISSING`, no BOX dimensions.
- `/visuals/13/opaque`: false / no supported material witness.
- `/visuals/13/parent`: 12 has the Light role, not an enumerated physical Link.

The process stopped with `unsupported complete inventory; see
inventory_comparison/issues`. The old two-visual/SceneManager gates would also
remain unsatisfied; they were not relaxed. SceneManager mapping, MRT draw,
occlusion and RGB/uint32-ID buffers were **not reached**. Pixel counts unavailable.
Render/physics synchronization, EPD association, penetration, extraction and
contact authority remain BLOCKED. No EPD, MoveIt, execution goals or new ACM
permissions; original envelopes and 0.1 mm threshold unchanged.

## One next action

Explicitly omit the authored directional light from the **disposable unlit MRT
fixture world**, which does not use lighting, with a focused regression proving
that cube model/collision XML and strict complete-inventory rejection remain
unchanged. Do not silently filter visual 13 out of captured inventories or exempt
an unsupported entity. This task made no such world change and conducted no retry;
downstream acquisition gates remain unqualified.

Raw evidence uses deterministic gzip where appropriate. `summary.json` records
uncompressed artifact SHA256 hashes; inspect with `gzip -dc`. Both snapshots and
all component type IDs are preserved losslessly, including values exceeding
signed 64-bit or 32-bit ranges where applicable to the regression/type IDs.
