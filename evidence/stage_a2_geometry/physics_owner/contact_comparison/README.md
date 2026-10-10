# Matched-step stock contact collection — 2026-10-10

Baseline `fa3d27ea0c189ff1ff43d14681a86bac56b6aeeb`, clean isolated PR worktree.
**Contact occurrence/positions/counts PASS for six finite samples. Full contact
comparison and runtime qualification remain BLOCKED: stock normals/depths are
not exposed. No penetration qualification, native permission or MoveIt run.**

## Lifecycle correction and stock API limitation

The previous observer requested `ContactSensorData` only in Configure, before
collision entities were available. The observer now implements ISystemPreUpdate:
for every existing collision it creates a missing request component, preserving
already populated components. This also handles late collision creation. Physics
Update then fills these components; PostUpdate records them at configured steps.
Stock and owner runs use the same observer, request every collision and record
both collision inventory and requested IDs, request step, read step/time/dt and
phase. The default non-diagnostic capture mode does not request contact data.
Neither solver settings nor physical contact response is changed.

The pinned Fortress 6.18.0 `PhysicsPrivate::UpdateCollisions` implementation
copies only backend contact positions into these components. It omits the extra
normal/depth data that the installed DART backend returns internally. Thus the
observer reports empty normal/depth arrays explicitly. `Component::SetData`
always copies the new data; its tolerance affects change notification, not the
stored contact positions. No cache-tolerance allowance is invented.

The supported stock [Contact sensor System](https://raw.githubusercontent.com/gazebosim/gz-sim/ignition-gazebo6_6.18.0/src/systems/contact/Contact.cc)
uses the same PreUpdate request/PostUpdate read lifecycle and forwards the same
component message with a timestamp. It cannot restore omitted fields. The public
`CollectContactSurfaceProperties` event requires optional
`SetContactPropertiesCallbackFeature`; the actual installed DART plugin does not
advertise that interface. `installed_backend_features.txt` records the plugin
loader inspection; `GetContactsFromLastStepFeature` is present. No contact event
or sensor values were fabricated and no system library was modified.

The owner readback now samples the same schedule and reports live DART contact
positions, normals, depth and authoritative collision IDs alongside node
identities. Reading LastCollisionResult after ForwardStep does not imply that
its contact points were evaluated against the post-integration transforms;
these records identify the producing/read physics step, not a newly proved
contact-evaluation instant. They cannot independently bound endpoint penetration.

## One new bounded comparison

Actual output: `/tmp/stage_a2_contact_comparison`. Exactly one stock server and
one owner server, each 2000 steps at 1 ms, each bounded to 60 wall seconds. Both
servers exited 0. The comparison runner correctly exited **2** for missing
normal/depth evidence. No second campaign was run.

Samples: **100, 250, 500, 1000, 1500, 2000**, with exact simulation timestamps
100000000, 250000000, 500000000, 1000000000, 1500000000, 2000000000 ns. Both runs
requested all eight collision entities before Physics Update at each sample.
Every sample contains pairs **(7,23)** and **(7,27)**, four points per pair:
**eight unique contacts**. Per-collision reporting duplicates each pair in both
directions; the comparator preserves multiplicity and canonicalises point order
within each directed pair. Reciprocal copies are not counted as extra contacts.

Across all six matched samples:

- Collision-pair identities, occurrence, point counts and positions match
  **exactly**, using zero numerical tolerance.
- Whole ECM records are identical, including the original BOX geometry,
  cached cube transforms, timestamps, steps, contact requests and inventories.
  Sampled transform changes therefore agree; instantaneous motion between
  samples and universal trajectory equivalence are not claimed.
- Owner raw contact points match stock ECM contact points exactly through the
  authoritative collision mapping, independently of node-address enumeration.
- Authored physics configuration and gravity match exactly. Loaded DART/physics
  library hashes match between runs; each process loads its single intended
  Physics System. Full runtime path/hash provenance is in `comparison.json`.
  This is not independent readback of every stock solver internal parameter.
- Stock/owner-facing normal/depth arrays are empty. Full field comparison is
  **BLOCKED_MISSING_NORMAL_OR_DEPTH**, despite the successful available-field
  comparisons. Manifold depth is never used as an exhaustive penetration proof.

The scope is these six samples of this identical two-cube world under the
recorded binaries and configuration. It does not certify arbitrary worlds,
controllers, motion or the stock binary's unexposed normal/depth values.

## Reproduction and verification

```bash
cmake -S scripts/stage_a_rgbd/physics_contact -B /tmp/stage_a2_physics_contact_build
cmake --build /tmp/stage_a2_physics_contact_build -j1
ctest --test-dir /tmp/stage_a2_physics_contact_build --output-on-failure
cmake -S scripts/stage_a_rgbd/physics_owner -B /tmp/stage_a2_owner_build
cmake --build /tmp/stage_a2_owner_build -j1
PYTHONPATH=scripts python3 -m pytest -q tests/test_stage_a_owner_contact_compare.py tests/test_stage_a_physics_contact.py
python3 scripts/stage_a_rgbd/physics_owner/compare.py \
  --world /tmp/workcell_stage_a2_dimensions/world.sdf \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --observer /tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so \
  --output /tmp/NEW_CONTACT_COMPARISON
```

Builds PASS. Lifecycle CTest PASS; existing binding CTest PASS. Focused Python
suite **41 PASS** (13 new comparison tests, 28 existing physics-contact tests).
The lifecycle test first failed without PreUpdate. Comparison tests first failed
without the verifier. Independent review identified missing cross-run inventory
comparison; a failing regression was added, then corrected and verified.
Regression cases cover order canonicalisation, missing/empty captures, missing
pairs, partial fields, NaN, exact position/normal discrepancies, mismatched
inventories, timestamps/sessions and duplicate steps. New backend penetration
qualification tests were not started because the contact-comparison gate failed.

## Exact next action and preserved gates

**Next action:** add reporting-only forwarding of backend `ExtraContactData`
normal/depth to a disposable, matching-source reference Physics System and
compare it with owner readback at matched steps. That would qualify a rebuilt
reference reporting path; it must not be labelled direct observation of fields
from the unmodified stock binary. The current stock public API cannot supply
these fields with this installed backend.

Physical deepest-penetration upper bound remains **unknown**. The existing
rational evaluator and unchanged 0.1 mm threshold were not promoted to backend
authority. EPD renderer-exposure/physics-state alignment and unique bounded mask
identity remain independently BLOCKED. Original EPD poses, envelopes and
uncertainties, frozen-replay restrictions, bridge restrictions and native
contact checks remain unchanged. No ROS, bridge, MoveIt or execution goal was
launched. Installed libraries, production worlds and protected Stage-A1 checkout
were not modified. PR #3176 remains draft.
