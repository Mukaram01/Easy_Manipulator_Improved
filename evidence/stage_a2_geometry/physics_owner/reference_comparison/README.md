# Reporting reference and whole-BOX readback — 2026-10-10

Follow-up: [DART→ODE numerical qualification](../ode_numeric/README.md) closes
the conversion uncertainty for six instrumented contact-input states. The
BLOCKED statements below describe this earlier baseline; EPD association remains
blocked in the follow-up.

**Finite contact comparison PASS. Stored DART geometry enclosure PASS.
Qualified physical penetration remains BLOCKED. No contact authority.**

Baseline: `6a949a2b17cba873bd532512b9daf6a14d37718a`. All work used the
isolated Stage-A2 checkout. No installed libraries, production worlds, planning
poses, envelopes or collision permissions changed. No MoveIt or execution goals.

## Reference implementation and provenance

`prepare.py --reference` verifies the same four hash-pinned Fortress 6.18.0
source files as the owner. `reporting_only.patch` is the complete reporting
change before plugin class renaming: retain pointers to existing optional
`ExtraContactData`, append its normal/depth to the existing contact message,
and reverse the normal for the reciprocal reported collision pair. It does
not alter construction, contacts, stepping, solver settings or response.
`reference_full.patch` includes the necessary isolated plugin class rename;
`owner_full.patch` includes the separate owner instrumentation.

Each disposable world replaces its single Physics plugin; it never adds a
second Physics owner. `comparison.json` records the observed `/proc` loaded
library paths and SHA-256 hashes, matching backend libraries, equal physics XML
and gravity, and absence of the stock Physics System in reference/owner runs.
`provenance.json` records generated source hashes and the upstream source manifest.

| Loaded Physics System | SHA-256 |
| --- | --- |
| Installed stock 6.18.0 | `f1ccf096669cfc6f9cdfb2d153e1e8a6101571f8abcb22a24ca9040b4a86842b` |
| Reporting reference | `12e9d0b7a6b365a45f96fed4754e9c42f3953a4be471bfbb68ce8bbe2a35c687` |
| Instrumented owner | `dbe7f3bafba260a3c0de50f45533540bb8fecfde2ee1547c1ed10bee0d807370` |

All three loaded DART physics plugin hash
`44e5d8573b44ef1f555eba6f2fbf01c1f0ab657b0fd32016c441e884ec5e71a1`.
The loaded DART ODE collision library hash is
`cc914ae1d651bfd79fa7eb308c6f39a12174e8b59d0fb45318652f0eb61761ee`;
ODE hash is `0b40593ed29a7b4f72f364a989ac8d8ea0bdab2a7578518366048b5128c0343b`.

## One bounded comparison

One campaign, three sequential disposable servers, each 2000 steps at 1 ms.
Steps 100, 250, 500, 1000, 1500 and 2000 were sampled. All servers exited zero.
Wall times differed (stock 8.159 s, reference 4.775 s, owner 4.691 s); wall-time
performance equivalence is not claimed. The nominal simulation schedule matched.

A. **Unmodified stock observable fields:** exact agreement against both rebuilt
runtimes for directed collision identities, contact occurrence/counts/positions,
complete eight-collision inventory, cached cube poses/dimensions, step/time/dt,
scene identity and contact request lifecycle. Stock does not expose normal/depth.

B. **Modified reporting reference:** observes normals/depths from the installed
backend through `ExtraContactData`. These are observations of the rebuilt
reference runtime, not measurements of unexposed stock fields.

C. **Instrumented owner:** live DART raw contact normal/depth/position, plus
construction-witness collision mapping, ShapeNode dimensions and transforms.
Reference and owner contact tuples agree **exactly** after reciprocal-normal
orientation and canonical sorting. Each sample contains two unique physical
pairs and eight unique points. Zero tolerance is justified by exact equality of
the serialized binary values; no numerical discrepancy allowance was needed.
This qualifies only the tested world, schedule, binaries and settings.

## Whole-shape geometry and uncertainty

`enclose_owner_geometry` extends the existing evaluator. It verifies hexadecimal
binary64 readback against JSON values, then evaluates all eight corners using
exact rational arithmetic. JSON interval endpoints round outwards. Arithmetic
error is zero for those stored affine inputs; the recorded endpoint enclosure
width accounts for output conversion. This is not a bound on detector conversion.

The complete inventory contains eight BOX shapes, with exactly two mobile cubes.
Cube collision 23 -> physics shape 18, cube 27 -> shape 19, support 7 -> shape 12.
Both cubes are 25 mm BOXes. The actual constructed support is a 2100 m BOX,
identity orientation, centre `(0,0,-1050)`, top `z=0`, footprint
`[-1050,1050]` in both horizontal axes. Both cubes' complete footprints fit.
This evaluator intentionally restricts support to that verified campaign geometry;
other support construction/rotation requires a new qualification.

The owner samples ShapeNode world transforms immediately before `ForwardStep`
and after position integration. In the installed DART step sequence, collision
constraints are evaluated before integration of positions; velocity integration
does not change these input positions. Thus the first snapshot describes DART's
stored geometry entering contact evaluation; the second describes the endpoint.
Neither asserts an independently read ODE transform. Frames are respectively
`step-1` and `step`. DART's accumulated time at frame 2000 is
`1.9999999999998905` s; nominal Gazebo time is exactly `2000000000` ns.
This difference is recorded, not replaced by a guessed clock-error tolerance.

Across both cubes and all sampled pre/post states, the maximum stored-geometry
penetration upper bound is **3.669867925019725 µm**. At frame 2000 the post-step
upper bound is **9.81010502258312 nm**. These enclose all corners, including
actual rotation; they are not deepest-contact-manifold claims. During these
samples cached ECM translation differs from live DART by up to
**0.9884279075778046 µm**. The support ECM PLANE origin and actual constructed
BOX centre intentionally differ by 1050 m. ECM geometry is never substituted.

**Remaining backend proof:** the installed DART/ODE collision object converts
`tf.linear()` to `Eigen::Quaterniond`, then calls `dBodySetQuaternion`. The stored
DART affine matrix therefore cannot silently attest the detector's converted
rotation. No verified rounding/conversion error enclosure or live ODE geometry
readback is present. DART 6.12.1 `OdeCollisionObject.hpp` exposes `getOdeGeomId`
and `getOdeBodyId` only as protected members; a separate Physics owner cannot
legally read them through that API. No private-memory or subclass-cast workaround
was implemented. Physical penetration and backend error bounds remain **null**.

The relevant source path is DART 6.12.1
`dart/collision/ode/OdeCollisionObject.cpp::updateEngineData`; the installed
ign-physics DART construction selects `OdeCollisionDetector`. Loaded libraries
and source inspection support this identified conversion obligation, not an
assertion of universal binary/source equivalence.

## Focused verification and reproduction

Both isolated libraries built; owner construction bindings CTest and observer
lifecycle CTest passed. **60 focused Python tests passed**. Negative cases reject
0.2 mm penetration, rotation with a penetrating lowest corner, wrong ShapeNode
mapping, missing contact/normal, unsupported shape, stale/pre-step identity,
incomplete inventory, finite-footprint overrun, altered support, wrong frame and
unqualified numeric conversion. A supplied zero backend-error scalar still cannot
grant physical authority. Review found no important issues in this scope.

```bash
cmake -S scripts/stage_a_rgbd/physics_owner -B /tmp/stage_a2_owner_build
cmake --build /tmp/stage_a2_owner_build -j2
ctest --test-dir /tmp/stage_a2_owner_build --output-on-failure
ctest --test-dir /tmp/stage_a2_physics_contact_build --output-on-failure
PYTHONPATH=scripts python3 -m pytest -q tests/test_stage_a_owner_geometry.py tests/test_stage_a_physics_contact.py tests/test_stage_a_owner_contact_compare.py
python3 scripts/stage_a_rgbd/physics_owner/compare.py \
  --world /tmp/workcell_stage_a2_dimensions/world.sdf \
  --reference /tmp/stage_a2_owner_build/libworkcell_reference_physics.so \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --observer /tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so \
  --output /tmp/stage_a2_reference_comparison_reproduction
```

The comparison command exits zero for finite contact agreement, while its geometry
results explicitly remain blocked for physical authority. Raw traces permit
offline reevaluation without another simulation. `summary.json` separates all
four decisions. EPD association remains independently unqualified: it still
needs bounded renderer/physics exposure alignment and a unique visible object-ID
witness to genuine EPD masks. No native extraction or MoveIt acceptance was run.

**One next action:** qualify the existing DART-to-ODE matrix/quaternion conversion
with a source- and binary-matched outward error enclosure (or supported readback),
before granting any physical contact evidence authority.
