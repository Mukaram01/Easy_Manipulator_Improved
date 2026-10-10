# Working simulation-only physics contact provider — 2026-10-10

Baseline verified clean: `7885a1a40e75399db4010b697c0ca23e7415fd78`, isolated
`/home/user/workcell_ws_stage_a2`. Code and runtime functionality are implemented;
**physical penetration proof, EPD binding and native extraction remain BLOCKED**.
No MoveIt acceptance or execution goal was sent.

## Implemented and run

- `scripts/stage_a_rgbd/physics_contact/provider.cpp`: read-only Fortress
  PostUpdate system. Every active physics step publishes loaded collision entity
  IDs/scoped names, run/world identity, step/time/dt, complete dynamic inventory,
  BOX dimensions/state and static support PLANE geometry. Missing, duplicate,
  unknown dynamic or unsupported collision geometry marks the record incomplete.
- Existing `capture.cpp`: optional final run-ID argument subscribes to the
  private-partition measurement stream and requires a record at the **exact**
  synchronized RGB/depth/info timestamp. Raw internal state goes only into
  separate `physics_measurement.json`; normalized EPD geometry receives only
  measurement reference/identity fields and explicit mask-file association.
  Default capture mode does not subscribe to the provider.
- `stage_a_physics_contact.py`: evaluates all eight BOX corners against the
  horizontal support plane using exact rational arithmetic on reported binary
  inputs, then rounds output endpoints outward. Linear plane clearance reaches
  its minimum at a BOX vertex. No manifold point or contact count is used to
  infer exhaustive coverage. Arbitrary rigid BOX rotations are handled;
  non-rigid matrices and tilted/unsupported support planes reject.
- The same isolated evaluator projects the complete physical BOX into the
  optical frame, encloses its eight projected vertices, and builds an EPD
  mask-intersection candidate graph. A bijection is necessary; ambiguity,
  missing masks, wrong frames, stale/different steps/sessions, wrong collision
  names/IDs and geometry/inventory mismatches reject. Projection is explicitly
  **conditional**, not a qualified identity assertion. Neither candidates nor
  state are written into EPD poses, dimensions or collision envelopes.
- `run_stage_a_physics_contact_capture.py`: reproducible single-world runner,
  unique transport partition/run ID, bounded capture and owned-process shutdown.
  It starts only Fortress and external EPD inference, never ROS/MoveIt/controllers.

The final current-build run is `/tmp/workcell_stage_a2_physics_contact_final`:
**2 genuine EPD detections, 2 reconstructed collision boxes**, physics step
**27638**, stamp **27638000000 ns**. Owned Gazebo exit **0**, capture exit **0**.
An earlier development capture verified the initial stream; the final capture
was necessary after adding exact support-ID header binding. Both are measurement
runs, not MoveIt acceptance campaigns. World geometry is unchanged; only the
measurement plugin/run configuration is appended to a disposable world copy.

| Physical collision | Entity ID | Unique nominal EPD candidate | Conditional penetration upper |
| --- | ---: | --- | ---: |
| `a0::part_00::link::collision` | 23 | `epd_27638000000_1` | 0.579972795154280 µm |
| `a0::part_01::link::collision` | 27 | `epd_27638000000_0` | 0.579972795154399 µm |

Support is entity **7**, exact name
`a0::pick_support::support_link::support_collision`. The reversed EPD suffixes
show that matching does not assume detection order. Unique nominal candidates
remain **BLOCKED_PROJECTION_UNQUALIFIED**. The physical penetration upper bound
is **unknown** for both pairs, despite these small conditional values.

## Authority and exact outstanding proof

The arithmetic enclosure is qualified **only for the reported finite binary ECM
centre/matrix/dimensions and analytic horizontal plane**. Its returned interval
width is about `1.06e-22 m`; this is an arithmetic representation enclosure,
not DART accuracy, sensor accuracy or physical certification. No supplied
`backend_state_error_bound_m` value can override the missing qualification.
All records/evaluations remain simulation-only, replay-only and non-authorizing.

The inspected version-5.4.0 DART source establishes two concrete gaps:

1. `SimulationFeatures::Write(ChangedWorldPoses)` converts
   `link->getWorldTransform()` and suppresses notifications while cached position
   and quaternion compare equal with `1e-6` tolerances. A PostUpdate ECM record
   identifies when it was read, not necessarily when the stored transform last
   changed. That tolerance alone is not a qualified whole-corner bound through
   conversion, hierarchy composition and backend collision-shape transforms.
2. `SDFFeatures::ConstructPlane` constructs a **2100 m BOX** translated down by
   1050 m, rather than a DART PlaneShape. The provider observes the loaded SDF
   plane in ECM, not independently read-back constructed backend shape geometry.
   Equivalence of the relevant top face and its numeric/transform error must be
   qualified before calling the analytic PLANE enclosure a backend bound.

Local inspected source:
`/home/user/workcell_ws/diagnostics/stage-a1-closeout-20261003-mimic-mechanics/native-clean-source/gz-physics-ignition-physics5_5.4.0/dartsim/src/`.
This is source inspection, not an attestation that those files built the loaded
library. Installed DART plugin inventory SHA256:
`44e5d8573b44ef1f555eba6f2fbf01c1f0ab657b0fd32016c441e884ec5e71a1`.
Fortress default backend selection was used; this hash is not claimed as
runtime-loaded binary attestation. Primary [version-5 engine documentation](https://gazebosim.org/api/physics/5/switchphysicsengines.html)
and [custom backend feature mechanism](https://staging.gazebosim.org/api/physics/5/createcustomfeature.html)
describe the relevant installed API boundary.

Additionally, exact camera/physics timestamp equality does not qualify the
renderer's sampled state or within-exposure motion. Camera intrinsics/transform
and EPD mask-error bounds remain unavailable. The rational projection encloses
reported binary camera coefficients; it does not manufacture a renderer/mask
uncertainty bound. No certified object/support association exists yet.

**Exact next action:** add a read-only feature at the DART physics-step boundary
that returns the actual constructed `ShapeNode` geometry/local transform and
world transform, with step/run/shape identities and loaded backend attestation.
Qualify or eliminate the state-copy/conversion error before reusing this
all-corner evaluator. Then qualify renderer-to-step alignment and bounded mask
projection association. Required output is a whole-shape penetration upper
bound **≤0.1 mm** plus verified EPD binding; neither a scalar configuration bound
nor the maximum of a truncated contact list is accepted. No native support
permission may precede that result.

## Admission and verification

The existing geometry estimator was rerun on the fresh EPD points with pinned
world SHA256 and loaded 25 mm inventory qualification. The provider never
replaces any EPD-derived pose, centre uncertainty or conservative collision box.
Existing native FCL extraction admission was then run on those original boxes,
all neighbors and static obstacles. Both reject `EXTRACTION_INITIAL_DEPTH`
against `workcell::support_surface_table`, unchanged depths **7.808697 / 8.678595
mm**, unchanged **0.1 mm** limit. Native support-conditioned certification is
**BLOCKED / not extended**. Approach, descent, closing, extraction, retreat,
transfer, placement, release and home are all **NOT RUN for this capture**.

- **110 focused Python tests PASS**, including 28 provider tests for deep
  penetration, omitted/shallow manifold data, complete rotated corners,
  unsupported geometry/support rotation, missing data, identities, frame errors,
  ambiguous masks, stale/mismatched timestamps and rejection reports.
- Provider shared library and affected capture C++ target build **PASS**;
  existing geometry CTest **1 PASS**.
- **44 existing native safety regressions PASS**: excessive depth, wrong support,
  unrelated collisions, non-deepening/return rejection, expiry/recontact,
  incomplete enumeration and uncertifiable intervals.
- Python compilation / `git diff --check` PASS; scoped independent review found
  no material issue. Current-head CI is checked separately after push.
- Zero execution goals; no bridge qualification change, ACM exemption, protected
  Stage-A1 write or automatic merge. The PR remains draft.

## Reproduce the final measurement path

```bash
cmake -S scripts/stage_a_rgbd/physics_contact \
  -B /tmp/stage_a2_physics_contact_build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/stage_a2_physics_contact_build -j2
# Existing optional RGB-D build configuration retains external EPD_SOURCE_DIR.
cmake --build /tmp/workcell_stage_a2_build --target stage_a_rgbd_capture -j2
python3 scripts/run_stage_a_physics_contact_capture.py \
  --world /tmp/workcell_stage_a2_dimensions/world.sdf \
  --provider-library /tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so \
  --capture-binary /tmp/workcell_stage_a2_build/stage_a_rgbd_capture \
  --model /home/user/stage_a_cube_model/cube_maskrcnn.onnx \
  --labels /home/user/stage_a_cube_model/labels.txt \
  --support-collision a0::pick_support::support_link::support_collision \
  --workpieces part_00 part_01 --output /tmp/workcell_stage_a2_physics_contact_new
python3 scripts/stage_a_physics_contact.py \
  /tmp/workcell_stage_a2_physics_contact_new/capture \
  --output /tmp/workcell_stage_a2_physics_contact_new/contact_evidence.json
```

The evaluator deliberately returns **2** with a retained BLOCKED report. This
is a working measurement path, not a qualified contact certificate. Raw state
remains in the local measurement file; the committed JSON contains bounds,
identity candidates, hashes and outcomes without dynamic object poses.
