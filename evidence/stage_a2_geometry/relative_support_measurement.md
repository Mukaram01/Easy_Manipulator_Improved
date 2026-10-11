# Relative support measurement — 2026-10-10

Baseline verified: `cbe19ac180f40494fd9e283a35631719eda5472a`, clean isolated
`/home/user/workcell_ws_stage_a2`, branch `codex/stage-a2-collision-geometry`.
This addition is a working measurement diagnostic. Support proof, native
certificate and extraction admission remain **BLOCKED**. No MoveIt acceptance,
Gazebo campaign, bridge, controller or robot execution was launched.

## Implemented measurement and actual evidence

The existing snapshot exporter optionally consumes the same capture's float32
axial depth and an explicit support ROI. It fits a table plane in the optical
frame and expresses the existing EPD segmented top-face fit and cube orientation
in that same frame. The qualified 25 mm simulation specification supplies shape;
all eight physical corners, rather than the collision envelope, supply nominal
lowest-corner clearance. A shared rigid camera/world transform cancels from
relative distances and angles. This does not cancel differential depth bias,
calibration/pixel errors, face misidentification, or visual/collision registration.

The frozen genuine capture is reused at its original stamp `27639000000` ns;
it is **replay-only**, not a new live observation. Raw files remain at
`/tmp/workcell_stage_a2_dimensions`. Existing world/dimension qualification is
rerun, not retrofitted onto a capture without loaded evidence. The explicit ROI
`[180,180,330,205]` selects 3,750 table-looking pixels away from the cube faces
and bin. Its association with static `pick_support` remains operator supplied,
not a verified support binding to planning ID `workcell::support_surface_table`.
Hashes, ROI, fitted plane, source, stamp, object IDs, physical pose/shape,
conditional original pose set and unresolved budget are exported through the
existing source/object attributes. The standalone helper exports the same
measurement. It never produces contact authority; both entry points return
exit **2** after writing diagnostics when support is blocked.

| EPD object suffix | Top-face height | Nominal lowest-corner gap | Remaining total error allowance |
| --- | ---: | ---: | ---: |
| `_0` | 24.999389 mm | -0.619 µm | 99.381 µm |
| `_1` | 24.998872 mm | -2.988 µm | 97.012 µm |

Table maximum fit residual: **0.034335 µm**. This is neither a worst-case
measurement-error bound nor evidence of submicrometre accuracy. The second
object's corner clearance includes its measured tilt and is stricter than its
face-centre height alone. All original reconstructed object fields, including
collision dimensions, poses and uncertainties, compare equal after removing the
new diagnostic attribute.

## Explicit uncertainty budget and decisive blocker

Only the loaded dimensional specification is qualified, simulation-only. Its
binary/decimal representation term is `6.93889390390723e-18 m`; it conveys no
pose, contact, physics or hardware authority. Float32 nominal axial half-ULP sum
is about **0.059605 µm**, a storage-resolution diagnostic only. It is deliberately
**not** a qualified corner-clearance contribution: raw contributing sample
rounding, projection, fitting and arithmetic require propagation.

Each of the following has an explicit **unknown** bound in the JSON budget:

- Differential depth/renderer measurement error, including systematic bias.
- Intrinsics, pixel correspondence, segmentation and edge/face-fitting error.
- Plane-fit normal and offset error; relative roll/pitch and extrapolation error.
- Input transform consistency and numerical error, despite common-transform cancellation.
- Within-exposure motion; exact RGB/depth/info timestamps alone are insufficient.
- Registration of the observed visual plane to the physical collision support.
- Floating-point SVD/projection/dot-product and geometric tolerance enclosure.

No residual, observed noise-free scene or arbitrary constant replaces those
unknowns. Thus there is **no qualified finite total clearance-error bound** and
no newly qualified conservative physical configuration set. The original
conditional 2.781/2.962 mm centre and 13/14.828 degree orientation sets remain
unchanged. The 0.2 mm downward-shift witness is **not excluded**: relative nominal
corner gaps would become about -200.619/-202.988 µm. It is an analytical admitted
configuration, not a statement of actual simulator penetration.

The implementable measurement requirement is a bounded set whose corner-gap
error is at most **99.381 µm** for object 0 (or **97.012 µm** for object 1),
including every term above. For example, a sufficient displacement enclosure
charges `u_translation + 2*r*sin(u_rotation/2) + u_shape + u_support_plane +
u_numeric`, where `r = sqrt(3)*0.025/2 m`, with depth/calibration/time errors
propagated into those terms without double counting. Each contribution must be
independently qualified, not assigned the entire budget. Even if every other
term were zero, object 0's entire corner-rotation allowance would be only about
0.263 degrees; the existing orientation set cannot be silently replaced by it.

## One simulation-only alternative investigated

**Fortress contact sensor with depth reporting**, not a fixture redesign.
The saved world has no contact sensor or Contact system, so this capture has no
contact evidence. Installed `ignition/msgs8/.../contact.proto` exposes repeated
point positions, normals and depths plus collision identities. The inspected
Ignition Physics 5.4.0 DART `SimulationFeatures::GetContactsFromLastStep` enumerates
backend contacts and copies each `_contact.penetrationDepth`. These point depths
alone do not establish a conservative maximum over the entire physical box,
numerical/backend error, omitted manifold points, or association to an EPD ID.
The SDF `min_depth=0.0001` is not treated as a penetration upper bound. No solver
parameter is promoted into a contact proof and no dynamic object pose is used.

Primary references: [Physics contact-data contract](https://gazebosim.org/api/physics/8/structgz_1_1physics_1_1GetContactsFromLastStepFeature_1_1ExtraContactDataT.html)
(the version-5 local backend implementation was inspected separately), and
[Fortress-era depth-camera interface](https://staging.gazebosim.org/api/rendering/6/classignition_1_1rendering_1_1DepthCamera.html).

**Exact next engineering action:** implement and qualify a simulation-only
contact-depth provider at the physics-step boundary for the loaded BOX–plane
pair. Its contract must establish complete deepest-corner/manifold coverage,
an outward-rounded numeric/backend-error bound, exact collision/support IDs and
capture-step binding. Associate contact points to the unique observed EPD mask
using bounded optical projection and reject ambiguity; do not import dynamic
object poses. Publish an upper bound ≤0.1 mm for the whole physical shape and
bound relative orientation/pose for subsequent carried motion. A boolean
contact flag or maximum of an unqualified/truncated contact list is insufficient.
This provider was **not implemented**, because those prerequisites are not yet
independently defensible. It is a concrete next measurement task, not a claimed
alternative PASS. Hardware certification is outside its authority.

## Admission, native proof and focused verification

Existing native FCL admission was rerun on both genuine EPD boxes and all
existing static/neighbor boxes. Both reject `EXTRACTION_INITIAL_DEPTH` against
`workcell::support_surface_table`, unchanged **7.808697/8.678595 mm** envelope
depths and **0.1 mm** limit. A first shell attempt lacked ROS Python packages;
the successful rerun used ROS Python, and the retained machine-readable report
also repeats admission through the explicit existing native ABI/library hash.

The support-conditioned native certificate was not extended: initial physical
support proof is absent. Approach, descent, closing, extraction, retreat,
transfer, placement, release and home are all **NOT RUN for this capture**.
Historical approach/descent/closing passes in the previous pile experiment are
not current-capture stage passes.

- **82 Python tests PASS:** relative geometry, common-translation cancellation,
  tilted physical corners/downshift, missing/stale/mismatched provenance,
  wrong support/frames/orientation/dimensions, invalid depth/ROI, integrated
  snapshot/report/replay export and existing RGB-D/geometry/extraction guards.
- **44 existing native tests PASS:** identified support/pair, excessive depth,
  unrelated obstacles, carried return/recontact, expiry, truncated enumeration,
  stale epoch, third-party collisions and uncertifiable intervals. Existing
  native binary; native source is unchanged, no new native build was needed.
- Python compilation and `git diff --check` PASS.
- Independent scoped review: two metadata/budget issues fixed with observed
  failing-then-passing regressions. No certificate/ACM change was made.
- Current-head CI is reported from GitHub after pushing, separately from local
  tests. No automatic merge or ready-for-review transition.
