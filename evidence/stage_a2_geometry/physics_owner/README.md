# Isolated live Physics owner readback — 2026-10-10

Baseline `6363d526fddba5872da9bac8ec0449856d73d8b2`, clean isolated
`/home/user/workcell_ws_stage_a2`. **Live backend access PASS; runtime equivalence
BLOCKED. Physical penetration, EPD association and extraction remain BLOCKED.**

## Working implementation

Standalone `scripts/stage_a_rgbd/physics_owner` fetches four SHA256-pinned files
from the upstream `ignition-gazebo6_6.18.0` tag. Configure re-verifies pristine
source and regenerates the overlay; changed source hashes/anchors reject. The
installed headers report Fortress 6.18.0 and ign-physics 5.4.0. This establishes
matching release source, not a reproducible-build attestation of the stock binary.
No installed source/library or production world is changed. There is no install
rule. The distinct `WorkcellOwnerPhysics` System replaces the single Physics
plugin entry only in a disposable world copy; the stock Physics System is absent
from the instrumented process library inventory.

The owner retains its original live engine, construction, stepping, solver,
collision-response and controller code. Additions are class/plugin identity
renaming, backend-loader provenance capture, an optional `RetrieveWorld` feature,
construction-boundary inventory reads and one configured end-of-Update readback.
`instrumentation.diff` is the complete generated Physics.cc diff. The companion
header renames the owner and its private class, retaining Configure/Update APIs.

A snapshot immediately before and after each authoritative collision-construction
call must add exactly one CollisionAspect ShapeNode without removing any. That
pointer is bound to the returned physics shape ID and the same Gazebo entity
inserted into `entityCollisionMap`. No name matching, presumed ID relation,
private `ShapeInfo` cast or separate engine/world is used. At readback the whole
DART collision inventory, authoritative physics map and ECM collision inventory
must agree bijectively. Missing, ambiguous, duplicate or changed bindings reject.
All shape types are reported; none can grant safety authority. A session-scoped
pointer string is an observation identity, never portable across runs.

Actual BoxShape dimensions and ShapeNode local/world matrices come directly from
public DART methods through the live owner's `GetDartsimWorld()`. ECM matrices
are separate diagnostic comparisons. These raw matrices are confined to these
simulation measurement artifacts; no normalized EPD/planning files are changed.
Every owner record explicitly remains BLOCKED.

## One bounded comparison actually run

Reproduce in an isolated overlay (network needed for pinned source on first build):

```bash
cmake -S scripts/stage_a_rgbd/physics_owner -B /tmp/stage_a2_owner_build
cmake --build /tmp/stage_a2_owner_build -j1
ctest --test-dir /tmp/stage_a2_owner_build --output-on-failure
cmake -S scripts/stage_a_rgbd/physics_contact -B /tmp/stage_a2_physics_contact_build
cmake --build /tmp/stage_a2_physics_contact_build -j1
python3 scripts/stage_a_rgbd/physics_owner/compare.py \
  --world /tmp/workcell_stage_a2_dimensions/world.sdf \
  --owner /tmp/stage_a2_owner_build/libworkcell_owner_physics.so \
  --observer /tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so \
  --output /tmp/NEW_OWNER_COMPARISON
```

Actual run: `/tmp/stage_a2_owner_comparison`, one stock and one instrumented
server, 2000 steps each, 1 ms step, identical initial geometry/physics settings,
separate transport partitions, each server bounded to 60 wall seconds. Both
servers exited 0. Recorded wall times were 4.669094 / 4.866247 seconds; this is
observed overhead, not a deterministic timing bound. No EPD inference, ROS,
bridge, MoveIt, controller or execution goal was launched.

At the exact endpoint, both ECM records (IDs, loaded SDF geometry, cached cube
states, dt, step and simulation time) were identical. This is endpoint agreement,
not a trajectory or universal-equivalence proof. Actual owner frame count is
2000, simulation time is 2,000,000,000 ns and DART accumulated time is
1.9999999999998905 s (difference 1.0946799022804043e-13 s, an observed discrepancy,
not a qualified clock bound).

**The comparison does not qualify contact behaviour.** Both ECM diagnostic
contact arrays are empty; the owner reports eight live DART manifold points.
The observer attempted contact-component initialization in Configure; collection
needs to be established after collision entities exist. The initial runner
returned 0 on matching ECM records; that exit was not equivalence qualification.
`comparison.json` preserves this initial output, including its overly broad
field name `exact_step_ecm_and_contact_records_equal`. The current runner checks
owner session/step/completeness, explicitly rejects absent contact evidence and
always exits 2 until qualification is implemented. It was not rerun. The
independent `summary.json` correctly marks the failed gate BLOCKED.

## Live geometry and provenance actually observed

There are eight bijectively mapped collision nodes, two mobile. Cubes are
Gazebo entities **23/27**, physics shapes **18/19**, each BoxShape **0.025 m** on
all axes. Support entity **7**, physics shape **12**, is an actual **2100 m BOX**
with identity rotation and local/world translation **(0, 0, -1050) m**. Its top
face is at z=0 with finite x/y footprint [-1050,1050] m. The 1050 m support
ECM/ShapeNode translation difference is the constructed BOX offset, not an
accuracy error. Each cube's maximum backend-versus-ECM matrix-entry difference
is **5.701626901322837e-7** (the maximum is a z-translation difference in metres).
This directly demonstrates why cached ECM is unsuitable for this measurement.

The successful loader reports `ignition::physics::dartsim::Plugin`; `/proc`
loaded-library inventories independently identify the binaries. Relevant SHA256:

- Stock Physics 6.18.0: `f1ccf096669cfc6f9cdfb2d153e1e8a6101571f8abcb22a24ca9040b4a86842b`
- Instrumented owner: `d2d0d7f47d4065c7f27fc5d889daea915b766a5382155978b33c92336b16a390`
- Same loaded DART backend 5.4.0 in both: `44e5d8573b44ef1f555eba6f2fbf01c1f0ab657b0fd32016c441e884ec5e71a1`
- Loaded DART 6.12.1: `5a0048707f55903e0ba62f9bba76456b6f0eb850c1d5016ea499432c1bab089b`

Full library paths/hashes, session and raw readbacks are attached JSON artifacts.
These attest the instrumented run, not direct certification of stock geometry.
No manifold depth is presented as a whole-shape penetration upper bound.

## Verification and stop gate

Owner overlay and ECM observer builds PASS. Construction-binding CTest PASS
(1 test with negative cases for wrong physics identity, vanished/missing nodes,
extra backend nodes, partial maps, duplicate and ambiguous construction, latched
failure). Existing physics-contact tests PASS (28). Focused independent review
found no remaining actionable issue after fixing stale-source reuse and vacuous
contact-comparison success. New geometry/numeric qualification tests were not
started because runtime equivalence stopped first. Existing deep/rotated BOX
unit tests are unchanged and are not new backend qualification.

**Exact immediate blocker:** stock contact-behaviour evidence is missing from
the runtime comparison. Smallest next action: initialize the disposable
observer's contact components after collision creation and collect non-empty
stock and instrumented contact traces at matched steps in a newly authorized
bounded comparison. Then qualify backend geometric/numeric bounds. Renderer
exposure/physics-step correspondence and unique bounded EPD mask identity remain
independently unqualified. Original EPD poses/envelopes, 0.1 mm limit, native
contact rules, frozen-replay and bridge restrictions remain unchanged. No MoveIt
or execution goal was sent.
