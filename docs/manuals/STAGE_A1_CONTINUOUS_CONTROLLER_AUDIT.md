# Stage A1 continuous controller collision audit

## Scope and root cause

The shared `auditControllerTrajectory` in `src_support_contact_adapter.cpp`
previously checked a grid chosen from 1 ms spacing and a Bernstein derivative
travel bound. This constrained sample density, not collision clearance between
samples. The ordinary, Cartesian and initial-support adapters all used that
function. Recovery remained blocked; it had no authorized withdrawal executor.

Before production edits, both the saved external probe and a new tracked
regression reproduced the same false acceptance: a 1 ms quintic has clear
endpoints, reaches 62.5 nm at 0.5 ms, collides with an obstacle initially 10 nm
away, and was accepted. The corrected adapter rejects it as `COLLISION` at the
first interval. The same motion with sufficient clearance is `CERTIFIED_CLEAR`.

## Certificate

`controller_interval_certificate.hpp` precedes the existing chronological
feasibility/contact-policy checks. Every interval is accepted only by proof,
subdivided, or rejected. Results are `CERTIFIED_CLEAR`, `COLLISION`, or
`UNCERTIFIED`; the latter two return adapter failure and discard the trajectory.
Exceptions cannot cause MoveIt to bypass the activated adapter.

* Installed JTC evaluates collision witnesses. Outward-rounded interval
  arithmetic encloses its linear, cubic or quintic power coefficients and
  converts them to Bernstein controls. De Casteljau subdivision encloses the
  exact nanosecond split ratio. Position/velocity/acceleration and time inputs
  must be finite and correctly sized; unsupported derivative-only data rejects.
* A full-segment absolute power-term sum bounds floating evaluation error,
  including cancellation. This bound persists through subdivision and propagates
  through mimic factors/offsets. Position-limit hulls include this uncertainty.
* Per-body displacement accumulates every ancestor joint. Prismatic motion is
  linear; revolute motion multiplies angular range by a conservative downstream
  radius. Radii include actual collision shapes/origins, attachment offsets,
  chain translations, full prismatic extension and numerical uncertainty.
  Self-pair motion includes **both** bodies. Transform arithmetic is charged
  against clearance. No endpoint-only travel inference is used.
* Geometry is validated before FCL. Complete relevant body pairs are enumerated,
  respecting the existing ACM and attachment touch links. Self uses the
  unpadded environment; robot/world uses the ordinary environment. Non-default
  padding/scaling has no implemented radius proof and therefore rejects.
* FCL distance is **not** assumed to be an exact clearance lower bound. Its
  convergence tolerance does not guarantee that. Candidate separating axes come
  from coordinate axes and FCL nearest points. Outward projection of every
  primitive support or mesh vertex yields an independent lower bound. Clearance
  must strictly exceed relative displacement plus numerical uncertainty.
  Concave shapes whose convex support projections cannot separate may reject.
* Already-proven coordinate-axis pairs are removed only from the private distance
  query. Unresolved pairs must appear in the completed FCL distance map. FCL's
  intentional early stop at an overlap triggers subdivision; an incomplete
  collision-free map is an error. Neither changes the scene ACM or geometry.
* Defaults: at most 120001 inspected intervals, depth 32, minimum interval 1 ns,
  trajectory duration 120 s. Limits never imply success. Reports include counts,
  maximum depth, first failing interval, reason and elapsed wall time.

The certificate concerns the emitted controller spline in an immutable,
stationary-world planning snapshot. `stationary_world=false` fails closed.
It does not certify moving-object predictions, physical servo tracking, runtime
contact retention, or arbitrary feasibility/path-constraint callbacks. Existing
chronological checks of those callbacks remain in place after the certificate.

## Existing conditional contacts

Conditional ACM entries remain the exact typed `SupportContact` or `PileContact`
predicates; their values and the 0.1 mm contact limit are unchanged. Arbitrary
conditional callbacks cannot supply an interval permission.

A restricted additional proof handles upright BOX/BOX contact under purely
vertical prismatic translation. Geometry establishes the face, overlap depth,
normal and unchanged footprint. Bernstein derivative bounds prove monotonicity.
Depth/floor checks charge full evaluation and transform uncertainty against the
existing allowance. A separation-spanning interval must also prove that its
minimum advance per nanosecond exceeds twice the numerical position error.
Exact stored-waypoint transitions are checked separately. Otherwise it subdivides
or returns `UNCERTIFIED`. A previously clear pair requires monotone geometric
separation and receives no renewed contact allowance.

Precisely: this proves monotonic continuous nominal segment polynomials and
monotonic floating JTC outputs at **every representable integer-nanosecond
controller timestamp**, including segment/terminal transitions. It is an analytic
bound over all timestamps, not a fixed sampling grid. It is not a physical
tracking or arbitrary sub-nanosecond error-tube guarantee. Unsupported contact
shapes, rotations or lateral motion fail closed. This attached-body proof is not
a detached robot-link/world recovery policy.

## Installed controller identity

Qualification environment on 2026-09-24:

* Debian package `ros-humble-joint-trajectory-controller`
  `2.54.0-1jammy.20260908.124006`.
* Installed/default interpolation mode `splines`; saved controller configuration
  has no override. Position-only is linear; positions/velocities are cubic;
  positions/velocities/accelerations are quintic at their common specification.
* Actual witness evaluator:
  `joint_trajectory_controller::Trajectory::interpolate_between_points`.
  Boundary handling corresponds to `Trajectory::sample`: next segment start or
  stored final point, rather than previous polynomial extrapolation.
* Official source:
  <https://github.com/ros-controls/ros2_controllers/blob/2.54.0/joint_trajectory_controller/src/trajectory.cpp>.
* Installed library SHA-256:
  `331b5dacbb3a358854c5b08e429acc7a33806bf21555b07b11cf7797886d5b44`.
* Installed trajectory header SHA-256:
  `7094f6508485eec73fd6aca11905914162fbcf23c953988fba2bf7fd88909fea`.
* Downloaded version-matched source SHA-256:
  `7a315f55734c03c63e976794d4b00a55e42c8eddaaf986abacebfe748b66f41b`.

Both production and test targets compile with `-frounding-math -ffp-contract=off`
for outward interval operations. A controller/version/interpolation change
requires renewed qualification against the actual implementation; these hashes
identify the environment qualified here. Nothing in `/opt` was modified.

## Validation and reproduction

Evidence directory:
`/home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924`.
The original pre-change probe remains in
`/home/ubuntu/workcell_ws/stage-a1-withdrawal-qualification-20260924`.

Focused C++ coverage includes the saved failure and clear control, collisions in
the first/last 2% with clear midpoint/endpoints, equal-endpoint interior extrema,
large motion with large clearance, revolute radii, both self bodies, malformed
fields/timing, nonfinite input, precision exhaustion, unsupported world geometry,
FCL overestimate rejection, odd-nanosecond subdivision, prior support/contact
regressions and the real Stage A approach.

The real replay uses the original **101-point** `EXECUTE_APPROACH` from saved
`stage-a1-finish-20260923-160609/03-telemetry`, its complete planning scene,
runtime URDF, semantic model and collision meshes. No synthetic geometry replaces
that scene. An opt-in fixture avoids embedding local evidence/assets in CI:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/workcell_ws/moveit_teardown_overlay/install/local_setup.bash
source /home/ubuntu/workcell_ws/install/local_setup.bash
python3 workcell_builder/workcell_builder/test/evidence/prepare_controller_certificate_fixture.py \
  /home/ubuntu/workcell_ws/stage-a1-finish-20260923-160609/03-telemetry \
  /home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924/real-stage-a
cmake --build /home/ubuntu/workcell_ws/build/workcell_builder \
  --target workcell_support_contact workcell_support_contact_test -j2
WORKCELL_STAGE_A_CERTIFICATE_FIXTURE=/home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924/real-stage-a \
  /home/ubuntu/workcell_ws/build/workcell_builder/workcell_support_contact_test
```

The prepared fixture records hashes of its source evidence and semantic includes.
Without this environment variable the real-data test explicitly skips; it must
run, not skip, for this commissioning qualification.

Independent review examined missing-pair coverage, FCL distance uncertainty,
polynomial evaluation/subdivision error, geometry radii, mimic motion, both self
bodies and conditional expiry. Its final answers were: sample-only certification
**no**; conservative supported joint bounds **yes**; both self bodies **yes**;
fail closed **yes**; actual installed JTC semantics **yes**. Test results and
performance are recorded in the evidence report.

## Recovery boundary

Fixing the ordinary continuous auditor does not issue recovery authority.
The stopped iteration 104485 is historical diagnostic evidence only. Detached
mesh-link/world initial contacts require their own exact contact set and
per-pair monotonic separation/irreversible-expiry proof. Existing attached BOX
policies cannot authorize those contacts. Attempt authority A remains revoked;
no Recovery Authority R or physical execution follows from this certificate.

### Recorded validation (2026-09-24)

* Final shared-library and test-target builds succeeded without compiler warnings.
* Focused C++ suite: **40 passed, zero skipped**, including the real-data gate.
* Original saved 1 ms probe: midpoint collision remains reproducible; adapter
  acceptance changed from `1` to `0`. Sufficient-clearance control accepts.
* Relevant Python execution/recovery suites: **342 passed** against the rebuilt
  shared library. An earlier unsourced shell lacked the `workcell_builder` ament
  package; rerunning in the required ROS workspace resolved those environment failures.
* Real approach: **936 inspected / 518 certified / 418 subdivided / depth 3**,
  **17.7559 s**. Caching and proving distant pairs before FCL reduced the earlier
  160.115 s replay without changing these counts.
* Existing Cartesian/support initial-contact fixture: **22 inspected / 14
  certified / 8 subdivided / depth 8**; its contact limit and ACM are unchanged.
* Counterexample: **1 inspected / 0 certified / depth 0**, `COLLISION` on
  `[0,1000000]` ns. Simple clear control: **1 inspected / 1 certified / depth 0**.
* `git diff --check` passed. No simulator or physical execution was started.
