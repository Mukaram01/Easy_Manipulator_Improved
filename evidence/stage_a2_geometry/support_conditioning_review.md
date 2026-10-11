# Support-conditioning proof review

Baseline `beb235b8`; same genuine separated-cube snapshot, no new perception,
world edits, robot execution or production implementation changes.

## Exact failure and existing authority

`pile_extraction._initial` compares the PlanningScene BOX against every BOX
neighbor and rejects depth above 0.1 mm. `_box` uses collision dimensions/pose,
not the retained observation's physical-dimension/uncertainty provenance.
The original conservative envelopes overlap the table by 7.808697 / 8.678595 mm.
This admission rejection is correct for the representation it receives; it
cannot distinguish proxy overlap from physical penetration.

The architecture already has phase-conditioned support contact, but it does
not provide uncertainty-conditioned support contact:

- `legitimate_support_ids` identifies eligible authored support owners and
  footprints in world coordinates. It explicitly grants no contact permission.
- `InitialSupportContact` checks the actual attached BOX bottom and support
  floor geometry, then installs a private typed, exact-pair `SupportContact`.
  The shape's bottom must be within 0.1 mm of the certified floor.
- `SupportContact` still requires exact identities/types, shallow depth,
  floor position and upward normal for every contact. It does not accept deep
  envelope contacts or arbitrary callback authority.
- `upwardBoxContact` retains the depth limit and proves a narrow monotone
  upward case; it cannot certify these deep envelopes. Its analytic shortcut
  also requires aligned boxes and only varying vertical prismatic joints;
  it is not a general UR5 carried-object monotonicity proof.
- Detached recovery rejects attachments and cannot serve as a carried-object
  support exemption. The diagnostic legacy pose-correction helper is not an
  execution path and must not be used to move a perceived proxy above the table.

Removing the Python guard would only move the failure into native admission/
collision certification. Removing the table from neighbor checks or granting
an unconditional ACM entry would erase a required proof.

## Why the present observation does not prove contact

Geometry provenance survives replay normalization, but neither observed object
contains a verified target-to-support binding or a conditioned physical-pose
set. `face_support_points` denotes points supporting a plane fit, not table
contact evidence. An authored zone's eligible support is not proof of an
individual object's resting/contact state.

Both captures are consistent with resting. However, the same measured top-face
centre/normal and exported dimension/pose bounds also admit a 25.25 mm declared
cube. Its centre shifts by only 0.125 mm (inside the 2.781/2.962 mm centre bounds),
and its bottom-face centre lies 0.250702 / 0.251190 mm below the support, beyond
0.1 mm. The existing bounds therefore do not exclude an invalid physical
configuration. These are analytical counterexamples within exported bounds,
not simulator measurements or claims that the actual cubes penetrate.
Independent simulator truth cannot be promoted to planning evidence to remove
that ambiguity. Merely finding one nominal resting configuration cannot prove
all admitted uncertain configurations safe.

## Necessary minimal representation/proof extension

Reuse the existing geometry provenance, PlanningScene object and typed contact
lifecycle, while keeping the full original envelope for all ordinary pairs:

1. Carry an explicitly verified target/support identity, support collision-
   geometry identity, world frame/plane and footprint, timestamps, measured
   face evidence, declared dimensions and calibration/uncertainty assumptions.
   Reject missing/stale/incorrect/ambiguous associations and non-horizontal
   support in this existing world-vertical profile.
2. Represent the physical pose/shape set conditioned on that verified support
   relation separately from the unchanged enclosing collision BOX. Establish
   the contact relation from planning-admissible evidence; do not silently
   truncate the uncertainty set merely because a nominal pose can rest.
3. A typed native exact-pair proof must bound physical penetration for **every**
   admitted configuration by 0.1 mm and certify non-deepening separation over
   the actual carried controller spline, including orientation, uncertainty,
   numeric error and stored-waypoint boundaries. Expire permission after
   separation; block downward motion, through-table extraction and recontact.
4. Keep every other world/robot/gripper collision check and the independent
   fingertip grasp-phase policy active. Bind the proof to the immutable scene
   and target geometry; unknown or incomplete evidence fails closed.

The current message/contact certificate has no input for this conditioned set.
No safe local admission correction was implemented. This is a representation
and evidence blocker, not permission to add a general collision exemption.
Independent focused review reached the same conclusion; it does not assert
that implementing the extension is impossible.

## Focused verification and stop gate

- Existing numerical shallow support/resting and strict lift: PASS.
- Above-tolerance contact, incorrect support, malformed binding, downward lift,
  recontact, raised lip, unrelated obstacle and deep cube/pile contact: rejected
  by existing native tests, PASS (24 native tests total).
- Unknown measured frame, missing/partial/merged/nonplanar geometry, bad profile,
  sideways/later obstacle collision, downward/through-support corridor,
  neighbor collision and incomplete geometry: rejected by existing Python
  geometry/extraction tests, PASS.
- Authored support owner missing/incorrect, unknown support frame and tilted
  support: no eligible binding in direct existing-helper checks, PASS.
- Existing geometry/extraction Python suites: 28 PASS; authored support-binding
  regression: 1 PASS. Current native library and test target builds: PASS.
- Actual separated cubes: **both BLOCKED**, unchanged `EXTRACTION_INITIAL_DEPTH`.
  Approach/descent/closing/lift/retreat/transfer/place/release/home are NOT RUN
  for this capture. Full acceptance is BLOCKED and was NOT RUN, per stop gate.

Raw checks: `/tmp/stage_a2_support_{admission,safety_python,safety_native}.log`,
`/tmp/stage_a2_support_binding_test.log`, `/tmp/stage_a2_support_build.log`.
The adjacent JSON preserves analytical counterexamples and admission results.
Use the exact separated-experiment admission command in `separated_experiment.md`;
it still reproduces both failures. No additional validator framework was added.

Next product action: define and validate the support-conditioned physical-pose
set and its verification source within existing geometry provenance, then extend
the typed native carried-contact proof. Recheck admission before any bounded
MoveIt run. AUTO/PREFERRED/EXACT, envelope dimensions, table geometry, the 0.1 mm
limit, frozen-replay prohibition, bridge qualification and protected checkout
remain unchanged. No new robot, MoveIt, controller or Gazebo process was started.
