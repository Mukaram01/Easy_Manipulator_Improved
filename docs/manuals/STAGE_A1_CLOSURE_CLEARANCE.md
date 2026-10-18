# Stage A1 closure-clearance screening

The 18 October 2026 run at reviewed workstation HEAD `322895aa` executed
approach, descent and closure, then stopped at stationary admission. At
iteration 171959, 0.399 simulated seconds after the contact set froze, the
previously separated pair `runtime::part_06` / `runtime::part_07` developed a
new physical contact. Both fingertips remained on the target. Native FCL
replay gives a 7.59365 micrometre gap at freeze and 1.0028 micrometre overlap
at rejection. Target translation/rotation during that interval were
0.6149 micrometres / 2.4271e-5 radians; neighbor movement was 5.6191
micrometres / 4.29044e-4 radians. This demonstrates closure/hold settling
across insufficient clearance. It does not establish a simulator defect.

## Change

Before planning a simulator candidate, use the existing native BOX geometry
predicate to screen every dynamic neighbor. A pair without measured contact
must have clearance greater than the sum of both BOX pose uncertainty bounds
and the existing geometric qualifier's maximum fingertip intrusion allowance.
The pose bounds come from unchanged `target_observation_agreement`: 0.25 mm
translation and quaternion chord distance 0.00025. The rotational displacement
uses the full BOX corner radius and `angle = 4 asin(chord / 2)`. The default
intrusion allowance is the existing 1 mm `max_penetration_m`; the combined
screening margin for two 25 mm cubes is approximately 1.522 mm.

This is a conservative rejection screen, not a bound on contact-driven
physics or a retention certificate. Existing measured contact can only waive
the target/neighbor gap screen after the unchanged native point/geometry
predicate checks actual measured BOX poses. It cannot permit gripper/neighbor
contact. Missing, stale, duplicate or unsupported dynamic geometry fails closed.

The existing MoveIt continuous controller-trajectory audit then checks the
full robot, including every gripper collision link and the entire closing
spline, against a private copy of the scene with **all** dynamic neighbors
expanded by the screening margin. The live/nominal scene, target dimensions,
ACM and exported execution start/end scenes remain unchanged. Planning metadata
records margins, receipt/observation identity and the expanded scene hash.
No target is physically attached for screening.

All dynamic pose bounds are checked again immediately before approach and
closure, including neighbor orientation, so planning cannot outlive the
uncertainty envelope silently. Existing contact-set, penetration, slip,
telemetry, coupling, cancellation and shutdown thresholds remain unchanged.
The generic candidate search remains responsible for finding another grasp;
there are no object IDs, grasp-index exceptions or preferred demo candidates
in production code.

## Validation and remaining gate

Focused tests cover the recorded failure with renamed IDs, safe separated
geometry, actual contact evidence, admitted measured-pose shifts, orientation
drift, compound geometry, private expansion immutability, early rejection
before approach/attachment, and forwarding clearance to the closing motion.
The existing full-cycle, extraction, capture and execution suites cover the
unchanged planning and physical guards. Runtime outcome is recorded in the
workstation attempt evidence; offline success alone is not Stage A acceptance.

## Fresh workstation attempt — 18 October 2026

Code `094f0f98` ran once through the original fresh stationary-session setup.
The known failed `part_06` / `finger_pinch_basic::001` candidate was rejected
before motion for insufficient clearance to `part_07`. The existing generic
search considered 77 candidates across 10 objects and three strategies, plus
one retry: 58 clearance rejections, 12 approach failures and 8 grasp failures.
Eleven approach failures returned IK code -31. A clearance-admissible
`part_08` / `top_2f::001` descent was rejected for a predicted
`gripper_finger1_finger_tip_link` / `part_09` collision, depth 3.972049 mm.

No complete cycle was selected: `execution_attempted=false`. This attempt
therefore sent no execution goals and did not prove physical closure,
stationary retention or lift. The bounded search did not exhaust every
possible catalog candidate; no claim is made that no safe grasp exists.
The full closing-padding runtime path was not reached. All physical gates
remain authoritative, real hardware stayed disabled, and shutdown was clean
with zero owned survivors. Dependencies remained identity-admitted and were
not rebuilt.

Evidence is retained at
`/home/user/workcell_ws/diagnostics/stage-a-closure-clearance-20261018/`:
`REPORT.md`, `attempt-outcome.json`, `evidence-sha256.json`, the original
session/resolve reports, exact receipt and logs. The next blocker is a
complete collision-valid approach/descent for a clearance-admissible candidate;
it must be resolved without weakening this screen or the physical gates.
