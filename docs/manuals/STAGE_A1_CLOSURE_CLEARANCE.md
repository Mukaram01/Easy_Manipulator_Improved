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

## Search coverage correction

The first screened attempt tested 77 unique catalogue IDs, with 53 `top_2f`
IDs still untested. Its 11 IK code -31 results prove only that those bounded
seeded calls returned no solution, not geometric unreachability. Eight descent
failures have explicit collision contacts; one approach invalid-plan failure
has no contact evidence and must not be labelled a proven collision.

A concrete scheduler limitation stopped top-grasp discovery with approximately
585 seconds left: one known retryable candidate reserved three 180-second
slots. Reservation now uses the number of actual retryable candidates, capped
at the existing beam width of three. Empty slots cannot starve untested
candidates. Global/discovery/retry deadlines, candidate order, IK, gripper
width, approach geometry, collision certificates and closure guards are
unchanged. A regression proves a later feasible catalogue candidate is reached
instead of being stranded behind an empty reservation. Search remains bounded;
this correction does not imply exhaustive catalogue coverage or resume every
candidate after a failed retry.


## Discovery continuation after spent retries

The reservation correction was tested at `fdb752b9` in a fresh identity-admitted
session. It considered 84 unique catalogue IDs (88 attempts), leaving 46 IDs
untested. Failures were 58 clearance rejections, 15 explicit descent collisions,
11 bounded IK code -31 outcomes, three uncertified approaches and one candidate
slice exhaustion. No physical execution occurred; shutdown was clean with zero
owned survivors. Exact evidence is retained under
`/home/user/workcell_ws/diagnostics/stage-a-safe-search-20261018/`.

A second scheduler limitation was demonstrated: top discovery paused with
392.795 seconds left for three real retries, but those retries used only about
132.4 seconds. The untested suffix was never resumed before fallback. After a
reserved beam is spent unsuccessfully, discovery now resumes once through the
same cache, candidate order and preplanner, using only the original remaining
budget. No candidate is retried again by continuation, and newly discovered
retryable failures do not receive a second beam. EXACT selection and resolved
revalidation are unchanged. Evidence counts distinguish resumed discovery.

Four regressions cover one/three failed retry slots and later safe/unsafe
candidates. Safe fixtures must complete the actual full-cycle preplanner; unsafe
fixtures remain rejected. Existing fallback-order checks require the resumed
preferred-strategy suffix before fallback and count all retryable candidates.
This improves coverage without claiming exhaustive search or changing any
physical acceptance requirement.


## Distinct poses before legacy aliases

The continuation trial at `2da84a13` exercised seven resumed candidates before
its unchanged global search budget expired. It evaluated 85 catalogue IDs in
88 attempts: 54 clearance rejections, 20 explicit descent collisions, eight
bounded IK no-solution calls, two uncertified approaches, three candidate slice
expirations and one global expiration. No motion occurred, and shutdown was
clean. Across retained screened runs 98 of 130 IDs were evaluated; the 32 top
IDs for parts 00, 03, 04 and 05 were still untested. Thus no scene-inaccessibility
claim is justified. Evidence is retained under
`/home/user/workcell_ws/diagnostics/stage-a-search-continuation-20261018/`.

The legacy eight top IDs contain four repeated physical poses: 004 equals 001,
005 equals 000, 006 equals 003 and 007 equals 002, allowing quaternion sign.
In budget-reserving discovery only, distinct poses across objects now precede
these aliases. Equivalence requires the same strategy, object, destination and
effective constraints plus matching grasp and approach poses (1e-12 numerical
tolerance for ordering only). Deferred aliases remain eligible after distinct
pose discovery and the original bounded retry beam; each receives its own
preplanning and cycle validation. No collision, IK or successful trajectory
result is inherited from another candidate. EXACT and resolved revalidation
are unchanged. Evidence lists deferred aliases separately from actual attempts.

Regressions prove that expensive repeated poses cannot strand a later object's
actually valid cycle and that an alias can independently pass all cycle checks
after its representative failed. This changes scheduling only, never admissible
geometry or execution rules. Runtime acceptance remains unproven until the
fresh simulator gates pass.
