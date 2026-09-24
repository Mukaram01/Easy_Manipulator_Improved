# Stage A1 detached-contact geometric certificate

## Scope and result

This change adds an opt-in, frozen-scene detached ROBOT_LINK/WORLD_OBJECT certificate to the existing continuous JTC auditor. It does not issue a recovery authority or enable an executor. Ordinary planning remains strict; attached support/pile policies remain unchanged. The implementation is generic in link/object identity, mesh vertices, BOX pose, joint chain, mimic relationships, trajectory, and epoch.

The historical iteration-104485 opening remains **UNCERTIFIED**. The exact measured start cannot be reproduced by applying its commanded leader position: three finger-2 mimic values change by approximately 5.2–5.4e-11 rad. The certificate rejects this discontinuity rather than silently normalizing the measured geometry. No recovery authority R, simulator/fake-transport acceptance, observation, or physical acceptance was run. Attempt A remains revoked by the existing recovery boundary.

## Why raw FCL depth ordering is insufficient

The saved FCL witness depth rises from approximately 57.72 nm at 4 ms to 64.56 nm at 5 ms. FCL penetration values can depend on the selected contact features. Their ordering alone establishes neither full-geometry approach nor full-geometry separation. The new proof never uses successive reported penetration depths as its monotonicity oracle. FCL still establishes initial contact evidence and detects forbidden collision witnesses.

## Certificate

Entry point: `controller_certificate::certifyDetached(trajectory, scene, epoch, options)` in `controller_interval_certificate.hpp`. Each invocation freshly enumerates its initial contacts; a report from a previous candidate is never consumed as an allowance.

1. Validate bounded full scene geometry, supported FCL backends, no attachment, explicit epoch, exact measured trajectory start, and exact controller-derived waypoint starts. Reject conditional ACM entries in detached mode.
2. Enumerate all baseline-relevant robot/world and self contacts with global and per-pair caps. Reaching either cap, inconsistent counts, or collision without complete contact evidence is UNCERTIFIED. Initial self/attached contacts reject. World/world contacts grant no permission.
3. For each exact robot mesh/world single-BOX pair, retain raw contact evidence, all current collision poses, inspectable exact geometry identity, epoch, and certificate type. Raw initial depth and conservative full-support overlap must satisfy the existing 100 µm bound, including numerical error. Unsupported geometry fails closed.
4. Consider the six BOX face directions. Select the best fixed plane using outward projections of **all vertices of every collision mesh on the link** and the complete BOX support. No FCL normal or nearest-point depth supplies the proof.
5. Reuse the installed JTC polynomial/Bernstein bounds. Compute derivative controls once at each segment root and subdivide them independently. Bound the whole ancestor chain with interval matrices, including fixed origins, revolute/prismatic motion, collision origins, and recursive mimic factors. Sum each independent leader's full Jacobian contributions before multiplying by its derivative interval. Sin/cos bounds use interval Taylor polynomials with explicit remainders, not unqualified assumptions about libm directed rounding.
6. Require every vertex's projected continuous derivative to be nonnegative. For nonstationary active links, the implementation uses the stronger condition `minimum projected advance per ns > 2 * uniform projection error`. This also proves monotonicity of evaluated JTC integer-nanosecond outputs. The uniform error uses the **full interval** coordinate ranges and prismatic reach, not only the midpoint. Stored-waypoint switches are checked separately. Exact constant-polynomial links may remain active while another pair separates; all pairs must eventually expire for success.
7. A positive outward-rounded, metric full-geometry clearance at an accepted interval end records SEPARATED then EXPIRED. Transitions commit only after every ordinary pair and joint-bound check for that interval also passes. Failed parent intervals cannot commit expiry before subdivision.
8. Active exact pairs receive the geometric proof. All other pairs receive the existing strict continuous proof. Expired pairs return to strict checking permanently. Only already-proven pairs are omitted from a private distance-query ACM copy; the PlanningScene ACM is never enlarged. New/self/expired collision witnesses are hard failures.
9. Qualification additionally requires every initial pair EXPIRED and the final robot/world/self state strictly collision-free.

Supported moving joints are one-variable revolute/prismatic joints and their mimics; revolute interval magnitudes above 16 rad fail closed in the Taylor-bound implementation.

This proves nominal continuous geometry and actual JTC outputs on its integer-nanosecond time lattice in an immutable world. It does not certify physical tracking or arbitrary motion inside an unconstrained error tube between controller timestamps. A monotone ideal curve plus a uniform error tube is insufficient to establish monotonic actual output near zero velocity. No contact tolerance or hysteresis is added to overcome that limitation.

## Per-pair state

`ACTIVE_INITIAL_CONTACT → SEPARATED → EXPIRED` is stored independently for each exact identity pair and epoch. Both transitions are recorded at the accepted positive-clearance boundary, including the certification interval and clearance lower bound. An active pair cannot authorize a different pair. A failed candidate's partial trace is diagnostic only; it cannot issue authority or seed another candidate. A candidate that ends with an active pair is UNCERTIFIED.

## Historical opening diagnostic

Source fixture: `/home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924/withdrawal`, reconstructed from the complete saved iteration-104485 measured scene. The fixture and baseline ACM are unchanged.

| Item | Result |
|---|---|
| Candidate | Derived gripper leader −0.01 rad, 1 s quintic, diagnostic only |
| Geometry | Complete robot/world/self; all eight affected links |
| Initial pair | `gripper_finger1_finger_tip_link` ↔ `runtime::part_07` |
| Enumeration | Complete; one pair, four contact witnesses |
| Maximum raw initial depth | 5.8070583773010374e-8 m |
| Fixed plane axis | (−0.15652657455413485, −0.98767374747843095, −5.1134882087740982e-7) |
| BOX support | 0.17007358009154258 |
| Initial full-mesh projection gap | −3.8172268036096302e-5 m (approximately −38.17 µm) |
| Result | UNCERTIFIED: `DETACHED_CONTROLLER_START_DISCONTINUITY` |
| Continuously separating | Not established |
| Certified separation interval / expiry clearance | None |
| Final pair state | ACTIVE_INITIAL_CONTACT; zero transitions |
| New collisions | No new pair in recorded early diagnostic witnesses; entire path not certified |
| Stored endpoint | Strict robot/world/self collision-free, zero contacts; not executed |
| Intervals / subdivisions / depth | 0 / 0 / 0; start consistency gate rejects first |

Measured/controller mismatches (rad):

| Joint | Measured | Controller-derived | Delta |
|---|---:|---:|---:|
| `gripper_finger2_inner_knuckle_joint` | 0.55269734136575255 | 0.55269734141968274 | +5.3930193644191604e-11 |
| `gripper_finger2_finger_tip_joint` | −0.55269734136579263 | −0.55269734141968274 | −5.3890114593002636e-11 |
| `gripper_finger2_joint` | 0.55269734136811732 | 0.55269734141968274 | +5.1565418601740021e-11 |

The first remaining blocker is a continuous start binding that preserves these measured mimic coordinates without silently replacing them with nominal values. Even a consistent synthetic zero-start quintic is deliberately rejected when monotonicity cannot be proved above the JTC/FK rounding bound. Neither limitation is repaired by relaxing contact thresholds.

## Verification and reproduction

Evidence directory: `/home/ubuntu/workcell_ws/stage-a1-detached-separation-20260924`. It contains red/green logs, full gate logs/XML, historical diagnostics, exact geometry records, build commands, and a hashed report. The numerical-guard review finding and metric-plane scaling defect were each reproduced by failing tests before correction. The mimic start-discontinuity test initially demonstrated an incorrect acceptance and now rejects it.

**Final gate:** 64 C++ tests passed (zero skipped), 342 Python tests passed, and the historical diagnostic reproduced the rejection. The known-clear real Stage-A approach certified 936 intervals inspected, 518 accepted, 418 subdivisions, maximum depth 3, in 19.9227 s. Historical qualification stopped before interval traversal; exact runtime and hashes are in `report.json`.

The C++ suite covers exact eligibility, other/new pairs, raw-depth independence, approach rejection, independent expiry, recontact, self collision, unavailable plane, enumeration truncation, midpoint collisions, ordinary clearance, stale/empty epoch, zero-velocity uncertainty, unchanged initial bound, strict default entry point, revolute mimics and fixed origins, full prismatic reach, transactional expiry rollback, and backwards stored-waypoint switches. Existing support/pile and real Stage-A trajectory tests remain part of the gate. The nine scoped Python suites exercise the unchanged recovery boundary, owned cancellation, A revocation, grasp execution, extraction and finish runner.

After sourcing ROS and the existing workspace overlays, run:

```bash
cmake --build /home/ubuntu/workcell_ws/build/workcell_builder \
  --target workcell_support_contact workcell_support_contact_test -j2
WORKCELL_STAGE_A_CERTIFICATE_FIXTURE=/home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924/real-stage-a \
  /home/ubuntu/workcell_ws/build/workcell_builder/workcell_support_contact_test
```

Only after the test gate passes, reproduce the offline diagnostic:

```bash
python3 workcell_builder/workcell_builder/test/evidence/run_detached_opening_probe.py \
  /home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924/withdrawal \
  /home/ubuntu/workcell_ws/build/workcell_builder \
  /home/ubuntu/workcell_ws/stage-a1-detached-separation-20260924
```

The diagnostic loads saved files and calls C++ geometry/controller functions. It creates no ROS node, transport, execution authority, or motion.

## Safety and rollback

The 100 µm initial-contact bound, baseline ACM, ordinary continuous auditor, support/pile policies, retention thresholds, A revocation, hardware lock, and blocked recovery executor remain intact. No generic recovery candidate selector or R integration is enabled because the historical candidate did not qualify. Reverting this certificate commit restores the previous strict detached-contact rejection; no runtime configuration or saved measured fixture needs reversal. PR #3174 remains draft and must not be merged as physical acceptance evidence.
