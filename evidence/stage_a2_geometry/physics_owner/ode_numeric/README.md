# DART → ODE BOX numerical qualification

**PASS_SIMULATION_ONLY_NUMERICS for the six recorded contact-input states of
this exact instrumented runtime. Contact authority and EPD association BLOCKED.**
Baseline `787799f82bfc75f0b296c91f6a8cef1b9b436ecd`.

## Implementation and measured scope

`scripts/stage_a_ode_numeric.py` extends the existing rational geometry evidence
contract. It requires the complete owner/observer readback, active ODE detector
before/after stepping, actual contact-object RTTI, complete detector ShapeFrame
inventory, authoritative collision mappings, matching sessions/steps, exact
loaded and locally audited binary hashes, and the canonical fixed-world identity.
Unknown binaries, geometry, worlds or incomplete state fail closed. Campaign IDs
are verified construction witnesses, not a guessed general ID mapping rule.

The only additional live instrumentation reads detector type, collision-group
ShapeFrames and contact-object type. `detector_witness.patch` is the exact
reporting-only diff against the baseline. It changes no construction, solver,
stepping or response. The actual eight-shape group exactly matches the owner's
inventory. Cube collision IDs are **23 and 27**; constructed support ID is **7**.

Existing saved traces were evaluated first (`saved_trace_conditional.json`).
They lack an active-detector witness and remain conditional. One bounded
owner-only disposable server then ran 2000 steps at 1 ms (exit 0, 4.759277 s).
No new stock/reference campaign was run. All sampled geometry, mappings and
contact identities/counts/positions/normals/depths exactly equal the previous
owner capture; process pointers/session identifiers differ. Both ODE and Bullet
libraries are loaded, so loaded-library presence alone is explicitly insufficient.
The new capture confirms active ODE and actual `OdeCollisionObject` contacts.

The original world and disposable derived world are committed. The evaluator
removes only the verified observer and owner reporting configuration, restores
the stock Physics plugin name, and compares canonical XML with the original
world. The loaded world's raw hash must also match `run.json`. No installed
libraries or protected Stage-A1 files were modified.

## Binary-supported conversion chain

`provenance.json` records exact SHA-256 identities, packages and selected ELF
function ranges; `*_audited_disassembly.txt` preserves the corresponding code.
DART is 6.12.1 (`6.12.1+dfsg4-11build2`), ODE is 0.16.2 (`2:0.16.2-1`),
Eigen headers are 3.4.0 (`3.4.0-2ubuntu2`), Fortress is 6.18.0 and the loaded
DART physics backend is 5.4.0. ODE's runtime configuration reports
`ODE_double_precision`; selected instructions use binary64 SSE2 arithmetic.
Eigen's relevant conversion is inline in the audited DART collision ELF.

The selected live BOX path is:

1. `CollisionObject::getTransform()` obtains the ShapeFrame world transform.
2. `OdeCollisionObject::updateEngineData()` copies translation and constructs
   a double quaternion from the matrix. Actual trace order is `(R22+R11)+R00`.
   The positive-trace and largest-diagonal branches use the audited order of
   subtractions, square root, divisions, products and component copies.
3. `dBodySetQuaternion()` copies the four doubles and calls
   `dxSafeNormalize4`: `(((w*w+x*x)+y*y)+z*z)`, square root, reciprocal, four
   multiplications. The valid near-unit domain never reaches its fallback.
4. `dRfromQ()` reconstructs the rotation using doubled products, differences
   and sums in the recorded binary order.
5. The BOX constructor passes all three dimensions directly to `dCreateBox`.
   The DART BOX geometry/body path creates zero position and identity rotation
   offsets. ODE body/geometry composition with those offsets is numerically
   exact for these finite inputs. ODE is used for collision detection; there
   is no separate ODE dynamics world or step. The fixed world has no joints or
   user transform mutations along this path.
6. The support is the **actual constructed 2100 × 2100 × 2100 m BOX**, with
   identity orientation and centre z = −1050 m. Its top is exactly zero and
   its finite footprint is explicitly checked. The original SDF plane alone
   is never accepted as detector geometry evidence.

Matching DART source from the retained source archive and matching ODE source
from the Ubuntu archive explain this path; source hashes/URLs are recorded.
Release labels alone are not the numeric proof. Complete historical compiler
flags and bit-for-bit source reproduction of installed libraries are unavailable.
The claim instead uses selected operations in the exact loaded ELFs: no fused
multiply-add or x87 arithmetic on this path, positive-domain hardware square
root, and double copies/operations. The installed physics backend has local
extensions; no claim of pristine upstream binary identity is made.

## Outward enclosure

Each input is a finite, exactly representable binary64 rational. At every audited
primitive, exact `Fraction` endpoint arithmetic is rounded down/up to adjacent
binary64 endpoints. This encloses every IEEE rounding mode, including cancellation.
Square roots are bracketed by integer square root on the exact 2^-1074 grid and
then rounded outward; no empirical epsilon or host libm approximation is used.
A trace interval crossing zero evaluates both quaternion branches and takes the
union after normalization/reconstruction. Largest-diagonal comparisons operate
on the exact copied binary coefficients.

Nonfinite values, overflow, invalid rotations, zero-containing normalization
or division intervals, unsupported semantics, and subnormal endpoint domains
are rejected. The existing rigid-matrix validity tolerance is an input gate,
not an error allowance: every accepted coefficient is propagated as its exact
binary rational. This proof does not assume an exact orthogonal input matrix.

All eight BOX corners propagate converted orientation, actual translation and
half dimensions. The independent stored-DART rational enclosure is preserved.
The converted support must remain exactly horizontal; all cube corner intervals
must fit wholly inside its finite footprint. The upper bound is support-top
upper endpoint minus minimum cube-corner lower endpoint, clamped at zero and
rounded outward for output. Contact-manifold depth is never a whole-shape bound.

| Quantity over the recorded states | Upper bound |
| --- | ---: |
| A: stored-DART deepest penetration | 3.6698679250196645e-6 m |
| B: cube conversion/corner enclosure error, maximum L∞ | 4.7243685999152796e-17 m |
| Support top/geometry conversion error | 0 m (exact identity/zero-offset case) |
| C: combined detector-side deepest penetration | **3.6698679250213306e-6 m** |
| Final contact-input combined bound | **9.810104940194408e-9 m** |
| Unchanged allowed maximum | **1e-4 m** |

The L∞ error includes conservative corner evaluation rounding as well as the
rotation conversion. The combined bound is computed directly from intervals;
it does not add a manually chosen backend allowance. Every sampled pair passes
both its contact-input and prospective post-step threshold checks.

## Exact state and time binding

DART's audited step computes velocities, performs constraint/contact solving,
and then integrates positions. The fixed-world Fortress/backend path calls that
step without an intervening position edit. The owner's pre-step ShapeNode
transform is therefore the transform supplied to that step's collision update.
Post-step ShapeNode transforms belong to the integrated endpoint; their modeled
conversion is labelled **prospective**, not an actual detector readback.

| Contact evaluation step / nominal report ns | DART input frame / nominal input ns |
| --- | --- |
| 100 / 100000000 | 99 / 99000000 |
| 250 / 250000000 | 249 / 249000000 |
| 500 / 500000000 | 499 / 499000000 |
| 1000 / 1000000000 | 999 / 999000000 |
| 1500 / 1500000000 | 1499 / 1499000000 |
| 2000 / 2000000000 | 1999 / 1999000000 |

These are nominal Gazebo schedule times, not qualified renderer exposure times
or a claim that accumulated floating DART time is an exact integer nanosecond.
No unobserved step, continuous trajectory, stock-unexposed field or real
hardware is certified. Earlier stock/reference/owner finite equivalence remains
separate historical evidence; this new runtime measures the instrumented owner.

## Focused verification and reproduction

95 focused Python tests pass, including independent 100-digit Decimal oracles,
identity/tiny/large rotations, all largest-diagonal branches, a trace-boundary
union, all four hardware rounding modes (96 native probe records), rotated
corner extremes, 99/100.1/200 µm threshold cases, the large support offset,
nonfinite/invalid matrices, unsupported arithmetic/binary configurations,
missing detector/contact data, wrong mapping/object type, incomplete inventory,
stale steps and changed-world rejection. The native probe is only an arithmetic
oracle; it constructs no physics world and does not grant authority. Native
checks may skip on CI hosts without the audited library/development toolchain.
The affected owner plugin builds and its binding CTest passes (1/1).

Offline re-evaluation, on the workstation retaining the exact audited binaries:

```sh
E=evidence/stage_a2_geometry/physics_owner/ode_numeric
python3 scripts/stage_a_ode_numeric.py --owner "$E/owner_readback.jsonl" \
  --observer "$E/observer.jsonl" --run "$E/run.json" --world "$E/world.sdf" \
  --output /tmp/stage_a2_ode_recheck.json
PYTHONPATH=scripts python3 -m pytest -q tests/test_stage_a_ode_numeric.py \
  tests/test_stage_a_owner_geometry.py tests/test_stage_a_physics_contact.py \
  tests/test_stage_a_owner_contact_compare.py
ctest --test-dir /tmp/stage_a2_owner_build --output-on-failure
```

The exact overlay hashes are deliberately required: a rebuild with a different
binary identity must be re-audited instead of silently inheriting qualification.
`qualification.json` exports existing physical-bound contract fields but keeps
`contact_authority=false` and `support_contact_permission=false`.
No EPD poses/envelopes changed; no native extraction, MoveIt or execution goals.

**Single next action:** independently establish renderer/physics exposure-step
alignment and a unique visible physical-object-to-genuine-EPD-mask identity
binding. Projection overlap alone is insufficient. Until then, admission remains
BLOCKED even though the scoped numerical gate passes.
