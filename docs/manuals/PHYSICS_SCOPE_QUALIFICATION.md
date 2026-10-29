# Multi-destination physics scope: trial and qualification contract

**Scope of this document:** `scripts/physics_scope_qualification.py` (`workcell_physics_scope/v1`).

**Default:** a world with several physical destination bins is plan-only. The executor refuses motion with `MULTI_DESTINATION_PHYSICS_UNQUALIFIED`, and the contact guard binds no destination floor.

## States (never conflated)
| State | Produced by | Admits motion? |
|---|---|---|
| Valid configuration | `derive_world(physical_destinations='all')`, launch binds every bin | no |
| Planned trajectory | plan-only resolve and revalidate in the session | no |
| `AUTHORISED_TRIAL` | CLI qualification runner (`--authorise-physics-trial`) after this session's revalidation | one guarded full-cycle into one bin, CLI only |
| Strict trial PASS | the executor's verdict on that trial, labelled `PHYSICS_SCOPE_TRIAL_NOT_QUALIFIED` | no further motion |
| `QUALIFIED` | nothing yet: no issuer exists | not wired: the executor, Builder and HMI accept no qualified record |

A trial needs no earlier success: it reuses the same-build Stage-A prerequisite trials (`require_trial_evidence`). A trial never qualifies itself.

## Trial record (`AUTHORISED_TRIAL`)
**What it binds:**
- the session cell's files;
- the derived world, with every launched bin and object model equal to it;
- every bin binding and the selected bin;
- the support binding;
- the robot, controller, control-plugin and hold-profile identities;
- the native libraries and bound sources, proved on disk;
- the registered runtime and its Stage-A evidence;
- the prerequisite-trial evidence;
- the revalidated plan;
- the session receipt and run id;
- an empty destination, measured in the world frame.

**What it allows,** fixed in code: one full-cycle, one object, one destination, no routing, no sorting, no real hardware. The executor also runs it under strict retention only.

**Executor:** recomputes every binding before planning, arms the guard on that bin's floor only, and re-checks the record before its verdict.

## Qualification contract (`QUALIFICATION_CONTRACT`, fixed in code)
A qualified scope would admit **one exact cell**: its task, placing into one bin of the multi-destination world. Readiness is recomputed from the trials' own evidence files on every check.

**Q1 Provenance:** each counted trial must have:
- an official runner report: `PASS`, scope `PHYSICS_SCOPE_TRIAL_NOT_QUALIFIED`, targeted full-cycle, `stage_a_acceptance` false, clean checkout;
- a structurally valid `AUTHORISED_TRIAL` record, the same one (by sha) that its session report and executor summary verified;
- a clean owned shutdown.

**Q2 Identity:** each trial's native identity (commission, support/Cartesian and telemetry libraries, MoveIt version, bound sources) and runtime identity (record, Stage-A evidence) must equal the current ones, proved on disk.

**Q3 Cell and geometry:**
- **Cell:** the trial's cell files must equal the exact cell's. `cell_definition.yaml` is excluded because it embeds the session's plan, which is bound per trial.
- **World:** same derived world.
- **Shared across trials:** the same bin and support geometry, and the same stable robot identity. The launched URDF embeds a session path, so its source is bound through the cell's xacro files instead.

**Q4 Selected bin:** every trial placed into the cell's own task bin. A trial for another bin or another cell never counts.

**Q5 Contact, retention, release:** each trial must show:
- a strict executor `PASS`: strict policy, strict-compliant retention, no findings;
- the complete stage sequence through `EXECUTE_HOME`;
- `RELEASE_CONFIRMED` and settled;
- no attachment, baseline ACM restored, final collision state valid;
- error to the planned seat ≤ 3 mm;
- the measured part's lowest corner within 1 mm of the selected bin's bound floor, and its centre over that floor;
- the four Stage-A prerequisite trials bound to the trial build.

**Q6 Replay protection:**
- every bound trial has a distinct session run id and scope record;
- a qualified record binds each trial's report, session report, scope record and summary digests, plus its own digest;
- any edited, swapped, duplicated or cross-session trial refuses.

**Q7 Cancellation:**
- the Stage-A cancellation trial is a prerequisite of every trial;
- in addition, R2.5B G5 (stop at cycle end, and SIGTERM cancellation during transfer, in the multi-destination world) must be accepted. That acceptance does not exist yet, so this item is always unmet.

**Q8 Repeatability:**
- at least **3** independent strict trial PASS sessions of the exact cell;
- any listed trial that failed after motion blocks qualification;
- trials that stopped before authorisation (no motion) are reported as reliability findings.

**Status (2026-10-29):** `qualification_readiness` returns `NOT_QUALIFIED` for both cells.

| Cell | Strict trials | Unmet | Reliability finding |
|---|---|---|---|
| `zz_r25b_two_bin_a` | 1 of 3 (`g3b/bin_a_trial_2`) | Q7, Q8 | trial 1 |
| `zz_r25b_two_bin_b` | 1 of 3 (`g3a/bin_b_trial`) | Q7, Q8 | — |

`verify_qualified_scope` therefore refuses every record, including a correctly bound one. Offline display:
`python3 scripts/physics_scope_qualification.py readiness --cell <cell> --trial <run> ...` (exit 0 only when ready).

## Unresolved reliability issue
**What happened:** the first Bin A trial (`log/qualification/g3b/bin_a_trial`) stopped before motion with `PREPLAN_TRANSFER: SEARCH_BUDGET_EXHAUSTED` (107.5 s + 202.0 s).
- The time was spent in controller-interpolation certificates.
- One transfer interval in free space was inconclusive (`PRECISION_OR_DEPTH_LIMIT`) and failed closed.
- The discovery pass had already certified the transfer, but was discarded for exceeding its 60 s budget.

**What has not changed:** budgets, tolerances and certificates.

**What remains open:** the search-time variance is unresolved. Repeatability (Q8) evidence must record such failures, not hide them.

## Operator admission
**Builder and HMI admission stays locked** until:
- a qualified record verifies, which needs Q7 and Q8;
- a reviewed issuer and an executor admission path exist.

No qualified record is consumed anywhere today.
