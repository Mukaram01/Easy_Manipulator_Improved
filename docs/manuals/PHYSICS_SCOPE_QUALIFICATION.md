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
| `QUALIFIED` | offline issuer only (`issue_qualified_scope`, when every requirement holds for one software version) | not wired: the executor, Builder and HMI accept no qualified record (admission is G6) |

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
- it also binds the two G5 runs (session run ids and the same four digests), which are re-judged on every check;
- any edited, swapped, duplicated or cross-session trial or G5 run refuses.

**Q7 Cancellation:**
- the Stage-A cancellation trial is a prerequisite of every trial;
- in addition, R2.5B G5 (stop at cycle end, and SIGTERM cancellation during transfer, in the multi-destination world) must be accepted.
- **Evidence:** `qualification_readiness(..., cancellation_trial=, stop_trial=)`, or `--cancellation-trial` and `--stop-trial`. Each run is judged from its own files:
  - **G5A** (`cancellation_evidence`):
    - Q1 provenance of a `CANCELLED` authorised run;
    - the operator SIGTERM forwarded to the executor, which was not killed;
    - `transfer_cancellation_acceptance` recomputed from the summary, equal to the executor's recorded verdict, and `RECOVERY_REQUIRED_HELD_STOPPED`.
  - **G5B:** `stop_after_cycle_acceptance` (`STOPPED_AFTER_CYCLE`, which includes the strict-trial criteria).
- **Binding:** both runs must have the current native and runtime identity and the exact cell's derived world. They must share the bin, support and robot geometry with each other and with the counted trials.
- **Shared between cells:** G5 runs once per multi-destination world (R2.5B), so they are not tied to one bin's task. The same runs serve both cells of the world.
- **Distinct sessions:** the two must be different sessions.
- **Refused:**
  - missing, modified, wrong-type or mismatched evidence;
  - a run offered in the other role.
- **What counts for Q8:**
  - G5A is cancellation evidence only and never counts as a placement;
  - listed as a trial, it is a failed run;
  - G5B counts toward Q8 only when listed as a trial and meeting every per-trial criterion.
- **Accepted (2026-10-29):** G5A `log/qualification/g5a/trial_2` (run `17a40d24…`), G5B `log/qualification/g5b/trial` (run `0f6c3690…`).

**Operator stops during a trial** (the runner's own process; nothing else is signalled):

| Request | Signal to the runner | Effect | Acceptance (from the trial's own evidence) |
|---|---|---|---|
| Cancel (G5A) | SIGTERM or SIGINT | Forwarded to the executor, which cancels its owned goal at once; the part stays held, recovery is required | `simulator_execution.transfer_cancellation_acceptance`: `RECOVERY_REQUIRED_HELD_STOPPED` |
| Stop at Cycle End (G5B) | SIGUSR1 | Latched and acknowledged (`stop-after-cycle.json`, a stderr line), never forwarded. The running cycle continues under every guard and its own failure handling; no further session starts | `physics_scope_qualification.stop_after_cycle_acceptance`: `STOPPED_AFTER_CYCLE` |

- **Precedence:** a cancel after a latched stop still cancels at once; the stop record is marked `superseded_by`.
- **Stop at Cycle End is the sorting session's software stop,** not an emergency stop. If the running cycle fails, its existing failure and recovery handling applies; completion is never forced.
- **Runner outcome:** a stop that leaves selected sessions unstarted reports `STOPPED` (exit 4); a stop in the last session leaves its result unchanged.
- **G5B acceptance needs all of these:**
  - the stop was latched during `EXECUTE_TRANSFER`, between the executor's transfer and place stage events;
  - it was not forwarded and not superseded, and no cancellation or recovery followed;
  - the cycle is a strict trial PASS (`trial_evidence`);
  - no other session ran.
- **Q7:** the two acceptances are the stop/cancel evidence (see Q7 above).

**Q8 Repeatability:**
- at least **3** independent strict trial PASS sessions of the exact cell;
- any listed trial that failed after motion blocks qualification;
- trials that stopped before authorisation (no motion) are reported as reliability findings.

**Software identity (version-aware repeatability):**
- **Planner identity** (`planner_identity`): sha256 over the git blob ids of `PLANNER_SOURCES`:
  - the executor's whole repo-local import closure (34 modules, pinned by a test against the real closure);
  - the scene template launch (MoveIt pipeline, TOTG blending);
  - `catalog/capabilities` (tool hold, simulator collision boxes).
  - **Excluded:** this judge, so editing it never resets a campaign.
- **Bound to every new `AUTHORISED_TRIAL`** (`scope_binding`); the executor recomputes it before motion.
  - Historical trials recover it from their recorded clean commit; no evidence file changes.
  - A bound identity must equal the recovered one, and a run is always judged as the version it really ran.
- **Q8 counts one version** (the current one):
  - other versions are listed under `other_versions` with their outcome, never counted and never blocking;
  - a failure after motion blocks its own version;
  - every run of the exact cell and version under the evidence roots must be listed (`unlisted` otherwise); operator-cancelled runs are G5 evidence.
- **Q7 across versions:** a G5 run of another version is admissible only while its stop/cancel functions are AST-identical (`mechanism_identity`, `STOP_CANCEL_MECHANISM`).
- **Issuance** (`issue_qualified_scope`, CLI `issue`/`verify`, offline only):
  - the record binds the operation, planner identity, trials and G5 runs;
  - verify re-derives everything and refuses another operation, version, cell, build or an unlisted failure;
  - nothing consumes a qualified record (runtime admission is G6).

**Free-space execution geometry and timing (G3C):**
- **Pads:** approach, transfer, retreat and home plan and certify against the receipt-bound physics fingertip boxes (`simulation_collision.physics_pad_scene`). The boxes are inflated by 15 mm on free OMPL stages and 1 mm on the straight retreat, and carried on the request's start state as well as the scene diff (`e1fcfcbb`).
- **Corner blending:** TOTG blends corners (`TOTG_PATH_TOLERANCE` 0.01 in the scene launch). With the robot package's 0.0, corners were unblended and implied up to 18.5 rad/s² (UR5 limit 3.15) and 14 mm of tracking deviation.

**Status (2026-10-30, software `09c477e0`, head `e1fcfcbb`):** `NOT_QUALIFIED` for both cells, 0 of 3 strict trials of this version.
- **Every earlier run is listed under `other_versions`:**
  - campaign `8a5d80d2`: 3 + 3 strict PASS;
  - Bin A trial 3: FAILED.
- **Why `8a5d80d2` is superseded:** that version (`caf7dbd7`) was `QUALIFIED_READY` for both cells by this contract. It was not issued, because its pads were found inactive at runtime (the start-state defect fixed in `e1fcfcbb`).
- **Q7:** met; the G5 mechanism is unchanged.
- **Plan-only proof at `e1fcfcbb`:** both cells pass, and the pad effect is verified by replay agreement and FK clearance.
- **Evidence:** `log/qualification/g3c/`.

Offline: `python3 scripts/physics_scope_qualification.py readiness|issue|verify --cell <cell> --trial <run> ... --cancellation-trial <G5A run> --stop-trial <G5B run>` (exit 0 only when ready, issued or verified).

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
