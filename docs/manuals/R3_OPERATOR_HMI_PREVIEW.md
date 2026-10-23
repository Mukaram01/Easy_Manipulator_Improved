# R3 — Integrated Operator HMI Preview

The Operator HMI is the operator-facing surface of a Workcell Studio cell. In R3 it is a **preview**: an engineer opens it from Workcell Builder to see exactly what an operator would see for the current cell, while continuing to edit the scene.

It is read-only engineering consumption plus offline, no-motion checks. It never edits the scene, never defines a task, grasp, target or destination, and never commands robot motion.

| Responsibility | Owner |
|---|---|
| Physical cell, task intent, generation, validation, simulation | Workcell Builder |
| Perception configuration and outputs | EPD |
| Operator view and permitted runtime controls | Operator HMI |

## Opening the preview

Open a cell in Workcell Builder, then use any of:

- **Operator Preview** on the Scene Builder toolbar;
- **View ▾ → Operator Preview** in the Scene Builder overflow menu;
- **More ▾ → Operator Preview** on the Workcell Studio Home inspector.

The HMI opens as a separate window and process, `workcell_operator_hmi`. Workcell Builder stays open and editable.

Clicking Operator Preview again refreshes and raises the same window; it never starts a second HMI. Closing the HMI does not affect the Builder. Closing the Builder closes the HMI.

## How scene data reaches the HMI

```text
Workcell Builder ──request (private temp file)──► workcell_operator_hmi
                                                       │
                     scripts/operator_preview_projection.py --scene <dir> [--draft-stdin] [--evaluate-input]
                                                       │
       task_intent_authoring.load_authoring · task_intent_resolver · saved Validate reports
       · bound commissioning replay · generated bundle approval record
```

- **Request.** `workcell_operator_preview_request/v1` lives in a Builder-owned temporary directory, never in the scene. It carries:
  - the scene directory;
  - whether the task editor holds unsaved edits, and if so its draft;
  - whether the layout has unsaved edits;
  - a revision number.
- **Process launch.** The HMI starts with an argument list (no shell) and `PR_SET_PDEATHSIG`, so it cannot outlive the Builder.
- **Projection.** `workcell_operator_preview/v1` is derived and read-only. It reuses the contracts the Builder already uses and writes nothing:
  - the authoring adapter (v1 → v2 migration, validation, normalized intent SHA-256);
  - task resolution (`READY` / `BLOCKED` with reason codes);
  - the last *Validate Generated Scene* and *Offline Validation* reports, with currency checked against `authored_input_fingerprint`;
  - the bound task input;
  - bundle approval, read from `generated/generated_workcell_summary.json` (`mark_workcell_bundle_approved.py`).

## Draft and saved revisions

| Builder state | HMI shows |
|---|---|
| Task editor clean | **SAVED** revision from disk, with its normalized intent SHA-256 |
| Unsaved task edits | **DRAFT**: the editor's own canonical v2 draft, validated read-only; a banner says *not saved, not approved* and whether it differs from the saved task |
| Unsaved layout edits | A notice that the physical scene shown is the **last saved** revision: geometry reaches `environment.yaml` only on Save |

When the Builder changes (task edit, Save, revert, scene switch), a running HMI shows **Workcell Builder has newer engineering state — Refresh**. It does not silently replace the view. Operator Preview in the Builder, or Refresh in the HMI, loads it.

A draft is never shown as approved and never enables production.

## What the operator sees

- **Header.** Cell name and badges for REVISION (SAVED/DRAFT), APPROVAL, MODE, OFFLINE CHECKS and PRODUCTION (always NOT AUTHORISED in R3).
- **Cell overview.** Robot, end effector and capability, camera and perception contract.
- **Task.** Target class and filters, the not-targeted rule, pick zone, destinations, routing and grasp policy.
- **Validation and readiness.** Task-intent validation, scene and offline validation (with STALE marking), task resolution and task input.
- **Cell monitor.** A top-view map of the authored pick and place zones. After a replay check it also shows the recorded objects, labelled **REPLAY · recorded commissioning snapshot, not a live camera**.
- **Detected objects.** ACCEPTED or REJECTED with the resolver's actual reason (for example `CLASS_NOT_TARGETED`, `OUTSIDE_PICK_ZONE`, `OBSERVATION_STALE`), plus details for the selected object.
- **Robot and cycle.** NOT CONNECTED / NO CYCLE RUNNING: there is no runtime link in the preview.
- **Events.** Task selected, validation results, object accepted or rejected, missing perception input, missing destination, execution backend unavailable, and offline test started, completed, failed or stopped. **Engineering details** shows the exact commands and reports.

Missing evidence is shown as UNAVAILABLE or UNKNOWN, never as zero or success.

TaskIntent v2 has no explicit ignore list. "Ignored objects" therefore states the actual rule: objects that do not match the pick filter are not picked.

## Operator workflow (R3)

1. **Recipe:** the cell's task, approval and draft state.
2. **Check readiness:** re-read the configuration and evidence (read-only).
3. **Run a permitted offline task:**
   - **Replay check:** evaluates the task's bound commissioning replay with the resolver's own selection rules. It changes the MODE to REPLAY.
   - **Generated-cell dry-run:** the existing `run_generated_workcell_bundle.py --gated-dry-run --dry-run --no-replay`. Its reports go to a private temporary directory. It is available after *Generate Scene Package* and shows its preflight blockers verbatim.
4. **Monitor:** activity indicator.
5. **Stop check:** ends a running offline check. It is not a safety function.
6. **Review results.**

**Start production cycle** is always disabled in R3, and the reasons are listed: no qualified execution backend, no live input, no approved bundle, draft or layout edits, resolution not ready. The HMI provides no emergency stop; use the cell's hardware E-stop.

## Standalone use

```bash
workcell_operator_hmi --scene <workspace>/src/easy_manipulation_deployment/scenes/ur5_2f_test
```

Standalone mode shows the saved revision only.

## Acceptance (hardware-free)

Recorded results and screens: [operator-hmi-preview-v1](../acceptance/operator-hmi-preview-v1.md).

```bash
# disposable workspace: <ws>/src/easy_manipulation_deployment is a copy of this checkout
WORKCELL_WORKSPACE_ROOT=<ws> workcell_builder --operator-preview-acceptance \
  --smoke-output <out> --scene ur5_2f_test --workspace <ws>
WORKCELL_WORKSPACE_ROOT=<ws> workcell_builder --operator-preview-acceptance \
  --smoke-output <out> --scene ur5_2f_test --workspace <ws> --reopen
```

The harness drives the real Builder and the real HMI process:
1. open the cell, launch the preview and check that it shows the saved task;
2. make a draft edit (update banner, then DRAFT on refresh), run the replay check, save, and refresh to SAVED/REPLAY;
3. run Generate and Validate, then the generated-cell dry-run;
4. bind a missing replay and confirm the replay check and Start are refused;
5. close the HMI with the Builder left usable, then run five open/close cycles with no leftover processes.

A separate reopen run checks that the saved intent SHA-256 is recovered. The HMI is driven only through its own buttons, via `--acceptance-commands`, a test-only option.

## Not in R3

These remain blocked or out of scope:
- simulation started from the HMI (fake-hardware simulation stays in Workcell Builder, Plan / Simulate);
- live perception consumption;
- robot state and cycle telemetry;
- approval workflow;
- production start and real-robot commissioning;
- PLC, OPC UA or Modbus, and remote or browser control.
