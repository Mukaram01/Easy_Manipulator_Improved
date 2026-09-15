# R2.0b Shared Readiness / Planner Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one truthful R2.0b task-intent resolver that reuses a single extracted collision-aware full-cycle preplanner, supports deterministic `AUTO`/`PREFERRED`/`EXACT` behavior for the existing 2F strategies, and preserves R1.9 target-local placement authority.

**Architecture:** Extract the already-proven full-cycle plan-only logic from `scripts/perceived_object_grasp_execute.py` behind a reusable preplanner API without changing its safety semantics. Feed that API deterministic strategy-specific grasp candidates, then make `task_intent_resolver.py` own policy/fallback/readiness while `physical_destination.py` remains the only local-to-world destination authority. `perceived_object_grasp_execute.py` becomes a consumer of the shared preplanner rather than containing a second planner.

**Tech Stack:** Python 3, ROS 2 Humble, MoveIt 2 services/actions, pytest, PyYAML/JSON, existing Workcell Studio scene contracts.

**Spec:** `docs/superpowers/specs/2026-09-15-r20b-shared-readiness-planner-extraction-design.md`

## Global Constraints

- Canonical branch: `codex/r20b-shared-readiness-resolver`; never implement on `main`.
- Canonical scene: `scenes/ur5_2f_test`.
- Fake hardware / plan-only first; no real robot motion.
- `TaskIntentModel v2` remains the only authored task source.
- `scripts/physical_destination.py` remains the sole physical destination resolver.
- No second planner and no `IK == READY` shortcut.
- No fabricated `PASS` for MoveIt-dependent checks.
- No scene-specific IDs in generic resolver/preplanner code.
- `EXACT` never substitutes, clamps, snaps, or silently falls back.
- `PREFERRED` fallback is explicit and yields `WARNING`.
- `AUTO` selection is deterministic for identical semantic inputs.
- Suction remains unsupported in R2.0b and must fail closed.
- YAML and JSON resolution artifacts must be semantically identical.
- R2.0a and R1.9 regressions must remain green.

---

### Task 1: Lock the RED resolver contract and add R1.9 arbitrary-local-pose support

**Files:**
- Modify: `scripts/physical_destination.py`
- Modify: `tests/test_physical_destination.py`
- Existing RED contract: `tests/test_task_intent_resolver.py`

**Interfaces:**
- Consumes: existing `resolve_destination(environment, zone_id, check_projection=True)`.
- Produces: `resolve_local_destination(environment: dict, asset_ref: str, region_ref: str, local_pose: dict) -> dict`.
- Invariant: all requested local placement poses are transformed to world only by `physical_destination.py`.

- [ ] **Step 1: Add failing local-pose resolver tests**

Add tests equivalent to:

```python
def test_resolve_local_destination_uses_target_frame_and_region_bounds():
    from physical_destination import resolve_local_destination
    env = scene()
    result = resolve_local_destination(
        env,
        "bin",
        "drop",
        {"xyz_m": [0.05, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]},
    )
    assert result["target_id"] == "bin"
    assert result["id"] == "drop"
    assert result["placement_local"]["pose_xyz"] == pytest.approx([0.05, 0.0, 0.10])
    assert result["pose_xyz"] == pytest.approx([1.0, 2.05, 3.10])
    assert result["physical_destination_contract"] == "target_local/v1"


def test_resolve_local_destination_rejects_pose_outside_named_region():
    from physical_destination import resolve_local_destination
    with pytest.raises(ValueError, match="outside"):
        resolve_local_destination(
            scene(), "bin", "drop",
            {"xyz_m": [0.30, 0.0, 0.10], "rpy_rad": [0.0, 0.0, 0.0]},
        )
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run:

```bash
python3 -m pytest -q \
  tests/test_physical_destination.py::test_resolve_local_destination_uses_target_frame_and_region_bounds \
  tests/test_physical_destination.py::test_resolve_local_destination_rejects_pose_outside_named_region
```

Expected: FAIL because `resolve_local_destination` does not exist.

- [ ] **Step 3: Implement the minimal R1.9 extension**

Implement in `scripts/physical_destination.py` with this shape:

```python
def resolve_local_destination(environment, asset_ref, region_ref, local_pose):
    baseline = resolve_destination(environment, region_ref)
    if baseline["target_id"] != asset_ref:
        raise ValueError(
            f"destination {region_ref}: target mismatch {baseline['target_id']!r} != {asset_ref!r}"
        )
    xyz = vector(local_pose.get("xyz_m"), "requested local xyz")
    rpy = vector(local_pose.get("rpy_rad"), "requested local rpy")
    local = {
        "pose_xyz": xyz,
        "pose_rpy": rpy,
        "dimensions": list(baseline["dimensions"]),
    }
    # Requested point/orientation must remain inside the named region and the
    # target usable volume. No clamp/snap/substitution occurs here.
    contains(baseline["placement_local"], xyz, rotation(rpy), [0.0, 0.0, 0.0])
    contains(baseline["usable_placement"], xyz, rotation(rpy), [0.0, 0.0, 0.0])
    target = next(a for a in environment["assets"] if a.get("id") == asset_ref)
    tp, tr = pose(target)
    world_xyz = [a + b for a, b in zip(tp, apply(tr, xyz))]
    world_rot = multiply(tr, rotation(rpy))
    result = dict(baseline)
    result.update(
        placement_local=local,
        pose_xyz=world_xyz,
        pose_rpy=angles(world_rot),
    )
    return result
```

If zero-size point containment is awkward with the existing `contains()` contract, add a focused point-in-box helper inside `physical_destination.py`; do not implement transforms elsewhere.

- [ ] **Step 4: Run all R1.9 physical destination tests**

Run:

```bash
python3 -m pytest -q tests/test_physical_destination.py
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add scripts/physical_destination.py tests/test_physical_destination.py
git commit -m "feat: resolve requested target-local placements through R1.9"
```

---

### Task 2: Extract deterministic grasp strategy candidate generation

**Files:**
- Create: `scripts/grasp_strategy_candidates.py`
- Create: `tests/test_grasp_strategy_candidates.py`
- Read/reuse helpers from: `scripts/perceived_object_grasp_plan.py`
- Read catalog: `catalog/grasp_strategies/top_2f.yaml`, `side_grip_basic.yaml`, `finger_pinch_basic.yaml`

**Interfaces:**
- Produces:

```python
@dataclass(frozen=True)
class GraspCandidate:
    strategy_ref: str
    candidate_id: str
    object_id: str
    grasp_pose: tuple[float, float, float, float, float, float, float]
    approach_pose: tuple[float, float, float, float, float, float, float]
    effective: dict


def generate_strategy_candidates(
    strategy_ref: str,
    observation: dict,
    grasp_intent: dict,
    catalog_dir: Path | None = None,
) -> list[GraspCandidate]: ...
```

- Deterministic ordering: strategy-local candidate IDs `strategy_ref::000`, `::001`, ... in a stable geometry-derived order.

- [ ] **Step 1: Write failing strategy tests**

Cover at minimum:

```python
def test_top_2f_matches_legacy_top_down_candidate_order(): ...
def test_side_grip_basic_uses_horizontal_x_plus_approach(): ...
def test_finger_pinch_basic_is_distinct_and_deterministic(): ...
def test_unknown_strategy_fails_closed(): ...
def test_candidate_generation_contains_no_scene_ids(): ...
```

For `top_2f`, compare candidate poses against the current `generate_box_grasp_candidates()` geometry for the same observation and authored approach distance.

- [ ] **Step 2: Run the new test file and confirm RED**

```bash
python3 -m pytest -q tests/test_grasp_strategy_candidates.py
```

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement `top_2f` first**

Use existing pose math from `perceived_object_grasp_plan.py` rather than duplicating quaternion math. Materialize effective fields from the authored intent, with the catalog used only for vocabulary/compatibility validation.

- [ ] **Step 4: Run only `top_2f` parity tests and make them GREEN**

```bash
python3 -m pytest -q tests/test_grasp_strategy_candidates.py -k top_2f
```

Expected: PASS.

- [ ] **Step 5: Implement `side_grip_basic` and `finger_pinch_basic`**

Rules:
- `side_grip_basic`: horizontal side approach derived from live object center/extents; catalog `x_plus` semantics must be represented physically.
- `finger_pinch_basic`: parallel-finger pinch using object/tool-aligned geometry; do not alias it to `top_2f` by name only.
- Both must derive positions from observation geometry; no `ur5_2f_test` constants.

- [ ] **Step 6: Run the full candidate suite**

```bash
python3 -m pytest -q tests/test_grasp_strategy_candidates.py
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add scripts/grasp_strategy_candidates.py tests/test_grasp_strategy_candidates.py
git commit -m "feat: add deterministic 2F grasp strategy candidates"
```

---

### Task 3: Extract one reusable full-cycle preplanner with top_2f parity

**Files:**
- Create: `scripts/full_cycle_preplanner.py`
- Create: `tests/test_full_cycle_preplanner.py`
- Modify later consumer only after parity: `scripts/perceived_object_grasp_execute.py`
- Reuse: `scripts/perceived_object_grasp_plan.py`

**Interfaces:**
- Produces a ROS-agnostic orchestration shell around injected MoveIt operations:

```python
@dataclass
class PreplanResult:
    success: bool
    candidate_id: str | None
    reason_code: str | None
    reason: str | None
    checks: list[dict]
    stages: list[dict]
    cycle: dict | None


def preplan_full_cycle(
    *,
    initial_scene,
    observation: dict,
    candidate,
    destination: dict,
    contract: dict,
    operations,
    deadline: float,
) -> PreplanResult: ...
```

`operations` owns ROS calls (`plan_segment`, `fk`, `state_validity`, scene copy/apply helpers). The core preplanner owns stage ordering and fail-closed cycle semantics.

- [ ] **Step 1: Add RED extraction/parity tests**

Use fake injected operations to assert the exact stage order:

```python
EXPECTED = [
    "PREPLAN_APPROACH",
    "PREPLAN_GRASP",
    "PREPLAN_CLOSE_GRIPPER",
    "ATTACH",
    "PREPLAN_LIFT",
    "PREPLAN_TRANSFER",
    "PREPLAN_PLACE",
    "PREPLAN_OPEN_GRIPPER",
    "DETACH",
    "PREPLAN_RETREAT",
    "PREPLAN_HOME",
]
```

Also test:
- failure at any stage yields `success=False` and preserves the first stable failure code;
- no execution operation exists in the preplanner API;
- candidate metadata survives unchanged;
- destination containment is checked before success.

- [ ] **Step 2: Confirm RED**

```bash
python3 -m pytest -q tests/test_full_cycle_preplanner.py
```

Expected: FAIL because the module/API does not exist.

- [ ] **Step 3: Extract minimal orchestration from `perceived_object_grasp_execute.py`**

Move only the reusable plan-only responsibilities. Keep existing safety details:
- private PlanningScene prediction;
- target-only ACM handling;
- contact validation;
- attach/detach transitions;
- lift/transfer/place/open/retreat/home;
- `check_object_containment`;
- stable stage diagnostics.

Do not move actual `ExecuteTrajectory` calls into this module.

- [ ] **Step 4: Run extraction tests GREEN**

```bash
python3 -m pytest -q tests/test_full_cycle_preplanner.py
```

Expected: PASS.

- [ ] **Step 5: Add a legacy top_2f adapter parity test**

Construct the same top-down candidate request through both the old helper path and the extracted API with mocked MoveIt operations; assert the same selected candidate order, stage order, and failure propagation.

- [ ] **Step 6: Commit**

```bash
git add scripts/full_cycle_preplanner.py tests/test_full_cycle_preplanner.py
git commit -m "refactor: extract full-cycle plan-only preplanner"
```

---

### Task 4: Make the runtime executor consume the shared preplanner

**Files:**
- Modify: `scripts/perceived_object_grasp_execute.py`
- Modify: `tests/test_transactional_pick_cycle.py`
- Modify/add focused tests in: `tests/test_perceived_object_grasp_execute.py`

**Interfaces:**
- Consumes: `generate_strategy_candidates()` and `preplan_full_cycle()`.
- Runtime retains execution-specific responsibilities: ROS node/client creation, fake-hardware proof, live scene acquisition, execution/cancellation/recovery, evidence writing.

- [ ] **Step 1: Add failing adapter regression tests**

Test that:
- runtime candidate enumeration delegates to shared strategy generation;
- plan-only selection delegates to shared preplanner;
- `--start` remains required for any trajectory execution;
- fake hardware guard remains unchanged;
- a failed preplan results in zero execution calls.

- [ ] **Step 2: Run focused runtime tests and confirm RED**

```bash
python3 -m pytest -q \
  tests/test_transactional_pick_cycle.py \
  tests/test_perceived_object_grasp_execute.py
```

- [ ] **Step 3: Replace the embedded candidate/preplan loop with adapter calls**

Keep ROS-specific closures such as `plan_segment`, `fk`, and validity service access local to the runtime, then pass them into the shared preplanner. Remove duplicated stage-orchestration code only after the new path is covered.

- [ ] **Step 4: Re-run focused runtime tests**

Expected: PASS with unchanged execution guard semantics.

- [ ] **Step 5: Commit**

```bash
git add scripts/perceived_object_grasp_execute.py \
  tests/test_transactional_pick_cycle.py \
  tests/test_perceived_object_grasp_execute.py
git commit -m "refactor: route fake runtime through shared preplanner"
```

---

### Task 5: Implement the R2.0b policy/readiness resolver

**Files:**
- Create: `scripts/task_intent_resolver.py`
- Modify: `tests/test_task_intent_resolver.py`
- Reuse: `scripts/task_intent_v2.py`, `scripts/physical_destination.py`, `scripts/grasp_strategy_candidates.py`

**Interfaces:**
- Required public API:

```python
def resolve_task_intent(
    intent: dict,
    environment: dict,
    cell: dict,
    observations: list[dict],
    cycle_evaluator,
    *,
    now: float,
) -> dict: ...


def write_resolution_artifacts(result: dict, output_dir: Path) -> dict[str, str]: ...
```

`cycle_evaluator(request)` consumes a request containing at least `strategy_ref`, `observation`, `candidate`, `destination`, and effective motion fields, and returns `success`, `checks`, `reason_code`, `reason`, plus optional cycle evidence.

- [ ] **Step 1: Run the existing R2.0b contract tests and confirm RED**

```bash
python3 -m pytest -q tests/test_task_intent_resolver.py
```

Expected: FAIL because `scripts/task_intent_resolver.py` does not exist.

- [ ] **Step 2: Implement normalized-intent and capability gates first**

Use `parse_validate_normalize()` and `normalized_intent_hash()` as the only task-intent validation/hash path. Map installed tool IDs through the existing 2F tool mapping and emit:

```python
{
  "installed_tool_id": tool_id,
  "capabilities": ["two_finger_parallel"],
  "release_mapping": {"tool_release": "fingers_open"},
}
```

Unsupported installed capability returns `BLOCKED` with `REQUIRED_CAPABILITY_UNAVAILABLE` and does not call the cycle evaluator.

- [ ] **Step 3: Implement deterministic observation filtering**

Filter by class/confidence/age/frame/finite geometry, then sort by:

```python
(-confidence, id)
```

for otherwise eligible observations. Preserve observation identity/provenance in the artifact.

- [ ] **Step 4: Implement place policy resolution through R1.9 only**

- `AUTO`: use the named region's authored `placement_local` as the deterministic first valid local point for R2.0b.
- `PREFERRED`: call `resolve_local_destination()` for the requested pose; on invalidity, fallback to the same named region's authored local default and record `PLACE_LOCAL_POSE_OUTSIDE_REGION` (or the more specific stable code).
- `EXACT`: call `resolve_local_destination()` once; on invalidity, `BLOCKED`, preserve requested pose for diagnostics, no fallback.

- [ ] **Step 5: Implement grasp policy resolution**

Deterministic strategy order:

```python
SUPPORTED_STRATEGY_ORDER = (
    "top_2f",
    "side_grip_basic",
    "finger_pinch_basic",
)
```

- `AUTO`: evaluate supported strategies in that order, then candidate order, until a complete preplan succeeds.
- `PREFERRED`: requested strategy first; if all its candidates fail, evaluate remaining supported strategies in canonical order and record the requested strategy's stable first blocker as fallback reason.
- `EXACT`: evaluate only requested strategy with the exact authored effective constraints; first complete failure yields `BLOCKED`; no substitute strategy.

- [ ] **Step 6: Aggregate truthful readiness**

Rules:
- any hard blocker -> `BLOCKED`;
- successful PREFERRED fallback -> `WARNING`;
- otherwise complete successful physical preplan -> `READY`.

MoveIt-dependent checks absent from evaluator output must be `NOT_RUN`, never invented as `PASS`.

- [ ] **Step 7: Implement artifact writing**

```python
def write_resolution_artifacts(result, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = output_dir / "task_intent_resolution.yaml"
    json_path = output_dir / "task_intent_resolution.json"
    yaml_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    json_path.write_text(json.dumps(result, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    return {"yaml": str(yaml_path), "json": str(json_path)}
```

- [ ] **Step 8: Run the resolver contract suite GREEN**

```bash
python3 -m pytest -q tests/test_task_intent_resolver.py
```

Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add scripts/task_intent_resolver.py tests/test_task_intent_resolver.py
git commit -m "feat: add shared task intent resolver and readiness"
```

---

### Task 6: Derive task_recipe/v1 from the resolution artifact

**Files:**
- Create: `scripts/task_intent_recipe.py`
- Create: `tests/test_task_intent_recipe.py`
- Do not modify GUI generation yet.

**Interfaces:**
- Produces:

```python
def resolution_to_task_recipe(intent: dict, resolution: dict) -> dict: ...
```

- Recipe is derived only; it must carry additive intent/resolution provenance and selected grasp/place values without becoming authoritative.

- [ ] **Step 1: Write failing recipe parity tests**

Assert:
- recipe selected strategy equals `resolution["grasp_resolution"]["selected_strategy_ref"]`;
- recipe destination local/world values equal resolution values;
- normalized intent hash is copied exactly;
- a `BLOCKED` resolution cannot produce an executable recipe;
- changing authored intent without re-resolution cannot be silently accepted because hash mismatch blocks conversion.

- [ ] **Step 2: Confirm RED**

```bash
python3 -m pytest -q tests/test_task_intent_recipe.py
```

- [ ] **Step 3: Implement the minimal derived adapter**

Emit `schema_version: task_recipe/v1` with additive:

```python
"provenance": {
    "normalized_intent_sha256": resolution["normalized_intent_sha256"],
    "resolution_schema": resolution["schema"],
}
```

and selected strategy/place fields copied from the resolution, not recomputed.

- [ ] **Step 4: Run recipe tests GREEN**

```bash
python3 -m pytest -q tests/test_task_intent_recipe.py
```

- [ ] **Step 5: Commit**

```bash
git add scripts/task_intent_recipe.py tests/test_task_intent_recipe.py
git commit -m "feat: derive task recipe from R2 resolution"
```

---

### Task 7: Full focused regression and canonical fake-hardware acceptance

**Files:**
- Modify only if evidence formatting needs additive fields: `scripts/run_r14_plan_only_acceptance.py` or a new focused `scripts/run_r20b_acceptance.py`.
- Evidence output target: `docs/manuals/evidence/r20b/` only after workstation run.

**Interfaces:**
- Acceptance consumes the same resolver/preplanner path used by runtime.
- No execution action is sent in plan-only acceptance.

- [ ] **Step 1: Run all offline focused tests**

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 -m pytest -q \
  tests/test_task_intent_v2.py \
  tests/test_physical_destination.py \
  tests/test_grasp_strategy_candidates.py \
  tests/test_full_cycle_preplanner.py \
  tests/test_task_intent_resolver.py \
  tests/test_task_intent_recipe.py \
  tests/test_transactional_pick_cycle.py \
  tests/test_perceived_object_grasp_execute.py
```

Expected: all PASS.

- [ ] **Step 2: Rebuild affected ROS packages**

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select workcell_builder ur5_2f_test \
  --allow-overriding workcell_builder ur5_2f_test
```

Expected: build succeeds.

- [ ] **Step 3: Run existing C++ R2.0a readiness/model tests**

```bash
ctest --test-dir build/workcell_builder \
  -R "workcell_task_intent_(model|readiness)_test" \
  --output-on-failure
```

Expected: 2/2 PASS.

- [ ] **Step 4: Run canonical plan-only fake-hardware acceptance**

Use the existing R1.4/R1.5 fake-hardware launch path with execution disabled and the canonical `ur5_2f_test` scene. Collect evidence showing:
- `use_fake_hardware=true`;
- trajectory execution disabled for plan-only acceptance;
- selected strategy/candidate;
- all full-cycle preplan stages succeed;
- R1.9 world destination matches the selected local pose;
- live PlanningScene remains unchanged after prevalidation;
- `execution_attempted=false`.

If a new runner is needed, create `scripts/run_r20b_acceptance.py` as a thin orchestrator over resolver + existing fake launch; do not duplicate planning logic.

- [ ] **Step 5: Add PREFERRED and EXACT workstation cases**

Collect at least:
- PREFERRED requested valid -> requested == selected, no fallback;
- PREFERRED requested invalid -> alternate selected, `WARNING`, explicit reason;
- EXACT valid -> exact strategy and local place pose consumed unchanged;
- EXACT invalid -> `BLOCKED`, no fallback, zero execution.

- [ ] **Step 6: Run repository hygiene checks**

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
git diff --check
git status --short
```

Expected: no whitespace errors; only intentional evidence files remain before commit.

- [ ] **Step 7: Commit acceptance evidence**

```bash
git add docs/manuals/evidence/r20b scripts/run_r20b_acceptance.py
git commit -m "test: add R2.0b fake-hardware acceptance evidence"
```

Skip `scripts/run_r20b_acceptance.py` in `git add` if the existing runner required no change.

- [ ] **Step 8: Push and stop for focused code review**

```bash
git push origin codex/r20b-shared-readiness-resolver
```

Before marking ready to merge, run the mandatory focused review for:
- duplicate source of truth;
- hidden fallback;
- world-space placement fallback;
- EXACT value mutation;
- scene-specific constants;
- `IK == readiness` shortcuts;
- fabricated check PASS states;
- recipe becoming authoritative;
- nondeterministic candidate ordering;
- accidental real-hardware path.

Fix all Critical/Important findings and rerun affected tests before moving draft PR #3169 to ready-for-review.

---

## Self-Review Results

- **Spec coverage:** planner extraction, strategy generation, policy semantics, R1.9 place authority, resolution artifacts, recipe derivation, safety, and workstation acceptance each map to explicit tasks above.
- **Placeholder scan:** no `TBD`/`TODO`/“implement later” steps remain; each code-producing task has concrete interfaces, tests, commands, and commit boundaries.
- **Type consistency:** `GraspCandidate`, `PreplanResult`, `resolve_task_intent()`, `write_resolution_artifacts()`, `resolve_local_destination()`, and `resolution_to_task_recipe()` are named once and consumed consistently by later tasks.
- **Scope:** Qt Task Authoring UI, Product View rendering, live EPD/RealSense, suction, operator HMI, and real hardware remain out of scope for R2.0b.
