# Generic single-cycle pile extraction implementation plan

**Goal:** Resolve any permitted fresh object/grasp only after a complete feasible extraction, placement and home cycle, then prove one physical cycle with the existing Stage A runner.

**Architecture:** Extend the existing complete-cycle evaluator at its private post-close attachment checkpoint. A compact geometry helper proposes at most four deterministic relative extraction intents; the existing MoveIt Cartesian planner and full-cycle suffix remain authoritative. Bind the successful intent and proven IK solution through the existing resolution hash; revalidation plans fresh trajectories.

**Spec:** User course correction in `/home/ubuntu/.codex/attachments/8c9b7b21-811d-4065-ac02-8a89a0a11eae/Pasted text.txt`.

**Constraints:** Same branch and draft PR #3174. No repeated picking loop, trajectory cache, additional planner, timeout increase, hardware authorization, or contact/freshness/slip/corridor relaxation. Keep all prior evidence. The abandoned test-only experiment is preserved in `/home/ubuntu/workcell_ws/stage-a-direction-change-20260923/bounded-lateral-experiment.patch`.

- [x] Geometry helper and focused real-FCL regressions: vertical first, then bounded directions derived from nearby BOX geometry; exact initial contacts retain the existing 0.1 mm depth bound, near-only pairs receive no contact allowance, all relevant pairs clear 0.1 mm by 10 mm rise, expired allowances never return.
- [x] Existing preplanner/resolver: share current candidate deadlines across extraction variants; reset only private state at the post-close checkpoint; require the full suffix; preserve variant failures; keep EXACT/PREFERRED/AUTO semantics and bind selected intent.
- [x] Existing runtime boundary: audit ideal extraction and actual planned object poses against current private geometry; retain full-arm MoveIt checks; propagate intent through resolve/retry/revalidation and physical evidence.
- [x] Transfer consistency: reject fresh IK branch substitution using the existing approach joint tolerance; require the saved transfer identity; keep fresh private-scene planning and collision checks.
- [ ] Run affected suites, independent review, commit/push coherent changes, and run the unchanged six-gate runner in a fresh root. Continue from any first scoped failure.

**Review focus:** near-only contacts never gain permission; an alternative extraction cannot inherit another variant's transfer seed; rejected variant FAIL checks cannot contaminate a successful variant; changed/missing resolved intent cannot silently substitute; budget exhaustion remains distinct from geometric impossibility.
