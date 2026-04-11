# Improvement Areas Audit (April 2026)

This audit highlights repository areas that likely need focused attention based on roadmap/document consistency checks and TODO/FIXME hotspots.

## Top priority areas

### 1) Resolve roadmap and status drift

**Why this needs attention**
- `docs/ROADMAP.md` lists **Demo node lifecycle conversion** and **Default executor watchdog and graceful shutdown** as pending, but also lists both as completed.
- This creates uncertainty for contributors choosing what to work on next.

**Suggested improvement**
- Split roadmap into three explicit sections: `Done`, `In progress`, `Planned`.
- Remove duplicated items from pending.
- Add target package/path next to each roadmap item.

## 2) Unify setup instructions across docs

**Why this needs attention**
- `README.md` uses `dependencies/emd_epd_ws.repos` for `vcs import`.
- `CONTRIBUTING.md` still points to `tesseract.repos`.
- This can cause contributors to import different dependency graphs and see inconsistent build behavior.

**Suggested improvement**
- Pick one default dependency manifest for the mainline workflow.
- Keep the other manifest explicitly documented as optional/legacy/advanced with when-to-use guidance.
- Add a short compatibility matrix (Humble required, Jazzy experimental) beside the import command.

### 3) Reduce unresolved TODO backlog in runtime-critical packages

**Why this needs attention**
- TODO/FIXME scan indicates the highest concentration in:
  - `emd_dynamic_safety` (12 hits)
  - `emd_grasp_execution` (4 hits)
  - `emd_demo_nodes` (3 hits)
- Several TODOs reference runtime behavior: collision mapping, termination handling, replanner state/speed, and workflow DAG prerequisites.

**Suggested improvement**
- Convert TODO comments into tracked issues with labels (`dynamic-safety`, `execution`, `demo`).
- Prioritize TODOs that affect safety and deterministic execution first.
- Add an acceptance checklist for each TODO category (unit test, integration test, docs update).

## 4) Improve reproducibility in constrained environments

**Why this needs attention**
- `BUILD_ATTEMPT_LOG.md` records recurring environment failures (`colcon`/ROS distro not present, rosdep path missing), which blocks verification.

**Suggested improvement**
- Add a lightweight container/devcontainer profile that includes minimum verification tools.
- Add a `preflight` script that fails fast with actionable errors (ROS distro, colcon, rosdep path checks).
- Publish a "minimal validation commands" section that can run even when full ROS builds are unavailable.

## Medium-priority areas

### 5) Expand CI coverage beyond single distro and happy path

**Current state**
- CI currently validates Humble only via `.github/workflows/humble-ci.yml`.

**Suggested improvement**
- Add a non-blocking matrix job for Jazzy (experimental) to surface regressions early.
- Add targeted jobs for script validation + selective package build to reduce diagnosis time.

### 6) Add ownership and triage process for old TODOs

**Why this matters**
- TODOs contain mixed assignees (`anyone`, named handles, unowned comments), making prioritization unclear.

**Suggested improvement**
- Adopt comment convention: `TODO(issue #, owner, target release): ...`.
- Periodically fail CI if TODO count increases in core runtime packages without linked issue IDs.

## Quick wins (1–2 PRs)

1. Clean `docs/ROADMAP.md` pending/completed overlap and add package mapping.
2. Align README and CONTRIBUTING dependency-import instructions.
3. Open issue batch for high-impact TODOs in `emd_dynamic_safety` and `emd_grasp_execution`.
4. Add preflight checks script (ROS distro, colcon, rosdep config, workspace layout).
