# Workcell Studio Roadmap

## Purpose

Workcell Studio is the internal configurable robotic-cell platform for defining, generating, validating, and operating deployment-ready workcells. It is the product platform direction for this repository.

Sorting is only one scenario template in Workcell Studio, alongside other templates such as inspection routing, reject handling, and palletizing.

## Near-term visual target

The near-term operator experience target is:

- RViz for robot/cell visualization and planning introspection.
- A simple web/Streamlit Studio layer for task and cell configuration, validation status, and artifact review.

This keeps the baseline practical for ROS 2 Humble teams while preserving fake-hardware-first and offline/dry-run validation workflows.

## Recommended simulation sequence

1. **RViz + MoveIt now**
   - Baseline for ROS 2 Humble commissioning and motion-planning workflows.
   - Keep fake hardware and headless validation paths as defaults.
2. **Gazebo Sim later**
   - Add conveyors and basic physics simulation when needed.
   - Optional extension, not a baseline requirement.
3. **Isaac Sim later**
   - Use for investor-grade visuals and advanced simulation demos.
   - Optional extension, not mandatory for functional deployment.

## Repository boundaries

High-level ownership boundaries:

- **EPD (Easy Perception Deployment)** owns perception pipelines and detected-object outputs.
- **Easy_Manipulator_Improved** owns Workcell Studio orchestration: generation, validation, and execution integration.
- **Generated scene packages** own launch/config output artifacts for specific workcell instances.

## Scope guardrails

- Preserve ROS 2 Humble as the supported baseline.
- Do not require Gazebo Sim or Isaac Sim for core operation.
- Preserve fake-hardware-first defaults and dry-run/offline validation language.
- Keep runtime launch behavior backward compatible.


## Architecture guardrail

- Workcell Studio does not replace `workcell_builder`; it augments it.
- `workcell_builder` remains the primary visual scene builder/editor.
- Capability/grasp catalogs and validators are backend services used by builder generation and QA tooling.
- CLI/wizard scripts are for automation/testing and are not a competing UI path.
- Any future Streamlit layer is dashboard/orchestration only.


## Builder scene exports for Workcell Studio

`workcell_builder` remains the primary visual workflow. Generated scenes can now export portable Workcell Studio source files using `scripts/export_builder_scene_to_cell_definition.py`. The export writes `generated/cell_definition.yaml`, `generated/environment_layout.yaml`, `generated/workcell_builder_task_intent.yaml` (when enough task authoring metadata exists), and `generated/builder_export_summary.json`. These files are for offline commissioning and backend tooling, and are not proof of reachability or runtime safety. Keep fake-hardware-first defaults and runtime send disabled unless separately commissioned.

## Curated Demo Catalog

`catalog/workcell_studio_demos.yaml` defines investor/customer-friendly offline demo templates. Generated demo bundles are strictly offline/demo/validation artifacts and do not alter runtime robot behavior or launch behavior. Preview-only demos support sales/concept visualization and must not be treated as real runtime support.

## Create-cell wizard

`workcell_builder` remains the visual scene editor. `workcell_studio.py create-cell` is a fast catalog-driven YAML creation path that generates cell definition/layout, validates, produces static previews, and can optionally emit offline bundle artifacts.


Next safe bridge: guarded RViz/MoveIt plan-preview session preparation from offline plan-preview requests, without ROS launch or motion execution.

- builder scene -> task intent -> task recipe -> offline plan request -> RViz preview session -> smoke launch -> planning scene readiness


Readiness pipeline: builder scene -> task intent -> task recipe -> task flow -> static preview -> offline plan request -> RViz preview session -> smoke launch -> planning scene readiness -> readiness pack.


## Readiness classification shorthand

- `physical_scene_only`: cell/layout metadata exists but task flow authoring is missing or incomplete.
- `task_intent_present` / `task_preview_ready`: builder task intent exists and validates for offline preview-oriented task flow checks.
- `runtime_ready`: only when downstream runtime safety gates, launch checks, and existing commissioning requirements pass; task intent alone does not imply runtime execution readiness.
