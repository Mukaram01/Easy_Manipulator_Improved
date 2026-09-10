# R1.1 canonical scene readiness

Scope: `ur5_2f_test`. This change repairs readiness and panel ownership; it does not certify a completed pick/place cycle or enable real hardware.

## Readiness contract

`scene_content_readiness()` evaluates disk content using the existing authored-input fingerprint. `MainWindow::selected_scene_readiness()` combines that result with unsaved edits, canonical launch prerequisites, and transform/parity blockers. The workflow stages, enabled Run Next action, selected-scene details, and Plan / Simulate page consume that result. Run Next executes the first enabled recommendation directly, and Checks displays simulation blockers in the panel itself.

A current, safe, passing `acceptance/generated_scene_acceptance.json` proves an already accepted package current without any ignored merge report. Successful package generation writes `acceptance/generation_fingerprint.json`; that durable receipt permits Validate after generation, but does not itself permit simulation. Commit the generation receipt with generated handoff changes when acceptance has not yet been refreshed. Missing receipts require Generate; missing/currently failing acceptance requires Validate. A report without a fingerprint requires fresh generation/validation rather than an mtime fallback.

Both fingerprints cover authored layout/configuration and the committed runtime handoff. Real content changes invalidate downstream readiness. Touching a file, reopening, or checking out identical bytes does not. Generated metadata, Python bytecode caches, and ignored merge reports are not authored inputs. A failed Validate cannot bless changed content merely by writing a newer report.

Existing fake-hardware launch validation, command safety checks, transform/parity blockers, collision handling, and real-hardware locks remain in force. Failed post-generation parity remains a blocker for its selected scene.

## Panels and HTTP diagnostics

- Inspector owns selected-item name, role, transform, editability and concise warnings. Primitive dimensions remain editable only under the existing geometry policy; authoritative mesh dimensions are not editable. Provenance remains under collapsed Advanced.
- Task owns target class, source/zone, grasp, destination, release/retreat, perception and confidence/age policy. A null confidence threshold explains the current localization adapter's missing score. File/debug actions are under Advanced task diagnostics.
- Checks owns the workflow, visible simulation blockers and one enabled next action. Command/report details are under Advanced diagnostics.

The owned loopback server logs the requested path and status. Only `/favicon.ico` and `/.well-known/appspecific/com.chrome.devtools.json` are classified as optional browser resources. Their 404s preserve the current scene activity. Unknown paths, scene payloads and meshes remain required-resource errors. Server lifecycle/ownership is unchanged.

## Evidence and limits (2026-09-10)

Focused pytest coverage includes the production Qt MainWindow linked from the colcon build, the real generator, validator and HTTP process. It checks cache absence, same-content touch, authored changes, Generate then Validate, enabled corrective actions, the Run Next click into Plan / Simulate, blocked Build & Run, panel ancestry and optional/required HTTP failures. The valid-scene fixture uses canonical committed authored files with a current acceptance fingerprint; this is regression evidence, not workstation acceptance.

Results: **31 focused pytest tests passed**, **4 Qt/gtest targets passed**, and the requested **2-package colcon build passed**.

Commands:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/test_r11_canonical_readiness.py tests/test_product_view_dynamic_server_port.py tests/test_validate_builder_generated_scene.py tests/test_workflow_rail_compact_rendering.py tests/test_workcell_studio_layout_stale_readiness.py tests/test_workcell_studio_scene_browser.py
cd ~/workcell_ws
colcon build --symlink-install --packages-select workcell_builder ur5_2f_test --allow-overriding ur5_2f_test
cd build/workcell_builder
QT_QPA_PLATFORM=offscreen ctest --output-on-failure -R '(rviz_preview|planning_readiness|task_intent_readiness|layout.*dimension)'
```

The Qt probe requires the colcon build objects; set `WORKCELL_BUILD_DIR` for a nonstandard build directory. It disables C++ access checks only for the test translation unit, not product code.

The display-enabled Product View smoke opened `ur5_2f_test`, loaded meshes and logged the optional favicon path. Its legacy native-Scene3D counter checks failed on the embedded web view. Evidence during this run: `/tmp/r11-gui-smoke.json`, `/tmp/r11-gui.log`, `/tmp/r11-gui.png`. This is partial GUI evidence, not a manual RViz launch/stop acceptance.

Main's historical committed canonical acceptance fingerprint predates later actual scene content changes. It must not be silently treated as current. Existing local authored layout/acceptance edits and runtime output were preserved and excluded from this change. Generate and Validate the intended authored scene before the remaining workstation check: open Checks, click Plan / Simulate, Build & Run RViz with fake hardware, stop/close RViz, then confirm Studio remains editable.

Ten unrelated legacy failures reproduce against unchanged main: six generation fixtures have invalid `zones.bounds_xyz` data, and four UI token tests describe removed labels/messages. They are not evidence of this patch's behavior.

Rollback: revert the product/test/documentation commit. No scene YAML, collision configuration, real-hardware setting, EPD or simulator backend is changed by this patch.
