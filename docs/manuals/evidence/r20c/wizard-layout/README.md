# New Cell wizard screen-height fix — HOME PC, 2026-09-16

## Root cause and production fix

The main QStackedWidget inherited the minimum size of its tallest page. With
Pick & Place / Manual simulated object selected, a requested 1000×600 dialog
became 1384 pixels tall. Navigation belonged to that oversized dialog and went
off-screen. An unwrapped help label also forced excess page width.

All six existing pages now use a resizable QScrollArea. Header, sidebar and
navigation stay outside the scrolling content. Page labels and form rows wrap;
no font/control scaling or task functionality was removed. The dialog caps its
size against QScreen::availableGeometry in logical pixels, reserves decoration
height, and responds to screen/work-area changes. Preferred initial size stays
1040×720; minimum width is 800, capped by available width.

## Verification

- Humble workcell_builder and affected Qt targets: PASS.
- Existing focused tests: 76 Python + 15 editor + 9 model + 15 wizard = 115 PASS.
- New wizard layout regression: PASS (116 total), first reproduced 1384 > 600.
- All six pages checked before/after content scrolling; five actions stay
  visible/in bounds. Task Intent has vertical overflow and no horizontal overflow.
- Real X display at default scaling, QT_SCALE_FACTOR=1.5 and =2: PASS.
  Screenshots show the bottom of Task Intent with the fixed action bar.
- Actual rebuilt Studio launched through Setup → Open Studio → Home New Cell.
  AT-SPI drove every wizard sidebar page. All five actions were showing and
  inside the dialog; Create and Open was enabled on Review with a valid name.
  The probe was canceled without creating or modifying a cell. See studio-pages.json.
- Canonical editor Save/destroy/reopen acceptance: PASS.
- Existing production Web3D handoff smoke: two matching-identity cycles PASS
  (`--scene3d-smoke --web3d-scene-sequence ur5_2f_test,ur5_2f_test`).

Commands from the repository root, after sourcing /opt/ros/humble/setup.bash
and /home/user/workcell_ws/install/setup.bash:

```bash
cmake --build /home/user/workcell_ws/build/workcell_builder --target workcell_builder workcell_new_cell_wizard_test workcell_task_intent_editor_test workcell_task_intent_model_test -j2
python3 -m pytest -q tests/test_task_intent_authoring.py tests/test_task_intent_v2.py tests/test_builder_task_intent.py tests/test_physical_destination.py tests/test_task_intent_resolver.py
/home/user/workcell_ws/build/workcell_builder/workcell_task_intent_editor_test
/home/user/workcell_ws/build/workcell_builder/workcell_task_intent_model_test
/home/user/workcell_ws/build/workcell_builder/workcell_new_cell_wizard_test
QT_QPA_PLATFORM=xcb WORKCELL_WIZARD_LAYOUT_EVIDENCE=/tmp/r20c-wizard-layout/display /home/user/workcell_ws/build/workcell_builder/workcell_new_cell_wizard_test --gtest_filter=NewCellWizard.PagesScrollWithoutDisplacingNavigation
QT_QPA_PLATFORM=xcb QT_SCALE_FACTOR=1.5 WORKCELL_WIZARD_LAYOUT_EVIDENCE=/tmp/r20c-wizard-layout/scaled /home/user/workcell_ws/build/workcell_builder/workcell_new_cell_wizard_test --gtest_filter=NewCellWizard.PagesScrollWithoutDisplacingNavigation
QT_QPA_PLATFORM=xcb QT_SCALE_FACTOR=2 WORKCELL_WIZARD_LAYOUT_EVIDENCE=/tmp/r20c-wizard-layout/scaled2 /home/user/workcell_ws/build/workcell_builder/workcell_new_cell_wizard_test --gtest_filter=NewCellWizard.PagesScrollWithoutDisplacingNavigation
```

Qt platform/shutdown warnings remain in logs; no warnings are hidden. Physical
monitor hot-plug/migration was not exercised. R2.0c full fresh-cell acceptance
must restart after this layout fix; PR #3170 remains draft. No robot motion,
EPD, native viewport changes, or R2.0d/e work.
