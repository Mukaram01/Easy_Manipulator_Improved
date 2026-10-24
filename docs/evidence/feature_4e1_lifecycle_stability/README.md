# Feature 4E.1 — Workcell Builder shutdown crash: root cause and fix (2026-10-18)

**Status: CONFIRMED.** The intermittent heap corruption while Workcell Builder
closes is reproduced, traced to its first invalid write, fixed at the cause and
re-measured. Plan-only fake-hardware results from 4E are unchanged.

## Symptom

After a session ends (window closed, result already written), the process
sometimes aborted with `corrupted double-linked list`, `double free` or a
`Segmentation fault` while `MainWindow` deleted its children. Every core showed
glibc detecting the corruption at a different free (`QGraphicsView`,
`OwnedProductViewServer`'s `QByteArray`s, the WebEngine view's `QObject`,
frees on WebEngine threads), so the detection site said nothing about the
cause.

## Reproduction

`tools/repro.sh` runs the real Builder lifecycle (open `ur5_2f_test`, verify
the reopened Task, close) N times under Xvfb, keeps every core and exit code,
and writes a gdb backtrace per core. Environment as in
`docs/evidence/feature_4d_sequence_placement`: Docker
`moveit/moveit2:humble-release`, Mesa software GL.

| Build | Lifecycle runs | Crashes |
|---|---|---|
| #3189 head (4E) | 20 | **6** (exit 134/139) |
| #3188 head (4D) | 20 | **2**, same signature |
| #3189, Product View backend `native_scene3d` (no WebEngine view) | 20 | 0 |
| #3189, WebEngine with `--disable-gpu` | 20 | 5 |
| #3189, `MALLOC_CHECK_=3` | 10 | 2 (no earlier detection) |

The defect predates 4E. It needs the embedded WebEngine Product View and does
not depend on the GPU path. Full per-run signatures are in
`crash_summaries.txt`.

## Finding the first invalid write

- **valgrind memcheck** (one full run, threads serialised): no invalid write.
  Under valgrind the page never reaches the state that triggers the bug.
- **AddressSanitizer** (6 runs): clean. The writer turned out to be inside
  `libQt5WebEngineWidgets`, which is not instrumented.
- **`tools/qcheck.c`**: a small `LD_PRELOAD` allocator shim written for this
  investigation. It poisons freed blocks, keeps them in a 96 MB quarantine and
  reports the first block whose poison changed, with the call site that freed
  it. It keeps the real thread timing. Result: **16 of 16 runs** reported the
  same thing (`qcheck_report_run1.txt`):

  ```
  USE-AFTER-FREE WRITE: block size 24, written at offset 8 after free;
  bytes: d5d5d5d5d5d5d5d5 0000000000000000 d500d5d5d5d5d5d5
  freed from QWebEngineView::~QWebEngineView()+0x47
  ```

  The 24-byte block is `QWebEngineViewPrivate` (`q_ptr`, `page`,
  `m_dragEntered`, `m_ownsPage`). Offset 8 is `page`, offset 17 is
  `m_ownsPage`. So after the view destructor freed its private, something
  wrote `page = nullptr; m_ownsPage = false` into it. The corruption happens
  in every run; glibc only notices it in about a third of them.
- **gdb hardware watchpoint** (`tools/watch.gdb`) on the freed `page` field,
  trace in `watchpoint_trace.txt`:

  1. `QWebEngineView::~QWebEngineView` → `bind(view, nullptr)` sets
     `d->page = nullptr` and deletes the page it owns.
  2. Deleting the page delivers the pending `runJavaScript` callback of
     `EmbeddedWebCuratedAddController::pollEditorState()` (Qt invokes pending
     callbacks with an invalid value while the page is destroyed). The callback
     calls `view_->url()`. `QWebEngineView::url()` calls `page()`, and since
     `d->page` is already null, **it creates a new `QWebEnginePage`** as a
     child of the view that is being destroyed.
  3. `~QWebEngineView` frees its private (the 24-byte block).
  4. `~QWidget` deletes the new child page. `~QWebEnginePage` runs
     `bind(page, nullptr)`, which writes `page = nullptr` and
     `m_ownsPage = false` into the view's **freed** private.

  Whether a poll callback is pending at close is a matter of timing (the view
  polls every 400 ms), which is why the crash looked intermittent.

## Fix

`gui/embedded_web_curated_add_controller.hpp` and
`gui/embedded_web_edit_save_controller.hpp`: the editor-state poll callbacks
return before touching the view when Qt delivers them with an invalid value,
and the curated-add callback holds a `QPointer` guard like the edit-save one
already did. This is the contract Qt documents for `runJavaScript`: a callback
delivered during page destruction must not use the page or the view. No delay,
no change to shutdown order, no change to the acceptance harness.

## Validation with the fix

| Check | Result |
|---|---|
| `qcheck` quarantine, 10 lifecycle runs | **0** reports (16/16 before) |
| Plain lifecycle, 30 runs | **0** crashes, all exit 0, 0 harness failures |
| Real Save → Close → Reopen → Generate → Plan (scenarios A and B, author + reopen) | exit 0 in all 4 runs; no non-zombie ROS, MoveIt, checker, Xvfb or Builder process left afterwards |
| `workcell_task_intent_editor_test` | 22/22 |

Planning results in those runs: A author/reopen and B reopen
`SEQUENCE_PLANNED` at the saved places. B author ended `SEQUENCE_BLOCKED`:
cup-01's automatic place was the flush slot next to cup-02's chosen place, and
MoveIt rejected cup-02's place in that run (`PREPLAN_PLACE`, gripper against
cup-01). The same saved sequence planned in the reopen run. That is the
sampling-planner variance and the unmodelled gripper footprint already noted in
the 4D/4E evidence, not the lifecycle change.

## Pre-existing test-binary defect (not changed here)

`workcell_scene_preview_widget_ui_test` behaves identically with and without
the fix: 10 cases pass, the same 14 cases fail (layout and Product View
preparation expectations in this headless container), and the binary ends with
a segmentation fault in `QApplication::~QApplication` → QtWebEngine post
routine → `QOpenGLContext::destroy`. The test keeps a function-static
`QApplication` that is destroyed from `exit()` handlers after the test
framework returns, which is a different teardown path from the application's.
It is reported here, not fixed.

## Notes

- The container's PID 1 is `sleep`, so crashed `xvfb-run` sessions leave
  zombie `Xvfb` entries; the leftover-process check excludes zombies.
- `rr` could not be used here (unsupported CPU model for the Ubuntu package).
