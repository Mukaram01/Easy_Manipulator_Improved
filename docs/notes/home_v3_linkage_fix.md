# Home v3 linkage fix

The Home v3 composition depends on `ScenePreviewWidget` runtime and Qt meta-object symbols. It must therefore be compiled only into the full `workcell_builder` executable, which already links `scene_preview_widget.cpp` and its automoc output.

`workcell_builder_ui_utils.hpp` is shared by helper/test targets such as `workcell_generated_environment_asset_writer_test`; those targets intentionally do not link the full ScenePreviewWidget implementation. Including Home v3 from that shared header caused undefined references at link time.

The bootstrap now lives in `gui/startup_dialog.cpp`, a production executable translation unit, while shared UI utilities remain independent of ScenePreviewWidget. This preserves the Home v3 UI without making helper/test binaries link the full 3D runtime.
