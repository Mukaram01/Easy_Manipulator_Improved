from pathlib import Path

UI_HEADER = Path(
    "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"
).read_text(encoding="utf-8")
STARTUP_DIALOG = Path(
    "workcell_builder/workcell_builder/gui/startup_dialog.cpp"
).read_text(encoding="utf-8")


def test_home_v3_bootstrap_is_not_in_shared_ui_utils_header():
    # workcell_builder_ui_utils.cpp is compiled into small helper/test targets that
    # do not link ScenePreviewWidget or its moc object. Keeping Home v3 out of the
    # shared header prevents those targets from acquiring ScenePreviewWidget
    # runtime symbol dependencies.
    assert '#include "workcell_home_polish_v3.hpp"' not in UI_HEADER
    assert '#include "workcell_home_polish_v3.hpp"' in STARTUP_DIALOG


def test_linkage_isolation_comment_documents_boundary():
    assert "Keep shared UI utilities link-safe" in UI_HEADER
    assert "full workcell_builder executable" in UI_HEADER
