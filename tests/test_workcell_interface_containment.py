from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
QT = (ROOT / "workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp").read_text()
CSS = (ROOT / "workcell_studio_web/viewer/style.css").read_text()


def test_qt_interface_is_screen_bounded_without_horizontal_side_scrolling():
    for token in ("ResponsiveWindowGuard", "availableGeometry()", "frameGeometry()",
                  "bounded_window_size", "setWidgetResizable(true)",
                  "setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)",
                  "QAbstractScrollArea::AdjustIgnored", "setTextElideMode(Qt::ElideRight)",
                  "setUsesScrollButtons(true)", "QTextEdit::WidgetWidth"):
        assert token in QT


def test_web_viewer_contains_all_scrolling_inside_bounded_panels():
    for token in ("grid-template-rows: auto minmax(0, 1fr)", "overflow-x: hidden",
                  "overflow-y: auto", "scrollbar-gutter: stable", "overscroll-behavior: contain",
                  "grid-template-columns: clamp(12rem, 18vw, 18rem) minmax(0, 1fr) clamp(15rem, 22vw, 22rem)",
                  "@media (max-width: 820px)", "grid-column: 1 / -1", "body.embedded-mode"):
        assert token in CSS
    assert "height: calc(100vh - 74px)" not in CSS
    assert "min-height: 34rem" not in CSS
