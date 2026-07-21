from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
QT = (ROOT / "workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp").read_text(encoding="utf-8")
CSS = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")


def test_qt_windows_and_side_content_remain_contained():
    for token in (
        "class ResponsiveWindowGuard", "availableGeometry()", "frameGeometry()",
        "QEvent::Show", "QEvent::Resize", "bounded_window_size",
        "setWidgetResizable(true)", "setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)",
        "QAbstractScrollArea::AdjustIgnored", "setTextElideMode(Qt::ElideRight)",
        "setUsesScrollButtons(true)", "setExpanding(false)",
        "QTextEdit::WidgetWidth", "QPlainTextEdit::WidgetWidth",
        "setMinimumWidth(0)", "QSizePolicy::Expanding",
    ):
        assert token in QT


def test_web_viewer_uses_bounded_internal_scrolling_only():
    for token in (
        "grid-template-rows: auto minmax(0, 1fr)", "overflow: hidden",
        "overflow-x: hidden", "overflow-y: auto", "scrollbar-gutter: stable",
        "overscroll-behavior: contain",
        "grid-template-columns: clamp(12rem, 18vw, 18rem) minmax(0, 1fr) clamp(15rem, 22vw, 22rem)",
        "@media (max-width: 820px)",
        "grid-template-columns: minmax(11rem, 15rem) minmax(0, 1fr)",
        "grid-column: 1 / -1", "max-height: none", "body.embedded-mode",
    ):
        assert token in CSS
    assert "height: calc(100vh - 74px)" not in CSS
    assert "min-height: 34rem" not in CSS
