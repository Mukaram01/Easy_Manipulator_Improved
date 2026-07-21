from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
QT_UTILS = ROOT / "workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp"
WEB_CSS = ROOT / "workcell_studio_web/viewer/style.css"


def test_qt_windows_are_screen_bounded_and_controls_shrink_inside_layouts():
    text = QT_UTILS.read_text(encoding="utf-8")

    for token in [
        "class ResponsiveWindowGuard",
        "availableGeometry()",
        "frameGeometry()",
        "QEvent::Show",
        "QEvent::Resize",
        "bounded_window_size",
        "setMinimumWidth(0)",
        "QSizePolicy::Expanding",
    ]:
        assert token in text


def test_qt_side_content_avoids_nested_horizontal_scrolling():
    text = QT_UTILS.read_text(encoding="utf-8")

    for token in [
        "setWidgetResizable(true)",
        "setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)",
        "QAbstractScrollArea::AdjustIgnored",
        "setTextElideMode(Qt::ElideRight)",
        "setUsesScrollButtons(true)",
        "setExpanding(false)",
        "QTextEdit::WidgetWidth",
        "QPlainTextEdit::WidgetWidth",
    ]:
        assert token in text


def test_web_viewer_owns_the_viewport_without_page_level_scrollbars():
    css = WEB_CSS.read_text(encoding="utf-8")

    for token in [
        "grid-template-rows: auto minmax(0, 1fr)",
        "overflow: hidden",
        "overflow-x: hidden",
        "overflow-y: auto",
        "scrollbar-gutter: stable",
        "overscroll-behavior: contain",
        "grid-template-columns: clamp(12rem, 18vw, 18rem) minmax(0, 1fr) clamp(15rem, 22vw, 22rem)",
    ]:
        assert token in css

    assert "height: calc(100vh - 74px)" not in css
    assert "min-height: 34rem" not in css


def test_web_viewer_narrow_layout_keeps_both_side_panels_bounded():
    css = WEB_CSS.read_text(encoding="utf-8")

    assert "@media (max-width: 820px)" in css
    assert "grid-template-columns: minmax(11rem, 15rem) minmax(0, 1fr)" in css
    assert "grid-column: 1 / -1" in css
    assert "max-height: none" in css
    assert "body.embedded-mode" in css
    assert "height: 100%" in css
