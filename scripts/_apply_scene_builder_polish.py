from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
LAYOUT = ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp"
CSS = ROOT / "workcell_studio_web/viewer/style.css"
PRODUCT_TEST = ROOT / "tests/test_scene_builder_product_layout.py"
LIVE_TEST = ROOT / "tests/test_live_authoring_incremental_refresh.py"


def function_span(text: str, signature: str) -> tuple[int, int]:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return start, i + 1
    raise SystemExit(f"unterminated function: {signature}")


def patch_function(text: str, signature: str, transform) -> str:
    start, end = function_span(text, signature)
    original = text[start:end]
    patched = transform(original)
    if patched == original:
        raise SystemExit(f"{signature}: patch made no change")
    return text[:start] + patched + text[end:]


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


def remove_full_refresh(body: str, label: str) -> str:
    needle = "refresh_scene_builder_left_explorer();"
    count = body.count(needle)
    if count != 1:
        raise SystemExit(f"{label}: expected one full-refresh call, found {count}")
    return body.replace(needle, "refresh_minimap_card();", 1)


# --- MainWindow: keep live CRUD incremental and preserve minimap/inspector state. ---
source = MAIN.read_text(encoding="utf-8")


def patch_duplicate(body: str) -> str:
    full = "  refresh_scene_builder_left_explorer();\n"
    hierarchy = "  refresh_scene_hierarchy_tree_from_current_items();\n"
    selection = "  apply_scene_selection(new_id, copy.role, false, false);\n"
    anchor = "  mark_layout_dirty(\"Duplicate Selected\");\n"
    for token, label in ((full, "full refresh"), (hierarchy, "hierarchy refresh"), (selection, "selection"), (anchor, "dirty anchor")):
        if body.count(token) != 1:
            raise SystemExit(f"duplicate: expected one {label}, found {body.count(token)}")
    body = body.replace(full, "", 1).replace(hierarchy, "", 1).replace(selection, "", 1)
    replacement = (
        "  // The duplicate already exists in the live authored session and Product View.\n"
        "  // Do not rebuild from the saved scene: preserve unsaved state and selection.\n"
        + selection
        + hierarchy
        + "  refresh_minimap_card();\n"
        + anchor
    )
    return body.replace(anchor, replacement, 1)


def patch_delete(body: str) -> str:
    body = remove_full_refresh(body, "delete")
    duplicate_refresh = "  refresh_duplicate_selected_action();\n  refresh_duplicate_selected_action();\n"
    if body.count(duplicate_refresh) != 1:
        raise SystemExit("delete: expected duplicated action refresh exactly once")
    return body.replace(duplicate_refresh, "  refresh_duplicate_selected_action();\n", 1)


def patch_inspector_labels(body: str) -> str:
    body = replace_once(
        body,
        '    live_coordinate_label_->setText("No item selected");',
        '    live_coordinate_label_->clear();\n    live_coordinate_label_->setVisible(false);',
        "inspector empty transform label",
    )
    transform = '  live_coordinate_label_->setText(QString("Transform: %1").arg(pose));'
    body = replace_once(
        body,
        transform,
        '  live_coordinate_label_->setVisible(true);\n' + transform,
        "inspector live transform label",
    )
    return body


source = patch_function(source, "void MainWindow::duplicate_selected_item()", patch_duplicate)
source = patch_function(source, "void MainWindow::delete_selected_item()", patch_delete)
source = patch_function(source, "void MainWindow::undo_layout_edit()", lambda b: remove_full_refresh(b, "undo"))
source = patch_function(source, "void MainWindow::redo_layout_edit()", lambda b: remove_full_refresh(b, "redo"))
source = patch_function(source, "void MainWindow::refresh_selected_scene_item_labels(const SelectedSceneItemState & state)", patch_inspector_labels)
source = patch_function(
    source,
    "void MainWindow::update_minimap_backend_presentation()",
    lambda _body: '''void MainWindow::update_minimap_backend_presentation()
{
  if (!minimap_view_) return;
  minimap_view_->setMinimumHeight(90);
  minimap_view_->setMaximumHeight(90);
  minimap_view_->setVisible(minimap_requested_visible_);
}''',
)
source = replace_once(
    source,
    '  live_coordinate_label_ = new QLabel("Selected: none", scene_builder); selected_item_card_layout->addWidget(live_coordinate_label_);',
    '  live_coordinate_label_ = new QLabel("", scene_builder); live_coordinate_label_->hide(); selected_item_card_layout->addWidget(live_coordinate_label_);',
    "initial live transform label",
)
source = replace_once(
    source,
    'minimap_view_ = new QGraphicsView(scene_builder); minimap_view_->setObjectName("digital_twin_minimap"); minimap_view_->setFixedSize(210, 140);',
    'minimap_view_ = new QGraphicsView(scene_builder); minimap_view_->setObjectName("digital_twin_minimap"); minimap_view_->setFixedSize(150, 90);',
    "initial minimap size",
)
MAIN.write_text(source, encoding="utf-8")


# --- Product layout: remove overlap/overflow and keep useful context compact. ---
layout = LAYOUT.read_text(encoding="utf-8")
layout = replace_once(layout, "#include <QComboBox>\n", "#include <QComboBox>\n#include <QDoubleSpinBox>\n", "QDoubleSpinBox include")
layout = replace_once(layout, "#include <QPushButton>\n", "#include <QPushButton>\n#include <QScrollArea>\n", "QScrollArea include")
layout = replace_once(
    layout,
    '    if (text == QStringLiteral("Scene Builder") || text.startsWith(QStringLiteral("Scene Builder:"))) {',
    '    if (text == QStringLiteral("Scene Builder") || text.startsWith(QStringLiteral("Scene Builder:")) ||\n        text.contains(QStringLiteral(">Scene Builder"))) {',
    "HTML scene title suppression",
)


def patch_simplify_inspector(body: str) -> str:
    insertion = r'''

  if (selection_page) {
    for (auto * spin : selection_page->findChildren<QDoubleSpinBox *>()) {
      if (spin) spin->setMinimumWidth(76);
    }
    for (auto * scroll : selection_page->findChildren<QScrollArea *>()) {
      if (scroll) scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }
    for (auto * group : selection_page->findChildren<QGroupBox *>()) {
      if (!group) continue;
      const QString title = group->title().trimmed();
      if (title != QStringLiteral("Advanced details") && title != QStringLiteral("Robot Base Pose")) continue;
      group->setCheckable(true);
      group->setChecked(false);
      group->setMaximumHeight(34);
      const auto direct_children = group->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly);
      for (auto * child : direct_children) if (child) child->hide();
      QObject::connect(group, &QGroupBox::toggled, group, [group, direct_children](bool expanded) {
        group->setMaximumHeight(expanded ? QWIDGETSIZE_MAX : 34);
        for (auto * child : direct_children) if (child) child->setVisible(expanded);
      });
    }
  }
'''
    pos = body.rfind("\n}")
    if pos < 0:
        raise SystemExit("simplify_inspector: closing brace not found")
    return body[:pos] + insertion + body[pos:]


def patch_status(body: str) -> str:
    body = replace_once(
        body,
        '  if (auto * minimap = scene_page->findChild<QWidget *>(QStringLiteral("digital_twin_minimap"))) minimap->hide();',
        '  if (auto * minimap = scene_page->findChild<QWidget *>(QStringLiteral("digital_twin_minimap"))) {\n    minimap->setFixedSize(150, 90);\n    minimap->show();\n  }\n  if (auto * bottom = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderBottomStatusBar"))) bottom->hide();',
        "product minimap/status bar",
    )
    return body


def patch_configure(body: str) -> str:
    replacements = {
        "left->setMinimumWidth(245);": "left->setMinimumWidth(300);",
        "left->setMaximumWidth(300);": "left->setMaximumWidth(380);",
        "right->setMinimumWidth(300);": "right->setMinimumWidth(350);",
        "right->setMaximumWidth(360);": "right->setMaximumWidth(440);",
        "splitter->setSizes({270, 1120, 320});": "splitter->setSizes({325, 1040, 370});",
        'settings.setValue(QStringLiteral("scene_builder/preferred_left_width"), 270);': 'settings.setValue(QStringLiteral("scene_builder/preferred_left_width"), 325);',
        'settings.setValue(QStringLiteral("scene_builder/preferred_right_width"), 320);': 'settings.setValue(QStringLiteral("scene_builder/preferred_right_width"), 370);',
    }
    for old, new in replacements.items():
        body = replace_once(body, old, new, f"product layout token {old}")
    qss_anchor = '    QTabWidget#sceneBuilderInspectorTabs QTabBar::tab { padding:7px 10px; }\n'
    qss_extra = qss_anchor + '''    QGraphicsView#digital_twin_minimap { min-width:150px; max-width:150px; min-height:90px; max-height:90px; border:1px solid #D8E2EC; border-radius:6px; background:#F8FAFC; }\n    QGroupBox#sceneBuilderInspectorAdvancedDetails { margin-top:6px; padding:5px; }\n    QFrame#sceneBuilderRightPanel QDoubleSpinBox { min-width:76px; }\n'''
    body = replace_once(body, qss_anchor, qss_extra, "product compact QSS")
    return body


layout = patch_function(layout, "inline void simplify_inspector(QWidget * scene_page)", patch_simplify_inspector)
layout = patch_function(layout, "inline void simplify_status_area(QWidget * scene_page)", patch_status)
layout = patch_function(layout, "inline void configure_scene_builder_product_layout(MainWindow * window)", patch_configure)
LAYOUT.write_text(layout, encoding="utf-8")


# --- Embedded Web3D: retain diagnostics but make the ready card a compact status chip. ---
css = CSS.read_text(encoding="utf-8")
css_marker = "/* Workcell Studio embedded product diagnostics: compact, still visible. */"
if css_marker in css:
    raise SystemExit("compact diagnostics CSS already present unexpectedly")
css += r'''

/* Workcell Studio embedded product diagnostics: compact, still visible. */
body.embedded-mode .scene-health {
  top: 0.5rem;
  left: 0.5rem;
  width: min(19rem, calc(100% - 1rem));
  font-size: 0.72rem;
}
body.embedded-mode .scene-health summary {
  grid-template-columns: auto minmax(0, 1fr) auto auto;
  gap: 0.38rem;
  min-height: 2.25rem;
  padding: 0.3rem 0.45rem;
  border-radius: 0.48rem;
  box-shadow: 0 0.18rem 0.55rem rgba(23, 32, 42, 0.13);
}
body.embedded-mode .health-dot {
  width: 0.52rem;
  height: 0.52rem;
}
body.embedded-mode .health-copy strong { font-size: 0.75rem; }
body.embedded-mode .health-copy small { font-size: 0.64rem; }
body.embedded-mode .health-count,
body.embedded-mode .health-review { font-size: 0.64rem; }
body.embedded-mode .scene-health-body {
  max-height: min(18rem, calc(100vh - 4rem));
}
'''
CSS.write_text(css, encoding="utf-8")


# --- Focused source-level regression coverage for the interaction contract. ---
LIVE_TEST.write_text(r'''from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
LAYOUT = (ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp").read_text(encoding="utf-8")
CSS = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[start:i + 1]
    raise AssertionError(f"unterminated function: {signature}")


def test_live_crud_does_not_rebuild_scene_builder_from_saved_scene():
    signatures = [
        "void MainWindow::duplicate_selected_item()",
        "void MainWindow::delete_selected_item()",
        "void MainWindow::undo_layout_edit()",
        "void MainWindow::redo_layout_edit()",
    ]
    for signature in signatures:
        body = function_body(MAIN, signature)
        assert "refresh_scene_builder_left_explorer();" not in body
        assert "refresh_scene_hierarchy_tree_from_current_items();" in body
        assert "refresh_minimap_card();" in body


def test_duplicate_selects_new_owner_before_hierarchy_refresh():
    body = function_body(MAIN, "void MainWindow::duplicate_selected_item()")
    assert "duplicate_authoring_item(target.state.id, copy)" in body
    selection = "apply_scene_selection(new_id, copy.role, false, false);"
    hierarchy = "refresh_scene_hierarchy_tree_from_current_items();"
    assert selection in body and hierarchy in body
    assert body.index(selection) < body.index(hierarchy)


def test_delete_refreshes_duplicate_action_once():
    body = function_body(MAIN, "void MainWindow::delete_selected_item()")
    assert "remove_authoring_item(id)" in body
    assert body.count("refresh_duplicate_selected_action();") == 1


def test_minimap_survives_embedded_web3d_ready_state():
    body = function_body(MAIN, "void MainWindow::update_minimap_backend_presentation()")
    assert "embedded_web3d_presented" not in body
    assert "setVisible(false)" not in body
    assert "setMinimumHeight(90)" in body
    assert "setMaximumHeight(90)" in body
    assert "setVisible(minimap_requested_visible_)" in body
    assert 'minimap->setFixedSize(150, 90)' in LAYOUT
    assert 'minimap->show()' in LAYOUT


def test_inspector_has_one_empty_selection_message_and_no_horizontal_overflow():
    body = function_body(MAIN, "void MainWindow::refresh_selected_scene_item_labels(const SelectedSceneItemState & state)")
    assert 'inspector_label_->setText("No item selected")' in body
    assert 'live_coordinate_label_->setText("No item selected")' not in body
    assert "live_coordinate_label_->clear();" in body
    assert "live_coordinate_label_->setVisible(false);" in body
    assert "live_coordinate_label_->setVisible(true);" in body
    assert "setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)" in LAYOUT
    assert "spin->setMinimumWidth(76)" in LAYOUT


def test_product_layout_compacts_secondary_inspector_and_status_chrome():
    assert 'text.contains(QStringLiteral(">Scene Builder"))' in LAYOUT
    assert 'QStringLiteral("Advanced details")' in LAYOUT
    assert 'QStringLiteral("Robot Base Pose")' in LAYOUT
    assert "group->setMaximumHeight(34)" in LAYOUT
    assert 'QStringLiteral("sceneBuilderBottomStatusBar"))) bottom->hide()' in LAYOUT
    assert "splitter->setSizes({325, 1040, 370})" in LAYOUT


def test_embedded_scene_health_remains_visible_but_compact():
    assert "body.embedded-mode .scene-health" in CSS
    assert "width: min(19rem, calc(100% - 1rem));" in CSS
    assert "body.embedded-mode #scene-health" not in CSS


def test_explicit_full_scene_refresh_is_still_available():
    body = function_body(MAIN, "void MainWindow::refresh_scene_builder_left_explorer()")
    assert "sync_selected_scene_state();" in body
    assert "rebuild_digital_twin_canvas();" in body
    assert "populate_scene_hierarchy();" in body
    assert "populate_asset_catalog();" in body
''', encoding="utf-8")


# Keep the existing product-layout expectations aligned with the wider, usable side panels.
product_test = PRODUCT_TEST.read_text(encoding="utf-8")
for old, new in {
    "'left->setMinimumWidth(245)'": "'left->setMinimumWidth(300)'",
    "'left->setMaximumWidth(300)'": "'left->setMaximumWidth(380)'",
    "'right->setMinimumWidth(300)'": "'right->setMinimumWidth(350)'",
    "'right->setMaximumWidth(360)'": "'right->setMaximumWidth(440)'",
    "'splitter->setSizes({270, 1120, 320})'": "'splitter->setSizes({325, 1040, 370})'",
}.items():
    product_test = replace_once(product_test, old, new, f"product test token {old}")
PRODUCT_TEST.write_text(product_test, encoding="utf-8")
