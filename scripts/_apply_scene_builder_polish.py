from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
LAYOUT = ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp"
CSS = ROOT / "workcell_studio_web/viewer/style.css"
PRODUCT_TEST = ROOT / "tests/test_scene_builder_product_layout.py"
LIVE_TEST = ROOT / "tests/test_live_authoring_incremental_refresh.py"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


# Ordinary CRUD must mutate the live authored session only. Rebuilding the
# explorer reloads the saved scene and destroys unsaved state/camera/selection.
s = MAIN.read_text(encoding="utf-8")
s = replace_once(
    s,
    '''  refresh_scene_hierarchy_tree_from_current_items();
  refresh_scene_builder_left_explorer();
  apply_scene_selection(new_id, copy.role, false, false);''',
    '''  // The duplicate already exists in the live authored session and Product View.
  // Preserve unsaved scene, camera and selection state instead of rebuilding from disk.
  apply_scene_selection(new_id, copy.role, false, false);
  refresh_scene_hierarchy_tree_from_current_items();''',
    "duplicate incremental refresh",
)
s = replace_once(
    s,
    '''  refresh_scene_hierarchy_tree_from_current_items();
  refresh_selected_scene_item_labels(selected_item_state_);
  refresh_scene_builder_left_explorer();
  mark_layout_dirty("Delete Selected");''',
    '''  refresh_scene_hierarchy_tree_from_current_items();
  refresh_selected_scene_item_labels(selected_item_state_);
  mark_layout_dirty("Delete Selected");''',
    "delete incremental refresh",
)
s = replace_once(
    s,
    '''redo_stack_.push_back(c); mark_layout_dirty("Undo"); refresh_scene_hierarchy_tree_from_current_items(); refresh_scene_builder_left_explorer(); refresh_delete_selected_action(); refresh_duplicate_selected_action();''',
    '''redo_stack_.push_back(c); mark_layout_dirty("Undo"); refresh_scene_hierarchy_tree_from_current_items(); refresh_delete_selected_action(); refresh_duplicate_selected_action();''',
    "undo incremental refresh",
)
s = replace_once(
    s,
    '''undo_stack_.push_back(c); mark_layout_dirty("Redo"); refresh_scene_hierarchy_tree_from_current_items(); refresh_scene_builder_left_explorer(); refresh_delete_selected_action(); refresh_duplicate_selected_action();''',
    '''undo_stack_.push_back(c); mark_layout_dirty("Redo"); refresh_scene_hierarchy_tree_from_current_items(); refresh_delete_selected_action(); refresh_duplicate_selected_action();''',
    "redo incremental refresh",
)

# Product decision: the 3D Product View is the sole spatial workspace. Remove
# the legacy 2D minimap completely from presentation.
old_minimap = '''void MainWindow::update_minimap_backend_presentation()
{
  if (!minimap_view_ || !scene_preview_widget_) return;
  const bool embedded_web3d_presented =
    scene_preview_widget_->active_product_view_backend() ==
      ScenePreviewWidget::ProductViewBackend::EmbeddedWeb3D &&
    scene_preview_widget_->embedded_web_authoring_active();
  if (embedded_web3d_presented) {
    minimap_view_->setVisible(false);
    minimap_view_->setMinimumHeight(0);
    minimap_view_->setMaximumHeight(0);
    return;
  }

  minimap_view_->setMinimumHeight(140);
  minimap_view_->setMaximumHeight(140);
  minimap_view_->setVisible(minimap_requested_visible_);
}'''
new_minimap = '''void MainWindow::update_minimap_backend_presentation()
{
  if (!minimap_view_) return;
  // Scene Builder uses the 3D Product View as the sole spatial workspace.
  minimap_view_->hide();
  minimap_view_->setMinimumSize(0, 0);
  minimap_view_->setMaximumSize(0, 0);
}'''
s = replace_once(s, old_minimap, new_minimap, "remove minimap presentation")

# Exactly one Inspector empty-state line.
s = replace_once(
    s,
    '''    live_coordinate_label_->setText("No item selected");''',
    '''    live_coordinate_label_->clear();
    live_coordinate_label_->setVisible(false);''',
    "single inspector empty state",
)
s = replace_once(
    s,
    '''  live_coordinate_label_->setText(QString("Transform: %1").arg(pose));''',
    '''  live_coordinate_label_->setVisible(true);
  live_coordinate_label_->setText(QString("Transform: %1").arg(pose));''',
    "restore transform label",
)
MAIN.write_text(s, encoding="utf-8")

# Product layout: readable hierarchy/Inspector, no duplicate title, no minimap,
# no horizontal Inspector overflow, compact secondary sections.
s = LAYOUT.read_text(encoding="utf-8")
s = replace_once(
    s,
    "#include <QPushButton>\n#include <QSettings>",
    "#include <QPushButton>\n#include <QScrollArea>\n#include <QDoubleSpinBox>\n#include <QSettings>",
    "layout includes",
)
s = replace_once(
    s,
    '''    if (text == QStringLiteral("Scene Builder") || text.startsWith(QStringLiteral("Scene Builder:"))) {
      label->hide();
    }''',
    '''    if (text == QStringLiteral("Scene Builder") ||
        text.startsWith(QStringLiteral("Scene Builder:")) ||
        text.contains(QStringLiteral(">Scene Builder"))) {
      label->hide();
    }''',
    "hide duplicate scene title",
)
for old, new, label in (
    ("left->setMinimumWidth(245);", "left->setMinimumWidth(300);", "hierarchy min width"),
    ("left->setMaximumWidth(300);", "left->setMaximumWidth(380);", "hierarchy max width"),
    ("right->setMinimumWidth(300);", "right->setMinimumWidth(350);", "inspector min width"),
    ("right->setMaximumWidth(360);", "right->setMaximumWidth(440);", "inspector max width"),
    ("splitter->setSizes({270, 1120, 320});", "splitter->setSizes({325, 1040, 370});", "splitter sizes"),
):
    s = replace_once(s, old, new, label)
s = replace_once(
    s,
    '''  settings.setValue(QStringLiteral("scene_builder/preferred_left_width"), 270);
  settings.setValue(QStringLiteral("scene_builder/preferred_right_width"), 320);''',
    '''  settings.setValue(QStringLiteral("scene_builder/preferred_left_width"), 325);
  settings.setValue(QStringLiteral("scene_builder/preferred_right_width"), 370);''',
    "preferred panel widths",
)
s = replace_once(
    s,
    '''  if (auto * minimap = scene_page->findChild<QWidget *>(QStringLiteral("digital_twin_minimap"))) minimap->hide();''',
    '''  if (auto * minimap = scene_page->findChild<QWidget *>(QStringLiteral("digital_twin_minimap"))) {
    minimap->hide();
    minimap->setFixedSize(0, 0);
    minimap->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    minimap->setFocusPolicy(Qt::NoFocus);
  }''',
    "remove minimap from product layout",
)

inspector_anchor = '''      if (scene_card) {
        card->hide();
        break;
      }
    }
  }
}

inline void simplify_status_area(QWidget * scene_page)'''
inspector_replacement = '''      if (scene_card) {
        card->hide();
        break;
      }
    }

    for (auto * spin : selection_page->findChildren<QDoubleSpinBox *>()) {
      if (!spin) continue;
      spin->setMinimumWidth(76);
      spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    if (auto * advanced = selection_page->findChild<QGroupBox *>(QStringLiteral("sceneBuilderInspectorAdvancedDetails"))) {
      advanced->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
      const auto sync_advanced_height = [advanced](bool expanded) {
        advanced->setMaximumHeight(expanded ? QWIDGETSIZE_MAX : 32);
      };
      sync_advanced_height(advanced->isChecked());
      QObject::connect(advanced, &QGroupBox::toggled, advanced, sync_advanced_height);
    }

    for (auto * group : selection_page->findChildren<QGroupBox *>()) {
      if (!group || group->title().trimmed() != QStringLiteral("Robot Base Pose")) continue;
      group->setObjectName(QStringLiteral("sceneBuilderRobotBasePoseGroup"));
      group->setCheckable(true);
      group->setChecked(false);
      group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
      const auto sync_robot_height = [group](bool expanded) {
        group->setMaximumHeight(expanded ? QWIDGETSIZE_MAX : 32);
      };
      sync_robot_height(false);
      QObject::connect(group, &QGroupBox::toggled, group, sync_robot_height);
      break;
    }
  }

  if (auto * right_panel = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderRightPanel"))) {
    for (auto * scroll : right_panel->findChildren<QScrollArea *>()) {
      if (!scroll) continue;
      scroll->setWidgetResizable(true);
      scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }
  }
}

inline void simplify_status_area(QWidget * scene_page)'''
s = replace_once(s, inspector_anchor, inspector_replacement, "compact inspector")
s = replace_once(
    s,
    '''    QTabWidget#sceneBuilderInspectorTabs QTabBar::tab { padding:7px 10px; }''',
    '''    QTabWidget#sceneBuilderInspectorTabs QTabBar::tab { padding:7px 10px; }
    QGroupBox#sceneBuilderInspectorAdvancedDetails, QGroupBox#sceneBuilderRobotBasePoseGroup { margin-top:4px; padding-top:8px; }''',
    "inspector qss",
)
LAYOUT.write_text(s, encoding="utf-8")

# Keep readiness diagnostics, but make the card compact in embedded mode.
s = CSS.read_text(encoding="utf-8")
marker = "/* Workcell Studio embedded Product View: compact ready diagnostics. */"
if marker not in s:
    s += r'''

/* Workcell Studio embedded Product View: compact ready diagnostics. */
body.embedded-mode .scene-health {
  top: 0.5rem;
  left: 0.5rem;
  width: min(24rem, calc(100% - 1rem));
  font-size: 0.72rem;
}
body.embedded-mode .scene-health summary {
  min-height: 2.45rem;
  padding: 0.34rem 0.5rem;
  gap: 0.4rem;
  border-radius: 0.5rem;
}
body.embedded-mode .health-dot { width: 0.56rem; height: 0.56rem; }
body.embedded-mode .health-copy strong { font-size: 0.76rem; }
body.embedded-mode .health-copy small { font-size: 0.65rem; }
body.embedded-mode .health-review { font-size: 0.65rem; }
'''
CSS.write_text(s, encoding="utf-8")

# Update existing product-layout expectations and add focused regression coverage.
t = PRODUCT_TEST.read_text(encoding="utf-8")
for old, new in (
    ("'left->setMinimumWidth(245)'", "'left->setMinimumWidth(300)'"),
    ("'left->setMaximumWidth(300)'", "'left->setMaximumWidth(380)'"),
    ("'right->setMinimumWidth(300)'", "'right->setMinimumWidth(350)'"),
    ("'right->setMaximumWidth(360)'", "'right->setMaximumWidth(440)'"),
    ("'splitter->setSizes({270, 1120, 320})'", "'splitter->setSizes({325, 1040, 370})'"),
):
    t = replace_once(t, old, new, f"product test {old}")
PRODUCT_TEST.write_text(t, encoding="utf-8")

LIVE_TEST.write_text(r'''from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
LAYOUT = (ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp").read_text(encoding="utf-8")
CSS = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")


def body(start: str, end: str) -> str:
    return MAIN.split(start, 1)[1].split(end, 1)[0]


def test_live_crud_never_rebuilds_scene_builder_from_disk():
    duplicate = body("void MainWindow::duplicate_selected_item()", "bool MainWindow::selected_item_can_be_deleted()")
    delete = body("void MainWindow::delete_selected_item()", "double MainWindow::current_nudge_step_m")
    undo = body("void MainWindow::undo_layout_edit()", "void MainWindow::redo_layout_edit()")
    redo = body("void MainWindow::redo_layout_edit()", "bool MainWindow::selected_item_can_be_duplicated()")
    for section in (duplicate, delete, undo, redo):
        assert "refresh_scene_builder_left_explorer();" not in section
        assert "refresh_scene_hierarchy_tree_from_current_items();" in section
    assert duplicate.index("apply_scene_selection(new_id") < duplicate.index("refresh_scene_hierarchy_tree_from_current_items();")


def test_product_has_no_2d_minimap():
    presentation = body("void MainWindow::update_minimap_backend_presentation()", "void MainWindow::select_canvas_item")
    assert "minimap_view_->hide();" in presentation
    assert "setMaximumSize(0, 0)" in presentation
    assert "setVisible(minimap_requested_visible_)" not in presentation
    assert "minimap->setFixedSize(0, 0)" in LAYOUT
    assert "minimap->setVisible(true)" not in LAYOUT


def test_inspector_and_panels_are_readable_and_compact():
    labels = body("void MainWindow::refresh_selected_scene_item_labels", "bool MainWindow::is_pick_source_candidate")
    assert 'live_coordinate_label_->clear();' in labels
    assert 'live_coordinate_label_->setVisible(false);' in labels
    assert 'live_coordinate_label_->setVisible(true);' in labels
    for token in (
        'text.contains(QStringLiteral(">Scene Builder"))',
        'left->setMinimumWidth(300)',
        'left->setMaximumWidth(380)',
        'right->setMinimumWidth(350)',
        'right->setMaximumWidth(440)',
        'setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)',
        'spin->setMinimumWidth(76)',
        'sceneBuilderRobotBasePoseGroup',
        'advanced->setMaximumHeight(expanded ? QWIDGETSIZE_MAX : 32)',
    ):
        assert token in LAYOUT


def test_embedded_ready_card_is_compact():
    assert 'Workcell Studio embedded Product View: compact ready diagnostics.' in CSS
    assert 'body.embedded-mode .scene-health {' in CSS
    assert 'min-height: 2.45rem' in CSS
''', encoding="utf-8")
