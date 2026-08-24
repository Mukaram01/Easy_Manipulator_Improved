#pragma once

#include <functional>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFrame>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSizePolicy>
#include <QSplitter>
#include <QStringList>
#include <QTabBar>
#include <QTabWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGlobal>

#include "gui/mainwindow.h"
#include "scene_preview_widget.h"

namespace workcell_builder
{
namespace scene_builder_product_layout
{

class HierarchyOnlyContextMenuFilter final : public QObject
{
public:
  explicit HierarchyOnlyContextMenuFilter(QObject * parent = nullptr)
  : QObject(parent)
  {
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    Q_UNUSED(watched);
    return event && event->type() == QEvent::ContextMenu;
  }
};

inline QPushButton * button_with_text(QWidget * root, const QString & text)
{
  if (!root) return nullptr;
  for (auto * button : root->findChildren<QPushButton *>()) {
    if (button && button->text().trimmed() == text) return button;
  }
  return nullptr;
}

inline QTabWidget * tab_widget_with_labels(QWidget * root, const QStringList & labels)
{
  if (!root) return nullptr;
  for (auto * tabs : root->findChildren<QTabWidget *>()) {
    if (!tabs || tabs->count() < labels.size()) continue;
    bool match = true;
    for (int i = 0; i < labels.size(); ++i) {
      if (tabs->tabText(i).trimmed() != labels[i]) {
        match = false;
        break;
      }
    }
    if (match) return tabs;
  }
  return nullptr;
}

inline void set_hierarchy_filter(QTreeWidget * tree, const QString & text)
{
  if (!tree) return;
  const QString needle = text.trimmed().toLower();

  std::function<bool(QTreeWidgetItem *)> apply = [&](QTreeWidgetItem * item) {
    if (!item) return false;
    bool child_match = false;
    for (int i = 0; i < item->childCount(); ++i) {
      child_match = apply(item->child(i)) || child_match;
    }
    const bool self_match = needle.isEmpty() ||
      item->text(0).toLower().contains(needle) ||
      item->text(1).toLower().contains(needle);
    const bool visible = self_match || child_match;
    item->setHidden(!visible);
    if (!needle.isEmpty() && child_match) item->setExpanded(true);
    return visible;
  };

  for (int i = 0; i < tree->topLevelItemCount(); ++i) apply(tree->topLevelItem(i));
}

inline void collapse_layers_group(QWidget * scene_page)
{
  if (!scene_page) return;
  for (auto * group : scene_page->findChildren<QGroupBox *>()) {
    if (!group || group->title().trimmed() != QStringLiteral("Layers")) continue;
    group->setObjectName(QStringLiteral("sceneBuilderLayersGroup"));
    group->setCheckable(true);
    group->setChecked(false);
    const auto boxes = group->findChildren<QCheckBox *>(QString(), Qt::FindDirectChildrenOnly);
    for (auto * box : boxes) if (box) box->hide();
    QObject::connect(group, &QGroupBox::toggled, group, [boxes](bool expanded) {
      for (auto * box : boxes) if (box) box->setVisible(expanded);
    });
    break;
  }
}

inline void hide_duplicate_scene_identity(QWidget * scene_page)
{
  if (!scene_page) return;
  if (auto * identity = scene_page->findChild<QLabel *>(QStringLiteral("sceneBuilderCompactSceneIdentity"))) {
    identity->hide();
  }
  for (auto * label : scene_page->findChildren<QLabel *>()) {
    if (!label) continue;
    const QString text = label->text().trimmed();
    if (text == QStringLiteral("Scene Builder") || text.startsWith(QStringLiteral("Scene Builder:")) ||
        text.contains(QStringLiteral(">Scene Builder"))) {
      label->hide();
    }
  }
}

inline void promote_major_tabs(QWidget * scene_page)
{
  if (!scene_page || scene_page->findChild<QTabBar *>(QStringLiteral("sceneBuilderMajorTabs"))) return;
  auto * source_tabs = tab_widget_with_labels(scene_page, {
    QStringLiteral("Scene"), QStringLiteral("Assets"), QStringLiteral("Workflow")});
  auto * page_layout = qobject_cast<QVBoxLayout *>(scene_page->layout());
  if (!source_tabs || !page_layout) return;

  auto * major_tabs = new QTabBar(scene_page);
  major_tabs->setObjectName(QStringLiteral("sceneBuilderMajorTabs"));
  major_tabs->setDocumentMode(true);
  major_tabs->setExpanding(false);
  major_tabs->addTab(QStringLiteral("Scene"));
  major_tabs->addTab(QStringLiteral("Assets"));
  major_tabs->addTab(QStringLiteral("Workflow"));
  major_tabs->setCurrentIndex(source_tabs->currentIndex());

  source_tabs->tabBar()->hide();
  QObject::connect(major_tabs, &QTabBar::currentChanged, source_tabs, &QTabWidget::setCurrentIndex);
  QObject::connect(source_tabs, &QTabWidget::currentChanged, major_tabs, &QTabBar::setCurrentIndex);

  QWidget * workspace = scene_page->findChild<QWidget *>(QStringLiteral("sceneBuilderWorkspace"));
  const int workspace_index = workspace ? page_layout->indexOf(workspace) : 0;
  page_layout->insertWidget(qMax(0, workspace_index), major_tabs, 0);
}

inline void hide_embedded_preview_chrome(ScenePreviewWidget * preview)
{
  if (!preview) return;
  // ScenePreviewWidget remains the sole renderer/editor owner.  Scene Builder
  // exposes the same authoring commands in one toolbar, so its internal control
  // header is hidden rather than duplicated above the viewport.
  if (auto * chip = preview->findChild<QLabel *>(QStringLiteral("previewToolbarChip"))) {
    if (auto * controls_host = chip->parentWidget()) {
      controls_host->setObjectName(QStringLiteral("sceneBuilderEmbeddedPreviewChrome"));
      controls_host->hide();
    }
  }
}

inline void keep_authoring_context_menu_in_hierarchy(
  MainWindow * window, QWidget * scene_page, ScenePreviewWidget * preview)
{
  if (!window || !scene_page) return;

  // Scene-item authoring commands belong to the Scene Hierarchy.  Keep the
  // Product View focused on selection/camera/gizmo interaction and disconnect
  // the two 3D-menu paths installed by the compatibility shell.
  if (preview) {
    QObject::disconnect(
      preview, SIGNAL(embedded_authoring_context_menu_requested(QPoint)), window, nullptr);
    QObject::disconnect(
      preview, SIGNAL(customContextMenuRequested(QPoint)), window, nullptr);
    preview->setContextMenuPolicy(Qt::NoContextMenu);
  }

  // The native compatibility canvas has a MainWindow event filter that also
  // opens the authoring menu. Install a later, narrowly scoped filter that only
  // consumes context-menu events; placement and normal mouse events continue to
  // the existing MainWindow filter unchanged.
  if (auto * canvas = scene_page->findChild<QGraphicsView *>(QStringLiteral("digital_twin_canvas_"))) {
    QWidget * viewport = canvas->viewport();
    if (viewport && !viewport->findChild<QObject *>(QStringLiteral("sceneBuilderHierarchyOnlyContextMenuFilter"))) {
      auto * filter = new HierarchyOnlyContextMenuFilter(viewport);
      filter->setObjectName(QStringLiteral("sceneBuilderHierarchyOnlyContextMenuFilter"));
      viewport->installEventFilter(filter);
    }
  }
}

inline QPushButton * make_proxy_button(
  QWidget * parent, const QString & text, QPushButton * target,
  const QString & object_name = QString())
{
  auto * button = new QPushButton(text, parent);
  if (!object_name.isEmpty()) button->setObjectName(object_name);
  button->setMinimumHeight(32);
  button->setEnabled(target != nullptr);
  if (target) {
    QObject::connect(button, &QPushButton::clicked, target, &QPushButton::click);
    QObject::connect(target, &QObject::destroyed, button, [button]() { button->setEnabled(false); });
  }
  return button;
}

inline void rebuild_single_toolbar(QWidget * scene_page, ScenePreviewWidget * preview)
{
  if (!scene_page) return;
  auto * host = scene_page->findChild<QWidget *>(QStringLiteral("scene_builder_top_controls_host"));
  auto * layout = host ? qobject_cast<QHBoxLayout *>(host->layout()) : nullptr;
  if (!host || !layout) return;

  QPushButton * select = button_with_text(host, QStringLiteral("Select"));
  QPushButton * place = button_with_text(host, QStringLiteral("Place Asset"));
  QPushButton * move = button_with_text(host, QStringLiteral("Move"));
  QPushButton * rotate = button_with_text(host, QStringLiteral("Rotate"));
  QPushButton * duplicate = button_with_text(host, QStringLiteral("Duplicate"));
  QPushButton * save = button_with_text(host, QStringLiteral("Save Layout"));

  QPushButton * embedded_add = button_with_text(preview, QStringLiteral("Add object"));
  QPushButton * embedded_fit = preview ? preview->findChild<QPushButton *>(QStringLiteral("embeddedFitButton")) : nullptr;
  QPushButton * embedded_undo = button_with_text(preview, QStringLiteral("Undo"));
  QPushButton * embedded_redo = button_with_text(preview, QStringLiteral("Redo"));

  auto * add_object = make_proxy_button(host, QStringLiteral("Add Object"), embedded_add, QStringLiteral("sceneBuilderToolbarAddObject"));
  auto * fit = make_proxy_button(host, QStringLiteral("Fit"), embedded_fit, QStringLiteral("sceneBuilderToolbarFit"));
  auto * undo = make_proxy_button(host, QStringLiteral("Undo"), embedded_undo, QStringLiteral("sceneBuilderToolbarUndo"));
  auto * redo = make_proxy_button(host, QStringLiteral("Redo"), embedded_redo, QStringLiteral("sceneBuilderToolbarRedo"));

  QToolButton * overflow = scene_page->findChild<QToolButton *>(QStringLiteral("scene_builder_secondary_overflow_button"));
  if (overflow) overflow->setText(QStringLiteral("View"));

  // Rebuild only the existing production toolbar. Widgets keep their existing
  // owners; ScenePreviewWidget/WebEngine is never reparented or recreated.
  while (layout->count() > 0) {
    QLayoutItem * item = layout->takeAt(0);
    if (!item) continue;
    if (item->widget()) item->widget()->hide();
    delete item;
  }

  // Duplicate is intentionally contextual (Scene Hierarchy / Ctrl+D). The
  // compatibility shell still owns this button and can toggle its visibility,
  // so collapse its geometry after removing it from the product toolbar. This
  // prevents the orphaned button from reappearing over the 3D canvas while the
  // underlying QAction remains fully functional.
  if (duplicate) {
    duplicate->hide();
    duplicate->setFixedSize(0, 0);
    duplicate->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    duplicate->setFocusPolicy(Qt::NoFocus);
  }

  const auto add_existing = [layout](QWidget * widget) {
    if (!widget) return;
    widget->show();
    widget->setMinimumHeight(32);
    layout->addWidget(widget);
  };
  add_existing(select);
  add_existing(move);
  add_existing(rotate);
  add_existing(place);
  add_existing(add_object);
  add_existing(fit);
  layout->addStretch(1);
  add_existing(undo);
  add_existing(redo);
  add_existing(save);
  add_existing(overflow);

  // The outer row also contains a textual copy of the current authoring mode.
  for (auto * label : scene_page->findChildren<QLabel *>()) {
    if (label && label->text().startsWith(QStringLiteral("Mode:"))) label->hide();
  }
}

inline void simplify_hierarchy(QWidget * scene_page)
{
  if (!scene_page) return;
  auto * tree = scene_page->findChild<QTreeWidget *>(QStringLiteral("studioSceneHierarchyTree"));
  if (!tree) return;
  tree->setHeaderHidden(true);
  tree->setColumnHidden(1, true);
  tree->setColumnHidden(2, true);
  tree->setIndentation(16);
  tree->setUniformRowHeights(true);
  tree->setAlternatingRowColors(false);

  if (auto * selected_card = scene_page->findChild<QFrame *>(QStringLiteral("studioSelectedItemCard"))) {
    selected_card->hide();
  }

  QWidget * card = tree->parentWidget();
  auto * card_layout = card ? qobject_cast<QVBoxLayout *>(card->layout()) : nullptr;
  if (card_layout && !card->findChild<QLineEdit *>(QStringLiteral("sceneBuilderHierarchySearch"))) {
    auto * search = new QLineEdit(card);
    search->setObjectName(QStringLiteral("sceneBuilderHierarchySearch"));
    search->setPlaceholderText(QStringLiteral("Search hierarchy…"));
    search->setClearButtonEnabled(true);
    const int tree_index = card_layout->indexOf(tree);
    card_layout->insertWidget(qMax(0, tree_index), search);
    QObject::connect(search, &QLineEdit::textChanged, tree, [tree](const QString & text) {
      set_hierarchy_filter(tree, text);
    });
  }

  collapse_layers_group(scene_page);
}

inline void simplify_inspector(QWidget * scene_page)
{
  if (!scene_page) return;
  auto * tabs = tab_widget_with_labels(scene_page, {
    QStringLiteral("Selection"), QStringLiteral("Workflow"), QStringLiteral("Readiness")});
  if (!tabs) return;
  tabs->setObjectName(QStringLiteral("sceneBuilderInspectorTabs"));
  tabs->setTabText(0, QStringLiteral("Inspector"));
  tabs->setTabText(1, QStringLiteral("Task"));
  tabs->setTabText(2, QStringLiteral("Checks"));
  tabs->setCurrentIndex(0);

  // The top context banner already identifies the workcell. Do not repeat the
  // same scene metadata card inside the contextual item inspector.
  QWidget * selection_page = tabs->widget(0);
  if (selection_page) {
    for (auto * card : selection_page->findChildren<QFrame *>(QStringLiteral("studioCard"), Qt::FindDirectChildrenOnly)) {
      const auto labels = card->findChildren<QLabel *>();
      bool scene_card = false;
      for (auto * label : labels) {
        if (label && label->text().contains(QStringLiteral("<b>Scene</b>"))) {
          scene_card = true;
          break;
        }
      }
      if (scene_card) {
        card->hide();
        break;
      }
    }
    if (auto * inspector = selection_page->findChild<QLabel *>(QStringLiteral("sceneBuilderInspectorLabel"))) {
      QWidget * card = inspector->parentWidget();
      while (card && card != selection_page && card->objectName() != QStringLiteral("studioCard")) {
        card = card->parentWidget();
      }
      if (card && card != selection_page) {
        card->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        if (card->layout()) card->layout()->setAlignment(Qt::AlignTop);
        card->setMaximumHeight(520);
        for (auto * label : card->findChildren<QLabel *>()) {
          if (label) label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        }
      }
    }
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
    // Anchor the inspector stack below the primary transform card; otherwise
    // QVBoxLayout distributes spare height between collapsed sections.
    if (!selection_page->property("sceneBuilderTrailingStretchAdded").toBool()) {
      if (auto * layout = qobject_cast<QVBoxLayout *>(selection_page->layout())) {
        layout->addStretch(1);
        selection_page->setProperty("sceneBuilderTrailingStretchAdded", true);
      }
    }
  }
}

inline void simplify_status_area(QWidget * scene_page)
{
  if (!scene_page) return;
  if (auto * status = scene_page->findChild<QLabel *>(QStringLiteral("sceneBuilderLatestStatus"))) status->hide();
  for (auto * label : scene_page->findChildren<QLabel *>()) {
    if (!label) continue;
    const QString text = label->text().trimmed();
    if (text.startsWith(QStringLiteral("Unsaved Layout Edits:")) ||
        text.startsWith(QStringLiteral("Legend:")) ||
        text.startsWith(QStringLiteral("Scene load:"))) {
      label->hide();
    }
  }
  if (auto * minimap = scene_page->findChild<QWidget *>(QStringLiteral("digital_twin_minimap"))) {
    minimap->setFixedSize(150, 90);
    minimap->show();
  }
  if (auto * bottom = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderBottomStatusBar"))) bottom->hide();
}

inline void configure_scene_builder_product_layout(MainWindow * window)
{
  if (!window) return;
  auto * scene_page = window->findChild<QWidget *>(QStringLiteral("workcellStudioSceneBuilderPage"));
  if (!scene_page || scene_page->property("sceneBuilderProductLayoutApplied").toBool()) return;
  scene_page->setProperty("sceneBuilderProductLayoutApplied", true);

  hide_duplicate_scene_identity(scene_page);
  promote_major_tabs(scene_page);

  auto * left = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderLeftPanel"));
  auto * center = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderProductViewPanel"));
  auto * right = scene_page->findChild<QFrame *>(QStringLiteral("sceneBuilderRightPanel"));
  auto * splitter = scene_page->findChild<QSplitter *>(QStringLiteral("sceneBuilderMainSplitter"));
  if (left) {
    left->setMinimumWidth(300);
    left->setMaximumWidth(380);
  }
  if (center) center->setMinimumWidth(760);
  if (right) {
    right->setMinimumWidth(350);
    right->setMaximumWidth(440);
    right->show();
  }
  if (splitter) {
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 10);
    splitter->setStretchFactor(2, 1);
    splitter->setSizes({325, 1040, 370});
  }

  QSettings settings;
  settings.setValue(QStringLiteral("scene_builder/preferred_left_width"), 325);
  settings.setValue(QStringLiteral("scene_builder/preferred_right_width"), 370);
  settings.setValue(QStringLiteral("scene_builder/right_panel_visible"), true);

  simplify_hierarchy(scene_page);
  simplify_inspector(scene_page);
  simplify_status_area(scene_page);

  auto * preview = scene_page->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"));
  keep_authoring_context_menu_in_hierarchy(window, scene_page, preview);
  rebuild_single_toolbar(scene_page, preview);
  hide_embedded_preview_chrome(preview);

  scene_page->setStyleSheet(scene_page->styleSheet() + QStringLiteral(R"QSS(
    QWidget#workcellStudioSceneBuilderPage { background:#F7F9FC; color:#172B3F; }
    QTabBar#sceneBuilderMajorTabs { background:#FFFFFF; border-bottom:1px solid #DDE5EE; }
    QTabBar#sceneBuilderMajorTabs::tab { min-width:74px; padding:10px 16px; color:#5C6F82; border:0; border-bottom:2px solid transparent; font-weight:600; }
    QTabBar#sceneBuilderMajorTabs::tab:selected { color:#125FC3; border-bottom:2px solid #246FE5; }
    QWidget#sceneBuilderWorkspace { background:#FFFFFF; border:1px solid #DDE5EE; border-radius:0px; }
    QFrame#sceneBuilderLeftPanel, QFrame#sceneBuilderRightPanel { background:#FFFFFF; border:0px; }
    QFrame#sceneBuilderProductViewPanel { background:#F8FAFC; border:0px; }
    QWidget#scene_builder_top_controls_host { background:#FFFFFF; border:0px; }
    QWidget#scene_builder_top_controls_host QPushButton { min-height:32px; padding:4px 12px; border-radius:6px; }
    QWidget#scene_builder_top_controls_host QPushButton:checked { background:#E7F0FF; color:#114C91; border:1px solid #95BCEB; }
    QTreeWidget#studioSceneHierarchyTree { background:#FFFFFF; border:0px; outline:0px; }
    QTreeWidget#studioSceneHierarchyTree::item { min-height:28px; padding:2px 4px; }
    QTreeWidget#studioSceneHierarchyTree::item:selected { background:#EAF2FB; color:#153B5F; border-radius:4px; }
    QLineEdit#sceneBuilderHierarchySearch { min-height:30px; border:1px solid #D8E2EC; border-radius:6px; padding:2px 8px; background:#FFFFFF; }
    QGroupBox#sceneBuilderLayersGroup { border:1px solid #DCE5EE; border-radius:6px; margin-top:8px; padding:8px; font-weight:600; }
    QFrame#sceneBuilderBottomStatusBar { background:#FFFFFF; border-top:1px solid #DDE5EE; border-left:0; border-right:0; border-bottom:0; border-radius:0; }
    QTabWidget#sceneBuilderInspectorTabs::pane { border:0px; background:#FFFFFF; }
    QTabWidget#sceneBuilderInspectorTabs QTabBar::tab { padding:7px 10px; }
    QGraphicsView#digital_twin_minimap { min-width:150px; max-width:150px; min-height:90px; max-height:90px; border:1px solid #D8E2EC; border-radius:6px; background:#F8FAFC; }
    QGroupBox#sceneBuilderInspectorAdvancedDetails { margin-top:6px; padding:5px; }
    QFrame#sceneBuilderRightPanel QDoubleSpinBox { min-width:76px; }
  )QSS"));
}

}  // namespace scene_builder_product_layout
}  // namespace workcell_builder
