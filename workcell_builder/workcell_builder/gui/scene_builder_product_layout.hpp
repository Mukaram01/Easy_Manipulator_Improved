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
#include <QStyle>
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
#include "workcell_builder_ui_utils.hpp"

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

// 8 px side padding on the button itself (wins over inherited toolbar rules) and a
// minimum width from its bold label, so "Select"/"Rotate" never collapse to
// "Selec/Rotat" and the toolbar still fits Mate at 1366 px.
enum class ToolbarSegment { None, First, Middle, Last };
inline void compact_toolbar_button(QPushButton * button, ToolbarSegment segment = ToolbarSegment::None)
{
  if (!button) return;
  // A widget's own style sheet wins over inherited rules property by property,
  // so a segment restates its joined-segment borders here.
  QString rules = QStringLiteral("padding-left: 8px; padding-right: 8px;");
  const QString radius = workcell_builder::studioTokens().radiusS;
  if (segment != ToolbarSegment::None) rules += QStringLiteral(" border-radius: 0; border-right-width: 0;");
  if (segment == ToolbarSegment::First) {
    rules += QStringLiteral(" border-top-left-radius: %1; border-bottom-left-radius: %1;").arg(radius);
  } else if (segment == ToolbarSegment::Last) {
    rules += QStringLiteral(" border-top-right-radius: %1; border-bottom-right-radius: %1; border-right-width: 1px;").arg(radius);
  }
  button->setStyleSheet(QStringLiteral("QPushButton { %1 }").arg(rules));
  button->ensurePolished();
  QFont bold = button->font();
  bold.setBold(true);
  button->setMinimumWidth(QFontMetrics(bold).horizontalAdvance(button->text()) + 2 * 8 + 6);
}

inline void give_toolbar_icon(QAbstractButton * button, const QString & icon, const QString & tooltip = QString())
{
  if (!button) return;
  if (button->property("fullText").toString().isEmpty()) button->setProperty("fullText", button->text());
  button->setIcon(studioIcon(icon));
  button->setIconSize(QSize(16, 16));
  button->setAccessibleName(button->property("fullText").toString());
  if (!tooltip.isEmpty()) button->setToolTip(tooltip);
  else if (button->toolTip().isEmpty()) button->setToolTip(button->property("fullText").toString());
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
  QPushButton * operator_preview = host->findChild<QPushButton *>(QStringLiteral("sceneBuilderToolbarOperatorPreview"));
  if (select) select->setObjectName(QStringLiteral("sceneBuilderModeSelect"));
  if (move) move->setObjectName(QStringLiteral("sceneBuilderModeMove"));
  if (rotate) rotate->setObjectName(QStringLiteral("sceneBuilderModeRotate"));
  if (save) save->setObjectName(QStringLiteral("sceneBuilderToolbarSave"));
  QPushButton * mate = scene_page->findChild<QPushButton *>(QStringLiteral("sceneBuilderToolbarMate"));

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

  layout->setSpacing(4);  // Mate fits at 1366 px without squeezing Select/Move/Rotate
  const auto add_existing = [layout](QWidget * widget) {
    if (!widget) return;
    widget->show();
    widget->setMinimumHeight(32);
    layout->addWidget(widget);
  };
  auto * edit_mode_group = new QWidget(host);
  edit_mode_group->setObjectName(QStringLiteral("sceneBuilderEditModeGroup"));
  auto * edit_mode_layout = new QHBoxLayout(edit_mode_group);
  edit_mode_layout->setContentsMargins(0, 0, 0, 0);
  edit_mode_layout->setSpacing(0);
  for (auto * button : {select, move, rotate}) {
    if (!button) continue;
    button->show();
    button->setMinimumHeight(32);
    compact_toolbar_button(button, button == select ? ToolbarSegment::First
      : button == rotate ? ToolbarSegment::Last : ToolbarSegment::Middle);
    button->setProperty("editModeSegment", true);
    edit_mode_layout->addWidget(button);
  }
  layout->addWidget(edit_mode_group);
  layout->addSpacing(4);
  if (place) place->hide();
  for (auto * compact : {fit, mate}) compact_toolbar_button(compact);
  add_existing(add_object);
  add_existing(fit);
  add_existing(mate);
  layout->addStretch(1);
  // Undo/Redo as compact icon buttons so the toolbar fits a 1366 px display;
  // the text stays as tooltip and accessible name.
  for (auto * button : {undo, redo}) {
    if (!button) continue;
    const bool is_undo = button == undo;
    button->setIcon(host->style()->standardIcon(is_undo ? QStyle::SP_ArrowBack : QStyle::SP_ArrowForward));
    button->setToolTip(is_undo ? QStringLiteral("Undo (Ctrl+Z)") : QStringLiteral("Redo (Ctrl+Shift+Z)"));
    button->setAccessibleName(button->text());
    button->setText(QString());
    button->setFixedWidth(34);
  }
  add_existing(undo);
  add_existing(redo);
  layout->addSpacing(4);
  add_existing(save);
  add_existing(operator_preview);
  add_existing(overflow);

  give_toolbar_icon(select, QStringLiteral("select"), QStringLiteral("Select"));
  give_toolbar_icon(move, QStringLiteral("move"), QStringLiteral("Move"));
  give_toolbar_icon(rotate, QStringLiteral("rotate"), QStringLiteral("Rotate"));
  give_toolbar_icon(add_object, QStringLiteral("add"));
  give_toolbar_icon(fit, QStringLiteral("fit"), QStringLiteral("Fit the view to the workcell"));
  give_toolbar_icon(mate, QStringLiteral("mate"));
  give_toolbar_icon(save, QStringLiteral("save"), QStringLiteral("Save Layout (Ctrl+S)"));
  give_toolbar_icon(operator_preview, QStringLiteral("preview"));
  if (overflow) {
    overflow->setIcon(studioIcon(QStringLiteral("view")));
    overflow->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    give_toolbar_icon(overflow, QStringLiteral("view"), QStringLiteral("View options"));
  }
  for (auto * button : {undo, redo}) {
    if (!button) continue;
    button->setIcon(studioIcon(button == undo ? QStringLiteral("undo") : QStringLiteral("redo")));
  }
  // Labels collapse to icons as the toolbar narrows (shared component, see workcell_builder_ui_utils).
  installResponsiveToolbar(host, {select, move, rotate}, {add_object, fit, mate, operator_preview, overflow}, save);

  if (save) {
    save->setProperty("layoutDirty", false);
  }

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
    QStringLiteral("Inspector"), QStringLiteral("Task"), QStringLiteral("Checks")});
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
        card->setMaximumHeight(560);
        for (auto * label : card->findChildren<QLabel *>()) {
          if (label) label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        }
      }
    }
    for (auto * spin : selection_page->findChildren<QDoubleSpinBox *>()) {
      if (!spin) continue;
      // Prefix + unit suffix make the spin box's size hint wider than a third
      // of the inspector; share the row width instead of overflowing it.
      spin->setMinimumWidth(64);
      spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
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
    if (auto * layout = qobject_cast<QVBoxLayout *>(selection_page->layout())) {
      layout->setContentsMargins(4, 4, 4, 4);
      layout->setSpacing(4);
      layout->setAlignment(Qt::AlignTop);
    }
  }
}

inline void simplify_status_area(QWidget * scene_page)
{
  if (!scene_page) return;
  for (auto * label : scene_page->findChildren<QLabel *>()) {
    if (!label) continue;
    const QString text = label->text().trimmed();
    if (text.startsWith(QStringLiteral("Unsaved Layout Edits:")) ||
        text.startsWith(QStringLiteral("Legend:")) ||
        text.startsWith(QStringLiteral("Scene load:"))) {
      label->hide();
    }
  }
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
    left->setMinimumWidth(180);
    left->setMaximumWidth(360);
  }
  // 180 + 440 + 476 px of panels fit the 1280 x 720 constrained mode. The toolbar adapts its labels
  // (installResponsiveToolbar), so the 3D view no longer reserves the full labelled toolbar width.
  if (center) center->setMinimumWidth(440);
  if (right) {
    // Unit suffixes (m, °) widen the transform spin boxes; keep the inspector
    // wide enough that no field or button is ever clipped.
    // The Task page needs ~434 px of content (three unit spin boxes per row); 476 keeps it whole.
    right->setMinimumWidth(476);
    keepWrappedLabelHeights(right);  // inspector forms: wrapped help text must not overlap the next row
    for (auto * area : right->findChildren<QScrollArea *>()) keepScrollContentHeight(area);
    right->setMaximumWidth(560);
  }
  QSettings settings;
  seedSceneBuilderLayoutDefaults();
  if (right) right->setVisible(settings.value(QStringLiteral("scene_builder/right_panel_visible")).toBool());
  if (splitter) {
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 10);
    splitter->setStretchFactor(2, 1);
    // Product defaults only for a first run (or after Reset Layout); a user's own panel widths,
    // saved by MainWindow::save_scene_builder_panel_state(), are restored instead of overwritten.
    QList<int> saved;
    for (const auto & value : settings.value(QStringLiteral("scene_builder/main_splitter_sizes")).toList()) saved.append(value.toInt());
    if (saved.size() == 3 && saved[1] > 0) {
      splitter->setSizes({right && left && left->isHidden() ? 0 : saved[0], saved[1], right && right->isHidden() ? 0 : saved[2]});
    } else {
      splitter->setSizes({260, 960, 470});
    }
  }

  simplify_hierarchy(scene_page);
  simplify_inspector(scene_page);
  simplify_status_area(scene_page);

  auto * preview = scene_page->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"));
  keep_authoring_context_menu_in_hierarchy(window, scene_page, preview);
  rebuild_single_toolbar(scene_page, preview);
  hide_embedded_preview_chrome(preview);

  scene_page->setStyleSheet(scene_page->styleSheet() + studioQss(QStringLiteral(R"QSS(
    QWidget#workcellStudioSceneBuilderPage { background: @bg; color: @text; }
    QTabBar#sceneBuilderMajorTabs { background: @surface; border-bottom: 1px solid @border; }
    QTabBar#sceneBuilderMajorTabs::tab { min-width: 74px; padding: 10px 16px; color: @textSecondary; border: 0; border-bottom: 2px solid transparent; font-weight: 600; }
    QTabBar#sceneBuilderMajorTabs::tab:selected { color: @accent; border-bottom: 2px solid @accent; }
    QWidget#sceneBuilderWorkspace { background: @surface; border: 1px solid @border; border-radius: 0px; }
    QFrame#sceneBuilderLeftPanel, QFrame#sceneBuilderRightPanel { background: @surface; border: 0px; }
    QFrame#sceneBuilderProductViewPanel { background: @surfaceSunken; border: 0px; }
    QWidget#scene_builder_top_controls_host { background: @glass; border: 0px; border-bottom: 1px solid @glassBorder; }
    QWidget#scene_builder_top_controls_host QPushButton { min-height: 32px; padding: 4px 12px; border: 1px solid @borderStrong; border-radius: @radiusS; background: @surface; color: @text; }
    QWidget#scene_builder_top_controls_host QPushButton:hover { background: @accentSoft; border-color: @accent; }
    QWidget#sceneBuilderEditModeGroup QPushButton { border-radius: 0; border-right-width: 0; }
    QWidget#scene_builder_top_controls_host QPushButton#sceneBuilderToolbarMate:checked { background: @accent; color: @accentText; border-color: @accent; }
    QPushButton#sceneBuilderModeSelect { border-top-left-radius: @radiusS; border-bottom-left-radius: @radiusS; }
    QPushButton#sceneBuilderModeRotate { border-top-right-radius: @radiusS; border-bottom-right-radius: @radiusS; border-right-width: 1px; }
    QWidget#sceneBuilderEditModeGroup QPushButton:checked { background: @accentSoft; color: @accent; border-color: @accent; }
    QPushButton#sceneBuilderToolbarSave[layoutDirty="true"] { background: @accent; color: @accentText; border-color: @accent; font-weight: 650; }
    QPushButton#sceneBuilderToolbarSave[layoutDirty="true"]:hover { background: @accentHover; border-color: @accentHover; }
    QPushButton#sceneBuilderToolbarSave[layoutDirty="false"] { background: @surfaceSunken; color: @textMuted; border-color: @border; }
    QTreeWidget#studioSceneHierarchyTree { background: @surface; border: 0px; outline: 0px; }
    QTreeWidget#studioSceneHierarchyTree::item { min-height: 28px; padding: 2px 4px; border-radius: 4px; }
    QTreeWidget#studioSceneHierarchyTree::item:hover { background: @surfaceSunken; }
    QTreeWidget#studioSceneHierarchyTree::item:selected { background: @accentSoft; color: @text; border-left: 3px solid @accent; }
    QLineEdit#sceneBuilderHierarchySearch { min-height: 30px; border: 1px solid @borderStrong; border-radius: @radiusS; padding: 2px 8px; background: @surface; }
    QGroupBox#sceneBuilderLayersGroup { border: 1px solid @border; border-radius: @radiusS; margin-top: 8px; padding: 8px; font-weight: 600; }
    QFrame#sceneBuilderBottomStatusBar { min-height: 32px; max-height: 32px; background: @surfaceRaised; border-top: 1px solid @border; border-left: 0; border-right: 0; border-bottom: 0; border-radius: 0; }
    QPushButton#sceneBuilderActivityStrip { min-height: 32px; max-height: 32px; padding: 0 10px; border: 0; border-radius: 0; background: transparent; color: @textSecondary; text-align: left; }
    QPushButton#sceneBuilderActivityStrip:hover, QPushButton#sceneBuilderActivityStrip:checked { background: @surfaceSunken; color: @text; }
    QPushButton#sceneBuilderActivityStrip[hasIssues="true"] { color: @warning; font-weight: 650; }
    QPushButton#sceneBuilderPlaceAsset { background: @accent; color: @accentText; border: 1px solid @accent; font-weight: 600; min-height: 32px; }
    QPushButton#sceneBuilderAssetDetails, QPushButton#sceneBuilderInspectorApply, QPushButton#sceneBuilderInspectorRevert, QPushButton#sceneBuilderInspectorCopyTransform, QPushButton#sceneBuilderInspectorPasteTransform { background: @surface; color: @text; border: 1px solid @borderStrong; }
    QPushButton#sceneBuilderInspectorApply { background: @accentSoft; color: @accent; border-color: @accent; font-weight: 650; }
    QTabWidget#sceneBuilderInspectorTabs::pane { border: 0px; background: @surface; }
    QTabWidget#sceneBuilderInspectorTabs QTabBar::tab { padding: 7px 10px; }
    QGraphicsView#digital_twin_minimap { border: 1px solid @border; border-radius: @radiusS; background: @surfaceSunken; }
    QGroupBox#sceneBuilderInspectorAdvancedDetails { margin-top: 6px; padding: 5px; }
    QFrame#sceneBuilderRightPanel QDoubleSpinBox { min-width: 64px; padding: 2px 4px; }
  )QSS")));
}

}  // namespace scene_builder_product_layout
}  // namespace workcell_builder
