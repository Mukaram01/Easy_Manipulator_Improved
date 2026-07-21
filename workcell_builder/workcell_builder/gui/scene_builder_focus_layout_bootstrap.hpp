#pragma once

#include "scene_preview_widget.h"

#include <QAction>
#include <QApplication>
#include <QBoxLayout>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>

namespace workcell_builder
{
namespace scene_builder_focus_layout_detail
{
inline int tabIndexByText(QTabWidget * tabs, const QString & text)
{
  if (!tabs) return -1;
  for (int index = 0; index < tabs->count(); ++index) {
    if (tabs->tabText(index).compare(text, Qt::CaseInsensitive) == 0) return index;
  }
  return -1;
}

inline QTabWidget * findTabs(QWidget * root, const QStringList & required_tabs)
{
  if (!root) return nullptr;
  for (QTabWidget * tabs : root->findChildren<QTabWidget *>()) {
    bool matches = true;
    for (const QString & text : required_tabs) {
      if (tabIndexByText(tabs, text) < 0) {
        matches = false;
        break;
      }
    }
    if (matches) return tabs;
  }
  return nullptr;
}

inline QFrame * containingCard(QWidget * child)
{
  for (QWidget * current = child; current; current = current->parentWidget()) {
    auto * frame = qobject_cast<QFrame *>(current);
    if (frame && frame->objectName() == QStringLiteral("studioCard")) return frame;
  }
  return nullptr;
}

inline QLabel * findLabelContaining(QWidget * root, const QString & token)
{
  if (!root) return nullptr;
  for (QLabel * label : root->findChildren<QLabel *>()) {
    if (label->text().contains(token, Qt::CaseInsensitive)) return label;
  }
  return nullptr;
}

inline void addPanelActionsToTop(QToolButton * button)
{
  if (!button || !button->menu() || button->menu()->property("workcell_panel_actions_promoted").toBool()) return;
  QMenu * menu = button->menu();
  QAction * show_left = button->findChild<QAction *>(QStringLiteral("sceneBuilderShowLeftPanelAction"));
  QAction * show_right = button->findChild<QAction *>(QStringLiteral("sceneBuilderShowRightPanelAction"));
  QAction * focus = button->findChild<QAction *>(QStringLiteral("sceneBuilderFocus3dViewAction"));
  if (!show_left || !show_right || !focus) return;

  QAction * before = menu->actions().isEmpty() ? nullptr : menu->actions().first();
  menu->insertSection(before, QStringLiteral("Panels"));
  menu->insertAction(before, show_left);
  menu->insertAction(before, show_right);
  menu->insertAction(before, focus);
  menu->insertSeparator(before);
  menu->setProperty("workcell_panel_actions_promoted", true);
  button->setText(QStringLiteral("Panels & tools"));
  button->setToolTip(QStringLiteral("Show or hide Scene, Assets, Selection and Workflow panels."));
}

inline void installSceneBuilderFocusLayout(QWidget * root)
{
  if (!root) return;
  QWidget * workspace = nullptr;
  if (root->objectName() == QStringLiteral("sceneBuilderWorkspace")) workspace = root;
  if (!workspace) workspace = root->findChild<QWidget *>(QStringLiteral("sceneBuilderWorkspace"));
  if (!workspace || workspace->property("workcell_scene_builder_focus_layout_v1").toBool()) return;

  auto * splitter = workspace->findChild<QSplitter *>(QStringLiteral("sceneBuilderMainSplitter"));
  auto * left_panel = workspace->findChild<QFrame *>(QStringLiteral("sceneBuilderLeftPanel"));
  auto * right_panel = workspace->findChild<QFrame *>(QStringLiteral("sceneBuilderRightPanel"));
  auto * asset_search = workspace->findChild<QLineEdit *>(QStringLiteral("assetLibrarySearchBox"));
  auto * hierarchy_tree = workspace->findChild<QTreeWidget *>(QStringLiteral("studioSceneHierarchyTree"));
  auto * left_tabs = findTabs(left_panel, {QStringLiteral("Scene"), QStringLiteral("Assets"), QStringLiteral("Files")});
  auto * right_tabs = findTabs(right_panel, {QStringLiteral("Selection"), QStringLiteral("Workflow"), QStringLiteral("Actions"), QStringLiteral("Readiness")});
  QLabel * workflow_title = findLabelContaining(right_panel, QStringLiteral("Scene Builder Workflow"));
  QFrame * asset_card = containingCard(asset_search);
  QFrame * workflow_card = containingCard(workflow_title);
  if (!splitter || !left_panel || !right_panel || !left_tabs || !right_tabs ||
      !asset_card || !workflow_card || !hierarchy_tree) return;

  const int assets_index = tabIndexByText(left_tabs, QStringLiteral("Assets"));
  const int workflow_index = tabIndexByText(right_tabs, QStringLiteral("Workflow"));
  QWidget * assets_page = assets_index >= 0 ? left_tabs->widget(assets_index) : nullptr;
  QWidget * workflow_page = workflow_index >= 0 ? right_tabs->widget(workflow_index) : nullptr;
  auto * assets_layout = assets_page ? qobject_cast<QBoxLayout *>(assets_page->layout()) : nullptr;
  auto * workflow_layout = workflow_page ? qobject_cast<QBoxLayout *>(workflow_page->layout()) : nullptr;
  if (!assets_layout || !workflow_layout) return;

  // The Scene tab is for hierarchy/layers. The catalog belongs in the Assets tab.
  assets_layout->addWidget(asset_card, 1);
  asset_card->show();
  for (QLabel * label : assets_page->findChildren<QLabel *>()) {
    if (label->text().contains(QStringLiteral("available below Scene Hierarchy"), Qt::CaseInsensitive)) label->hide();
  }

  // Put workflow guidance inside its existing Workflow tab instead of stacking it
  // above the inspector and reducing every right-panel tab to a narrow strip.
  workflow_layout->insertWidget(0, workflow_card);
  workflow_card->show();

  left_panel->setMinimumWidth(300);
  right_panel->setMinimumWidth(360);
  left_tabs->setUsesScrollButtons(true);
  left_tabs->setElideMode(Qt::ElideRight);
  right_tabs->setUsesScrollButtons(true);
  right_tabs->setElideMode(Qt::ElideRight);

  // These actions already exist in the global Workcell Studio header. Keep the
  // local header focused on the selected scene identity and path.
  for (const QString & name : {
      QStringLiteral("sceneBuilderHeaderFilesButton"),
      QStringLiteral("sceneBuilderHeaderSaveLayoutButton"),
      QStringLiteral("sceneBuilderHeaderRunNextButton")}) {
    if (QWidget * duplicate = workspace->findChild<QWidget *>(name)) duplicate->hide();
  }

  for (QLabel * label : workspace->findChildren<QLabel *>()) {
    const QString text = label->text().trimmed();
    if ((text.startsWith(QStringLiteral("Mode:")) && text.contains(QStringLiteral("Scene3D Product Preview"))) ||
        text.startsWith(QStringLiteral("Legend:"))) {
      label->hide();
    }
  }

  auto * panel_button = workspace->findChild<QToolButton *>(QStringLiteral("scene_builder_secondary_overflow_button"));
  addPanelActionsToTop(panel_button);
  QAction * show_left = panel_button ? panel_button->findChild<QAction *>(QStringLiteral("sceneBuilderShowLeftPanelAction")) : nullptr;
  QAction * show_right = panel_button ? panel_button->findChild<QAction *>(QStringLiteral("sceneBuilderShowRightPanelAction")) : nullptr;
  QAction * focus = panel_button ? panel_button->findChild<QAction *>(QStringLiteral("sceneBuilderFocus3dViewAction")) : nullptr;

  // One-time migration from the old three-panel-at-once default. Keep hierarchy
  // available, give the Product View the majority of the window, and open the
  // inspector only when the operator explicitly asks for it.
  QSettings settings;
  constexpr int kFocusLayoutVersion = 1;
  if (settings.value(QStringLiteral("scene_builder/focus_layout_version"), 0).toInt() < kFocusLayoutVersion) {
    if (focus) focus->setChecked(false);
    if (show_left) show_left->setChecked(true);
    else left_panel->show();
    if (show_right) show_right->setChecked(false);
    else right_panel->hide();
    const QList<int> old_sizes = splitter->sizes();
    int total = 0;
    for (const int size : old_sizes) total += size;
    total = qMax(total, 1200);
    const int center_width = qMax(760, total - 320);
    splitter->setSizes({320, center_width, 0});
    QVariantList migrated_sizes;
    migrated_sizes << 320 << center_width << 0;
    settings.setValue(QStringLiteral("scene_builder/main_splitter_sizes"), migrated_sizes);
    settings.setValue(QStringLiteral("scene_builder/left_panel_visible"), true);
    settings.setValue(QStringLiteral("scene_builder/right_panel_visible"), false);
    settings.setValue(QStringLiteral("scene_builder/focus_layout_version"), kFocusLayoutVersion);
  }

  const auto revealSelection = [right_panel, right_tabs, show_right](const QString & selected_id) {
    if (selected_id.trimmed().isEmpty()) return;
    if (show_right) show_right->setChecked(true);
    else right_panel->show();
    const int selection_index = tabIndexByText(right_tabs, QStringLiteral("Selection"));
    if (selection_index >= 0) right_tabs->setCurrentIndex(selection_index);
  };
  QObject::connect(hierarchy_tree, &QTreeWidget::itemClicked, workspace,
    [revealSelection](QTreeWidgetItem * item, int) {
      if (item) revealSelection(item->text(0));
    });
  if (auto * preview = workspace->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"))) {
    QObject::connect(preview, &ScenePreviewWidget::preview_item_selected, workspace,
      [revealSelection](const QString & id, const QString &) {
        if (QApplication::mouseButtons() != Qt::NoButton) revealSelection(id);
      });
  }

  workspace->setProperty("workcell_scene_builder_focus_layout_v1", true);
}
}  // namespace scene_builder_focus_layout_detail

inline void bootstrapSceneBuilderFocusLayout()
{
  if (!qApp || qApp->property("workcell_scene_builder_focus_layout_bootstrap").toBool()) return;
  qApp->setProperty("workcell_scene_builder_focus_layout_bootstrap", true);
  auto * timer = new QTimer(qApp);
  timer->setInterval(400);
  QObject::connect(timer, &QTimer::timeout, qApp, []() {
    for (QWidget * window : QApplication::topLevelWidgets()) {
      scene_builder_focus_layout_detail::installSceneBuilderFocusLayout(window);
    }
  });
  timer->start();
  QTimer::singleShot(0, qApp, []() {
    for (QWidget * window : QApplication::topLevelWidgets()) {
      scene_builder_focus_layout_detail::installSceneBuilderFocusLayout(window);
    }
  });
}
}  // namespace workcell_builder

inline void workcellBuilderBootstrapSceneBuilderFocusLayout()
{
  workcell_builder::bootstrapSceneBuilderFocusLayout();
}
Q_COREAPP_STARTUP_FUNCTION(workcellBuilderBootstrapSceneBuilderFocusLayout)
