#pragma once

#include <QAction>
#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QStyle>
#include <QTableWidget>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

#include <functional>

#include "home_workcells_target_shell.hpp"

namespace workcell_builder
{
namespace home_workcells
{

inline QString contextual_page_name(int index)
{
  switch (index) {
    case 1: return QStringLiteral("Scene Builder");
    case 2: return QStringLiteral("Scenes");
    case 3: return QStringLiteral("Demo");
    case 4: return QStringLiteral("Plan & Simulate");
    case 5: return QStringLiteral("Diagnostics");
    case 6: return QStringLiteral("Validation");
    case 7: return QStringLiteral("Export");
    default: return QStringLiteral("Workcell Studio");
  }
}

inline QStackedWidget * contextual_pages(QMainWindow * window)
{
  QWidget * dashboard = window ? window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage")) : nullptr;
  return dashboard ? qobject_cast<QStackedWidget *>(dashboard->parentWidget()) : nullptr;
}

inline void hide_all_legacy_studio_toolbars(QMainWindow * window)
{
  if (!window) return;

  // The target shell owns navigation.  The legacy toolbar may be nested under
  // central/UI hosts rather than parented directly to QMainWindow, so hide every
  // toolbar recursively instead of relying on fragile object names/parentage.
  for (QToolBar * toolbar : window->findChildren<QToolBar *>()) {
    if (!toolbar) continue;
    toolbar->hide();
    toolbar->setVisible(false);
    toolbar->setMaximumHeight(0);
    toolbar->setMinimumHeight(0);
  }
}

inline void set_context_status_kind(QLabel * label, const QString & status)
{
  if (!label) return;
  QString kind = QStringLiteral("neutral");
  if (status == QStringLiteral("Ready")) kind = QStringLiteral("ready");
  else if (status == QStringLiteral("Needs Attention")) kind = QStringLiteral("attention");
  else if (status == QStringLiteral("Blocked")) kind = QStringLiteral("blocked");
  label->setProperty("kind", kind);
  label->style()->unpolish(label);
  label->style()->polish(label);
}

inline void simplify_home_inspector(QMainWindow * window)
{
  if (!window) return;
  // Home owns its visible action hierarchy. Contextual navigation must not
  // hide or rewrite those controls when the dashboard becomes active.
  if (auto * more = window->findChild<QToolButton *>(QStringLiteral("studioTargetInspectorMore"))) more->show();
}

inline void refresh_context_header(QMainWindow * window, QStackedWidget * pages, QTableWidget * table)
{
  if (!window || !pages) return;
  const int page_index = pages->currentIndex();
  const bool home_page = page_index == 0;

  // Hard shell policy:
  //   Home        -> left rail only, no top navigation/header.
  //   Other pages -> compact context header only, no left rail.
  hide_all_legacy_studio_toolbars(window);
  if (auto * sidebar = window->findChild<QFrame *>(QStringLiteral("studioTargetSidebar"))) sidebar->setVisible(home_page);
  if (auto * old_topbar = window->findChild<QFrame *>(QStringLiteral("studioTargetTopbar"))) {
    old_topbar->hide();
    old_topbar->setVisible(false);
    old_topbar->setMaximumHeight(0);
    old_topbar->setMinimumHeight(0);
  }

  auto * context = window->findChild<QFrame *>(QStringLiteral("studioContextTopbar"));
  if (context) context->setVisible(!home_page);
  if (home_page) {
    simplify_home_inspector(window);
    return;
  }

  const int row = table ? table->currentRow() : -1;
  const bool selected = table && row >= 0 && row < table->rowCount() && table->item(row, 0);
  const QString scene_id = selected ? scene_id_at(table, row) : QString();
  const QString scene_title = selected ? friendly_workcell_name(scene_id) : QStringLiteral("Workcell Studio");
  const QString status = selected ? scene_status_at(table, row) : QStringLiteral("No scene selected");

  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextSceneTitle"))) {
    label->setText(scene_title);
    label->setToolTip(scene_id);
  }
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextPage"))) label->setText(contextual_page_name(page_index));
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextStatus"))) {
    label->setText(status);
    set_context_status_kind(label, status);
  }
}

inline void install_contextual_navigation(
  QMainWindow * window, const std::function<void()> & navigate_home)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("studioContextNavigationInstalled").toBool()) return;

  QStackedWidget * pages = contextual_pages(window);
  QTableWidget * table = scene_table(window);
  auto * right_shell = window->findChild<QWidget *>(QStringLiteral("studioTargetRightShell"));
  auto * right_layout = right_shell ? qobject_cast<QVBoxLayout *>(right_shell->layout()) : nullptr;
  if (!pages || !table || !right_shell || !right_layout) return;
  window->setProperty("studioContextNavigationInstalled", true);

  // Retire the target shell's first-generation safety topbar permanently. It is
  // intentionally replaced by the contextual bar below on non-Home pages.
  if (auto * old_topbar = window->findChild<QFrame *>(QStringLiteral("studioTargetTopbar"))) {
    old_topbar->hide();
    old_topbar->setVisible(false);
    old_topbar->setMaximumHeight(0);
    old_topbar->setMinimumHeight(0);
  }
  hide_all_legacy_studio_toolbars(window);

  auto * context = new QFrame(right_shell);
  context->setObjectName(QStringLiteral("studioContextTopbar"));
  context->setFixedHeight(40);
  auto * row = new QHBoxLayout(context);
  row->setContentsMargins(10, 4, 12, 4);
  row->setSpacing(8);

  auto * home = new QToolButton(context);
  home->setObjectName(QStringLiteral("studioContextHome"));
  home->setText(QStringLiteral("← Workcells"));
  home->setToolTip(QStringLiteral("Back to Your workcells"));
  row->addWidget(home);

  auto * scene_title = new QLabel(QStringLiteral("Workcell Studio"), context);
  scene_title->setObjectName(QStringLiteral("studioContextSceneTitle"));
  row->addWidget(scene_title);

  auto * divider = new QLabel(QStringLiteral("/"), context);
  divider->setObjectName(QStringLiteral("studioContextDivider"));
  row->addWidget(divider);

  auto * page = new QLabel(QStringLiteral("Scene Builder"), context);
  page->setObjectName(QStringLiteral("studioContextPage"));
  row->addWidget(page);

  row->addStretch(1);

  auto * status = new QLabel(QStringLiteral("No scene selected"), context);
  status->setObjectName(QStringLiteral("studioContextStatus"));
  status->setProperty("kind", QStringLiteral("neutral"));
  row->addWidget(status);

  auto * fake = new QLabel(QStringLiteral("●  FAKE HARDWARE"), context);
  fake->setObjectName(QStringLiteral("studioContextFake"));
  fake->setToolTip(QStringLiteral("Simulation mode. Real hardware execution is not active."));
  row->addWidget(fake);

  auto * locked = new QLabel(QStringLiteral("▣  REAL LOCKED"), context);
  locked->setObjectName(QStringLiteral("studioContextLocked"));
  locked->setToolTip(QStringLiteral("Real hardware remains locked."));
  row->addWidget(locked);

  const int pages_index = right_layout->indexOf(pages);
  right_layout->insertWidget(qMax(0, pages_index), context);

  QObject::connect(home, &QToolButton::clicked, window, [navigate_home]() {
    if (navigate_home) navigate_home();
  });
  QObject::connect(pages, &QStackedWidget::currentChanged, context,
    [window, pages, table](int) { refresh_context_header(window, pages, table); });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, context,
    [window, pages, table]() { refresh_context_header(window, pages, table); });
  QObject::connect(table, &QTableWidget::cellClicked, context,
    [window, pages, table](int, int) { refresh_context_header(window, pages, table); });

  right_shell->setStyleSheet(right_shell->styleSheet() + QStringLiteral(
    "QFrame#studioContextTopbar{background:#FFFFFF;border:0;border-bottom:1px solid #DCE5EF;}"
    "QToolButton#studioContextHome{background:transparent;color:#17446F;border:0;border-radius:5px;padding:4px 7px;font-size:10px;font-weight:750;}"
    "QToolButton#studioContextHome:hover{background:#EDF4FB;}"
    "QLabel#studioContextSceneTitle{color:#102B4E;font-size:11px;font-weight:850;}"
    "QLabel#studioContextDivider{color:#A1AFBD;font-size:10px;}"
    "QLabel#studioContextPage{color:#647B92;font-size:10px;font-weight:650;}"
    "QLabel#studioContextStatus{border-radius:8px;padding:3px 7px;font-size:8px;font-weight:800;}"
    "QLabel#studioContextStatus[kind=neutral]{background:#F1F4F7;color:#63788D;}"
    "QLabel#studioContextStatus[kind=ready]{background:#ECF8F1;color:#147A49;}"
    "QLabel#studioContextStatus[kind=attention]{background:#FFF5E8;color:#C66708;}"
    "QLabel#studioContextStatus[kind=blocked]{background:#FDEEEF;color:#BC2B34;}"
    "QLabel#studioContextFake{background:#F0F9F4;color:#137A48;border:1px solid #D2E9DC;border-radius:8px;padding:3px 7px;font-size:8px;font-weight:800;}"
    "QLabel#studioContextLocked{background:#F6F8FB;color:#234466;border:1px solid #DCE5EF;border-radius:8px;padding:3px 7px;font-size:8px;font-weight:800;}"));

  simplify_home_inspector(window);
  refresh_context_header(window, pages, table);
}

}  // namespace home_workcells
}  // namespace workcell_builder
