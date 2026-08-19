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

inline QToolBar * legacy_studio_toolbar(QMainWindow * window)
{
  if (!window) return nullptr;
  for (QToolBar * toolbar : window->findChildren<QToolBar *>(QString(), Qt::FindDirectChildrenOnly)) {
    if (!toolbar) continue;
    int matches = 0;
    for (QAction * action : toolbar->actions()) {
      if (!action) continue;
      const QString text = action->text().remove('&').trimmed();
      if (text.contains(QStringLiteral("Studio Home"), Qt::CaseInsensitive) ||
          text.contains(QStringLiteral("New Cell"), Qt::CaseInsensitive) ||
          text.contains(QStringLiteral("Run Next"), Qt::CaseInsensitive) ||
          text.contains(QStringLiteral("Product View"), Qt::CaseInsensitive)) ++matches;
    }
    if (matches >= 2) return toolbar;
  }
  return nullptr;
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
  for (QPushButton * button : window->findChildren<QPushButton *>(QStringLiteral("studioTargetPrimaryAction"))) button->hide();
  for (QPushButton * button : window->findChildren<QPushButton *>(QStringLiteral("studioTargetSecondaryAction"))) button->hide();
  for (QPushButton * button : window->findChildren<QPushButton *>(QStringLiteral("studioTargetSimulateAction"))) button->hide();
  if (auto * more = window->findChild<QToolButton *>(QStringLiteral("studioTargetInspectorMore"))) more->hide();
  if (auto * readiness = window->findChild<QLabel *>(QStringLiteral("studioTargetReadiness"))) {
    QString text = readiness->text();
    text.replace(QStringLiteral("\nView details →"), QString());
    readiness->setText(text);
  }
}

inline void refresh_context_header(QMainWindow * window, QStackedWidget * pages, QTableWidget * table)
{
  if (!window || !pages) return;
  const int page_index = pages->currentIndex();
  const bool home_page = page_index == 0;

  if (auto * sidebar = window->findChild<QFrame *>(QStringLiteral("studioTargetSidebar"))) sidebar->setVisible(home_page);
  if (auto * old_topbar = window->findChild<QFrame *>(QStringLiteral("studioTargetTopbar"))) old_topbar->hide();
  if (QToolBar * legacy = legacy_studio_toolbar(window)) legacy->hide();

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
  const QString robot = selected && table->item(row, 2) ? clean_robot(table->item(row, 2)->text()) : QStringLiteral("—");
  const QString tool = selected && table->item(row, 3) ? clean_tool(table->item(row, 3)->text()) : QStringLiteral("—");

  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextSceneTitle"))) label->setText(scene_title);
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextSceneId"))) label->setText(scene_id);
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextPage"))) label->setText(contextual_page_name(page_index));
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextStatus"))) {
    label->setText(status);
    set_context_status_kind(label, status);
  }
  if (auto * label = window->findChild<QLabel *>(QStringLiteral("studioContextRobotTool"))) {
    QStringList pieces;
    if (robot != QStringLiteral("—") && robot.compare(QStringLiteral("Unknown"), Qt::CaseInsensitive) != 0) pieces << robot;
    if (tool != QStringLiteral("—") && tool.compare(QStringLiteral("Unknown"), Qt::CaseInsensitive) != 0) pieces << tool;
    label->setText(pieces.join(QStringLiteral("  ·  ")));
    label->setVisible(!pieces.isEmpty());
  }
}

inline void install_contextual_navigation(QMainWindow * window)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (window->property("studioContextNavigationInstalled").toBool()) return;

  QStackedWidget * pages = contextual_pages(window);
  QTableWidget * table = scene_table(window);
  auto * right_shell = window->findChild<QWidget *>(QStringLiteral("studioTargetRightShell"));
  auto * right_layout = right_shell ? qobject_cast<QVBoxLayout *>(right_shell->layout()) : nullptr;
  if (!pages || !table || !right_shell || !right_layout) return;
  window->setProperty("studioContextNavigationInstalled", true);

  auto * context = new QFrame(right_shell);
  context->setObjectName(QStringLiteral("studioContextTopbar"));
  context->setFixedHeight(46);
  auto * row = new QHBoxLayout(context);
  row->setContentsMargins(14, 5, 16, 5);
  row->setSpacing(9);

  auto * home = new QToolButton(context);
  home->setObjectName(QStringLiteral("studioContextHome"));
  home->setText(QStringLiteral("←  Home"));
  home->setToolTip(QStringLiteral("Back to Your workcells"));
  row->addWidget(home);

  auto * identity = new QWidget(context);
  auto * identity_layout = new QVBoxLayout(identity);
  identity_layout->setContentsMargins(2, 0, 8, 0);
  identity_layout->setSpacing(0);
  auto * scene_title = new QLabel(QStringLiteral("Workcell Studio"), identity);
  scene_title->setObjectName(QStringLiteral("studioContextSceneTitle"));
  auto * scene_id = new QLabel(identity);
  scene_id->setObjectName(QStringLiteral("studioContextSceneId"));
  identity_layout->addWidget(scene_title);
  identity_layout->addWidget(scene_id);
  row->addWidget(identity);

  auto * page = new QLabel(QStringLiteral("Scene Builder"), context);
  page->setObjectName(QStringLiteral("studioContextPage"));
  row->addWidget(page);

  auto * status = new QLabel(QStringLiteral("No scene selected"), context);
  status->setObjectName(QStringLiteral("studioContextStatus"));
  status->setProperty("kind", QStringLiteral("neutral"));
  row->addWidget(status);
  row->addStretch(1);

  auto * robot_tool = new QLabel(context);
  robot_tool->setObjectName(QStringLiteral("studioContextRobotTool"));
  row->addWidget(robot_tool);

  auto * fake = new QLabel(QStringLiteral("●  FAKE HARDWARE"), context);
  fake->setObjectName(QStringLiteral("studioContextFake"));
  row->addWidget(fake);
  auto * locked = new QLabel(QStringLiteral("▣  REAL HARDWARE LOCKED"), context);
  locked->setObjectName(QStringLiteral("studioContextLocked"));
  row->addWidget(locked);

  const int pages_index = right_layout->indexOf(pages);
  right_layout->insertWidget(qMax(0, pages_index), context);

  QObject::connect(home, &QToolButton::clicked, pages, [pages]() { pages->setCurrentIndex(0); });
  QObject::connect(pages, &QStackedWidget::currentChanged, context,
    [window, pages, table](int) { refresh_context_header(window, pages, table); });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, context,
    [window, pages, table]() { refresh_context_header(window, pages, table); });
  QObject::connect(table, &QTableWidget::cellClicked, context,
    [window, pages, table](int, int) { refresh_context_header(window, pages, table); });

  right_shell->setStyleSheet(right_shell->styleSheet() + QStringLiteral(
    "QFrame#studioContextTopbar{background:#FFFFFF;border:0;border-bottom:1px solid #DCE5EF;}"
    "QToolButton#studioContextHome{background:#F7F9FC;color:#17446F;border:1px solid #D8E3ED;border-radius:6px;padding:5px 10px;font-weight:750;}"
    "QToolButton#studioContextHome:hover{background:#EDF4FB;border-color:#B9CEE2;}"
    "QLabel#studioContextSceneTitle{color:#102B4E;font-size:12px;font-weight:850;}"
    "QLabel#studioContextSceneId{color:#7A8DA1;font-size:8px;}"
    "QLabel#studioContextPage{color:#526C86;font-size:10px;font-weight:700;padding:0 8px;border-left:1px solid #DCE5EF;}"
    "QLabel#studioContextStatus{border-radius:9px;padding:4px 8px;font-size:9px;font-weight:800;}"
    "QLabel#studioContextStatus[kind=neutral]{background:#F1F4F7;color:#63788D;}"
    "QLabel#studioContextStatus[kind=ready]{background:#ECF8F1;color:#147A49;}"
    "QLabel#studioContextStatus[kind=attention]{background:#FFF5E8;color:#C66708;}"
    "QLabel#studioContextStatus[kind=blocked]{background:#FDEEEF;color:#BC2B34;}"
    "QLabel#studioContextRobotTool{color:#536C86;font-size:9px;font-weight:650;}"
    "QLabel#studioContextFake{background:#F0F9F4;color:#137A48;border:1px solid #D2E9DC;border-radius:10px;padding:4px 8px;font-size:8px;font-weight:800;}"
    "QLabel#studioContextLocked{background:#F6F8FB;color:#234466;border:1px solid #DCE5EF;border-radius:10px;padding:4px 8px;font-size:8px;font-weight:800;}"));

  simplify_home_inspector(window);
  refresh_context_header(window, pages, table);
}

}  // namespace home_workcells
}  // namespace workcell_builder
