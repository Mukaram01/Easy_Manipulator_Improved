#pragma once

#include <QAction>
#include <QApplication>
#include <QDynamicPropertyChangeEvent>
#include <QEvent>
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
#include <utility>

#include "home_workcells_target_shell.hpp"
#include "workcell_builder_ui_utils.hpp"

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

  // The open cell (published by MainWindow) wins over the Home table selection:
  // a cell can be opened without touching that table, and the header must name it.
  int row = table ? table->currentRow() : -1;
  const QString opened_scene_id = window->property("studioOpenedSceneId").toString();
  if (table && !opened_scene_id.isEmpty() && scene_id_at(table, row) != opened_scene_id) {
    row = -1;
    for (int r = 0; r < table->rowCount(); ++r) if (scene_id_at(table, r) == opened_scene_id) { row = r; break; }
  }
  const bool selected = table && row >= 0 && row < table->rowCount() && table->item(row, 0);
  const QString scene_id = selected ? scene_id_at(table, row) : QString();
  // The same name as the Home row: the cell's display name, else the friendly id.
  const QString display_name = selected ? table->item(row, 0)->data(kDisplayNameRole).toString().trimmed() : QString();
  const QString scene_title = !selected ? QStringLiteral("Workcell Studio") :
    (display_name.isEmpty() || display_name == scene_id ? friendly_workcell_name(scene_id) : display_name);
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

// Refreshes the header when MainWindow publishes a newly opened cell.
class OpenedSceneWatcher : public QObject
{
public:
  OpenedSceneWatcher(QObject * parent, std::function<void()> refresh)
  : QObject(parent), refresh_(std::move(refresh)) {}

  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (event->type() == QEvent::DynamicPropertyChange &&
      static_cast<QDynamicPropertyChangeEvent *>(event)->propertyName() == "studioOpenedSceneId" && refresh_) {
      refresh_();
    }
    return QObject::eventFilter(watched, event);
  }

private:
  std::function<void()> refresh_;
};

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

  // Outcome of the last Save / Validate action. MainWindow::show_workflow_feedback
  // fills it; it stays hidden until there is something to report.
  auto * feedback = new QLabel(context);
  feedback->setObjectName(QStringLiteral("studioContextFeedback"));
  feedback->setProperty("kind", QStringLiteral("success"));
  feedback->setTextInteractionFlags(Qt::TextSelectableByMouse);
  feedback->hide();
  row->addSpacing(6);
  row->addWidget(feedback);

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
  window->installEventFilter(new OpenedSceneWatcher(context,
    [window, pages, table]() { refresh_context_header(window, pages, table); }));

  right_shell->setStyleSheet(right_shell->styleSheet() + studioQss(QStringLiteral(R"QSS(
    QFrame#studioContextTopbar { background: @glass; border: 0; border-bottom: 1px solid @glassBorder; }
    QToolButton#studioContextHome { background: transparent; color: @accent; border: 0; border-radius: @radiusS; padding: 4px 8px; font-size: 12px; font-weight: 650; }
    QToolButton#studioContextHome:hover { background: @accentSoft; }
    QLabel#studioContextSceneTitle { color: @text; font-size: 13px; font-weight: 700; }
    QLabel#studioContextDivider { color: @textMuted; font-size: 12px; }
    QLabel#studioContextPage { color: @textSecondary; font-size: 12px; font-weight: 600; }
    QLabel#studioContextFeedback { border-radius: @radiusS; padding: 4px 10px; font-size: 12px; font-weight: 600; }
    QLabel#studioContextFeedback[kind=success] { background: @successSoft; color: @success; border: 1px solid @success; }
    QLabel#studioContextFeedback[kind=warning] { background: @warningSoft; color: @warning; border: 1px solid @warning; }
    QLabel#studioContextFeedback[kind=error] { background: @dangerSoft; color: @danger; border: 1px solid @danger; }
    QLabel#studioContextStatus { border-radius: 9px; padding: 3px 9px; font-size: 10px; font-weight: 750; }
    QLabel#studioContextStatus[kind=neutral] { background: @surfaceSunken; color: @textSecondary; }
    QLabel#studioContextStatus[kind=ready] { background: @successSoft; color: @success; }
    QLabel#studioContextStatus[kind=attention] { background: @warningSoft; color: @warning; }
    QLabel#studioContextStatus[kind=blocked] { background: @dangerSoft; color: @danger; }
    QLabel#studioContextFake { background: @successSoft; color: @success; border: 1px solid @border; border-radius: 9px; padding: 3px 9px; font-size: 10px; font-weight: 750; }
    QLabel#studioContextLocked { background: @surfaceSunken; color: @textSecondary; border: 1px solid @border; border-radius: 9px; padding: 3px 9px; font-size: 10px; font-weight: 750; }
  )QSS")));

  simplify_home_inspector(window);
  refresh_context_header(window, pages, table);
}

}  // namespace home_workcells
}  // namespace workcell_builder
