#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractScrollArea>
#include <QAbstractSpinBox>
#include <QAction>
#include <QApplication>
#include <QBoxLayout>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QFrame>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHeaderView>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListView>
#include <QListWidget>
#include <QMainWindow>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScreen>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabBar>
#include <QTabWidget>
#include <QTableView>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QTreeView>
#include <QVariant>
#include <QVBoxLayout>
#include <QWindow>

#include <algorithm>

namespace workcell_builder
{
namespace
{
constexpr int kHomeSceneColumn = 0;
constexpr int kHomeStatusColumn = 1;
constexpr int kHomeRobotColumn = 2;
constexpr int kHomeToolColumn = 3;
constexpr int kHomeTaskColumn = 4;
constexpr int kHomeLaunchColumn = 5;
constexpr int kHomeUpdatedColumn = 6;
constexpr int kHomePinColumn = 7;
constexpr int kHomeColumnCount = 8;

QScreen * screenForWidget(QWidget * widget)
{
  if (widget && widget->windowHandle() && widget->windowHandle()->screen()) {
    return widget->windowHandle()->screen();
  }
  return QGuiApplication::primaryScreen();
}

QRect usableScreenGeometry(QWidget * widget)
{
  QScreen * screen = screenForWidget(widget);
  if (!screen) return QRect(0, 0, 1280, 720);

  QRect available = screen->availableGeometry();
  const int inset = std::min(16, std::min(available.width(), available.height()) / 20);
  return available.adjusted(inset, inset, -inset, -inset);
}

void keepWindowInsideScreen(QWidget * widget)
{
  if (!widget || widget->isFullScreen() || widget->isMaximized()) return;

  const QRect available = usableScreenGeometry(widget);
  if (!available.isValid()) return;

  const QSize bounded_window_size(
    std::min(widget->width(), available.width()),
    std::min(widget->height(), available.height()));

  const QSize current_minimum = widget->minimumSize();
  widget->setMinimumSize(
    std::min(current_minimum.width(), available.width()),
    std::min(current_minimum.height(), available.height()));

  if (bounded_window_size != widget->size()) {
    widget->resize(bounded_window_size);
  }

  const QRect frame = widget->frameGeometry();
  const int max_x = std::max(available.left(), available.right() - frame.width() + 1);
  const int max_y = std::max(available.top(), available.bottom() - frame.height() + 1);
  const int bounded_x = std::max(available.left(), std::min(frame.left(), max_x));
  const int bounded_y = std::max(available.top(), std::min(frame.top(), max_y));
  if (bounded_x != frame.left() || bounded_y != frame.top()) {
    widget->move(bounded_x, bounded_y);
  }
}

class ResponsiveWindowGuard : public QObject
{
public:
  explicit ResponsiveWindowGuard(QWidget * window)
  : QObject(window), window_(window)
  {
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (watched == window_ && !adjusting_ &&
      (event->type() == QEvent::Show || event->type() == QEvent::Resize ||
      event->type() == QEvent::WindowStateChange))
    {
      adjusting_ = true;
      keepWindowInsideScreen(window_);
      adjusting_ = false;
    }
    return QObject::eventFilter(watched, event);
  }

private:
  QWidget * window_ = nullptr;
  bool adjusting_ = false;
};

void installResponsiveWindowGuard(QWidget * widget)
{
  if (!widget || widget->property("workcell_responsive_window_guard").toBool()) return;
  widget->installEventFilter(new ResponsiveWindowGuard(widget));
  widget->setProperty("workcell_responsive_window_guard", true);
}

void configureContainedWidgets(QWidget * widget)
{
  if (!widget) return;

  for (QLayout * layout : widget->findChildren<QLayout *>()) {
    const QMargins margins = layout->contentsMargins();
    layout->setContentsMargins(
      std::min(margins.left(), 12),
      std::min(margins.top(), 12),
      std::min(margins.right(), 12),
      std::min(margins.bottom(), 12));
    if (layout->spacing() > 10) layout->setSpacing(8);
  }

  for (QScrollArea * area : widget->findChildren<QScrollArea *>()) {
    area->setWidgetResizable(true);
    area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    area->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    area->setFrameShape(QFrame::NoFrame);
    area->setMinimumWidth(0);
    area->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  for (QAbstractItemView * view : widget->findChildren<QAbstractItemView *>()) {
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    view->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    view->setTextElideMode(Qt::ElideRight);
    view->setMinimumWidth(0);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  for (QTableView * table : widget->findChildren<QTableView *>()) {
    if (table->horizontalHeader()) {
      table->horizontalHeader()->setStretchLastSection(true);
      table->horizontalHeader()->setMinimumSectionSize(44);
    }
    table->setWordWrap(true);
  }

  for (QTextEdit * editor : widget->findChildren<QTextEdit *>()) {
    editor->setLineWrapMode(QTextEdit::WidgetWidth);
    editor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    editor->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    editor->setMinimumWidth(0);
  }

  for (QPlainTextEdit * editor : widget->findChildren<QPlainTextEdit *>()) {
    editor->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    editor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    editor->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    editor->setMinimumWidth(0);
  }

  for (QTabWidget * tabs : widget->findChildren<QTabWidget *>()) {
    tabs->setUsesScrollButtons(true);
    tabs->setElideMode(Qt::ElideRight);
    tabs->setDocumentMode(true);
    tabs->setMinimumWidth(0);
    tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if (tabs->tabBar()) tabs->tabBar()->setExpanding(false);
  }

  for (QLineEdit * edit : widget->findChildren<QLineEdit *>()) {
    edit->setMinimumWidth(0);
    edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QComboBox * combo : widget->findChildren<QComboBox *>()) {
    combo->setMinimumWidth(0);
    combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    combo->setMinimumContentsLength(8);
    combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QAbstractSpinBox * spin : widget->findChildren<QAbstractSpinBox *>()) {
    spin->setMinimumWidth(0);
    spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QPushButton * button : widget->findChildren<QPushButton *>()) {
    button->setMinimumWidth(0);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  }
  for (QGroupBox * group : widget->findChildren<QGroupBox *>()) {
    group->setMinimumWidth(0);
    group->setSizePolicy(QSizePolicy::Expanding, group->sizePolicy().verticalPolicy());
  }
}

QString normalizedSceneName(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount()) return QString();
  auto * item = table->item(row, kHomeSceneColumn);
  if (!item) return QString();
  const QString tooltip = item->toolTip().trimmed();
  return tooltip.isEmpty() ? item->text().trimmed() : tooltip;
}

QStringList pinnedHomeScenes()
{
  QSettings settings;
  QStringList values = settings.value(QStringLiteral("studio_home/pinned_scenes")).toStringList();
  QStringList normalized;
  for (const QString & value : values) {
    const QString scene = value.trimmed();
    if (!scene.isEmpty() && !normalized.contains(scene)) normalized.append(scene);
  }
  return normalized;
}

void savePinnedHomeScenes(const QStringList & scenes)
{
  QSettings settings;
  settings.setValue(QStringLiteral("studio_home/pinned_scenes"), scenes);
}

QAction * findStudioAction(QObject * root, const QStringList & exact_texts)
{
  if (!root) return nullptr;
  for (QAction * action : root->findChildren<QAction *>()) {
    if (!action) continue;
    QString action_text = action->text();
    action_text.remove('&');
    for (const QString & expected : exact_texts) {
      if (action_text.compare(expected, Qt::CaseInsensitive) == 0) return action;
    }
  }
  return nullptr;
}

void triggerStudioAction(QObject * root, const QStringList & exact_texts)
{
  if (QAction * action = findStudioAction(root, exact_texts)) {
    if (action->isEnabled()) action->trigger();
  }
}

QString homeSceneRoot(QMainWindow * window)
{
  if (!window) return QString();
  auto * summary = window->findChild<QLabel *>(QStringLiteral("dashboardSummaryLabel"));
  if (!summary) return QString();
  const QRegularExpression re(QStringLiteral("Source:\\s*([^|\\n]+)"));
  const auto match = re.match(summary->text());
  return match.hasMatch() ? match.captured(1).trimmed() : QString();
}

QDateTime sceneLastUpdated(const QString & scene_root, const QString & scene_name)
{
  const QDir scene_dir(QDir(scene_root).filePath(scene_name));
  QDateTime newest = QFileInfo(scene_dir.absolutePath()).lastModified();
  const QStringList candidates = {
    QStringLiteral("scene_manifest.yaml"),
    QStringLiteral("cell_definition.yaml"),
    QStringLiteral("environment.yaml"),
    QStringLiteral("layout/workcell_studio_layout.yaml"),
    QStringLiteral("generated/scene_visual_mesh_index.json")};
  for (const QString & relative : candidates) {
    const QFileInfo info(scene_dir.filePath(relative));
    if (info.exists() && info.lastModified().isValid() &&
      (!newest.isValid() || info.lastModified() > newest))
    {
      newest = info.lastModified();
    }
  }
  return newest;
}

void styleHomeStatusCell(QTableWidgetItem * item)
{
  if (!item) return;
  QString status = item->text().trimmed().toUpper();
  if (status.contains(QStringLiteral("READY"))) {
    item->setText(QStringLiteral("●  READY"));
    item->setForeground(QColor(QStringLiteral("#147A47")));
    item->setBackground(QColor(QStringLiteral("#EAF7F0")));
    item->setToolTip(QStringLiteral("Ready to open and continue the configured workflow."));
  } else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) {
    item->setText(QStringLiteral("▲  NEEDS ATTENTION"));
    item->setForeground(QColor(QStringLiteral("#B65A00")));
    item->setBackground(QColor(QStringLiteral("#FFF4E5")));
    item->setToolTip(QStringLiteral("Scene has warnings or incomplete readiness checks."));
  } else {
    item->setText(QStringLiteral("●  BLOCKED"));
    item->setForeground(QColor(QStringLiteral("#C53232")));
    item->setBackground(QColor(QStringLiteral("#FFF0F0")));
    item->setToolTip(QStringLiteral("Scene currently has a blocker that prevents the next safe workflow step."));
  }
}

void applyHomeExtraFilters(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table) return;
  auto * robot_filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeRobotFilter"));
  auto * tool_filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeToolFilter"));
  auto * favorites_only = window->findChild<QCheckBox *>(QStringLiteral("studioHomeFavoritesOnly"));
  const QString robot = robot_filter ? robot_filter->currentData().toString().trimmed() : QString();
  const QString tool = tool_filter ? tool_filter->currentData().toString().trimmed() : QString();
  const bool favorites = favorites_only && favorites_only->isChecked();
  const QStringList pinned = pinnedHomeScenes();

  for (int row = 0; row < table->rowCount(); ++row) {
    const QString scene = normalizedSceneName(table, row);
    const QString row_robot = table->item(row, kHomeRobotColumn) ? table->item(row, kHomeRobotColumn)->text().trimmed() : QString();
    const QString row_tool = table->item(row, kHomeToolColumn) ? table->item(row, kHomeToolColumn)->text().trimmed() : QString();
    const bool matches_robot = robot.isEmpty() || row_robot.compare(robot, Qt::CaseInsensitive) == 0;
    const bool matches_tool = tool.isEmpty() || row_tool.compare(tool, Qt::CaseInsensitive) == 0;
    const bool matches_favorite = !favorites || pinned.contains(scene);
    table->setRowHidden(row, !(matches_robot && matches_tool && matches_favorite));
  }
}

void refreshHomeStats(QMainWindow * window)
{
  if (!window) return;
  auto * raw_summary = window->findChild<QLabel *>(QStringLiteral("dashboardSummaryLabel"));
  int total = 0;
  int ready = 0;
  int warnings = 0;
  int blocked = 0;
  if (raw_summary) {
    const QRegularExpression re(
      QStringLiteral("Total scenes:\\s*(\\d+)\\s*\\|\\s*Ready:\\s*(\\d+)\\s*\\|\\s*Warnings:\\s*(\\d+)\\s*\\|\\s*Blocked/Scaffold:\\s*(\\d+)"));
    const auto match = re.match(raw_summary->text());
    if (match.hasMatch()) {
      total = match.captured(1).toInt();
      ready = match.captured(2).toInt();
      warnings = match.captured(3).toInt();
      blocked = match.captured(4).toInt();
    }
  }

  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (total == 0 && table) {
    total = table->rowCount();
    for (int row = 0; row < table->rowCount(); ++row) {
      const QString status = table->item(row, kHomeStatusColumn) ? table->item(row, kHomeStatusColumn)->text().toUpper() : QString();
      if (status.contains(QStringLiteral("READY"))) ++ready;
      else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) ++warnings;
      else ++blocked;
    }
  }

  for (QLabel * card : window->findChildren<QLabel *>(QStringLiteral("studioHomeSummaryCard"))) {
    if (!card) continue;
    QString kind = card->property("homeStatKind").toString();
    if (kind.isEmpty()) {
      const QString initial = card->text().toLower();
      if (initial.contains(QStringLiteral("total"))) kind = QStringLiteral("total");
      else if (initial.contains(QStringLiteral("ready"))) kind = QStringLiteral("ready");
      else if (initial.contains(QStringLiteral("warning"))) kind = QStringLiteral("attention");
      else kind = QStringLiteral("blocked");
      card->setProperty("homeStatKind", kind);
    }

    QString title;
    QString detail;
    QString accent;
    int value = 0;
    if (kind == QStringLiteral("total")) {
      title = QStringLiteral("TOTAL SCENES"); detail = QStringLiteral("Across all folders"); accent = QStringLiteral("#1D5DA8"); value = total;
    } else if (kind == QStringLiteral("ready")) {
      title = QStringLiteral("READY"); detail = QStringLiteral("Ready to simulate"); accent = QStringLiteral("#15803D"); value = ready;
    } else if (kind == QStringLiteral("attention")) {
      title = QStringLiteral("NEEDS ATTENTION"); detail = QStringLiteral("Warnings or minor issues"); accent = QStringLiteral("#C56A00"); value = warnings;
    } else {
      title = QStringLiteral("BLOCKED"); detail = QStringLiteral("Cannot run yet"); accent = QStringLiteral("#C53232"); value = blocked;
    }
    card->setTextFormat(Qt::RichText);
    card->setText(QStringLiteral(
      "<span style='font-size:11px;color:#64748B;font-weight:700'>%1</span><br/>"
      "<span style='font-size:26px;color:%2;font-weight:700'>%3</span><br/>"
      "<span style='font-size:11px;color:#8492A6'>%4</span>")
      .arg(title, accent).arg(value).arg(detail));
    card->setMinimumHeight(88);
  }
}

void populateHomeFilterOptions(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  auto * robot_filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeRobotFilter"));
  auto * tool_filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeToolFilter"));
  if (!table || !robot_filter || !tool_filter) return;

  const QString selected_robot = robot_filter->currentData().toString();
  const QString selected_tool = tool_filter->currentData().toString();
  QStringList robots;
  QStringList tools;
  for (int row = 0; row < table->rowCount(); ++row) {
    if (auto * item = table->item(row, kHomeRobotColumn)) {
      const QString value = item->text().trimmed();
      if (!value.isEmpty() && !robots.contains(value, Qt::CaseInsensitive)) robots.append(value);
    }
    if (auto * item = table->item(row, kHomeToolColumn)) {
      const QString value = item->text().trimmed();
      if (!value.isEmpty() && !tools.contains(value, Qt::CaseInsensitive)) tools.append(value);
    }
  }
  robots.sort(Qt::CaseInsensitive);
  tools.sort(Qt::CaseInsensitive);

  const QSignalBlocker robot_blocker(robot_filter);
  const QSignalBlocker tool_blocker(tool_filter);
  robot_filter->clear();
  robot_filter->addItem(QStringLiteral("Robot: All"), QString());
  for (const QString & value : robots) robot_filter->addItem(value, value);
  tool_filter->clear();
  tool_filter->addItem(QStringLiteral("Tool / Gripper: All"), QString());
  for (const QString & value : tools) tool_filter->addItem(value, value);
  const int robot_index = robot_filter->findData(selected_robot);
  const int tool_index = tool_filter->findData(selected_tool);
  if (robot_index >= 0) robot_filter->setCurrentIndex(robot_index);
  if (tool_index >= 0) tool_filter->setCurrentIndex(tool_index);
}

void applyHomeSort(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  auto * sort = window->findChild<QComboBox *>(QStringLiteral("studioHomeSort"));
  if (!table || !sort || table->rowCount() <= 1) return;
  const QString mode = sort->currentData().toString();
  table->setSortingEnabled(true);
  if (mode == QStringLiteral("name")) table->sortItems(kHomeSceneColumn, Qt::AscendingOrder);
  else if (mode == QStringLiteral("pinned")) table->sortItems(kHomePinColumn, Qt::AscendingOrder);
  else table->sortItems(kHomeUpdatedColumn, Qt::DescendingOrder);
  table->setSortingEnabled(false);
}

void refreshHomeInspector(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  auto * preview = window->findChild<QLabel *>(QStringLiteral("studioHomeScenePreview"));
  auto * readiness = window->findChild<QLabel *>(QStringLiteral("studioHomeReadinessSummary"));
  auto * pin_button = window->findChild<QPushButton *>(QStringLiteral("studioHomePinButton"));
  const int row = table ? table->currentRow() : -1;
  const bool has_scene = table && row >= 0 && row < table->rowCount();
  const QString scene = has_scene ? normalizedSceneName(table, row) : QString();

  for (const QString & button_name : {
      QStringLiteral("studioHomeOpenSceneButton"), QStringLiteral("studioHomeValidateButton"),
      QStringLiteral("studioHomeGenerateButton"), QStringLiteral("studioHomeSimulateButton")})
  {
    if (auto * button = window->findChild<QPushButton *>(button_name)) button->setEnabled(has_scene);
  }
  if (pin_button) {
    pin_button->setEnabled(has_scene);
    const bool pinned = has_scene && pinnedHomeScenes().contains(scene);
    pin_button->setText(pinned ? QStringLiteral("★  Pinned") : QStringLiteral("☆  Pin Favorite"));
  }

  if (!has_scene) {
    if (readiness) readiness->setText(QStringLiteral("<b>Readiness Summary</b><br/><span style='color:#64748B'>Select a scene to inspect readiness.</span>"));
    if (preview) {
      preview->setPixmap(QPixmap());
      preview->setText(QStringLiteral("Select a scene to preview"));
    }
    return;
  }

  const QString status = table->item(row, kHomeStatusColumn) ? table->item(row, kHomeStatusColumn)->text() : QStringLiteral("Unknown");
  const QString robot = table->item(row, kHomeRobotColumn) ? table->item(row, kHomeRobotColumn)->text() : QStringLiteral("unknown");
  const QString tool = table->item(row, kHomeToolColumn) ? table->item(row, kHomeToolColumn)->text() : QStringLiteral("unknown");
  if (readiness) {
    readiness->setText(QStringLiteral(
      "<b>Readiness Summary</b><br/><span style='color:#64748B'>%1</span><br/>"
      "<span style='color:#64748B'>Robot: %2 &nbsp; · &nbsp; Tool: %3</span>")
      .arg(status.toHtmlEscaped(), robot.toHtmlEscaped(), tool.toHtmlEscaped()));
  }

  if (preview) {
    const QString root = homeSceneRoot(window);
    const QDir scene_dir(QDir(root).filePath(scene));
    QString preview_path;
    const QStringList candidates = {
      QStringLiteral("preview/workcell_studio_canvas_snapshot.png"),
      QStringLiteral("preview/static_preview.png"),
      QStringLiteral("preview/scene3d_preview.png"),
      QStringLiteral("smoke/scene3d_smoke.png")};
    for (const QString & relative : candidates) {
      const QString candidate = scene_dir.filePath(relative);
      if (QFileInfo::exists(candidate)) { preview_path = candidate; break; }
    }
    if (preview_path.isEmpty()) {
      QDir preview_dir(scene_dir.filePath(QStringLiteral("preview")));
      const QStringList images = preview_dir.entryList({QStringLiteral("*.png"), QStringLiteral("*.jpg"), QStringLiteral("*.jpeg")}, QDir::Files, QDir::Time);
      if (!images.isEmpty()) preview_path = preview_dir.filePath(images.first());
    }
    const QPixmap image(preview_path);
    if (!image.isNull()) {
      preview->setText(QString());
      preview->setPixmap(image.scaled(
        std::max(280, preview->width()), 176,
        Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
      preview->setToolTip(preview_path);
    } else {
      preview->setPixmap(QPixmap());
      preview->setText(QStringLiteral("3D preview\nOpen Product View for the live scene"));
      preview->setToolTip(QStringLiteral("No cached preview image found for this scene."));
    }
  }
}

void refreshHomeTable(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table || table->property("workcell_home_refreshing").toBool()) return;
  table->setProperty("workcell_home_refreshing", true);

  table->setColumnCount(kHomeColumnCount);
  table->setHorizontalHeaderLabels({
    QStringLiteral("Scene"), QStringLiteral("Status"), QStringLiteral("Robot"),
    QStringLiteral("Tool / Gripper"), QStringLiteral("Task"), QStringLiteral("Launch"),
    QStringLiteral("Updated"), QStringLiteral("Pin")});
  table->setShowGrid(false);
  table->setAlternatingRowColors(false);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(54);
  table->setWordWrap(false);
  table->horizontalHeader()->setStretchLastSection(false);
  table->horizontalHeader()->setSectionResizeMode(kHomeSceneColumn, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(kHomeStatusColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kHomeRobotColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kHomeToolColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kHomeTaskColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kHomeLaunchColumn, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(kHomeUpdatedColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kHomePinColumn, QHeaderView::Fixed);
  table->setColumnWidth(kHomeUpdatedColumn, 132);
  table->setColumnWidth(kHomePinColumn, 48);

  const QString scene_root = homeSceneRoot(window);
  const QStringList pinned = pinnedHomeScenes();
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString scene = normalizedSceneName(table, row);
    if (auto * scene_item = table->item(row, kHomeSceneColumn)) {
      scene_item->setText(scene);
      scene_item->setToolTip(scene);
    }
    styleHomeStatusCell(table->item(row, kHomeStatusColumn));

    if (auto * task_item = table->item(row, kHomeTaskColumn)) {
      const QString task = task_item->text().trimmed().toLower();
      task_item->setText(task == QStringLiteral("present") ? QStringLiteral("Ready") : QStringLiteral("Missing"));
      task_item->setForeground(QColor(task == QStringLiteral("present") ? QStringLiteral("#334155") : QStringLiteral("#B65A00")));
    }
    if (auto * launch_item = table->item(row, kHomeLaunchColumn)) {
      const bool ready = launch_item->text().trimmed().compare(QStringLiteral("ready"), Qt::CaseInsensitive) == 0;
      launch_item->setText(ready ? QStringLiteral("Ready") : QStringLiteral("Blocked"));
      launch_item->setForeground(QColor(ready ? QStringLiteral("#147A47") : QStringLiteral("#C53232")));
    }

    auto * updated_item = table->item(row, kHomeUpdatedColumn);
    if (!updated_item) {
      updated_item = new QTableWidgetItem();
      table->setItem(row, kHomeUpdatedColumn, updated_item);
    }
    const QDateTime updated = sceneLastUpdated(scene_root, scene);
    updated_item->setText(updated.isValid() ? updated.toLocalTime().toString(QStringLiteral("yyyy-MM-dd HH:mm")) : QStringLiteral("—"));
    updated_item->setForeground(QColor(QStringLiteral("#64748B")));
    updated_item->setToolTip(updated.isValid() ? updated.toLocalTime().toString(Qt::DefaultLocaleLongDate) : QStringLiteral("No timestamp available"));

    auto * pin_item = table->item(row, kHomePinColumn);
    if (!pin_item) {
      pin_item = new QTableWidgetItem();
      table->setItem(row, kHomePinColumn, pin_item);
    }
    const bool is_pinned = pinned.contains(scene);
    pin_item->setText(is_pinned ? QStringLiteral("★") : QStringLiteral("☆"));
    pin_item->setTextAlignment(Qt::AlignCenter);
    pin_item->setForeground(QColor(is_pinned ? QStringLiteral("#1D5DA8") : QStringLiteral("#94A3B8")));
    pin_item->setToolTip(is_pinned ? QStringLiteral("Unpin favorite scene") : QStringLiteral("Pin favorite scene"));
  }

  table->setProperty("workcell_home_refreshing", false);
  populateHomeFilterOptions(window);
  applyHomeSort(window);
  applyHomeExtraFilters(window);
  refreshHomeStats(window);
  refreshHomeInspector(window);
}

void scheduleHomeRefresh(QMainWindow * window)
{
  if (!window || window->property("workcell_home_refresh_scheduled").toBool()) return;
  window->setProperty("workcell_home_refresh_scheduled", true);
  QTimer::singleShot(0, window, [window]() {
    window->setProperty("workcell_home_refresh_scheduled", false);
    refreshHomeTable(window);
  });
}

void toggleCurrentHomeFavorite(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table || table->currentRow() < 0) return;
  const QString scene = normalizedSceneName(table, table->currentRow());
  if (scene.isEmpty()) return;
  QStringList pinned = pinnedHomeScenes();
  if (pinned.contains(scene)) pinned.removeAll(scene);
  else pinned.prepend(scene);
  savePinnedHomeScenes(pinned);
  refreshHomeTable(window);
}

void appendHomeExperienceStyle(QMainWindow * window)
{
  if (!window) return;
  window->setStyleSheet(window->styleSheet() + QStringLiteral(R"QSS(
QToolBar#studioTopBar {
  background: #FFFFFF;
  border: 0;
  border-bottom: 1px solid #DDE5EE;
  spacing: 8px;
  padding: 7px 14px;
}
QLabel#studioTopBrand {
  color: #0D2D4D;
  font-size: 18px;
  font-weight: 800;
  letter-spacing: 0.6px;
}
QLabel#studioTopSafetyChip, QLabel#studioTopHardwareChip, QLabel#studioTopStatusChip {
  background: #FFFFFF;
  color: #21364B;
  border: 1px solid #D8E1EA;
  border-radius: 8px;
  padding: 6px 12px;
  font-size: 12px;
  font-weight: 600;
}
QLabel#studioTopSafetyChip { color: #147A47; }
QLabel#studioTopHardwareChip { color: #31455A; }
QLabel#studioTopStatusChip { color: #1D5DA8; }
QFrame#studioHomeSidebar {
  background: #082A4A;
  border: 0;
  border-right: 1px solid #0B345B;
}
QLabel#studioSidebarProductName {
  color: #FFFFFF;
  font-size: 15px;
  font-weight: 800;
  padding: 4px 6px 10px 6px;
}
QPushButton#studioSidebarNewCellButton {
  background: #0F4E84;
  color: #FFFFFF;
  border: 1px solid #1B639E;
  border-radius: 7px;
  min-height: 34px;
  font-weight: 700;
  text-align: left;
  padding-left: 12px;
}
QPushButton#studioSidebarNewCellButton:hover { background: #17649F; }
QListWidget#studioHomeSidebarNav {
  background: transparent;
  border: 0;
  outline: 0;
  color: #D9E8F5;
}
QListWidget#studioHomeSidebarNav::item {
  border: 0;
  border-radius: 7px;
  color: #D9E8F5;
  padding: 10px 11px;
  margin: 2px 0;
}
QListWidget#studioHomeSidebarNav::item:hover { background: #103C64; color: #FFFFFF; }
QListWidget#studioHomeSidebarNav::item:selected { background: #14558E; color: #FFFFFF; font-weight: 700; }
QLabel#studioSidebarSystemStatus {
  color: #A8C6DD;
  font-size: 11px;
  padding: 8px 6px;
}
QWidget#workcellStudioDashboardPage { background: #F7F9FC; color: #17283B; }
QFrame#studioHomeHeroCard {
  background: transparent;
  border: 0;
  border-radius: 0;
  padding: 0;
}
QLabel#dashboardTitleLabel { color: #102B46; font-size: 25px; font-weight: 800; }
QLabel#dashboardSubtitleLabel { color: #64748B; font-size: 13px; }
QLabel#studioHomeSummaryCard {
  background: #FFFFFF;
  border: 1px solid #E0E7EF;
  border-radius: 9px;
  padding: 12px 15px;
  color: #102B46;
}
QFrame#studioHomeScenesCard, QFrame#studioHomeDetailsCard {
  background: #FFFFFF;
  border: 1px solid #E0E7EF;
  border-radius: 9px;
}
QLabel#studioHomeSectionTitle { color: #102B46; font-size: 16px; font-weight: 750; }
QLineEdit#studioHomeSearchBox, QComboBox#studioHomeStatusFilter,
QComboBox#studioHomeRobotFilter, QComboBox#studioHomeToolFilter, QComboBox#studioHomeSort {
  background: #FFFFFF;
  color: #24384D;
  border: 1px solid #D7E0E9;
  border-radius: 6px;
  min-height: 30px;
  padding: 3px 8px;
}
QLineEdit#studioHomeSearchBox:focus, QComboBox#studioHomeStatusFilter:focus,
QComboBox#studioHomeRobotFilter:focus, QComboBox#studioHomeToolFilter:focus,
QComboBox#studioHomeSort:focus { border: 1px solid #1D5DA8; }
QCheckBox#studioHomeFavoritesOnly { color: #465A70; spacing: 6px; }
QTableWidget#studioHomeSceneTable {
  background: #FFFFFF;
  alternate-background-color: #FFFFFF;
  color: #24384D;
  border: 1px solid #E2E8F0;
  border-radius: 7px;
  gridline-color: transparent;
  outline: 0;
}
QTableWidget#studioHomeSceneTable::item {
  border: 0;
  border-bottom: 1px solid #EDF1F5;
  padding: 7px 8px;
  color: #24384D;
}
QTableWidget#studioHomeSceneTable::item:hover { background: #F6F9FC; }
QTableWidget#studioHomeSceneTable::item:selected { background: #E8F1FA; color: #163A5C; }
QTableWidget#studioHomeSceneTable QHeaderView::section {
  background: #F8FAFC;
  color: #43566B;
  border: 0;
  border-bottom: 1px solid #DCE4EC;
  padding: 9px 8px;
  font-weight: 700;
}
QLabel#studioHomeScenePreview {
  background: #17212B;
  color: #CAD5DF;
  border: 1px solid #DDE5ED;
  border-radius: 7px;
  min-height: 176px;
  padding: 0;
}
QLabel#studioHomeReadinessSummary {
  background: #F8FAFC;
  color: #31455A;
  border: 1px solid #E2E8F0;
  border-radius: 7px;
  padding: 10px;
}
QPushButton#studioHomeOpenSceneButton {
  background: #0E4F86;
  border: 1px solid #0A416F;
  color: #FFFFFF;
  font-weight: 700;
}
QPushButton#studioHomeOpenSceneButton:hover { background: #12629F; }
QPushButton#studioHomeValidateButton, QPushButton#studioHomeGenerateButton,
QPushButton#studioHomeSimulateButton, QPushButton#studioHomePinButton {
  background: #FFFFFF;
  color: #174B79;
  border: 1px solid #CBD8E5;
  font-weight: 650;
}
QPushButton#studioHomeValidateButton:hover, QPushButton#studioHomeGenerateButton:hover,
QPushButton#studioHomeSimulateButton:hover, QPushButton#studioHomePinButton:hover {
  background: #F1F6FB;
  border-color: #7AA5CA;
}
QLabel#studioHomeFooter { color: #738397; font-size: 11px; padding: 3px 2px; }
)QSS"));
}

void enhanceHomeTopBar(QMainWindow * window)
{
  auto * top_bar = window ? window->findChild<QToolBar *>(QStringLiteral("studioTopBar")) : nullptr;
  if (!top_bar || top_bar->property("homeExperienceApplied").toBool()) return;
  top_bar->setProperty("homeExperienceApplied", true);
  for (QAction * action : top_bar->actions()) {
    if (action) action->setVisible(false);
  }

  auto * brand = new QLabel(QStringLiteral("WORKCELL STUDIO"), top_bar);
  brand->setObjectName(QStringLiteral("studioTopBrand"));
  top_bar->addWidget(brand);
  auto * spacer = new QWidget(top_bar);
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  top_bar->addWidget(spacer);
  auto * safety = new QLabel(QStringLiteral("●  Simulation Mode   FAKE HARDWARE"), top_bar);
  safety->setObjectName(QStringLiteral("studioTopSafetyChip"));
  safety->setToolTip(QStringLiteral("Simulation remains fake-hardware-first. No real robot motion is enabled from the Home page."));
  top_bar->addWidget(safety);
  auto * hardware = new QLabel(QStringLiteral("🔒  Real hardware   LOCKED"), top_bar);
  hardware->setObjectName(QStringLiteral("studioTopHardwareChip"));
  hardware->setToolTip(QStringLiteral("Real-hardware execution remains explicitly guarded and locked by default."));
  top_bar->addWidget(hardware);
  auto * status = new QLabel(QStringLiteral("●  Studio ready"), top_bar);
  status->setObjectName(QStringLiteral("studioTopStatusChip"));
  top_bar->addWidget(status);
  top_bar->setMinimumHeight(56);
}

void enhanceHomeSidebar(QMainWindow * window)
{
  if (!window || window->findChild<QFrame *>(QStringLiteral("studioHomeSidebar"))) return;
  QWidget * central = window->centralWidget();
  auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  if (!central || !dashboard) return;
  auto * stack = qobject_cast<QStackedWidget *>(dashboard->parentWidget());
  if (!stack) return;

  QListWidget * nav = nullptr;
  for (QListWidget * candidate : central->findChildren<QListWidget *>()) {
    if (!candidate || candidate->count() == 0) continue;
    if (candidate->item(0) && candidate->item(0)->text().contains(QStringLiteral("Studio Home"))) {
      nav = candidate;
      break;
    }
  }
  if (!nav) return;

  QBoxLayout * body = nullptr;
  if (QLayout * root = central->layout()) {
    for (int i = 0; i < root->count(); ++i) {
      QLayout * candidate = root->itemAt(i) ? root->itemAt(i)->layout() : nullptr;
      auto * box = qobject_cast<QBoxLayout *>(candidate);
      if (box && box->indexOf(stack) >= 0) { body = box; break; }
    }
  }
  if (!body) return;

  auto * sidebar = new QFrame(central);
  sidebar->setObjectName(QStringLiteral("studioHomeSidebar"));
  sidebar->setMinimumWidth(168);
  sidebar->setMaximumWidth(184);
  auto * sidebar_layout = new QVBoxLayout(sidebar);
  sidebar_layout->setContentsMargins(10, 16, 10, 12);
  sidebar_layout->setSpacing(8);
  auto * product = new QLabel(QStringLiteral("WORKCELL\nSTUDIO"), sidebar);
  product->setObjectName(QStringLiteral("studioSidebarProductName"));
  sidebar_layout->addWidget(product);
  auto * new_cell = new QPushButton(QStringLiteral("＋  New Cell"), sidebar);
  new_cell->setObjectName(QStringLiteral("studioSidebarNewCellButton"));
  sidebar_layout->addWidget(new_cell);

  nav->setParent(sidebar);
  nav->setObjectName(QStringLiteral("studioHomeSidebarNav"));
  {
    const QSignalBlocker blocker(nav);
    nav->clear();
    nav->addItems({
      QStringLiteral("⌂  Home"),
      QStringLiteral("◇  Product View"),
      QStringLiteral("▦  Scenes"),
      QStringLiteral("▶  Demo Mode"),
      QStringLiteral("▷  Simulate"),
      QStringLiteral("◉  System Status"),
      QStringLiteral("✓  Validation"),
      QStringLiteral("⇧  Export")});
    nav->setCurrentRow(stack->currentIndex());
  }
  nav->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nav->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  nav->setMinimumHeight(390);
  nav->show();
  sidebar_layout->addWidget(nav, 1);
  auto * system = new QLabel(QStringLiteral("●  System online\n   WS  ·  ROS 2 Humble"), sidebar);
  system->setObjectName(QStringLiteral("studioSidebarSystemStatus"));
  sidebar_layout->addWidget(system);
  body->insertWidget(0, sidebar, 0);

  QObject::connect(new_cell, &QPushButton::clicked, window, [window]() {
    triggerStudioAction(window, {QStringLiteral("New Cell")});
  });
  QObject::connect(stack, &QStackedWidget::currentChanged, nav, [nav](int index) {
    if (!nav || nav->currentRow() == index) return;
    const QSignalBlocker blocker(nav);
    nav->setCurrentRow(index);
  });
}

void enhanceHomeDashboard(QMainWindow * window)
{
  auto * dashboard = window ? window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage")) : nullptr;
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!dashboard || !table || dashboard->property("homeExperienceApplied").toBool()) return;
  dashboard->setProperty("homeExperienceApplied", true);

  if (auto * layout = qobject_cast<QVBoxLayout *>(dashboard->layout())) {
    layout->setContentsMargins(26, 20, 26, 16);
    layout->setSpacing(16);
  }
  if (auto * title = window->findChild<QLabel *>(QStringLiteral("dashboardTitleLabel"))) {
    title->setText(QStringLiteral("Welcome to Workcell Studio"));
  }
  if (auto * subtitle = window->findChild<QLabel *>(QStringLiteral("dashboardSubtitleLabel"))) {
    subtitle->setText(QStringLiteral("Select a workcell, review readiness, or start a new robotic cell."));
  }
  if (auto * raw_summary = window->findChild<QLabel *>(QStringLiteral("dashboardSummaryLabel"))) {
    raw_summary->hide();
  }
  for (QLabel * safety : window->findChildren<QLabel *>(QStringLiteral("studioHomeSafetyPill"))) {
    if (safety) safety->hide();
  }
  for (QPushButton * hero_button : window->findChildren<QPushButton *>(QStringLiteral("studioHomePrimaryButton"))) {
    if (!hero_button) continue;
    if (hero_button->text().contains(QStringLiteral("New Cell"), Qt::CaseInsensitive)) {
      hero_button->setText(QStringLiteral("＋  New Cell"));
      hero_button->setMinimumWidth(118);
    } else if (hero_button->text().contains(QStringLiteral("Open Selected"), Qt::CaseInsensitive)) {
      hero_button->hide();
    }
  }

  for (QLabel * label : dashboard->findChildren<QLabel *>(QStringLiteral("studioHomeDetailsCard"))) {
    if (label) label->hide();
  }

  if (QSplitter * splitter = dashboard->findChild<QSplitter *>()) {
    if (splitter->count() >= 3) {
      QWidget * left_library = splitter->widget(0);
      if (left_library) left_library->hide();
      splitter->setStretchFactor(0, 0);
      splitter->setStretchFactor(1, 5);
      splitter->setStretchFactor(2, 2);
      splitter->setSizes({0, 980, 360});
      splitter->setHandleWidth(1);
      if (auto * selected_card = qobject_cast<QFrame *>(splitter->widget(2))) {
        selected_card->setMinimumWidth(320);
        selected_card->setMaximumWidth(390);
      }
    }
  }

  QWidget * center_card = table->parentWidget();
  if (center_card) {
    center_card->setObjectName(QStringLiteral("studioHomeScenesCard"));
    for (QLabel * label : center_card->findChildren<QLabel *>(QString(), Qt::FindDirectChildrenOnly)) {
      if (label && label->text().contains(QStringLiteral("Scenes"), Qt::CaseInsensitive)) {
        label->setText(QStringLiteral("Scenes"));
        label->setObjectName(QStringLiteral("studioHomeSectionTitle"));
        break;
      }
    }
    if (auto * center_layout = qobject_cast<QVBoxLayout *>(center_card->layout())) {
      auto * controls = new QFrame(center_card);
      controls->setObjectName(QStringLiteral("studioHomeFilterStrip"));
      auto * controls_layout = new QHBoxLayout(controls);
      controls_layout->setContentsMargins(0, 0, 0, 0);
      controls_layout->setSpacing(8);
      auto * robot = new QComboBox(controls);
      robot->setObjectName(QStringLiteral("studioHomeRobotFilter"));
      robot->setMinimumWidth(130);
      auto * tool = new QComboBox(controls);
      tool->setObjectName(QStringLiteral("studioHomeToolFilter"));
      tool->setMinimumWidth(155);
      auto * favorites = new QCheckBox(QStringLiteral("★  Favorites only"), controls);
      favorites->setObjectName(QStringLiteral("studioHomeFavoritesOnly"));
      auto * sort = new QComboBox(controls);
      sort->setObjectName(QStringLiteral("studioHomeSort"));
      sort->addItem(QStringLiteral("Sort: Recently updated"), QStringLiteral("updated"));
      sort->addItem(QStringLiteral("Sort: Scene name"), QStringLiteral("name"));
      sort->addItem(QStringLiteral("Sort: Pinned first"), QStringLiteral("pinned"));
      sort->setMinimumWidth(172);
      controls_layout->addWidget(robot);
      controls_layout->addWidget(tool);
      controls_layout->addWidget(favorites);
      controls_layout->addStretch(1);
      controls_layout->addWidget(sort);
      const int table_index = center_layout->indexOf(table);
      center_layout->insertWidget(std::max(0, table_index), controls);

      auto * footer = new QLabel(QStringLiteral("Scene source will appear after discovery."), center_card);
      footer->setObjectName(QStringLiteral("studioHomeFooter"));
      center_layout->addWidget(footer);

      QObject::connect(robot, qOverload<int>(&QComboBox::currentIndexChanged), window, [window](int) { applyHomeExtraFilters(window); });
      QObject::connect(tool, qOverload<int>(&QComboBox::currentIndexChanged), window, [window](int) { applyHomeExtraFilters(window); });
      QObject::connect(favorites, &QCheckBox::toggled, window, [window](bool) { applyHomeExtraFilters(window); });
      QObject::connect(sort, qOverload<int>(&QComboBox::currentIndexChanged), window, [window](int) {
        applyHomeSort(window);
        applyHomeExtraFilters(window);
        refreshHomeInspector(window);
      });
    }
  }

  if (auto * search = window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox"))) {
    search->setPlaceholderText(QStringLiteral("Search scenes…"));
    search->setMaximumWidth(420);
  }
  if (auto * status_filter = window->findChild<QComboBox *>(QStringLiteral("studioHomeStatusFilter"))) {
    status_filter->setMinimumWidth(128);
    status_filter->setMaximumWidth(150);
    status_filter->setItemText(0, QStringLiteral("Status: All"));
  }

  QFrame * details_card = window->findChild<QFrame *>(QStringLiteral("studioHomeDetailsCard"));
  if (details_card) {
    auto * details_layout = qobject_cast<QVBoxLayout *>(details_card->layout());
    if (details_layout) {
      auto * preview = new QLabel(QStringLiteral("Select a scene to preview"), details_card);
      preview->setObjectName(QStringLiteral("studioHomeScenePreview"));
      preview->setAlignment(Qt::AlignCenter);
      preview->setMinimumHeight(176);
      preview->setMaximumHeight(176);
      preview->setScaledContents(false);
      details_layout->insertWidget(1, preview);

      auto * readiness = new QLabel(
        QStringLiteral("<b>Readiness Summary</b><br/><span style='color:#64748B'>Select a scene to inspect readiness.</span>"), details_card);
      readiness->setObjectName(QStringLiteral("studioHomeReadinessSummary"));
      readiness->setWordWrap(true);
      details_layout->addWidget(readiness);

      auto * pin = new QPushButton(QStringLiteral("☆  Pin Favorite"), details_card);
      pin->setObjectName(QStringLiteral("studioHomePinButton"));
      details_layout->addWidget(pin);

      auto * open = new QPushButton(QStringLiteral("Open Scene"), details_card);
      open->setObjectName(QStringLiteral("studioHomeOpenSceneButton"));
      details_layout->addWidget(open);
      auto * validate = new QPushButton(QStringLiteral("Validate"), details_card);
      validate->setObjectName(QStringLiteral("studioHomeValidateButton"));
      details_layout->addWidget(validate);
      auto * generate = new QPushButton(QStringLiteral("Generate Package"), details_card);
      generate->setObjectName(QStringLiteral("studioHomeGenerateButton"));
      details_layout->addWidget(generate);
      auto * simulate = new QPushButton(QStringLiteral("▷  Simulate (Fake Hardware)"), details_card);
      simulate->setObjectName(QStringLiteral("studioHomeSimulateButton"));
      simulate->setToolTip(QStringLiteral("Open the guarded fake-hardware Plan / Simulate workflow. Real hardware remains locked."));
      details_layout->addWidget(simulate);

      if (auto * more = details_card->findChild<QToolButton *>(QStringLiteral("studioHomeSecondaryButton"))) {
        more->setText(QStringLiteral("More Actions"));
      }
      QObject::connect(pin, &QPushButton::clicked, window, [window]() { toggleCurrentHomeFavorite(window); });
      QObject::connect(open, &QPushButton::clicked, window, [window]() {
        triggerStudioAction(window, {QStringLiteral("Open in Scene Builder"), QStringLiteral("Open Scene Builder")});
      });
      QObject::connect(validate, &QPushButton::clicked, window, [window]() {
        triggerStudioAction(window, {QStringLiteral("Validate")});
      });
      QObject::connect(generate, &QPushButton::clicked, window, [window]() {
        triggerStudioAction(window, {QStringLiteral("Generate Scene Package")});
      });
      QObject::connect(simulate, &QPushButton::clicked, window, [window]() {
        triggerStudioAction(window, {QStringLiteral("Plan / Simulate"), QStringLiteral("Open RViz Truth Preview"), QStringLiteral("Plan/Simulate Preview")});
      });
    }
  }

  QObject::connect(table, &QTableWidget::cellClicked, window, [window](int, int column) {
    if (column == kHomePinColumn) toggleCurrentHomeFavorite(window);
    else refreshHomeInspector(window);
  });
  QObject::connect(table, &QTableWidget::itemSelectionChanged, window, [window]() { refreshHomeInspector(window); });
  if (table->model()) {
    QObject::connect(table->model(), &QAbstractItemModel::rowsInserted, window, [window]() { scheduleHomeRefresh(window); });
    QObject::connect(table->model(), &QAbstractItemModel::rowsRemoved, window, [window]() { scheduleHomeRefresh(window); });
    QObject::connect(table->model(), &QAbstractItemModel::modelReset, window, [window]() { scheduleHomeRefresh(window); });
  }

  refreshHomeTable(window);
  if (auto * footer = window->findChild<QLabel *>(QStringLiteral("studioHomeFooter"))) {
    const QString root = homeSceneRoot(window);
    footer->setText(root.isEmpty()
      ? QStringLiteral("Scene source: discovering…")
      : QStringLiteral("Source: %1   ·   Favorites are stored locally for this Workcell Studio profile.").arg(root));
  }
}

void applyHomeResponsiveMode(QMainWindow * window)
{
  if (!window || !window->property("workcell_home_experience_applied").toBool()) return;
  auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  if (!dashboard) return;
  QSplitter * splitter = dashboard->findChild<QSplitter *>();
  if (!splitter || splitter->count() < 3) return;
  QWidget * details = splitter->widget(2);
  if (details) details->setVisible(window->width() >= 1260);
}

void applyWorkcellHomeExperience(QMainWindow * window)
{
  if (!window || window->property("workcell_home_experience_applied").toBool()) return;
  if (QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (!window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) return;
  window->setProperty("workcell_home_experience_applied", true);
  window->setMinimumSize(1100, 700);
  appendHomeExperienceStyle(window);
  enhanceHomeTopBar(window);
  enhanceHomeSidebar(window);
  enhanceHomeDashboard(window);
  applyHomeResponsiveMode(window);

  // Interactive Workcell Studio should never appear as the tiny legacy utility window.
  // Keep the normal desktop title bar while starting maximized; Scene3D smoke remains unchanged.
  QTimer::singleShot(0, window, [window]() {
    if (window && !window->isMaximized()) window->showMaximized();
  });
}

class WorkcellHomeExperienceGuard : public QObject
{
public:
  explicit WorkcellHomeExperienceGuard(QObject * parent)
  : QObject(parent)
  {
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    auto * window = qobject_cast<QMainWindow *>(watched);
    if (!window) return QObject::eventFilter(watched, event);
    if (event->type() == QEvent::Show &&
      window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage")) &&
      !window->property("workcell_home_experience_applied").toBool())
    {
      QTimer::singleShot(0, window, [window]() { applyWorkcellHomeExperience(window); });
    } else if (event->type() == QEvent::Resize &&
      window->property("workcell_home_experience_applied").toBool())
    {
      QTimer::singleShot(0, window, [window]() { applyHomeResponsiveMode(window); });
    }
    return QObject::eventFilter(watched, event);
  }
};

void installWorkcellHomeExperienceGuard()
{
  if (!qApp || qApp->property("workcell_home_experience_guard_installed").toBool()) return;
  qApp->installEventFilter(new WorkcellHomeExperienceGuard(qApp));
  qApp->setProperty("workcell_home_experience_guard_installed", true);
}
}  // namespace

QString workcellStudioStyleSheet()
{
  installWorkcellHomeExperienceGuard();
  return QStringLiteral(
    "QWidget { font-size: 13px; color: #17202a; }"
    "QMainWindow, QDialog { background-color: #f4f7fb; }"
    "QGroupBox { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 8px; margin-top: 10px; padding-top: 5px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4d5b6a; font-weight: 600; }"
    "QTabWidget::pane { border: 1px solid #c7d3df; background: #ffffff; border-radius: 6px; }"
    "QTabBar::tab { background: #eef3f8; color: #4d5b6a; border: 1px solid #c7d3df; padding: 7px 10px; }"
    "QTabBar::tab:selected { background: #ffffff; color: #17202a; border-bottom-color: #ffffff; }"
    "QTabBar::tab:hover { background: #e1eaf3; }"
    "QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextBrowser, QTextEdit, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 4px; selection-background-color: #d7edf7; selection-color: #17202a; }"
    "QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QListWidget:focus, QTreeWidget:focus, QTableWidget:focus { border: 2px solid #1d5da8; }"
    "QHeaderView::section { background: #eef3f8; color: #17202a; border: 0; border-right: 1px solid #c7d3df; border-bottom: 1px solid #c7d3df; padding: 5px; font-weight: 600; }"
    "QPushButton, QToolButton { min-height: 30px; background-color: #60758a; color: #ffffff; border: 1px solid #4d5b6a; border-radius: 5px; padding: 5px 10px; }"
    "QPushButton:hover, QToolButton:hover { background-color: #4d647a; }"
    "QPushButton:pressed, QToolButton:pressed { background-color: #3f5366; }"
    "QPushButton:disabled, QToolButton:disabled { background-color: #d8e0e8; color: #6c7884; border: 1px solid #c7d3df; }"
    "QPushButton[class='primary_action'] { background-color: #1d5da8; border-color: #174a86; color: #ffffff; font-weight: 600; }"
    "QPushButton[class='secondary_action'] { background-color: #60758a; border-color: #4d647a; }"
    "QPushButton[class='safe_action'] { background-color: #217a34; border-color: #1a622a; }"
    "QPushButton[class='preview_action'] { background-color: #6f42c1; border-color: #57329a; }"
    "QPushButton[class='destructive_action'] { background-color: #fce8e7; color: #b3261e; border: 1px solid #b3261e; }"
    "QPushButton[class='disabled_placeholder'] { background-color: #d8e0e8; color: #6c7884; border: 1px dashed #6c7884; }"
    "QScrollArea { background: transparent; border: 0; }"
    "QAbstractScrollArea { background: #ffffff; border: 1px solid #c7d3df; }"
    "QScrollBar:vertical { width: 10px; margin: 0; background: transparent; }"
    "QScrollBar::handle:vertical { min-height: 28px; background: #b7c4d1; border-radius: 5px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
    "QToolTip { background: #17202a; color: #ffffff; border: 0; padding: 5px; }");
}

void applyWorkcellStudioTheme(QWidget * widget)
{
  if (widget) widget->setStyleSheet(workcellStudioStyleSheet());
}

void applyButtonRoleStyle(QPushButton * button, ButtonRole role)
{
  if (!button) return;
  switch (role) {
    case ButtonRole::primary_action: button->setProperty("class", "primary_action"); break;
    case ButtonRole::secondary_action: button->setProperty("class", "secondary_action"); break;
    case ButtonRole::safe_action: button->setProperty("class", "safe_action"); break;
    case ButtonRole::preview_action: button->setProperty("class", "preview_action"); break;
    case ButtonRole::destructive_action: button->setProperty("class", "destructive_action"); break;
    case ButtonRole::disabled_placeholder: button->setProperty("class", "disabled_placeholder"); break;
  }
}

void applyStatusBadgeStyle(QLabel * label, StatusType status)
{
  if (!label) return;
  QString bg = "#eef3f8"; QString fg = "#4d5b6a";
  if (status == StatusType::Success || status == StatusType::Ready) { bg = "#e8f5eb"; fg = "#217a34"; }
  else if (status == StatusType::Warning) { bg = "#fff3e0"; fg = "#b26a00"; }
  else if (status == StatusType::Error) { bg = "#fdecea"; fg = "#b3261e"; }
  else if (status == StatusType::PreviewOnly) { bg = "#f1ebff"; fg = "#6f42c1"; }
  else if (status == StatusType::FakeHardware || status == StatusType::Info || status == StatusType::LiveEpd || status == StatusType::NoRuntimeMotion) { bg = "#e7f0fb"; fg = "#2769b3"; }
  else if (status == StatusType::ScaffoldOnly) { bg = "#eef3f8"; fg = "#60758a"; }
  label->setStyleSheet(QString("background:%1; color:%2; border:1px solid #c7d3df; border-radius:10px; padding:2px 8px; font-weight:600;").arg(bg, fg));
}

void capDialogSize(QWidget * widget, int, int)
{
  if (!widget) return;

  const QRect available = usableScreenGeometry(widget);
  widget->setMinimumSize(
    std::min(640, available.width()),
    std::min(420, available.height()));
  widget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  installResponsiveWindowGuard(widget);
  keepWindowInsideScreen(widget);
}

void makeTextWidgetsWrap(QWidget * widget)
{
  if (!widget) return;
  for (QLabel * label : widget->findChildren<QLabel *>()) {
    label->setWordWrap(true);
    label->setMinimumWidth(0);
    label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  }
}

void applyPrimarySecondaryButtonStyle(QWidget * widget)
{
  if (!widget) return;
  for (QDialogButtonBox * box : widget->findChildren<QDialogButtonBox *>()) {
    if (auto * ok = box->button(QDialogButtonBox::Ok)) {
      applyButtonRoleStyle(ok, ButtonRole::primary_action);
      ok->setDefault(true);
    }
    if (auto * save = box->button(QDialogButtonBox::Save)) {
      applyButtonRoleStyle(save, ButtonRole::primary_action);
      save->setDefault(true);
    }
  }
}

void applyStatusLabelStyle(QLabel * label, StatusType status)
{
  applyStatusBadgeStyle(label, status);
}

void applyCompactDialogDefaults(QWidget * widget)
{
  if (!widget) return;
  capDialogSize(widget);
  makeTextWidgetsWrap(widget);
  configureContainedWidgets(widget);
  applyWorkcellStudioTheme(widget);
  applyPrimarySecondaryButtonStyle(widget);
}
}  // namespace workcell_builder
