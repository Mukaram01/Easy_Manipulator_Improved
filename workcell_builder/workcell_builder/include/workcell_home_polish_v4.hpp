#pragma once

// Home v4 structural repair.
//
// The earlier Home polish used transparent QWidget cell overlays on top of a
// QTableWidget.  That was visually fragile: model refreshes could remove some
// overlays while leaving the backing QTableWidgetItem text transparent, which
// produced either double-painted rows or apparently missing columns.  This pass
// keeps the canonical table model untouched and renders the presentation with a
// single QStyledItemDelegate instead.  The Home preview is also deterministic:
// it prefers an existing scene smoke/acceptance image, then a genuinely usable
// canonical live preview, and otherwise shows an explicit "not generated" state
// rather than an endless PREVIEW PREPARING placeholder.
#include "workcell_home_polish_v3.hpp"

#include <QAbstractItemModel>
#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QMetaObject>
#include <QPainter>
#include <QPixmap>
#include <QSettings>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>

namespace workcell_builder
{
namespace home_polish_v4
{
constexpr int kSceneColumn = 0;
constexpr int kStatusColumn = 1;
constexpr int kRobotColumn = 2;
constexpr int kToolColumn = 3;
constexpr int kTaskColumn = 4;
constexpr int kLaunchColumn = 5;
constexpr int kUpdatedColumn = 6;
constexpr int kPinColumn = 7;

inline QString sceneNameAt(QTableWidget * table, int row)
{
  return home_polish_v3::sceneNameAt(table, row);
}

inline QString friendlyTool(QString tool)
{
  const QString normalized = tool.trimmed().toLower();
  if (normalized.isEmpty()) return QStringLiteral("—");
  if (normalized == QStringLiteral("unknown")) return QStringLiteral("Unknown");
  if (normalized == QStringLiteral("robotiq_85") ||
      normalized == QStringLiteral("robotiq_2f_85") ||
      normalized == QStringLiteral("robotiq_85_gripper") ||
      normalized == QStringLiteral("robotiq_2f_85_gripper")) {
    return QStringLiteral("Robotiq 2F-85");
  }
  if (normalized == QStringLiteral("single_suction")) return QStringLiteral("Single Suction");
  if (normalized == QStringLiteral("airpick4") || normalized == QStringLiteral("onrobot_airpick4")) {
    return QStringLiteral("OnRobot AirPick4");
  }
  tool.replace('_', ' ');
  return tool;
}

inline QString displayStatus(const QString & raw)
{
  const QString status = home_polish_v2::cleanStatusText(raw).trimmed().toUpper();
  if (status.contains(QStringLiteral("READY"))) return QStringLiteral("Ready");
  if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) {
    return QStringLiteral("Needs attention");
  }
  return QStringLiteral("Blocked");
}

inline QString rowSceneId(const QModelIndex & index)
{
  const QModelIndex scene_index = index.sibling(index.row(), kSceneColumn);
  const QString tooltip = scene_index.data(Qt::ToolTipRole).toString().trimmed();
  return tooltip.isEmpty() ? scene_index.data(Qt::DisplayRole).toString().trimmed() : tooltip;
}

class HomeTableDelegate final : public QStyledItemDelegate
{
public:
  explicit HomeTableDelegate(QObject * parent = nullptr) : QStyledItemDelegate(parent) {}

  QSize sizeHint(const QStyleOptionViewItem & option, const QModelIndex & index) const override
  {
    QSize size = QStyledItemDelegate::sizeHint(option, index);
    size.setHeight(58);
    return size;
  }

  void paint(QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index) const override
  {
    if (!painter || !index.isValid()) return;
    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);

    const bool selected = option.state.testFlag(QStyle::State_Selected);
    const bool hovered = option.state.testFlag(QStyle::State_MouseOver);
    const QColor background = selected
      ? QColor(QStringLiteral("#EAF3FC"))
      : (hovered ? QColor(QStringLiteral("#F7FAFD")) : QColor(QStringLiteral("#FFFFFF")));
    painter->fillRect(option.rect, background);
    painter->setPen(QColor(QStringLiteral("#EDF2F7")));
    painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight());

    QRect content = option.rect.adjusted(12, 0, -10, 0);
    const QString raw = index.data(Qt::DisplayRole).toString().trimmed();
    const int column = index.column();

    if (column == kSceneColumn) {
      const QString scene_id = rowSceneId(index);
      const QString title = home_polish_v3::friendlySceneName(scene_id);
      QFont title_font = option.font;
      title_font.setBold(true);
      title_font.setPointSizeF(qMax(9.0, title_font.pointSizeF()));
      painter->setFont(title_font);
      painter->setPen(QColor(QStringLiteral("#143A5B")));
      const QFontMetrics title_metrics(title_font);
      painter->drawText(
        QRect(content.left(), content.top() + 8, content.width(), 21),
        Qt::AlignLeft | Qt::AlignVCenter,
        title_metrics.elidedText(title, Qt::ElideRight, content.width()));

      QFont id_font = option.font;
      id_font.setPointSizeF(qMax(8.0, option.font.pointSizeF() - 1.0));
      painter->setFont(id_font);
      painter->setPen(QColor(QStringLiteral("#7890A6")));
      const QFontMetrics id_metrics(id_font);
      painter->drawText(
        QRect(content.left(), content.top() + 31, content.width(), 18),
        Qt::AlignLeft | Qt::AlignVCenter,
        id_metrics.elidedText(scene_id, Qt::ElideRight, content.width()));
    } else if (column == kStatusColumn) {
      const QString display = displayStatus(raw);
      QColor fill(QStringLiteral("#FDECEC"));
      QColor text(QStringLiteral("#B42318"));
      if (display == QStringLiteral("Ready")) {
        fill = QColor(QStringLiteral("#EAF7F0"));
        text = QColor(QStringLiteral("#147A47"));
      } else if (display == QStringLiteral("Needs attention")) {
        fill = QColor(QStringLiteral("#FFF4E5"));
        text = QColor(QStringLiteral("#B65A00"));
      }
      const int pill_width = qMin(content.width(), display == QStringLiteral("Needs attention") ? 124 : 82);
      const QRect pill(content.left(), option.rect.center().y() - 13, pill_width, 26);
      painter->setPen(Qt::NoPen);
      painter->setBrush(fill);
      painter->drawRoundedRect(pill, 13, 13);
      QFont font = option.font;
      font.setBold(true);
      painter->setFont(font);
      painter->setPen(text);
      painter->drawText(pill.adjusted(8, 0, -8, 0), Qt::AlignCenter, display);
    } else if (column == kRobotColumn) {
      const QString display = home_polish_v3::displayRobot(raw);
      painter->setPen(raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0
        ? QColor(QStringLiteral("#94A3B8")) : QColor(QStringLiteral("#2D4358")));
      painter->drawText(content, Qt::AlignLeft | Qt::AlignVCenter, display);
    } else if (column == kToolColumn) {
      const QString display = friendlyTool(raw);
      painter->setPen(raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0
        ? QColor(QStringLiteral("#94A3B8")) : QColor(QStringLiteral("#2D4358")));
      const QFontMetrics metrics(option.font);
      painter->drawText(content, Qt::AlignLeft | Qt::AlignVCenter,
        metrics.elidedText(display, Qt::ElideRight, content.width()));
    } else if (column == kUpdatedColumn) {
      painter->setPen(QColor(QStringLiteral("#536B82")));
      painter->drawText(content, Qt::AlignLeft | Qt::AlignVCenter,
        home_polish_v3::relativeUpdatedText(raw));
    } else if (column == kPinColumn) {
      const QString scene = rowSceneId(index);
      const bool pinned = home_polish_v3::pinnedScenes().contains(scene);
      QFont font = option.font;
      font.setPointSizeF(qMax(12.0, option.font.pointSizeF() + 2.0));
      painter->setFont(font);
      painter->setPen(pinned ? QColor(QStringLiteral("#155B91")) : QColor(QStringLiteral("#A8B7C6")));
      painter->drawText(option.rect, Qt::AlignCenter, pinned ? QStringLiteral("★") : QStringLiteral("☆"));
    } else {
      QStyledItemDelegate::paint(painter, option, index);
    }

    painter->restore();
  }
};

inline void deleteLegacyCellWidgets(QTableWidget * table)
{
  if (!table) return;
  for (int row = 0; row < table->rowCount(); ++row) {
    for (const int column : {kSceneColumn, kStatusColumn, kRobotColumn, kToolColumn, kUpdatedColumn}) {
      if (QWidget * widget = table->cellWidget(row, column)) {
        table->removeCellWidget(row, column);
        widget->deleteLater();
      }
    }
  }
}

inline void repairTable(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table) return;

  deleteLegacyCellWidgets(table);
  if (!table->property("homeV4DelegateInstalled").toBool()) {
    table->setItemDelegate(new HomeTableDelegate(table));
    table->setProperty("homeV4DelegateInstalled", true);
  }

  table->setShowGrid(false);
  table->setAlternatingRowColors(false);
  table->setWordWrap(false);
  table->setMouseTracking(true);
  table->setTextElideMode(Qt::ElideRight);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(58);
  table->horizontalHeader()->setMinimumHeight(42);
  table->horizontalHeader()->setStretchLastSection(false);

  table->setColumnHidden(kSceneColumn, false);
  table->setColumnHidden(kStatusColumn, false);
  table->setColumnHidden(kRobotColumn, false);
  table->setColumnHidden(kToolColumn, false);
  table->setColumnHidden(kTaskColumn, true);
  table->setColumnHidden(kLaunchColumn, true);
  table->setColumnHidden(kUpdatedColumn, false);
  table->setColumnHidden(kPinColumn, false);

  table->horizontalHeader()->setSectionResizeMode(kSceneColumn, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(kStatusColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kRobotColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kToolColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kUpdatedColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kPinColumn, QHeaderView::Fixed);
  table->setColumnWidth(kStatusColumn, 150);
  table->setColumnWidth(kRobotColumn, 88);
  table->setColumnWidth(kToolColumn, 156);
  table->setColumnWidth(kUpdatedColumn, 104);
  table->setColumnWidth(kPinColumn, 62);
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"),
    QStringLiteral("Tool / Gripper"), QStringLiteral("Task"), QStringLiteral("Launch"),
    QStringLiteral("Updated"), QStringLiteral("Pinned")});
  table->viewport()->update();
}

inline void repairShell(QMainWindow * window)
{
  if (!window) return;
  if (QToolBar * top_bar = window->findChild<QToolBar *>(QStringLiteral("studioTopBar"))) {
    // Do not depend on object names here.  The screenshot regression proved the
    // top brand/status can be rebuilt by earlier shell polish under different
    // object identities.  Hide only duplicate/generic labels that live inside
    // the top bar; the sidebar brand is outside this subtree and remains intact.
    for (QLabel * label : top_bar->findChildren<QLabel *>()) {
      if (!label) continue;
      const QString text = label->text().simplified();
      if (text.compare(QStringLiteral("WORKCELL STUDIO"), Qt::CaseInsensitive) == 0 ||
          text.compare(QStringLiteral("Ready"), Qt::CaseInsensitive) == 0 ||
          text.compare(QStringLiteral("Studio ready"), Qt::CaseInsensitive) == 0) {
        label->hide();
      }
    }
    top_bar->setMinimumHeight(48);
    top_bar->setMaximumHeight(48);
  }

  if (QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
    for (QLabel * label : dashboard->findChildren<QLabel *>()) {
      if (label && label->text().trimmed() == QStringLiteral("Scenes")) label->setText(QStringLiteral("Workcells"));
    }
  }
  if (QToolButton * more = window->findChild<QToolButton *>(QStringLiteral("homeV3InspectorMore"))) more->hide();
  if (QToolButton * details = window->findChild<QToolButton *>(QStringLiteral("homeV3ViewDetails"))) details->hide();
}

inline QStringList sceneRootCandidates(QMainWindow * window)
{
  QStringList roots;
  const auto add_root = [&roots](const QString & root) {
    const QString cleaned = QDir::cleanPath(root.trimmed());
    if (!cleaned.isEmpty() && QDir(cleaned).exists() && !roots.contains(cleaned)) roots.append(cleaned);
  };

  add_root(home_polish_v3::sceneRoot(window));

  QSettings explicit_settings(QStringLiteral("easy_manipulation_deployment"), QStringLiteral("workcell_builder"));
  const QString workspace = explicit_settings.value(QStringLiteral("startup/last_workspace")).toString().trimmed();
  if (!workspace.isEmpty()) {
    add_root(QDir(workspace).filePath(QStringLiteral("src/easy_manipulation_deployment/scenes")));
    add_root(QDir(workspace).filePath(QStringLiteral("src/Easy_Manipulator_Improved/scenes")));
    add_root(QDir(workspace).filePath(QStringLiteral("src/scenes")));
  }

  // Launching `workcell_builder` from the workspace root is common during local
  // development.  These candidates keep Home preview discovery working even if
  // the startup settings were never persisted on this machine.
  const QString cwd = QDir::currentPath();
  add_root(QDir(cwd).filePath(QStringLiteral("src/easy_manipulation_deployment/scenes")));
  add_root(QDir(cwd).filePath(QStringLiteral("scenes")));
  return roots;
}

inline QString reliableCachedPreviewPath(QMainWindow * window, const QString & scene)
{
  if (scene.trimmed().isEmpty()) return QString();
  const QStringList relative_candidates = {
    QStringLiteral("smoke/scene3d_gui_smoke.png"),
    QStringLiteral("acceptance/scene3d_gui_smoke.png"),
    QStringLiteral("smoke/scene3d_smoke.png"),
    QStringLiteral("acceptance/scene3d_smoke.png"),
    QStringLiteral("preview/workcell_studio_canvas_snapshot.png"),
    QStringLiteral("preview/scene3d_preview.png"),
    QStringLiteral("preview/static_preview.png"),
    QStringLiteral("preview_launch/scene3d_preview.png"),
    QStringLiteral("preview_launch/product_view.png"),
    QStringLiteral("generated/scene3d_preview.png")};

  for (const QString & root : sceneRootCandidates(window)) {
    const QDir scene_dir(QDir(root).filePath(scene));
    if (!scene_dir.exists()) continue;
    for (const QString & relative : relative_candidates) {
      const QString candidate = scene_dir.filePath(relative);
      if (QFileInfo::exists(candidate)) return candidate;
    }
  }
  return QString();
}

inline QString selectedScene(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table || table->currentRow() < 0 || table->currentRow() >= table->rowCount()) return QString();
  return sceneNameAt(table, table->currentRow());
}

inline void repairInspectorText(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table || table->currentRow() < 0 || table->currentRow() >= table->rowCount()) return;
  const int row = table->currentRow();
  const QString tool_raw = table->item(row, kToolColumn) ? table->item(row, kToolColumn)->text() : QString();
  if (QLabel * tool = window->findChild<QLabel *>(QStringLiteral("homeV3MetaTool"))) {
    tool->setText(friendlyTool(tool_raw));
  }
}

inline void refreshHomePreview(QMainWindow * window, const QString & requested_scene = QString())
{
  if (!window) return;
  auto * preview = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview"));
  if (!preview) return;
  const QString scene = requested_scene.trimmed().isEmpty() ? selectedScene(window) : requested_scene.trimmed();

  preview->setPixmap(QPixmap());
  const QSize target(qMax(300, preview->width() - 4), qMax(204, preview->height() - 4));
  const QString cached_path = reliableCachedPreviewPath(window, scene);
  const QPixmap cached(cached_path);
  if (!cached.isNull()) {
    preview->setText(QString());
    preview->setPixmap(cached.scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    preview->setToolTip(QStringLiteral("Read-only scene preview: %1").arg(cached_path));
    return;
  }

  // If Product View is already loaded for this exact scene, use it.  We never
  // force-load hidden 3D UI from Home; that was the source of the previous
  // PREVIEW PREPARING dead state.
  const QPixmap live = home_polish_v3::liveCanonicalPreview(window, scene, target);
  if (!live.isNull()) {
    preview->setText(QString());
    preview->setPixmap(live);
    preview->setToolTip(QStringLiteral("Live read-only preview from Product View."));
    return;
  }

  preview->setText(scene.isEmpty()
    ? QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here")
    : QStringLiteral("NO PREVIEW GENERATED\nOpen Product View to render this workcell"));
  preview->setToolTip(QStringLiteral("No smoke, acceptance, preview, or active Product View image is available for this workcell."));
}

inline void scheduleHomePreview(QMainWindow * window, const QString & scene)
{
  if (!window) return;
  const qulonglong generation = window->property("homeV4PreviewGeneration").toULongLong() + 1;
  window->setProperty("homeV4PreviewGeneration", generation);
  for (const int delay : {0, 120, 600, 1600, 3300}) {
    QTimer::singleShot(delay, window, [window, scene, generation]() {
      if (window->property("homeV4PreviewGeneration").toULongLong() != generation) return;
      repairInspectorText(window);
      refreshHomePreview(window, scene);
    });
  }
}

inline int rowForScene(QTableWidget * table, const QString & scene)
{
  if (!table || scene.trimmed().isEmpty()) return -1;
  for (int row = 0; row < table->rowCount(); ++row) {
    if (!table->isRowHidden(row) && sceneNameAt(table, row).compare(scene, Qt::CaseInsensitive) == 0) return row;
  }
  return -1;
}

inline int firstVisibleRow(QTableWidget * table)
{
  if (!table) return -1;
  for (int row = 0; row < table->rowCount(); ++row) if (!table->isRowHidden(row)) return row;
  return -1;
}

inline void ensureUsefulSelection(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table || table->rowCount() <= 0) return;

  if (table->currentRow() >= 0 && table->currentRow() < table->rowCount() && !table->isRowHidden(table->currentRow())) {
    scheduleHomePreview(window, sceneNameAt(table, table->currentRow()));
    return;
  }

  QSettings settings;
  int row = rowForScene(table, settings.value(QStringLiteral("studio_home/last_selected_scene")).toString());
  if (row < 0) {
    for (const QString & pinned : home_polish_v3::pinnedScenes()) {
      row = rowForScene(table, pinned);
      if (row >= 0) break;
    }
  }
  if (row < 0) row = firstVisibleRow(table);
  if (row < 0) return;

  table->setCurrentCell(row, kSceneColumn);
  table->selectRow(row);
  const QString scene = sceneNameAt(table, row);
  settings.setValue(QStringLiteral("studio_home/last_selected_scene"), scene);
  QMetaObject::invokeMethod(table, "cellClicked", Qt::DirectConnection,
    Q_ARG(int, row), Q_ARG(int, kSceneColumn));
  scheduleHomePreview(window, scene);
}

inline void togglePinned(QTableWidget * table, int row)
{
  if (!table || row < 0 || row >= table->rowCount()) return;
  const QString scene = sceneNameAt(table, row);
  if (scene.isEmpty()) return;
  QStringList pinned = home_polish_v3::pinnedScenes();
  if (pinned.contains(scene)) pinned.removeAll(scene);
  else pinned.prepend(scene);
  home_polish_v3::savePinnedScenes(pinned);
  table->viewport()->update();
}

inline void connectRuntimeRepairHooks(QMainWindow * window)
{
  if (!window || window->property("homeV4RepairHooksConnected").toBool()) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table) return;
  window->setProperty("homeV4RepairHooksConnected", true);

  QObject::connect(table, &QTableWidget::cellClicked, window, [window, table](int row, int column) {
    if (row < 0 || row >= table->rowCount()) return;
    const QString scene = sceneNameAt(table, row);
    if (column == kPinColumn) togglePinned(table, row);
    QSettings settings;
    settings.setValue(QStringLiteral("studio_home/last_selected_scene"), scene);
    for (const int delay : {20, 140}) {
      QTimer::singleShot(delay, window, [window]() {
        repairTable(window);
        repairShell(window);
        repairInspectorText(window);
      });
    }
    scheduleHomePreview(window, scene);
  });

  QObject::connect(table, &QTableWidget::itemSelectionChanged, window, [window]() {
    QTimer::singleShot(40, window, [window]() {
      repairInspectorText(window);
      refreshHomePreview(window);
    });
  });

  if (table->model()) {
    const auto repair_after_model_change = [window]() {
      for (const int delay : {30, 150, 420}) {
        QTimer::singleShot(delay, window, [window]() {
          repairTable(window);
          repairShell(window);
          ensureUsefulSelection(window);
        });
      }
    };
    QObject::connect(table->model(), &QAbstractItemModel::rowsInserted, window, repair_after_model_change);
    QObject::connect(table->model(), &QAbstractItemModel::modelReset, window, repair_after_model_change);
    QObject::connect(table->model(), &QAbstractItemModel::dataChanged, window, repair_after_model_change);
  }
}

inline void appendV4Style(QMainWindow * window)
{
  if (!window || window->property("homeV4StyleApplied2").toBool()) return;
  window->setProperty("homeV4StyleApplied2", true);
  window->setStyleSheet(window->styleSheet() + QStringLiteral(R"QSS(
QTableWidget#studioHomeSceneTable {
  background:#FFFFFF;
  border:0;
  outline:0;
  selection-background-color:transparent;
  selection-color:#143A5B;
}
QTableWidget#studioHomeSceneTable::item { padding:0px; border:0; }
QHeaderView::section {
  background:#F8FAFC;
  color:#415A70;
  border:0;
  border-bottom:1px solid #DCE5EF;
  padding:0 10px;
  font-size:11px;
  font-weight:800;
}
QLabel#homeV3InspectorPreview {
  background:#111D28;
  color:#D6E2ED;
  border:1px solid #26394A;
  border-radius:8px;
  font-size:11px;
  font-weight:700;
}
)QSS"));
}

inline void applyFixes(QMainWindow * window)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (!window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) return;
  appendV4Style(window);
  repairShell(window);
  repairTable(window);
  connectRuntimeRepairHooks(window);
  ensureUsefulSelection(window);
  repairInspectorText(window);
  refreshHomePreview(window);
}

class HomePolishV4Guard : public QObject
{
public:
  explicit HomePolishV4Guard(QObject * parent) : QObject(parent) {}

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    auto * window = qobject_cast<QMainWindow *>(watched);
    if (!window || !window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
      return QObject::eventFilter(watched, event);
    }
    if (event->type() == QEvent::Show) {
      // v2/v3 still own the underlying data/model setup.  Run after their
      // delayed passes and once more after v3's longest preview timer so this
      // structural presentation remains the final visible state.
      for (const int delay : {1480, 2100, 3400}) {
        QTimer::singleShot(delay, window, [window]() { applyFixes(window); });
      }
    } else if (event->type() == QEvent::Resize) {
      QTimer::singleShot(0, window, [window]() {
        repairShell(window);
        repairTable(window);
        refreshHomePreview(window);
      });
    }
    return QObject::eventFilter(watched, event);
  }
};

inline void install()
{
  if (!qApp || qApp->property("workcellHomePolishV4Installed").toBool()) return;
  qApp->setProperty("workcellHomePolishV4Installed", true);
  qApp->installEventFilter(new HomePolishV4Guard(qApp));
}
}  // namespace home_polish_v4
}  // namespace workcell_builder

inline void workcellHomePolishV4Startup()
{
  workcell_builder::home_polish_v4::install();
}

Q_COREAPP_STARTUP_FUNCTION(workcellHomePolishV4Startup)
