#pragma once

// Home v4 is a surgical runtime polish pass over v3.  It fixes the two issues
// exposed by workstation screenshot acceptance: transparent custom table cells
// were painting on top of the original QTableWidgetItem text, and the Home
// preview attempted to grab a hidden Product View widget instead of owning a
// visible read-only viewport.
#include "workcell_home_polish_v3.hpp"
#include "gui/mainwindow.h"
#include "gui/scene3d_viewport_widget.h"

#include <QAbstractItemModel>
#include <QApplication>
#include <QBrush>
#include <QEvent>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QMetaObject>
#include <QSettings>
#include <QSizePolicy>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

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

inline void hideBackingItemText(QTableWidget * table)
{
  if (!table) return;
  // v3 deliberately keeps the canonical QTableWidgetItems because filtering,
  // sorting and MainWindow row identity read their text/UserRole data.  The
  // custom presentation widgets are transparent, so the item text must be made
  // paint-transparent instead of cleared.
  const int presentation_columns[] = {
    kSceneColumn, kStatusColumn, kRobotColumn, kToolColumn, kUpdatedColumn};
  for (int row = 0; row < table->rowCount(); ++row) {
    for (const int column : presentation_columns) {
      if (QTableWidgetItem * item = table->item(row, column)) {
        item->setForeground(QBrush(Qt::transparent));
        item->setBackground(QBrush(Qt::transparent));
      }
    }
  }
}

inline void repairPresentationLabels(QTableWidget * table)
{
  if (!table) return;
  for (int row = 0; row < table->rowCount(); ++row) {
    if (QWidget * robot_cell = table->cellWidget(row, kRobotColumn)) {
      if (QLabel * label = robot_cell->findChild<QLabel *>(QStringLiteral("homeV3ValueLabel"))) {
        const QString raw = table->item(row, kRobotColumn) ? table->item(row, kRobotColumn)->text() : QString();
        label->setText(home_polish_v3::displayRobot(raw));
      }
    }
    if (QWidget * tool_cell = table->cellWidget(row, kToolColumn)) {
      if (QLabel * label = tool_cell->findChild<QLabel *>(QStringLiteral("homeV3ValueLabel"))) {
        const QString raw = table->item(row, kToolColumn) ? table->item(row, kToolColumn)->text() : QString();
        label->setText(friendlyTool(raw));
      }
    }
    if (QWidget * updated_cell = table->cellWidget(row, kUpdatedColumn)) {
      if (QLabel * label = updated_cell->findChild<QLabel *>(QStringLiteral("homeV3ValueLabel"))) {
        const QString raw = table->item(row, kUpdatedColumn) ? table->item(row, kUpdatedColumn)->text() : QString();
        label->setText(home_polish_v3::relativeUpdatedText(raw));
      }
    }
  }
}

inline void repairTable(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table) return;

  table->setAlternatingRowColors(false);
  table->setWordWrap(false);
  table->setTextElideMode(Qt::ElideRight);
  table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table->verticalHeader()->setDefaultSectionSize(62);
  table->horizontalHeader()->setMinimumHeight(40);
  table->horizontalHeader()->setSectionResizeMode(kSceneColumn, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(kStatusColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kRobotColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kToolColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kUpdatedColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kPinColumn, QHeaderView::Fixed);
  table->setColumnWidth(kStatusColumn, 188);
  table->setColumnWidth(kRobotColumn, 92);
  table->setColumnWidth(kToolColumn, 158);
  table->setColumnWidth(kUpdatedColumn, 112);
  table->setColumnWidth(kPinColumn, 74);
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"),
    QStringLiteral("Tool / Gripper"), QStringLiteral("Task"), QStringLiteral("Launch"),
    QStringLiteral("Updated"), QStringLiteral("Pinned")});

  repairPresentationLabels(table);
  hideBackingItemText(table);
}

inline void repairShell(QMainWindow * window)
{
  if (!window) return;
  // The sidebar is the product identity.  Keep the top bar for the persistent
  // safety contract only, not a third copy of the product name or generic Ready.
  for (QLabel * brand : window->findChildren<QLabel *>(QStringLiteral("studioTopBrand"))) {
    if (brand) brand->hide();
  }
  for (QLabel * status : window->findChildren<QLabel *>(QStringLiteral("studioTopStatusChip"))) {
    if (status) status->hide();
  }
  if (QToolBar * top_bar = window->findChild<QToolBar *>(QStringLiteral("studioTopBar"))) {
    top_bar->setMinimumHeight(50);
    top_bar->setMaximumHeight(50);
  }

  // Home itself is the workcell library, so use one vocabulary everywhere.
  if (QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
    for (QLabel * label : dashboard->findChildren<QLabel *>()) {
      if (label && label->text().trimmed() == QStringLiteral("Scenes")) label->setText(QStringLiteral("Workcells"));
    }
  }

  // The right inspector no longer repeats actions already available through
  // navigation or row interaction.
  if (QToolButton * more = window->findChild<QToolButton *>(QStringLiteral("homeV3InspectorMore"))) more->hide();
  if (QToolButton * details = window->findChild<QToolButton *>(QStringLiteral("homeV3ViewDetails"))) details->hide();
}

inline Scene3DViewportWidget * ensureHomeViewport(QMainWindow * window)
{
  if (!window) return nullptr;
  if (auto * existing = window->findChild<Scene3DViewportWidget *>(QStringLiteral("homeV4InspectorViewport"))) {
    return existing;
  }

  auto * placeholder = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview"));
  if (!placeholder || !placeholder->parentWidget()) return nullptr;
  auto * layout = qobject_cast<QVBoxLayout *>(placeholder->parentWidget()->layout());
  if (!layout) return nullptr;

  auto * viewport = new Scene3DViewportWidget(placeholder->parentWidget());
  viewport->setObjectName(QStringLiteral("homeV4InspectorViewport"));
  viewport->setMinimumHeight(218);
  viewport->setMaximumHeight(242);
  viewport->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  viewport->label_mode = ScenePreviewWidget::LabelMode::Off;
  viewport->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Auto;
  viewport->show_warnings = false;
  viewport->show_safety = false;
  viewport->show_pick_place = false;
  viewport->show_reachability_heatmap = false;
  viewport->show_collision_warnings = false;
  viewport->show_work_envelope = false;
  viewport->show_warning_labels = false;
  viewport->show_task_route = false;
  viewport->show_approach_retreat = false;
  viewport->show_camera_fov = false;
  viewport->show_pick_coverage = false;
  viewport->show_epd_detections = false;
  viewport->show_detection_labels = false;
  viewport->debug_overlays_mode = false;
  viewport->diagnostic_transparency_mode = false;
  viewport->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select;
  viewport->setToolTip(QStringLiteral("Read-only Home preview. Open Product View for the full interactive scene."));

  const int index = layout->indexOf(placeholder);
  layout->insertWidget(index >= 0 ? index : 4, viewport);
  viewport->hide();
  return viewport;
}

inline bool contextMatchesScene(const ScenePreviewWidget::PreviewContext & context, const QString & scene)
{
  if (scene.isEmpty()) return false;
  if (context.scene_id.trimmed().compare(scene, Qt::CaseInsensitive) == 0) return true;
  if (!context.absolute_scene_dir.trimmed().isEmpty() &&
      QFileInfo(context.absolute_scene_dir).fileName().compare(scene, Qt::CaseInsensitive) == 0) return true;
  return false;
}

inline bool loadNativeHomePreview(QMainWindow * window, const QString & scene)
{
  if (!window || scene.isEmpty()) return false;
  auto * home_viewport = ensureHomeViewport(window);
  auto * placeholder = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview"));
  if (!home_viewport || !placeholder) return false;

  auto * main_window = qobject_cast<MainWindow *>(window);
  ScenePreviewWidget * source_preview = main_window
    ? main_window->active_scene_preview_widget()
    : window->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"));
  if (!source_preview || !contextMatchesScene(source_preview->preview_context(), scene)) return false;

  auto * source_viewport = source_preview->findChild<Scene3DViewportWidget *>(QStringLiteral("scene3dViewportWidget"));
  if (!source_viewport || source_viewport->items.isEmpty()) return false;

  home_viewport->scene_name = scene;
  home_viewport->selected_id.clear();
  home_viewport->label_mode = ScenePreviewWidget::LabelMode::Off;
  home_viewport->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Auto;
  home_viewport->ingest_preview_items(source_viewport->items);
  placeholder->hide();
  home_viewport->show();
  home_viewport->raise();

  QTimer::singleShot(0, home_viewport, [home_viewport]() {
    home_viewport->set_isometric_view();
    home_viewport->fit_product_view();
    home_viewport->update();
  });
  QTimer::singleShot(180, home_viewport, [home_viewport]() {
    home_viewport->fit_product_view();
    home_viewport->update();
  });
  return true;
}

inline QString selectedScene(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table || table->currentRow() < 0 || table->currentRow() >= table->rowCount()) return QString();
  return sceneNameAt(table, table->currentRow());
}

inline void scheduleHomePreview(QMainWindow * window, const QString & scene)
{
  if (!window) return;
  const qulonglong generation = window->property("homeV4PreviewGeneration").toULongLong() + 1;
  window->setProperty("homeV4PreviewGeneration", generation);

  if (scene.isEmpty()) {
    if (auto * viewport = ensureHomeViewport(window)) viewport->hide();
    if (auto * placeholder = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview"))) {
      placeholder->setPixmap(QPixmap());
      placeholder->setText(QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here"));
      placeholder->show();
    }
    return;
  }

  if (auto * main_window = qobject_cast<MainWindow *>(window)) {
    // Selecting on Home updates MainWindow's canonical selected-scene state;
    // this refresh populates the same ScenePreviewWidget payload used by the
    // Scene Builder/Product View, without opening another page.
    main_window->refresh_scene_builder_state_from_active_scene();
  }

  if (auto * viewport = ensureHomeViewport(window)) viewport->hide();
  if (auto * placeholder = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview"))) {
    placeholder->setPixmap(QPixmap());
    placeholder->setText(QStringLiteral("LOADING 3D PREVIEW…"));
    placeholder->show();
  }

  for (const int delay : {40, 180, 450, 900, 1700, 2800}) {
    QTimer::singleShot(delay, window, [window, scene, generation, delay]() {
      if (window->property("homeV4PreviewGeneration").toULongLong() != generation) return;
      if (loadNativeHomePreview(window, scene)) return;
      if (delay == 2800) {
        // Keep v3's canonical/cached snapshot path as the final fallback, but
        // do not replace a successfully loaded visible native viewport.
        home_polish_v3::refreshInspectorPreview(window, scene);
      }
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
  settings.setValue(QStringLiteral("studio_home/last_selected_scene"), sceneNameAt(table, row));

  // MainWindow owns canonical scene selection and is connected to cellClicked.
  // Invoke that signal once for programmatic startup selection so Home and the
  // hidden Scene Builder preview payload stay on the same selected scene.
  QMetaObject::invokeMethod(table, "cellClicked", Qt::DirectConnection,
    Q_ARG(int, row), Q_ARG(int, kSceneColumn));
  scheduleHomePreview(window, sceneNameAt(table, row));
}

inline void connectRuntimeRepairHooks(QMainWindow * window)
{
  if (!window || window->property("homeV4RepairHooksConnected").toBool()) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  if (!table) return;
  window->setProperty("homeV4RepairHooksConnected", true);

  QObject::connect(table, &QTableWidget::cellClicked, window, [window, table](int row, int) {
    if (row < 0 || row >= table->rowCount()) return;
    const QString scene = sceneNameAt(table, row);
    QSettings settings;
    settings.setValue(QStringLiteral("studio_home/last_selected_scene"), scene);
    QTimer::singleShot(20, window, [window]() { repairTable(window); repairShell(window); });
    scheduleHomePreview(window, scene);
  });

  if (table->model()) {
    const auto repair_after_model_change = [window]() {
      for (const int delay : {30, 120}) {
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
  if (!window || window->property("homeV4StyleApplied").toBool()) return;
  window->setProperty("homeV4StyleApplied", true);
  window->setStyleSheet(window->styleSheet() + QStringLiteral(R"QSS(
QWidget#homeV4InspectorViewport {
  background:#121D28;
  border:1px solid #243747;
  border-radius:8px;
}
QTableWidget#studioHomeSceneTable::item {
  padding:0px;
}
QTableWidget#studioHomeSceneTable::item:selected {
  background:#EAF2FB;
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
  ensureHomeViewport(window);
  connectRuntimeRepairHooks(window);
  ensureUsefulSelection(window);
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
      // v2/v3 both install delayed polish passes.  v4 intentionally runs after
      // those passes so screenshot acceptance cannot regress back to duplicate
      // item painting or placeholder-only preview.
      for (const int delay : {1480, 1900, 2600}) {
        QTimer::singleShot(delay, window, [window]() { applyFixes(window); });
      }
    } else if (event->type() == QEvent::Resize) {
      QTimer::singleShot(0, window, [window]() { repairShell(window); repairTable(window); });
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
