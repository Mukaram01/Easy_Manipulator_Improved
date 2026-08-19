#pragma once

// Final Home-page composition layered over the proven v2 shell.  This keeps the
// existing scene model, filters, pinned-scene persistence and safety actions
// intact while replacing the visible Home composition with the approved
// workcell-library design.
#include "workcell_home_polish_v2.hpp"
#include "scene_preview_widget.h"

#include <QAbstractItemModel>
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMenu>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

namespace workcell_builder
{
namespace home_polish_v3
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
  if (!table || row < 0 || row >= table->rowCount()) return QString();
  QTableWidgetItem * item = table->item(row, kSceneColumn);
  if (!item) return QString();
  const QString tooltip = item->toolTip().trimmed();
  return tooltip.isEmpty() ? item->text().trimmed() : tooltip;
}

inline QString sceneRoot(QMainWindow * window)
{
  if (!window) return QString();
  if (auto * summary = window->findChild<QLabel *>(QStringLiteral("dashboardSummaryLabel"))) {
    const QRegularExpression re(QStringLiteral("Source:\\s*([^|\\n]+)"));
    const auto match = re.match(summary->text());
    if (match.hasMatch()) return match.captured(1).trimmed();
  }
  if (auto * footer = window->findChild<QLabel *>(QStringLiteral("studioHomeFooter"))) {
    const QRegularExpression re(QStringLiteral("Source:\\s*([^·\\n]+)"));
    const auto match = re.match(footer->text());
    if (match.hasMatch()) return match.captured(1).trimmed();
  }
  return QString();
}

inline QString friendlySceneName(const QString & scene_id)
{
  QStringList words;
  for (QString token : scene_id.split('_', Qt::SkipEmptyParts)) {
    const QString lower = token.toLower();
    if (QRegularExpression(QStringLiteral("^ur\\d+$")).match(lower).hasMatch() ||
        lower == QStringLiteral("2f") || lower == QStringLiteral("3f")) {
      token = lower.toUpper();
    } else if (lower == QStringLiteral("airpick4")) {
      token = QStringLiteral("AirPick4");
    } else {
      token = lower;
      if (!token.isEmpty()) token[0] = token[0].toUpper();
    }
    words << token;
  }
  return words.isEmpty() ? scene_id : words.join(' ');
}

inline QString displayRobot(QString robot)
{
  robot = robot.trimmed();
  if (robot.isEmpty()) return QStringLiteral("—");
  if (robot.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0) return QStringLiteral("Unknown");
  if (QRegularExpression(QStringLiteral("^ur\\d+$"), QRegularExpression::CaseInsensitiveOption).match(robot).hasMatch()) {
    return robot.toUpper();
  }
  return robot;
}

inline QString displayTool(QString tool)
{
  tool = tool.trimmed();
  if (tool.isEmpty()) return QStringLiteral("—");
  if (tool.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0) return QStringLiteral("Unknown");
  if (tool.compare(QStringLiteral("robotiq_85"), Qt::CaseInsensitive) == 0 ||
      tool.compare(QStringLiteral("robotiq_2f_85"), Qt::CaseInsensitive) == 0) {
    return QStringLiteral("Robotiq 2F-85");
  }
  if (tool.compare(QStringLiteral("single_suction"), Qt::CaseInsensitive) == 0) return QStringLiteral("Single Suction");
  return tool.replace('_', ' ');
}

inline QStringList pinnedScenes()
{
  QSettings settings;
  QStringList values = settings.value(QStringLiteral("studio_home/pinned_scenes")).toStringList();
  QStringList result;
  for (const QString & value : values) {
    const QString scene = value.trimmed();
    if (!scene.isEmpty() && !result.contains(scene)) result.append(scene);
  }
  return result;
}

inline void savePinnedScenes(const QStringList & scenes)
{
  QSettings settings;
  settings.setValue(QStringLiteral("studio_home/pinned_scenes"), scenes);
}

inline void triggerAction(QMainWindow * window, const QStringList & candidates)
{
  if (!window) return;
  for (QAction * action : window->findChildren<QAction *>()) {
    if (!action) continue;
    QString text = action->text();
    text.remove('&');
    for (const QString & candidate : candidates) {
      if (text.compare(candidate, Qt::CaseInsensitive) == 0 && action->isEnabled()) {
        action->trigger();
        return;
      }
    }
  }
}

inline QString cachedPreviewPath(QMainWindow * window, const QString & scene)
{
  const QString root = sceneRoot(window);
  if (root.isEmpty() || scene.isEmpty()) return QString();
  const QDir scene_dir(QDir(root).filePath(scene));
  const QStringList candidates = {
    QStringLiteral("preview/workcell_studio_canvas_snapshot.png"),
    QStringLiteral("preview/scene3d_preview.png"),
    QStringLiteral("preview/static_preview.png"),
    QStringLiteral("smoke/scene3d_smoke.png"),
    QStringLiteral("smoke/scene3d_gui_smoke.png"),
    QStringLiteral("acceptance/scene3d_smoke.png"),
    QStringLiteral("acceptance/scene3d_gui_smoke.png"),
    QStringLiteral("preview_launch/scene3d_preview.png"),
    QStringLiteral("preview_launch/product_view.png"),
    QStringLiteral("generated/scene3d_preview.png")};
  for (const QString & relative : candidates) {
    const QString candidate = scene_dir.filePath(relative);
    if (QFileInfo::exists(candidate)) return candidate;
  }
  return QString();
}

inline QPixmap cropCover(const QPixmap & source, const QSize & size)
{
  if (source.isNull() || size.isEmpty()) return QPixmap();
  const QPixmap scaled = source.scaled(size, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation);
  const int x = qMax(0, (scaled.width() - size.width()) / 2);
  const int y = qMax(0, (scaled.height() - size.height()) / 2);
  return scaled.copy(x, y, qMin(size.width(), scaled.width()), qMin(size.height(), scaled.height()));
}

inline bool pixmapHasUsefulVariation(const QPixmap & pixmap)
{
  if (pixmap.isNull() || pixmap.width() < 48 || pixmap.height() < 32) return false;
  const QImage image = pixmap.toImage().convertToFormat(QImage::Format_RGB32);
  if (image.isNull()) return false;
  QColor first;
  bool have_first = false;
  int varied_samples = 0;
  const int step_x = qMax(1, image.width() / 12);
  const int step_y = qMax(1, image.height() / 8);
  for (int y = 0; y < image.height(); y += step_y) {
    for (int x = 0; x < image.width(); x += step_x) {
      const QColor color(image.pixel(x, y));
      if (!have_first) {
        first = color;
        have_first = true;
      } else if (qAbs(color.red() - first.red()) + qAbs(color.green() - first.green()) +
                 qAbs(color.blue() - first.blue()) > 42) {
        ++varied_samples;
        if (varied_samples >= 4) return true;
      }
    }
  }
  return false;
}

inline QPixmap liveCanonicalPreview(QMainWindow * window, const QString & scene, const QSize & size)
{
  if (!window || scene.isEmpty()) return QPixmap();
  auto * source_preview = window->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"));
  if (!source_preview || !source_preview->runtime_preview_has_usable_content()) return QPixmap();
  const ScenePreviewWidget::PreviewContext context = source_preview->preview_context();
  if (!context.scene_id.trimmed().isEmpty() && context.scene_id.trimmed() != scene) return QPixmap();

  QWidget * source = source_preview->findChild<QWidget *>(QStringLiteral("embeddedWeb3dProductView"));
  if (!source || source->width() < 48 || source->height() < 32) {
    source = source_preview->findChild<QWidget *>(QStringLiteral("scene3dViewportWidget"));
  }
  if (!source || source->width() < 48 || source->height() < 32) return QPixmap();

  source->ensurePolished();
  const QPixmap shot = source->grab();
  if (!pixmapHasUsefulVariation(shot)) return QPixmap();
  return cropCover(shot, size);
}

inline void refreshInspectorPreview(QMainWindow * window, const QString & scene)
{
  auto * preview = window ? window->findChild<QLabel *>(QStringLiteral("homeV3InspectorPreview")) : nullptr;
  if (!preview) return;
  preview->setPixmap(QPixmap());
  const QSize target(qMax(300, preview->width()), qMax(204, preview->height()));

  const QPixmap live = liveCanonicalPreview(window, scene, target);
  if (!live.isNull()) {
    preview->setText(QString());
    preview->setPixmap(live);
    preview->setToolTip(QStringLiteral("Live read-only preview from the canonical Product View renderer."));
    return;
  }

  const QString cached_path = cachedPreviewPath(window, scene);
  const QPixmap cached(cached_path);
  if (!cached.isNull()) {
    preview->setText(QString());
    preview->setPixmap(cropCover(cached, target));
    preview->setToolTip(cached_path);
    return;
  }

  preview->setText(scene.isEmpty()
    ? QStringLiteral("SELECT A WORKCELL\nPreview and readiness appear here")
    : QStringLiteral("PREVIEW PREPARING\nThe selected workcell will appear here"));
  preview->setToolTip(QStringLiteral("No live or cached preview is ready yet."));
}

inline void schedulePreviewRefresh(QMainWindow * window, const QString & scene)
{
  if (!window) return;
  const qulonglong generation = window->property("homeV3PreviewGeneration").toULongLong() + 1;
  window->setProperty("homeV3PreviewGeneration", generation);
  for (const int delay : {0, 180, 600, 1400, 2800}) {
    QTimer::singleShot(delay, window, [window, scene, generation]() {
      if (window->property("homeV3PreviewGeneration").toULongLong() != generation) return;
      refreshInspectorPreview(window, scene);
    });
  }
}

inline QPixmap rowThumbnail(QMainWindow * window, const QString & scene)
{
  const QPixmap cached(cachedPreviewPath(window, scene));
  if (!cached.isNull()) return cropCover(cached, QSize(62, 42));

  QPixmap fallback(62, 42);
  fallback.fill(QColor(QStringLiteral("#EEF3F8")));
  QPainter painter(&fallback);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(QColor(QStringLiteral("#9AAFC2")), 2));
  painter.drawRoundedRect(QRectF(10, 7, 42, 28), 5, 5);
  painter.drawLine(QPointF(17, 29), QPointF(45, 14));
  painter.drawEllipse(QPointF(25, 21), 3, 3);
  return fallback;
}

inline QWidget * makeSceneCell(QMainWindow * window, QTableWidget * table, int row)
{
  const QString scene = sceneNameAt(table, row);
  auto * cell = new QWidget(table);
  cell->setObjectName(QStringLiteral("homeV3SceneCell"));
  cell->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * layout = new QHBoxLayout(cell);
  layout->setContentsMargins(10, 5, 6, 5);
  layout->setSpacing(10);

  auto * thumb = new QLabel(cell);
  thumb->setObjectName(QStringLiteral("homeV3SceneThumb"));
  thumb->setFixedSize(62, 42);
  thumb->setPixmap(rowThumbnail(window, scene));
  thumb->setAlignment(Qt::AlignCenter);
  thumb->setAttribute(Qt::WA_TransparentForMouseEvents, true);

  auto * text_host = new QWidget(cell);
  text_host->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * text_layout = new QVBoxLayout(text_host);
  text_layout->setContentsMargins(0, 2, 0, 1);
  text_layout->setSpacing(1);
  auto * title = new QLabel(friendlySceneName(scene), text_host);
  title->setObjectName(QStringLiteral("homeV3SceneTitle"));
  title->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * id = new QLabel(scene, text_host);
  id->setObjectName(QStringLiteral("homeV3SceneId"));
  id->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  text_layout->addWidget(title);
  text_layout->addWidget(id);

  layout->addWidget(thumb);
  layout->addWidget(text_host, 1);
  return cell;
}

inline QWidget * makeStatusCell(QTableWidget * table, int row)
{
  const QString raw = table->item(row, kStatusColumn) ? table->item(row, kStatusColumn)->text() : QStringLiteral("BLOCKED");
  const QString status = home_polish_v2::cleanStatusText(raw).toUpper();
  auto * cell = new QWidget(table);
  cell->setObjectName(QStringLiteral("homeV3StatusCell"));
  cell->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * layout = new QVBoxLayout(cell);
  layout->setContentsMargins(9, 5, 5, 4);
  layout->setSpacing(0);
  auto * primary = new QLabel(cell);
  primary->setObjectName(QStringLiteral("homeV3StatusPrimary"));
  primary->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * secondary = new QLabel(cell);
  secondary->setObjectName(QStringLiteral("homeV3StatusSecondary"));
  secondary->setAttribute(Qt::WA_TransparentForMouseEvents, true);

  if (status.contains(QStringLiteral("READY"))) {
    primary->setText(QStringLiteral("●  Ready"));
    primary->setProperty("statusKind", QStringLiteral("ready"));
    secondary->setText(QStringLiteral("Ready to simulate"));
  } else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) {
    primary->setText(QStringLiteral("▲  Needs Attention"));
    primary->setProperty("statusKind", QStringLiteral("attention"));
    secondary->setText(QStringLiteral("Warnings to review"));
  } else {
    primary->setText(QStringLiteral("●  Blocked"));
    primary->setProperty("statusKind", QStringLiteral("blocked"));
    secondary->setText(QStringLiteral("Cannot run yet"));
  }
  layout->addWidget(primary);
  layout->addWidget(secondary);
  return cell;
}

inline QWidget * makeSimpleValueCell(QTableWidget * table, const QString & text, bool muted = false)
{
  auto * cell = new QWidget(table);
  cell->setObjectName(QStringLiteral("homeV3ValueCell"));
  cell->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  auto * layout = new QHBoxLayout(cell);
  layout->setContentsMargins(7, 0, 5, 0);
  auto * label = new QLabel(text, cell);
  label->setObjectName(QStringLiteral("homeV3ValueLabel"));
  label->setProperty("mutedValue", muted);
  label->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  layout->addWidget(label);
  return cell;
}

inline QString relativeUpdatedText(const QString & exact)
{
  QDateTime time = QDateTime::fromString(exact.trimmed(), QStringLiteral("yyyy-MM-dd HH:mm"));
  if (!time.isValid()) return exact.trimmed();
  time.setTimeSpec(Qt::LocalTime);
  const qint64 seconds = time.secsTo(QDateTime::currentDateTime());
  if (seconds < 0) return exact.trimmed();
  if (seconds < 60) return QStringLiteral("Just now");
  if (seconds < 3600) return QStringLiteral("%1 min ago").arg(qMax(qint64(1), seconds / 60));
  if (seconds < 86400) return QStringLiteral("%1 hr ago").arg(qMax(qint64(1), seconds / 3600));
  if (seconds < 172800) return QStringLiteral("Yesterday");
  if (seconds < 604800) return QStringLiteral("%1 days ago").arg(seconds / 86400);
  return time.date().toString(QStringLiteral("dd MMM"));
}

inline void polishTable(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table) return;
  table->setShowGrid(false);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->setSelectionMode(QAbstractItemView::SingleSelection);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setColumnHidden(kTaskColumn, true);
  table->setColumnHidden(kLaunchColumn, true);
  table->verticalHeader()->hide();
  table->verticalHeader()->setDefaultSectionSize(62);
  table->horizontalHeader()->setMinimumHeight(40);
  table->horizontalHeader()->setSectionResizeMode(kSceneColumn, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(kStatusColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kRobotColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kToolColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kUpdatedColumn, QHeaderView::Fixed);
  table->horizontalHeader()->setSectionResizeMode(kPinColumn, QHeaderView::Fixed);
  table->setColumnWidth(kStatusColumn, 190);
  table->setColumnWidth(kRobotColumn, 96);
  table->setColumnWidth(kToolColumn, 155);
  table->setColumnWidth(kUpdatedColumn, 116);
  table->setColumnWidth(kPinColumn, 58);
  table->setHorizontalHeaderLabels({QStringLiteral("Workcell"), QStringLiteral("Status"), QStringLiteral("Robot"),
    QStringLiteral("Tool / Gripper"), QStringLiteral("Task"), QStringLiteral("Launch"),
    QStringLiteral("Updated"), QStringLiteral("Pinned")});

  for (int row = 0; row < table->rowCount(); ++row) {
    table->setRowHeight(row, 62);
    table->setCellWidget(row, kSceneColumn, makeSceneCell(window, table, row));
    table->setCellWidget(row, kStatusColumn, makeStatusCell(table, row));

    const QString robot_raw = table->item(row, kRobotColumn) ? table->item(row, kRobotColumn)->text() : QString();
    const QString tool_raw = table->item(row, kToolColumn) ? table->item(row, kToolColumn)->text() : QString();
    const QString updated_raw = table->item(row, kUpdatedColumn) ? table->item(row, kUpdatedColumn)->text() : QString();
    table->setCellWidget(row, kRobotColumn,
      makeSimpleValueCell(table, displayRobot(robot_raw), robot_raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0));
    table->setCellWidget(row, kToolColumn,
      makeSimpleValueCell(table, displayTool(tool_raw), tool_raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0));
    QWidget * updated_cell = makeSimpleValueCell(table, relativeUpdatedText(updated_raw));
    updated_cell->setToolTip(updated_raw);
    table->setCellWidget(row, kUpdatedColumn, updated_cell);
  }
}

inline QLabel * makeMetaLabel(QWidget * parent, const QString & text, const QString & object_name = QString())
{
  auto * label = new QLabel(text, parent);
  if (!object_name.isEmpty()) label->setObjectName(object_name);
  return label;
}

inline void addMetaRow(QGridLayout * grid, int row, QWidget * parent, const QString & symbol,
  const QString & key_text, const QString & value_object)
{
  auto * icon = makeMetaLabel(parent, symbol);
  icon->setObjectName(QStringLiteral("homeV3MetaIcon"));
  icon->setAlignment(Qt::AlignCenter);
  auto * key = makeMetaLabel(parent, key_text);
  key->setObjectName(QStringLiteral("homeV3MetaKey"));
  auto * value = makeMetaLabel(parent, QStringLiteral("—"), value_object);
  grid->addWidget(icon, row, 0);
  grid->addWidget(key, row, 1);
  grid->addWidget(value, row, 2);
}

inline void setMetaValue(QMainWindow * window, const QString & object_name, const QString & value, bool muted = false)
{
  if (auto * label = window ? window->findChild<QLabel *>(object_name) : nullptr) {
    label->setText(value);
    label->setProperty("mutedValue", muted);
    label->style()->unpolish(label);
    label->style()->polish(label);
  }
}

inline void updateInspector(QMainWindow * window)
{
  if (!window) return;
  auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
  auto * title = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorTitle"));
  auto * id_label = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorId"));
  auto * status_label = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorStatus"));
  auto * readiness = window->findChild<QLabel *>(QStringLiteral("homeV3InspectorReadinessText"));
  auto * pin = window->findChild<QToolButton *>(QStringLiteral("homeV3InspectorPin"));
  auto * view_details = window->findChild<QToolButton *>(QStringLiteral("homeV3ViewDetails"));
  const int row = table ? table->currentRow() : -1;
  const bool has_scene = table && row >= 0 && row < table->rowCount();
  const QString scene = has_scene ? sceneNameAt(table, row) : QString();

  if (pin) pin->setEnabled(has_scene);
  if (view_details) view_details->setEnabled(has_scene);
  if (!has_scene) {
    if (title) title->setText(QStringLiteral("No workcell selected"));
    if (id_label) id_label->setText(QString());
    if (status_label) {
      status_label->setText(QStringLiteral("SELECT A WORKCELL"));
      status_label->setProperty("statusKind", QStringLiteral("neutral"));
      status_label->style()->unpolish(status_label);
      status_label->style()->polish(status_label);
    }
    setMetaValue(window, QStringLiteral("homeV3MetaRobot"), QStringLiteral("—"), true);
    setMetaValue(window, QStringLiteral("homeV3MetaTool"), QStringLiteral("—"), true);
    setMetaValue(window, QStringLiteral("homeV3MetaTask"), QStringLiteral("—"), true);
    setMetaValue(window, QStringLiteral("homeV3MetaLaunch"), QStringLiteral("—"), true);
    setMetaValue(window, QStringLiteral("homeV3MetaUpdated"), QStringLiteral("—"), true);
    if (readiness) readiness->setText(QStringLiteral("Select a workcell to review its readiness."));
    schedulePreviewRefresh(window, QString());
    return;
  }

  const QString raw_status = table->item(row, kStatusColumn) ? table->item(row, kStatusColumn)->text() : QStringLiteral("BLOCKED");
  const QString status = home_polish_v2::cleanStatusText(raw_status).toUpper();
  const QString robot_raw = table->item(row, kRobotColumn) ? table->item(row, kRobotColumn)->text() : QStringLiteral("—");
  const QString tool_raw = table->item(row, kToolColumn) ? table->item(row, kToolColumn)->text() : QStringLiteral("—");
  const QString task = table->item(row, kTaskColumn) ? table->item(row, kTaskColumn)->text() : QStringLiteral("—");
  const QString launch = table->item(row, kLaunchColumn) ? table->item(row, kLaunchColumn)->text() : QStringLiteral("—");
  const QString updated = table->item(row, kUpdatedColumn) ? table->item(row, kUpdatedColumn)->text() : QStringLiteral("—");

  if (title) title->setText(friendlySceneName(scene));
  if (id_label) id_label->setText(scene);
  if (status_label) {
    if (status.contains(QStringLiteral("READY"))) {
      status_label->setText(QStringLiteral("●  Ready"));
      status_label->setProperty("statusKind", QStringLiteral("ready"));
    } else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) {
      status_label->setText(QStringLiteral("▲  Needs Attention"));
      status_label->setProperty("statusKind", QStringLiteral("attention"));
    } else {
      status_label->setText(QStringLiteral("●  Blocked"));
      status_label->setProperty("statusKind", QStringLiteral("blocked"));
    }
    status_label->style()->unpolish(status_label);
    status_label->style()->polish(status_label);
  }

  setMetaValue(window, QStringLiteral("homeV3MetaRobot"), displayRobot(robot_raw),
    robot_raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0);
  setMetaValue(window, QStringLiteral("homeV3MetaTool"), displayTool(tool_raw),
    tool_raw.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0);
  setMetaValue(window, QStringLiteral("homeV3MetaTask"), task);
  setMetaValue(window, QStringLiteral("homeV3MetaLaunch"), launch);
  setMetaValue(window, QStringLiteral("homeV3MetaUpdated"), relativeUpdatedText(updated));

  if (readiness) {
    if (status.contains(QStringLiteral("READY"))) {
      readiness->setText(QStringLiteral("This workcell is ready for the next fake-hardware workflow step."));
    } else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) {
      readiness->setText(QStringLiteral("Readiness checks need attention before simulation."));
    } else {
      readiness->setText(QStringLiteral("A blocker prevents the next safe workflow step."));
    }
  }
  if (pin) pin->setText(pinnedScenes().contains(scene) ? QStringLiteral("★") : QStringLiteral("☆"));
  schedulePreviewRefresh(window, scene);
}

inline void addMenuAction(QMenu * menu, QMainWindow * window, const QString & label,
  const std::function<void()> & callback)
{
  if (!menu || !window) return;
  QAction * action = menu->addAction(label);
  QObject::connect(action, &QAction::triggered, window, [callback](bool) { callback(); });
}

inline void buildInspector(QMainWindow * window)
{
  if (!window || window->findChild<QWidget *>(QStringLiteral("homeV3InspectorContent"))) return;
  QWidget * details_panel = nullptr;
  for (QWidget * candidate : window->findChildren<QWidget *>(QStringLiteral("studioHomeDetailsCard"))) {
    if (candidate && candidate->layout()) {
      details_panel = candidate;
      break;
    }
  }
  if (!details_panel || !details_panel->layout()) return;
  details_panel->setMinimumWidth(360);
  details_panel->setMaximumWidth(420);

  for (QWidget * child : details_panel->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) {
    if (child) child->hide();
  }

  auto * content = new QWidget(details_panel);
  content->setObjectName(QStringLiteral("homeV3InspectorContent"));
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(16, 16, 16, 16);
  layout->setSpacing(9);

  auto * top = new QHBoxLayout();
  auto * eyebrow = new QLabel(QStringLiteral("WORKCELL DETAILS"), content);
  eyebrow->setObjectName(QStringLiteral("homeV3InspectorEyebrow"));
  top->addWidget(eyebrow);
  top->addStretch(1);

  auto * pin = new QToolButton(content);
  pin->setObjectName(QStringLiteral("homeV3InspectorPin"));
  pin->setText(QStringLiteral("☆"));
  pin->setToolTip(QStringLiteral("Pin or unpin this workcell"));
  auto * more = new QToolButton(content);
  more->setObjectName(QStringLiteral("homeV3InspectorMore"));
  more->setText(QStringLiteral("⋮"));
  more->setPopupMode(QToolButton::InstantPopup);
  auto * menu = new QMenu(more);
  addMenuAction(menu, window, QStringLiteral("Open in Scene Builder"), [window]() {
    triggerAction(window, {QStringLiteral("Open Scene Builder"), QStringLiteral("Open in Scene Builder")});
  });
  addMenuAction(menu, window, QStringLiteral("Open Product View"), [window]() {
    triggerAction(window, {QStringLiteral("Product View"), QStringLiteral("Open Product View")});
  });
  menu->addSeparator();
  addMenuAction(menu, window, QStringLiteral("Validate"), [window]() {
    triggerAction(window, {QStringLiteral("Validate")});
  });
  addMenuAction(menu, window, QStringLiteral("Generate Package"), [window]() {
    triggerAction(window, {QStringLiteral("Generate Scene Package")});
  });
  addMenuAction(menu, window, QStringLiteral("Simulate · Fake Hardware"), [window]() {
    triggerAction(window, {QStringLiteral("Open RViz Truth Preview"), QStringLiteral("Plan/Simulate Preview"),
      QStringLiteral("Plan / Simulate")});
  });
  more->setMenu(menu);
  top->addWidget(pin);
  top->addWidget(more);

  auto * title = new QLabel(QStringLiteral("No workcell selected"), content);
  title->setObjectName(QStringLiteral("homeV3InspectorTitle"));
  title->setWordWrap(true);
  auto * id = new QLabel(content);
  id->setObjectName(QStringLiteral("homeV3InspectorId"));
  auto * status = new QLabel(QStringLiteral("SELECT A WORKCELL"), content);
  status->setObjectName(QStringLiteral("homeV3InspectorStatus"));
  status->setProperty("statusKind", QStringLiteral("neutral"));
  status->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);

  auto * preview = new QLabel(content);
  preview->setObjectName(QStringLiteral("homeV3InspectorPreview"));
  preview->setAlignment(Qt::AlignCenter);
  preview->setMinimumHeight(210);
  preview->setMaximumHeight(238);
  preview->setScaledContents(false);

  auto * meta_host = new QWidget(content);
  meta_host->setObjectName(QStringLiteral("homeV3MetaHost"));
  auto * meta = new QGridLayout(meta_host);
  meta->setContentsMargins(2, 2, 2, 2);
  meta->setHorizontalSpacing(8);
  meta->setVerticalSpacing(7);
  meta->setColumnStretch(2, 1);
  addMetaRow(meta, 0, meta_host, QStringLiteral("R"), QStringLiteral("Robot"), QStringLiteral("homeV3MetaRobot"));
  addMetaRow(meta, 1, meta_host, QStringLiteral("T"), QStringLiteral("Tool / Gripper"), QStringLiteral("homeV3MetaTool"));
  addMetaRow(meta, 2, meta_host, QStringLiteral("▣"), QStringLiteral("Task"), QStringLiteral("homeV3MetaTask"));
  addMetaRow(meta, 3, meta_host, QStringLiteral("▶"), QStringLiteral("Launch (Fake Hardware)"), QStringLiteral("homeV3MetaLaunch"));
  addMetaRow(meta, 4, meta_host, QStringLiteral("◷"), QStringLiteral("Last Updated"), QStringLiteral("homeV3MetaUpdated"));

  auto * readiness_card = new QFrame(content);
  readiness_card->setObjectName(QStringLiteral("homeV3ReadinessCard"));
  auto * readiness_layout = new QVBoxLayout(readiness_card);
  readiness_layout->setContentsMargins(11, 9, 11, 8);
  readiness_layout->setSpacing(3);
  auto * readiness = new QLabel(QStringLiteral("Select a workcell to review its readiness."), readiness_card);
  readiness->setObjectName(QStringLiteral("homeV3InspectorReadinessText"));
  readiness->setWordWrap(true);
  auto * view_details = new QToolButton(readiness_card);
  view_details->setObjectName(QStringLiteral("homeV3ViewDetails"));
  view_details->setText(QStringLiteral("View details  →"));
  view_details->setToolButtonStyle(Qt::ToolButtonTextOnly);
  view_details->setCursor(Qt::PointingHandCursor);
  readiness_layout->addWidget(readiness);
  readiness_layout->addWidget(view_details, 0, Qt::AlignLeft);

  layout->addLayout(top);
  layout->addWidget(title);
  layout->addWidget(id);
  layout->addWidget(status, 0, Qt::AlignLeft);
  layout->addWidget(preview);
  layout->addWidget(meta_host);
  layout->addWidget(readiness_card);
  layout->addStretch(1);
  details_panel->layout()->addWidget(content);

  QObject::connect(pin, &QToolButton::clicked, window, [window]() {
    auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
    if (!table || table->currentRow() < 0) return;
    const QString scene = sceneNameAt(table, table->currentRow());
    if (scene.isEmpty()) return;
    QStringList pinned = pinnedScenes();
    if (pinned.contains(scene)) pinned.removeAll(scene);
    else pinned.prepend(scene);
    savePinnedScenes(pinned);
    updateInspector(window);
  });
  QObject::connect(view_details, &QToolButton::clicked, window, [window]() {
    triggerAction(window, {QStringLiteral("Validate"), QStringLiteral("Validation")});
  });

  if (auto * source_preview = window->findChild<ScenePreviewWidget *>(QStringLiteral("scenePreviewWidget"))) {
    QObject::connect(source_preview, &ScenePreviewWidget::embedded_product_view_runtime_state_changed,
      window, [window](const QString &, bool) {
        auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
        const QString scene = table && table->currentRow() >= 0 ? sceneNameAt(table, table->currentRow()) : QString();
        schedulePreviewRefresh(window, scene);
      });
    QObject::connect(source_preview, &ScenePreviewWidget::post_save_product_view_refresh_finished,
      window, [window](int, quint64, quint64, bool, const QString &) {
        auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"));
        const QString scene = table && table->currentRow() >= 0 ? sceneNameAt(table, table->currentRow()) : QString();
        schedulePreviewRefresh(window, scene);
      });
  }
}

inline QFrame * makeKpiCard(QWidget * parent, const QString & object_name, const QString & icon_text,
  const QString & title, const QString & value_object, const QString & detail,
  const QString & color, const QString & icon_background)
{
  auto * card = new QFrame(parent);
  card->setObjectName(object_name);
  card->setMinimumHeight(96);
  card->setMaximumHeight(108);
  auto * layout = new QHBoxLayout(card);
  layout->setContentsMargins(15, 12, 15, 12);
  layout->setSpacing(12);

  auto * icon = new QLabel(icon_text, card);
  icon->setObjectName(QStringLiteral("homeV3KpiIcon"));
  icon->setAlignment(Qt::AlignCenter);
  icon->setFixedSize(42, 42);
  icon->setStyleSheet(QStringLiteral("background:%1;color:%2;border:0;border-radius:21px;font-size:20px;font-weight:900;")
    .arg(icon_background, color));

  auto * copy = new QWidget(card);
  auto * copy_layout = new QVBoxLayout(copy);
  copy_layout->setContentsMargins(0, 0, 0, 0);
  copy_layout->setSpacing(1);
  auto * title_label = new QLabel(title, copy);
  title_label->setObjectName(QStringLiteral("homeV3KpiTitle"));
  auto * value = new QLabel(QStringLiteral("0"), copy);
  value->setObjectName(value_object);
  value->setStyleSheet(QStringLiteral("font-size:25px;font-weight:800;color:%1;background:transparent;border:0;").arg(color));
  auto * detail_label = new QLabel(detail, copy);
  detail_label->setObjectName(QStringLiteral("homeV3KpiDetail"));
  copy_layout->addWidget(title_label);
  copy_layout->addWidget(value);
  copy_layout->addWidget(detail_label);
  layout->addWidget(icon, 0, Qt::AlignVCenter);
  layout->addWidget(copy, 1);
  return card;
}

inline void buildKpiRow(QMainWindow * window)
{
  if (!window || window->findChild<QFrame *>(QStringLiteral("homeV3KpiRow"))) return;
  auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
  auto * root = dashboard ? qobject_cast<QVBoxLayout *>(dashboard->layout()) : nullptr;
  if (!dashboard || !root) return;
  if (auto * old = window->findChild<QFrame *>(QStringLiteral("homeV2KpiRow"))) old->hide();

  auto * row = new QFrame(dashboard);
  row->setObjectName(QStringLiteral("homeV3KpiRow"));
  auto * layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(12);
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV3KpiTotal"), QStringLiteral("◇"), QStringLiteral("Total Workcells"),
    QStringLiteral("homeV3TotalValue"), QStringLiteral("Across all folders"), QStringLiteral("#1D5DA8"), QStringLiteral("#EEF4FF")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV3KpiReady"), QStringLiteral("✓"), QStringLiteral("Ready"),
    QStringLiteral("homeV3ReadyValue"), QStringLiteral("Ready to simulate"), QStringLiteral("#15803D"), QStringLiteral("#EAF7F0")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV3KpiAttention"), QStringLiteral("!"), QStringLiteral("Needs Attention"),
    QStringLiteral("homeV3AttentionValue"), QStringLiteral("Warnings to review"), QStringLiteral("#C56A00"), QStringLiteral("#FFF4E5")));
  layout->addWidget(makeKpiCard(row, QStringLiteral("homeV3KpiBlocked"), QStringLiteral("×"), QStringLiteral("Blocked"),
    QStringLiteral("homeV3BlockedValue"), QStringLiteral("Cannot run"), QStringLiteral("#C53232"), QStringLiteral("#FFF0F0")));
  for (int i = 0; i < 4; ++i) layout->setStretch(i, 1);

  QWidget * hero = window->findChild<QWidget *>(QStringLiteral("studioHomeHeroCard"));
  const int hero_index = hero ? root->indexOf(hero) : -1;
  root->insertWidget(hero_index >= 0 ? hero_index + 1 : 0, row);
}

inline void updateKpis(QMainWindow * window)
{
  auto * table = window ? window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable")) : nullptr;
  if (!table) return;
  int ready = 0;
  int attention = 0;
  int blocked = 0;
  for (int row = 0; row < table->rowCount(); ++row) {
    const QString status = table->item(row, kStatusColumn) ? table->item(row, kStatusColumn)->text().toUpper() : QString();
    if (status.contains(QStringLiteral("READY"))) ++ready;
    else if (status.contains(QStringLiteral("WARN")) || status.contains(QStringLiteral("ATTENTION"))) ++attention;
    else ++blocked;
  }
  const struct { const char * object_name; int value; } values[] = {
    {"homeV3TotalValue", table->rowCount()}, {"homeV3ReadyValue", ready},
    {"homeV3AttentionValue", attention}, {"homeV3BlockedValue", blocked}};
  for (const auto & entry : values) {
    if (auto * label = window->findChild<QLabel *>(QString::fromLatin1(entry.object_name))) {
      label->setText(QString::number(entry.value));
    }
  }
}

inline void polishFilters(QMainWindow * window)
{
  if (!window) return;
  auto * bar = window->findChild<QFrame *>(QStringLiteral("homeV2UnifiedFilterBar"));
  if (bar) bar->setObjectName(QStringLiteral("homeV3UnifiedFilterBar"));
  if (auto * search = window->findChild<QLineEdit *>(QStringLiteral("studioHomeSearchBox"))) {
    search->setPlaceholderText(QStringLiteral("Search workcells..."));
    search->setClearButtonEnabled(true);
    search->setMinimumWidth(235);
    search->setMaximumWidth(380);
  }
  if (auto * pinned = window->findChild<QCheckBox *>(QStringLiteral("studioHomeFavoritesOnly"))) {
    pinned->setText(pinned->isChecked() ? QStringLiteral("★  Pinned") : QStringLiteral("☆  Pinned"));
    if (!pinned->property("homeV3PinnedConnected").toBool()) {
      pinned->setProperty("homeV3PinnedConnected", true);
      QObject::connect(pinned, &QCheckBox::toggled, window, [pinned](bool checked) {
        pinned->setText(checked ? QStringLiteral("★  Pinned") : QStringLiteral("☆  Pinned"));
      });
    }
  }
}

inline void polishShell(QMainWindow * window)
{
  if (!window) return;
  if (auto * top_bar = window->findChild<QToolBar *>(QStringLiteral("studioTopBar"))) {
    top_bar->setMinimumHeight(54);
    top_bar->setMaximumHeight(54);
  }
  if (auto * brand = window->findChild<QLabel *>(QStringLiteral("studioTopBrand"))) brand->hide();
  if (auto * status = window->findChild<QLabel *>(QStringLiteral("studioTopStatusChip"))) status->hide();
  if (auto * sidebar = window->findChild<QFrame *>(QStringLiteral("studioHomeSidebar"))) {
    sidebar->setMinimumWidth(200);
    sidebar->setMaximumWidth(214);
  }
  if (auto * nav = window->findChild<QListWidget *>(QStringLiteral("studioHomeSidebarNav"))) {
    if (nav->viewport()) nav->viewport()->setStyleSheet(QStringLiteral("background:#071F39;border:0;"));
    if (nav->count() > 2) nav->item(2)->setHidden(true);  // Home already is the workcell library.
    if (nav->count() > 3) nav->item(3)->setHidden(true);  // Demo Mode is secondary, not primary navigation.
    if (nav->count() > 5) nav->item(5)->setText(QStringLiteral("⚙  System"));
  }
  if (auto * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
    if (auto * root = qobject_cast<QVBoxLayout *>(dashboard->layout())) {
      root->setContentsMargins(26, 18, 26, 16);
      root->setSpacing(12);
    }
  }
  if (auto * hero = window->findChild<QWidget *>(QStringLiteral("studioHomeHeroCard"))) {
    hero->setMaximumHeight(78);
    hero->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  }
  if (auto * title = window->findChild<QLabel *>(QStringLiteral("dashboardTitleLabel"))) title->setText(QStringLiteral("Your workcells"));
  if (auto * subtitle = window->findChild<QLabel *>(QStringLiteral("dashboardSubtitleLabel"))) {
    subtitle->setText(QStringLiteral("Select a workcell or start a new robotic cell."));
  }
  for (QPushButton * button : window->findChildren<QPushButton *>(QStringLiteral("studioHomePrimaryButton"))) if (button) button->hide();
  if (auto * footer = window->findChild<QLabel *>(QStringLiteral("studioHomeFooter"))) footer->hide();

  if (QStatusBar * status_bar = window->statusBar()) {
    QWidget * dashboard = window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"));
    auto * stack = dashboard ? qobject_cast<QStackedWidget *>(dashboard->parentWidget()) : nullptr;
    if (stack) status_bar->setVisible(stack->currentWidget() != dashboard);
  }
}

inline void appendPolishStyle(QMainWindow * window)
{
  if (!window || window->property("homeV3PolishStyleApplied").toBool()) return;
  window->setProperty("homeV3PolishStyleApplied", true);
  window->setStyleSheet(window->styleSheet() + QStringLiteral(R"QSS(
QWidget#workcellStudioDashboardPage { background:#F5F8FC; }
QFrame#homeV3KpiTotal, QFrame#homeV3KpiReady, QFrame#homeV3KpiAttention, QFrame#homeV3KpiBlocked {
  background:#FFFFFF; border:1px solid #DCE5EF; border-radius:10px;
}
QLabel#homeV3KpiTitle { color:#20364E; font-size:11px; font-weight:800; border:0; background:transparent; }
QLabel#homeV3KpiDetail { color:#7C8FA3; font-size:10px; border:0; background:transparent; }
QFrame#homeV3UnifiedFilterBar { background:transparent; border:0; }
QFrame#studioHomeScenesCard, QWidget#studioHomeDetailsCard { background:#FFFFFF; border:1px solid #DCE5EF; border-radius:10px; }
QLabel#homeV3InspectorEyebrow { color:#6C7E91; font-size:10px; font-weight:800; letter-spacing:0.7px; }
QLabel#homeV3InspectorTitle { color:#102B46; font-size:20px; font-weight:800; }
QLabel#homeV3InspectorId { color:#688096; font-size:11px; }
QLabel#homeV3InspectorStatus { border-radius:10px; padding:3px 8px; font-size:10px; font-weight:800; }
QLabel#homeV3InspectorStatus[statusKind="neutral"] { background:#EEF2F7; color:#64748B; border:1px solid #D9E1E9; }
QLabel#homeV3InspectorStatus[statusKind="ready"] { background:#E8F6EE; color:#137A46; border:1px solid #BBDDC9; }
QLabel#homeV3InspectorStatus[statusKind="attention"] { background:#FFF4E5; color:#A45500; border:1px solid #EDB879; }
QLabel#homeV3InspectorStatus[statusKind="blocked"] { background:#FDECEC; color:#B42318; border:1px solid #E6B3AF; }
QLabel#homeV3InspectorPreview { background:#121D28; color:#C7D3DF; border:1px solid #243747; border-radius:8px; font-size:11px; font-weight:700; }
QWidget#homeV3MetaHost { background:transparent; border:0; }
QLabel#homeV3MetaIcon { color:#4B6B8A; font-size:11px; font-weight:800; min-width:18px; }
QLabel#homeV3MetaKey { color:#64788D; font-size:11px; }
QLabel#homeV3MetaRobot, QLabel#homeV3MetaTool, QLabel#homeV3MetaTask, QLabel#homeV3MetaLaunch, QLabel#homeV3MetaUpdated { color:#17324C; font-size:11px; font-weight:800; }
QLabel[mutedValue="true"] { color:#94A3B8; }
QFrame#homeV3ReadinessCard { background:#FFF8EE; border:1px solid #F0DFC6; border-radius:8px; }
QLabel#homeV3InspectorReadinessText { color:#3D5267; font-size:11px; background:transparent; border:0; }
QToolButton#homeV3ViewDetails { color:#0E4F86; background:transparent; border:0; font-weight:800; padding:2px 0; }
QToolButton#homeV3InspectorPin, QToolButton#homeV3InspectorMore { background:transparent; color:#173B5D; border:0; font-size:18px; padding:3px 5px; }
QToolButton#homeV3InspectorPin:hover, QToolButton#homeV3InspectorMore:hover { background:#EDF3F8; border-radius:5px; }
QTableWidget#studioHomeSceneTable { border:1px solid #DCE5EF; border-radius:8px; background:#FFFFFF; outline:0; }
QTableWidget#studioHomeSceneTable::item { border-bottom:1px solid #EEF2F6; padding:6px 9px; color:#253A50; }
QTableWidget#studioHomeSceneTable::item:selected { background:#EDF4FC; color:#163A5C; }
QTableWidget#studioHomeSceneTable QHeaderView::section { background:#F8FAFC; border:0; border-bottom:1px solid #DCE5EF; padding:8px; color:#465A70; font-weight:800; }
QWidget#homeV3SceneCell, QWidget#homeV3StatusCell, QWidget#homeV3ValueCell { background:transparent; border:0; }
QLabel#homeV3SceneThumb { background:#EEF3F8; border:1px solid #DCE5EF; border-radius:5px; }
QLabel#homeV3SceneTitle { color:#153451; font-size:11px; font-weight:800; }
QLabel#homeV3SceneId { color:#74899E; font-size:10px; }
QLabel#homeV3StatusPrimary { font-size:10px; font-weight:800; }
QLabel#homeV3StatusPrimary[statusKind="ready"] { color:#15803D; }
QLabel#homeV3StatusPrimary[statusKind="attention"] { color:#C56A00; }
QLabel#homeV3StatusPrimary[statusKind="blocked"] { color:#C53232; }
QLabel#homeV3StatusSecondary { color:#8192A4; font-size:9px; }
QLabel#homeV3ValueLabel { color:#253A50; font-size:10px; }
QLabel#homeV3ValueLabel[mutedValue="true"] { color:#94A3B8; }
QCheckBox#studioHomeFavoritesOnly { background:#FFFFFF; border:1px solid #CDD9E5; border-radius:6px; padding:8px 12px; color:#174B79; font-weight:700; }
QCheckBox#studioHomeFavoritesOnly::indicator { width:0px; height:0px; }
QCheckBox#studioHomeFavoritesOnly:checked { background:#EDF4FC; border-color:#7EA7CE; }
QToolBar#studioTopBar { padding:5px 14px; border-bottom:1px solid #DDE5EE; background:#FFFFFF; }
QFrame#studioHomeSidebar { background:#071F39; border:0; }
)QSS"));
}

inline void applyPolish(QMainWindow * window)
{
  if (!window || QApplication::arguments().contains(QStringLiteral("--scene3d-smoke"))) return;
  if (!window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) return;
  if (!window->property("homeV3PolishApplied").toBool()) {
    window->setProperty("homeV3PolishApplied", true);
    appendPolishStyle(window);
    buildKpiRow(window);
    buildInspector(window);
    if (auto * table = window->findChild<QTableWidget *>(QStringLiteral("studioHomeSceneTable"))) {
      QObject::connect(table, &QTableWidget::itemSelectionChanged, window, [window]() { updateInspector(window); });
      QObject::connect(table, &QTableWidget::cellClicked, window, [window](int, int) { updateInspector(window); });
      if (table->model()) {
        QObject::connect(table->model(), &QAbstractItemModel::rowsInserted, window, [window]() {
          QTimer::singleShot(10, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
        QObject::connect(table->model(), &QAbstractItemModel::modelReset, window, [window]() {
          QTimer::singleShot(10, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
        QObject::connect(table->model(), &QAbstractItemModel::dataChanged, window, [window]() {
          QTimer::singleShot(10, window, [window]() { polishTable(window); updateKpis(window); updateInspector(window); });
        });
      }
    }
  }
  polishShell(window);
  polishFilters(window);
  polishTable(window);
  updateKpis(window);
  updateInspector(window);
}

class HomePolishGuard : public QObject
{
public:
  explicit HomePolishGuard(QObject * parent) : QObject(parent) {}

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    auto * window = qobject_cast<QMainWindow *>(watched);
    if (!window || !window->findChild<QWidget *>(QStringLiteral("workcellStudioDashboardPage"))) {
      return QObject::eventFilter(watched, event);
    }
    if (event->type() == QEvent::Show) {
      // v2 applies at 180/700 ms.  Run the approved v3 composition afterwards.
      QTimer::singleShot(760, window, [window]() { applyPolish(window); });
      QTimer::singleShot(1350, window, [window]() { applyPolish(window); });
    } else if (event->type() == QEvent::Resize) {
      QTimer::singleShot(0, window, [window]() { polishShell(window); });
    }
    return QObject::eventFilter(watched, event);
  }
};

inline void install()
{
  if (!qApp || qApp->property("workcellHomePolishV3Installed").toBool()) return;
  qApp->setProperty("workcellHomePolishV3Installed", true);
  qApp->installEventFilter(new HomePolishGuard(qApp));
}
}  // namespace home_polish_v3
}  // namespace workcell_builder

inline void workcellHomePolishV3Startup()
{
  workcell_builder::home_polish_v3::install();
}

Q_COREAPP_STARTUP_FUNCTION(workcellHomePolishV3Startup)
