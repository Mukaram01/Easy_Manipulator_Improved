#include "operator_hmi_window.hpp"
#include "operator_simulation_link.hpp"
#include <cstdio>
#include <QMessageBox>

#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "workcell_builder_ui_utils.hpp"

namespace workcell_builder
{

namespace
{

constexpr int kProjectionTimeoutMs = 60000;

QString escaped(const QString & value) { return value.toHtmlEscaped(); }

QString muted(const QString & value)
{
  return QString("<span style='color:%1'>%2</span>").arg(studioTokens().textMuted, escaped(value));
}

QString or_unavailable(const QString & value, const QString & missing = QStringLiteral("UNAVAILABLE"))
{
  return value.trimmed().isEmpty() ? muted(missing) : escaped(value);
}

QString number_or_unavailable(const std::optional<double> & value, const QString & unit, const QString & missing)
{
  return value ? escaped(QString::number(*value) + unit) : muted(missing);
}

QString rows(const QList<QPair<QString, QString>> & entries)
{
  QString html = QStringLiteral("<table cellspacing='0' cellpadding='3' width='100%'>");
  for (const auto & entry : entries) {
    html += QString("<tr><td width='38%' style='color:%1'>%2</td><td>%3</td></tr>")
      .arg(studioTokens().textSecondary, escaped(entry.first), entry.second);
  }
  return html + QStringLiteral("</table>");
}

QString section(const QString & title)
{
  return QString("<div style='color:%1;font-size:9pt;font-weight:700;letter-spacing:1px;margin-top:6px'>%2</div>")
    .arg(studioTokens().textMuted, escaped(title.toUpper()));
}

QString status_colour(const QString & state)
{
  const auto & t = studioTokens();
  if (state == "ok") return t.success;
  if (state == "warn") return t.warning;
  if (state == "error") return t.danger;
  if (state == "info") return t.info;
  return t.textMuted;
}

QString state_text(const QString & value, const QString & state)
{
  return QString("<b style='color:%1'>%2</b>").arg(status_colour(state), escaped(value));
}

void set_badge(QLabel * label, const QString & caption, const QString & value, const QString & state)
{
  label->setText(QString("<span style='font-size:8pt;color:%1'>%2</span><br><b>%3</b>")
                   .arg(studioTokens().textMuted, escaped(caption), escaped(value)));
  label->setProperty("hmiState", state);
  label->style()->unpolish(label);
  label->style()->polish(label);
}

QString when(const std::optional<double> & epoch)
{
  if (!epoch) return {};
  return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(*epoch * 1000.0)).toString("yyyy-MM-dd HH:mm");
}

QLabel * value_label(QWidget * parent)
{
  auto * label = new QLabel(parent);
  label->setTextFormat(Qt::RichText);
  label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  return label;
}

QFrame * card(QWidget * parent, const QString & title, QVBoxLayout ** body)
{
  auto * frame = new QFrame(parent);
  frame->setObjectName("hmiCard");
  auto * layout = new QVBoxLayout(frame);
  layout->setContentsMargins(12, 10, 12, 10);
  layout->setSpacing(6);
  auto * heading = new QLabel(title.toUpper(), frame);
  heading->setObjectName("hmiCardTitle");
  layout->addWidget(heading);
  *body = layout;
  return frame;
}

}  // namespace

// Top view of the authored pick/place zones and, when a replay was evaluated,
// the recorded objects. World X to the right, world Y up.
class CellMapWidget : public QWidget
{
public:
  explicit CellMapWidget(QWidget * parent = nullptr) : QWidget(parent) { setMinimumSize(320, 200); }

  void set_view(const OperatorPreviewView * view, bool show_objects, const QString & selected)
  {
    view_ = view;
    show_objects_ = show_objects;
    selected_ = selected;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    const auto & t = studioTokens();
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(t.previewBg));
    if (!view_ || (!view_->pick_zone_geometry.valid && !view_->place_zone_geometry.valid)) {
      painter.setPen(QColor(t.textMuted));
      painter.drawText(rect().adjusted(12, 12, -12, -12), Qt::AlignCenter | Qt::TextWordWrap,
                       "Zone geometry UNAVAILABLE for this task.");
      return;
    }
    // World-space bounds of everything drawn.
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    auto include = [&](const std::array<double, 3> & c, const std::array<double, 3> & d) {
        min_x = std::min(min_x, c[0] - d[0] / 2); max_x = std::max(max_x, c[0] + d[0] / 2);
        min_y = std::min(min_y, c[1] - d[1] / 2); max_y = std::max(max_y, c[1] + d[1] / 2);
      };
    for (const auto * zone : {&view_->pick_zone_geometry, &view_->place_zone_geometry}) {
      if (zone->valid) include(zone->xyz, zone->dimensions);
    }
    if (show_objects_) {
      for (const auto & object : view_->objects) if (object.has_geometry) include(object.xyz, object.dimensions);
    }
    const double margin = 0.08;
    min_x -= margin; max_x += margin; min_y -= margin; max_y += margin;
    const QRectF area = QRectF(rect()).adjusted(16, 28, -16, -16);
    const double scale = std::min(area.width() / (max_x - min_x), area.height() / (max_y - min_y));
    const QPointF offset(area.center().x() - (min_x + max_x) / 2 * scale,
                         area.center().y() + (min_y + max_y) / 2 * scale);
    auto to_screen = [&](double x, double y) { return QPointF(offset.x() + x * scale, offset.y() - y * scale); };
    auto rect_of = [&](const std::array<double, 3> & c, const std::array<double, 3> & d) {
        return QRectF(to_screen(c[0] - d[0] / 2, c[1] + d[1] / 2), to_screen(c[0] + d[0] / 2, c[1] - d[1] / 2));
      };

    // 10 cm grid.
    painter.setPen(QPen(QColor(t.border), 1, Qt::DotLine));
    for (double x = std::floor(min_x * 10) / 10; x <= max_x; x += 0.1) {
      painter.drawLine(to_screen(x, min_y), to_screen(x, max_y));
    }
    for (double y = std::floor(min_y * 10) / 10; y <= max_y; y += 0.1) {
      painter.drawLine(to_screen(min_x, y), to_screen(max_x, y));
    }
    QFont label_font = font();
    label_font.setPointSizeF(8.5);
    label_font.setBold(true);
    painter.setFont(label_font);
    auto draw_zone = [&](const OperatorZone & zone, const QString & colour, const QString & caption) {
        if (!zone.valid) return;
        const QRectF r = rect_of(zone.xyz, zone.dimensions);
        QColor fill(colour); fill.setAlpha(36);
        painter.setPen(QPen(QColor(colour), 1.6, Qt::DashLine));
        painter.setBrush(fill);
        painter.drawRoundedRect(r, 4, 4);
        // Short captions at the bottom edge keep the top free for object labels;
        // zone IDs are listed in the Task card.
        painter.setPen(QColor(colour));
        const QFontMetricsF metrics(painter.font());
        if (metrics.horizontalAdvance(caption) + 8 <= r.width() && metrics.height() + 4 <= r.height()) {
          painter.drawText(r.adjusted(4, 2, -2, -3), Qt::AlignLeft | Qt::AlignBottom, caption);
        } else {
          painter.drawText(QPointF(r.left(), r.bottom() + metrics.ascent() + 2), caption);
        }
      };
    draw_zone(view_->pick_zone_geometry, t.info, "PICK");
    draw_zone(view_->place_zone_geometry, t.success, "PLACE");

    if (show_objects_) {
      int index = 0;
      for (const auto & object : view_->objects) {
        if (!object.has_geometry) continue;
        const bool label_above = (index++ % 2) == 1;  // neighbours alternate so labels never overlap
        const QRectF r = rect_of(object.xyz, object.dimensions);
        const bool selected = object.object_id == selected_;
        const QColor colour(object.accepted ? t.success : t.textMuted);
        painter.setPen(QPen(selected ? QColor(t.accent) : colour, selected ? 3.0 : 1.5));
        QColor fill(colour); fill.setAlpha(object.accepted ? 120 : 60);
        painter.setBrush(fill);
        painter.drawRect(r);
        painter.setPen(QColor(t.text));
        const QRectF label_box = label_above ? QRectF(r.center().x() - 50, r.top() - 18, 100, 16)
                                             : QRectF(r.center().x() - 50, r.bottom() + 2, 100, 16);
        painter.drawText(label_box, Qt::AlignHCenter | Qt::AlignVCenter,
                         object.class_id + (object.accepted ? QString(" ✓") : QString(" ✕")));
      }
    }
    painter.setPen(QColor(t.textMuted));
    QFont small = font();
    small.setPointSizeF(8);
    painter.setFont(small);
    painter.drawText(QRectF(12, 6, width() - 24, 18), Qt::AlignLeft | Qt::AlignVCenter,
                     "Top view, world frame · grid 10 cm · X →  Y ↑");
  }

private:
  const OperatorPreviewView * view_{nullptr};
  bool show_objects_{false};
  QString selected_;
};

OperatorHmiWindow::OperatorHmiWindow(const OperatorHmiOptions & options, QWidget * parent)
: QMainWindow(parent), options_(options)
{
  setWindowTitle("Workcell Operator HMI");
  setObjectName("operatorHmiWindow");
  projection_timeout_ = new QTimer(this);
  projection_timeout_->setSingleShot(true);
  connect(projection_timeout_, &QTimer::timeout, this, [this]() {
      if (projection_process_ && projection_process_->state() != QProcess::NotRunning) {
        projection_process_->kill();
        log_event("error", "PREVIEW_TIMEOUT", "The configuration projection did not finish in time.");
      }
    });
  build_ui();
  apply_style();
  resize(1366, 768);
}

OperatorHmiWindow::~OperatorHmiWindow() { stop_checks(); }

void OperatorHmiWindow::start()
{
  // Request ids are unique per HMI process, so a decision on another (earlier) HMI's request never matches ours.
  sim_command_id_ = static_cast<int>(QDateTime::currentMSecsSinceEpoch() % 1000000000);
  if (!options_.acceptance_commands.isEmpty()) {
    // Commands written for an earlier HMI process are not replayed.
    QFile existing(options_.acceptance_commands);
    if (existing.open(QIODevice::ReadOnly)) {
      for (const auto & value : QJsonDocument::fromJson(existing.readAll()).array()) {
        last_command_ = std::max(last_command_, value.toObject().value("id").toInt());
      }
    }
    command_poll_ = new QTimer(this);
    command_poll_->setInterval(250);
    connect(command_poll_, &QTimer::timeout, this, [this]() { poll_acceptance_commands(); });
    command_poll_->start();
  }
  if (!options_.request_path.isEmpty()) {
    request_watcher_ = new QFileSystemWatcher(this);
    request_watcher_->addPath(options_.request_path);
    request_watcher_->addPath(QFileInfo(options_.request_path).absolutePath());
    connect(request_watcher_, &QFileSystemWatcher::fileChanged, this, [this]() { check_request_file(); });
    connect(request_watcher_, &QFileSystemWatcher::directoryChanged, this, [this]() { check_request_file(); });
    // The watcher can miss an atomic replace; a slow poll keeps the hand-off robust.
    request_poll_ = new QTimer(this);
    request_poll_->setInterval(1500);
    connect(request_poll_, &QTimer::timeout, this, [this]() { check_request_file(); });
    request_poll_->start();
    render();
    check_request_file();
    if (!request_) {
      log_event("error", "PREVIEW_REQUEST_INVALID", "Workcell Builder's preview request could not be read.");
      render();
    }
    return;
  }
  scene_dir_ = options_.scene_dir;
  render();
  log_event("info", "PREVIEW_OPENED", "Standalone preview of the saved revision of " + scene_dir_);
  refresh_preview(false);
}

void OperatorHmiWindow::check_request_file()
{
  check_simulation_status();
  if (request_watcher_ && !request_watcher_->files().contains(options_.request_path) &&
    QFileInfo::exists(options_.request_path))
  {
    request_watcher_->addPath(options_.request_path);
  }
  QString error;
  const auto request = read_operator_preview_request(options_.request_path, &error);
  if (!request) return;
  const int applied = request_ ? request_->revision : -1;
  if (request->revision <= applied) return;  // nothing newer than what is shown
  if (!request_ || request->activate) {
    apply_request(*request);
    if (request->activate) {
      showNormal();
      raise();
      activateWindow();
    }
    return;
  }
  if (request->revision == pending_revision_) return;  // already announced
  pending_revision_ = request->revision;
  const bool scene_changed = request->scene_dir != request_->scene_dir;
  update_banner_text_->setText(scene_changed
    ? QString("Workcell Builder selected another cell (%1). Refresh to show it.").arg(request->scene_name)
    : QString("Workcell Builder has newer engineering state (%1). Refresh to show it.")
        .arg(request->task_dirty ? "unsaved task edits" : "saved changes"));
  update_banner_->setVisible(true);
  log_event("notice", "ENGINEERING_CHANGE_AVAILABLE", update_banner_text_->text());
  write_status();
}

void OperatorHmiWindow::apply_request(const OperatorPreviewRequest & request)
{
  const bool scene_changed = !request_ || request_->scene_dir != request.scene_dir;
  request_ = request;
  pending_revision_ = -1;
  update_banner_->setVisible(false);
  scene_dir_ = request.scene_dir;
  if (scene_changed) {
    replay_shown_ = false;
    dry_run_ = {};
    dry_run_raw_ = {};
  }
  log_event("info", "PREVIEW_REQUESTED",
    QString("Revision %1 of %2 from Workcell Builder (%3).").arg(request.revision).arg(request.scene_name,
      request.task_dirty ? "unsaved task draft" : "saved task"));
  refresh_preview(replay_shown_);
}

QString OperatorHmiWindow::resolve_script(const QString & name) const
{
  QStringList candidates;
  if (!options_.scripts_dir.isEmpty()) candidates << QDir(options_.scripts_dir).filePath(name);
  if (request_ && !request_->scripts_dir.isEmpty()) candidates << QDir(request_->scripts_dir).filePath(name);
  const QString env = qEnvironmentVariable("WORKCELL_STUDIO_SCRIPTS_DIR");
  if (!env.isEmpty()) candidates << QDir(env).filePath(name);
  candidates << QCoreApplication::applicationDirPath() + "/" + name  // installed beside the HMI
             << QCoreApplication::applicationDirPath() + "/../../share/workcell_builder/scripts/" + name
             << QCoreApplication::applicationDirPath() + "/../../../scripts/" + name
             << QDir::current().filePath("scripts/" + name);
  for (const auto & candidate : candidates) {
    if (QFileInfo::exists(candidate)) return QFileInfo(candidate).absoluteFilePath();
  }
  return {};
}

void OperatorHmiWindow::refresh_preview(bool evaluate_input)
{
  if (scene_dir_.isEmpty()) {
    log_event("error", "SCENE_MISSING", "No cell was selected for the preview.");
    render();
    return;
  }
  const QString script = resolve_script("operator_preview_projection.py");
  if (script.isEmpty()) {
    log_event("error", "PREVIEW_ADAPTER_MISSING",
      "operator_preview_projection.py was not found; the configuration cannot be read.");
    render();
    return;
  }
  if (projection_process_) {
    projection_process_->disconnect(this);
    projection_process_->kill();
    projection_process_->waitForFinished(1000);
    projection_process_->deleteLater();
  }
  const int generation = ++projection_generation_;
  projection_wants_input_ = evaluate_input;
  projection_process_ = new QProcess(this);
  QStringList arguments{script, "--scene", scene_dir_};
  const bool draft = request_ && request_->task_dirty;
  if (draft) arguments << "--draft-stdin";
  if (evaluate_input) arguments << "--evaluate-input";
  details_->setPlainText(QString("Projection command:\npython3 %1\n").arg(arguments.join(' ')));
  connect(projection_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
          [this, generation](int code, QProcess::ExitStatus) { projection_finished(generation, code); });
  set_busy(evaluate_input ? "Running replay check…" : "Reading the engineering configuration…");
  projection_process_->start("python3", arguments);
  if (draft) projection_process_->write(QJsonDocument(request_->task_draft).toJson(QJsonDocument::Compact));
  projection_process_->closeWriteChannel();
  projection_timeout_->start(kProjectionTimeoutMs);
}

void OperatorHmiWindow::projection_finished(int generation, int exit_code)
{
  if (generation != projection_generation_ || !projection_process_) return;
  projection_timeout_->stop();
  const QByteArray out = projection_process_->readAllStandardOutput();
  const QString err = QString::fromUtf8(projection_process_->readAllStandardError()).trimmed();
  projection_process_->deleteLater();
  projection_process_ = nullptr;
  set_busy({});
  QJsonParseError parse{};
  const auto document = QJsonDocument::fromJson(out, &parse);
  if (parse.error != QJsonParseError::NoError || !document.isObject()) {
    log_event("error", "PREVIEW_FAILED", QString("Configuration projection failed (exit %1): %2")
      .arg(exit_code).arg(err.left(400)));
    view_ = parse_operator_preview({}, request_ && request_->layout_dirty);
    view_loaded_ = true;
    render();
    return;
  }
  projection_raw_ = document.object();
  view_ = parse_operator_preview(projection_raw_, request_ && request_->layout_dirty);
  view_loaded_ = true;
  replay_shown_ = projection_wants_input_ && view_.replay_evaluated;
  log_event("info", "PREVIEW_REFRESHED", QString("%1 revision of %2 loaded.")
    .arg(view_.draft ? "Draft" : "Saved", view_.scene_name));
  for (const auto & event : view_.events) log_event(event.level, event.code, event.message);
  if (projection_wants_input_ && view_.input_kind != "replay") {
    log_event("warning", "REPLAY_NOT_RUN", view_.input_kind == "live"
      ? QString("The task uses live perception; the preview evaluates recorded replays only.")
      : QString("Replay check not run: the task input is unavailable (see PERCEPTION_INPUT_UNAVAILABLE)."));
  }
  render();
}

void OperatorHmiWindow::run_gated_dry_run()
{
  const QString script = resolve_script("run_generated_workcell_bundle.py");
  if (script.isEmpty() || !view_.dry_run_available) return;
  if (dry_run_process_ && dry_run_process_->state() != QProcess::NotRunning) return;
  dry_run_dir_ = std::make_unique<QTemporaryDir>(QDir::tempPath() + "/workcell_operator_dry_run_XXXXXX");
  dry_run_process_ = new QProcess(this);
  // Existing operator-permitted, no-motion tool; reports go to a private
  // directory so the engineering scene is not modified.
  const QStringList arguments{script, "--workcell", scene_dir_, "--gated-dry-run", "--dry-run", "--no-replay",
                              "--json", "--output-dir", dry_run_dir_->path()};
  connect(dry_run_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus) { dry_run_finished(code); });
  log_event("info", "OFFLINE_TEST_STARTED", "Generated-cell dry-run started (no robot motion).");
  details_->appendPlainText("\nDry-run command:\npython3 " + arguments.join(' '));
  set_busy("Generated-cell dry-run running (no motion)…");
  dry_run_process_->start("python3", arguments);
}

void OperatorHmiWindow::dry_run_finished(int exit_code)
{
  if (!dry_run_process_) return;
  const QByteArray out = dry_run_process_->readAllStandardOutput();
  const QString err = QString::fromUtf8(dry_run_process_->readAllStandardError()).trimmed();
  const bool crashed = dry_run_process_->exitStatus() != QProcess::NormalExit;
  dry_run_process_->deleteLater();
  dry_run_process_ = nullptr;
  set_busy({});
  if (crashed) {
    log_event("notice", "OFFLINE_TEST_STOPPED", "Generated-cell dry-run was stopped before it finished.");
    render();
    return;
  }
  const auto document = QJsonDocument::fromJson(out);
  dry_run_raw_ = document.object();
  dry_run_ = parse_gated_dry_run(dry_run_raw_);
  if (!dry_run_.parsed) {
    log_event("error", "OFFLINE_TEST_FAILED", QString("Dry-run produced no report (exit %1): %2")
      .arg(exit_code).arg(err.left(400)));
  } else if (dry_run_.status == "PASS") {
    log_event("info", "OFFLINE_TEST_COMPLETED", "Generated-cell dry-run passed. No robot motion was commanded.");
  } else {
    log_event("error", "OFFLINE_TEST_BLOCKED", QString("Generated-cell dry-run %1: %2")
      .arg(dry_run_.status.isEmpty() ? QString("FAILED") : dry_run_.status,
           dry_run_.blockers.isEmpty() ? dry_run_.next_action : dry_run_.blockers.join("; ")));
  }
  details_->appendPlainText("\nDry-run report:\n" + QString::fromUtf8(QJsonDocument(dry_run_raw_).toJson()));
  render();
}

void OperatorHmiWindow::stop_checks()
{
  for (auto * process : {projection_process_, dry_run_process_}) {
    if (process && process->state() != QProcess::NotRunning) {
      process->kill();
      process->waitForFinished(2000);
    }
  }
  if (projection_timeout_) projection_timeout_->stop();
}

void OperatorHmiWindow::set_busy(const QString & activity)
{
  busy_ = activity;
  if (!monitor_status_) return;
  render_controls();
}

void OperatorHmiWindow::log_event(const QString & level, const QString & code, const QString & message)
{
  if (!events_) return;
  events_->insertRow(0);
  const auto & t = studioTokens();
  const QString colour = level == "error" ? t.danger : level == "warning" ? t.warning
    : level == "notice" ? t.info : t.textSecondary;
  const QStringList cells{QTime::currentTime().toString("HH:mm:ss"), level.toUpper(), code, message};
  for (int column = 0; column < cells.size(); ++column) {
    auto * item = new QTableWidgetItem(cells.at(column));
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    if (column == 1) item->setForeground(QColor(colour));
    item->setToolTip(cells.at(column));
    events_->setItem(0, column, item);
  }
  while (events_->rowCount() > 400) events_->removeRow(events_->rowCount() - 1);
}

void OperatorHmiWindow::closeEvent(QCloseEvent * event)
{
  // A running simulation belongs to Workcell Builder: closing this window neither cancels nor stops it.
  if (sim_status_.value("session").toObject().value("active").toBool())
    std::fprintf(stderr, "Operator HMI closed; the running simulation continues under Workcell Builder.\n");
  stop_checks();
  event->accept();
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

void OperatorHmiWindow::build_ui()
{
  auto * root = new QWidget(this);
  root->setObjectName("hmiRoot");
  auto * layout = new QVBoxLayout(root);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(build_header());

  update_banner_ = new QFrame(root);
  update_banner_->setObjectName("hmiBanner");
  auto * banner = new QHBoxLayout(update_banner_);
  banner->setContentsMargins(18, 6, 18, 6);
  update_banner_text_ = new QLabel(update_banner_);
  update_banner_text_->setWordWrap(true);
  auto * banner_refresh = new QPushButton("Refresh preview", update_banner_);
  banner_refresh->setObjectName("hmiPrimary");
  connect(banner_refresh, &QPushButton::clicked, this, [this]() {
      QString error;
      if (const auto request = read_operator_preview_request(options_.request_path, &error)) apply_request(*request);
    });
  banner->addWidget(update_banner_text_, 1);
  banner->addWidget(banner_refresh);
  update_banner_->setVisible(false);
  layout->addWidget(update_banner_);

  draft_notice_ = new QLabel(root);
  draft_notice_->setObjectName("hmiDraftNotice");
  draft_notice_->setWordWrap(true);
  draft_notice_->setVisible(false);
  layout->addWidget(draft_notice_);

  auto * vertical = new QSplitter(Qt::Vertical, root);
  vertical->setChildrenCollapsible(false);
  auto * body = new QSplitter(Qt::Horizontal, vertical);
  body->setChildrenCollapsible(false);
  body->addWidget(build_overview());
  body->addWidget(build_monitoring());
  body->addWidget(build_controls());
  body->setStretchFactor(0, 0);
  body->setStretchFactor(1, 1);
  body->setStretchFactor(2, 0);
  body->setSizes({360, 640, 340});
  vertical->addWidget(body);
  vertical->addWidget(build_events());
  vertical->setStretchFactor(0, 1);
  vertical->setStretchFactor(1, 0);
  vertical->setSizes({540, 150});
  layout->addWidget(vertical, 1);
  setCentralWidget(root);
}

QFrame * OperatorHmiWindow::build_header()
{
  auto * header = new QFrame(this);
  header->setObjectName("hmiHeader");
  auto * layout = new QHBoxLayout(header);
  layout->setContentsMargins(18, 10, 18, 10);
  layout->setSpacing(10);
  auto * titles = new QVBoxLayout();
  titles->setSpacing(1);
  auto * eyebrow = new QLabel("WORKCELL OPERATOR HMI · PREVIEW", header);
  eyebrow->setObjectName("hmiEyebrow");
  title_ = new QLabel("No cell", header);
  title_->setObjectName("hmiTitle");
  subtitle_ = new QLabel("Read-only view of the engineering configuration from Workcell Builder.", header);
  subtitle_->setObjectName("hmiSubtitle");
  subtitle_->setWordWrap(true);
  titles->addWidget(eyebrow);
  titles->addWidget(title_);
  titles->addWidget(subtitle_);
  layout->addLayout(titles, 1);
  for (QLabel ** badge : {&revision_badge_, &approval_badge_, &mode_badge_, &readiness_badge_, &production_badge_}) {
    *badge = new QLabel(header);
    (*badge)->setObjectName("hmiBadge");
    (*badge)->setAlignment(Qt::AlignCenter);
    (*badge)->setTextFormat(Qt::RichText);
    (*badge)->setMinimumWidth(112);
    layout->addWidget(*badge, 0, Qt::AlignVCenter);
  }
  refresh_button_ = new QPushButton("Refresh", header);
  refresh_button_->setObjectName("hmiSecondary");
  refresh_button_->setToolTip("Reload the engineering configuration (keeps the replay view if shown).");
  connect(refresh_button_, &QPushButton::clicked, this, [this]() {
      QString error;
      if (!options_.request_path.isEmpty()) {
        if (const auto request = read_operator_preview_request(options_.request_path, &error)) {
          apply_request(*request);
          return;
        }
      }
      refresh_preview(replay_shown_);
    });
  layout->addWidget(refresh_button_, 0, Qt::AlignVCenter);
  return header;
}

QWidget * OperatorHmiWindow::build_overview()
{
  auto * scroll = new QScrollArea(this);
  scroll->setObjectName("hmiColumn");
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setMinimumWidth(320);
  auto * host = new QWidget(scroll);
  host->setObjectName("hmiColumnHost");
  auto * layout = new QVBoxLayout(host);
  layout->setContentsMargins(12, 12, 6, 12);
  layout->setSpacing(10);
  QVBoxLayout * body = nullptr;
  layout->addWidget(card(host, "Cell overview", &body));
  cell_values_ = value_label(host);
  body->addWidget(cell_values_);
  layout->addWidget(card(host, "Task", &body));
  task_values_ = value_label(host);
  body->addWidget(task_values_);
  layout->addWidget(card(host, "Validation & readiness", &body));
  validation_values_ = value_label(host);
  body->addWidget(validation_values_);
  layout->addStretch(1);
  scroll->setWidget(host);
  return scroll;
}

QWidget * OperatorHmiWindow::build_monitoring()
{
  auto * host = new QWidget(this);
  host->setObjectName("hmiColumnHost");
  auto * layout = new QVBoxLayout(host);
  layout->setContentsMargins(6, 12, 6, 12);
  layout->setSpacing(10);
  QVBoxLayout * body = nullptr;
  auto * map_card = card(host, "Cell monitor", &body);
  map_caption_ = new QLabel(map_card);
  map_caption_->setObjectName("hmiCaption");
  map_caption_->setWordWrap(true);
  body->addWidget(map_caption_);
  auto * split = new QHBoxLayout();
  split->setSpacing(10);
  map_ = new CellMapWidget(map_card);
  split->addWidget(map_, 3);
  auto * status_stack = new QVBoxLayout();
  status_stack->setSpacing(6);
  robot_status_ = value_label(map_card);
  robot_status_->setObjectName("hmiStatusTile");
  robot_status_->setMinimumHeight(76);
  cycle_status_ = value_label(map_card);
  cycle_status_->setObjectName("hmiStatusTile");
  cycle_status_->setMinimumHeight(76);
  status_stack->addWidget(robot_status_);
  status_stack->addWidget(cycle_status_);
  status_stack->addStretch(1);
  split->addLayout(status_stack, 2);
  body->addLayout(split, 1);
  layout->addWidget(map_card, 3);

  auto * objects_card = card(host, "Detected objects", &body);
  objects_ = new QTableWidget(0, 4, objects_card);
  objects_->setObjectName("hmiTable");
  objects_->setHorizontalHeaderLabels({"Object", "Class", "Confidence", "Decision"});
  objects_->verticalHeader()->setVisible(false);
  objects_->setSelectionBehavior(QAbstractItemView::SelectRows);
  objects_->setSelectionMode(QAbstractItemView::SingleSelection);
  objects_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  objects_->horizontalHeader()->setStretchLastSection(true);
  objects_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  objects_->verticalHeader()->setDefaultSectionSize(26);
  objects_->setMinimumHeight(96);
  connect(objects_, &QTableWidget::itemSelectionChanged, this, [this]() { render_selected_object(); });
  body->addWidget(objects_, 1);
  object_detail_ = value_label(objects_card);
  object_detail_->setObjectName("hmiCaption");
  body->addWidget(object_detail_);
  layout->addWidget(objects_card, 2);
  return host;
}

QWidget * OperatorHmiWindow::build_controls()
{
  auto * scroll = new QScrollArea(this);
  scroll->setObjectName("hmiColumn");
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setMinimumWidth(300);
  auto * host = new QWidget(scroll);
  host->setObjectName("hmiColumnHost");
  auto * layout = new QVBoxLayout(host);
  layout->setContentsMargins(6, 12, 12, 12);
  layout->setSpacing(10);
  QVBoxLayout * body = nullptr;
  layout->addWidget(card(host, "Task workflow", &body));
  auto step = [&](const QString & text) {
      auto * label = new QLabel(text, host);
      label->setObjectName("hmiStep");
      body->addWidget(label);
    };
  step("1  Recipe");
  recipe_summary_ = value_label(host);
  body->addWidget(recipe_summary_);
  step("2  Check readiness");
  check_button_ = new QPushButton("Check readiness", host);
  check_button_->setObjectName("hmiSecondary");
  check_button_->setToolTip("Re-read the engineering configuration and its validation evidence (read-only).");
  connect(check_button_, &QPushButton::clicked, this, [this]() {
      log_event("info", "READINESS_CHECK", "Readiness check requested.");
      refresh_preview(replay_shown_);
    });
  body->addWidget(check_button_);
  step("3  Run permitted offline task");
  replay_button_ = new QPushButton("Run replay check", host);
  replay_button_->setObjectName("hmiPrimary");
  replay_button_->setToolTip("Evaluate the task's recorded commissioning replay with the task's selection rules. "
                             "No robot motion.");
  connect(replay_button_, &QPushButton::clicked, this, [this]() {
      log_event("info", "OFFLINE_TEST_STARTED", "Replay check started (recorded input, no robot motion).");
      refresh_preview(true);
    });
  body->addWidget(replay_button_);
  replay_note_ = value_label(host);
  replay_note_->setObjectName("hmiCaption");
  body->addWidget(replay_note_);
  dry_run_button_ = new QPushButton("Run generated-cell dry-run", host);
  dry_run_button_->setObjectName("hmiSecondary");
  dry_run_button_->setToolTip("Existing gated dry-run of the generated cell package. No robot motion.");
  connect(dry_run_button_, &QPushButton::clicked, this, [this]() { run_gated_dry_run(); });
  body->addWidget(dry_run_button_);
  dry_run_note_ = value_label(host);
  dry_run_note_->setObjectName("hmiCaption");
  body->addWidget(dry_run_note_);
  step("4  Monitor");
  monitor_status_ = value_label(host);
  body->addWidget(monitor_status_);
  step("5  Stop");
  stop_button_ = new QPushButton("Stop check", host);
  stop_button_->setObjectName("hmiSecondary");
  stop_button_->setToolTip("Stops the running offline check process. This is not a safety function.");
  connect(stop_button_, &QPushButton::clicked, this, [this]() {
      const bool running = (projection_process_ && projection_process_->state() != QProcess::NotRunning) ||
        (dry_run_process_ && dry_run_process_->state() != QProcess::NotRunning);
      stop_checks();
      set_busy({});
      if (running) log_event("notice", "OFFLINE_TEST_STOPPED", "Offline check stopped by the operator.");
    });
  body->addWidget(stop_button_);
  step("6  Review results");
  results_summary_ = value_label(host);
  body->addWidget(results_summary_);

  layout->addWidget(card(host, "Simulation · qualified Gazebo", &body));
  sim_state_ = value_label(host);
  body->addWidget(sim_state_);
  sim_run_button_ = new QPushButton("Run Simulation Cycle", host);
  sim_run_button_->setObjectName("hmiPrimary");
  sim_run_button_->setToolTip("Asks Workcell Builder to run one bounded pick-and-place cycle of the saved cell in the "
                              "qualified physics simulation. Simulated robot only; the real robot stays locked.");
  connect(sim_run_button_, &QPushButton::clicked, this, [this]() { request_simulation("run_simulation"); });
  body->addWidget(sim_run_button_);
  sim_cancel_button_ = new QPushButton("Cancel Simulation", host);
  sim_cancel_button_->setObjectName("hmiSecondary");
  sim_cancel_button_->setToolTip("Asks Workcell Builder to cancel the running simulation with its supervised cleanup. "
                                 "This is not an emergency stop.");
  connect(sim_cancel_button_, &QPushButton::clicked, this, [this]() { request_simulation("cancel_simulation"); });
  body->addWidget(sim_cancel_button_);
  sim_note_ = value_label(host);
  sim_note_->setObjectName("hmiCaption");
  body->addWidget(sim_note_);
  sim_result_ = value_label(host);
  body->addWidget(sim_result_);

  layout->addWidget(card(host, "Sorting session", &body));
  sort_state_ = value_label(host);
  body->addWidget(sort_state_);
  {
    auto * row = new QHBoxLayout;
    sort_start_button_ = new QPushButton("Start Sorting", host);
    sort_start_button_->setObjectName("hmiPrimary");
    sort_start_button_->setToolTip("Asks Workcell Builder to sort every eligible object of the saved routing table, one cycle "
                                   "each, on MoveIt fake hardware. Simulated robot only; the real robot stays locked.");
    sort_pause_button_ = new QPushButton("Pause", host);
    sort_resume_button_ = new QPushButton("Resume", host);
    sort_stop_button_ = new QPushButton("Stop", host);
    sort_stop_button_->setToolTip("Software stop at the end of the current cycle. This is not an emergency stop.");
    for (auto * button : {sort_pause_button_, sort_resume_button_, sort_stop_button_}) button->setObjectName("hmiSecondary");
    sort_motion_button_ = new QPushButton("Start Sorting · Gazebo Motion (2 cycles)", host);
    sort_motion_button_->setObjectName("hmiPrimary");
    sort_motion_button_->setToolTip("A bounded sorting session in which the simulated robot moves in Gazebo (at most 2 cycles). "
                                    "Admitted by Workcell Builder only for a G4-qualified cell; the real robot is not available.");
    connect(sort_start_button_, &QPushButton::clicked, this, [this]() { request_simulation("start_sorting"); });
    connect(sort_motion_button_, &QPushButton::clicked, this, [this]() { request_simulation("start_sorting", true); });
    connect(sort_pause_button_, &QPushButton::clicked, this, [this]() { request_simulation("pause_sorting"); });
    connect(sort_resume_button_, &QPushButton::clicked, this, [this]() { request_simulation("resume_sorting"); });
    connect(sort_stop_button_, &QPushButton::clicked, this, [this]() { request_simulation("stop_sorting"); });
    body->addWidget(sort_start_button_);
    body->addWidget(sort_motion_button_);
    for (auto * button : {sort_pause_button_, sort_resume_button_, sort_stop_button_}) row->addWidget(button);
    body->addLayout(row);
  }
  sort_note_ = value_label(host);
  sort_note_->setObjectName("hmiCaption");
  body->addWidget(sort_note_);
  sort_result_ = value_label(host);
  body->addWidget(sort_result_);

  layout->addWidget(card(host, "Production", &body));
  start_button_ = new QPushButton("Start production cycle", host);
  start_button_->setObjectName("hmiDanger");
  start_button_->setEnabled(false);
  start_button_->setToolTip("Not authorised in this release: no qualified execution backend.");
  body->addWidget(start_button_);
  production_blockers_ = value_label(host);
  body->addWidget(production_blockers_);
  auto * capabilities_title = new QLabel("Capabilities of this cell", host);
  capabilities_title->setObjectName("hmiStep");
  body->addWidget(capabilities_title);
  capabilities_ = value_label(host);
  body->addWidget(capabilities_);
  auto * safety = new QLabel("This HMI has no emergency stop. Use the cell's hardware emergency stop. "
                             "Stop check only ends an offline check.", host);
  safety->setObjectName("hmiCaption");
  safety->setWordWrap(true);
  body->addWidget(safety);
  layout->addStretch(1);
  scroll->setWidget(host);
  return scroll;
}

QWidget * OperatorHmiWindow::build_events()
{
  auto * tabs = new QTabWidget(this);
  tabs->setObjectName("hmiTabs");
  tabs->setMinimumHeight(130);
  events_ = new QTableWidget(0, 4, tabs);
  events_->setObjectName("hmiTable");
  events_->setHorizontalHeaderLabels({"Time", "Level", "Event", "Details"});
  events_->verticalHeader()->setVisible(false);
  events_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  events_->setSelectionBehavior(QAbstractItemView::SelectRows);
  events_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  events_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  events_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  events_->horizontalHeader()->setStretchLastSection(true);
  events_->verticalHeader()->setDefaultSectionSize(24);
  tabs->addTab(events_, "Events");
  details_ = new QPlainTextEdit(tabs);
  details_->setObjectName("hmiDetails");
  details_->setReadOnly(true);
  tabs->addTab(details_, "Engineering details");
  return tabs;
}

void OperatorHmiWindow::apply_style()
{
  setStyleSheet(studioQss(R"(
    QWidget#hmiRoot, QWidget#hmiColumnHost { background: @bg; }
    QScrollArea#hmiColumn { background: @bg; border: none; }
    QFrame#hmiHeader { background: @surfaceSunken; border-bottom: 1px solid @border; }
    QLabel#hmiEyebrow { color: @textMuted; font-size: 8pt; font-weight: 700; letter-spacing: 1px; }
    QLabel#hmiTitle { color: @text; font-size: 17pt; font-weight: 700; }
    QLabel#hmiSubtitle { color: @textSecondary; font-size: 9pt; }
    QLabel#hmiBadge { background: @surface; border: 1px solid @border; border-radius: @radiusM;
      padding: 4px 10px; color: @text; font-size: 10pt; }
    QLabel#hmiBadge[hmiState="ok"] { border-color: @success; background: @successSoft; }
    QLabel#hmiBadge[hmiState="warn"] { border-color: @warning; background: @warningSoft; }
    QLabel#hmiBadge[hmiState="error"] { border-color: @danger; background: @dangerSoft; }
    QLabel#hmiBadge[hmiState="info"] { border-color: @info; background: @infoSoft; }
    QFrame#hmiBanner { background: @infoSoft; border-bottom: 1px solid @info; }
    QFrame#hmiBanner QLabel { color: @text; font-size: 10pt; }
    QLabel#hmiDraftNotice { background: @warningSoft; color: @text; border-bottom: 1px solid @warning;
      padding: 6px 18px; font-size: 10pt; }
    QFrame#hmiCard { background: @surface; border: 1px solid @border; border-radius: @radiusL; }
    QLabel#hmiCardTitle { color: @textMuted; font-size: 8.5pt; font-weight: 700; letter-spacing: 1px; }
    QLabel { color: @text; font-size: 10pt; }
    QLabel#hmiCaption { color: @textSecondary; font-size: 9pt; }
    QLabel#hmiStep { color: @accentHover; font-size: 9pt; font-weight: 700; margin-top: 2px; }
    QLabel#hmiStatusTile { background: @surfaceSunken; border: 1px solid @border; border-radius: @radiusM;
      padding: 8px; }
    QPushButton { min-height: 30px; border-radius: @radiusM; padding: 3px 12px; font-size: 10pt; }
    QPushButton#hmiPrimary { background: @accent; color: @accentText; border: 1px solid @accent; font-weight: 600; }
    QPushButton#hmiPrimary:hover { background: @accentHover; }
    QPushButton#hmiSecondary { background: @surfaceRaised; color: @text; border: 1px solid @borderStrong; }
    QPushButton#hmiSecondary:hover { border-color: @accent; }
    QPushButton#hmiDanger { background: @dangerSoft; color: @danger; border: 1px solid @danger; font-weight: 600; }
    QPushButton:disabled, QPushButton#hmiPrimary:disabled, QPushButton#hmiSecondary:disabled,
    QPushButton#hmiDanger:disabled { background: @surfaceSunken; color: @textMuted; border: 1px solid @border;
      font-weight: 400; }
    QTableWidget#hmiTable { background: @surfaceSunken; alternate-background-color: @surface; color: @text;
      gridline-color: @border; border: 1px solid @border; border-radius: @radiusS; font-size: 9.5pt;
      selection-background-color: @selection; }
    QHeaderView::section { background: @surfaceRaised; color: @textSecondary; border: none;
      border-right: 1px solid @border; padding: 4px 6px; font-size: 9pt; font-weight: 600; }
    QTabWidget#hmiTabs::pane { border-top: 1px solid @border; background: @bg; }
    QTabBar::tab { background: @surface; color: @textSecondary; padding: 6px 14px; border: 1px solid @border;
      border-bottom: none; border-top-left-radius: @radiusS; border-top-right-radius: @radiusS; }
    QTabBar::tab:selected { background: @surfaceRaised; color: @text; }
    QPlainTextEdit#hmiDetails { background: @surfaceSunken; color: @textSecondary; border: none;
      font-family: monospace; font-size: 9pt; }
    QSplitter::handle { background: @bg; }
  )"));
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

void OperatorHmiWindow::render()
{
  render_badges();
  render_overview();
  render_monitoring();
  render_controls();
  render_simulation();
  write_status();
}

void OperatorHmiWindow::render_badges()
{
  const bool have = view_loaded_ && view_.usable;
  title_->setText(have ? view_.scene_name : (request_ ? request_->scene_name : QFileInfo(scene_dir_).fileName()));
  setWindowTitle(QString("Workcell Operator HMI — %1").arg(title_->text().isEmpty() ? "no cell" : title_->text()));
  if (!have) {
    set_badge(revision_badge_, "REVISION", "UNKNOWN", "neutral");
    set_badge(approval_badge_, "APPROVAL", "UNKNOWN", "neutral");
    set_badge(mode_badge_, "MODE", "UNAVAILABLE", "neutral");
    set_badge(readiness_badge_, "READINESS", view_loaded_ ? "BLOCKED" : "LOADING", view_loaded_ ? "error" : "neutral");
    set_badge(production_badge_, "PRODUCTION", "NOT AUTHORISED", "error");
    draft_notice_->setVisible(false);
    return;
  }
  set_badge(revision_badge_, "REVISION", view_.draft ? "DRAFT" : "SAVED", view_.draft ? "warn" : "ok");
  set_badge(approval_badge_, "APPROVAL", view_.approval_status == "APPROVED" ? "APPROVED" : "NOT APPROVED",
            view_.approval_status == "APPROVED" ? "ok" : "warn");
  const auto mode = operator_mode(view_, replay_shown_);
  set_badge(mode_badge_, "MODE", operator_mode_label(mode),
            mode == OperatorMode::Draft ? "warn" : mode == OperatorMode::Replay ? "info" : "neutral");
  const auto readiness = operator_readiness(view_);
  set_badge(readiness_badge_, "OFFLINE CHECKS", readiness.offline_checks_allowed ? "READY" : "BLOCKED",
            readiness.offline_checks_allowed ? "ok" : "error");
  set_badge(production_badge_, "PRODUCTION", "NOT AUTHORISED", "error");
  QStringList notices;
  if (view_.draft) {
    notices << QString("DRAFT — showing unsaved task edits from Workcell Builder (not saved, not approved)%1.")
      .arg(view_.draft_differs == std::optional<bool>(false) ? "; identical to the saved task" : "");
  }
  if (view_.layout_unsaved) {
    notices << "Workcell Builder has unsaved layout edits; the physical scene shown is the last saved revision.";
  }
  draft_notice_->setText(notices.join("  "));
  draft_notice_->setVisible(!notices.isEmpty());
}

void OperatorHmiWindow::render_overview()
{
  if (!view_loaded_ || !view_.usable) {
    const QString reason = view_.errors.isEmpty() ? QString("Loading the engineering configuration…")
                                                  : view_.errors.join("<br>");
    cell_values_->setText(view_.errors.isEmpty() ? muted(reason) : state_text("BLOCKED", "error") + "<br>" + reason);
    task_values_->setText(muted("UNAVAILABLE"));
    validation_values_->setText(muted("UNAVAILABLE"));
    return;
  }
  QString tool = or_unavailable(view_.tool);
  if (!view_.tool_type.isEmpty()) tool += muted(" · " + view_.tool_type);
  cell_values_->setText(rows({
      {"Cell", escaped(view_.scene_name)},
      {"Robot", or_unavailable(view_.robot) + (view_.planning_group.isEmpty() ? QString() : muted(" · " + view_.planning_group))},
      {"End effector", tool},
      {"Tool capability", view_.capabilities.isEmpty() ? muted("UNKNOWN") : escaped(view_.capabilities.join(", "))},
      {"Camera", or_unavailable(view_.camera) + (view_.camera_source.isEmpty() ? QString() : muted(" · " + view_.camera_source))},
      {"Perception", or_unavailable(view_.perception_contract) +
        (view_.perception_mode.isEmpty() ? QString() : muted(" · " + view_.perception_mode))},
    }));
  QString target = !view_.target_class.isEmpty() ? state_text(view_.target_class, "info")
    : !view_.routing_rules.isEmpty() ? escaped("ROUTED CLASSES (see Routing)") : muted("ANY CLASS (no class filter)");
  if (!view_.target_color.isEmpty()) target += escaped(" · colour " + view_.target_color);
  task_values_->setText(rows({
      {"Task", or_unavailable(view_.task_id) + muted(" · " + view_.task_type)},
      {"Target objects", target},
      {"Ignored objects", escaped(view_.not_targeted_rule.isEmpty() ? QString("UNKNOWN") : view_.not_targeted_rule)},
      {"Min confidence", number_or_unavailable(view_.min_confidence, "", "not set")},
      {"Max age", number_or_unavailable(view_.max_age_seconds, " s", "not set")},
      {"Pick zone", or_unavailable(view_.pick_zone)},
      {"Input source", or_unavailable(view_.source)},
      {"Destinations", view_.destinations.isEmpty() ? state_text("MISSING", "error") : escaped(view_.destinations.join(", "))},
      {"Routing", view_.routing_rules.isEmpty() ? muted("none") : escaped(view_.routing_rules.join("; "))},
      {"Grasp", or_unavailable(view_.grasp)},
    }));
  const auto check = [](const QString & status) {
      if (status.isEmpty()) return muted("NOT RUN");
      const QString state = (status == "PASS" || status == "READY") ? "ok"
        : (status == "WARNING" || status == "WARNINGS" || status == "PREVIEW_ONLY") ? "warn" : "error";
      return state_text(status, state);
    };
  QString offline = check(view_.offline_validation_status);
  if (!view_.offline_validation_status.isEmpty()) {
    offline += muted(" · " + when(view_.offline_validation_modified));
    if (view_.offline_validation_current) {
      offline += *view_.offline_validation_current ? muted(" · current") : " " + state_text("STALE", "warn");
    }
  }
  QString scene_validation = check(view_.scene_validation_status);
  if (!view_.scene_validation_status.isEmpty()) scene_validation += muted(" · " + when(view_.scene_validation_modified));
  QString task_validation = check(view_.validation_status) + muted(" · " + view_.readiness_classification);
  if (!view_.validation_errors.isEmpty()) task_validation += "<br>" + escaped(view_.validation_errors.join("; "));
  QString resolution = check(view_.resolution_status);
  if (!view_.resolution_code.isEmpty()) resolution += muted(" · " + view_.resolution_code);
  if (!view_.resolution_reason.isEmpty()) resolution += "<br>" + muted(view_.resolution_reason);
  QString input = view_.input_kind == "replay" ? state_text("RECORDED REPLAY", "info")
    : view_.input_kind == "live" ? state_text("LIVE (not consumed in preview)", "warn")
    : view_.input_kind == "session" ? state_text("PER CYCLE", "info") +
        muted(" · the sorting session renders the cell with its camera and classifies each frame with EPD (synthetic)")
    : state_text("UNAVAILABLE", "error");
  if (!view_.input_blocker.isEmpty()) input += "<br>" + escaped(view_.input_blocker);
  validation_values_->setText(rows({
      {"Task intent", task_validation},
      {"Scene validation", scene_validation},
      {"Offline validation", offline},
      {"Task resolution", resolution},
      {"Task input", input},
      {"Simulation", simulation_linked() ? escaped("Qualified Gazebo simulation, run by Workcell Builder")
                                         : muted("Not available: standalone HMI (no Workcell Builder control session)")},
      {"Live operation", state_text("NOT SUPPORTED", "warn")},
    }));
}

QJsonObject OperatorHmiWindow::session_g4() const
{
  const auto canonical = [](const QString & path) { const auto c = QFileInfo(path).canonicalFilePath(); return c.isEmpty() ? path : c; };
  const auto session = sim_status_.value("session").toObject();
  if (session.isEmpty() || !(session.value("plan_only").toBool() || session.value("motion_trial").toBool()) || scene_dir_.isEmpty() ||
      canonical(session.value("scene_dir").toString()) != canonical(scene_dir_)) return {};
  return session.value("g4").toObject();
}

void OperatorHmiWindow::render_monitoring()
{
  const auto g4 = session_g4();
  session_objects_ = !g4.value("objects").toArray().isEmpty() && !(replay_shown_ && view_.replay_evaluated);
  if (session_objects_) {
    // Identity and pose are Gazebo's measurement, the class is EPD's on the session's rendered frame, the decision is
    // the sorting ledger's and the plan is the executor's plan-only resolve. Nothing here is a count of sorted objects.
    const bool trial = sim_status_.value("session").toObject().value("motion_trial").toBool();
    session_view_ = view_;
    session_view_.objects.clear();
    objects_->setRowCount(0);
    for (const auto & item : g4.value("objects").toArray()) {
      const auto o = item.toObject();
      const auto plan = o.value("plan").toObject();
      OperatorObjectRow row;
      row.object_id = o.value("object_id").toString();
      row.class_id = o.value("class_id").toString();
      if (o.value("confidence").isDouble()) row.confidence = o.value("confidence").toDouble();
      // Marked on the map: objects the ledger may still select, has planned or has counted. COMPLETED is the ledger's
      // own count (after the runner verified and reconciled the cycle), never a plan or a motion alone.
      const bool completed = o.value("status").toString() == "COMPLETED";
      row.accepted = completed || plan.value("state").toString() == "PLANNED" || (plan.isEmpty() && o.value("status").toString() == "PENDING");
      row.reason_code = completed ? QString("COMPLETED") : plan.isEmpty() ? o.value("status").toString() : "PLAN_" + plan.value("state").toString();
      row.reason = completed
        ? QString("Sorted in cycle %1 → %2: verified and reconciled by the runner, counted by the sorting ledger.")
            .arg(plan.value("cycle").toInt()).arg(o.value("destination").toString())
        : plan.isEmpty() ? o.value("reason").toString(o.value("status").toString())
        : QString("Planned in cycle %1 → %2 (%3, grasp %4)%5").arg(plan.value("cycle").toInt())
            .arg(plan.value("destination").toString(), plan.value("bin").toString(), plan.value("grasp").toString(),
                 trial ? QString(".") : QString("; not executed."));
      const auto pose = o.value("pose").toArray(), size = o.value("dimensions").toArray();
      if (pose.size() >= 3 && size.size() >= 3) {
        for (int i = 0; i < 3; ++i) { row.xyz[i] = pose.at(i).toDouble(); row.dimensions[i] = size.at(i).toDouble(); }
        row.has_geometry = true;
      }
      session_view_.objects.push_back(row);
      const int r = objects_->rowCount();
      objects_->insertRow(r);
      const QStringList cells{row.object_id, row.class_id,
        row.confidence ? QString::number(*row.confidence, 'f', 2) : QString("—"),
        completed ? QString("COMPLETED → %1 · verified").arg(o.value("destination").toString())
        : plan.isEmpty() ? row.reason_code
        : QString("PLANNED → %1%2").arg(plan.value("destination").toString(), trial ? QString() : QString(" · not executed"))};
      for (int c = 0; c < cells.size(); ++c) {
        auto * cell = new QTableWidgetItem(cells.at(c));
        cell->setToolTip(row.reason);
        if (c == 3) cell->setForeground(QColor(completed ? studioTokens().success : row.accepted ? studioTokens().info : studioTokens().textMuted));
        objects_->setItem(r, c, cell);
      }
    }
    map_->set_view(&session_view_, true, QString());
    const auto cycles = g4.value("cycles").toArray();
    QJsonObject frame; int frame_cycle = 0;                // the latest cycle that has its rendered frame
    for (const auto & item : cycles)
      if (!item.toObject().value("frame").toObject().isEmpty()) {
        frame = item.toObject().value("frame").toObject(); frame_cycle = item.toObject().value("cycle").toInt();
      }
    map_caption_->setText(frame.isEmpty() ? QString("<b style='color:%1'>SESSION</b> · positions measured in Gazebo; "
                                                    "waiting for the first rendered frame").arg(studioTokens().info)
      : QString("<b style='color:%1'>SESSION</b> · cycle %2 frame rendered by the cell camera (%3) and classified by EPD · "
                "synthetic · positions measured in Gazebo")
          .arg(studioTokens().info).arg(frame_cycle).arg(escaped(frame.value("camera_id").toString())));
    const auto motion = g4.value("motion").toObject();
    object_detail_->setText(muted(QString(trial ? "COMPLETED is counted only after verification and reconciliation. Evidence of "
                                                  "motion: cycle authorisations granted %1, executed cycles %2, controller goals %3. "
                                                  "Select an object for its decision."
                                                : "Plans are not executions: cycle authorisations granted %1, executed cycles %2, "
                                                  "controller goals %3. Select an object for its decision.")
      .arg(motion.value("authorised_cycles").toInt()).arg(motion.value("executed_cycles").toInt())
      .arg(motion.value("controller_goals").toInt())));
    return;
  }
  const bool objects = replay_shown_ && view_.replay_evaluated;
  map_->set_view(view_loaded_ && view_.usable ? &view_ : nullptr, objects, view_.selected_object);
  if (objects) {
    map_caption_->setText(QString("<b style='color:%1'>REPLAY</b> · recorded commissioning snapshot, not a live camera"
                                  "%2").arg(studioTokens().info,
                                            view_.replay_description.isEmpty() ? QString() : " · " + escaped(view_.replay_description)));
  } else {
    map_caption_->setText(view_.input_kind == "replay"
      ? "Camera: no live feed in the preview. Run the replay check to show the recorded objects."
      : "Camera: no live feed in the preview. No observation data.");
  }
  objects_->setRowCount(0);
  if (objects) {
    for (const auto & row : view_.objects) {
      const int r = objects_->rowCount();
      objects_->insertRow(r);
      const QStringList cells{row.object_id, row.class_id,
        row.confidence ? QString::number(*row.confidence, 'f', 2) : QString("UNAVAILABLE"),
        row.accepted ? QString("ACCEPTED") : QString("REJECTED · ") + row.reason_code};
      for (int c = 0; c < cells.size(); ++c) {
        auto * item = new QTableWidgetItem(cells.at(c));
        item->setToolTip(row.reason);
        if (c == 3) item->setForeground(QColor(row.accepted ? studioTokens().success : studioTokens().warning));
        objects_->setItem(r, c, item);
      }
    }
    if (!view_.replay_error.isEmpty()) object_detail_->setText(state_text("Replay check failed: ", "error") + escaped(view_.replay_error));
  }
  if (!objects) object_detail_->setText(muted("No detected-object data. Detections are UNAVAILABLE until a replay is evaluated."));
  else if (view_.objects.isEmpty() && view_.replay_error.isEmpty()) object_detail_->setText(muted("The replay contained no objects."));
  else if (view_.replay_error.isEmpty()) object_detail_->setText(muted("Select an object to see why it was accepted or rejected."));

  robot_status_->setText(QString("<b>Robot</b><br>%1<br>%2").arg(state_text("NOT CONNECTED", "neutral"),
    muted("State UNKNOWN (no runtime link).")));
  QString cycle = QString("<b>Cycle stage</b><br>%1").arg(state_text("NO CYCLE RUNNING", "neutral"));
  if (dry_run_.parsed) {
    cycle += QString("<br>Last dry-run: %1").arg(state_text(dry_run_.status.isEmpty() ? "UNKNOWN" : dry_run_.status,
      dry_run_.status == "PASS" ? "ok" : "error"));
    if (!dry_run_.stages.isEmpty()) cycle += "<br>" + muted("Planned stages: " + dry_run_.stages.join(" → "));
  } else if (objects && !view_.selected_object.isEmpty()) {
    cycle += "<br>" + muted("Selected " + view_.selected_object + "; not executed.");
  }
  cycle_status_->setText(cycle);
}

void OperatorHmiWindow::render_selected_object()
{
  const auto selected = objects_->selectionModel() ? objects_->selectionModel()->selectedRows() : QModelIndexList{};
  if (session_objects_) {
    if (selected.isEmpty() || selected.first().row() >= session_view_.objects.size()) return;
    const auto & row = session_view_.objects.at(selected.first().row());
    map_->set_view(&session_view_, true, row.object_id);
    object_detail_->setText(QString("%1 · <b>%2</b> · %3<br>%4 %5").arg(escaped(row.object_id), escaped(row.class_id),
      row.confidence ? escaped(QString("EPD confidence %1").arg(*row.confidence, 0, 'f', 2)) : muted("not classified by EPD"),
      row.accepted ? state_text("PLANNED", "info") : state_text(row.reason_code, "neutral"), escaped(row.reason)));
    return;
  }
  if (selected.isEmpty() || selected.first().row() >= view_.objects.size()) {
    map_->set_view(view_loaded_ && view_.usable ? &view_ : nullptr, replay_shown_ && view_.replay_evaluated, view_.selected_object);
    return;
  }
  const auto & row = view_.objects.at(selected.first().row());
  map_->set_view(&view_, true, row.object_id);
  object_detail_->setText(QString("%1 · <b>%2</b> · %3<br>%4 %5")
    .arg(escaped(row.object_id), escaped(row.class_id),
         row.confidence ? escaped(QString("confidence %1").arg(*row.confidence, 0, 'f', 2)) : muted("confidence UNAVAILABLE"),
         row.accepted ? state_text("ACCEPTED", "ok") : state_text("REJECTED", "warn"),
         escaped(row.reason) + muted(" (" + row.reason_code + ")")));
}

void OperatorHmiWindow::render_controls()
{
  const bool have = view_loaded_ && view_.usable;
  const auto readiness = operator_readiness(view_);
  const bool running = !busy_.isEmpty();
  if (have) {
    recipe_summary_->setText(QString("%1 · target %2<br>%3")
      .arg(escaped(view_.task_id), !view_.target_class.isEmpty() ? state_text(view_.target_class, "info")
           : !view_.routing_rules.isEmpty() ? escaped("routed classes") : muted("any class"),
           view_.draft ? state_text("DRAFT — not an approved recipe", "warn")
           : view_.approval_status == "APPROVED" ? state_text("Approved by " + view_.approval_by, "ok")
           : state_text("Saved, not approved", "warn")));
  } else {
    recipe_summary_->setText(muted("UNAVAILABLE"));
  }
  check_button_->setEnabled(!scene_dir_.isEmpty() && !running);
  replay_button_->setEnabled(have && readiness.offline_checks_allowed && view_.input_kind == "replay" && !running);
  replay_button_->setToolTip(view_.input_kind == "replay" ? "Evaluate the recorded commissioning replay. No robot motion."
                                                         : "No recorded replay is bound to this task.");
  QString replay_note;
  if (have && view_.input_kind != "replay") {
    QString blocker = view_.input_blocker.isEmpty() ? QString("no recorded replay is bound to this task.") : view_.input_blocker;
    if (!scene_dir_.isEmpty()) blocker.replace(QDir(scene_dir_).absolutePath() + "/", QString());  // scene-relative
    replay_note = state_text("Replay check unavailable: ", "warn") + escaped(blocker);
  } else if (have && !readiness.offline_checks_allowed) {
    replay_note = state_text("Blocked: ", "error") + escaped(readiness.offline_blockers.join("; "));
  }
  replay_note_->setText(replay_note);
  replay_note_->setVisible(!replay_note.isEmpty());
  const bool dry_run_ok = have && view_.dry_run_available && !view_.draft && !running &&
    !resolve_script("run_generated_workcell_bundle.py").isEmpty();
  dry_run_button_->setEnabled(dry_run_ok);
  QString dry_note;
  if (!have) dry_note = muted("Dry-run UNAVAILABLE.");
  else if (view_.draft) dry_note = muted("Dry-run uses the saved generated package; save and generate in Workcell Builder first.");
  else if (!view_.dry_run_available) dry_note = muted(view_.dry_run_unavailable_reason);
  else if (dry_run_.parsed && !dry_run_.blockers.isEmpty())
    dry_note = state_text("Blocked: ", "error") + escaped(dry_run_.blockers.join("; "));
  dry_run_note_->setText(dry_note);
  dry_run_note_->setVisible(!dry_note.isEmpty());
  monitor_status_->setText(running ? state_text(busy_, "info") : muted("Idle — no offline check running."));
  stop_button_->setEnabled(running);

  QStringList results;
  if (replay_shown_ && view_.replay_evaluated) {
    int accepted = 0;
    for (const auto & row : view_.objects) accepted += row.accepted ? 1 : 0;
    results << QString("Replay check: %1 of %2 objects accepted%3.").arg(accepted).arg(view_.objects.size())
      .arg(view_.selected_object.isEmpty() ? QString() : "; selected " + view_.selected_object);
  }
  if (dry_run_.parsed) {
    results << QString("Dry-run: %1%2").arg(dry_run_.status.isEmpty() ? QString("UNKNOWN") : dry_run_.status,
      dry_run_.safe_for_robot_motion ? QString() : QString(" · safe for robot motion: false"));
  }
  results_summary_->setText(results.isEmpty() ? muted("No results yet.") : escaped(results.join("\n")).replace("\n", "<br>"));

  start_button_->setEnabled(false);
  QString blockers = state_text("NOT AUTHORISED", "error") + "<br>";
  for (const auto & blocker : readiness.production_blockers) blockers += "• " + escaped(blocker) + "<br>";
  production_blockers_->setText(blockers);
}

void OperatorHmiWindow::write_status() const
{
  if (options_.status_out.isEmpty()) return;
  QJsonArray objects;
  for (const auto & row : view_.objects) {
    objects.append(QJsonObject{{"object_id", row.object_id}, {"class_id", row.class_id},
                               {"accepted", row.accepted}, {"reason_code", row.reason_code}});
  }
  const auto readiness = operator_readiness(view_);
  const QJsonObject status{
    {"schema", "workcell_operator_hmi_status/v1"},
    {"applied_revision", request_ ? request_->revision : -1},
    {"pending_revision", pending_revision_},
    {"update_banner_visible", update_banner_ && update_banner_->isVisible()},
    {"scene_name", view_.scene_name}, {"scene_path", view_.scene_path},
    {"loaded", view_loaded_}, {"usable", view_.usable},
    {"revision", view_.draft ? "DRAFT" : "SAVED"},
    {"layout_unsaved", view_.layout_unsaved},
    {"intent_sha256", view_.intent_sha256}, {"saved_intent_sha256", view_.saved_intent_sha256},
    {"mode", operator_mode_label(operator_mode(view_, replay_shown_))},
    {"approval", view_.approval_status},
    {"task_id", view_.task_id}, {"target_class", view_.target_class},
    {"destinations", QJsonArray::fromStringList(view_.destinations)},
    {"validation_status", view_.validation_status},
    {"resolution_status", view_.resolution_status},
    {"offline_checks_allowed", readiness.offline_checks_allowed},
    {"start_enabled", start_button_ && start_button_->isEnabled()},
    {"production_blockers", QJsonArray::fromStringList(readiness.production_blockers)},
    {"replay_shown", replay_shown_}, {"objects", objects}, {"selected_object", view_.selected_object},
    {"dry_run_status", dry_run_.parsed ? dry_run_.status : QString()},
    {"dry_run_blockers", QJsonArray::fromStringList(dry_run_.blockers)},
    {"busy", busy_}, {"errors", QJsonArray::fromStringList(view_.errors)},
    {"event_count", events_ ? events_->rowCount() : 0},
    {"last_command", last_command_}, {"command_results", command_results_},
    {"replay_enabled", replay_button_ && replay_button_->isEnabled()},
    {"dry_run_enabled", dry_run_button_ && dry_run_button_->isEnabled()},
    {"input_kind", view_.input_kind}, {"input_blocker", view_.input_blocker},
    {"simulation_linked", simulation_linked()},
    {"simulation_run_enabled", sim_run_button_ && sim_run_button_->isEnabled()},
    {"simulation_cancel_enabled", sim_cancel_button_ && sim_cancel_button_->isEnabled()},
    {"simulation_session", sim_status_.value("session")},
    {"session_objects_shown", session_objects_},
    {"capabilities", sim_status_.value("readiness").toObject().value("capabilities")},
    {"sorting_start_enabled", sort_start_button_ && sort_start_button_->isEnabled()},
    {"sorting_motion_enabled", sort_motion_button_ && sort_motion_button_->isEnabled()},
  };
  QSaveFile file(options_.status_out);
  if (file.open(QIODevice::WriteOnly)) {
    file.write(QJsonDocument(status).toJson());
    file.commit();
  }
}

// ---------------------------------------------------------------------------
// Simulation link: Workcell Builder owns, starts, supervises and cancels the run
// ---------------------------------------------------------------------------

bool OperatorHmiWindow::simulation_linked() const
{
  return !options_.request_path.isEmpty() && request_ && !request_->control_token.isEmpty();
}

void OperatorHmiWindow::check_simulation_status()
{
  if (options_.request_path.isEmpty()) return;
  QFile file(operator_status_file(options_.request_path));
  if (!file.open(QIODevice::ReadOnly)) return;
  const auto document = QJsonDocument::fromJson(file.readAll());
  if (!document.isObject() || document.object().value("schema").toString() != operator_status_schema()) return;
  const auto status = document.object();
  if (status == sim_status_) return;
  sim_status_ = status;
  const auto session = status.value("session").toObject();
  const QString id = session.value("session_id").toString();
  if (!id.isEmpty() && id != sim_last_session_) {
    sim_last_session_ = id; sim_last_code_.clear(); sim_last_event_.clear();
    log_event("info", "SIMULATION_SESSION", QString("Session %1 of %2 (%3), started %4 by %5.").arg(id.left(8),
      session.value("cell").toString(), session.value("kind").toString(), session.value("started").toString(),
      session.value("origin").toString()));
  }
  if (!id.isEmpty() && session.value("state_code").toString() != sim_last_code_) {
    sim_last_code_ = session.value("state_code").toString();
    const QString level = sim_last_code_ == "COMPLETED" ? "info" : sim_last_code_ == "RUNNING" ? "info" : "notice";
    log_event(level, "SIMULATION_" + sim_last_code_, session.value("state").toString());
  }
  if (!id.isEmpty() && session.value("last_event").toString() != sim_last_event_) {
    sim_last_event_ = session.value("last_event").toString();
    if (sim_last_event_ != "none yet") log_event("info", "SIMULATION_STAGE", sim_last_event_);   // only real events
  }
  const auto decision = status.value("last_command").toObject();
  if (sim_waiting_for_ >= 0 && decision.value("id").toInt(-1) == sim_waiting_for_) {
    log_event(decision.value("accepted").toBool() ? "info" : "error",
      decision.value("accepted").toBool() ? "SIMULATION_REQUEST_ACCEPTED" : "SIMULATION_REQUEST_REFUSED",
      decision.value("action").toString() + (decision.value("accepted").toBool() ? QString() : ": " + decision.value("reason").toString()));
    sim_waiting_for_ = -1;
  }
  render_monitoring();
  render_simulation();
  write_status();
}

void OperatorHmiWindow::render_simulation()
{
  if (!sim_run_button_) return;
  const auto disable = [this](const QString & state, const QString & note) {
      sim_run_button_->setEnabled(false); sim_cancel_button_->setEnabled(false);
      sim_state_->setText(state); sim_note_->setText(note); sim_result_->clear();
    };
  if (!simulation_linked()) {
    disable(state_text("NOT AVAILABLE", "neutral"),
      muted("Standalone HMI: there is no Workcell Builder control session, so nothing can be started from here."));
    return;
  }
  if (sim_status_.isEmpty()) { disable(muted("Waiting for Workcell Builder's simulation status…"), QString()); return; }
  const auto canonical = [](const QString & path) { const auto c = QFileInfo(path).canonicalFilePath(); return c.isEmpty() ? path : c; };
  const QString here = canonical(scene_dir_);
  const auto cell = sim_status_.value("cell").toObject();
  const auto readiness = sim_status_.value("readiness").toObject();
  const auto session = sim_status_.value("session").toObject();
  const bool same_cell = !here.isEmpty() && canonical(cell.value("scene_dir").toString()) == here;
  const bool session_here = !session.isEmpty() && canonical(session.value("scene_dir").toString()) == here;
  const bool active = session.value("active").toBool();
  const bool waiting = sim_waiting_for_ >= 0;

  // State of the session, only if it is this cell's: never another cell's or an earlier session's as current.
  QString state;
  if (active && !session_here) {
    state = state_text("BUSY", "warn") + " " + escaped(QString("Workcell Builder is running a session of %1.").arg(session.value("cell").toString()));
  } else if (session_here) {
    const QString code = session.value("state_code").toString();
    // Sorting sessions report their lifecycle state (STOPPED is a normal end; PAUSED/STOPPING are operator states).
    const QString tone = code == "COMPLETED" || code == "STOPPED" ? "ok" : code == "RUNNING" ? "info"
      : code == "CANCELLING" || code == "PAUSING" || code == "PAUSED" || code == "STOPPING" || code == "READY" || code == "STARTING" ? "warn" : "error";
    state = state_text(escaped(session.value("state").toString()), tone) + "<br>" +
      muted(QString("%1 · %2 · session %3 · started %4 by %5").arg(session.value("kind").toString() == "physics" ? "Qualified physics simulation"
          : session.value("plan_only").toBool() ? "Physics sorting session, plan only (the robot does not move)"
          : session.value("motion_trial").toBool() ? "Bounded Gazebo sorting session (G4 trial): the simulated robot moves"
          : session.value("kind").toString() == "sorting" ? "Sorting session, fake hardware" : "Fake-hardware plan only",
        session.value("process").toString(), session.value("session_id").toString().left(8), session.value("started").toString(),
        session.value("origin").toString()));
    if (!active) state = "<b>Last session of this cell</b><br>" + state;
  } else {
    state = muted("No simulation of this cell in this Workcell Builder session.");
  }
  sim_state_->setText(state);

  QString reason;
  if (!same_cell) reason = "Workcell Builder has another cell selected; Refresh this HMI.";
  else if (update_banner_ && update_banner_->isVisible()) reason = "Workcell Builder has newer engineering state; Refresh first.";
  else if (!view_loaded_ || !view_.usable) reason = "The cell's configuration is not available.";
  else if (view_.draft) reason = "A DRAFT task cannot run: save it in Workcell Builder.";
  else if (view_.layout_unsaved) reason = "Workcell Builder has unsaved layout edits: save them first.";
  else if (active) reason = "A session is running.";
  else if (!readiness.value("ready").toBool())
    reason = readiness.value("blocker").toString() + (readiness.value("action").toString().isEmpty() ? QString() : " What to do: " + readiness.value("action").toString());
  else if (view_.saved_intent_sha256 != readiness.value("intent_sha256").toString())
    reason = "The saved task shown here is not the one the cell was generated from: Generate in Workcell Builder, then Refresh.";
  else if (waiting) reason = "Waiting for Workcell Builder to decide on the last request.";
  sim_run_button_->setEnabled(reason.isEmpty());
  sim_cancel_button_->setEnabled(active && session_here && session.value("kind").toString() == "physics" &&
                                 !session.value("cancel_requested").toBool() && !waiting);
  sim_note_->setText(reason.isEmpty()
    ? muted(QString("Ready: %1. Simulated robot only; the real robot stays locked.").arg(readiness.value("backend").toString()))
    : state_text("Run unavailable: ", "warn") + escaped(reason));

  // Live and final evidence from the Builder's monitor of this cell's session (R2.2 semantics, nothing derived here).
  QString result;
  if (session_here) {
    result = "<b>Stages</b><br>" + session.value("timeline_html").toString() +
      "<br>" + muted("Last verified event: " + session.value("last_event").toString()) +
      "<br>Selected object: " + escaped(session.value("selected_object").toString());
    QStringList rejected; for (const auto & r : session.value("rejected").toArray()) rejected << r.toString();
    if (!rejected.isEmpty()) result += "<br>Not selected: " + escaped(rejected.join("; "));
    const auto lines = session.value("outcome_lines").toArray();
    if (!lines.isEmpty()) {
      result += "<br><b>Final evidence</b>";
      for (const auto & line : lines) result += "<br>" + escaped(line.toString());
    }
    const auto diagnosis = session.value("diagnosis").toObject();
    if (!diagnosis.isEmpty()) {
      result += "<br>" + state_text(diagnosis.value("code").toString(), "error") + " " + escaped(diagnosis.value("meaning").toString());
      if (!diagnosis.value("action").toString().isEmpty()) result += "<br>" + muted("What to do: " + diagnosis.value("action").toString());
    }
    robot_status_->setText(QString("<b>Robot · simulated</b><br>%1<br>%2<br>%3").arg(escaped(session.value("robot").toString()),
      muted("Gripper: " + session.value("gripper").toString()), state_text("Real robot: LOCKED", "ok")));
    cycle_status_->setText(QString("<b>Cycle · %1</b><br>%2").arg(escaped(session.value("state_code").toString()),
      muted(session.value("last_event").toString())));
  }
  sim_result_->setText(result);
  render_sorting();
  render_capabilities();
}

void OperatorHmiWindow::render_sorting()
{
  if (!sort_start_button_) return;
  for (auto * button : {sort_start_button_, sort_pause_button_, sort_resume_button_, sort_stop_button_}) button->setEnabled(false);
  sort_result_->clear();
  if (!simulation_linked()) {
    sort_state_->setText(state_text("NOT AVAILABLE", "neutral"));
    sort_note_->setText(muted("Standalone HMI: sorting is started and owned by Workcell Builder."));
    return;
  }
  const auto canonical = [](const QString & path) { const auto c = QFileInfo(path).canonicalFilePath(); return c.isEmpty() ? path : c; };
  const auto session = sim_status_.value("session").toObject();
  const auto readiness = sim_status_.value("readiness").toObject().value("sorting").toObject();
  const bool here = !session.isEmpty() && canonical(session.value("scene_dir").toString()) == canonical(scene_dir_);
  const auto sorting = here && session.value("kind").toString() == "sorting" ? session.value("sorting").toObject() : QJsonObject();
  const bool active = session.value("active").toBool();
  const bool waiting = sim_waiting_for_ >= 0;
  const QString state = sorting.value("state").toString();
  if (sorting.isEmpty()) sort_state_->setText(muted("No sorting session of this cell in this Workcell Builder session."));
  else {
    const QString tone = state == "RUNNING" ? "info" : state == "STOPPED" ? "ok" : state == "FAULT" ? "error" : "warn";
    sort_state_->setText(state_text(escaped(state.isEmpty() ? QString("STARTING") : state), tone) + " " +
                         muted(sorting.value("state_reason").toString()) + (active ? QString() : "<br>" + muted("Session ended.")));
  }
  if (here && active && session.value("stop_requested").toBool())
    sort_state_->setText(sort_state_->text() + "<br>" + state_text("STOP REQUESTED", "warn") + " " +
                         muted("The current cycle finishes under every guard and is verified; no further cycle starts."));
  QString reason;
  if (active) reason = "A session is running.";
  else if (view_.draft) reason = "A DRAFT task cannot run: save it in Workcell Builder.";
  else if (update_banner_ && update_banner_->isVisible()) reason = "Workcell Builder has newer engineering state; Refresh first.";
  else if (!readiness.value("ready").toBool()) reason = readiness.value("blocker").toString("Sorting readiness unavailable.");
  else if (view_.saved_intent_sha256 != sim_status_.value("readiness").toObject().value("intent_sha256").toString())
    reason = "The saved task shown here is not the one the cell was generated from: Generate in Workcell Builder, then Refresh.";
  else if (waiting) reason = "Waiting for Workcell Builder to decide on the last request.";
  sort_start_button_->setEnabled(reason.isEmpty());
  const auto admission = readiness.value("motion").toObject();
  const QString motion_reason = !reason.isEmpty() ? reason : admission.value("admitted").toBool() ? QString()
    : admission.value("blocker").toString("Robot motion is not admitted for this cell.");
  sort_motion_button_->setEnabled(motion_reason.isEmpty());
  sort_motion_button_->setToolTip(motion_reason.isEmpty()
    ? "Bounded G4 sorting session in Gazebo, at most 2 cycles: the simulated robot moves. Workcell Builder recomputes the "
      "cell's G4 qualification from the raw evidence before the runner starts." : motion_reason);
  // A bounded G4 trial has no pause; Stop is its stop at cycle end and is available for the whole session.
  const bool trial = here && session.value("motion_trial").toBool();
  const bool controllable = active && !sorting.isEmpty() && !waiting;
  sort_pause_button_->setEnabled(controllable && !trial && state == "RUNNING");
  sort_resume_button_->setEnabled(controllable && !trial && (state == "PAUSED" || state == "PAUSING"));
  sort_stop_button_->setEnabled(trial ? active && !waiting && !session.value("cancel_requested").toBool() &&
                                         !session.value("stop_requested").toBool()
                                      : controllable && (state == "RUNNING" || state == "PAUSING" || state == "PAUSED" || state == "READY"));
  const bool plan_only = readiness.value("plan_only").toBool() || session.value("plan_only").toBool();
  sort_start_button_->setText(plan_only ? "Start Sorting · Plan Only" : "Start Sorting");
  sort_note_->setText(reason.isEmpty() || active
    ? muted(trial
        ? "Gazebo physics: the simulated robot picks, transports and places, at most 2 cycles. An object is counted only "
          "after the runner verified and reconciled its cycle. Stop finishes the current cycle (not an emergency stop); "
          "Cancel Simulation stops the motion under supervision. Real robot: not available."
        : plan_only
        ? "Physics world (Gazebo): each cycle renders the cell, classifies it with EPD, routes and plans one object. Plan "
          "only: every cycle authorisation is refused, the robot never moves and nothing is counted as sorted. Pause and "
          "Stop act at the end of the current cycle; they are not an emergency stop. Real robot: LOCKED."
        : "MoveIt fake hardware: planning and kinematic execution only, no physics. Pause and Stop act at the end of the "
          "current cycle; they are not an emergency stop. Real robot: LOCKED.")
      + (motion_reason.isEmpty() || trial ? QString() : "<br>" + muted("Gazebo motion: " + escaped(motion_reason)))
    : state_text("Start unavailable: ", "warn") + escaped(reason));
  if (sorting.isEmpty()) return;
  // Counts and the active object are the session's own (counted only from verified execution evidence).
  const auto counts = sorting.value("counts").toObject();
  const auto current = sorting.value("active").toObject();
  QString text = current.isEmpty() ? muted("Active object: none (between cycles)")
    : QString("Active object: <b>%1</b> (%2) → <b>%3</b> · cycle %4 · stage %5").arg(escaped(current.value("id").toString()),
        escaped(current.value("class_id").toString()), escaped(current.value("destination").toString()))
        .arg(current.value("cycle").toInt()).arg(escaped(sorting.value("stage").toString("starting")));
  const auto observation = sorting.value("observation").toObject();
  if (!observation.value("acquiring").toString().isEmpty())
    text += "<br>Observation: <b>acquiring</b> — " + escaped(observation.value("acquiring").toString());
  else if (observation.value("producer").toString() == "simulator_render")
    text += QString("<br>Observation: frame %1 · simulator-rendered (synthetic), classified by EPD · %2 classified, %3 unperceived")
      .arg(observation.value("frames").toInt()).arg(observation.value("classified").toInt()).arg(observation.value("unperceived").toInt());
  text += QString("<br>Completed <b>%1</b> · Failed <b>%2</b> · Skipped %3 · Waiting (bin full) %4 · Pending %5")
    .arg(counts.value("completed").toInt()).arg(counts.value("failed").toInt()).arg(counts.value("skipped").toInt())
    .arg(counts.value("waiting_destination_full").toInt()).arg(counts.value("pending").toInt());
  QStringList classes, bins;
  const auto per_class = counts.value("per_class").toObject();
  for (auto it = per_class.begin(); it != per_class.end(); ++it)
    classes << QString("%1: %2 done, %3 failed").arg(escaped(it.key())).arg(it.value().toObject().value("completed").toInt())
                 .arg(it.value().toObject().value("failed").toInt());
  const auto per_destination = counts.value("per_destination").toObject();
  for (auto it = per_destination.begin(); it != per_destination.end(); ++it) bins << QString("%1: %2").arg(escaped(it.key())).arg(it.value().toInt());
  text += "<br>Per class: " + (classes.isEmpty() ? muted("none") : classes.join(" · "));
  text += "<br>Per destination: " + (bins.isEmpty() ? muted("none yet") : bins.join(" · "));
  const auto fault = sorting.value("fault").toObject();
  if (!fault.isEmpty()) {
    text += "<br>" + state_text("FAULT " + escaped(fault.value("code").toString()), "error") + " " + escaped(fault.value("reason").toString());
    const auto diagnosis = session.value("diagnosis").toObject();
    if (!diagnosis.value("action").toString().isEmpty()) text += "<br>" + muted("What to do: " + diagnosis.value("action").toString());
  }
  const auto g4 = session_g4();
  if (!g4.isEmpty()) {
    QStringList cycles;
    for (const auto & item : g4.value("cycles").toArray()) {
      const auto c = item.toObject();
      const auto plan = c.value("plan").toObject();
      const auto authorisation = c.value("authorisation").toObject();
      QStringList seen;
      for (const auto & object : c.value("objects").toArray()) {
        const auto o = object.toObject();
        seen << o.value("object_id").toString() + " " + o.value("class_id").toString() +
                (o.value("confidence").isDouble() ? QString(" %1").arg(o.value("confidence").toDouble(), 0, 'f', 2) : QString());
      }
      QString line = QString("Cycle %1 · EPD: %2").arg(c.value("cycle").toInt()).arg(escaped(seen.isEmpty() ? QString("classifying…") : seen.join(", ")));
      if (!plan.isEmpty())
        line += QString(" · plan %1%2").arg(escaped(plan.value("state").toString()), plan.value("target").toString().isEmpty() ? QString()
          : escaped(QString(": %1 → %2 (%3)").arg(plan.value("target").toString().section("::", -1), plan.value("destination").toString(),
                                                 plan.value("grasp").toString())));
      if (!authorisation.isEmpty())
        line += " · " + (authorisation.value("granted").toBool() ? state_text("AUTHORISED", "warn")
                                                                 : muted("authorisation refused (" + authorisation.value("refusal").toString().section(':', 0, 0) + ")"));
      if (c.value("executed").toBool()) line += " · executed";
      if (!c.value("outcome").toString().isEmpty())
        line += " · " + state_text(escaped(c.value("outcome").toString()), c.value("outcome").toString() == "COMPLETED" ? "ok" : "neutral");
      cycles << line;
    }
    const auto motion = g4.value("motion").toObject();
    text += "<br><b>Cycles</b><br>" + (cycles.isEmpty() ? muted("starting…") : cycles.join("<br>"));
    text += "<br>" + muted(QString("Evidence of motion: authorisations granted %1 · executed cycles %2 · controller goals %3")
      .arg(motion.value("authorised_cycles").toInt()).arg(motion.value("executed_cycles").toInt()).arg(motion.value("controller_goals").toInt()));
    if (g4.value("finished").toBool())
      text += "<br>" + muted(QString("Session ended %1 · %2 · shutdown %3").arg(g4.value("status").toString(), g4.value("result").toString(),
        g4.value("shutdown").toObject().value("clean").toBool() ? "clean" : "NOT clean"));
  }
  sort_result_->setText(text);
}

void OperatorHmiWindow::render_capabilities()
{
  if (!capabilities_) return;
  const auto rows = sim_status_.value("readiness").toObject().value("capabilities").toArray();
  if (!simulation_linked() || rows.isEmpty()) {
    capabilities_->setText(muted("Capabilities are reported by Workcell Builder for its selected cell."));
    return;
  }
  QString text;
  for (const auto & item : rows) {
    const auto row = item.toObject();
    const QString status = row.value("status").toString();
    const QString tone = status == "QUALIFIED" ? "ok" : status == "EXPERIMENTAL" ? "info"
                       : status == "LOCKED" || status == "BLOCKED" ? "error" : "neutral";
    text += state_text(status, tone) + " <b>" + escaped(row.value("title").toString()) + "</b><br>" +
            muted(row.value("detail").toString()) + "<br>";
  }
  capabilities_->setText(text);
}

void OperatorHmiWindow::request_simulation(const QString & action, bool motion)
{
  if (!simulation_linked() || sim_waiting_for_ >= 0) return;
  const auto session = sim_status_.value("session").toObject();
  if (action == "run_simulation") {
    const auto readiness = sim_status_.value("readiness").toObject();
    // An explicit, specific confirmation: nothing runs on a single click.
    if (QMessageBox::question(this, "Run Simulation Cycle",
          QString("Run one bounded pick-and-place SIMULATION of the saved cell \"%1\"?\n\nTask: pick %2 → %3\nBackend: %4\n\n"
                  "Simulated robot only: the real robot stays locked and this is not production.\n"
                  "Workcell Builder runs and supervises the simulation. Cancel Simulation is not an emergency stop.")
            .arg(view_.scene_name, view_.target_class.isEmpty() ? QString("any class") : view_.target_class,
                 view_.destinations.join(", "), readiness.value("backend").toString()),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
      log_event("info", "SIMULATION_REQUEST_DECLINED", "The operator did not confirm the simulation.");
      return;
    }
  }
  const bool plan_only = sim_status_.value("readiness").toObject().value("sorting").toObject().value("plan_only").toBool();
  if (motion &&
      QMessageBox::question(this, "Start Sorting · Gazebo Motion",
        QString("Start a BOUNDED sorting session of the saved cell \"%1\" in which the SIMULATED robot moves in Gazebo?\n\n"
                "At most 2 cycles, one routed object each: rendered camera frame → EPD → routing → certified MoveIt plan → "
                "grasp, lift, transport, place, release → re-observation. An object is counted only after it was verified "
                "and reconciled.\n\nWorkcell Builder first recomputes this cell's G4 qualification from the raw evidence "
                "(several minutes); any change refuses motion. Stop finishes the current cycle (not an emergency stop); "
                "Cancel Simulation stops the motion under supervision.\nThe real robot is not available; this is not "
                "production.").arg(view_.scene_name), QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
    log_event("info", "SORTING_REQUEST_DECLINED", "The operator did not confirm the Gazebo motion session.");
    return;
  }
  if (action == "start_sorting" && !motion &&
      QMessageBox::question(this, plan_only ? "Start Sorting · Plan Only" : "Start Sorting",
        (plan_only
          ? QString("Plan the saved cell \"%1\" with its routing table, one object per cycle, until every routed object is "
                    "planned or you stop it?\n\nPhysics world (Gazebo): each cycle renders the cell with its camera, "
                    "classifies it with EPD, routes and plans one object with MoveIt. PLAN ONLY: every cycle authorisation is "
                    "refused, the robot never moves and nothing is counted as sorted. The real robot stays locked and this is "
                    "not production.\nPause and Stop act at the end of the current cycle; they are not an emergency stop.")
          : QString("Sort the saved cell \"%1\" with its routing table, one object per cycle, until the batch is done or you stop it?\n\n"
                    "MoveIt fake hardware only (planning and kinematic execution, no physics). The real robot stays locked and "
                    "this is not production.\nPause and Stop act at the end of the current cycle; they are not an emergency stop."))
          .arg(view_.scene_name), QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
    log_event("info", "SORTING_REQUEST_DECLINED", "The operator did not confirm the sorting session.");
    return;
  }
  auto command = make_operator_command(++sim_command_id_, action, request_->control_token, scene_dir_,
    view_.saved_intent_sha256, request_->revision, session.value("session_id").toString());
  if (motion) { command["mode"] = "bounded_motion"; command["cycles"] = 2; }
  const QByteArray line = operator_command_line(command);
  std::fwrite(line.constData(), 1, static_cast<size_t>(line.size()), stdout);   // only the owning Builder reads this
  std::fflush(stdout);
  sim_waiting_for_ = sim_command_id_;
  if (action.endsWith("_sorting"))
    log_event("info", "SORTING_" + action.section('_', 0, 0).toUpper() + (motion ? "_MOTION" : "") + "_REQUESTED",
      QString("Asked Workcell Builder to %1 the sorting session%2.").arg(action.section('_', 0, 0),
        action == "start_sorting" ? QString() : QString(" (applied at the end of the current cycle; not an emergency stop)")));
  else
    log_event("info", action == "run_simulation" ? "SIMULATION_REQUESTED" : "SIMULATION_CANCEL_REQUESTED",
      action == "run_simulation" ? "Asked Workcell Builder to run the simulation." :
                                   "Asked Workcell Builder to cancel the simulation (supervised cleanup, not an emergency stop).");
  render_simulation();
}

// Test-only: acceptance runs drive the real buttons through a command file so
// the separate HMI process is exercised exactly as an operator would use it.
void OperatorHmiWindow::poll_acceptance_commands()
{
  QFile file(options_.acceptance_commands);
  if (!file.open(QIODevice::ReadOnly)) return;
  const auto commands = QJsonDocument::fromJson(file.readAll()).array();
  for (const auto & value : commands) {
    const auto command = value.toObject();
    const int id = command.value("id").toInt();
    if (id <= last_command_) continue;
    last_command_ = id;
    const QString action = command.value("action").toString();
    QString result = "done";
    auto click = [&result](QPushButton * button) {
        if (button && button->isEnabled()) {
          button->click();
        } else {
          result = "rejected: control disabled";
        }
      };
    const QHash<QString, QPushButton *> buttons{
      {"replay_check", replay_button_}, {"dry_run", dry_run_button_}, {"check", check_button_},
      {"refresh", refresh_button_}, {"stop", stop_button_}, {"start_production", start_button_},
      {"cancel_simulation", sim_cancel_button_}, {"pause_sorting", sort_pause_button_},
      {"resume_sorting", sort_resume_button_}, {"stop_sorting", sort_stop_button_}};
    if (buttons.contains(action)) {
      click(buttons.value(action));
    } else if (action == "select_object") {
      result = "rejected: object not shown";
      for (int row = 0; row < objects_->rowCount(); ++row) {
        if (objects_->item(row, 0) && objects_->item(row, 0)->text() == command.value("object_id").toString()) {
          objects_->selectRow(row);
          result = "done";
        }
      }
    } else if (action == "capture" && !options_.capture_dir.isEmpty()) {
      QDir().mkpath(options_.capture_dir);
      grab().save(QDir(options_.capture_dir).filePath(command.value("name").toString() + ".png"));
    } else if (action == "close") {
      QTimer::singleShot(0, this, &QWidget::close);
    } else {
      result = "rejected: unknown action";
    }
    command_results_.append(QJsonObject{{"id", id}, {"action", action}, {"result", result}});
    write_status();
  }
}

}  // namespace workcell_builder
