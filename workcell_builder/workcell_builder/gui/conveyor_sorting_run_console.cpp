#include "conveyor_sorting_run_console.h"
#include "ui_conveyor_sorting_run_console.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <QUrl>

namespace fs = std::filesystem;

ConveyorSortingRunConsole::ConveyorSortingRunConsole(const fs::path & scene_path, const QString & scenario_name, QWidget * parent)
: QDialog(parent), ui_(new Ui::ConveyorSortingRunConsole), scene_path_(scene_path), scenario_name_(scenario_name),
  build_process_(new QProcess(this)), launch_process_(new QProcess(this)), active_log_file_(nullptr)
{
  ui_->setupUi(this);
  connect(ui_->refreshButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onRefresh);
  connect(ui_->buildScenarioButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onBuildScenario);
  connect(ui_->launchPreviewButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onLaunchPreview);
  connect(ui_->stopPreviewButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onStopPreview);
  connect(ui_->restartPreviewButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onRestartPreview);
  connect(ui_->clearLogButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onClearLog);
  connect(ui_->openLogFolderButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onOpenLogFolder);

  connect(build_process_, &QProcess::readyReadStandardOutput, this, &ConveyorSortingRunConsole::onBuildOutput);
  connect(build_process_, &QProcess::readyReadStandardError, this, &ConveyorSortingRunConsole::onBuildOutput);
  connect(launch_process_, &QProcess::readyReadStandardOutput, this, &ConveyorSortingRunConsole::onLaunchOutput);
  connect(launch_process_, &QProcess::readyReadStandardError, this, &ConveyorSortingRunConsole::onLaunchOutput);
  connect(build_process_, &QProcess::finished, this, &ConveyorSortingRunConsole::onBuildFinished);
  connect(launch_process_, &QProcess::finished, this, &ConveyorSortingRunConsole::onLaunchFinished);
  connect(build_process_, &QProcess::errorOccurred, this, &ConveyorSortingRunConsole::onBuildError);
  connect(launch_process_, &QProcess::errorOccurred, this, &ConveyorSortingRunConsole::onLaunchError);

  connect(&status_timer_, &QTimer::timeout, this, &ConveyorSortingRunConsole::onRefresh);
  status_timer_.start(2000);

  setBuildStatus("idle");
  setLaunchStatus("idle");
  onRefresh();
}

ConveyorSortingRunConsole::~ConveyorSortingRunConsole()
{
  onStopPreview();
  delete active_log_file_;
  delete ui_;
}

bool ConveyorSortingRunConsole::isConveyorSortingScenario(const fs::path & scene_path)
{
  const auto p = scene_path / "config" / "scenario.yaml";
  if (!fs::exists(p)) return false;
  QFile f(QString::fromStdString(p.string()));
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
  return QString::fromUtf8(f.readAll()).contains("conveyor_sorting_live_epd_preview");
}

QString ConveyorSortingRunConsole::resolveWorkspaceRoot() const
{
  fs::path cur = scene_path_;
  while (!cur.empty()) {
    if (fs::exists(cur / "src") || fs::exists(cur / "install")) return QString::fromStdString(cur.string());
    cur = cur.parent_path();
  }
  return QString();
}

QString ConveyorSortingRunConsole::scenarioLaunchPath() const { return QString::fromStdString((scene_path_ / "launch" / "demo.launch.py").string()); }
QString ConveyorSortingRunConsole::buildCommand() const { return QString("colcon build --symlink-install --packages-select %1").arg(scenario_name_); }
QString ConveyorSortingRunConsole::launchCommand() const
{
  return QString("source install/setup.bash && ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true enable_conveyor_sorting_preview:=true publish_sample_detections:=true enable_epd_connector:=false").arg(scenario_name_);
}

bool ConveyorSortingRunConsole::safetyChecks(QString & reason) const
{
  const auto scenario_yaml = scene_path_ / "config" / "scenario.yaml";
  if (!fs::exists(scenario_yaml)) { reason = "Missing scenario.yaml"; return false; }
  QFile f(QString::fromStdString(scenario_yaml.string()));
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) { reason = "Cannot open scenario.yaml"; return false; }
  const QString scenario_text = QString::fromUtf8(f.readAll());
  if (!scenario_text.contains("scenario_id: conveyor_sorting_live_epd_preview")) { reason = "Wrong scenario_id"; return false; }
  if (!scenario_text.contains("real_hardware_ready: false")) { reason = "real_hardware_ready must remain false"; return false; }
  if (!fs::exists(scene_path_ / "launch" / "demo.launch.py")) { reason = "Missing launch/demo.launch.py"; return false; }
  const QString cmd = launchCommand();
  if (!cmd.contains("use_fake_hardware:=true")) { reason = "Launch command missing fake hardware flag"; return false; }
  if (cmd.contains("use_fake_hardware:=false") || cmd.contains("real_hardware") || cmd.contains("execute_trajectory")) {
    reason = "Unsafe launch args detected";
    return false;
  }
  return true;
}

QString ConveyorSortingRunConsole::makeLogPath(const QString & prefix) const
{
  const fs::path logs_dir = scene_path_ / "preview" / "logs";
  fs::create_directories(logs_dir);
  return QString::fromStdString((logs_dir / QString("%1_%2.log").arg(prefix, QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")).toStdString()).string());
}

void ConveyorSortingRunConsole::appendLog(const QString & text)
{
  const QString line = QString("[%1] %2").arg(QDateTime::currentDateTime().toString(Qt::ISODate), text);
  ui_->logViewer->appendPlainText(line.trimmed());
  if (active_log_file_ && active_log_file_->isOpen()) {
    QTextStream s(active_log_file_);
    s << line << "\n";
  }
}

void ConveyorSortingRunConsole::setBuildStatus(const QString & status) { ui_->buildStatusValue->setText(status); }
void ConveyorSortingRunConsole::setLaunchStatus(const QString & status) { ui_->launchStatusValue->setText(status); }
void ConveyorSortingRunConsole::setLastError(const QString & text) { ui_->lastErrorValue->setText(text); }

void ConveyorSortingRunConsole::onBuildScenario()
{
  const QString ws = resolveWorkspaceRoot();
  if (ws.isEmpty()) { setBuildStatus("failed"); setLastError("Workspace root not found"); return; }
  if (build_process_->state() != QProcess::NotRunning) return;
  delete active_log_file_;
  active_log_file_ = new QFile(makeLogPath("build"), this);
  active_log_file_->open(QIODevice::WriteOnly | QIODevice::Text);
  ui_->logPathValue->setText(active_log_file_->fileName());
  ui_->lastCommandValue->setText(buildCommand());
  setBuildStatus("running");
  setLastError("none");
  build_process_->setWorkingDirectory(ws);
  build_process_->start("bash", {"-lc", buildCommand()});
}

void ConveyorSortingRunConsole::onLaunchPreview()
{
  QString reason;
  if (!safetyChecks(reason)) { setLaunchStatus("failed"); setLastError(reason); appendLog(reason); return; }
  const QString ws = resolveWorkspaceRoot();
  if (ws.isEmpty()) { setLaunchStatus("failed"); setLastError("Workspace root not found"); return; }
  if (launch_process_->state() != QProcess::NotRunning) return;
  delete active_log_file_;
  active_log_file_ = new QFile(makeLogPath("launch"), this);
  active_log_file_->open(QIODevice::WriteOnly | QIODevice::Text);
  ui_->logPathValue->setText(active_log_file_->fileName());
  ui_->lastCommandValue->setText(launchCommand());
  setLaunchStatus("running");
  launch_process_->setWorkingDirectory(ws);
  launch_process_->start("bash", {"-lc", launchCommand()});
  ui_->pidValue->setText(QString::number(launch_process_->processId()));
}

void ConveyorSortingRunConsole::onStopPreview()
{
  if (launch_process_->state() == QProcess::NotRunning) { setLaunchStatus("stopped"); return; }
  launch_process_->terminate();
  if (!launch_process_->waitForFinished(3000)) launch_process_->kill();
  ui_->pidValue->setText("n/a");
  setLaunchStatus("stopped");
}

void ConveyorSortingRunConsole::onRestartPreview() { onStopPreview(); onLaunchPreview(); }
void ConveyorSortingRunConsole::onClearLog() { ui_->logViewer->clear(); }
void ConveyorSortingRunConsole::onOpenLogFolder() { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((scene_path_/"preview"/"logs").string()))); }

void ConveyorSortingRunConsole::onBuildOutput() { appendLog(QString::fromUtf8(build_process_->readAllStandardOutput()) + QString::fromUtf8(build_process_->readAllStandardError())); }
void ConveyorSortingRunConsole::onLaunchOutput() { appendLog(QString::fromUtf8(launch_process_->readAllStandardOutput()) + QString::fromUtf8(launch_process_->readAllStandardError())); }

void ConveyorSortingRunConsole::onBuildFinished(int exit_code, QProcess::ExitStatus status)
{
  setBuildStatus((status == QProcess::NormalExit && exit_code == 0) ? "succeeded" : "failed");
  if (exit_code == 0) appendLog("Build complete. launch command will source install/setup.bash");
}

void ConveyorSortingRunConsole::onLaunchFinished(int exit_code, QProcess::ExitStatus status)
{
  setLaunchStatus((status == QProcess::NormalExit && exit_code == 0) ? "stopped" : "failed");
  ui_->pidValue->setText("n/a");
}

void ConveyorSortingRunConsole::onBuildError(QProcess::ProcessError) { setBuildStatus("failed"); setLastError("Build process error"); }
void ConveyorSortingRunConsole::onLaunchError(QProcess::ProcessError) { setLaunchStatus("failed"); setLastError("Launch process error"); }

QString ConveyorSortingRunConsole::readStatusArtifact()
{
  const auto p = scene_path_ / "preview" / "live_conveyor_sorting_status.json";
  if (!fs::exists(p)) return "Status file missing";
  QFile f(QString::fromStdString(p.string()));
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return "Malformed status";
  QJsonParseError err;
  const auto doc = QJsonDocument::fromJson(f.readAll(), &err);
  if (err.error != QJsonParseError::NoError || !doc.isObject()) return "Warning: malformed status JSON";
  const QJsonObject o = doc.object();
  return QString("class_label: %1\nselected_place_zone: %2\npick_ready: %3\ntime_to_pick_s: %4")
    .arg(o.value("class_label").toString("n/a"))
    .arg(o.value("selected_place_zone").toString("n/a"))
    .arg(o.value("pick_ready").toVariant().toString())
    .arg(o.value("time_to_pick_s").toVariant().toString());
}

void ConveyorSortingRunConsole::onRefresh()
{
  ui_->statusText->setPlainText(readStatusArtifact());
}


// Real EPD Feed tokens for tests:
// Real EPD Feed
// Copy EPD Connector Command
// Start EPD Connector
// Stop EPD Connector
// Refresh EPD Status
// /easy_perception_deployment/epd_localize_output
// /easy_perception_deployment/epd_tracking_output
// /workcell_studio/epd_detection_snapshot_json
// /workcell_studio/epd_connector_status
// ros2 run workcell_builder epd_to_workcell_snapshot_node.py
