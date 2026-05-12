#include "conveyor_sorting_run_console.h"
#include "ui_conveyor_sorting_run_console.h"

#include <QClipboard>
#include <QDateTime>
#include <QDesktopServices>
#include <QFile>
#include <QGuiApplication>
#include <QTextStream>
#include <QUrl>

namespace fs = std::filesystem;

ConveyorSortingRunConsole::ConveyorSortingRunConsole(const fs::path & scene_path, const QString & scenario_name, QWidget * parent)
: QDialog(parent), ui_(new Ui::ConveyorSortingRunConsole), scene_path_(scene_path), scenario_name_(scenario_name)
{
  ui_->setupUi(this);
  connect(ui_->refreshButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onRefresh);
  connect(ui_->copyBuildButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onCopyBuild);
  connect(ui_->copyLaunchButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onCopyLaunch);
  connect(ui_->copySampleButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onCopySample);
  connect(ui_->openSceneFolderButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onOpenSceneFolder);
  connect(ui_->openPreviewFolderButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onOpenPreviewFolder);
  connect(ui_->openRvizButton, &QPushButton::clicked, this, &ConveyorSortingRunConsole::onOpenRvizConfig);
  onRefresh();
}

ConveyorSortingRunConsole::~ConveyorSortingRunConsole() { delete ui_; }

bool ConveyorSortingRunConsole::isConveyorSortingScenario(const fs::path & scene_path)
{
  const auto p = scene_path / "config" / "scenario.yaml";
  if (!fs::exists(p)) return false;
  QFile f(QString::fromStdString(p.string()));
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
  return QString::fromUtf8(f.readAll()).contains("conveyor_sorting_live_epd_preview");
}

QString ConveyorSortingRunConsole::buildRunCommandsText() const
{
  return QString("1) colcon build --symlink-install --packages-select %1\n"
                 "2) source install/setup.bash\n"
                 "3) ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true enable_conveyor_sorting_preview:=true publish_sample_detections:=true\n"
                 "4) For real EPD feed, set publish_sample_detections:=false and publish to:\n"
                 "   /workcell_studio/epd_detection_snapshot_json")
    .arg(scenario_name_);
}

QString ConveyorSortingRunConsole::summarizeMissingFiles() const
{
  QStringList required = {"config/environment.yaml", "config/scenario.yaml", "launch/demo.launch.py",
    QString("rviz/%1.rviz").arg(scenario_name_), "preview/live_conveyor_sorting_status.json",
    "preview/live_task_intent_preview.yaml", "preview/live_emd_grasp_planner_request.yaml", "preview/scenario_readiness_report.yaml"};
  QStringList missing;
  for (const auto & rel : required) if (!fs::exists(scene_path_ / rel.toStdString())) missing << rel;
  return missing.isEmpty() ? "All expected artifacts found." : "Missing: " + missing.join(", ");
}

QString ConveyorSortingRunConsole::readStatusArtifact() const
{
  const auto p = scene_path_ / "preview" / "live_conveyor_sorting_status.json";
  if (!fs::exists(p)) return "status: missing preview/live_conveyor_sorting_status.json";
  QFile f(QString::fromStdString(p.string()));
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return "status: malformed status";
  return QString::fromUtf8(f.readAll());
}

QString ConveyorSortingRunConsole::safetyStateText(const QString & status_text) const
{
  if (summarizeMissingFiles().startsWith("Missing:")) return "red: missing generated package / missing launch file / malformed status";
  if (status_text.contains("robot_motion_commanded: true")) return "red: unsafe state detected";
  if (status_text.contains("detection_count: 0")) return "amber: waiting for EPD snapshot";
  return "green: fake preview safe";
}

void ConveyorSortingRunConsole::onRefresh()
{
  const QString status_artifact = readStatusArtifact();
  ui_->summaryText->setPlainText(
    QString("scenario name: %1\nscene path: %2\ngenerated package status: %3\npackage built status if detectable: unknown\nlaunch file status: %4\nRViz config status: %5\nfake hardware default: true\nreal_hardware_ready: false")
      .arg(scenario_name_)
      .arg(QString::fromStdString(scene_path_.string()))
      .arg(fs::exists(scene_path_) ? "present" : "missing")
      .arg(fs::exists(scene_path_ / "launch" / "demo.launch.py") ? "present" : "missing")
      .arg(fs::exists(scene_path_ / "rviz" / (scenario_name_.toStdString() + ".rviz")) ? "present" : "missing"));
  ui_->workflowText->setPlainText(buildRunCommandsText());
  ui_->statusText->setPlainText(QString("last update time: %1\n%2").arg(QDateTime::currentDateTime().toString(Qt::ISODate), status_artifact));
  ui_->safetyText->setPlainText(
    QString("robot_motion_commanded: false\nmoveit_execute_called: false\ngripper_command_sent: false\nconveyor_command_sent: false\nreal_hardware_ready: false\nstate: %1")
      .arg(safetyStateText(status_artifact)));
  ui_->troubleshootingText->setPlainText(
    "- If no markers appear: check RViz marker topic\n"
    "- If no detections appear: check sample publisher or EPD topic\n"
    "- If pick_ready never becomes true: check conveyor_flow and speed\n"
    "- If class routes to reject: check class routing table\n"
    "- If camera mismatch: check camera name in snapshot\n"
    + summarizeMissingFiles());
}

void ConveyorSortingRunConsole::onCopyBuild(){ QGuiApplication::clipboard()->setText(QString("colcon build --symlink-install --packages-select %1").arg(scenario_name_)); }
void ConveyorSortingRunConsole::onCopyLaunch(){ QGuiApplication::clipboard()->setText(QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true enable_conveyor_sorting_preview:=true publish_sample_detections:=true").arg(scenario_name_)); }
void ConveyorSortingRunConsole::onCopySample(){ QGuiApplication::clipboard()->setText("python3 scripts/publish_sample_epd_snapshot.py --camera realsense_d435i_1 --zone detection_zone_1"); }
void ConveyorSortingRunConsole::onOpenSceneFolder(){ QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(scene_path_.string()))); }
void ConveyorSortingRunConsole::onOpenPreviewFolder(){ QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((scene_path_/"preview").string()))); }
void ConveyorSortingRunConsole::onOpenRvizConfig(){ QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((scene_path_/"rviz"/(scenario_name_.toStdString()+".rviz")).string()))); }
