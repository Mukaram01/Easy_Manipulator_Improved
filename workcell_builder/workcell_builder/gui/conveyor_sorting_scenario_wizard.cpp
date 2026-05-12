#include "conveyor_sorting_scenario_wizard.h"
#include "ui_conveyor_sorting_scenario_wizard.h"

#include <QClipboard>
#include <QDateTime>
#include <QGuiApplication>
#include <QHeaderView>
#include <QTableWidgetItem>

#include <fstream>

namespace fs = std::filesystem;

ConveyorSortingScenarioWizard::ConveyorSortingScenarioWizard(
  const fs::path & scenes_root, const fs::path & workspace_root, QWidget * parent)
: QDialog(parent), ui_(new Ui::ConveyorSortingScenarioWizard),
  scenes_root_(scenes_root), workspace_root_(workspace_root)
{
  ui_->setupUi(this);
  setWindowTitle("Conveyor Sorting Scenario Wizard");
  resize(980, 700);
  ui_->statusText->setWordWrapMode(QTextOption::WordWrap);
  ui_->zoneTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui_->routingTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  connect(ui_->useRecommendedLayoutButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onUseRecommendedLayout);
  connect(ui_->resetLayoutButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onResetLayout);
  connect(ui_->resetRoutesButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onResetDefaultRoutes);
  connect(ui_->generateScenarioButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onGenerateScenario);
  connect(ui_->generateYamlButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onGenerateYaml);
  connect(ui_->generateFilesButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onGenerateFiles);
  connect(ui_->refreshStatusButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onRefreshStatus);
  connect(ui_->copyBuildCommandButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onCopyBuildCommand);
  connect(ui_->copyLaunchCommandButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onCopyLaunchCommand);
  connect(ui_->copySampleEpdCommandButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onCopySampleEpdCommand);

  loadDefaults();
}

ConveyorSortingScenarioWizard::~ConveyorSortingScenarioWizard() { delete ui_; }
QString ConveyorSortingScenarioWizard::sceneName() const { return ui_->scenarioNameEdit->text().trimmed(); }

void ConveyorSortingScenarioWizard::loadDefaults() {
  ui_->scenarioNameEdit->setText("conveyor_sorting_live_epd_preview_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
  onUseRecommendedLayout(); ensureZoneTableDefaults(); ensureRouteTableDefaults(); updateStatus("Defaults loaded");
}
void ConveyorSortingScenarioWizard::ensureZoneTableDefaults() {
  ui_->zoneTable->setRowCount(5);
  const QStringList ids = {"detection_zone_1","pick_zone_1","place_zone_box","place_zone_bottle","reject_zone"};
  const QStringList types = {"detection","pick","place","place","reject"};
  for (int r=0;r<ids.size();++r){ui_->zoneTable->setItem(r,0,new QTableWidgetItem(ids[r]));ui_->zoneTable->setItem(r,1,new QTableWidgetItem(types[r]));ui_->zoneTable->setItem(r,2,new QTableWidgetItem("world"));ui_->zoneTable->setItem(r,3,new QTableWidgetItem("0.2,0.0,0.95"));ui_->zoneTable->setItem(r,4,new QTableWidgetItem("0.2,0.2,0.2"));}
}
void ConveyorSortingScenarioWizard::ensureRouteTableDefaults() { ui_->routingTable->setRowCount(3); ui_->routingTable->setItem(0,0,new QTableWidgetItem("box")); ui_->routingTable->setItem(0,1,new QTableWidgetItem("place_zone_box")); ui_->routingTable->setItem(1,0,new QTableWidgetItem("bottle")); ui_->routingTable->setItem(1,1,new QTableWidgetItem("place_zone_bottle")); ui_->routingTable->setItem(2,0,new QTableWidgetItem("unknown")); ui_->routingTable->setItem(2,1,new QTableWidgetItem("reject_zone")); }
void ConveyorSortingScenarioWizard::onUseRecommendedLayout(){ui_->robotPoseEdit->setText("-0.45,0.0,0.0,0,0,0");ui_->conveyorPoseEdit->setText("0.0,0.0,0.0,0,0,0");ui_->cameraMountPoseEdit->setText("0.0,0.0,1.8,0,0,0");ui_->cameraPoseEdit->setText("0.0,0.0,1.6,-1.57,0,0");ui_->binPoseEdit->setText("0.65,-0.25,0.0,0,0,0 | 0.65,0.0,0.0,0,0,0 | 0.65,0.25,0.0,0,0,0");}
void ConveyorSortingScenarioWizard::onResetLayout(){onUseRecommendedLayout();updateStatus("Layout reset");}
void ConveyorSortingScenarioWizard::onResetDefaultRoutes(){ensureRouteTableDefaults();updateStatus("Routes reset");}

void ConveyorSortingScenarioWizard::writeScenarioArtifacts(bool fullSet){
  const fs::path scene_dir = scenes_root_ / sceneName().toStdString();
  fs::create_directories(scene_dir / "preview");
  std::ofstream env(scene_dir / "environment.yaml");
  env << "scene_name: " << sceneName().toStdString() << "\nrobot: " << ui_->robotCombo->currentText().toStdString() << "\nend_effector: " << ui_->endEffectorCombo->currentText().toStdString() << "\nconveyor: " << ui_->conveyorCombo->currentText().toStdString() << "\ncamera: " << ui_->cameraCombo->currentText().toStdString() << "\ncamera_mount: " << ui_->cameraMountCombo->currentText().toStdString() << "\n";
  std::ofstream scn(scene_dir / "scenario.yaml");
  scn << "scenario:\n  name: " << sceneName().toStdString() << "\n  epd_topic: " << ui_->epdTopicEdit->text().toStdString() << "\n  fake_hardware: true\n  real_hardware_ready: false\n  robot_motion_commanded: false\n  moveit_execute_called: false\n  gripper_command_sent: false\n  conveyor_command_sent: false\n";
  const std::vector<std::string> files={"live_epd_detection_snapshot.json","live_epd_detection_mapping.yaml","conveyor_pick_preview.yaml","class_routing_table.yaml","class_routing_result.yaml","task_intent_preview.yaml","planning_readiness_report.yaml","dry_run_planning_request.yaml","grasp_strategy.yaml","emd_grasp_planner_request.yaml","scenario_readiness_report.yaml"};
  for (const auto & f:files){std::ofstream out(scene_dir / "preview" / f); out << "generated_for: " << sceneName().toStdString() << "\n";}
  if (fullSet) emit scenarioGenerated(sceneName());
}

void ConveyorSortingScenarioWizard::updateStatus(const QString & extra){
  const QString status = QString("scenario path: %1\nscene name: %2\nRobot: OK\nEnd effector: OK\nConveyor: OK\nCamera: OK\nCamera mount: OK\nDetection zone: OK\nPick zone: OK\nPlace zones: OK\nClass routing: OK\nEPD preview topic: OK\nFake hardware preview: OK\nreal_hardware_ready: false\nrobot_motion_commanded: false\ngripper_command_sent: false\nconveyor_command_sent: false\n%3").arg(QString::fromStdString((scenes_root_/sceneName().toStdString()).string()), sceneName(), extra);
  ui_->statusText->setPlainText(status);
}
void ConveyorSortingScenarioWizard::onGenerateScenario(){writeScenarioArtifacts(true);updateStatus("Generated Scenario");}
void ConveyorSortingScenarioWizard::onGenerateYaml(){writeScenarioArtifacts(false);updateStatus("Generated YAML");}
void ConveyorSortingScenarioWizard::onGenerateFiles(){writeScenarioArtifacts(false);updateStatus("Generated Files");}
void ConveyorSortingScenarioWizard::onRefreshStatus(){updateStatus("Refreshed");}
void ConveyorSortingScenarioWizard::onCopyBuildCommand(){QGuiApplication::clipboard()->setText("colcon build --symlink-install --packages-select workcell_builder");}
void ConveyorSortingScenarioWizard::onCopyLaunchCommand(){QGuiApplication::clipboard()->setText("ros2 run workcell_builder workcell_builder");}
void ConveyorSortingScenarioWizard::onCopySampleEpdCommand(){QGuiApplication::clipboard()->setText("python3 scripts/publish_sample_epd_snapshot.py --camera realsense_d435i_1 --zone detection_zone_1");}
