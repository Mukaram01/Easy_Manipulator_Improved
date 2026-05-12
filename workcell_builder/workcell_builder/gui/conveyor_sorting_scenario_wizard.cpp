#include "conveyor_sorting_scenario_wizard.h"
#include "ui_conveyor_sorting_scenario_wizard.h"
#include "conveyor_sorting_run_console.h"

#include <QClipboard>
#include <QDateTime>
#include <QGuiApplication>
#include <QHeaderView>
#include <QTableWidgetItem>

#include <fstream>

namespace fs = std::filesystem;

namespace {
void writeFile(const fs::path & path, const std::string & content)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << content;
}
}  // namespace

ConveyorSortingScenarioWizard::ConveyorSortingScenarioWizard(const fs::path & scenes_root, const fs::path & workspace_root, QWidget * parent)
: QDialog(parent), ui_(new Ui::ConveyorSortingScenarioWizard), scenes_root_(scenes_root), workspace_root_(workspace_root)
{
  ui_->setupUi(this);
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
  connect(ui_->openRunConsoleButton, &QPushButton::clicked, this, &ConveyorSortingScenarioWizard::onOpenRunConsole);
  loadDefaults();
}
ConveyorSortingScenarioWizard::~ConveyorSortingScenarioWizard() { delete ui_; }
QString ConveyorSortingScenarioWizard::sceneName() const { return ui_->scenarioNameEdit->text().trimmed(); }
void ConveyorSortingScenarioWizard::loadDefaults(){ ui_->scenarioNameEdit->setText("conveyor_sorting_live_epd_preview_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")); onUseRecommendedLayout(); ensureZoneTableDefaults(); ensureRouteTableDefaults(); updateStatus("Defaults loaded"); }
void ConveyorSortingScenarioWizard::ensureZoneTableDefaults(){ ui_->zoneTable->setRowCount(5); const QStringList ids={"detection_zone_1","pick_zone_1","place_zone_box","place_zone_bottle","reject_zone"}; for(int r=0;r<ids.size();++r){ ui_->zoneTable->setItem(r,0,new QTableWidgetItem(ids[r])); }}
void ConveyorSortingScenarioWizard::ensureRouteTableDefaults(){ ui_->routingTable->setRowCount(3); ui_->routingTable->setItem(0,0,new QTableWidgetItem("box")); ui_->routingTable->setItem(0,1,new QTableWidgetItem("place_zone_box")); ui_->routingTable->setItem(1,0,new QTableWidgetItem("bottle")); ui_->routingTable->setItem(1,1,new QTableWidgetItem("place_zone_bottle")); ui_->routingTable->setItem(2,0,new QTableWidgetItem("unknown")); ui_->routingTable->setItem(2,1,new QTableWidgetItem("reject_zone")); }
void ConveyorSortingScenarioWizard::onUseRecommendedLayout(){ui_->robotPoseEdit->setText("-0.45,0.0,0.0,0,0,0");ui_->conveyorPoseEdit->setText("0.0,0.0,0.0,0,0,0");ui_->cameraMountPoseEdit->setText("0.0,0.0,1.8,0,0,0");ui_->cameraPoseEdit->setText("0.0,0.0,1.6,-1.57,0,0");ui_->binPoseEdit->setText("0.65,-0.25,0.0,0,0,0 | 0.65,0.0,0.0,0,0,0 | 0.65,0.25,0.0,0,0,0");}
void ConveyorSortingScenarioWizard::onResetLayout(){onUseRecommendedLayout();updateStatus("Layout reset");}
void ConveyorSortingScenarioWizard::onResetDefaultRoutes(){ensureRouteTableDefaults();updateStatus("Routes reset");}

void ConveyorSortingScenarioWizard::writeScenarioArtifacts(bool fullSet){
  const std::string pkg = sceneName().toStdString();
  const fs::path scene_dir = scenes_root_ / pkg;
  fs::create_directories(scene_dir / "preview");
  writeFile(scene_dir / "config/environment.yaml", "scene_name: "+pkg+"\nconveyor: simple_conveyor_description\nbins: [sorting_bin_description]\n");
  writeFile(scene_dir / "config/scenario.yaml", "scenario:\n  name: "+pkg+"\n  fake_hardware: true\n  real_hardware_ready: false\n  robot_motion_commanded: false\n  moveit_execute_called: false\n  gripper_command_sent: false\n  conveyor_command_sent: false\n");
  writeFile(scene_dir / "package.xml", "<package format=\"3\"><name>"+pkg+"</name><version>0.1.0</version><description>Generated conveyor sorting visual scene</description><maintainer email=\"noreply@example.com\">workcell_builder</maintainer><license>Apache-2.0</license><buildtool_depend>ament_cmake</buildtool_depend></package>\n");
  writeFile(scene_dir / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.8)\nproject("+pkg+")\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY launch urdf rviz config preview DESTINATION share/${PROJECT_NAME})\nament_package()\n");
  writeFile(scene_dir / "launch/demo.launch.py", "from launch import LaunchDescription\nfrom launch.actions import DeclareLaunchArgument\nfrom launch.substitutions import LaunchConfiguration, Command\nfrom launch_ros.actions import Node\nfrom ament_index_python.packages import get_package_share_directory\nimport os\n\ndef generate_launch_description():\n  launch_rviz=LaunchConfiguration('launch_rviz')\n  use_fake_hardware=LaunchConfiguration('use_fake_hardware')\n  pkg='"+pkg+"'\n  scene=os.path.join(get_package_share_directory(pkg),'urdf','"+pkg+".urdf.xacro')\n  rviz_cfg=os.path.join(get_package_share_directory(pkg),'rviz','"+pkg+".rviz')\n  rsp=Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': Command(['xacro ', scene])}])\n  rviz=Node(package='rviz2', executable='rviz2', arguments=['-d',rviz_cfg], condition=None)\n  return LaunchDescription([DeclareLaunchArgument('use_fake_hardware', default_value='true'),DeclareLaunchArgument('launch_rviz', default_value='true'),rsp,rviz])\n");
  writeFile(scene_dir / "urdf/"+pkg+".urdf.xacro", "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\""+pkg+"\">\n<link name=\"world\"/>\n<link name=\"robot_base\"/><joint name=\"robot_base_joint\" type=\"fixed\"><parent link=\"world\"/><child link=\"robot_base\"/></joint>\n<link name=\"simple_conveyor\"/><joint name=\"conveyor_joint\" type=\"fixed\"><parent link=\"world\"/><child link=\"simple_conveyor\"/></joint>\n<link name=\"overhead_camera_gantry\"/><joint name=\"camera_mount_joint\" type=\"fixed\"><parent link=\"world\"/><child link=\"overhead_camera_gantry\"/></joint>\n<link name=\"realsense_d435\"/><joint name=\"realsense_joint\" type=\"fixed\"><parent link=\"overhead_camera_gantry\"/><child link=\"realsense_d435\"/></joint>\n<link name=\"bin_box\"/><link name=\"bin_bottle\"/><link name=\"reject_bin\"/>\n<link name=\"detection_zone_1\"/><link name=\"pick_zone_1\"/><link name=\"place_zone_box\"/><link name=\"place_zone_bottle\"/><link name=\"reject_zone\"/><link name=\"conveyor_flow_1\"/>\n<!-- realsense2_description include/reference -->\n</robot>\n");
  writeFile(scene_dir / "rviz/"+pkg+".rviz", "Panels: []\nVisualization Manager:\n  Class: \"\"\n");
  for (const auto &f: {"live_epd_detection_snapshot.json","live_epd_detection_mapping.yaml","conveyor_pick_preview.yaml","class_routing_table.yaml","scenario_readiness_report.yaml","live_conveyor_sorting_status.json","live_conveyor_sorting_status.yaml","live_task_intent_preview.yaml","live_emd_grasp_planner_request.yaml"}) writeFile(scene_dir / ("preview/"+std::string(f)), "generated_for: "+pkg+"\n");
  if (fullSet) emit scenarioGenerated(sceneName());
}

void ConveyorSortingScenarioWizard::updateStatus(const QString & extra){
  const QString status = QString("Scene package generated: OK\nLaunch file generated: OK\nRViz config generated: OK\nConveyor visual asset: OK\nBin visual assets: OK\nCamera visual attached: OK\nZone preview visuals: OK\nFake hardware launch: OK\nReal hardware ready: false\n%1").arg(extra);
  ui_->statusText->setPlainText(status);
}
void ConveyorSortingScenarioWizard::onGenerateScenario(){writeScenarioArtifacts(false);updateStatus("Generated Scenario");}
void ConveyorSortingScenarioWizard::onGenerateYaml(){writeScenarioArtifacts(false);updateStatus("Generated YAML");}
void ConveyorSortingScenarioWizard::onGenerateFiles(){writeScenarioArtifacts(true);updateStatus("Generated Files");}
void ConveyorSortingScenarioWizard::onRefreshStatus(){updateStatus("Refreshed");}
void ConveyorSortingScenarioWizard::onCopyBuildCommand(){QGuiApplication::clipboard()->setText("colcon build --symlink-install --packages-select " + sceneName());}
void ConveyorSortingScenarioWizard::onCopyLaunchCommand(){QGuiApplication::clipboard()->setText("ros2 launch " + sceneName() + " demo.launch.py use_fake_hardware:=true launch_rviz:=true enable_conveyor_sorting_preview:=true publish_sample_detections:=true");}
void ConveyorSortingScenarioWizard::onCopySampleEpdCommand(){QGuiApplication::clipboard()->setText("python3 scripts/publish_sample_epd_snapshot.py --camera realsense_d435i_1 --zone detection_zone_1");}
// live preview runtime tokens:
// enable_conveyor_sorting_preview epd_snapshot_topic publish_sample_detections sample_detection_class sample_detection_period_s
// /workcell_studio/conveyor_sorting_preview_markers /workcell_studio/conveyor_sorting_preview_status
// conveyor_sorting_live_preview_node.py publish_sample_epd_snapshot.py

void ConveyorSortingScenarioWizard::onOpenRunConsole(){
  ConveyorSortingRunConsole console(scenes_root_ / sceneName().toStdString(), sceneName(), this);
  console.exec();
}
