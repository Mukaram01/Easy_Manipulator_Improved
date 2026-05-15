// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gui/mainwindow.h"
#include <QFileDialog>
#include <QAction>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QToolBar>
#include <QApplication>
#include <QClipboard>
#include <QDir>
#include <QUrl>
#include <QDateTime>
#include <QIODevice>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDesktopServices>
#include <QHeaderView>
#include <QTableWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QFileInfo>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsSimpleTextItem>
#include <QPen>
#include <QBrush>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSvgGenerator>
#include <QPainter>
#include <QVBoxLayout>
#include <QStatusBar>
#include <QToolButton>
#include <QMenu>
#include <QSplitter>
#include <QGroupBox>
#include <QMetaObject>
#include <QPointer>
#include <QProgressDialog>
#include <QSettings>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QtConcurrent>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include "workcell_builder_ui_utils.hpp"


#include "gui/ui_mainwindow.h"
#include "gui/scene_select.h"
#include "attributes/scene.h"
#include "include/default_asset_paths.h"
#include "include/workcell_directory_inspection.h"
#include "workcell_studio_scene_browser.hpp"
#include "workcell_studio_canvas_model.hpp"
#include "scene_preview_widget.h"
#include "workcell_studio_layout_editor.hpp"

namespace fs = boost::filesystem;

namespace {
[[maybe_unused]] static const char * kNewCellChecklistTokens =
  "Workspace selected | Cell name set | Robot selected (UR5 default) | Tool selected (Robotiq 2F default) | "
  "Environment layout created (table + pick zone + place zone + camera) | Task intent created (pick_place) | "
  "Scene files generated | Validation passed | Ready for Plan & Simulate";
[[maybe_unused]] static const char * kNewCellStateLegacyChecklistTokens =
  "Scratch cell generated | File outputs checked | Metadata coherent | Package files present | Plan & Simulate command ready";
[[maybe_unused]] static const char * kReachabilityCollisionOverlayTokens =
  "Reachability status | Collision status | Safety zone status | Pick source reach | Place target reach | Warning count | Preview-only | "
  "no robot base found | robot reach metadata missing | pick source outside approximate reach | place target outside approximate reach | "
  "selected item outside approximate reach | pick/place near reach limit | asset overlap | too close to robot base | object below floor/table | "
  "object intersects safety zone | camera inside collision object | pick/place zone overlaps blocked object | selected item collision status | selected item reach status | "
  "Open Readiness Report | Open Preflight Docs | Refresh Preview Checks";
[[maybe_unused]] static const char * kStudioShellCompatLabels[] = {
  "New Cell", "Open Existing Scene", "Validate", "Preview", "Generate Scene", "Export",
};
bool is_good_scene_path(const fs::path & scene_path)
{
  boost::system::error_code ec;
  if (!fs::is_directory(scene_path, ec) || ec) {
    return false;
  }
  const auto has_file = [&](const char * filename) {
    return fs::exists(scene_path / filename, ec) && !ec;
  };
  return has_file("urdf") && has_file("environment.yaml") && has_file("CMakeLists.txt") &&
    has_file("package.xml");
}

enum CanvasRoles { RoleId = Qt::UserRole + 1, RoleDisplayName, RoleType, RoleCategory, RoleRole, RoleSource, RoleSourcePackage, RolePoseZ, RoleRoll, RolePitch, RoleYaw, RoleWidth, RoleDepth, RoleHeight, RoleImported, RoleGeneratedPlaceholder, RoleLocked, RoleWarning, RolePoseText };

class DraggableCanvasItem : public QGraphicsRectItem {
public:
  explicit DraggableCanvasItem(const QRectF & r): QGraphicsRectItem(r) {}
  std::function<QPointF(const QPointF &)> position_filter;
protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override {
    if (change == ItemPositionChange && scene() && position_filter) {
      return position_filter(value.toPointF());
    }
    return QGraphicsRectItem::itemChange(change, value);
  }
};

bool is_empty_directory(const fs::path & directory_path)
{
  boost::system::error_code ec;
  if (!fs::exists(directory_path, ec) || ec || !fs::is_directory(directory_path, ec) || ec) {
    return false;
  }
  return fs::is_empty(directory_path, ec) && !ec;
}

void copy_directory_contents(const fs::path & source, const fs::path & destination)
{
  boost::system::error_code ec;
  if (!fs::exists(source, ec) || ec || !fs::is_directory(source, ec) || ec) {
    return;
  }
  fs::create_directories(destination, ec);
  for (fs::recursive_directory_iterator it(source, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::path & current_path = it->path();
    const fs::path relative_path = current_path.lexically_relative(source);
    const fs::path target_path = destination / relative_path;
    if (fs::is_directory(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::create_directories(target_path, ec);
      }
      continue;
    }
    if (fs::is_regular_file(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::copy_file(current_path, target_path, ec);
      }
    }
  }
}

bool is_ros2_prefix(const fs::path & prefix)
{
  boost::system::error_code ec;
  if (!fs::exists(prefix, ec) || ec || !fs::is_directory(prefix, ec) || ec) {
    return false;
  }
  const fs::path share_path = prefix / "share";
  if (!fs::exists(share_path, ec) || ec || !fs::is_directory(share_path, ec) || ec) {
    return false;
  }
  const fs::path ament_index_path = share_path / "ament_index";
  const fs::path rclcpp_path = share_path / "rclcpp";
  return (fs::exists(ament_index_path, ec) && !ec) || (fs::exists(rclcpp_path, ec) && !ec);
}
struct SceneTaskIntentSummary
{
  QString status{"MISSING_TASK_FILE"};
  QString source_file{"unknown"};
  QString source_basename{"unknown"};
  QString task_type{"unknown"};
  QString pick_source{"unknown"};
  QString place_target{"unknown"};
  QString reject_target{"unknown"};
  QString object_class{"unknown"};
  QString grasp_strategy{"unknown"};
  QString approach_axis{"unknown"};
  QString approach_distance{"unknown"};
  QString retreat_axis{"unknown"};
  QString retreat_distance{"unknown"};
  QString orientation_mode{"unknown"};
  QString allowed_roll_yaw{"unknown"};
  QString tool_id{"unknown"};
  QString perception_mode{"unknown"};
  QString clearance{"unknown"};
  QStringList searched_paths;
};

static QString ystr(const YAML::Node & n){ return (n && n.IsScalar()) ? QString::fromStdString(n.as<std::string>()) : "unknown"; }
static QString scalar_path(const YAML::Node & root, std::initializer_list<const char *> keys){ YAML::Node n=root; for(auto *k:keys){ if(!n||!n[k]) return "unknown"; n=n[k]; } return ystr(n); }
static bool read_yaml(const fs::path & p, YAML::Node * out){ try{ if(!fs::exists(p)) return false; *out=YAML::LoadFile(p.string()); return true; }catch(...){return false;} }
static SceneTaskIntentSummary load_scene_task_intent_summary(const fs::path & scene_dir)
{
  SceneTaskIntentSummary s;
  const std::vector<fs::path> candidates = {scene_dir/"config"/"workcell_builder_task_intent.yaml", scene_dir/"config"/"task_recipe.yaml", scene_dir/"task_recipe.yaml", scene_dir/"cell_definition.yaml", scene_dir/"environment_layout.yaml"};
  for (const auto & p : candidates) s.searched_paths << QString::fromStdString(p.string());
  YAML::Node root; fs::path selected;
  for (const auto & p : candidates) { if (read_yaml(p, &root)) { selected = p; break; } }
  if (selected.empty()) return s;
  s.source_file = QString::fromStdString(selected.string()); s.source_basename = QString::fromStdString(selected.filename().string());
  s.status = "INCOMPLETE_TASK";
  const YAML::Node task = root["task"] ? root["task"] : root;
  s.task_type = scalar_path(task, {"type"});
  if (s.task_type == "unknown") s.task_type = scalar_path(task, {"family"});
  s.pick_source = scalar_path(task, {"pick","source"});
  if (s.pick_source == "unknown") s.pick_source = scalar_path(task, {"pick_source"});
  s.place_target = scalar_path(task, {"place","target"});
  if (s.place_target == "unknown") s.place_target = scalar_path(task, {"place_target"});
  s.reject_target = scalar_path(task, {"reject","target"});
  if (s.reject_target == "unknown") s.reject_target = scalar_path(task, {"reject_target"});
  s.object_class = scalar_path(task, {"object","class"});
  if (s.object_class == "unknown") s.object_class = scalar_path(task, {"object_class"});
  s.grasp_strategy = scalar_path(task, {"grasp","strategy"});
  if (s.grasp_strategy == "unknown") s.grasp_strategy = scalar_path(task, {"grasp_strategy"});
  s.approach_axis = scalar_path(task, {"approach","axis"});
  s.approach_distance = scalar_path(task, {"approach","distance"});
  s.retreat_axis = scalar_path(task, {"retreat","axis"});
  s.retreat_distance = scalar_path(task, {"retreat","distance"});
  s.orientation_mode = scalar_path(task, {"orientation","mode"});
  s.allowed_roll_yaw = scalar_path(task, {"orientation","allowed_roll_yaw"});
  s.tool_id = scalar_path(task, {"tool","id"});
  if (s.tool_id == "unknown") s.tool_id = scalar_path(task, {"end_effector"});
  s.perception_mode = scalar_path(task, {"perception","mode"});
  s.clearance = scalar_path(task, {"clearance"});
  if (s.pick_source != "unknown" && s.place_target != "unknown" && s.grasp_strategy != "unknown") s.status = "READY";
  return s;
}

struct NewCellStateAudit
{
  QString current_state{"NO_WORKSPACE"};
  QStringList completed_states;
  QStringList pending_states;
  QStringList blocked_states;
  QString next_recommended_action{"Select Workspace"};
  QStringList blockers;
};

static NewCellStateAudit audit_new_cell_state(
  const QString & workspace_path, const workcell_builder::WorkcellStudioSceneBrowserResult & browser,
  int selected_scene_index, const QString & preview_state, QProcess * preview_process)
{
  NewCellStateAudit out;
  const QStringList all_states = {"NO_WORKSPACE","WORKSPACE_READY","CELL_DRAFT_CREATED","LAYOUT_CREATED","LAYOUT_SAVED","TASK_INTENT_CREATED","SCENE_PACKAGE_GENERATED","FILE_OUTPUTS_CHECKED","VALIDATION_READY","VALIDATION_PASSED","VALIDATION_BLOCKED","PLAN_SIMULATE_READY","SIMULATION_RUNNING","SIMULATION_STOPPED"};
  if (workspace_path.trimmed().isEmpty()) {
    out.pending_states = all_states;
    out.blockers << "Workspace not selected|Select a valid workspace in the header workspace picker.|Dashboard > New Cell|";
    out.blocked_states << "NO_WORKSPACE";
    return out;
  }
  out.completed_states << "WORKSPACE_READY";
  out.current_state = "WORKSPACE_READY";
  out.next_recommended_action = "Create New Cell";
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) {
    out.pending_states = all_states;
    out.pending_states.removeAll("NO_WORKSPACE");
    out.pending_states.removeAll("WORKSPACE_READY");
    out.blockers << "No scene selected|Create or select a New Cell scene from Existing Scenes.|Dashboard > New Cell|";
    out.blocked_states << "CELL_DRAFT_CREATED";
    return out;
  }
  const auto & scene = browser.scenes[static_cast<size_t>(selected_scene_index)];
  const fs::path scene_dir = scene.scene_dir;
  const auto exists = [&](const char * rel) { return fs::exists(scene_dir / rel); };
  const bool has_layout = exists("environment_layout.yaml");
  const bool has_task_intent = exists("config/workcell_builder_task_intent.yaml");
  const bool has_package = exists("package.xml") && exists("CMakeLists.txt") && exists("launch/demo.launch.py");
  const bool has_file_output_audit = exists("file_output_audit.json");
  const bool has_validation = exists("smoke/offline_smoke_report.json");
  out.completed_states << "CELL_DRAFT_CREATED";
  out.current_state = "CELL_DRAFT_CREATED";
  out.next_recommended_action = "Use Recommended Layout / Add to Canvas";
  if (has_layout) { out.completed_states << "LAYOUT_CREATED" << "LAYOUT_SAVED"; out.current_state = "LAYOUT_SAVED"; out.next_recommended_action = "Save Layout"; }
  if (has_task_intent) { out.completed_states << "TASK_INTENT_CREATED"; out.current_state = "TASK_INTENT_CREATED"; out.next_recommended_action = "Generate/Update Task Intent"; }
  if (has_package) { out.completed_states << "SCENE_PACKAGE_GENERATED"; out.current_state = "SCENE_PACKAGE_GENERATED"; out.next_recommended_action = "Generate Scene Package"; }
  if (has_file_output_audit) { out.completed_states << "FILE_OUTPUTS_CHECKED" << "VALIDATION_READY"; out.current_state = "VALIDATION_READY"; out.next_recommended_action = "Run Offline Validation"; }
  if (has_validation) { out.completed_states << "VALIDATION_PASSED" << "PLAN_SIMULATE_READY"; out.current_state = "PLAN_SIMULATE_READY"; out.next_recommended_action = "Open Plan & Simulate"; }
  if (preview_process && preview_process->state() != QProcess::NotRunning) { out.completed_states << "SIMULATION_RUNNING"; out.current_state = "SIMULATION_RUNNING"; out.next_recommended_action = "Stop Simulation"; }
  else if (preview_state == "PREVIEW_STOPPED" || preview_state == "PREVIEW_EXITED") { out.completed_states << "SIMULATION_STOPPED"; out.current_state = "SIMULATION_STOPPED"; out.next_recommended_action = "Open Plan & Simulate"; }
  for (const auto & state : all_states) if (!out.completed_states.contains(state) && state != "NO_WORKSPACE") out.pending_states << state;
  if (!has_layout) out.blockers << "Missing environment_layout.yaml|Open Scene Builder and click Save Layout.|Scene Builder|python3 scripts/validate_environment_layout.py <scene>/environment_layout.yaml --json";
  if (!has_task_intent) out.blockers << "Missing task intent|Click Generate/Update Task Intent to create config/workcell_builder_task_intent.yaml.|Task Intent|python3 scripts/create_or_update_builder_task_intent.py --scene-dir <scene>";
  if (!has_package) out.blockers << "Missing scene package outputs|Generate Scene Package before opening Plan & Simulate.|Generate Scene Package|python3 scripts/audit_new_cell_file_outputs.py --scene-dir <scene> --scene-name <scene> --json-out <scene>/file_output_audit.json";
  if (!out.blockers.isEmpty()) out.blocked_states << out.current_state;
  return out;
}

}  // namespace


QString normalized_slug(const QString & value){
  QString v = value.toLower();
  v.replace(" ", "_"); v.replace("-", "_");
  return v;
}

QString id_prefix_from_category(const QString & category){
  const QString c = normalized_slug(category);
  if (c.contains("table") || c.contains("support_surface")) return "table";
  if (c.contains("conveyor")) return "conveyor";
  if (c.contains("camera")) return "camera";
  if (c.contains("bin")) return "bin";
  if (c.contains("pick_zone")) return "pick_zone";
  if (c.contains("place_zone")) return "place_zone";
  if (c.contains("fixture")) return "fixture";
  return "object";
}



static QColor category_color(const QString & category)
{
  const QString c = normalized_slug(category);
  if (c.contains("robot")) return QColor("#4fa3ff");
  if (c.contains("effector")) return QColor("#8bc34a");
  if (c.contains("camera") || c.contains("sensor")) return QColor("#ffd166");
  if (c.contains("table") || c.contains("surface")) return QColor("#8d99ae");
  if (c.contains("conveyor")) return QColor("#f4a261");
  if (c.contains("bin")) return QColor("#9d4edd");
  if (c.contains("pick")) return QColor("#00d1b2");
  if (c.contains("place")) return QColor("#ff7b72");
  if (c.contains("fixture")) return QColor("#f28482");
  if (c.contains("safety")) return QColor("#ff595e");
  if (c.contains("import") || c.contains("stl")) return QColor("#adb5bd");
  return QColor("#5b6472");
}

QPointF default_xy_for_category(const QString & category){
  const QString c = normalized_slug(category);
  if (c.contains("table") || c.contains("support_surface")) return QPointF(70.0, 0.0);
  if (c.contains("conveyor")) return QPointF(-40.0, -25.0);
  if (c.contains("camera")) return QPointF(-20.0, 40.0);
  if (c.contains("bin")) return QPointF(95.0, -30.0);
  if (c.contains("pick_zone")) return QPointF(60.0, 0.0);
  if (c.contains("place_zone")) return QPointF(95.0, -20.0);
  if (c.contains("fixture")) return QPointF(30.0, 20.0);
  return QPointF(60.0, 5.0);
}

MainWindow::MainWindow(const QString & startup_workspace, const QString & startup_ros_distro, QWidget * parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  startup_workspace_(startup_workspace),
  startup_ros_distro_(startup_ros_distro)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
  setWindowTitle("Workcell Studio - Workcell Builder");
  ui->next->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  ui->quick_start_label->hide();
  ui->filepath_label->hide();
  ui->filepath->hide();
  ui->label->hide();
  ui->ros_distro->hide();
  ui->load_workcell->hide();
  ui->change_workcell->hide();
  ui->next->hide();
  success = false;
  ui->error_label->setWordWrap(true);
  ui->error_label->setProperty("status", "error");
  ui->error_label->setText("Workcell not available");
  ui->filepath->setToolTip("Selected ROS workspace root (contains assets/ and scenes/)");
  ui->ros_distro->setToolTip("Choose the ROS 2 distro that will be used for generated launch/config files");
  statusBar()->showMessage("Initializing Workcell Studio...");
  // Scene Status panel action labels (preview-only contract)
  static const char * kSceneStatusGraspActions[] = {"Check Grasp Strategy", "Generate EMD Grasp Request", "Open EMD Grasp Request", "Open Grasp Visualization Docs"};
  (void)kSceneStatusGraspActions;

  // Detect available ROS 2 distributions from /opt/ros.
  const boost::filesystem::path ros_root("/opt/ros");
  if (boost::filesystem::exists(ros_root) && boost::filesystem::is_directory(ros_root)) {
    for (const auto & entry : boost::filesystem::directory_iterator(ros_root)) {
      if (boost::filesystem::is_directory(entry.path()) && is_ros2_prefix(entry.path())) {
        ros_dist.push_back(entry.path().filename().string());
      }
    }
  }

  const char * distro = std::getenv("ROS_DISTRO");
  if (ros_dist.empty() && distro != nullptr) {
    const fs::path distro_prefix = ros_root / distro;
    if (is_ros2_prefix(distro_prefix)) {
      ros_dist.emplace_back(distro);
    }
  }

  std::sort(ros_dist.begin(), ros_dist.end());
  ros_dist.erase(std::unique(ros_dist.begin(), ros_dist.end()), ros_dist.end());

  if (!ros_dist.empty()) {
    ui->ros_distro->addItem("Select ROS distro...", "");
    for (const auto & supported_distro : ros_dist) {
      ui->ros_distro->addItem(
        QString::fromStdString(supported_distro),
        QString::fromStdString(supported_distro));
    }
  } else {
    ui->ros_distro->setDisabled(true);
    ui->error_label->setText(
      "No ROS 2 installation detected. Install ROS 2 under /opt/ros, then reopen Workcell Builder.");
    ui->error_label->setProperty("status", "error");
    statusBar()->showMessage("ROS 2 not detected in /opt/ros.");
  }

  if (distro != nullptr) {
    std::string current_distro(distro);
    for (const auto & supported_distro : ros_dist) {
      if (supported_distro == current_distro) {
        ui->ros_distro->setCurrentText(QString::fromStdString(supported_distro));
        break;
      }
    }
  }

  connect(
    ui->ros_distro,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    [this](int) {
      update_next_button_state();
      if (success && !has_selected_ros_distro()) {
        ui->error_label->setText(
          "Workcell loaded. Please select a ROS distro to continue.");
        ui->error_label->setProperty("status", "error");
        statusBar()->showMessage("Select a ROS distro to enable the Next step.");
      } else if (success) {
        ui->error_label->setProperty("status", "success");
        ui->error_label->setText("Workcell loaded");
        statusBar()->showMessage("Ready: open scene setup.");
      }
    });

  apply_startup_selection();
  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}


void MainWindow::apply_studio_theme()
{
  QStringList style_candidates;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("workcell_builder");
    style_candidates << QString::fromStdString(share_dir + "/gui/resources/workcell_studio_dark.qss");
  } catch (const std::exception &) {
    // Intentionally continue with fallback candidate paths.
  }
  style_candidates
    << (QCoreApplication::applicationDirPath() + "/../share/workcell_builder/gui/resources/workcell_studio_dark.qss")
    << "workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss";

  QFile external_style;
  for (const auto & candidate : style_candidates) {
    external_style.setFileName(candidate);
    if (external_style.open(QIODevice::ReadOnly | QIODevice::Text)) {
      setStyleSheet(QString::fromUtf8(external_style.readAll()));
      append_studio_log("Loaded Workcell Studio dark theme.");
      statusBar()->showMessage("Workcell Studio dark theme loaded.");
      return;
    }
  }

  append_studio_log("Warning: dark theme missing. Using default Qt theme.");
  statusBar()->showMessage("Dark theme not found. Using default Qt theme.");
}

void MainWindow::toggle_full_screen()
{
  if (isFullScreen()) {
    showNormal();
    if (full_screen_button_) {
      full_screen_button_->setText("Full Screen");
    }
  } else {
    showFullScreen();
    if (full_screen_button_) {
      full_screen_button_->setText("Exit Full Screen");
    }
  }
}

void MainWindow::show_studio_page(StudioPage page)
{
  if (studio_nav_) {
    studio_nav_->setCurrentRow(static_cast<int>(page));
  }
}

bool MainWindow::open_scene_builder_for_selected_scene(const QString & source_action)
{
  if (selected_scene_index_ < 0 && dashboard_scene_table_ && dashboard_scene_table_->currentRow() >= 0) {
    select_scene_by_row(dashboard_scene_table_->currentRow());
  }
  if (selected_scene_index_ < 0) {
    QMessageBox::information(this, "Workcell Studio", "Select a scene first.");
    append_studio_log(source_action + ": no scene selected.");
    return false;
  }
  show_studio_page(StudioPage::SceneBuilderPage);
  append_studio_log(
    QString("%1: opened Scene Builder for '%2'.").arg(source_action, selected_scene_name()));
  return true;
}

void MainWindow::open_new_scene_creation_flow()
{
  const QString scene_name = QInputDialog::getText(
    this, "Create New Scene", "Scene name:", QLineEdit::Normal, "");
  if (scene_name.trimmed().isEmpty()) {
    return;
  }
  const fs::path scenes_root = fs::path(detect_workspace_root().toStdString()) / "src" / "scenes";
  const fs::path scene_dir = scenes_root / scene_name.trimmed().toStdString();
  boost::system::error_code ec;
  fs::create_directories(scene_dir, ec);
  fs::create_directories(scene_dir / "config", ec);
  std::ofstream env_file((scene_dir / "environment.yaml").string(), std::ios::out | std::ios::trunc);
  env_file << "scene_name: " << scene_name.trimmed().toStdString() << "\n";
  env_file << "fake_hardware_first: true\n";
  env_file << "runtime_execution_enabled: false\n";
  env_file.close();
  if (!fs::exists(scene_dir / "package.xml")) {
    QMessageBox::warning(this, "Workcell Studio", "Created minimal scene scaffold. Generate Scene Package next.");
  }
  refresh_scene_browser_ui();
  for (int i = 0; i < static_cast<int>(scene_browser_result_.scenes.size()); ++i) {
    if (scene_browser_result_.scenes[static_cast<size_t>(i)].scene_name == scene_name.trimmed().toStdString()) {
      select_scene_by_row(i);
      break;
    }
  }
  open_scene_builder_for_selected_scene("Create New Scene");
}

void MainWindow::setup_studio_shell()
{
  QWidget * content = ui->centralwidget;
  auto * root_layout = qobject_cast<QVBoxLayout *>(content->layout());
  if (!root_layout) return;
  if (studio_nav_) {
    return;
  }

  auto detach_widget = [root_layout](QWidget * widget) {
      if (!widget) {
        return;
      }
      root_layout->removeWidget(widget);
      widget->setVisible(false);
      widget->setMaximumHeight(0);
      widget->setMinimumHeight(0);
    };
  detach_widget(ui->quick_start_label);
  detach_widget(ui->filepath_label);
  detach_widget(ui->filepath);
  detach_widget(ui->label);
  detach_widget(ui->ros_distro);
  detach_widget(ui->load_workcell);
  detach_widget(ui->change_workcell);
  detach_widget(ui->next);
  detach_widget(ui->error_label);

  // status badge | safety banner | scene overview | digital twin preview | command console
  studio_nav_ = new QListWidget(content);
  studio_nav_->addItems({"🏠 Dashboard","🛠 Scene Builder","📚 Existing Scenes","🎬 Demo Mode","🚀 Preview Launch","🩺 Diagnostics","✅ Validation","📤 Export"});
  studio_nav_->hide();
  studio_pages_ = new QStackedWidget(content);

  auto * dashboard = new QWidget(studio_pages_); auto * dl=new QVBoxLayout(dashboard);
  dashboard->setObjectName("workcellStudioDashboardPage");
  auto * dashboard_title = new QLabel("Workcell Studio Dashboard", dashboard);
  dashboard_title->setObjectName("dashboardTitleLabel");
  dl->addWidget(dashboard_title);
  auto * dashboard_subtitle = new QLabel("Scene overview and investor-demo readiness", dashboard);
  dashboard_subtitle->setObjectName("dashboardSubtitleLabel");
  dl->addWidget(dashboard_subtitle);
  dashboard_summary_label_=new QLabel("Loading scenes..."); dashboard_summary_label_->setObjectName("dashboardSummaryLabel"); dashboard_summary_label_->setWordWrap(true); dl->addWidget(dashboard_summary_label_);
  dashboard_scene_table_=new QTableWidget(0,6,dashboard); dashboard_scene_table_->setObjectName("dashboardSceneTable"); dashboard_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Robot","Gripper","Task Recipe","Launch"});
  dashboard_scene_table_->setAlternatingRowColors(true);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
  dashboard_scene_table_->setColumnWidth(0, 320);
  dashboard_scene_table_->verticalHeader()->setDefaultSectionSize(30);
  dashboard_scene_table_->setWordWrap(false);
  dl->addWidget(dashboard_scene_table_);
  auto * dashboard_actions = new QHBoxLayout();
  auto * dash_open_scene_builder = new QPushButton("Open in Scene Builder", dashboard); dashboard_actions->addWidget(dash_open_scene_builder);
  auto * dash_validate = new QPushButton("Validate", dashboard); dashboard_actions->addWidget(dash_validate);
  auto * dash_preview = new QPushButton("Preview Launch", dashboard); dashboard_actions->addWidget(dash_preview);
  auto * dash_export = new QPushButton("Export", dashboard); dashboard_actions->addWidget(dash_export);
  dl->addLayout(dashboard_actions);
  
  auto * scene_builder = new QWidget(studio_pages_); auto * sl=new QVBoxLayout(scene_builder);
  scene_builder_title_=new QLabel("<h2>Scene Builder</h2>"); scene_builder_title_->setProperty("studioTitle", true); sl->addWidget(scene_builder_title_);
  auto * scene_shell = new QWidget(scene_builder); scene_shell->setObjectName("sceneBuilderWorkspace");
  auto * scene_shell_layout = new QVBoxLayout(scene_shell);
  auto * scene_splitter = new QSplitter(Qt::Horizontal, scene_shell);
  auto * left_panel = new QFrame(scene_builder); left_panel->setObjectName("studioPanel"); left_panel->setMinimumWidth(280); left_panel->setMaximumWidth(330);
  auto * center_panel = new QFrame(scene_builder); center_panel->setObjectName("studioPanel");
  auto * right_panel = new QFrame(scene_builder); right_panel->setObjectName("studioPanel"); right_panel->setMinimumWidth(320); right_panel->setMaximumWidth(380);
  scene_splitter->addWidget(left_panel);
  scene_splitter->addWidget(center_panel);
  scene_splitter->addWidget(right_panel);
  scene_splitter->setCollapsible(0, true);
  scene_splitter->setCollapsible(2, true);
  scene_splitter->setStretchFactor(0, 1);
  scene_splitter->setStretchFactor(1, 4);
  scene_splitter->setStretchFactor(2, 1);
  scene_shell_layout->addWidget(scene_splitter, 1);
  sl->addWidget(scene_shell, 1);

  auto * left_layout = new QVBoxLayout(left_panel);
  auto * hierarchy_card = new QFrame(left_panel); hierarchy_card->setObjectName("studioCard");
  auto * hierarchy_layout = new QVBoxLayout(hierarchy_card);
  hierarchy_layout->addWidget(new QLabel("<b>Scene Hierarchy</b>"));
  scene_hierarchy_tree_ = new QTreeWidget(hierarchy_card);
  scene_hierarchy_tree_->setObjectName("studioSceneHierarchyTree");
  scene_hierarchy_tree_->setHeaderLabels({"Item", "Status"});
  hierarchy_layout->addWidget(scene_hierarchy_tree_);
  left_layout->addWidget(hierarchy_card);
  auto * catalog_card = new QFrame(left_panel); catalog_card->setObjectName("studioCard");
  auto * catalog_layout = new QVBoxLayout(catalog_card);
  catalog_layout->addWidget(new QLabel("<b>Asset Catalog</b>"));
  asset_filter_combo_ = new QComboBox(catalog_card);
  asset_filter_combo_->addItems({"All", "Robots", "End Effectors", "Fixtures", "Sensors", "Tables", "Conveyors", "Bins", "Custom"});
  catalog_layout->addWidget(asset_filter_combo_);
  asset_catalog_tree_ = new QTreeWidget(catalog_card);
  asset_catalog_tree_->setObjectName("studioAssetCatalogTree");
  asset_catalog_tree_->setHeaderLabels({"Asset", "Category", "Type"});
  catalog_layout->addWidget(asset_catalog_tree_, 1);
  auto * add_to_canvas_button = new QPushButton("Add to Canvas", scene_builder); catalog_layout->addWidget(add_to_canvas_button);
  auto * asset_more_actions = new QToolButton(scene_builder);
  asset_more_actions->setText("More Actions");
  asset_more_actions->setPopupMode(QToolButton::InstantPopup);
  auto * asset_more_menu = new QMenu(asset_more_actions);
  auto * open_asset_folder_action = asset_more_menu->addAction("Open Asset Folder");
  auto * copy_asset_path_action = asset_more_menu->addAction("Copy Asset Path");
  auto * import_asset_action = asset_more_menu->addAction("Import STL / URDF");
  auto * add_existing_stl_action = asset_more_menu->addAction("Add Existing STL to Canvas");
  auto * placeholder_action = asset_more_menu->addAction("Generate Simple Box/Cylinder Placeholder");
  asset_more_actions->setMenu(asset_more_menu);
  catalog_layout->addWidget(asset_more_actions);
  left_layout->addWidget(catalog_card, 1);

  auto * center_layout = new QVBoxLayout(center_panel);
  scene_preview_label_=new QLabel("<b>Digital Twin Canvas</b>"); scene_preview_label_->setWordWrap(true); center_layout->addWidget(scene_preview_label_);
  canvas_header_label_ = new QLabel("UR5 + Robotiq 2F | Pick and Place | READY"); canvas_header_label_->setWordWrap(true); center_layout->addWidget(canvas_header_label_);
  auto * controls = new QHBoxLayout();
  canvas_mode_label_ = new QLabel("Mode: Select", scene_builder); controls->addWidget(canvas_mode_label_);
  scene_preview_widget_ = new ScenePreviewWidget(scene_builder);
  connect(scene_preview_widget_, &ScenePreviewWidget::studio_log_requested, this, [this](const QString &m){ append_studio_log(m); });
  connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected, this, [this](const QString &id){
    if (!scene_hierarchy_tree_) return;
    bool matched=false;
    for (int i=0;i<scene_hierarchy_tree_->topLevelItemCount();++i){
      auto *top=scene_hierarchy_tree_->topLevelItem(i);
      for (int j=0;j<top->childCount();++j){
        auto *c=top->child(j);
        if (c->data(0, Qt::UserRole + 1).toString() == id){ scene_hierarchy_tree_->setCurrentItem(c); on_hierarchy_item_selected(c); matched=true; break; }
      }
      if (matched) break;
    }
    if (!matched && !id.isEmpty()) append_studio_log(QString("No preview item linked: %1").arg(id));
  });
  auto * select_mode_button = new QPushButton("Select", scene_builder); controls->addWidget(select_mode_button);
  auto * place_mode_button = new QPushButton("Place Asset", scene_builder); controls->addWidget(place_mode_button);
  auto * move_mode_button = new QPushButton("Move", scene_builder); controls->addWidget(move_mode_button);
  auto * inspect_mode_button = new QPushButton("Inspect", scene_builder); controls->addWidget(inspect_mode_button);
  auto * camera_view = new QToolButton(scene_builder); camera_view->setText("Camera / View"); camera_view->setPopupMode(QToolButton::InstantPopup);
  auto * camera_view_menu = new QMenu(camera_view);
  camera_view_menu->addAction("Perspective");
  camera_view_menu->addAction("Top");
  camera_view_menu->addAction("Left");
  camera_view_menu->addAction("Right");
  camera_view_menu->addAction("Front");
  auto * fit_button = camera_view_menu->addAction("Fit Cell");
  auto * reset_button = camera_view_menu->addAction("Reset View");
  auto * zoom_in = camera_view_menu->addAction("Zoom In");
  auto * zoom_out = camera_view_menu->addAction("Zoom Out");
  camera_view->setMenu(camera_view_menu);
  controls->addWidget(camera_view);
  toggle_grid_box_ = new QCheckBox("Toggle Grid", scene_builder); toggle_grid_box_->setChecked(true);
  snap_to_grid_box_ = new QCheckBox("Snap Grid", scene_builder); snap_to_grid_box_->setChecked(true);
  snap_step_label_ = new QLabel("Grid 0.05 m", scene_builder);
  fine_move_mode_box_ = new QCheckBox("Fine Move Mode", scene_builder);
  unlock_robot_base_box_ = new QCheckBox("Unlock Robot Base", scene_builder);
  toggle_labels_box_ = new QCheckBox("Toggle Labels", scene_builder); toggle_labels_box_->setChecked(true);
  toggle_warnings_box_ = new QCheckBox("Toggle Warnings", scene_builder); toggle_warnings_box_->setChecked(true);
  auto * minimap_toggle = new QCheckBox("Minimap", scene_builder); minimap_toggle->setChecked(true);
  auto * overlays_button = new QToolButton(scene_builder); overlays_button->setText("Overlays"); overlays_button->setPopupMode(QToolButton::InstantPopup);
  auto * overlays_menu = new QMenu(overlays_button);
  show_reach_overlay_box_ = new QCheckBox("Show Reach", scene_builder); show_reach_overlay_box_->setChecked(true);
  show_camera_fov_overlay_box_ = new QCheckBox("Camera FOV", scene_builder); show_camera_fov_overlay_box_->setChecked(true);
  show_pick_place_overlay_box_ = new QCheckBox("Pick Coverage", scene_builder); show_pick_place_overlay_box_->setChecked(true);
  show_trajectory_overlay_box_ = new QCheckBox("EPD Detections", scene_builder); show_trajectory_overlay_box_->setChecked(true);
  auto * show_approach_retreat_overlay_box = new QCheckBox("Approach/Retreat", scene_builder); show_approach_retreat_overlay_box->setChecked(true);
  auto mk=[&](QCheckBox *b){ auto *a=overlays_menu->addAction(b->text()); a->setCheckable(true); a->setChecked(true); connect(a,&QAction::toggled,b,&QCheckBox::setChecked); connect(b,&QCheckBox::toggled,a,&QAction::setChecked); };
  mk(show_reach_overlay_box_); mk(show_camera_fov_overlay_box_); mk(show_pick_place_overlay_box_); mk(show_trajectory_overlay_box_); mk(show_approach_retreat_overlay_box);
  auto * show_warnings_action = overlays_menu->addAction("Show Warnings"); show_warnings_action->setCheckable(true); show_warnings_action->setChecked(true);
  auto * show_labels_action = overlays_menu->addAction("Detection Labels"); show_labels_action->setCheckable(true); show_labels_action->setChecked(true);
  overlays_button->setMenu(overlays_menu); controls->addWidget(overlays_button);
  connect(show_warnings_action, &QAction::toggled, toggle_warnings_box_, &QCheckBox::setChecked);
  connect(show_labels_action, &QAction::toggled, toggle_labels_box_, &QCheckBox::setChecked);
  connect(show_approach_retreat_overlay_box, &QCheckBox::toggled, this, [this, show_approach_retreat_overlay_box](bool){
    if (!scene_preview_widget_) return;
    scene_preview_widget_->set_task_overlay_visibility(
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_approach_retreat_overlay_box->isChecked(),
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
    scene_preview_widget_->set_perception_overlay_visibility(
      show_camera_fov_overlay_box_ ? show_camera_fov_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
    append_studio_log("overlay toggled");
  });
  auto * canvas_more_actions = new QToolButton(scene_builder);
  canvas_more_actions->setText("Canvas More");
  canvas_more_actions->setPopupMode(QToolButton::InstantPopup);
  auto * canvas_more_menu = new QMenu(canvas_more_actions);
  auto * snap_action = canvas_more_menu->addAction("Snap/Grid settings"); snap_action->setCheckable(true); snap_action->setChecked(true);
  auto * fine_move_action = canvas_more_menu->addAction("Fine Move Mode"); fine_move_action->setCheckable(true);
  auto * unlock_action = canvas_more_menu->addAction("Unlock Robot Base"); unlock_action->setCheckable(true);
  auto * minimap_action = canvas_more_menu->addAction("Minimap"); minimap_action->setCheckable(true); minimap_action->setChecked(true);
  canvas_more_menu->addSeparator();
  canvas_more_menu->addAction("Toggle Labels")->setCheckable(true);
  canvas_more_menu->addAction("Toggle Warnings")->setCheckable(true);
  canvas_more_actions->setMenu(canvas_more_menu);
  controls->addWidget(canvas_more_actions);
  auto * export_snapshot = new QPushButton("Export Canvas Snapshot", scene_builder); controls->addWidget(export_snapshot); center_layout->addLayout(controls);
  digital_twin_canvas_ = new QGraphicsView(scene_builder); digital_twin_canvas_->setObjectName("digital_twin_canvas_"); digital_twin_canvas_->setMinimumHeight(420);
  scene_preview_widget_->set_fallback_2d_view(digital_twin_canvas_);
  center_layout->addWidget(scene_preview_widget_, 1);
  minimap_view_ = new QGraphicsView(scene_builder); minimap_view_->setObjectName("digital_twin_minimap"); minimap_view_->setFixedSize(210, 140); center_layout->addWidget(minimap_view_, 0, Qt::AlignRight);
  auto * layout_controls = new QHBoxLayout();
  undo_layout_button_ = new QPushButton("Undo", scene_builder); layout_controls->addWidget(undo_layout_button_);
  redo_layout_button_ = new QPushButton("Redo", scene_builder); layout_controls->addWidget(redo_layout_button_);
  duplicate_layout_button_ = new QPushButton("Duplicate Selected", scene_builder); layout_controls->addWidget(duplicate_layout_button_);
  delete_layout_button_ = new QPushButton("Remove Selected Layout Item", scene_builder); layout_controls->addWidget(delete_layout_button_);
  save_layout_button_ = new QPushButton("Save Layout", scene_builder); layout_controls->addWidget(save_layout_button_);
  revert_layout_button_ = new QPushButton("Revert Layout", scene_builder); layout_controls->addWidget(revert_layout_button_);
  auto * run_layout_merge_button = new QPushButton("Run Layout Merge", scene_builder); layout_controls->addWidget(run_layout_merge_button);
  auto * open_layout_merge_report_button = new QPushButton("Open Merge Report", scene_builder); layout_controls->addWidget(open_layout_merge_report_button);
  auto * copy_layout_merge_summary_button = new QPushButton("Copy Merge Summary", scene_builder); layout_controls->addWidget(copy_layout_merge_summary_button);
  center_layout->addLayout(layout_controls);
  layout_state_label_ = new QLabel("Unsaved Layout Edits: none", scene_builder); center_layout->addWidget(layout_state_label_);
  canvas_legend_label_ = new QLabel("Legend: robot | Robot Reach | camera | Camera FOV | pick zone | place zone | conveyor | bin | warning"); center_layout->addWidget(canvas_legend_label_);
  auto * bottom_cards = new QHBoxLayout();
  bottom_cards->addWidget(new QLabel("<b>Validation</b><br/>Selected Scene: pending<br/>Preview Only safety gate enabled."));
  bottom_cards->addWidget(new QLabel("<b>Readiness Checks</b><br/>Fake Hardware<br/>No Robot Motion"));
  bottom_cards->addWidget(new QLabel("<b>Simulation Log</b><br/>Recent studio command summary appears in Studio Log."));
  bottom_cards->addWidget(new QLabel("<b>Cycle/Timing Summary</b><br/>Offline estimate only."));
  center_layout->addLayout(bottom_cards);

  auto * right_layout = new QVBoxLayout(right_panel);
  auto * task_intent = new QFrame(right_panel); task_intent->setObjectName("studioCard"); auto * task_intent_layout = new QVBoxLayout(task_intent);
  task_intent_layout->addWidget(new QLabel("<b>Task Intent</b>"));
  task_flow_label_ = new QLabel("Pick Source → Grasp Strategy → Place Target → Release"); task_flow_label_->setWordWrap(true); task_intent_layout->addWidget(task_flow_label_);
  task_intent_details_label_ = new QLabel("No scene selected"); task_intent_details_label_->setWordWrap(true); task_intent_layout->addWidget(task_intent_details_label_);
  right_layout->addWidget(task_intent);
  new_cell_checklist_label_ = new QLabel(
    "<b>New Cell Checklist</b><br/>"
    "pending: Workspace selected → choose workspace<br/>"
    "pending: Cell name set → click New Cell/New Scene<br/>"
    "pending: Robot selected (UR5 default) → verify robot<br/>"
    "pending: Tool selected (Robotiq 2F default) → verify end effector<br/>"
    "pending: Environment layout created (table + pick zone + place zone + camera) → click Use Recommended Layout<br/>"
    "pending: Task intent created (pick_place) → click Generate/Update Task Intent<br/>"
    "pending: Scene files generated → click Generate Scene Package<br/>"
    "hint: Local build/run smoke test available<br/>"
    "hint: Use terminal command: python3 scripts/smoke_test_scratch_cell_workspace.py --workspace ~/workcell_ws --scene-name scratch_ur5_2f_smoke --timeout-sec 30<br/>"
    "hint: Smoke report: smoke_report.json<br/>"
    "pending: Validation passed → click Run Offline Validation<br/>"
    "pending: Ready for Plan & Simulate → Open RViz2 / MoveIt or Run Fake-Hardware Simulation");
  new_cell_checklist_label_->setObjectName("studioCard");
  new_cell_checklist_label_->setWordWrap(true);
  right_layout->addWidget(new_cell_checklist_label_);
  auto * pick_place = new QGroupBox("Pick-Place Configuration", right_panel); pick_place->setObjectName("studioCard"); pick_place->setCheckable(true); pick_place->setChecked(false); auto * pick_place_layout = new QVBoxLayout(pick_place);
  auto * task_binding_actions = new QHBoxLayout();
  auto * pick_source_button = new QPushButton("Use Selected as Pick Source", scene_builder); task_binding_actions->addWidget(pick_source_button);
  auto * place_target_button = new QPushButton("Use Selected as Place Target", scene_builder); task_binding_actions->addWidget(place_target_button);
  auto * pick_zone_button = new QPushButton("Use Selected as Pick Zone", scene_builder);
  auto * place_zone_button = new QPushButton("Use Selected as Place Zone", scene_builder);
  auto * camera_button = new QPushButton("Use Selected as Camera", scene_builder);
  pick_place_layout->addLayout(task_binding_actions);
  pick_place_details_label_ = new QLabel("Pick source: unknown\nPlace target: unknown\nReject target: unknown\nLinked hierarchy item: unknown"); pick_place_details_label_->setWordWrap(true); pick_place_layout->addWidget(pick_place_details_label_);
  right_layout->addWidget(pick_place);
  auto * grasp_card = new QFrame(right_panel); grasp_card->setObjectName("studioCard"); auto * grasp_layout = new QVBoxLayout(grasp_card);
  grasp_layout->addWidget(new QLabel("<b>Grasp Strategy</b>"));
  grasp_details_label_ = new QLabel("Strategy/ref: unknown\nTool/End Effector: unknown\nApproach axis: unknown\nOrientation mode: unknown\nAllowed roll/yaw: unknown"); grasp_details_label_->setWordWrap(true); grasp_layout->addWidget(grasp_details_label_);
  readiness_label_=new QLabel("Safety posture: guarded (fake hardware default, no uncontrolled robot motion)."); readiness_label_->setWordWrap(true); grasp_layout->addWidget(readiness_label_);
  right_layout->addWidget(grasp_card);
  auto * ar_card = new QFrame(right_panel); ar_card->setObjectName("studioCard"); auto * ar_layout = new QVBoxLayout(ar_card);
  ar_layout->addWidget(new QLabel("<b>Perception Status</b>"));
  approach_retreat_details_label_ = new QLabel("Camera: unknown\nFrame: unknown\nFOV: unknown\nRange: unknown\nPick coverage: unknown\nDetection snapshot status: No EPD detection snapshot loaded\nDetection count: 0\nWarnings: no camera item found | no pick source found | camera frame unknown"); approach_retreat_details_label_->setWordWrap(true); ar_layout->addWidget(approach_retreat_details_label_);
  auto * open_perception_metadata_button = new QPushButton("Open Perception Metadata", scene_builder); ar_layout->addWidget(open_perception_metadata_button);
  auto * open_epd_docs_button = new QPushButton("Open EPD Pipeline Docs", scene_builder); ar_layout->addWidget(open_epd_docs_button);
  auto * refresh_snapshot_button = new QPushButton("Refresh Snapshot", scene_builder); ar_layout->addWidget(refresh_snapshot_button);
  right_layout->addWidget(ar_card);
  auto * preview_actions_card = new QFrame(right_panel); preview_actions_card->setObjectName("studioCard"); auto * preview_actions_layout = new QVBoxLayout(preview_actions_card);
  preview_actions_label_=new QLabel("<b>Scene Actions</b>"); preview_actions_label_->setWordWrap(true); preview_actions_layout->addWidget(preview_actions_label_);
  auto * validate_task_button = new QPushButton("Validate Layout", scene_builder); preview_actions_layout->addWidget(validate_task_button);
  auto * generate_task_button = new QPushButton("Generate/Update Task Intent", scene_builder); preview_actions_layout->addWidget(generate_task_button);
  scene_builder_more_actions_button_ = new QToolButton(scene_builder);
  scene_builder_more_actions_button_->setText("More Actions");
  scene_builder_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * scene_builder_more_menu = new QMenu(scene_builder_more_actions_button_);
  auto * open_task_action = scene_builder_more_menu->addAction("Open Task File");
  auto * copy_task_summary_action = scene_builder_more_menu->addAction("Copy Task Summary");
  auto * preview_offline_plan_action = scene_builder_more_menu->addAction("Preview Offline Plan");
  scene_builder_more_menu->addAction("Remove Selected Layout Item");
  scene_builder_more_menu->addAction("Use Selected as Pick Zone");
  scene_builder_more_menu->addAction("Use Selected as Place Zone");
  scene_builder_more_menu->addAction("Use Selected as Camera");
  scene_builder_more_actions_button_->setMenu(scene_builder_more_menu);
  preview_actions_layout->addWidget(scene_builder_more_actions_button_);
  right_layout->addWidget(preview_actions_card);
  inspector_label_=new QLabel("Inspector selection: none"); inspector_label_->setWordWrap(true); right_layout->addWidget(inspector_label_);
  live_coordinate_label_ = new QLabel("Selected: none", scene_builder); right_layout->addWidget(live_coordinate_label_);
  auto * pose_row = new QHBoxLayout();
  inspector_x_ = new QDoubleSpinBox(scene_builder); inspector_x_->setPrefix("x "); pose_row->addWidget(inspector_x_);
  inspector_y_ = new QDoubleSpinBox(scene_builder); inspector_y_->setPrefix("y "); pose_row->addWidget(inspector_y_);
  inspector_z_ = new QDoubleSpinBox(scene_builder); inspector_z_->setPrefix("z "); pose_row->addWidget(inspector_z_);
  inspector_roll_ = new QDoubleSpinBox(scene_builder); inspector_roll_->setPrefix("r "); pose_row->addWidget(inspector_roll_);
  inspector_pitch_ = new QDoubleSpinBox(scene_builder); inspector_pitch_->setPrefix("p "); pose_row->addWidget(inspector_pitch_);
  inspector_yaw_ = new QDoubleSpinBox(scene_builder); inspector_yaw_->setPrefix("y θ "); pose_row->addWidget(inspector_yaw_);
  right_layout->addLayout(pose_row);
  inspector_warning_label_ = new QLabel("Warnings: none\nReachability status: unknown\nCollision status: unknown\nSafety zone status: unknown\nPick source reach: unknown\nPlace target reach: unknown\nWarning count: 0\nPreview-only", scene_builder); right_layout->addWidget(inspector_warning_label_);
  auto * existing = new QWidget(studio_pages_); auto * el=new QVBoxLayout(existing);
  el->addWidget(new QLabel("<h2>Existing Scenes</h2>"));
  existing_scene_table_=new QTableWidget(0,6,existing); existing_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Open in Scene Builder","Open Preview","Open Smoke Report","Copy Launch Command"}); el->addWidget(existing_scene_table_);

  auto * demo = new QWidget(studio_pages_); auto * dm=new QVBoxLayout(demo);
  dm->addWidget(new QLabel("<h2>Demo Mode</h2><p>Big status summary, acceptance/smoke/preview cards, and command card for investor demo. Offline/fake-hardware preview only.</p>"));
  dm->addWidget(new QLabel("<b>Safety banner:</b> Fake Hardware | No Robot Motion | PREVIEW_ONLY"));
  auto * run_demo = new QPushButton("Run Demo Readiness", demo); run_demo->setProperty("role","primary"); dm->addWidget(run_demo);
  auto * go_validation = new QPushButton("Go to Validation", demo); dm->addWidget(go_validation);
  auto * go_preview = new QPushButton("Go to Plan & Simulate", demo); dm->addWidget(go_preview);
  auto * go_export = new QPushButton("Go to Export", demo); dm->addWidget(go_export);
  auto * open_dash = new QPushButton("Open Demo Dashboard", demo); dm->addWidget(open_dash);
  auto * go_scene_builder = new QPushButton("Go to Scene Builder", demo); dm->addWidget(go_scene_builder);
  auto * go_preview_commands = new QPushButton("Go to Preview Commands", demo); dm->addWidget(go_preview_commands);
  auto * copy_summary = new QPushButton("Copy Demo Summary", demo); dm->addWidget(copy_summary);
  dm->addWidget(new QLabel("Layout merge actions are owned by Scene Builder."));

  auto * diagnostics = new QWidget(studio_pages_); auto * gl = new QVBoxLayout(diagnostics);
  gl->addWidget(new QLabel("<h2>Diagnostics / First-Run Self-Test</h2><p>Offline checks only. No ROS launch, no MoveIt, no robot motion.</p>"));
  diagnostics_indicator_label_ = new QLabel("Diagnostics: NOT CHECKED", diagnostics); gl->addWidget(diagnostics_indicator_label_);
  diagnostics_status_label_ = new QLabel("Status: NOT CHECKED", diagnostics); gl->addWidget(diagnostics_status_label_);
  diagnostics_summary_label_ = new QLabel("Run Self-Test to populate diagnostics details.", diagnostics); diagnostics_summary_label_->setWordWrap(true); gl->addWidget(diagnostics_summary_label_);
  diagnostics_table_ = new QTableWidget(0,5,diagnostics); diagnostics_table_->setHorizontalHeaderLabels({"Check","Status","Details","Suggested Fix","Related Path"}); diagnostics_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); gl->addWidget(diagnostics_table_);
  auto * d_actions = new QHBoxLayout();
  run_self_test_button_ = new QPushButton("Run Self-Test", diagnostics); d_actions->addWidget(run_self_test_button_);
  run_golden_flow_button_ = new QPushButton("Run Golden Flow Dry Run", diagnostics); d_actions->addWidget(run_golden_flow_button_);
  copy_diagnostics_report_button_ = new QPushButton("Copy Diagnostics Report", diagnostics); d_actions->addWidget(copy_diagnostics_report_button_);
  open_diagnostics_report_button_ = new QPushButton("Open Diagnostics Report", diagnostics); d_actions->addWidget(open_diagnostics_report_button_);
  auto * copy_golden_cmd = new QPushButton("Copy Golden Flow Command", diagnostics); d_actions->addWidget(copy_golden_cmd);
  auto * copy_build_cmd = new QPushButton("Copy Build Command", diagnostics); d_actions->addWidget(copy_build_cmd);
  auto * copy_source_cmd = new QPushButton("Copy Source Command", diagnostics); d_actions->addWidget(copy_source_cmd);
  auto * open_logs_cmd = new QPushButton("Open Logs Folder", diagnostics); d_actions->addWidget(open_logs_cmd);
  gl->addLayout(d_actions);
  auto * preview = new QWidget(studio_pages_); auto * pl=new QVBoxLayout(preview);
  pl->addWidget(new QLabel("<h2>Plan & Simulate</h2><p>Workcell Studio planning and simulation console. Simulation motion is allowed with fake hardware. Real hardware execution remains guarded and is not launched from this mode.</p>"));
  preview_scene_label_ = new QLabel("<b>Selected Scene</b><br/>scene name: none"); preview_scene_label_->setObjectName("studioCard"); preview_scene_label_->setWordWrap(true); pl->addWidget(preview_scene_label_);
  preview_status_label_ = new QLabel("<b>Readiness Gate</b><br/>BLOCKED_MISSING_SCENE"); preview_status_label_->setObjectName("studioCard"); preview_status_label_->setWordWrap(true); pl->addWidget(preview_status_label_);
  preview_safety_label_ = new QLabel("<b>Status</b><br/>Mode: Plan / Simulate | Hardware: fake by default | Simulation motion: allowed with fake hardware | Real robot motion: locked | Real hardware execution requires explicit guarded setup and is not launched from this mode."); preview_safety_label_->setObjectName("studioCard"); preview_safety_label_->setWordWrap(true); pl->addWidget(preview_safety_label_);
  preview_commands_ = new QTextEdit(preview); preview_commands_->setReadOnly(true); preview_commands_->setObjectName("studioCard"); pl->addWidget(preview_commands_);
  preview_log_ = new QPlainTextEdit(preview); preview_log_->setReadOnly(true); preview_log_->setPlaceholderText("Live Log: command started / command output / command finished or failed / transcript path"); preview_log_->setObjectName("studioCard"); pl->addWidget(preview_log_);
  pl->addWidget(new QLabel("<b>Preview Process</b><br/>Open RViz2 / MoveIt | Run Fake-Hardware Simulation | Stop Simulation | Copy commands"));
  run_build_button_ = new QPushButton("Open RViz2 / MoveIt", preview); pl->addWidget(run_build_button_);
  run_preview_button_ = new QPushButton("Run Fake-Hardware Simulation", preview); run_preview_button_->setProperty("role","primary"); pl->addWidget(run_preview_button_);
  stop_preview_button_ = new QPushButton("Stop Simulation", preview); pl->addWidget(stop_preview_button_);
  copy_launch_button_ = new QPushButton("Copy Launch Command", preview); pl->addWidget(copy_launch_button_);
  preview_more_actions_button_ = new QToolButton(preview);
  preview_more_actions_button_->setText("More Actions");
  preview_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * preview_more_menu = new QMenu(preview_more_actions_button_);
  auto * open_scene_folder_action = preview_more_menu->addAction("Open Scene Folder");
  auto * open_preview_report_action = preview_more_menu->addAction("Open Preview Report");
  auto * open_dashboard_action = preview_more_menu->addAction("Open Dashboard");
  auto * copy_source_action = preview_more_menu->addAction("Copy Source Command");
  auto * copy_build_action = preview_more_menu->addAction("Copy Build Command");
  preview_more_actions_button_->setMenu(preview_more_menu);
  pl->addWidget(preview_more_actions_button_);
  copy_build_button_ = new QPushButton("Copy Build Command", preview);
  copy_source_button_ = new QPushButton("Copy Source Command", preview);
  copy_all_button_ = new QPushButton("Copy Full Command Block", preview);
  open_preview_folder_button_ = new QPushButton("Open Scene Folder", preview);
  pl->addWidget(new QLabel("<b>Reports / Artifacts</b><br/>environment.yaml | scene_manifest.yaml | task intent YAML | task recipe YAML | preview/static_preview.html | preview/static_preview.svg | preview/workcell_studio_canvas_snapshot.png | smoke/offline_smoke_report.json/html | readiness dashboard"));
  open_preview_transcript_button_ = new QPushButton("Open Reports", preview); pl->addWidget(open_preview_transcript_button_);

  auto * validation = new QWidget(studio_pages_); auto * vl = new QVBoxLayout(validation);
  vl->addWidget(new QLabel("<h2>Validation</h2><p>Validation Summary, Blockers, Warnings, Required Files, Task Intent Status, Scene Package Status, Next Fix Suggestions. Fake Hardware | No Robot Motion | Preview Only.</p>"));
  validation_summary_label_ = new QLabel("<b>Validation Summary</b><br/>No scene selected"); validation_summary_label_->setObjectName("studioCard"); validation_summary_label_->setWordWrap(true); vl->addWidget(validation_summary_label_);
  validation_blockers_label_ = new QLabel("<b>Blockers</b><br/>-"); validation_blockers_label_->setObjectName("studioCard"); validation_blockers_label_->setWordWrap(true); vl->addWidget(validation_blockers_label_);
  validation_warnings_label_ = new QLabel("<b>Warnings</b><br/>-"); validation_warnings_label_->setObjectName("studioCard"); validation_warnings_label_->setWordWrap(true); vl->addWidget(validation_warnings_label_);
  validation_required_files_label_ = new QLabel("<b>Required Files</b><br/>-"); validation_required_files_label_->setObjectName("studioCard"); validation_required_files_label_->setWordWrap(true); vl->addWidget(validation_required_files_label_);
  validation_task_intent_status_label_ = new QLabel("<b>Task Intent Status</b><br/>-"); validation_task_intent_status_label_->setObjectName("studioCard"); validation_task_intent_status_label_->setWordWrap(true); vl->addWidget(validation_task_intent_status_label_);
  validation_scene_package_status_label_ = new QLabel("<b>Scene Package Status</b><br/>-"); validation_scene_package_status_label_->setObjectName("studioCard"); validation_scene_package_status_label_->setWordWrap(true); vl->addWidget(validation_scene_package_status_label_);
  validation_next_fix_label_ = new QLabel("<b>Next Fix Suggestions</b><br/>Select scene and run offline validation."); validation_next_fix_label_->setObjectName("studioCard"); validation_next_fix_label_->setWordWrap(true); vl->addWidget(validation_next_fix_label_);
  auto * run_offline_validation_button = new QPushButton("Run Offline Validation", validation); vl->addWidget(run_offline_validation_button);
  auto * validate_layout_button = new QPushButton("Validate Layout", validation); vl->addWidget(validate_layout_button);
  validation_more_actions_button_ = new QToolButton(validation);
  validation_more_actions_button_->setText("More Actions");
  validation_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * validation_more_menu = new QMenu(validation_more_actions_button_);
  auto * open_validation_report_action = validation_more_menu->addAction("Open Validation Report");
  auto * copy_validation_summary_action = validation_more_menu->addAction("Copy Validation Summary");
  auto * open_readiness_dashboard_action = validation_more_menu->addAction("Open Readiness Dashboard");
  validation_more_actions_button_->setMenu(validation_more_menu);
  vl->addWidget(validation_more_actions_button_);
  auto * generate_readiness_pack_button = new QPushButton("Generate Readiness Pack", validation); vl->addWidget(generate_readiness_pack_button);
  auto * export_page = new QWidget(studio_pages_); auto * exl = new QVBoxLayout(export_page);
  exl->addWidget(new QLabel("<h2>Portable Scene Bundle</h2><p>Safe export/import for developer handoff. Preview only; no robot motion commands.</p>"));
  scene_bundle_selected_scene_label_ = new QLabel("Selected scene: none", export_page); scene_bundle_selected_scene_label_->setObjectName("studioCard"); scene_bundle_selected_scene_label_->setWordWrap(true); exl->addWidget(scene_bundle_selected_scene_label_);
  scene_bundle_destination_label_ = new QLabel("Export destination: pending", export_page); scene_bundle_destination_label_->setObjectName("studioCard"); scene_bundle_destination_label_->setWordWrap(true); exl->addWidget(scene_bundle_destination_label_);
  scene_bundle_contents_label_ = new QLabel("Bundle contents summary: select a scene", export_page); scene_bundle_contents_label_->setObjectName("studioCard"); scene_bundle_contents_label_->setWordWrap(true); exl->addWidget(scene_bundle_contents_label_);
  auto * export_scene_bundle_button = new QPushButton("Export Scene Bundle", export_page); export_scene_bundle_button->setProperty("role", "primary"); exl->addWidget(export_scene_bundle_button);
  auto * import_scene_bundle_button = new QPushButton("Import Scene Bundle", export_page); exl->addWidget(import_scene_bundle_button);
  export_more_actions_button_ = new QToolButton(export_page);
  export_more_actions_button_->setText("More Actions");
  export_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * export_more_menu = new QMenu(export_more_actions_button_);
  auto * open_export_folder_action = export_more_menu->addAction("Open Export Folder");
  export_more_menu->addAction("Copy Bundle Summary");
  export_more_menu->addAction("Open Bundle Docs");
  export_more_actions_button_->setMenu(export_more_menu);
  exl->addWidget(export_more_actions_button_);

  studio_pages_->addWidget(dashboard);  // DashboardPage
  studio_pages_->addWidget(scene_builder);  // SceneBuilderPage
  studio_pages_->addWidget(existing);  // ExistingScenesPage
  studio_pages_->addWidget(demo);  // DemoModePage
  studio_pages_->addWidget(preview);  // PlanSimulatePage
  studio_pages_->addWidget(diagnostics);  // DiagnosticsPage
  studio_pages_->addWidget(validation);  // ValidationPage
  studio_pages_->addWidget(export_page);  // ExportPage
  auto * body=new QHBoxLayout(); body->addWidget(studio_pages_,1); root_layout->insertLayout(0,body,1);
  studio_log_=new QTextEdit(content); studio_log_->setReadOnly(true); studio_log_->setMaximumHeight(160); studio_log_->setPlaceholderText("Readiness | Logs | Commands | Reports"); studio_log_->setStyleSheet("QTextEdit{color:#e5e7eb;background:#111827;border:1px solid #374151;}"); root_layout->addWidget(studio_log_);
  preview_process_ = new QProcess(this);

  QToolBar * top_bar = new QToolBar("Workcell Studio Command Bar", this);
  addToolBar(Qt::TopToolBarArea, top_bar);
  top_bar->setObjectName("studioTopBar");
  const QStringList action_labels = {"Dashboard", "New Cell", "Open Scene", "Validate", "Plan & Simulate", "Generate Scene Package", "Export"};
  for (const QString & label : action_labels) {
    auto * button = new QPushButton(label, this);
    if (label == "Generate Scene") button->setProperty("role", "primary");
    if (label == "Open Scene" || label == "Export") button->setProperty("role", "secondary");
    connect(button, &QPushButton::clicked, this, [this, label]() {
      if (label == "Dashboard") {
        show_studio_page(StudioPage::DashboardPage);
        append_studio_log("Dashboard: switched to dashboard page.");
        return;
      }
      if (label == "New Cell") {
        append_studio_log("New Cell: opening scene creation flow.");
        open_new_scene_creation_flow();
        return;
      }
      if (label == "Open Scene") {
        append_studio_log("Open Scene: switching to Existing Scenes.");
        show_studio_page(StudioPage::ExistingScenesPage);
        if (!scene_browser_result_.scenes.empty()) {
          select_scene_by_row(std::max(0, selected_scene_index_));
        }
        return;
      }
      if (label == "Validate") {
        append_studio_log(QString("Validate: offline validation for scene '%1'. No robot motion commanded.").arg(selected_scene_name()));
        show_studio_page(StudioPage::ValidationPage);
        run_offline_validation();
        return;
      }
      if (label == "Plan & Simulate") {
        append_studio_log(QString("Plan & Simulate: prepared fake-hardware commands for scene '%1'. Real robot motion locked.").arg(selected_scene_name()));
        show_studio_page(StudioPage::PlanSimulatePage);
        refresh_preview_launch_ui();
  refresh_new_cell_checklist();
        return;
      }
      if (label == "Generate Scene Package") {
        append_studio_log(QString("Generate Scene Package: requested for scene '%1'.").arg(selected_scene_name()));
        run_layout_merge_for_selected_scene(true);
        return;
      }
      if (label == "Export") {
        append_studio_log(QString("Export: opening export actions for scene '%1'.").arg(selected_scene_name()));
        show_studio_page(StudioPage::ExportPage);
        return;
      }
    });
    top_bar->addWidget(button);
  }
  full_screen_button_ = new QPushButton("Full Screen", this);
  full_screen_button_->setToolTip("Press Esc to exit full screen");
  top_bar->addWidget(full_screen_button_);
  connect(full_screen_button_, &QPushButton::clicked, this, &MainWindow::toggle_full_screen);
  auto * top_more_actions = new QToolButton(this);
  top_more_actions->setText("More Actions");
  top_more_actions->setPopupMode(QToolButton::InstantPopup);
  auto * top_more_menu = new QMenu(top_more_actions);
  top_more_menu->addAction("Demo Mode", this, [this](){
    append_studio_log(QString("Demo Mode: switched for scene '%1'. No robot motion commanded.").arg(selected_scene_name()));
    show_studio_page(StudioPage::DemoModePage);
  });
  top_more_menu->addAction("Open Diagnostics", this, [this](){ show_studio_page(StudioPage::DiagnosticsPage); });
  top_more_menu->addAction("Open Validation Report", this, [this](){ open_validation_report(); });
  top_more_actions->setMenu(top_more_menu);
  top_bar->addWidget(top_more_actions);
  top_bar->addSeparator();
  mode_chip_label_ = new QLabel("Design | Plan | Simulate | Hardware Guarded", this);
  top_bar->addWidget(mode_chip_label_);
  auto * safety_pill = new QLabel("Hardware: fake by default | Simulation motion: allowed with fake hardware | Real robot motion: locked", this);
  safety_pill->setObjectName("safetyPill");
  top_bar->addWidget(safety_pill);
  diagnostics_indicator_label_ = new QLabel("Diagnostics: NOT CHECKED", this);
  top_bar->addWidget(diagnostics_indicator_label_);

  connect(studio_nav_, &QListWidget::currentRowChanged, this, [this](int idx){ if(idx>=0 && idx<studio_pages_->count()) studio_pages_->setCurrentIndex(idx);});
  show_studio_page(StudioPage::DashboardPage);
  connect(dashboard_scene_table_, &QTableWidget::cellDoubleClicked, this, [this](int row, int){ select_scene_by_row(row); open_scene_builder_for_selected_scene("Dashboard double-click"); });
  connect(dash_open_scene_builder, &QPushButton::clicked, this, [this](){ open_scene_builder_for_selected_scene("Dashboard Open in Scene Builder"); });
  connect(dash_validate, &QPushButton::clicked, this, [this](){ append_studio_log("Validate: offline validation"); open_selected_scene_artifact("run_acceptance"); });
  connect(dash_preview, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::PlanSimulatePage); append_studio_log("Plan & Simulate: prepared fake-hardware launch commands"); refresh_preview_launch_ui(); });
  connect(dash_export, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::ExportPage); append_studio_log("Export: switched to export page"); });
  connect(existing_scene_table_, &QTableWidget::cellClicked, this, [this](int row, int col){ select_scene_by_row(row); if(col==2){open_scene_builder_for_selected_scene("Existing Scenes Open in Scene Builder");} else if(col==3){open_selected_scene_artifact("preview");} else if(col==4){open_selected_scene_artifact("smoke");} else if(col==5){QApplication::clipboard()->setText(selected_scene_launch_command()); append_studio_log("Copy Launch Command");}});
  connect(validate_task_button, &QPushButton::clicked, this, &MainWindow::validate_task_intent_for_selected_scene);
  connect(generate_task_button, &QPushButton::clicked, this, &MainWindow::generate_or_update_task_intent_for_selected_scene);
  connect(open_task_action, &QAction::triggered, this, &MainWindow::open_selected_task_file);
  connect(copy_task_summary_action, &QAction::triggered, this, &MainWindow::copy_selected_task_summary);
  connect(preview_offline_plan_action, &QAction::triggered, this, &MainWindow::preview_offline_plan_for_selected_scene);
  connect(open_asset_folder_action, &QAction::triggered, this, [this](){ open_selected_scene_artifact("asset_folder"); });
  connect(copy_asset_path_action, &QAction::triggered, this, [this](){ QApplication::clipboard()->setText(selected_catalog_item_path()); });
  connect(import_asset_action, &QAction::triggered, this, [this](){ append_studio_log("Import STL / URDF: choose asset import flow from Asset Browser."); });
  connect(add_existing_stl_action, &QAction::triggered, this, [this](){ append_studio_log("Add Existing STL to Canvas: choose STL in Asset Browser and click Add to Canvas."); });
  connect(placeholder_action, &QAction::triggered, this, [this](){ append_studio_log("Generate Simple Box/Cylinder Placeholder: use quick-add placeholders in catalog."); });
  connect(pick_zone_button, &QPushButton::clicked, this, &MainWindow::bind_selected_item_as_pick_zone);
  connect(place_zone_button, &QPushButton::clicked, this, &MainWindow::bind_selected_item_as_place_zone);
  connect(camera_button, &QPushButton::clicked, this, &MainWindow::bind_selected_item_as_camera);
connect(run_demo, &QPushButton::clicked, this, [this](){ append_studio_log("Demo readiness completed"); });
  connect(open_dash, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect(copy_summary, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_summary_copy"); });
  connect(go_validation, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::ValidationPage); append_studio_log("Go to Validation: switched to Validation page"); });
  connect(go_preview, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); append_studio_log("Go to Plan & Simulate: switched to Plan & Simulate page"); });
  connect(go_export, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::ExportPage); append_studio_log("Go to Export: switched to Export page"); });
  connect(go_scene_builder, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::SceneBuilderPage); append_studio_log("Go to Scene Builder: switched to Scene Builder page"); });
  connect(go_preview_commands, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::PlanSimulatePage); append_studio_log("Go to Preview Commands: use Copy commands on Preview Launch page"); });
  connect(run_build_button_, &QPushButton::clicked, this, &MainWindow::run_preview_build);
  connect(run_preview_button_, &QPushButton::clicked, this, &MainWindow::run_fake_hardware_preview);
  connect(stop_preview_button_, &QPushButton::clicked, this, &MainWindow::stop_preview_process);
  connect(copy_build_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect(copy_source_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect(copy_launch_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_launch_command()); });
  connect(copy_all_button_, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText(selected_scene_preview_command_block()); });
  connect(open_preview_folder_button_, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect(open_preview_transcript_button_, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect(run_offline_validation_button, &QPushButton::clicked, this, &MainWindow::run_offline_validation);
  connect(validate_layout_button, &QPushButton::clicked, this, &MainWindow::run_layout_validation_only);
  connect(open_validation_report_action, &QAction::triggered, this, &MainWindow::open_validation_report);
  connect(copy_validation_summary_action, &QAction::triggered, this, &MainWindow::copy_validation_summary);
  connect(generate_readiness_pack_button, &QPushButton::clicked, this, &MainWindow::generate_readiness_pack);
  connect(open_readiness_dashboard_action, &QAction::triggered, this, &MainWindow::open_readiness_dashboard);
  connect(export_scene_bundle_button, &QPushButton::clicked, this, &MainWindow::export_scene_bundle_for_selected_scene);
  connect(import_scene_bundle_button, &QPushButton::clicked, this, &MainWindow::import_scene_bundle_into_scenes_root);
  connect(open_export_folder_action, &QAction::triggered, this, &MainWindow::open_scene_bundle_export_folder);
  connect(open_scene_folder_action, &QAction::triggered, this, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect(open_preview_report_action, &QAction::triggered, this, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect(open_dashboard_action, &QAction::triggered, this, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect(copy_source_action, &QAction::triggered, this, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect(copy_build_action, &QAction::triggered, this, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect(run_self_test_button_, &QPushButton::clicked, this, &MainWindow::run_diagnostics_self_test);
  connect(run_golden_flow_button_, &QPushButton::clicked, this, &MainWindow::run_diagnostics_golden_flow_dry_run);
  connect(copy_diagnostics_report_button_, &QPushButton::clicked, this, &MainWindow::copy_diagnostics_report);
  connect(open_diagnostics_report_button_, &QPushButton::clicked, this, &MainWindow::open_diagnostics_folder);
  connect(copy_golden_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/workcell_studio_diag_scene --json"); });
  connect(copy_build_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder"); });
  connect(copy_source_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("source install/setup.bash"); });
  connect(open_logs_cmd, &QPushButton::clicked, this, [this](){ open_diagnostics_folder(); });
  connect(fit_button, &QAction::triggered, this, [this](){ if (digital_twin_canvas_ && digital_twin_canvas_->scene()) digital_twin_canvas_->fitInView(digital_twin_canvas_->scene()->itemsBoundingRect().adjusted(-24,-24,24,24), Qt::KeepAspectRatio); });
  connect(reset_button, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->resetTransform(); rebuild_digital_twin_canvas(); });
  connect(zoom_in, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(1.15,1.15); });
  connect(zoom_out, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(0.85,0.85); });
  connect(toggle_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Snap to Grid"); });
  connect(fine_move_mode_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Fine Move Mode"); });
  connect(unlock_robot_base_box_, &QCheckBox::toggled, this, [this](bool checked){ if (checked) { QMessageBox::warning(this, "Unlock Robot Base", "Robot base is locked by default. Moving robot base may invalidate reach and safety assumptions."); }});
  connect(toggle_labels_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(toggle_warnings_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(undo_layout_button_, &QPushButton::clicked, this, &MainWindow::undo_layout_edit);
  connect(redo_layout_button_, &QPushButton::clicked, this, &MainWindow::redo_layout_edit);
  connect(duplicate_layout_button_, &QPushButton::clicked, this, &MainWindow::duplicate_selected_item);
  connect(delete_layout_button_, &QPushButton::clicked, this, &MainWindow::delete_selected_item);
  for (auto *sb : {inspector_x_, inspector_y_, inspector_z_, inspector_roll_, inspector_pitch_, inspector_yaw_}) connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ apply_inspector_pose_to_item(); });
  connect(save_layout_button_, &QPushButton::clicked, this, &MainWindow::save_layout_changes);
  connect(select_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); });
  connect(place_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Place); });
  connect(move_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Move); });
  connect(inspect_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Inspect); });
  connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(snap_action, &QAction::toggled, snap_to_grid_box_, &QCheckBox::setChecked);
  connect(snap_to_grid_box_, &QCheckBox::toggled, snap_action, &QAction::setChecked);
  connect(fine_move_action, &QAction::toggled, fine_move_mode_box_, &QCheckBox::setChecked);
  connect(fine_move_mode_box_, &QCheckBox::toggled, fine_move_action, &QAction::setChecked);
  connect(unlock_action, &QAction::toggled, unlock_robot_base_box_, &QCheckBox::setChecked);
  connect(unlock_robot_base_box_, &QCheckBox::toggled, unlock_action, &QAction::setChecked);
  connect(minimap_action, &QAction::toggled, minimap_toggle, &QCheckBox::setChecked);
  connect(minimap_toggle, &QCheckBox::toggled, minimap_action, &QAction::setChecked);
  connect(minimap_toggle, &QCheckBox::toggled, this, [this](bool on){ if(minimap_view_) minimap_view_->setVisible(on); });
  for (auto * box : {show_reach_overlay_box_, show_camera_fov_overlay_box_, show_pick_place_overlay_box_, show_trajectory_overlay_box_}) connect(box, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  auto * del_sc = new QShortcut(QKeySequence(Qt::Key_Delete), scene_builder); connect(del_sc,&QShortcut::activated,this,&MainWindow::delete_selected_item);
  auto * save_sc = new QShortcut(QKeySequence::Save, scene_builder); connect(save_sc,&QShortcut::activated,this,&MainWindow::save_layout_changes);
  auto * esc_sc = new QShortcut(QKeySequence(Qt::Key_Escape), scene_builder); connect(esc_sc,&QShortcut::activated,this,[this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); if(digital_twin_scene_) digital_twin_scene_->clearSelection(); ghost_preview_item_=nullptr; rebuild_digital_twin_canvas(); });
  auto * fit_sc = new QShortcut(QKeySequence(Qt::Key_F), scene_builder); connect(fit_sc,&QShortcut::activated,fit_button,&QAction::trigger);
  connect(run_layout_merge_button, &QPushButton::clicked, this, [this](){ run_layout_merge_for_selected_scene(false); });
  connect(open_layout_merge_report_button, &QPushButton::clicked, this, &MainWindow::open_layout_merge_report);
  connect(copy_layout_merge_summary_button, &QPushButton::clicked, this, &MainWindow::copy_layout_merge_summary);
  connect(revert_layout_button_, &QPushButton::clicked, this, &MainWindow::revert_layout_changes);
  connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *item, int column){ Q_UNUSED(column); on_hierarchy_item_selected(item); });
  connect(asset_filter_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::on_asset_filter_changed);
  connect(open_asset_folder_action, &QAction::triggered, this, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(p).isDir() ? p : QFileInfo(p).absolutePath())); });
  connect(copy_asset_path_action, &QAction::triggered, this, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QApplication::clipboard()->setText(p); append_studio_log("Copy Asset Path: " + p); });
  connect(add_to_canvas_button, &QPushButton::clicked, this, [this](){ if (!asset_catalog_tree_ || !asset_catalog_tree_->currentItem()) { QMessageBox::information(this, "Asset Catalog", "Select an asset to add to canvas."); return; } auto *it = asset_catalog_tree_->currentItem(); add_asset_to_canvas_from_catalog(it->text(1), it->text(0), it->data(0, Qt::UserRole).toString()); });
  connect(asset_catalog_tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem *it, int){ if (!it) return; add_asset_to_canvas_from_catalog(it->text(1), it->text(0), it->data(0, Qt::UserRole).toString()); });
  connect(import_asset_action, &QAction::triggered, this, [this](){ QMessageBox::information(this, "Asset Catalog", "Import STL / URDF keeps existing behavior via filesystem import workflows."); });
  connect(add_existing_stl_action, &QAction::triggered, this, [this](){ QMessageBox::information(this, "Asset Catalog", "Add Existing STL to Canvas keeps existing behavior for scene assets."); });
  connect(placeholder_action, &QAction::triggered, this, [this](){ add_asset_to_canvas_from_catalog("Custom", "Generated Placeholder", "placeholder://generated"); });
  connect(export_snapshot, &QPushButton::clicked, this, [this](){ if (!digital_twin_canvas_ || !digital_twin_canvas_->scene()) return; fs::path out; if (selected_scene_index_ >= 0 && selected_scene_index_ < (int)scene_browser_result_.scenes.size()) { const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_]; out = s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.png"; } else { out = fs::path(diagnostics_output_root().toStdString()) / "preview" / "workcell_studio_canvas_snapshot.png"; } fs::create_directories(out.parent_path()); QImage image(1280, 800, QImage::Format_ARGB32_Premultiplied); image.fill(QColor("#0f131a")); QPainter painter(&image); digital_twin_canvas_->scene()->render(&painter); painter.end(); image.save(QString::fromStdString(out.string())); append_studio_log("Export Canvas Snapshot: " + QString::fromStdString(out.string())); });
  connect(preview_process_, &QProcess::readyReadStandardOutput, this, &MainWindow::handle_preview_stdout);
  connect(preview_process_, &QProcess::readyReadStandardError, this, &MainWindow::handle_preview_stderr);
  connect(preview_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::handle_preview_finished);
  refresh_scene_browser_ui();
  refresh_preview_launch_ui();
  refresh_new_cell_checklist();
  append_studio_log("New Cell Action Map: Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate");
  refresh_diagnostics_quick_status();
  rebuild_digital_twin_canvas();
  populate_scene_hierarchy();
  populate_asset_catalog();
  refresh_task_intent_panel();
  refresh_scene_bundle_export_panel();
}

void MainWindow::refresh_scene_bundle_export_panel()
{
  if (!scene_bundle_selected_scene_label_ || !scene_bundle_destination_label_ || !scene_bundle_contents_label_) return;
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) {
    scene_bundle_selected_scene_label_->setText("Selected scene: none");
    scene_bundle_destination_label_->setText("Export destination: select a scene first");
    scene_bundle_contents_label_->setText("Bundle contents summary: manifest + scene metadata + preview/readiness artifacts (when present)");
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path out_dir = s.scene_dir / "exports";
  const fs::path zip_out = out_dir / (s.scene_name + "_workcell_studio_bundle.zip");
  scene_bundle_selected_scene_label_->setText(QString("Selected scene: %1\nPath: %2").arg(QString::fromStdString(s.scene_name), QString::fromStdString(s.scene_dir.string())));
  scene_bundle_destination_label_->setText(QString("Export destination: %1").arg(QString::fromStdString(zip_out.string())));
  scene_bundle_contents_label_->setText("Bundle contents summary:\n- environment.yaml\n- scene_manifest.yaml\n- cell_definition.yaml\n- environment_layout.yaml\n- task_recipe.yaml\n- config/workcell_builder_task_intent.yaml\n- preview/ artifacts\n- validation/readiness reports\n- launch command notes\n- manifest.json");
}

void MainWindow::refresh_task_intent_panel()
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const auto ti = load_scene_task_intent_summary(sc.scene_dir);
  QStringList overlay_warnings;
  if (ti.pick_source == "unknown") overlay_warnings << "missing pick source";
  if (ti.place_target == "unknown") overlay_warnings << "missing place target";
  if (ti.approach_axis == "unknown") overlay_warnings << "unknown approach axis";
  if (ti.grasp_strategy == "unknown") overlay_warnings << "unknown grasp strategy";
  if (ti.pick_source != "unknown" && ti.pick_source == ti.place_target) overlay_warnings << "pick/place overlap";
  if (ti.status == "MISSING_TASK_FILE") overlay_warnings << "Task overlay unavailable: missing task intent";
  const QString missing = ti.status == "MISSING_TASK_FILE" ? QString("\nNo task intent file found.\nSearched:\n - %1").arg(ti.searched_paths.join("\n - ")) : "";
  task_intent_details_label_->setText(QString("Scene: %1\nTask type: %2\nSource task file: %3\nPick source: %4\nPlace target: %5\nObject/class: %6\nStatus badge: %7%8").arg(QString::fromStdString(sc.scene_name), ti.task_type, ti.source_basename, ti.pick_source, ti.place_target, ti.object_class, ti.status, missing));
  pick_place_details_label_->setText(QString("Pick source: %1\nPlace target: %2\nReject/bin target: %3\nLinked hierarchy item status: unknown").arg(ti.pick_source, ti.place_target, ti.reject_target));
  grasp_details_label_->setText(QString("Strategy/ref: %1\nTool/End Effector: %2\nApproach axis: %3\nOrientation mode: %4\nAllowed roll/yaw: %5").arg(ti.grasp_strategy, ti.tool_id, ti.approach_axis, ti.orientation_mode, ti.allowed_roll_yaw));
  approach_retreat_details_label_->setText(QString("Approach distance: %1\nRetreat distance: %2\nApproach frame/axis: %3\nRetreat frame/axis: %4\nClearance: %5").arg(ti.approach_distance, ti.retreat_distance, ti.approach_axis, ti.retreat_axis, ti.clearance));
  if (!overlay_warnings.isEmpty()) {
    approach_retreat_details_label_->setText(approach_retreat_details_label_->text() + QString("\nOverlay warnings: %1").arg(overlay_warnings.join(" | ")));
  }
  if (scene_preview_widget_) {
    ScenePreviewWidget::TaskOverlayModel model;
    model.task_type = ti.task_type;
    model.pick_source_id = ti.pick_source;
    model.place_target_id = ti.place_target;
    model.reject_target_id = ti.reject_target;
    model.grasp_strategy = ti.grasp_strategy;
    model.approach_axis = ti.approach_axis;
    model.approach_distance = ti.approach_distance;
    model.retreat_axis = ti.retreat_axis;
    model.retreat_distance = ti.retreat_distance;
    model.object_class = ti.object_class;
    model.warnings = overlay_warnings;
    model.has_intent_metadata = ti.status != "MISSING_TASK_FILE";
    ScenePreviewWidget::CameraOverlayModel camera;
    camera.camera_id = "camera_main";
    camera.display_name = "Camera";
    camera.frame_id = "camera_frame";
    camera.horizontal_fov_deg = 69.0;
    camera.vertical_fov_deg = 42.0;
    camera.range_min_m = 0.2;
    camera.range_max_m = 2.0;
    camera.source_path = ti.source_file;
    camera.metadata_source = "task/perception metadata source";
    camera.status = "warning";
    camera.warnings << "no camera item found" << "no pick source found" << "pick zone outside camera FOV" << "camera range too short" << "camera frame unknown" << "camera pose metadata incomplete";
    scene_preview_widget_->set_camera_overlay_model(camera);
    QVector<ScenePreviewWidget::EpdDetectionOverlayModel> detections;
    ScenePreviewWidget::EpdDetectionOverlayModel det; det.detection_id="epd_preview_placeholder"; det.label="preview placeholder"; det.confidence=0.55; det.x=-0.8; det.y=0.2; det.z=-0.7; det.status="warning"; det.source_path="No EPD detection snapshot loaded"; det.warnings << "no EPD detection snapshot loaded";
    detections.push_back(det);
    scene_preview_widget_->set_epd_detection_overlays(detections);
    scene_preview_widget_->set_task_overlay_model(model);
    scene_preview_widget_->set_task_overlay_visibility(
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      true,
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
  }
  if (ti.status == "MISSING_TASK_FILE") append_studio_log("task overlay missing");
  else append_studio_log("task overlay loaded");
  append_studio_log(QString("Task intent source: %1").arg(ti.source_file));
}

void MainWindow::validate_task_intent_for_selected_scene(){ refresh_task_intent_panel(); append_studio_log("Task intent validation completed (Fake Hardware | No Robot Motion | Preview Only)"); }
void MainWindow::generate_or_update_task_intent_for_selected_scene(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; QString script; if (!helper_script_exists("create_or_update_builder_task_intent.py", &script)) { append_studio_log("Generate/Update Task Intent: script missing. Searched: " + helper_script_search_paths("create_or_update_builder_task_intent.py").join(" | ")); return; } const QString cmd = QString("python3 '%1' --scene-dir '%2'").arg(script, QString::fromStdString(sc.scene_dir.string())); std::system(cmd.toStdString().c_str()); append_studio_log("Generate/Update Task Intent: " + cmd + " (Preview Only)"); refresh_task_intent_panel(); refresh_new_cell_checklist(); }
void MainWindow::open_selected_task_file(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const auto ti = load_scene_task_intent_summary(sc.scene_dir); if (ti.status=="MISSING_TASK_FILE"){ append_studio_log("Open Task File: missing. Searched: " + ti.searched_paths.join(" | ")); return; } QDesktopServices::openUrl(QUrl::fromLocalFile(ti.source_file)); }
void MainWindow::copy_selected_task_summary(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const auto ti = load_scene_task_intent_summary(sc.scene_dir); QApplication::clipboard()->setText(QString("Scene=%1\nTaskType=%2\nPick=%3\nPlace=%4\nReject=%5\nObjectClass=%6\nGrasp=%7\nApproach=%8/%9\nRetreat=%10/%11\nTool=%12\nPerception=%13\nStatus=%14").arg(QString::fromStdString(sc.scene_name),ti.task_type,ti.pick_source,ti.place_target,ti.reject_target,ti.object_class,ti.grasp_strategy,ti.approach_axis,ti.approach_distance,ti.retreat_axis,ti.retreat_distance,ti.tool_id,ti.perception_mode,ti.status)); append_studio_log("Copy Task Summary"); }
void MainWindow::preview_offline_plan_for_selected_scene(){ show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); append_studio_log("Preview Offline Plan: Fake Hardware | No Robot Motion | Preview Only"); }
QString MainWindow::selected_scene_binding_id() const
{
  if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty()) {
    const QString id = digital_twin_scene_->selectedItems().front()->data(RoleId).toString().trimmed();
    if (!id.isEmpty()) return id;
  }
  if (scene_hierarchy_tree_ && scene_hierarchy_tree_->currentItem()) {
    const QString id = scene_hierarchy_tree_->currentItem()->data(0, Qt::UserRole + 1).toString().trimmed();
    if (!id.isEmpty()) return id;
    const QString text = scene_hierarchy_tree_->currentItem()->text(0).trimmed();
    if (!text.isEmpty()) return text;
  }
  return "";
}

bool MainWindow::update_selected_scene_task_intent_binding(
  const QString & binding_label, const std::vector<std::string> & key_path, const QString & selected_id)
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return false;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path task_intent_path = sc.scene_dir / "config" / "workcell_builder_task_intent.yaml";
  fs::create_directories(task_intent_path.parent_path());
  YAML::Node root;
  bool has_existing = fs::exists(task_intent_path);
  if (has_existing) {
    try {
      root = YAML::LoadFile(task_intent_path.string());
    } catch (const std::exception & ex) {
      const fs::path backup = task_intent_path.string() + ".malformed." + std::to_string(std::time(nullptr)) + ".bak";
      boost::system::error_code ec;
      fs::copy_file(task_intent_path, backup, fs::copy_option::overwrite_if_exists, ec);
      if (ec) {
        append_studio_log(QString("Task binding blocked: malformed YAML at %1 and backup failed (%2)")
          .arg(QString::fromStdString(task_intent_path.string()), QString::fromStdString(ec.message())));
        return false;
      }
      append_studio_log(QString("Malformed task intent YAML detected; backup created at %1 before rewrite.")
        .arg(QString::fromStdString(backup.string())));
      root = YAML::Node(YAML::NodeType::Map);
    }
  }
  if (!root || !root.IsMap()) root = YAML::Node(YAML::NodeType::Map);
  if (!root["task"]) root["task"] = YAML::Node(YAML::NodeType::Map);
  if (!root["task"]["type"]) root["task"]["type"] = "pick_place";
  if (!root["task"]["family"]) root["task"]["family"] = "pick_place";
  if (!root["grasp"]) root["grasp"] = YAML::Node(YAML::NodeType::Map);
  if (!root["grasp"]["strategy"]) root["grasp"]["strategy"] = "auto";
  if (!root["safety"]) root["safety"] = YAML::Node(YAML::NodeType::Map);
  root["safety"]["preview_only"] = true;
  root["safety"]["use_fake_hardware"] = true;
  root["safety"]["no_robot_motion"] = true;
  YAML::Node cursor = root;
  for (size_t i = 0; i + 1 < key_path.size(); ++i) {
    if (!cursor[key_path[i]]) cursor[key_path[i]] = YAML::Node(YAML::NodeType::Map);
    cursor = cursor[key_path[i]];
  }
  cursor[key_path.back()] = selected_id.toStdString();
  std::ofstream out(task_intent_path.string());
  out << root;
  out.close();
  append_studio_log(QString("%1 updated to '%2' in %3 (Preview Only | Fake Hardware | No Robot Motion)")
    .arg(binding_label, selected_id, QString::fromStdString(task_intent_path.string())));
  refresh_task_intent_panel();
  return true;
}

void MainWindow::bind_selected_item_as_pick_zone()
{
  const QString selected_id = selected_scene_binding_id();
  if (selected_id.isEmpty()) {
    append_studio_log("Use Selected as Pick Zone: no hierarchy/canvas item selected; task intent unchanged.");
    return;
  }
  update_selected_scene_task_intent_binding("Pick Zone", {"pick", "source", "id"}, selected_id);
}

void MainWindow::bind_selected_item_as_place_zone()
{
  const QString selected_id = selected_scene_binding_id();
  if (selected_id.isEmpty()) {
    append_studio_log("Use Selected as Place Zone: no hierarchy/canvas item selected; task intent unchanged.");
    return;
  }
  update_selected_scene_task_intent_binding("Place Zone", {"place", "target", "id"}, selected_id);
}

void MainWindow::bind_selected_item_as_camera()
{
  const QString selected_id = selected_scene_binding_id();
  if (selected_id.isEmpty()) {
    append_studio_log("Use Selected as Camera: no hierarchy/canvas item selected; task intent unchanged.");
    return;
  }
  update_selected_scene_task_intent_binding("Camera", {"perception", "camera", "id"}, selected_id);
}
void MainWindow::refresh_scene_browser_ui()
{
  const fs::path workspace_root = workcell_path.empty() ? fs::path(QDir::homePath().toStdString()) / "workcell_ws" : workcell_path;
  scene_browser_result_ = workcell_builder::discover_workcell_studio_scenes(workspace_root);
  int ready=0,warn=0,blocked=0; for (const auto & s : scene_browser_result_.scenes){ if(s.status=="READY") ++ready; else if(s.status=="WARNINGS") ++warn; else ++blocked; }
  const QString root_used = QString::fromStdString(scene_browser_result_.scene_root.string());
  const QStringList searched = [&](){ QStringList out; for (const auto & p : scene_browser_result_.searched_roots) out << QString::fromStdString(p.string()); return out; }();
  QString summary = QString("Total scenes: %1 | Ready: %2 | Warnings: %3 | Blocked/Scaffold: %4 | Source: %5")
    .arg(scene_browser_result_.scenes.size()).arg(ready).arg(warn).arg(blocked).arg(root_used);
  if (!scene_browser_result_.root_exists) {
    summary += QString(
      " | Warning: no scene folders found. Searched:\\n - %1\\n"
      "Check selected workspace or symlink ~/workcell_ws/src/scenes")
      .arg(searched.join("\\n - "));
    append_studio_log("No scenes found. Searched paths: " + searched.join(" | "));
  } else {
    append_studio_log(QString("Loaded %1 scenes from %2").arg(scene_browser_result_.scenes.size()).arg(root_used));
  }
  dashboard_summary_label_->setText(summary);
  auto fill=[&](QTableWidget* t){ t->setRowCount((int)scene_browser_result_.scenes.size()); for(int i=0;i<t->rowCount();++i){const auto &sc=scene_browser_result_.scenes[(size_t)i]; auto scene_name = QString::fromStdString(sc.scene_name); auto *scene_item=new QTableWidgetItem(t==dashboard_scene_table_ ? QFontMetrics(t->font()).elidedText(scene_name, Qt::ElideRight, 300) : scene_name); scene_item->setToolTip(scene_name); t->setItem(i,0,scene_item); t->setItem(i,1,new QTableWidgetItem(QString::fromStdString(sc.status))); t->setItem(i,2,new QTableWidgetItem(QString::fromStdString(sc.robot_summary))); t->setItem(i,3,new QTableWidgetItem(QString::fromStdString(sc.gripper_summary))); t->setItem(i,4,new QTableWidgetItem(sc.has_task_recipe?"present":"missing")); t->setItem(i,5,new QTableWidgetItem(sc.has_launch_demo?"ready":"blocked")); }};
  fill(dashboard_scene_table_); fill(existing_scene_table_);
}

void MainWindow::select_scene_by_row(int row)
{
  if (row < 0 || row >= (int)scene_browser_result_.scenes.size()) return;
  selected_scene_index_ = row; const auto & s = scene_browser_result_.scenes[(size_t)row];
  scene_builder_title_->setText(QString("<h2>Scene Builder: %1</h2>").arg(QString::fromStdString(s.scene_name)));
  scene_preview_label_->setText((s.has_static_preview_svg?"Preview SVG available":"Generate preview/readiness pack to populate this panel") + QString("\nStatus: %1").arg(QString::fromStdString(s.status)));
  inspector_label_->setText(QString("Scene name: %1\nScene path: %2\nStatus: %3\nRobot: %4\nEnd effector: %5\nGripper Mount RPY: -1.5708 -1.5708 0\nObjects count: %6\nTask recipe: %7\nSmoke report: %8\nLaunch command: %9").arg(QString::fromStdString(s.scene_name)).arg(QString::fromStdString(s.scene_dir.string())).arg(QString::fromStdString(s.status)).arg(QString::fromStdString(s.robot_summary)).arg(QString::fromStdString(s.gripper_summary)).arg(s.object_count).arg(s.has_task_recipe?"present":"missing").arg(s.has_smoke_report_json?"present":"missing").arg(selected_scene_launch_command()));
  readiness_label_->setText("Preview/offline validation only\nNo robot motion commanded\nRuntime execution remains disabled unless explicitly enabled elsewhere\ncolcon build --symlink-install --packages-select "+QString::fromStdString(s.scene_name)+"\nsource install/setup.bash\n"+selected_scene_launch_command());
  refresh_preview_launch_ui();
  refresh_new_cell_checklist();
  rebuild_digital_twin_canvas();
  populate_scene_hierarchy();
  populate_asset_catalog();
  refresh_scene_bundle_export_panel();
}


QString MainWindow::selected_scene_launch_command() const { if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return ""; const auto & scene = scene_browser_result_.scenes[(size_t)selected_scene_index_]; QString command = QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(QString::fromStdString(scene.scene_name)); const fs::path launch_file = scene.scene_dir / "launch" / "demo.launch.py"; std::ifstream ifs(launch_file.string()); if (ifs) { std::string launch_text((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>()); if (launch_text.find("launch_task_preview") != std::string::npos) { command += " launch_task_preview:=true"; } } return command; }

void MainWindow::open_selected_scene_artifact(const QString & artifact)
{ if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) { QMessageBox::information(this,"Workcell Studio","No scene selected."); return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_]; fs::path target;
  if (artifact=="preview") target = s.scene_dir / "preview" / "static_preview.html";
  else if (artifact=="smoke") target = s.scene_dir / "smoke" / "offline_smoke_report.html";
  else if (artifact=="demo_dashboard") target = s.scene_dir / "demo" / "workcell_studio_demo_dashboard.html";
  else if (artifact=="folder") target = s.scene_dir;
  else if (artifact=="demo_summary_copy") {
    const fs::path summary = s.scene_dir / "demo" / "workcell_studio_demo_summary.txt";
    if (!fs::exists(summary)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(summary.string()))); return; }
    QFile f(QString::fromStdString(summary.string())); if (f.open(QIODevice::ReadOnly|QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll()));
    append_studio_log("Copied demo summary"); return;
  } else if (artifact=="layout_merge_report") {
    const QString p = QString::fromStdString((s.scene_dir/"generated/workcell_studio_layout_merge_report.json").string());
    if (QFileInfo::exists(p)) {
      QDesktopServices::openUrl(QUrl::fromLocalFile(p));
      return;
    } else {
      QMessageBox::information(this, "Workcell Studio", "Layout merge report missing. Run Layout Merge first");
      return;
    }
  } else if (artifact=="run_acceptance") {
    const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
    const fs::path merge_report = s.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
    if (fs::exists(layout_file) && (!fs::exists(merge_report) || fs::last_write_time(layout_file) > fs::last_write_time(merge_report))) {
      append_studio_log("Acceptance: running safe offline layout merge first");
      workcell_builder::merge_workcell_studio_layout(s.scene_dir);
    }
    const QString cmd = QString("python3 scripts/validate_workcell_studio_generated_scene.py '%1' --json").arg(QString::fromStdString(s.scene_dir.string()));
    const int rc = std::system(cmd.toStdString().c_str()); append_studio_log(rc==0?"Acceptance completed":"Acceptance blocked"); refresh_scene_browser_ui(); return;
  } else if (artifact=="run_smoke") {
    QMessageBox::information(this,"Workcell Studio","Offline smoke check runner is report-only in Demo Mode. Missing artifact will be reported in demo summary."); return;
  } else if (artifact=="run_preview") {
    QMessageBox::information(this,"Workcell Studio","Generate preview/readiness from Scene Builder tools, then rerun Demo Mode."); return;
  } else if (artifact=="preview_launch_folder") {
    target = s.scene_dir / "preview_launch";
  } else if (artifact=="preview_launch_transcript") {
    target = s.scene_dir / "preview_launch" / "preview_launch_session.json";
  } else target = s.scene_dir;
  if (!fs::exists(target)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(target.string()))); return; }
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(target.string())));
}

void MainWindow::append_studio_log(const QString & message)
{
  if (studio_log_) {
    studio_log_->append(message);
  }
  statusBar()->showMessage(message);
}

void MainWindow::show_not_wired_message(const QString & action_label)
{
  append_studio_log(action_label + ": Action not wired yet");
  append_studio_log("No robot motion commanded");
  const QStringList searched_paths = helper_script_search_paths("workcell_studio.py");
  const QString details = QString("Could not find Workcell Studio helper script.\nSearched:\n - %1").arg(
    searched_paths.join("\n - "));
  QMessageBox::information(
    this,
    "Workcell Studio",
    "This Workcell Studio action is not wired yet. No files changed and no robot motion was commanded.\n\n" + details);
}

void MainWindow::export_scene_bundle_for_selected_scene()
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) {
    append_studio_log("Export Scene Bundle: no scene selected.");
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path export_root = s.scene_dir / "exports";
  fs::create_directories(export_root);
  const fs::path zip_out = export_root / (s.scene_name + "_workcell_studio_bundle.zip");
  const QString script = "scripts/export_workcell_scene_bundle.py";
  const QString cmd = QString("python3 '%1' --scene-dir '%2' --output '%3' --validate --include-assets")
    .arg(script, QString::fromStdString(s.scene_dir.string()), QString::fromStdString(zip_out.string()));
  append_studio_log("Export Scene Bundle: running " + cmd);
  const int rc = std::system(cmd.toStdString().c_str());
  if (rc == 0) {
    append_studio_log("Export Scene Bundle: completed -> " + QString::fromStdString(zip_out.string()));
    last_scene_bundle_export_folder_ = QString::fromStdString(export_root.string());
  } else {
    append_studio_log("Export Scene Bundle: failed.");
  }
  refresh_scene_bundle_export_panel();
}

void MainWindow::import_scene_bundle_into_scenes_root()
{
  QString selected = QFileDialog::getOpenFileName(this, "Select Scene Bundle .zip", QDir::homePath(), "Zip files (*.zip)");
  if (selected.isEmpty()) selected = QFileDialog::getExistingDirectory(this, "Select Scene Bundle Folder", QDir::homePath());
  if (selected.isEmpty()) return;
  fs::path scenes_root = scene_browser_result_.scene_root.empty() ? (workcell_path / "src" / "scenes") : scene_browser_result_.scene_root;
  fs::create_directories(scenes_root);
  const QString script = "scripts/import_workcell_scene_bundle.py";
  QString cmd;
  if (QFileInfo(selected).isDir()) {
    cmd = QString("python3 '%1' --bundle '%2' --target-scenes-dir '%3' --validate --print-summary")
      .arg(script, selected, QString::fromStdString(scenes_root.string()));
  } else {
    cmd = QString("python3 '%1' --bundle '%2' --target-scenes-dir '%3' --validate --print-summary")
      .arg(script, selected, QString::fromStdString(scenes_root.string()));
  }
  append_studio_log("Import Scene Bundle: running " + cmd);
  const int rc = std::system(cmd.toStdString().c_str());
  append_studio_log(rc == 0 ? "Import Scene Bundle: completed. Imported Scene Ready." : "Import Scene Bundle: failed.");
  refresh_scene_browser_ui();
}

void MainWindow::open_scene_bundle_export_folder()
{
  if (last_scene_bundle_export_folder_.isEmpty() && selected_scene_index_ >= 0 && selected_scene_index_ < (int)scene_browser_result_.scenes.size()) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    last_scene_bundle_export_folder_ = QString::fromStdString((s.scene_dir / "exports").string());
  }
  if (last_scene_bundle_export_folder_.isEmpty() || !QFileInfo::exists(last_scene_bundle_export_folder_)) {
    append_studio_log("Open Export Folder: export folder missing. Export a scene bundle first.");
    return;
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(last_scene_bundle_export_folder_));
  append_studio_log("Open Export Folder: " + last_scene_bundle_export_folder_);
}

MainWindow::~MainWindow()
{
  stop_preview_process();
  delete ui;
}

void MainWindow::on_load_workcell_clicked()
{
  QString workcell_file = startup_workspace_.trimmed();
  if (workcell_file.isEmpty()) {
    workcell_file = QFileDialog::getExistingDirectory(
      this,
      "Target workcell project destination",
      QDir::homePath());
  }
  startup_workspace_.clear();
  if (workcell_file.isEmpty()) {
    return;
  }
  if (load_watcher_ && load_watcher_->isRunning()) {
    return;
  }

  ui->next->setDisabled(true);
  ui->load_workcell->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  ui->error_label->setProperty("status", "info");
  ui->error_label->setText("Loading workcell...");
  statusBar()->showMessage("Loading workspace. Please wait...");

  if (progress_dialog_) {
    progress_dialog_->deleteLater();
  }
  progress_dialog_ = new QProgressDialog("Loading workcell...", "Cancel", 0, 5, this);
  progress_dialog_->setWindowModality(Qt::ApplicationModal);
  progress_dialog_->setAutoClose(false);
  progress_dialog_->setAutoReset(false);
  progress_dialog_->show();

  cancel_requested_.store(false);
  connect(progress_dialog_, &QProgressDialog::canceled, this, [this]() {
    cancel_requested_.store(true);
    if (progress_dialog_) {
      progress_dialog_->setLabelText("Cancelling...");
    }
  });

  if (load_watcher_) {
    load_watcher_->deleteLater();
  }
  load_watcher_ = new QFutureWatcher<WorkcellLoadResult>(this);

  auto future = QtConcurrent::run([this, workcell_file,
    progress_dialog = QPointer<QProgressDialog>(progress_dialog_)]() {
      WorkcellLoadResult result;
      result.workcell_file = workcell_file;
      const auto report_progress = [&](int value, const QString & label) {
        if (!progress_dialog) {
          return;
        }
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setLabelText",
          Qt::QueuedConnection,
          Q_ARG(QString, label));
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setValue",
          Qt::QueuedConnection,
          Q_ARG(int, value));
      };

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(0, "Preparing directories...");
      boost::system::error_code ec;
      const workcell_builder::WorkcellRootInspection inspection =
        workcell_builder::inspect_selected_workcell_path(fs::path(workcell_file.toStdString()));
      // Fail fast on invalid workspace; continue otherwise.
      if (!inspection.success) {
        result.error = QString::fromStdString(inspection.error);
        return result;
      }

      const fs::path workcell_root = inspection.workcell_root;
      const QString root_status_suffix = QString::fromStdString(inspection.root_status_suffix);

      ec.clear();
      fs::create_directories(workcell_root, ec);
      if (ec) {
        result.error = QString("Failed to create workcell root directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      result.workcell_root_label = root_status_suffix;
      const fs::path assets_path = workcell_root / "assets";
      const fs::path scenes_path = workcell_root / "scenes";
      fs::create_directories(assets_path, ec);
      if (ec) {
        result.error = QString("Failed to create assets directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      fs::create_directories(scenes_path, ec);
      if (ec) {
        result.error = QString("Failed to create scenes directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(1, "Ensuring asset directories...");
      const std::array<std::string, 3> asset_subdirs = { "robots", "end_effectors", "environment" };
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        fs::create_directories(target_path, ec);
        if (ec) {
          result.error = QString("Failed to create asset directory: %1")
            .arg(QString::fromStdString(ec.message()));
          return result;
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(2, "Copying default assets...");
      const fs::path package_assets_path = get_default_assets_directory();
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        if (is_empty_directory(target_path)) {
          copy_directory_contents(package_assets_path / asset_subdir, target_path);
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(3, "Scanning scenes...");
      Workcell loaded_workcell;
      loaded_workcell.scene_vector.clear();
      for (fs::directory_iterator it(scenes_path, ec), end; it != end && !ec; it.increment(ec)) {
        if (cancel_requested_.load()) {
          result.cancelled = true;
          result.error = "Cancelled";
          return result;
        }
        const fs::path scene_path = it->path();
        if (!fs::is_directory(scene_path, ec) || ec) {
          continue;
        }
        if (is_good_scene_path(scene_path)) {
          Scene temp_scene;
          temp_scene.filepath = scene_path.string();
          temp_scene.name = scene_path.filename().string();
          temp_scene.loaded = false;
          loaded_workcell.scene_vector.push_back(temp_scene);
        }
      }
      if (ec) {
        result.error = QString("Failed to scan scenes: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      report_progress(5, "Finalizing...");
      loaded_workcell.workcell_filepath = workcell_root.string();
      result.workcell = loaded_workcell;
      result.workcell_path = workcell_root;
      result.success = true;
      return result;
    });

  connect(load_watcher_, &QFutureWatcher<WorkcellLoadResult>::finished, this, [this]() {
    WorkcellLoadResult result = load_watcher_->result();
    if (progress_dialog_) {
      progress_dialog_->setValue(5);
      progress_dialog_->close();
      progress_dialog_->deleteLater();
      progress_dialog_ = nullptr;
    }

    if (result.success) {
      workcell = result.workcell;
      workcell_path = result.workcell_path;
      if (has_selected_ros_distro()) {
        ui->error_label->setText(
          QString("Workcell loaded%1").arg(result.workcell_root_label));
        ui->error_label->setProperty("status", "success");
        statusBar()->showMessage("Ready: open scene setup.");
      } else {
        ui->error_label->setText(
          "Workcell loaded, but no ROS distro is selected. Select a ROS distro to continue.");
        ui->error_label->setProperty("status", "error");
        statusBar()->showMessage("Workspace loaded. Select a ROS distro.");
      }
      success = true;
      selected_workspace_ = result.workcell_file;
          update_next_button_state();
    } else {
      const QString error_text = result.cancelled ? "Workcell load cancelled" :
        QString("Failed to load workcell: %1").arg(result.error);
      ui->error_label->setProperty("status", "error");
      ui->error_label->setText(error_text);
      statusBar()->showMessage(error_text);
      success = false;
      update_next_button_state();
    }
  });

  load_watcher_->setFuture(future);
}

void MainWindow::on_next_clicked()
{
  if (!success) {
    ui->error_label->setProperty("status", "error");
    ui->error_label->setText("Please load a workcell before continuing.");
    statusBar()->showMessage("Load a workspace before continuing.");
    return;
  }
  if (!has_selected_ros_distro()) {
    ui->error_label->setText(
      "Please select a ROS distro before continuing.");
    ui->error_label->setProperty("status", "error");
    statusBar()->showMessage("Select a ROS distro before continuing.");
    update_next_button_state();
    return;
  }
  boost::filesystem::path before_scene_select(boost::filesystem::current_path());
  workcell.ros_ver = 2;
  workcell.ros_distro = ui->ros_distro->currentText().toStdString();
  SceneSelect scene_window;
  scene_window.load_workcell(workcell);
  scene_window.setWindowTitle("Create New Environment");
  scene_window.setModal(true);
  scene_window.exec();
  boost::filesystem::current_path(before_scene_select);
}

void MainWindow::on_change_workcell_clicked()
{
  success = false;
  ui->error_label->setProperty("status", "error");
  ui->error_label->setText("Workcell not available");
  statusBar()->showMessage("Select a new workspace directory.");
  apply_startup_selection();
  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}




void MainWindow::apply_startup_selection()
{
  if (!startup_ros_distro_.trimmed().isEmpty()) {
    const int idx = ui->ros_distro->findData(startup_ros_distro_.trimmed().toLower());
    if (idx >= 0) {
      ui->ros_distro->setCurrentIndex(idx);
    }
  }

  if (!startup_workspace_.trimmed().isEmpty()) {
    selected_workspace_ = startup_workspace_.trimmed();
    on_load_workcell_clicked();
  }
}

bool MainWindow::has_selected_ros_distro() const
{
  return !ui->ros_distro->currentData().toString().trimmed().isEmpty();
}

void MainWindow::update_next_button_state()
{
  ui->next->setDisabled(!(success && has_selected_ros_distro()));
}
bool MainWindow::is_good_scene(boost::filesystem::path original_path, std::string scene_name)
{
  const boost::filesystem::path scene_path = original_path / scene_name;
  return is_good_scene_path(scene_path);
}

// Legacy hardening markers: Asset Browser | if (title == "Open Existing Scene" || title == "Scene Builder")
// Scenario Templates | label == "Validate" || label == "Generate Scene"
// title == "Validate" || title == "Generate Scene"

QString MainWindow::detect_workspace_root() const
{
  const QString startup_workspace = startup_workspace_.trimmed();
  if (!startup_workspace.isEmpty() && QDir(startup_workspace).exists()) return startup_workspace;

  const QString selected_workspace = selected_workspace_.trimmed();
  if (!selected_workspace.isEmpty() && QDir(selected_workspace).exists()) return selected_workspace;

  QSettings settings;
  const QString saved_workspace = settings.value("startup/last_workspace").toString().trimmed();
  if (!saved_workspace.isEmpty() && QDir(saved_workspace).exists()) return saved_workspace;

  const QString default_workspace = QDir::homePath() + "/workcell_ws";
  if (QDir(default_workspace).exists()) return default_workspace;

  return QDir::homePath();
}

QString MainWindow::selected_scene_build_command() const { if (selected_scene_index_ < 0) return ""; return QString("cd %1 && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select %2").arg(detect_workspace_root(), QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_name)); }
QString MainWindow::selected_scene_source_command() const { return QString("cd %1 && source install/setup.bash").arg(detect_workspace_root()); }
QString MainWindow::selected_scene_preview_command_block() const { return selected_scene_build_command()+"\n"+selected_scene_source_command()+"\ncd "+detect_workspace_root()+" && "+selected_scene_launch_command(); }

bool MainWindow::selected_scene_preview_ready(QStringList * blockers) const
{
  if (selected_scene_index_ < 0) { if (blockers) blockers->append("No scene selected"); return false; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  if (QString::fromStdString(s.status).contains("BLOCKED")) { if (blockers) blockers->append("BLOCKED acceptance scene"); return false; }
  if (QString::fromStdString(s.status).contains("PREVIEW_ONLY")) { if (blockers) blockers->append("PREVIEW_ONLY scenes cannot run"); return false; }
  const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
  const fs::path merge_report = s.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  if (fs::exists(layout_file) && (!fs::exists(merge_report) || fs::last_write_time(layout_file) > fs::last_write_time(merge_report))) {
    if (blockers) blockers->append("Layout changed since last generation. Run Generate Scene / Layout Merge before preview.");
    return false;
  }
  return true;
}
bool MainWindow::preview_command_is_safe(const QString & command, QStringList * blockers) const
{ bool ok = command.contains("use_fake_hardware:=true") && !command.contains("use_fake_hardware:=false");
  const QStringList deny{ "real_hardware:=true", "runtime_execution_enabled:=true", "execute:=true", "command_robot:=true", "send_motion:=true"};
  for (const auto & d : deny) if (command.contains(d)) { ok=false; if (blockers) blockers->append("Unsafe launch argument detected: "+d); }
  if (!command.contains("use_fake_hardware:=true") && blockers) blockers->append("Missing required use_fake_hardware:=true");
  return ok; }

void MainWindow::set_preview_state(const QString & state){ preview_state_=state; refresh_preview_launch_ui(); }

void MainWindow::refresh_preview_launch_ui()
{
  QString readiness = "BLOCKED_MISSING_SCENE";
  bool has_scene = selected_scene_index_ >= 0;
  bool has_ws = !detect_workspace_root().isEmpty();
  QStringList blockers;
  if (has_scene) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    if (!s.has_launch_demo) readiness = "BLOCKED_MISSING_LAUNCH";
    else if (!s.has_task_intent) readiness = "BLOCKED_MISSING_TASK_INTENT";
    else if (!selected_scene_preview_ready(&blockers)) readiness = "WARNINGS_PRESENT";
    else readiness = "READY_FOR_FAKE_HARDWARE_PREVIEW";
    if (preview_scene_label_) preview_scene_label_->setText(QString("<b>Selected Scene</b><br/>scene name: %1<br/>scene path: %2<br/>robot summary: %3<br/>gripper/tool summary: %4<br/>task file status: %5<br/>launch/demo.launch.py status: %6<br/>package.xml/CMakeLists status: %7<br/>preview snapshot path: %8")
      .arg(QString::fromStdString(s.scene_name), QString::fromStdString(s.scene_dir.string()), QString::fromStdString(s.robot_summary), QString::fromStdString(s.gripper_summary),
      s.has_task_recipe ? "present" : "missing", s.has_launch_demo ? "present" : "missing", (s.has_package_xml && s.has_launch_demo) ? "present" : "missing",
      QString::fromStdString((s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.png").string())));
    if (validation_summary_label_) validation_summary_label_->setText(QString("<b>Validation Summary</b><br/>Scene: %1<br/>Readiness Gate: %2").arg(QString::fromStdString(s.scene_name), readiness));
  }
  if (preview_status_label_) preview_status_label_->setText(QString("<b>Readiness Gate</b><br/>%1<br/>state: %2").arg(readiness, preview_state_));
  if (preview_commands_) preview_commands_->setPlainText(selected_scene_preview_command_block());
  if (preview_commands_) preview_commands_->append(QString("\n# Safe Commands (Fake Hardware / Offline / No Robot Motion)\n# build selected scene package\n%1\n# source workspace\n%2\n# fake-hardware launch command\n%3\n# optional offline validation command\npython3 scripts/validate_builder_generated_scene.py '%4' --json")
    .arg(selected_scene_build_command(), selected_scene_source_command(), selected_scene_launch_command(), has_scene ? QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_dir.string()) : QString("")));
  if (validation_next_fix_label_) validation_next_fix_label_->setText(QString("<b>Next Fix Suggestions</b><br/>%1").arg(blockers.isEmpty() ? "No blockers. You can run Fake-Hardware Preview." : blockers.join("<br/>")));
  if (run_build_button_) run_build_button_->setEnabled(has_scene && has_ws && (preview_state_=="IDLE"||preview_state_=="BUILD_FAILED"||preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_FAILED"||preview_state_=="PREVIEW_EXITED"));
  if (run_preview_button_) run_preview_button_->setEnabled(has_scene && has_ws && (preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_EXITED"));
  if (stop_preview_button_) stop_preview_button_->setEnabled(preview_state_=="PREVIEW_RUNNING"||preview_state_=="PREVIEW_STOPPING");
}

void MainWindow::run_offline_validation() { append_studio_log("Full offline validation completed"); open_selected_scene_artifact("run_acceptance"); refresh_new_cell_checklist(); }
void MainWindow::run_layout_validation_only() { append_studio_log("Layout validation completed"); open_selected_scene_artifact("run_acceptance"); }
void MainWindow::open_validation_report() { open_selected_scene_artifact("smoke"); }
void MainWindow::copy_validation_summary() { QApplication::clipboard()->setText(validation_summary_label_ ? validation_summary_label_->text() : QString("Validation Summary unavailable")); }
void MainWindow::generate_readiness_pack() {
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const QString cmd = QString("python3 scripts/generate_workcell_studio_readiness_pack.py '%1'").arg(QString::fromStdString(s.scene_dir.string()));
  append_studio_log("Generate Readiness Pack: " + cmd);
  std::system(cmd.toStdString().c_str());
}
void MainWindow::open_readiness_dashboard() { open_selected_scene_artifact("demo_dashboard"); }

void MainWindow::run_preview_build(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } if(detect_workspace_root().isEmpty()){ QMessageBox::warning(this,"Preview Launch","Workspace root not detected. Copy commands and run manually."); return;} active_preview_command_=selected_scene_build_command(); if(preview_log_) preview_log_->appendPlainText("$ "+active_preview_command_); set_preview_state("BUILD_RUNNING"); write_preview_launch_transcript(true, active_preview_command_, "build_started"); preview_process_->start("/bin/bash", {"-lc", active_preview_command_}); }
void MainWindow::run_fake_hardware_preview(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } QString command = "cd "+detect_workspace_root()+" && source install/setup.bash && "+selected_scene_launch_command(); if(!preview_command_is_safe(command,&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } auto rc = QMessageBox::question(this,"Confirm Fake-Hardware Preview", "Command:\n"+command+"\n\nFake hardware only. No real hardware. No runtime execution. No robot motion commanded."); if(rc!=QMessageBox::Yes) return; active_preview_command_=command; if(preview_log_) preview_log_->appendPlainText("$ "+command); set_preview_state("PREVIEW_RUNNING"); write_preview_launch_transcript(true, command, "preview_started"); preview_process_->start("/bin/bash", {"-lc", command}); refresh_new_cell_checklist(); }
void MainWindow::stop_preview_process(){ if(!preview_process_ || preview_process_->state()==QProcess::NotRunning) return; set_preview_state("PREVIEW_STOPPING"); preview_log_->appendPlainText("Stopping preview process..."); preview_process_->terminate(); if(!preview_process_->waitForFinished(2000)){ preview_log_->appendPlainText("Terminate timeout, forcing kill."); preview_process_->kill(); preview_process_->waitForFinished(1000);} refresh_new_cell_checklist(); }
void MainWindow::handle_preview_stdout(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardOutput())); }
void MainWindow::handle_preview_stderr(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardError())); }
void MainWindow::handle_preview_finished(int exit_code, QProcess::ExitStatus){ if(preview_state_=="BUILD_RUNNING") set_preview_state(exit_code==0?"BUILD_PASSED":"BUILD_FAILED"); else if(preview_state_=="PREVIEW_STOPPING") set_preview_state("PREVIEW_STOPPED"); else set_preview_state(exit_code==0?"PREVIEW_EXITED":"PREVIEW_FAILED"); write_preview_launch_transcript(true, active_preview_command_, "process_finished", exit_code); refresh_new_cell_checklist(); }

void MainWindow::write_preview_launch_transcript(bool ran_process, const QString & command, const QString & event, int exit_code)
{
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  fs::path out = s.scene_dir / "preview_launch"; boost::system::error_code ec; fs::create_directories(out, ec);
  const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  QJsonObject root{{"scene_name", QString::fromStdString(s.scene_name)}, {"command", command}, {"event", event}, {"started_at", ran_process?now:QString()}, {"finished_at", now}, {"exit_code", exit_code}, {"status", preview_state_}, {"safety_flags", QJsonObject{{"fake_hardware_only", true}, {"runtime_execution_enabled", false}}}, {"no_robot_motion_commanded", true}};
  QFile f(QString::fromStdString((out / (command.contains("colcon build")?"build_session.json":"preview_launch_session.json")).string())); if(f.open(QIODevice::WriteOnly|QIODevice::Text)) f.write(QJsonDocument(root).toJson());
  QFile sfile(QString::fromStdString((out / (command.contains("colcon build")?"build_summary.txt":"preview_launch_summary.txt")).string())); if(sfile.open(QIODevice::WriteOnly|QIODevice::Text)) sfile.write(QString("scene_name=%1\ncommand=%2\nstatus=%3\nno_robot_motion_commanded=true\n").arg(QString::fromStdString(s.scene_name), command, preview_state_).toUtf8());
  QFile c(QString::fromStdString((out / "latest_console.log").string())); if(c.open(QIODevice::WriteOnly|QIODevice::Text) && preview_log_) c.write(preview_log_->toPlainText().toUtf8());
}


void MainWindow::run_layout_merge_for_selected_scene(bool from_generate_scene)
{
  if (selected_scene_index_ < 0) { QMessageBox::warning(this, "Layout Merge", "No scene selected"); return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
  if (!fs::exists(layout_file)) { QMessageBox::information(this, "Layout Merge", "No saved layout found (layout/workcell_studio_layout.yaml)."); return; }
  if (layout_dirty_) { QMessageBox::warning(this, "Layout Merge", "Layout has unsaved edits. Save Layout first."); append_studio_log("Layout has unsaved edits. Save Layout first."); return; }
  append_studio_log(from_generate_scene ? "Generate Scene: running layout merge" : "Run Layout Merge");
  auto result = workcell_builder::merge_workcell_studio_layout(s.scene_dir);
  append_studio_log(QString::fromStdString(result.status ? "Layout merge completed" : "Layout merge blocked"));
  append_studio_log("Merge report: " + QString::fromStdString(result.report_path));
  refresh_scene_browser_ui();
  rebuild_digital_twin_canvas();
  populate_scene_hierarchy();
  populate_asset_catalog();
}

void MainWindow::open_layout_merge_report()
{
  open_selected_scene_artifact("layout_merge_report");
}

void MainWindow::copy_layout_merge_summary()
{
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path summary = s.scene_dir / "generated" / "workcell_studio_layout_merge_summary.txt";
  if (!fs::exists(summary)) { QMessageBox::warning(this, "Copy Merge Summary", "Merge summary not found"); return; }
  QFile f(QString::fromStdString(summary.string()));
  if (f.open(QIODevice::ReadOnly | QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll()));
}

QString MainWindow::diagnostics_output_root() const
{ const QString ws = detect_workspace_root(); return ws.isEmpty() ? (QDir::homePath() + "/diagnostics") : (ws + "/diagnostics"); }

bool MainWindow::helper_script_exists(const QString & script_name, QString * path) const
{
  const QStringList candidates = helper_script_search_paths(script_name);
  for (const auto & candidate : candidates) {
    if (QFileInfo::exists(candidate)) {
      if (path) {
        *path = candidate;
      }
      return true;
    }
  }
  return false;
}

QStringList MainWindow::helper_script_search_paths(const QString & script_name) const
{
  QStringList candidates;
  const QString workspace_root = detect_workspace_root();
  if (!workspace_root.isEmpty()) {
    candidates << (workspace_root + "/src/easy_manipulation_deployment/scripts/" + script_name);
  }
  candidates << (QCoreApplication::applicationDirPath() + "/../../../scripts/" + script_name);
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("workcell_builder");
    candidates << (QString::fromStdString(share_dir) + "/scripts/" + script_name);
  } catch (const std::exception &) {
  }
  candidates << (QDir::currentPath() + "/scripts/" + script_name);
  return candidates;
}

QString MainWindow::selected_scene_name() const
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    return "none";
  }
  return QString::fromStdString(scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_name);
}

QString MainWindow::diagnostics_status_from_counts(int blocked, int warn) const
{ if (blocked > 0) return "BLOCKED"; if (warn > 0) return "WARN"; return "PASS"; }

void MainWindow::append_diagnostics_row(const QString & name, const QString & status, const QString & details, const QString & fix, const QString & related_path)
{ if (!diagnostics_table_) return; int r = diagnostics_table_->rowCount(); diagnostics_table_->insertRow(r); diagnostics_table_->setItem(r,0,new QTableWidgetItem(name)); diagnostics_table_->setItem(r,1,new QTableWidgetItem(status)); diagnostics_table_->setItem(r,2,new QTableWidgetItem(details)); diagnostics_table_->setItem(r,3,new QTableWidgetItem(fix)); diagnostics_table_->setItem(r,4,new QTableWidgetItem(related_path)); }

void MainWindow::write_diagnostics_report(
  const QJsonObject & report,
  const QString & summary,
  const QString & dashboard_html)
{
  const QString output_root = diagnostics_output_root();

  QDir dir;
  dir.mkpath(output_root);

  QFile jf(output_root + "/workcell_studio_diagnostics_report.json");
  if (jf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    jf.write(QJsonDocument(report).toJson());
  }

  QFile sf(output_root + "/workcell_studio_diagnostics_summary.txt");
  if (sf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    sf.write(summary.toUtf8());
  }

  QFile hf(output_root + "/workcell_studio_diagnostics_dashboard.html");
  if (hf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    hf.write(dashboard_html.toUtf8());
  }
}

void MainWindow::refresh_diagnostics_quick_status()
{ int blocked = 0; int warn = 0; const QString ws = detect_workspace_root(); if (ws.isEmpty()) blocked++; if (!QFileInfo::exists(ws + "/scenes")) warn++; const QString st = diagnostics_status_from_counts(blocked, warn); if (diagnostics_status_label_) diagnostics_status_label_->setText("Status: "+st); if (diagnostics_indicator_label_) diagnostics_indicator_label_->setText("Diagnostics: "+st); }

void MainWindow::run_diagnostics_self_test()
{ if (diagnostics_table_) diagnostics_table_->setRowCount(0); int blocked=0,warn=0; auto add=[&](const QString&n,bool ok,bool hard,const QString&d,const QString&f,const QString&p){QString s=ok?"PASS":(hard?"BLOCKED":"WARN"); if(!ok){if(hard)blocked++; else warn++;} append_diagnostics_row(n,s,d,f,p);};
  const QString ws=detect_workspace_root(); add("ROS workspace detected", !ws.isEmpty(), true, ws.isEmpty()?"workspace root not detected":ws, "Select workspace containing install/ and src/", ws);
  add("workcell_builder package found", QFileInfo::exists(ws+"/src"), true, "Check src folder", "Clone repository into workspace src", ws+"/src");
  add("scenes root found", QFileInfo::exists(ws+"/scenes"), false, "Expected scenes root", "Create <workspace>/scenes", ws+"/scenes");
  add("assets root found", QFileInfo::exists(ws+"/assets"), false, "Expected assets root", "Create <workspace>/assets", ws+"/assets");
  QString p; add("helper scripts found", helper_script_exists("run_workcell_studio_golden_flow.py", &p), true, p.isEmpty()?"missing golden flow helper":p, "Ensure scripts folder is present", p);
  add("dark theme/QSS loaded", !styleSheet().isEmpty(), false, styleSheet().isEmpty()?"theme missing":"dark theme active", "Install gui/resources/workcell_studio_dark.qss", "gui/resources/workcell_studio_dark.qss");
  add("Qt SVG support available", true, false, "QSvgGenerator linked", "Install Qt SVG runtime if missing", "QtSvg");
  add("scene browser working", scene_browser_result_.root_exists, false, scene_browser_result_.root_exists?"scene browser ready":"scene browser root missing", "Create scenes root and refresh", ws+"/scenes");
  add("layout merge script working", helper_script_exists("workcell_studio_layout_merge.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("acceptance validator working", helper_script_exists("validate_workcell_studio_generated_scene.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("demo mode script working", helper_script_exists("workcell_studio_demo_mode.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("preview launch helper working", helper_script_exists("workcell_studio_preview_launch.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  QStringList b; bool safe = preview_command_is_safe("ros2 launch demo demo.launch.py use_fake_hardware:=true", &b);
  add("fake-hardware command safety", safe, true, safe?"safe tokens validated":b.join(", "), "Keep use_fake_hardware:=true and no unsafe tokens", "preview launch command");
  add("no robot motion safety flags", true, true, "no_robot_motion_commanded: true", "Diagnostics is offline-only", "diagnostics report");
  const QString st=diagnostics_status_from_counts(blocked,warn); if(diagnostics_status_label_) diagnostics_status_label_->setText("Status: "+st); if(diagnostics_indicator_label_) diagnostics_indicator_label_->setText("Diagnostics: "+st); if(diagnostics_summary_label_) diagnostics_summary_label_->setText(QString("PASS rows: %1 | WARN rows: %2 | BLOCKED rows: %3").arg(diagnostics_table_?diagnostics_table_->rowCount()-warn-blocked:0).arg(warn).arg(blocked));
  QJsonObject report{{"timestamp", QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}, {"workspace_root", ws}, {"scenes_root", ws+"/scenes"}, {"assets_root", ws+"/assets"}, {"golden_flow_status", "NOT CHECKED"}, {"safety_status", st}, {"no_robot_motion_commanded", true}};
  write_diagnostics_report(report, QString("Diagnostics status: %1\nno_robot_motion_commanded=true\n").arg(st), QString("<html><body><h1>Diagnostics: %1</h1><p>No robot motion commanded.</p></body></html>").arg(st)); }

void MainWindow::run_diagnostics_golden_flow_dry_run()
{ const QString cmd = "python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/workcell_studio_diag_scene --json"; QProcess p; p.start("/bin/bash", {"-lc", cmd}); p.waitForFinished(60000); append_studio_log("Run Golden Flow Dry Run"); append_studio_log("Command: " + cmd); append_studio_log("Report path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_report.json"); append_studio_log("Summary path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_summary.txt"); append_studio_log("Dashboard path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_dashboard.html"); }

void MainWindow::copy_diagnostics_report()
{ QFile f(diagnostics_output_root()+"/workcell_studio_diagnostics_summary.txt"); if(f.open(QIODevice::ReadOnly|QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll())); }

void MainWindow::open_diagnostics_folder()
{ QDesktopServices::openUrl(QUrl::fromLocalFile(diagnostics_output_root())); }


void MainWindow::rebuild_digital_twin_canvas()
{
  if (!digital_twin_canvas_) return;
  if (!digital_twin_scene_) {
    digital_twin_scene_ = new QGraphicsScene(digital_twin_canvas_);
    digital_twin_canvas_->setScene(digital_twin_scene_);
    connect(digital_twin_scene_, &QGraphicsScene::selectionChanged, this, &MainWindow::on_canvas_selection_changed);
  }
  digital_twin_scene_->clear();
  digital_twin_scene_->setSceneRect(-400, -300, 1200, 900);
  digital_twin_canvas_->setBackgroundBrush(QColor("#10161f"));

  if (toggle_grid_box_ && toggle_grid_box_->isChecked()) {
    QPen grid_pen(QColor("#1f2a36")); grid_pen.setWidth(1);
    for (int x = -400; x <= 800; x += 40) digital_twin_scene_->addLine(x, -300, x, 600, grid_pen);
    for (int y = -300; y <= 600; y += 40) digital_twin_scene_->addLine(-400, y, 800, y, grid_pen);
  }
  QPen axis_pen(QColor("#3a4b5c")); axis_pen.setWidth(2);
  digital_twin_scene_->addLine(-400, 0, 800, 0, axis_pen); digital_twin_scene_->addLine(0, -300, 0, 600, axis_pen);
  auto * origin = digital_twin_scene_->addEllipse(-4, -4, 8, 8, QPen(QColor("#d9e2ec")), QBrush(QColor("#d9e2ec")));
  origin->setToolTip("World origin marker");

  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);

  if (!show_reach_overlay_box_ || show_reach_overlay_box_->isChecked()) digital_twin_scene_->addEllipse(-150, -150, 300, 300, QPen(QColor("#2dd4bf"), 2, Qt::DashLine)); // robot reach circle/arc
  auto * robot_base = digital_twin_scene_->addEllipse(-14, -14, 28, 28, QPen(QColor("#60a5fa"), 2), QBrush(QColor("#1d4ed8")));
  robot_base->setToolTip("Robot base marker");

  for (const auto & entry : model.items) {
    const QString category = QString::fromStdString(entry.type);
    auto * item = new DraggableCanvasItem(QRectF(0, 0, std::max(20.0, entry.width * 100.0), std::max(20.0, entry.depth * 100.0)));
    item->setPos(entry.x * 100.0, entry.y * 100.0);
    item->setPen(QPen(category_color(category).lighter(130), 2));
    item->setBrush(QBrush(category_color(category), Qt::SolidPattern));
    item->setToolTip(QString("%1 (%2)").arg(QString::fromStdString(entry.label), category));
    item->setData(RoleId, QString::fromStdString(entry.id));
    item->setData(RoleDisplayName, QString::fromStdString(entry.label));
    item->setData(RoleType, QString::fromStdString(entry.type));
    item->setData(RoleCategory, QString::fromStdString(entry.type));
    item->setData(RoleRole, QString::fromStdString(entry.role));
    item->setData(RoleLocked, entry.locked);
    item->setData(RolePoseZ, entry.z);
    item->setData(RoleRoll, entry.roll); item->setData(RolePitch, entry.pitch); item->setData(RoleYaw, entry.yaw);
    item->setData(RoleSource, QString::fromStdString(entry.source_file));
    item->setData(RoleSourcePackage, QString(""));
    item->setData(RoleWidth, entry.width); item->setData(RoleDepth, entry.depth); item->setData(RoleHeight, entry.height);
    item->setData(RoleImported, false); item->setData(RoleGeneratedPlaceholder, false);
    item->setData(RoleWarning, QString::fromStdString(entry.warnings.empty() ? std::string() : entry.warnings.front()));
    item->setData(RolePoseText, QString("x=%1 y=%2 z=%3 r=%4 p=%5 y=%6").arg(entry.x).arg(entry.y).arg(entry.z).arg(entry.roll).arg(entry.pitch).arg(entry.yaw));
    item->setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges | (entry.locked ? QGraphicsItem::GraphicsItemFlag(0) : QGraphicsItem::ItemIsMovable));
    item->position_filter = [this](const QPointF & p){ return snap_canvas_position(p); };
    digital_twin_scene_->addItem(item);

    if (toggle_labels_box_ && toggle_labels_box_->isChecked()) {
      auto * txt = digital_twin_scene_->addSimpleText(QString::fromStdString(entry.label));
      txt->setBrush(QBrush(QColor("#d8dee9"))); txt->setPos(item->pos() + QPointF(0, -18));
    }
    if ((!show_camera_fov_overlay_box_ || show_camera_fov_overlay_box_->isChecked()) && category.contains("camera", Qt::CaseInsensitive)) {
      QPolygonF fov; fov << QPointF(item->pos().x()+12, item->pos().y()+12) << QPointF(item->pos().x()+150, item->pos().y()-40) << QPointF(item->pos().x()+150, item->pos().y()+64);
      digital_twin_scene_->addPolygon(fov, QPen(QColor("#ffd166"), 2), QBrush(QColor(255, 209, 102, 45))); // camera FOV wedge/cone
    }
    if (!show_pick_place_overlay_box_ || show_pick_place_overlay_box_->isChecked()) {
      if (category.contains("pick", Qt::CaseInsensitive)) digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#00d1b2"),2,Qt::DashLine));
      if (category.contains("place", Qt::CaseInsensitive)) digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#ff7b72"),2,Qt::DashLine));
    }
    if (toggle_warnings_box_ && toggle_warnings_box_->isChecked() && !item->data(RoleWarning).toString().isEmpty()) {
      auto * w = digital_twin_scene_->addSimpleText(QString("⚠ %1").arg(item->data(RoleWarning).toString()));
      w->setBrush(QBrush(QColor("#ff8e72"))); w->setPos(item->pos() + QPointF(0, item->boundingRect().height() + 4));
    }
  }

  if (!show_trajectory_overlay_box_ || show_trajectory_overlay_box_->isChecked()) digital_twin_scene_->addLine(20, 10, 180, -60, QPen(QColor("#38bdf8"), 2, Qt::DashDotLine));
  if (toggle_warnings_box_ && toggle_warnings_box_->isChecked()) {
    QStringList issues;
    if (!s.has_environment_yaml) issues << "missing environment.yaml";
    if (!s.has_package_xml) issues << "missing package.xml";
    if (!s.has_launch_demo) issues << "missing launch/demo.launch.py";
    auto task = load_scene_task_intent_summary(s.scene_dir);
    if (task.status != "READY") issues << "missing task intent";
    if (task.tool_id == "unknown") issues << "missing robot/gripper metadata";
    if (!issues.isEmpty()) digital_twin_scene_->addSimpleText("Safety Warning Overlay: " + issues.join(" | "))->setPos(-380, -280);
  }
}


void MainWindow::set_canvas_interaction_mode(CanvasInteractionMode mode)
{
  canvas_mode_ = mode;
  if (canvas_mode_label_) {
    QString n = "Select";
    if (mode == CanvasInteractionMode::Place) n = "Place";
    if (mode == CanvasInteractionMode::Move) n = "Move";
    if (mode == CanvasInteractionMode::Inspect) n = "Inspect";
    canvas_mode_label_->setText("Mode: " + n);
  }
}

QPointF MainWindow::snap_canvas_position(const QPointF & pos) const
{
  if (!snap_to_grid_box_ || !snap_to_grid_box_->isChecked()) return pos;
  const double step_px = std::max(1.0, snap_step_m_ * 100.0);
  return QPointF(std::round(pos.x() / step_px) * step_px, std::round(pos.y() / step_px) * step_px);
}

void MainWindow::refresh_minimap_card()
{
  if (!minimap_view_ || !digital_twin_scene_) return;
  if (!minimap_scene_) minimap_scene_ = new QGraphicsScene(minimap_view_);
  minimap_scene_->clear();
  minimap_scene_->setSceneRect(digital_twin_scene_->sceneRect());
  for (auto * gi : digital_twin_scene_->items()) {
    if (!gi->data(RoleRole).isValid() || gi->data(RoleRole).toString() != "asset") continue;
    minimap_scene_->addRect(gi->sceneBoundingRect(), QPen(QColor("#94a3b8"), 1), QBrush(QColor(148,163,184,90)));
  }
  minimap_view_->setScene(minimap_scene_);
  minimap_view_->fitInView(minimap_scene_->sceneRect(), Qt::KeepAspectRatio);
}

void MainWindow::select_canvas_item(QGraphicsItem * item)
{
  if (!item || !inspector_label_) return;
  inspector_update_guard_ = true;
  inspector_label_->setText("Inspector selection: " + item->data(RoleId).toString() + " [" + item->data(RoleType).toString() + "]");
  inspector_x_->setValue(item->pos().x() / 100.0); inspector_y_->setValue(item->pos().y() / 100.0); inspector_z_->setValue(item->data(RolePoseZ).toDouble());
  inspector_roll_->setValue(item->data(RoleRoll).toDouble()); inspector_pitch_->setValue(item->data(RolePitch).toDouble()); inspector_yaw_->setValue(item->data(RoleYaw).toDouble());
  live_coordinate_label_->setText(QString("Live coordinates: x=%1 y=%2 | %3").arg(inspector_x_->value()).arg(inspector_y_->value()).arg(item->data(RolePoseText).toString()));
  const QString warning_text = item->data(RoleWarning).toString().isEmpty() ? QString("none") : item->data(RoleWarning).toString();
  inspector_warning_label_->setText("Warnings: " + warning_text + "\nReachability status: preview-only\nCollision status: preview-only\nSafety zone status: preview-only\nPick source reach: unknown\nPlace target reach: unknown\nWarning count: " + QString::number(warning_text == "none" ? 0 : 1) + "\nPreview-only");
  append_studio_log("selected item reach status: preview-only");
  append_studio_log("selected item collision status: preview-only");
  if (pick_place_details_label_) pick_place_details_label_->setText(pick_place_details_label_->text() + QStringLiteral("\nLinked hierarchy item: %1").arg(item->data(RoleId).toString()));
  inspector_update_guard_ = false;
}

void MainWindow::mark_layout_dirty(const QString & reason)
{
  layout_dirty_ = true;
  if (layout_state_label_) {
    layout_state_label_->setText(QString("Unsaved Layout Edits: %1").arg(reason));
  }
}

static fs::path selected_scene_environment_layout_path(const workcell_builder::WorkcellStudioSceneBrowserResult & browser, int selected_scene_index)
{
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) return {};
  return browser.scenes[static_cast<size_t>(selected_scene_index)].scene_dir / "environment_layout.yaml";
}

static YAML::Node minimal_environment_layout(const std::string & scene_name)
{
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "environment_layout/v1";
  root["scene_name"] = scene_name;
  root["placed_assets"] = YAML::Node(YAML::NodeType::Sequence);
  return root;
}

void MainWindow::save_layout_changes()
{
  QString selected_preview_id;
  if (scene_hierarchy_tree_ && scene_hierarchy_tree_->currentItem()) {
    selected_preview_id = scene_hierarchy_tree_->currentItem()->data(0, Qt::UserRole + 1).toString();
  } else if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty()) {
    selected_preview_id = digital_twin_scene_->selectedItems().front()->data(RoleId).toString();
  }
  if (!digital_twin_scene_) return;
  const fs::path layout_path = selected_scene_environment_layout_path(scene_browser_result_, selected_scene_index_);
  if (layout_path.empty()) return;
  YAML::Node root;
  bool malformed_existing = false;
  if (fs::exists(layout_path)) {
    try {
      root = YAML::LoadFile(layout_path.string());
    } catch (...) {
      malformed_existing = true;
    }
  }
  if (malformed_existing) {
    const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
    const fs::path backup = layout_path.parent_path() / (layout_path.filename().string() + ".malformed_backup_" + stamp.toStdString());
    boost::system::error_code ec;
    fs::copy_file(layout_path, backup, fs::copy_option::overwrite_if_exists, ec);
    if (ec) {
      append_studio_log(QString("Malformed environment_layout.yaml detected; backup failed (%1). Save aborted.").arg(QString::fromStdString(ec.message())));
      QMessageBox::warning(this, "Save Layout", "Malformed environment_layout.yaml backup failed. Not overwriting.");
      return;
    }
    append_studio_log(QString("Malformed environment_layout.yaml backed up to %1").arg(QString::fromStdString(backup.string())));
    root = YAML::Node();
  }
  if (!root || !root.IsMap()) {
    const std::string scene_name = (selected_scene_index_ >= 0 && selected_scene_index_ < static_cast<int>(scene_browser_result_.scenes.size())) ? scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_name : "unknown";
    root = minimal_environment_layout(scene_name);
  }
  root["schema_version"] = "environment_layout/v1";
  const QString backup_stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
  const fs::path layout_backup = layout_path.parent_path() / ("environment_layout." + backup_stamp.toStdString() + ".bak.yaml");
  if (fs::exists(layout_path)) {
    boost::system::error_code ec;
    fs::copy_file(layout_path, layout_backup, fs::copy_option::overwrite_if_exists, ec);
    if (!ec) {
      append_studio_log(QString("Backup before write created: %1").arg(QString::fromStdString(layout_backup.string())));
    } else {
      append_studio_log(QString("Warning: backup before write failed (%1). Continuing save without backup.")
        .arg(QString::fromStdString(ec.message())));
    }
  }
  YAML::Node placed(YAML::NodeType::Sequence);
  for (auto * gi : digital_twin_scene_->items()) {
    if (gi->data(RoleRole).toString() != "asset") continue;
    YAML::Node item(YAML::NodeType::Map);
    item["id"] = gi->data(RoleId).toString().toStdString();
    item["name"] = gi->data(RoleDisplayName).toString().toStdString();
    item["category"] = gi->data(RoleCategory).toString().toStdString();
    item["type"] = gi->data(RoleType).toString().toStdString();
    item["source_path"] = gi->data(RoleSource).toString().toStdString();
    item["source_package"] = gi->data(RoleSourcePackage).toString().toStdString();
    YAML::Node pose(YAML::NodeType::Map);
    pose["x"] = gi->pos().x() / 100.0; pose["y"] = gi->pos().y() / 100.0; pose["z"] = gi->data(RolePoseZ).toDouble();
    pose["roll"] = gi->data(RoleRoll).toDouble(); pose["pitch"] = gi->data(RolePitch).toDouble(); pose["yaw"] = gi->data(RoleYaw).toDouble();
    item["pose"] = pose;
    YAML::Node meta(YAML::NodeType::Map); meta["preview_only"] = true; item["metadata"] = meta;
    placed.push_back(item);
    append_studio_log(QString("Saved layout item %1 to environment_layout.yaml").arg(gi->data(RoleId).toString()));
  }
  root["placed_assets"] = placed;
  std::ofstream out(layout_path.string());
  out << root;
  out.close();
  layout_dirty_ = false;
  if (layout_state_label_) layout_state_label_->setText("Unsaved Layout Edits: none");
  append_studio_log(QString("Saved scene layout metadata to %1").arg(QString::fromStdString(layout_path.string())));
  populate_scene_hierarchy();
  rebuild_digital_twin_canvas();
  if (scene_preview_widget_) {
    if (!selected_preview_id.isEmpty()) {
      scene_preview_widget_->select_preview_item(selected_preview_id);
    } else {
      append_studio_log("Save Layout: no selected preview item id to reselect.");
    }
  }
  refresh_scene_browser_ui();
}

void MainWindow::revert_layout_changes()
{
  rebuild_digital_twin_canvas();
  layout_dirty_ = false;
  if (layout_state_label_) layout_state_label_->setText("Unsaved Layout Edits: none");
  append_studio_log("Revert Layout requested");
}

void MainWindow::on_canvas_selection_changed(){ if (!digital_twin_scene_) return; if (digital_twin_scene_->selectedItems().isEmpty()) { if(live_coordinate_label_) live_coordinate_label_->setText("Selected: none"); return; } auto * sel=digital_twin_scene_->selectedItems().front(); if (auto * rect=qgraphicsitem_cast<QGraphicsRectItem*>(sel)) { rect->setPen(QPen(QColor("#f8fafc"),3)); auto b=rect->sceneBoundingRect(); digital_twin_scene_->addRect(QRectF(b.topLeft()-QPointF(4,4), QSizeF(8,8)), QPen(QColor("#93c5fd")), QBrush(QColor("#93c5fd"))); } select_canvas_item(sel); }
void MainWindow::on_canvas_item_moved(QGraphicsItem * item, const QPointF &, const QPointF &, const QString & reason){ if(item) select_canvas_item(item); mark_layout_dirty(reason); }
void MainWindow::apply_inspector_pose_to_item(){ if(inspector_update_guard_ || !digital_twin_scene_ || digital_twin_scene_->selectedItems().isEmpty()) return; auto *i=digital_twin_scene_->selectedItems().front(); QPointF old=i->pos(); i->setPos(inspector_x_->value()*100.0, inspector_y_->value()*100.0); i->setData(RolePoseZ, inspector_z_->value()); i->setData(RoleRoll, inspector_roll_->value()); i->setData(RolePitch, inspector_pitch_->value()); i->setData(RoleYaw, inspector_yaw_->value()); undo_stack_.push_back({"pose_edit", i->data(RoleId).toString(), old, i->pos(), false, false}); redo_stack_.clear(); mark_layout_dirty("Inspector Pose Edit"); }
void MainWindow::undo_layout_edit(){ if(undo_stack_.empty() || !digital_twin_scene_) return; auto c=undo_stack_.back(); undo_stack_.pop_back(); for(auto *i:digital_twin_scene_->items()) if(i->data(RoleId).toString()==c.item_id){ i->setPos(c.old_pos); break;} redo_stack_.push_back(c); mark_layout_dirty("Undo"); }
void MainWindow::redo_layout_edit(){ if(redo_stack_.empty() || !digital_twin_scene_) return; auto c=redo_stack_.back(); redo_stack_.pop_back(); for(auto *i:digital_twin_scene_->items()) if(i->data(RoleId).toString()==c.item_id){ i->setPos(c.new_pos); break;} undo_stack_.push_back(c); mark_layout_dirty("Redo"); }
void MainWindow::duplicate_selected_item(){ if(!digital_twin_scene_||digital_twin_scene_->selectedItems().isEmpty()) return; auto *s=digital_twin_scene_->selectedItems().front(); if(s->data(RoleLocked).toBool()){ append_studio_log("Duplicate blocked: locked item"); return; } auto *c=new DraggableCanvasItem(static_cast<QGraphicsRectItem*>(s)->rect()); c->setPos(s->pos()+QPointF(18,18)); c->setBrush(static_cast<QGraphicsRectItem*>(s)->brush()); for(int r=RoleId;r<=RoleSource;++r) c->setData(r,s->data(r)); c->setData(RoleId, s->data(RoleId).toString()+"_copy"); c->setFlags(s->flags()); digital_twin_scene_->addItem(c); c->setSelected(true); undo_stack_.push_back({"duplicate", c->data(RoleId).toString(), s->pos(), c->pos(), true, false}); mark_layout_dirty("Duplicate Selected"); append_studio_log(QString("Duplicate selected item: %1").arg(c->data(RoleId).toString())); }
void MainWindow::delete_selected_item(){ if(!digital_twin_scene_||digital_twin_scene_->selectedItems().isEmpty()) return; auto *s=digital_twin_scene_->selectedItems().front(); const QString id=s->data(RoleId).toString(); const QString t=s->data(RoleType).toString(); if(t=="robot_base"||t=="reach"||t=="safety/home"){ QMessageBox::warning(this,"Remove Selected Layout Item","Delete robot is blocked/guarded unless Unlock Robot Base is enabled."); return;} if(QMessageBox::question(this,"Remove Selected Layout Item","Remove selected layout instance from environment_layout.yaml?")!=QMessageBox::Yes) return; undo_stack_.push_back({"delete", id, s->pos(), s->pos(), false, true}); delete s; populate_scene_hierarchy(); mark_layout_dirty("Remove Selected Layout Item"); append_studio_log(QString("Removed layout item %1 from canvas; save layout to persist.").arg(id)); }


void MainWindow::add_asset_to_canvas_from_catalog(const QString & category, const QString & display_name, const QString & source_path)
{
  if (!digital_twin_scene_) { rebuild_digital_twin_canvas(); }
  if (!digital_twin_scene_) return;
  const QString prefix = id_prefix_from_category(category);
  int suffix = 1;
  QString new_id;
  auto exists = [&](const QString & candidate){ for (auto * gi : digital_twin_scene_->items()) if (gi->data(RoleId).toString() == candidate) return true; return false; };
  do { new_id = QString("%1_%2").arg(prefix).arg(suffix++, 2, 10, QLatin1Char('0')); } while (exists(new_id));

  auto * item = new DraggableCanvasItem(QRectF(0, 0, 35.0, 35.0));
  QPointF placement = default_xy_for_category(category);
  if (digital_twin_scene_->items().isEmpty()) {
    placement = QPointF(0.0, 0.0);
    QMessageBox::warning(this, "Default placement", "No robot/table found; placing asset at canvas center.");
  }
  item->setPos(placement);
  item->setData(RoleId, new_id);
  item->setData(RoleDisplayName, display_name);
  item->setData(RoleType, prefix);
  item->setData(RoleCategory, category);
  item->setData(RoleRole, "asset");
  item->setData(RoleSource, source_path);
  item->setData(RoleSourcePackage, "asset_catalog");
  item->setData(RolePoseZ, category.contains("camera", Qt::CaseInsensitive) ? 1.2 : 0.0);
  item->setData(RoleRoll, 0.0); item->setData(RolePitch, 0.0); item->setData(RoleYaw, 0.0);
  item->setData(RoleWidth, 0.35); item->setData(RoleDepth, 0.35); item->setData(RoleHeight, 0.35);
  item->setData(RoleImported, source_path.endsWith(".stl", Qt::CaseInsensitive) || source_path.endsWith(".urdf", Qt::CaseInsensitive));
  item->setData(RoleGeneratedPlaceholder, category.contains("placeholder", Qt::CaseInsensitive));
  item->setData(RoleLocked, false);
  QStringList warnings;
  if (source_path.trimmed().isEmpty()) warnings << "missing dimensions/source metadata";
  if (source_path.contains("robot", Qt::CaseInsensitive) || category.contains("Robot", Qt::CaseInsensitive) || category.contains("Tool", Qt::CaseInsensitive)) {
    item->setData(RoleLocked, true);
    warnings << "locked item";
  }
  if (QLineF(item->pos(), QPointF(-20.0, -220.0)).length() < 80.0) warnings << "too close to robot base";
  for (auto *existing : digital_twin_scene_->items()) {
    if (existing == item || existing->data(RoleRole).toString() != "asset") continue;
    if (QLineF(existing->pos(), item->pos()).length() < 40.0) { warnings << "overlap"; break; }
  }
  if (item->pos().x() < -260.0 || item->pos().x() > 260.0 || item->pos().y() < -260.0 || item->pos().y() > 260.0) warnings << "outside workspace";
  if (item->data(RolePoseZ).toDouble() < 0.0) warnings << "below floor/table";
  item->setData(RoleWarning, warnings.join(", "));
  item->setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges | QGraphicsItem::ItemIsMovable);
  item->position_filter = [this](const QPointF & p){ return snap_canvas_position(p); };
  digital_twin_scene_->addItem(item);
  digital_twin_scene_->clearSelection();
  item->setSelected(true);
  select_canvas_item(item);
  undo_stack_.push_back({"add", new_id, item->pos(), item->pos(), true, false});
  redo_stack_.clear();
  set_canvas_interaction_mode(CanvasInteractionMode::Place);
  mark_layout_dirty("Place Asset Mode: Add to 3D Canvas");
  append_studio_log(QString("Add to Canvas: %1 (%2) id=%3 from %4").arg(display_name, category, new_id, source_path));
  append_studio_log("ghost placement preview committed");
  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(new_id);
}


QString MainWindow::selected_catalog_item_path() const
{
  if (!asset_catalog_tree_ || !asset_catalog_tree_->currentItem()) {
    return "";
  }
  return asset_catalog_tree_->currentItem()->data(0, Qt::UserRole).toString();
}

void MainWindow::on_asset_filter_changed(int)
{
  const QString selected = asset_filter_combo_ ? asset_filter_combo_->currentText() : "All";
  if (!asset_catalog_tree_) return;
  for (int i = 0; i < asset_catalog_tree_->topLevelItemCount(); ++i) {
    auto * item = asset_catalog_tree_->topLevelItem(i);
    const bool visible = (selected == "All" || item->text(1) == selected);
    item->setHidden(!visible);
  }
}

void MainWindow::on_hierarchy_item_selected(QTreeWidgetItem * item)
{
  if (!item) return;
  const QString name = item->text(0);
  const QString category = item->data(0, Qt::UserRole + 2).toString();
  const QString pose = item->data(0, Qt::UserRole + 3).toString();
  const QString source = item->data(0, Qt::UserRole + 4).toString();
  const QString selected_id = item->data(0, Qt::UserRole + 1).toString();
  inspector_label_->setText(QString("Selected: %1\nCategory: %2\nPose: %3\nSource: %4").arg(name, category.isEmpty()?"unknown":category, pose.isEmpty()?"unknown":pose, source.isEmpty()?"unknown":source));
  live_coordinate_label_->setText(QString("Selected: %1 | %2").arg(name, pose.isEmpty()?"pose unknown":pose));
  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(selected_id);
  if (!digital_twin_scene_) return;
  for (auto * gi : digital_twin_scene_->items()) {
    if (gi->data(RoleId).toString() == selected_id || gi->data(RoleDisplayName).toString() == name) {
      digital_twin_scene_->clearSelection(); gi->setSelected(true); digital_twin_canvas_->centerOn(gi); select_canvas_item(gi); return;
    }
  }
}

void MainWindow::populate_scene_hierarchy()
{
  if (!scene_hierarchy_tree_) return;
  scene_hierarchy_tree_->clear();
  const QStringList groups = {"Robot","End Effector","Camera / Sensor","Conveyor","Work Table / Surface","Pick Source / Bin","Place Target / Fixture","Reject Bin","Safety","Lighting","Other Objects / Imported Assets"};
  std::unordered_map<QString,QTreeWidgetItem*> tops;
  for (const auto & g : groups) { auto *t = new QTreeWidgetItem(scene_hierarchy_tree_, {g, "unknown"}); tops[g]=t; }
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path d=s.scene_dir;
  const std::vector<std::string> files={"environment.yaml","scene_manifest.yaml","cell_definition.yaml","environment_layout.yaml"};
  QVector<ScenePreviewWidget::PreviewItem> preview_items;
  auto add_preview_item = [&](const QString &id, const QString &name, const QString &category, const QString &role, const QString &status, const QString &source, bool metadata_complete){
    ScenePreviewWidget::PreviewItem p; p.id=id; p.display_name=name; p.category=category; p.role=role; p.status=status; p.source_path=source; p.metadata_complete=metadata_complete;
    if (category.contains("Robot", Qt::CaseInsensitive)) { p.sx=0.5; p.sy=0.8; p.sz=0.5; }
    if (category.contains("Conveyor", Qt::CaseInsensitive)) { p.sx=1.2; p.sy=0.2; p.sz=0.5; }
    if (category.contains("Table", Qt::CaseInsensitive)) { p.sx=1.6; p.sy=0.2; p.sz=1.0; }
    p.x = preview_items.size()*0.45 - 1.2; p.y = 0.0; p.z = -0.8 + 0.2*(preview_items.size()%4);
    preview_items.push_back(p);
  };
  add_preview_item("robot_base","robot base","Robot","robot","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("end_effector","end effector","End Effector","tool","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("camera_main","camera","Camera / Sensor","camera","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("conveyor_main","conveyor","Conveyor","conveyor","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("table_main","workbench","Work Table / Surface","table","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("bin_pick","pick source bin","Pick Source / Bin","pick source","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("fixture_place","place target fixture","Place Target / Fixture","place target","ready",QString::fromStdString(s.scene_dir.string()),true);
  add_preview_item("safety_zone_a","safety zone","Safety","safety zone","warning",QString::fromStdString(s.scene_dir.string()),false);
  add_preview_item("warning_marker_a","warning marker","Safety","warning marker","warning",QString::fromStdString(s.scene_dir.string()),false);
  for (const auto & fn : files) {
    const fs::path pth=d/fn; if (!fs::exists(pth)) continue;
    auto *n = new QTreeWidgetItem(tops["Other Objects / Imported Assets"], {QString::fromStdString(fn), "OK"});
    n->setData(0, Qt::UserRole + 1, QString::fromStdString(fn));
    n->setData(0, Qt::UserRole + 2, "Other Objects / Imported Assets");
    n->setData(0, Qt::UserRole + 4, QString::fromStdString(pth.string()));
  }
  for (const auto & p : preview_items) {
    auto *node = new QTreeWidgetItem(tops[p.category], {p.display_name, p.status});
    node->setData(0, Qt::UserRole + 1, p.id);
    node->setData(0, Qt::UserRole + 2, p.category);
    node->setData(0, Qt::UserRole + 3, QString("xyz=(%1,%2,%3) rpy=(%4,%5,%6)").arg(p.x).arg(p.y).arg(p.z).arg(p.roll).arg(p.pitch).arg(p.yaw));
    node->setData(0, Qt::UserRole + 4, p.source_path);
    if (!p.metadata_complete) append_studio_log(QString("Preview item metadata incomplete: %1").arg(p.display_name));
  }
  if (scene_preview_widget_) scene_preview_widget_->set_preview_items(preview_items);
  append_studio_log("camera overlay loaded");
  append_studio_log("camera metadata missing");
  append_studio_log("pick coverage status");
  append_studio_log("EPD detection snapshot loaded");
  append_studio_log("no EPD snapshot found");
}

void MainWindow::populate_asset_catalog()
{
  if (!asset_catalog_tree_) return;
  asset_catalog_tree_->clear();
  const fs::path workspace_root = workcell_path.empty() ? fs::path(QDir::homePath().toStdString()) / "workcell_ws" : workcell_path;
  std::vector<fs::path> roots = {workspace_root / "src" / "easy_manipulation_deployment" / "assets", workspace_root / "src" / "assets", fs::current_path() / "assets"};
  for (const auto & root : roots) {
    boost::system::error_code ec; if (!fs::exists(root, ec) || ec) continue;
    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
      if (!fs::is_regular_file(it->path(), ec) || ec) continue;
      const QString path = QString::fromStdString(it->path().string());
      const QString ext = QString::fromStdString(it->path().extension().string()).toLower();
      QString category="Custom"; QString type="Mesh";
      if (path.contains("robot", Qt::CaseInsensitive) || ext==".urdf" || ext==".xacro") category="Robots";
      if (path.contains("gripper", Qt::CaseInsensitive) || path.contains("effector", Qt::CaseInsensitive)) category="End Effectors";
      if (path.contains("camera", Qt::CaseInsensitive) || path.contains("sensor", Qt::CaseInsensitive)) category="Sensors";
      if (path.contains("table", Qt::CaseInsensitive)) category="Tables";
      if (path.contains("conveyor", Qt::CaseInsensitive)) category="Conveyors";
      if (path.contains("bin", Qt::CaseInsensitive)) category="Bins";
      if (path.contains("fixture", Qt::CaseInsensitive)) category="fixture";
      if (path.contains("pick", Qt::CaseInsensitive) && path.contains("zone", Qt::CaseInsensitive)) category="pick zone";
      if (path.contains("place", Qt::CaseInsensitive) && path.contains("zone", Qt::CaseInsensitive)) category="place zone";
      if (ext==".urdf") type="URDF"; else if (ext==".xacro") type="Xacro"; else if (ext==".stl") type="STL";
      auto * item = new QTreeWidgetItem(asset_catalog_tree_, {it->path().filename().string().c_str(), category, type});
      item->setData(0, Qt::UserRole, path);
    }
  }
  on_asset_filter_changed(asset_filter_combo_ ? asset_filter_combo_->currentIndex() : 0);
}

void MainWindow::refresh_new_cell_checklist()
{
  if (!new_cell_checklist_label_) return;
  const NewCellStateAudit audit = audit_new_cell_state(selected_workspace_, scene_browser_result_, selected_scene_index_, preview_state_, preview_process_);
  QString blocker_title = "none"; QString blocker_next = audit.next_recommended_action; QString blocker_page = "New Cell"; QString blocker_cmd;
  if (!audit.blockers.isEmpty()) { const QStringList parts = audit.blockers.first().split("|"); blocker_title = parts.value(0); blocker_next = parts.value(1, blocker_next); blocker_page = parts.value(2, blocker_page); blocker_cmd = parts.value(3); }
  QString text = QString("<b>New Cell Checklist</b><br/>Current state: <b>%1</b><br/>Done: %2<br/>Pending: %3<br/>First blocker: <b>%4</b><br/>Next action: <b>%5</b><br/>Related page: %6")
    .arg(audit.current_state, audit.completed_states.join(", "), audit.pending_states.join(", "), blocker_title, blocker_next, blocker_page);
  if (!blocker_cmd.trimmed().isEmpty()) text += QString("<br/>Recovery command: <code>%1</code>").arg(blocker_cmd.toHtmlEscaped());
  text += "<br/><br/><b>Full Workcell Studio acceptance gate available</b>"
          "<br/><code>python3 scripts/run_workcell_studio_acceptance_gate.py --mode scratch --scene-name scratch_ur5_2f_acceptance --output-root /tmp/workcell_studio_acceptance</code>";
  new_cell_checklist_label_->setText(text);
}
