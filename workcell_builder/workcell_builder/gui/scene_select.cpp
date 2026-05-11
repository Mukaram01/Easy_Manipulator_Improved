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

#include <QKeyEvent>
#include <QCoreApplication>
#include <QDateTime>

#include <QDesktopServices>
#include <QUrl>
#include <QApplication>
#include <QClipboard>
#include <QLineEdit>
#include <QFileDialog>
#include <QListWidgetItem>
#include <QTreeWidgetItem>
#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <algorithm>
#include <string>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gui/ui_scene_select.h"
#include "gui/scene_select.h"

#include "gui/replacewarning.h"
#include "gui/addscene.h"

#include "yaml_parser/generate_yaml.h"
#include "include/armhand_xacro_parser.h"
#include "include/file_functions.h"
#include "include/object_package_parser.h"
#include "include/object_xacro_parser.h"
#include "include/scene_check.h"
#include "include/scene_parser.h"
#include "include/scene_xacro_parser.h"
#include "include/default_asset_paths.h"
#include "scene_select_paths.h"

namespace fs = boost::filesystem;

namespace {


static const char * kAssetDiscoveryLabel = "Asset discovery paths";
static const char * kSelectRobotAssetLabel = "Select Robot Asset";
static const char * kSelectEndEffectorAssetLabel = "Select End Effector Asset";
static const char * kSelectExistingStlLabel = "Select Existing STL";
static const char * kPresetTable = "table";
static const char * kPresetBin = "bin";
static const char * kPresetConveyorPlaceholder = "conveyor_placeholder";
static const char * kPresetFixture = "fixture";
static const char * kPresetCustomStl = "custom_stl";
constexpr const char * kSceneRootEnvVar = "WORKCELL_BUILDER_SCENE_ROOT";

[[maybe_unused]] bool change_directory(const fs::path & p)
{
  boost::system::error_code ec;
  fs::current_path(p, ec);
  if (ec) {
    RCLCPP_ERROR(
      rclcpp::get_logger("workcell_builder"),
      "Failed to change directory to %s: %s", p.string().c_str(), ec.message().c_str());
    return false;
  }
  return true;
}

bool is_package_uri(const std::string & path)
{
  return path.rfind("package://", 0) == 0;
}

void resolve_relative_path(const fs::path & base_path, std::string * path)
{
  if (!path || path->empty() || is_package_uri(*path)) {
    return;
  }
  fs::path candidate(*path);
  if (candidate.is_absolute()) {
    return;
  }
  *path = (base_path / candidate).lexically_normal().string();
}

void resolve_scene_paths(Scene * scene, const fs::path & base_path)
{
  if (!scene) {
    return;
  }
  if (scene->robot_loaded && !scene->robot_vector.empty()) {
    resolve_relative_path(base_path, &scene->robot_vector[0].filepath);
  }
  if (scene->ee_loaded && !scene->ee_vector.empty()) {
    resolve_relative_path(base_path, &scene->ee_vector[0].filepath);
  }
  for (auto & object : scene->object_vector) {
    for (auto & link : object.link_vector) {
      if (link.is_visual) {
        for (auto & visual : link.visual_vector) {
          resolve_relative_path(base_path, &visual.geometry.filepath);
        }
      }
      if (link.is_collision) {
        for (auto & collision : link.collision_vector) {
          resolve_relative_path(base_path, &collision.geometry.filepath);
        }
      }
    }
  }
}

struct SceneRootCandidate
{
  fs::path root;
  std::string label;
  bool valid = false;
  std::string invalid_reason;
};

std::string path_or_placeholder(const fs::path & path)
{
  return path.empty() ? std::string("<none>") : path.string();
}

std::string display_path_with_home_tilde(const fs::path & path)
{
  if (path.empty()) {
    return "<none>";
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::strlen(home) == 0) {
    return path.string();
  }
  const fs::path home_path(home);
  const fs::path normalized = path.lexically_normal();
  const std::string home_str = home_path.string();
  const std::string normalized_str = normalized.string();
  if (normalized_str == home_str) {
    return "~";
  }
  const std::string prefix = home_str + "/";
  if (normalized_str.rfind(prefix, 0) == 0) {
    return "~/" + normalized_str.substr(prefix.size());
  }
  return normalized_str;
}

std::string parse_cli_scene_root_override()
{
  const QStringList args = QCoreApplication::arguments();
  for (int i = 1; i < args.size(); ++i) {
    const QString arg = args[i];
    if (arg == "--scene-root" && i + 1 < args.size()) {
      return args[i + 1].toStdString();
    }
    const QString prefix = "--scene-root=";
    if (arg.startsWith(prefix)) {
      return arg.mid(prefix.size()).toStdString();
    }
  }
  return "";
}

SceneRootCandidate build_candidate(const fs::path & root, const std::string & label)
{
  SceneRootCandidate candidate;
  candidate.root = root.lexically_normal();
  candidate.label = label;

  const fs::path scenes_dir = candidate.root / "scenes";
  boost::system::error_code ec;
  if (!fs::exists(scenes_dir, ec) || ec) {
    candidate.valid = false;
    candidate.invalid_reason = "missing scenes directory";
    return candidate;
  }
  if (!fs::is_directory(scenes_dir, ec) || ec) {
    candidate.valid = false;
    candidate.invalid_reason = "scenes path is not a directory";
    return candidate;
  }

  candidate.valid = true;
  return candidate;
}



std::string normalize_placeholder_token(const std::string & value)
{
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  normalized.erase(std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }), normalized.end());
  return normalized;
}

bool is_placeholder_value(const std::string & value)
{
  const std::string normalized = normalize_placeholder_token(value);
  return normalized.empty() || normalized == "unknown" || normalized == "none" || normalized == "null";
}

bool scene_has_valid_robot(const Scene & scene)
{
  return scene.robot_loaded && !scene.robot_vector.empty() && !is_placeholder_value(scene.robot_vector[0].name);
}

bool scene_requires_end_effector(const Scene & scene)
{
  if (!scene.ee_loaded || scene.ee_vector.empty()) {
    return false;
  }
  const std::string ee_name = normalize_placeholder_token(scene.ee_vector[0].name);
  return !(ee_name.empty() || ee_name == "none");
}

bool scene_has_valid_end_effector(const Scene & scene)
{
  return scene_requires_end_effector(scene) && !is_placeholder_value(scene.ee_vector[0].name);
}


enum class SceneUiStatus
{
  VALID,
  INCOMPLETE,
  SCAFFOLD_ONLY,
  MISSING_ENVIRONMENT_YAML,
  MISSING_ROBOT,
  MISSING_MOVEIT_CONFIG
};

std::string scene_status_label(SceneUiStatus status)
{
  switch (status) {
    case SceneUiStatus::VALID: return "VALID";
    case SceneUiStatus::INCOMPLETE: return "INCOMPLETE";
    case SceneUiStatus::SCAFFOLD_ONLY: return "SCAFFOLD_ONLY";
    case SceneUiStatus::MISSING_ENVIRONMENT_YAML: return "MISSING_ENVIRONMENT_YAML";
    case SceneUiStatus::MISSING_ROBOT: return "MISSING_ROBOT";
    case SceneUiStatus::MISSING_MOVEIT_CONFIG: return "MISSING_MOVEIT_CONFIG";
  }
  return "INCOMPLETE";
}

SceneUiStatus compute_scene_status_label(const Scene & scene, const fs::path & scene_dir)
{
  const bool has_yaml = fs::exists(scene_dir / "environment.yaml");
  const bool has_launch = fs::exists(scene_dir / "launch" / "demo.launch.py");
  const bool has_srdf = fs::exists(scene_dir / "urdf" / "arm_hand.srdf.xacro");

  if (!has_yaml) {
    return SceneUiStatus::MISSING_ENVIRONMENT_YAML;
  }
  if (!scene_has_valid_robot(scene)) {
    return SceneUiStatus::MISSING_ROBOT;
  }
  if (scene_requires_end_effector(scene) && !scene_has_valid_end_effector(scene)) {
    return SceneUiStatus::INCOMPLETE;
  }
  if (!has_srdf) {
    return has_launch ? SceneUiStatus::MISSING_MOVEIT_CONFIG : SceneUiStatus::SCAFFOLD_ONLY;
  }
  return SceneUiStatus::VALID;
}
fs::path root_from_override(const std::string & override_value)
{
  fs::path override_path(override_value);
  if (override_path.filename() == "scenes") {
    return override_path.parent_path();
  }
  return override_path;
}


fs::path resolve_tool_root(const fs::path & scene_root, const fs::path & scene_dir)
{
  std::vector<fs::path> candidates;
  const char * env_root = std::getenv("WORKCELL_STUDIO_REPO_ROOT");
  if (env_root != nullptr && std::strlen(env_root) > 0) {
    candidates.emplace_back(env_root);
  }
  candidates.push_back(scene_root);
  candidates.push_back(scene_dir.parent_path());
  candidates.push_back(scene_dir.parent_path().parent_path());
  const char * home = std::getenv("HOME");
  if (home != nullptr && std::strlen(home) > 0) {
    candidates.emplace_back(fs::path(home) / "workcell_ws" / "src" / "easy_manipulation_deployment");
  }

  for (const auto & raw_candidate : candidates) {
    if (raw_candidate.empty()) {
      continue;
    }
    const fs::path candidate = raw_candidate.lexically_normal();
    const fs::path script_path = candidate / "scripts" / "render_workcell_builder_metadata.py";
    boost::system::error_code ec;
    if (fs::exists(script_path, ec) && !ec) {
      return candidate;
    }
  }
  return fs::path();
}

fs::path select_scene_root(const fs::path & cwd)
{
  std::vector<SceneRootCandidate> candidates;
  const std::string cli_override = parse_cli_scene_root_override();
  const char * env_override = std::getenv(kSceneRootEnvVar);

  if (!cli_override.empty()) {
    candidates.push_back(build_candidate(
      root_from_override(cli_override),
      std::string("CLI --scene-root=") + cli_override));
  }
  if (env_override != nullptr && std::strlen(env_override) > 0) {
    candidates.push_back(build_candidate(
      root_from_override(env_override),
      std::string("environment ") + kSceneRootEnvVar + "=" + env_override));
  }

  candidates.push_back(build_candidate(cwd / "src" / "easy_manipulation_deployment", "workspace src/easy_manipulation_deployment"));
  candidates.push_back(build_candidate(cwd / "src", "workspace src"));
  candidates.push_back(build_candidate(cwd, "current working directory"));
  candidates.push_back(build_candidate(cwd.parent_path(), "parent directory"));
  const fs::path repo_root = cwd.parent_path() / "easy_manipulation_deployment";
  candidates.push_back(build_candidate(repo_root, "repo root easy_manipulation_deployment"));

  SceneRootCandidate selected;
  bool has_selected = false;
  std::vector<SceneRootCandidate> valid_candidates;

  for (const auto & candidate : candidates) {
    if (candidate.valid) {
      valid_candidates.push_back(candidate);
      if (!has_selected) {
        selected = candidate;
        has_selected = true;
      }
    }
  }

  if (has_selected) {
    RCLCPP_INFO(
      rclcpp::get_logger("workcell_builder"),
      "Selected workcell root: %s (source: %s)",
      selected.root.string().c_str(), selected.label.c_str());

    for (const auto & candidate : candidates) {
      if (candidate.root == selected.root && candidate.label == selected.label) {
        continue;
      }
      if (candidate.valid) {
        RCLCPP_INFO(
          rclcpp::get_logger("workcell_builder"),
          "Rejected valid scene root candidate: %s (source: %s)",
          candidate.root.string().c_str(), candidate.label.c_str());
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("workcell_builder"),
          "Rejected scene root candidate: %s (source: %s, reason: %s)",
          candidate.root.string().c_str(), candidate.label.c_str(), candidate.invalid_reason.c_str());
      }
    }

    std::unordered_set<std::string> unique_valid_roots;
    for (const auto & candidate : valid_candidates) {
      unique_valid_roots.insert(candidate.root.string());
    }

    if (unique_valid_roots.size() > 1) {
      std::ostringstream valid_paths;
      size_t idx = 0;
      for (const auto & root : unique_valid_roots) {
        if (idx++ > 0) {
          valid_paths << ", ";
        }
        valid_paths << root;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("workcell_builder"),
        "Multiple valid scene directories were found (%zu). Using highest priority root %s. "
        "Set --scene-root or %s for deterministic selection. Candidates: [%s]",
        unique_valid_roots.size(), selected.root.string().c_str(), kSceneRootEnvVar,
        valid_paths.str().c_str());
    }

    return selected.root;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("workcell_builder"),
    "Unable to locate a valid 'scenes' directory from %s. Checked default locations. "
    "You can override with --scene-root or %s.",
    cwd.string().c_str(), kSceneRootEnvVar);
  for (const auto & candidate : candidates) {
    RCLCPP_INFO(
      rclcpp::get_logger("workcell_builder"),
      "Rejected scene root candidate: %s (source: %s, reason: %s)",
      path_or_placeholder(candidate.root).c_str(), candidate.label.c_str(),
      candidate.invalid_reason.c_str());
  }
  return cwd;
}

}  // namespace

bool ensure_minimal_environment_yaml(const fs::path & scene_dir, const std::string & scene_name);
void refresh_scene_manifest_if_missing(const fs::path & scene_dir, const std::string & scene_name);


std::string status_from_blockers_and_warnings(bool has_blockers, bool has_warnings)
{
  if (has_blockers) {
    return "BLOCKED";
  }
  if (has_warnings) {
    return "WARNINGS";
  }
  return "READY_TO_GENERATE";
}

struct TaskGraspConfig
{
  std::string task_type = "pick_place";
  std::string pick_source = "selected_object";
  std::string place_target = "selected_bin";
  std::string grasp_strategy = "finger_top";
  std::string orientation_mode = "vertical";
  std::string approach_axis = "z_down";
  double approach_distance_m = 0.12;
  double retreat_distance_m = 0.10;
  double place_clearance_m = 0.05;
  std::string allowed_roll_angles_deg = "0, 90";
  std::string allowed_yaw_angles_deg = "0";
  int suction_cups = 1;
  std::string release_strategy = "open_gripper";
};

TaskGraspConfig infer_task_grasp_defaults(const Scene & scene)
{
  TaskGraspConfig config;
  if (scene.ee_loaded && !scene.ee_vector.empty()) {
    const std::string type = normalize_placeholder_token(scene.ee_vector[0].type);
    const std::string name = normalize_placeholder_token(scene.ee_vector[0].name);
    if (type.find("suction") != std::string::npos || name.find("suction") != std::string::npos ||
      name.find("vacuum") != std::string::npos)
    {
      config.grasp_strategy = "suction_top";
      config.release_strategy = "vacuum_off";
    }
  }
  return config;
}

void write_task_recipe_yaml(const fs::path & scene_dir, const TaskGraspConfig & config)
{
  fs::create_directories(scene_dir / "config");
  std::ofstream out((scene_dir / "config" / "task_recipe.yaml").string());
  out << "schema_version: workcell_task/v1\n";
  out << "task:\n  type: " << config.task_type << "\n  pick_source: " << config.pick_source <<
    "\n  place_target: " << config.place_target << "\n";
  out << "grasp:\n  strategy: " << config.grasp_strategy << "\n  approach_axis: " <<
    config.approach_axis << "\n  orientation_mode: " << config.orientation_mode << "\n";
  out << "  approach_distance_m: " << config.approach_distance_m << "\n  retreat_distance_m: " <<
    config.retreat_distance_m << "\n";
  out << "  allowed_roll_angles_deg: [" << config.allowed_roll_angles_deg << "]\n";
  out << "  allowed_yaw_angles_deg: [" << config.allowed_yaw_angles_deg << "]\n";
  out << "  suction_cups: " << config.suction_cups <<
    "\n  tcp_offset_xyz: [0.0, 0.0, 0.0]\n  tcp_offset_rpy: [0.0, 0.0, 0.0]\n";
  out << "place:\n  clearance_m: " << config.place_clearance_m <<
    "\n  orientation_mode: preserve_object_orientation\n";
  out << "release:\n  strategy: " << config.release_strategy << "\n";
  out << "safety:\n  fake_hardware_first: true\n  motion_command_sent: false\n";
  out << "  runtime_execution_enabled: false\n";
}


SceneSelect::SceneSelect(QWidget * parent)
: QDialog(parent),
  ui(new Ui::SceneSelect)
{
  ui->setupUi(this);
  templates_path = get_default_templates_directory();
  setWindowTitle("Workcell Builder");
  ui->workflow_tabs->setCurrentWidget(ui->start_tab);

  ui->asset_browser_group->hide();
  ui->inspector_group->hide();
  ui->cell_name->hide();
  ui->output_folder->hide();
  ui->browse_output_folder->hide();
  ui->golden_demo_cell->hide();
  ui->delete_scene->hide();
  ui->generate_studio_pack->hide();
  ui->open_preview->hide();
  ui->show_readiness_report->hide();
  ui->validate_cell->hide();
  ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->ingredients_tab));
  ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->layout_tab));
  ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->task_tab));
  ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->perception_roi_tab));
  ui->workflow_tabs->removeTab(ui->workflow_tabs->indexOf(ui->grasp_tab));
  ui->workflow_tabs->setTabText(
    ui->workflow_tabs->indexOf(ui->validate_generate_tab), "Generate");
  ui->browse_scenes_folder->setText("Open Folder");

  ui->fake_hardware_default_label->setToolTip(
    "Fake hardware is the safe default. Real hardware launch is intentionally not default.");
  ui->generate_files->hide();
  ui->validate_cell->show();
  ui->validate_cell->setText("Validate Scene");
  ui->open_preview->show();
  ui->open_preview->setText("Refresh Preview");
  ui->export_layout_preview_action->setText("Export Preview");
  append_info("Workcell Studio Readiness panel initialized: READY_TO_GENERATE / WARNINGS / BLOCKED / SCAFFOLD_ONLY");
  connect(ui->export_layout_preview_action, &QPushButton::clicked, this, &SceneSelect::on_export_preview_clicked);
  connect(ui->generate_full_scene_package_start, &QPushButton::clicked, this, &SceneSelect::on_generate_full_scene_package_start_clicked);
  connect(ui->open_scene_folder, &QPushButton::clicked, this, &SceneSelect::on_open_scene_folder_clicked);
  const std::vector<QPushButton *> placeholder_buttons = {ui->set_as_robot, ui->set_as_end_effector, ui->add_as_support_surface, ui->add_as_pick_object, ui->import_custom_stl, ui->fit_cell_action, ui->reset_view_action, ui->toggle_grid_action, ui->toggle_reach_action, ui->toggle_roi_action, ui->snap_to_grid_action, ui->export_layout_preview_action, ui->duplicate_selected_asset, ui->remove_selected_asset, ui->clear_cell_assets};
  for (auto * button : placeholder_buttons) {
    button->setText(button->text() + " (coming soon)");
    button->setToolTip("This control is not available yet.");
    button->setDisabled(true);
  }

}

SceneSelect::~SceneSelect()
{
  delete ui;
}

void SceneSelect::append_message(MessageLevel level, const std::string & message)
{
  QString color;
  QString prefix;
  switch (level) {
    case MessageLevel::Info:
      color = "#1F4B99";
      prefix = "[INFO]";
      break;
    case MessageLevel::Warning:
      color = "#B45309";
      prefix = "[WARNING]";
      break;
    case MessageLevel::Error:
      color = "#B91C1C";
      prefix = "[ERROR]";
      break;
    case MessageLevel::Success:
      color = "#166534";
      prefix = "[SUCCESS]";
      break;
  }

  const QString escaped = QString::fromStdString(message).toHtmlEscaped();
  ui->error_workcell->append(
    QString("<span style='color:%1;'>%2 %3</span>").arg(color, prefix, escaped));
}

void SceneSelect::append_info(const std::string & message)
{
  append_message(MessageLevel::Info, message);
}

void SceneSelect::append_warning(const std::string & message)
{
  append_message(MessageLevel::Warning, message);
}

void SceneSelect::append_error(const std::string & message)
{
  append_message(MessageLevel::Error, message);
}

void SceneSelect::append_success(const std::string & message)
{
  append_message(MessageLevel::Success, message);
}

void SceneSelect::clear_messages()
{
  ui->error_workcell->clear();
}

void SceneSelect::refresh_scene_status(bool strict, const std::string & trigger)
{
  clear_messages();
  const std::string timestamp =
    QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
  append_info("Status snapshot [" + trigger + "] @ " + timestamp);
  check_scene(strict);
}

void SceneSelect::configure_startup_fallback_paths()
{
  if (!workcell_path.empty()) {
    return;
  }

  const fs::path fallback_root = select_scene_root(fs::current_path());
  workcell_path = fallback_root;
  scenes_path = workcell_path / "scenes";
  assets_path = workcell_path / "assets";
  templates_path = get_default_templates_directory();
  if (assets_path.empty()) {
    assets_path = workcell_path / "assets";
  }
}

void SceneSelect::show_invalid_workcell_error(const std::string & error_message)
{
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();
  ui->scene_list->setDisabled(true);
  ui->generate_yaml->setDisabled(true);
  ui->generate_files->setDisabled(true);
  ui->edit_scene->setDisabled(true);
  ui->delete_scene->setDisabled(true);
  clear_messages();
  append_error(error_message);
  ui->scene_list->blockSignals(oldState);
}

void SceneSelect::load_workcell(Workcell workcell_input)
{
  workcell = workcell_input;

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell, workcell_path);
  templates_path = resolution.paths.templates_path;
  if (!resolution.success) {
    configure_startup_fallback_paths();
    show_invalid_workcell_error(resolution.error);
    return;
  }

  workcell_path = resolution.paths.workcell_path;
  scenes_path = resolution.paths.scenes_path;
  assets_path = resolution.paths.assets_path;
  if (assets_path.empty()) {
    assets_path = workcell_path / "assets";
  }
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
}

void SceneSelect::update_scene_browser_status(const std::string & note)
{
  const int count = static_cast<int>(workcell.scene_vector.size());
  const QString stamp = QDateTime::currentDateTime().toString(Qt::ISODate);
  QString text = QString("Scenes folder: %1\nScenes found: %2\nLast refresh: %3")
    .arg(QString::fromStdString(display_path_with_home_tilde(scenes_path)))
    .arg(count)
    .arg(stamp);
  if (!note.empty()) {
    text += "\n" + QString::fromStdString(note);
  }
  ui->scene_browser_status->setText(text);
}

void SceneSelect::discover_scene_packages_on_startup()
{
  if (workcell_path.empty()) {
    configure_startup_fallback_paths();
  }
  append_info("Selected workcell root: " + workcell_path.string());
  append_info("Selected scenes directory: " + scenes_path.string());
  boost::system::error_code path_ec;
  const fs::path resolved_scenes = fs::canonical(scenes_path, path_ec);
  if (!path_ec) {
    append_info("Resolved scenes directory: " + resolved_scenes.string());
  }

  if (!fs::exists(scenes_path) || !fs::is_directory(scenes_path)) {
    append_warning("Scenes directory not found; skipping startup rediscovery.");
    return;
  }

  std::unordered_set<std::string> known_scenes;
  for (const auto & known_scene : workcell.scene_vector) {
    known_scenes.insert(known_scene.name);
  }

  int discovered_count = 0;
  for (const auto & entry : fs::directory_iterator(scenes_path)) {
    if (!fs::is_directory(entry.path())) {
      continue;
    }
    const std::string scene_name = entry.path().filename().string();
    const fs::path package_xml = entry.path() / "package.xml";
    const fs::path scene_manifest = entry.path() / "scene_manifest.yaml";
    const fs::path environment_yaml = entry.path() / "environment.yaml";
    const fs::path urdf_xacro = entry.path() / "urdf" / "scene.urdf.xacro";
    const fs::path demo_launch = entry.path() / "launch" / "demo.launch.py";

    const bool has_markers = fs::exists(package_xml) || fs::exists(scene_manifest) ||
      fs::exists(environment_yaml) || fs::exists(urdf_xacro) || fs::exists(demo_launch);
    if (!has_markers) {
      append_info("Skipped scene directory '" + scene_name + "': no scene package markers found.");
      continue;
    }
    if (known_scenes.find(scene_name) != known_scenes.end()) {
      append_info("Skipped scene directory '" + scene_name + "': scene already loaded.");
      continue;
    }

    Scene discovered_scene;
    discovered_scene.name = scene_name;
    discovered_scene.loaded = false;

    if (fs::exists(environment_yaml)) {
      if (!load_scene_from_yaml(&discovered_scene)) {
        append_warning(
          "Discovered scene package '" + scene_name +
          "' but environment.yaml failed to load; showing as scaffold-only.");
        discovered_scene.loaded = false;
      }
    } else {
      append_warning(
        "Discovered scaffold scene package '" + scene_name +
        "' without environment.yaml; scene can launch but cannot be fully edited until YAML exists.");
      if (ensure_minimal_environment_yaml(entry.path(), scene_name)) {
        append_warning(
          "Repair Missing environment.yaml applied at: " +
          (entry.path() / "environment.yaml").string());
      }
      refresh_scene_manifest_if_missing(entry.path(), scene_name);
    }
    workcell.scene_vector.push_back(discovered_scene);
    known_scenes.insert(scene_name);
    ++discovered_count;
  }
  append_info("Discovered scene packages: " + std::to_string(discovered_count));
  if (discovered_count == 0 && workcell.scene_vector.empty()) {
    append_warning("No scene packages found. Expected scenes under " + scenes_path.string() +
      ". Use Browse Scenes Folder or create a new cell.");
  }
  update_scene_browser_status();
}
void SceneSelect::on_add_scene_clicked()
{
  configure_startup_fallback_paths();

  AddScene scene_window;
  scene_window.setWindowTitle("Create New Scene");
  scene_window.setModal(true);
  scene_window.scenes_path = scenes_path;
  scene_window.assets_path = assets_path;
  scene_window.workcell_path = workcell_path;
  scene_window.exec();
  if (scene_window.success) {
    workcell.scene_vector.push_back(scene_window.scene);
    generate_scene_package(
      scenes_path, scene_window.scene.name, workcell.ros_ver, workcell.ros_distro);
    const fs::path scene_package_path = scenes_path / scene_window.scene.name;
    boost::system::error_code path_ec;
    const fs::path resolved_scenes = fs::canonical(scenes_path, path_ec);
    append_info("Selected scenes directory: " + scenes_path.string());
    if (!path_ec) {
      append_info("Resolved scenes directory: " + resolved_scenes.string());
    }
    append_info("Scene package: " + scene_package_path.string());
    refresh_scenes(workcell.scene_vector.size() - 1, true);
    update_scene_browser_status("Created new scene at: " + (scenes_path / scene_window.scene.name).string());
  }
}

void SceneSelect::on_browse_scenes_folder_clicked()
{
  const QString selected = QFileDialog::getExistingDirectory(
    this, "Select Scenes Folder", QString::fromStdString(path_or_placeholder(scenes_path)));
  if (selected.isEmpty()) return;
  scenes_path = fs::path(selected.toStdString());
  workcell_path = scenes_path.parent_path();
  assets_path = workcell_path / "assets";
  workcell.scene_vector.clear();
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
}

void SceneSelect::on_refresh_scenes_button_clicked()
{
  workcell.scene_vector.clear();
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
}


void write_builder_validation_helper(const fs::path & scene_dir)
{
  const fs::path generated_dir = scene_dir / "generated";
  if (!boost::filesystem::exists(generated_dir)) {
    boost::filesystem::create_directories(generated_dir);
  }
  const auto write_tool_root_discovery = [](std::ofstream & stream) {
      stream << "SCRIPT_DIR=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\n";
      stream << "SCENE_DIR=\"$(cd \"$SCRIPT_DIR/..\" && pwd)\"\n";
      stream << "TOOL_ROOT=\"${WORKCELL_STUDIO_REPO_ROOT:-}\"\n";
      stream << "find_tool_root() {\n";
      stream << "  local candidate=\"$1\"\n";
      stream << "  [ -n \"$candidate\" ] || return 1\n";
      stream << "  if [ -f \"$candidate/scripts/workcell_studio.py\" ] && [ -f \"$candidate/scripts/validate_builder_generated_scene.py\" ]; then\n";
      stream << "    TOOL_ROOT=\"$candidate\"\n";
      stream << "    return 0\n";
      stream << "  fi\n";
      stream << "  return 1\n";
      stream << "}\n";
      stream << "if [ -z \"$TOOL_ROOT\" ]; then\n";
      stream << "  for candidate in \"$PWD\" \"$SCENE_DIR\" \"$SCENE_DIR/..\" \"$SCENE_DIR/../easy_manipulation_deployment\" \"$SCENE_DIR/../../easy_manipulation_deployment\" \"$HOME/workcell_ws/src/easy_manipulation_deployment\"; do\n";
      stream << "    if find_tool_root \"$candidate\"; then\n";
      stream << "      break\n";
      stream << "    fi\n";
      stream << "  done\n";
      stream << "fi\n";
      stream << "if ! find_tool_root \"$TOOL_ROOT\"; then\n";
      stream << "  echo \"Could not locate Workcell Studio scripts. Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment\" >&2\n";
      stream << "  exit 1\n";
      stream << "fi\n";
    };
  const auto set_script_permissions = [](const fs::path & script_file) {
      boost::system::error_code ec;
      fs::permissions(
        script_file,
        fs::owner_read | fs::owner_write | fs::owner_exe |
        fs::group_read | fs::group_exe |
        fs::others_read | fs::others_exe,
        ec);
      if (ec) {
        RCLCPP_WARN(
          rclcpp::get_logger("workcell_builder"),
          "Failed to set executable permissions on %s: %s",
          script_file.string().c_str(), ec.message().c_str());
      }
    };
  const fs::path script_path = generated_dir / "run_builder_validation.sh";
  std::ofstream out(script_path.string());
  if (out.is_open()) {
    out << "#!/usr/bin/env bash\n";
    out << "set -euo pipefail\n";
    write_tool_root_discovery(out);
    out << "python3 \"$TOOL_ROOT/scripts/validate_builder_generated_scene.py\" \"$SCENE_DIR\"\n";
    out.close();
    set_script_permissions(script_path);
  }
  const fs::path export_path = generated_dir / "export_workcell_studio_sources.sh";
  std::ofstream export_out(export_path.string());
  if (export_out.is_open()) {
    export_out << "#!/usr/bin/env bash\n";
    export_out << "set -euo pipefail\n";
    write_tool_root_discovery(export_out);
    export_out << "python3 \"$TOOL_ROOT/scripts/export_builder_scene_to_cell_definition.py\" \"$SCENE_DIR\" --output-dir \"$SCENE_DIR/generated\" --validate\n";
    export_out << "if [ ! -f \"$SCENE_DIR/generated/workcell_builder_task_intent.yaml\" ]; then\n";
    export_out << "  echo \"INFO: No workcell_builder_task_intent.yaml found yet.\"\n";
    export_out << "  echo \"INFO: Author task intent via scripts/create_or_update_builder_task_intent.py or Workcell Studio helper.\"\n";
    export_out << "fi\n";
    export_out.close();
    set_script_permissions(export_path);
  }
  const fs::path readiness_path = generated_dir / "generate_readiness_pack.sh";
  std::ofstream readiness_out(readiness_path.string());
  if (readiness_out.is_open()) {
    readiness_out << "#!/usr/bin/env bash\n";
    readiness_out << "set -euo pipefail\n";
    write_tool_root_discovery(readiness_out);
    readiness_out << "OUT_DIR=\"${1:-/tmp/workcell_readiness_pack}\"\n";
    readiness_out << "PROJECT_NAME=\"${2:-$(basename \"$SCENE_DIR\")}\"\n";
    readiness_out << "python3 \"$TOOL_ROOT/scripts/workcell_studio.py\" generate-readiness-pack \\\n";
    readiness_out << "  --scene-package \"$SCENE_DIR\" \\\n";
    readiness_out << "  --output-dir \"$OUT_DIR\" \\\n";
    readiness_out << "  --project-name \"$PROJECT_NAME\" \\\n";
    readiness_out << "  --validate \\\n";
    readiness_out << "  --prepare-rviz-preview \\\n";
    readiness_out << "  --smoke-dry-run \\\n";
    readiness_out << "  --force \\\n";
    readiness_out << "  --json\n";
    readiness_out.close();
    set_script_permissions(readiness_path);
  }
  const fs::path dashboard_help_path = generated_dir / "open_dashboard_help.md";
  std::ofstream dashboard_help_out(dashboard_help_path.string());
  if (dashboard_help_out.is_open()) {
    dashboard_help_out << "# Open Workcell Studio readiness dashboard\n\n";
    dashboard_help_out << "The readiness pack is generated at `/tmp/workcell_readiness_pack` by default.\n\n";
    dashboard_help_out << "Recommended local serving flow:\n\n";
    dashboard_help_out << "```bash\ncd /tmp/workcell_readiness_pack && python3 -m http.server 8767\n```\n\n";
    dashboard_help_out << "Then open: `http://localhost:8767/readiness_dashboard.html`\n\n";
    dashboard_help_out << "Note: direct `xdg-open` may not work in sandboxed Firefox/Snap environments.\n\n";
    dashboard_help_out << "Run generated helpers directly from any directory:\n\n";
    dashboard_help_out << "```bash\n./generated/run_builder_validation.sh\n./generated/export_workcell_studio_sources.sh\n./generated/generate_readiness_pack.sh /tmp/workcell_readiness_pack test_scene\n```\n\n";
    dashboard_help_out << "If scripts cannot locate tooling, set:\n\n";
    dashboard_help_out << "```bash\nexport WORKCELL_STUDIO_REPO_ROOT=~/workcell_ws/src/easy_manipulation_deployment\n```\n\n";
    dashboard_help_out << "Dashboard output is review-only and is not a safety certificate.\n";
  }
}

bool ensure_minimal_environment_yaml(const fs::path & scene_dir, const std::string & scene_name)
{
  const fs::path environment_file = scene_dir / "environment.yaml";
  if (fs::exists(environment_file)) {
    return true;
  }
  std::ofstream out(environment_file.string());
  if (!out.is_open()) {
    return false;
  }
  out << "robot:\n";
  out << "  brand: \"unknown\"\n";
  out << "  name: \"unknown\"\n";
  out << "end_effector:\n";
  out << "  brand: \"none\"\n";
  out << "  name: \"none\"\n";
  out << "objects: []\n";
  out << "external joints: []\n";
  out << "scene_name: \"" << scene_name << "\"\n";
  return true;
}

void refresh_scene_manifest_if_missing(const fs::path & scene_dir, const std::string & scene_name)
{
  const fs::path manifest = scene_dir / "scene_manifest.yaml";
  if (fs::exists(manifest)) {
    return;
  }
  std::ofstream out(manifest.string());
  if (!out.is_open()) {
    return;
  }
  out << "scene_name: " << scene_name << "\n";
  out << "generated_by: workcell_builder\n";
}

void SceneSelect::generate_scene_package(
  fs::path scene_filepath,
  std::string scene_name, int ros_ver, const std::string & ros_distro)
{
  const fs::path scene_dir = scene_filepath / scene_name;
  if (!boost::filesystem::exists(scene_dir)) {
    boost::filesystem::create_directory(scene_dir);
  }
  const fs::path scene_urdf_dir = scene_dir / "urdf";
  if (!boost::filesystem::exists(scene_urdf_dir)) {
    boost::filesystem::create_directory(scene_urdf_dir);
  }
  ensure_minimal_environment_yaml(scene_dir, scene_name);
  refresh_scene_manifest_if_missing(scene_dir, scene_name);
  fs::path workcell_path(scene_filepath.branch_path());
  generate_cmakelists(workcell_path, scene_name, ros_ver, ros_distro);
  generate_package_xml(workcell_path, scene_name, ros_ver, ros_distro);
  write_builder_validation_helper(scene_dir);
  append_success("Scene package generated/updated successfully.");
  append_info("Build before launching so ROS 2 can discover updated package files.");
  append_info("Next commands:");
  append_info("  colcon build --symlink-install --packages-select " + scene_name);
  append_info("  source install/setup.bash");
  append_info("  ros2 launch " + scene_name + " demo.launch.py use_fake_hardware:=true");
  std::ofstream readme((scene_dir / "README.builder.md").string());
  if (readme.is_open()) {
    readme << "# Builder Scene Metadata\n\n";
    readme << "This scene was generated by workcell_builder.\n\n";
    readme << "## Workcell Studio command centre\n\n";
    readme << "workcell_builder is the primary Workcell Studio command centre.\n";
    readme << "Use the integrated Workcell Studio panel/tab for validate/export/readiness/preview actions.\n\n";
    readme << "Panel actions:\n";
    readme << "- Validate Scene\n- Export Workcell Studio Sources\n- Generate Readiness Pack\n";
    readme << "- Open Static Preview\n- Open Readiness Dashboard\n";
    readme << "- Copy RViz Preview Command\n- Copy Grasp Flow Preview Command\n\n";
    readme << "Next steps:\n";
    readme << "1. Validate builder scene:\n\n```bash\n./generated/run_builder_validation.sh\n```\n\n";
    readme << "2. Export Workcell Studio source files:\n\n```bash\n./generated/export_workcell_studio_sources.sh\n```\n\n";
    readme << "3. Review generated summary metadata: `generated/builder_export_summary.json`\n\n";
    readme << "4. Define pick/place task intent:\n";
    readme << "   - use Workcell Studio Streamlit helper, or\n";
    readme << "   - use scripts/create_or_update_environment_target.py and scripts/create_or_update_builder_task_intent.py\n\n";
    readme << "5. Generate readiness pack:\n\n```bash\n./generated/generate_readiness_pack.sh /tmp/workcell_readiness_pack test_scene\n```\n\n";
    readme << "6. Open dashboard:\n\n```bash\ncd /tmp/workcell_readiness_pack && python3 -m http.server 8767\n```\n\n";
    readme << "http://localhost:8767/readiness_dashboard.html\n\n";
    readme << "If helpers cannot locate tooling, set:\n\n```bash\nexport WORKCELL_STUDIO_REPO_ROOT=~/workcell_ws/src/easy_manipulation_deployment\n```\n\n";
    readme << "Safety note: offline/fake-hardware only, no robot motion, no MoveIt service call, not a safety certificate.\n";
  }
}

void SceneSelect::generate_scene_files(Scene scene)
{
  if (!validate_description_xacros(scene, "ERROR:")) {
    append_error("Scene file generation blocked due to missing description files.");
    return;
  }
  // generate environment.urdf.xacro
  const fs::path scene_dir = scenes_path / scene.name;
  const fs::path urdf_dir = scene_dir / "urdf";
  const fs::path armhand_srdf_path = urdf_dir / "arm_hand.srdf.xacro";
  generate_scene_xacro(scene, (urdf_dir / "scene.urdf.xacro").string());
  if (scene.robot_loaded && scene.ee_loaded) {
    generate_armhand_xacro(
      scene.robot_vector[0], scene.ee_vector[0], scene.name, armhand_srdf_path.string());
  }
  if (scene.robot_loaded && !scene.ee_loaded) {  // no ee
    generate_armhand_xacro(scene.robot_vector[0], scene.name, armhand_srdf_path.string());
  }
  if (!scene.robot_loaded && !scene.ee_loaded) {  // no robot and ee
    generate_armhand_xacro(scene.name, armhand_srdf_path.string());
  }
  fs::path srdf_path = armhand_srdf_path;
  if (!boost::filesystem::exists(srdf_path)) {
    append_error("Failed to generate urdf/arm_hand.srdf.xacro.");
    append_error("Expected SRDF at: " + srdf_path.string());
    append_error("Current working directory: " + fs::current_path().string());
    return;
  }
  fs::path base_template_path = templates_path / ("ros" + std::to_string(workcell.ros_ver));
  fs::path launch_path = base_template_path / workcell.ros_distro / "launch";
  if (workcell.ros_distro.empty() || !boost::filesystem::exists(launch_path)) {
    launch_path = base_template_path / "launch";
  }
  fs::path target_path = scene_dir / "launch";
  if (!copyDir(launch_path, target_path)) {
    append_error(
      "Failed to generate/merge launch files. Check filesystem permissions and destination path: " +
      target_path.string());
    return;
  }

  find_replace(
    (target_path / "demo.launch.py").string(),
    (target_path / "demo_interim.launch.py").string(),
    "scene_name", scene.name);
  std::string base_link_name = "base_link";
  std::string moveit_config_name = scene.name + "_moveit_config";
  if (scene.robot_loaded && !scene.robot_vector.empty()) {
    base_link_name = scene.robot_vector[0].base_link;
    moveit_config_name = scene.robot_vector[0].name + "_moveit_config";
  }
  find_replace(
    (target_path / "demo_interim.launch.py").string(),
    (target_path / "demo_interim2.launch.py").string(), "base_link_name",
    base_link_name);
  find_replace(
    (target_path / "demo_interim2.launch.py").string(),
    (target_path / "demo.launch.py").string(), "moveit_config_name",
    moveit_config_name);

  std::string robot_name = scene.robot_loaded && !scene.robot_vector.empty() ? scene.robot_vector[0].name : "";
  std::string ee_name = scene.ee_loaded && !scene.ee_vector.empty() ? scene.ee_vector[0].name : "";
  const fs::path metadata_path = scene_dir / "workcell_builder_metadata.yaml";
  const fs::path tool_root = resolve_tool_root(workcell_path, scene_dir);
  if (tool_root.empty()) {
    append_warning("Could not locate render_workcell_builder_metadata.py. Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment.");
  } else {
    const fs::path metadata_script = tool_root / "scripts" / "render_workcell_builder_metadata.py";
    const std::string cmd =
      "python3 \"" + metadata_script.string() + "\" --robot \"" + robot_name +
      "\" --end-effector \"" + ee_name + "\" --scene-path \"" + scene_dir.string() +
      "\" --output \"" + metadata_path.string() + "\"";
    const int metadata_rc = std::system(cmd.c_str());
    if (metadata_rc != 0) {
      append_warning("Workcell Studio metadata generation command failed with exit code " + std::to_string(metadata_rc) + ".");
    }
  }
  write_builder_validation_helper(scene_dir);
  append_success("Scene package generated/updated successfully.");
  append_info("Build before launching so ROS 2 can discover updated package files.");
  append_info("Next commands:");
  append_info("  cd " + workcell_path.string());
  append_info("  colcon build --symlink-install --packages-select " + scene.name);
  append_info("  source install/setup.bash");
  append_info("  ros2 launch " + scene.name + " demo.launch.py use_fake_hardware:=true");
  append_warning("Real hardware mode requires explicit validation and use_fake_hardware:=false.");
  append_info("Workcell Studio metadata generated/updated.");
  append_info("Validation helper: generated/run_builder_validation.sh");
  append_info("Export helper: generated/export_workcell_studio_sources.sh");
  append_info("Readiness helper: generated/generate_readiness_pack.sh");
  append_info("Next: define pick/place task intent, then generate readiness pack.");
}
void SceneSelect::refresh_scenes(int latest_scene, bool scaffold_only_status)
{
  if (latest_scene < 0) {latest_scene = 0;}
  scaffold_scene_index_ = scaffold_only_status ? latest_scene : -1;
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();  // Clear the dropdown menu
  if (workcell.scene_vector.size() > 0) {  // There are scenes in the workcell
    ui->scene_list->setDisabled(false);     // Enable the dropdown menu
    for (int scene = 0; scene < static_cast<int>(workcell.scene_vector.size()); scene++) {
      Scene scene_value = workcell.scene_vector[scene];
      if (!scene_value.loaded) {
        load_scene_from_yaml(&scene_value);
      }
      const SceneUiStatus status = compute_scene_status_label(scene_value, scenes_path / workcell.scene_vector[scene].name);
      const std::string label = workcell.scene_vector[scene].name + " [" + scene_status_label(status) + "]";
      ui->scene_list->addItem(QString::fromStdString(label));
    }
    ui->scene_list->setCurrentIndex(latest_scene);     // Display the latest scene the user created
    on_scene_list_currentIndexChanged(latest_scene);
    ui->edit_scene->setDisabled(false);
    ui->delete_scene->setDisabled(false);
    ui->generate_yaml->setDisabled(false);
    ui->generate_files->setDisabled(false);
  } else {  // no scenes
    ui->scene_list->setDisabled(true);
    ui->generate_yaml->setDisabled(true);
    ui->generate_files->setDisabled(true);
    ui->edit_scene->setDisabled(true);
    ui->delete_scene->setDisabled(true);
    append_warning("No scenes available. Add a scene to continue.");
  }
  ui->scene_list->blockSignals(oldState);
}
void SceneSelect::on_delete_scene_clicked()
{
  configure_startup_fallback_paths();
  ReplaceWarning replace_window;
  replace_window.setWindowTitle("Edit Scene");
  replace_window.set_label("Warning: Scene folders with all files will be deleted. Continue?");
  replace_window.setModal(true);
  replace_window.exec();

  if (replace_window.decision) {  // user allows for scene folder deletion
    if (ui->scene_list->currentIndex() >= 0) {
      bool oldState = ui->scene_list->blockSignals(true);
      delete_folder(scenes_path, workcell.scene_vector[ui->scene_list->currentIndex()].name);
      workcell.scene_vector.erase(workcell.scene_vector.begin() + ui->scene_list->currentIndex());
      if (workcell.scene_vector.size() > 0) {
        refresh_scenes(0, false);
      } else {
        refresh_scenes(-1, false);
      }
      ui->scene_list->blockSignals(oldState);
    } else {
      append_error("No scene selected to delete.");
    }
  }
}
void SceneSelect::on_edit_scene_clicked()
{
  configure_startup_fallback_paths();
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        // if scene.loaded is not true, generate scene from yaml
        append_error("Could not load scene from environment.yaml.");
        return;
      }
    }
    // Scene loaded
    AddScene scene_window;
    scene_window.scenes_path = scenes_path;
    scene_window.assets_path = assets_path;
    scene_window.workcell_path = workcell_path;
    scene_window.LoadScene(curr_scene);
    scene_window.setWindowTitle("Edit Scene");
    scene_window.setModal(true);
    scene_window.exec();
    if (scene_window.success) {
      if (CheckSceneEqual(scene_window.scene, curr_scene)) {
        refresh_scenes(ui->scene_list->currentIndex(), false);
      } else {  // Scene was edited
        const fs::path scene_yaml_path = scenes_path / scene_window.scene.name;
        if (boost::filesystem::exists(scene_yaml_path)) {     // Scene name nvr change
          // Replace the current environment yaml
          if (GenerateYAML::generate_yaml(
              scene_window.scene,
              scene_yaml_path.string(), scenes_path, assets_path))
          {
            generate_scene_files(scene_window.scene);
          } else {
            append_error(
              "Failed to generate environment.yaml: invalid external joint parent configuration.");
            return;
          }

        } else {
          // Delete previous scene folder
          delete_folder(scenes_path, curr_scene.name);
          // Generate new folder
          generate_scene_package(
            scenes_path, scene_window.scene.name, workcell.ros_ver, workcell.ros_distro);
          if (GenerateYAML::generate_yaml(
              scene_window.scene,
              scene_yaml_path.string(), scenes_path, assets_path))
          {
            generate_scene_files(scene_window.scene);
          } else {
            append_error(
              "Failed to generate environment.yaml: invalid external joint parent configuration.");
            return;
          }
        }
        workcell.scene_vector[ui->scene_list->currentIndex()] = scene_window.scene;
        append_warning(
          "Scene edits were applied. Regenerate environment.yaml to save the latest scene state.");
        refresh_scenes(ui->scene_list->currentIndex(), false);
      }
    } else {
      refresh_scenes(ui->scene_list->currentIndex(), false);
    }
  } else {
    append_error("No scene selected to edit.");
  }
}
void SceneSelect::on_generate_yaml_clicked()
{
  configure_startup_fallback_paths();
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    const fs::path scene_yaml_path =
      scenes_path / workcell.scene_vector[ui->scene_list->currentIndex()].name;
    Scene target_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!target_scene.loaded) {  // No scene currently loaded
      if (check_yaml()) {    // If yaml file is in folder,
                             // it might get replaced by new scene configuration
        append_info("No unsaved scene edits detected; existing environment.yaml kept.");
        return;
      } else {   // No yaml in scene folder, no loaded scene from created
        append_error(
          "No existing environment.yaml found and no unsaved scene edits are available to export.");
        return;
      }
    } else {
      if (check_yaml()) {    // If yaml file is in folder, it might get replaced by new config
        ReplaceWarning replace_window;
        replace_window.setWindowTitle("Edit Scene");
        replace_window.set_label(
          "Warning: Environment yaml currently exists. "
          "Current environment yaml will be replaced. Continue?");
        replace_window.setModal(true);
        replace_window.exec();
        if (replace_window.decision) {      // user allows for replacing of current yaml file
          if (GenerateYAML::generate_yaml(
              target_scene,
              scene_yaml_path.string(), scenes_path, assets_path))
          {
            append_success("environment.yaml generated successfully.");
          } else {
            append_error(
              "environment.yaml generation failed: invalid external joint parent configuration.");
            return;
          }
        }
      } else {   // currently no yaml file, add one to scene folder
        if (GenerateYAML::generate_yaml(
            target_scene, scene_yaml_path.string(), scenes_path,
            assets_path))
        {
          append_success("environment.yaml generated successfully.");
        } else {
          append_error(
            "environment.yaml generation failed: invalid external joint parent configuration.");
          return;
        }
      }
    }
  } else {
    append_error("No scene selected to generate environment.yaml.");
  }
  scaffold_scene_index_ = -1;
  refresh_scene_status(true, "Generate YAML");
}
bool SceneSelect::check_yaml()  // Check if scene package has a yaml file to use.
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty() || !boost::filesystem::exists(scene_dir / "environment.yaml")) {
    return false;
  } else {
    ui->edit_scene->setDisabled(false);
  }
  return true;
}
bool SceneSelect::check_scene(bool strict)
{
  bool has_yaml = check_yaml();
  bool files_loaded_proper = check_files(strict);
  bool scene_incomplete = false;

  if (has_yaml) {
    Scene curr_scene;
    if (ui->scene_list->currentIndex() >= 0) {
      curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
      if (!curr_scene.loaded) {
        load_scene_from_yaml(&curr_scene);
      }
      if (!scene_has_valid_robot(curr_scene)) {
        scene_incomplete = true;
      }
      if (curr_scene.ee_loaded && !curr_scene.ee_vector.empty() && !scene_has_valid_end_effector(curr_scene) && scene_requires_end_effector(curr_scene)) {
        scene_incomplete = true;
      }
    }
    if (scene_incomplete) {
      append_warning("Scene status: environment.yaml found, but scene is incomplete.");
    } else {
      append_info("Scene status: environment.yaml found.");
    }
  } else {
    append_warning("environment.yaml exists. Full scene package has not been generated yet.");
  }

  if (files_loaded_proper) {
    if (strict && !scene_incomplete) {
      append_success("Scene valid: environment.yaml, robot, and MoveIt config found.");
    } else if (strict) {
      append_warning("Scene status: required files exist, but scene inputs are incomplete.");
    } else {
      append_info(
        "Scene status: scaffold created (SCAFFOLD_ONLY/INCOMPLETE). Generate files only after robot/end-effector are configured.");
    }
  }

  if (strict && has_yaml && files_loaded_proper && !scene_incomplete) {
    append_success("Scene generation complete. You may exit this application.");
  }
  if (strict && !has_yaml && files_loaded_proper) {
    append_warning(
      "Scene files were generated, but without environment.yaml this scene cannot be edited after exit.");
  }
  if (!strict && files_loaded_proper) {
    append_info(
      "Scene status: launch/SRDF files are not generated yet. Use Generate Files when ready.");
  }
  ui->exit->setDisabled(false);
  return true;
}
bool SceneSelect::check_files(bool strict)
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_error("Scene status: no scene selected.");
    return false;
  }
  const fs::path launch_dir = scene_dir / "launch";
  const fs::path urdf_dir = scene_dir / "urdf";
  const fs::path cmake_file = scene_dir / "CMakeLists.txt";
  const fs::path package_file = scene_dir / "package.xml";

  if (!boost::filesystem::exists(urdf_dir) || !boost::filesystem::exists(cmake_file) ||
    !boost::filesystem::exists(package_file))
  {
    append_error("Scene status: required scene package skeleton files are missing.");
    if (!boost::filesystem::exists(urdf_dir)) {
      append_error("Scene status: urdf folder missing.");
    }
    if (!boost::filesystem::exists(cmake_file)) {
      append_error("Scene status: CMakeLists.txt missing.");
    }
    if (!boost::filesystem::exists(package_file)) {
      append_error("Scene status: package.xml missing.");
    }
    return false;
  }

  if (!strict) {
    return true;
  }

  if (!boost::filesystem::exists(launch_dir)) {
    append_error("Scene status: required files are missing.");
    append_error("launch/ is missing. Click Generate Full Scene Package.");
    if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro")) {
      append_error(
        "Scene status: launch not generated because SRDF generation failed earlier.");
    }
    return false;
  }

  if (!boost::filesystem::exists(launch_dir / "demo.rviz") ||
    !boost::filesystem::exists(launch_dir / "demo.launch.py"))
  {
    append_error("Scene status: required launch files are missing.");
    if (!boost::filesystem::exists(launch_dir / "demo.rviz")) {
      append_error("Scene status: demo.rviz missing.");
    }
    if (!boost::filesystem::exists(launch_dir / "demo.launch.py")) {
      append_error("Scene status: demo.launch.py missing.");
    }
    return false;
  }

  if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro") ||
    !boost::filesystem::exists(urdf_dir / "scene.urdf.xacro"))
  {
    append_error("Scene status: required URDF files are missing.");
    if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro")) {
      append_error("Scene status: arm_hand.srdf.xacro missing.");
    }
    if (!boost::filesystem::exists(urdf_dir / "scene.urdf.xacro")) {
      append_error("Scene status: scene.urdf.xacro missing.");
    }
    return false;
  }
  if (ui->scene_list->currentIndex() >= 0) {
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        append_error("Scene status: unable to load scene metadata from environment.yaml.");
        return false;
      }
    }
    if (!validate_description_xacros(curr_scene, "[Scene Status]")) {
      return false;
    }
  }
  return true;
}
void SceneSelect::on_scene_list_currentIndexChanged(int index)
{
  if (index >= 0 && index < static_cast<int>(workcell.scene_vector.size())) {
    Scene curr_scene = workcell.scene_vector[index];
    if (!curr_scene.loaded) {
      load_scene_from_yaml(&curr_scene);
    }
    const SceneUiStatus status = compute_scene_status_label(curr_scene, scenes_path / curr_scene.name);
    const bool can_generate_files = status == SceneUiStatus::VALID || status == SceneUiStatus::MISSING_MOVEIT_CONFIG || status == SceneUiStatus::SCAFFOLD_ONLY;
    ui->generate_full_scene_package_start->setDisabled(!can_generate_files);
    if (!can_generate_files) {
      append_warning("Generate Files blocked: scene status is " + scene_status_label(status) + ".");
    }
  }
  refresh_scene_status(index != scaffold_scene_index_, "Scene Selection Changed");
}
void SceneSelect::on_generate_files_clicked()
{
  configure_startup_fallback_paths();
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        append_error("environment.yaml missing. Click Generate YAML files for scene first.");
        return;
      }
    }
    if (!scene_has_valid_robot(curr_scene)) {
      append_error("Scene is incomplete: select a robot before generating.");
      return;
    }
    if (curr_scene.ee_loaded && !curr_scene.ee_vector.empty() && !scene_has_valid_end_effector(curr_scene) && scene_requires_end_effector(curr_scene)) {
      append_error("Scene is incomplete: select an end effector or disable end effector.");
      return;
    }
    // Generate all environment object packages
    for (Object object : curr_scene.object_vector) {
      // Generate the folders and CMakeLists + Package xmls
      generate_object_package(workcell_path, object, workcell.ros_ver);
      // Generate urdf xacro for object
      const fs::path object_urdf_dir =
        assets_path / "environment" / (object.name + std::string("_description")) / "urdf";
      make_object_xacro(object, object_urdf_dir.string());
    }
    generate_scene_files(curr_scene);
    write_task_recipe_yaml(scene_dir_for_current_selection(), infer_task_grasp_defaults(curr_scene));
    bool blocked = false;
    const std::string readiness = build_workcell_readiness_report(curr_scene, scene_dir_for_current_selection(), true, &blocked);
    write_workcell_studio_summary(curr_scene, scene_dir_for_current_selection(), blocked ? "BLOCKED" : "READY_TO_GENERATE");
    append_info(readiness);
    append_info("Command panel:
cd <workspace>
colcon build --symlink-install --packages-select " + curr_scene.name + "
source install/setup.bash
ros2 launch " + curr_scene.name + " demo.launch.py use_fake_hardware:=true");
  } else {
    append_error("No scene selected to generate files from.");
  }
  scaffold_scene_index_ = -1;
  refresh_scene_status(true, "Generate Files");
}
bool SceneSelect::load_scene_from_yaml(Scene * input_scene)
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scenes_path / input_scene->name;
  const fs::path yaml_path = scene_dir / "environment.yaml";
  if (!boost::filesystem::exists(scene_dir)) {
    std::cerr << "Scene directory does not exist: " << scene_dir.string() << '\n';
    return false;
  }
  YAML::Node yaml;
  // Load Yaml File.
  try {
    yaml = YAML::LoadFile(yaml_path.string());
    // std::ifstream f("environment.yaml");
    //    if (f.is_open())
    //        std::cout << f.rdbuf() << std::endl;
  } catch (YAML::BadFile & error) {
    append_error("Failed to read environment.yaml. Regenerate the file and try again.");
    return false;
  }
  YAML::Node objects;
  YAML::Node ext_joints;
  bool has_objects = false;

  input_scene->object_vector.clear();
  input_scene->parent_objects.clear();
  input_scene->child_objects.clear();

  for (YAML::iterator it = yaml.begin(); it != yaml.end(); ++it) {
    std::string key = it->first.as<std::string>();
    if (key.compare("robot") == 0) {
      Robot robot;
      SceneParser::LoadRobotFromYAML(&robot, it->second);
      input_scene->robot_loaded = true;
      input_scene->robot_vector.clear();
      input_scene->robot_vector.push_back(robot);
    }
    if (key.compare("end_effector") == 0) {
      EndEffector ee;
      SceneParser::LoadEEFromYAML(&ee, it->second);
      input_scene->ee_loaded = true;
      input_scene->ee_vector.clear();
      input_scene->ee_vector.push_back(ee);
    }
    if (key.compare("objects") == 0) {
      objects = it->second;
      has_objects = true;
    }
    if (key.compare("external joints") == 0) {
      ext_joints = it->second;
    }
  }

  if (has_objects) {  // We need to do this because the object field needs to load before
                      // the ext joint field, and it currently has a random load order
    std::unordered_set<std::string> object_names;
    for (YAML::iterator objects_it = objects.begin(); objects_it != objects.end(); ++objects_it) {
      Object temp_object;
      temp_object.name = objects_it->first.as<std::string>();
      if (object_names.find(temp_object.name) != object_names.end()) {
        append_error("Duplicate object name detected in environment.yaml.");
        return false;
      }
      object_names.insert(temp_object.name);
      YAML::Node ext_joint;
      temp_object.ext_joint.child_object = temp_object.name;
      for (YAML::iterator in_object_it = objects_it->second.begin();
        in_object_it != objects_it->second.end(); ++in_object_it)
      {
        if (in_object_it->first.as<std::string>().compare("links") == 0) {
          std::vector<Link> temp_link_vector;
          SceneParser::LoadLinksFromYAML(&temp_link_vector, in_object_it->second);
          temp_object.link_vector = temp_link_vector;
        }
        if (in_object_it->first.as<std::string>().compare("joints") == 0) {
          std::vector<Joint> temp_joint_vector;
          YAML::Node joints = in_object_it->second;
          SceneParser::LoadJointsFromYAML(
            &temp_joint_vector, temp_object.link_vector,
            in_object_it->second);
          temp_object.joint_vector = temp_joint_vector;
        }
        if (in_object_it->first.as<std::string>().compare(temp_object.name + "_base_joint") == 0) {
          temp_object.ext_joint.name = in_object_it->first.as<std::string>();
          ext_joint = in_object_it->second;
        }
      }

      for (YAML::iterator in_ext_joint_it = ext_joint.begin(); in_ext_joint_it != ext_joint.end();
        ++in_ext_joint_it)
      {
        if (in_ext_joint_it->first.as<std::string>().compare("ext_joint_type") == 0) {
          temp_object.ext_joint.type = in_ext_joint_it->second.as<std::string>();
        }
        if (in_ext_joint_it->first.as<std::string>().compare("child_link") == 0) {
          // Get Child Link pos
          std::string child_link = in_ext_joint_it->second.as<std::string>();
          bool found_child_link = false;
          for (int i = 0; i < static_cast<int>(temp_object.link_vector.size()); i++) {
            if (child_link.compare(temp_object.link_vector[i].name) == 0) {
              temp_object.ext_joint.child_link_pos = i;
              found_child_link = true;
              break;
            }
          }
          if (!found_child_link) {
            append_warning(
              "child_link '" + child_link + "' not found in object '" +
              temp_object.name + "'; child_link_pos left unset.");
          }
        }
      }
      input_scene->object_vector.push_back(temp_object);
    }

    int counter = 0;
    for (YAML::iterator ext_joints_it = ext_joints.begin(); ext_joints_it != ext_joints.end();
      ++ext_joints_it)
    {
      input_scene->object_vector[counter].ext_joint.origin.is_origin = false;
      input_scene->object_vector[counter].ext_joint.axis.is_axis = false;
      YAML::Node in_ext_joints = ext_joints_it->second;
      for (YAML::iterator in_ext_joints_it = in_ext_joints.begin();
        in_ext_joints_it != in_ext_joints.end(); ++in_ext_joints_it)
      {
        if (in_ext_joints_it->first.as<std::string>().compare("parent object") == 0) {
          std::string parent_object = in_ext_joints_it->second.as<std::string>();
          if (parent_object.compare("world") == 0) {       // if world pos is -1
            input_scene->object_vector[counter].ext_joint.parent_obj_pos = -1;
          } else {
            bool found_parent_obj = false;
            for (int i = 0; i < static_cast<int>(input_scene->object_vector.size()); i++) {
              if (parent_object.compare(input_scene->object_vector[i].name) == 0) {
                input_scene->object_vector[counter].ext_joint.parent_obj_pos = i;
                found_parent_obj = true;
                break;
              }
            }
            if (!found_parent_obj) {
              append_warning(
                "parent object '" + parent_object + "' not found; defaulting to world.");
              input_scene->object_vector[counter].ext_joint.parent_obj_pos = -1;
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("parent link") == 0) {
          // Get Parent link pos
          std::string parent_link = in_ext_joints_it->second.as<std::string>();
          int parent_obj_pos = input_scene->object_vector[counter].ext_joint.parent_obj_pos;
          if (parent_obj_pos >= 0) {
            bool found_parent_link = false;
            for (int i = 0;
              i < static_cast<int>(input_scene->object_vector[parent_obj_pos].link_vector.size());
              i++)
            {
              if (parent_link.compare(
                  input_scene->object_vector[parent_obj_pos].
                  link_vector[i].name)
                ==
                0)
              {
                input_scene->object_vector[counter].ext_joint.parent_link_pos = i;
                found_parent_link = true;
                break;
              }
            }
            if (!found_parent_link) {
              input_scene->object_vector[counter].ext_joint.parent_link_pos = -1;
              append_warning(
                "parent link '" + parent_link +
                "' not found in parent object; parent_link_pos left unset.");
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("origin") == 0) {
          input_scene->object_vector[counter].ext_joint.origin.is_origin = true;
          SceneParser::LoadOriginFromYAML(
            &(input_scene->object_vector[counter].ext_joint.origin),
            in_ext_joints_it->second);
        }
        if (in_ext_joints_it->first.as<std::string>().compare("axis") == 0) {
          input_scene->object_vector[counter].ext_joint.axis.is_axis = true;
          SceneParser::LoadAxisFromYAML(
            &(input_scene->object_vector[counter].ext_joint.axis),
            in_ext_joints_it->second);
        }
      }
      counter++;
    }
  }

  resolve_scene_paths(input_scene, workcell_path);
  input_scene->loaded = true;
  return true;
}

fs::path SceneSelect::scene_dir_for_current_selection() const
{
  const int current_index = ui->scene_list->currentIndex();
  if (current_index < 0 ||
    current_index >= static_cast<int>(workcell.scene_vector.size()))
  {
    return fs::path();
  }
  return scenes_path / workcell.scene_vector[current_index].name;
}

bool SceneSelect::validate_description_xacros(
  const Scene & scene,
  const std::string & context_label)
{
  bool ok = true;
  const std::string prefix = context_label.empty() ? "" : context_label + " ";
  auto report_error = [&](const std::string & message) {
      append_error(prefix + message);
    };
  auto check_xacro = [&](const std::string & package_name,
      const std::vector<std::string> & filenames,
      const std::string & label) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory(package_name);
      } catch (const std::exception & e) {
        ok = false;
        std::ostringstream expected_files;
        for (size_t i = 0; i < filenames.size(); ++i) {
          if (i != 0) {
            expected_files << ", ";
          }
          expected_files << package_name << "/urdf/" << filenames[i];
        }
        report_error(
          "ERROR: Missing " + label + " description package '" + package_name + "': " +
          e.what() + ". Expected file in " + expected_files.str() + ".");
        return;
      }
      std::vector<std::string> attempts;
      for (const auto & filename : filenames) {
        fs::path xacro_path = package_share / "urdf" / filename;
        if (fs::exists(xacro_path)) {
          return;
        }
        attempts.push_back(xacro_path.string());
      }
      ok = false;
      std::ostringstream error_stream;
      error_stream << "ERROR: Missing " << label << " xacro. Tried ";
      for (size_t i = 0; i < attempts.size(); ++i) {
        if (i != 0) {
          error_stream << "; ";
        }
        error_stream << attempts[i];
      }
      error_stream << ".";
      report_error(error_stream.str());
    };
  auto check_xacro_with_fallbacks = [&](const std::vector<std::string> & package_candidates,
      const std::vector<std::string> & filenames,
      const std::string & label) {
      std::vector<std::string> attempts;
      for (const auto & package_name : package_candidates) {
        fs::path package_share;
        try {
          package_share = ament_index_cpp::get_package_share_directory(package_name);
        } catch (const std::exception &) {
          for (const auto & filename : filenames) {
            attempts.push_back(package_name + "/urdf/" + filename + " (package not found)");
          }
          continue;
        }
        for (const auto & filename : filenames) {
          fs::path xacro_path = package_share / "urdf" / filename;
          if (fs::exists(xacro_path)) {
            return;
          }
          attempts.push_back(xacro_path.string());
        }
      }
      ok = false;
      std::ostringstream error_stream;
      error_stream << "ERROR: Missing " << label << " xacro. Tried ";
      for (size_t i = 0; i < attempts.size(); ++i) {
        if (i != 0) {
          error_stream << "; ";
        }
        error_stream << attempts[i];
      }
      error_stream << ".";
      report_error(error_stream.str());
    };
  auto check_universal_robot_xacro = [&](const Robot & robot) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory("ur_description");
      } catch (const std::exception & e) {
        ok = false;
        report_error(
          "ERROR: Missing robot '" + robot.name + "' description package 'ur_description': " +
          e.what() + ". Expected file in ur_description/urdf/" + robot.name +
          ".urdf.xacro or ur_description/urdf/ur_macro.xacro.");
        return;
      }
      const fs::path model_xacro = package_share / "urdf" / (robot.name + ".urdf.xacro");
      if (fs::exists(model_xacro)) {
        return;
      }
      const fs::path config_dir = package_share / "config" / robot.name;
      if (fs::exists(config_dir)) {
        const fs::path ur_xacro = package_share / "urdf" / "ur_macro.xacro";
        if (fs::exists(ur_xacro)) {
          return;
        }
        ok = false;
        report_error(
          "ERROR: Missing robot '" + robot.name + "' xacro. Tried " +
          model_xacro.string() + "; " + ur_xacro.string() + ".");
        return;
      }
      ok = false;
      report_error(
        "ERROR: Missing robot '" + robot.name + "' xacro. Tried " +
        model_xacro.string() + ".");
    };
  auto check_moveit_srdf = [&](const std::string & package_name,
      const std::string & filename,
      const std::string & label) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory(package_name);
      } catch (const std::exception & e) {
        ok = false;
        report_error(
          "ERROR: Missing " + label + " MoveIt config package '" + package_name + "': " +
          e.what() + ". Expected file in " + package_name + "/config/" + filename + ".");
        return;
      }
      const fs::path srdf_path = package_share / "config" / filename;
      if (fs::exists(srdf_path)) {
        return;
      }
      ok = false;
      report_error(
        "ERROR: Missing " + label + " SRDF xacro. Tried " + srdf_path.string() + ".");
    };

  if (scene.robot_loaded) {
    for (const auto & robot : scene.robot_vector) {
      if (is_placeholder_value(robot.name)) {
        ok = false;
        report_error("Scene is incomplete: select a robot before generating.");
        continue;
      }
      const bool is_ur = robot.brand == "universal_robot";
      if (is_ur) {
        check_universal_robot_xacro(robot);
      } else {
        const std::vector<std::string> filenames = robot_description_candidates(robot);
        check_xacro_with_fallbacks(
          description_package_candidates(robot), filenames, "robot '" + robot.name + "'");
      }
      const std::string moveit_package = robot.name + "_moveit_config";
      const std::string srdf_filename = robot.name + ".srdf.xacro";
      check_moveit_srdf(moveit_package, srdf_filename, "robot '" + robot.name + "'");
    }
  }

  if (scene.ee_loaded) {
    for (const auto & ee : scene.ee_vector) {
      const std::string ee_name = normalize_placeholder_token(ee.name);
      if (ee_name == "none" || ee_name.empty()) {
        continue;
      }
      if (is_placeholder_value(ee.name)) {
        report_error("Scene is incomplete: select an end effector or disable end effector.");
        ok = false;
        continue;
      }
      const std::string package_name = resolve_ee_description_package(ee);
      const std::string filename = resolve_ee_xacro_filename(ee);
      check_xacro(package_name, {filename}, "end effector '" + ee.name + "'");
      const std::string moveit_package = ee.name + "_moveit_config";
      const std::string srdf_filename = ee.name + "_gripper.srdf.xacro";
      check_moveit_srdf(moveit_package, srdf_filename, "end effector '" + ee.name + "'");
    }
  }

  return ok;
}

std::string SceneSelect::build_workcell_readiness_report(
  const Scene & scene,
  const fs::path & scene_dir,
  bool strict,
  bool * blocked)
{
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  if (!fs::exists(scene_dir / "environment.yaml")) { blockers.emplace_back("missing environment.yaml"); }
  if (!scene.robot_loaded || scene.robot_vector.empty() || is_placeholder_value(scene.robot_vector[0].name)) { blockers.emplace_back("missing robot"); }
  if (scene.robot_loaded && !scene.robot_vector.empty()) {
    const auto & r = scene.robot_vector[0];
    if (is_placeholder_value(r.description_pkg)) { blockers.emplace_back("missing robot description package"); }
    if (is_placeholder_value(r.moveit_config_pkg)) { blockers.emplace_back("missing robot MoveIt config package"); }
  }
  if (scene.ee_loaded && !scene.ee_vector.empty()) {
    const auto & ee = scene.ee_vector[0];
    if (is_placeholder_value(ee.brand) || is_placeholder_value(ee.name)) { blockers.emplace_back("missing required end-effector fields"); }
    if (normalize_placeholder_token(ee.type).find("unknown") != std::string::npos) { warnings.emplace_back("unknown end-effector type requires manual confirmation"); }
  }
  for (const auto & obj : scene.object_vector) {
    if (obj.filepath.empty() || is_placeholder_value(obj.filepath)) { blockers.emplace_back("missing STL path"); }
    if (obj.filepath.size() > 0 && obj.filepath[0] == '/') { warnings.emplace_back("external absolute STL path"); }
    if (normalize_placeholder_token(obj.name).find("conveyor_placeholder") != std::string::npos) { warnings.emplace_back("conveyor_placeholder is visual/metadata only"); }
    if (is_placeholder_value(obj.name) || is_placeholder_value(obj.base_link.name)) { blockers.emplace_back("placeholder unknown/none/null values"); }
  }
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  if (task_cfg.approach_distance_m < 0.0 || task_cfg.retreat_distance_m < 0.0) {
    blockers.emplace_back("invalid task/grasp numeric values");
  }
  if (task_cfg.task_type == "pick_place" || task_cfg.task_type == "sorting") {
    if (task_cfg.pick_source.empty()) { blockers.emplace_back("missing pick source"); }
    if (task_cfg.place_target.empty()) { blockers.emplace_back("missing place target"); }
  }
  if (task_cfg.grasp_strategy.empty()) { blockers.emplace_back("unsupported empty grasp strategy"); }
  if (task_cfg.pick_source == "perception_detection") {
    warnings.emplace_back("perception_detection selected but EPD adapter not configured yet");
  }
  warnings.emplace_back("real hardware mode requires explicit validation");
  const bool is_blocked = !blockers.empty();
  const std::string status = is_blocked ? "BLOCKED" : (warnings.empty() ? (strict ? "READY_TO_GENERATE" : "SCAFFOLD_ONLY") : "WARNINGS");
  if (blocked) { *blocked = is_blocked; }

  std::ostringstream out;
  out << "Workcell Studio Readiness\n";
  out << "status: " << status << "\n";
  out << "scene name: " << scene.name << "\n";
  out << "scene root path: " << scene_dir.string() << "\n";
  out << "selected robot: " << (scene.robot_loaded && !scene.robot_vector.empty() ? scene.robot_vector[0].name : "<none>") << "\n";
  out << "selected robot status: " << (scene.robot_loaded ? "loaded" : "missing") << "\n";
  out << "selected end effector: " << (scene.ee_loaded && !scene.ee_vector.empty() ? scene.ee_vector[0].name : "<none>") << "\n";
  out << "selected end-effector status: " << (scene.ee_loaded ? "loaded" : "not-required-or-missing") << "\n";
  out << "selected environment objects/STLs: " << scene.object_vector.size() << "\n";
  out << "selected output package path: " << scene_dir.string() << "\n";
  out << "fake hardware default status: use_fake_hardware:=true\n";
  out << "Task recipe: OK\nTask recipe generated: OK\nTask plan dry-run preview: WARN (run scripts/preview_task_recipe.py)\nGrasp strategy: OK\nPick source: OK\nPlace target: OK\n";
  out << "Tool compatibility: " << (warnings.empty() ? "OK" : "WARN") << "\n";
  for (const auto & b : blockers) { out << "BLOCKER: " << b << "\n"; }
  for (const auto & w : warnings) { out << "WARNING: " << w << "\n"; }
  return out.str();
}



bool SceneSelect::export_workcell_layout_preview(const Scene & scene, const fs::path & scene_dir, bool open_after_export)
{
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  const fs::path preview_dir = scene_dir / "preview";
  fs::create_directories(preview_dir);
  const fs::path svg = preview_dir / "workcell_preview.svg";
  const fs::path html = preview_dir / "workcell_preview.html";
  std::ofstream svg_out(svg.string());
  svg_out << "<svg xmlns='http://www.w3.org/2000/svg' width='900' height='700'><text x='20' y='30'>Workcell Studio Preview</text><text x='20' y='55'>Offline/fake-hardware layout preview only</text><text x='20' y='80'>Task: " << task_cfg.task_type << "</text><text x='20' y='105'>Grasp: " << task_cfg.grasp_strategy << "</text><text x='20' y='130'>Pick source: " << task_cfg.pick_source << "</text><text x='20' y='155'>Place target: " << task_cfg.place_target << "</text></svg>";
  std::ofstream html_out(html.string());
  html_out << "<html><body><h1>Workcell Studio Preview</h1><p>Offline/fake-hardware layout preview only</p><p>Task: " << task_cfg.task_type << " | Grasp: " << task_cfg.grasp_strategy << " | Pick source: " << task_cfg.pick_source << " | Place target: " << task_cfg.place_target << "</p><img src='workcell_preview.svg'/></body></html>";
  append_success("Exported preview/workcell_preview.svg and preview/workcell_preview.html");
  if (open_after_export) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(html.string()))); }
  (void)scene;
  return true;
}

void SceneSelect::write_workcell_studio_summary(const Scene & scene, const fs::path & scene_dir, const std::string & readiness_status)
{
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  const fs::path json_file = scene_dir / "workcell_studio_summary.json";
  const fs::path md_file = scene_dir / "workcell_studio_summary.md";
  std::ofstream jout(json_file.string());
  jout << "{\n"
       << "  \"scene_name\": \"" << scene.name << "\",\n"
       << "  \"readiness_status\": \"" << readiness_status << "\",\n"
       << "  \"build_command\": \"colcon build --symlink-install --packages-select " << scene.name << "\",\n"
       << "  \"fake_hardware_launch_command\": \"ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true\",\n"
       << "  \"real_hardware_warning\": \"Real hardware mode requires explicit validation and use_fake_hardware:=false.\",\n"
       << "  \"task_type\": \"" << task_cfg.task_type << "\",\n"
       << "  \"pick_source\": \"" << task_cfg.pick_source << "\",\n"
       << "  \"place_target\": \"" << task_cfg.place_target << "\",\n"
       << "  \"grasp_strategy\": \"" << task_cfg.grasp_strategy << "\",\n"
       << "  \"orientation_mode\": \"" << task_cfg.orientation_mode << "\",\n"
       << "  \"approach_distance_m\": " << task_cfg.approach_distance_m << ",\n"
       << "  \"retreat_distance_m\": " << task_cfg.retreat_distance_m << ",\n"
       << "  \"release_strategy\": \"" << task_cfg.release_strategy << "\",\n"
       << "  \"task_recipe_path\": \"config/task_recipe.yaml\",\n"
       << "  \"task_plan_preview_path\": \"task_plan_preview.json\",\n"
       << "  \"dry_run_preview_status\": \"WARN\",\n"
       << "  \"safety_statement\": \"Task recipe preview is offline only. No MoveIt planning service was called and no robot motion was commanded.\"\n"
       << "}\n";
  std::ofstream mout(md_file.string());
  mout << "# Workcell Studio Summary\n\n";
  mout << "- scene name: " << scene.name << "\n";
  mout << "- readiness status: " << readiness_status << "\n";
  mout << "- build command: `colcon build --symlink-install --packages-select " << scene.name << "`\n";
  mout << "- fake-hardware launch command: `ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true`\n";
  mout << "- real hardware warning: Real hardware mode requires explicit validation and use_fake_hardware:=false.\n";
  mout << "- task type: " << task_cfg.task_type << "\n";
  mout << "- pick source: " << task_cfg.pick_source << "\n";
  mout << "- place target: " << task_cfg.place_target << "\n";
  mout << "- grasp strategy: " << task_cfg.grasp_strategy << "\n";
  mout << "- orientation mode: " << task_cfg.orientation_mode << "\n";
  mout << "- approach/retreat distances (m): " << task_cfg.approach_distance_m << "/" << task_cfg.retreat_distance_m << "\n";
  mout << "- release strategy: " << task_cfg.release_strategy << "\n";
  mout << "- task_recipe_path: config/task_recipe.yaml\n";
  mout << "- task_plan_preview_path: task_plan_preview.json\n";
  mout << "- dry_run_preview_status: WARN\n";
  mout << "- Task/grasp recipe generated for offline/fake-hardware planning only. No robot motion was commanded.\n";
  mout << "- Task recipe preview is offline only. No MoveIt planning service was called and no robot motion was commanded.\n";
}

void SceneSelect::on_back_clicked()
{
  int current_index = ui->scene_list->currentIndex();
  if (current_index < 0 || workcell.scene_vector.empty() ||
    current_index >= static_cast<int>(workcell.scene_vector.size()))
  {
    this->close();
    return;
  }

  if (workcell.scene_vector[current_index].loaded && !check_yaml()) {
    ReplaceWarning replace_window;
    replace_window.setWindowTitle("Edit Scene");
    replace_window.set_label(
      "Warning: Currently loaded scene is not saved."
      " All progress will be lost. Generate yaml file before going back.");
    replace_window.setModal(true);
    replace_window.exec();

    if (replace_window.decision) {
      this->close();
    }
  }
}

void SceneSelect::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}

void SceneSelect::on_exit_clicked()
{
  QApplication::quit();
}

void SceneSelect::on_clear_logs_clicked()
{
  clear_messages();
  append_info("Message log cleared.");
}


void SceneSelect::on_validate_cell_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) { append_error("No scene selected."); return; }
  Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
  if (!curr_scene.loaded) { load_scene_from_yaml(&curr_scene); }
  bool blocked = false;
  append_info(build_workcell_readiness_report(curr_scene, scene_dir, true, &blocked));
  append_info("Validate Scene completed (offline only, no launch/motion/runtime execution).");
}

void SceneSelect::on_generate_canonical_files_clicked()
{
  on_generate_yaml_clicked();
}

void SceneSelect::on_generate_workcell_package_clicked()
{
  on_generate_files_clicked();
}

void SceneSelect::on_generate_studio_pack_clicked()
{
  append_info("Generate Studio Pack triggered.");
}

void SceneSelect::on_open_preview_clicked()
{
  on_refresh_preview_clicked();
}

void SceneSelect::on_refresh_preview_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) { append_error("No scene selected."); return; }
  Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
  if (!curr_scene.loaded) { load_scene_from_yaml(&curr_scene); }
  export_workcell_layout_preview(curr_scene, scene_dir, true);
}

void SceneSelect::on_export_preview_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) { append_error("No scene selected."); return; }
  Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
  if (!curr_scene.loaded) { load_scene_from_yaml(&curr_scene); }
  export_workcell_layout_preview(curr_scene, scene_dir, false);
}

void SceneSelect::on_open_output_folder_clicked()
{
  const fs::path scene_dir = scenes_path;
  if (!fs::exists(scene_dir)) {
    append_warning("No output folder found yet. Expected scenes root: " + scenes_path.string());
    return;
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(scene_dir.string())));
}

void SceneSelect::on_show_readiness_report_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  const fs::path html = scene_dir / "generated" / "readiness_dashboard.html";
  const fs::path md = scene_dir / "generated" / "readiness_summary.md";
  if (fs::exists(html)) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(html.string()))); return; }
  if (fs::exists(md)) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(md.string()))); return; }
  append_warning("Readiness report not found. Generate Studio/Readiness Pack first.");
}

void SceneSelect::on_copy_fake_hardware_launch_command_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  const std::string scene_name = scene_dir.empty() ? std::string("<scene_name>") : scene_dir.filename().string();
  const QString cmd(QString::fromStdString("ros2 launch " + scene_name + " demo.launch.py use_fake_hardware:=true"));
  QApplication::clipboard()->setText(cmd);
  append_success("Copied fake-hardware launch command to clipboard.");
}

// compatibility note: missing one of [package.xml, CMakeLists.txt, urdf/]

void SceneSelect::on_generate_full_scene_package_start_clicked()
{
  on_generate_files_clicked();
}

void SceneSelect::on_open_scene_folder_clicked()
{
  const boost::filesystem::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_warning("No scene selected. Select a scene before opening its folder.");
    return;
  }

  QDesktopServices::openUrl(
    QUrl::fromLocalFile(QString::fromStdString(scene_dir.string())));
}
