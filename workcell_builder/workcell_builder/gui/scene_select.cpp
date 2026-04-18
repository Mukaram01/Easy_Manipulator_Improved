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

fs::path root_from_override(const std::string & override_value)
{
  fs::path override_path(override_value);
  if (override_path.filename() == "scenes") {
    return override_path.parent_path();
  }
  return override_path;
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

  candidates.push_back(build_candidate(cwd, "current working directory"));
  candidates.push_back(build_candidate(cwd.parent_path(), "parent directory"));
  candidates.push_back(build_candidate(
    cwd / "src" / "easy_manipulation_deployment",
    "cwd/src/easy_manipulation_deployment"));
  candidates.push_back(build_candidate(cwd / "src", "cwd/src"));

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


SceneSelect::SceneSelect(QWidget * parent)
: QDialog(parent),
  ui(new Ui::SceneSelect)
{
  ui->setupUi(this);
  templates_path = get_default_templates_directory();
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

void SceneSelect::configure_startup_fallback_paths()
{
  if (!workcell_path.empty()) {
    return;
  }

  const fs::path fallback_root = select_scene_root(fs::current_path());
  workcell_path = fallback_root;
  scenes_path = workcell_path / "scenes";
  assets_path = get_runtime_assets_directory(workcell_path);
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

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell);
  templates_path = resolution.paths.templates_path;
  if (!resolution.success) {
    workcell_path.clear();
    scenes_path.clear();
    assets_path.clear();
    show_invalid_workcell_error(resolution.error);
    return;
  }

  workcell_path = resolution.paths.workcell_path;
  scenes_path = resolution.paths.scenes_path;
  assets_path = resolution.paths.assets_path;
  if (assets_path.empty()) {
    assets_path = workcell_path / "assets";
  }
  refresh_scenes(0);
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
    refresh_scenes(workcell.scene_vector.size() - 1);
  }
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
  fs::path workcell_path(scene_filepath.branch_path());
  generate_cmakelists(workcell_path, scene_name, ros_ver, ros_distro);
  generate_package_xml(workcell_path, scene_name, ros_ver, ros_distro);
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
  generate_scene_xacro(scene, (urdf_dir / "scene.urdf.xacro").string());
  if (scene.robot_loaded && scene.ee_loaded) {
    generate_armhand_xacro(scene.robot_vector[0], scene.ee_vector[0], scene.name);
  }
  if (scene.robot_loaded && !scene.ee_loaded) {  // no ee
    generate_armhand_xacro(scene.robot_vector[0], scene.name);
  }
  if (!scene.robot_loaded && !scene.ee_loaded) {  // no robot and ee
    generate_armhand_xacro(scene.name);
  }
  fs::path srdf_path =
    workcell_path / "scenes" / scene.name / "urdf" / "arm_hand.srdf.xacro";
  if (!boost::filesystem::exists(srdf_path)) {
    append_error("Failed to generate urdf/arm_hand.srdf.xacro.");
    return;
  }
  fs::path base_template_path = templates_path / ("ros" + std::to_string(workcell.ros_ver));
  fs::path launch_path = base_template_path / workcell.ros_distro / "launch";
  if (workcell.ros_distro.empty() || !boost::filesystem::exists(launch_path)) {
    launch_path = base_template_path / "launch";
  }
  fs::path target_path = scene_dir / "launch";
  copyDir(launch_path, target_path);

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
}
void SceneSelect::refresh_scenes(int latest_scene)
{
  if (latest_scene < 0) {latest_scene = 0;}
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();  // Clear the dropdown menu
  if (workcell.scene_vector.size() > 0) {  // There are scenes in the workcell
    ui->scene_list->setDisabled(false);     // Enable the dropdown menu
    for (int scene = 0; scene < static_cast<int>(workcell.scene_vector.size()); scene++) {
      ui->scene_list->addItem(QString::fromStdString(workcell.scene_vector[scene].name));
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
        refresh_scenes(0);
      } else {
        refresh_scenes(-1);
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
        refresh_scenes(ui->scene_list->currentIndex());
      } else {  // Scene was edited
        const fs::path scene_yaml_path = scenes_path / scene_window.scene.name;
        if (boost::filesystem::exists(scene_yaml_path)) {     // Scene name nvr change
          // Replace the current environment yaml
          GenerateYAML::generate_yaml(
            scene_window.scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          generate_scene_files(scene_window.scene);

        } else {
          // Delete previous scene folder
          delete_folder(scenes_path, curr_scene.name);
          // Generate new folder
          generate_scene_package(
            scenes_path, scene_window.scene.name, workcell.ros_ver, workcell.ros_distro);
          GenerateYAML::generate_yaml(
            scene_window.scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          generate_scene_files(scene_window.scene);
        }
        workcell.scene_vector[ui->scene_list->currentIndex()] = scene_window.scene;
        append_warning(
          "Scene edits were applied. Regenerate environment.yaml to save the latest scene state.");
        refresh_scenes(ui->scene_list->currentIndex());
      }
    } else {
      refresh_scenes(ui->scene_list->currentIndex());
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
          GenerateYAML::generate_yaml(
            target_scene,
            scene_yaml_path.string(), scenes_path, assets_path);
          append_success("environment.yaml generated successfully.");
        }
      } else {   // currently no yaml file, add one to scene folder
        GenerateYAML::generate_yaml(
          target_scene, scene_yaml_path.string(), scenes_path,
          assets_path);
        append_success("environment.yaml generated successfully.");
      }
    }
  } else {
    append_error("No scene selected to generate environment.yaml.");
  }
  check_scene();
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
bool SceneSelect::check_scene()
{
  bool has_yaml = check_yaml();
  bool files_loaded_proper = check_files();
  if (has_yaml) {
    append_success("Scene status: environment.yaml found.");
  } else {
    append_warning("Scene status: environment.yaml not found. Save the scene to enable future edits.");
  }

  if (files_loaded_proper) {
    append_success("Scene status: required files are present.");
  }

  if (has_yaml && files_loaded_proper) {
    append_success("Scene generation complete. You may exit this application.");
  }
  if (!has_yaml && files_loaded_proper) {
    append_warning(
      "Scene files were generated, but without environment.yaml this scene cannot be edited after exit.");
  }
  ui->exit->setDisabled(false);
  return true;
}
bool SceneSelect::check_files()
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

  if (!boost::filesystem::exists(launch_dir) || !boost::filesystem::exists(urdf_dir) ||
    !boost::filesystem::exists(cmake_file) || !boost::filesystem::exists(package_file))
  {
    append_error("Scene status: required files are missing.");

    if (!boost::filesystem::exists(launch_dir)) {
      append_error("Scene status: launch folder missing.");
    }
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
  } else {
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
  (void)index;
  check_scene();
}
void SceneSelect::on_generate_files_clicked()
{
  configure_startup_fallback_paths();
  if (ui->scene_list->currentIndex() >= 0) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[ui->scene_list->currentIndex()];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        append_error("Could not load scene from environment.yaml.");
        return;
      }
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
  } else {
    append_error("No scene selected to generate files from.");
  }
  check_scene();
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
