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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_

#include <QDialog>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "attributes/workcell.h"
#include "scene_select_paths.h"


namespace Ui
{
  class SceneSelect; // NOLINT
}

class SceneSelect: public QDialog
{
  Q_OBJECT

public:
  boost::filesystem::path scenes_path;
  boost::filesystem::path templates_path;
  boost::filesystem::path workcell_path;
  boost::filesystem::path assets_path;

  Workcell workcell;
  void generate_scene_package(
    boost::filesystem::path scene_filepath, std::string scene_name,
    int ros_ver, const std::string & ros_distro);
  void generate_scene_files(Scene scene);
  bool check_yaml();
  bool check_files(bool strict = true);
  bool check_scene(bool strict = true);
  void load_workcell(Workcell workcell);
  bool load_scene_from_yaml(Scene * input_scene);
  void refresh_scenes(int latest_scene, bool scaffold_only_status = false);
  void keyPressEvent(QKeyEvent * e);

  // void GeneratePackageXML(std::string target_filepath ,std::string package_name,int ros_ver);
  // void GenerateCMakeLists(std::string target_filepath ,std::string package_name,int ros_ver);
  explicit SceneSelect(QWidget * parent = nullptr);
  ~SceneSelect();

private slots:
  void on_add_scene_clicked();

  void on_delete_scene_clicked();

  void on_edit_scene_clicked();

  void on_generate_yaml_clicked();

  void on_scene_list_currentIndexChanged(int index);

  void on_generate_files_clicked();
  void on_generate_full_scene_package_start_clicked();
  void on_open_scene_folder_clicked();

  void on_back_clicked();

  void on_exit_clicked();

  void on_clear_logs_clicked();
  void on_validate_cell_clicked();
  void on_generate_canonical_files_clicked();
  void on_generate_workcell_package_clicked();
  void on_generate_studio_pack_clicked();
  void on_open_preview_clicked();
  void on_open_output_folder_clicked();
  void on_show_readiness_report_clicked();
  void on_copy_fake_hardware_launch_command_clicked();
  void on_refresh_preview_clicked();
  void on_export_preview_clicked();
  void on_browse_scenes_folder_clicked();
  void on_refresh_scenes_button_clicked();

private:
  enum class MessageLevel
  {
    Info,
    Warning,
    Error,
    Success
  };

  void append_message(MessageLevel level, const std::string & message);
  void append_info(const std::string & message);
  void append_warning(const std::string & message);
  void append_error(const std::string & message);
  void append_success(const std::string & message);
  void clear_messages();
  void refresh_scene_status(bool strict, const std::string & trigger);
  std::string build_workcell_readiness_report(const Scene & scene, const boost::filesystem::path & scene_dir, bool strict, bool * blocked = nullptr);
  void write_workcell_studio_summary(const Scene & scene, const boost::filesystem::path & scene_dir, const std::string & readiness_status);
  bool export_workcell_layout_preview(const Scene & scene, const boost::filesystem::path & scene_dir, bool open_after_export);

  Ui::SceneSelect * ui;
  boost::filesystem::path scene_dir_for_current_selection() const;
  bool validate_description_xacros(const Scene & scene, const std::string & context_label);
  void configure_startup_fallback_paths();
  void show_invalid_workcell_error(const std::string & error_message);
  void discover_scene_packages_on_startup();
  void update_scene_browser_status(const std::string & note = "");

  int scaffold_scene_index_ = -1;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_
