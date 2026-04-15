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
  bool check_files();
  bool check_scene();
  void load_workcell(Workcell workcell);
  bool load_scene_from_yaml(Scene * input_scene);
  void refresh_scenes(int latest_scene);
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

  void on_back_clicked();

  void on_exit_clicked();

  void on_clear_logs_clicked();

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

  Ui::SceneSelect * ui;
  boost::filesystem::path scene_dir_for_current_selection() const;
  bool validate_description_xacros(const Scene & scene, const std::string & context_label);
  void configure_startup_fallback_paths();
  void show_invalid_workcell_error(const std::string & error_message);
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__SCENE_SELECT_H_
