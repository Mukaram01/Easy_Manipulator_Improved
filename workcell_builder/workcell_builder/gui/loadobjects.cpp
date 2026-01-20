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

#include "gui/loadobjects.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "gui/ui_loadobjects.h"
#include "include/file_functions.h"
#include "include/scene_parser.h"
#include "path_resolver.h"

LoadObjects::LoadObjects(const boost::filesystem::path & assets_root_path, QWidget * parent)
: QDialog(parent),
  ui(new Ui::LoadObjects)
{
  ui->setupUi(this);
  assets_root = assets_root_path.empty() ? PathResolver::assets_root() : assets_root_path;
  if (assets_root.empty()) {
    ui->object_name->setDisabled(true);
    ui->available_objects->setDisabled(true);
    ui->ok->setDisabled(true);
    ui->error_message->append(
      "<font color='red'> Error: No assets folder found."
      " Check your workspace assets layout. </font>");
    return;
  }
  environment_path = assets_root / "environment";
  if (!boost::filesystem::exists(environment_path)) {
    ui->object_name->setDisabled(true);
    ui->available_objects->setDisabled(true);
    ui->ok->setDisabled(true);
    ui->error_message->append(
      "<font color='red'> Error: No environment assets found."
      " Check assets/environment. </font>");
    return;
  }
  get_all_objects();
  if (available_objects.size() <= 0) {
    ui->object_name->setDisabled(true);
    ui->available_objects->setDisabled(true);
    ui->ok->setDisabled(true);
    ui->error_message->append(
      "<font color='red'> Error: No object to be loaded."
      " Is the object initially created with the Workcell Builder? </font>");
  } else {
    for (int i = 0; i < static_cast<int>(available_objects.size()); i++) {
      ui->available_objects->addItem(QString::fromStdString(available_objects[i]));
    }
    ui->object_name->setText(ui->available_objects->currentText());
  }
}

LoadObjects::~LoadObjects()
{
  delete ui;
}

void LoadObjects::on_ok_clicked()
{
  bool name_exists = std::find(
    current_object_names.begin(),
    current_object_names.end(),
    ui->object_name->text().toStdString()) != current_object_names.end();
  if (load_object_from_yaml(available_objects[ui->available_objects->currentIndex()]) &&
    !name_exists)
  {
    success = true;
    this->close();
  } else {
    if (name_exists) {
      ui->error_message->append("<font color='red'> Error: Object Name already exists </font>");
    } else {
      ui->error_message->append("<font color='red'> Error: Cannot Load Object </font>");
    }
  }
}

void LoadObjects::on_exit_clicked()
{
  success = false;
  this->close();
}

void LoadObjects::get_all_objects()
{
  if (environment_path.empty()) {
    return;
  }
  for (auto & entry : boost::filesystem::directory_iterator(environment_path)) {
    if (!boost::filesystem::is_directory(entry.path())) {
      continue;
    }
    const std::string folder_name = entry.path().filename().string();
    const std::string suffix = "_description";
    if (folder_name.size() <= suffix.size() ||
      folder_name.compare(folder_name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      continue;
    }
    const std::string object_name = folder_name.substr(0, folder_name.size() - suffix.size());
    if (boost::filesystem::exists(entry.path() / (object_name + ".yaml"))) {
      available_objects.push_back(object_name);
    }
  }
}

// Assumes you are at the environment directory
bool LoadObjects::load_object_from_yaml(std::string object_name)
{
  const boost::filesystem::path object_dir = environment_path / (object_name + "_description");
  if (!boost::filesystem::exists(object_dir)) {
    return false;
  }
  Object temp_object;
  YAML::Node yaml;
  // Load Yaml File.
  try {
    yaml = YAML::LoadFile((object_dir / (object_name + ".yaml")).string());
  } catch (YAML::BadFile & error) {
    return false;
  }

  YAML::Node ext_joint;
  for (YAML::iterator it = yaml.begin(); it != yaml.end(); ++it) {
    temp_object.name = it->first.as<std::string>();
    temp_object.ext_joint.child_object = it->first.as<std::string>();
    for (YAML::iterator in_object_it = it->second.begin(); in_object_it != it->second.end();
      ++in_object_it)
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
        for (int i = 0; i < static_cast<int>(temp_object.link_vector.size()); i++) {
          if (child_link.compare(temp_object.link_vector[i].name) == 0) {
            temp_object.ext_joint.child_link_pos = i;
            break;
          }
        }
      }
    }
  }

  chosen_object = temp_object;
  chosen_object.name = ui->object_name->text().toStdString();
  return true;
}


void LoadObjects::on_available_objects_currentIndexChanged(int index)
{
  ui->object_name->setText(QString::fromStdString(available_objects[index]));
}
