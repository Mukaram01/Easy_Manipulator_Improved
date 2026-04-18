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
#include <QMessageBox>
#include <iostream>
#include <string>
#include <vector>

#include "gui/ui_loadobjects.h"
#include "include/scene_parser.h"

LoadObjects::LoadObjects(QWidget * parent)
: QDialog(parent),
  ui(new Ui::LoadObjects)
{
  ui->setupUi(this);
  success = false;
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

void LoadObjects::refresh_available_objects()
{
  ui->available_objects->clear();
  get_all_objects();
  const bool has_objects = !available_objects.empty();
  ui->object_name->setDisabled(!has_objects);
  ui->available_objects->setDisabled(!has_objects);
  ui->ok->setDisabled(!has_objects);

  if (!has_objects) {
    ui->error_message->append(
      "<font color='red'> Error: No object to be loaded."
      " Is the object initially created with the Workcell Builder? </font>");
    return;
  }

  for (const auto & available_object : available_objects) {
    ui->available_objects->addItem(QString::fromStdString(available_object));
  }
  ui->object_name->setText(ui->available_objects->currentText());
}

void LoadObjects::get_all_objects()
{
  available_objects.clear();
  try {
    const boost::filesystem::path resolved_assets_path = assets_path.empty() ?
      (workcell_path / "assets") : assets_path;
    const boost::filesystem::path environment_path = resolved_assets_path / "environment";
    if (!boost::filesystem::exists(environment_path) ||
      !boost::filesystem::is_directory(environment_path))
    {
      append_error(
        "Environment assets folder not found: " + environment_path.string());
      return;
    }

    for (const auto & filepath : boost::filesystem::directory_iterator(environment_path)) {
      if (!boost::filesystem::is_directory(filepath.path())) {
        continue;
      }
      std::string temp_name = filepath.path().filename().string();
      if (temp_name.size() <= 12 || temp_name.substr(temp_name.size() - 12) != "_description") {
        continue;
      }
      temp_name = temp_name.substr(0, temp_name.size() - 12);
      if (boost::filesystem::exists(filepath.path() / (temp_name + ".yaml"))) {
        available_objects.push_back(temp_name);
      }
    }
  } catch (const boost::filesystem::filesystem_error & error) {
    append_error("Filesystem error while enumerating objects: " + std::string(error.what()));
  } catch (const std::exception & error) {
    append_error("Error while enumerating objects: " + std::string(error.what()));
  }
}

bool LoadObjects::load_object_from_yaml(std::string object_name)
{
  Object temp_object;
  YAML::Node yaml;
  const boost::filesystem::path resolved_assets_path = assets_path.empty() ?
    (workcell_path / "assets") : assets_path;
  const boost::filesystem::path object_directory =
    resolved_assets_path / "environment" / (object_name + "_description");
  const boost::filesystem::path yaml_path = object_directory / (object_name + ".yaml");

  if (!boost::filesystem::exists(yaml_path)) {
    append_error("Object YAML file not found: " + yaml_path.string());
    return false;
  }

  try {
    yaml = YAML::LoadFile(yaml_path.string());
  } catch (YAML::BadFile & error) {
    append_error("Cannot load object YAML file: " + yaml_path.string());
    return false;
  } catch (const std::exception & error) {
    append_error("Error parsing object YAML file: " + std::string(error.what()));
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
  if (index >= 0 && index < static_cast<int>(available_objects.size())) {
    ui->object_name->setText(QString::fromStdString(available_objects[index]));
  }
}

void LoadObjects::append_error(const std::string & message)
{
  ui->error_message->append(QString::fromStdString("<font color='red'> " + message + " </font>"));
  QMessageBox::warning(this, "Load Object Error", QString::fromStdString(message));
}
