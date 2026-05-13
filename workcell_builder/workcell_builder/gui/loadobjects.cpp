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
#include <sstream>
#include <string>
#include <vector>
#include "workcell_builder_ui_utils.hpp"


#include "gui/ui_loadobjects.h"
#include "include/scene_parser.h"

namespace
{
bool get_map_string(const YAML::Node & node, const std::string & key, std::string & out)
{
  if (!node.IsMap() || !node[key] || !node[key].IsScalar()) {
    return false;
  }
  out = node[key].as<std::string>();
  return true;
}

std::string get_asset_yaml_error(const std::string & object_name, const YAML::Node & root)
{
  if (!root.IsDefined()) {
    return "YAML root is not defined";
  }
  if (!root.IsMap()) {
    return "YAML root must be a map";
  }
  if (!root[object_name]) {
    return "Missing object key '" + object_name + "'";
  }
  const YAML::Node object_node = root[object_name];
  if (!object_node.IsMap()) {
    return "Object node for '" + object_name + "' must be a map";
  }
  if (!object_node["links"]) {
    return "Missing required key: links";
  }
  if (!object_node["links"].IsMap()) {
    return "links must be a map";
  }
  for (YAML::const_iterator link_it = object_node["links"].begin(); link_it != object_node["links"].end(); ++link_it) {
    if (!link_it->second.IsMap()) {
      return "Each link entry must be a map";
    }
  }
  const std::string ext_joint_key = object_name + "_base_joint";
  if (!object_node[ext_joint_key] || !object_node[ext_joint_key].IsMap()) {
    return "Missing required ext joint map: " + ext_joint_key;
  }
  std::string child_link;
  if (!get_map_string(object_node[ext_joint_key], "child_link", child_link)) {
    return "Ext joint missing scalar child_link";
  }
  if (!object_node["links"][child_link]) {
    return "Ext joint child_link '" + child_link + "' is not present in links";
  }
  return "";
}

bool is_conveyor_asset(const std::string & object_name)
{
  return object_name.find("conveyor") != std::string::npos;
}

Link make_placeholder_link(const std::string & object_name)
{
  Link link;
  link.name = "base_link";
  link.is_visual = true;
  link.is_collision = true;

  Visual visual;
  visual.name = object_name + "_placeholder_visual";
  visual.geometry.is_stl = false;
  visual.geometry.shape = "box";
  visual.geometry.length = 0.6F;
  visual.geometry.breadth = 0.2F;
  visual.geometry.height = 0.1F;
  link.visual_vector.push_back(visual);

  Collision collision;
  collision.name = object_name + "_placeholder_collision";
  collision.geometry.is_stl = false;
  collision.geometry.shape = "box";
  collision.geometry.length = 0.6F;
  collision.geometry.breadth = 0.2F;
  collision.geometry.height = 0.1F;
  link.collision_vector.push_back(collision);
  return link;
}
}


LoadObjects::LoadObjects(QWidget * parent)
: QDialog(parent),
  ui(new Ui::LoadObjects)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
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
      const boost::filesystem::path yaml_path = filepath.path() / (temp_name + ".yaml");
      if (!boost::filesystem::exists(yaml_path)) {
        continue;
      }
      try {
        const YAML::Node yaml = YAML::LoadFile(yaml_path.string());
        const std::string parse_error = get_asset_yaml_error(temp_name, yaml);
        if (!parse_error.empty()) {
          std::cout << "[LoadObjects] Skipping invalid asset '" << temp_name << "': " << parse_error << std::endl;
          continue;
        }
        available_objects.push_back(temp_name);
      } catch (const YAML::Exception & error) {
        std::cout << "[LoadObjects] Skipping invalid YAML asset '" << temp_name << "': " << error.what() << std::endl;
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

  std::cout << "[LoadObjects] asset=" << object_name << " path=" << object_directory.string() << " yaml=" << yaml_path.string() << std::endl;
  try {
    yaml = YAML::LoadFile(yaml_path.string());
  } catch (YAML::BadFile & error) {
    append_error("Cannot load object YAML file: " + yaml_path.string());
    return false;
  } catch (const std::exception & error) {
    append_error("Error parsing object YAML file: " + std::string(error.what()));
    return false;
  }

  const std::string validation_error = get_asset_yaml_error(object_name, yaml);
  if (!validation_error.empty()) {
    append_error("Failed to load object asset '" + object_name + "': " + validation_error);
    return false;
  }

  YAML::Node object_node = yaml[object_name];
  YAML::Node ext_joint = object_node[object_name + "_base_joint"];
  temp_object.name = object_name;
  temp_object.ext_joint.child_object = object_name;
  size_t visual_count = 0;
  size_t collision_count = 0;
  try {
    for (YAML::iterator in_object_it = object_node.begin(); in_object_it != object_node.end(); ++in_object_it) {
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
      }
    }

  for (const Link & link : temp_object.link_vector) {
    visual_count += link.visual_vector.size();
    collision_count += link.collision_vector.size();
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
  } catch (const YAML::Exception & error) {
    append_error("Failed to load object asset '" + object_name + "': " + std::string(error.what()));
    return false;
  } catch (const std::exception & error) {
    append_error("Failed to load object asset '" + object_name + "': " + std::string(error.what()));
    return false;
  }

  if (temp_object.link_vector.empty() || temp_object.ext_joint.child_link_pos < 0 ||
    temp_object.ext_joint.child_link_pos >= static_cast<int>(temp_object.link_vector.size()) ||
    (visual_count == 0 && collision_count == 0))
  {
    if (!is_conveyor_asset(object_name)) {
      append_error("Asset validation failed for '" + object_name + "' from " + yaml_path.string());
      return false;
    }
    append_error("Conveyor asset is incomplete; using placeholder geometry. Source: " + yaml_path.string());
    temp_object.link_vector.clear();
    temp_object.link_vector.push_back(make_placeholder_link(object_name));
    temp_object.joint_vector.clear();
    temp_object.ext_joint.name = object_name + "_base_joint";
    temp_object.ext_joint.type = "fixed";
    temp_object.ext_joint.child_link_pos = 0;
    visual_count = 1;
    collision_count = 1;
  }

  std::cout << "[LoadObjects] asset=" << object_name << " type=environment object=" << temp_object.name
            << " links=" << temp_object.link_vector.size() << " visuals=" << visual_count
            << " collisions=" << collision_count << std::endl;

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
