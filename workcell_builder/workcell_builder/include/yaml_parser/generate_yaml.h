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


#ifndef YAML_PARSER__GENERATE_YAML_H_
#define YAML_PARSER__GENERATE_YAML_H_


// For file creation
#include <boost/filesystem.hpp>
#include <fstream>

// General
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

// For Yaml parsing
#include "yaml-cpp/yaml.h"

#include "attributes/scene.h"
#include "attributes/environment.h"
#include "yaml_parser/externaljoint_parser.h"
#include "yaml_parser/object_parser.h"
#include "yaml_parser/origin_parser.h"

namespace {
std::vector<std::string> normalize_robot_links(const Robot & robot)
{
  std::vector<std::string> links = robot.robot_links;
  if (robot.ee_link.empty() || robot.ee_link == "ee_link") {
    return links;
  }

  bool has_preferred = false;
  bool has_placeholder = false;
  for (const auto & link : links) {
    if (link == robot.ee_link) {
      has_preferred = true;
    } else if (link == "ee_link") {
      has_placeholder = true;
    }
  }

  if (has_placeholder) {
    links.erase(
      std::remove(links.begin(), links.end(), "ee_link"),
      links.end());
    if (!has_preferred) {
      links.push_back(robot.ee_link);
    }
  }

  return links;
}
}  // namespace


class GenerateYAML
{
public:
  static void generate_yaml(
    Scene scene, std::string filepath,
    boost::filesystem::path scene_filepath,
    boost::filesystem::path assets_filepath)
  {
    (void)scene_filepath;
    YAML::Emitter out;
    out << YAML::BeginMap;

    if (scene.robot_vector.size() > 0) {
      out << YAML::Key << "robot";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "name";
      out << YAML::Value << scene.robot_vector[0].name;
      out << YAML::Key << "brand";
      out << YAML::Value << scene.robot_vector[0].brand;
      out << YAML::Key << "filepath";
      out << YAML::Value << scene.robot_vector[0].filepath;
      out << YAML::Key << "base_link";
      out << YAML::Value << scene.robot_vector[0].base_link;
      out << YAML::Key << "ee_link";
      out << YAML::Value << scene.robot_vector[0].ee_link;

      if (scene.robot_vector[0].origin.is_origin) {
        OriginParser::generate_origin(&out, scene.robot_vector[0].origin);
      }
      out << YAML::Key << "links";
      out << YAML::Value << YAML::BeginSeq;

      const auto normalized_robot_links = normalize_robot_links(scene.robot_vector[0]);
      for (int i = 0; i < static_cast < int > (normalized_robot_links.size()); i++) {
        out << normalized_robot_links[i];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }

    if (scene.ee_vector.size() > 0) {
      out << YAML::Key << "end_effector";
      out << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "name";
      out << YAML::Value << scene.ee_vector[0].name;
      out << YAML::Key << "brand";
      out << YAML::Value << scene.ee_vector[0].brand;
      out << YAML::Key << "filepath";
      out << YAML::Value << scene.ee_vector[0].filepath;
      out << YAML::Key << "base_link";
      out << YAML::Value << scene.ee_vector[0].base_link;
      out << YAML::Key << "robot_link";
      out << YAML::Value << scene.ee_vector[0].robot_link;
      out << YAML::Key << "ee_type";
      out << YAML::Value << scene.ee_vector[0].ee_type;
      out << YAML::Key << "attributes";
      out << YAML::Value << YAML::BeginMap;
      if (scene.ee_vector[0].ee_type.compare("finger") == 0) {
        out << YAML::Key << "fingers";
        out << YAML::Value << scene.ee_vector[0].attribute_1;
      } else if (scene.ee_vector[0].ee_type.compare("suction") == 0) {
        out << YAML::Key << "array_width";
        out << YAML::Value << scene.ee_vector[0].attribute_1;
        out << YAML::Key << "array_height";
        out << YAML::Value << scene.ee_vector[0].attribute_2;
      }
      out << YAML::EndMap;

      if (scene.ee_vector[0].origin.is_origin) {
        OriginParser::generate_origin(&out, scene.ee_vector[0].origin);
      }

      out << YAML::Key << "links";
      out << YAML::Value << YAML::BeginSeq;

      for (int i = 0; i < static_cast < int > (scene.ee_vector[0].ee_links.size()); i++) {
        out << scene.ee_vector[0].ee_links[i];
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }
    const boost::filesystem::path env_assets_filepath = assets_filepath / "environment";

    if (scene.object_vector.size() > 0) {
      out << YAML::Key << "objects";
      out << YAML::Value << YAML::BeginMap;
      for (int i = 0; i < static_cast < int > (scene.object_vector.size()); i++) {
        const boost::filesystem::path object_desc_dir =
          env_assets_filepath / (scene.object_vector[i].name + "_description");
        const boost::filesystem::path object_yaml_path =
          object_desc_dir / (scene.object_vector[i].name + ".yaml");

        if (!boost::filesystem::exists(object_desc_dir)) {
          boost::filesystem::create_directory(object_desc_dir);
        }

        YAML::Emitter temp_obj_out;
        temp_obj_out << YAML::BeginMap;
        ObjectParser::generate_object(&temp_obj_out, scene.object_vector[i]);
        temp_obj_out << YAML::EndMap;

        std::ofstream objectfile(object_yaml_path.string());
        objectfile << temp_obj_out.c_str();
        objectfile.close();

        ObjectParser::generate_object(&out, scene.object_vector[i]);
      }
      out << YAML::EndMap;
    }

    if (scene.object_vector.size() > 0) {
      out << YAML::Key << "external joints";
      out << YAML::Value << YAML::BeginMap;
      for (int i = 0; i < static_cast < int > (scene.object_vector.size()); i++) {
        int parent_object_pos = scene.object_vector[i].ext_joint.parent_obj_pos;
        int parent_link_pos = scene.object_vector[i].ext_joint.parent_link_pos;

        if (
          parent_object_pos >= 0 &&
          parent_object_pos < static_cast<int>(scene.object_vector.size()) &&
          parent_link_pos >= 0 &&
          parent_link_pos < static_cast<int>(
            scene.object_vector[parent_object_pos].link_vector.size()))
        {
          ExternalJointParser::generate_ext_joints(
            &out, scene.object_vector[i].ext_joint,
            scene.object_vector[parent_object_pos].name,
            scene.object_vector[parent_object_pos].link_vector[
              parent_link_pos].name);
        } else {
          std::cerr <<
            "Warning: invalid external joint parent indices for object '" <<
            scene.object_vector[i].name << "' (parent_obj_pos=" <<
            parent_object_pos << ", parent_link_pos=" << parent_link_pos <<
            "). Falling back to world/world." << std::endl;
          ExternalJointParser::generate_ext_joints(
            &out, scene.object_vector[i].ext_joint, "world",
            "world");
        }
      }
      out << YAML::EndMap;
    }

    out << YAML::EndMap;

    const boost::filesystem::path environment_yaml_path =
      boost::filesystem::path(filepath) / "environment.yaml";
    std::ofstream myfile(environment_yaml_path.string());
    myfile << out.c_str();
    myfile.close();
  }
};


#endif  // YAML_PARSER__GENERATE_YAML_H_
