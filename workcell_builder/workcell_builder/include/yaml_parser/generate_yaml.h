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
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// For Yaml parsing
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "attributes/scene.h"
#include "attributes/environment.h"
#include "yaml_parser/externaljoint_parser.h"
#include "yaml_parser/object_parser.h"
#include "yaml_parser/origin_parser.h"
#include "tool_mount_defaults.hpp"

namespace {
Origin identity_origin()
{
  Origin origin{};
  origin.is_origin = true;
  origin.x = 0.0F;
  origin.y = 0.0F;
  origin.z = 0.0F;
  origin.roll = -1.5708F;
  origin.pitch = -1.5708F;
  origin.yaw = 0.0F;
  return origin;
}

bool is_valid_origin(const Origin & origin)
{
  const bool finite = std::isfinite(origin.x) && std::isfinite(origin.y) && std::isfinite(origin.z) &&
    std::isfinite(origin.roll) && std::isfinite(origin.pitch) && std::isfinite(origin.yaw);
  const bool sentinel = origin.x == -1.0F && origin.y == -1.0F && origin.z == -1.0F &&
    origin.roll == -1.0F && origin.pitch == -1.0F && origin.yaw == -1.0F;
  return origin.is_origin && finite && !sentinel;
}

Origin resolve_end_effector_mount_origin(const EndEffector & ee)
{
  if (is_valid_origin(ee.origin)) {
    return ee.origin;
  }

  const auto profile = workcell_builder::resolve_tool_mount_profile(ee.name, ee.ee_type);
  if (profile.apply_default) {
    Origin origin = identity_origin();
    origin.x = profile.xyz[0];
    origin.y = profile.xyz[1];
    origin.z = profile.xyz[2];
    origin.roll = profile.rpy[0];
    origin.pitch = profile.rpy[1];
    origin.yaw = profile.rpy[2];
    return origin;
  }

  return identity_origin();
}

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

std::string normalized_planner_id(const EndEffector & ee)
{
  return ee.planner_id.empty() ? ee.brand : ee.planner_id;
}

std::string normalized_gripper_type(const EndEffector & ee)
{
  return ee.gripper_type.empty() ? ee.ee_type : ee.gripper_type;
}

std::string normalized_grasp_frame(const EndEffector & ee)
{
  if (!ee.grasp_frame.empty()) {
    return ee.grasp_frame;
  }
  if (!ee.tcp_link.empty()) {
    return ee.tcp_link;
  }
  return ee.base_link;
}

std::string normalized_tcp_link(const EndEffector & ee)
{
  if (!ee.tcp_link.empty()) {
    return ee.tcp_link;
  }
  if (!ee.grasp_frame.empty()) {
    return ee.grasp_frame;
  }
  return ee.base_link;
}

bool normalized_spawn_gripper_controller(const EndEffector & ee)
{
  const std::string gripper_type = normalized_gripper_type(ee);
  if (gripper_type == "finger") {
    return true;
  }
  if (gripper_type == "suction" || gripper_type == "airpick") {
    return false;
  }
  return ee.spawn_gripper_controller;
}

void emit_origin_map(YAML::Emitter * out, const Origin & origin)
{
  *out << YAML::BeginMap;
  *out << YAML::Key << "xyz";
  *out << YAML::Value << YAML::Flow << YAML::BeginSeq << origin.x << origin.y << origin.z <<
    YAML::EndSeq;
  *out << YAML::Key << "rpy";
  *out << YAML::Value << YAML::Flow << YAML::BeginSeq << origin.roll << origin.pitch << origin.yaw <<
    YAML::EndSeq;
  *out << YAML::EndMap;
}
}  // namespace


class GenerateYAML
{
public:
  static bool generate_yaml(
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
      out << YAML::Key << "robot_mount";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "pose";
      out << YAML::Value;
      if (scene.robot_vector[0].origin.is_origin) {
        emit_origin_map(&out, scene.robot_vector[0].origin);
      } else {
        Origin fallback{};
        fallback.is_origin = true;
        fallback.x = 0.0F;
        fallback.y = 0.0F;
        fallback.z = 0.0F;
        fallback.roll = 0.0F;
        fallback.pitch = 0.0F;
        fallback.yaw = 0.0F;
        emit_origin_map(&out, fallback);
      }
      out << YAML::EndMap;
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
      out << YAML::Key << "planner_id";
      out << YAML::Value << normalized_planner_id(scene.ee_vector[0]);
      out << YAML::Key << "grasp_frame";
      out << YAML::Value << normalized_grasp_frame(scene.ee_vector[0]);
      out << YAML::Key << "tcp_link";
      out << YAML::Value << normalized_tcp_link(scene.ee_vector[0]);
      out << YAML::Key << "gripper_type";
      out << YAML::Value << normalized_gripper_type(scene.ee_vector[0]);
      out << YAML::Key << "spawn_gripper_controller";
      out << YAML::Value << normalized_spawn_gripper_controller(scene.ee_vector[0]);
      if (normalized_gripper_type(scene.ee_vector[0]) == "finger") {
        const int normalized_finger_count = scene.ee_vector[0].finger_count > 0 ?
          scene.ee_vector[0].finger_count : scene.ee_vector[0].attribute_1;
        if (normalized_finger_count > 0) {
          out << YAML::Key << "finger_count";
          out << YAML::Value << normalized_finger_count;
        }
      }
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

      OriginParser::generate_origin(&out, resolve_end_effector_mount_origin(scene.ee_vector[0]));
      const Origin ee_mount_origin = resolve_end_effector_mount_origin(scene.ee_vector[0]);
      out << YAML::Key << "tool_attachment";
      out << YAML::Value << YAML::BeginMap;
      const std::string default_joint_name =
        scene.ee_vector[0].name.empty() ? "ee_fixed_joint" : scene.ee_vector[0].name + "_fixed_joint";
      out << YAML::Key << "joint_name";
      out << YAML::Value << default_joint_name;
      out << YAML::Key << "parent_link";
      out << YAML::Value << scene.ee_vector[0].robot_link;
      out << YAML::Key << "child_link";
      out << YAML::Value << scene.ee_vector[0].base_link;
      out << YAML::Key << "origin";
      out << YAML::Value;
      emit_origin_map(&out, ee_mount_origin);
      out << YAML::EndMap;

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
        const int parent_object_pos = scene.object_vector[i].ext_joint.parent_obj_pos;
        const int parent_link_pos = scene.object_vector[i].ext_joint.parent_link_pos;
        const std::string & object_name = scene.object_vector[i].name;
        const bool parent_object_unset = parent_object_pos == -1;
        const bool parent_link_unset = parent_link_pos == -1;
        const bool explicitly_free_standing = parent_object_unset && parent_link_unset;
        const bool expects_parent_attachment = !explicitly_free_standing;

        if (parent_object_pos >= 0 &&
          parent_object_pos < static_cast<int>(scene.object_vector.size()) &&
          parent_link_pos >= 0 &&
          parent_link_pos < static_cast<int>(scene.object_vector[parent_object_pos].link_vector.size()))
        {
          ExternalJointParser::generate_ext_joints(
            &out, scene.object_vector[i].ext_joint,
            scene.object_vector[parent_object_pos].name,
            scene.object_vector[parent_object_pos].link_vector[
              parent_link_pos].name);
        } else if (explicitly_free_standing) {
          RCLCPP_DEBUG(
            rclcpp::get_logger("workcell_builder"),
            "Object '%s' is configured as free-standing (parent object/link unset); using world/world "
            "fallback.",
            object_name.c_str());
          ExternalJointParser::generate_ext_joints(
            &out, scene.object_vector[i].ext_joint, "world",
            "world");
        } else {
          std::stringstream error_stream;
          error_stream <<
            "Cannot generate external joint for object '" << object_name <<
            "': invalid parent selection context. Expected an attached parent "
            "(choose a concrete parent object and parent link in Add External Joint). "
            "Received parent_obj_pos=" << parent_object_pos <<
            ", parent_link_pos=" << parent_link_pos << ". ";

          if (expects_parent_attachment && (parent_object_unset || parent_link_unset)) {
            error_stream <<
              "Parent indices are unset; this means the attachment target was not fully selected. "
              "If this object should be attached, re-open Add External Joint and set both parent "
              "object and parent link. If the object should be free-standing, explicitly select "
              "'Connect to world' so both indices are -1.";
          } else if (parent_object_pos < 0 ||
            parent_object_pos >= static_cast<int>(scene.object_vector.size()))
          {
            error_stream <<
              "parent_obj_pos is out of range. Valid range is [0, " <<
              static_cast<int>(scene.object_vector.size()) - 1 << "].";
          } else {
            error_stream <<
              "parent_link_pos is out of range for parent object '" <<
              scene.object_vector[parent_object_pos].name << "'. Valid range is [0, " <<
              static_cast<int>(scene.object_vector[parent_object_pos].link_vector.size()) - 1 << "].";
          }

          RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", error_stream.str().c_str());
          return false;
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
    return true;
  }
};


#endif  // YAML_PARSER__GENERATE_YAML_H_
