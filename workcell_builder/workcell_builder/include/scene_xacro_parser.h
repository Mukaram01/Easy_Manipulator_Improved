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


#ifndef SCENE_XACRO_PARSER_H_
#define SCENE_XACRO_PARSER_H_


#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "attributes/workcell.h"

namespace {
std::vector<std::string> description_package_candidates(const Robot & robot)
{
  std::vector<std::string> candidates;
  candidates.push_back(robot.name + "_description");
  if (robot.brand == "robotiq_3f_gripper") {
    candidates.push_back("robotiq_3f_gripper_description");
  } else if (robot.brand == "panda_robot") {
    candidates.push_back("moveit_resources_panda_description");
  } else if (robot.brand == "fanuc") {
    candidates.push_back("moveit_resources_fanuc_description");
  } else {
    candidates.push_back("moveit_resources_" + robot.name + "_description");
  }
  return candidates;
}

std::string resolve_robot_xacro_filename(const Robot & robot)
{
  if (robot.brand == "robotiq_3f_gripper") {
    return "robotiq-3f-gripper_articulated.urdf.xacro";
  }
  return robot.name + ".urdf.xacro";
}

std::string resolve_description_package(const Robot & robot)
{
  const std::string filename = resolve_robot_xacro_filename(robot);
  const auto candidates = description_package_candidates(robot);
  for (const auto & package_name : candidates) {
    try {
      const auto package_share = ament_index_cpp::get_package_share_directory(package_name);
      const boost::filesystem::path xacro_path =
        boost::filesystem::path(package_share) / "urdf" / filename;
      if (boost::filesystem::exists(xacro_path)) {
        return package_name;
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  return candidates.front();
}

std::string resolve_ee_description_package(const EndEffector & ee)
{
  if (ee.brand == "robotiq_3f_gripper") {
    return "robotiq_3f_gripper_description";
  }
  return ee.name + "_description";
}

std::string resolve_ee_xacro_filename(const EndEffector & ee)
{
  if (ee.brand == "robotiq_3f_gripper") {
    return "robotiq-3f-gripper_articulated.urdf.xacro";
  }
  return ee.name + "_gripper.urdf.xacro";
}

std::string resolve_ee_xacro_macro(const EndEffector & ee)
{
  if (ee.brand == "robotiq_3f_gripper") {
    return "robotiq-3f-gripper_articulated";
  }
  return ee.name + "_gripper";
}
}  // namespace


void generate_scene_xacro(Scene scene)
{
  std::ofstream MyFile("scene.urdf.xacro");
  MyFile << "<?xml version=\"1.0\" ?> \n\n";
  MyFile << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" +
    scene.name + "\">\n\n";  // Change it if you are generating multiple robots
  MyFile << " <link name=\"world\"/>\n\n";  // Declare world joint

  if (scene.robot_loaded) {
    for (int i = 0; i < static_cast < int > (scene.robot_vector.size()); i++) {
      bool add_world_joint = true;
      if (scene.robot_vector[i].brand.compare("universal_robot") == 0) {
        // Current ur packages are done differently
        add_world_joint = false;
        MyFile << " <xacro:include filename=\"$(find ur_description)/urdf/ur.urdf.xacro\"/>\n";
        MyFile << " <xacro:ur_robot name=\"" + scene.robot_vector[i].name + "\" tf_prefix=\"\" " +
          "parent=\"" + scene.robot_vector[i].parent_link + "\" " +
          "joint_limits_parameters_file=\"$(find ur_description)/config/" +
          scene.robot_vector[i].name + "/joint_limits.yaml\" " +
          "kinematics_parameters_file=\"$(find ur_description)/config/" +
          scene.robot_vector[i].name + "/default_kinematics.yaml\" " +
          "physical_parameters_file=\"$(find ur_description)/config/" +
          scene.robot_vector[i].name + "/physical_parameters.yaml\" " +
          "visual_parameters_file=\"$(find ur_description)/config/" +
          scene.robot_vector[i].name + "/visual_parameters.yaml\" " +
          "safety_limits=\"false\" safety_pos_margin=\"0.15\" safety_k_position=\"20\" " +
          "force_abs_paths=\"false\">\n";
        if (scene.robot_vector[i].origin.is_origin) {
          MyFile << "\t<origin xyz=\"" + std::to_string(scene.robot_vector[i].origin.x) + " " +
            std::to_string(scene.robot_vector[i].origin.y) + " " + std::to_string(
            scene.robot_vector[i].origin.z) + "\" rpy=\"" + std::to_string(
            scene.robot_vector[i].origin.roll) + " " + std::to_string(
            scene.robot_vector[i].origin.pitch) + " " + std::to_string(
            scene.robot_vector[i].origin.yaw) + "\"/>\n";
        } else {
          MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        }
        MyFile << " </xacro:ur_robot>\n";
      } else {
        const std::string package_name = resolve_description_package(scene.robot_vector[i]);
        const std::string xacro_filename = resolve_robot_xacro_filename(scene.robot_vector[i]);
        MyFile << " <xacro:include filename=\"$(find " + package_name +
          ")/urdf/" + xacro_filename + "\"/>\n";
        MyFile << " <xacro:" + scene.robot_vector[i].name + "_robot/>\n";
      }

      if (add_world_joint) {
        MyFile << "  <joint name=\"world_" + scene.robot_vector[i].name + "\" type=\"" +
          scene.robot_vector[i].parent_robot_joint_type + "\">\n";
        MyFile << "\t<parent link=\"" + scene.robot_vector[i].parent_link + "\" />\n";
        MyFile << "\t<child link=\"" + scene.robot_vector[i].base_link + "\" />\n";
        if (scene.robot_vector[i].origin.is_origin) {
          MyFile << "\t<origin xyz=\"" + std::to_string(scene.robot_vector[i].origin.x) + " " +
            std::to_string(scene.robot_vector[i].origin.y) + " " + std::to_string(
            scene.robot_vector[i].origin.z) + "\" rpy=\"" + std::to_string(
            scene.robot_vector[i].origin.roll) + " " + std::to_string(
            scene.robot_vector[i].origin.pitch) + " " + std::to_string(
            scene.robot_vector[i].origin.yaw) + "\"/>\n";
        } else {
          MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        }
        MyFile << "  </joint>\n";
      }
    }
    MyFile << "\n";
  }

  if (scene.ee_loaded) {
    for (int i = 0; i < static_cast < int > (scene.ee_vector.size()); i++) {
      const std::string ee_package = resolve_ee_description_package(scene.ee_vector[i]);
      const std::string ee_xacro = resolve_ee_xacro_filename(scene.ee_vector[i]);
      const std::string ee_macro = resolve_ee_xacro_macro(scene.ee_vector[i]);
      MyFile << " <xacro:include filename=\"$(find " + ee_package +
        ")/urdf/" + ee_xacro + "\"/>\n";

      MyFile << " <xacro:" + ee_macro + " prefix=\"\" parent=\"" +
        scene.robot_vector[scene.ee_vector[i].robot_pos].ee_link + "\">\n";
      if (scene.ee_vector[i].origin.is_origin) {
        std::cout << "Xacro parser has origin" << std::endl;
        MyFile << "\t<origin xyz=\"" + std::to_string(scene.ee_vector[i].origin.x) + " " +
          std::to_string(scene.ee_vector[i].origin.y) + " " + std::to_string(
          scene.ee_vector[i].origin.z) + "\" rpy=\"" +
          std::to_string(scene.ee_vector[i].origin.roll) + " " + std::to_string(
          scene.ee_vector[i].origin.pitch) + " " + std::to_string(scene.ee_vector[i].origin.yaw) +
          "\"/>\n";
      } else {
        std::cout << "Xacro parser has no origin" << std::endl;
        MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
      }
      MyFile << " </xacro:" + ee_macro + ">\n";
    }
    MyFile << "\n";
  }

  for (int i = 0; i < static_cast < int > (scene.object_vector.size()); i++) {
    std::string parent_link;
    if (scene.object_vector[i].ext_joint.parent_obj_pos >= 0 ||
      scene.object_vector[i].ext_joint.parent_link_pos >= 0)
    {
      parent_link =
        scene.object_vector[scene.object_vector[i].ext_joint.parent_obj_pos].link_vector[scene.
          object_vector[i].ext_joint.parent_link_pos].name;
    } else {
      parent_link = "world";
    }
    MyFile << " <xacro:include filename=\"$(find " + scene.object_vector[i].name +
      "_description)/urdf/" + scene.object_vector[i].name + ".urdf.xacro\"/>\n";
    MyFile << " <xacro:" + scene.object_vector[i].name + " prefix=\"\" parent=\"" + parent_link +
      "\">\n";
    if (scene.object_vector[i].ext_joint.origin.is_origin) {
      MyFile << "\t<origin xyz=\"" + std::to_string(scene.object_vector[i].ext_joint.origin.x) +
        " " + std::to_string(scene.object_vector[i].ext_joint.origin.y) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.z) + "\" rpy=\"" + std::to_string(
        scene.object_vector[i].ext_joint.origin.roll) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.pitch) + " " + std::to_string(
        scene.object_vector[i].ext_joint.origin.yaw) + "\"/>\n";
    } else {
      MyFile << "\t<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    }
    MyFile << " </xacro:" + scene.object_vector[i].name + ">\n";
  }
  MyFile << "\n";
  MyFile << "</robot>";
  // connect EE to robot
}
#endif  // SCENE_XACRO_PARSER_H_
