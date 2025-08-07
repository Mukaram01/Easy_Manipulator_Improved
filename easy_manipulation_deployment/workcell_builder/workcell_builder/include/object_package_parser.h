// Copyright 2020 Advanced Remanufacturing and Technology Centre //NOLINT
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


#ifndef OBJECT_PACKAGE_PARSER_H_
#define OBJECT_PACKAGE_PARSER_H_


#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "attributes/object.h"
#include "include/file_functions.h"

namespace fs = boost::filesystem;

bool package_exists(Object object);
void GenerateObjectPackageXML(
  fs::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  fs::path package_filepath(
    workcell_filepath / "assets" / "environment" / package_name);
  fs::path example_file;
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    example_file = fs::path(share) / "templates" /
      ("ros" + std::to_string(ros_ver)) / "package_example.xml";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  fs::path target_location(package_filepath / "package_example.xml");
  ensure_parent(target_location);
  bool copied = false;
  try {
    if (!example_file.empty() && fs::exists(example_file)) {
      fs::copy_file(example_file, target_location, fs::copy_option::overwrite_if_exists);
      copied = true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Template %s missing", example_file.string().c_str());
    }
  } catch(fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  safe_chdir(package_filepath);
  if (copied) {
    find_replace("package_example.xml", "package.xml", "workcellexample", package_name);
  } else {
    std::ofstream out((package_filepath / "package.xml").string());
    if (out.is_open()) {
      const char * pkgxml =
        "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  <name>%s</name>\n"
        "  <version>0.0.0</version>\n  <description>Auto-generated scene</description>\n"
        "  <maintainer email=\"user@example.com\">user</maintainer>\n"
        "  <license>Apache-2.0</license>\n</package>\n";
      char buffer[1024];
      std::snprintf(buffer, sizeof(buffer), pkgxml, package_name.c_str());
      out << buffer;
    }
  }
}
void GenerateObjectCMakeLists(
  fs::path workcell_filepath,
  fs::path package_filepath, std::string package_name,
  int ros_ver)
{
  fs::path example_file;
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    example_file = fs::path(share) / "templates" /
      ("ros" + std::to_string(ros_ver)) / "CMakeLists_object_example.txt";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  fs::path target_location(package_filepath / "CMakeLists_object_example.txt");
  ensure_parent(target_location);
  bool copied = false;
  try {
    if (!example_file.empty() && fs::exists(example_file)) {
      fs::copy_file(example_file, target_location, fs::copy_option::overwrite_if_exists);
      copied = true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Template %s missing", example_file.string().c_str());
    }
  } catch(fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  safe_chdir(package_filepath);
  if (copied) {
    find_replace("CMakeLists_object_example.txt", "CMakeLists.txt", "workcellexample", package_name);
  } else {
    std::ofstream out((package_filepath / "CMakeLists.txt").string());
    if (out.is_open()) {
      out << "cmake_minimum_required(VERSION 3.5)\n";
      out << "project(" << package_name << ")\n";
      out << "find_package(ament_cmake REQUIRED)\n";
      out << "install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})\n";
      out << "ament_package()\n";
    }
  }
}
void generate_object_package(fs::path workcell_filepath, Object object, int ros_ver)
{
  safe_chdir(workcell_filepath);
  safe_chdir("assets/environment");
  if (!package_exists(object)) {
    std::cout << "generate_object_package: " << fs::current_path() << std::endl;
    safe_chdir(object.name + std::string("_description"));
    fs::path package_filepath(
      workcell_filepath / "assets" / "environment" /
      (object.name + std::string("_description")));
    GenerateObjectCMakeLists(
      workcell_filepath, package_filepath, object.name + "_description",
      ros_ver);
    GenerateObjectPackageXML(workcell_filepath, object.name + "_description", ros_ver);

    fs::create_directory("urdf");
    fs::create_directory("meshes");
    safe_chdir("meshes");
    fs::create_directory("visual");
    fs::create_directory("collision");
    fs::path collision_path(fs::current_path() / "collision");
    fs::path visual_path(fs::current_path() / "visual");
    for (Link link : object.link_vector) {
      if (link.is_visual && link.visual_vector[0].geometry.is_stl) {
        std::string stl_name;
        for (auto it = link.visual_vector[0].geometry.filepath.crbegin();
          it != link.visual_vector[0].geometry.filepath.crend(); ++it)
        {
          if (*it != '/') {
            stl_name = std::string(1, *it) + stl_name;
          } else {
            break;
          }
        }
        fs::path link_visual_path = visual_path / stl_name;
        safe_chdir(visual_path);
        if (!fs::exists(stl_name)) {
          ensure_parent(link_visual_path);
          try {
            fs::copy_file(link.visual_vector[0].geometry.filepath, link_visual_path);
          } catch(fs::filesystem_error const & e) {
            RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
          }
        }
      }
      if (link.is_collision && link.collision_vector[0].geometry.is_stl) {
        std::string stl_name;
        for (auto it = link.collision_vector[0].geometry.filepath.crbegin();
          it != link.collision_vector[0].geometry.filepath.crend(); ++it)
        {
          if (*it != '/') {
            stl_name = std::string(1, *it) + stl_name;
          } else {
            break;
          }
        }
        fs::path link_collision_path = collision_path / stl_name;
        safe_chdir(collision_path);
        if (!fs::exists(stl_name)) {
          ensure_parent(link_collision_path);
          try {
            fs::copy_file(
              link.collision_vector[0].geometry.filepath,
              link_collision_path);
          } catch(fs::filesystem_error const & e) {
            RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
          }
        }
      }
    }
  }
}

bool package_exists(Object object)
{
  std::cout << "Package exists: " << fs::current_path() << std::endl;
  fs::path temp_path(fs::current_path());
  std::string package_name = object.name + "_description";
  if (fs::is_directory(package_name)) {  // folder exists
    safe_chdir(package_name);
    if (fs::is_directory("urdf")) {
      safe_chdir("urdf");
    } else {  // no urdf folder
      std::cout << "No URDF folder" << std::endl;
      safe_chdir(temp_path);
      return false;
    }
    if (!fs::exists("CMakeLists.txt") || !fs::exists("package.xml")) {
      std::cout << "No CMakeLists or package.xml available" << std::endl;
      safe_chdir(temp_path);
      return false;
    }
  } else {
    std::cout << "No description folders" << std::endl;
    safe_chdir(temp_path);
    return false;
  }
  std::cout << "Object package ok. " << std::endl;
  safe_chdir(temp_path);
  return true;
}
#endif  // OBJECT_PACKAGE_PARSER_H_
