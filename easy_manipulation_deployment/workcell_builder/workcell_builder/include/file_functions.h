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


#ifndef FILE_FUNCTIONS_H_
#define FILE_FUNCTIONS_H_

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "attributes/workcell.h"

namespace fs = boost::filesystem;

inline void safe_chdir(const fs::path & p)
{
  try {
    if (fs::exists(p)) {
      fs::current_path(p);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Path %s does not exist", p.string().c_str());
    }
  } catch (fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
}

inline void ensure_parent(const fs::path & p)
{
  try {
    fs::create_directories(p.parent_path());
  } catch (fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
}

void find_replace(
  std::string example_text, std::string target_text, std::string current_text,
  std::string replaced_text)
{
  std::string file_contents;
  std::ifstream filein(example_text);
  std::ofstream fileout(target_text);
  for (char ch; filein.get(ch); file_contents.push_back(ch)) {
  }

  // This searches the file for the first occurence of the morn string.
  auto pos = file_contents.find(current_text);
  int counter = 0;
  // std::cout<<file_contents<<std::endl;
  while (pos != std::string::npos) {
    file_contents.replace(pos, current_text.length(), replaced_text);
    pos = file_contents.find(current_text);
    std::cout << "position: " << pos << std::endl;
    counter++;
    if (counter > 100) {
      break;
    }
  }
  fileout << file_contents;
  std::remove(example_text.c_str());
}

// void GenerateCMakeLists(boost::filesystem::path workcell_filepath,
// boost::filesystem::path package_filepath,
// std::string package_name,int ros_ver)
void generate_cmakelists(
  fs::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  fs::path package_filepath(workcell_filepath / "scenes" / package_name);
  fs::path example_file;
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    example_file = fs::path(share) / "templates" /
      ("ros" + std::to_string(ros_ver)) / "CMakeLists_example.txt";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  fs::path target_location(package_filepath / "CMakeLists_example.txt");
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
    find_replace("CMakeLists_example.txt", "CMakeLists.txt",
      "workcellexample", package_name);
  } else {
    std::ofstream out((package_filepath / "CMakeLists.txt").string());
    if (out.is_open()) {
      out << "cmake_minimum_required(VERSION 3.5)\n";
      out << "project(" << package_name << ")\n";
      out << "find_package(ament_cmake REQUIRED)\n";
      out << "install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})\n";
      out << "ament_package()\n";
    }
  }
}

void delete_folder(fs::path scene_filepath, std::string scene_name)
{
  safe_chdir(scene_filepath);
  try {
    if (fs::exists(scene_name)) {
      fs::remove_all(scene_name);
    }
  } catch(fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
}

void generate_package_xml(
  fs::path workcell_filepath, std::string package_name,
  int ros_ver)
{
  fs::path package_filepath(workcell_filepath / "scenes" / package_name);
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
bool copyDir(fs::path const & source, fs::path const & destination)
{
  try {
    if (!fs::exists(source) || !fs::is_directory(source)) {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Source directory %s does not exist or is not a directory.",
        source.string().c_str());
      return false;
    }
    if (fs::exists(destination)) {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Destination directory %s already exists.", destination.string().c_str());
    }
    if (!fs::create_directory(destination)) {
      RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
        "Unable to create destination directory %s", destination.string().c_str());
    }
  } catch(fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
    return false;
  }
  try {
    for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file) {
      fs::path current(file->path());
      if (fs::is_directory(current)) {
        if (!copyDir(current, destination / current.filename())) {
          return false;
        }
      } else {
        fs::path dst = destination / current.filename();
        ensure_parent(dst);
        try {
          fs::copy_file(current, dst, fs::copy_option::overwrite_if_exists);
        } catch(fs::filesystem_error const & e) {
          RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
        }
      }
    }
  } catch(fs::filesystem_error const & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
    return false;
  }
  return true;
}


#endif  // FILE_FUNCTIONS_H_
