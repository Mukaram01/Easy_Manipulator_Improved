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
#include "include/default_asset_paths.h"
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
  int ros_ver, const std::string & ros_distro)
{
  fs::path package_filepath(workcell_filepath / "scenes" / package_name);
  fs::path example_file;
  try {
    fs::path base_template_path = get_default_templates_directory() /
      ("ros" + std::to_string(ros_ver));
    fs::path distro_template_path = base_template_path / ros_distro / "CMakeLists_example.txt";
    if (!ros_distro.empty() && fs::exists(distro_template_path)) {
      example_file = distro_template_path;
    } else {
      example_file = base_template_path / "CMakeLists_example.txt";
    }
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
      out << "\n";
      out << "# Default to C++17\n";
      out << "if(NOT CMAKE_CXX_STANDARD)\n";
      out << "  set(CMAKE_CXX_STANDARD 17)\n";
      out << "endif()\n";
      out << "\n";
      out << "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n";
      out << "  add_compile_options(-Wall -Wextra -Wpedantic)\n";
      out << "endif()\n";
      out << "\n";
      out << "# find dependencies\n";
      out << "find_package(ament_cmake REQUIRED)\n";
      out << "find_package(rosidl_default_runtime REQUIRED)\n";
      out << "install(DIRECTORY launch\n";
      out << "  DESTINATION share/${PROJECT_NAME}\n";
      out << ")\n";
      out << "\n";
      out << "install(DIRECTORY urdf\n";
      out << "  DESTINATION share/${PROJECT_NAME}\n";
      out << ")\n";
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
  int ros_ver, const std::string & ros_distro)
{
  fs::path package_filepath(workcell_filepath / "scenes" / package_name);
  fs::path example_file;
  try {
    fs::path base_template_path = get_default_templates_directory() /
      ("ros" + std::to_string(ros_ver));
    fs::path distro_template_path = base_template_path / ros_distro / "package_example.xml";
    if (!ros_distro.empty() && fs::exists(distro_template_path)) {
      example_file = distro_template_path;
    } else {
      example_file = base_template_path / "package_example.xml";
    }
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
        "<?xml version=\"1.0\"?>\n"
        "<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" "
        "schematypens=\"http://www.w3.org/2001/XMLSchema\"?>\n"
        "<package format=\"3\">\n"
        "  <name>%s</name>\n"
        "  <version>2.0.0</version>\n"
        "  <description>%s description</description>\n"
        "  <maintainer email=\"\">Mukaram01</maintainer>\n"
        "  <license>Apache-2.0</license>\n"
        "\n"
        "  <buildtool_depend>ament_cmake</buildtool_depend>\n"
        "\n"
        "  <exec_depend>xacro</exec_depend>\n"
        "  <exec_depend>launch</exec_depend>\n"
        "  <exec_depend>launch_ros</exec_depend>\n"
        "  <exec_depend>rviz2</exec_depend>\n"
        "  <exec_depend>tf2_ros</exec_depend>\n"
        "  <exec_depend>ament_index_python</exec_depend>\n"
        "  <exec_depend>python3-yaml</exec_depend>\n"
        "\n"
        "  <test_depend>ament_lint_auto</test_depend>\n"
        "  <test_depend>ament_lint_common</test_depend>\n"
        "\n"
        "  <export>\n"
        "    <build_type>ament_cmake</build_type>\n"
        "  </export>\n"
        "</package>\n";
      char buffer[1024];
      std::snprintf(buffer, sizeof(buffer), pkgxml, package_name.c_str(), package_name.c_str());
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
