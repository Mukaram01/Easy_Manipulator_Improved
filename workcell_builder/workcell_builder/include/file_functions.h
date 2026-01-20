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
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "attributes/workcell.h"

namespace fs = boost::filesystem;

inline fs::path resolve_assets_root(const fs::path & start_path);

inline fs::path resolve_templates_root()
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    const fs::path templates_root = fs::path(share) / "templates";
    if (fs::exists(templates_root) && fs::is_directory(templates_root)) {
      return templates_root;
    }
  } catch (const std::exception &) {
  }
  return {};
}

inline fs::path resolve_workspace_root(const fs::path & start_path)
{
  if (const char * env_root = std::getenv("WORKCELL_BUILDER_ROOT")) {
    fs::path candidate(env_root);
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate;
    }
  }
  for (fs::path current = start_path; !current.empty(); current = current.parent_path()) {
    if (current.filename() == "src") {
      const fs::path candidate = current.parent_path();
      if (fs::exists(candidate) && fs::is_directory(candidate)) {
        return candidate;
      }
    }
    if (current == current.root_path()) {
      break;
    }
  }
  return {};
}

inline fs::path resolve_scenes_root(const fs::path & start_path, const fs::path & workspace_root)
{
  for (fs::path current = start_path; !current.empty(); current = current.parent_path()) {
    const fs::path candidate = current / "scenes";
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate;
    }
    if (current == current.root_path()) {
      break;
    }
  }
  if (!workspace_root.empty()) {
    const fs::path candidate = workspace_root / "scenes";
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate;
    }
  }
  return {};
}

inline std::string path_or_unresolved(const fs::path & path)
{
  return path.empty() ? std::string("<unresolved>") : path.string();
}

inline void log_missing_path(
  const fs::path & missing_path,
  const std::string & guidance,
  const fs::path & context_path,
  const fs::path & scenes_root_override = {})
{
  const fs::path assets_root = resolve_assets_root(context_path);
  const fs::path templates_root = resolve_templates_root();
  const fs::path workspace_root = resolve_workspace_root(context_path);
  const fs::path scenes_root = scenes_root_override.empty() ?
    resolve_scenes_root(context_path, workspace_root) : scenes_root_override;
  RCLCPP_ERROR(
    rclcpp::get_logger("workcell_builder"),
    "Missing path: %s. assets_root=%s templates_root=%s workspace_root=%s scenes_root=%s. "
    "Guidance: %s",
    missing_path.string().c_str(),
    path_or_unresolved(assets_root).c_str(),
    path_or_unresolved(templates_root).c_str(),
    path_or_unresolved(workspace_root).c_str(),
    path_or_unresolved(scenes_root).c_str(),
    guidance.c_str());
}

inline bool safe_chdir(
  const fs::path & p,
  const fs::path & context_path = {},
  const std::string & guidance =
  "Ensure the directory exists. If needed, set WORKCELL_BUILDER_ROOT to your workspace root.")
{
  const fs::path resolved_context = context_path.empty() ? p : context_path;
  if (!fs::exists(p) || !fs::is_directory(p)) {
    log_missing_path(p, guidance, resolved_context);
    return false;
  }
  boost::system::error_code ec;
  fs::current_path(p, ec);
  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", ec.message().c_str());
    return false;
  }
  return true;
}

inline fs::path resolve_assets_root(const fs::path & start_path)
{
  for (fs::path current = start_path; !current.empty(); current = current.parent_path()) {
    const fs::path direct_assets = current / "assets";
    if (fs::exists(direct_assets) && fs::is_directory(direct_assets)) {
      return direct_assets;
    }
    const fs::path src_assets = current / "src" / "assets";
    if (fs::exists(src_assets) && fs::is_directory(src_assets)) {
      return src_assets;
    }
    if (current == current.root_path()) {
      break;
    }
  }
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    const fs::path package_assets = fs::path(share) / "assets";
    if (fs::exists(package_assets) && fs::is_directory(package_assets)) {
      return package_assets;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  return {};
}

inline void ensure_parent(const fs::path & p)
{
  boost::system::error_code ec;
  fs::create_directories(p.parent_path(), ec);
  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", ec.message().c_str());
  }
}

inline bool safe_copy_file(
  const fs::path & source,
  const fs::path & destination,
  const fs::path & context_path = {},
  const std::string & guidance =
  "Ensure the source file exists. If templates are missing, re-run install to populate "
  "share/workcell_builder/templates.",
  const fs::path & scenes_root_override = {})
{
  const fs::path resolved_context = context_path.empty() ? source : context_path;
  if (!fs::exists(source) || !fs::is_regular_file(source)) {
    log_missing_path(source, guidance, resolved_context, scenes_root_override);
    return false;
  }
  boost::system::error_code ec;
  fs::copy_file(source, destination, fs::copy_option::overwrite_if_exists, ec);
  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", ec.message().c_str());
    return false;
  }
  return true;
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
  fs::path expected_template_path;
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    fs::path base_template_path = fs::path(share) / "templates" /
      ("ros" + std::to_string(ros_ver));
    fs::path distro_template_path = base_template_path / ros_distro / "CMakeLists_example.txt";
    expected_template_path = ros_distro.empty() ?
      (base_template_path / "CMakeLists_example.txt") : distro_template_path;
    if (!ros_distro.empty() && fs::exists(distro_template_path)) {
      example_file = distro_template_path;
    } else {
      example_file = base_template_path / "CMakeLists_example.txt";
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  if (expected_template_path.empty()) {
    const fs::path templates_root = resolve_templates_root();
    if (!templates_root.empty()) {
      expected_template_path = ros_distro.empty() ?
        (templates_root / ("ros" + std::to_string(ros_ver)) / "CMakeLists_example.txt") :
        (templates_root / ("ros" + std::to_string(ros_ver)) / ros_distro /
        "CMakeLists_example.txt");
    }
  }
  fs::path target_location(package_filepath / "CMakeLists_example.txt");
  ensure_parent(target_location);
  bool copied = false;
  if (!example_file.empty()) {
    copied = safe_copy_file(
      example_file, target_location, package_filepath,
      "Re-run install to populate share/workcell_builder/templates for the selected ROS distro.",
      workcell_filepath / "scenes");
  } else {
    log_missing_path(
      expected_template_path.empty() ? target_location : expected_template_path,
      "Re-run install to populate share/workcell_builder/templates for the selected ROS distro.",
      package_filepath,
      workcell_filepath / "scenes");
  }
  safe_chdir(package_filepath, workcell_filepath);
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
  safe_chdir(scene_filepath, scene_filepath);
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
  fs::path expected_template_path;
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    fs::path base_template_path = fs::path(share) / "templates" /
      ("ros" + std::to_string(ros_ver));
    fs::path distro_template_path = base_template_path / ros_distro / "package_example.xml";
    expected_template_path = ros_distro.empty() ?
      (base_template_path / "package_example.xml") : distro_template_path;
    if (!ros_distro.empty() && fs::exists(distro_template_path)) {
      example_file = distro_template_path;
    } else {
      example_file = base_template_path / "package_example.xml";
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  if (expected_template_path.empty()) {
    const fs::path templates_root = resolve_templates_root();
    if (!templates_root.empty()) {
      expected_template_path = ros_distro.empty() ?
        (templates_root / ("ros" + std::to_string(ros_ver)) / "package_example.xml") :
        (templates_root / ("ros" + std::to_string(ros_ver)) / ros_distro /
        "package_example.xml");
    }
  }
  fs::path target_location(package_filepath / "package_example.xml");
  ensure_parent(target_location);
  bool copied = false;
  if (!example_file.empty()) {
    copied = safe_copy_file(
      example_file, target_location, package_filepath,
      "Re-run install to populate share/workcell_builder/templates for the selected ROS distro.",
      workcell_filepath / "scenes");
  } else {
    log_missing_path(
      expected_template_path.empty() ? target_location : expected_template_path,
      "Re-run install to populate share/workcell_builder/templates for the selected ROS distro.",
      package_filepath,
      workcell_filepath / "scenes");
  }
  safe_chdir(package_filepath, workcell_filepath);
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
        "  <maintainer email=\"example@gmail.com\">name</maintainer>\n"
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
  if (!fs::exists(source) || !fs::is_directory(source)) {
    log_missing_path(
      source,
      "Ensure the source directory exists in your workspace assets and re-run install if needed.",
      source);
    return false;
  }
  if (fs::exists(destination)) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"),
      "Destination directory %s already exists.", destination.string().c_str());
  }
  boost::system::error_code ec;
  fs::create_directory(destination, ec);
  if (ec) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", ec.message().c_str());
  }
  for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file) {
    fs::path current(file->path());
    if (fs::is_directory(current)) {
      if (!copyDir(current, destination / current.filename())) {
        return false;
      }
    } else {
      fs::path dst = destination / current.filename();
      ensure_parent(dst);
      safe_copy_file(
        current, dst, source,
        "Ensure the source files exist. If assets or templates are missing, re-run install.",
        destination.parent_path());
    }
  }
  return true;
}


#endif  // FILE_FUNCTIONS_H_
