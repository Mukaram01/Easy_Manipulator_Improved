#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder
{

struct SceneStatusItem
{
  std::string name;
  std::string status;
  std::string message;
  std::string path;
};

struct SceneStatusReport
{
  std::string scene_name;
  std::string scene_path;
  bool environment_yaml_ok = false;
  bool generated_files_ok = false;
  bool launch_file_ok = false;
  bool fake_hardware_default_ok = false;
  std::vector<SceneStatusItem> items;
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  std::vector<std::string> safety_notes;
  std::vector<std::string> next_commands;
};

SceneStatusReport inspect_scene_status(
  const boost::filesystem::path & workcell_path,
  const boost::filesystem::path & scenes_path,
  const boost::filesystem::path & assets_path,
  const std::string & scene_name);

}  // namespace workcell_builder
