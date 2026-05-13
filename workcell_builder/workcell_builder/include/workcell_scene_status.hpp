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

enum class SceneLifecycleState
{
  NO_SCENE_SELECTED,
  TEMPLATE_SELECTED,
  SCENE_SCAFFOLD,
  YAML_MISSING,
  YAML_INVALID_REPAIRABLE,
  YAML_READY,
  LAYOUT_READY,
  TASK_READY,
  VALIDATION_WARNINGS,
  VALIDATION_BLOCKED,
  GENERATED_PACKAGE_READY,
  BUILD_READY,
  LAUNCH_READY,
  LEGACY_SCENE_NEEDS_REPAIR
};

struct SceneLifecycleSnapshot
{
  SceneLifecycleState state{SceneLifecycleState::NO_SCENE_SELECTED};
  std::string label;
  std::string severity;
  std::string message;
  std::string next_action;
  bool allow_generation{false};
  bool allow_bundle_export{false};
  bool launch_validation_meaningful{false};
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
  SceneLifecycleSnapshot lifecycle;
};

SceneLifecycleSnapshot determine_scene_lifecycle(const SceneStatusReport & report);
bool lifecycle_allows_recommended_layout(SceneLifecycleState state);
bool lifecycle_allows_validate(SceneLifecycleState state);
bool lifecycle_allows_generate(SceneLifecycleState state);
bool lifecycle_allows_export(SceneLifecycleState state);


SceneStatusReport inspect_scene_status(
  const boost::filesystem::path & workcell_path,
  const boost::filesystem::path & scenes_path,
  const boost::filesystem::path & assets_path,
  const std::string & scene_name);

}  // namespace workcell_builder
