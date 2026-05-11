#pragma once
#include <map>
#include <string>
#include <vector>

namespace workcell_builder
{
struct SceneRoundtripState
{
  std::string scene_name;
  std::string package_name;
  std::string schema_version{"workcell_scene/v1"};
  std::string robot_name;
  std::string tool_name;
  std::string compatibility_status{"UNKNOWN_COMPATIBILITY"};
  std::vector<std::string> placed_objects;
  std::vector<std::string> mesh_paths;
  std::vector<std::string> camera_metadata_lines;
  std::vector<std::string> task_metadata_lines;
  std::vector<std::string> workspace_metadata_lines;
  bool fake_hardware_first{true};
  bool real_hardware_enabled{false};
  bool runtime_execution_enabled{false};
  std::vector<std::string> warnings;
};

SceneRoundtripState load_workcell_scene_v1_from_file(const std::string & scene_yaml_path);
void populate_builder_state_from_scene(const SceneRoundtripState & loaded, SceneRoundtripState * builder_state);
SceneRoundtripState extract_builder_state_to_scene(const SceneRoundtripState & builder_state);
std::vector<std::string> validate_roundtrip_scene_state(const SceneRoundtripState & state);
std::string roundtrip_status_label(const SceneRoundtripState & state);
}  // namespace workcell_builder
