#include "workcell_scene_roundtrip.hpp"
#include <fstream>

namespace workcell_builder
{
namespace
{
bool contains(const std::string & t, const std::string & k) { return t.find(k) != std::string::npos; }
std::string line_for(const std::string & text, const std::string & key)
{
  const auto p = text.find(key);
  if (p == std::string::npos) { return ""; }
  const auto e = text.find('\n', p);
  return text.substr(p, e == std::string::npos ? std::string::npos : e - p);
}
}

SceneRoundtripState load_workcell_scene_v1_from_file(const std::string & scene_yaml_path)
{
  SceneRoundtripState s;
  std::ifstream in(scene_yaml_path);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (!contains(text, "schema_version: workcell_scene/v1")) { s.warnings.emplace_back("Legacy/partial scene warning"); }
  s.schema_version = contains(text, "workcell_scene/v1") ? "workcell_scene/v1" : "legacy";
  s.scene_name = line_for(text, "name:");
  s.package_name = line_for(text, "package:");
  s.robot_name = line_for(text, "robot:");
  s.tool_name = line_for(text, "tool:");
  s.compatibility_status = contains(text, "INCOMPATIBLE") ? "INCOMPATIBLE" : (contains(text, "COMPATIBLE") ? "COMPATIBLE" : "UNKNOWN_COMPATIBILITY");
  if (contains(text, "placed_objects:")) { s.placed_objects.emplace_back("placed_objects"); }
  if (contains(text, "meshes/generated_objects/")) { s.mesh_paths.emplace_back("meshes/generated_objects/"); }
  if (contains(text, "custom_meshes")) { s.mesh_paths.emplace_back("custom_meshes"); }
  if (contains(text, "camera:")) { s.camera_metadata_lines.emplace_back("camera enabled/disabled"); }
  if (contains(text, "task:")) { s.task_metadata_lines.emplace_back("task/grasp recipe metadata"); }
  if (contains(text, "workspace:")) { s.workspace_metadata_lines.emplace_back("workspace bounds/zones"); }
  s.fake_hardware_first = contains(text, "fake_hardware_first: true");
  s.real_hardware_enabled = contains(text, "real_hardware_enabled: true");
  s.runtime_execution_enabled = contains(text, "runtime_execution_enabled: true");
  return s;
}

void populate_builder_state_from_scene(const SceneRoundtripState & loaded, SceneRoundtripState * builder_state)
{
  if (!builder_state) { return; }
  *builder_state = loaded;  // Loaded from workcell_scene/v1
}

SceneRoundtripState extract_builder_state_to_scene(const SceneRoundtripState & builder_state)
{
  SceneRoundtripState out = builder_state;
  out.fake_hardware_first = true;
  out.real_hardware_enabled = false;
  out.runtime_execution_enabled = false;
  return out;
}

std::vector<std::string> validate_roundtrip_scene_state(const SceneRoundtripState & state)
{
  std::vector<std::string> blockers;
  if (state.schema_version != "workcell_scene/v1") { blockers.emplace_back("Legacy/partial scene warning"); }
  if (!state.fake_hardware_first) { blockers.emplace_back("fake_hardware_first must stay true"); }
  if (state.real_hardware_enabled) { blockers.emplace_back("real_hardware_enabled must stay false"); }
  if (state.runtime_execution_enabled) { blockers.emplace_back("runtime_execution_enabled must stay false"); }
  return blockers;
}

std::string roundtrip_status_label(const SceneRoundtripState & state)
{
  const auto blockers = validate_roundtrip_scene_state(state);
  return blockers.empty() ? "Scene Round-trip Status: Loaded from workcell_scene/v1" : "Scene Round-trip Status: Legacy/partial scene warning";
}
}  // namespace workcell_builder
