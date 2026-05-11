#include "workcell_scene_schema.hpp"
#include <cmath>
#include <fstream>

namespace workcell_builder
{
namespace
{
bool contains_token(const std::string & text, const std::string & token) { return text.find(token) != std::string::npos; }
}

bool write_workcell_scene_v1(const std::string & scene_yaml_path, const std::string & content)
{
  std::ofstream out(scene_yaml_path);
  if (!out.good()) { return false; }
  out << content;
  return true;
}

std::string parse_workcell_scene_v1(const std::string & scene_yaml_path)
{
  std::ifstream in(scene_yaml_path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

SceneSchemaValidationResult validate_workcell_scene_v1(const std::string & scene_yaml_path, bool strict)
{
  SceneSchemaValidationResult result;
  const std::string text = parse_workcell_scene_v1(scene_yaml_path);
  if (!contains_token(text, "schema_version: workcell_scene/v1")) { result.blockers.emplace_back("schema_version must be workcell_scene/v1"); }
  for (const std::string section : {"scene:", "robot:", "tool:", "compatibility:", "placed_objects:", "camera:", "task:", "safety:", "metadata:"}) {
    if (!contains_token(text, section)) { result.blockers.emplace_back("missing section " + section); }
  }
  if (contains_token(text, "workspace:")) {
    if (!contains_token(text, "bounds:")) { (strict ? result.blockers : result.warnings).emplace_back("workspace missing bounds"); }
    if (contains_token(text, "zones:")) {
      if (!contains_token(text, "shape: circle") && !contains_token(text, "shape: rectangle")) { (strict ? result.blockers : result.warnings).emplace_back("workspace zone shape unknown"); }
      if (!contains_token(text, "type: exclusion") && !contains_token(text, "type: warning")) { (strict ? result.blockers : result.warnings).emplace_back("workspace zone type unknown"); }
    }
  }
  if (contains_token(text, "UNKNOWN_COMPATIBILITY")) { result.warnings.emplace_back("unknown compatibility warnings only"); }
  if (contains_token(text, "INCOMPATIBLE")) { result.blockers.emplace_back("known incompatible pair"); }
  if (!contains_token(text, "fake_hardware_first: true")) { result.blockers.emplace_back("fake_hardware_first must remain true"); }
  if (!contains_token(text, "runtime_execution_enabled: false")) { result.blockers.emplace_back("runtime_execution_enabled must remain false"); }
  if (!contains_token(text, "real_hardware_enabled: false")) { result.blockers.emplace_back("real_hardware_enabled must remain false"); }
  if (strict && contains_token(text, "external_stl_warning")) { result.blockers.emplace_back("external mesh warning promoted by strict mode"); }
  if (contains_token(text, "camera:") && contains_token(text, "enabled: true")) {
    if (!contains_token(text, "camera_id:")) { result.blockers.emplace_back("camera_id must be non-empty"); }
    if (!contains_token(text, "frame_id:")) { result.blockers.emplace_back("frame_id must be non-empty"); }
    if (!contains_token(text, "pointcloud_topic:")) { result.warnings.emplace_back("missing pointcloud topic"); }
    if (!contains_token(text, "rgb_topic:") || !contains_token(text, "depth_topic:")) { (strict ? result.blockers : result.warnings).emplace_back("missing rgb/depth topics"); }
  }
  result.status = !result.blockers.empty() ? "FAIL" : (result.warnings.empty() ? "PASS" : "WARN");
  return result;
}

std::string scene_schema_status_label(const SceneSchemaValidationResult & result) { return "Schema " + result.status; }
std::vector<std::string> collect_scene_schema_warnings(const SceneSchemaValidationResult & result) { return result.warnings; }
std::vector<std::string> collect_scene_schema_blockers(const SceneSchemaValidationResult & result) { return result.blockers; }
}  // namespace workcell_builder
