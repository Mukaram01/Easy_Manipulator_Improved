#pragma once
#include <string>
#include <vector>

namespace workcell_builder
{
struct SceneSchemaValidationResult
{
  std::string status{"PASS"};
  std::vector<std::string> warnings;
  std::vector<std::string> blockers;
};

bool write_workcell_scene_v1(const std::string & scene_yaml_path, const std::string & content);
SceneSchemaValidationResult validate_workcell_scene_v1(const std::string & scene_yaml_path, bool strict);
std::string parse_workcell_scene_v1(const std::string & scene_yaml_path);
std::string scene_schema_status_label(const SceneSchemaValidationResult & result);
std::vector<std::string> collect_scene_schema_warnings(const SceneSchemaValidationResult & result);
std::vector<std::string> collect_scene_schema_blockers(const SceneSchemaValidationResult & result);
}  // namespace workcell_builder
