#include "include/workcell_builder_command_builders.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace workcell_builder
{
namespace fs = std::filesystem;

namespace
{
QString shell_quote(const QString & value)
{
  QString quoted = value;
  quoted.replace("'", "'\\''");
  return QString("'%1'").arg(quoted);
}

void append_missing_if_empty(const QString & value, const QString & field_name, QStringList * missing)
{
  if (value.trimmed().isEmpty()) {
    missing->push_back(field_name);
  }
}

QString first_enabled_zone_id_for_type(const fs::path & environment_yaml, const std::string & zone_type)
{
  try {
    const YAML::Node root = YAML::LoadFile(environment_yaml.string());
    const YAML::Node zones = root["task_zones"];
    if (!zones || !zones.IsSequence()) {
      return QString();
    }
    for (const auto & zone : zones) {
      if (!zone.IsMap()) {
        continue;
      }
      const bool enabled = !zone["enabled"] || zone["enabled"].as<bool>(true);
      const std::string type = zone["type"] ? zone["type"].as<std::string>() : std::string();
      const std::string id = zone["id"] ? zone["id"].as<std::string>() : std::string();
      if (enabled && type == zone_type && !id.empty()) {
        return QString::fromStdString(id);
      }
    }
  } catch (...) {
  }
  return QString();
}

}  // namespace

QString ScriptCommandPlan::missing_fields_message() const
{
  if (missing_fields.isEmpty()) {
    return QString();
  }
  return QString("Missing required fields: %1").arg(missing_fields.join(", "));
}

QString ScriptCommandPlan::display_command() const
{
  if (script_path.trimmed().isEmpty()) {
    return QString();
  }
  QStringList tokens;
  tokens << "python3" << shell_quote(script_path);
  for (const auto & arg : arguments) {
    tokens << shell_quote(arg);
  }
  return tokens.join(" ");
}

ScriptCommandPlan build_task_intent_command_plan(
  const QString & script_path,
  const TaskIntentCommandInput & input)
{
  ScriptCommandPlan plan;
  plan.script_name = "create_or_update_builder_task_intent.py";
  plan.script_path = script_path;
  plan.scene_name = input.scene_package;
  append_missing_if_empty(script_path, "helper script path", &plan.missing_fields);
  append_missing_if_empty(input.scene_package, "scene package", &plan.missing_fields);
  append_missing_if_empty(input.task_id, "task id", &plan.missing_fields);
  append_missing_if_empty(input.task_type, "task type", &plan.missing_fields);
  append_missing_if_empty(input.pick_source, "pick source", &plan.missing_fields);
  append_missing_if_empty(input.place_target, "place target", &plan.missing_fields);
  append_missing_if_empty(input.grasp_strategy, "grasp strategy", &plan.missing_fields);
  if (plan.ready()) {
    const QString task_template = input.task_template.trimmed().isEmpty() ? "pick_place" : input.task_template;
    plan.arguments << "--scene-package" << input.scene_package
                   << "--task-id" << input.task_id
                   << "--task-type" << input.task_type
                   << "--task-template" << task_template
                   << "--pick-source" << input.pick_source
                   << "--place-target" << input.place_target
                   << "--grasp-strategy" << input.grasp_strategy
                   << "--auto-resolve-zones";
  }
  return plan;
}

TaskIntentCommandInput resolve_task_intent_command_input_defaults(const TaskIntentCommandInput & input)
{
  TaskIntentCommandInput resolved = input;
  if (input.scene_package.trimmed().isEmpty()) {
    return resolved;
  }
  const fs::path scene_dir = input.scene_package.toStdString();
  const fs::path environment_yaml = scene_dir / "environment.yaml";

  QString pick_from_zone;
  QString place_from_zone;
  if (fs::exists(environment_yaml)) {
    pick_from_zone = first_enabled_zone_id_for_type(environment_yaml, "pick");
    place_from_zone = first_enabled_zone_id_for_type(environment_yaml, "place");
  }

  if (resolved.pick_source.trimmed().isEmpty()) {
    resolved.pick_source = pick_from_zone.isEmpty() ? "pick_zone_01" : pick_from_zone;
  }
  if (resolved.place_target.trimmed().isEmpty()) {
    resolved.place_target = place_from_zone.isEmpty() ? "place_zone_01" : place_from_zone;
  }
  return resolved;
}

ScriptCommandPlan build_generate_workcell_command_plan(
  const QString & script_path,
  const QString & scene_dir,
  const QString & output_dir,
  const QString & scene_name)
{
  ScriptCommandPlan plan;
  plan.script_name = "generate_workcell_from_cell_definition.py";
  plan.script_path = script_path;
  plan.scene_dir = scene_dir;
  plan.scene_name = scene_name;
  append_missing_if_empty(script_path, "helper script path", &plan.missing_fields);
  append_missing_if_empty(scene_dir, "scene directory", &plan.missing_fields);
  append_missing_if_empty(output_dir, "output directory", &plan.missing_fields);
  append_missing_if_empty(scene_name, "scene name", &plan.missing_fields);
  if (plan.ready()) {
    plan.arguments << (scene_dir + "/cell_definition.yaml")
                   << "--output-dir" << output_dir
                   << "--package-name" << scene_name
                   << "--existing-package-dir" << scene_dir;
  }
  return plan;
}

ScriptCommandPlan build_validate_generated_scene_command_plan(
  const QString & script_path,
  const QString & scene_dir)
{
  ScriptCommandPlan plan;
  plan.script_name = "validate_builder_generated_scene.py";
  plan.script_path = script_path;
  plan.scene_dir = scene_dir;
  append_missing_if_empty(script_path, "helper script path", &plan.missing_fields);
  append_missing_if_empty(scene_dir, "scene directory", &plan.missing_fields);
  if (plan.ready()) {
    plan.arguments << scene_dir << "--json";
  }
  return plan;
}

}  // namespace workcell_builder
