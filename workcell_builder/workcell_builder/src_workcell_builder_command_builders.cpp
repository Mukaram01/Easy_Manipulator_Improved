#include "include/workcell_builder_command_builders.hpp"

namespace workcell_builder
{

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
                   << "--grasp-strategy" << input.grasp_strategy;
  }
  return plan;
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
                   << "--force";
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
