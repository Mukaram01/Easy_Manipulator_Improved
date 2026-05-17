#pragma once

#include <QString>
#include <QStringList>

namespace workcell_builder
{

struct ScriptCommandPlan
{
  QString script_name;
  QString script_path;
  QString scene_dir;
  QString scene_name;
  QStringList missing_fields;
  QStringList arguments;

  bool ready() const {return missing_fields.isEmpty();}
  QString missing_fields_message() const;
  QString display_command() const;
};

struct TaskIntentCommandInput
{
  QString scene_package;
  QString task_id;
  QString task_type;
  QString task_template;
  QString pick_source;
  QString place_target;
  QString grasp_strategy;
};

ScriptCommandPlan build_task_intent_command_plan(
  const QString & script_path,
  const TaskIntentCommandInput & input);

ScriptCommandPlan build_generate_workcell_command_plan(
  const QString & script_path,
  const QString & scene_dir,
  const QString & output_dir,
  const QString & scene_name);

ScriptCommandPlan build_validate_generated_scene_command_plan(
  const QString & script_path,
  const QString & scene_dir);

}  // namespace workcell_builder
