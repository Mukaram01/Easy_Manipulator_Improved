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

ScriptCommandPlan build_task_intent_command_plan(
  const QString & script_path,
  const QString & scene_dir);

ScriptCommandPlan build_generate_workcell_command_plan(
  const QString & script_path,
  const QString & scene_dir,
  const QString & scene_name);

}  // namespace workcell_builder
