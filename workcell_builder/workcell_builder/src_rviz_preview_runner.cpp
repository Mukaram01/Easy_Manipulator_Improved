#include "rviz_preview_runner.hpp"

#include <QProcess>
#include <QRegularExpression>

namespace workcell_builder
{

namespace
{

PreviewReadinessStatus blocked(const QString & reason)
{
  PreviewReadinessStatus result;
  result.ready = false;
  result.blocker_reason = reason;
  return result;
}

bool launch_command_is_safe(const QString & command, QString * reason)
{
  auto reject = [reason]() {
      if (reason) {
        *reason = "command rejected by fake-hardware safety guard";
      }
      return false;
    };

  if (!command.contains("use_fake_hardware:=true") || command.contains("use_fake_hardware:=false")) {
    return reject();
  }
  if (!command.contains("launch_rviz:=true")) {
    return reject();
  }
  const QStringList deny{
    "real_hardware:=true",
    "runtime_execution_enabled:=true",
    "execute:=true",
    "command_robot:=true",
    "send_motion:=true",
    "fake_hardware:=false",
    "ur_robot_driver",
    "ethercat",
    "canopen"};
  for (const auto & token : deny) {
    if (command.contains(token)) {
      return reject();
    }
  }

  const QString trimmed = command.trimmed();
  const QRegularExpression expected_launch_re(
    R"(^ros2\s+launch\s+\S+\s+demo\.launch\.py(?:\s+.*)?$)");
  if (!expected_launch_re.match(trimmed).hasMatch()) {
    return reject();
  }

  return true;
}

}  // namespace

QString build_command(const QString & scene_pkg)
{
  return QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(scene_pkg);
}

QString build_shell_command(const QString & scene_pkg, const boost::filesystem::path & workspace_root)
{
  const QString workspace_setup = QString::fromStdString((workspace_root / "install" / "setup.bash").string());
  return QString("bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'")
    .arg(workspace_setup, build_command(scene_pkg));
}

PreviewReadinessStatus validate_readiness(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root)
{
  if (scene_info.scene_name.empty()) return blocked("selected scene missing");
  if (!boost::filesystem::exists(workspace_root)) return blocked("Workspace root does not exist");
  if (!scene_info.has_package_xml) return blocked("package.xml missing");
  if (scene_info.status.find("BLOCKED") != std::string::npos) return blocked("BLOCKED acceptance scene");
  if (scene_info.status.find("PREVIEW_ONLY") != std::string::npos) return blocked("PREVIEW_ONLY scenes cannot run");
  if (!scene_info.has_launch_demo) return blocked("launch/demo.launch.py missing");
  if (!scene_info.has_task_intent) return blocked("Missing config/workcell_builder_task_intent.yaml");

  const boost::filesystem::path workspace_setup = workspace_root / "install" / "setup.bash";
  if (!boost::filesystem::exists(workspace_setup)) return blocked("install/setup.bash missing under workspace root");

  const boost::filesystem::path layout_file = scene_info.scene_dir / "layout" / "workcell_studio_layout.yaml";
  const boost::filesystem::path merge_report = scene_info.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  if (boost::filesystem::exists(layout_file) &&
    (!boost::filesystem::exists(merge_report) || boost::filesystem::last_write_time(layout_file) > boost::filesystem::last_write_time(merge_report))) {
    return blocked("Layout changed since last generation. Run Generate Scene / Layout Merge before preview.");
  }

  PreviewReadinessStatus ok;
  ok.ready = true;
  return ok;
}

PreviewReadinessStatus run(
  const QString & command,
  const std::function<void(const QString &)> & stdout_callback,
  const std::function<void(const QString &)> & stderr_callback)
{
  QString reason;
  if (!launch_command_is_safe(command, &reason)) {
    return blocked(reason);
  }

  QString shell_command = command;
  if (shell_command.startsWith("bash -lc ")) {
    shell_command = shell_command.mid(QString("bash -lc ").size());
    if (shell_command.size() >= 2 && shell_command.startsWith("'") && shell_command.endsWith("'")) {
      shell_command = shell_command.mid(1, shell_command.size() - 2);
    }
  }

  QProcess process;
  process.setProgram("/bin/bash");
  process.setArguments({"-lc", shell_command});
  process.start();
  if (!process.waitForStarted()) {
    return blocked("Failed to start preview process");
  }
  while (process.state() != QProcess::NotRunning) {
    process.waitForReadyRead(100);
    const QString out = QString::fromUtf8(process.readAllStandardOutput());
    if (!out.isEmpty() && stdout_callback) stdout_callback(out);
    const QString err = QString::fromUtf8(process.readAllStandardError());
    if (!err.isEmpty() && stderr_callback) stderr_callback(err);
  }
  const QString out = QString::fromUtf8(process.readAllStandardOutput());
  if (!out.isEmpty() && stdout_callback) stdout_callback(out);
  const QString err = QString::fromUtf8(process.readAllStandardError());
  if (!err.isEmpty() && stderr_callback) stderr_callback(err);

  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    return blocked("ros2 launch failed");
  }

  PreviewReadinessStatus ok;
  ok.ready = true;
  return ok;
}

PreviewReadinessStatus dry_run(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root,
  QString * command)
{
  const PreviewReadinessStatus readiness = validate_readiness(scene_info, workspace_root);
  if (!readiness.ready) return readiness;
  const QString generated = build_shell_command(QString::fromStdString(scene_info.scene_name), workspace_root);
  if (command) {
    *command = generated;
  }
  QString reason;
  if (!launch_command_is_safe(generated, &reason)) {
    return blocked(reason);
  }

  QProcess pkg_check;
  pkg_check.setProgram("/bin/bash");
  pkg_check.setArguments({
      "-lc",
      QString("source /opt/ros/humble/setup.bash && source '%1' && ros2 pkg prefix %2")
        .arg(QString::fromStdString((workspace_root / "install" / "setup.bash").string()))
        .arg(QString::fromStdString(scene_info.scene_name))});
  pkg_check.start();
  if (!pkg_check.waitForFinished(10000) ||
    pkg_check.exitStatus() != QProcess::NormalExit || pkg_check.exitCode() != 0)
  {
    return blocked("package not found by ros2");
  }

  PreviewReadinessStatus ok;
  ok.ready = true;
  return ok;
}

}  // namespace workcell_builder
