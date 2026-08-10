#include "rviz_preview_runner.hpp"

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

bool command_is_safe(const QString & command, QString * reason)
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
    R"(\bros2\s+launch\s+\S+\s+\S+\.launch\.py(?:\s+.*)?$)");
  if (!expected_launch_re.match(trimmed).hasMatch()) {
    return reject();
  }

  return true;
}

}  // namespace

bool launch_command_is_safe(const QString & command, QString * reason)
{
  return command_is_safe(command, reason);
}

QString build_command(const QString & scene_pkg)
{
  return QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(scene_pkg);
}

QString build_command(const WorkcellStudioSceneInfo & scene_info)
{
  const QString launch_package = QString::fromStdString(
    scene_info.launch_package.empty() ? scene_info.scene_name : scene_info.launch_package);
  const QString launch_file = QString::fromStdString(
    scene_info.launch_file.empty() ? std::string("demo.launch.py") : scene_info.launch_file);
  return QString("ros2 launch %1 %2 use_fake_hardware:=true launch_rviz:=true")
    .arg(launch_package, launch_file);
}

QString build_selected_package_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root)
{
  const QString package = QString::fromStdString(
    scene_info.launch_package.empty() ? scene_info.scene_name : scene_info.launch_package);
  return QString("source /opt/ros/humble/setup.bash && cd '%1' && "
                 "colcon build --symlink-install --packages-select '%2'")
    .arg(QString::fromStdString(workspace_root.string()), package);
}

QString package_prefix_check_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root)
{
  const QString package = QString::fromStdString(
    scene_info.launch_package.empty() ? scene_info.scene_name : scene_info.launch_package);
  return QString("source /opt/ros/humble/setup.bash && source '%1' && ros2 pkg prefix '%2'")
    .arg(QString::fromStdString((workspace_root / "install" / "setup.bash").string()), package);
}

QString build_launch_shell_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root)
{
  return QString("source /opt/ros/humble/setup.bash && source '%1' && exec %2")
    .arg(QString::fromStdString((workspace_root / "install" / "setup.bash").string()),
      build_command(scene_info));
}

QString build_shell_command(const QString & scene_pkg, const boost::filesystem::path & workspace_root)
{
  const QString workspace_setup = QString::fromStdString((workspace_root / "install" / "setup.bash").string());
  return QString("bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'")
    .arg(workspace_setup, build_command(scene_pkg));
}

QString build_shell_command(const WorkcellStudioSceneInfo & scene_info, const boost::filesystem::path & workspace_root)
{
  const QString workspace_setup = QString::fromStdString((workspace_root / "install" / "setup.bash").string());
  return QString("bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'")
    .arg(workspace_setup, build_command(scene_info));
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
  if (scene_info.launch_metadata_present && !scene_info.launch_metadata_warning.empty()) {
    return blocked(QString::fromStdString(scene_info.launch_metadata_warning));
  }
  if (!scene_info.has_launch_demo && scene_info.launch_file.empty()) return blocked("launch/demo.launch.py missing");
  if (!scene_info.has_task_intent) return blocked("Missing config/workcell_builder_task_intent.yaml");

  if (scene_info.scene_dir.empty() || !boost::filesystem::exists(scene_info.scene_dir)) {
    return blocked("selected scene directory missing");
  }
  if (!boost::filesystem::exists(scene_info.scene_dir / "CMakeLists.txt")) {
    return blocked("CMakeLists.txt missing");
  }
  if (!scene_info.canonical_scene_dir.empty()) {
    boost::system::error_code ec;
    const auto selected = boost::filesystem::weakly_canonical(scene_info.scene_dir, ec);
    ec.clear();
    const auto canonical = boost::filesystem::weakly_canonical(scene_info.canonical_scene_dir, ec);
    if (!ec && selected != canonical) return blocked("selected scene is not the canonical scene package");
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
  const QString generated = build_shell_command(scene_info, workspace_root);
  if (command) {
    *command = generated;
  }
  QString reason;
  if (!command_is_safe(generated, &reason)) {
    return blocked(reason);
  }

  PreviewReadinessStatus ok;
  ok.ready = true;
  return ok;
}

}  // namespace workcell_builder
