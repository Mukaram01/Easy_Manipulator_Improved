#include "rviz_preview_runner.hpp"

#include <QRegularExpression>

#include <vector>

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

QString preview_group_supervisor_path(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root)
{
  const std::vector<boost::filesystem::path> candidates{
    scene_info.scene_dir.parent_path().parent_path() / "scripts" / "workcell_preview_process_group.py",
    workspace_root / "src" / "easy_manipulation_deployment" / "scripts" / "workcell_preview_process_group.py"};
  for (const auto & candidate : candidates) {
    boost::system::error_code ec;
    if (boost::filesystem::is_regular_file(candidate, ec) && !ec) {
      return QString::fromStdString(candidate.string());
    }
  }
  return QString::fromStdString(candidates.front().string());
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
  const QString workspace = QString::fromStdString(workspace_root.string());
  const QString setup = QString::fromStdString((workspace_root / "install" / "setup.bash").string());

  // Workcell Studio is normally launched from a shell that already sourced this
  // workspace.  Do not inherit that workspace as its own overlay during a
  // generated-scene rebuild.  Start from ROS Humble, intentionally add the
  // existing install as the dependency underlay, and override only the selected
  // generated package.  This avoids rebuilding unrelated dependency packages
  // and removes the noisy/unsafe self-overlay warning for every dependency.
  // Discover packages only in the workspace's src/ (the colcon layout). A workspace root can also hold other
  // workspaces or runtime mounts (e.g. a deployment view of the same source); scanning them finds the selected
  // scene twice and colcon refuses the build ("Duplicate package names").
  boost::system::error_code ec;
  const QString base_paths = boost::filesystem::is_directory(workspace_root / "src", ec) ? QStringLiteral("--base-paths src ") : QString();
  return QString(
    "env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH "
    "bash --noprofile --norc -c \"source /opt/ros/humble/setup.bash && "
    "source '%1' && cd '%2' && "
    "colcon build %4--symlink-install --packages-select '%3' --allow-overriding '%3'\"")
    .arg(setup, workspace, package, base_paths);
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
  // The QProcess tracks a small supervisor rather than ros2 launch directly.
  // The supervisor creates an isolated POSIX session/process group for the
  // launch and owns every descendant. QProcess::terminate() therefore asks the
  // supervisor to shut down only that owned group (SIGINT -> TERM -> KILL) and
  // the supervisor does not exit until the group is gone. This prevents RViz,
  // MoveIt or ros2_control descendants from being orphaned while Studio falsely
  // reports PREVIEW_STOPPED.
  const QString supervisor = preview_group_supervisor_path(scene_info, workspace_root);

  // All generated scenes currently use the global controller-manager namespace.
  // Starting a second preview therefore creates ambiguous ROS services and can
  // make one spawner load a controller on one manager and configure it on
  // another. Fail closed instead of launching a conflicting runtime. The
  // bracketed pgrep pattern intentionally does not match the pgrep process itself.
  return QString(
    "source /opt/ros/humble/setup.bash && source '%1' && "
    "if pgrep -f '[c]ontroller_manager/ros2_control_node' >/dev/null 2>&1; "
    "then echo 'Workcell Studio BLOCKER: another ros2_control preview is already running. Stop the existing simulation before launching a new one.' >&2; exit 73; fi && "
    "if [ ! -f '%2' ]; then echo 'Workcell Studio BLOCKER: preview process-group supervisor is missing: %2' >&2; exit 74; fi && "
    "export RCUTILS_COLORIZED_OUTPUT=0 && exec python3 '%2' -- %3")
    .arg(QString::fromStdString((workspace_root / "install" / "setup.bash").string()),
      supervisor,
      build_command(scene_info));
}

QString build_shell_command(const QString & scene_pkg, const boost::filesystem::path & workspace_root)
{
  const QString workspace_setup = QString::fromStdString((workspace_root / "install" / "setup.bash").string());
  return QString("bash -lc 'source /opt/ros/humble/setup.bash && source %1 && %2'")
    .arg(workspace_setup, build_command(scene_pkg));
}

QString build_full_cycle_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir,
  int domain_id, bool launch_rviz)
{
  if (scene_info.scene_name.empty() || domain_id < 0 || domain_id > 232) return {};
  const auto quote = [](QString value) { return "'" + value.replace("'", "'\\''") + "'"; };
  const auto supervisor = boost::filesystem::path(
    preview_group_supervisor_path(scene_info, workspace_root).toStdString()).parent_path() /
    "run_r14_plan_only_acceptance.py";
  // This supervisor owns the ONE scene launch and invokes the installed R1.5
  // executor. Never launch the separate RViz preview alongside this mode.
  // Installed Humble MoveIt crashes (-11) in its own teardown; the workspace's
  // registered teardown overlay (if any) is sourced, and the supervisor verifies
  // and reports what MoveGroup actually loaded and how every process exited.
  return QString("eval \"$(python3 %1 moveit-env)\" && source %2 && "
    "export ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0 && "
    "exec python3 %3 --resolve-if-needed --stream-status --timeout 900 --domain-id %4 --output-dir %5%6 --scene-dir %7")
    .arg(quote(QString::fromStdString((supervisor.parent_path() / "qualified_physics_runtime.py").string())),
      quote(QString::fromStdString((workspace_root / "install/local_setup.bash").string())),
      quote(QString::fromStdString(supervisor.string())))
    .arg(domain_id).arg(quote(output_dir), launch_rviz ? " --launch-rviz" : "", quote(QString::fromStdString(scene_info.scene_dir.string())));
}

QString build_sorting_session_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir, int domain_id,
  const QString & control_file)
{
  if (scene_info.scene_name.empty() || scene_info.scene_dir.empty() || output_dir.isEmpty() || control_file.isEmpty() ||
      domain_id < 0 || domain_id > 232) return {};
  const auto quote = [](QString value) { return "'" + value.replace("'", "'\\''") + "'"; };
  const auto supervisor = boost::filesystem::path(
    preview_group_supervisor_path(scene_info, workspace_root).toStdString()).parent_path() /
    "run_r14_plan_only_acceptance.py";
  // The same owned supervisor as Plan Saved Task: one fake-hardware launch, the installed executor once per object.
  return QString("eval \"$(python3 %1 moveit-env)\" && source %2 && "
    "export ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0 && "
    "exec python3 %3 --sorting-session --stream-status --timeout 7200 --domain-id %4 --output-dir %5 --control-file %6 --scene-dir %7")
    .arg(quote(QString::fromStdString((supervisor.parent_path() / "qualified_physics_runtime.py").string())),
      quote(QString::fromStdString((workspace_root / "install/local_setup.bash").string())),
      quote(QString::fromStdString(supervisor.string())))
    .arg(domain_id).arg(quote(output_dir), quote(control_file), quote(QString::fromStdString(scene_info.scene_dir.string())));
}

QString build_physics_plan_session_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir, int domain_id,
  const QString & control_file)
{
  // The runner's own ranges: 1..232, never the render domains 170..219.
  if (scene_info.scene_name.empty() || scene_info.scene_dir.empty() || output_dir.isEmpty() || control_file.isEmpty() ||
      domain_id < 1 || domain_id > 232 || (domain_id >= 170 && domain_id <= 219) ||
      control_file.startsWith(output_dir + "/")) return {};
  const auto quote = [](QString value) { return "'" + value.replace("'", "'\\''") + "'"; };
  const auto scripts = boost::filesystem::path(
    preview_group_supervisor_path(scene_info, workspace_root).toStdString()).parent_path();
  // The qualified runtime's environment, as for the physics cycle. No trial authority: no --authorise-g4-trial and no
  // --prior-evidence, so the session record is G4_PLAN_ONLY and every cycle authorisation is refused.
  return QString("eval \"$(python3 %1 env)\" && export ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0 && "
    "exec python3 %2 --scene-package %3 --output %4 --domain %5 --plan-every-target --stream-status --control-file %6")
    .arg(quote(QString::fromStdString((scripts / "qualified_physics_runtime.py").string())),
      quote(QString::fromStdString((scripts / "run_g4_sorting.py").string())),
      quote(QString::fromStdString(scene_info.scene_dir.string())), quote(output_dir))
    .arg(domain_id).arg(quote(control_file));
}

QString build_physics_trial_session_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir, int domain_id, int max_cycles,
  const QStringList & qualification_runs, const QString & admission_file)
{
  if (scene_info.scene_name.empty() || scene_info.scene_dir.empty() || output_dir.isEmpty() || admission_file.isEmpty() ||
      domain_id < 1 || domain_id > 232 || (domain_id >= 170 && domain_id <= 219) || max_cycles < 1 || max_cycles > 2 ||
      admission_file.startsWith(output_dir + "/") || qualification_runs.size() % 2) return {};
  int sessions = 0, stops = 0, cancels = 0;
  for (int i = 0; i < qualification_runs.size(); i += 2) {
    const auto & flag = qualification_runs.at(i);
    if (qualification_runs.at(i + 1).isEmpty()) return {};
    if (flag == "--session") ++sessions;
    else if (flag == "--stop") ++stops;
    else if (flag == "--cancel") ++cancels;
    else if (flag != "--other") return {};
  }
  if (sessions != 3 || stops != 1 || cancels != 1) return {};       // the G4 contract's five counted sessions
  const auto quote = [](QString value) { return "'" + value.replace("'", "'\\''") + "'"; };
  const auto scripts = boost::filesystem::path(
    preview_group_supervisor_path(scene_info, workspace_root).toStdString()).parent_path();
  const auto script = [&](const char * name) { return quote(QString::fromStdString((scripts / name).string())); };
  QStringList runs;
  for (const auto & item : qualification_runs) runs << (item.startsWith("--") ? item : quote(item));
  const QString scene = quote(QString::fromStdString(scene_info.scene_dir.string()));
  return QString("eval \"$(python3 %1 env)\" && test -n \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\" && "
    "export ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0 WORKCELL_PLANNING_TRACE_DIR=%2 && "
    "python3 %3 readiness --cell %4 %5 > %6 && "
    "exec python3 %7 --scene-package %4 --output %8 --domain %9 --stream-status --authorise-g4-trial "
    "--prior-evidence \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\" --max-cycles %10")
    .arg(script("qualified_physics_runtime.py"), quote(output_dir + ".trace"), script("g4_qualification.py"), scene,
         runs.join(' '), quote(admission_file), script("run_g4_sorting.py"), quote(output_dir))
    .arg(domain_id).arg(max_cycles);
}

QString build_physics_cycle_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir)
{
  if (scene_info.scene_name.empty() || scene_info.scene_dir.empty() || output_dir.isEmpty()) return {};
  const auto quote = [](QString value) { return "'" + value.replace("'", "'\\''") + "'"; };
  const auto scripts = boost::filesystem::path(
    preview_group_supervisor_path(scene_info, workspace_root).toStdString()).parent_path();
  // The workspace's registered qualified runtime is re-validated and its environment printed by
  // qualified_physics_runtime.py (it fails the command, never guesses). The qualified runner owns the
  // simulator, MoveIt and the executor; the saved cell is used unchanged and must select the qualified
  // physics simulation in its task. Real hardware is never part of this command.
  return QString("eval \"$(python3 %1 env)\" && export ROS_LOCALHOST_ONLY=1 RCUTILS_COLORIZED_OUTPUT=0 && "
    "exec python3 %2 --mode targeted --gate full-cycle --scene-package %3 --prior-evidence \"$WORKCELL_STAGE_A_PRIOR_EVIDENCE\" "
    "--search-timeout 900 --stream-status --output %4")
    .arg(quote(QString::fromStdString((scripts / "qualified_physics_runtime.py").string())),
      quote(QString::fromStdString((scripts / "run_stage_a1_finish.py").string())),
      quote(QString::fromStdString(scene_info.scene_dir.string())), quote(output_dir));
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
