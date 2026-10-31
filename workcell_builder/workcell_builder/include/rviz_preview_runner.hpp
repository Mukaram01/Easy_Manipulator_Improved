#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__RVIZ_PREVIEW_RUNNER_HPP_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__RVIZ_PREVIEW_RUNNER_HPP_

#include <QString>

#include <boost/filesystem.hpp>

#include <string>

#include "workcell_studio_scene_browser.hpp"

namespace workcell_builder
{

struct PreviewReadinessStatus
{
  bool ready{false};
  QString blocker_reason;
};

QString build_command(const QString & scene_pkg);
QString build_command(const WorkcellStudioSceneInfo & scene_info);
QString build_selected_package_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root);
QString package_prefix_check_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root);
QString build_launch_shell_command(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root);
// One bounded qualified-physics pick/place cycle of the saved cell (Stage-A runner, targeted full-cycle).
// runtime_env is the workstation's qualified physics runtime (overlays, physics root, prior evidence);
// empty when it is not configured, in which case no command is produced.
QString build_physics_cycle_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir);
QString build_full_cycle_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir,
  int domain_id, bool launch_rviz);
// R2.4: a continuous class-routed sorting session of the saved cell on MoveIt fake hardware (the same
// supervisor, --sorting-session); operator pause/resume/stop are written to control_file.
QString build_sorting_session_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir, int domain_id,
  const QString & control_file);
// G6: a G4 physics sorting session of the saved cell, PLAN ONLY (run_g4_sorting.py --plan-every-target, never
// --authorise-g4-trial): one Gazebo + MoveIt launch, simulator-rendered EPD frames, routing and a plan per object;
// the runner refuses every cycle authorisation, so nothing moves. Operator pause/resume/stop go to control_file,
// which must lie outside output_dir (the runner creates output_dir itself and refuses an existing one).
QString build_physics_plan_session_command(const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root, const QString & output_dir, int domain_id,
  const QString & control_file);
bool launch_command_is_safe(const QString & command, QString * reason = nullptr);
QString build_shell_command(const QString & scene_pkg, const boost::filesystem::path & workspace_root);
QString build_shell_command(const WorkcellStudioSceneInfo & scene_info, const boost::filesystem::path & workspace_root);
PreviewReadinessStatus validate_readiness(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root);
PreviewReadinessStatus dry_run(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root,
  QString * command);

}  // namespace workcell_builder

#endif
