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
