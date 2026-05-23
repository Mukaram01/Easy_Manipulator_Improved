#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__RVIZ_PREVIEW_RUNNER_HPP_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__RVIZ_PREVIEW_RUNNER_HPP_

#include <QString>

#include <boost/filesystem.hpp>

#include <functional>
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
PreviewReadinessStatus validate_readiness(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root);
PreviewReadinessStatus run(
  const QString & command,
  const std::function<void(const QString &)> & stdout_callback,
  const std::function<void(const QString &)> & stderr_callback);
PreviewReadinessStatus dry_run(
  const WorkcellStudioSceneInfo & scene_info,
  const boost::filesystem::path & workspace_root,
  QString * command);

}  // namespace workcell_builder

#endif
