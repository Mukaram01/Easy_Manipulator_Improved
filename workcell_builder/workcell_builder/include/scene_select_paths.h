#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__SCENE_SELECT_PATHS_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__SCENE_SELECT_PATHS_H_

#include <boost/filesystem.hpp>
#include <string>

#include "attributes/workcell.h"

namespace workcell_builder
{
struct SceneSelectPaths
{
  boost::filesystem::path workcell_path;
  boost::filesystem::path scenes_path;
  boost::filesystem::path assets_path;
  boost::filesystem::path templates_path;
};

struct SceneSelectPathResolution
{
  bool success = false;
  SceneSelectPaths paths;
  std::string error;
};

SceneSelectPathResolution resolve_scene_select_paths(const Workcell & workcell);

}  // namespace workcell_builder

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__SCENE_SELECT_PATHS_H_
