#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__WORKCELL_STUDIO_SCENE_BROWSER_HPP_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__WORKCELL_STUDIO_SCENE_BROWSER_HPP_

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder {

struct WorkcellStudioSceneInfo
{
  std::string scene_name;
  boost::filesystem::path scene_dir;
  boost::filesystem::path canonical_scene_dir;
  bool has_environment_yaml{false};
  bool has_package_xml{false};
  bool has_launch_demo{false};
  bool has_scene_urdf_xacro{false};
  bool has_arm_hand_srdf_xacro{false};
  bool has_task_recipe{false};
  bool has_task_intent{false};
  bool has_smoke_report_json{false};
  bool has_smoke_report_html{false};
  bool has_static_preview_svg{false};
  bool has_static_preview_html{false};
  bool has_scene_manifest_yaml{false};
  std::string launch_package;
  std::string launch_file;
  bool launch_metadata_present{false};
  bool launch_metadata_file_exists{false};
  std::string launch_metadata_warning;
  std::string status{"BLOCKED"};
  std::string robot_summary{"unknown"};
  std::string gripper_summary{"unknown"};
  std::size_t object_count{0U};
  std::string parse_warning;
};

struct WorkcellStudioSceneBrowserResult
{
  boost::filesystem::path scene_root;
  bool root_exists{false};
  std::vector<boost::filesystem::path> searched_roots;
  std::vector<WorkcellStudioSceneInfo> scenes;
};

WorkcellStudioSceneBrowserResult discover_workcell_studio_scenes(const boost::filesystem::path & workspace_root);

// Scene identity is the resolved physical source directory, not the spelling by
// which a workspace happened to discover it (for example src/scenes symlinks).
boost::filesystem::path canonical_scene_identity(const boost::filesystem::path & scene_dir);
bool same_scene_identity(const WorkcellStudioSceneInfo & scene, const boost::filesystem::path & scene_dir);
int find_scene_by_identity(
  const WorkcellStudioSceneBrowserResult & scenes,
  const boost::filesystem::path & scene_dir,
  const std::string & stable_scene_name = "");

}  // namespace workcell_builder

#endif
