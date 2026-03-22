#include "scene_select_paths.h"

#include <boost/system/error_code.hpp>

#include "default_asset_paths.h"

namespace fs = boost::filesystem;

namespace workcell_builder
{
SceneSelectPathResolution resolve_scene_select_paths(const Workcell & workcell)
{
  SceneSelectPathResolution resolution;
  resolution.paths.templates_path = get_default_templates_directory();

  if (workcell.workcell_filepath.empty()) {
    resolution.error = "Loaded workcell is missing workcell_filepath; unable to locate scenes directory.";
    return resolution;
  }

  const fs::path workcell_path(workcell.workcell_filepath);
  boost::system::error_code ec;
  if (!fs::exists(workcell_path, ec) || ec) {
    resolution.error = "Loaded workcell path does not exist: " + workcell_path.string();
    return resolution;
  }
  if (!fs::is_directory(workcell_path, ec) || ec) {
    resolution.error = "Loaded workcell path is not a directory: " + workcell_path.string();
    return resolution;
  }

  resolution.success = true;
  resolution.paths.workcell_path = workcell_path;
  resolution.paths.scenes_path = workcell_path / "scenes";
  resolution.paths.assets_path = get_runtime_assets_directory(workcell_path);
  return resolution;
}

}  // namespace workcell_builder
