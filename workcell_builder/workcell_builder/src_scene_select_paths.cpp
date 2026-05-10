#include "scene_select_paths.h"

#include <boost/system/error_code.hpp>
#include <vector>
#include <cstdlib>

#include "default_asset_paths.h"

namespace fs = boost::filesystem;

namespace workcell_builder
{
namespace
{
bool has_dir(const fs::path & p)
{
  boost::system::error_code ec;
  return !p.empty() && fs::exists(p, ec) && !ec && fs::is_directory(p, ec) && !ec;
}

bool is_repo_root(const fs::path & root)
{
  return has_dir(root / "scenes") && has_dir(root / "assets");
}

fs::path find_repo_from_cwd(fs::path cwd)
{
  while (!cwd.empty()) {
    if (is_repo_root(cwd)) {
      return cwd;
    }
    const fs::path parent = cwd.parent_path();
    if (parent == cwd) {
      break;
    }
    cwd = parent;
  }
  return fs::path();
}
}

SceneSelectPathResolution resolve_scene_select_paths(const Workcell & workcell)
{
  return resolve_scene_select_paths(workcell, fs::path());
}

SceneSelectPathResolution resolve_scene_select_paths(
  const Workcell & workcell,
  const fs::path & preferred_root)
{
  SceneSelectPathResolution resolution;
  resolution.paths.templates_path = get_default_templates_directory();

  std::vector<fs::path> candidates;
  if (has_dir(preferred_root)) candidates.push_back(preferred_root);
  if (!workcell.workcell_filepath.empty()) {
    const fs::path loaded(workcell.workcell_filepath);
    if (has_dir(loaded)) candidates.push_back(loaded);
  }
  const fs::path cwd = fs::current_path();
  if (has_dir(cwd / "scenes")) candidates.push_back(cwd);
  const fs::path parent_found = find_repo_from_cwd(cwd);
  if (!parent_found.empty()) candidates.push_back(parent_found);
  const char * repo_env = std::getenv("WORKCELL_STUDIO_REPO_ROOT");
  if (repo_env != nullptr) candidates.emplace_back(repo_env);
  const char * home = std::getenv("HOME");
  if (home != nullptr) {
    candidates.push_back(fs::path(home) / "workcell_ws" / "src");
    candidates.push_back(fs::path(home) / "workcell_ws" / "src" / "easy_manipulation_deployment");
  }

  fs::path workcell_path;
  for (const auto & c : candidates) {
    if (has_dir(c / "scenes")) {
      workcell_path = c;
      break;
    }
  }
  if (workcell_path.empty()) {
    resolution.error = "No scene packages found. Expected scenes under a repo root with scenes/ and assets/.";
    return resolution;
  }

  resolution.success = true;
  resolution.paths.workcell_path = workcell_path;
  resolution.paths.scenes_path = workcell_path / "scenes";
  resolution.paths.assets_path = get_runtime_assets_directory(workcell_path);
  return resolution;
}

}  // namespace workcell_builder
