#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <boost/system/error_code.hpp>

#include "scene_select_paths.h"

namespace fs = boost::filesystem;

namespace
{
class ScopedTestDirectory
{
public:
  ScopedTestDirectory()
  {
    path_ = fs::temp_directory_path() / fs::unique_path("scene-select-paths-%%%%-%%%%-%%%%");
    fs::create_directories(path_);
  }

  ~ScopedTestDirectory()
  {
    boost::system::error_code ec;
    fs::remove_all(path_, ec);
  }

  const fs::path & path() const
  {
    return path_;
  }

private:
  fs::path path_;
};
}  // namespace

TEST(SceneSelectPaths, UsesLoadedWorkcellPathInsteadOfCurrentDirectory)
{
  ScopedTestDirectory temp_dir;
  const fs::path cwd_root = temp_dir.path() / "cwd_root";
  const fs::path workcell_root = temp_dir.path() / "loaded_workcell";
  ASSERT_TRUE(fs::create_directories(cwd_root / "scenes"));
  ASSERT_TRUE(fs::create_directories(workcell_root / "scenes"));
  ASSERT_TRUE(fs::create_directories(workcell_root / "assets"));

  const fs::path original_cwd = fs::current_path();
  fs::current_path(cwd_root);

  Workcell workcell;
  workcell.workcell_filepath = workcell_root.string();

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell);

  fs::current_path(original_cwd);

  ASSERT_TRUE(resolution.success) << resolution.error;
  EXPECT_EQ(resolution.paths.workcell_path, workcell_root);
  EXPECT_EQ(resolution.paths.scenes_path, workcell_root / "scenes");
  EXPECT_NE(resolution.paths.scenes_path, cwd_root / "scenes");
}

TEST(SceneSelectPaths, EmptyWorkcellPathIsRejected)
{
  Workcell workcell;

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell);

  EXPECT_FALSE(resolution.success);
  EXPECT_NE(resolution.error.find("workcell_filepath"), std::string::npos);
}

TEST(SceneSelectPaths, NonDirectoryWorkcellPathIsRejected)
{
  ScopedTestDirectory temp_dir;
  const fs::path not_a_directory = temp_dir.path() / "workcell.txt";
  std::ofstream(not_a_directory.string()) << "placeholder";

  Workcell workcell;
  workcell.workcell_filepath = not_a_directory.string();

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell);

  EXPECT_FALSE(resolution.success);
  EXPECT_NE(resolution.error.find("not a directory"), std::string::npos);
}
