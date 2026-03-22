#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

#include "workcell_directory_inspection.h"

namespace fs = boost::filesystem;

namespace
{
class ScopedTestDirectory
{
public:
  ScopedTestDirectory()
  {
    path_ = fs::temp_directory_path() / fs::unique_path("workcell-directory-inspection-%%%%-%%%%-%%%%");
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

TEST(WorkcellDirectoryInspection, ExistingWorkspaceRootUsesSelectedPath)
{
  ScopedTestDirectory temp_dir;
  const fs::path workspace_root = temp_dir.path() / "workspace";
  ASSERT_TRUE(fs::create_directories(workspace_root / "scenes"));
  ASSERT_TRUE(fs::create_directories(workspace_root / "assets"));

  const auto inspection = workcell_builder::inspect_selected_workcell_path(workspace_root);

  ASSERT_TRUE(inspection.success);
  EXPECT_TRUE(inspection.use_existing_root);
  EXPECT_EQ(inspection.workcell_root, fs::weakly_canonical(workspace_root));
  EXPECT_EQ(inspection.root_status_suffix, " (using selected path as workcell root)");
}

TEST(WorkcellDirectoryInspection, SrcOnlyWorkspaceCreatesInsideSrc)
{
  ScopedTestDirectory temp_dir;
  const fs::path selected_root = temp_dir.path() / "workspace";
  ASSERT_TRUE(fs::create_directories(selected_root / "src"));

  const auto inspection = workcell_builder::inspect_selected_workcell_path(selected_root);

  ASSERT_TRUE(inspection.success);
  EXPECT_FALSE(inspection.use_existing_root);
  EXPECT_EQ(inspection.workcell_root, fs::weakly_canonical(selected_root / "src"));
  EXPECT_EQ(inspection.root_status_suffix, " (created selected path/src as workcell root)");
}

TEST(WorkcellDirectoryInspection, SymlinkedWorkspaceRootResolvesCanonicalPath)
{
  ScopedTestDirectory temp_dir;
  const fs::path workspace_root = temp_dir.path() / "actual_workspace";
  const fs::path symlink_root = temp_dir.path() / "workspace_link";
  ASSERT_TRUE(fs::create_directories(workspace_root / "scenes"));
  ASSERT_TRUE(fs::create_directories(workspace_root / "assets"));
  boost::system::error_code ec;
  fs::create_directory_symlink(workspace_root, symlink_root, ec);
  ASSERT_FALSE(ec) << ec.message();

  const auto inspection = workcell_builder::inspect_selected_workcell_path(symlink_root);

  ASSERT_TRUE(inspection.success);
  EXPECT_TRUE(inspection.use_existing_root);
  EXPECT_EQ(inspection.canonical_selected_path, fs::weakly_canonical(workspace_root));
  EXPECT_EQ(inspection.workcell_root, fs::weakly_canonical(workspace_root));
}

TEST(WorkcellDirectoryInspection, NonexistentPathReportsSelectedPath)
{
  ScopedTestDirectory temp_dir;
  const fs::path missing_path = temp_dir.path() / "missing_workspace";

  const auto inspection = workcell_builder::inspect_selected_workcell_path(missing_path);

  EXPECT_FALSE(inspection.success);
  EXPECT_NE(inspection.error.find(missing_path.string()), std::string::npos);
  EXPECT_NE(inspection.error.find("does not exist"), std::string::npos);
}
