#include <gtest/gtest.h>
#include <QDir>
#include <QTemporaryDir>

#include "include/workspace_validation.hpp"

TEST(WorkspaceValidationTest, MissingFolderInvalid)
{
  EXPECT_FALSE(workcell_builder::is_valid_workcell_workspace("/tmp/this_path_should_not_exist_12345"));
}

TEST(WorkspaceValidationTest, MissingSrcInvalid)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  EXPECT_FALSE(workcell_builder::is_valid_workcell_workspace(dir.path()));
}

TEST(WorkspaceValidationTest, SrcOnlyInvalid)
{
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  ASSERT_TRUE(QDir(dir.path()).mkpath("src"));
  EXPECT_FALSE(workcell_builder::is_valid_workcell_workspace(dir.path()));
}

TEST(WorkspaceValidationTest, EmdFolderValid)
{
  QTemporaryDir dir;
  ASSERT_TRUE(QDir(dir.path()).mkpath("src/easy_manipulation_deployment"));
  EXPECT_TRUE(workcell_builder::is_valid_workcell_workspace(dir.path()));
}

TEST(WorkspaceValidationTest, ScenesFolderValid)
{
  QTemporaryDir dir;
  ASSERT_TRUE(QDir(dir.path()).mkpath("src/scenes"));
  EXPECT_TRUE(workcell_builder::is_valid_workcell_workspace(dir.path()));
}

TEST(WorkspaceValidationTest, AssetsFolderValid)
{
  QTemporaryDir dir;
  ASSERT_TRUE(QDir(dir.path()).mkpath("src/assets"));
  EXPECT_TRUE(workcell_builder::is_valid_workcell_workspace(dir.path()));
}
