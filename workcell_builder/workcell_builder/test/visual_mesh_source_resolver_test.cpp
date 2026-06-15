// Copyright 2026 Mukaram01
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <QFileInfo>
#include <QString>
#include <QStringList>

#include <utility>
#include <vector>

#include "visual_mesh_source_resolver.hpp"

namespace fs = boost::filesystem;

#ifndef WORKCELL_BUILDER_REPO_ROOT
#define WORKCELL_BUILDER_REPO_ROOT ""
#endif

TEST(VisualMeshSourceResolver, ResolvesKnownPackageUrisToRepoAssets)
{
  const fs::path repo_root(WORKCELL_BUILDER_REPO_ROOT);
  ASSERT_FALSE(repo_root.empty());
  ASSERT_TRUE(fs::exists(repo_root / "assets"));

  const fs::path scene_dir = repo_root / "scenes" / "ur5_2f_test";
  const QString workspace_root = QString::fromStdString(repo_root.string());

  const std::vector<std::pair<QString, fs::path>> cases = {
    {QStringLiteral("package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"),
      repo_root / "assets/end_effectors/robotiq_85_gripper/robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"},
    {QStringLiteral("package://workbench_description/meshes/visual/table.stl"),
      repo_root / "assets/environment/workbench_description/meshes/visual/table.stl"},
    {QStringLiteral("package://realsense2_description/meshes/d415.stl"),
      repo_root / "assets/environment/realsense2_description/meshes/d415.stl"},
  };

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.first.toStdString());
    QStringList tried_candidates;
    const QString resolved = workcell_builder::resolve_visual_mesh_source_path(
      QString(), test_case.first, scene_dir, workspace_root, &tried_candidates);

    ASSERT_FALSE(resolved.isEmpty());
    const QFileInfo resolved_info(resolved);
    ASSERT_TRUE(resolved_info.exists());
    ASSERT_TRUE(resolved_info.isFile());
    EXPECT_EQ(
      QFileInfo(QString::fromStdString(test_case.second.string())).canonicalFilePath().toStdString(),
      resolved_info.canonicalFilePath().toStdString());
    EXPECT_TRUE(resolved_info.canonicalFilePath().contains(QStringLiteral("/assets/")));
  }
}

TEST(VisualMeshSourceResolver, LeavesNonexistentPackageUriUnresolved)
{
  const fs::path repo_root(WORKCELL_BUILDER_REPO_ROOT);
  const fs::path scene_dir = repo_root / "scenes" / "ur5_2f_test";
  const QString workspace_root = QString::fromStdString(repo_root.string());
  QStringList tried_candidates;

  const QString resolved = workcell_builder::resolve_visual_mesh_source_path(
    QString(),
    QStringLiteral("package://missing_description/meshes/visual/missing.stl"),
    scene_dir,
    workspace_root,
    &tried_candidates);

  EXPECT_TRUE(resolved.isEmpty());
}
