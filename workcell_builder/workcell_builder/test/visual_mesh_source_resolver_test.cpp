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
#include <QMap>
#include <fstream>

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
    {QStringLiteral("package://table_description/meshes/visual/table.stl"),
      repo_root / "assets/environment/table_description/meshes/visual/table.stl"},
    {QStringLiteral("package://realsense2_description/meshes/d415.stl"),
      repo_root / "assets/environment/realsense2_description/meshes/d415.stl"},
    {QStringLiteral("package://realsense2_description/meshes/d435.dae"),
      repo_root / "assets/environment/realsense2_description/meshes/d435.dae"},
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

TEST(VisualMeshSourceResolver, PrefersWorkspaceSrcAssetsPackagesOverRepoAssets)
{
  const fs::path repo_root(WORKCELL_BUILDER_REPO_ROOT);
  ASSERT_FALSE(repo_root.empty());

  const fs::path tmp = fs::temp_directory_path() / fs::unique_path("workcell_assets_test_%%%%-%%%%-%%%%");
  const fs::path ws = tmp / "workcell_ws";
  const fs::path local_pkg = ws / "src/assets/robots/universal_robot/ur_description";
  const fs::path local_mesh = local_pkg / "meshes/ur5/visual/base.dae";
  fs::create_directories(local_mesh.parent_path());
  {
    std::ofstream pkg((local_pkg / "package.xml").string());
    pkg << "<package format=\"3\"><name>ur_description</name><version>0.0.0</version><description>test</description><maintainer email=\"a@b.c\">t</maintainer><license>Apache-2.0</license></package>";
  }
  {
    std::ofstream mesh(local_mesh.string());
    mesh << "local asset mesh";
  }

  QStringList detected_roots;
  const QMap<QString, QString> package_map = workcell_builder::discover_visual_mesh_package_map(
    repo_root / "scenes" / "ur5_2f_test", QString::fromStdString(ws.string()), &detected_roots);
  ASSERT_TRUE(package_map.contains(QStringLiteral("ur_description")));
  EXPECT_EQ(
    QFileInfo(QString::fromStdString(local_pkg.string())).canonicalFilePath(),
    QFileInfo(package_map.value(QStringLiteral("ur_description"))).canonicalFilePath());

  QStringList tried_candidates;
  const QString resolved = workcell_builder::resolve_visual_mesh_source_path(
    QString(),
    QStringLiteral("package://ur_description/meshes/ur5/visual/base.dae"),
    repo_root / "scenes" / "ur5_2f_test",
    QString::fromStdString(ws.string()),
    &tried_candidates);
  EXPECT_EQ(
    QFileInfo(QString::fromStdString(local_mesh.string())).canonicalFilePath(),
    QFileInfo(resolved).canonicalFilePath());

  fs::remove_all(tmp);
}

TEST(VisualMeshSourceResolver, DiscoversOnlyProductAssetRootsAndMeshCategories)
{
  const fs::path repo_root(WORKCELL_BUILDER_REPO_ROOT);
  ASSERT_FALSE(repo_root.empty());

  QStringList detected_roots;
  const QMap<QString, QString> package_map = workcell_builder::discover_visual_mesh_package_map(
    repo_root / "scenes" / "ur5_2f_test", QString::fromStdString(repo_root.string()), &detected_roots);

  ASSERT_FALSE(detected_roots.isEmpty());
  for (const QString & root : detected_roots) {
    EXPECT_TRUE(root.contains(QStringLiteral("/assets"))) << root.toStdString();
    EXPECT_FALSE(root.contains(QStringLiteral("/opt/ros"))) << root.toStdString();
    EXPECT_FALSE(root.contains(QStringLiteral("/install/share"))) << root.toStdString();
  }
  EXPECT_TRUE(package_map.contains(QStringLiteral("robotiq_85_description")));
  EXPECT_TRUE(package_map.contains(QStringLiteral("workbench_description")));
  EXPECT_TRUE(package_map.contains(QStringLiteral("realsense2_description")));
  EXPECT_FALSE(package_map.contains(QStringLiteral("ament_cmake")));
  EXPECT_FALSE(package_map.contains(QStringLiteral("rviz2")));
  EXPECT_FALSE(package_map.contains(QStringLiteral("rosbag2")));

  int mesh_count = 0;
  const QMap<QString, int> categories = workcell_builder::discover_visual_mesh_asset_category_counts(
    detected_roots, &mesh_count, nullptr);
  EXPECT_GT(mesh_count, 0);
  EXPECT_GT(categories.value(QStringLiteral("gripper")), 0);
  EXPECT_GT(categories.value(QStringLiteral("table/workbench")), 0);
  EXPECT_GT(categories.value(QStringLiteral("camera")), 0);
}
