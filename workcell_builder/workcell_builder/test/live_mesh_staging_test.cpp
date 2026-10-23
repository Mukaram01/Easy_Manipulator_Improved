// A part added to the live Web3D session gets a mesh URI the viewer can load:
// served meshes are used in place, Asset Library meshes are staged into the
// exporter's cache, and anything else is refused with a reason.

#include <gtest/gtest.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <utility>
#include <vector>

#include "live_mesh_staging.hpp"

using workcell_builder::stage_live_authoring_mesh;

namespace
{

constexpr char kLibraryMesh[] = "workcell_builder/workcell_builder/assets/environment/camera_stand_description/meshes/camera_stand.stl";

class LiveMeshStagingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(repo_.isValid());
    root_ = QFileInfo(repo_.path()).canonicalFilePath();
    scene_dir_ = QDir(root_).filePath("scenes/ur5_2f_test");
    write(kLibraryMesh, "solid camera_stand\nendsolid camera_stand\n");
  }

  QString write(const QString & relative, const QByteArray & bytes)
  {
    const QString path = QDir(root_).filePath(relative);
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile file(path);
    EXPECT_TRUE(file.open(QIODevice::WriteOnly));
    file.write(bytes);
    return path;
  }

  QByteArray read(const QString & relative) const
  {
    QFile file(QDir(root_).filePath(relative));
    return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
  }

  QTemporaryDir repo_;
  QString root_;
  QString scene_dir_;
};

TEST_F(LiveMeshStagingTest, LibraryMeshIsStagedIntoTheExporterCache)
{
  const QString source = QDir(root_).filePath(kLibraryMesh);  // catalog entries hold canonical absolute paths
  const auto staged = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test", source);
  const QString expected = QStringLiteral("build/workcell_studio_web_scene/assets/ur5_2f_test/") + kLibraryMesh;
  ASSERT_TRUE(staged.ok()) << staged.reason.toStdString();
  EXPECT_EQ(staged.status, "staged");
  EXPECT_EQ(staged.uri, expected);
  EXPECT_EQ(read(expected), read(kLibraryMesh));
  // The source is untouched, and a repo-relative source stages to the same file.
  const auto again = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test", kLibraryMesh);
  EXPECT_EQ(again.uri, expected);
  EXPECT_EQ(again.status, "staged");
}

TEST_F(LiveMeshStagingTest, StaleStagedCopyIsReplaced)
{
  const QString expected = QStringLiteral("build/workcell_studio_web_scene/assets/ur5_2f_test/") + kLibraryMesh;
  write(expected, "stale");
  write(kLibraryMesh, "solid fixed\nendsolid fixed\n");
  const auto staged = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test", kLibraryMesh);
  ASSERT_TRUE(staged.ok()) << staged.reason.toStdString();
  EXPECT_EQ(read(expected), QByteArray("solid fixed\nendsolid fixed\n"));
}

TEST_F(LiveMeshStagingTest, ServedMeshesAreUsedInPlace)
{
  write("assets/environment/bins/sorting_bin.stl", "bin");
  write("scenes/ur5_2f_test/assets/imported/catalog/0123456789abcdef.stl", "imported");
  write("build/workcell_studio_web_scene/assets/ur5_2f_test/table.dae", "table");

  const auto repo_relative = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test", "assets/environment/bins/sorting_bin.stl");
  EXPECT_EQ(repo_relative.uri, "assets/environment/bins/sorting_bin.stl");
  EXPECT_EQ(repo_relative.status, "available");
  const auto absolute = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test",
    QDir(root_).filePath("build/workcell_studio_web_scene/assets/ur5_2f_test/table.dae"));
  EXPECT_EQ(absolute.uri, "build/workcell_studio_web_scene/assets/ur5_2f_test/table.dae");
  // A scene-relative saved mesh resolves inside the scene, like the exporter.
  const auto scene_relative = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test",
    "assets/imported/catalog/0123456789abcdef.stl");
  EXPECT_EQ(scene_relative.uri, "scenes/ur5_2f_test/assets/imported/catalog/0123456789abcdef.stl");
  EXPECT_FALSE(QFileInfo::exists(QDir(root_).filePath("build/workcell_studio_web_scene/assets/ur5_2f_test/assets")));
}

TEST_F(LiveMeshStagingTest, UnloadableMeshesAreRefusedWithAReason)
{
  QTemporaryDir outside;
  ASSERT_TRUE(outside.isValid());
  const QString external = QDir(outside.path()).filePath("part.stl");
  QFile file(external);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write("x");
  file.close();
  write("workcell_builder/workcell_builder/assets/environment/part.step", "step");

  const auto cases = std::vector<std::pair<QString, QString>>{
    {QString(), "no_mesh_uri"},
    {"workcell_builder/workcell_builder/assets/environment/part.step", "unsupported_format"},
    {"workcell_builder/workcell_builder/assets/environment/missing.stl", "missing_file"},
    {external, "unsafe_path"},
  };
  for (const auto & [source, status] : cases) {
    const auto refused = stage_live_authoring_mesh(root_, scene_dir_, "ur5_2f_test", source);
    EXPECT_FALSE(refused.ok()) << source.toStdString();
    EXPECT_TRUE(refused.uri.isEmpty());
    EXPECT_EQ(refused.status, status) << source.toStdString();
    EXPECT_FALSE(refused.reason.isEmpty());
  }
  for (const QString & scene : {QString(), QString(".."), QString("a/b")}) {
    const auto refused = stage_live_authoring_mesh(root_, scene_dir_, scene, kLibraryMesh);
    EXPECT_EQ(refused.status, "unsafe_destination") << scene.toStdString();
  }
  EXPECT_FALSE(QFileInfo::exists(QDir(root_).filePath("build")));
}

}  // namespace
