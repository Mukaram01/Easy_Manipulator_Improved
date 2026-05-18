#include <gtest/gtest.h>

#include "../gui/scene3d_viewport_widget.h"

#include <cstring>

TEST(Scene3DStlParserTest, ParsesAsciiStl)
{
  const QByteArray ascii =
    "solid t\n"
    "facet normal 0 0 1\n"
    "outer loop\n"
    "vertex 0 0 0\n"
    "vertex 1 0 0\n"
    "vertex 0 1 0\n"
    "endloop\n"
    "endfacet\n"
    "endsolid\n";

  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  EXPECT_TRUE(Scene3DViewportWidget::parse_stl_bytes_for_test(ascii, "mesh.stl", mesh, err));
  ASSERT_EQ(mesh.triangles.size(), 1);
  EXPECT_FLOAT_EQ(mesh.triangles[0].vertices[1].x(), 1.0f);
}

TEST(Scene3DStlParserTest, ParsesBinaryStl)
{
  QByteArray bytes;
  bytes.resize(84 + 50);
  bytes.fill('\0');
  quint32 tri_count = 1;
  std::memcpy(bytes.data() + 80, &tri_count, sizeof(tri_count));
  auto putf = [&](int off, float v){ std::memcpy(bytes.data() + off, &v, sizeof(float)); };
  putf(84 + 0, 0.0f); putf(84 + 4, 0.0f); putf(84 + 8, 1.0f);
  putf(84 + 12, 0.0f); putf(84 + 16, 0.0f); putf(84 + 20, 0.0f);
  putf(84 + 24, 1.0f); putf(84 + 28, 0.0f); putf(84 + 32, 0.0f);
  putf(84 + 36, 0.0f); putf(84 + 40, 1.0f); putf(84 + 44, 0.0f);

  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  EXPECT_TRUE(Scene3DViewportWidget::parse_stl_bytes_for_test(bytes, "mesh.stl", mesh, err));
  ASSERT_EQ(mesh.triangles.size(), 1);
  EXPECT_FLOAT_EQ(mesh.triangles[0].normal.z(), 1.0f);
}

TEST(Scene3DStlParserTest, RejectsTriangleLimit)
{
  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  const QByteArray ascii =
    "solid t\n"
    "facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    "facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    "endsolid\n";
  EXPECT_FALSE(Scene3DViewportWidget::parse_stl_bytes_for_test(ascii, "many.stl", mesh, err, 1));
  EXPECT_TRUE(err.contains("exceeds limit"));
}

TEST(Scene3DStlParserTest, ReportsInvalidParseReason)
{
  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  EXPECT_FALSE(Scene3DViewportWidget::parse_stl_bytes_for_test("solid broken", "broken.stl", mesh, err));
  EXPECT_FALSE(err.isEmpty());
}

TEST(Scene3DStlParserTest, ComputesMeshBoundsFromKnownTriangleSet)
{
  const QByteArray ascii =
    "solid t\n"
    "facet normal 0 0 1\nouter loop\nvertex -1 -2 -3\nvertex 4 5 6\nvertex 0 1 2\nendloop\nendfacet\n"
    "facet normal 0 0 1\nouter loop\nvertex 2 1 0\nvertex -3 2 1\nvertex 0 -4 3\nendloop\nendfacet\n"
    "endsolid\n";
  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  ASSERT_TRUE(Scene3DViewportWidget::parse_stl_bytes_for_test(ascii, "bounds.stl", mesh, err));
  QVector3D min_bounds;
  QVector3D max_bounds;
  ASSERT_TRUE(Scene3DViewportWidget::compute_mesh_bounds_for_test(mesh, min_bounds, max_bounds));
  EXPECT_FLOAT_EQ(min_bounds.x(), -3.0f);
  EXPECT_FLOAT_EQ(min_bounds.y(), -4.0f);
  EXPECT_FLOAT_EQ(min_bounds.z(), -3.0f);
  EXPECT_FLOAT_EQ(max_bounds.x(), 4.0f);
  EXPECT_FLOAT_EQ(max_bounds.y(), 5.0f);
  EXPECT_FLOAT_EQ(max_bounds.z(), 6.0f);
}

TEST(Scene3DStlParserTest, MeshModeFallbackBehaviorAtMethodLevel)
{
  using Mode = ScenePreviewWidget::MeshPreviewMode;
  EXPECT_FALSE(Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(Mode::Primitives, true, true));
  EXPECT_TRUE(Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(Mode::Meshes, true, true));
  EXPECT_FALSE(Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(Mode::Meshes, true, false));
  EXPECT_TRUE(Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(Mode::Auto, true, true));
  EXPECT_FALSE(Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(Mode::Auto, false, false));
}
