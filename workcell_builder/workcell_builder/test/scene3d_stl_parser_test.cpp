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

TEST(Scene3DStlParserTest, FailsOnInvalidAndOversized)
{
  Scene3DViewportWidget::InternalTriangleMesh mesh;
  QString err;
  EXPECT_FALSE(Scene3DViewportWidget::parse_stl_bytes_for_test("solid broken", "broken.stl", mesh, err));

  const QByteArray ascii =
    "solid t\n"
    "facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    "facet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
    "endsolid\n";
  err.clear();
  EXPECT_FALSE(Scene3DViewportWidget::parse_stl_bytes_for_test(ascii, "many.stl", mesh, err, 1));
  EXPECT_TRUE(err.contains("exceeds limit"));
}
