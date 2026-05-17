#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string load_file(const std::string & path)
{
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}

TEST(Scene3DMeshPreviewRegression, KeepsTransformStackAndFallback)
{
  const std::string src = load_file("gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto t = src.find("glTranslated(it.x, it.y, it.z)");
  const auto r = src.find("glRotated(qRadiansToDegrees(it.roll)");
  const auto mr = src.find("glRotated(qRadiansToDegrees(it.mesh_r)");
  const auto s = src.find("glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z)");
  ASSERT_NE(t, std::string::npos);
  ASSERT_NE(r, std::string::npos);
  ASSERT_NE(mr, std::string::npos);
  ASSERT_NE(s, std::string::npos);
  EXPECT_LT(t, r);
  EXPECT_LT(r, mr);
  EXPECT_LT(mr, s);

  EXPECT_NE(src.find("if (preview_path && it.has_origin_offset)"), std::string::npos);
  EXPECT_NE(src.find("if (!draw_mesh_preview_if_available(*it, item_color(*it), true))"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsSelectionAndOverlayRendering)
{
  const std::string src = load_file("gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("if (it->id == selected_id)"), std::string::npos);
  EXPECT_NE(src.find("draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz"), std::string::npos);
  EXPECT_NE(src.find("if (show_warning_labels && !it.warnings.isEmpty())"), std::string::npos);
}
