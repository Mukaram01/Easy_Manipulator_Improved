#include <gtest/gtest.h>

#include "../gui/scene3d_viewport_widget.h"

namespace {
ScenePreviewWidget::PreviewItem make_item(const QString & id)
{
  ScenePreviewWidget::PreviewItem it;
  it.id = id;
  it.sx = 0.3;
  it.sy = 0.3;
  it.sz = 0.3;
  return it;
}
}

TEST(Scene3DRenderRoleClassifier, ClassifiesGeneratedAndHelperAndFallback)
{
  auto generated = make_item("urdf_link");
  generated.locked = true;
  generated.editable = false;
  generated.lock_reason = "Generated URDF visual";
  generated.source_layer = "generated_urdf_visual";
  generated.mesh_available = true;
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(generated), "generated_urdf_mesh");

  auto helper = make_item("reach");
  helper.role = "helper_overlay";
  helper.source_layer = "overlay";
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(helper), "helper_overlay");

  auto missing = make_item("missing");
  missing.mesh_available = false;
  missing.mesh_path.clear();
  missing.has_mesh_metadata = false;
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(missing), "missing_mesh_fallback");
}

TEST(Scene3DRenderRoleClassifier, FitExcludesHelpersIncludesGenerated)
{
  auto generated = make_item("generated");
  generated.locked = true;
  generated.editable = false;
  generated.lock_reason = "URDF visual";
  generated.source_layer = "generated_urdf_visual";
  EXPECT_TRUE(Scene3DViewportWidget::should_include_in_default_fit_for_test(generated));

  auto helper = make_item("helper");
  helper.role = "overlay_helper";
  helper.source_layer = "overlay";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(helper));
}

TEST(Scene3DRenderRoleClassifier, MeshesModeDoesNotDrawFallbackSolid)
{
  auto missing = make_item("missing2");
  missing.mesh_available = false;
  missing.mesh_path.clear();
  missing.has_mesh_metadata = false;
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_solid_for_test(missing, ScenePreviewWidget::MeshPreviewMode::Meshes));
  EXPECT_TRUE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(missing, ScenePreviewWidget::MeshPreviewMode::Meshes));
}

TEST(Scene3DRenderRoleClassifier, AcceptsCanonicalAndLegacyGeneratedUrdfTokens)
{
  auto canonical = make_item("canonical");
  canonical.source_layer = "generated_urdf_visual";
  canonical.mesh_available = true;
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(canonical), "generated_urdf_mesh");

  auto legacy = make_item("legacy");
  legacy.active_visual_source = "locked_generated_urdf_visual";
  legacy.mesh_available = true;
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(legacy), "generated_urdf_mesh");
}
