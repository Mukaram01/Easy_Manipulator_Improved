#include <gtest/gtest.h>

#include <QApplication>

#include "../gui/scene3d_viewport_widget.h"

namespace {
QApplication * ensure_app()
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  if (QApplication::instance()) return qobject_cast<QApplication *>(QApplication::instance());
  static int argc = 0;
  static char * argv[] = { nullptr };
  static QApplication app(argc, argv);
  return &app;
}

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

TEST(Scene3DRenderRoleClassifier, ValidUrdfPrimitivesAreSolidNotWireframeFallback)
{
  auto primitive = make_item("urdf_cylinder");
  primitive.locked = true;
  primitive.editable = false;
  primitive.lock_reason = "Generated URDF visual";
  primitive.source_layer = "generated_urdf_visual";
  primitive.active_visual_source = "urdf_primitive";
  primitive.mesh_available = false;
  primitive.has_mesh_metadata = false;
  primitive.mesh_path.clear();
  primitive.primitive_geometry_type = "cylinder";
  primitive.primitive_radius = 0.05;
  primitive.primitive_length = 0.20;

  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(primitive), "generated_urdf_primitive");
  EXPECT_TRUE(Scene3DViewportWidget::should_draw_as_solid_for_test(primitive, ScenePreviewWidget::MeshPreviewMode::Auto));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(primitive, ScenePreviewWidget::MeshPreviewMode::Auto));
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

TEST(Scene3DRenderRoleClassifier, IngestCountsOnlyPhysicalMeshHandoffs)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget::PreviewItem mesh_metadata_source = make_item("metadata_mesh_source");
  mesh_metadata_source.mesh_available = false;
  mesh_metadata_source.mesh_path.clear();
  mesh_metadata_source.has_mesh_metadata = true;
  mesh_metadata_source.source_path = "package://workcell_builder/meshes/metadata_only.stl";

  ScenePreviewWidget::PreviewItem mesh_package_uri = make_item("metadata_package_uri");
  mesh_package_uri.mesh_available = false;
  mesh_package_uri.mesh_path.clear();
  mesh_package_uri.has_mesh_metadata = true;
  mesh_package_uri.package_uri = "package://workcell_builder/meshes/metadata_only.dae";
  mesh_package_uri.source_path = "config/workcell_studio_layout.yaml";

  ScenePreviewWidget::PreviewItem explicit_mesh_path = make_item("explicit_mesh_path");
  explicit_mesh_path.mesh_available = false;
  explicit_mesh_path.mesh_path = "meshes/manual_handoff.obj";
  explicit_mesh_path.has_mesh_metadata = false;
  explicit_mesh_path.source_path = "config/environment.yaml";

  ScenePreviewWidget::PreviewItem layout_source = make_item("layout_source");
  layout_source.mesh_available = false;
  layout_source.mesh_path.clear();
  layout_source.has_mesh_metadata = true;
  layout_source.source_path = "config/workcell_studio_layout.yaml";

  Scene3DViewportWidget viewport;
  viewport.ingest_preview_items(QVector<ScenePreviewWidget::PreviewItem>{
    mesh_metadata_source, mesh_package_uri, explicit_mesh_path, layout_source
  });

  const auto counters = viewport.render_debug_counters();
  EXPECT_EQ(counters.mesh_source_count, 3);
  EXPECT_EQ(counters.mesh_backed_count, 3);
  EXPECT_EQ(counters.mesh_rendered_count, 0);
  EXPECT_FALSE(counters.last_paint_completed);
}

TEST(Scene3DRenderRoleClassifier, SemanticRolesUseCleanPrimitiveWhenDimensionsExist)
{
  auto pick_zone = make_item("pick_zone");
  pick_zone.role = "pick_zone";
  pick_zone.mesh_available = false;
  pick_zone.mesh_path.clear();
  pick_zone.has_mesh_metadata = false;

  EXPECT_TRUE(Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(pick_zone));
  EXPECT_TRUE(Scene3DViewportWidget::should_draw_as_solid_for_test(pick_zone, ScenePreviewWidget::MeshPreviewMode::Meshes));
  EXPECT_FALSE(Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(pick_zone));
}

TEST(Scene3DRenderRoleClassifier, SemanticRolesWithoutDimensionsSuppressMissingGeometryMarker)
{
  auto object = make_item("object_without_size");
  object.role = "object";
  object.mesh_available = false;
  object.mesh_path.clear();
  object.has_mesh_metadata = false;
  object.sx = 0.0;
  object.sy = 0.0;
  object.sz = 0.0;

  EXPECT_FALSE(Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(object));
  EXPECT_TRUE(Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(object));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(object, ScenePreviewWidget::MeshPreviewMode::Auto));
}
