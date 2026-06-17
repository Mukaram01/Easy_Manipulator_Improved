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
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(missing), "hidden_missing_mesh");
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


TEST(Scene3DRenderRoleClassifier, DefaultProductFitIncludesPhysicalItemsAndExcludesDiagnostics)
{
  auto robot_mesh = make_item("robot_tool_mesh");
  robot_mesh.locked = true;
  robot_mesh.editable = false;
  robot_mesh.lock_reason = "Generated URDF visual";
  robot_mesh.source_layer = "generated_urdf_visual";
  robot_mesh.mesh_available = true;
  EXPECT_TRUE(Scene3DViewportWidget::should_include_in_default_fit_for_test(robot_mesh));

  auto editable_layout = make_item("editable_table");
  editable_layout.source_layer = "editable_layout";
  editable_layout.linked_to_editable_layout_state = true;
  editable_layout.editable = true;
  EXPECT_TRUE(Scene3DViewportWidget::should_include_in_default_fit_for_test(editable_layout));

  auto camera_fov = make_item("camera_fov");
  camera_fov.role = "camera_fov_cone";
  camera_fov.category = "diagnostic_overlay";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(camera_fov));

  auto reachability = make_item("reachability");
  reachability.role = "reachability_heatmap";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(reachability));

  auto route_helper = make_item("task_route");
  route_helper.role = "task_route_helper";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(route_helper));

  auto warning_anchor = make_item("warning_anchor");
  warning_anchor.role = "warning_anchor";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(warning_anchor));

  auto bounds_box = make_item("bounds_box");
  bounds_box.category = "diagnostic_bounds_box";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(bounds_box));
  auto generic_primitive = make_item("generic_box");
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(generic_primitive));

  auto mesh_preview = make_item("mesh_preview_asset");
  mesh_preview.source_layer = "mesh_preview";
  mesh_preview.mesh_available = true;
  EXPECT_TRUE(Scene3DViewportWidget::should_include_in_default_fit_for_test(mesh_preview));

}


TEST(Scene3DRenderRoleClassifier, GeneratedUrdfMeshPreviewMaterialIsOpaqueInProductView)
{
  auto generated = make_item("generated_transparent_robot_mesh");
  generated.locked = true;
  generated.editable = false;
  generated.lock_reason = "Generated URDF visual";
  generated.source_layer = "generated_urdf_visual";
  generated.active_visual_source = "generated_urdf_visual";
  generated.mesh_available = true;
  generated.has_mesh_metadata = true;
  generated.has_material_color = true;
  generated.material_r = 0.4;
  generated.material_g = 0.5;
  generated.material_b = 0.6;
  generated.material_a = 0.18;

  const QColor product_color = Scene3DViewportWidget::material_color_for_test(generated, false);
  EXPECT_GE(product_color.alphaF(), 0.92);

  const QColor diagnostic_color = Scene3DViewportWidget::material_color_for_test(generated, true);
  EXPECT_LT(diagnostic_color.alphaF(), 0.25);
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
  mesh_package_uri.has_mesh_metadata = false;
  mesh_package_uri.package_uri = "package://workcell_builder/meshes/metadata_only.dae";
  mesh_package_uri.source_path = "config/workcell_studio_layout.yaml";

  ScenePreviewWidget::PreviewItem explicit_mesh_path = make_item("explicit_mesh_path");
  explicit_mesh_path.mesh_available = false;
  explicit_mesh_path.mesh_path = "meshes/manual_handoff.obj";
  explicit_mesh_path.has_mesh_metadata = false;
  explicit_mesh_path.source_path = "config/environment.yaml";

  ScenePreviewWidget::PreviewItem source_mesh_path = make_item("source_mesh_path");
  source_mesh_path.mesh_available = false;
  source_mesh_path.mesh_path.clear();
  source_mesh_path.has_mesh_metadata = false;
  source_mesh_path.source_path = "assets/meshes/source_handoff.glb?version=1";

  ScenePreviewWidget::PreviewItem layout_source = make_item("layout_source");
  layout_source.role = "pick_zone";
  layout_source.mesh_available = false;
  layout_source.mesh_path.clear();
  layout_source.has_mesh_metadata = false;
  layout_source.source_path = "config/workcell_studio_layout.yaml";

  ScenePreviewWidget::PreviewItem authoring_package_uri = make_item("authoring_package_uri");
  authoring_package_uri.role = "object_a";
  authoring_package_uri.mesh_available = false;
  authoring_package_uri.mesh_path.clear();
  authoring_package_uri.has_mesh_metadata = false;
  authoring_package_uri.package_uri = "package://workcell_builder/config/environment.yaml";
  authoring_package_uri.source_path = "config/environment.yaml";

  Scene3DViewportWidget viewport;
  viewport.ingest_preview_items(QVector<ScenePreviewWidget::PreviewItem>{
    mesh_metadata_source, mesh_package_uri, explicit_mesh_path, source_mesh_path, layout_source, authoring_package_uri
  });

  const auto counters = viewport.render_debug_counters();
  EXPECT_EQ(counters.mesh_source_count, 4);
  EXPECT_EQ(counters.mesh_backed_count, 4);
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

TEST(Scene3DRenderRoleClassifier, EditableLayoutPrimitivesDoNotCountAsGeneratedFallbacks)
{
  ASSERT_NE(ensure_app(), nullptr);

  auto editable_table = make_item("editable_table_primitive");
  editable_table.role = "table";
  editable_table.source_layer = "editable_layout";
  editable_table.active_visual_source = "";
  editable_table.linked_to_editable_layout_state = true;
  editable_table.editable = true;
  editable_table.locked = false;
  editable_table.mesh_available = false;
  editable_table.has_mesh_metadata = false;
  editable_table.mesh_path.clear();

  Scene3DViewportWidget viewport;
  viewport.ingest_preview_items(QVector<ScenePreviewWidget::PreviewItem>{ editable_table });
  ASSERT_TRUE(viewport.render_smoke_fallback_frame(nullptr));

  const auto counters = viewport.render_debug_counters();
  EXPECT_EQ(counters.primitive_fallback_count, 0);
  EXPECT_EQ(counters.primitive_fallback_rendered_count, 0);
  EXPECT_EQ(counters.valid_physical_fallback_count, 0);
  EXPECT_EQ(counters.editable_primitive_rendered_count, 1);
}

TEST(Scene3DRenderRoleClassifier, ValidStlAndDaePreviewItemsAreMeshSurfaceCandidates)
{
  auto stl = make_item("valid_stl_preview");
  stl.locked = true;
  stl.editable = false;
  stl.lock_reason = "Generated URDF visual";
  stl.source_layer = "generated_urdf_visual";
  stl.active_visual_source = "generated_urdf_visual";
  stl.has_mesh_metadata = true;
  stl.mesh_available = true;
  stl.mesh_path = "meshes/valid_part.stl";

  auto dae = make_item("valid_dae_preview");
  dae.locked = true;
  dae.editable = false;
  dae.lock_reason = "Generated URDF visual";
  dae.source_layer = "generated_urdf_visual";
  dae.active_visual_source = "generated_urdf_visual";
  dae.has_mesh_metadata = true;
  dae.mesh_available = true;
  dae.mesh_path = "package://demo_cell/meshes/valid_part.dae";

  EXPECT_TRUE(Scene3DViewportWidget::has_mesh_surface_candidate_for_test(stl));
  EXPECT_TRUE(Scene3DViewportWidget::has_mesh_surface_candidate_for_test(dae));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(stl, ScenePreviewWidget::MeshPreviewMode::Auto));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(dae, ScenePreviewWidget::MeshPreviewMode::Auto));
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(stl), "generated_urdf_mesh");
  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(dae), "generated_urdf_mesh");
}
