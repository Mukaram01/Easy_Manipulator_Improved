#include <gtest/gtest.h>

#include <QApplication>
#include <QFile>
#include <QIODevice>
#include <QDir>

#include "../gui/scene3d_viewport_widget.h"
#include "../include/object_placement_yaml_io.hpp"

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

  auto observation_zone = make_item("camera_observation_1");
  observation_zone.role = "camera_observation";
  observation_zone.category = "Task Zones";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(observation_zone));

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


TEST(Scene3DRenderRoleClassifier, RobotReachIdIsSemanticEvenWhenLockedWithoutMeshMetadata)
{
  ASSERT_NE(ensure_app(), nullptr);

  auto robot_reach = make_item("robot_reach");
  robot_reach.role = "reach";
  robot_reach.category = "reach";
  robot_reach.source_layer = "environment_yaml";
  robot_reach.active_visual_source = "environment_yaml";
  robot_reach.locked = true;
  robot_reach.editable = false;
  robot_reach.lock_reason = "generated environment preview";
  robot_reach.mesh_available = false;
  robot_reach.has_mesh_metadata = false;
  robot_reach.mesh_path.clear();

  EXPECT_TRUE(Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(robot_reach));
  EXPECT_TRUE(Scene3DViewportWidget::should_draw_as_solid_for_test(robot_reach, ScenePreviewWidget::MeshPreviewMode::Meshes));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_as_wireframe_for_test(robot_reach, ScenePreviewWidget::MeshPreviewMode::Auto));
  EXPECT_FALSE(Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(robot_reach));

  Scene3DViewportWidget viewport;
  viewport.ingest_preview_items(QVector<ScenePreviewWidget::PreviewItem>{ robot_reach });
  ASSERT_TRUE(viewport.render_smoke_fallback_frame(nullptr));

  const auto counters = viewport.render_debug_counters();
  EXPECT_EQ(counters.missing_geometry_count, 0);
  EXPECT_EQ(counters.primitive_fallback_count, 0);
  EXPECT_EQ(counters.editable_primitive_rendered_count, 1);
}

TEST(Scene3DRenderRoleClassifier, GeneratedUrdfBaseLinkInertiaIsMeshNotSemanticHelper)
{
  ASSERT_NE(ensure_app(), nullptr);

  auto base_link_inertia = make_item("generated_urdf::base_link_inertia::visual_0::0");
  base_link_inertia.display_name = "base_link_inertia";
  base_link_inertia.role = "robot";
  base_link_inertia.category = "robot/ur5";
  base_link_inertia.source_layer = "locked_generated_urdf_visual";
  base_link_inertia.active_visual_source = "mesh_preview";
  base_link_inertia.locked = true;
  base_link_inertia.editable = false;
  base_link_inertia.lock_reason = "generated URDF visual";
  base_link_inertia.visual_index_link = "base_link_inertia";
  base_link_inertia.visual_index_link_name = "base_link_inertia";
  base_link_inertia.visual_index_visual = "visual_0";
  base_link_inertia.visual_index_visual_name = "visual_0";
  base_link_inertia.package_uri = "package://ur_description/meshes/ur5/visual/base.dae";
  base_link_inertia.mesh_path = "assets/robots/universal_robot/ur_description/meshes/ur5/visual/base.dae";
  base_link_inertia.source_path = base_link_inertia.mesh_path;
  base_link_inertia.mesh_available = true;
  base_link_inertia.has_mesh_metadata = true;

  EXPECT_EQ(Scene3DViewportWidget::render_role_for_test(base_link_inertia), "generated_urdf_mesh");
  EXPECT_TRUE(Scene3DViewportWidget::should_include_in_default_fit_for_test(base_link_inertia));
  EXPECT_FALSE(Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(base_link_inertia));
  EXPECT_FALSE(Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(base_link_inertia));

  Scene3DViewportWidget viewport;
  viewport.ingest_preview_items(QVector<ScenePreviewWidget::PreviewItem>{ base_link_inertia });
  const QJsonArray final_draw = viewport.final_draw_visual_items_export();
  ASSERT_EQ(final_draw.size(), 1);
  const QJsonObject row = final_draw.at(0).toObject();
  EXPECT_EQ(row.value("source_layer").toString(), "locked_generated_urdf_visual");
  EXPECT_EQ(row.value("active_visual_source").toString(), "mesh_preview");
  EXPECT_EQ(row.value("canonical_link_name").toString(), "base_link_inertia");
  EXPECT_TRUE(row.value("package_uri").toString().contains("ur_description/meshes/ur5/visual/base.dae"));
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

TEST(CameraObservationZoneAuthoringSource, UsesIndependentTaskZoneModelAndProjection)
{
  QString root = QString::fromUtf8(__FILE__);
  root = root.left(root.lastIndexOf(QStringLiteral("/test/")));
  auto read = [](const QString & path) {
    QFile file(path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    return QString::fromUtf8(file.readAll());
  };
  const QString model = read(root + QStringLiteral("/include/object_placement_model.hpp"));
  const QString yaml = read(root + QStringLiteral("/gui/object_placement_yaml_io.cpp"));
  const QString dialog = read(root + QStringLiteral("/gui/object_placement_dialog.cpp"));
  EXPECT_TRUE(model.contains(QStringLiteral("std::string camera_id;")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("camera_observation")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("z.camera_id = camera.name")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("pick_zone_id")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("Camera view does not intersect the work surface")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("Observation zone uses default dimensions")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("camera.x + t * dx")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("camera.y + t * dy")));
  EXPECT_TRUE(dialog.contains(QStringLiteral("Create Observation Zone")));
  EXPECT_FALSE(dialog.contains(QStringLiteral("robot_motion")));
}

TEST(IndependentPickZoneAuthoring, ModelYamlAndOverlayContract)
{
  using namespace workcell_builder;
  std::string message;
  EXPECT_FALSE(can_create_pick_zone_for_robots({}, &message));
  EXPECT_EQ(message, "Add or select a robot before creating a Pick Zone");
  EXPECT_TRUE(can_create_pick_zone_for_robots({"ur5"}, &message));
  EXPECT_EQ(message, "ur5");
  EXPECT_TRUE(can_create_pick_zone_for_robots({"ur5", "ur10"}, &message));
  EXPECT_EQ(message, "Choose a robot for the Pick Zone");

  std::vector<TaskZone> existing;
  auto suggestion = suggest_robot_pick_zone("ur5", existing, 1.8, 0.2, 0.75, 0.3);
  ASSERT_TRUE(suggestion.ok);
  const auto & z = suggestion.zone;
  EXPECT_EQ(z.type, "pick");
  EXPECT_EQ(z.role, "pick");
  EXPECT_EQ(z.robot_id, "ur5");
  EXPECT_TRUE(z.camera_id.empty());
  EXPECT_NEAR(z.x, 1.8, 1e-9);
  EXPECT_NEAR(z.y, 0.2, 1e-9);
  EXPECT_NEAR(z.z, 0.75, 1e-9);
  const auto defaults = default_pick_zone_dimensions();
  EXPECT_GT(defaults.width, 0.0);
  EXPECT_GT(defaults.depth, 0.0);
  EXPECT_GT(defaults.height, 0.0);
  EXPECT_DOUBLE_EQ(z.dim_x, defaults.width);
  EXPECT_DOUBLE_EQ(z.dim_y, defaults.depth);
  EXPECT_DOUBLE_EQ(z.dim_z, defaults.height);

  TaskZone bad = z;
  bad.dim_x = 0.0;
  EXPECT_FALSE(validate_task_zone_dimensions(bad, &message));
  EXPECT_EQ(message, "Pick Zone dimensions must be positive");

  auto item = make_item("pick_zone_1");
  item.role = "pick";
  item.category = "Task Zones";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(item));
}

TEST(IndependentPlaceZoneAuthoring, ModelYamlAndOverlayContract)
{
  using namespace workcell_builder;
  std::string message;
  EXPECT_FALSE(can_create_place_zone_for_robots({}, &message));
  EXPECT_EQ(message, "Add or select a robot before creating a Place Zone");
  EXPECT_TRUE(can_create_place_zone_for_robots({"ur5"}, &message));
  EXPECT_EQ(message, "ur5");
  EXPECT_TRUE(can_create_place_zone_for_robots({"ur5", "ur10"}, &message));
  EXPECT_EQ(message, "Choose a robot for the Place Zone");

  std::vector<TaskZone> existing;
  auto pick = suggest_robot_pick_zone("ur5", existing, 0.1, 0.2, 0.75, 0.0);
  ASSERT_TRUE(pick.ok);
  existing.push_back(pick.zone);
  auto suggestion = suggest_robot_place_zone("ur5", existing, 1.8, -0.6, 0.82, 0.3);
  ASSERT_TRUE(suggestion.ok);
  const auto & z = suggestion.zone;
  EXPECT_EQ(z.type, "place");
  EXPECT_EQ(z.role, "place");
  EXPECT_EQ(z.robot_id, "ur5");
  EXPECT_TRUE(z.camera_id.empty());
  EXPECT_TRUE(z.object_ref.empty());
  EXPECT_TRUE(z.target_ref.empty());
  EXPECT_NEAR(z.x, 1.8, 1e-9);
  EXPECT_NEAR(z.y, -0.6, 1e-9);
  EXPECT_NEAR(z.z, 0.82, 1e-9);
  EXPECT_NE(z.x, pick.zone.x);
  EXPECT_NE(z.y, pick.zone.y);
  const auto defaults = default_task_zone_dimensions();
  EXPECT_GT(defaults.width, 0.0);
  EXPECT_GT(defaults.depth, 0.0);
  EXPECT_GT(defaults.height, 0.0);
  EXPECT_DOUBLE_EQ(z.dim_x, defaults.width);
  EXPECT_DOUBLE_EQ(z.dim_y, defaults.depth);
  EXPECT_DOUBLE_EQ(z.dim_z, defaults.height);

  TaskZone bad = z;
  bad.dim_z = -0.1;
  EXPECT_FALSE(validate_task_zone_dimensions(bad, &message));
  EXPECT_EQ(message, "Place Zone dimensions must be positive");

  auto item = make_item("place_zone_1");
  item.role = "place";
  item.category = "Task Zones";
  EXPECT_FALSE(Scene3DViewportWidget::should_include_in_default_fit_for_test(item));
  EXPECT_EQ(Scene3DViewportWidget::material_color_for_test(item), QColor("#a855f7"));
}

TEST(IndependentPlaceZoneAuthoring, SaveReloadPreservesRobotAndIndependence)
{
  using namespace workcell_builder;
  const QString path = QDir::tempPath() + QStringLiteral("/place_zone_contract_environment.yaml");
  QFile::remove(path);
  TaskZone z;
  z.id = "place_zone_1";
  z.type = "place";
  z.role = "place";
  z.robot_id = "ur5";
  z.x = 1.8; z.y = -0.6; z.z = 0.82; z.yaw = 0.3;
  z.dim_x = 0.5; z.dim_y = 0.5; z.dim_z = 0.12;
  z.enabled = true; z.visible = false;
  auto result = save_task_zones_to_environment_yaml(path.toStdString(), {z});
  ASSERT_TRUE(result.ok);
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString yaml = QString::fromUtf8(file.readAll());
  EXPECT_TRUE(yaml.contains(QStringLiteral("type: place")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("robot_id: ur5")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("target_ref: place_zone_1")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("camera_id")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("pick_zone_id")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("observation_zone_id")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("/workspace/")));

  std::vector<std::string> warnings;
  const auto loaded = load_task_zones_from_environment_yaml(path.toStdString(), &warnings);
  ASSERT_EQ(loaded.size(), 1u);
  EXPECT_EQ(loaded.front().id, z.id);
  EXPECT_EQ(loaded.front().type, "place");
  EXPECT_EQ(loaded.front().robot_id, "ur5");
  EXPECT_TRUE(loaded.front().camera_id.empty());
  EXPECT_DOUBLE_EQ(loaded.front().z, 0.82);
  EXPECT_FALSE(loaded.front().visible);
}

TEST(IndependentPickZoneAuthoring, SaveReloadPreservesRobotAndNoCameraLink)
{
  using namespace workcell_builder;
  const QString path = QDir::tempPath() + QStringLiteral("/pick_zone_contract_environment.yaml");
  QFile::remove(path);
  TaskZone z;
  z.id = "pick_zone_1";
  z.type = "pick";
  z.role = "pick";
  z.robot_id = "ur5";
  z.x = 1.8; z.y = 0.2; z.z = 0.75; z.yaw = 0.3;
  z.dim_x = 0.4; z.dim_y = 0.5; z.dim_z = 0.1;
  z.enabled = true; z.visible = false;
  auto result = save_task_zones_to_environment_yaml(path.toStdString(), {z});
  ASSERT_TRUE(result.ok);
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString yaml = QString::fromUtf8(file.readAll());
  EXPECT_TRUE(yaml.contains(QStringLiteral("type: pick")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("robot_id: ur5")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("camera_id")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("observation_zone_id")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("/workspace/")));

  std::vector<std::string> warnings;
  const auto loaded = load_task_zones_from_environment_yaml(path.toStdString(), &warnings);
  ASSERT_EQ(loaded.size(), 1u);
  EXPECT_EQ(loaded.front().id, z.id);
  EXPECT_EQ(loaded.front().type, "pick");
  EXPECT_EQ(loaded.front().robot_id, "ur5");
  EXPECT_TRUE(loaded.front().camera_id.empty());
  EXPECT_DOUBLE_EQ(loaded.front().z, 0.75);
  EXPECT_FALSE(loaded.front().visible);
}

TEST(ObservationToPickZoneLinks, ValidateSaveReloadAndIndependence)
{
  using namespace workcell_builder;
  TaskZone obs; obs.id = "camera_observation_1"; obs.type = "camera_observation"; obs.role = "camera_observation";
  obs.x = 0.1; obs.y = 0.2; obs.z = 0.3; obs.dim_x = 0.4; obs.dim_y = 0.5; obs.dim_z = 0.6;
  TaskZone pick; pick.id = "pick_zone_1"; pick.type = "pick"; pick.role = "pick";
  pick.x = 1.1; pick.y = 1.2; pick.z = 1.3; pick.dim_x = 0.7; pick.dim_y = 0.8; pick.dim_z = 0.9;
  TaskZone place; place.id = "place_zone_1"; place.type = "place"; place.role = "place";
  std::vector<TaskZone> zones{obs, pick, place};

  std::string warning;
  auto links = set_observation_pick_link({}, obs.id, pick.id, zones, &warning);
  ASSERT_EQ(links.size(), 1u);
  EXPECT_EQ(links.front().type, "observation_to_pick");
  EXPECT_EQ(links.front().source_zone_id, obs.id);
  EXPECT_EQ(links.front().target_zone_id, pick.id);
  EXPECT_TRUE(links.front().enabled);
  EXPECT_TRUE(validate_observation_to_pick_link(links.front(), zones, links, &warning));
  EXPECT_NE(links.front().status.find("tracking and timing not yet configured"), std::string::npos);
  EXPECT_EQ(obs.x, 0.1);
  EXPECT_EQ(pick.x, 1.1);

  const QString path = QDir::tempPath() + QStringLiteral("/observation_pick_links_environment.yaml");
  QFile::remove(path);
  ASSERT_TRUE(save_task_zones_to_environment_yaml(path.toStdString(), zones).ok);
  ASSERT_TRUE(save_task_zone_links_to_environment_yaml(path.toStdString(), links).ok);
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString yaml = QString::fromUtf8(file.readAll());
  EXPECT_TRUE(yaml.contains(QStringLiteral("task_zone_links:")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("type: observation_to_pick")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("source_zone_id: camera_observation_1")));
  EXPECT_TRUE(yaml.contains(QStringLiteral("target_zone_id: pick_zone_1")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("conveyor_speed")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("arrival")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("robot_motion")));
  EXPECT_FALSE(yaml.contains(QStringLiteral("/workspace/")));

  std::vector<std::string> warnings;
  const auto loaded = load_task_zone_links_from_environment_yaml(path.toStdString(), zones, &warnings);
  ASSERT_EQ(loaded.size(), 1u);
  EXPECT_TRUE(loaded.front().resolved);
  EXPECT_EQ(loaded.front().source_zone_id, obs.id);
  EXPECT_EQ(loaded.front().target_zone_id, pick.id);
  EXPECT_TRUE(warnings.empty());

  TaskZoneLink invalid = links.front();
  invalid.target_zone_id = place.id;
  EXPECT_FALSE(validate_observation_to_pick_link(invalid, zones, {}, &warning));
  EXPECT_EQ(warning, "Observation-to-Pick link target must be a Pick Zone");
}

TEST(ObservationToPickZoneLinks, CardinalityChangeRemovalAndMissingReferenceWarnings)
{
  using namespace workcell_builder;
  TaskZone obs1; obs1.id = "camera_observation_1"; obs1.type = "camera_observation"; obs1.role = "camera_observation";
  TaskZone obs2 = obs1; obs2.id = "camera_observation_2";
  TaskZone pick1; pick1.id = "pick_zone_1"; pick1.type = "pick"; pick1.role = "pick";
  TaskZone pick2 = pick1; pick2.id = "pick_zone_2";
  std::vector<TaskZone> zones{obs1, obs2, pick1, pick2};

  std::string warning;
  auto links = set_observation_pick_link({}, obs1.id, pick1.id, zones, &warning);
  links = set_observation_pick_link(links, obs1.id, pick1.id, zones, &warning);
  ASSERT_EQ(links.size(), 1u);
  links = set_observation_pick_link(links, obs2.id, pick1.id, zones, &warning);
  ASSERT_EQ(links.size(), 2u);
  links = set_observation_pick_link(links, obs1.id, pick2.id, zones, &warning);
  ASSERT_EQ(links.size(), 2u);
  auto obs1_link = std::find_if(links.begin(), links.end(), [](const TaskZoneLink & l){ return l.source_zone_id == "camera_observation_1"; });
  ASSERT_NE(obs1_link, links.end());
  EXPECT_EQ(obs1_link->target_zone_id, pick2.id);
  links = set_observation_pick_link(links, obs1.id, "", zones, &warning);
  EXPECT_EQ(links.size(), 1u);
  EXPECT_EQ(links.front().source_zone_id, obs2.id);

  TaskZoneLink bad; bad.id = "bad"; bad.type = "observation_to_pick"; bad.source_zone_id = obs1.id; bad.target_zone_id = "missing_pick";
  const QString path = QDir::tempPath() + QStringLiteral("/observation_pick_missing_target.yaml");
  QFile::remove(path);
  ASSERT_TRUE(save_task_zone_links_to_environment_yaml(path.toStdString(), {bad}).ok);
  std::vector<std::string> warnings;
  const auto loaded = load_task_zone_links_from_environment_yaml(path.toStdString(), zones, &warnings);
  ASSERT_EQ(loaded.size(), 1u);
  EXPECT_FALSE(loaded.front().resolved);
  ASSERT_FALSE(warnings.empty());
  EXPECT_NE(warnings.front().find("target is unavailable"), std::string::npos);
  EXPECT_EQ(zones.size(), 4u);
}
