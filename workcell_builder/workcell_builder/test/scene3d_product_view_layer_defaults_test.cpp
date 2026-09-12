#include "scene3d_candidate_assembly.h"

#include <gtest/gtest.h>

namespace
{
ScenePreviewWidget::PreviewItem make_item(
  const QString & source_layer,
  const QString & active_visual_source = QString(),
  const QString & id = QString())
{
  ScenePreviewWidget::PreviewItem item;
  item.source_layer = source_layer;
  item.active_visual_source = active_visual_source;
  item.id = id;
  item.status = QStringLiteral("ok");
  return item;
}

ScenePreviewWidget::PreviewItem generated_robot_mesh()
{
  auto item = make_item(
    QStringLiteral("locked_generated_urdf_visual"),
    QStringLiteral("mesh_preview"),
    QStringLiteral("generated_urdf::shoulder_link::visual_1::1"));
  item.display_name = QStringLiteral("shoulder_link");
  item.role = QStringLiteral("robot");
  item.category = QStringLiteral("robot/ur5");
  item.mesh_available = true;
  item.has_mesh_metadata = true;
  item.locked = true;
  item.mesh_path = QStringLiteral("package://ur_description/meshes/ur5/visual/shoulder.dae");
  item.package_uri = item.mesh_path;
  item.visual_index_mesh_uri = item.mesh_path;
  return item;
}

ScenePreviewWidget::PreviewItem stale_robot_primitive_box()
{
  auto item = make_item(
    QStringLiteral("primitive_fallback"),
    QStringLiteral("primitive_fallback"),
    QStringLiteral("semantic_helper::shoulder_link::bounds_box"));
  item.display_name = QStringLiteral("shoulder_link bounds box");
  item.role = QStringLiteral("robot_link");
  item.category = QStringLiteral("robot/ur5");
  item.locked = true;
  item.warnings = QStringList{QStringLiteral("mesh unavailable; using primitive fallback")};
  return item;
}

ScenePreviewWidget::PreviewItem missing_mesh_without_authoritative_visual()
{
  auto item = make_item(
    QStringLiteral("primitive_fallback"),
    QStringLiteral("primitive_fallback"),
    QStringLiteral("generated_primitive::tool0"));
  item.display_name = QStringLiteral("tool0 fallback");
  item.role = QStringLiteral("tool");
  item.category = QStringLiteral("gripper");
  item.mesh_load_warning = QStringLiteral("missing mesh");
  return item;
}

ScenePreviewWidget::PreviewItem overlay_only_payload()
{
  auto item = make_item(
    QStringLiteral("overlay"),
    QStringLiteral("overlay"),
    QStringLiteral("camera_fov::main"));
  item.display_name = QStringLiteral("Camera FOV overlay");
  item.role = QStringLiteral("camera_fov");
  item.category = QStringLiteral("diagnostic overlay");
  return item;
}

ScenePreviewWidget::PreviewItem editable_physical_item_with_helper_like_name()
{
  auto item = make_item(
    QStringLiteral("editable_layout"),
    QStringLiteral("mesh_preview"),
    QStringLiteral("layout::helper_fixture_table"));
  item.display_name = QStringLiteral("Helper Fixture Table");
  item.role = QStringLiteral("workbench");
  item.category = QStringLiteral("environment/table");
  item.mesh_available = true;
  item.has_mesh_metadata = true;
  item.editable = true;
  item.linked_to_editable_layout_state = true;
  return item;
}

void expect_clean_product_layers(const workcell_builder::Scene3DLayerVisibilityDefaults & defaults)
{
  EXPECT_TRUE(defaults.locked_generated_urdf_visual);
  EXPECT_TRUE(defaults.mesh_preview);
  EXPECT_TRUE(defaults.editable_layout);
  EXPECT_FALSE(defaults.overlay);
  EXPECT_FALSE(defaults.warning);
}
}  // namespace

TEST(Scene3DProductViewLayerDefaults, CleanGeneratedRobotMeshPayloadKeepsProductLayersAndHidesFallbacks)
{
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility({generated_robot_mesh()});

  expect_clean_product_layers(defaults);
  EXPECT_FALSE(defaults.primitive_fallback);
}

TEST(Scene3DProductViewLayerDefaults, GeneratedMeshSuppressesStalePrimitiveRobotBoxesByDefault)
{
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility(
    {generated_robot_mesh(), stale_robot_primitive_box()});

  expect_clean_product_layers(defaults);
  EXPECT_FALSE(defaults.primitive_fallback);
}

TEST(Scene3DProductViewLayerDefaults, MissingMeshWithoutAuthoritativeGeneratedVisualEnablesPrimitiveFallback)
{
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility(
    {missing_mesh_without_authoritative_visual()});

  expect_clean_product_layers(defaults);
  EXPECT_TRUE(defaults.primitive_fallback);
}

TEST(Scene3DProductViewLayerDefaults, OverlayOnlyPayloadKeepsOverlayLayerExplicitOptIn)
{
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility({overlay_only_payload()});

  expect_clean_product_layers(defaults);
  EXPECT_FALSE(defaults.primitive_fallback);
}

TEST(Scene3DProductViewLayerDefaults, EditablePhysicalItemWithHelperLikeNameRemainsDefaultVisible)
{
  const auto physical_item = editable_physical_item_with_helper_like_name();
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility({physical_item});
  const QSet<QString> enabled_layers{
    QStringLiteral("editable_layout"),
    QStringLiteral("mesh_preview"),
    QStringLiteral("locked_generated_urdf_visual")};

  expect_clean_product_layers(defaults);
  EXPECT_FALSE(defaults.primitive_fallback);
  EXPECT_TRUE(workcell_builder::include_preview_item_for_scene3d(physical_item, enabled_layers));
}

TEST(Scene3DProductViewLayerDefaults, AuthoredTaskZoneVisualUsesHelpersWithoutChangingProvenance)
{
  for (const QString & role : {QStringLiteral("pick_zone"), QStringLiteral("place_zone"),
       QStringLiteral("drop_zone"), QStringLiteral("task_zone")}) {
    auto zone = make_item(QStringLiteral("editable_layout"), QStringLiteral("semantic_primitive"), QStringLiteral("authored_region"));
    zone.role = role;
    zone.editable = true;
    zone.linked_to_editable_layout_state = true;
    zone.semantic_task_zone_helper = true;
    const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility({zone});
    expect_clean_product_layers(defaults);
    EXPECT_FALSE(defaults.primitive_fallback);
    EXPECT_FALSE(workcell_builder::include_preview_item_for_scene3d(zone, {QStringLiteral("editable_layout"), QStringLiteral("mesh_preview")}));
    EXPECT_TRUE(workcell_builder::include_preview_item_for_scene3d(zone, {QStringLiteral("overlay")}));
    EXPECT_TRUE(zone.editable);
    EXPECT_TRUE(zone.linked_to_editable_layout_state);
    EXPECT_EQ(zone.source_layer, QStringLiteral("editable_layout"));
    // A genuine diagnostic is not hidden behind the helper toggle.
    zone.mesh_load_warning = QStringLiteral("mesh resolution failed");
    EXPECT_FALSE(workcell_builder::include_preview_item_for_scene3d(zone, {QStringLiteral("overlay")}));
    EXPECT_TRUE(workcell_builder::include_preview_item_for_scene3d(zone, {QStringLiteral("warning")}));
  }
}
