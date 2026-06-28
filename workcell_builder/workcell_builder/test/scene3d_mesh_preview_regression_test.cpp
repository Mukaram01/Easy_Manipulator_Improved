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
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto draw_fn = src.find("bool Scene3DViewportWidget::draw_mesh_preview_if_available");
  ASSERT_NE(draw_fn, std::string::npos);
  const auto draw_fn_end = src.find("bool Scene3DViewportWidget::", draw_fn + 1);
  ASSERT_NE(draw_fn_end, std::string::npos);
  const std::string draw_body = src.substr(draw_fn, draw_fn_end - draw_fn);

  const auto baked_branch = draw_body.find("if (it.has_baked_world_visual_transform)");
  ASSERT_NE(baked_branch, std::string::npos);
  const auto non_baked_branch = draw_body.find("} else {", baked_branch);
  ASSERT_NE(non_baked_branch, std::string::npos);
  const auto scale = draw_body.find("glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z)", non_baked_branch);
  ASSERT_NE(scale, std::string::npos);

  const std::string baked_body = draw_body.substr(baked_branch, non_baked_branch - baked_branch);
  EXPECT_NE(baked_body.find("apply_authoritative_world_visual_transform_gl(it)"), std::string::npos);
  EXPECT_EQ(baked_body.find("apply_urdf_rpy_gl(it.mesh_r, it.mesh_p, it.mesh_y)"), std::string::npos);
  EXPECT_EQ(baked_body.find("origin_offset_x"), std::string::npos);
  EXPECT_EQ(baked_body.find("origin_offset_y"), std::string::npos);
  EXPECT_EQ(baked_body.find("origin_offset_z"), std::string::npos);
  EXPECT_LT(baked_branch, scale);

  const std::string non_baked_body = draw_body.substr(non_baked_branch, scale - non_baked_branch);
  const auto authoritative = non_baked_body.find("apply_authoritative_world_visual_transform_gl(it)");
  const auto mesh_rpy = non_baked_body.find("apply_urdf_rpy_gl(it.mesh_r, it.mesh_p, it.mesh_y)");
  const auto origin_offset = non_baked_body.find("if (preview_path && it.has_origin_offset) glTranslated(it.origin_offset_x, it.origin_offset_y, it.origin_offset_z)");
  ASSERT_NE(authoritative, std::string::npos);
  ASSERT_NE(mesh_rpy, std::string::npos);
  ASSERT_NE(origin_offset, std::string::npos);
  EXPECT_LT(authoritative, mesh_rpy);
  EXPECT_LT(mesh_rpy, origin_offset);

  EXPECT_NE(src.find("if (draw_mesh_preview_if_available(it, visual_color, true))"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsSelectionAndOverlayRendering)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("if (it->id == selected_id)"), std::string::npos);
  EXPECT_NE(src.find("draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz"), std::string::npos);
  EXPECT_NE(src.find("if (show_warning_labels && !it.warnings.isEmpty())"), std::string::npos);
  EXPECT_NE(src.find("LABEL_OVERLAP_SUPPRESS_LOWER_PRIORITY"), std::string::npos);
  EXPECT_NE(src.find("LABEL_PRIORITY_SELECTED_WARN_ANCHOR"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsMeshCacheBoundsAndModeHooks)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("entry.has_bounds = compute_mesh_bounds_for_test(entry.mesh, entry.min_bounds, entry.max_bounds);"), std::string::npos);
  EXPECT_NE(src.find("if (mode == ScenePreviewWidget::MeshPreviewMode::Primitives) return false;"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsVisualDifferentiationTokens)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("Generated / locked preview"), std::string::npos);
  EXPECT_NE(src.find("Editable layout"), std::string::npos);
  EXPECT_NE(src.find("generated_locked_preview_material"), std::string::npos);
  EXPECT_NE(src.find("generated_locked_preview_outline"), std::string::npos);
  EXPECT_NE(src.find("generated_primitive_fallback_fill"), std::string::npos);
  EXPECT_NE(src.find("editable_layout_accent_outline"), std::string::npos);
  EXPECT_NE(src.find("do not show a warning marker"), std::string::npos);
  EXPECT_NE(src.find("Layer: %3"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, RealSenseCameraPrimitiveUsesMeshAlignedSurrogate)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  EXPECT_NE(src.find("bool item_identifies_realsense_d435"), std::string::npos);
  EXPECT_NE(src.find("it.detection_label + QStringLiteral(\"|\") + it.camera_id"), std::string::npos);
  EXPECT_NE(src.find("it.metadata_tags + QStringLiteral(\"|\") + it.mesh_type"), std::string::npos);
  EXPECT_NE(src.find("return item_identifies_realsense_d435(it, mesh_source);"), std::string::npos);

  const auto primitive_branch = src.find("case NormalizedRole::Camera:\n      if (item_should_use_realsense_visual_surrogate(draw_item))");
  ASSERT_NE(primitive_branch, std::string::npos);
  const auto surrogate_call = src.find("draw_realsense_d435_visual_surrogate(draw_item);", primitive_branch);
  const auto generic_call = src.find("draw_camera_body_with_frustum(draw_item);", primitive_branch);
  ASSERT_NE(surrogate_call, std::string::npos);
  ASSERT_NE(generic_call, std::string::npos);
  EXPECT_LT(surrogate_call, generic_call);

  const auto surrogate = src.find("void Scene3DViewportWidget::draw_realsense_d435_visual_surrogate");
  ASSERT_NE(surrogate, std::string::npos);
  const auto generic = src.find("void Scene3DViewportWidget::draw_camera_body_with_frustum", surrogate);
  ASSERT_NE(generic, std::string::npos);
  const std::string body = src.substr(surrogate, generic - surrogate);
  EXPECT_NE(body.find("apply_urdf_pose_gl(it.x, it.y, it.z, it.roll, it.pitch, it.yaw)"), std::string::npos);
  EXPECT_NE(body.find("if (it.visual_origin_applied)"), std::string::npos);
  EXPECT_NE(body.find("apply_urdf_rpy_gl(it.mesh_r, it.mesh_p, it.mesh_y)"), std::string::npos);
  EXPECT_NE(body.find("if (it.has_origin_offset) glTranslated(it.origin_offset_x"), std::string::npos);
  EXPECT_NE(body.find("glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z)"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, UsesUrdfRvizRpyOrderForGeneratedVisuals)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto helper = src.find("void apply_urdf_rpy_gl(double roll, double pitch, double yaw)");
  ASSERT_NE(helper, std::string::npos);
  const auto helper_end = src.find("void apply_urdf_pose_gl", helper);
  ASSERT_NE(helper_end, std::string::npos);
  const std::string helper_body = src.substr(helper, helper_end - helper);
  const auto yaw = helper_body.find("glRotated(qRadiansToDegrees(yaw)");
  const auto pitch = helper_body.find("glRotated(qRadiansToDegrees(pitch)");
  const auto roll = helper_body.find("glRotated(qRadiansToDegrees(roll)");
  ASSERT_NE(yaw, std::string::npos);
  ASSERT_NE(pitch, std::string::npos);
  ASSERT_NE(roll, std::string::npos);
  EXPECT_LT(yaw, pitch);
  EXPECT_LT(pitch, roll);

  EXPECT_NE(src.find("apply_urdf_pose_matrix(final_transform, it.x, it.y, it.z, it.roll, it.pitch, it.yaw)"), std::string::npos);
  EXPECT_NE(src.find("apply_urdf_pose_gl(it.x, it.y, it.z, it.roll, it.pitch, it.yaw)"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsScene3DDiagnosticsSummaryLine)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/mainwindow.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("Scene3D: editable=%1, mesh=%2, generated=%3, fallback=%4, missing=%5, locked=%6"), std::string::npos);
  EXPECT_NE(src.find("Scene3D warnings: extraction_mode=%1, safe_for_preview=%2, missing_mesh=%3, unresolved_package_uri=%4, unsupported_extension=%5, stale_or_absolute_only_mesh_index=%6"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, HealthyXacroMeshIndexSuppressesLegacyPrimitiveFallbackState)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/mainwindow.cpp");
  ASSERT_FALSE(src.empty());

  EXPECT_NE(src.find("authoritative_mesh_index_healthy"), std::string::npos);
  EXPECT_NE(src.find("visual_index_extraction_mode.trimmed().compare(QStringLiteral(\"xacro_expanded\"), Qt::CaseInsensitive) == 0"), std::string::npos);
  EXPECT_NE(src.find("visual_index_safe_for_preview &&"), std::string::npos);
  EXPECT_NE(src.find("mesh_item_count > 0"), std::string::npos);
  EXPECT_NE(src.find("primitive_item_count == 0"), std::string::npos);
  EXPECT_NE(src.find("missing_mesh_count == 0"), std::string::npos);
  EXPECT_NE(src.find("unresolved_package_uri_count == 0"), std::string::npos);
  EXPECT_NE(src.find("unsupported_extension_count == 0"), std::string::npos);
  EXPECT_NE(src.find("healthy_xacro_expanded_mesh_index_omits_legacy_placeholder"), std::string::npos);
  EXPECT_NE(src.find("mesh_metadata_missing_or_legacy"), std::string::npos);
  EXPECT_NE(src.find("Preview placeholder suppression: omitted %1 legacy placeholders because authoritative xacro-expanded mesh index is healthy"), std::string::npos);
  EXPECT_NE(src.find("if (is_true_editable_source_of_truth(item))"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsMeshLoadFailureReasonDiagnostics)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("package_uri_unresolved"), std::string::npos);
  EXPECT_NE(src.find("stale_path"), std::string::npos);
  EXPECT_NE(src.find("file_not_found"), std::string::npos);
  EXPECT_NE(src.find("row[\"load_failure_reason\"]"), std::string::npos);
  EXPECT_NE(src.find("mesh missing on disk (reason_code: %1)"), std::string::npos);
  EXPECT_NE(src.find("ensure_mesh_cached(it, mesh_source)"), std::string::npos);
  EXPECT_NE(src.find("ensure_mesh_cached(item, mesh_source)"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsStaleResolvedSourcePathPackageUriDiagnostics)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/mainwindow.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("should_try_package_uri_after_stale_resolved_source_path"), std::string::npos);
  EXPECT_NE(src.find("resolved_via_package_uri_after_stale_resolved_source_path"), std::string::npos);
  EXPECT_NE(src.find("package_uri_resolved_by_loader"), std::string::npos);
  EXPECT_NE(src.find("package_uri_resolved_after_stale_resolved_source_path"), std::string::npos);
  EXPECT_NE(src.find("stale_resolved_source_path_unresolved_after_package_uri_attempt"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, KeepsSceneAuthoringSemanticPrimitiveAssembly)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/mainwindow.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("add_scene_authoring_semantic_primitives"), std::string::npos);
  EXPECT_NE(src.find("layout/workcell_studio_layout.yaml"), std::string::npos);
  EXPECT_NE(src.find("environment.yaml"), std::string::npos);
  EXPECT_NE(src.find("cell_definition.yaml"), std::string::npos);
  EXPECT_NE(src.find("scene_manifest.yaml"), std::string::npos);
  EXPECT_NE(src.find("semantic_primitive"), std::string::npos);
  EXPECT_NE(src.find("editable_semantic_keys.contains(semantic_key)"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, ProductWarningStateExcludesEditableAndOverlayDiagnostics)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto product = src.find("bool product_view_has_generated_mesh_warning_content");
  const auto debug = src.find("bool debug_view_has_full_diagnostic_warning_content");
  ASSERT_NE(product, std::string::npos);
  ASSERT_NE(debug, std::string::npos);
  ASSERT_LT(product, debug);

  const std::string product_body = src.substr(product, debug - product);
  EXPECT_NE(src.find("WORKCELL_SCENE3D_DEBUG_FALLBACK_BOXES"), std::string::npos);
  EXPECT_NE(src.find("scene3d_debug_fallback_boxes_enabled() ? qMax(0, mesh_source_count - mesh_surface_rendered_count) : 0"), std::string::npos);
  EXPECT_NE(product_body.find("generated_mesh_bounds_fallback_rendered_count"), std::string::npos);
  EXPECT_NE(product_body.find("generated_missing_geometry_count"), std::string::npos);
  EXPECT_NE(product_body.find("generated_fallback_count"), std::string::npos);
  EXPECT_EQ(product_body.find("wireframe_fallback_count"), std::string::npos);
  EXPECT_EQ(product_body.find("overlay_helper_count"), std::string::npos);
  EXPECT_EQ(product_body.find("editable_primitive_rendered_count"), std::string::npos);
  EXPECT_EQ(product_body.find("primitive_fallback_rendered_count"), std::string::npos);
  EXPECT_EQ(product_body.find("visual_quality_status"), std::string::npos);

  const auto warning_selector = src.find("debug_overlays_mode\n    ? debug_view_has_full_diagnostic_warning_content(last_render_counters)\n    : product_view_has_generated_mesh_warning_content(last_render_counters)");
  EXPECT_NE(warning_selector, std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, CentralizesRosToViewportBasisForGeneratedVisuals)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto basis = src.find("QMatrix4x4 ros_to_viewport_basis_matrix()");
  ASSERT_NE(basis, std::string::npos);
  const auto authoritative = src.find("QMatrix4x4 authoritative_world_visual_transform", basis);
  ASSERT_NE(authoritative, std::string::npos);
  const std::string basis_body = src.substr(basis, authoritative - basis);
  EXPECT_NE(basis_body.find("basis(1, 2) = 1.0f"), std::string::npos);  // ROS Z becomes viewport up/Y.
  EXPECT_NE(basis_body.find("basis(2, 1) = -1.0f"), std::string::npos); // ROS Y becomes viewport depth.

  const auto viewport_root = src.find("QMatrix4x4 viewport_world_visual_transform", authoritative);
  ASSERT_NE(viewport_root, std::string::npos);
  const auto viewport_root_end = src.find("void apply_authoritative_world_visual_transform_gl", viewport_root);
  ASSERT_NE(viewport_root_end, std::string::npos);
  const std::string viewport_body = src.substr(viewport_root, viewport_root_end - viewport_root);
  EXPECT_NE(viewport_body.find("return ros_to_viewport_basis_matrix() * authoritative_world_visual_transform(item);"), std::string::npos);

  const auto final_mesh = src.find("QMatrix4x4 final_mesh_transform_matrix");
  ASSERT_NE(final_mesh, std::string::npos);
  const auto final_mesh_end = src.find("void apply_mesh_local_correction_gl", final_mesh);
  ASSERT_NE(final_mesh_end, std::string::npos);
  const std::string final_body = src.substr(final_mesh, final_mesh_end - final_mesh);
  EXPECT_NE(final_body.find("QMatrix4x4 transform = ros_to_viewport_basis_matrix() * item.baked_world_visual_matrix;"), std::string::npos)
    << "matrix-baked rows must use the raw ROS matrix plus the Scene3D viewport basis";
  EXPECT_NE(final_body.find("QMatrix4x4 transform = viewport_world_visual_transform(item);"), std::string::npos);
  EXPECT_NE(final_body.find("apply_mesh_local_correction_matrix(transform, item);"), std::string::npos);
  EXPECT_NE(final_body.find("transform.scale"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, MatrixBakedUr5VisualsApplyScopedAssetCorrectionBeforeScale)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());

  const auto final_mesh = src.find("QMatrix4x4 final_mesh_transform_matrix");
  ASSERT_NE(final_mesh, std::string::npos);
  const auto final_mesh_end = src.find("void apply_mesh_local_correction_gl", final_mesh);
  ASSERT_NE(final_mesh_end, std::string::npos);
  const std::string final_body = src.substr(final_mesh, final_mesh_end - final_mesh);

  const auto baked_matrix_branch = final_body.find("if (item.has_baked_world_visual_transform && item.has_baked_world_visual_matrix)");
  ASSERT_NE(baked_matrix_branch, std::string::npos);
  const auto fallback_branch = final_body.find("if (item.has_baked_world_visual_transform)", baked_matrix_branch + 1);
  ASSERT_NE(fallback_branch, std::string::npos);
  const std::string baked_matrix_body = final_body.substr(baked_matrix_branch, fallback_branch - baked_matrix_branch);

  const auto matrix = baked_matrix_body.find("QMatrix4x4 transform = ros_to_viewport_basis_matrix() * item.baked_world_visual_matrix;");
  const auto scoped_correction = baked_matrix_body.find("apply_baked_mesh_asset_local_correction_matrix(transform, item);");
  const auto legacy_correction = baked_matrix_body.find("apply_mesh_local_correction_matrix(transform, item);");
  const auto scale = baked_matrix_body.find("transform.scale");
  ASSERT_NE(matrix, std::string::npos);
  ASSERT_NE(scoped_correction, std::string::npos);
  ASSERT_NE(scale, std::string::npos);
  EXPECT_LT(matrix, scoped_correction);
  EXPECT_LT(scoped_correction, scale)
    << "matrix-baked UR5 generated visual rows need asset-local correction before final draw bounds are exported";
  EXPECT_EQ(legacy_correction, std::string::npos)
    << "matrix-baked generated URDF visuals must not re-enter broad legacy mesh-local corrections";
}

TEST(Scene3DMeshPreviewRegression, SuppressesFallbackBoxesWhenMeshSurfaceLoads)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("if (draw_mesh_preview_if_available(it, visual_color, true))"), std::string::npos);
  EXPECT_NE(src.find("return true;\n  }\n  if (generated_or_locked_preview && item_has_valid_urdf_primitive(it))"), std::string::npos)
    << "successful mesh draw must return before URDF primitive/fallback boxes are considered";
  EXPECT_NE(src.find("WORKCELL_SCENE3D_DEBUG_FALLBACK_BOXES"), std::string::npos);
  EXPECT_NE(src.find("if (!editable_layout_preview && !scene3d_debug_fallback_boxes_enabled())"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, BakedUrdfVisualsDoNotUseAdHocUr5MeshCorrections)
{
  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  EXPECT_NE(src.find("bool item_is_urdf_flattened_generated_preview"), std::string::npos);
  const auto helper = src.find("bool should_apply_baked_mesh_asset_local_correction");
  ASSERT_NE(helper, std::string::npos);
  const auto helper_end = src.find("QString baked_mesh_asset_local_correction_reason", helper);
  ASSERT_NE(helper_end, std::string::npos);
  const std::string body = src.substr(helper, helper_end - helper);
  EXPECT_NE(body.find("source=urdf_flattened"), std::string::npos)
    << "flattened generated entries must be explicitly excluded from legacy UR5 correction";
  EXPECT_NE(body.find("item_is_urdf_flattened_generated_preview(item)"), std::string::npos);
  EXPECT_NE(body.find("return false;"), std::string::npos);
  EXPECT_NE(body.find("Do not reintroduce the old"), std::string::npos);
  EXPECT_EQ(body.find("item_references_ur5_baked_mesh_asset_local_correction(item)"), std::string::npos);
}

TEST(Scene3DMeshPreviewRegression, FlattenedUrdfEntriesAreRawRosAndCorrectedOnce)
{
  const std::string extractor = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/../../scripts/extract_scene_urdf_visual_mesh_index.py");
  ASSERT_FALSE(extractor.empty());
  EXPECT_NE(extractor.find("'source':'urdf_flattened'"), std::string::npos);
  EXPECT_NE(extractor.find("'ros_to_viewport_basis_applied':False"), std::string::npos)
    << "scene_visual_mesh_index.json must declare that extractor output is raw ROS/RViz basis";
  EXPECT_EQ(extractor.find("ros_to_viewport_basis_matrix"), std::string::npos)
    << "extractor must not bake Scene3D viewport basis into world-space URDF poses";

  const std::string src = load_file(std::string(WORKCELL_BUILDER_SOURCE_DIR) + "/gui/scene3d_viewport_widget.cpp");
  ASSERT_FALSE(src.empty());
  const auto viewport_root = src.find("QMatrix4x4 viewport_world_visual_transform");
  ASSERT_NE(viewport_root, std::string::npos);
  const auto viewport_root_end = src.find("void apply_authoritative_world_visual_transform_gl", viewport_root);
  ASSERT_NE(viewport_root_end, std::string::npos);
  const std::string viewport_body = src.substr(viewport_root, viewport_root_end - viewport_root);
  EXPECT_NE(viewport_body.find("ros_to_viewport_basis_matrix() * authoritative_world_visual_transform(item)"), std::string::npos);

  const auto correction_helper = src.find("bool should_apply_baked_mesh_asset_local_correction");
  ASSERT_NE(correction_helper, std::string::npos);
  const auto correction_helper_end = src.find("QString baked_mesh_asset_local_correction_reason", correction_helper);
  ASSERT_NE(correction_helper_end, std::string::npos);
  const std::string correction_body = src.substr(correction_helper, correction_helper_end - correction_helper);
  EXPECT_NE(correction_body.find("if (item_is_urdf_flattened_generated_preview(item)) return false;"), std::string::npos)
    << "source=urdf_flattened entries must not receive legacy UR5 mesh correction after basis conversion";
}
