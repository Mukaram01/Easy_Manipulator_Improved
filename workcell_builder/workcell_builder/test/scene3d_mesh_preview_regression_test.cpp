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
