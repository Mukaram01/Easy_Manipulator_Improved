#include "scene3d_candidate_assembly.h"

namespace {
QString token(QString s) { return s.trimmed().toLower(); }
}

namespace workcell_builder {

bool include_preview_item_for_scene3d(
  const ScenePreviewWidget::PreviewItem & item,
  const QSet<QString> & enabled_layers)
{
  const QString source_layer = token(item.source_layer);
  const QString visual_source = token(item.active_visual_source);
  const QString role = token(item.role);
  const QString category = token(item.category);
  const QString combined = role + "|" + category + "|" + token(item.status) + "|" + item.warnings.join("|").toLower();
  const bool is_warning_or_missing = combined.contains("warning") || combined.contains("missing") || !item.mesh_load_warning.trimmed().isEmpty();
  const bool is_overlay_or_helper = combined.contains("overlay") || combined.contains("helper") || combined.contains("safety zone");

  if (source_layer == "editable_layout") return enabled_layers.contains("editable_layout");
  if (source_layer == "locked_generated_urdf_visual" || source_layer == "generated_urdf_visual") return enabled_layers.contains("locked_generated_urdf_visual");
  if (source_layer == "primitive_fallback") return enabled_layers.contains("primitive_fallback");
  if (visual_source == "mesh_preview") return enabled_layers.contains("mesh_preview");
  if (is_overlay_or_helper) return enabled_layers.contains("overlay");
  if (is_warning_or_missing) return enabled_layers.contains("warning");
  return true;
}

Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility(
  const QVector<ScenePreviewWidget::PreviewItem> & all_items)
{
  bool has_editable = false, has_mesh = false, has_primitive = false;
  for (const auto & item : all_items) {
    const QString source_layer = token(item.source_layer);
    const QString visual_source = token(item.active_visual_source);
    if (source_layer == "editable_layout") has_editable = true;
    if (visual_source == "mesh_preview") has_mesh = true;
    if (source_layer == "primitive_fallback" || visual_source == "primitive_fallback") has_primitive = true;
  }
  Scene3DLayerVisibilityDefaults out;
  out.locked_generated_urdf_visual = !(has_editable || has_mesh || has_primitive);
  return out;
}

}  // namespace workcell_builder
