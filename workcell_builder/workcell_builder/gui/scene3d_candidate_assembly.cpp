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
  Q_UNUSED(all_items);
  Scene3DLayerVisibilityDefaults out;
  // Generated/locked URDF visuals are part of the full Scene3D payload, not a
  // mutually exclusive fallback. Keep them visible by default so opening a
  // scene with editable layout rows plus mesh-index rows commits the combined
  // payload to the viewport instead of leaving the render loop on the initial
  // editable-only subset.
  out.locked_generated_urdf_visual = true;
  return out;
}

}  // namespace workcell_builder
