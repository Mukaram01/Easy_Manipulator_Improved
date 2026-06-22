#include "scene3d_candidate_assembly.h"

namespace {
QString token(QString s) { return s.trimmed().toLower().replace('-', '_').replace(' ', '_'); }

bool item_has_urdf_visual_mesh_identity(const ScenePreviewWidget::PreviewItem & item)
{
  const QString id = token(item.id);
  const QString source_layer = token(item.source_layer);
  const QString visual_source = token(item.active_visual_source);
  const QString source = token(item.visual_index_source + "|" + item.source_path + "|" + item.package_uri + "|" +
                               item.visual_index_package_uri + "|" + item.visual_index_mesh_uri + "|" +
                               item.mesh_path + "|" + item.resolved_source_path_original + "|" + item.metadata_tags);

  if (id.startsWith(QStringLiteral("urdf_visual_"))) return true;
  if (source_layer.contains(QStringLiteral("urdf_flattened")) ||
      visual_source.contains(QStringLiteral("urdf_flattened"))) return true;
  if (source.contains(QStringLiteral("source_urdf_flattened")) ||
      source.contains(QStringLiteral("source=urdf_flattened"))) return true;
  if (source.contains(QStringLiteral("package://ur_description/meshes/ur5/visual")) ||
      source.contains(QStringLiteral("ur_description/meshes/ur5/visual")) ||
      source.contains(QStringLiteral("ur_description_meshes_ur5_visual"))) return true;
  return false;
}

bool is_generated_robot_visual(const ScenePreviewWidget::PreviewItem & item)
{
  const QString source_layer = token(item.source_layer);
  const QString visual_source = token(item.active_visual_source);
  const QString role = token(item.role);
  const QString category = token(item.category);
  const QString id = token(item.id);
  const QString lock_reason = token(item.lock_reason);
  const QString combined = role + "|" + category + "|" + id + "|" + lock_reason;

  if (source_layer == "locked_generated_urdf_visual" || source_layer == "generated_urdf_visual") return true;
  if (visual_source == "locked_generated_urdf_visual" || visual_source == "generated_urdf_visual" ||
      visual_source == "generated_urdf_visual_fallback")
  {
    return true;
  }
  if (id.startsWith(QStringLiteral("generated_urdf_fallback::"))) return true;
  if (id.startsWith(QStringLiteral("generated_urdf::"))) return true;
  if (item_has_urdf_visual_mesh_identity(item)) return true;
  return item.locked && (combined.contains(QStringLiteral("urdf")) ||
                         combined.contains(QStringLiteral("generated")) ||
                         combined.contains(QStringLiteral("robot_link")) ||
                         combined.contains(QStringLiteral("robot_model")));
}
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

  // Treat generated robot visuals as a first-class layer, equivalent to the
  // legacy "Show Robot Links" control.  This check intentionally precedes
  // mesh-preview, label/selection, warning, and primitive-fallback decisions:
  // generated robot meshes and locked generated fallback link visuals must
  // stay visible whenever the generated robot layer is enabled. Raw
  // scene_visual_mesh_index.json rows can arrive with ids like urdf_visual_0_*
  // before downstream normalisation; these are also authoritative generated
  // URDF visuals and must not be dropped before the viewport renderer.
  if (is_generated_robot_visual(item)) return enabled_layers.contains("locked_generated_urdf_visual");
  if (source_layer == "editable_layout") return enabled_layers.contains("editable_layout");
  if (source_layer == "primitive_fallback") return enabled_layers.contains("primitive_fallback");
  if (visual_source == "mesh_preview") return enabled_layers.contains("mesh_preview");
  if (is_overlay_or_helper) return enabled_layers.contains("overlay");
  if (is_warning_or_missing) return enabled_layers.contains("warning");
  return true;
}

Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility(
  const QVector<ScenePreviewWidget::PreviewItem> & all_items)
{
  Scene3DLayerVisibilityDefaults out;
  // Product-view defaults should favor authored layout plus generated mesh
  // visuals, while leaving diagnostics hidden unless the loaded payload proves
  // they are needed for understanding missing assets.
  out.editable_layout = true;
  out.mesh_preview = true;
  out.locked_generated_urdf_visual = true;
  out.overlay = false;

  int primitive_fallback_count = 0;
  int missing_mesh_count = 0;
  int unresolved_package_uri_count = 0;
  int unsupported_extension_count = 0;
  int fallback_warning_count = 0;

  for (const auto & item : all_items) {
    const QString source_layer = token(item.source_layer);
    const QString visual_source = token(item.active_visual_source);
    const QString status = token(item.status);
    const QString warnings = item.warnings.join(QLatin1Char('|')).toLower();
    const QString mesh_warning = token(item.mesh_load_warning);
    const QString resolution_outcome = token(item.source_path_resolution_outcome);
    const QString combined = source_layer + QLatin1Char('|') + visual_source + QLatin1Char('|') +
      status + QLatin1Char('|') + warnings + QLatin1Char('|') + mesh_warning + QLatin1Char('|') +
      resolution_outcome;

    if (source_layer == QStringLiteral("primitive_fallback") ||
        visual_source == QStringLiteral("primitive_fallback")) {
      ++primitive_fallback_count;
    }
    if (combined.contains(QStringLiteral("missing mesh")) ||
        combined.contains(QStringLiteral("missing_source_path")) ||
        combined.contains(QStringLiteral("mesh unavailable")) ||
        combined.contains(QStringLiteral("mesh unresolved")) ||
        combined.contains(QStringLiteral("unresolved"))) {
      ++missing_mesh_count;
    }
    if (combined.contains(QStringLiteral("unresolved_package_uri")) ||
        combined.contains(QStringLiteral("package uri unresolved"))) {
      ++unresolved_package_uri_count;
    }
    if (combined.contains(QStringLiteral("unsupported")) &&
        (combined.contains(QStringLiteral("extension")) || combined.contains(QStringLiteral("format")))) {
      ++unsupported_extension_count;
    }
    if (combined.contains(QStringLiteral("fallback")) &&
        (combined.contains(QStringLiteral("missing")) || combined.contains(QStringLiteral("unavailable")) ||
         combined.contains(QStringLiteral("unresolved")))) {
      ++fallback_warning_count;
    }
  }

  // Normal product view must not show stale/static URDF primitive robot fallbacks
  // once an authoritative generated URDF mesh payload is present. Keep the
  // layer available for explicit diagnostics, but leave it off by default so
  // fallback robot boxes do not duplicate or clutter real robot visuals.
  int authoritative_generated_mesh_count = 0;
  for (const auto & item : all_items) {
    const QString source_layer = token(item.source_layer);
    if ((source_layer == QStringLiteral("locked_generated_urdf_visual") ||
         source_layer == QStringLiteral("generated_urdf_visual") ||
         item_has_urdf_visual_mesh_identity(item)) &&
        (item.mesh_available || item.has_mesh_metadata ||
         item.id.startsWith(QStringLiteral("generated_urdf_fallback::")) ||
         item.id.startsWith(QStringLiteral("urdf_visual_")))) {
      ++authoritative_generated_mesh_count;
    }
  }
  out.primitive_fallback = authoritative_generated_mesh_count == 0 && (primitive_fallback_count + missing_mesh_count) > 0;
  Q_UNUSED(unresolved_package_uri_count);
  Q_UNUSED(unsupported_extension_count);
  Q_UNUSED(fallback_warning_count);
  out.warning = false;
  return out;
}

}  // namespace workcell_builder
