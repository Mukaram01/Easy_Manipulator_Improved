#include "scene3d_visual_classification.h"


namespace workcell_builder::scene3d_visual_classification {

QString normalized_token(QString value)
{
  return value.trimmed().toLower().replace(QLatin1Char('-'), QLatin1Char('_')).replace(QLatin1Char(' '), QLatin1Char('_'));
}

QString normalized_layer_token(const QString & value)
{
  const QString normalized = normalized_token(value);
  if (normalized == QStringLiteral("generated_preview")) return QStringLiteral("generated_urdf_visual");
  if (normalized == QStringLiteral("locked_generated_urdf")) return QStringLiteral("locked_generated_urdf_visual");
  if (normalized == QStringLiteral("legacy_static_fallback")) return QStringLiteral("primitive_fallback");
  if (normalized == QStringLiteral("overlays") || normalized == QStringLiteral("helper_overlay")) return QStringLiteral("overlay");
  return normalized;
}

QStringList canonical_helper_overlay_tokens()
{
  return {
    QStringLiteral("overlay"),
    QStringLiteral("helper"),
    QStringLiteral("diagnostic"),
    QStringLiteral("safety_zone"),
    QStringLiteral("pick_zone"),
    QStringLiteral("place_zone"),
    QStringLiteral("robot_reach"),
    QStringLiteral("warning_anchor"),
    QStringLiteral("warning_badge"),
    QStringLiteral("camera_fov"),
    QStringLiteral("camera_observation"),
    QStringLiteral("fov"),
    QStringLiteral("pick_coverage"),
    QStringLiteral("reachability"),
    QStringLiteral("collision"),
    QStringLiteral("work_envelope"),
    QStringLiteral("task_route"),
    QStringLiteral("approach_retreat"),
    QStringLiteral("epd_detection"),
    QStringLiteral("detection_label"),
    QStringLiteral("bounds_box"),
    QStringLiteral("bounding_box"),
  };
}

QString mesh_identity_text(const ScenePreviewWidget::PreviewItem & item)
{
  return normalized_token(item.visual_index_source + QStringLiteral("|") + item.source_path + QStringLiteral("|") +
                          item.package_uri + QStringLiteral("|") + item.visual_index_package_uri + QStringLiteral("|") +
                          item.visual_index_mesh_uri + QStringLiteral("|") + item.mesh_path + QStringLiteral("|") +
                          item.resolved_source_path_original + QStringLiteral("|") + item.metadata_tags + QStringLiteral("|") +
                          item.mesh_type + QStringLiteral("|") + item.material_name + QStringLiteral("|") +
                          item.source_path_resolution_outcome);
}

bool is_generated_urdf_visual_identity(const ScenePreviewWidget::PreviewItem & item)
{
  const QString source_layer = normalized_layer_token(item.source_layer);
  const QString visual_source = normalized_layer_token(item.active_visual_source);
  const QString id = normalized_token(item.id);
  const QString role = normalized_token(item.role);
  const QString category = normalized_token(item.category);
  const QString lock_reason = normalized_token(item.lock_reason);
  const QString source = mesh_identity_text(item);
  const QString combined = role + QStringLiteral("|") + category + QStringLiteral("|") + id + QStringLiteral("|") + lock_reason;

  if (source_layer == QStringLiteral("locked_generated_urdf_visual") || source_layer == QStringLiteral("generated_urdf_visual")) return true;
  if (visual_source == QStringLiteral("locked_generated_urdf_visual") || visual_source == QStringLiteral("generated_urdf_visual") ||
      visual_source == QStringLiteral("generated_urdf_visual_fallback")) return true;
  if (id.startsWith(QStringLiteral("generated_urdf_fallback::")) || id.startsWith(QStringLiteral("generated_urdf::")) ||
      id.startsWith(QStringLiteral("urdf_visual_"))) return true;
  if (source_layer.contains(QStringLiteral("urdf_flattened")) || visual_source.contains(QStringLiteral("urdf_flattened"))) return true;
  if (source.contains(QStringLiteral("source_urdf_flattened")) || source.contains(QStringLiteral("source=urdf_flattened"))) return true;
  if (source.contains(QStringLiteral("package://ur_description/meshes/ur5/visual")) ||
      source.contains(QStringLiteral("ur_description/meshes/ur5/visual")) ||
      source.contains(QStringLiteral("ur_description_meshes_ur5_visual"))) return true;
  return item.locked && (combined.contains(QStringLiteral("urdf")) || combined.contains(QStringLiteral("generated")) ||
                         combined.contains(QStringLiteral("robot_link")) || combined.contains(QStringLiteral("robot_model")));
}

bool token_matches(const QString & text, const QString & helper_token)
{
  return text == helper_token || text.contains(helper_token);
}

bool identity_contains_helper_overlay_token(const ScenePreviewWidget::PreviewItem & item)
{
  const QString direct_identity = normalized_layer_token(item.source_layer) + QStringLiteral("|") +
    normalized_layer_token(item.active_visual_source) + QStringLiteral("|") + normalized_token(item.role) + QStringLiteral("|") +
    normalized_token(item.category) + QStringLiteral("|") + normalized_token(item.id) + QStringLiteral("|") +
    normalized_token(item.display_name) + QStringLiteral("|") + normalized_token(item.status) + QStringLiteral("|") +
    normalized_token(item.warnings.join(QStringLiteral("|"))) + QStringLiteral("|") + normalized_token(item.mesh_load_warning) +
    QStringLiteral("|") + mesh_identity_text(item);

  for (const QString & helper_token : canonical_helper_overlay_tokens()) {
    if (token_matches(direct_identity, helper_token)) return true;
  }
  return false;
}

bool is_helper_overlay_identity(const ScenePreviewWidget::PreviewItem & item)
{
  // Authoritative generated URDF visuals and mesh-backed product geometry must
  // not be hidden solely because a link, mesh, or package identifier happens to
  // contain a helper-like substring.
  if (is_generated_urdf_visual_identity(item)) return false;
  if ((item.mesh_available || item.has_mesh_metadata) &&
      (normalized_layer_token(item.source_layer) == QStringLiteral("mesh_preview") ||
       normalized_layer_token(item.active_visual_source) == QStringLiteral("mesh_preview") ||
       normalized_layer_token(item.source_layer) == QStringLiteral("editable_layout"))) {
    const QString role_category = normalized_token(item.role) + QStringLiteral("|") + normalized_token(item.category) +
      QStringLiteral("|") + normalized_layer_token(item.source_layer) + QStringLiteral("|") + normalized_layer_token(item.active_visual_source);
    for (const QString & helper_token : canonical_helper_overlay_tokens()) {
      if (token_matches(role_category, helper_token)) return true;
    }
    return false;
  }
  return identity_contains_helper_overlay_token(item);
}

}  // namespace workcell_builder::scene3d_visual_classification
