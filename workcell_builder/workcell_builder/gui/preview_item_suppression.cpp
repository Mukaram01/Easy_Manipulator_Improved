#include "preview_item_suppression.h"

#include <QFileInfo>
#include <QHash>
#include <QRegularExpression>
#include <QStringList>

#include <cmath>

namespace workcell_builder
{
namespace
{
QString canonical_scene3d_token(QString value)
{
  const QString normalized = value.trimmed().toLower().replace('-', '_').replace(' ', '_');
  if (normalized == QStringLiteral("generated_preview") ||
      normalized == QStringLiteral("generated_urdf_visual") ||
      normalized == QStringLiteral("locked_generated_urdf") ||
      normalized == QStringLiteral("locked_generated_urdf_visual")) {
    return QStringLiteral("locked_generated_urdf_visual");
  }
  if (normalized == QStringLiteral("legacy_static_fallback")) return QStringLiteral("primitive_fallback");
  if (normalized == QStringLiteral("overlays") || normalized == QStringLiteral("helper_overlay")) return QStringLiteral("overlay");
  return normalized;
}

QString normalize_role(const QString & raw_role, const QString & fallback_text)
{
  const QString lower = (raw_role + " " + fallback_text).toLower();
  if (lower.contains("robot")) return QStringLiteral("robot");
  if (lower.contains("end_effector") || lower.contains("gripper") || lower.contains("tool")) return QStringLiteral("end_effector/tool");
  if (lower.contains("camera") || lower.contains("sensor")) return QStringLiteral("camera");
  if (lower.contains("support_surface") || lower.contains("table") || lower.contains("workbench")) return QStringLiteral("support_surface/table");
  if (lower.contains("conveyor")) return QStringLiteral("conveyor");
  if (lower.contains("pick_source") || lower.contains("pick zone") || lower.contains("pick_zone")) return QStringLiteral("pick source/zone");
  if (lower.contains("place_target") || lower.contains("place zone") || lower.contains("place_zone") || lower.contains("bin")) return QStringLiteral("place target/bin");
  if (lower.contains("home") || lower.contains("safe_joint_state") || lower.contains("safe joint") || lower.contains("safety pose")) return QStringLiteral("home/safety pose");
  if (lower.contains("safety")) return QStringLiteral("safety zone");
  if (lower.contains("object") || lower.contains("fixture") || lower.contains("asset")) return QStringLiteral("object");
  return QStringLiteral("object");
}

QString normalized_equivalence_token(QString value)
{
  value = value.trimmed().toLower().replace('-', '_').replace(' ', '_');
  value.replace(QRegularExpression(QStringLiteral("[^a-z0-9_]+")), QStringLiteral("_"));
  value.replace(QRegularExpression(QStringLiteral("_+")), QStringLiteral("_"));
  while (value.startsWith('_')) value.remove(0, 1);
  while (value.endsWith('_')) value.chop(1);
  return value;
}

bool path_has_mesh_asset_extension(QString path)
{
  path = path.trimmed();
  const int fragment_index = path.indexOf(QLatin1Char('#'));
  if (fragment_index >= 0) path = path.left(fragment_index);
  const int query_index = path.indexOf(QLatin1Char('?'));
  if (query_index >= 0) path = path.left(query_index);
  const QString suffix = QFileInfo(path).suffix().toLower();
  return suffix == QStringLiteral("dae") || suffix == QStringLiteral("stl") || suffix == QStringLiteral("obj") ||
         suffix == QStringLiteral("glb") || suffix == QStringLiteral("gltf");
}

QString normalized_equivalence_basename(const QString & value)
{
  QString trimmed = value.trimmed();
  const int fragment_index = trimmed.indexOf(QLatin1Char('#'));
  if (fragment_index >= 0) trimmed = trimmed.left(fragment_index);
  const int query_index = trimmed.indexOf(QLatin1Char('?'));
  if (query_index >= 0) trimmed = trimmed.left(query_index);
  if (trimmed.startsWith(QStringLiteral("package://"))) trimmed = trimmed.mid(QStringLiteral("package://").size());
  if (trimmed.startsWith(QStringLiteral("file://"))) trimmed = trimmed.mid(QStringLiteral("file://").size());
  const QString basename = QFileInfo(trimmed).fileName();
  return normalized_equivalence_token(basename.isEmpty() ? trimmed : basename);
}

double approximate_pose_token(double value)
{
  return QString::number(static_cast<int>(std::llround(value * 20.0)));
}

QString normalized_item_role(const ScenePreviewWidget::PreviewItem & item)
{
  return normalized_equivalence_token(normalize_role(item.role, item.category + " " + item.display_name + " " + item.id));
}

bool physical_asset_role(const ScenePreviewWidget::PreviewItem & item)
{
  const QString role = normalized_item_role(item);
  return role == QStringLiteral("robot") || role == QStringLiteral("gripper") ||
         role == QStringLiteral("end_effector_tool") || role == QStringLiteral("camera") ||
         role == QStringLiteral("table") || role == QStringLiteral("workbench") ||
         role == QStringLiteral("support_surface_table") || role == QStringLiteral("conveyor") ||
         role == QStringLiteral("object");
}

bool is_true_editable_source_of_truth(const ScenePreviewWidget::PreviewItem & item)
{
  return item.source_layer == QStringLiteral("editable_layout") && item.linked_to_editable_layout_state && item.editable && !item.locked;
}

bool is_syntactically_resolvable_package_mesh_uri(QString uri)
{
  uri = uri.trimmed();
  if (!uri.startsWith(QStringLiteral("package://"))) return false;
  if (!path_has_mesh_asset_extension(uri)) return false;
  const QString remainder = uri.mid(QStringLiteral("package://").size());
  const int slash_index = remainder.indexOf(QLatin1Char('/'));
  return slash_index > 0 && slash_index < remainder.size() - 1;
}

bool is_resolved_mesh_path_or_resolvable_package_uri(const QString & value)
{
  const QString trimmed = value.trimmed();
  if (trimmed.isEmpty() || !path_has_mesh_asset_extension(trimmed)) return false;
  if (is_syntactically_resolvable_package_mesh_uri(trimmed)) return true;
  QString local_path = trimmed;
  if (local_path.startsWith(QStringLiteral("file://"))) local_path = local_path.mid(QStringLiteral("file://").size());
  if (local_path.startsWith(QStringLiteral("package://"))) return false;
  const QFileInfo info(local_path);
  return info.exists() && info.isFile();
}

const QSet<QString> & required_ur5_scene3d_links()
{
  static const QSet<QString> links{
    QStringLiteral("base_link_inertia"),
    QStringLiteral("shoulder_link"),
    QStringLiteral("upper_arm_link"),
    QStringLiteral("forearm_link"),
    QStringLiteral("wrist_1_link"),
    QStringLiteral("wrist_2_link"),
    QStringLiteral("wrist_3_link")
  };
  return links;
}

QString required_ur5_link_name_from_preview_item(const ScenePreviewWidget::PreviewItem & item)
{
  const QStringList candidates{
    item.visual_index_link_name,
    item.visual_index_link,
    item.visual_index_object_name,
    item.display_name,
    item.id,
    item.frame_id,
    item.visual_index_visual,
    item.visual_index_visual_name,
    item.package_uri,
    item.visual_index_package_uri,
    item.visual_index_mesh_uri,
    item.mesh_path,
    item.source_path,
    item.resolved_source_path_original
  };

  for (const QString & value : candidates) {
    const QString token = normalized_equivalence_token(value);
    for (const QString & required : required_ur5_scene3d_links()) {
      if (token == required || token.contains(required)) return required;
    }
  }

  const QString mesh_hint = normalized_equivalence_token(candidates.join(QStringLiteral(" ")));
  if (mesh_hint.contains(QStringLiteral("base_dae"))) return QStringLiteral("base_link_inertia");
  if (mesh_hint.contains(QStringLiteral("shoulder_dae"))) return QStringLiteral("shoulder_link");
  if (mesh_hint.contains(QStringLiteral("upperarm_dae")) || mesh_hint.contains(QStringLiteral("upper_arm_dae"))) return QStringLiteral("upper_arm_link");
  if (mesh_hint.contains(QStringLiteral("forearm_dae"))) return QStringLiteral("forearm_link");
  if (mesh_hint.contains(QStringLiteral("wrist1_dae")) || mesh_hint.contains(QStringLiteral("wrist_1_dae"))) return QStringLiteral("wrist_1_link");
  if (mesh_hint.contains(QStringLiteral("wrist2_dae")) || mesh_hint.contains(QStringLiteral("wrist_2_dae"))) return QStringLiteral("wrist_2_link");
  if (mesh_hint.contains(QStringLiteral("wrist3_dae")) || mesh_hint.contains(QStringLiteral("wrist_3_dae"))) return QStringLiteral("wrist_3_link");
  return {};
}

bool preview_item_references_ur5_visual_mesh(const ScenePreviewWidget::PreviewItem & item)
{
  const QString mesh_hint = normalized_equivalence_token(QStringList{
    item.package_uri,
    item.visual_index_package_uri,
    item.visual_index_mesh_uri,
    item.mesh_path,
    item.source_path,
    item.resolved_source_path_original,
    item.id
  }.join(QStringLiteral(" ")));

  if (mesh_hint.contains(QStringLiteral("ur_description")) &&
      mesh_hint.contains(QStringLiteral("meshes_ur5_visual"))) {
    return true;
  }

  return mesh_hint.contains(QStringLiteral("base_dae")) ||
         mesh_hint.contains(QStringLiteral("shoulder_dae")) ||
         mesh_hint.contains(QStringLiteral("upperarm_dae")) ||
         mesh_hint.contains(QStringLiteral("upper_arm_dae")) ||
         mesh_hint.contains(QStringLiteral("forearm_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist1_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist_1_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist2_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist_2_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist3_dae")) ||
         mesh_hint.contains(QStringLiteral("wrist_3_dae"));
}

QString item_equivalence_link_name(const ScenePreviewWidget::PreviewItem & item)
{
  const QString required_ur5_link = required_ur5_link_name_from_preview_item(item);
  if (!required_ur5_link.isEmpty()) return required_ur5_link;
  if (!item.visual_index_link_name.trimmed().isEmpty()) return item.visual_index_link_name;
  if (!item.visual_index_link.trimmed().isEmpty()) return item.visual_index_link;
  if (!item.display_name.trimmed().isEmpty()) return item.display_name;
  return item.id;
}

}  // namespace

bool is_required_ur5_generated_visual_mesh_index_row(const ScenePreviewWidget::PreviewItem & item)
{
  const QString required_link = required_ur5_link_name_from_preview_item(item);
  if (required_link.isEmpty()) return false;
  if (!preview_item_references_ur5_visual_mesh(item)) return false;
  if (is_true_editable_source_of_truth(item) || item.linked_to_editable_layout_state) return false;

  const QString source_layer = canonical_scene3d_token(item.source_layer);
  const QString visual_source = canonical_scene3d_token(item.active_visual_source);
  const QString id_token = normalized_equivalence_token(item.id);
  const QString visual_index_source = normalized_equivalence_token(item.visual_index_source);

  const bool generated_visual_row =
    item.id.startsWith(QStringLiteral("generated_urdf::"), Qt::CaseInsensitive) ||
    item.id.startsWith(QStringLiteral("urdf_visual_"), Qt::CaseInsensitive) ||
    id_token.contains(QStringLiteral("urdf_visual")) ||
    source_layer == QStringLiteral("locked_generated_urdf_visual") ||
    source_layer.contains(QStringLiteral("generated_urdf")) ||
    source_layer.contains(QStringLiteral("urdf_visual")) ||
    visual_index_source.contains(QStringLiteral("urdf_flattened")) ||
    visual_index_source.contains(QStringLiteral("xacro"));

  const bool mesh_preview_source = visual_source.isEmpty() ||
    visual_source == QStringLiteral("mesh_preview") ||
    visual_source.contains(QStringLiteral("mesh"));

  return generated_visual_row && mesh_preview_source;
}

bool is_authoritative_generated_urdf_mesh_preview_item(const ScenePreviewWidget::PreviewItem & item)
{
  if (is_required_ur5_generated_visual_mesh_index_row(item)) return true;
  const auto is_protected_generated_mesh_index_visual = [](const ScenePreviewWidget::PreviewItem & candidate) {
    const bool generated_identity = candidate.id.trimmed().startsWith(QStringLiteral("generated_urdf::")) ||
      candidate.id.trimmed().startsWith(QStringLiteral("urdf_visual_"));
    if (!generated_identity) return false;
    if (candidate.linked_to_editable_layout_state || candidate.editable) return false;
    const QString source_layer = canonical_scene3d_token(candidate.source_layer);
    const QString visual_source = canonical_scene3d_token(candidate.active_visual_source);
    if (source_layer != QStringLiteral("locked_generated_urdf_visual") ||
        !(visual_source == QStringLiteral("mesh_preview") || visual_source.contains(QStringLiteral("mesh")))) {
      return false;
    }
    if (candidate.visual_index_link_name.trimmed().isEmpty() && candidate.visual_index_link.trimmed().isEmpty()) return false;
    const QStringList mesh_identity_fields = {candidate.package_uri, candidate.visual_index_package_uri, candidate.visual_index_mesh_uri,
      candidate.mesh_path, candidate.source_path, candidate.resolved_source_path_original};
    bool has_mesh_identity = false;
    bool has_resolvable_mesh_identity = false;
    for (const QString & value : mesh_identity_fields) {
      if (!path_has_mesh_asset_extension(value)) continue;
      has_mesh_identity = true;
      if (is_resolved_mesh_path_or_resolvable_package_uri(value)) {
        has_resolvable_mesh_identity = true;
        break;
      }
    }
    return has_mesh_identity && (candidate.mesh_available || has_resolvable_mesh_identity);
  };
  if (is_protected_generated_mesh_index_visual(item)) return true;
  if (!physical_asset_role(item)) return false;
  if (!item.locked || item.linked_to_editable_layout_state || item.editable) return false;
  const QString visual_source = canonical_scene3d_token(item.active_visual_source);
  const QString source_layer = canonical_scene3d_token(item.source_layer);
  if (source_layer != QStringLiteral("locked_generated_urdf_visual") || visual_source != QStringLiteral("mesh_preview")) return false;
  const bool has_valid_mesh_reference = path_has_mesh_asset_extension(item.resolved_source_path_original) ||
    path_has_mesh_asset_extension(item.mesh_path) || path_has_mesh_asset_extension(item.package_uri) ||
    path_has_mesh_asset_extension(item.visual_index_package_uri) || path_has_mesh_asset_extension(item.visual_index_mesh_uri) ||
    path_has_mesh_asset_extension(item.source_path);
  return item.mesh_available || item.has_mesh_metadata || has_valid_mesh_reference;
}

bool is_lower_fidelity_generated_placeholder(const ScenePreviewWidget::PreviewItem & item)
{
  if (is_true_editable_source_of_truth(item)) return false;
  if (item.editable || item.linked_to_editable_layout_state) return false;
  if (is_authoritative_generated_urdf_mesh_preview_item(item)) return false;
  if (!physical_asset_role(item)) return false;
  const QString layer = canonical_scene3d_token(item.source_layer);
  const QString visual_source = canonical_scene3d_token(item.active_visual_source);
  if (visual_source == QStringLiteral("urdf_primitive")) return false;
  const QString text = normalized_equivalence_token(item.id + " " + item.display_name + " " + item.category + " " + item.lock_reason + " " + item.mesh_load_warning + " " + item.warnings.join(" "));
  if (visual_source == QStringLiteral("primitive_fallback") || layer == QStringLiteral("primitive_fallback")) return true;
  if (text.contains(QStringLiteral("placeholder")) || text.contains(QStringLiteral("generated_bounds")) || text.contains(QStringLiteral("bounds_only")) || text.contains(QStringLiteral("raw_bounds"))) return true;
  if (layer == QStringLiteral("static_fallback") || layer == QStringLiteral("legacy_static_fallback")) return true;
  if (layer == QStringLiteral("locked_generated_urdf_visual")) {
    const bool lacks_mesh_identity = item.visual_index_link_name.trimmed().isEmpty() && item.visual_index_link.trimmed().isEmpty() &&
      !path_has_mesh_asset_extension(item.package_uri) && !path_has_mesh_asset_extension(item.visual_index_package_uri) &&
      !path_has_mesh_asset_extension(item.visual_index_mesh_uri) && !path_has_mesh_asset_extension(item.mesh_path) &&
      !path_has_mesh_asset_extension(item.source_path) && !path_has_mesh_asset_extension(item.resolved_source_path_original);
    if (lacks_mesh_identity) return true;
  }
  if (text.contains(QStringLiteral("legacy")) && text.contains(QStringLiteral("primitive_preview"))) return true;
  if (text.contains(QStringLiteral("mesh_metadata_missing_or_legacy"))) return true;
  return false;
}

QSet<QString> generated_urdf_preview_equivalence_keys_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  QSet<QString> keys;
  const QString role = normalized_item_role(item);
  if (role.isEmpty()) return keys;
  const QString link = normalized_equivalence_token(item_equivalence_link_name(item));
  const QString id_token = normalized_equivalence_token(item.id);
  const QString pose = QStringLiteral("%1_%2_%3_%4_%5_%6")
    .arg(approximate_pose_token(item.x), approximate_pose_token(item.y), approximate_pose_token(item.z),
         approximate_pose_token(item.roll), approximate_pose_token(item.pitch), approximate_pose_token(item.yaw));
  if (!link.isEmpty()) keys.insert(QStringLiteral("role_link:%1:%2").arg(role, link));
  if (!id_token.isEmpty()) keys.insert(QStringLiteral("role_link:%1:%2").arg(role, id_token));
  for (const QString & path_value : {item.package_uri, item.visual_index_package_uri, item.visual_index_mesh_uri,
                                    item.mesh_path, item.source_path, item.resolved_source_path_original}) {
    const QString source_basename = normalized_equivalence_basename(path_value);
    if (source_basename.isEmpty()) continue;
    keys.insert(QStringLiteral("role_source_basename:%1:%2").arg(role, source_basename));
    keys.insert(QStringLiteral("source_pose:%1:%2").arg(source_basename, pose));
  }
  keys.insert(QStringLiteral("role_pose:%1:%2").arg(role, pose));
  return keys;
}

QString generated_urdf_preview_suppression_reason_for_equivalence_key(const QString & key)
{
  if (key.startsWith(QStringLiteral("role_link:"))) return QStringLiteral("normalized_role_plus_link_or_display_name");
  if (key.startsWith(QStringLiteral("role_source_basename:"))) return QStringLiteral("normalized_role_plus_source_basename");
  if (key.startsWith(QStringLiteral("role_pose:"))) return QStringLiteral("normalized_role_plus_approximate_pose");
  if (key.startsWith(QStringLiteral("source_pose:"))) return QStringLiteral("normalized_source_basename_plus_approximate_pose");
  return QStringLiteral("equivalent_authoritative_mesh_index_visual");
}

PreviewSuppressionResult suppress_lower_fidelity_preview_items(
  const QVector<ScenePreviewWidget::PreviewItem> & preview_items,
  bool authoritative_mesh_index_healthy)
{
  QHash<QString, QStringList> authoritative_visual_equivalence_map;
  for (const auto & item : preview_items) {
    if (!is_authoritative_generated_urdf_mesh_preview_item(item)) continue;
    for (const QString & key : generated_urdf_preview_equivalence_keys_for_item(item)) {
      authoritative_visual_equivalence_map[key].append(item.id);
    }
  }

  PreviewSuppressionResult result;
  result.authoritative_visual_equivalence_key_count = authoritative_visual_equivalence_map.size();
  result.items.reserve(preview_items.size());
  for (const auto & item : preview_items) {
    if (is_true_editable_source_of_truth(item)) {
      ++result.preserved_editable_source_count;
      result.items.push_back(item);
      continue;
    }
    QString matched_equivalence_key;
    const bool lower_fidelity_generated_placeholder = is_lower_fidelity_generated_placeholder(item);
    if (lower_fidelity_generated_placeholder) {
      for (const QString & key : generated_urdf_preview_equivalence_keys_for_item(item)) {
        if (authoritative_visual_equivalence_map.contains(key)) {
          matched_equivalence_key = key;
          break;
        }
      }
    }
    const bool suppress_because_mesh_index_is_healthy =
      authoritative_mesh_index_healthy && lower_fidelity_generated_placeholder && matched_equivalence_key.isEmpty();
    if (!matched_equivalence_key.isEmpty() || suppress_because_mesh_index_is_healthy) {
      ++result.suppressed_preview_placeholder_count;
      const QString reason = suppress_because_mesh_index_is_healthy
        ? QStringLiteral("healthy_xacro_mesh_index_omits_legacy_placeholder")
        : generated_urdf_preview_suppression_reason_for_equivalence_key(matched_equivalence_key);
      result.suppression_reason_counts[reason] += 1;
      if (result.suppression_diagnostics.size() < 12) {
        const QString matched_ids = suppress_because_mesh_index_is_healthy
          ? QStringLiteral("authoritative_xacro_mesh_index")
          : authoritative_visual_equivalence_map.value(matched_equivalence_key).join(QLatin1Char('|'));
        result.suppression_diagnostics << QStringLiteral("%1=>%2 via %3").arg(item.id, matched_ids, reason);
      }
      continue;
    }
    result.items.push_back(item);
  }
  return result;
}

}  // namespace workcell_builder
