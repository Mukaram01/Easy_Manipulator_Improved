// Implementation fragment included by scene3d_viewport_widget.cpp so diagnostics helpers
// can keep access to the viewport private render/cache state during incremental extraction.

QJsonArray scene3d_matrix_to_json(const QMatrix4x4 & matrix)
{
  QJsonArray out;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) out.append(matrix(row, col));
  }
  return out;
}

QJsonArray scene3d_vec_to_json(const QVector3D & v)
{
  return QJsonArray{v.x(), v.y(), v.z()};
}

QJsonObject scene3d_bbox_to_json(const QVector3D & min, const QVector3D & max)
{
  QJsonObject bbox;
  bbox["min"] = scene3d_vec_to_json(min);
  bbox["max"] = scene3d_vec_to_json(max);
  bbox["span"] = scene3d_vec_to_json(max - min);
  return bbox;
}

QJsonArray scene3d_pose_to_json(double x, double y, double z, double roll, double pitch, double yaw)
{
  return QJsonArray{x, y, z, roll, pitch, yaw};
}

QString scene3d_link_name_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  if (!item.visual_index_link_name.trimmed().isEmpty()) return item.visual_index_link_name.trimmed();
  if (!item.visual_index_link.trimmed().isEmpty()) return item.visual_index_link.trimmed();
  if (!item.frame_id.trimmed().isEmpty()) return item.frame_id.trimmed();
  const QString display = item.display_name.trimmed();
  if (!display.isEmpty() && display != item.id.trimmed()) return display;
  const QString id = item.id.trimmed();
  for (const QString & sep : {QStringLiteral("::"), QStringLiteral("/visual"), QStringLiteral("__visual")}) {
    const int idx = id.indexOf(sep);
    if (idx > 0) return id.left(idx);
  }
  return id;
}

QString scene3d_canonical_link_name(const QString & raw_link)
{
  QString link = raw_link.trimmed();
  if (link.isEmpty()) return link;
  const QString lower = link.toLower();
  if (lower.contains(QStringLiteral("robotiq")) && lower.contains(QStringLiteral("base"))) {
    return QStringLiteral("robotiq_85_base_link");
  }
  if (lower.contains(QStringLiteral("gripper_base_link"))) return QStringLiteral("gripper_base_link");
  return link;
}

QString scene3d_canonical_link_name_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  const QString direct = scene3d_link_name_for_item(item);
  QString canonical = scene3d_canonical_link_name(direct);
  const QString id = item.id.trimmed();
  const bool has_normalized_identity =
    !item.visual_index_link_name.trimmed().isEmpty() ||
    !item.visual_index_link.trimmed().isEmpty() ||
    !item.frame_id.trimmed().isEmpty() ||
    (!item.display_name.trimmed().isEmpty() && item.display_name.trimmed() != id);
  if (!canonical.isEmpty() && (has_normalized_identity || canonical != id)) return canonical;
  const QString haystack = scene3d_robot_identity_haystack_for_item(item);
  for (const auto & profile : generated_robot_viewport_profiles()) {
    for (const QString & alias : profile.required_links) {
      if (haystack.contains(alias)) return scene3d_canonical_link_name(alias);
    }
  }
  if (haystack.contains(QStringLiteral("robotiq")) && haystack.contains(QStringLiteral("base"))) {
    return QStringLiteral("robotiq_85_base_link");
  }
  return canonical;
}


bool is_required_generated_robot_viewport_link(const ScenePreviewWidget::PreviewItem & item)
{
  // Resolve required generated robot links through capability profiles derived
  // from PreviewItem identity metadata instead of hardcoding UR5 in the generic
  // render flow.
  return generated_robot_profile_for_required_link_item(item) != nullptr;
}

int scene3d_visual_index_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  QRegularExpression re(QStringLiteral("(?:visual[_-]?|/visual)(\\d+)"));
  const QRegularExpressionMatch match = re.match(item.id);
  if (match.hasMatch()) return match.captured(1).toInt();
  return -1;
}

bool scene3d_final_draw_bbox_for_mesh(const Scene3DViewportWidget::InternalTriangleMesh & mesh,
                                      const QMatrix4x4 & transform,
                                      QVector3D & out_min,
                                      QVector3D & out_max)
{
  bool initialized = false;
  for (const auto & tri : mesh.triangles) {
    for (const auto & vertex : tri.vertices) {
      const QVector3D mapped = transform.map(vertex);
      if (!qIsFinite(mapped.x()) || !qIsFinite(mapped.y()) || !qIsFinite(mapped.z())) continue;
      if (!initialized) {
        out_min = mapped;
        out_max = mapped;
        initialized = true;
      } else {
        out_min.setX(qMin(out_min.x(), mapped.x()));
        out_min.setY(qMin(out_min.y(), mapped.y()));
        out_min.setZ(qMin(out_min.z(), mapped.z()));
        out_max.setX(qMax(out_max.x(), mapped.x()));
        out_max.setY(qMax(out_max.y(), mapped.y()));
        out_max.setZ(qMax(out_max.z(), mapped.z()));
      }
    }
  }
  return initialized;
}
}  // namespace

QJsonArray Scene3DViewportWidget::mesh_diagnostics_export() const
{
  QJsonArray out;
  for (auto it = mesh_cache_.cbegin(); it != mesh_cache_.cend(); ++it) {
    const QString & canonical_path = it.key();
    const MeshCacheEntry & e = it.value();
    QJsonObject row;
    row["canonical_path"] = canonical_path;
    row["loaded"] = e.loaded;
    row["valid"] = e.valid;
    row["warning"] = e.warning;
    row["load_failure_reason"] = e.load_failure_reason;
    row["failure_reason_code"] = e.failure_reason_code;
    row["requested_path"] = e.requested_path;
    row["path_resolved"] = e.path_resolved;
    row["package_uri"] = e.package_uri;
    row["resolved_source_path_stale"] = e.resolved_source_path_stale;
    row["resolved_source_path_original"] = e.resolved_source_path_original;
    row["source_path_resolution_outcome"] = e.source_path_resolution_outcome;
    row["oversized"] = e.oversized;
    const bool known_parser = e.parser_type == "stl" || e.parser_type == "dae" || e.parser_type == "obj" ||
                              e.parser_type.startsWith(QStringLiteral("assimp:"));
    row["parser_type"] = known_parser ? e.parser_type : QStringLiteral("unsupported");
    row["parse_error"] = e.parse_error;
    row["rejected_reason_code"] = e.failure_reason_code.trimmed().isEmpty() ? e.parse_status : e.failure_reason_code;
    row["triangle_count"] = static_cast<int>(e.mesh.triangles.size());
    row["has_bounds"] = e.has_bounds;
    row["local_min"] = QJsonArray{e.local_min.x(), e.local_min.y(), e.local_min.z()};
    row["local_max"] = QJsonArray{e.local_max.x(), e.local_max.y(), e.local_max.z()};
    row["span"] = QJsonArray{e.local_span.x(), e.local_span.y(), e.local_span.z()};
    row["rendering_classification"] = e.visual_surrogate_available ? QStringLiteral("visual_surrogate") : (e.valid ? QStringLiteral("mesh") : QStringLiteral("missing_geometry"));
    row["visual_surrogate_available"] = e.visual_surrogate_available;
    row["visual_surrogate_type"] = e.visual_surrogate_type;
    row["visual_surrogate_reason"] = e.visual_surrogate_reason;
    row["dae_unit_meter"] = e.dae_unit_meter;
    row["dae_has_pre_unit_bounds"] = e.dae_has_pre_unit_bounds;
    row["dae_pre_unit_min"] = QJsonArray{e.dae_pre_unit_min.x(), e.dae_pre_unit_min.y(), e.dae_pre_unit_min.z()};
    row["dae_pre_unit_max"] = QJsonArray{e.dae_pre_unit_max.x(), e.dae_pre_unit_max.y(), e.dae_pre_unit_max.z()};
    row["dae_pre_unit_span"] = QJsonArray{e.dae_pre_unit_span.x(), e.dae_pre_unit_span.y(), e.dae_pre_unit_span.z()};
    row["dae_post_unit_min"] = QJsonArray{e.local_min.x(), e.local_min.y(), e.local_min.z()};
    row["dae_post_unit_max"] = QJsonArray{e.local_max.x(), e.local_max.y(), e.local_max.z()};
    row["dae_post_unit_span"] = QJsonArray{e.local_span.x(), e.local_span.y(), e.local_span.z()};

    QJsonArray guard_details;
    for (const auto & item : items) {
      const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path : item.source_path;
      QString canonical_source;
      if (!try_resolve_canonical_mesh_path(mesh_source, canonical_source, &item)) canonical_source = QFileInfo(mesh_source).absoluteFilePath();
      if (canonical_source != canonical_path) continue;
      QVector3D raw_span, final_span;
      QString reason;
      const bool accepted = validate_mesh_final_span(item, e, mesh_source, reason, &raw_span, &final_span);
      QJsonObject gd;
      gd["item_id"] = item.id;
      gd["accepted"] = accepted;
      gd["reason"] = reason;
      gd["reason_code"] = accepted ? QStringLiteral("ok") : QStringLiteral("unreasonable_bounds_final_span");
      gd["raw_span"] = QJsonArray{raw_span.x(), raw_span.y(), raw_span.z()};
      gd["final_span"] = QJsonArray{final_span.x(), final_span.y(), final_span.z()};
      guard_details.append(gd);
    }
    row["guard_decision_details"] = guard_details;
    out.append(row);
  }
  return out;
}

QJsonArray Scene3DViewportWidget::generated_robot_final_draw_candidate_diagnostics_export() const
{
  QJsonArray out;
  QSet<QString> seen_candidate_keys;
  for (const auto & item : items) {
    const QString joined_identity = QStringList{
      item.id, item.display_name, item.category, item.role, item.source_layer, item.active_visual_source,
      item.mesh_path, item.source_path, item.package_uri, item.visual_index_mesh_uri,
      scene3d_link_name_for_item(item), scene3d_canonical_link_name_for_item(item), item.metadata_tags
    }.join(QStringLiteral("|")).toLower();
    if (!is_required_generated_robot_viewport_link(item) &&
        !joined_identity.contains(QStringLiteral("ur5")) &&
        !joined_identity.contains(QStringLiteral("universal_robot")) &&
        !joined_identity.contains(QStringLiteral("ur_description"))) {
      continue;
    }

    const NormalizedRole role = classify_item_role(item);
    const bool generated_or_locked = is_generated_urdf_visual_item(item) || is_locked_urdf_item(item);
    const bool overlay_helper = !generated_or_locked && (is_overlay_only_item(item) || is_overlay_visual_role(role));
    const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path.trimmed() : item.source_path.trimmed();

    QString first_stage;
    if (!generated_or_locked) {
      first_stage = QStringLiteral("not_generated_locked_urdf_visual");
    } else if (overlay_helper) {
      first_stage = QStringLiteral("classified_helper_overlay");
    } else if (!generated_urdf_item_has_renderable_geometry(item)) {
      first_stage = QStringLiteral("missing_renderable_geometry");
    } else if (!item.has_mesh_metadata && !is_generated_urdf_visual_fallback_item(item) && !item_has_valid_urdf_primitive(item)) {
      first_stage = QStringLiteral("missing_mesh_metadata");
    } else if (mesh_source.isEmpty() && !(is_generated_urdf_visual_fallback_item(item) && is_required_generated_robot_viewport_link(item))) {
      first_stage = QStringLiteral("missing_mesh_source");
    } else {
      const QString duplicate_key = QStringList{
        scene3d_canonical_link_name_for_item(item), mesh_source, item.source_path.trimmed(), item.package_uri.trimmed(),
        QString::number(item.x, 'g', 12), QString::number(item.y, 'g', 12), QString::number(item.z, 'g', 12)
      }.join(QStringLiteral("|"));
      if (seen_candidate_keys.contains(duplicate_key)) {
        first_stage = QStringLiteral("duplicate_suppression");
      } else {
        seen_candidate_keys.insert(duplicate_key);
        QString canonical_mesh_source;
        if (!mesh_source.isEmpty() && !try_resolve_canonical_mesh_path(mesh_source, canonical_mesh_source, &item)) {
          canonical_mesh_source = QFileInfo(mesh_source).absoluteFilePath();
        }
        const auto cache_it = mesh_cache_.constFind(canonical_mesh_source);
        if (!mesh_source.isEmpty() && cache_it == mesh_cache_.constEnd()) {
          first_stage = QStringLiteral("missing_mesh_cache");
        } else if (!mesh_source.isEmpty()) {
          const MeshCacheEntry & cache = cache_it.value();
          if (!cache.loaded || !cache.valid || !cache.has_bounds || cache.mesh.triangles.isEmpty()) {
            first_stage = QStringLiteral("invalid_mesh_bounds");
          }
        }
      }
    }
    if (first_stage.isEmpty()) first_stage = QStringLiteral("accepted");

    QJsonObject row;
    row["id"] = item.id;
    if (item.source_row_index >= 0) row["source_row_index"] = item.source_row_index;
    row["link"] = !item.visual_index_link.trimmed().isEmpty() ? item.visual_index_link.trimmed() : scene3d_link_name_for_item(item);
    row["link_name"] = scene3d_link_name_for_item(item);
    row["canonical_link_name"] = scene3d_canonical_link_name_for_item(item);
    row["source_layer"] = item.source_layer;
    row["active_visual_source"] = item.active_visual_source;
    row["role"] = item.role;
    row["category"] = item.category;
    row["mesh_path"] = item.mesh_path;
    row["source_path"] = item.source_path;
    row["package_uri"] = !item.visual_index_package_uri.trimmed().isEmpty() ? item.visual_index_package_uri.trimmed() : item.package_uri;
    row["first_rejection_stage"] = first_stage;
    row["first_drop_stage"] = first_stage;
    out.append(row);
  }
  return out;
}


QJsonArray Scene3DViewportWidget::final_draw_visual_items_export() const
{
  QJsonArray out;
  const std::vector<const ScenePreviewWidget::PreviewItem *> final_renderables =
    build_final_generated_urdf_robot_renderables(items, show_safety);
  for (const auto * item_ptr : final_renderables) {
    if (!item_ptr) continue;
    const ScenePreviewWidget::PreviewItem & item = *item_ptr;
    if (!is_generated_urdf_visual_item(item) && !is_locked_urdf_item(item)) continue;
    if (!generated_urdf_item_has_renderable_geometry(item)) continue;
    if (!item.has_mesh_metadata && !is_generated_urdf_visual_fallback_item(item) && !item_has_valid_urdf_primitive(item)) continue;

    const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path : item.source_path;
    const bool required_ur5_fallback = is_generated_urdf_visual_fallback_item(item) && is_required_generated_robot_viewport_link(item);
    if (mesh_source.trimmed().isEmpty() && !required_ur5_fallback) continue;

    QString canonical_mesh_source;
    QString resolve_failure_reason;
    const bool path_resolved = !mesh_source.trimmed().isEmpty() &&
      try_resolve_canonical_mesh_path(mesh_source, canonical_mesh_source, &item, &resolve_failure_reason);
    if (!path_resolved && !mesh_source.trimmed().isEmpty()) canonical_mesh_source = QFileInfo(mesh_source).absoluteFilePath();

    QJsonObject row;
    row["item_id"] = item.id;
    row["id"] = item.id;
    row["display_name"] = item.display_name;
    const QString link_name = scene3d_link_name_for_item(item);
    const QString canonical_link_name = scene3d_canonical_link_name_for_item(item);
    row["link"] = !item.visual_index_link.trimmed().isEmpty() ? item.visual_index_link.trimmed() : link_name;
    row["link_name"] = link_name;
    row["canonical_link"] = canonical_link_name;
    row["canonical_link_name"] = canonical_link_name;
    row["visual_index_link"] = !item.visual_index_link.trimmed().isEmpty() ? item.visual_index_link.trimmed() : link_name;
    if (!item.visual_index_object_name.trimmed().isEmpty()) row["object_name"] = item.visual_index_object_name.trimmed();
    const int visual_index = item.visual_index_value >= 0 ? item.visual_index_value : scene3d_visual_index_for_item(item);
    if (visual_index >= 0) row["visual_index"] = visual_index;
    row["visual_name"] = !item.visual_index_visual_name.trimmed().isEmpty() ? item.visual_index_visual_name.trimmed() :
      (!item.visual_index_visual.trimmed().isEmpty() ? item.visual_index_visual.trimmed() :
        (visual_index >= 0 ? QStringLiteral("visual_%1").arg(visual_index) : item.display_name));
    if (!item.visual_index_visual.trimmed().isEmpty()) row["visual"] = item.visual_index_visual.trimmed();
    if (item.source_row_index >= 0) row["source_row_index"] = item.source_row_index;
    if (!item.visual_index_parent_link.trimmed().isEmpty()) row["parent_link"] = item.visual_index_parent_link.trimmed();
    if (!item.visual_index_link_chain.isEmpty()) {
      QJsonArray chain;
      for (const QString & link : item.visual_index_link_chain) chain.append(link);
      row["link_chain"] = chain;
    }
    if (!item.visual_index_source.trimmed().isEmpty()) row["source"] = item.visual_index_source.trimmed();
    row["frame_id"] = !item.frame_id.trimmed().isEmpty() ? item.frame_id.trimmed() : canonical_link_name;
    row["source_layer"] = item.source_layer;
    row["category"] = item.category;
    row["role"] = item.role;
    if (!item.metadata_tags.trimmed().isEmpty()) {
      row["metadata_tags"] = item.metadata_tags;
      const QStringList tags = item.metadata_tags.split(QLatin1Char(';'), Qt::SkipEmptyParts);
      for (const QString & raw_tag : tags) {
        const QString tag = raw_tag.trimmed();
        const int equals_index = tag.indexOf(QLatin1Char('='));
        if (equals_index <= 0) continue;
        const QString key = tag.left(equals_index).trimmed();
        const QString value = tag.mid(equals_index + 1).trimmed();
        if ((key == QStringLiteral("robot_family") || key == QStringLiteral("asset_category")) && !value.isEmpty()) {
          row[key] = value;
        }
      }
    }
    row["active_visual_source"] = item.active_visual_source;
    if (canonical_link_name == QStringLiteral("base_link_inertia")) {
      qInfo().noquote() << QStringLiteral("Scene3D base_link_inertia trace: stage=final_draw_visual_items item_id=%1 link=%2 canonical_link=%3 mesh_source=%4 source_layer=%5 active_visual_source=%6")
        .arg(item.id, link_name, canonical_link_name, mesh_source, item.source_layer, item.active_visual_source);
    }
    row["locked"] = item.locked;
    row["editable"] = item.editable;
    row["lock_reason"] = item.lock_reason;
    row["mesh_source"] = mesh_source;
    row["mesh_uri"] = !item.visual_index_mesh_uri.trimmed().isEmpty() ? item.visual_index_mesh_uri.trimmed() : mesh_source;
    row["package_uri"] = !item.visual_index_package_uri.trimmed().isEmpty() ? item.visual_index_package_uri.trimmed() : item.package_uri;
    row["source_type"] = QStringLiteral("generated_urdf_visual_mesh");
    row["rviz_parity_robot_layer"] = is_rviz_parity_robot_layer_item(item);
    row["mesh_path"] = item.mesh_path;
    row["source_path"] = item.source_path;
    row["mesh_source_field"] = !item.mesh_path.trimmed().isEmpty() ? QStringLiteral("mesh_path") : QStringLiteral("source_path");
    row["canonical_mesh_source"] = canonical_mesh_source;
    row["path_resolved"] = path_resolved;
    row["resolve_failure_reason"] = resolve_failure_reason;

    const QMatrix4x4 baked_transform = authoritative_world_visual_transform(item);
    const QMatrix4x4 viewport_root_transform = viewport_world_visual_transform(item);
    const QMatrix4x4 final_transform = final_mesh_transform_matrix(item);
    row["baked_world_visual_matrix"] = scene3d_matrix_to_json(baked_transform);
    row["ros_to_viewport_basis_matrix"] = scene3d_matrix_to_json(ros_to_viewport_basis_matrix());
    row["viewport_world_visual_matrix"] = scene3d_matrix_to_json(viewport_root_transform);
    row["final_draw_model_matrix"] = scene3d_matrix_to_json(final_transform);
    row["final_draw_world_pose"] = scene3d_vec_to_json(final_transform.map(QVector3D(0.0f, 0.0f, 0.0f)));
    if (required_ur5_fallback) {
      row["source_type"] = QStringLiteral("generated_urdf_visual_fallback");
      row["final_draw_status"] = QStringLiteral("ur5_emergency_visible_fallback");
      row["fallback_visible_primitive"] = true;
      row["path_resolved"] = true;
      row["resolve_failure_reason"] = QStringLiteral("mesh_submission_failed_visible_fallback_submitted");
      row["final_draw_bbox_span"] = scene3d_vec_to_json(QVector3D(
        static_cast<float>(qMax(0.08, item.sx)),
        static_cast<float>(qMax(0.08, item.sy)),
        static_cast<float>(qMax(0.08, item.sz))));
      out.append(row);
      continue;
    }
    if (is_required_generated_robot_viewport_link(item)) {
      const QString required_link = scene3d_canonical_link_name_for_item(item);
      const GeneratedRobotViewportProfile * profile = generated_robot_profile_for_required_link_item(item);
      const QString expected_mesh = profile ? profile->expected_visual_meshes.value(required_link) : QString();
      const QString expected_uri = (profile && !expected_mesh.trimmed().isEmpty())
        ? QStringLiteral("package://ur_description/meshes/%1/visual/%2").arg(profile->key, expected_mesh)
        : QString();
      const QString exported_mesh_uri = row.value(QStringLiteral("mesh_uri")).toString().trimmed();
      const QString exported_mesh_source = row.value(QStringLiteral("mesh_source")).toString().trimmed();
      const bool expected_uri_matches = !expected_uri.isEmpty() &&
        (exported_mesh_uri == expected_uri || exported_mesh_source == expected_uri);
      const bool canonical_matches_expected_asset = !expected_mesh.isEmpty() &&
        (canonical_mesh_source.endsWith(QStringLiteral("/assets/robots/universal_robot/ur_description/meshes/%1/visual/%2").arg(profile ? profile->key : QString(), expected_mesh)) ||
         canonical_mesh_source.endsWith(QStringLiteral("/ur_description/meshes/%1/visual/%2").arg(profile ? profile->key : QString(), expected_mesh)));
      row["expected_ur5_mesh_uri"] = expected_uri;
      row["ur5_final_viewport_uri_matches_expected"] = expected_uri_matches;
      row["ur5_final_viewport_canonical_asset_matches_expected"] = canonical_matches_expected_asset;
      row["ur5_final_viewport_path_resolution_required"] = true;
      row["ur5_final_viewport_contract_ok"] =
        expected_uri_matches && path_resolved && canonical_matches_expected_asset;
      if (!expected_uri_matches || !path_resolved || !canonical_matches_expected_asset) {
        row["ur5_final_viewport_blocker"] = QStringLiteral("ur5_final_viewport_links_missing");
      }
    }
    row["has_mesh_metadata"] = item.has_mesh_metadata;
    row["has_baked_world_visual_transform"] = item.has_baked_world_visual_transform;
    row["has_baked_world_visual_matrix"] = item.has_baked_world_visual_matrix;
    row["baked_world_visual_pose"] = scene3d_pose_to_json(item.x, item.y, item.z, item.roll, item.pitch, item.yaw);
    row["visual_origin_applied"] = item.visual_origin_applied;
    row["visual_origin_pose"] = scene3d_pose_to_json(item.visual_origin_x, item.visual_origin_y, item.visual_origin_z,
                                                      item.visual_origin_roll, item.visual_origin_pitch, item.visual_origin_yaw);
    row["local_mesh_correction_rpy"] = QJsonArray{item.mesh_r, item.mesh_p, item.mesh_y};
    row["origin_offset"] = QJsonArray{item.origin_offset_x, item.origin_offset_y, item.origin_offset_z};
    row["has_origin_offset"] = item.has_origin_offset;
    row["mesh_scale"] = QJsonArray{item.mesh_scale_x, item.mesh_scale_y, item.mesh_scale_z};
    const bool baked_mesh_asset_correction_candidate = item.has_baked_world_visual_transform &&
      item_references_ur5_baked_mesh_asset_local_correction(item);
    const bool baked_mesh_asset_correction_has_values = item_has_non_identity_mesh_asset_local_correction(item);
    const bool baked_mesh_asset_correction_applied = should_apply_baked_mesh_asset_local_correction(item);
    row["baked_mesh_asset_local_correction_candidate"] = baked_mesh_asset_correction_candidate;
    row["baked_mesh_asset_local_correction_has_values"] = baked_mesh_asset_correction_has_values;
    row["baked_mesh_asset_local_correction_applied"] = baked_mesh_asset_correction_applied;
    row["baked_mesh_asset_local_correction_scope"] = baked_mesh_asset_correction_candidate
      ? ur5_baked_mesh_asset_local_correction_scope_reason(item)
      : QStringLiteral("none");
    row["resolved_source_path"] = item.resolved_source_path_original;

    const QString asset_local_correction_reason = baked_mesh_asset_local_correction_reason(item);
    const QMatrix4x4 asset_local_correction_transform = baked_mesh_asset_local_correction_matrix(item);
    row["baked_mesh_asset_local_correction_reason"] = asset_local_correction_reason;
    row["baked_mesh_asset_local_correction_matrix"] = scene3d_matrix_to_json(asset_local_correction_transform);
    row["final_draw_asset_local_correction_applied"] = baked_mesh_asset_correction_applied;
    row["final_draw_asset_local_correction_reason"] = asset_local_correction_reason;
    row["final_draw_asset_local_correction_matrix"] = scene3d_matrix_to_json(asset_local_correction_transform);
    row["final_draw_asset_local_correction_rpy"] = QJsonArray{item.mesh_r, item.mesh_p, item.mesh_y};
    row["final_draw_asset_local_correction_xyz"] = QJsonArray{item.origin_offset_x, item.origin_offset_y, item.origin_offset_z};
    row["final_draw_asset_local_correction_has_origin_offset"] = item.has_origin_offset;

    const auto cache_it = mesh_cache_.constFind(canonical_mesh_source);
    if (cache_it == mesh_cache_.constEnd()) {
      row["final_draw_status"] = is_required_generated_robot_viewport_link(item)
        ? QStringLiteral("ur5_emergency_visible_fallback")
        : QStringLiteral("missing_mesh_cache");
      out.append(row);
      continue;
    }

    const MeshCacheEntry & cache = cache_it.value();
    row["cache_loaded"] = cache.loaded;
    row["cache_valid"] = cache.valid;
    row["cache_has_bounds"] = cache.has_bounds;
    row["cache_warning"] = cache.warning;
    row["cache_failure_reason_code"] = cache.failure_reason_code;
    row["triangle_count"] = static_cast<int>(cache.mesh.triangles.size());
    row["local_min"] = scene3d_vec_to_json(cache.local_min);
    row["local_max"] = scene3d_vec_to_json(cache.local_max);

    if (!cache.loaded || !cache.valid || !cache.has_bounds || cache.mesh.triangles.isEmpty()) {
      row["final_draw_status"] = is_required_generated_robot_viewport_link(item)
        ? QStringLiteral("ur5_emergency_visible_fallback")
        : (cache.has_bounds ? QStringLiteral("invalid_mesh") : QStringLiteral("missing_bounds"));
      if (is_required_generated_robot_viewport_link(item) && cache.has_bounds) {
        QVector3D final_min;
        QVector3D final_max;
        if (scene3d_final_draw_bbox_for_mesh(cache.mesh, final_transform, final_min, final_max)) {
          const QVector3D final_span = final_max - final_min;
          row["final_draw_bbox"] = scene3d_bbox_to_json(final_min, final_max);
          row["final_draw_bbox_min"] = scene3d_vec_to_json(final_min);
          row["final_draw_bbox_max"] = scene3d_vec_to_json(final_max);
          row["final_draw_bbox_span"] = scene3d_vec_to_json(final_span);
          row["final_draw_bbox_center"] = scene3d_vec_to_json((final_min + final_max) * 0.5f);
          row["final_rendered_mesh_bbox"] = scene3d_bbox_to_json(final_min, final_max);
          row["final_rendered_mesh_bbox_min"] = scene3d_vec_to_json(final_min);
          row["final_rendered_mesh_bbox_max"] = scene3d_vec_to_json(final_max);
          row["final_rendered_mesh_bbox_center"] = scene3d_vec_to_json((final_min + final_max) * 0.5f);
        }
      }
      out.append(row);
      continue;
    }

    QVector3D final_min;
    QVector3D final_max;
    if (!scene3d_final_draw_bbox_for_mesh(cache.mesh, final_transform, final_min, final_max)) {
      row["final_draw_status"] = QStringLiteral("non_finite_bounds");
      out.append(row);
      continue;
    }
    const QVector3D final_span = final_max - final_min;
    row["final_draw_bbox"] = scene3d_bbox_to_json(final_min, final_max);
    row["final_draw_bbox_min"] = scene3d_vec_to_json(final_min);
    row["final_draw_bbox_max"] = scene3d_vec_to_json(final_max);
    row["final_draw_bbox_span"] = scene3d_vec_to_json(final_span);
    row["final_draw_bbox_center"] = scene3d_vec_to_json((final_min + final_max) * 0.5f);
    row["final_rendered_mesh_bbox"] = scene3d_bbox_to_json(final_min, final_max);
    row["final_rendered_mesh_bbox_min"] = scene3d_vec_to_json(final_min);
    row["final_rendered_mesh_bbox_max"] = scene3d_vec_to_json(final_max);
    row["final_rendered_mesh_bbox_center"] = scene3d_vec_to_json((final_min + final_max) * 0.5f);
    row["final_draw_status"] = QStringLiteral("ok");
    out.append(row);

    if (scene3d_debug_logs_enabled()) {
      qInfo().noquote() << QStringLiteral(
        "Scene3D final draw: item_id=%1 link=%2 mesh=%3 baked_pose=[%4,%5,%6,%7,%8,%9] scale=[%10,%11,%12] final_matrix=%13 bbox_min=[%14,%15,%16] bbox_max=[%17,%18,%19]")
        .arg(item.id, scene3d_link_name_for_item(item), mesh_source)
        .arg(item.x, 0, 'g', 8).arg(item.y, 0, 'g', 8).arg(item.z, 0, 'g', 8)
        .arg(item.roll, 0, 'g', 8).arg(item.pitch, 0, 'g', 8).arg(item.yaw, 0, 'g', 8)
        .arg(item.mesh_scale_x, 0, 'g', 8).arg(item.mesh_scale_y, 0, 'g', 8).arg(item.mesh_scale_z, 0, 'g', 8)
        .arg(QString::fromUtf8(QJsonDocument(scene3d_matrix_to_json(final_transform)).toJson(QJsonDocument::Compact)))
        .arg(final_min.x(), 0, 'g', 8).arg(final_min.y(), 0, 'g', 8).arg(final_min.z(), 0, 'g', 8)
        .arg(final_max.x(), 0, 'g', 8).arg(final_max.y(), 0, 'g', 8).arg(final_max.z(), 0, 'g', 8);
    }
  }
  return out;
}

QJsonArray Scene3DViewportWidget::final_draw_diagnostics_export() const
{
  return final_draw_visual_items_export();
}

