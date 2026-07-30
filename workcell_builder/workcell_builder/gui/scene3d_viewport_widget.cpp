#include "scene3d_viewport_widget.h"
#include "scene3d_visual_classification.h"
// Static contract token: return !item_has_mesh_uri_or_path(it) && !item_has_valid_urdf_primitive(it);
// Static contract token: REJECT_RAW_GENERATED_BOUNDS_SUPPRESSED: generated bounds item has no mesh URI/path and no URDF primitive
// Static contract token: generated_bounds_suppressed
// Static contract token: REJECT_MESH_PARSE_FAILED
// Static contract token: REJECT_MESH_BOUNDS_FAILED
// Static contract token: semantic_mesh_fallback
// Static contract token: if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);
// Compatibility token for static tests: Overlays %1 Items %1 • Mesh %2 • Boxes %3 • Missing %4
// Compatibility token for smoke counter gate tests: const bool physical_mesh_source = !overlay_helper && item_has_credible_mesh_handoff(it);
// Compatibility token for extracted diagnostics: row["load_failure_reason"]

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QCursor>
#include <QPainter>
#include <QSurfaceFormat>
#include <QVector3D>
#include <QVector4D>
#include <QToolTip>
#include <QFile>
#include <QFileInfo>
#include <QDataStream>
#include <QDebug>
#include <QTextStream>
#include <QtEndian>
#include <QMimeData>
#include <QJsonDocument>
#include <QJsonArray>
#include <QHash>
#include <QXmlStreamReader>
#include <QImage>
#include <QRegularExpression>
#include <QStringList>
#include <cstring>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <initializer_list>

#ifdef WORKCELL_BUILDER_HAS_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

namespace {
constexpr int kMeshTriangleLimit = 1000000;
constexpr double kWorkspaceLimitMeters = 1000.0;
constexpr const char * kWorkcellStudioAssetMime = "application/x-workcell-studio-asset";

[[maybe_unused]] QString snap_mode_label(Scene3DViewportWidget::SnapMode mode)
{
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Off: return "Off";
    case Scene3DViewportWidget::SnapMode::Cm1: return "1 cm";
    case Scene3DViewportWidget::SnapMode::Cm5: return "5 cm";
    case Scene3DViewportWidget::SnapMode::Cm10: return "10 cm";
    case Scene3DViewportWidget::SnapMode::Deg5: return "5 deg";
    case Scene3DViewportWidget::SnapMode::Deg15: return "15 deg";
  }
  return "Off";
}

double snap_translation_value(double raw_m, Scene3DViewportWidget::SnapMode mode)
{
  double step_m = 0.0;
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Cm1: step_m = 0.01; break;
    case Scene3DViewportWidget::SnapMode::Cm5: step_m = 0.05; break;
    case Scene3DViewportWidget::SnapMode::Cm10: step_m = 0.10; break;
    case Scene3DViewportWidget::SnapMode::Off:
    case Scene3DViewportWidget::SnapMode::Deg5:
    case Scene3DViewportWidget::SnapMode::Deg15:
      break;
  }
  return step_m > 0.0 ? std::round(raw_m / step_m) * step_m : raw_m;
}

double snap_rotation_value(double raw_rad, Scene3DViewportWidget::SnapMode mode)
{
  double step_rad = 0.0;
  switch (mode) {
    case Scene3DViewportWidget::SnapMode::Deg5: step_rad = qDegreesToRadians(5.0); break;
    case Scene3DViewportWidget::SnapMode::Deg15: step_rad = qDegreesToRadians(15.0); break;
    case Scene3DViewportWidget::SnapMode::Off:
    case Scene3DViewportWidget::SnapMode::Cm1:
    case Scene3DViewportWidget::SnapMode::Cm5:
    case Scene3DViewportWidget::SnapMode::Cm10:
      break;
  }
  return step_rad > 0.0 ? std::round(raw_rad / step_rad) * step_rad : raw_rad;
}

QString path_without_uri_suffixes(QString path)
{
  path = path.trimmed();
  const int fragment_index = path.indexOf(QLatin1Char('#'));
  if (fragment_index >= 0) path = path.left(fragment_index);
  const int query_index = path.indexOf(QLatin1Char('?'));
  if (query_index >= 0) path = path.left(query_index);
  return path.trimmed();
}

bool path_has_mesh_asset_extension(const QString & path)
{
  const QString suffix = QFileInfo(path_without_uri_suffixes(path)).suffix().toLower();
  return suffix == QStringLiteral("dae") ||
         suffix == QStringLiteral("stl") ||
         suffix == QStringLiteral("obj") ||
         suffix == QStringLiteral("glb") ||
         suffix == QStringLiteral("gltf");
}

bool scene3d_env_flag_enabled(const char * name)
{
  const QByteArray value = qgetenv(name).trimmed().toLower();
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

bool scene3d_debug_logs_enabled()
{
  return scene3d_env_flag_enabled("WORKCELL_SCENE3D_DEBUG_LOGS");
}

bool scene3d_debug_fallback_boxes_enabled()
{
  return scene3d_env_flag_enabled("WORKCELL_SCENE3D_DEBUG_FALLBACK_BOXES");
}


[[maybe_unused]] void apply_urdf_rpy_gl(double roll, double pitch, double yaw)
{
  // URDF RPY is fixed-axis roll/pitch/yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  // OpenGL post-multiplies the current matrix, so issue rotations in Z/Y/X order.
  glRotated(qRadiansToDegrees(yaw), 0.0, 0.0, 1.0);
  glRotated(qRadiansToDegrees(pitch), 0.0, 1.0, 0.0);
  glRotated(qRadiansToDegrees(roll), 1.0, 0.0, 0.0);
}

void apply_urdf_rpy_matrix(QMatrix4x4 & transform, double roll, double pitch, double yaw)
{
  // Keep Scene3D's CPU bounds path in the same URDF/RViz transform convention as rendering.
  transform.rotate(static_cast<float>(qRadiansToDegrees(yaw)), 0.0f, 0.0f, 1.0f);
  transform.rotate(static_cast<float>(qRadiansToDegrees(pitch)), 0.0f, 1.0f, 0.0f);
  transform.rotate(static_cast<float>(qRadiansToDegrees(roll)), 1.0f, 0.0f, 0.0f);
}

void apply_urdf_pose_matrix(QMatrix4x4 & transform, double x, double y, double z, double roll, double pitch, double yaw)
{
  transform.translate(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  apply_urdf_rpy_matrix(transform, roll, pitch, yaw);
}

QMatrix4x4 ros_to_viewport_basis_matrix()
{
  // Central ROS/RViz (X forward, Y left, Z up) to Scene3D/OpenGL viewport
  // basis (X right, Y up, Z depth) conversion.  All generated visual preview
  // model matrices enter this basis exactly once at the transform root.
  QMatrix4x4 basis;
  basis.setToIdentity();
  basis(0, 0) = 1.0f; basis(0, 1) = 0.0f; basis(0, 2) = 0.0f;
  basis(1, 0) = 0.0f; basis(1, 1) = 0.0f; basis(1, 2) = 1.0f;
  basis(2, 0) = 0.0f; basis(2, 1) = -1.0f; basis(2, 2) = 0.0f;
  return basis;
}

QMatrix4x4 authoritative_world_visual_transform(const ScenePreviewWidget::PreviewItem & item)
{
  QMatrix4x4 transform;
  if (item.has_baked_world_visual_transform) {
    if (item.has_baked_world_visual_matrix) {
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          transform(row, col) = static_cast<float>(item.baked_world_visual_matrix(row, col));
        }
      }
    } else {
      apply_urdf_pose_matrix(transform, item.x, item.y, item.z, item.roll, item.pitch, item.yaw);
    }
    return transform;
  }
  apply_urdf_pose_matrix(transform, item.x, item.y, item.z, item.roll, item.pitch, item.yaw);
  if (item.visual_origin_applied) {
    apply_urdf_pose_matrix(transform, item.visual_origin_x, item.visual_origin_y, item.visual_origin_z,
                           item.visual_origin_roll, item.visual_origin_pitch, item.visual_origin_yaw);
  }
  return transform;
}

QMatrix4x4 viewport_world_visual_transform(const ScenePreviewWidget::PreviewItem & item)
{
  return ros_to_viewport_basis_matrix() * authoritative_world_visual_transform(item);
}

void apply_authoritative_world_visual_transform_gl(const ScenePreviewWidget::PreviewItem & item)
{
  const QMatrix4x4 transform = viewport_world_visual_transform(item);
  glMultMatrixf(transform.constData());
}

QString normalize_mesh_scope_candidate(QString candidate)
{
  candidate = path_without_uri_suffixes(candidate).trimmed().toLower();
  candidate.replace(QLatin1Char('\\'), QLatin1Char('/'));
  while (candidate.contains(QStringLiteral("//")) && !candidate.startsWith(QStringLiteral("package://"))) {
    candidate.replace(QStringLiteral("//"), QStringLiteral("/"));
  }
  return candidate;
}

bool is_ur5_baked_mesh_asset_local_correction_path(const QString & candidate, QString * matched_reason = nullptr)
{
  const QString normalized = normalize_mesh_scope_candidate(candidate);
  if (normalized.isEmpty()) return false;

  const QStringList arm_meshes = {
    QStringLiteral("upperarm.dae"),
    QStringLiteral("forearm.dae"),
    QStringLiteral("wrist1.dae"),
  };
  bool matches_known_mesh = false;
  QString matched_mesh;
  for (const QString & mesh : arm_meshes) {
    if (normalized.endsWith(QStringLiteral("/") + mesh) || normalized == mesh) {
      matches_known_mesh = true;
      matched_mesh = mesh;
      break;
    }
  }
  if (!matches_known_mesh) return false;

  if (normalized.startsWith(QStringLiteral("package://ur_description/meshes/ur5/visual/"))) {
    if (matched_reason) *matched_reason = QStringLiteral("matched_package_uri_ur_description_ur5_visual_%1").arg(matched_mesh);
    return true;
  }
  if (normalized.contains(QStringLiteral("/ur_description/meshes/ur5/visual/")) &&
      (normalized.contains(QStringLiteral("/assets/robots/universal_robot/")) ||
       normalized.contains(QStringLiteral("/src/assets/robots/universal_robot/")))) {
    if (matched_reason) *matched_reason = QStringLiteral("matched_resolved_universal_robot_ur5_visual_%1").arg(matched_mesh);
    return true;
  }
  return false;
}

QString ur5_baked_mesh_asset_local_correction_scope_reason(const ScenePreviewWidget::PreviewItem & item)
{
  QString reason;
  if (is_ur5_baked_mesh_asset_local_correction_path(item.package_uri, &reason)) return reason;
  if (is_ur5_baked_mesh_asset_local_correction_path(item.mesh_path, &reason)) return reason;
  if (is_ur5_baked_mesh_asset_local_correction_path(item.source_path, &reason)) return reason;
  if (is_ur5_baked_mesh_asset_local_correction_path(item.resolved_source_path_original, &reason)) return reason;
  return QString();
}

bool item_references_ur5_baked_mesh_asset_local_correction(const ScenePreviewWidget::PreviewItem & item)
{
  return !ur5_baked_mesh_asset_local_correction_scope_reason(item).isEmpty();
}

bool item_is_urdf_flattened_generated_preview(const ScenePreviewWidget::PreviewItem & item)
{
  return item.metadata_tags.contains(QStringLiteral("source=urdf_flattened"), Qt::CaseInsensitive);
}

bool item_has_non_identity_mesh_asset_local_correction(const ScenePreviewWidget::PreviewItem & item)
{
  constexpr double kEpsilon = 1e-12;
  const bool has_rpy = std::abs(item.mesh_r) > kEpsilon ||
                       std::abs(item.mesh_p) > kEpsilon ||
                       std::abs(item.mesh_y) > kEpsilon;
  const bool has_offset = item.has_origin_offset &&
                          (std::abs(item.origin_offset_x) > kEpsilon ||
                           std::abs(item.origin_offset_y) > kEpsilon ||
                           std::abs(item.origin_offset_z) > kEpsilon);
  return has_rpy || has_offset;
}

bool should_apply_baked_mesh_asset_local_correction(const ScenePreviewWidget::PreviewItem & item)
{
  // Generated visual mesh entries are already RViz/URDF-authored as
  // world_T_link_or_object * visual_origin_T. Most baked rows therefore must not
  // re-apply legacy mesh RPY/origin fields.  The exception is the narrow UR5 DAE
  // visual asset set where mesh_r/p/y and origin_offset describe asset-local
  // corrections needed by Scene3D after the authoritative world_T_visual matrix.
  //
  // In particular, source=urdf_flattened entries from scene_visual_mesh_index.json
  // are raw ROS/RViz world-space poses. Scene3D applies only the centralized
  // ROS-to-viewport basis at viewport_world_visual_transform(); applying the
  // legacy UR5 mesh correction here would double-correct those locked previews.
  if (item_is_urdf_flattened_generated_preview(item)) return false;
  if (!item.has_baked_world_visual_transform) return false;
  if (!item_references_ur5_baked_mesh_asset_local_correction(item)) return false;
  return item_has_non_identity_mesh_asset_local_correction(item);
}

QString baked_mesh_asset_local_correction_reason(const ScenePreviewWidget::PreviewItem & item)
{
  if (should_apply_baked_mesh_asset_local_correction(item)) {
    const QString scope_reason = ur5_baked_mesh_asset_local_correction_scope_reason(item);
    return scope_reason.isEmpty() ? QStringLiteral("known_ur5_mesh_asset_local_correction") : scope_reason;
  }
  if (!item.has_baked_world_visual_transform) {
    return QStringLiteral("not_baked_world_visual");
  }
  if (!item_references_ur5_baked_mesh_asset_local_correction(item)) {
    return QStringLiteral("mesh_not_in_initial_ur5_asset_scope");
  }
  return QStringLiteral("identity_mesh_asset_correction");
}

QMatrix4x4 baked_mesh_asset_local_correction_matrix(const ScenePreviewWidget::PreviewItem & item)
{
  QMatrix4x4 correction;
  if (!should_apply_baked_mesh_asset_local_correction(item)) return correction;

  apply_urdf_rpy_matrix(correction, item.mesh_r, item.mesh_p, item.mesh_y);
  if (item.has_origin_offset) {
    correction.translate(static_cast<float>(item.origin_offset_x),
                         static_cast<float>(item.origin_offset_y),
                         static_cast<float>(item.origin_offset_z));
  }
  return correction;
}

void apply_baked_mesh_asset_local_correction_matrix(QMatrix4x4 & transform, const ScenePreviewWidget::PreviewItem & item)
{
  // Baked generated URDF visuals already include the URDF visual origin in
  // authoritative_world_visual_transform(). Only re-apply legacy mesh_r/p/y and
  // origin_offset when the asset is in the narrow, known UR5 visual-mesh set where
  // those values describe mesh-asset-local DAE corrections rather than old URDF
  // visual-origin transforms.
  if (!should_apply_baked_mesh_asset_local_correction(item)) return;

  transform *= baked_mesh_asset_local_correction_matrix(item);
}

void apply_mesh_local_correction_matrix(QMatrix4x4 & transform, const ScenePreviewWidget::PreviewItem & item)
{
  // Baked generated/locked URDF visuals already include the full URDF visual origin
  // in authoritative_world_visual_transform(). Do not re-apply legacy mesh RPY or
  // origin-offset fields globally; only the explicit baked mesh-asset correction
  // path above may opt in a known asset-local correction.
  if (item.has_baked_world_visual_transform) {
    apply_baked_mesh_asset_local_correction_matrix(transform, item);
    return;
  }

  apply_urdf_rpy_matrix(transform, item.mesh_r, item.mesh_p, item.mesh_y);
  if (item.has_origin_offset) {
    transform.translate(static_cast<float>(item.origin_offset_x),
                        static_cast<float>(item.origin_offset_y),
                        static_cast<float>(item.origin_offset_z));
  }
}

bool is_generated_urdf_visual_item(const ScenePreviewWidget::PreviewItem & it);
bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it);
bool is_required_generated_robot_viewport_link(const ScenePreviewWidget::PreviewItem & item);
QString scene3d_canonical_link_name_for_item(const ScenePreviewWidget::PreviewItem & item);
struct GeneratedRobotViewportProfile
{
  QString key;
  QStringList identity_tokens;
  QStringList required_links;
  QHash<QString, QString> expected_visual_meshes;
};

const QVector<GeneratedRobotViewportProfile> & generated_robot_viewport_profiles()
{
  static const QVector<GeneratedRobotViewportProfile> profiles{
    {QStringLiteral("ur5"),
     {QStringLiteral("ur5"), QStringLiteral("ur_description/meshes/ur5"), QStringLiteral("universal_robot")},
     {QStringLiteral("base_link_inertia"), QStringLiteral("shoulder_link"), QStringLiteral("upper_arm_link"), QStringLiteral("forearm_link"), QStringLiteral("wrist_1_link"), QStringLiteral("wrist_2_link"), QStringLiteral("wrist_3_link")},
     {{QStringLiteral("base_link_inertia"), QStringLiteral("base.dae")}, {QStringLiteral("shoulder_link"), QStringLiteral("shoulder.dae")}, {QStringLiteral("upper_arm_link"), QStringLiteral("upperarm.dae")}, {QStringLiteral("forearm_link"), QStringLiteral("forearm.dae")}, {QStringLiteral("wrist_1_link"), QStringLiteral("wrist1.dae")}, {QStringLiteral("wrist_2_link"), QStringLiteral("wrist2.dae")}, {QStringLiteral("wrist_3_link"), QStringLiteral("wrist3.dae")}}},
    {QStringLiteral("ur10"),
     {QStringLiteral("ur10"), QStringLiteral("ur_description/meshes/ur10")},
     {QStringLiteral("base_link_inertia"), QStringLiteral("shoulder_link"), QStringLiteral("upper_arm_link"), QStringLiteral("forearm_link"), QStringLiteral("wrist_1_link"), QStringLiteral("wrist_2_link"), QStringLiteral("wrist_3_link")},
     {{QStringLiteral("base_link_inertia"), QStringLiteral("base.dae")}, {QStringLiteral("shoulder_link"), QStringLiteral("shoulder.dae")}, {QStringLiteral("upper_arm_link"), QStringLiteral("upperarm.dae")}, {QStringLiteral("forearm_link"), QStringLiteral("forearm.dae")}, {QStringLiteral("wrist_1_link"), QStringLiteral("wrist1.dae")}, {QStringLiteral("wrist_2_link"), QStringLiteral("wrist2.dae")}, {QStringLiteral("wrist_3_link"), QStringLiteral("wrist3.dae")}}},
    {QStringLiteral("ur3"),
     {QStringLiteral("ur3"), QStringLiteral("ur_description/meshes/ur3")},
     {QStringLiteral("base_link_inertia"), QStringLiteral("shoulder_link"), QStringLiteral("upper_arm_link"), QStringLiteral("forearm_link"), QStringLiteral("wrist_1_link"), QStringLiteral("wrist_2_link"), QStringLiteral("wrist_3_link")},
     {{QStringLiteral("base_link_inertia"), QStringLiteral("base.dae")}, {QStringLiteral("shoulder_link"), QStringLiteral("shoulder.dae")}, {QStringLiteral("upper_arm_link"), QStringLiteral("upperarm.dae")}, {QStringLiteral("forearm_link"), QStringLiteral("forearm.dae")}, {QStringLiteral("wrist_1_link"), QStringLiteral("wrist1.dae")}, {QStringLiteral("wrist_2_link"), QStringLiteral("wrist2.dae")}, {QStringLiteral("wrist_3_link"), QStringLiteral("wrist3.dae")}}}
  };
  return profiles;
}

QString scene3d_robot_identity_haystack_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  return QStringList{item.id, item.display_name, item.category, item.role, item.source_layer, item.active_visual_source,
                     item.visual_index_link, item.visual_index_link_name, item.visual_index_mesh_uri, item.visual_index_package_uri,
                     item.package_uri, item.mesh_path, item.source_path, item.resolved_source_path_original, item.metadata_tags}
    .join(QStringLiteral("|")).toLower();
}

const GeneratedRobotViewportProfile * generated_robot_profile_for_item(const ScenePreviewWidget::PreviewItem & item)
{
  const QString haystack = scene3d_robot_identity_haystack_for_item(item);
  for (const auto & profile : generated_robot_viewport_profiles()) {
    for (const QString & token : profile.identity_tokens) {
      if (!token.trimmed().isEmpty() && haystack.contains(token.toLower())) return &profile;
    }
  }
  return nullptr;
}

const GeneratedRobotViewportProfile * generated_robot_profile_for_required_link_item(const ScenePreviewWidget::PreviewItem & item)
{
  const auto * profile = generated_robot_profile_for_item(item);
  if (!profile) return nullptr;
  return profile->required_links.contains(scene3d_canonical_link_name_for_item(item)) ? profile : nullptr;
}

QString scene3d_canonical_link_name_for_item(const ScenePreviewWidget::PreviewItem & item);

QMatrix4x4 final_mesh_transform_matrix(const ScenePreviewWidget::PreviewItem & item)
{
  const bool required_generated_robot_visual =
    (is_generated_urdf_visual_item(item) || is_locked_urdf_item(item)) &&
    is_required_generated_robot_viewport_link(item);

  if (item.has_baked_world_visual_transform && item.has_baked_world_visual_matrix) {
    // Matrix-baked generated URDF rows already carry world_T_visual from the
    // visual mesh index (or the one-time pose-to-matrix handoff). Apply the
    // Scene3D ROS-to-viewport basis, then only the narrow asset-local correction
    // path for known UR5 DAE visual meshes before mesh scale. Do not rebuild
    // from xyz/rpy or reapply the URDF visual origin.
    QMatrix4x4 transform = ros_to_viewport_basis_matrix() * item.baked_world_visual_matrix;
    apply_baked_mesh_asset_local_correction_matrix(transform, item);
    transform.scale(static_cast<float>(item.mesh_scale_x),
                    static_cast<float>(item.mesh_scale_y),
                    static_cast<float>(item.mesh_scale_z));
    if (required_generated_robot_visual) {
      qInfo().noquote() << QStringLiteral("UR5_BAKED_MATRIX_APPLIED link=%1 source=%2")
        .arg(scene3d_canonical_link_name_for_item(item), item.baked_world_visual_transform_source);
    }
    return transform;
  }

  if (item.has_baked_world_visual_transform) {
    if (required_generated_robot_visual) {
      qWarning().noquote() << QStringLiteral("ur5_baked_matrix_available_but_not_used link=%1 source=%2")
        .arg(scene3d_canonical_link_name_for_item(item), item.baked_world_visual_transform_source);
    }
    QMatrix4x4 transform = viewport_world_visual_transform(item);
    transform.scale(static_cast<float>(item.mesh_scale_x),
                    static_cast<float>(item.mesh_scale_y),
                    static_cast<float>(item.mesh_scale_z));
    return transform;
  }

  QMatrix4x4 transform = viewport_world_visual_transform(item);
  apply_mesh_local_correction_matrix(transform, item);
  transform.scale(static_cast<float>(item.mesh_scale_x),
                  static_cast<float>(item.mesh_scale_y),
                  static_cast<float>(item.mesh_scale_z));
  return transform;
}

[[maybe_unused]] void apply_mesh_local_correction_gl(const ScenePreviewWidget::PreviewItem & item)
{
  // Legacy GL-path helper retained for regression coverage only. The active
  // renderer uses final_mesh_transform_matrix() so CPU bounds and GL draw paths
  // share the same baked-matrix transform.
  if (item.has_baked_world_visual_transform) return;
  apply_urdf_rpy_gl(item.mesh_r, item.mesh_p, item.mesh_y);
  if (item.has_origin_offset) {
    glTranslated(item.origin_offset_x, item.origin_offset_y, item.origin_offset_z);
  }
}

bool item_has_credible_mesh_handoff(const ScenePreviewWidget::PreviewItem & item)
{
  const QString mesh_path = item.mesh_path.trimmed();
  const QString package_uri = item.package_uri.trimmed();
  const QString source_path = item.source_path.trimmed();
  // Authoring files such as environment.yaml identify semantic/layout rows, not
  // renderable meshes.  Treat a row as mesh-backed only when the handoff has
  // real mesh metadata/availability or a source field that looks like a mesh
  // asset. This keeps semantic robot_base/robot_reach/conveyor/object/warning
  // rows out of the generated-URDF mesh renderer and its rejection logs.
  return item.mesh_available ||
         item.has_mesh_metadata ||
         (!mesh_path.isEmpty() && path_has_mesh_asset_extension(mesh_path)) ||
         (!package_uri.isEmpty() && path_has_mesh_asset_extension(package_uri)) ||
         (!source_path.isEmpty() && path_has_mesh_asset_extension(source_path));
}

bool is_generated_urdf_visual_item(const ScenePreviewWidget::PreviewItem & it);
bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it);
bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it);
bool is_required_generated_robot_viewport_link(const ScenePreviewWidget::PreviewItem & item);
QString scene3d_link_name_for_item(const ScenePreviewWidget::PreviewItem & item);

bool item_has_mesh_surface_candidate(const ScenePreviewWidget::PreviewItem & item)
{
  const QString mesh_path = item.mesh_path.trimmed();
  const QString package_uri = item.package_uri.trimmed();
  const QString source_path = item.source_path.trimmed();
  return item.mesh_available ||
         item.has_mesh_metadata ||
         (!mesh_path.isEmpty() && path_has_mesh_asset_extension(mesh_path)) ||
         (!package_uri.isEmpty() && path_has_mesh_asset_extension(package_uri)) ||
         (!source_path.isEmpty() && path_has_mesh_asset_extension(source_path));
}

bool is_rviz_parity_robot_layer_item(const ScenePreviewWidget::PreviewItem & item)
{
  // RVizParityRobotLayer: generated/locked robot visuals are rendered from the
  // same flattened URDF visual mesh rows used by the RViz truth path.  This
  // deliberately does not depend on editable placeholders such as robot_base,
  // and it is evaluated before overlay/helper classification can suppress the
  // generated robot mesh layer.
  const bool generated_or_locked_visual = is_generated_urdf_visual_item(item) || is_locked_urdf_item(item);
  if (!generated_or_locked_visual) return false;
  if (is_overlay_only_item(item)) return false;
  return item_has_mesh_surface_candidate(item);
}

bool item_has_valid_urdf_primitive(const ScenePreviewWidget::PreviewItem & item)
{
  const QString type = item.primitive_geometry_type.trimmed().toLower().replace(QLatin1Char('-'), QLatin1Char('_')).replace(QLatin1Char(' '), QLatin1Char('_'));
  if (type == QStringLiteral("box")) return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
  if (type == QStringLiteral("cylinder")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  if (type == QStringLiteral("sphere")) return item.primitive_radius > 0.001;
  if (type == QStringLiteral("capsule")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  return false;
}

bool item_has_explicit_primitive_dimensions(const ScenePreviewWidget::PreviewItem & item)
{
  return item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001);
}

bool primitive_local_bounds_for_item(const ScenePreviewWidget::PreviewItem & item, QVector3D & out_min, QVector3D & out_max)
{
  const QString type = item.primitive_geometry_type.trimmed().toLower().replace(QLatin1Char('-'), QLatin1Char('_')).replace(QLatin1Char(' '), QLatin1Char('_'));
  if (type == QStringLiteral("cylinder")) {
    if (item.primitive_radius <= 0.001 || item.primitive_length <= 0.001) return false;
    const float r = static_cast<float>(item.primitive_radius);
    const float h = static_cast<float>(item.primitive_length);
    // draw_urdf_primitive_geometry renders cylinders along the local Y axis.
    out_min = QVector3D(-r, -h * 0.5f, -r);
    out_max = QVector3D(r, h * 0.5f, r);
    return true;
  }
  if (type == QStringLiteral("sphere")) {
    if (item.primitive_radius <= 0.001) return false;
    const float r = static_cast<float>(item.primitive_radius);
    out_min = QVector3D(-r, -r, -r);
    out_max = QVector3D(r, r, r);
    return true;
  }
  if (type == QStringLiteral("capsule")) {
    if (item.primitive_radius <= 0.001 || item.primitive_length <= 0.001) return false;
    const float r = static_cast<float>(item.primitive_radius);
    const float h = static_cast<float>(item.primitive_length);
    // Capsule is drawn as a local-Y cylinder plus radius-sized end spheres.
    out_min = QVector3D(-r, -h * 0.5f - r, -r);
    out_max = QVector3D(r, h * 0.5f + r, r);
    return true;
  }
  if (type == QStringLiteral("box") &&
      item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001) {
    out_min = QVector3D(static_cast<float>(-item.sx * 0.5),
                        static_cast<float>(-item.sy * 0.5),
                        static_cast<float>(-item.sz * 0.5));
    out_max = QVector3D(static_cast<float>(item.sx * 0.5),
                        static_cast<float>(item.sy * 0.5),
                        static_cast<float>(item.sz * 0.5));
    return true;
  }
  if (type.isEmpty() && item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001) {
    // Semantic fallback/editable boxes are rendered as axis-aligned min-corner boxes.
    out_min = QVector3D(0.0f, 0.0f, 0.0f);
    out_max = QVector3D(static_cast<float>(item.sx), static_cast<float>(item.sy), static_cast<float>(item.sz));
    return true;
  }
  return false;
}

struct PrimitiveWorldBounds { double x, y, z, sx, sy, sz; };

bool primitive_world_bounds_for_item(const ScenePreviewWidget::PreviewItem & item, PrimitiveWorldBounds & out_bounds)
{
  QVector3D lmin, lmax;
  if (!primitive_local_bounds_for_item(item, lmin, lmax)) return false;

  const QString type = item.primitive_geometry_type.trimmed().toLower().replace(QLatin1Char('-'), QLatin1Char('_')).replace(QLatin1Char(' '), QLatin1Char('_'));
  const bool axis_aligned_scene_box = (type.isEmpty() || type == QStringLiteral("box")) &&
                                      !item_has_valid_urdf_primitive(item);
  if (axis_aligned_scene_box) {
    out_bounds = { item.x, item.y, item.z, item.sx, item.sy, item.sz };
    return true;
  }

  QMatrix4x4 transform = authoritative_world_visual_transform(item);
  transform.scale(item.mesh_scale_x, item.mesh_scale_y, item.mesh_scale_z);

  QVector3D wmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D wmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int xi = 0; xi < 2; ++xi) {
    for (int yi = 0; yi < 2; ++yi) {
      for (int zi = 0; zi < 2; ++zi) {
        const QVector3D local_corner(
          xi == 0 ? lmin.x() : lmax.x(),
          yi == 0 ? lmin.y() : lmax.y(),
          zi == 0 ? lmin.z() : lmax.z());
        const QVector3D world_corner = transform * local_corner;
        wmin.setX(qMin(wmin.x(), world_corner.x()));
        wmin.setY(qMin(wmin.y(), world_corner.y()));
        wmin.setZ(qMin(wmin.z(), world_corner.z()));
        wmax.setX(qMax(wmax.x(), world_corner.x()));
        wmax.setY(qMax(wmax.y(), world_corner.y()));
        wmax.setZ(qMax(wmax.z(), world_corner.z()));
      }
    }
  }
  out_bounds = { wmin.x(), wmin.y(), wmin.z(),
                 qMax(0.0f, wmax.x() - wmin.x()),
                 qMax(0.0f, wmax.y() - wmin.y()),
                 qMax(0.0f, wmax.z() - wmin.z()) };
  return true;
}

void finalize_visual_quality(Scene3DViewportWidget::RenderDebugCounters & counters)
{
  counters.total_payload_count = counters.preview_items_count;
  counters.mesh_backed_count = counters.mesh_source_count;
  if (!scene3d_debug_fallback_boxes_enabled()) {
    counters.mesh_bounds_fallback_rendered_count = 0;
    counters.generated_mesh_bounds_fallback_rendered_count = 0;
  }
  counters.overlay_count = counters.overlay_helper_count;
  counters.overlay_rendered_count = counters.overlay_helper_count;
  counters.valid_physical_fallback_count = counters.primitive_fallback_rendered_count;

  QStringList warnings;
  QString status = QStringLiteral("PASS");

  if (counters.visible_count <= 0 && counters.total_payload_count <= 0) {
    status = QStringLiteral("UNAVAILABLE");
    warnings.append(QStringLiteral("no_preview_payload"));
  }
  const int rendered_mesh_success_count = qMax(counters.mesh_rendered_count, counters.mesh_surface_rendered_count);
  const int rendered_primitive_success_count = qMax(counters.urdf_primitive_rendered_count, counters.editable_primitive_rendered_count);
  if (counters.last_paint_completed && counters.mesh_source_count > 0 && rendered_mesh_success_count <= 0) {
    status = QStringLiteral("FAIL");
    warnings.append(QStringLiteral("mesh_sources_present_but_none_rendered"));
  }
  if (counters.last_paint_completed && counters.urdf_primitive_source_count > 0 && rendered_primitive_success_count <= 0) {
    status = QStringLiteral("FAIL");
    warnings.append(QStringLiteral("urdf_primitive_sources_present_but_none_rendered"));
  }

  const bool high_mesh_source_low_render =
    counters.mesh_source_count >= 4 &&
    counters.mesh_rendered_count > 0 &&
    counters.mesh_rendered_count * 2 < counters.mesh_source_count;
  if (high_mesh_source_low_render) {
    if (status != QStringLiteral("FAIL")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("mesh_source_count_high_but_mesh_rendered_count_low"));
  }
  if (counters.missing_geometry_count > 0) {
    if (status == QStringLiteral("PASS")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("missing_geometry_requires_diagnostics"));
  }
  if (counters.placeholder_count > 0) {
    if (status == QStringLiteral("PASS")) status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("placeholder_geometry_rendered"));
  }
  if (counters.visible_count > 0 &&
      counters.mesh_source_count <= 0 &&
      counters.urdf_primitive_source_count <= 0 &&
      counters.wireframe_fallback_count > 0 &&
      status == QStringLiteral("PASS")) {
    status = QStringLiteral("WARNING");
    warnings.append(QStringLiteral("wireframe_fallback_only_preview"));
  }

  counters.visual_quality_status = status;
  counters.visual_quality_warnings = warnings;
}


bool product_view_has_generated_mesh_warning_content(const Scene3DViewportWidget::RenderDebugCounters & counters)
{
  // Product view warnings are reserved for generated/locked URDF preview quality problems.
  // Editable layout primitives, helper overlays, FOV/reach aids, debug bounds, and generic
  // wireframe diagnostics remain visible diagnostics only; they must not look like generated
  // mesh fallback failures in the normal Workcell Studio product view.
  return counters.generated_mesh_bounds_fallback_rendered_count > 0 ||
         counters.generated_missing_geometry_count > 0 ||
         counters.generated_fallback_count > 0;
}

bool debug_view_has_full_diagnostic_warning_content(const Scene3DViewportWidget::RenderDebugCounters & counters)
{
  return counters.missing_geometry_count > 0 ||
         counters.mesh_bounds_fallback_rendered_count > 0 ||
         counters.primitive_fallback_rendered_count > 0 ||
         counters.wireframe_fallback_count > 0 ||
         counters.placeholder_count > 0 ||
         counters.overlay_helper_count > 0 ||
         counters.visual_quality_status == QStringLiteral("FAIL");
}

QString mesh_load_failure_reason_for_item(const QString & path, const ScenePreviewWidget::PreviewItem * item)
{
  const QString trimmed_path = path.trimmed();
  if (item) {
    const QString outcome = item->source_path_resolution_outcome.trimmed().toLower();
    if (item->resolved_source_path_stale || outcome.contains(QStringLiteral("stale"))) return QStringLiteral("stale_path");
    if (trimmed_path.startsWith(QStringLiteral("package://"), Qt::CaseInsensitive) ||
        !item->package_uri.trimmed().isEmpty() || outcome.contains(QStringLiteral("package_uri"))) {
      if (outcome.contains(QStringLiteral("unresolved")) || outcome.contains(QStringLiteral("missing")) ||
          trimmed_path.startsWith(QStringLiteral("package://"), Qt::CaseInsensitive)) {
        return QStringLiteral("package_uri_unresolved");
      }
    }
  }
  return QStringLiteral("file_not_found");
}

QString mesh_parse_failure_code(const QString & parse_error)
{
  const QString normalized = parse_error.trimmed().toLower();
  if (normalized.contains(QStringLiteral("no triangles")) || normalized.contains(QStringLiteral("contains no triangles"))) {
    return QStringLiteral("zero_triangle_mesh");
  }
  if (normalized.contains(QStringLiteral("exceeds limit"))) return QStringLiteral("triangle_budget_exceeded_preview_surrogate");
  return QStringLiteral("parse_failed");
}

bool parse_ascii_stl(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                     QString & out_error, int triangle_limit)
{
  std::istringstream stream(QString::fromUtf8(bytes).toStdString());
  std::string token;
  while (stream >> token) {
    if (QString::fromStdString(token).compare("facet", Qt::CaseInsensitive) != 0) continue;
    std::string normal_tok;
    if (!(stream >> normal_tok) || QString::fromStdString(normal_tok).compare("normal", Qt::CaseInsensitive) != 0) {
      out_error = "invalid facet normal token";
      return false;
    }
    float nx, ny, nz;
    if (!(stream >> nx >> ny >> nz)) { out_error = "invalid normal values"; return false; }
    std::string outer, loop;
    if (!(stream >> outer >> loop) ||
        QString::fromStdString(outer).compare("outer", Qt::CaseInsensitive) != 0 ||
        QString::fromStdString(loop).compare("loop", Qt::CaseInsensitive) != 0) {
      out_error = "invalid outer loop token";
      return false;
    }
    Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
    tri.normal = QVector3D(nx, ny, nz);
    for (int i = 0; i < 3; ++i) {
      std::string vertex;
      float vx, vy, vz;
      if (!(stream >> vertex >> vx >> vy >> vz) ||
          QString::fromStdString(vertex).compare("vertex", Qt::CaseInsensitive) != 0) {
        out_error = "invalid vertex token";
        return false;
      }
      tri.vertices[i] = QVector3D(vx, vy, vz);
    }
    std::string endloop, endfacet;
    if (!(stream >> endloop >> endfacet) ||
        QString::fromStdString(endloop).compare("endloop", Qt::CaseInsensitive) != 0 ||
        QString::fromStdString(endfacet).compare("endfacet", Qt::CaseInsensitive) != 0) {
      out_error = "invalid endloop/endfacet token";
      return false;
    }
    out_mesh.triangles.push_back(tri);
    if (out_mesh.triangles.size() > triangle_limit) { out_error = "mesh triangle count exceeds limit"; return false; }
  }
  if (out_mesh.triangles.isEmpty()) { out_error = "ascii STL contains no triangles"; return false; }
  return true;
}

bool parse_binary_stl(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                      QString & out_error, int triangle_limit)
{
  if (bytes.size() < 84) { out_error = "binary STL too small"; return false; }
  const uchar * data = reinterpret_cast<const uchar *>(bytes.constData());
  const quint32 tri_count = qFromLittleEndian<quint32>(data + 80);
  const bool exceeds_triangle_limit = tri_count > static_cast<quint32>(triangle_limit);
  const qint64 expected_size = 84LL + static_cast<qint64>(tri_count) * 50LL;
  if (bytes.size() < expected_size) { out_error = "binary STL truncated"; return false; }
  out_mesh.triangles.reserve(static_cast<int>(qMin(tri_count, static_cast<quint32>(triangle_limit))));
  const uchar * tri_ptr = data + 84;
  auto read_float = [](const uchar * p) {
    quint32 raw = qFromLittleEndian<quint32>(p);
    float value;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
  };
  const quint32 tri_read_count = exceeds_triangle_limit ? static_cast<quint32>(triangle_limit) : tri_count;
  for (quint32 i = 0; i < tri_read_count; ++i, tri_ptr += 50) {
    Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
    tri.normal = QVector3D(read_float(tri_ptr), read_float(tri_ptr + 4), read_float(tri_ptr + 8));
    for (int vi = 0; vi < 3; ++vi) {
      const uchar * vp = tri_ptr + 12 + vi * 12;
      tri.vertices[vi] = QVector3D(read_float(vp), read_float(vp + 4), read_float(vp + 8));
    }
    out_mesh.triangles.push_back(tri);
  }
  if (out_mesh.triangles.isEmpty()) { out_error = "binary STL contains no triangles"; return false; }
  if (exceeds_triangle_limit) { out_error = "mesh triangle count exceeds limit"; return false; }
  return true;
}

bool looks_like_ascii_stl(const QByteArray & bytes)
{
  const QByteArray prefix = bytes.left(256).trimmed().toLower();
  return prefix.startsWith("solid") && prefix.contains("facet");
}



bool parse_obj_face_vertex_index(const QString & token, int vertex_count, int & out_index)
{
  const QString vertex_token = token.section(QLatin1Char('/'), 0, 0).trimmed();
  if (vertex_token.isEmpty()) return false;
  bool ok = false;
  const int raw_index = vertex_token.toInt(&ok);
  if (!ok || raw_index == 0) return false;
  const int zero_based = raw_index > 0 ? raw_index - 1 : vertex_count + raw_index;
  if (zero_based < 0 || zero_based >= vertex_count) return false;
  out_index = zero_based;
  return true;
}

bool parse_obj_bytes(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                     QString & out_error, int triangle_limit)
{
  const QString text = QString::fromUtf8(bytes);
  const QStringList lines = text.split(QRegExp("\\r?\\n"));
  QVector<QVector3D> vertices;
  vertices.reserve(lines.size());
  for (int line_number = 0; line_number < lines.size(); ++line_number) {
    QString line = lines.at(line_number).trimmed();
    const int comment_pos = line.indexOf(QLatin1Char('#'));
    if (comment_pos >= 0) line = line.left(comment_pos).trimmed();
    if (line.isEmpty()) continue;
    const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (parts.isEmpty()) continue;
    const QString kind = parts.first();
    if (kind == QStringLiteral("v")) {
      if (parts.size() < 4) {
        out_error = QStringLiteral("obj vertex on line %1 has fewer than 3 coordinates").arg(line_number + 1);
        return false;
      }
      bool ok_x = false, ok_y = false, ok_z = false;
      const float x = parts.at(1).toFloat(&ok_x);
      const float y = parts.at(2).toFloat(&ok_y);
      const float z = parts.at(3).toFloat(&ok_z);
      if (!ok_x || !ok_y || !ok_z || !qIsFinite(x) || !qIsFinite(y) || !qIsFinite(z)) {
        out_error = QStringLiteral("obj vertex on line %1 has invalid coordinates").arg(line_number + 1);
        return false;
      }
      vertices.push_back(QVector3D(x, y, z));
      continue;
    }
    if (kind != QStringLiteral("f")) continue;
    if (parts.size() < 4) continue;
    QVector<int> face_indices;
    face_indices.reserve(parts.size() - 1);
    for (int i = 1; i < parts.size(); ++i) {
      int vertex_index = -1;
      if (!parse_obj_face_vertex_index(parts.at(i), vertices.size(), vertex_index)) {
        out_error = QStringLiteral("obj face on line %1 references invalid vertex '%2'").arg(line_number + 1).arg(parts.at(i));
        return false;
      }
      face_indices.push_back(vertex_index);
    }
    const int base = face_indices.first();
    for (int i = 1; i + 1 < face_indices.size(); ++i) {
      Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
      tri.vertices[0] = vertices.at(base);
      tri.vertices[1] = vertices.at(face_indices.at(i));
      tri.vertices[2] = vertices.at(face_indices.at(i + 1));
      tri.normal = QVector3D::crossProduct(tri.vertices[1] - tri.vertices[0], tri.vertices[2] - tri.vertices[0]);
      out_mesh.triangles.push_back(tri);
      if (out_mesh.triangles.size() > triangle_limit) {
        out_error = QStringLiteral("mesh triangle count exceeds limit");
        return false;
      }
    }
  }
  if (out_mesh.triangles.isEmpty()) {
    out_error = QStringLiteral("obj contains no triangles");
    return false;
  }
  return true;
}

bool parse_collada_bytes(const QByteArray & bytes, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                         QString & out_error, double * out_unit_meter, int triangle_limit)
{
  QXmlStreamReader xml(bytes);
  if (out_unit_meter) *out_unit_meter = 1.0;
  QVector<float> positions;
  bool in_geometry = false;
  while (!xml.atEnd()) {
    xml.readNext();
    if (xml.isStartElement() && xml.name() == QStringLiteral("asset")) {
      while (!(xml.isEndElement() && xml.name() == QStringLiteral("asset")) && !xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement() && xml.name() == QStringLiteral("unit")) {
          const double meter = xml.attributes().value(QStringLiteral("meter")).toDouble();
          if (out_unit_meter && meter > 0.0 && qIsFinite(meter)) *out_unit_meter = meter;
        }
      }
    }
    if (xml.isStartElement() && xml.name() == QStringLiteral("geometry")) {
      in_geometry = true;
      positions.clear();
    }
    if (xml.isEndElement() && xml.name() == QStringLiteral("geometry")) {
      in_geometry = false;
      positions.clear();
      continue;
    }
    if (!in_geometry || !xml.isStartElement()) continue;
    if (xml.name() == QStringLiteral("float_array") && positions.isEmpty()) {
      // RViz/Assimp-style tolerance: many ROS Collada exports use opaque IDs
      // (for example ID5) rather than names containing "position".  The first
      // mesh float array in these assets is the vertex position stream; later
      // arrays are normals/UVs/material data and are ignored by this fallback.
      const QStringList vals = xml.readElementText().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      if (vals.size() >= 9 && vals.size() % 3 == 0) {
        positions.reserve(vals.size());
        for (const auto & v : vals) positions.push_back(v.toFloat());
      }
    }
    if ((xml.name() == QStringLiteral("triangles") || xml.name() == QStringLiteral("polylist")) && !positions.isEmpty()) {
      int vcount = 3;
      QVector<int> poly_vcounts;
      QString ptext;
      while (!(xml.isEndElement() && (xml.name() == QStringLiteral("triangles") || xml.name() == QStringLiteral("polylist"))) && !xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement() && xml.name() == QStringLiteral("input")) {
          const QString semantic = xml.attributes().value(QStringLiteral("semantic")).toString();
          if (semantic == QStringLiteral("VERTEX")) vcount = qMax(vcount, xml.attributes().value(QStringLiteral("offset")).toInt() + 1);
        }
        if (xml.isStartElement() && xml.name() == QStringLiteral("vcount")) {
          const QStringList vals = xml.readElementText().split(QRegExp("\\s+"), Qt::SkipEmptyParts);
          for (const auto & v : vals) poly_vcounts.push_back(v.toInt());
        }
        if (xml.isStartElement() && xml.name() == QStringLiteral("p")) ptext = xml.readElementText();
      }
      const QStringList idxs = ptext.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      QVector<int> all; all.reserve(idxs.size());
      for (const auto & t : idxs) all.push_back(t.toInt());
      const double unit_meter = (out_unit_meter && *out_unit_meter > 0.0 && qIsFinite(*out_unit_meter)) ? *out_unit_meter : 1.0;
      auto addTri=[&](int a,int b,int c){
        const int na=a*3, nb=b*3, nc=c*3;
        if (na+2>=positions.size()||nb+2>=positions.size()||nc+2>=positions.size()) return;
        Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
        tri.vertices[0]=QVector3D(positions[na],positions[na+1],positions[na+2]) * static_cast<float>(unit_meter);
        tri.vertices[1]=QVector3D(positions[nb],positions[nb+1],positions[nb+2]) * static_cast<float>(unit_meter);
        tri.vertices[2]=QVector3D(positions[nc],positions[nc+1],positions[nc+2]) * static_cast<float>(unit_meter);
        tri.normal = QVector3D::crossProduct(tri.vertices[1]-tri.vertices[0], tri.vertices[2]-tri.vertices[0]);
        out_mesh.triangles.push_back(tri);
      };
      if (xml.name() == QStringLiteral("triangles") || poly_vcounts.isEmpty()) {
        for (int i=0;i+vcount*3-1<all.size(); i+=vcount*3) { addTri(all[i], all[i+vcount], all[i+2*vcount]); if (out_mesh.triangles.size()>triangle_limit){ out_error="mesh triangle count exceeds limit"; return false; } }
      } else {
        int off=0;
        for (int pc : poly_vcounts) {
          if (pc<3) { off += pc*vcount; continue; }
          const int base = all[off];
          for (int k=1;k+1<pc;++k) { addTri(base, all[off + k*vcount], all[off + (k+1)*vcount]); if (out_mesh.triangles.size()>triangle_limit){ out_error="mesh triangle count exceeds limit"; return false; } }
          off += pc*vcount; if (off>=all.size()) break;
        }
      }
    }
  }
  if (xml.hasError()) { out_error = QStringLiteral("collada parse error: ") + xml.errorString(); return false; }
  if (out_mesh.triangles.isEmpty()) { out_error = QStringLiteral("collada contains no triangles"); return false; }
  return true;
}
#ifdef WORKCELL_BUILDER_HAS_ASSIMP
QVector3D assimp_vec_to_qvector(const aiVector3D & v)
{
  return QVector3D(v.x, v.y, v.z);
}

bool append_assimp_node_meshes(const aiScene * scene, const aiNode * node, const aiMatrix4x4 & parent_transform,
                               Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                               QString & out_error, int triangle_limit)
{
  if (!scene || !node) return true;
  const aiMatrix4x4 world_transform = parent_transform * node->mTransformation;
  for (unsigned int mesh_index_i = 0; mesh_index_i < node->mNumMeshes; ++mesh_index_i) {
    const unsigned int mesh_index = node->mMeshes[mesh_index_i];
    if (mesh_index >= scene->mNumMeshes || !scene->mMeshes[mesh_index]) continue;
    const aiMesh * mesh = scene->mMeshes[mesh_index];
    if (!mesh->HasPositions()) continue;
    for (unsigned int face_i = 0; face_i < mesh->mNumFaces; ++face_i) {
      const aiFace & face = mesh->mFaces[face_i];
      if (face.mNumIndices != 3) continue;
      Scene3DViewportWidget::InternalTriangleMesh::Triangle tri;
      for (int vi = 0; vi < 3; ++vi) {
        const unsigned int vertex_index = face.mIndices[vi];
        if (vertex_index >= mesh->mNumVertices) {
          out_error = QStringLiteral("assimp mesh face references invalid vertex");
          return false;
        }
        aiVector3D transformed = mesh->mVertices[vertex_index];
        transformed *= world_transform;
        tri.vertices[vi] = assimp_vec_to_qvector(transformed);
      }
      tri.normal = QVector3D::crossProduct(tri.vertices[1] - tri.vertices[0], tri.vertices[2] - tri.vertices[0]);
      if (tri.normal.lengthSquared() > 0.0f) tri.normal.normalize();
      out_mesh.triangles.push_back(tri);
      if (out_mesh.triangles.size() > triangle_limit) {
        out_error = QStringLiteral("mesh triangle count exceeds limit");
        return false;
      }
    }
  }
  for (unsigned int child_i = 0; child_i < node->mNumChildren; ++child_i) {
    if (!append_assimp_node_meshes(scene, node->mChildren[child_i], world_transform, out_mesh, out_error, triangle_limit)) {
      return false;
    }
  }
  return true;
}

bool parse_mesh_with_assimp(const QString & canonical_path, Scene3DViewportWidget::InternalTriangleMesh & out_mesh,
                            QString & out_error, int triangle_limit)
{
  out_mesh.triangles.clear();
  Assimp::Importer importer;
  const unsigned int flags = aiProcess_Triangulate |
                             aiProcess_GenSmoothNormals |
                             aiProcess_JoinIdenticalVertices |
                             aiProcess_SortByPType |
                             aiProcess_ValidateDataStructure;
  const aiScene * scene = importer.ReadFile(canonical_path.toStdString(), flags);
  if (!scene) {
    out_error = QStringLiteral("assimp import failed: %1").arg(QString::fromUtf8(importer.GetErrorString()));
    return false;
  }
  if (!scene->mRootNode) {
    out_error = QStringLiteral("assimp import produced no root node");
    return false;
  }
  aiMatrix4x4 identity;
  if (!append_assimp_node_meshes(scene, scene->mRootNode, identity, out_mesh, out_error, triangle_limit)) return false;
  if (out_mesh.triangles.isEmpty()) {
    out_error = QStringLiteral("assimp import contains no triangles");
    return false;
  }
  return true;
}
#endif

enum class NormalizedRole
{
  RobotBase,
  RobotReach,
  Table,
  Conveyor,
  Camera,
  PickZone,
  PlaceZone,
  PlaceBin,
  Object,
  SafetyZone,
  HomePose,
  WarningAnchor,
  Generic
};

QString normalized_token(const QString & value)
{
  return workcell_builder::scene3d_visual_classification::normalized_token(value);
}

QString normalized_scene3d_layer_token(const QString & value)
{
  return workcell_builder::scene3d_visual_classification::normalized_layer_token(value);
}

bool is_generated_urdf_visual_item(const ScenePreviewWidget::PreviewItem & it)
{
  return workcell_builder::scene3d_visual_classification::is_generated_urdf_visual_identity(it);
}

bool is_generated_urdf_visual_fallback_item(const ScenePreviewWidget::PreviewItem & it)
{
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  return it.id.startsWith(QStringLiteral("generated_urdf_fallback::")) &&
         source_layer == QStringLiteral("locked_generated_urdf_visual") &&
         (visual_source == QStringLiteral("generated_urdf_visual_fallback") ||
          it.category.compare(QStringLiteral("URDF Visual Fallback"), Qt::CaseInsensitive) == 0);
}

bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it);

QString clean_label_from_item(const ScenePreviewWidget::PreviewItem & it)
{
  QString label = it.display_name.trimmed();
  if (label.isEmpty()) label = it.id.trimmed();
  if (label.isEmpty()) return QStringLiteral("Item");

  const int slash = qMax(label.lastIndexOf('/'), label.lastIndexOf(':'));
  if (slash >= 0 && slash + 1 < label.size()) label = label.mid(slash + 1);
  if (label.startsWith(QStringLiteral("urdf_visual"), Qt::CaseInsensitive)) label.remove(0, QStringLiteral("urdf_visual").size());
  label = label.trimmed();
  while (label.startsWith('_') || label.startsWith('-') || label.startsWith('/')) label.remove(0, 1);
  if (label.isEmpty()) label = it.id.trimmed();
  if (label.size() > 18) label = label.left(17) + QStringLiteral("…");
  return label;
}
QString warning_badge_text(const QStringList & warnings)
{
  if (warnings.isEmpty()) return QStringLiteral("!");
  if (warnings.size() == 1) return QStringLiteral("!");
  return QStringLiteral("%1").arg(qMin(warnings.size(), 9));
}

QString warning_debug_text(const QStringList & warnings)
{
  if (warnings.isEmpty()) return QStringLiteral("none");
  if (warnings.size() == 1) return warnings.front();
  return QStringLiteral("%1 (+%2 more)").arg(warnings.front()).arg(warnings.size() - 1);
}

bool item_is_editable_for_gizmo(const ScenePreviewWidget::PreviewItem & it)
{
  if (it.locked) return false;
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const QString role = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString lock_reason = normalized_token(it.lock_reason);

  const bool overlay_or_helper = role.contains("overlay") || role.contains("helper") || role.contains("guide") ||
                                 category.contains("overlay") || category.contains("helper") ||
                                 lock_reason.contains("overlay") || lock_reason.contains("helper");
  if (overlay_or_helper) return false;

  const bool generated_robot_visual = is_generated_urdf_visual_item(it) ||
                                      source_layer.contains("generated") || source_layer.contains("urdf") ||
                                      visual_source.contains("generated") || lock_reason.contains("urdf") ||
                                      lock_reason.contains("robot model") || lock_reason.contains("camera generated") ||
                                      lock_reason.contains("tool generated");
  if (generated_robot_visual) return false;

  if (source_layer == "editable_layout") return it.editable;
  if (source_layer == "primitive_fallback" || visual_source == "primitive_fallback") {
    return it.editable && it.linked_to_editable_layout_state;
  }
  if (visual_source == "mesh_preview" || source_layer == "mesh_preview") {
    return it.editable && it.linked_to_editable_layout_state;
  }
  return false;
}

QString item_locked_reason(const ScenePreviewWidget::PreviewItem & it)
{
  const QString reason = it.lock_reason.trimmed();
  return reason.isEmpty() ? QStringLiteral("item is locked") : reason;
}

QString item_visual_ownership_label(const ScenePreviewWidget::PreviewItem & it)
{
  if (is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) return QStringLiteral("Generated / locked preview");
  if (it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it)) return QStringLiteral("Editable layout");
  return QStringLiteral("Reference preview");
}

constexpr double kGeneratedLockedProductMinAlpha = 0.92;

QColor generated_locked_preview_material() { return QColor("#d6dde6"); }
QColor generated_locked_preview_outline() { return QColor(226, 232, 240, 150); }
QColor generated_primitive_fallback_fill() { return QColor(96, 165, 250, 52); }
QColor generated_primitive_fallback_outline() { return QColor(147, 197, 253, 108); }
QColor editable_layout_accent_outline() { return QColor("#22d3ee"); }
QColor editable_layout_selected_highlight() { return QColor("#f8fafc"); }


NormalizedRole classify_item_role(const ScenePreviewWidget::PreviewItem & it)
{
  const QString role = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString id = normalized_token(it.id);
  const QString display_name = normalized_token(it.display_name);
  const QString metadata = normalized_token(it.metadata_tags + QStringLiteral("|") + it.mesh_type + QStringLiteral("|") +
                                           it.material_name + QStringLiteral("|") + it.status);
  const QString mix = role + "|" + category + "|" + id + "|" + display_name + "|" + metadata;

  if (mix.contains("robot_reach") || mix.contains("reachability") || mix.contains("reach_envelope") ||
      mix.contains("reach_zone") || mix.contains("workspace_reach")) return NormalizedRole::RobotReach;
  if (mix.contains("robot_base") || mix.contains("robot base")) return NormalizedRole::RobotBase;
  if (mix.contains("home_pose") || mix.contains("home_position") || mix.contains("safe_pose") ||
      mix.contains("safety_pose") || mix.contains("stow_pose") || mix.contains("park_pose")) return NormalizedRole::HomePose;
  if (mix.contains("support_surface") || mix.contains("table") || mix.contains("work_surface") ||
      mix.contains("workbench") || mix.contains("fixture_surface")) return NormalizedRole::Table;
  if (mix.contains("conveyor") || mix.contains("belt")) return NormalizedRole::Conveyor;
  if (mix.contains("camera") || mix.contains("sensor") || mix.contains("realsense") ||
      mix.contains("depth_camera") || mix.contains("rgbd")) return NormalizedRole::Camera;
  if (mix.contains("pick_zone") || mix.contains("pick_area") || mix.contains("pick_region")) return NormalizedRole::PickZone;
  if (role == "place" || mix.contains("place_zone") || mix.contains("place_area") || mix.contains("drop_zone") ||
      mix.contains("drop_area")) return NormalizedRole::PlaceZone;
  if (mix.contains("place_target") || mix.contains("target_bin") || mix.contains("sorting_bin") ||
      mix.contains("part_bin") || mix.contains("bin") || mix.contains("container") ||
      mix.contains("tote") || mix.contains("tray")) return NormalizedRole::PlaceBin;
  if (mix.contains("safety_zone") || mix.contains("safety_boundary") || mix.contains("keepout") ||
      mix.contains("keep_out") || mix.contains("hazard_zone")) return NormalizedRole::SafetyZone;
  if (mix.contains("warning_anchor") || mix.contains("warning_badge") ||
      role == QStringLiteral("warning") || category == QStringLiteral("warning")) return NormalizedRole::WarningAnchor;
  if (mix.contains("object") || mix.contains("part") || mix.contains("item")) return NormalizedRole::Object;
  return NormalizedRole::Generic;
}

QString scene3d_canonical_link_name_for_item(const ScenePreviewWidget::PreviewItem & item);

bool item_identity_contains_any(const ScenePreviewWidget::PreviewItem & it, std::initializer_list<QString> tokens)
{
  const QString mix = normalized_token(it.id + QStringLiteral("|") + it.display_name + QStringLiteral("|") +
                                       it.role + QStringLiteral("|") + it.category + QStringLiteral("|") +
                                       it.metadata_tags + QStringLiteral("|") +
                                       it.mesh_type + QStringLiteral("|") + it.material_name + QStringLiteral("|") +
                                       it.mesh_path + QStringLiteral("|") + it.package_uri + QStringLiteral("|") +
                                       it.source_path + QStringLiteral("|") + it.resolved_source_path_original);
  for (const QString & token : tokens) {
    if (mix.contains(normalized_token(token))) return true;
  }
  return false;
}

bool item_is_tool_or_gripper_visual(const ScenePreviewWidget::PreviewItem & it)
{
  return item_identity_contains_any(it, {
    QStringLiteral("gripper"), QStringLiteral("tool"), QStringLiteral("end_effector"),
    QStringLiteral("end effector"), QStringLiteral("robotiq"), QStringLiteral("finger"),
    QStringLiteral("knuckle"), QStringLiteral("suction"), QStringLiteral("airpick"), QStringLiteral("vacuum")
  });
}

bool item_is_generated_robot_arm_visual(const ScenePreviewWidget::PreviewItem & it)
{
  if (!(is_generated_urdf_visual_item(it) || is_locked_urdf_item(it))) return false;
  if (item_is_tool_or_gripper_visual(it)) return false;
  if (classify_item_role(it) == NormalizedRole::Camera) return false;
  const QString canonical = scene3d_canonical_link_name_for_item(it);
  if (canonical.contains(QStringLiteral("camera")) || canonical.contains(QStringLiteral("sensor"))) return false;
  return true;
}


bool item_identifies_realsense_d435(const ScenePreviewWidget::PreviewItem & it, const QString & mesh_source = QString())
{
  const QString path_mix = (mesh_source + QStringLiteral("|") + it.mesh_path + QStringLiteral("|") +
                            it.source_path + QStringLiteral("|") + it.package_uri + QStringLiteral("|") +
                            it.resolved_source_path_original).toLower();
  const QString id_mix = normalized_token(it.id + QStringLiteral("|") + it.display_name + QStringLiteral("|") +
                                          it.detection_label + QStringLiteral("|") + it.camera_id + QStringLiteral("|") +
                                          it.role + QStringLiteral("|") + it.category + QStringLiteral("|") +
                                          it.metadata_tags + QStringLiteral("|") + it.mesh_type + QStringLiteral("|") +
                                          it.material_name);
  return path_mix.contains(QStringLiteral("realsense2_description/meshes/d435.dae")) ||
         path_mix.contains(QStringLiteral("realsense2_description\\meshes\\d435.dae")) ||
         path_mix.contains(QStringLiteral("realsense2_description/meshes/d435i.dae")) ||
         path_mix.contains(QStringLiteral("realsense2_description\\meshes\\d435i.dae")) ||
         id_mix.contains(QStringLiteral("d435i")) || id_mix.contains(QStringLiteral("d435"));
}

bool item_should_use_realsense_visual_surrogate(const ScenePreviewWidget::PreviewItem & it, const QString & mesh_source = QString())
{
  return item_identifies_realsense_d435(it, mesh_source);
}

bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it);


bool include_in_fit_bounds(const ScenePreviewWidget::PreviewItem & it, bool include_overlays)
{
  // FIT_PHYSICAL_ONLY_FILTER: default/product fits exclude overlays and helpers.
  if (include_overlays) return true;

  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const bool generated_or_locked_preview = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  const bool helper_overlay = !generated_or_locked_preview && (is_overlay_only_item(it) || source_layer == "overlay" || visual_source == "overlay");
  if (helper_overlay) return false;

  const bool generated_urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  const bool mesh_backed = item_has_credible_mesh_handoff(it);
  const bool explicit_primitive = item_has_explicit_primitive_dimensions(it);
  const bool missing_geometry = !mesh_backed && !explicit_primitive;
  if (missing_geometry && !it.linked_to_editable_layout_state && !generated_urdf_visual) return false;

  if (generated_urdf_visual) return true;
  if (it.linked_to_editable_layout_state) return true;
  if (source_layer == "mesh_preview" || visual_source == "mesh_preview") return mesh_backed || explicit_primitive;

  const NormalizedRole role = classify_item_role(it);
  const QString role_text = normalized_token(it.role);
  const QString category = normalized_token(it.category);
  const QString id = normalized_token(it.id);
  const QString display_name = normalized_token(it.display_name);
  const QString mix = role_text + "|" + category + "|" + id + "|" + display_name;
  const bool product_physical_role =
    role == NormalizedRole::RobotBase ||
    role == NormalizedRole::RobotReach ||
    role == NormalizedRole::Table ||
    role == NormalizedRole::Camera ||
    mix.contains("gripper") || mix.contains("tool") ||
    mix.contains("end_effector") || mix.contains("end effector");
  return product_physical_role && (mesh_backed || explicit_primitive);
}

QString scene3d_canonical_link_name_for_item(const ScenePreviewWidget::PreviewItem & item);
bool item_is_enabled_for_fit(const ScenePreviewWidget::PreviewItem & it);
bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it);

bool item_is_robot_generated_link_for_initial_fit(const ScenePreviewWidget::PreviewItem & it)
{
  if (!(is_generated_urdf_visual_item(it) || is_locked_urdf_item(it))) return false;
  const QString canonical = scene3d_canonical_link_name_for_item(it);
  static const QSet<QString> common_robot_links{
    QStringLiteral("base_link_inertia"), QStringLiteral("base_link"), QStringLiteral("shoulder_link"),
    QStringLiteral("upper_arm_link"), QStringLiteral("forearm_link"), QStringLiteral("wrist_1_link"),
    QStringLiteral("wrist_2_link"), QStringLiteral("wrist_3_link"), QStringLiteral("tool0"),
    QStringLiteral("base"), QStringLiteral("shoulder"), QStringLiteral("elbow"), QStringLiteral("wrist"),
    QStringLiteral("flange"), QStringLiteral("tcp")
  };
  if (common_robot_links.contains(canonical)) return true;
  const QString mix = normalized_token(it.id + QStringLiteral("|") + it.display_name + QStringLiteral("|") +
                                       it.frame_id + QStringLiteral("|") + it.mesh_path + QStringLiteral("|") +
                                       it.package_uri + QStringLiteral("|") + it.source_path + QStringLiteral("|") +
                                       it.role + QStringLiteral("|") + it.category + QStringLiteral("|") +
                                       it.metadata_tags);
  if (mix.contains(QStringLiteral("gripper")) || mix.contains(QStringLiteral("robotiq")) ||
      mix.contains(QStringLiteral("end_effector")) || mix.contains(QStringLiteral("camera"))) return false;
  return mix.contains(QStringLiteral("robot")) || mix.contains(QStringLiteral("robot_arm")) ||
         mix.contains(QStringLiteral("ur_description/meshes/")) ||
         mix.contains(QStringLiteral("ur_description\\meshes\\")) ||
         mix.contains(QStringLiteral("fanuc")) || mix.contains(QStringLiteral("abb")) ||
         mix.contains(QStringLiteral("panda")) || mix.contains(QStringLiteral("ur3")) ||
         mix.contains(QStringLiteral("ur5")) || mix.contains(QStringLiteral("ur10"));
}

bool item_is_gripper_generated_link_for_initial_fit(const ScenePreviewWidget::PreviewItem & it)
{
  if (!(is_generated_urdf_visual_item(it) || is_locked_urdf_item(it))) return false;
  const QString canonical = scene3d_canonical_link_name_for_item(it);
  if (canonical == QStringLiteral("robotiq_85_base_link") ||
      canonical == QStringLiteral("gripper_base_link")) return true;
  const QString mix = normalized_token(it.id + QStringLiteral("|") + it.display_name + QStringLiteral("|") +
                                       it.frame_id + QStringLiteral("|") + it.mesh_path + QStringLiteral("|") +
                                       it.package_uri + QStringLiteral("|") + it.source_path);
  return mix.contains(QStringLiteral("robotiq")) || mix.contains(QStringLiteral("gripper")) ||
         mix.contains(QStringLiteral("finger")) || mix.contains(QStringLiteral("knuckle")) ||
         mix.contains(QStringLiteral("end_effector"));
}

QString initial_physical_fit_anchor_role(const ScenePreviewWidget::PreviewItem & it)
{
  if (!item_is_enabled_for_fit(it)) return QString();
  if (is_overlay_only_item(it)) return QString();
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  if (source_layer == QStringLiteral("overlay") || visual_source == QStringLiteral("overlay")) return QString();

  if (item_is_robot_generated_link_for_initial_fit(it)) return QStringLiteral("generated_robot_visual");
  if (item_is_gripper_generated_link_for_initial_fit(it)) return QStringLiteral("tool_gripper_visual");

  const NormalizedRole role = classify_item_role(it);
  if (role == NormalizedRole::Table) return QStringLiteral("workbench_table");
  if (role == NormalizedRole::Camera) return QStringLiteral("camera_body");
  if (it.linked_to_editable_layout_state && include_in_fit_bounds(it, false)) return QStringLiteral("authored_physical_environment");
  const QString layer = source_layer + QStringLiteral("|") + visual_source;
  if ((layer.contains(QStringLiteral("layout")) || layer.contains(QStringLiteral("environment"))) && include_in_fit_bounds(it, false)) {
    return QStringLiteral("authored_physical_environment");
  }
  return QString();
}

bool item_is_enabled_for_fit(const ScenePreviewWidget::PreviewItem & it)
{
  const QString status = normalized_token(it.status);
  return status != QStringLiteral("disabled") &&
         status != QStringLiteral("off") &&
         status != QStringLiteral("hidden") &&
         status != QStringLiteral("suppressed");
}

bool is_overlay_only_item(const ScenePreviewWidget::PreviewItem & it)
{
  return workcell_builder::scene3d_visual_classification::is_helper_overlay_identity(it);
}

bool is_locked_urdf_item(const ScenePreviewWidget::PreviewItem & it)
{
  if (!(it.locked && !it.editable)) return false;
  const QString category = it.category.trimmed().toLower();
  const QString lock_reason = it.lock_reason.trimmed().toLower();
  return category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot model") ||
         lock_reason.contains("robotmodel") || lock_reason.contains("urdf visual");
}

void populate_runtime_transform_counters(
  Scene3DViewportWidget::RenderDebugCounters & counters,
  const QVector<ScenePreviewWidget::PreviewItem> & items)
{
  int transform_chain_applied_count = 0;
  int visual_origin_applied_count = 0;
  int baked_world_visual_transform_count = 0;
  int legacy_viewport_transform_count = 0;
  for (const auto & item : items) {
    const bool generated_or_locked_visual = is_generated_urdf_visual_item(item) || is_locked_urdf_item(item);
    if (!generated_or_locked_visual) continue;
    if (item.has_baked_world_visual_transform) {
      ++baked_world_visual_transform_count;
      continue;
    }
    ++legacy_viewport_transform_count;
    if (item.transform_chain_applied) ++transform_chain_applied_count;
    if (item.visual_origin_applied) ++visual_origin_applied_count;
  }
  counters.transform_chain_applied_count = transform_chain_applied_count;
  counters.visual_origin_applied_count = visual_origin_applied_count;
  counters.baked_world_visual_transform_count = baked_world_visual_transform_count;
  counters.legacy_viewport_transform_count = legacy_viewport_transform_count;
}


bool generated_urdf_item_has_renderable_geometry(const ScenePreviewWidget::PreviewItem & item)
{
  if (!(is_generated_urdf_visual_item(item) || is_locked_urdf_item(item))) return false;
  if (is_generated_urdf_visual_fallback_item(item) && is_required_generated_robot_viewport_link(item)) return true;
  const bool has_real_mesh = item.has_mesh_metadata && item_has_mesh_surface_candidate(item);
  return has_real_mesh || item_has_valid_urdf_primitive(item);
}

QString scene3d_camera_dedupe_key(const ScenePreviewWidget::PreviewItem & item)
{
  return QStringList{
    scene3d_canonical_link_name_for_item(item),
    item.mesh_path.trimmed(),
    item.source_path.trimmed(),
    item.package_uri.trimmed(),
    QString::number(item.x, 'g', 12), QString::number(item.y, 'g', 12), QString::number(item.z, 'g', 12),
    QString::number(item.roll, 'g', 12), QString::number(item.pitch, 'g', 12), QString::number(item.yaw, 'g', 12),
    normalized_scene3d_layer_token(item.source_layer)
  }.join(QStringLiteral("|"));
}

bool is_raw_generated_bounds_only_item(const ScenePreviewWidget::PreviewItem & it)
{
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  if (visual_source == QStringLiteral("primitive_fallback") || source_layer == QStringLiteral("primitive_fallback")) {
    return false;
  }
  const bool generated_or_locked = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  if (!generated_or_locked || !item_has_explicit_primitive_dimensions(it)) return false;
  return item_has_mesh_surface_candidate(it) || item_has_valid_urdf_primitive(it);
}



bool is_clean_semantic_primitive_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::RobotBase:
    case NormalizedRole::RobotReach:
    case NormalizedRole::Table:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::Conveyor:
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceZone:
    case NormalizedRole::Camera:
    case NormalizedRole::SafetyZone:
    case NormalizedRole::HomePose:
    case NormalizedRole::Object:
      return true;
    default:
      return false;
  }
}

bool should_suppress_missing_geometry_marker_for_semantic_role(const ScenePreviewWidget::PreviewItem & it)
{
  const NormalizedRole role = classify_item_role(it);
  return (is_clean_semantic_primitive_role(role) || role == NormalizedRole::WarningAnchor) &&
         !item_has_explicit_primitive_dimensions(it);
}

bool is_intentional_semantic_editor_primitive(const ScenePreviewWidget::PreviewItem & it)
{
  switch (classify_item_role(it)) {
    case NormalizedRole::RobotBase:
    case NormalizedRole::RobotReach:
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceZone:
    case NormalizedRole::SafetyZone:
    case NormalizedRole::HomePose:
    case NormalizedRole::WarningAnchor:
      return true;
    default:
      return false;
  }
}

bool is_overlay_visual_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceZone:
    case NormalizedRole::RobotReach:
    case NormalizedRole::SafetyZone:
    case NormalizedRole::WarningAnchor:
      return true;
    default:
      return false;
  }
}

std::vector<const ScenePreviewWidget::PreviewItem *> build_final_generated_urdf_robot_renderables(
  const QVector<ScenePreviewWidget::PreviewItem> & source_items,
  bool show_safety_layer,
  std::vector<const ScenePreviewWidget::PreviewItem *> * out_overlay_items = nullptr,
  std::vector<const ScenePreviewWidget::PreviewItem *> * out_physical_items = nullptr)
{
  std::vector<const ScenePreviewWidget::PreviewItem *> ordered_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> generated_robot_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> physical_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  generated_robot_items.reserve(source_items.size());
  physical_items.reserve(source_items.size());
  overlay_items.reserve(source_items.size());

  for (const auto & item : source_items) {
    const NormalizedRole role = classify_item_role(item);
    if (!show_safety_layer && role == NormalizedRole::SafetyZone) continue;
    const bool generated_or_locked = is_generated_urdf_visual_item(item) || is_locked_urdf_item(item);
    const bool overlay_helper = !generated_or_locked && (is_overlay_only_item(item) || is_overlay_visual_role(role));
    if (scene3d_canonical_link_name_for_item(item) == QStringLiteral("base_link_inertia")) {
      qInfo().noquote() << QStringLiteral("Scene3D base_link_inertia trace: stage=dedupe/final_renderable_assembly item_id=%1 generated_or_locked=%2 overlay_helper=%3 source_layer=%4 active_visual_source=%5")
        .arg(item.id, generated_or_locked ? QStringLiteral("true") : QStringLiteral("false"),
             overlay_helper ? QStringLiteral("true") : QStringLiteral("false"), item.source_layer, item.active_visual_source);
    }
    if (generated_or_locked && generated_urdf_item_has_renderable_geometry(item)) {
      generated_robot_items.push_back(&item);
    } else if (overlay_helper) {
      overlay_items.push_back(&item);
    } else {
      physical_items.push_back(&item);
    }
  }

  QSet<QString> seen_camera_visuals;
  std::vector<const ScenePreviewWidget::PreviewItem *> deduped_generated_robot_items;
  deduped_generated_robot_items.reserve(generated_robot_items.size());
  for (const auto * item : generated_robot_items) {
    if (item && classify_item_role(*item) == NormalizedRole::Camera) {
      const QString key = scene3d_camera_dedupe_key(*item);
      if (seen_camera_visuals.contains(key)) continue;
      seen_camera_visuals.insert(key);
    }
    deduped_generated_robot_items.push_back(item);
  }
  generated_robot_items.swap(deduped_generated_robot_items);

  auto z_sort = [](const auto * a, const auto * b) { return a->z > b->z; };
  std::sort(generated_robot_items.begin(), generated_robot_items.end(), z_sort);
  std::sort(physical_items.begin(), physical_items.end(), z_sort);
  std::sort(overlay_items.begin(), overlay_items.end(), z_sort);

  ordered_items.insert(ordered_items.end(), generated_robot_items.begin(), generated_robot_items.end());
  ordered_items.insert(ordered_items.end(), physical_items.begin(), physical_items.end());
  ordered_items.insert(ordered_items.end(), overlay_items.begin(), overlay_items.end());

  if (out_overlay_items) *out_overlay_items = overlay_items;
  if (out_physical_items) {
    *out_physical_items = generated_robot_items;
    out_physical_items->insert(out_physical_items->end(), physical_items.begin(), physical_items.end());
  }
  return ordered_items;
}


bool is_critical_label_role(NormalizedRole role)
{
  switch (role) {
    case NormalizedRole::RobotBase:
    case NormalizedRole::Camera:
    case NormalizedRole::PickZone:
    case NormalizedRole::PlaceZone:
    case NormalizedRole::PlaceBin:
    case NormalizedRole::SafetyZone:
    case NormalizedRole::HomePose:
      return true;
    default:
      return false;
  }
}
QColor explicit_material_color(const ScenePreviewWidget::PreviewItem & it)
{
  QColor c;
  c.setRgbF(qBound(0.0, it.material_r, 1.0), qBound(0.0, it.material_g, 1.0),
            qBound(0.0, it.material_b, 1.0), qBound(0.0, it.material_a, 1.0));
  return c;
}

QColor product_view_generated_locked_material(const ScenePreviewWidget::PreviewItem & it, bool diagnostic_transparency_mode)
{
  QColor c = it.has_material_color ? explicit_material_color(it) : generated_locked_preview_material();
  const NormalizedRole semantic_role = classify_item_role(it);
  if (!it.has_material_color) {
    if (item_is_tool_or_gripper_visual(it)) {
      c = QColor("#475569");
    } else if (semantic_role == NormalizedRole::Camera || item_identity_contains_any(it, {QStringLiteral("camera"), QStringLiteral("realsense"), QStringLiteral("d435")})) {
      c = QColor("#111827");
    } else if (semantic_role == NormalizedRole::Table) {
      c = QColor("#8b8175");
    } else if (item_is_generated_robot_arm_visual(it)) {
      c = QColor("#d8dee6");
    }
  }
  if (!diagnostic_transparency_mode) {
    c.setAlphaF(qMax(c.alphaF(), kGeneratedLockedProductMinAlpha));
  }
  return c;
}

QColor item_color(const ScenePreviewWidget::PreviewItem & it, bool diagnostic_transparency_mode = false)
{
  if (is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) {
    return product_view_generated_locked_material(it, diagnostic_transparency_mode);
  }
  if (it.has_material_color) {
    return explicit_material_color(it);
  }
  if (it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it)) {
    const NormalizedRole editable_role = classify_item_role(it);
    if (editable_role == NormalizedRole::Table) return QColor("#8b8175");
    if (editable_role == NormalizedRole::Camera) return QColor("#1f2937");
    if (item_is_tool_or_gripper_visual(it)) return QColor("#475569");
    return QColor("#67e8f9");
  }
  switch (classify_item_role(it)) {
    case NormalizedRole::RobotBase: return QColor("#a78bfa");
    case NormalizedRole::RobotReach: return QColor("#60a5fa");
    case NormalizedRole::Table: return QColor("#8b8175");
    case NormalizedRole::Conveyor: return QColor("#06b6d4");
    case NormalizedRole::Camera: return QColor("#1f2937");
    case NormalizedRole::PickZone: return QColor("#22c55e");
    case NormalizedRole::PlaceZone: return QColor("#a855f7");
    case NormalizedRole::PlaceBin: return QColor("#fb7185");
    case NormalizedRole::Object: return QColor("#67e8f9");
    case NormalizedRole::SafetyZone: return QColor("#f59e0b");
    case NormalizedRole::HomePose: return QColor("#e2e8f0");
    case NormalizedRole::WarningAnchor: return QColor("#fbbf24");
    case NormalizedRole::Generic: return QColor("#94a3b8");
  }
  return QColor("#94a3b8");
}

double distance_to_segment_2d(const QPointF & p, const QPointF & a, const QPointF & b)
{
  const QPointF ab = b - a;
  const double ab_len2 = (ab.x() * ab.x()) + (ab.y() * ab.y());
  if (ab_len2 <= 1e-9) return QLineF(p, a).length();
  const QPointF ap = p - a;
  const double t = qBound(0.0, ((ap.x() * ab.x()) + (ap.y() * ab.y())) / ab_len2, 1.0);
  const QPointF c = a + (ab * t);
  return QLineF(p, c).length();
}

double distance_to_polyline_2d(const QPointF & p, const QVector<QPointF> & polyline)
{
  if (polyline.size() < 2) return std::numeric_limits<double>::max();
  double best = std::numeric_limits<double>::max();
  for (int i = 1; i < polyline.size(); ++i) best = qMin(best, distance_to_segment_2d(p, polyline[i - 1], polyline[i]));
  return best;
}

[[maybe_unused]] double wrap_angle_pi(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  double wrapped = std::fmod(angle + M_PI, kTwoPi);
  if (wrapped < 0.0) wrapped += kTwoPi;
  return wrapped - M_PI;
}

QPointF apply_label_overlap_offset(const QPointF & anchor, const QVector<QPointF> & placed_points,
                                  const QVector<QPointF> & robot_base_points, bool critical)
{
  const double min_spacing = critical ? 14.0 : 20.0;
  const double robot_cluster_radius = 82.0;
  const bool near_robot_cluster = std::any_of(robot_base_points.cbegin(), robot_base_points.cend(), [&](const QPointF & base) {
    return QLineF(anchor, base).length() <= robot_cluster_radius;
  });
  QVector<QPointF> candidates{anchor, anchor + QPointF(0.0, -12.0), anchor + QPointF(16.0, -8.0),
                              anchor + QPointF(-16.0, -8.0), anchor + QPointF(0.0, 14.0)};
  if (near_robot_cluster) {
    candidates.push_back(anchor + QPointF(20.0, -18.0));
    candidates.push_back(anchor + QPointF(-20.0, -18.0));
    candidates.push_back(anchor + QPointF(24.0, 4.0));
    candidates.push_back(anchor + QPointF(-24.0, 4.0));
  }

  auto far_enough = [&](const QPointF & candidate) {
    for (const QPointF & placed : placed_points) {
      if (QLineF(candidate, placed).length() < min_spacing) return false;
    }
    return true;
  };

  for (const QPointF & candidate : candidates) {
    if (far_enough(candidate)) return candidate;
  }

  return candidates.back();
}

int label_priority_bucket(bool selected, bool has_warnings, NormalizedRole role)
{
  // LABEL_PRIORITY_SELECTED_WARN_ANCHOR: selected > critical warnings > key anchors > others.
  if (selected) return 0;
  if (has_warnings) return 1;
  if (is_critical_label_role(role)) return 2;
  return 3;
}

struct LabelDrawCandidate
{
  int item_index{ -1 };
  QPointF anchor;
  QPointF base_point;
  QString text;
  QColor color;
  int priority{ 3 };
  bool critical{ false };
};


}


Scene3DViewportWidget::Scene3DViewportWidget(QWidget * parent) : QOpenGLWidget(parent)
{
  setMinimumHeight(420);
  setAcceptDrops(true);
  QSurfaceFormat fmt = format();
  fmt.setDepthBufferSize(qMax(fmt.depthBufferSize(), 24));
  fmt.setStencilBufferSize(qMax(fmt.stencilBufferSize(), 8));
  fmt.setSamples(qMax(fmt.samples(), 4));
  fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
  setFormat(fmt);
}
void Scene3DViewportWidget::reset_view() { set_isometric_view(); }
void Scene3DViewportWidget::set_isometric_view()
{
  yaw_ = -0.78539816339;
  pitch_ = -0.61547970867;
  orbit_offset_ = QVector3D(0.0f, 0.0f, 0.0f);
  distance_ = 6.0;
  update();
}
void Scene3DViewportWidget::set_top_view() { yaw_ = 0.0; pitch_ = -M_PI_2; update(); }
void Scene3DViewportWidget::set_front_view() { yaw_ = M_PI; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::set_side_view() { yaw_ = -M_PI_2; pitch_ = 0.0; update(); }
void Scene3DViewportWidget::invalidate_mesh_cache()
{
  mesh_cache_.clear();
  last_mesh_rejection_reasons_.clear();
  warned_mesh_fallbacks_.clear();
  for (const auto & it : items) {
    const NormalizedRole role = classify_item_role(it);
    const bool generated_or_locked = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool overlay_helper = !generated_or_locked && (is_overlay_only_item(it) || is_overlay_visual_role(role));
    const bool semantic_editor_primitive = !generated_or_locked && is_intentional_semantic_editor_primitive(it);
    if (overlay_helper || semantic_editor_primitive || !item_has_credible_mesh_handoff(it)) continue;
    const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
    if (!mesh_source.trimmed().isEmpty()) ensure_mesh_cached(it, mesh_source);
  }
  last_render_counters.render_cache_count = mesh_cache_.size();
  update();
}
void Scene3DViewportWidget::ingest_preview_items(const QVector<ScenePreviewWidget::PreviewItem> & preview_items)
{
  items.clear();
  items.reserve(preview_items.size());
  for (const auto & item : preview_items) {
    // Selection-owner registry rows exist only in ScenePreviewWidget's
    // identity inventory. They intentionally have no native viewport object,
    // bounds, picking record, transform gizmo, or renderable geometry.
    if (item.source_layer == QStringLiteral("selection_owner_registry")) continue;
    items.push_back(item);
  }
  const bool debug_logs = scene3d_debug_logs_enabled();
  const QString scene_key = scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed();
  int visible_item_count = 0;
  int skipped_item_count = 0;
  int mesh_source_count = 0;
  int urdf_primitive_source_count = 0;
  int locked_urdf_count = 0;
  int editable_layout_count = 0;
  int missing_geometry_warning_count = 0;
  int rejected_mesh_warning_count = 0;
  int transform_chain_failure_warning_count = 0;
  int fallback_item_count = 0;
  QSet<QString> unique_visible_ids;
  QStringList scene_load_warning_tokens;
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  for (const auto & it : items) {
    const NormalizedRole role = classify_item_role(it);
    if (!show_safety && role == NormalizedRole::SafetyZone) { ++skipped_item_count; continue; }
    ++visible_item_count;
    unique_visible_ids.insert(it.id);
    const bool generated_urdf = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool overlay_helper = !generated_urdf && (is_overlay_only_item(it) || is_overlay_visual_role(role));
    const bool semantic_editor_primitive = !generated_urdf && is_intentional_semantic_editor_primitive(it);
    if (scene3d_canonical_link_name_for_item(it) == QStringLiteral("base_link_inertia")) {
      qInfo().noquote() << QStringLiteral("Scene3D base_link_inertia trace: stage=Scene3DViewportWidget_ingest item_id=%1 source_layer=%2 active_visual_source=%3 generated_urdf=%4 overlay_helper=%5 semantic_editor_primitive=%6 mesh_path=%7 source_path=%8")
        .arg(it.id, it.source_layer, it.active_visual_source, generated_urdf ? QStringLiteral("true") : QStringLiteral("false"),
             overlay_helper ? QStringLiteral("true") : QStringLiteral("false"), semantic_editor_primitive ? QStringLiteral("true") : QStringLiteral("false"),
             it.mesh_path.trimmed().isEmpty() ? QStringLiteral("<empty>") : it.mesh_path.trimmed(),
             it.source_path.trimmed().isEmpty() ? QStringLiteral("<empty>") : it.source_path.trimmed());
    }
    const QString generated_mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
    QString generated_cache_result = QStringLiteral("not_attempted");
    if (!overlay_helper && !semantic_editor_primitive && item_has_credible_mesh_handoff(it)) {
      ++mesh_source_count;
      const QString mesh_source = generated_mesh_source;
      if (!mesh_source.trimmed().isEmpty()) {
        const MeshCacheEntry & entry = ensure_mesh_cached(it, mesh_source);
        generated_cache_result = QStringLiteral("loaded=%1 valid=%2 triangles=%3 path_resolved=%4 parser=%5 reason=%6")
          .arg(entry.loaded ? QStringLiteral("true") : QStringLiteral("false"),
               entry.valid ? QStringLiteral("true") : QStringLiteral("false"))
          .arg(static_cast<int>(entry.mesh.triangles.size()))
          .arg(entry.path_resolved ? QStringLiteral("true") : QStringLiteral("false"),
               entry.parser_type.trimmed().isEmpty() ? QStringLiteral("<none>") : entry.parser_type,
               !entry.failure_reason_code.trimmed().isEmpty() ? entry.failure_reason_code.trimmed() :
                 (!entry.warning.trimmed().isEmpty() ? entry.warning.trimmed() : QStringLiteral("ok")));
        if (!entry.valid || !entry.warning.trimmed().isEmpty()) {
          ++rejected_mesh_warning_count;
          const QString reason = !entry.failure_reason_code.trimmed().isEmpty()
            ? entry.failure_reason_code.trimmed()
            : (!entry.warning.trimmed().isEmpty() ? entry.warning.trimmed() : QStringLiteral("mesh_rejected"));
          scene_load_warning_tokens << QStringLiteral("rejected_mesh:%1:%2").arg(it.id, reason);
        }
      }
    } else if (generated_mesh_source.trimmed().isEmpty()) {
      generated_cache_result = QStringLiteral("skipped:no_mesh_path");
    } else if (overlay_helper) {
      generated_cache_result = QStringLiteral("skipped:overlay_helper");
    } else if (semantic_editor_primitive) {
      generated_cache_result = QStringLiteral("skipped:semantic_editor_primitive");
    } else {
      generated_cache_result = QStringLiteral("skipped:no_credible_mesh_handoff");
    }
    if (generated_urdf) {
      const QString skip_reason = !show_safety && role == NormalizedRole::SafetyZone ? QStringLiteral("safety_layer_hidden") :
        (overlay_helper ? QStringLiteral("overlay_helper") :
          (semantic_editor_primitive ? QStringLiteral("semantic_editor_primitive") :
            (!item_has_credible_mesh_handoff(it) && !item_has_valid_urdf_primitive(it) ? QStringLiteral("no_renderable_geometry") : QStringLiteral("none"))));
      qInfo().noquote() << QStringLiteral(
        "Scene3D renderer generated_urdf ingest: id=%1 link=%2 canonical_link=%3 source_layer=%4 active_visual_source=%5 type=%6 category=%7 mesh_path=%8 package_uri=%9 cache=%10 submitted_to_draw=%11 skip_reason=%12")
        .arg(it.id,
             scene3d_link_name_for_item(it),
             scene3d_canonical_link_name_for_item(it),
             it.source_layer,
             it.active_visual_source,
             it.primitive_geometry_type.trimmed().isEmpty() ? QStringLiteral("<none>") : it.primitive_geometry_type.trimmed(),
             it.category.trimmed().isEmpty() ? QStringLiteral("<none>") : it.category.trimmed(),
             generated_mesh_source.trimmed().isEmpty() ? QStringLiteral("<none>") : generated_mesh_source.trimmed(),
             it.package_uri.trimmed().isEmpty() ? QStringLiteral("<none>") : it.package_uri.trimmed(),
             generated_cache_result,
             skip_reason == QStringLiteral("none") ? QStringLiteral("yes") : QStringLiteral("no"),
             skip_reason);
    }
    if (!overlay_helper && generated_urdf && item_has_valid_urdf_primitive(it)) ++urdf_primitive_source_count;
    if (!overlay_helper && !semantic_editor_primitive &&
        !item_has_mesh_surface_candidate(it) && !item_has_explicit_primitive_dimensions(it)) {
      ++missing_geometry_warning_count;
      scene_load_warning_tokens << QStringLiteral("missing_geometry:%1").arg(it.id);
    }
    const QString layer_token = (it.source_layer + QStringLiteral("|") + it.active_visual_source).toLower();
    if (!overlay_helper && layer_token.contains(QStringLiteral("fallback"))) ++fallback_item_count;
    if (!it.transform_chain_applied) {
      const QString warning_text = (it.status + QStringLiteral("|") + it.warnings.join(QLatin1Char('|'))).toLower();
      if (warning_text.contains(QStringLiteral("transform")) ||
          warning_text.contains(QStringLiteral("urdf chain")) ||
          warning_text.contains(QStringLiteral("broken chain")) ||
          warning_text.contains(QStringLiteral("missing_chain"))) {
        ++transform_chain_failure_warning_count;
        scene_load_warning_tokens << QStringLiteral("transform_chain_failure:%1").arg(it.id);
      }
    }
    for (const QString & warning : it.warnings) {
      const QString trimmed = warning.trimmed();
      if (!trimmed.isEmpty()) scene_load_warning_tokens << QStringLiteral("item_warning:%1:%2").arg(it.id, trimmed);
    }
    if (!it.mesh_load_warning.trimmed().isEmpty()) {
      scene_load_warning_tokens << QStringLiteral("mesh_warning:%1:%2").arg(it.id, it.mesh_load_warning.trimmed());
    }
    if (!it.alignment_warning.trimmed().isEmpty()) {
      scene_load_warning_tokens << QStringLiteral("alignment_warning:%1:%2").arg(it.id, it.alignment_warning.trimmed());
    }
    if (generated_urdf) ++locked_urdf_count;
    if (it.linked_to_editable_layout_state) ++editable_layout_count;
    if (overlay_helper) overlay_items.push_back(&it);
  }
  const int physical_visible_count = qMax(0, visible_item_count - static_cast<int>(overlay_items.size()));
  if (physical_visible_count > 0 && fallback_item_count * 2 >= physical_visible_count) {
    scene_load_warning_tokens << QStringLiteral("fallback_dominant:%1/%2").arg(fallback_item_count).arg(physical_visible_count);
  }
  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = items.size();
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = 0;
  last_render_counters.mesh_surface_rendered_count = 0;
  last_render_counters.mesh_bounds_fallback_rendered_count = 0;
  last_render_counters.mesh_path_resolved_count = 0;
  last_render_counters.mesh_file_loaded_count = 0;
  last_render_counters.mesh_triangles_loaded_count = 0;
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = 0;
  last_render_counters.overlay_helper_count = static_cast<int>(overlay_items.size());
  last_render_counters.overlay_count = static_cast<int>(overlay_items.size());
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  populate_runtime_transform_counters(last_render_counters, items);
  last_render_counters.editable_layout_count = editable_layout_count;
  last_render_counters.hierarchy_child_row_count = visible_item_count;
  last_render_counters.last_paint_completed = false;
  last_render_counters.smoke_fallback_render_used = false;
  finalize_visual_quality(last_render_counters);
  scene_load_warning_tokens.append(last_render_counters.visual_quality_warnings);
  scene_load_warning_tokens.removeDuplicates();
  scene_load_warning_tokens.sort();
  const QString warning_signature = scene_load_warning_tokens.join(QLatin1Char('|'));
  const bool should_emit_scene_load_summary =
    debug_logs ||
    last_scene_load_summary_item_count_ != items.size() ||
    last_scene_load_summary_scene_name_ != scene_key ||
    last_scene_load_summary_warning_signature_ != warning_signature;
  if (should_emit_scene_load_summary) {
    last_scene_load_summary_item_count_ = items.size();
    last_scene_load_summary_scene_name_ = scene_key;
    last_scene_load_summary_warning_signature_ = warning_signature;
    qInfo().noquote() << QStringLiteral(
      "Scene3D scene load: scene=%1 received=%2 visible=%3 mesh_sources=%4 urdf_primitives=%5 overlays=%6 mesh_cache=%7 baked_world_visual_transform_count=%8 legacy_viewport_transform_count=%9 warnings=%10")
      .arg(scene_key)
      .arg(last_render_counters.viewport_received_count)
      .arg(last_render_counters.visible_count)
      .arg(last_render_counters.mesh_source_count)
      .arg(last_render_counters.urdf_primitive_source_count)
      .arg(last_render_counters.overlay_count)
      .arg(last_render_counters.render_cache_count)
      .arg(last_render_counters.baked_world_visual_transform_count)
      .arg(last_render_counters.legacy_viewport_transform_count)
      .arg(scene_load_warning_tokens.isEmpty() ? QStringLiteral("none") : scene_load_warning_tokens.join(QLatin1Char(',')));
  }
  if (should_emit_scene_load_summary && !scene_load_warning_tokens.isEmpty()) {
    qWarning().noquote() << QStringLiteral(
      "Scene3D scene-load warnings: scene=%1 missing_geometry=%2 rejected_meshes=%3 transform_chain_failures=%4 fallback_items=%5/%6 details=%7")
      .arg(scene_key)
      .arg(missing_geometry_warning_count)
      .arg(rejected_mesh_warning_count)
      .arg(transform_chain_failure_warning_count)
      .arg(fallback_item_count)
      .arg(physical_visible_count)
      .arg(scene_load_warning_tokens.join(QLatin1Char(',')));
  }
  const QString diagnostics_path = QString::fromUtf8(qgetenv("SCENE3D_MESH_DIAGNOSTICS_JSON"));
  if (!diagnostics_path.trimmed().isEmpty()) {
    QFile out_file(diagnostics_path);
    if (out_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
      QJsonObject root;
      root["camera_fit_target"] = last_camera_fit_target_;
      root["scene_radius"] = scene_radius_;
      root["camera_distance"] = distance_;
      root["camera_fit_margin"] = last_camera_fit_margin_;
      root["camera_fit_margin_value"] = last_camera_fit_margin_value_;
      root["camera_fit_bounds_min"] = QJsonArray{last_camera_fit_bounds_min_.x(), last_camera_fit_bounds_min_.y(), last_camera_fit_bounds_min_.z()};
      root["camera_fit_bounds_max"] = QJsonArray{last_camera_fit_bounds_max_.x(), last_camera_fit_bounds_max_.y(), last_camera_fit_bounds_max_.z()};
      root["camera_fit_bounds_span"] = QJsonArray{last_camera_fit_bounds_span_.x(), last_camera_fit_bounds_span_.y(), last_camera_fit_bounds_span_.z()};
      root["initial_fit_included_robot_bounds"] = last_initial_fit_included_robot_bounds_;
      root["initial_fit_physical_anchor_count"] = last_initial_fit_physical_anchor_count_;
      root["initial_fit_anchor_roles"] = QJsonArray::fromStringList(last_initial_fit_anchor_roles_);
      root["initial_fit_audit_token"] = last_initial_fit_included_robot_bounds_
        ? QStringLiteral("ROBOT_BOUNDS_INCLUDED_IN_INITIAL_FIT")
        : QStringLiteral("ROBOT_BOUNDS_NOT_INCLUDED_IN_INITIAL_FIT");
      root["initial_fit_included_ur5_bounds"] = last_initial_fit_included_robot_bounds_;
      root["initial_fit_ur5_bounds_transitional"] = QStringLiteral("transitional compatibility key; use initial_fit_included_robot_bounds");
      if (has_robot_aabb_diag_) {
        root["robot_aabb_min"] = QJsonArray{last_robot_aabb_min_.x(), last_robot_aabb_min_.y(), last_robot_aabb_min_.z()};
        root["robot_aabb_max"] = QJsonArray{last_robot_aabb_max_.x(), last_robot_aabb_max_.y(), last_robot_aabb_max_.z()};
      }
      root["mesh_diagnostics"] = mesh_diagnostics_export();
      root["generated_robot_final_draw_candidate_diagnostics"] = generated_robot_final_draw_candidate_diagnostics_export();
      root["ur5_final_draw_candidate_diagnostics"] = root["generated_robot_final_draw_candidate_diagnostics"];
      root["final_draw_visual_items"] = final_draw_visual_items_export();
      QJsonObject render_debug;
      const auto counters = render_debug_counters();
      render_debug["locked_generated_urdf_visual_count"] = counters.locked_generated_urdf_visual_count;
      render_debug["transform_chain_applied_count"] = counters.transform_chain_applied_count;
      render_debug["visual_origin_applied_count"] = counters.visual_origin_applied_count;
      render_debug["baked_world_visual_transform_count"] = counters.baked_world_visual_transform_count;
      render_debug["legacy_viewport_transform_count"] = counters.legacy_viewport_transform_count;
      root["render_debug_counters"] = render_debug;
      out_file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
      out_file.close();
    }
  }
  update();
}
void Scene3DViewportWidget::fit_scene() {
  QVector3D bmin, bmax;
  if (!scene_bounds_from_visible_items(bmin, bmax, fit_include_overlays)) {
    set_isometric_view();
    return;
  }
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 0.95;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  last_camera_fit_bounds_min_ = bmin;
  last_camera_fit_bounds_max_ = bmax;
  last_camera_fit_bounds_span_ = ext;
  last_camera_fit_margin_ = QStringLiteral("scene: geometric_fov_distance * 0.95");
  last_camera_fit_margin_value_ = 0.95;
  pitch_ = qBound(-0.9, pitch_, -0.28);
  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.10, radius * 0.05)));
  if (fit_include_overlays) {
    last_camera_fit_target_ = QStringLiteral("scene_with_overlays");
  } else {
    last_camera_fit_target_ = QStringLiteral("scene");
  }
  has_robot_aabb_diag_ = false;
  update();
}

void Scene3DViewportWidget::fit_product_view()
{
  fit_include_overlays = false;

  QVector3D bmin, bmax;
  bool robot_included = false;
  int anchor_count = 0;
  QStringList anchor_roles;
  if (!initial_physical_fit_bounds(bmin, bmax, &robot_included, &anchor_count, &anchor_roles) &&
      !scene_bounds_from_visible_items(bmin, bmax, false)) {
    set_isometric_view();
    last_initial_fit_included_robot_bounds_ = false;
    last_initial_fit_physical_anchor_count_ = 0;
    last_initial_fit_anchor_roles_.clear();
    return;
  }

  const double fov = qDegreesToRadians(50.0);
  // Initial product framing is intentionally physical-only: generated robot links,
  // generated tool/gripper links, table/workbench, camera body, and authored physical environment items.
  // Overlay-only FOV/reachability/collision bounds remain excluded unless the
  // explicit overlay fit action sets fit_include_overlays for fit_scene().
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D product_ext = bmax - bmin;
  const double product_radius = qMax(0.25, 0.5 * qSqrt(product_ext.x() * product_ext.x() + product_ext.y() * product_ext.y() + product_ext.z() * product_ext.z()));
  scene_radius_ = product_radius;
  const double base_fit_distance = product_radius / qTan(fov * 0.5);
  const double fit_distance = qMax(qMax(base_fit_distance * 2.4, product_radius * 5.0), 4.0);
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  last_camera_fit_bounds_min_ = bmin;
  last_camera_fit_bounds_max_ = bmax;
  last_camera_fit_bounds_span_ = product_ext;
  last_camera_fit_margin_ = QStringLiteral("product: max(base_fit_distance * 2.4, product_radius * 5.0, 4.0)");
  last_camera_fit_margin_value_ = fit_distance;
  yaw_ = -0.86;
  pitch_ = -0.60;
  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.06, product_radius * 0.035)));
  last_initial_fit_included_robot_bounds_ = robot_included;
  last_initial_fit_physical_anchor_count_ = anchor_count;
  last_initial_fit_anchor_roles_ = anchor_roles;
  last_camera_fit_target_ = robot_included
    ? QStringLiteral("product_physical_initial_fit_robot_included")
    : QStringLiteral("product_physical_initial_fit_robot_absent");
  if (robot_included) {
    qInfo().noquote() << QStringLiteral("Scene3D initial fit: ROBOT_BOUNDS_INCLUDED_IN_INITIAL_FIT target=%1 scene=%2 anchors=%3 roles=%4")
      .arg(last_camera_fit_target_,
           scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed())
      .arg(anchor_count)
      .arg(anchor_roles.isEmpty() ? QStringLiteral("none") : anchor_roles.join(QStringLiteral(",")));
  }
  has_robot_aabb_diag_ = false;
  fit_include_overlays = false;
  update();
}
void Scene3DViewportWidget::fit_robot()
{
  QVector3D bmin, bmax;
  if (!robot_bounds_from_rendered_visuals(bmin, bmax)) {
    fit_scene();
    return;
  }
  orbit_offset_ = (bmin + bmax) * 0.5f;
  const QVector3D ext = bmax - bmin;
  const double radius = qMax(0.2, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));
  scene_radius_ = radius;
  const double fov = qDegreesToRadians(50.0);
  const double fit_distance = (radius / qTan(fov * 0.5)) * 1.05;
  distance_ = qBound(min_distance_, fit_distance, max_distance_);
  pitch_ = qBound(-0.9, pitch_, -0.28);
  last_camera_fit_target_ = QStringLiteral("robot");
  has_robot_aabb_diag_ = true;
  last_robot_aabb_min_ = bmin;
  last_robot_aabb_max_ = bmax;
  update();
}
void Scene3DViewportWidget::focus_selected() {
  for (const auto & it : items) {
    if (it.id != selected_id) continue;
    const ItemBounds b = item_bounds_for_role(it);
    orbit_offset_ = QVector3D(static_cast<float>(b.x + b.sx * 0.5), static_cast<float>(b.y + b.sy * 0.5), static_cast<float>(b.z + b.sz * 0.5));
    scene_radius_ = qMax(0.2, 0.5 * qSqrt(b.sx * b.sx + b.sy * b.sy + b.sz * b.sz));
    distance_ = qBound(min_distance_, scene_radius_ * 4.0, max_distance_);
    update();
    return;
  }
  fit_scene();
}
void Scene3DViewportWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glShadeModel(GL_SMOOTH);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_NORMALIZE);
  glClearColor(0.015f, 0.023f, 0.045f, 1.0f);
}
void Scene3DViewportWidget::resizeGL(int w, int h) { glViewport(0, 0, w, h); }

void Scene3DViewportWidget::configure_product_render_state()
{
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_SMOOTH);

  const GLfloat ambient[] = { 0.28f, 0.31f, 0.36f, 1.0f };
  const GLfloat key_diffuse[] = { 0.82f, 0.90f, 1.0f, 1.0f };
  const GLfloat fill_diffuse[] = { 0.33f, 0.41f, 0.52f, 1.0f };
  const GLfloat key_position[] = { 4.0f, 7.0f, 5.0f, 0.0f };
  const GLfloat fill_position[] = { -6.0f, 3.5f, -4.0f, 0.0f };
  glEnable(GL_LIGHTING);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, key_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, key_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, key_position);
  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, fill_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, fill_diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, fill_position);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  const GLfloat specular[] = { 0.20f, 0.24f, 0.30f, 1.0f };
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 28.0f);
}

void Scene3DViewportWidget::draw_viewport_quality_overlay(QPainter & painter, int visible_item_count, int physical_item_count) const
{
  const QRectF card(width() - 218.0, 12.0, 206.0, 82.0);
  if (card.left() < 12.0) return;
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(2, 6, 23, debug_overlays_mode ? 212 : 172));
  painter.drawRoundedRect(card, 10.0, 10.0);

  painter.setPen(QColor("#bfdbfe"));
  painter.drawText(card.adjusted(12.0, 8.0, -12.0, -56.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("Digital twin viewport"));
  painter.setPen(QColor("#cbd5e1"));
  painter.drawText(card.adjusted(12.0, 28.0, -12.0, -36.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("MSAA · lit meshes · depth sorted"));
  painter.setPen(QColor("#94a3b8"));
  painter.drawText(card.adjusted(12.0, 48.0, -12.0, -16.0), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 visible · %2 physical").arg(visible_item_count).arg(physical_item_count));

  const QPointF origin(card.right() - 46.0, card.bottom() - 22.0);
  const QPointF x_axis(origin.x() + 22.0, origin.y());
  const QPointF y_axis(origin.x() - 10.0, origin.y() - 18.0);
  const QPointF z_axis(origin.x(), origin.y() - 26.0);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(QColor("#ef4444"), 2.0));
  painter.drawLine(origin, x_axis);
  painter.drawText(x_axis + QPointF(2.0, 4.0), QStringLiteral("X"));
  painter.setPen(QPen(QColor("#22c55e"), 2.0));
  painter.drawLine(origin, y_axis);
  painter.drawText(y_axis + QPointF(-10.0, -2.0), QStringLiteral("Y"));
  painter.setPen(QPen(QColor("#38bdf8"), 2.0));
  painter.drawLine(origin, z_axis);
  painter.drawText(z_axis + QPointF(2.0, -2.0), QStringLiteral("Z"));
}

void Scene3DViewportWidget::paintGL()
{
  static const char * kPaintGLCacheOnlyGuard = "paintGL cache-only guard: no YAML/file IO in paint path";
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.constData());
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.constData());

  draw_ground_grid_pass();
  draw_world_axes_pass();
  configure_product_render_state();
  if (debug_overlays_mode || fit_include_overlays) {
    draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.01, QColor(34, 197, 94, 26), true);
    draw_cylinder(-0.2, 0.0, -2.2, reach_overlay.preferred_work_zone_radius_m, 0.005, QColor(34, 197, 94, 110), false);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  std::vector<const ScenePreviewWidget::PreviewItem *> overlay_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> physical_items;
  std::vector<const ScenePreviewWidget::PreviewItem *> draw_items =
    build_final_generated_urdf_robot_renderables(items, show_safety, &overlay_items, &physical_items);

  int received_item_count = static_cast<int>(draw_items.size());
  int visible_item_count = 0;
  int skipped_item_count = qMax(0, static_cast<int>(items.size()) - received_item_count);
  int rendered_item_count = 0;
  int mesh_source_count = 0;
  int mesh_rendered_count = 0;
  int mesh_surface_rendered_count = 0;
  int urdf_primitive_source_count = 0;
  int urdf_primitive_rendered_count = 0;
  int primitive_fallback_count = 0;
  int editable_primitive_count = 0;
  int placeholder_count = 0;
  int missing_geometry_count = 0;
  int generated_missing_geometry_count = 0;
  int wireframe_box_count = 0;
  int generated_mesh_bounds_fallback_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int physical_item_count = 0;
  QSet<QString> unique_visible_ids;
  int editable_layout_count = 0;
  last_render_counters = RenderDebugCounters{};
  for (const auto * it : draw_items) {
    const NormalizedRole role = classify_item_role(*it);
    ++visible_item_count;
    unique_visible_ids.insert(it->id);
    const bool generated_urdf = is_generated_urdf_visual_item(*it) || is_locked_urdf_item(*it);
    const bool overlay_helper = is_overlay_only_item(*it) || is_overlay_visual_role(role);
    if (generated_urdf) ++locked_urdf_count;
    if (it->linked_to_editable_layout_state) ++editable_layout_count;
    if (!overlay_helper) {
      ++physical_item_count;
      ++last_render_counters.physical_anchor_count;
      if (item_is_generated_robot_arm_visual(*it) && item_has_credible_mesh_handoff(*it)) ++last_render_counters.generated_robot_mesh_count;
      if (item_is_tool_or_gripper_visual(*it)) ++last_render_counters.tool_gripper_visual_count;
      if (role == NormalizedRole::Table) ++last_render_counters.table_workbench_visual_count;
      if (role == NormalizedRole::Camera) ++last_render_counters.camera_body_visual_count;
      if (!is_intentional_semantic_editor_primitive(*it) && item_has_credible_mesh_handoff(*it)) ++mesh_source_count;
      if (generated_urdf && item_has_valid_urdf_primitive(*it)) {
        ++urdf_primitive_source_count;
      }
    }
  }
  overlay_count = static_cast<int>(overlay_items.size());
  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = received_item_count;
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.overlay_helper_count = overlay_count;
  last_render_counters.overlay_count = overlay_count;
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  populate_runtime_transform_counters(last_render_counters, items);
  last_render_counters.editable_layout_count = editable_layout_count;
  int visible_hierarchy_items = 0;
  for (const auto * it : draw_items) {
    if (!it) continue;
    ++visible_hierarchy_items;
  }
  last_render_counters.hierarchy_child_row_count = visible_hierarchy_items;

  auto draw_item_batch = [&](const std::vector<const ScenePreviewWidget::PreviewItem *> & batch, bool count_in_stats) {
    for (const auto * it : batch) {
      int item_placeholder_count = 0;
      int item_mesh_backed_count = 0;
      int item_wireframe_box_count = 0;
      int item_urdf_primitive_count = 0;
      int item_missing_geometry_count = 0;
      int item_primitive_fallback_count = 0;
      int item_editable_primitive_count = 0;
      const bool drew_physical_geometry =
        draw_truthful_item_geometry(*it, &item_placeholder_count, &item_mesh_backed_count, &item_wireframe_box_count,
                                    &item_urdf_primitive_count, &item_missing_geometry_count, &item_primitive_fallback_count,
                                    &item_editable_primitive_count);
      if (drew_physical_geometry) ++rendered_item_count;
      if (count_in_stats) {
        placeholder_count += item_placeholder_count;
        mesh_rendered_count += item_mesh_backed_count;
        mesh_surface_rendered_count += item_mesh_backed_count;
        urdf_primitive_rendered_count += item_urdf_primitive_count;
        missing_geometry_count += item_missing_geometry_count;
        wireframe_box_count += item_wireframe_box_count;
        primitive_fallback_count += item_primitive_fallback_count;
        editable_primitive_count += item_editable_primitive_count;
        const bool item_generated_or_locked = is_generated_urdf_visual_item(*it) || is_locked_urdf_item(*it);
        if (item_generated_or_locked && !is_intentional_semantic_editor_primitive(*it) &&
            item_has_credible_mesh_handoff(*it) && item_mesh_backed_count <= 0) {
          ++generated_mesh_bounds_fallback_count;
        }
        if (item_generated_or_locked) generated_missing_geometry_count += item_missing_geometry_count;
      } else {
        missing_geometry_count += item_missing_geometry_count;
        primitive_fallback_count += item_primitive_fallback_count;
        editable_primitive_count += item_editable_primitive_count;
      }
      if (it->id == selected_id) {
        const ItemBounds bounds = item_bounds_for_role(*it);
        const bool editable = item_is_editable_for_gizmo(*it);
        const bool selected_generated = is_generated_urdf_visual_item(*it) || is_locked_urdf_item(*it);
        const QColor selection_outline = editable ? editable_layout_selected_highlight()
          : (selected_generated ? generated_locked_preview_outline() : QColor("#94a3b8"));
        draw_box_outline(bounds.x, bounds.y, bounds.z, bounds.sx, bounds.sy, bounds.sz, selection_outline, editable ? 3.0f : 1.4f);
      }
    }
  };  // draw_item_batch
  draw_item_batch(overlay_items, false);  // draw translucent overlays before solids to keep robot/table meshes legible.
  draw_item_batch(physical_items, true);
  last_render_counters.rendered_count = rendered_item_count;
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = mesh_rendered_count;
  if (mesh_surface_rendered_count <= 0 && mesh_rendered_count > 0) mesh_surface_rendered_count = mesh_rendered_count;
  last_render_counters.mesh_surface_rendered_count = mesh_surface_rendered_count;
  last_render_counters.mesh_bounds_fallback_rendered_count = scene3d_debug_fallback_boxes_enabled() ? qMax(0, mesh_source_count - mesh_surface_rendered_count) : 0;
  last_render_counters.generated_mesh_bounds_fallback_rendered_count = scene3d_debug_fallback_boxes_enabled() ? generated_mesh_bounds_fallback_count : 0;
  last_render_counters.mesh_path_resolved_count = 0;
  last_render_counters.mesh_file_loaded_count = 0;
  last_render_counters.mesh_triangles_loaded_count = 0;
  for (auto cache_it = mesh_cache_.cbegin(); cache_it != mesh_cache_.cend(); ++cache_it) {
    const MeshCacheEntry & cache_entry = cache_it.value();
    if (cache_entry.path_resolved) ++last_render_counters.mesh_path_resolved_count;
    if (cache_entry.loaded && cache_entry.valid && !cache_entry.mesh.triangles.isEmpty()) {
      ++last_render_counters.mesh_file_loaded_count;
      last_render_counters.mesh_triangles_loaded_count += static_cast<int>(cache_entry.mesh.triangles.size());
    }
  }
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = urdf_primitive_rendered_count;
  last_render_counters.placeholder_count = placeholder_count;
  last_render_counters.missing_geometry_count = missing_geometry_count;
  last_render_counters.generated_missing_geometry_count = generated_missing_geometry_count;
  last_render_counters.wireframe_fallback_count = wireframe_box_count;
  last_render_counters.primitive_fallback_rendered_count = primitive_fallback_count;
  last_render_counters.primitive_fallback_count = primitive_fallback_count;
  last_render_counters.editable_primitive_rendered_count = editable_primitive_count;
  last_render_counters.valid_physical_fallback_count = primitive_fallback_count;
  last_render_counters.overlay_rendered_count = overlay_count;
  last_render_counters.last_paint_completed = true;
  finalize_visual_quality(last_render_counters);
  last_render_counters.smoke_fallback_render_used = false;

  glDisable(GL_BLEND);
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);

  if (scene3d_debug_logs_enabled()) {
    qDebug() << "Scene3D runtime render: received=" << received_item_count
             << "visible=" << visible_item_count
             << "rendered=" << rendered_item_count
             << "skipped=" << skipped_item_count
             << "mesh_sources=" << mesh_source_count
             << "placeholder=" << placeholder_count
             << "overlay=" << overlay_count
             << "primitive_fallback_rendered_count=" << last_render_counters.primitive_fallback_rendered_count
             << "editable_primitive_rendered_count=" << last_render_counters.editable_primitive_rendered_count
             << "mesh_rendered_count=" << last_render_counters.mesh_rendered_count
             << "mesh_surface_rendered_count=" << last_render_counters.mesh_surface_rendered_count
             << "mesh_bounds_fallback_rendered_count=" << last_render_counters.mesh_bounds_fallback_rendered_count
             << "mesh_file_loaded_count=" << last_render_counters.mesh_file_loaded_count
             << "mesh_triangles_loaded_count=" << last_render_counters.mesh_triangles_loaded_count
             << "generated_fallback_count=" << last_render_counters.generated_fallback_count
             << "labels_drawn=" << last_render_counters.labels_drawn
             << "labels_suppressed_overlap=" << last_render_counters.labels_suppressed_overlap
             << "hierarchy_child_row_count=" << last_render_counters.hierarchy_child_row_count;
    qDebug() << kPaintGLCacheOnlyGuard;
    qDebug() << "Scene3D diagnostics {viewport_received_count=" << received_item_count
             << ", render_cache_count=" << mesh_cache_.size()
             << ", rendered_count=" << rendered_item_count
             << ", skipped_count=" << skipped_item_count
             << "}";
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  if (visible_item_count == 0) {
    painter.setPen(QColor("#f8fafc"));
    painter.drawText(rect(), Qt::AlignCenter,
      "Scene3D empty state\nNo visible candidate visuals.\nCheck preview layer visibility and scene source files.");
    if (status_message_cb) {
      status_message_cb("Scene3D blocker: no visible candidate visuals after ingestion and layer filtering.");
    }
  }
  const auto compact_role = [](const QString & role) {
    const QString r = role.trimmed().toLower();
    if (r.contains("pick")) return QStringLiteral("Pick");
    if (r.contains("place")) return QStringLiteral("Place");
    if (r.contains("camera")) return QStringLiteral("Cam");
    if (r.contains("safety")) return QStringLiteral("Safe");
    if (r.isEmpty()) return QStringLiteral("Item");
    return role.left(10);
  };
  const double zoom_factor = distance_ / qMax(0.25, scene_radius_);
  const bool crowded = items.size() > 30;
  const bool zoomed_far = zoom_factor > 5.2;
  const bool suppress_dense_non_critical_labels = crowded || zoomed_far;
  QVector<QPointF> robot_base_points;
  for (const auto & it : items) {
    if (classify_item_role(it) != NormalizedRole::RobotBase) continue;
    const ItemBounds bounds = item_bounds_for_role(it);
    robot_base_points.push_back(project_to_screen(bounds.x + (bounds.sx * 0.5), bounds.y + (bounds.sy * 0.5), bounds.z + (bounds.sz * 0.5)));
  }
  QVector<LabelDrawCandidate> label_candidates;
  for (int i = 0; i < items.size(); ++i) {
    const auto & it = items[i];
    const ItemBounds bounds = item_bounds_for_role(it);
    const QPointF p = project_to_screen(bounds.x + (bounds.sx * 0.5), bounds.y + (bounds.sy * 0.5), bounds.z + (bounds.sz * 0.5));
    const bool selected = (it.id == selected_id);
    const NormalizedRole role = classify_item_role(it);
    const ScenePreviewWidget::LabelMode effective_label_mode = label_mode;
    bool draw_label = false;
    switch (effective_label_mode) {
      case ScenePreviewWidget::LabelMode::Off: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::Important: draw_label = selected || is_critical_label_role(role); break;
      case ScenePreviewWidget::LabelMode::Selected: draw_label = selected; break;
      case ScenePreviewWidget::LabelMode::All: draw_label = true; break;
    }
    const bool is_urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    if (suppress_dense_non_critical_labels && !selected && !is_critical_label_role(role)) draw_label = false;
    if (is_urdf_visual && !selected && effective_label_mode != ScenePreviewWidget::LabelMode::All) draw_label = false;
    const bool render_counters_clean =
      last_render_counters.missing_geometry_count == 0 &&
      last_render_counters.generated_fallback_count == 0 &&
      last_render_counters.wireframe_fallback_count == 0;
    const bool draw_warning_badges =
      show_warning_labels && (debug_overlays_mode || !render_counters_clean);
    if (draw_warning_badges && !it.warnings.isEmpty()) {
      if (debug_overlays_mode && !it.warnings.isEmpty()) {
        const QString debug_warning_text = warning_debug_text(it.warnings);
        Q_UNUSED(debug_warning_text);
      }
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor("#f59e0b"));
      painter.drawEllipse(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0));
      painter.setPen(QColor("#111827"));
      painter.drawText(QRectF(p.x() - 7.0, p.y() - 18.0, 14.0, 14.0), Qt::AlignCenter, warning_badge_text(it.warnings));
    }
    if (!draw_label) continue;
    const QString missing_reason = placeholder_reason_for_item(it);
    const bool is_critical_scene_anchor = (role == NormalizedRole::RobotBase || role == NormalizedRole::Table || role == NormalizedRole::Camera);
    if (!selected && is_urdf_visual && missing_reason.isEmpty() && !is_critical_scene_anchor &&
        effective_label_mode != ScenePreviewWidget::LabelMode::All) continue;

    const QString compact_text = clean_label_from_item(it);
    const QString ownership_text = item_visual_ownership_label(it);
    const QString text = selected ? QStringLiteral("%1 • %2").arg(compact_text, ownership_text)
                                  : (missing_reason.isEmpty() ? compact_text : QString("%1 missing").arg(compact_role(it.role)));
    LabelDrawCandidate candidate;
    candidate.item_index = i;
    candidate.base_point = p;
    candidate.anchor = QPointF(p.x() + 12.0, p.y() - 10.0);
    candidate.text = text;
    candidate.color = QColor("#e2e8f0");
    candidate.critical = is_critical_label_role(role);
    candidate.priority = label_priority_bucket(selected, !it.warnings.isEmpty(), role);
    label_candidates.push_back(candidate);
  }

  std::stable_sort(label_candidates.begin(), label_candidates.end(), [](const LabelDrawCandidate & a, const LabelDrawCandidate & b) {
    return a.priority < b.priority;
  });

  QVector<QRectF> placed_label_boxes;
  QVector<QPointF> placed_label_points;
  constexpr int kLowPriorityOverlapBudget = 6;
  int overlap_budget_hits = 0;
  bool low_priority_overlap_budget_exhausted = false;
  int labels_drawn = 0;
  int labels_suppressed_overlap = 0;
  for (const auto & candidate : label_candidates) {
    const bool low_priority = candidate.priority >= 3;
    if (low_priority_overlap_budget_exhausted && low_priority) {
      ++labels_suppressed_overlap;
      continue;
    }
    const QPointF label_pos = apply_label_overlap_offset(candidate.anchor, placed_label_points, robot_base_points, candidate.critical);
    const QRectF label_rect = painter.boundingRect(QRectF(label_pos.x() - 2.0, label_pos.y() - 14.0, 220.0, 18.0), Qt::AlignLeft | Qt::AlignVCenter, candidate.text);

    bool overlaps = false;
    for (const QRectF & existing_rect : placed_label_boxes) {
      if (label_rect.adjusted(-2.0, -1.0, 2.0, 1.0).intersects(existing_rect)) {
        overlaps = true;
        break;
      }
    }
    // LABEL_OVERLAP_SUPPRESS_LOWER_PRIORITY: keep high priority, suppress lower when overlap remains.
    if (overlaps) {
      ++labels_suppressed_overlap;
      if (low_priority && (++overlap_budget_hits > kLowPriorityOverlapBudget)) low_priority_overlap_budget_exhausted = true;
      continue;
    }

    placed_label_points.push_back(label_pos);
    placed_label_boxes.push_back(label_rect);
    painter.setPen(candidate.color);
    painter.drawText(label_pos, candidate.text);
    ++labels_drawn;
  }
  last_render_counters.labels_drawn = labels_drawn;
  last_render_counters.labels_suppressed_overlap = labels_suppressed_overlap;
  const bool has_missing_or_fallback_content = debug_overlays_mode
    ? debug_view_has_full_diagnostic_warning_content(last_render_counters)
    : product_view_has_generated_mesh_warning_content(last_render_counters);
  const bool concise_warning = show_warnings && has_missing_or_fallback_content;
  const QRectF overlay_rect = debug_overlays_mode ? QRectF(12.0, 12.0, 520.0, 88.0)
                                                  : QRectF(12.0, 12.0, concise_warning ? 310.0 : 250.0, concise_warning ? 46.0 : 30.0);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, debug_overlays_mode ? 205 : 165));
  painter.drawRoundedRect(overlay_rect, 8.0, 8.0);
  painter.setPen(QColor("#e2e8f0"));
  if (debug_overlays_mode) {
    painter.drawText(QRectF(20.0, 18.0, 410.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter, "Robot Debug View: 3D diagnostics");
    painter.drawText(QRectF(20.0, 34.0, 410.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                     QString("Scene: %1").arg(scene_name));
    painter.drawText(QRectF(20.0, 50.0, 410.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                     QString("Generated mesh %1/%2 • Surface %3 • Bounds fallback %4 • URDF primitives %5/%6 • Helpers %7 • Missing geometry %8")
                       .arg(mesh_rendered_count).arg(mesh_source_count)
                       .arg(last_render_counters.mesh_surface_rendered_count)
                       .arg(last_render_counters.mesh_bounds_fallback_rendered_count)
                       .arg(urdf_primitive_rendered_count).arg(urdf_primitive_source_count)
                       .arg(overlay_count).arg(missing_geometry_count));
    painter.drawText(QRectF(20.0, 66.0, 410.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                     QString("Physical %1 • Locked URDF %2 • Fit: %3")
                       .arg(physical_item_count).arg(locked_urdf_count).arg(fit_include_overlays ? "all_items" : "generated_visuals"));
  } else {
    painter.drawText(QRectF(20.0, 18.0, 290.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                     concise_warning
                       ? QStringLiteral("3D Preview Warnings · see Diagnostics")
                       : QString("3D Preview Ready · %1 items").arg(visible_item_count));
    painter.setPen(concise_warning ? QColor("#fde68a") : QColor("#cbd5e1"));
    if (concise_warning) {
      painter.setPen(QColor("#fbbf24"));
      painter.drawText(QRectF(20.0, 36.0, 280.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                       QStringLiteral("Use Overlays → Diagnostics for details."));
    }
  }
  draw_viewport_quality_overlay(painter, visible_item_count, physical_item_count);
  if (drag_asset_preview_visible_) {
    draw_box(drag_asset_world_pos_.x(), drag_asset_world_pos_.y(), drag_asset_world_pos_.z(), 0.35, 0.35, 0.35, QColor(56, 189, 248, 120), true);
    QToolTip::showText(mapToGlobal(drag_asset_screen_pos_), drag_asset_drop_status_, this);
  }
}

Scene3DViewportWidget::RenderDebugCounters Scene3DViewportWidget::render_debug_counters() const
{
  RenderDebugCounters counters = last_render_counters;
  populate_runtime_transform_counters(counters, items);
  finalize_visual_quality(counters);
  return counters;
}

bool Scene3DViewportWidget::last_initial_fit_included_robot_bounds() const
{
  return last_initial_fit_included_robot_bounds_;
}

int Scene3DViewportWidget::last_initial_fit_physical_anchor_count() const
{
  return last_initial_fit_physical_anchor_count_;
}

QStringList Scene3DViewportWidget::last_initial_fit_anchor_roles() const
{
  return last_initial_fit_anchor_roles_;
}

QString Scene3DViewportWidget::last_camera_fit_target() const
{
  return last_camera_fit_target_;
}

bool Scene3DViewportWidget::render_smoke_fallback_frame(QImage * out_image)
{
  if (items.isEmpty()) return false;
  QSize target_size = size();
  if (target_size.width() < 32 || target_size.height() < 32) {
    target_size = QSize(640, 420);
  }
  QImage img(target_size, QImage::Format_ARGB32_Premultiplied);
  img.fill(QColor(5, 10, 24));

  int visible_item_count = 0;
  int skipped_item_count = 0;
  int rendered_item_count = 0;
  int mesh_source_count = 0;
  int mesh_rendered_count = 0;
  int mesh_surface_rendered_count = 0;
  int urdf_primitive_source_count = 0;
  int urdf_primitive_rendered_count = 0;
  int primitive_fallback_count = 0;
  int editable_primitive_count = 0;
  int placeholder_count = 0;
  int missing_geometry_count = 0;
  int generated_missing_geometry_count = 0;
  int generated_fallback_count = 0;
  int wireframe_fallback_count = 0;
  int generated_mesh_bounds_fallback_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int editable_layout_count = 0;
  QSet<QString> unique_visible_ids;

  QVector3D bmin, bmax;
  const bool has_bounds = scene_bounds_from_visible_items(bmin, bmax, true);
  const double span_x = has_bounds ? qMax(0.25, static_cast<double>(bmax.x() - bmin.x())) : 1.0;
  const double span_z = has_bounds ? qMax(0.25, static_cast<double>(bmax.z() - bmin.z())) : 1.0;
  const double scale = qMin((target_size.width() - 80.0) / span_x, (target_size.height() - 80.0) / span_z);
  auto project_top = [&](double x, double z) {
    const double px = 40.0 + (x - bmin.x()) * scale;
    const double py = target_size.height() - 40.0 - (z - bmin.z()) * scale;
    return QPointF(px, py);
  };

  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(71, 85, 105));
  for (int gx = 0; gx <= 10; ++gx) {
    const double x = 40.0 + gx * ((target_size.width() - 80.0) / 10.0);
    painter.drawLine(QPointF(x, 20.0), QPointF(x, target_size.height() - 20.0));
  }
  for (int gy = 0; gy <= 8; ++gy) {
    const double y = 40.0 + gy * ((target_size.height() - 80.0) / 8.0);
    painter.drawLine(QPointF(20.0, y), QPointF(target_size.width() - 20.0, y));
  }

  last_render_counters = RenderDebugCounters{};
  const std::vector<const ScenePreviewWidget::PreviewItem *> final_renderables =
    build_final_generated_urdf_robot_renderables(items, show_safety);
  skipped_item_count = qMax(0, static_cast<int>(items.size()) - static_cast<int>(final_renderables.size()));
  for (const auto * item_ptr : final_renderables) {
    const auto & it = *item_ptr;
    const NormalizedRole role = classify_item_role(it);
    ++visible_item_count;
    unique_visible_ids.insert(it.id);
    const bool overlay_helper = is_overlay_only_item(it) || is_overlay_visual_role(role);
    const bool generated_urdf = is_locked_urdf_item(it) || is_generated_urdf_visual_item(it);
    if (overlay_helper) ++overlay_count;
    if (generated_urdf) ++locked_urdf_count;
    if (it.linked_to_editable_layout_state) ++editable_layout_count;
    if (!overlay_helper) {
      ++last_render_counters.physical_anchor_count;
      if (item_is_generated_robot_arm_visual(it) && item_has_credible_mesh_handoff(it)) ++last_render_counters.generated_robot_mesh_count;
      if (item_is_tool_or_gripper_visual(it)) ++last_render_counters.tool_gripper_visual_count;
      if (role == NormalizedRole::Table) ++last_render_counters.table_workbench_visual_count;
      if (role == NormalizedRole::Camera) ++last_render_counters.camera_body_visual_count;
    }
    const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
    const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
    const bool intentional_primitive_fallback = source_layer == QStringLiteral("primitive_fallback") || visual_source == QStringLiteral("primitive_fallback");
    const bool generated_mesh_to_primitive_fallback = generated_urdf && item_has_credible_mesh_handoff(it) && item_has_explicit_dimensions(it);
    const bool editable_or_semantic_primitive = !intentional_primitive_fallback && !generated_mesh_to_primitive_fallback &&
      (it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it) || is_clean_semantic_primitive_role(role));
    const bool physical_mesh_source = !overlay_helper && !is_intentional_semantic_editor_primitive(it) && item_has_credible_mesh_handoff(it);
    if (physical_mesh_source) {
      ++mesh_source_count;
      if (generated_urdf && mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) ++generated_mesh_bounds_fallback_count;
      if (intentional_primitive_fallback || generated_mesh_to_primitive_fallback) ++primitive_fallback_count;
      else if (editable_or_semantic_primitive && item_has_explicit_dimensions(it)) ++editable_primitive_count;
    } else if (!overlay_helper && generated_urdf && item_has_explicit_dimensions(it)) {
      ++urdf_primitive_source_count;
      ++urdf_primitive_rendered_count;
      if (intentional_primitive_fallback || generated_mesh_to_primitive_fallback) ++primitive_fallback_count;
      else if (editable_or_semantic_primitive) ++editable_primitive_count;
      else ++wireframe_fallback_count;
    } else if (!overlay_helper && !item_has_explicit_dimensions(it)) {
      ++placeholder_count;
      ++missing_geometry_count;
      if (generated_urdf) {
        ++generated_missing_geometry_count;
        ++generated_fallback_count;
      }
    } else if (!overlay_helper) {
      if (editable_or_semantic_primitive) ++editable_primitive_count;
      else ++wireframe_fallback_count;
    }

    const ItemBounds bounds = item_bounds_for_role(it);
    const QPointF p0 = project_top(bounds.x, bounds.z);
    const QPointF p1 = project_top(bounds.x + bounds.sx, bounds.z + bounds.sz);
    QRectF rect(QPointF(qMin(p0.x(), p1.x()), qMin(p0.y(), p1.y())),
                QPointF(qMax(p0.x(), p1.x()), qMax(p0.y(), p1.y())));
    if (rect.width() < 4.0) rect.setWidth(4.0);
    if (rect.height() < 4.0) rect.setHeight(4.0);
    QColor fill = item_color(it);
    fill.setAlpha(is_overlay_visual_role(role) ? 90 : 170);
    painter.setBrush(fill);
    painter.setPen(QPen(item_color(it).lighter(135), it.id == selected_id ? 3.0 : 1.5));
    painter.drawRect(rect);
    ++rendered_item_count;
    if (physical_mesh_source) ++mesh_rendered_count;
  }

  painter.setPen(QColor("#e2e8f0"));
  painter.drawText(QRectF(16.0, 12.0, target_size.width() - 32.0, 24.0),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QString("Scene3D smoke fallback render: %1").arg(scene_name));
  painter.end();

  last_render_counters.preview_items_count = items.size();
  last_render_counters.total_payload_count = items.size();
  last_render_counters.viewport_received_count = items.size();
  last_render_counters.render_cache_count = mesh_cache_.size();
  last_render_counters.visible_count = visible_item_count;
  last_render_counters.rendered_count = rendered_item_count;
  last_render_counters.skipped_count = skipped_item_count;
  last_render_counters.unique_visible_item_count = unique_visible_ids.size();
  last_render_counters.mesh_source_count = mesh_source_count;
  last_render_counters.mesh_backed_count = mesh_source_count;
  last_render_counters.mesh_rendered_count = mesh_rendered_count;
  if (mesh_surface_rendered_count <= 0 && mesh_rendered_count > 0) mesh_surface_rendered_count = mesh_rendered_count;
  last_render_counters.mesh_surface_rendered_count = mesh_surface_rendered_count;
  last_render_counters.mesh_bounds_fallback_rendered_count = scene3d_debug_fallback_boxes_enabled() ? qMax(0, mesh_source_count - mesh_surface_rendered_count) : 0;
  last_render_counters.generated_mesh_bounds_fallback_rendered_count = scene3d_debug_fallback_boxes_enabled() ? generated_mesh_bounds_fallback_count : 0;
  last_render_counters.mesh_path_resolved_count = 0;
  last_render_counters.mesh_file_loaded_count = 0;
  last_render_counters.mesh_triangles_loaded_count = 0;
  for (auto cache_it = mesh_cache_.cbegin(); cache_it != mesh_cache_.cend(); ++cache_it) {
    const MeshCacheEntry & cache_entry = cache_it.value();
    if (cache_entry.path_resolved) ++last_render_counters.mesh_path_resolved_count;
    if (cache_entry.loaded && cache_entry.valid && !cache_entry.mesh.triangles.isEmpty()) {
      ++last_render_counters.mesh_file_loaded_count;
      last_render_counters.mesh_triangles_loaded_count += static_cast<int>(cache_entry.mesh.triangles.size());
    }
  }
  last_render_counters.urdf_primitive_source_count = urdf_primitive_source_count;
  last_render_counters.urdf_primitive_rendered_count = urdf_primitive_rendered_count;
  last_render_counters.placeholder_count = placeholder_count;
  last_render_counters.missing_geometry_count = missing_geometry_count;
  last_render_counters.generated_missing_geometry_count = generated_missing_geometry_count;
  last_render_counters.generated_fallback_count = generated_fallback_count;
  last_render_counters.wireframe_fallback_count = wireframe_fallback_count;
  last_render_counters.primitive_fallback_rendered_count = primitive_fallback_count;
  last_render_counters.primitive_fallback_count = primitive_fallback_count;
  last_render_counters.editable_primitive_rendered_count = editable_primitive_count;
  last_render_counters.valid_physical_fallback_count = primitive_fallback_count;
  last_render_counters.overlay_rendered_count = overlay_count;
  last_render_counters.overlay_helper_count = overlay_count;
  last_render_counters.overlay_count = overlay_count;
  last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;
  populate_runtime_transform_counters(last_render_counters, items);
  last_render_counters.editable_layout_count = editable_layout_count;
  last_render_counters.hierarchy_child_row_count = visible_item_count;
  last_render_counters.last_paint_completed = rendered_item_count > 0;
  last_render_counters.smoke_fallback_render_used = rendered_item_count > 0;
  finalize_visual_quality(last_render_counters);
  if (out_image) *out_image = img;
  return rendered_item_count > 0;
}

bool Scene3DViewportWidget::scene_bounds_from_visible_items(QVector3D & out_min, QVector3D & out_max, bool include_overlays) const
{
  if (items.isEmpty()) return false;
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_fittable_item = false;
  for (const auto & it : items) {
    if (!include_in_fit_bounds(it, include_overlays)) continue;
    if (!item_is_enabled_for_fit(it)) continue;

    ItemBounds bounds{};
    const bool generated_or_locked_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool should_prefer_transformed_mesh_bounds = item_has_mesh_surface_candidate(it) || generated_or_locked_visual;
    if (should_prefer_transformed_mesh_bounds && mesh_world_bounds_for_item(it, bounds)) {
      // Mesh-backed generated URDF visuals and physical asset previews use their transformed
      // render bounds so product fitting follows what is actually drawn, not raw layout extents.
    } else {
      PrimitiveWorldBounds primitive_bounds{};
      if (item_has_valid_urdf_primitive(it) && primitive_world_bounds_for_item(it, primitive_bounds)) {
        bounds = { primitive_bounds.x, primitive_bounds.y, primitive_bounds.z,
                   primitive_bounds.sx, primitive_bounds.sy, primitive_bounds.sz };
      } else {
        bounds = item_bounds_for_role(it);
      }
    }
    has_fittable_item = true;
    out_min.setX(std::min(out_min.x(), static_cast<float>(bounds.x)));
    out_min.setY(std::min(out_min.y(), static_cast<float>(bounds.y)));
    out_min.setZ(std::min(out_min.z(), static_cast<float>(bounds.z)));
    out_max.setX(std::max(out_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    out_max.setY(std::max(out_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    out_max.setZ(std::max(out_max.z(), static_cast<float>(bounds.z + bounds.sz)));
  }
  return has_fittable_item;
}

bool Scene3DViewportWidget::initial_physical_fit_bounds(QVector3D & out_min, QVector3D & out_max,
                                                        bool * out_robot_included,
                                                        int * out_anchor_count,
                                                        QStringList * out_anchor_roles) const
{
  if (out_robot_included) *out_robot_included = false;
  if (out_anchor_count) *out_anchor_count = 0;
  if (out_anchor_roles) out_anchor_roles->clear();
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_anchor = false;
  bool robot_included = false;
  QStringList roles;
  int anchor_count = 0;
  for (const auto & it : items) {
    const QString anchor_role = initial_physical_fit_anchor_role(it);
    if (anchor_role.isEmpty()) continue;

    ItemBounds bounds{};
    const bool generated_or_locked_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool should_prefer_transformed_mesh_bounds = item_has_mesh_surface_candidate(it) || generated_or_locked_visual;
    if (should_prefer_transformed_mesh_bounds && mesh_world_bounds_for_item(it, bounds)) {
      // Initial product fit uses the same transformed mesh bounds as rendering for generated robot/tool visuals.
    } else {
      PrimitiveWorldBounds primitive_bounds{};
      if (item_has_valid_urdf_primitive(it) && primitive_world_bounds_for_item(it, primitive_bounds)) {
        bounds = { primitive_bounds.x, primitive_bounds.y, primitive_bounds.z,
                   primitive_bounds.sx, primitive_bounds.sy, primitive_bounds.sz };
      } else {
        bounds = item_bounds_for_role(it);
      }
    }

    has_anchor = true;
    ++anchor_count;
    if (!roles.contains(anchor_role)) roles << anchor_role;
    robot_included = robot_included || anchor_role == QStringLiteral("generated_robot_visual");
    out_min.setX(std::min(out_min.x(), static_cast<float>(bounds.x)));
    out_min.setY(std::min(out_min.y(), static_cast<float>(bounds.y)));
    out_min.setZ(std::min(out_min.z(), static_cast<float>(bounds.z)));
    out_max.setX(std::max(out_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    out_max.setY(std::max(out_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    out_max.setZ(std::max(out_max.z(), static_cast<float>(bounds.z + bounds.sz)));
  }
  roles.sort();
  if (out_robot_included) *out_robot_included = robot_included;
  if (out_anchor_count) *out_anchor_count = anchor_count;
  if (out_anchor_roles) *out_anchor_roles = roles;
  return has_anchor;
}

bool Scene3DViewportWidget::robot_bounds_from_rendered_visuals(QVector3D & out_min, QVector3D & out_max) const
{
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  bool has_robot_visual = false;
  for (const auto & it : items) {
    const bool urdf_visual = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
    const bool robot_base_marker = classify_item_role(it) == NormalizedRole::RobotBase && it.linked_to_editable_layout_state;
    if (!urdf_visual && !robot_base_marker) continue;
    if (is_overlay_only_item(it)) continue;
    ItemBounds bounds{};
    if (!mesh_world_bounds_for_item(it, bounds)) bounds = { it.x, it.y, it.z, it.sx, it.sy, it.sz };
    out_min.setX(std::min(out_min.x(), static_cast<float>(bounds.x)));
    out_min.setY(std::min(out_min.y(), static_cast<float>(bounds.y)));
    out_min.setZ(std::min(out_min.z(), static_cast<float>(bounds.z)));
    out_max.setX(std::max(out_max.x(), static_cast<float>(bounds.x + bounds.sx)));
    out_max.setY(std::max(out_max.y(), static_cast<float>(bounds.y + bounds.sy)));
    out_max.setZ(std::max(out_max.z(), static_cast<float>(bounds.z + bounds.sz)));
    has_robot_visual = true;
  }
  return has_robot_visual;
}

bool Scene3DViewportWidget::item_has_explicit_dimensions(const ScenePreviewWidget::PreviewItem & item) const
{
  return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
}

QString Scene3DViewportWidget::placeholder_reason_for_item(const ScenePreviewWidget::PreviewItem & item) const
{
  if (item_has_credible_mesh_handoff(item)) return QString();
  if (item_has_valid_urdf_primitive(item) || item_has_explicit_dimensions(item)) return QString();
  return QStringLiteral("missing geometry");
}


bool Scene3DViewportWidget::draw_required_generated_robot_emergency_fallback(const ScenePreviewWidget::PreviewItem & it, const QColor & color)
{
  if (!is_required_generated_robot_viewport_link(it)) return false;

  QVector3D local_min;
  QVector3D local_max;
  bool have_local_bounds = false;

  const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
  if (!mesh_source.trimmed().isEmpty()) {
    QString canonical_mesh_source;
    if (!try_resolve_canonical_mesh_path(mesh_source, canonical_mesh_source, &it)) {
      canonical_mesh_source = QFileInfo(mesh_source).absoluteFilePath();
    }
    const auto cache_it = mesh_cache_.constFind(canonical_mesh_source);
    if (cache_it != mesh_cache_.constEnd() && cache_it.value().has_bounds) {
      local_min = cache_it.value().local_min;
      local_max = cache_it.value().local_max;
      have_local_bounds = true;
    }
  }

  if (!have_local_bounds) {
    const QString link = scene3d_canonical_link_name_for_item(it);
    const double length = (link == QStringLiteral("upper_arm_link") || link == QStringLiteral("forearm_link")) ? 0.42 :
                          (link == QStringLiteral("base_link_inertia") || link == QStringLiteral("shoulder_link")) ? 0.16 : 0.12;
    const double radius = (link == QStringLiteral("upper_arm_link") || link == QStringLiteral("forearm_link")) ? 0.045 : 0.055;
    // Local-Y capsule/box surrogate, matching the primitive draw convention and
    // then using the baked world visual pose + ROS-to-viewport basis exactly once.
    local_min = QVector3D(static_cast<float>(-radius), static_cast<float>(-length * 0.5), static_cast<float>(-radius));
    local_max = QVector3D(static_cast<float>(radius), static_cast<float>(length * 0.5), static_cast<float>(radius));
  }

  const QVector3D span = local_max - local_min;
  if (span.x() <= 0.0f || span.y() <= 0.0f || span.z() <= 0.0f) return false;

  QColor fill = color.isValid() ? color : QColor(203, 213, 225, 230);
  fill.setAlpha(qMax(fill.alpha(), 210));
  QColor outline = generated_locked_preview_outline();
  outline.setAlpha(210);

  glPushMatrix();
  QMatrix4x4 transform = viewport_world_visual_transform(it);
  transform.scale(static_cast<float>(it.mesh_scale_x), static_cast<float>(it.mesh_scale_y), static_cast<float>(it.mesh_scale_z));
  glMultMatrixf(transform.constData());
  draw_box(local_min.x(), local_min.y(), local_min.z(), span.x(), span.y(), span.z(), fill, false);
  draw_box_outline(local_min.x(), local_min.y(), local_min.z(), span.x(), span.y(), span.z(), outline, 1.25f);
  glPopMatrix();
  return true;
}

bool Scene3DViewportWidget::draw_urdf_primitive_geometry(const ScenePreviewWidget::PreviewItem & it, const QColor & color)
{
  const QString type = normalized_token(it.primitive_geometry_type);
  if (!item_has_valid_urdf_primitive(it)) return false;

  glPushMatrix();
  apply_authoritative_world_visual_transform_gl(it);
  glScaled(it.mesh_scale_x, it.mesh_scale_y, it.mesh_scale_z);

  if (type == QStringLiteral("box")) {
    draw_box(-it.sx * 0.5, -it.sy * 0.5, -it.sz * 0.5, it.sx, it.sy, it.sz, color, color.alphaF() < 0.99);
  } else if (type == QStringLiteral("cylinder")) {
    const double r = it.primitive_radius;
    const double h = it.primitive_length;
    draw_cylinder(0.0, -h * 0.5, 0.0, r, h, color, color.alphaF() < 0.99, 32);
  } else if (type == QStringLiteral("sphere")) {
    draw_sphere(0.0, 0.0, 0.0, it.primitive_radius, color, color.alphaF() < 0.99, 24, 12);
  } else if (type == QStringLiteral("capsule")) {
    const double r = it.primitive_radius;
    const double h = it.primitive_length;
    draw_cylinder(0.0, -h * 0.5, 0.0, r, h, color, color.alphaF() < 0.99, 32);
    draw_sphere(0.0, -h * 0.5, 0.0, r, color, color.alphaF() < 0.99, 24, 8);
    draw_sphere(0.0, h * 0.5, 0.0, r, color, color.alphaF() < 0.99, 24, 8);
  } else {
    glPopMatrix();
    return false;
  }
  glPopMatrix();
  return true;
}

bool Scene3DViewportWidget::draw_truthful_item_geometry(const ScenePreviewWidget::PreviewItem & it, int * out_placeholder_count,
                                                        int * out_mesh_count, int * out_wireframe_count,
                                                        int * out_urdf_primitive_count, int * out_missing_geometry_count,
                                                        int * out_primitive_fallback_count,
                                                        int * out_editable_primitive_count)
{
  const QString source_layer = normalized_scene3d_layer_token(it.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(it.active_visual_source);
  const NormalizedRole semantic_role = classify_item_role(it);
  const bool generated_or_locked_preview = is_generated_urdf_visual_item(it) || is_locked_urdf_item(it);
  const bool helper_overlay = !generated_or_locked_preview && (is_overlay_only_item(it) || source_layer == "overlay" || visual_source == "overlay");
  if (helper_overlay) {
    if (is_clean_semantic_primitive_role(semantic_role) && draw_clean_semantic_primitive(it)) {
      if (out_editable_primitive_count) ++(*out_editable_primitive_count);
      return true;
    }
    if (item_has_explicit_dimensions(it)) {
      draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, QColor(148, 163, 184, 58), 0.75f);
      if (out_wireframe_count) ++(*out_wireframe_count);
      return false;
    }
    return false;
  }
  // Semantic/layout primitives are intentional product overlays, not failed mesh
  // assets.  Only physical mesh candidates proceed through mesh rejection;
  // semantic items without mesh handoff render through the semantic primitive
  // path so they do not emit REJECT_MESH_METADATA_MISSING or increment missing
  // geometry/fallback counters.
  QColor visual_color = item_color(it, diagnostic_transparency_mode);
  const bool editable_layout_preview = it.linked_to_editable_layout_state || item_is_editable_for_gizmo(it);
  if (semantic_role == NormalizedRole::WarningAnchor) {
    return false;
  }
  if (!generated_or_locked_preview && is_intentional_semantic_editor_primitive(it) && draw_clean_semantic_primitive(it)) {
    if (out_editable_primitive_count) ++(*out_editable_primitive_count);
    return true;
  }
  const bool semantic_without_mesh_handoff =
    is_clean_semantic_primitive_role(semantic_role) &&
    !generated_or_locked_preview &&
    !item_has_credible_mesh_handoff(it);
  if (semantic_without_mesh_handoff && draw_clean_semantic_primitive(it)) {
    if (out_editable_primitive_count) ++(*out_editable_primitive_count);
    return true;
  }
  // Always try mesh-backed draw first for physical items.
  if (draw_mesh_preview_if_available(it, visual_color, true)) {
    if (out_mesh_count) ++(*out_mesh_count);
    ItemBounds mesh_bounds{};
    if (!mesh_world_bounds_for_item(it, mesh_bounds)) mesh_bounds = item_bounds_for_role(it);
    if (debug_overlays_mode && generated_or_locked_preview) {
      draw_box_outline(mesh_bounds.x, mesh_bounds.y, mesh_bounds.z, mesh_bounds.sx, mesh_bounds.sy, mesh_bounds.sz,
                       generated_locked_preview_outline(), 0.85f);
    } else if (debug_overlays_mode && editable_layout_preview) {
      draw_box_outline(mesh_bounds.x, mesh_bounds.y, mesh_bounds.z, mesh_bounds.sx, mesh_bounds.sy, mesh_bounds.sz,
                       editable_layout_accent_outline(), 1.6f);
    }
    return true;
  }
  if (generated_or_locked_preview && draw_required_generated_robot_emergency_fallback(it, product_view_generated_locked_material(it, diagnostic_transparency_mode))) {
    if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);
    return true;
  }
  if (generated_or_locked_preview && item_has_valid_urdf_primitive(it)) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      // Primitive geometry exists but is intentionally hidden by Meshes-only mode; do not show a red missing-geometry marker
      // or count an expected suppression as a generated fallback during normal product rendering.
      if (debug_overlays_mode && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.6f);
      }
      if (debug_overlays_mode || scene3d_debug_logs_enabled()) {
        warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_PRIMITIVE_SUPPRESSED_BY_MESH_ONLY_MODE: URDF primitive available but disabled"), it.source_path);
      }
      return false;
    }
    const QColor primitive_fill = generated_or_locked_preview ? product_view_generated_locked_material(it, diagnostic_transparency_mode) : visual_color;
    if (draw_urdf_primitive_geometry(it, primitive_fill)) {
      if (debug_overlays_mode && generated_or_locked_preview && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.7f);
      } else if (debug_overlays_mode && editable_layout_preview && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, editable_layout_accent_outline(), 1.4f);
      }
      if (out_urdf_primitive_count) ++(*out_urdf_primitive_count);
      return true;
    }
  }
  if (is_clean_semantic_primitive_role(semantic_role)) {
    if (draw_clean_semantic_primitive(it)) {
      if (out_editable_primitive_count) ++(*out_editable_primitive_count);
      return true;
    }
    // Recognized semantic roles that still cannot be rendered remain diagnostics-only.
    // Unknown physical items continue through the missing-geometry path below.
    if (!item_has_explicit_primitive_dimensions(it)) {
      if (out_missing_geometry_count) ++(*out_missing_geometry_count);
      return false;
    }
  }

  const QString missing_reason = placeholder_reason_for_item(it);
  if (!missing_reason.isEmpty()) {
    draw_missing_geometry_marker(it);
    if (out_placeholder_count) ++(*out_placeholder_count);
    if (out_missing_geometry_count) ++(*out_missing_geometry_count);
    if (generated_or_locked_preview) ++last_render_counters.generated_fallback_count;
    if (show_warning_labels) warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_MISSING_GEOMETRY: no mesh metadata or explicit primitive dimensions"), it.source_path);
    return false;
  }
  if (!generated_or_locked_preview && item_has_valid_urdf_primitive(it)) {
    if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
      // Primitive geometry exists but is intentionally hidden by Meshes-only mode; do not show a red missing-geometry marker
      // or count an expected suppression as a generated fallback during normal product rendering.
      if (debug_overlays_mode && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.6f);
      }
      if (debug_overlays_mode || scene3d_debug_logs_enabled()) {
        warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_PRIMITIVE_SUPPRESSED_BY_MESH_ONLY_MODE: URDF primitive available but disabled"), it.source_path);
      }
      return false;
    }
    const QColor primitive_fill = visual_color;
    if (draw_urdf_primitive_geometry(it, primitive_fill)) {
      if (debug_overlays_mode && editable_layout_preview && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, editable_layout_accent_outline(), 1.4f);
      }
      if (out_urdf_primitive_count) ++(*out_urdf_primitive_count);
      return true;
    }
  }
  if (item_has_explicit_dimensions(it)) {
    const bool locked_generated_urdf_visual_fallback = is_generated_urdf_visual_fallback_item(it);
    const bool intentional_primitive_fallback =
      source_layer == QStringLiteral("primitive_fallback") || visual_source == QStringLiteral("primitive_fallback") ||
      locked_generated_urdf_visual_fallback;
    const bool generated_mesh_to_primitive_fallback =
      generated_or_locked_preview && item_has_credible_mesh_handoff(it) && !locked_generated_urdf_visual_fallback;
    const bool raw_generated_bounds_only = is_raw_generated_bounds_only_item(it);
    if ((mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes && !locked_generated_urdf_visual_fallback) || raw_generated_bounds_only) {
      if (raw_generated_bounds_only) {
        const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
        const bool mesh_candidate = item_has_mesh_surface_candidate(it);
        const QString stored_reason = last_mesh_rejection_reason_for_item(it.id);
        if (mesh_candidate && !stored_reason.trimmed().isEmpty()) {
          warn_mesh_fallback_once(it.id, stored_reason, mesh_source);
        } else if (mesh_candidate) {
          warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_MESH_CANDIDATE_SUPPRESSED: mesh surface candidate could not be rendered"), mesh_source);
        } else {
          warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_RAW_GENERATED_BOUNDS_SUPPRESSED: raw generated bounds placeholder has no valid mesh source candidate"), it.source_path);
        }
        return false;
      }
      // Semantic primitive geometry exists but is intentionally hidden by Meshes-only mode; do not show a warning marker
      // or count an expected suppression as a generated fallback during normal product rendering.
      if (debug_overlays_mode) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.6f);
      }
      if (debug_overlays_mode || scene3d_debug_logs_enabled()) {
        warn_mesh_fallback_once(it.id, QStringLiteral("REJECT_PRIMITIVE_SUPPRESSED_BY_MESH_ONLY_MODE: semantic primitive dimensions available but disabled"), it.source_path);
      }
      return false;
    }
    if (generated_mesh_to_primitive_fallback && !intentional_primitive_fallback && !editable_layout_preview) {
      const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
      const QString stored_reason = last_mesh_rejection_reason_for_item(it.id);
      if (debug_overlays_mode && item_has_explicit_dimensions(it)) {
        draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, generated_primitive_fallback_outline(), 0.55f);
      }
      if (show_warning_labels || item_should_surface_mesh_warning(it) || scene3d_debug_logs_enabled()) {
        warn_mesh_fallback_once(it.id,
          stored_reason.trimmed().isEmpty()
            ? QStringLiteral("REJECT_GENERATED_BOUNDS_BOX_SUPPRESSED: generated locked mesh failed and only generic dimensions were available")
            : stored_reason,
          mesh_source);
      }
      if (out_missing_geometry_count) ++(*out_missing_geometry_count);
      return false;
    }
    const QColor fallback_fill = generated_or_locked_preview ? generated_primitive_fallback_fill()
      : (editable_layout_preview ? QColor(34, 211, 238, 60) : QColor(148, 163, 184, 28));
    const QColor fallback_line = generated_or_locked_preview ? generated_primitive_fallback_outline()
      : (editable_layout_preview ? editable_layout_accent_outline() : QColor(148, 163, 184, 76));
    const float fallback_line_width = generated_or_locked_preview ? 0.7f : (editable_layout_preview ? 1.6f : 0.75f);
    if (!editable_layout_preview && !scene3d_debug_fallback_boxes_enabled() && !locked_generated_urdf_visual_fallback) {
      if (!intentional_primitive_fallback && out_missing_geometry_count) ++(*out_missing_geometry_count);
      return false;
    }
    if (locked_generated_urdf_visual_fallback) {
      draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, product_view_generated_locked_material(it, diagnostic_transparency_mode), false);
    } else if (scene3d_debug_fallback_boxes_enabled() || editable_layout_preview) {
      draw_box(it.x, it.y, it.z, it.sx, it.sy, it.sz, fallback_fill, true);
    }
    // Generated primitive fallback outlines are bounds diagnostics; keep them out of normal Product View.
    if (debug_overlays_mode || scene3d_debug_fallback_boxes_enabled()) {
      draw_box_outline(it.x, it.y, it.z, it.sx, it.sy, it.sz, fallback_line, fallback_line_width);
    }
    if (intentional_primitive_fallback || generated_mesh_to_primitive_fallback) {
      if (out_primitive_fallback_count) ++(*out_primitive_fallback_count);
      return true;
    }
    if (editable_layout_preview) {
      if (out_editable_primitive_count) ++(*out_editable_primitive_count);
      return true;
    }
    if (out_wireframe_count) ++(*out_wireframe_count);
    return false;
  }
  draw_missing_geometry_marker(it);
  if (out_placeholder_count) ++(*out_placeholder_count);
  if (out_missing_geometry_count) ++(*out_missing_geometry_count);
  if (generated_or_locked_preview) ++last_render_counters.generated_fallback_count;
  return false;
}

QString Scene3DViewportWidget::gizmo_mode_label() const
{
  switch (gizmo_mode) {
    case GizmoMode::Select: return "Select";
    case GizmoMode::Move: return "Move";
    case GizmoMode::Rotate: return "Rotate";
    case GizmoMode::ScaleDisabled: return "Scale";
  }
  return "Select";
}

bool Scene3DViewportWidget::parse_stl_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                     InternalTriangleMesh & out_mesh, QString & out_error,
                                                     int triangle_limit)
{
  out_mesh.triangles.clear();
  const bool ext_ascii_hint = source_hint.endsWith(".stla", Qt::CaseInsensitive);
  const bool ext_binary_hint = source_hint.endsWith(".stlb", Qt::CaseInsensitive);
  const bool ascii = ext_ascii_hint || (!ext_binary_hint && looks_like_ascii_stl(bytes));
  if (ascii) return parse_ascii_stl(bytes, out_mesh, out_error, triangle_limit);
  return parse_binary_stl(bytes, out_mesh, out_error, triangle_limit);
}

bool Scene3DViewportWidget::parse_collada_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                         InternalTriangleMesh & out_mesh, QString & out_error,
                                                         double * out_unit_meter, int triangle_limit)
{
  Q_UNUSED(source_hint);
  out_mesh.triangles.clear();
  return parse_collada_bytes(bytes, out_mesh, out_error, out_unit_meter, triangle_limit);
}

bool Scene3DViewportWidget::parse_obj_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                                     InternalTriangleMesh & out_mesh, QString & out_error,
                                                     int triangle_limit)
{
  Q_UNUSED(source_hint);
  out_mesh.triangles.clear();
  return parse_obj_bytes(bytes, out_mesh, out_error, triangle_limit);
}

bool Scene3DViewportWidget::compute_mesh_bounds_for_test(const InternalTriangleMesh & mesh, QVector3D & out_min, QVector3D & out_max)
{
  if (mesh.triangles.isEmpty()) return false;
  out_min = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  out_max = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (const auto & tri : mesh.triangles) {
    for (const auto & v : tri.vertices) {
      out_min.setX(qMin(out_min.x(), v.x()));
      out_min.setY(qMin(out_min.y(), v.y()));
      out_min.setZ(qMin(out_min.z(), v.z()));
      out_max.setX(qMax(out_max.x(), v.x()));
      out_max.setY(qMax(out_max.y(), v.y()));
      out_max.setZ(qMax(out_max.z(), v.z()));
    }
  }
  return true;
}

bool Scene3DViewportWidget::should_attempt_mesh_draw_for_mode_for_test(ScenePreviewWidget::MeshPreviewMode mode,
                                                                        bool cache_loaded, bool cache_valid)
{
  if (mode == ScenePreviewWidget::MeshPreviewMode::Primitives) return false;
  if (mode == ScenePreviewWidget::MeshPreviewMode::Meshes) return cache_loaded && cache_valid;
  return cache_loaded && cache_valid;
}

QString Scene3DViewportWidget::render_role_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  const QString source_layer = normalized_scene3d_layer_token(item.source_layer);
  const QString visual_source = normalized_scene3d_layer_token(item.active_visual_source);
  if (is_overlay_only_item(item) || source_layer == "overlay" || visual_source == "overlay") return QStringLiteral("helper_overlay");
  if (is_generated_urdf_visual_item(item) || is_locked_urdf_item(item)) {
    return item.mesh_available ? QStringLiteral("generated_urdf_mesh") : QStringLiteral("generated_urdf_primitive");
  }
  if (item.linked_to_editable_layout_state) return QStringLiteral("physical_layout_item");
  if (is_clean_semantic_primitive_role(classify_item_role(item))) return QStringLiteral("semantic_editor_primitive");
  if (!item.mesh_available && item.mesh_path.trimmed().isEmpty() && !item.has_mesh_metadata) return QStringLiteral("hidden_missing_mesh");
  return QStringLiteral("real_urdf_primitive");
}

bool Scene3DViewportWidget::should_include_in_default_fit_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return include_in_fit_bounds(item, false);
}

bool Scene3DViewportWidget::should_draw_as_solid_for_test(const ScenePreviewWidget::PreviewItem & item,
                                                           ScenePreviewWidget::MeshPreviewMode mode)
{
  if (should_draw_clean_semantic_primitive_for_test(item)) return true;
  const QString role = render_role_for_test(item);
  if (role == "helper_overlay" || role == "missing_mesh_fallback") return false;
  if (mode == ScenePreviewWidget::MeshPreviewMode::Meshes && !item.mesh_available) return false;
  return true;
}

bool Scene3DViewportWidget::should_draw_as_wireframe_for_test(const ScenePreviewWidget::PreviewItem & item,
                                                               ScenePreviewWidget::MeshPreviewMode mode)
{
  Q_UNUSED(mode);
  const QString role = render_role_for_test(item);
  if (item_has_valid_urdf_primitive(item)) return false;
  if (should_suppress_missing_geometry_marker_for_semantic_role(item)) return false;
  return role == "helper_overlay" || role == "missing_mesh_fallback";
}

bool Scene3DViewportWidget::should_draw_clean_semantic_primitive_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return is_clean_semantic_primitive_role(classify_item_role(item));
}

bool Scene3DViewportWidget::should_suppress_missing_geometry_marker_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return should_suppress_missing_geometry_marker_for_semantic_role(item);
}

bool Scene3DViewportWidget::has_mesh_surface_candidate_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return item_has_mesh_surface_candidate(item);
}

QColor Scene3DViewportWidget::material_color_for_test(const ScenePreviewWidget::PreviewItem & item, bool diagnostic_transparency_mode)
{
  return item_color(item, diagnostic_transparency_mode);
}

bool Scene3DViewportWidget::is_raw_generated_bounds_only_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return is_raw_generated_bounds_only_item(item);
}

QMatrix4x4 Scene3DViewportWidget::final_mesh_transform_matrix_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return final_mesh_transform_matrix(item);
}

QMatrix4x4 Scene3DViewportWidget::baked_mesh_asset_local_correction_matrix_for_test(const ScenePreviewWidget::PreviewItem & item)
{
  return baked_mesh_asset_local_correction_matrix(item);
}

bool Scene3DViewportWidget::try_resolve_canonical_mesh_path(const QString & path, QString & out_canonical,
                                                               const ScenePreviewWidget::PreviewItem * item,
                                                               QString * out_failure_reason) const
{
  if (out_failure_reason) out_failure_reason->clear();
  QFileInfo info(path);
  if (!info.exists() || !info.isFile()) {
    if (out_failure_reason) *out_failure_reason = mesh_load_failure_reason_for_item(path, item);
    return false;
  }
  out_canonical = info.canonicalFilePath();
  if (out_canonical.isEmpty()) out_canonical = info.absoluteFilePath();
  if (out_canonical.isEmpty() && out_failure_reason) *out_failure_reason = mesh_load_failure_reason_for_item(path, item);
  return !out_canonical.isEmpty();
}

QString Scene3DViewportWidget::mesh_rejection_diagnostic_detail(const ScenePreviewWidget::PreviewItem & item,
                                                                const QString & mesh_source,
                                                                const QString & canonical_mesh_source,
                                                                const MeshCacheEntry * entry,
                                                                const QString & extra_detail) const
{
  QStringList parts;
  parts << QStringLiteral("item_id=%1").arg(item.id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item.id.trimmed());
  parts << QStringLiteral("package_uri=%1").arg(item.package_uri.trimmed().isEmpty() ? QStringLiteral("<none>") : item.package_uri.trimmed());
  parts << QStringLiteral("mesh_path=%1").arg(item.mesh_path.trimmed().isEmpty() ? QStringLiteral("<none>") : item.mesh_path.trimmed());
  parts << QStringLiteral("source_path=%1").arg(item.source_path.trimmed().isEmpty() ? QStringLiteral("<none>") : item.source_path.trimmed());
  parts << QStringLiteral("requested_path=%1").arg(mesh_source.trimmed().isEmpty() ? QStringLiteral("<none>") : mesh_source.trimmed());
  if (!canonical_mesh_source.trimmed().isEmpty()) parts << QStringLiteral("canonical_path=%1").arg(canonical_mesh_source.trimmed());
  if (!item.resolved_source_path_original.trimmed().isEmpty()) parts << QStringLiteral("resolved_source_path_original=%1").arg(item.resolved_source_path_original.trimmed());
  if (!item.source_path_resolution_outcome.trimmed().isEmpty()) parts << QStringLiteral("source_path_resolution_outcome=%1").arg(item.source_path_resolution_outcome.trimmed());
  if (item.resolved_source_path_stale) parts << QStringLiteral("resolved_source_path_stale=true");
  if (entry) {
    if (!entry->load_failure_reason.trimmed().isEmpty()) parts << QStringLiteral("load_failure_reason=%1").arg(entry->load_failure_reason.trimmed());
    if (!entry->failure_reason_code.trimmed().isEmpty()) parts << QStringLiteral("failure_reason_code=%1").arg(entry->failure_reason_code.trimmed());
    if (!entry->warning.trimmed().isEmpty()) parts << QStringLiteral("cache_warning=%1").arg(entry->warning.trimmed());
    if (!entry->parse_error.trimmed().isEmpty()) parts << QStringLiteral("parse_error=%1").arg(entry->parse_error.trimmed());
    parts << QStringLiteral("path_resolved=%1").arg(entry->path_resolved ? QStringLiteral("true") : QStringLiteral("false"));
  }
  if (!extra_detail.trimmed().isEmpty()) parts << extra_detail.trimmed();
  return parts.join(QStringLiteral("; "));
}

bool Scene3DViewportWidget::item_should_surface_mesh_warning(const ScenePreviewWidget::PreviewItem & item) const
{
  const QString mix = normalized_token(item.id + QStringLiteral("|") + item.display_name + QStringLiteral("|") +
                                       item.role + QStringLiteral("|") + item.category + QStringLiteral("|") +
                                       item.lock_reason + QStringLiteral("|") + item.mesh_path + QStringLiteral("|") +
                                       item.package_uri + QStringLiteral("|") + item.source_path);
  return classify_item_role(item) == NormalizedRole::RobotBase ||
         mix.contains(QStringLiteral("robot")) || mix.contains(QStringLiteral("gripper")) ||
         mix.contains(QStringLiteral("end_effector")) || mix.contains(QStringLiteral("end effector")) ||
         mix.contains(QStringLiteral("tool"));
}

void Scene3DViewportWidget::remember_mesh_rejection_reason(const QString & item_id, const QString & reason)
{
  const QString key = item_id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item_id.trimmed();
  last_mesh_rejection_reasons_.insert(key, reason.trimmed().isEmpty() ? QStringLiteral("REJECT_UNKNOWN") : reason);
}

QString Scene3DViewportWidget::last_mesh_rejection_reason_for_item(const QString & item_id) const
{
  const QString key = item_id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item_id.trimmed();
  return last_mesh_rejection_reasons_.value(key);
}

bool Scene3DViewportWidget::warn_mesh_fallback_once(const QString & item_id, const QString & reason, const QString & path)
{
  const QString scene_key = scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed();
  const QString path_key = path.trimmed().isEmpty() ? QStringLiteral("<none>") : path.trimmed();
  const QString key = QStringLiteral("%1|%2|%3|%4").arg(scene_key, item_id, reason, path_key);
  if (warned_mesh_fallbacks_.contains(key)) return false;
  warned_mesh_fallbacks_.insert(key);
  const QString reason_code = reason.section(QLatin1Char(':'), 0, 0).trimmed();
  qWarning().noquote() << QStringLiteral("Scene3D render fallback: scene=%1 item_id=%2 reason_code=%3 detail=%4 mesh_path=%5")
                            .arg(scene_key,
                                 item_id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item_id.trimmed(),
                                 reason_code.isEmpty() ? QStringLiteral("REJECT_UNKNOWN") : reason_code,
                                 reason,
                                 path_key);
  return true;
}

const Scene3DViewportWidget::MeshCacheEntry & Scene3DViewportWidget::ensure_mesh_cached(const ScenePreviewWidget::PreviewItem & item,
                                                                                           const QString & path)
{
  const QFileInfo input_info(path);
  QString canonical;
  QString load_failure_reason;
  const bool path_resolved = try_resolve_canonical_mesh_path(path, canonical, &item, &load_failure_reason);
  if (!path_resolved) canonical = input_info.absoluteFilePath();
  const QFileInfo canonical_info(canonical);
  auto it = mesh_cache_.find(canonical);
  if (it != mesh_cache_.end()) return it.value();
  MeshCacheEntry entry;
  entry.loaded = true;
  entry.requested_path = path;
  entry.path_resolved = path_resolved;
  entry.package_uri = item.package_uri;
  entry.resolved_source_path_original = item.resolved_source_path_original;
  entry.source_path_resolution_outcome = item.source_path_resolution_outcome;
  entry.resolved_source_path_stale = item.resolved_source_path_stale;
  entry.load_failure_reason = load_failure_reason;
  entry.failure_reason_code = load_failure_reason;
  if (!canonical_info.exists() || !canonical_info.isFile()) {
    entry.valid = false;
    if (entry.load_failure_reason.trimmed().isEmpty()) {
      entry.load_failure_reason = mesh_load_failure_reason_for_item(path, &item);
    }
    entry.failure_reason_code = entry.load_failure_reason.trimmed().isEmpty() ? QStringLiteral("file_not_found") : entry.load_failure_reason;
    entry.warning = QStringLiteral("mesh missing on disk (reason_code: %1)").arg(entry.failure_reason_code);
    qWarning().noquote() << QStringLiteral("Scene3D mesh load failed: item_id=%1 mesh_path=%2 canonical_path=%3 loader_reason=%4")
                              .arg(item.id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item.id.trimmed(),
                                   path.trimmed().isEmpty() ? QStringLiteral("<none>") : path.trimmed(),
                                   canonical,
                                   entry.warning);
    return mesh_cache_.insert(canonical, entry).value();
  }
  QFile file(canonical_info.absoluteFilePath());
  if (!file.open(QIODevice::ReadOnly)) {
    entry.failure_reason_code = QStringLiteral("unreadable");
    entry.load_failure_reason = file.errorString();
    entry.warning = QStringLiteral("mesh unreadable (reason: %1)").arg(entry.load_failure_reason);
    qWarning().noquote() << QStringLiteral("Scene3D mesh load failed: item_id=%1 mesh_path=%2 canonical_path=%3 loader_reason=%4")
                              .arg(item.id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item.id.trimmed(),
                                   path.trimmed().isEmpty() ? QStringLiteral("<none>") : path.trimmed(),
                                   canonical,
                                   entry.warning);
    return mesh_cache_.insert(canonical, entry).value();
  }
  const QByteArray bytes = file.readAll();
  const QString ext = canonical_info.suffix().toLower();
  QString parse_error;
  if (ext == QStringLiteral("stl")) {
#ifdef WORKCELL_BUILDER_HAS_ASSIMP
    entry.parser_type = QStringLiteral("assimp:stl");
    entry.valid = parse_mesh_with_assimp(canonical, entry.mesh, parse_error, kMeshTriangleLimit);
    if (!entry.valid) {
      entry.mesh.triangles.clear();
      entry.parser_type = QStringLiteral("stl");
      QString fallback_error;
      entry.valid = parse_stl_bytes_for_test(bytes, canonical, entry.mesh, fallback_error, kMeshTriangleLimit);
      if (!entry.valid) parse_error = QStringLiteral("%1; fallback stl parser failed: %2").arg(parse_error, fallback_error);
      else parse_error.clear();
    }
#else
    entry.parser_type = QStringLiteral("stl");
    entry.valid = parse_stl_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
#endif
  } else if (ext == QStringLiteral("dae")) {
#ifdef WORKCELL_BUILDER_HAS_ASSIMP
    entry.parser_type = QStringLiteral("assimp:dae");
    entry.valid = parse_mesh_with_assimp(canonical, entry.mesh, parse_error, kMeshTriangleLimit);
    if (!entry.valid) {
      entry.mesh.triangles.clear();
      entry.parser_type = QStringLiteral("dae");
      QString fallback_error;
      entry.valid = parse_collada_bytes_for_test(bytes, canonical, entry.mesh, fallback_error, &entry.dae_unit_meter, kMeshTriangleLimit);
      if (!entry.valid) parse_error = QStringLiteral("%1; fallback collada parser failed: %2").arg(parse_error, fallback_error);
      else parse_error.clear();
    }
#else
    entry.parser_type = QStringLiteral("dae");
    entry.valid = parse_collada_bytes_for_test(bytes, canonical, entry.mesh, parse_error, &entry.dae_unit_meter, kMeshTriangleLimit);
#endif
  } else if (ext == QStringLiteral("obj")) {
#ifdef WORKCELL_BUILDER_HAS_ASSIMP
    entry.parser_type = QStringLiteral("assimp:obj");
    entry.valid = parse_mesh_with_assimp(canonical, entry.mesh, parse_error, kMeshTriangleLimit);
    if (!entry.valid) {
      entry.mesh.triangles.clear();
      entry.parser_type = QStringLiteral("obj");
      QString fallback_error;
      entry.valid = parse_obj_bytes_for_test(bytes, canonical, entry.mesh, fallback_error, kMeshTriangleLimit);
      if (!entry.valid) parse_error = QStringLiteral("%1; fallback obj parser failed: %2").arg(parse_error, fallback_error);
      else parse_error.clear();
    }
#else
    entry.parser_type = QStringLiteral("obj");
    entry.valid = parse_obj_bytes_for_test(bytes, canonical, entry.mesh, parse_error, kMeshTriangleLimit);
#endif
  } else {
    entry.parser_type = ext;
    entry.valid = false;
    entry.failure_reason_code = QStringLiteral("unsupported_extension");
    parse_error = QStringLiteral("unsupported mesh format: .%1").arg(ext.isEmpty() ? QStringLiteral("<none>") : ext);
  }
  entry.parse_error = parse_error;
  if (entry.valid && entry.mesh.triangles.isEmpty()) {
    entry.valid = false;
    parse_error = QStringLiteral("%1 contains no triangles").arg(entry.parser_type);
    entry.parse_error = parse_error;
  }
  entry.parse_status = entry.valid ? QStringLiteral("ok") : QStringLiteral("error");
  if (!entry.valid) {
    entry.oversized = parse_error.contains("exceeds limit");
    if (entry.failure_reason_code.trimmed().isEmpty()) entry.failure_reason_code = mesh_parse_failure_code(parse_error);
    entry.load_failure_reason = parse_error;
    entry.warning = QStringLiteral("%1 reason_code=%2 (%3)").arg(entry.oversized ? QStringLiteral("mesh oversized") : QStringLiteral("mesh invalid"), entry.failure_reason_code, parse_error);
    qWarning().noquote() << QStringLiteral("Scene3D mesh load failed: item_id=%1 mesh_path=%2 canonical_path=%3 loader_reason=%4")
                              .arg(item.id.trimmed().isEmpty() ? QStringLiteral("<unknown>") : item.id.trimmed(),
                                   path.trimmed().isEmpty() ? QStringLiteral("<none>") : path.trimmed(),
                                   canonical,
                                   entry.warning);
    const bool dae_triangulation_unavailable = ext == QStringLiteral("dae") &&
      (entry.failure_reason_code == QStringLiteral("zero_triangle_mesh") || parse_error.contains(QStringLiteral("triang"), Qt::CaseInsensitive) ||
       parse_error.contains(QStringLiteral("contains no triangles"), Qt::CaseInsensitive));
    if (!entry.oversized && dae_triangulation_unavailable && item_should_use_realsense_visual_surrogate(item, path)) {
      entry.visual_surrogate_available = true;
      entry.visual_surrogate_type = QStringLiteral("realsense_d435_visual_surrogate");
      entry.visual_surrogate_reason = QStringLiteral("visual_surrogate: deterministic RealSense D435/D435i semantic preview; real DAE unavailable or zero-triangle after triangulation");
    }
  } else {
    entry.failure_reason_code.clear();
  }
  if (entry.valid && entry.parser_type == QStringLiteral("dae") && entry.dae_unit_meter > 0.0 && qIsFinite(entry.dae_unit_meter)) {
    Scene3DViewportWidget::InternalTriangleMesh pre_unit_mesh = entry.mesh;
    const float inv_unit = static_cast<float>(1.0 / entry.dae_unit_meter);
    for (auto & tri : pre_unit_mesh.triangles) {
      for (int i = 0; i < 3; ++i) tri.vertices[i] *= inv_unit;
    }
    entry.dae_has_pre_unit_bounds = compute_mesh_bounds_for_test(pre_unit_mesh, entry.dae_pre_unit_min, entry.dae_pre_unit_max);
    if (entry.dae_has_pre_unit_bounds) entry.dae_pre_unit_span = entry.dae_pre_unit_max - entry.dae_pre_unit_min;
  }
  entry.has_bounds = compute_mesh_bounds_for_test(entry.mesh, entry.local_min, entry.local_max);
  if (entry.has_bounds) entry.local_span = entry.local_max - entry.local_min;
  return mesh_cache_.insert(canonical, entry).value();
}





bool Scene3DViewportWidget::validate_mesh_final_span(const ScenePreviewWidget::PreviewItem & it,
                                                     const MeshCacheEntry & entry,
                                                     const QString & mesh_source,
                                                     QString & out_reason,
                                                     QVector3D * out_raw_span,
                                                     QVector3D * out_final_span) const
{
  if (!entry.has_bounds) return true;
  constexpr double kFinalSpanThresholdMeters = 50.0;
  const QVector3D raw_span = entry.local_span;

  QMatrix4x4 final_transform = final_mesh_transform_matrix(it);

  QVector3D final_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D final_max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int xi = 0; xi < 2; ++xi) {
    for (int yi = 0; yi < 2; ++yi) {
      for (int zi = 0; zi < 2; ++zi) {
        const QVector3D corner(xi ? entry.local_max.x() : entry.local_min.x(),
                               yi ? entry.local_max.y() : entry.local_min.y(),
                               zi ? entry.local_max.z() : entry.local_min.z());
        const QVector3D mapped = final_transform.map(corner);
        final_min.setX(qMin(final_min.x(), mapped.x()));
        final_min.setY(qMin(final_min.y(), mapped.y()));
        final_min.setZ(qMin(final_min.z(), mapped.z()));
        final_max.setX(qMax(final_max.x(), mapped.x()));
        final_max.setY(qMax(final_max.y(), mapped.y()));
        final_max.setZ(qMax(final_max.z(), mapped.z()));
      }
    }
  }
  const QVector3D final_span = final_max - final_min;
  if (out_raw_span) *out_raw_span = raw_span;
  if (out_final_span) *out_final_span = final_span;
  const QVector3D abs_final(qAbs(final_span.x()), qAbs(final_span.y()), qAbs(final_span.z()));
  const bool finite_final = qIsFinite(abs_final.x()) && qIsFinite(abs_final.y()) && qIsFinite(abs_final.z()) &&
                            qIsFinite(final_min.x()) && qIsFinite(final_min.y()) && qIsFinite(final_min.z()) &&
                            qIsFinite(final_max.x()) && qIsFinite(final_max.y()) && qIsFinite(final_max.z());
  const double max_final_span = qMax(abs_final.x(), qMax(abs_final.y(), abs_final.z()));
  if (!finite_final || max_final_span > kFinalSpanThresholdMeters) {
    out_reason = QStringLiteral("unreasonable_bounds_final_span item_id=%1 mesh_path=%2 raw_span=[%3,%4,%5] final_span=[%6,%7,%8] threshold_m=%9")
      .arg(it.id, mesh_source)
      .arg(raw_span.x(), 0, 'g', 8).arg(raw_span.y(), 0, 'g', 8).arg(raw_span.z(), 0, 'g', 8)
      .arg(abs_final.x(), 0, 'g', 8).arg(abs_final.y(), 0, 'g', 8).arg(abs_final.z(), 0, 'g', 8)
      .arg(kFinalSpanThresholdMeters, 0, 'g', 8);
    return false;
  }
  return true;
}


namespace {
#include "scene3d_diagnostics.cpp"

bool Scene3DViewportWidget::draw_mesh_preview_if_available(const ScenePreviewWidget::PreviewItem & it, const QColor & color, bool preview_path)
{
  Q_UNUSED(preview_path);
  const auto log_generated_draw = [&](const QString & cache_result, const QString & submitted, const QString & skip_reason,
                                      const QString & mesh_source = QString(), const QString & canonical_source = QString()) {
    if (!(is_generated_urdf_visual_item(it) || is_locked_urdf_item(it))) return;
    qInfo().noquote() << QStringLiteral(
      "Scene3D renderer generated_urdf draw: id=%1 link=%2 canonical_link=%3 source_layer=%4 active_visual_source=%5 type=%6 category=%7 mesh_path=%8 canonical_mesh=%9 cache=%10 submitted_to_draw=%11 skip_reason=%12 applied_scale=[%13,%14,%15]")
      .arg(it.id,
           scene3d_link_name_for_item(it),
           scene3d_canonical_link_name_for_item(it),
           it.source_layer,
           it.active_visual_source,
           it.primitive_geometry_type.trimmed().isEmpty() ? QStringLiteral("<none>") : it.primitive_geometry_type.trimmed(),
           it.category.trimmed().isEmpty() ? QStringLiteral("<none>") : it.category.trimmed(),
           mesh_source.trimmed().isEmpty() ? (!it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path).trimmed() : mesh_source.trimmed(),
           canonical_source.trimmed().isEmpty() ? QStringLiteral("<none>") : canonical_source.trimmed(),
           cache_result,
           submitted,
           skip_reason)
      .arg(it.mesh_scale_x, 0, 'g', 8).arg(it.mesh_scale_y, 0, 'g', 8).arg(it.mesh_scale_z, 0, 'g', 8);
  };

  if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Primitives) {
    log_generated_draw(QStringLiteral("not_attempted"), QStringLiteral("no"), QStringLiteral("mesh_preview_mode_primitives"));
    return false;
  }

  const bool meshes_only_mode = (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes);
  const bool always_surface_mesh_warning = item_should_surface_mesh_warning(it);
  if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Meshes) {
    // explicit branch kept for static mesh-preview contract checks
  } else if (mesh_preview_mode == ScenePreviewWidget::MeshPreviewMode::Auto) {
    // explicit branch kept for static mesh-preview contract checks
  }
  const auto warn_for_mode = [&](const QString & reason, const QString & path) {
    remember_mesh_rejection_reason(it.id, reason);
    if (meshes_only_mode || always_surface_mesh_warning || scene3d_debug_logs_enabled()) warn_mesh_fallback_once(it.id, reason, path);
  };

  if ((is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) && !generated_urdf_item_has_renderable_geometry(it)) {
    log_generated_draw(QStringLiteral("not_attempted"), QStringLiteral("no"), QStringLiteral("not_generated_urdf_renderable_geometry"));
    return false;
  }

  if (!it.has_mesh_metadata) {
    const QString detail = mesh_rejection_diagnostic_detail(it, it.source_path);
    warn_for_mode(QStringLiteral("REJECT_MESH_METADATA_MISSING: mesh metadata missing; %1").arg(detail), it.source_path);
    log_generated_draw(QStringLiteral("missing_metadata"), QStringLiteral("no"), QStringLiteral("REJECT_MESH_METADATA_MISSING"), it.source_path);
    return false;
  }

  const QString mesh_source = !it.mesh_path.trimmed().isEmpty() ? it.mesh_path : it.source_path;
  if (mesh_source.trimmed().isEmpty()) {
    const QString detail = mesh_rejection_diagnostic_detail(it, mesh_source);
    warn_for_mode(QStringLiteral("REJECT_MESH_SOURCE_MISSING: mesh source missing; %1").arg(detail), mesh_source);
    log_generated_draw(QStringLiteral("missing_source"), QStringLiteral("no"), QStringLiteral("REJECT_MESH_SOURCE_MISSING"), mesh_source);
    return false;
  }
  QString canonical_mesh_source;
  QString resolve_failure_reason;
  if (!try_resolve_canonical_mesh_path(mesh_source, canonical_mesh_source, &it, &resolve_failure_reason)) {
    canonical_mesh_source = QFileInfo(mesh_source).absoluteFilePath();
  }
  const auto cache_it = mesh_cache_.constFind(canonical_mesh_source);
  if (cache_it == mesh_cache_.constEnd()) {
    const QString detail = mesh_rejection_diagnostic_detail(it, mesh_source, canonical_mesh_source, nullptr,
      resolve_failure_reason.trimmed().isEmpty() ? QString() : QStringLiteral("resolve_failure_reason=%1").arg(resolve_failure_reason.trimmed()));
    const QString reason = QStringLiteral("REJECT_MESH_CACHE_MISS: mesh cache entry missing; waiting for controlled preview ingest/reload; %1").arg(detail);
    remember_mesh_rejection_reason(it.id, reason);
    warn_mesh_fallback_once(it.id, reason, mesh_source);
    log_generated_draw(QStringLiteral("cache_miss"), QStringLiteral("no"), QStringLiteral("REJECT_MESH_CACHE_MISS"), mesh_source, canonical_mesh_source);
    return false;
  }
  const MeshCacheEntry & entry = cache_it.value();
  if (scene3d_canonical_link_name_for_item(it) == QStringLiteral("base_link_inertia")) {
    qInfo().noquote() << QStringLiteral("Scene3D base_link_inertia trace: stage=mesh_cache_load item_id=%1 requested_path=%2 canonical_path=%3 loaded=%4 valid=%5 triangles=%6 path_resolved=%7 reason=%8")
      .arg(it.id, mesh_source, canonical_mesh_source, entry.loaded ? QStringLiteral("true") : QStringLiteral("false"),
           entry.valid ? QStringLiteral("true") : QStringLiteral("false"))
      .arg(static_cast<int>(entry.mesh.triangles.size()))
      .arg(entry.path_resolved ? QStringLiteral("true") : QStringLiteral("false"),
           entry.failure_reason_code.trimmed().isEmpty() ? QStringLiteral("ok") : entry.failure_reason_code.trimmed());
  }
  auto reject = [&](const QString & code, const QString & detail = QString()) {
    const QString diagnostics = mesh_rejection_diagnostic_detail(it, mesh_source, canonical_mesh_source, &entry, detail);
    const QString reason = QStringLiteral("%1: %2").arg(code, diagnostics);
    warn_for_mode(reason, mesh_source);
    log_generated_draw(
      QStringLiteral("loaded=%1 valid=%2 triangles=%3 parser=%4 reason=%5")
        .arg(entry.loaded ? QStringLiteral("true") : QStringLiteral("false"),
             entry.valid ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(static_cast<int>(entry.mesh.triangles.size()))
        .arg(entry.parser_type.trimmed().isEmpty() ? QStringLiteral("<none>") : entry.parser_type,
             code),
      QStringLiteral("no"), code, mesh_source, canonical_mesh_source);
    return false;
  };
  if (!entry.loaded || entry.oversized || !entry.valid || entry.mesh.triangles.isEmpty()) {
    if (entry.visual_surrogate_available && item_should_use_realsense_visual_surrogate(it, mesh_source)) {
      const QString reason = QStringLiteral("visual_surrogate: %1 source_mesh_status=%2 warning=%3")
        .arg(entry.visual_surrogate_type,
             entry.failure_reason_code.trimmed().isEmpty() ? QStringLiteral("zero_triangle_mesh") : entry.failure_reason_code,
             entry.warning);
      remember_mesh_rejection_reason(it.id, reason);
      draw_realsense_d435_visual_surrogate(it);
      return true;
    }
    if (!entry.loaded) return reject(QStringLiteral("parse_failed"), QStringLiteral("loaded=false"));
    if (entry.oversized) {
      warn_for_mode(QStringLiteral("triangle_budget_exceeded_preview_surrogate: %1").arg(mesh_rejection_diagnostic_detail(it, mesh_source, canonical_mesh_source, &entry, entry.warning)), mesh_source);
      if (classify_item_role(it) == NormalizedRole::Table && draw_clean_semantic_primitive(it)) return true;
      return false;
    }
    if (!entry.valid) return reject(entry.failure_reason_code.trimmed().isEmpty() ? QStringLiteral("parse_failed") : entry.failure_reason_code, entry.warning);
    return reject(QStringLiteral("zero_triangle_mesh"), entry.warning);
  }

  glPushMatrix();
  const QMatrix4x4 final_draw_transform = final_mesh_transform_matrix(it);
  const QString canonical_link = scene3d_canonical_link_name_for_item(it);
  const bool log_ur5_final_transform = (is_generated_urdf_visual_item(it) || is_locked_urdf_item(it)) &&
      it.has_baked_world_visual_transform &&
      is_required_generated_robot_viewport_link(it);
  glMultMatrixf(final_draw_transform.constData());

  const MeshCacheEntry & cache = entry;  // ensure_mesh_cached(it, mesh_source) is intentionally upstream of final draw.
  if (log_ur5_final_transform && cache.has_bounds) {
    QVector3D final_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    QVector3D final_max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    for (int xi = 0; xi < 2; ++xi) {
      for (int yi = 0; yi < 2; ++yi) {
        for (int zi = 0; zi < 2; ++zi) {
          const QVector3D local_corner(xi == 0 ? cache.local_min.x() : cache.local_max.x(),
                                       yi == 0 ? cache.local_min.y() : cache.local_max.y(),
                                       zi == 0 ? cache.local_min.z() : cache.local_max.z());
          const QVector3D final_corner = final_draw_transform * local_corner;
          final_min.setX(qMin(final_min.x(), final_corner.x()));
          final_min.setY(qMin(final_min.y(), final_corner.y()));
          final_min.setZ(qMin(final_min.z(), final_corner.z()));
          final_max.setX(qMax(final_max.x(), final_corner.x()));
          final_max.setY(qMax(final_max.y(), final_corner.y()));
          final_max.setZ(qMax(final_max.z(), final_corner.z()));
        }
      }
    }
    const QVector3D final_center = (final_min + final_max) * 0.5f;
    qInfo().noquote() << QStringLiteral("UR5_FINAL_TRANSFORM link=%1 baked_xyz=[%2,%3,%4] baked_rpy=[%5,%6,%7] final_center=[%8,%9,%10] bbox_min=[%11,%12,%13] bbox_max=[%14,%15,%16]")
      .arg(canonical_link)
      .arg(it.x, 0, 'g', 8).arg(it.y, 0, 'g', 8).arg(it.z, 0, 'g', 8)
      .arg(it.roll, 0, 'g', 8).arg(it.pitch, 0, 'g', 8).arg(it.yaw, 0, 'g', 8)
      .arg(final_center.x(), 0, 'g', 8).arg(final_center.y(), 0, 'g', 8).arg(final_center.z(), 0, 'g', 8)
      .arg(final_min.x(), 0, 'g', 8).arg(final_min.y(), 0, 'g', 8).arg(final_min.z(), 0, 'g', 8)
      .arg(final_max.x(), 0, 'g', 8).arg(final_max.y(), 0, 'g', 8).arg(final_max.z(), 0, 'g', 8);
  }
  if (!cache.valid || cache.mesh.triangles.isEmpty()) {
    glPopMatrix();
    log_generated_draw(QStringLiteral("valid=false_or_zero_triangles"), QStringLiteral("no"), QStringLiteral("invalid_cache_after_transform"), mesh_source, canonical_mesh_source);
    return false;
  }
  log_generated_draw(
    QStringLiteral("loaded=%1 valid=%2 triangles=%3 parser=%4")
      .arg(cache.loaded ? QStringLiteral("true") : QStringLiteral("false"),
           cache.valid ? QStringLiteral("true") : QStringLiteral("false"))
      .arg(static_cast<int>(cache.mesh.triangles.size()))
      .arg(cache.parser_type.trimmed().isEmpty() ? QStringLiteral("<none>") : cache.parser_type),
    QStringLiteral("yes"), QStringLiteral("none"), mesh_source, canonical_mesh_source);

  const QVector3D default_up_normal(0.0f, 1.0f, 0.0f);
  glBegin(GL_TRIANGLES);
  for (const auto & tri : cache.mesh.triangles) {
    QVector3D normal = tri.normal;
    const bool normal_invalid = !qIsFinite(normal.x()) || !qIsFinite(normal.y()) || !qIsFinite(normal.z()) ||
                                normal.lengthSquared() <= 1e-12f;
    if (normal_invalid) {
      const QVector3D edge0 = tri.vertices[1] - tri.vertices[0];
      const QVector3D edge1 = tri.vertices[2] - tri.vertices[0];
      normal = QVector3D::crossProduct(edge0, edge1);
    }
    if (qIsFinite(normal.x()) && qIsFinite(normal.y()) && qIsFinite(normal.z()) && normal.lengthSquared() > 1e-12f) {
      normal.normalize();
    } else {
      normal = default_up_normal;
    }

    glNormal3f(normal.x(), normal.y(), normal.z());
    glColor4f(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    glVertex3f(tri.vertices[0].x(), tri.vertices[0].y(), tri.vertices[0].z());
    glVertex3f(tri.vertices[1].x(), tri.vertices[1].y(), tri.vertices[1].z());
    glVertex3f(tri.vertices[2].x(), tri.vertices[2].y(), tri.vertices[2].z());
  }
  glEnd();

  glPopMatrix();
  return true;
}

void Scene3DViewportWidget::draw_ground_grid_pass()
{
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  const int grid_extent = debug_overlays_mode ? 20 : 6;
  const QColor floor = QColor(15, 23, 42, debug_overlays_mode ? 38 : 24);
  glColor4f(floor.redF(), floor.greenF(), floor.blueF(), floor.alphaF());
  glBegin(GL_QUADS);
  glVertex3f(static_cast<float>(-grid_extent), -0.002f, static_cast<float>(-grid_extent));
  glVertex3f(static_cast<float>( grid_extent), -0.002f, static_cast<float>(-grid_extent));
  glVertex3f(static_cast<float>( grid_extent), -0.002f, static_cast<float>( grid_extent));
  glVertex3f(static_cast<float>(-grid_extent), -0.002f, static_cast<float>( grid_extent));
  glEnd();
  glLineWidth(debug_overlays_mode ? 1.0f : 0.85f);
  glBegin(GL_LINES);
  for (int i = -grid_extent; i <= grid_extent; ++i) {
    const bool major = (i % 5 == 0);
    const QColor c = debug_overlays_mode
      ? (major ? QColor(100, 116, 139, 140) : QColor(71, 85, 105, 80))
      : (major ? QColor(100, 116, 139, 46) : QColor(71, 85, 105, 18));
    glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
    glVertex3f(static_cast<float>(i), 0.0f, static_cast<float>(-grid_extent)); glVertex3f(static_cast<float>(i), 0.0f, static_cast<float>(grid_extent));
    glVertex3f(static_cast<float>(-grid_extent), 0.0f, static_cast<float>(i)); glVertex3f(static_cast<float>(grid_extent), 0.0f, static_cast<float>(i));
  }
  glEnd();
  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
}

void Scene3DViewportWidget::draw_world_axes_pass()
{
  glLineWidth(2.4f);
  glBegin(GL_LINES);
  glColor4f(0.95f, 0.35f, 0.35f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.45f, 0.0f, 0.0f);
  glColor4f(0.35f, 0.95f, 0.35f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.45f, 0.0f);
  glColor4f(0.35f, 0.65f, 0.98f, 1.0f); glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 0.45f);
  glEnd();
  QPainter painter(this);
  painter.setPen(QColor("#f8fafc"));
  painter.drawText(project_to_screen(0.50, 0.0, 0.0), "X");
  painter.drawText(project_to_screen(0.0, 0.50, 0.0), "Y");
  painter.drawText(project_to_screen(0.0, 0.0, 0.50), "Z");
  if (debug_overlays_mode) painter.drawText(QPointF(10.0, height() - 14.0), "Scale: major grid 5 m");
}

void Scene3DViewportWidget::draw_unit_cube_triangles(const QColor & color)
{
  glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0f);
  glBegin(GL_TRIANGLES);
  const GLfloat v[8][3] = {
    {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {1.f, 1.f, 0.f}, {0.f, 1.f, 0.f},
    {0.f, 0.f, 1.f}, {1.f, 0.f, 1.f}, {1.f, 1.f, 1.f}, {0.f, 1.f, 1.f}
  };
  auto tri = [&](int a, int b, int c){ glVertex3fv(v[a]); glVertex3fv(v[b]); glVertex3fv(v[c]); };
  tri(0,1,2); tri(0,2,3);
  tri(4,6,5); tri(4,7,6);
  tri(0,4,5); tri(0,5,1);
  tri(1,5,6); tri(1,6,2);
  tri(2,6,7); tri(2,7,3);
  tri(3,7,4); tri(3,4,0);
  glEnd();
}

void Scene3DViewportWidget::draw_robot_base_with_axis(const ScenePreviewWidget::PreviewItem & it)
{
  const double radius = qMax(0.06, qMin(it.sx, it.sz) * 0.45);
  draw_cylinder(it.x + it.sx * 0.5, it.y, it.z + it.sz * 0.5, radius, qMax(0.05, it.sy), item_color(it));
  glColor4f(0.96f, 0.97f, 0.99f, 1.0f);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex3f(it.x + it.sx * 0.5, it.y, it.z + it.sz * 0.5);
  glVertex3f(it.x + it.sx * 0.5, it.y + qMax(0.2, it.sy * 2.0), it.z + it.sz * 0.5);
  glEnd();
}
void Scene3DViewportWidget::draw_table_slab(const ScenePreviewWidget::PreviewItem & it)
{
  // support_surface/table: render as a valid semantic physical primitive, not an
  // unknown-geometry placeholder.  The tabletop keeps the item's existing
  // lower-corner coordinate convention: it.x/it.y/it.z is the tabletop min XYZ.
  const double tabletop_sx = it.sx > 0.001 ? it.sx : 0.80;
  const double tabletop_sy = it.sy > 0.001 ? it.sy : 0.06;
  const double tabletop_sz = it.sz > 0.001 ? it.sz : 0.60;
  const double leg_width = qBound(0.025, qMin(tabletop_sx, tabletop_sz) * 0.055, 0.075);
  const double leg_height = qBound(0.35, qMax(tabletop_sx, tabletop_sz) * 0.55, 0.85);
  const double inset_x = qMin(qMax(leg_width, tabletop_sx * 0.08), qMax(0.0, tabletop_sx * 0.5 - leg_width));
  const double inset_z = qMin(qMax(leg_width, tabletop_sz * 0.08), qMax(0.0, tabletop_sz * 0.5 - leg_width));
  const double leg_y = it.y - leg_height;
  const double rail_height = qMax(0.018, leg_width * 0.55);
  const double rail_y = leg_y + leg_height * 0.28;

  const QColor tabletop_fill(126, 118, 105, 228);
  const QColor frame_fill(47, 57, 70, 232);
  const QColor outline(203, 213, 225, 96);
  const QColor frame_outline(148, 163, 184, 86);

  draw_box(it.x, it.y, it.z, tabletop_sx, tabletop_sy, tabletop_sz, tabletop_fill, false);
  draw_box_outline(it.x, it.y, it.z, tabletop_sx, tabletop_sy, tabletop_sz, outline, 0.85f);

  const double left_x = it.x + inset_x;
  const double right_x = it.x + tabletop_sx - inset_x - leg_width;
  const double front_z = it.z + inset_z;
  const double back_z = it.z + tabletop_sz - inset_z - leg_width;
  draw_box(left_x, leg_y, front_z, leg_width, leg_height, leg_width, frame_fill, false);
  draw_box(right_x, leg_y, front_z, leg_width, leg_height, leg_width, frame_fill, false);
  draw_box(left_x, leg_y, back_z, leg_width, leg_height, leg_width, frame_fill, false);
  draw_box(right_x, leg_y, back_z, leg_width, leg_height, leg_width, frame_fill, false);

  const double rail_x = left_x;
  const double rail_sx = qMax(leg_width, right_x - left_x + leg_width);
  const double rail_z = front_z;
  const double rail_sz = qMax(leg_width, back_z - front_z + leg_width);
  draw_box(rail_x, rail_y, front_z, rail_sx, rail_height, leg_width, frame_fill, false);
  draw_box(rail_x, rail_y, back_z, rail_sx, rail_height, leg_width, frame_fill, false);
  draw_box(left_x, rail_y, rail_z, leg_width, rail_height, rail_sz, frame_fill, false);
  draw_box(right_x, rail_y, rail_z, leg_width, rail_height, rail_sz, frame_fill, false);
  draw_box_outline(rail_x, rail_y, rail_z, rail_sx, rail_height, rail_sz, frame_outline, 0.65f);
}

void Scene3DViewportWidget::draw_conveyor(const ScenePreviewWidget::PreviewItem & it)
{
  // conveyor: belt rectangle with a direction arrow in the local +X direction.
  const QColor belt_fill(8, 145, 178, 120);
  const QColor belt_line(165, 243, 252, 170);
  draw_box(it.x, it.y, it.z, qMax(0.05, it.sx), qMax(0.02, it.sy), qMax(0.05, it.sz), belt_fill, true);
  draw_box_outline(it.x, it.y, it.z, qMax(0.05, it.sx), qMax(0.02, it.sy), qMax(0.05, it.sz), belt_line, 1.0f);
  const double top_y = it.y + qMax(0.02, it.sy) + 0.012;
  const double start_x = it.x + qMax(0.05, it.sx) * 0.24;
  const double end_x = it.x + qMax(0.05, it.sx) * 0.76;
  const double mid_z = it.z + qMax(0.05, it.sz) * 0.5;
  const double arrow = qMin(0.12, qMax(0.04, qMax(0.05, it.sx) * 0.08));
  glColor4f(belt_line.redF(), belt_line.greenF(), belt_line.blueF(), 0.86f);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex3f(start_x, top_y, mid_z); glVertex3f(end_x, top_y, mid_z);
  glVertex3f(end_x, top_y, mid_z); glVertex3f(end_x - arrow, top_y, mid_z - arrow * 0.65);
  glVertex3f(end_x, top_y, mid_z); glVertex3f(end_x - arrow, top_y, mid_z + arrow * 0.65);
  glEnd();
}

void Scene3DViewportWidget::draw_realsense_d435_visual_surrogate(const ScenePreviewWidget::PreviewItem & it)
{
  // Preview-safe visual_surrogate for RealSense D435/D435i meshes whose DAE cannot be triangulated.
  // It uses the same final mesh placement path as real meshes.
  glPushMatrix();
  const QMatrix4x4 final_draw_transform = final_mesh_transform_matrix(it);
  glMultMatrixf(final_draw_transform.constData());

  const QColor body(31, 41, 55, 232);
  const QColor face(56, 189, 248, 220);
  const QColor glass(14, 165, 233, 220);
  const QColor line(224, 242, 254, 185);
  const double w = 0.090;
  const double h = 0.025;
  const double d = 0.025;
  draw_box(-w * 0.5, -h * 0.5, -d * 0.5, w, h, d, body, false);
  draw_box_outline(-w * 0.5, -h * 0.5, -d * 0.5, w, h, d, line, 1.0f);

  // Front accent strip and three imager/lens cues on the camera face.
  draw_box(-w * 0.44, -h * 0.55, d * 0.50, w * 0.88, h * 0.10, 0.004, face, false);
  const double face_z = d * 0.58;
  draw_sphere(-0.028, 0.002, face_z, 0.0075, glass, false, 18, 8);
  draw_sphere(0.000, 0.002, face_z, 0.0060, QColor(15, 23, 42, 240), false, 18, 8);
  draw_sphere(0.028, 0.002, face_z, 0.0075, glass, false, 18, 8);

  // Small mount cue below the body.
  draw_box(-0.018, -h * 0.78, -0.006, 0.036, 0.008, 0.012, QColor(71, 85, 105, 220), false);
  draw_box_outline(-0.018, -h * 0.78, -0.006, 0.036, 0.008, 0.012, QColor(203, 213, 225, 145), 0.8f);

  // Deterministic label tick: labels are also supplied by the viewport 2D label pass when enabled.
  glColor4f(line.redF(), line.greenF(), line.blueF(), 0.92f);
  glLineWidth(1.2f);
  glBegin(GL_LINES);
  glVertex3f(-0.035f, 0.023f, 0.0f); glVertex3f(0.035f, 0.023f, 0.0f);
  glVertex3f(-0.035f, 0.023f, 0.0f); glVertex3f(-0.028f, 0.030f, 0.0f);
  glVertex3f(0.035f, 0.023f, 0.0f); glVertex3f(0.028f, 0.030f, 0.0f);
  glEnd();

  glPopMatrix();
}

void Scene3DViewportWidget::draw_camera_body_with_frustum(const ScenePreviewWidget::PreviewItem & it)
{
  // camera: independent Workcell Studio cue, not an EPD/RealSense runtime dependency.
  const double sx = qMax(0.06, it.sx > 0.001 ? it.sx : 0.10);
  const double sy = qMax(0.04, it.sy > 0.001 ? it.sy : 0.06);
  const double sz = qMax(0.04, it.sz > 0.001 ? it.sz : 0.07);
  const QColor body(17, 24, 39, 225);
  const QColor frustum(125, 211, 252, 95);
  draw_box(it.x, it.y, it.z, sx, sy, sz, body, false);
  draw_box_outline(it.x, it.y, it.z, sx, sy, sz, QColor(224, 242, 254, 160), 0.9f);

  const double lens_x = it.x + sx;
  const double lens_y = it.y + sy * 0.5;
  const double lens_z = it.z + sz * 0.5;
  const double range = qMax(0.25, sx * 2.8);
  const double half_w = qMax(0.10, sz * 1.4);
  const double half_h = qMax(0.08, sy * 1.6);
  const double far_x = lens_x + range;
  glColor4f(frustum.redF(), frustum.greenF(), frustum.blueF(), 0.62f);
  glLineWidth(1.1f);
  glBegin(GL_LINES);
  glVertex3f(lens_x, lens_y, lens_z); glVertex3f(far_x, lens_y - half_h, lens_z - half_w);
  glVertex3f(lens_x, lens_y, lens_z); glVertex3f(far_x, lens_y - half_h, lens_z + half_w);
  glVertex3f(lens_x, lens_y, lens_z); glVertex3f(far_x, lens_y + half_h, lens_z - half_w);
  glVertex3f(lens_x, lens_y, lens_z); glVertex3f(far_x, lens_y + half_h, lens_z + half_w);
  glVertex3f(far_x, lens_y - half_h, lens_z - half_w); glVertex3f(far_x, lens_y - half_h, lens_z + half_w);
  glVertex3f(far_x, lens_y - half_h, lens_z + half_w); glVertex3f(far_x, lens_y + half_h, lens_z + half_w);
  glVertex3f(far_x, lens_y + half_h, lens_z + half_w); glVertex3f(far_x, lens_y + half_h, lens_z - half_w);
  glVertex3f(far_x, lens_y + half_h, lens_z - half_w); glVertex3f(far_x, lens_y - half_h, lens_z - half_w);
  glEnd();
}

void Scene3DViewportWidget::draw_pick_zone(const ScenePreviewWidget::PreviewItem & it)
{
  // pick/place zones: translucent floor/table overlays; labels are supplied by the 2D label pass.
  const double h = qMax(0.006, qMin(it.sy > 0.001 ? it.sy : 0.01, 0.025));
  draw_box(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(34, 197, 94, 46), true);
  draw_box_outline(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(134, 239, 172, 175), 1.6f);
}

void Scene3DViewportWidget::draw_place_zone(const ScenePreviewWidget::PreviewItem & it)
{
  // place_zone: purple translucent floor/table overlay with a label through the 2D label pass.
  const double h = qMax(0.006, qMin(it.sy > 0.001 ? it.sy : 0.01, 0.025));
  draw_box(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(168, 85, 247, 44), true);
  draw_box_outline(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(216, 180, 254, 175), 1.6f);
}

void Scene3DViewportWidget::draw_place_target_bin(const ScenePreviewWidget::PreviewItem & it)
{
  // bin/place target: translucent container-like box with visible rim and inner cavity.
  const double sx = qMax(0.06, it.sx);
  const double sy = qMax(0.04, it.sy);
  const double sz = qMax(0.06, it.sz);
  draw_box(it.x, it.y, it.z, sx, sy, sz, QColor(251, 113, 133, 52), true);
  draw_box_outline(it.x, it.y, it.z, sx, sy, sz, QColor(254, 205, 211, 135), 1.1f);
  const double wall = qMax(0.015, qMin(sx, sz) * 0.10);
  draw_box_outline(it.x + wall, it.y + wall, it.z + wall,
                   qMax(0.01, sx - 2 * wall), qMax(0.01, sy - wall), qMax(0.01, sz - 2 * wall),
                   QColor(254, 205, 211, 125), 0.85f);
}

void Scene3DViewportWidget::draw_safety_zone(const ScenePreviewWidget::PreviewItem & it)
{
  // safety zone: amber translucent boundary. This is visual only; it does not certify safety.
  const double h = qMax(0.006, qMin(it.sy > 0.001 ? it.sy : 0.012, 0.035));
  draw_box(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(245, 158, 11, 36), true);
  draw_box_outline(it.x, it.y, it.z, qMax(0.05, it.sx), h, qMax(0.05, it.sz), QColor(251, 191, 36, 170), 1.8f);
}

void Scene3DViewportWidget::draw_home_pose_marker(const ScenePreviewWidget::PreviewItem & it)
{
  // home/safety pose: small labeled pose marker / axis triad; labels are supplied by the 2D label pass.
  const double cx = it.x + qMax(0.0, it.sx) * 0.5;
  const double cy = it.y + qMax(0.0, it.sy) * 0.5;
  const double cz = it.z + qMax(0.0, it.sz) * 0.5;
  const double axis = qMax(0.10, qMax(qMax(it.sx, it.sy), it.sz));
  draw_box(cx - 0.018, cy - 0.018, cz - 0.018, 0.036, 0.036, 0.036, QColor(226, 232, 240, 210), false);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glColor4f(0.95f, 0.20f, 0.20f, 1.0f); glVertex3f(cx, cy, cz); glVertex3f(cx + axis, cy, cz);
  glColor4f(0.20f, 0.85f, 0.35f, 1.0f); glVertex3f(cx, cy, cz); glVertex3f(cx, cy + axis, cz);
  glColor4f(0.25f, 0.45f, 1.0f, 1.0f); glVertex3f(cx, cy, cz); glVertex3f(cx, cy, cz + axis);
  glEnd();
}

void Scene3DViewportWidget::draw_object_cube(const ScenePreviewWidget::PreviewItem & it)
{
  if (draw_mesh_preview_if_available(it, item_color(it), true)) return;
  const double cube = qMax(0.05, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, item_color(it));
}
void Scene3DViewportWidget::draw_missing_geometry_marker(const ScenePreviewWidget::PreviewItem & it)
{
  const double marker = 0.045;
  if (!scene3d_debug_fallback_boxes_enabled()) return;
  draw_box(it.x, it.y, it.z, marker, marker, marker, QColor(239, 68, 68, 96), true);
  draw_box_outline(it.x, it.y, it.z, marker, marker, marker, QColor(252, 165, 165, 150), 1.0f);
}
void Scene3DViewportWidget::draw_warning_badge_anchor(const ScenePreviewWidget::PreviewItem & it)
{
  const double cube = qMax(0.04, qMin(it.sx, qMin(it.sy, it.sz)));
  draw_box(it.x, it.y, it.z, cube, cube, cube, QColor("#f59e0b"));
}

bool Scene3DViewportWidget::draw_clean_semantic_primitive(const ScenePreviewWidget::PreviewItem & it)
{
  const NormalizedRole role = classify_item_role(it);
  if (!is_clean_semantic_primitive_role(role)) return false;

  const bool has_dimensions = item_has_explicit_dimensions(it);
  ScenePreviewWidget::PreviewItem draw_item = it;
  if (!has_dimensions) {
    // Semantic roles may be authored before full geometry is available. Render a
    // small, role-specific cue instead of a red unknown-geometry marker. Truly
    // unknown items still fall through to missing-geometry diagnostics.
    switch (role) {
      case NormalizedRole::Camera:
        draw_item.sx = 0.10; draw_item.sy = 0.06; draw_item.sz = 0.07;
        break;
      case NormalizedRole::RobotBase:
        draw_item.sx = 0.16; draw_item.sy = 0.16; draw_item.sz = 0.04;
        break;
      case NormalizedRole::HomePose:
        draw_item.sx = 0.12; draw_item.sy = 0.12; draw_item.sz = 0.12;
        break;
      case NormalizedRole::RobotReach:
      case NormalizedRole::SafetyZone:
      case NormalizedRole::PickZone:
      case NormalizedRole::PlaceZone:
        draw_item.sx = 0.60; draw_item.sy = 0.01; draw_item.sz = 0.60;
        break;
      case NormalizedRole::Table:
        draw_item.sx = 0.80; draw_item.sy = 0.05; draw_item.sz = 0.60;
        break;
      case NormalizedRole::PlaceBin:
        draw_item.sx = 0.35; draw_item.sy = 0.18; draw_item.sz = 0.25;
        break;
      case NormalizedRole::Conveyor:
        draw_item.sx = 0.80; draw_item.sy = 0.05; draw_item.sz = 0.28;
        break;
      case NormalizedRole::Object:
        draw_item.sx = 0.06; draw_item.sy = 0.06; draw_item.sz = 0.06;
        break;
      default:
        return false;
    }
  }

  switch (role) {
    case NormalizedRole::RobotBase:
      draw_home_pose_marker(draw_item);
      return true;
    case NormalizedRole::RobotReach:
      draw_safety_zone(draw_item);
      return true;
    case NormalizedRole::Table:
      draw_table_slab(draw_item);
      return true;
    case NormalizedRole::PlaceBin:
      draw_place_target_bin(draw_item);
      return true;
    case NormalizedRole::Conveyor:
      draw_conveyor(draw_item);
      return true;
    case NormalizedRole::PickZone:
      draw_pick_zone(draw_item);
      return true;
    case NormalizedRole::PlaceZone:
      draw_place_zone(draw_item);
      return true;
    case NormalizedRole::Camera:
      if (item_should_use_realsense_visual_surrogate(draw_item)) {
        draw_realsense_d435_visual_surrogate(draw_item);
      } else {
        draw_camera_body_with_frustum(draw_item);
      }
      return true;
    case NormalizedRole::SafetyZone:
      draw_safety_zone(draw_item);
      return true;
    case NormalizedRole::HomePose:
      draw_home_pose_marker(draw_item);
      return true;
    case NormalizedRole::Object: {
      QColor fill = item_color(draw_item);
      QColor line = fill.lighter(130);
      fill.setAlpha(64);
      line.setAlpha(128);
      if (item_has_explicit_dimensions(draw_item)) {
        draw_box(draw_item.x, draw_item.y, draw_item.z, draw_item.sx, draw_item.sy, draw_item.sz, fill, true);
        draw_box_outline(draw_item.x, draw_item.y, draw_item.z, draw_item.sx, draw_item.sy, draw_item.sz, line, 0.9f);
        return true;
      }
      return draw_urdf_primitive_geometry(draw_item, fill);
    }
    default:
      return false;
  }
}

void Scene3DViewportWidget::draw_box(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, bool translucent)
{
  glDisable(GL_CULL_FACE);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());
  const double x = cx, y = cy, z = cz;
  glBegin(GL_QUADS);
  glVertex3f(x, y, z); glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x, y + sy, z);
  glVertex3f(x, y, z + sz); glVertex3f(x, y + sy, z + sz); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x, y, z); glVertex3f(x, y, z + sz); glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y, z);
  glVertex3f(x, y + sy, z); glVertex3f(x + sx, y + sy, z); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x, y + sy, z + sz);
  glVertex3f(x, y, z); glVertex3f(x, y + sy, z); glVertex3f(x, y + sy, z + sz); glVertex3f(x, y, z + sz);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x + sx, y + sy, z);
  glEnd();
  glEnable(GL_CULL_FACE);
}
void Scene3DViewportWidget::draw_box_outline(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, float line_width)
{
  glDisable(GL_CULL_FACE);
  glLineWidth(line_width);
  glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0f);
  const double x = cx, y = cy, z = cz;
  glBegin(GL_LINES);
  glVertex3f(x, y, z); glVertex3f(x + sx, y, z);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y + sy, z);
  glVertex3f(x + sx, y + sy, z); glVertex3f(x, y + sy, z);
  glVertex3f(x, y + sy, z); glVertex3f(x, y, z);

  glVertex3f(x, y, z + sz); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x + sx, y, z + sz); glVertex3f(x + sx, y + sy, z + sz);
  glVertex3f(x + sx, y + sy, z + sz); glVertex3f(x, y + sy, z + sz);
  glVertex3f(x, y + sy, z + sz); glVertex3f(x, y, z + sz);

  glVertex3f(x, y, z); glVertex3f(x, y, z + sz);
  glVertex3f(x + sx, y, z); glVertex3f(x + sx, y, z + sz);
  glVertex3f(x + sx, y + sy, z); glVertex3f(x + sx, y + sy, z + sz);
  glVertex3f(x, y + sy, z); glVertex3f(x, y + sy, z + sz);
  glEnd();
  glEnable(GL_CULL_FACE);
}
void Scene3DViewportWidget::draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color, bool translucent, int segment_count)
{
  const int segments = qMax(3, segment_count);
  const double safe_radius = qMax(0.0, radius);
  const double y_bottom = cy;
  const double y_top = cy + height;

  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());

  // Side wall. The cylinder is Y-up to match the viewport's box helper, where
  // the height axis starts at cy and extends by the supplied height argument.
  glBegin(GL_QUADS);
  for (int i = 0; i < segments; ++i) {
    const double a0 = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    const double a1 = 2.0 * M_PI * static_cast<double>(i + 1) / static_cast<double>(segments);
    const double x0 = cx + safe_radius * qCos(a0);
    const double z0 = cz + safe_radius * qSin(a0);
    const double x1 = cx + safe_radius * qCos(a1);
    const double z1 = cz + safe_radius * qSin(a1);
    glVertex3f(static_cast<GLfloat>(x0), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(z0));
    glVertex3f(static_cast<GLfloat>(x0), static_cast<GLfloat>(y_top), static_cast<GLfloat>(z0));
    glVertex3f(static_cast<GLfloat>(x1), static_cast<GLfloat>(y_top), static_cast<GLfloat>(z1));
    glVertex3f(static_cast<GLfloat>(x1), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(z1));
  }
  glEnd();

  // Bottom cap.
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(static_cast<GLfloat>(cx), static_cast<GLfloat>(y_bottom), static_cast<GLfloat>(cz));
  for (int i = 0; i <= segments; ++i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex3f(static_cast<GLfloat>(cx + safe_radius * qCos(a)),
               static_cast<GLfloat>(y_bottom),
               static_cast<GLfloat>(cz + safe_radius * qSin(a)));
  }
  glEnd();

  // Top cap.
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(static_cast<GLfloat>(cx), static_cast<GLfloat>(y_top), static_cast<GLfloat>(cz));
  for (int i = segments; i >= 0; --i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(segments);
    glVertex3f(static_cast<GLfloat>(cx + safe_radius * qCos(a)),
               static_cast<GLfloat>(y_top),
               static_cast<GLfloat>(cz + safe_radius * qSin(a)));
  }
  glEnd();
}

void Scene3DViewportWidget::draw_sphere(double cx, double cy, double cz, double radius, const QColor & color, bool translucent, int slice_count, int stack_count)
{
  const int slices = qMax(6, slice_count);
  const int stacks = qMax(4, stack_count);
  const double r = qMax(0.0, radius);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? qMin(0.35f, static_cast<float>(color.alphaF())) : color.alphaF());
  for (int stack = 0; stack < stacks; ++stack) {
    const double phi0 = -M_PI_2 + M_PI * static_cast<double>(stack) / static_cast<double>(stacks);
    const double phi1 = -M_PI_2 + M_PI * static_cast<double>(stack + 1) / static_cast<double>(stacks);
    glBegin(GL_QUAD_STRIP);
    for (int slice = 0; slice <= slices; ++slice) {
      const double theta = 2.0 * M_PI * static_cast<double>(slice) / static_cast<double>(slices);
      const double c = qCos(theta);
      const double s = qSin(theta);
      glVertex3f(static_cast<GLfloat>(cx + r * qCos(phi0) * c),
                 static_cast<GLfloat>(cy + r * qSin(phi0)),
                 static_cast<GLfloat>(cz + r * qCos(phi0) * s));
      glVertex3f(static_cast<GLfloat>(cx + r * qCos(phi1) * c),
                 static_cast<GLfloat>(cy + r * qSin(phi1)),
                 static_cast<GLfloat>(cz + r * qCos(phi1) * s));
    }
    glEnd();
  }
}

void Scene3DViewportWidget::draw_frustum(const QColor & color, bool translucent)
{
  const double h = qDegreesToRadians(camera_overlay.horizontal_fov_deg * 0.5);
  const double v = qDegreesToRadians(camera_overlay.vertical_fov_deg * 0.5);
  const double n = qMax(0.05, camera_overlay.range_min_m);
  const double f = qMax(n + 0.05, camera_overlay.range_max_m);
  auto ray = [&](double r, double ah, double av) { return QVector3D(camera_overlay.x + r, camera_overlay.y + qTan(av) * r, camera_overlay.z + qTan(ah) * r); };
  QVector3D ffl = ray(f, -h, -v), ffr = ray(f, h, -v), fbl = ray(f, -h, v), fbr = ray(f, h, v);
  glColor4f(color.redF(), color.greenF(), color.blueF(), translucent ? 0.12f : 0.7f);
  glBegin(GL_TRIANGLE_FAN); glVertex3f(camera_overlay.x, camera_overlay.y, camera_overlay.z); glVertex3f(ffl.x(), ffl.y(), ffl.z()); glVertex3f(ffr.x(), ffr.y(), ffr.z()); glVertex3f(fbr.x(), fbr.y(), fbr.z()); glVertex3f(fbl.x(), fbl.y(), fbl.z()); glVertex3f(ffl.x(), ffl.y(), ffl.z()); glEnd();
}
QPointF Scene3DViewportWidget::project_to_screen(double x, double y, double z) const { return QPointF(width() * 0.5 + x * 50.0, height() * 0.6 - y * 50.0 + z * 5.0); }
bool Scene3DViewportWidget::drag_position_to_world_xy(const QPoint & pos, double z_m, double & out_x, double & out_y) const
{
  if (width() <= 0 || height() <= 0 || !std::isfinite(z_m)) return false;
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  const QMatrix4x4 inv = (proj * view).inverted();
  const float ndc_x = (2.0f * static_cast<float>(pos.x()) / static_cast<float>(width())) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(pos.y()) / static_cast<float>(height()));
  const QVector4D near_clip(ndc_x, ndc_y, -1.0f, 1.0f);
  const QVector4D far_clip(ndc_x, ndc_y, 1.0f, 1.0f);
  QVector4D near_world = inv * near_clip;
  QVector4D far_world = inv * far_clip;
  if (qFuzzyIsNull(near_world.w()) || qFuzzyIsNull(far_world.w())) return false;
  near_world /= near_world.w();
  far_world /= far_world.w();
  const QVector3D origin = near_world.toVector3D();
  const QVector3D dir = (far_world.toVector3D() - origin).normalized();
  if (qAbs(dir.z()) < 1e-6f) return false;
  const float t = (static_cast<float>(z_m) - origin.z()) / dir.z();
  if (!std::isfinite(t) || t < 0.0f) return false;
  const QVector3D hit = origin + dir * t;
  out_x = snap_translation_value(hit.x(), snap_mode);
  out_y = snap_translation_value(hit.y(), snap_mode);
  return std::isfinite(out_x) && std::isfinite(out_y);
}
void Scene3DViewportWidget::camera_matrices(QMatrix4x4 & out_proj, QMatrix4x4 & out_view) const
{
  const float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
  const float clamped_distance = static_cast<float>(qBound(min_distance_, distance_, max_distance_));
  const float cp = static_cast<float>(qCos(pitch_));
  const QVector3D forward(
    cp * static_cast<float>(qSin(yaw_)),
    static_cast<float>(qSin(pitch_)),
    cp * static_cast<float>(qCos(yaw_)));
  const QVector3D eye = orbit_offset_ - (forward * clamped_distance);
  const float radius = static_cast<float>(qMax(0.2, scene_radius_));
  const float near_plane = qMax(0.01f, qMin(0.2f, radius * 0.05f));
  const float far_plane = qMax(clamped_distance + (radius * 8.0f), radius * 20.0f);
  out_proj.setToIdentity();
  out_proj.perspective(50.0f, aspect, near_plane, far_plane);
  out_view.setToIdentity();
  out_view.lookAt(eye, orbit_offset_, QVector3D(0.0f, 1.0f, 0.0f));
}
bool Scene3DViewportWidget::ray_intersects_aabb(const QVector3D & ray_origin, const QVector3D & ray_dir,
                                                const ScenePreviewWidget::PreviewItem & item, float & out_t) const
{
  const ItemBounds bounds = item_bounds_for_role(item);
  const QVector3D bmin(bounds.x, bounds.y, bounds.z);
  const QVector3D bmax(bounds.x + bounds.sx, bounds.y + bounds.sy, bounds.z + bounds.sz);
  float tmin = 0.0f;
  float tmax = 1e9f;
  for (int axis = 0; axis < 3; ++axis) {
    const float origin = axis == 0 ? ray_origin.x() : (axis == 1 ? ray_origin.y() : ray_origin.z());
    const float dir = axis == 0 ? ray_dir.x() : (axis == 1 ? ray_dir.y() : ray_dir.z());
    const float minv = axis == 0 ? bmin.x() : (axis == 1 ? bmin.y() : bmin.z());
    const float maxv = axis == 0 ? bmax.x() : (axis == 1 ? bmax.y() : bmax.z());
    if (qAbs(dir) < 1e-6f) {
      if (origin < minv || origin > maxv) return false;
      continue;
    }
    const float invd = 1.0f / dir;
    float t0 = (minv - origin) * invd;
    float t1 = (maxv - origin) * invd;
    if (t0 > t1) std::swap(t0, t1);
    tmin = qMax(tmin, t0);
    tmax = qMin(tmax, t1);
    if (tmax < tmin) return false;
  }
  out_t = tmin;
  return true;
}
Scene3DViewportWidget::ItemBounds Scene3DViewportWidget::item_bounds_for_role(const ScenePreviewWidget::PreviewItem & item) const
{
  ItemBounds mesh_bounds{};
  if (mesh_world_bounds_for_item(item, mesh_bounds)) return mesh_bounds;
  PrimitiveWorldBounds primitive_bounds{};
  if (primitive_world_bounds_for_item(item, primitive_bounds)) {
    return { primitive_bounds.x, primitive_bounds.y, primitive_bounds.z,
             primitive_bounds.sx, primitive_bounds.sy, primitive_bounds.sz };
  }

  const NormalizedRole role = classify_item_role(item);
  if (role == NormalizedRole::Object || role == NormalizedRole::WarningAnchor) {
    const double cube = (role == NormalizedRole::Object)
      ? qMax(0.05, qMin(item.sx, qMin(item.sy, item.sz)))
      : qMax(0.04, qMin(item.sx, qMin(item.sy, item.sz)));
    return { item.x, item.y, item.z, cube, cube, cube };
  }
  return { item.x, item.y, item.z, item.sx, item.sy, item.sz };
}

bool Scene3DViewportWidget::mesh_world_bounds_for_item(const ScenePreviewWidget::PreviewItem & item, ItemBounds & out_bounds) const
{
  const QString mesh_source = !item.mesh_path.trimmed().isEmpty() ? item.mesh_path : item.source_path;
  if (mesh_source.trimmed().isEmpty()) return false;
  QString canonical_mesh_source;
  if (!try_resolve_canonical_mesh_path(mesh_source, canonical_mesh_source, &item)) {
    canonical_mesh_source = QFileInfo(mesh_source).absoluteFilePath();
  }
  const auto cache_it = mesh_cache_.constFind(canonical_mesh_source);
  if (cache_it == mesh_cache_.constEnd()) return false;
  const MeshCacheEntry & cache = cache_it.value();
  if (!cache.loaded || !cache.valid || !cache.has_bounds) return false;

  QMatrix4x4 transform = final_mesh_transform_matrix(item);

  const QVector3D lmin = cache.local_min;
  const QVector3D lmax = cache.local_max;
  QVector3D wmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  QVector3D wmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int xi = 0; xi < 2; ++xi) {
    for (int yi = 0; yi < 2; ++yi) {
      for (int zi = 0; zi < 2; ++zi) {
        const QVector3D local_corner(
          xi == 0 ? lmin.x() : lmax.x(),
          yi == 0 ? lmin.y() : lmax.y(),
          zi == 0 ? lmin.z() : lmax.z());
        const QVector3D world_corner = transform * local_corner;
        wmin.setX(qMin(wmin.x(), world_corner.x()));
        wmin.setY(qMin(wmin.y(), world_corner.y()));
        wmin.setZ(qMin(wmin.z(), world_corner.z()));
        wmax.setX(qMax(wmax.x(), world_corner.x()));
        wmax.setY(qMax(wmax.y(), world_corner.y()));
        wmax.setZ(qMax(wmax.z(), world_corner.z()));
      }
    }
  }
  out_bounds = { wmin.x(), wmin.y(), wmin.z(),
                 qMax(0.0f, wmax.x() - wmin.x()),
                 qMax(0.0f, wmax.y() - wmin.y()),
                 qMax(0.0f, wmax.z() - wmin.z()) };
  return true;
}
bool Scene3DViewportWidget::pick_item_at_screen(
  const QPoint & pos, QString & out_id, QString & out_role, QString * out_tooltip) const
{
  QMatrix4x4 proj, view;
  camera_matrices(proj, view);
  const QMatrix4x4 inv = (proj * view).inverted();
  const float ndc_x = (2.0f * static_cast<float>(pos.x()) / qMax(1, width())) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(pos.y()) / qMax(1, height()));
  QVector4D near_h = inv * QVector4D(ndc_x, ndc_y, -1.0f, 1.0f);
  QVector4D far_h = inv * QVector4D(ndc_x, ndc_y, 1.0f, 1.0f);
  if (qFuzzyIsNull(near_h.w()) || qFuzzyIsNull(far_h.w())) return false;
  const QVector3D p0 = near_h.toVector3DAffine();
  const QVector3D p1 = far_h.toVector3DAffine();
  const QVector3D dir = (p1 - p0).normalized();

  float best_t = 1e9f;
  const ScenePreviewWidget::PreviewItem * best_item = nullptr;
  for (const auto & item : items) {
    if (!item.selectable) continue;
    float t = 0.0f;
    if (ray_intersects_aabb(p0, dir, item, t) && t < best_t) {
      best_t = t;
      best_item = &item;
    }
  }
  if (best_item == nullptr) return false;
  out_id = best_item->id;
  out_role = best_item->role.trimmed().isEmpty() ? QStringLiteral("unknown") : best_item->role.trimmed();
  if (out_tooltip != nullptr) {
    const QString role_normalized = normalized_token(out_role);
    const QString metadata_tags = best_item->metadata_tags.trimmed();
    *out_tooltip = QStringLiteral("Item: %1\nRole: %2\nLayer: %3\nWarnings: %4")
      .arg(out_id, role_normalized, item_visual_ownership_label(*best_item), warning_debug_text(best_item->warnings));
    if (!metadata_tags.isEmpty()) *out_tooltip += QStringLiteral("\nTags: ") + metadata_tags;
    if (!best_item->warnings.isEmpty()) *out_tooltip += QStringLiteral("\n\nDetails:\n- ") + best_item->warnings.join("\n- ");
  }
  return true;
}

bool Scene3DViewportWidget::pick_gizmo_axis_at_screen(const QPoint & pos, QString & out_axis, double * out_score) const
{
  out_axis.clear();
  if (selected_id.isEmpty()) return false;
  const ScenePreviewWidget::PreviewItem * selected = nullptr;
  for (const auto & it : items) if (it.id == selected_id) { selected = &it; break; }
  if (selected == nullptr) return false;

  const QVector3D origin(selected->x + (selected->sx * 0.5), selected->y + (selected->sy * 0.5), selected->z + (selected->sz * 0.5));
  const double item_extent = std::max({selected->sx, selected->sy, selected->sz});
  const double axis_len = qMax(0.25, 0.75 * item_extent);
  const QPointF screen_p = QPointF(pos);
  const QPointF origin_s = project_to_screen(origin.x(), origin.y(), origin.z());
  if (!qIsFinite(origin_s.x()) || !qIsFinite(origin_s.y())) return false;

  const struct AxisCandidate { const char * axis; QVector3D dir; } candidates[] = {
    {"x", QVector3D(1.0f, 0.0f, 0.0f)},
    {"y", QVector3D(0.0f, 1.0f, 0.0f)},
    {"z", QVector3D(0.0f, 0.0f, 1.0f)}
  };
  constexpr double kAxisThresholdPx = 12.0;
  double best = std::numeric_limits<double>::max();
  QString best_axis;
  for (const auto & candidate : candidates) {
    const QVector3D endpoint = origin + (candidate.dir * axis_len);
    const QPointF endpoint_s = project_to_screen(endpoint.x(), endpoint.y(), endpoint.z());
    if (!qIsFinite(endpoint_s.x()) || !qIsFinite(endpoint_s.y())) continue;
    const double score = distance_to_segment_2d(screen_p, origin_s, endpoint_s);
    if (score < best) {
      best = score;
      best_axis = QString::fromLatin1(candidate.axis);
    }
  }
  if (out_score != nullptr) *out_score = best;
  if (best <= kAxisThresholdPx && !best_axis.isEmpty()) { out_axis = best_axis; return true; }
  // Deterministic fallback when no segment is precisely hit: choose the axis with the lowest score.
  out_axis = best_axis;
  return !out_axis.isEmpty();
}

bool Scene3DViewportWidget::pick_gizmo_rotation_ring_at_screen(const QPoint & pos, QString & out_axis, double * out_score) const
{
  out_axis.clear();
  if (selected_id.isEmpty()) return false;
  const ScenePreviewWidget::PreviewItem * selected = nullptr;
  for (const auto & it : items) if (it.id == selected_id) { selected = &it; break; }
  if (selected == nullptr) return false;

  const QVector3D origin(selected->x + (selected->sx * 0.5), selected->y + (selected->sy * 0.5), selected->z + (selected->sz * 0.5));
  const double item_extent = std::max({selected->sx, selected->sy, selected->sz});
  const double radius = qMax(0.2, 0.65 * item_extent);
  const QPointF screen_p = QPointF(pos);
  constexpr int kRingSamples = 48;
  constexpr double kRingThresholdPx = 11.0;
  double best = std::numeric_limits<double>::max();
  QString best_axis;

  const struct RingCandidate { const char * axis; QVector3D u; QVector3D v; } rings[] = {
    {"x", QVector3D(0.0f, 1.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)},
    {"y", QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)},
    {"z", QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f)}
  };
  for (const auto & ring : rings) {
    QVector<QPointF> polyline;
    polyline.reserve(kRingSamples + 1);
    for (int i = 0; i <= kRingSamples; ++i) {
      const double t = (2.0 * M_PI * i) / static_cast<double>(kRingSamples);
      const QVector3D sample = origin + (ring.u * static_cast<float>(radius * qCos(t))) + (ring.v * static_cast<float>(radius * qSin(t)));
      const QPointF sample_s = project_to_screen(sample.x(), sample.y(), sample.z());
      if (!qIsFinite(sample_s.x()) || !qIsFinite(sample_s.y())) continue;
      polyline.push_back(sample_s);
    }
    const double score = distance_to_polyline_2d(screen_p, polyline);
    if (score < best) {
      best = score;
      best_axis = QString::fromLatin1(ring.axis);
    }
  }
  if (out_score != nullptr) *out_score = best;
  if (best <= kRingThresholdPx && !best_axis.isEmpty()) { out_axis = best_axis; return true; }
  // Deterministic fallback when no ring polyline is within threshold: pick the minimum-score ring.
  out_axis = best_axis;
  return !out_axis.isEmpty();
}

void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {
  last_ = e->pos();
  if (e->button() != Qt::LeftButton) return;

  drag_start_screen_ = e->pos();
  dragging_gizmo_ = false;
  drag_active_handle_ = GizmoHandle::None;
  drag_in_progress_ = false;
  drag_cancelled_ = false;
  active_axis_.clear();

  if ((gizmo_mode == GizmoMode::Move || gizmo_mode == GizmoMode::Rotate) && !selected_id.isEmpty()) {
    const ScenePreviewWidget::PreviewItem * selected_item = nullptr;
    for (const auto & it : items) {
      if (it.id == selected_id) {
        selected_item = &it;
        break;
      }
    }

    if (selected_item != nullptr) {
      if (!item_is_editable_for_gizmo(*selected_item)) {
        if (status_message_cb) status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(*selected_item)));
      } else {
        double score = std::numeric_limits<double>::max();
        QString axis;
        bool picked = false;
        if (gizmo_mode == GizmoMode::Move) {
          picked = pick_gizmo_axis_at_screen(e->pos(), axis, &score);
          if (picked) drag_active_handle_ = (axis == "x") ? GizmoHandle::MoveX : (axis == "y" ? GizmoHandle::MoveY : GizmoHandle::MoveZ);
        } else if (gizmo_mode == GizmoMode::Rotate) {
          picked = pick_gizmo_rotation_ring_at_screen(e->pos(), axis, &score);
          if (picked) drag_active_handle_ = (axis == "x") ? GizmoHandle::Roll : (axis == "y" ? GizmoHandle::Pitch : GizmoHandle::Yaw);
        }

        if (picked && !axis.isEmpty()) {
          active_axis_ = axis;
          dragging_gizmo_ = true;
          drag_in_progress_ = true;
          drag_start_pose_.item_id = selected_item->id;
          drag_start_pose_.x = selected_item->x;
          drag_start_pose_.y = selected_item->y;
          drag_start_pose_.z = selected_item->z;
          drag_start_pose_.roll = selected_item->roll;
          drag_start_pose_.pitch = selected_item->pitch;
          drag_start_pose_.yaw = selected_item->yaw;
          return;
        }
      }
    }
  }

  QString best_id, best_role;
  if (pick_item_at_screen(e->pos(), best_id, best_role) && !best_id.isEmpty() && select_cb) select_cb(best_id, best_role);
}
void Scene3DViewportWidget::mouseMoveEvent(QMouseEvent * e)
{
  if (dragging_gizmo_ && (e->buttons() & Qt::LeftButton) && !selected_id.isEmpty()) {
    for (auto & it : items) {
      if (it.id != selected_id) continue;
      if (!item_is_editable_for_gizmo(it)) {
        dragging_gizmo_ = false;
        if (status_message_cb) status_message_cb(QStringLiteral("Locked: %1").arg(item_locked_reason(it)));
        return;
      }
      const QPoint delta = e->pos() - drag_start_screen_;
      if (gizmo_mode == GizmoMode::Move) {
        const double raw = (active_axis_ == "x" ? delta.x() : -delta.y()) * 0.002;
        const double snapped = snap_translation_value(raw, snap_mode);
        if (active_axis_ == "x") it.x = drag_start_pose_.x + snapped; else if (active_axis_ == "y") it.y = drag_start_pose_.y + snapped; else it.z = drag_start_pose_.z + snapped;
      } else if (gizmo_mode == GizmoMode::Rotate) {
        const double raw = (delta.x() - delta.y()) * 0.01;
        const double snapped = snap_rotation_value(raw, snap_mode);
        if (active_axis_ == "x") it.roll = drag_start_pose_.roll + snapped; else if (active_axis_ == "y") it.pitch = drag_start_pose_.pitch + snapped; else it.yaw = drag_start_pose_.yaw + snapped;
      }
      update();
      return;
    }
  }
  auto d = e->pos() - last_;
  last_ = e->pos();
  const bool pan_mode = (e->buttons() & Qt::MiddleButton) || ((e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ShiftModifier));
  if (pan_mode) {
    const float pan_scale = static_cast<float>(distance_ * 0.0018);
    const QVector3D forward(
      static_cast<float>(qCos(pitch_) * qSin(yaw_)),
      static_cast<float>(qSin(pitch_)),
      static_cast<float>(qCos(pitch_) * qCos(yaw_)));
    const QVector3D world_up(0.0f, 1.0f, 0.0f);
    QVector3D right = QVector3D::crossProduct(forward, world_up).normalized();
    if (right.lengthSquared() < 1e-6f) right = QVector3D(1.0f, 0.0f, 0.0f);
    const QVector3D up = QVector3D::crossProduct(right, forward).normalized();
    orbit_offset_ += (-right * (d.x() * pan_scale)) + (up * (d.y() * pan_scale));
    update();
    return;
  }
  if (e->buttons() & Qt::LeftButton) {
    yaw_ += d.x() * 0.01;
    pitch_ = qBound(-1.4, pitch_ + d.y() * 0.01, 1.4);
    update();
    return;
  }
  QString hovered, hovered_role, hover_tooltip;
  hovered_gizmo_handle_ = GizmoHandle::None;
  if (!selected_id.isEmpty() && (gizmo_mode == GizmoMode::Move || gizmo_mode == GizmoMode::Rotate)) {
    QString axis;
    bool picked = (gizmo_mode == GizmoMode::Move) ? pick_gizmo_axis_at_screen(e->pos(), axis)
                                                  : pick_gizmo_rotation_ring_at_screen(e->pos(), axis);
    if (picked) {
      hovered_gizmo_handle_ = (gizmo_mode == GizmoMode::Move)
        ? ((axis == "x") ? GizmoHandle::MoveX : (axis == "y" ? GizmoHandle::MoveY : GizmoHandle::MoveZ))
        : ((axis == "x") ? GizmoHandle::Roll : (axis == "y" ? GizmoHandle::Pitch : GizmoHandle::Yaw));
      hovered = selected_id;
      hovered_role = QStringLiteral("gizmo_%1").arg(axis);
      hover_tooltip = (gizmo_mode == GizmoMode::Move)
        ? QStringLiteral("Move %1 axis").arg(axis.toUpper())
        : QStringLiteral("Rotate %1 axis").arg(axis.toUpper());
    }
  }
  if (hovered_gizmo_handle_ == GizmoHandle::None) {
    if (pick_item_at_screen(e->pos(), hovered, hovered_role, &hover_tooltip) && !hovered.isEmpty()) {
      QToolTip::showText(e->globalPos(), hover_tooltip, this);
    } else {
      QToolTip::hideText();
    }
  } else {
    QToolTip::showText(e->globalPos(), hover_tooltip, this);
  }
  hovered_id_ = hovered;
}
void Scene3DViewportWidget::mouseReleaseEvent(QMouseEvent * e)
{
  if (e->button() == Qt::LeftButton) {
    if (drag_in_progress_ && !drag_cancelled_ && transform_changed_cb) {
      bool committed = false;
      for (auto & it : items) {
        if (it.id != drag_start_pose_.item_id) continue;
        const bool id_valid = !it.id.trimmed().isEmpty() && !drag_start_pose_.item_id.trimmed().isEmpty() && it.id == selected_id;
        const bool finite_xyz = std::isfinite(it.x) && std::isfinite(it.y) && std::isfinite(it.z);
        const bool bounded_xyz = qAbs(it.x) <= kWorkspaceLimitMeters &&
                                 qAbs(it.y) <= kWorkspaceLimitMeters &&
                                 qAbs(it.z) <= kWorkspaceLimitMeters;
        if (!id_valid || !finite_xyz || !bounded_xyz) {
          it.x = drag_start_pose_.x;
          it.y = drag_start_pose_.y;
          it.z = drag_start_pose_.z;
          it.roll = drag_start_pose_.roll;
          it.pitch = drag_start_pose_.pitch;
          it.yaw = drag_start_pose_.yaw;
          const QString message = QStringLiteral(
            "Rejected gizmo drag commit for '%1': invalid final XYZ or mismatched drag source; pose reverted.")
                                    .arg(drag_start_pose_.item_id);
          qWarning().noquote() << message;
          if (status_message_cb) status_message_cb(message);
          update();
          break;
        }
        transform_changed_cb(it.id, it.x, it.y, it.z, it.roll, it.pitch, it.yaw);
        committed = true;
        break;
      }
      if (!committed && status_message_cb) {
        status_message_cb(QStringLiteral("No valid drag source found for commit; change discarded."));
      }
    }
    dragging_gizmo_ = false;
    drag_in_progress_ = false;
    drag_cancelled_ = false;
    drag_active_handle_ = GizmoHandle::None;
    active_axis_.clear();
  }
  QOpenGLWidget::mouseReleaseEvent(e);
}

void Scene3DViewportWidget::wheelEvent(QWheelEvent * e)
{
  const double delta_steps = static_cast<double>(e->angleDelta().y()) / 120.0;
  const double zoom_factor = std::pow(0.9, delta_steps);
  distance_ = qBound(min_distance_, distance_ * zoom_factor, max_distance_);
  update();
}

void Scene3DViewportWidget::keyPressEvent(QKeyEvent * e)
{
  if (e->key() == Qt::Key_Escape) {
    if (drag_asset_preview_visible_) {
      drag_asset_preview_visible_ = false;
      drag_asset_drop_status_ = QStringLiteral("Drag/drop cancelled.");
      QToolTip::showText(QCursor::pos(), drag_asset_drop_status_, this);
      update();
      e->accept();
      return;
    }
    if (!drag_in_progress_) {
      e->ignore();
      return;
    }
    for (auto & it : items) {
      if (it.id != drag_start_pose_.item_id) continue;
      it.x = drag_start_pose_.x;
      it.y = drag_start_pose_.y;
      it.z = drag_start_pose_.z;
      it.roll = drag_start_pose_.roll;
      it.pitch = drag_start_pose_.pitch;
      it.yaw = drag_start_pose_.yaw;
      // Cancel restores the in-memory preview pose only; commit/save callback is release-only.
      break;
    }
    dragging_gizmo_ = false;
    drag_in_progress_ = false;
    drag_cancelled_ = true;
    drag_active_handle_ = GizmoHandle::None;
    active_axis_.clear();
    QToolTip::showText(QCursor::pos(), QStringLiteral("Gizmo drag cancelled."), this);
    update();
    e->accept();
    return;
  }
  QOpenGLWidget::keyPressEvent(e);
}

void Scene3DViewportWidget::dragEnterEvent(QDragEnterEvent * event)
{
  if (!event->mimeData() || !event->mimeData()->hasFormat(kWorkcellStudioAssetMime)) return;
  const QJsonObject payload = QJsonDocument::fromJson(event->mimeData()->data(kWorkcellStudioAssetMime)).object();
  if (payload.value("asset_id").toString().trimmed().isEmpty()) return;
  event->acceptProposedAction();
}

void Scene3DViewportWidget::dragMoveEvent(QDragMoveEvent * event)
{
  if (!event->mimeData() || !event->mimeData()->hasFormat(kWorkcellStudioAssetMime)) return;
  const QByteArray payload = event->mimeData()->data(kWorkcellStudioAssetMime);
  drag_asset_payload_ = QJsonDocument::fromJson(payload).object();
  const QString asset_id = drag_asset_payload_.value("asset_id").toString().trimmed();
  if (asset_id.isEmpty()) return;
  double x = 0.0, y = 0.0;
  if (!drag_position_to_world_xy(event->pos(), 0.0, x, y)) {
    drag_asset_preview_visible_ = false;
    drag_asset_drop_status_ = QStringLiteral("No valid workcell surface under the pointer");
    update();
    return;
  }
  drag_asset_label_ = asset_id;
  drag_asset_screen_pos_ = event->pos();
  drag_asset_world_pos_ = QVector3D(x, y, 0.0f);
  drag_asset_preview_visible_ = true;
  drag_asset_drop_status_ = QStringLiteral("Drop to place %1").arg(drag_asset_label_);
  event->acceptProposedAction();
  update();
}

void Scene3DViewportWidget::dragLeaveEvent(QDragLeaveEvent *)
{
  drag_asset_preview_visible_ = false;
  update();
}

void Scene3DViewportWidget::dropEvent(QDropEvent * event)
{
  if (!event->mimeData() || !event->mimeData()->hasFormat(kWorkcellStudioAssetMime)) return;
  const QJsonObject payload = QJsonDocument::fromJson(event->mimeData()->data(kWorkcellStudioAssetMime)).object();
  double x = 0.0, y = 0.0;
  if (payload.value("asset_id").toString().trimmed().isEmpty() || !drag_position_to_world_xy(event->pos(), 0.0, x, y)) {
    drag_asset_preview_visible_ = false;
    if (status_message_cb) status_message_cb(QStringLiteral("No valid workcell surface under the pointer"));
    update();
    return;
  }
  const double z = 0.0;
  const bool shift_drop = (event->keyboardModifiers() & Qt::ShiftModifier);
  if (asset_drop_cb) asset_drop_cb(payload, x, y, z, shift_drop);
  drag_asset_preview_visible_ = false;
  event->acceptProposedAction();
  update();
}
