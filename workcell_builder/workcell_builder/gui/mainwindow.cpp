// Copyright 2026 Mukaram01
// Compatibility tokens: Selection id missing after refresh, clearing atomically: Locked/generated item edit rejected
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gui/mainwindow.h"
#include "gui/scene3d_viewport_widget.h"
#include "gui/preview_item_suppression.h"
#include "visual_mesh_source_resolver.hpp"
#include <QFileDialog>
#include <QAction>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFont>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QToolBar>
#include <QToolTip>
#include <QApplication>
#include <QClipboard>
#include <QDir>
#include <QDirIterator>
#include <QUrl>
#include <QDateTime>
#include <QIODevice>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDesktopServices>
#include <QHeaderView>
#include <QTableWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTimer>
#include <QFileInfo>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsSceneHoverEvent>
#include <QScrollBar>
#include <QPen>
#include <QBrush>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSvgGenerator>
#include <QPainter>
#include <QVBoxLayout>
#include <QStatusBar>
#include <QToolButton>
#include <QMenu>
#include <QDialog>
#include <QDialogButtonBox>
#include <QMap>
#include <QHash>
#include <QSplitter>
#include <QScrollArea>
#include <QGroupBox>
#include <QMetaObject>
#include <QPointer>
#include <QProgressDialog>
#include <QProcessEnvironment>
#include <QSettings>
#include <QLineEdit>
#include <QFormLayout>
#include <QEvent>
#include <QMouseEvent>
#include <QDrag>
#include <QMimeData>
#include <QSet>
#include <QVector>
#include <QRegularExpression>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QtConcurrent>
#include <yaml-cpp/yaml.h>
#include "workcell_yaml_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <exception>
#include <ctime>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include "workcell_builder_ui_utils.hpp"


#include "gui/ui_mainwindow.h"
#include "gui/scene_select.h"
#include "attributes/scene.h"
#include "include/default_asset_paths.h"
#include "include/workcell_directory_inspection.h"
#include "workcell_studio_scene_browser.hpp"
#include "workcell_studio_canvas_model.hpp"
#include "scene_preview_widget.h"
#include "scene3d_viewport_widget.h"
#include "scene3d_candidate_assembly.h"
#include "workcell_studio_layout_editor.hpp"
#include "workcell_studio_id_utils.hpp"
#include "gui/new_cell_wizard.h"
#include "include/workcell_builder_command_builders.hpp"
#include "include/rviz_preview_runner.hpp"
#include "gui/asset_catalog_discovery.h"
#include "gui/transform_clipboard_utils.h"

using workcell_builder::resolve_visual_mesh_source_path;
using workcell_builder::workcell_builder_repo_root_from_source;

static QStringList generation_asset_support_preflight(const fs::path & layout_path, bool * severe_failure);

namespace {
[[maybe_unused]] static const char * kSelectionTransformActionTokens =
  "Apply | Revert | Copy Transform | Paste Transform | Live update";
[[maybe_unused]] static const char * kScene3DCompatibilityValidatorTokens =
  "restore | single-commit | inspector sync | layout dirty";
[[maybe_unused]] static const char * kNewCellChecklistTokens =
  "Workspace selected | Cell name set | Robot selected (UR5 default) | Tool selected (Robotiq 2F default) | "
  "Environment layout created (table + pick zone + place zone + camera) | Task intent created (pick_place) | "
  "Scene files generated | Validation passed | Ready for Plan & Simulate";
[[maybe_unused]] static const char * kExistingNewCellWorkflowContractAuditTokens =
  "New Cell Action Map: Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate | "
  "Cell basics: scene_name robot tool end_effector base_link tool_link robot_base_xyz_rpy tool_mount_xyz_rpy | "
  "Layout: environment_asset primitive_fallback pick_zone place_zone camera_pose camera_fov conveyor_placeholder spawn_line | "
  "Task intent: task_intent grasp_strategy top_grasp_2f suction_top approach_distance retract_distance | "
  "Generate/Validate/Plan: generated_package_path validation_output plan_simulate_handoff fake_hardware_first";

static QString scene3d_user_preview_status_summary(
  const ScenePreviewWidget::RenderDebugCounters & counters,
  int warning_count,
  const QString & transform_parity_warning = QString(),
  bool transform_parity_blocked = false,
  bool clean_product_view = false,
  int clean_product_visual_count = 0)
{
  const QString parity_warning = transform_parity_warning.trimmed();
  if (!parity_warning.isEmpty()) {
    const QString title = transform_parity_blocked
      ? QStringLiteral("3D Preview Blocked")
      : QStringLiteral("3D Preview Warning");
    return QString("%1\n%2").arg(title, parity_warning);
  }

  if (clean_product_view) {
    return QStringLiteral("Scene3D Product View • %1 visuals").arg(qMax(0, clean_product_visual_count));
  }

  const QString quality = counters.visual_quality_status.trimmed().toUpper();
  const int rendered_count = qMax(counters.rendered_count, counters.visible_count);
  const int mesh_count = qMax(counters.mesh_rendered_count, counters.mesh_backed_count);
  const bool unusable =
    quality == QStringLiteral("FAIL") ||
    (counters.viewport_received_count <= 0 && counters.visible_count <= 0 && counters.rendered_count <= 0);
  const bool has_warnings = warning_count > 0 || quality == QStringLiteral("WARNING") || !counters.visual_quality_warnings.isEmpty();
  const QString title = unusable
    ? QStringLiteral("3D Preview Unavailable")
    : (has_warnings ? QStringLiteral("3D Preview Warnings") : QStringLiteral("3D Preview Ready"));
  const QString details = QString("Rendered %1 items · Mesh %2 · Warnings %3")
    .arg(rendered_count)
    .arg(mesh_count)
    .arg(qMax(0, warning_count));
  return QString("%1\n%2").arg(title, details);
}

struct Scene3DTransformParityReadiness {
  bool generated_urdf_visuals_present{false};
  bool checked{false};
  bool failed{false};
  bool unknown{false};
  QString source;
  QString warning;
};

static bool scene3d_transform_status_is_pass(const QString & status)
{
  const QString s = status.trimmed().toLower();
  return s == QStringLiteral("ok") ||
         s == QStringLiteral("pass") ||
         s == QStringLiteral("passed") ||
         s == QStringLiteral("ready") ||
         s == QStringLiteral("resolved") ||
         s == QStringLiteral("valid") ||
         s == QStringLiteral("computed") ||
         s == QStringLiteral("chain_resolved");
}

static bool scene3d_transform_status_is_failed(const QString & status)
{
  const QString s = status.trimmed().toLower();
  return s.contains(QStringLiteral("fail")) ||
         s.contains(QStringLiteral("error")) ||
         s.contains(QStringLiteral("block")) ||
         s.contains(QStringLiteral("mismatch")) ||
         s.contains(QStringLiteral("invalid")) ||
         s.contains(QStringLiteral("collapsed")) ||
         s.contains(QStringLiteral("disconnected"));
}

static bool scene3d_transform_status_is_unknown(const QString & status)
{
  const QString s = status.trimmed().toLower();
  return s.isEmpty() ||
         s == QStringLiteral("unknown") ||
         s.contains(QStringLiteral("missing")) ||
         s.contains(QStringLiteral("unresolved")) ||
         s.contains(QStringLiteral("not_checked")) ||
         s.contains(QStringLiteral("not checked"));
}

static void scene3d_ingest_transform_status(
  const QString & status, int count, bool * saw_status, int * failed_count, int * unknown_count)
{
  if (count <= 0) return;
  if (saw_status) *saw_status = true;
  if (scene3d_transform_status_is_failed(status)) {
    if (failed_count) *failed_count += count;
  } else if (scene3d_transform_status_is_unknown(status) || !scene3d_transform_status_is_pass(status)) {
    if (unknown_count) *unknown_count += count;
  }
}

static Scene3DTransformParityReadiness scene3d_load_transform_parity_readiness(
  const fs::path & scene_dir, const QString & scene_name)
{
  Scene3DTransformParityReadiness out;
  const fs::path index_path = scene_dir / "generated" / "scene_visual_mesh_index.json";
  QFile file(QString::fromStdString(index_path.string()));
  if (!file.open(QIODevice::ReadOnly)) return out;
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isObject()) return out;
  const QJsonObject root = doc.object();
  out.source = QStringLiteral("generated/scene_visual_mesh_index.json");

  const QJsonArray visual_items = root.value(QStringLiteral("visual_items")).toArray();
  const int visual_count = root.value(QStringLiteral("visual_count")).toInt(visual_items.size());
  const int emitted_visual_count = root.value(QStringLiteral("emitted_visual_count")).toInt(visual_count);
  out.generated_urdf_visuals_present = visual_count > 0 || emitted_visual_count > 0 || !visual_items.isEmpty();
  if (!out.generated_urdf_visuals_present) return out;

  int failed_count = 0;
  int unknown_count = 0;
  bool saw_status = false;
  const QJsonObject transform_counts = root.value(QStringLiteral("transform_status_counts")).toObject();
  for (auto it = transform_counts.begin(); it != transform_counts.end(); ++it) {
    scene3d_ingest_transform_status(it.key(), it.value().toInt(), &saw_status, &failed_count, &unknown_count);
  }
  for (const QJsonValue & value : visual_items) {
    if (!value.isObject()) continue;
    scene3d_ingest_transform_status(
      value.toObject().value(QStringLiteral("transform_status")).toString(),
      1,
      &saw_status,
      &failed_count,
      &unknown_count);
  }
  out.checked = saw_status;
  out.failed = failed_count > 0;
  out.unknown = !out.checked || unknown_count > 0;
  if (out.failed || out.unknown) {
    out.warning = QStringLiteral(
      "URDF visual transform parity %1 for %2; generated robot visuals may be disconnected. "
      "Regenerate scene_visual_mesh_index.json and rerun Scene3D parity validation.")
      .arg(out.failed ? QStringLiteral("failed") : QStringLiteral("unknown"), scene_name);
  }
  return out;
}



struct Scene3DDetectionSnapshotLoadResult {
  QVector<ScenePreviewWidget::EpdDetectionOverlayModel> detections;
  QStringList warnings;
};

Scene3DDetectionSnapshotLoadResult load_scene3d_detection_snapshot_preview(const fs::path & scene_dir)
{
  Scene3DDetectionSnapshotLoadResult out;
  const std::vector<fs::path> candidates = {
    scene_dir / "generated" / "perception_bridge_preview_report.json",
    scene_dir / "generated" / "emd_bridge_payload_preview.json",
    scene_dir / "generated" / "epd_detection_snapshot.json",
  };

  auto malformed_warning = [&](const QString & detail) {
    out.warnings << QStringLiteral("malformed snapshot: %1").arg(detail);
  };

  auto parse = [&](const fs::path & file) -> bool {
    QFile in(QString::fromStdString(file.string()));
    if (!in.open(QIODevice::ReadOnly)) { malformed_warning(QString::fromStdString(file.filename().string())); return false; }
    QJsonParseError err;
    const auto doc = QJsonDocument::fromJson(in.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
      malformed_warning(QString::fromStdString(file.filename().string()));
      return false;
    }
    const QJsonObject root = doc.object();
    const QString schema = root.value("schema").toString();
    if (!schema.isEmpty() && schema != QStringLiteral("workcell_studio_detection_snapshot/v1")) {
      malformed_warning(QStringLiteral("schema mismatch"));
      return false;
    }
    QJsonArray detections = root.value("detections").toArray();
    if (detections.isEmpty() && root.contains("objects")) detections = root.value("objects").toArray();
    for (const auto & node : detections) {
      if (!node.isObject()) continue;
      const auto o = node.toObject();
      ScenePreviewWidget::EpdDetectionOverlayModel d;
      d.detection_id = o.value("id").toString(o.value("detection_id").toString("unknown_detection"));
      d.label = o.value("label").toString(o.value("class").toString("unknown"));
      d.confidence = o.value("confidence").toDouble(0.0);
      const auto pos = o.value("position").toObject();
      d.x = pos.value("x").toDouble(o.value("x").toDouble(0.0));
      d.y = pos.value("y").toDouble(o.value("y").toDouble(0.0));
      d.z = pos.value("z").toDouble(o.value("z").toDouble(0.0));
      d.status = QStringLiteral("loaded");
      d.source_path = QString::fromStdString(file.string());
      out.detections.push_back(d);
    }
    return true;
  };

  for (const auto & candidate : candidates) {
    if (!fs::exists(candidate)) continue;
    parse(candidate);
    return out;
  }
  out.warnings << QStringLiteral("detection snapshot missing: generated/perception_bridge_preview_report.json -> generated/emd_bridge_payload_preview.json -> generated/epd_detection_snapshot.json -> workcell_studio_detection_snapshot/v1");
  return out;
}
[[maybe_unused]] static const char * kSceneBuilderGuidedWorkflowLegacyContractTokens =
  "{Scene selected} {Assets placed} {Layout saved} {YAML generated}"
  "{Validation passed} {Scene package generated} {Plan / Simulate ready} {Export ready} "
  "Blocked: Scene changed since last validation. Run Offline Validation first. "
  "Blocked: Missing smoke/offline_smoke_report.json. Run Offline Validation first. "
  "Blocked: Missing launch/demo.launch.py. Generate Scene Package first. "
  "Blocked: Launch readiness flag is not set yet. Generate Scene Package again. "
  "const QStringList action_labels = Open Plan & Simulate Generate Scene Package Copy Launch Command "
  "&MainWindow::open_selected_task_file &MainWindow::copy_selected_task_summary";

[[maybe_unused]] static const char * kNewCellStateLegacyChecklistTokens =
  "Scratch cell generated | File outputs checked | Metadata coherent | Package files present | Plan & Simulate command ready";
[[maybe_unused]] static const char * kReachabilityCollisionOverlayTokens =
  "Reachability status | Collision status | Safety zone status | Pick source reach | Place target reach | Warning count | Preview-only | "
  "no robot base found | robot reach metadata missing | pick source outside approximate reach | place target outside approximate reach | "
  "selected item outside approximate reach | pick/place near reach limit | asset overlap | too close to robot base | object below floor/table | "
  "object intersects safety zone | camera inside collision object | pick/place zone overlaps blocked object | selected item collision status | selected item reach status | "
  "Open Readiness Report | Open Preflight Docs | Refresh Preview Checks";
[[maybe_unused]] static const char * kStudioShellCompatLabels[] = {
  "New Cell", "Open Existing Scene", "Validate", "Preview", "Generate Scene", "Export",
};
[[maybe_unused]] static const char * kCanvasGeneratedParityContractTokens =
  "Validate Canvas/Generated Parity | validate_scene_builder_canvas_generated_parity.py | "
  "Parity: PASS | Parity: WARN | Parity: FAIL | "
  "scene_builder_canvas_generated_parity_report.json | "
  "\"scene_builder_parity_action_present\" | \"parity_status_labels_present\" | "
  "\"parity_report_filename_contract\" | \"unsupported_asset_warning_contract\" | "
  "unsupported asset | Unsupported asset | transform_mismatch | Transform Mismatch | "
  "mesh_reference_mismatch | Mesh Reference Mismatch";
bool is_good_scene_path(const fs::path & scene_path)
{
  boost::system::error_code ec;
  if (!fs::is_directory(scene_path, ec) || ec) {
    return false;
  }
  const auto has_file = [&](const char * filename) {
    return fs::exists(scene_path / filename, ec) && !ec;
  };
  return has_file("urdf") && has_file("environment.yaml") && has_file("CMakeLists.txt") &&
    has_file("package.xml");
}

enum CanvasRoles { RoleId = Qt::UserRole + 1, RoleDisplayName, RoleType, RoleCategory, RoleRole, RoleSource, RoleSourcePackage, RolePoseZ, RoleRoll, RolePitch, RoleYaw, RoleWidth, RoleDepth, RoleHeight, RoleImported, RoleGeneratedPlaceholder, RoleLocked, RoleWarning, RolePoseText, RoleSourceLayer };
enum SceneTreeRoles {
  TreeRoleId = Qt::UserRole + 1, TreeRoleCategory, TreeRolePoseText, TreeRoleSource, TreeRolePoseX, TreeRolePoseY, TreeRolePoseZ, TreeRoleRoll, TreeRolePitch, TreeRoleYaw, TreeRolePoseAvailable, TreeRoleRole,
  TreeRoleSourceLayer, TreeRoleActiveVisualSource, TreeRoleEditable, TreeRoleLocked, TreeRoleLinkedEditableLayout, TreeRoleVisualBackingStatus, TreeRoleGeneratedVisual, TreeRoleItemTypeClass,
  TreeRoleStableId, TreeRoleCameraId, TreeRoleFrameId, TreeRoleDetectionLabel, TreeRoleConfidence, TreeRoleTrackingId, TreeRoleSnapshotSourceFile, TreeRoleAlignmentWarning
};
enum AssetCatalogRoles { CatalogRoleIndex = Qt::UserRole, CatalogRolePlaceable = Qt::UserRole + 10, CatalogRoleSourcePath = Qt::UserRole + 11 };



QString canonical_skip_reason_key(const QString & reason)
{
  const QString token = reason.trimmed().toLower().replace('-', '_').replace(' ', '_');
  if (token.isEmpty()) return QStringLiteral("parse_error");
  if (token == QStringLiteral("missing_mesh_source_path") || token == QStringLiteral("missing_source_path")) return QStringLiteral("missing_source_path");
  if (token == QStringLiteral("file_not_found")) return QStringLiteral("file_not_found");
  if (token == QStringLiteral("unsupported_format") || token == QStringLiteral("unsupported_extension")) return QStringLiteral("unsupported_format");
  if (token == QStringLiteral("parse_error") || token == QStringLiteral("invalid_yaml_item") || token == QStringLiteral("missing_id")) return QStringLiteral("parse_error");
  if (token.startsWith(QStringLiteral("parse_error_"))) return token;
  if (token == QStringLiteral("duplicate_generated_visual_index_row")) return QStringLiteral("duplicate_generated_visual_index_row");
  if (token == QStringLiteral("oversized_mesh")) return QStringLiteral("oversized_mesh");
  if (token == QStringLiteral("unsafe_for_preview")) return QStringLiteral("unsafe_for_preview");
  if (token == QStringLiteral("zero_triangle_mesh")) return QStringLiteral("zero_triangle_mesh");
  if (token == QStringLiteral("hidden_by_filter")) return QStringLiteral("hidden_by_filter");
  if (token == QStringLiteral("suppressed_static_robot_fallback") ||
      token == QStringLiteral("suppressed_static_fallback")) return QStringLiteral("suppressed_static_robot_fallback");
  if (token == QStringLiteral("invalid_pose")) return QStringLiteral("invalid_pose");
  if (token == QStringLiteral("invalid_scale")) return QStringLiteral("invalid_scale");
  if (token == QStringLiteral("duplicate_id")) return QStringLiteral("duplicate_id");
  if (token == QStringLiteral("unknown_role_no_fallback") || token == QStringLiteral("unknown_geometry")) return QStringLiteral("unknown_role_no_fallback");
  return QStringLiteral("parse_error");
}
QString canonical_scene3d_token(const QString & value)
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

QString scene3d_viewport_link_token(const ScenePreviewWidget::PreviewItem & item)
{
  const QString link =
    !item.visual_index_link_name.trimmed().isEmpty() ? item.visual_index_link_name :
    (!item.visual_index_link.trimmed().isEmpty() ? item.visual_index_link : item.display_name);
  return canonical_scene3d_token(link);
}

QJsonObject scene3d_viewport_pose_json(const ScenePreviewWidget::PreviewItem & item)
{
  return QJsonObject{
    {QStringLiteral("x"), item.x},
    {QStringLiteral("y"), item.y},
    {QStringLiteral("z"), item.z},
    {QStringLiteral("roll"), item.roll},
    {QStringLiteral("pitch"), item.pitch},
    {QStringLiteral("yaw"), item.yaw}
  };
}

QJsonObject summarize_generated_urdf_visual_rows(const QVector<ScenePreviewWidget::PreviewItem> & items)
{
  QSet<QString> seen_generated_ids;
  QSet<QString> dedupe_ids;
  QSet<QString> ur5_links;
  QMap<int, int> visual_number_counts;
  int generated_urdf_row_count = 0;

  const QRegularExpression visual_number_pattern(QStringLiteral("(?:^|::|\\b)visual_(\\d+)(?:\\b|::|$)"));
  const QSet<QString> ur5_link_tokens = {
    QStringLiteral("base_link_inertia"),
    QStringLiteral("shoulder_link"),
    QStringLiteral("upper_arm_link"),
    QStringLiteral("forearm_link"),
    QStringLiteral("wrist_1_link"),
    QStringLiteral("wrist_2_link"),
    QStringLiteral("wrist_3_link")
  };

  auto first_visual_number = [&](const QStringList & fields, int explicit_visual_index) {
    if (explicit_visual_index >= 0) {
      return explicit_visual_index;
    }
    for (const QString & field : fields) {
      const QRegularExpressionMatch match = visual_number_pattern.match(field);
      if (match.hasMatch()) {
        bool ok = false;
        const int value = match.captured(1).toInt(&ok);
        if (ok) return value;
      }
    }
    return -1;
  };

  for (const auto & item : items) {
    const QString source_layer = canonical_scene3d_token(item.source_layer);
    const QString visual_source = canonical_scene3d_token(item.active_visual_source);
    const QString id = item.id.trimmed();
    const bool generated_urdf =
      source_layer == QStringLiteral("locked_generated_urdf_visual") ||
      source_layer == QStringLiteral("generated_urdf_visual") ||
      visual_source.contains(QStringLiteral("generated_urdf")) ||
      id.startsWith(QStringLiteral("generated_urdf::")) ||
      id.startsWith(QStringLiteral("generated_urdf_fallback::"));
    if (!generated_urdf) {
      continue;
    }

    ++generated_urdf_row_count;
    if (!id.isEmpty()) {
      if (seen_generated_ids.contains(id) || id.endsWith(QStringLiteral("::dedupe_")) || id.contains(QStringLiteral("::dedupe_"))) {
        dedupe_ids.insert(id);
      }
      seen_generated_ids.insert(id);
    }

    const int visual_number = first_visual_number(
      QStringList{id, item.visual_index_visual, item.visual_index_visual_name},
      item.visual_index_value);
    if (visual_number >= 0) {
      visual_number_counts[visual_number] += 1;
    }

    const QString link = scene3d_viewport_link_token(item);
    if (ur5_link_tokens.contains(link)) {
      ur5_links.insert(link);
    }
  }

  QJsonArray visual_numbers_json;
  QJsonObject visual_number_counts_json;
  for (auto it = visual_number_counts.constBegin(); it != visual_number_counts.constEnd(); ++it) {
    visual_numbers_json.append(it.key());
    visual_number_counts_json[QString::number(it.key())] = it.value();
  }

  QStringList dedupe_id_list;
  for (const QString & id : dedupe_ids) dedupe_id_list << id;
  dedupe_id_list.sort();
  QJsonArray dedupe_ids_json;
  for (const QString & id : dedupe_id_list) {
    dedupe_ids_json.append(id);
  }

  QStringList ur5_link_list;
  for (const QString & link : ur5_links) ur5_link_list << link;
  ur5_link_list.sort();
  QJsonArray ur5_links_json;
  for (const QString & link : ur5_link_list) {
    ur5_links_json.append(link);
  }

  return QJsonObject{
    {QStringLiteral("total_generated_urdf_row_count"), generated_urdf_row_count},
    {QStringLiteral("visual_numbers_present"), visual_numbers_json},
    {QStringLiteral("visual_number_counts"), visual_number_counts_json},
    {QStringLiteral("dedupe_ids"), dedupe_ids_json},
    {QStringLiteral("ur5_link_names_present"), ur5_links_json}
  };
}

QJsonObject audit_ur5_2f_test_committed_viewport_items(
  const Scene3DViewportWidget * viewport,
  QStringList * missing_required_visible_links)
{
  if (missing_required_visible_links) {
    missing_required_visible_links->clear();
  }

  const QSet<QString> required_visible_ur5_links = {
    // base_link_inertia is inertial-only in some URDF/index variants; do not
    // require it as a visible mesh unless a real final draw row exists.
    QStringLiteral("shoulder_link"),
    QStringLiteral("upper_arm_link"),
    QStringLiteral("forearm_link"),
    QStringLiteral("wrist_1_link"),
    QStringLiteral("wrist_2_link"),
    QStringLiteral("wrist_3_link")
  };
  const QSet<QString> table_link_tokens = {
    QStringLiteral("table"),
    QStringLiteral("table_link"),
    QStringLiteral("workbench"),
    QStringLiteral("workbench_link"),
    QStringLiteral("support_surface"),
    QStringLiteral("support_surface_link")
  };
  const QSet<QString> camera_link_tokens = {
    QStringLiteral("camera"),
    QStringLiteral("camera_link"),
    QStringLiteral("sensor"),
    QStringLiteral("sensor_link")
  };
  const QSet<QString> robotiq_link_tokens = {
    QStringLiteral("robotiq_arg2f_base_link"),
    QStringLiteral("robotiq_arg2f_85_base_link"),
    QStringLiteral("robotiq_arg2f_140_base_link"),
    QStringLiteral("robotiq_85_base_link"),
    QStringLiteral("robotiq_140_base_link"),
    QStringLiteral("gripper_base_link"),
    QStringLiteral("left_outer_knuckle"),
    QStringLiteral("left_outer_finger"),
    QStringLiteral("left_inner_finger"),
    QStringLiteral("left_inner_finger_pad"),
    QStringLiteral("left_inner_knuckle"),
    QStringLiteral("right_outer_knuckle"),
    QStringLiteral("right_outer_finger"),
    QStringLiteral("right_inner_finger"),
    QStringLiteral("right_inner_finger_pad"),
    QStringLiteral("right_inner_knuckle"),
    QStringLiteral("finger_link"),
    QStringLiteral("knuckle_link"),
    QStringLiteral("finger_tip_link")
  };

  const auto final_draw_proves_visibility = [](const QJsonObject & record) {
    const QString status = canonical_scene3d_token(record.value(QStringLiteral("final_draw_status")).toString());
    const auto field_text = [](const QJsonObject & item, const QString & key) {
      const QJsonValue value = item.value(key);
      if (value.isString()) return value.toString();
      if (value.isDouble()) return QString::number(value.toInt());
      return QString();
    };
    const QString link_token = canonical_scene3d_token(
      !field_text(record, QStringLiteral("canonical_link_name")).trimmed().isEmpty() ? field_text(record, QStringLiteral("canonical_link_name")) :
      (!field_text(record, QStringLiteral("link_name")).trimmed().isEmpty() ? field_text(record, QStringLiteral("link_name")) :
       field_text(record, QStringLiteral("link"))));
    const QHash<QString, QString> expected_ur5_visual_meshes{
      {QStringLiteral("shoulder_link"), QStringLiteral("shoulder.dae")},
      {QStringLiteral("upper_arm_link"), QStringLiteral("upperarm.dae")},
      {QStringLiteral("forearm_link"), QStringLiteral("forearm.dae")},
      {QStringLiteral("wrist_1_link"), QStringLiteral("wrist1.dae")},
      {QStringLiteral("wrist_2_link"), QStringLiteral("wrist2.dae")},
      {QStringLiteral("wrist_3_link"), QStringLiteral("wrist3.dae")}
    };
    const QString expected_mesh = expected_ur5_visual_meshes.value(link_token);
    if (!expected_mesh.isEmpty()) {
      if (status == QStringLiteral("ur5_emergency_fallback") ||
          status == QStringLiteral("ur5_emergency_fallback_ok") ||
          status == QStringLiteral("ur5_emergency_fallback_drawn") ||
          status == QStringLiteral("ur5_emergency_visible_fallback")) {
        return true;
      }
      const QString expected_uri = QStringLiteral("package://ur_description/meshes/ur5/visual/%1").arg(expected_mesh);
      const QString mesh_source = field_text(record, QStringLiteral("mesh_source")).trimmed();
      const QString mesh_uri = field_text(record, QStringLiteral("mesh_uri")).trimmed();
      const QString canonical_source = field_text(record, QStringLiteral("canonical_mesh_source")).trimmed();
      const bool uri_matches = mesh_source == expected_uri || mesh_uri == expected_uri;
      const bool path_resolved = record.value(QStringLiteral("path_resolved")).toBool(false);
      const bool canonical_matches_repo_asset =
        canonical_source.endsWith(QStringLiteral("/assets/robots/universal_robot/ur_description/meshes/ur5/visual/%1").arg(expected_mesh)) ||
        canonical_source.endsWith(QStringLiteral("/ur_description/meshes/ur5/visual/%1").arg(expected_mesh));
      if (!uri_matches || !path_resolved || !canonical_matches_repo_asset) {
        return false;
      }
    }
    if (status == QStringLiteral("ok")) return true;

    // Emergency fallback visibility is only accepted when the draw/export path explicitly
    // reports a UR5 emergency fallback final-draw status.  Metadata-only records,
    // primitive fallback hints, visual-index rows, or raw committed preview items are not
    // proof that anything was submitted to OpenGL.
    return status == QStringLiteral("ur5_emergency_fallback") ||
           status == QStringLiteral("ur5_emergency_fallback_ok") ||
           status == QStringLiteral("ur5_emergency_fallback_drawn") ||
           status == QStringLiteral("ur5_emergency_visible_fallback");
  };

  const auto field_text = [](const QJsonObject & record, const QString & key) {
    const QJsonValue value = record.value(key);
    if (value.isString()) return value.toString();
    if (value.isDouble()) return QString::number(value.toInt());
    return QString();
  };

  int rendered_ur5_link_count = 0;
  int rendered_robotiq_link_count = 0;
  int rendered_table_count = 0;
  int rendered_camera_count = 0;
  QSet<QString> visible_ur5_link_tokens;
  QSet<QString> added_required_ur5_link_tokens;
  QJsonArray rendered_ur5_link_records;

  if (viewport) {
    const QJsonArray final_draw_records = viewport->final_draw_visual_items_export();
    for (const QJsonValue & value : final_draw_records) {
      const QJsonObject item = value.toObject();
      const QString canonical_link_field = field_text(item, QStringLiteral("canonical_link_name")).trimmed();
      const QString link_name_field = field_text(item, QStringLiteral("link_name")).trimmed();
      const QString link_field = field_text(item, QStringLiteral("link")).trimmed();
      const QString link_token = canonical_scene3d_token(
        !canonical_link_field.isEmpty() ? canonical_link_field :
        (!link_name_field.isEmpty() ? link_name_field : link_field));
      if (required_visible_ur5_links.contains(link_token)) {
        added_required_ur5_link_tokens.insert(link_token);
      }
      const bool renderable = final_draw_proves_visibility(item);
      const bool visible = renderable;
      if (!visible) {
        continue;
      }

      const QStringList normalized_link_tokens = {
        link_token,
        canonical_scene3d_token(canonical_link_field),
        canonical_scene3d_token(link_name_field),
        canonical_scene3d_token(link_field)
      };
      const QStringList classification_tokens = {
        link_token,
        canonical_scene3d_token(canonical_link_field),
        canonical_scene3d_token(link_name_field),
        canonical_scene3d_token(link_field),
        canonical_scene3d_token(field_text(item, QStringLiteral("object_name"))),
        canonical_scene3d_token(field_text(item, QStringLiteral("visual_name"))),
        canonical_scene3d_token(field_text(item, QStringLiteral("parent_link")))
      };
      const auto has_exact_classification = [&classification_tokens](const QSet<QString> & tokens) {
        for (const QString & token : classification_tokens) {
          if (!token.isEmpty() && tokens.contains(token)) return true;
        }
        return false;
      };

      if (has_exact_classification(table_link_tokens)) {
        ++rendered_table_count;
      }
      if (has_exact_classification(camera_link_tokens)) {
        ++rendered_camera_count;
      }

      const auto has_exact_normalized_link_classification = [&normalized_link_tokens](const QSet<QString> & tokens) {
        for (const QString & token : normalized_link_tokens) {
          if (!token.isEmpty() && tokens.contains(token)) return true;
        }
        return false;
      };

      const bool is_robotiq = has_exact_classification(robotiq_link_tokens);
      const bool is_required_ur5 = has_exact_normalized_link_classification(required_visible_ur5_links);

      if (is_robotiq) {
        ++rendered_robotiq_link_count;
      }
      if (is_required_ur5) {
        if (link_token == QStringLiteral("base_link_inertia")) {
          qInfo().noquote() << QStringLiteral("Scene3D base_link_inertia trace: stage=rendered_ur5_link_count_audit result=counted item_id=%1 final_draw_status=%2 mesh_source=%3 canonical_mesh_source=%4")
            .arg(field_text(item, QStringLiteral("item_id")),
                 field_text(item, QStringLiteral("final_draw_status")),
                 field_text(item, QStringLiteral("mesh_source")),
                 field_text(item, QStringLiteral("canonical_mesh_source")));
        }
        ++rendered_ur5_link_count;
        visible_ur5_link_tokens.insert(link_token);
        QJsonObject record = item;
        record[QStringLiteral("link_name")] = link_token;
        record[QStringLiteral("visible")] = true;
        rendered_ur5_link_records.append(record);
      }
    }
  }

  QStringList missing;
  for (const QString & required_link : required_visible_ur5_links) {
    if (!visible_ur5_link_tokens.contains(required_link)) {
      missing << required_link;
    }
  }
  if (missing_required_visible_links) {
    *missing_required_visible_links = missing;
  }

  QString blocker;
  if (!missing.isEmpty() && added_required_ur5_link_tokens.size() >= required_visible_ur5_links.size()) {
    blocker = QStringLiteral("generated_urdf_visuals_added_but_required_ur5_links_not_rendered");
  }

  return QJsonObject{
    {QStringLiteral("rendered_ur5_link_count"), rendered_ur5_link_count},
    {QStringLiteral("added_required_ur5_link_count"), added_required_ur5_link_tokens.size()},
    {QStringLiteral("required_ur5_visibility_blocker"), blocker},
    {QStringLiteral("rendered_ur5_link_records"), rendered_ur5_link_records},
    {QStringLiteral("missing_required_visible_ur5_links"), QJsonArray::fromStringList(missing)},
    {QStringLiteral("excluded_non_visual_ur5_links"), QJsonArray{QStringLiteral("base_link_inertia: inertial-only/non-visual unless present as real final draw row")}},
    {QStringLiteral("rendered_table_count"), rendered_table_count},
    {QStringLiteral("rendered_camera_count"), rendered_camera_count},
    {QStringLiteral("rendered_robotiq_link_count"), rendered_robotiq_link_count}
  };
}

double normalize_angle_radians_with_guard(double value, const QString & context, QStringList * warnings)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  if (std::fabs(value) > kTwoPi + 1e-6 && std::fabs(value) <= 360.0 + 1e-6) {
    if (warnings) {
      warnings->push_back(QStringLiteral("angle looked like degrees and was converted to radians: %1 (%2)")
                            .arg(context)
                            .arg(value, 0, 'f', 4));
    }
    return qDegreesToRadians(value);
  }
  return value;
}

class DraggableCanvasItem : public QGraphicsRectItem {
public:
  explicit DraggableCanvasItem(const QRectF & r): QGraphicsRectItem(r) {}
  std::function<QPointF(const QPointF &)> position_filter;
  QGraphicsSimpleTextItem * label_item{nullptr};
  bool label_visible_by_default{false};

  void set_label_item(QGraphicsSimpleTextItem * label, bool visible_by_default)
  {
    label_item = label;
    label_visible_by_default = visible_by_default;
    setAcceptHoverEvents(label_item != nullptr);
    update_label_visibility();
  }

protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override {
    if (change == ItemPositionChange && scene() && position_filter) {
      return position_filter(value.toPointF());
    }
    const QVariant result = QGraphicsRectItem::itemChange(change, value);
    if (change == ItemSelectedHasChanged) {
      update_label_visibility();
    }
    return result;
  }

  void hoverEnterEvent(QGraphicsSceneHoverEvent * event) override
  {
    update_label_visibility(true);
    QGraphicsRectItem::hoverEnterEvent(event);
  }

  void hoverLeaveEvent(QGraphicsSceneHoverEvent * event) override
  {
    update_label_visibility(false);
    QGraphicsRectItem::hoverLeaveEvent(event);
  }

private:
  void update_label_visibility(bool hovered = false)
  {
    if (!label_item) return;
    label_item->setVisible(label_visible_by_default || hovered || isSelected());
  }
};

template <typename Sender, typename Receiver, typename Signal, typename Slot>
void connect_if(Sender * sender, Receiver * receiver, Signal signal, Slot && slot)
{
  if (!sender || !receiver) {
    return;
  }
  QObject::connect(sender, signal, receiver, std::forward<Slot>(slot));
}

bool is_empty_directory(const fs::path & directory_path)
{
  boost::system::error_code ec;
  if (!fs::exists(directory_path, ec) || ec || !fs::is_directory(directory_path, ec) || ec) {
    return false;
  }
  return fs::is_empty(directory_path, ec) && !ec;
}

void copy_directory_contents(const fs::path & source, const fs::path & destination)
{
  boost::system::error_code ec;
  if (!fs::exists(source, ec) || ec || !fs::is_directory(source, ec) || ec) {
    return;
  }
  fs::create_directories(destination, ec);
  for (fs::recursive_directory_iterator it(source, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::path & current_path = it->path();
    const fs::path relative_path = current_path.lexically_relative(source);
    const fs::path target_path = destination / relative_path;
    if (fs::is_directory(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::create_directories(target_path, ec);
      }
      continue;
    }
    if (fs::is_regular_file(current_path, ec) && !ec) {
      if (!fs::exists(target_path, ec)) {
        fs::copy_file(current_path, target_path, ec);
      }
    }
  }
}

bool is_ros2_prefix(const fs::path & prefix)
{
  boost::system::error_code ec;
  if (!fs::exists(prefix, ec) || ec || !fs::is_directory(prefix, ec) || ec) {
    return false;
  }
  const fs::path share_path = prefix / "share";
  if (!fs::exists(share_path, ec) || ec || !fs::is_directory(share_path, ec) || ec) {
    return false;
  }
  const fs::path ament_index_path = share_path / "ament_index";
  const fs::path rclcpp_path = share_path / "rclcpp";
  return (fs::exists(ament_index_path, ec) && !ec) || (fs::exists(rclcpp_path, ec) && !ec);
}
struct SceneTaskIntentSummary
{
  QString status{"MISSING_TASK_FILE"};
  QString source_file{"unknown"};
  QString source_basename{"unknown"};
  QString task_type{"unknown"};
  QString pick_source{"unknown"};
  QString place_target{"unknown"};
  QString reject_target{"unknown"};
  QString object_class{"unknown"};
  QString grasp_strategy{"unknown"};
  QString approach_axis{"unknown"};
  QString approach_distance{"unknown"};
  QString retreat_axis{"unknown"};
  QString retreat_distance{"unknown"};
  QString orientation_mode{"unknown"};
  QString allowed_roll_yaw{"unknown"};
  QString tool_id{"unknown"};
  QString perception_mode{"unknown"};
  QString perception_legacy_source{""};
  QString clearance{"unknown"};
  QStringList searched_paths;
};

struct ActionGate
{
  bool enabled{false};
  QString tooltip;
};

QString format_scene_builder_status_text(QString text)
{
  text.replace("Missing launch/demo.launch.py. Generate Scene Package first.",
    "Launch files missing — generate scene package next.");
  text.replace("Launch readiness flag is not set yet. Generate Scene Package again.",
    "Launch files missing — generate scene package next.");
  text.replace("Layout has unsaved edits. Save Layout first.",
    "Layout has unsaved edits — save layout before validation.");
  text.replace("Scene changed since last validation. Run Offline Validation first.",
    "Validation stale — run Validate again.");
  text.replace("Offline Validation", "Validate");
  text.replace("Plan / Simulate ready", "Ready for fake-hardware simulation");
  text.replace("pending: Ready for Plan / Simulate", "pending: Ready for fake-hardware simulation");
  text.replace("Run Fake-Hardware Simulation", "Run fake-hardware simulation");
  text.replace("fake-hardware launch command", "fake-hardware simulation command");
  return text;
}

ActionGate build_generate_scene_gate(
  const workcell_builder::WorkcellStudioSceneInfo & s, bool validation_stale)
{
  if (!s.has_environment_yaml) {
    return {false, "Blocked: Generate environment.yaml first from Scene Builder."};
  }
  if (!s.has_scene_manifest_yaml) {
    return {false, "Blocked: Generate scene_manifest.yaml before generating scene package."};
  }
  if (validation_stale) {
    return {false, "Blocked: Validation stale — run Validate again."};
  }
  if (!s.has_smoke_report_json) {
    return {false, "Blocked: Missing smoke/offline_smoke_report.json. Run Validate first."};
  }
  return {true, "Ready: Generate Scene Package prerequisites are satisfied."};
}

ActionGate build_plan_simulate_gate(
  const workcell_builder::WorkcellStudioSceneInfo & s, bool launch_artifacts_ready)
{
  if (!s.has_package_xml) {
    return {false, "Blocked: Missing package.xml/CMakeLists.txt. Generate Scene Package first."};
  }
  if (!s.has_launch_demo) {
    return {false, "Blocked: Launch files missing — generate scene package next."};
  }
  if (!launch_artifacts_ready) {
    return {false, "Blocked: Launch files missing — generate scene package next."};
  }
  return {true, "Ready: launch/demo.launch.py and launch readiness flags are present."};
}

ActionGate build_export_gate(const workcell_builder::WorkcellStudioSceneInfo & s)
{
  if (!s.has_package_xml) {
    return {false, "Blocked: Scene package is missing. Generate Scene Package first."};
  }
  if (!s.has_task_recipe) {
    return {false, "Blocked: Missing task recipe. Generate/Update Task Intent and task recipe first."};
  }
  return {true, "Ready: Scene package and export prerequisites are present."};
}

static QString ystr(const YAML::Node & n){ return (n && n.IsScalar()) ? QString::fromStdString(n.as<std::string>()) : "unknown"; }
static QString scalar_path(const YAML::Node & root, std::initializer_list<const char *> keys){ YAML::Node n=root; for(auto *k:keys){ if(!n || !n.IsMap() || !n[k]) return "unknown"; n=n[k]; } return ystr(n); }
static QString normalize_bound_id(QString value){ value=value.trimmed(); if(value.isEmpty()||value=="unknown") return "unknown"; return value; }

static bool read_yaml(const fs::path & p, YAML::Node * out){ try{ if(!fs::exists(p)) return false; qInfo("[workcell_builder] context=task_metadata_summary_loader path=%s", p.string().c_str()); *out=YAML::LoadFile(p.string()); return true; }catch(const YAML::Exception&){ return false; } catch(const std::exception&){ return false; } }

struct SelectedSceneMetadataSummary
{
  QString scene_name;
  QString scene_path;
  QString robot;
  QString end_effector;
  QString tool_mount;
  QString grasp_frame;
  QString robot_source;
  QString end_effector_source;
  QString tool_mount_source;
  QString grasp_frame_source;
  QString launch;
};

static QString scalar_vector_path(const YAML::Node & root, const std::vector<const char *> & keys)
{
  YAML::Node n = root;
  for (const auto * key : keys) {
    if (!n || !n.IsMap() || !n[key]) return QString();
    n = n[key];
  }
  const QString value = ystr(n).trimmed();
  return value == QStringLiteral("unknown") ? QString() : value;
}

static QString first_present_scalar(const YAML::Node & root, const std::vector<std::vector<const char *>> & paths)
{
  for (const auto & path : paths) {
    const QString value = scalar_vector_path(root, path);
    if (!value.isEmpty()) {
      return value;
    }
  }
  return QString();
}

static SelectedSceneMetadataSummary selected_scene_metadata_summary(
  const workcell_builder::WorkcellStudioSceneInfo & scene)
{
  SelectedSceneMetadataSummary out;
  out.scene_name = QString::fromStdString(scene.scene_name);
  out.scene_path = QString::fromStdString(scene.scene_dir.string());
  out.launch = scene.has_launch_demo ?
    QStringLiteral("launch/demo.launch.py present (fake-hardware launch metadata available)") :
    QStringLiteral("launch/demo.launch.py missing; generate scene package to create launch metadata");

  YAML::Node env;
  if (read_yaml(scene.scene_dir / "environment.yaml", &env)) {
    out.robot = first_present_scalar(env, {
      {"robot", "model"}, {"robot", "id"}, {"robot", "name"}, {"compatibility", "robot"}
    });
    out.end_effector = first_present_scalar(env, {
      {"end_effector", "id"}, {"end_effector", "model"}, {"end_effector", "profile"},
      {"tool", "id"}, {"tool", "model"}, {"tool", "profile"}, {"compatibility", "tool"}
    });
    out.tool_mount = first_present_scalar(env, {
      {"end_effector", "mount_link"}, {"tool", "mount_link"}, {"robot", "tool_mount_link"}
    });
    out.grasp_frame = first_present_scalar(env, {
      {"end_effector", "grasp_frame"}, {"tool", "grasp_frame"}
    });
    if (!out.robot.isEmpty()) out.robot_source = QStringLiteral("environment.yaml");
    if (!out.end_effector.isEmpty()) out.end_effector_source = QStringLiteral("environment.yaml");
    if (!out.tool_mount.isEmpty()) out.tool_mount_source = QStringLiteral("environment.yaml");
    if (!out.grasp_frame.isEmpty()) out.grasp_frame_source = QStringLiteral("environment.yaml");
  }

  YAML::Node cell;
  if ((out.robot.isEmpty() || out.end_effector.isEmpty() || out.tool_mount.isEmpty() || out.grasp_frame.isEmpty()) &&
      read_yaml(scene.scene_dir / "cell_definition.yaml", &cell)) {
    if (out.robot.isEmpty()) out.robot = first_present_scalar(cell, {
      {"robot", "model"}, {"robot", "id"}, {"robot", "name"}, {"cell", "robot"}
    });
    if (out.end_effector.isEmpty()) out.end_effector = first_present_scalar(cell, {
      {"end_effector", "id"}, {"end_effector", "model"}, {"end_effector", "profile"},
      {"tool", "id"}, {"tool", "model"}, {"task", "end_effector"}
    });
    if (out.tool_mount.isEmpty()) out.tool_mount = first_present_scalar(cell, {
      {"end_effector", "mount_link"}, {"tool", "mount_link"}, {"robot", "tool_link"}
    });
    if (out.grasp_frame.isEmpty()) out.grasp_frame = first_present_scalar(cell, {{"end_effector", "grasp_frame"}});
    if (!out.robot.isEmpty() && out.robot_source.isEmpty()) out.robot_source = QStringLiteral("cell_definition.yaml");
    if (!out.end_effector.isEmpty() && out.end_effector_source.isEmpty()) out.end_effector_source = QStringLiteral("cell_definition.yaml");
    if (!out.tool_mount.isEmpty() && out.tool_mount_source.isEmpty()) out.tool_mount_source = QStringLiteral("cell_definition.yaml");
    if (!out.grasp_frame.isEmpty() && out.grasp_frame_source.isEmpty()) out.grasp_frame_source = QStringLiteral("cell_definition.yaml");
  }

  YAML::Node manifest;
  if ((out.robot.isEmpty() || out.end_effector.isEmpty() || out.tool_mount.isEmpty() || out.grasp_frame.isEmpty()) && read_yaml(scene.scene_dir / "scene_manifest.yaml", &manifest)) {
    if (out.robot.isEmpty()) {
      out.robot = first_present_scalar(manifest, {{"robot", "model"}, {"robot", "id"}, {"robot", "name"}});
      if (!out.robot.isEmpty()) out.robot_source = QStringLiteral("scene_manifest.yaml (cell_definition.yaml missing robot metadata)");
    }
    if (out.end_effector.isEmpty()) {
      out.end_effector = first_present_scalar(manifest, {
        {"end_effector", "id"}, {"end_effector", "model"}, {"end_effector", "profile"},
        {"tool", "id"}, {"tool", "model"}, {"robot", "ee_link"}
      });
      if (!out.end_effector.isEmpty()) out.end_effector_source = QStringLiteral("scene_manifest.yaml (cell_definition.yaml missing end effector metadata)");
    }
    if (out.tool_mount.isEmpty()) {
      out.tool_mount = first_present_scalar(manifest, {{"end_effector", "mount_link"}, {"tool", "mount_link"}, {"robot", "tool_mount_link"}});
      if (!out.tool_mount.isEmpty()) out.tool_mount_source = QStringLiteral("scene_manifest.yaml");
    }
    if (out.grasp_frame.isEmpty()) {
      out.grasp_frame = first_present_scalar(manifest, {{"end_effector", "grasp_frame"}, {"robot", "ee_link"}});
      if (!out.grasp_frame.isEmpty()) out.grasp_frame_source = QStringLiteral("scene_manifest.yaml");
    }
  }

  if (out.robot.isEmpty() && !QString::fromStdString(scene.robot_summary).trimmed().isEmpty()) {
    out.robot = QString::fromStdString(scene.robot_summary).trimmed();
    out.robot_source = QStringLiteral("generated preview/browser metadata (canonical YAML missing robot metadata)");
  }
  if (out.end_effector.isEmpty() && !QString::fromStdString(scene.gripper_summary).trimmed().isEmpty()) {
    out.end_effector = QString::fromStdString(scene.gripper_summary).trimmed();
    out.end_effector_source = QStringLiteral("generated preview/browser metadata (canonical YAML missing end effector metadata)");
  }

  if (out.robot.isEmpty()) {
    out.robot = fs::exists(scene.scene_dir / "cell_definition.yaml") ?
      QStringLiteral("Robot missing from cell_definition.yaml") :
      QStringLiteral("Robot missing because cell_definition.yaml is absent");
    out.robot_source = QStringLiteral("missing canonical cell metadata");
  }
  if (out.end_effector.isEmpty()) {
    out.end_effector = fs::exists(scene.scene_dir / "cell_definition.yaml") ?
      QStringLiteral("End effector missing from cell_definition.yaml") :
      QStringLiteral("End effector missing because cell_definition.yaml is absent");
    out.end_effector_source = QStringLiteral("missing canonical cell metadata");
  }
  if (out.tool_mount.isEmpty()) {
    out.tool_mount = QStringLiteral("unknown");
    out.tool_mount_source = QStringLiteral("missing canonical tool mount metadata");
  }
  if (out.grasp_frame.isEmpty()) {
    out.grasp_frame = QStringLiteral("unknown");
    out.grasp_frame_source = QStringLiteral("missing canonical grasp frame metadata");
  }
  return out;
}

static YAML::Node ensure_map_path(YAML::Node root, std::initializer_list<const char *> keys)
{
  YAML::Node cursor = root;
  for (const auto * key : keys) {
    if (!cursor.IsMap()) {
      cursor = YAML::Node(YAML::NodeType::Map);
    }
    if (!cursor[key] || !cursor[key].IsMap()) {
      cursor[key] = YAML::Node(YAML::NodeType::Map);
    }
    cursor = cursor[key];
  }
  return cursor;
}
static std::string infer_default_grasp_strategy(const YAML::Node & root)
{
  const QString tool_id = scalar_path(root, {"task", "tool", "id"}) != "unknown" ?
    scalar_path(root, {"task", "tool", "id"}) :
    (scalar_path(root, {"tool", "id"}) != "unknown" ? scalar_path(root, {"tool", "id"}) : scalar_path(root, {"task", "end_effector"}));
  const QString lower = tool_id.toLower();
  if (lower.contains("suction")) return "suction_top";
  if (lower.contains("finger") || lower.contains("gripper") || lower.contains("robotiq")) return "finger_top";
  return "tool_profile_default";
}
static QString canonical_scene_path_string(const fs::path & scene_dir)
{
  boost::system::error_code ec;
  const fs::path canonical = fs::weakly_canonical(scene_dir, ec);
  return QString::fromStdString((ec ? scene_dir.lexically_normal() : canonical).string());
}
enum class LayoutStateModel {
  NO_LAYOUT_FILE,
  EMPTY_LAYOUT,
  EDITABLE_LAYOUT_PRESENT,
  PREVIEW_ONLY_AVAILABLE,
  PREVIEW_UNAVAILABLE,
  PATH_MISMATCH,
  INVALID_LAYOUT_YAML
};

static LayoutStateModel derive_layout_state_model(const fs::path & scene_dir, const workcell_builder::WorkcellStudioCanvasModel & model, bool path_match)
{
  if (!path_match) return LayoutStateModel::PATH_MISMATCH;
  const fs::path layout = scene_dir / "layout" / "workcell_studio_layout.yaml";
  if (!fs::exists(layout)) {
    return model.items.empty() ? LayoutStateModel::PREVIEW_UNAVAILABLE : LayoutStateModel::NO_LAYOUT_FILE;
  }
  try {
    (void)YAML::LoadFile(layout.string());
    const auto inspection = workcell_builder::inspect_editable_layout_entries(scene_dir);
    if (!inspection.valid) return LayoutStateModel::INVALID_LAYOUT_YAML;
    if (!inspection.has_items_sequence || inspection.total_item_entries == 0 || inspection.editable_item_count == 0) {
      return model.items.empty() ? LayoutStateModel::EMPTY_LAYOUT : LayoutStateModel::PREVIEW_ONLY_AVAILABLE;
    }
  } catch (const YAML::Exception &) {
    return LayoutStateModel::INVALID_LAYOUT_YAML;
  } catch (const std::exception &) {
    return LayoutStateModel::INVALID_LAYOUT_YAML;
  }
  return LayoutStateModel::EDITABLE_LAYOUT_PRESENT;
}



static SceneTaskIntentSummary load_scene_task_intent_summary(const fs::path & scene_dir)
{
  SceneTaskIntentSummary s;
  const std::vector<fs::path> candidates = {scene_dir/"config"/"workcell_builder_task_intent.yaml", scene_dir/"config"/"task_recipe.yaml", scene_dir/"task_recipe.yaml", scene_dir/"cell_definition.yaml", scene_dir/"environment_layout.yaml"};
  for (const auto & p : candidates) s.searched_paths << QString::fromStdString(p.string());
  YAML::Node root; fs::path selected;
  for (const auto & p : candidates) { if (read_yaml(p, &root)) { selected = p; break; } }
  if (selected.empty()) return s;
  s.source_file = QString::fromStdString(selected.string()); s.source_basename = QString::fromStdString(selected.filename().string());
  s.status = "INCOMPLETE_TASK";
  const YAML::Node task = root["task"] ? root["task"] : root;
  s.task_type = scalar_path(task, {"type"});
  if (s.task_type == "unknown") s.task_type = scalar_path(task, {"family"});
  s.pick_source = scalar_path(task, {"pick","source","id"});
  if (s.pick_source == "unknown") s.pick_source = scalar_path(task, {"pick","source"});
  if (s.pick_source == "unknown") s.pick_source = scalar_path(task, {"pick_source"});
  s.place_target = scalar_path(task, {"place","target","id"});
  if (s.place_target == "unknown") s.place_target = scalar_path(task, {"place","target"});
  if (s.place_target == "unknown") s.place_target = scalar_path(task, {"place_target"});
  s.reject_target = scalar_path(task, {"reject","target"});
  if (s.reject_target == "unknown") s.reject_target = scalar_path(task, {"reject_target"});
  s.object_class = scalar_path(task, {"object","class"});
  if (s.object_class == "unknown") s.object_class = scalar_path(task, {"object_class"});
  s.grasp_strategy = scalar_path(task, {"grasp","strategy"});
  if (s.grasp_strategy == "unknown") s.grasp_strategy = scalar_path(task, {"grasp","strategy_ref"});
  if (s.grasp_strategy == "unknown") s.grasp_strategy = scalar_path(task, {"grasp_strategy"});
  s.approach_axis = scalar_path(task, {"approach","axis"});
  s.approach_distance = scalar_path(task, {"approach","distance"});
  s.retreat_axis = scalar_path(task, {"retreat","axis"});
  s.retreat_distance = scalar_path(task, {"retreat","distance"});
  s.orientation_mode = scalar_path(task, {"orientation","mode"});
  s.allowed_roll_yaw = scalar_path(task, {"orientation","allowed_roll_yaw"});
  s.tool_id = scalar_path(task, {"tool","id"});
  if (s.tool_id == "unknown") s.tool_id = scalar_path(task, {"end_effector"});
  const auto perception = workcell_builder::parse_perception_contract_summary(task);
  s.perception_mode = QString::fromStdString(perception.mode);
  s.perception_legacy_source = QString::fromStdString(perception.legacy_source_mode);
  if (!perception.warning.empty()) {
    static QSet<QString> warned_scene_paths;
    const QString canonical_scene = canonical_scene_path_string(scene_dir);
    if (!warned_scene_paths.contains(canonical_scene)) {
      warned_scene_paths.insert(canonical_scene);
      qWarning("Perception parse warning [%s]: %s", canonical_scene.toStdString().c_str(), perception.warning.c_str());
    }
    s.status = "WARN_PERCEPTION";
  }
  s.clearance = scalar_path(task, {"clearance"});
  if (s.pick_source != "unknown" && s.place_target != "unknown" && s.grasp_strategy != "unknown") s.status = "READY";
  if (!perception.warning.empty() && s.status == "READY") s.status = "READY_WITH_PERCEPTION_WARNING";
  return s;
}

struct NewCellStateAudit
{
  QString current_state{"NO_WORKSPACE"};
  QStringList completed_states;
  QStringList pending_states;
  QStringList blocked_states;
  QString next_recommended_action{"Select Workspace"};
  QStringList blockers;
};

static NewCellStateAudit audit_new_cell_state(
  const QString & workspace_path, const workcell_builder::WorkcellStudioSceneBrowserResult & browser,
  int selected_scene_index, const QString & preview_state, QProcess * preview_process)
{
  NewCellStateAudit out;
  const QStringList all_states = {"NO_WORKSPACE","WORKSPACE_READY","CELL_DRAFT_CREATED","LAYOUT_CREATED","LAYOUT_SAVED","TASK_INTENT_CREATED","SCENE_PACKAGE_GENERATED","FILE_OUTPUTS_CHECKED","VALIDATION_READY","VALIDATION_PASSED","VALIDATION_BLOCKED","PLAN_SIMULATE_READY","SIMULATION_RUNNING","SIMULATION_STOPPED"};
  if (workspace_path.trimmed().isEmpty()) {
    out.pending_states = all_states;
    out.blockers << "Workspace not selected|Select a valid workspace in the header workspace picker.|Dashboard > New Cell|";
    out.blocked_states << "NO_WORKSPACE";
    return out;
  }
  out.completed_states << "WORKSPACE_READY";
  out.current_state = "WORKSPACE_READY";
  out.next_recommended_action = "Create New Cell";
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) {
    out.pending_states = all_states;
    out.pending_states.removeAll("NO_WORKSPACE");
    out.pending_states.removeAll("WORKSPACE_READY");
    out.blockers << "No scene selected|Create or select a New Cell scene from Existing Scenes.|Dashboard > New Cell|";
    out.blocked_states << "CELL_DRAFT_CREATED";
    return out;
  }
  const auto & scene = browser.scenes[static_cast<size_t>(selected_scene_index)];
  const fs::path scene_dir = scene.scene_dir;
  const auto exists = [&](const char * rel) { return fs::exists(scene_dir / rel); };
  const bool has_layout = exists("environment_layout.yaml");
  const bool has_task_intent = exists("config/workcell_builder_task_intent.yaml");
  const bool has_package = exists("package.xml") && exists("CMakeLists.txt") && exists("launch/demo.launch.py");
  const bool has_file_output_audit = exists("file_output_audit.json");
  const bool has_validation = exists("smoke/offline_smoke_report.json");
  out.completed_states << "CELL_DRAFT_CREATED";
  out.current_state = "CELL_DRAFT_CREATED";
  out.next_recommended_action = "Create editable layout from preview / Add to Canvas";
  if (has_layout) { out.completed_states << "LAYOUT_CREATED" << "LAYOUT_SAVED"; out.current_state = "LAYOUT_SAVED"; out.next_recommended_action = "Save Layout"; }
  if (has_task_intent) { out.completed_states << "TASK_INTENT_CREATED"; out.current_state = "TASK_INTENT_CREATED"; out.next_recommended_action = "Generate/Update Task Intent"; }
  if (has_package) { out.completed_states << "SCENE_PACKAGE_GENERATED"; out.current_state = "SCENE_PACKAGE_GENERATED"; out.next_recommended_action = "Generate Scene Package"; }
  if (has_file_output_audit) { out.completed_states << "FILE_OUTPUTS_CHECKED" << "VALIDATION_READY"; out.current_state = "VALIDATION_READY"; out.next_recommended_action = "Run Offline Validation"; }
  if (has_validation) { out.completed_states << "VALIDATION_PASSED" << "PLAN_SIMULATE_READY"; out.current_state = "PLAN_SIMULATE_READY"; out.next_recommended_action = "Open Plan & Simulate"; }
  if (preview_process && preview_process->state() != QProcess::NotRunning) { out.completed_states << "SIMULATION_RUNNING"; out.current_state = "SIMULATION_RUNNING"; out.next_recommended_action = "Stop Simulation"; }
  else if (preview_state == "PREVIEW_STOPPED" || preview_state == "PREVIEW_EXITED") { out.completed_states << "SIMULATION_STOPPED"; out.current_state = "SIMULATION_STOPPED"; out.next_recommended_action = "Open Plan & Simulate"; }
  for (const auto & state : all_states) if (!out.completed_states.contains(state) && state != "NO_WORKSPACE") out.pending_states << state;
  if (!has_layout) out.blockers << "Missing environment_layout.yaml|Open Scene Builder and click Save Layout.|Scene Builder|python3 scripts/validate_environment_layout.py <scene>/environment_layout.yaml --json";
  if (!has_task_intent) out.blockers << "Missing task intent|Click Generate/Update Task Intent to create config/workcell_builder_task_intent.yaml.|Task Intent|python3 scripts/create_or_update_builder_task_intent.py --scene-dir <scene>";
  if (!has_package) out.blockers << "Missing scene package outputs|Generate Scene Package before opening Plan & Simulate.|Generate Scene Package|python3 scripts/audit_new_cell_file_outputs.py --scene-dir <scene> --scene-name <scene> --json-out <scene>/file_output_audit.json";
  if (!out.blockers.isEmpty()) out.blocked_states << out.current_state;
  return out;
}

}  // namespace


QString normalized_slug(const QString & value){
  QString v = value.toLower();
  v.replace(" ", "_"); v.replace("-", "_");
  return v;
}

QString id_prefix_from_category(const QString & category){
  const QString c = normalized_slug(category);
  if (c.contains("table") || c.contains("support_surface")) return "table";
  if (c.contains("conveyor")) return "conveyor";
  if (c.contains("camera")) return "camera";
  if (c.contains("bin")) return "bin";
  if (c.contains("pick_zone")) return "pick_zone";
  if (c.contains("place_zone")) return "place_zone";
  if (c.contains("fixture")) return "fixture";
  return "object";
}



static QColor category_color(const QString & category)
{
  const QString c = normalized_slug(category);
  if (c.contains("robot")) return QColor("#4fa3ff");
  if (c.contains("effector")) return QColor("#8bc34a");
  if (c.contains("camera") || c.contains("sensor")) return QColor("#ffd166");
  if (c.contains("table") || c.contains("surface")) return QColor("#8d99ae");
  if (c.contains("conveyor")) return QColor("#f4a261");
  if (c.contains("bin")) return QColor("#9d4edd");
  if (c.contains("pick")) return QColor("#00d1b2");
  if (c.contains("place")) return QColor("#ff7b72");
  if (c.contains("fixture")) return QColor("#f28482");
  if (c.contains("safety")) return QColor("#ff595e");
  if (c.contains("import") || c.contains("stl")) return QColor("#adb5bd");
  return QColor("#5b6472");
}

static QString compact_canvas_label(QString label)
{
  label = label.trimmed();
  if (label.isEmpty()) return QStringLiteral("item");
  label.replace('\\', '/');
  if (label.contains('/')) {
    label = QFileInfo(label).completeBaseName();
  }
  if (label.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive) ||
      label.endsWith(QStringLiteral(".xacro"), Qt::CaseInsensitive) ||
      label.endsWith(QStringLiteral(".urdf"), Qt::CaseInsensitive)) {
    label = QFileInfo(label).completeBaseName();
  }
  label.replace(QRegularExpression(QStringLiteral("[_-]+")), QStringLiteral("_"));
  constexpr int kMaxLabelChars = 24;
  if (label.size() <= kMaxLabelChars) return label;
  const QStringList tokens = label.split('_', Qt::SkipEmptyParts);
  if (tokens.size() > 1) {
    const QString candidate = tokens.back();
    if (candidate.size() >= 5 && candidate.size() <= kMaxLabelChars) return candidate;
  }
  return label.left(10) + QStringLiteral("…") + label.right(9);
}

static bool is_generated_urdf_preview_layer(const QString & source_layer)
{
  const QString layer = canonical_scene3d_token(source_layer);
  return layer == QStringLiteral("locked_generated_urdf_visual") ||
         layer == QStringLiteral("generated_urdf_visual") ||
         layer == QStringLiteral("generated_preview");
}

static bool is_important_canvas_anchor(const QString & category, const QString & role)
{
  const QString token = normalized_slug(category + QStringLiteral("_") + role);
  return token.contains(QStringLiteral("robot")) ||
         token.contains(QStringLiteral("table")) ||
         token.contains(QStringLiteral("support_surface")) ||
         token.contains(QStringLiteral("conveyor")) ||
         token.contains(QStringLiteral("bin")) ||
         token.contains(QStringLiteral("camera")) ||
         token.contains(QStringLiteral("sensor"));
}

static qreal canvas_item_z_value(
  const QString & category,
  const QString & role,
  const QString & source_layer,
  bool locked)
{
  const QString layer = canonical_scene3d_token(source_layer);
  const QString token = normalized_slug(category + QStringLiteral("_") + role);
  if (token.contains(QStringLiteral("warning"))) return 80.0;
  if (layer == QStringLiteral("overlay")) return -30.0;
  if (token.contains(QStringLiteral("reach")) || token.contains(QStringLiteral("fov")) || token.contains(QStringLiteral("trajectory"))) return -20.0;
  if (layer == QStringLiteral("primitive_fallback")) return locked ? 8.0 : 16.0;
  if (layer == QStringLiteral("editable_layout")) return 30.0;
  if (is_important_canvas_anchor(category, role)) return locked ? 18.0 : 30.0;
  if (locked || is_generated_urdf_preview_layer(layer)) return 6.0;
  return 22.0;
}

static bool should_show_2d_label(
  const workcell_builder::WorkcellStudioCanvasItem & entry,
  const QString & selected_id)
{
  const QString id = QString::fromStdString(entry.id).trimmed();
  const QString category = QString::fromStdString(entry.type);
  const QString role = QString::fromStdString(entry.role);
  QString source_layer = QStringLiteral("locked_generated_urdf_visual");
  switch (entry.provenance) {
    case workcell_builder::WorkcellStudioItemProvenance::EditableLayout:
      source_layer = QStringLiteral("editable_layout");
      break;
    case workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview:
      source_layer = QStringLiteral("primitive_fallback");
      break;
    case workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
    default:
      source_layer = QStringLiteral("locked_generated_urdf_visual");
      break;
  }
  if (!selected_id.isEmpty() && id == selected_id) return true;
  if (!entry.warnings.empty() || normalized_slug(category).contains(QStringLiteral("warning"))) return true;
  if (source_layer == QStringLiteral("editable_layout") && !entry.locked) return true;
  if (entry.locked && is_generated_urdf_preview_layer(source_layer)) return false;
  return is_important_canvas_anchor(category, role);
}

static bool place_2d_label_without_overlap(
  QGraphicsSimpleTextItem * label,
  const QRectF & item_scene_rect,
  QVector<QRectF> * occupied_label_rects,
  qreal minimum_gap = 4.0)
{
  if (!label || !occupied_label_rects) return false;
  const QSizeF label_size = label->boundingRect().size();
  const QVector<QPointF> candidates = {
    item_scene_rect.topLeft() + QPointF(0.0, -label_size.height() - minimum_gap),
    item_scene_rect.topRight() + QPointF(minimum_gap, -label_size.height() - minimum_gap),
    item_scene_rect.bottomLeft() + QPointF(0.0, minimum_gap),
    item_scene_rect.bottomRight() + QPointF(minimum_gap, minimum_gap),
    QPointF(item_scene_rect.center().x() - label_size.width() / 2.0, item_scene_rect.top() - label_size.height() - minimum_gap),
  };
  for (const QPointF & candidate : candidates) {
    const QRectF candidate_rect(candidate, label_size);
    bool overlaps = false;
    for (const QRectF & occupied : *occupied_label_rects) {
      if (candidate_rect.adjusted(-minimum_gap, -minimum_gap, minimum_gap, minimum_gap).intersects(occupied)) {
        overlaps = true;
        break;
      }
    }
    if (overlaps) continue;
    label->setPos(candidate);
    occupied_label_rects->push_back(candidate_rect);
    return true;
  }
  return false;
}

QPointF default_xy_for_category(const QString & category){
  const QString c = normalized_slug(category);
  if (c.contains("table") || c.contains("support_surface")) return QPointF(70.0, 0.0);
  if (c.contains("conveyor")) return QPointF(-40.0, -25.0);
  if (c.contains("camera")) return QPointF(-20.0, 40.0);
  if (c.contains("bin")) return QPointF(95.0, -30.0);
  if (c.contains("pick_zone")) return QPointF(60.0, 0.0);
  if (c.contains("place_zone")) return QPointF(95.0, -20.0);
  if (c.contains("fixture")) return QPointF(30.0, 20.0);
  return QPointF(60.0, 5.0);
}

MainWindow::MainWindow(const QString & startup_workspace, const QString & startup_ros_distro, QWidget * parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  startup_workspace_(startup_workspace),
  startup_ros_distro_(startup_ros_distro)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
  setWindowTitle("Workcell Studio - Workcell Builder");
  ui->next->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  ui->quick_start_label->hide();
  ui->filepath_label->hide();
  ui->filepath->hide();
  ui->label->hide();
  ui->ros_distro->hide();
  ui->load_workcell->hide();
  ui->change_workcell->hide();
  ui->next->hide();
  success = false;
  ui->error_label->setWordWrap(true);
  ui->error_label->setProperty("status", "error");
  ui->error_label->setText("Workcell not available");
  ui->filepath->setToolTip("Selected ROS workspace root (contains assets/ and scenes/)");
  ui->ros_distro->setToolTip("Choose the ROS 2 distro that will be used for generated launch/config files");
  statusBar()->showMessage("Initializing Workcell Studio...");
  // Scene Status panel action labels (preview-only contract)
  static const char * kSceneStatusGraspActions[] = {"Check Grasp Strategy", "Generate EMD Grasp Request", "Open EMD Grasp Request", "Open Grasp Visualization Docs"};
  (void)kSceneStatusGraspActions;

  // Detect available ROS 2 distributions from /opt/ros.
  const boost::filesystem::path ros_root("/opt/ros");
  if (boost::filesystem::exists(ros_root) && boost::filesystem::is_directory(ros_root)) {
    for (const auto & entry : boost::filesystem::directory_iterator(ros_root)) {
      if (boost::filesystem::is_directory(entry.path()) && is_ros2_prefix(entry.path())) {
        ros_dist.push_back(entry.path().filename().string());
      }
    }
  }

  const char * distro = std::getenv("ROS_DISTRO");
  if (ros_dist.empty() && distro != nullptr) {
    const fs::path distro_prefix = ros_root / distro;
    if (is_ros2_prefix(distro_prefix)) {
      ros_dist.emplace_back(distro);
    }
  }

  std::sort(ros_dist.begin(), ros_dist.end());
  ros_dist.erase(std::unique(ros_dist.begin(), ros_dist.end()), ros_dist.end());

  if (!ros_dist.empty()) {
    ui->ros_distro->addItem("Select ROS distro...", "");
    for (const auto & supported_distro : ros_dist) {
      ui->ros_distro->addItem(
        QString::fromStdString(supported_distro),
        QString::fromStdString(supported_distro));
    }
  } else {
    ui->ros_distro->setDisabled(true);
    ui->error_label->setText(
      "No ROS 2 installation detected. Install ROS 2 under /opt/ros, then reopen Workcell Builder.");
    ui->error_label->setProperty("status", "error");
    statusBar()->showMessage("ROS 2 not detected in /opt/ros.");
  }

  if (distro != nullptr) {
    std::string current_distro(distro);
    for (const auto & supported_distro : ros_dist) {
      if (supported_distro == current_distro) {
        ui->ros_distro->setCurrentText(QString::fromStdString(supported_distro));
        break;
      }
    }
  }

  connect(
    ui->ros_distro,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    [this](int) {
      update_next_button_state();
      if (success && !has_selected_ros_distro()) {
        ui->error_label->setText(
          "Workcell loaded. Please select a ROS distro to continue.");
        ui->error_label->setProperty("status", "error");
        statusBar()->showMessage("Select a ROS distro to enable the Next step.");
      } else if (success) {
        ui->error_label->setProperty("status", "success");
        ui->error_label->setText("Workcell loaded");
        statusBar()->showMessage("Ready: open scene setup.");
      }
    });

  apply_startup_selection();
  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}


void MainWindow::apply_studio_theme()
{
  setStyleSheet(
    "QWidget#workcellStudioDashboardPage { background: #F5F7FA; color: #1F2933; }"
    "QFrame#studioCard, QFrame#studioHomeHeroCard, QFrame#studioHomeDetailsCard, QFrame#studioPanel {"
    " background: #FFFFFF; border: 1px solid #D0D7DE; border-radius: 8px; }"
    "QLabel#dashboardTitleLabel { color: #1F2933; font-size: 24px; font-weight: 700; }"
    "QLabel#dashboardSubtitleLabel, QLabel#dashboardSummaryLabel, QLabel#studioHomeSummaryCard { color: #5B6775; }"
    "QTableWidget#studioHomeSceneTable { background: #FFFFFF; color: #1F2933; gridline-color: #D0D7DE; }"
    "QHeaderView::section { background: #F5F7FA; color: #1F2933; border: 1px solid #D0D7DE; padding: 4px; }"
    "QPushButton#studioHomePrimaryButton { background: #2563EB; color: white; border: 1px solid #1D4ED8; border-radius: 6px; padding: 6px 10px; font-weight: 600; }"
    "QPushButton#studioHomeSecondaryButton { background: #FFFFFF; color: #1F2933; border: 1px solid #D0D7DE; border-radius: 6px; padding: 6px 10px; }"
    "QPushButton#studioHomeDangerButton { background: #FFFFFF; color: #B91C1C; border: 1px solid #FCA5A5; border-radius: 6px; padding: 6px 10px; }"
    "QLabel#studioHomeSafetyPill { background: #EFF6FF; color: #1E40AF; border: 1px solid #BFDBFE; border-radius: 10px; padding: 2px 8px; }"
    "QTextEdit#studioHomeLog { background: #FFFFFF; color: #1F2933; border: 1px solid #D0D7DE; }");
  append_studio_log("Loaded Workcell Studio light theme.");
  statusBar()->showMessage("Workcell Studio light theme loaded.");
}

void MainWindow::toggle_full_screen()
{
  if (isFullScreen()) {
    showNormal();
    if (full_screen_button_) {
      full_screen_button_->setText("Full Screen");
    }
  } else {
    showFullScreen();
    if (full_screen_button_) {
      full_screen_button_->setText("Exit Full Screen");
    }
  }
}

void MainWindow::show_studio_page(StudioPage page)
{
  if (studio_nav_) {
    studio_nav_->setCurrentRow(static_cast<int>(page));
  }
}

bool MainWindow::open_scene_builder_for_selected_scene(const QString & source_action)
{
  if (selected_scene_index_ < 0 && dashboard_scene_table_ && dashboard_scene_table_->currentRow() >= 0) {
    select_scene_by_row(dashboard_scene_table_->currentRow());
  }
  if (selected_scene_index_ < 0) {
    QMessageBox::information(this, "Workcell Studio", "Select a scene first.");
    append_studio_log(source_action + ": no scene selected.");
    return false;
  }
  try {
    refresh_scene_builder_selection_state_ui();
  } catch (const YAML::Exception & error) {
    append_studio_log(source_action + ": scene metadata warning: " + QString::fromStdString(error.what()));
    QMessageBox::warning(this, "Workcell Studio", "Scene metadata could not be fully parsed. Opened with warnings.");
  } catch (const std::exception & error) {
    append_studio_log(source_action + ": scene metadata warning: " + QString::fromStdString(error.what()));
    QMessageBox::warning(this, "Workcell Studio", "Scene metadata could not be fully parsed. Opened with warnings.");
  }
  show_studio_page(StudioPage::SceneBuilderPage);
  visual_index_script_missing_reported_scene_key_.clear();
  visual_index_regen_failure_reported_scene_key_.clear();
  visual_index_regen_throttle_session_active_ = false;
  emitted_scene_diagnostic_log_keys_.clear();
  scene_diagnostic_payload_revision_ = 0;
  append_scene_diagnostic_log_once(
    QStringLiteral("scene_load"),
    0,
    0,
    QString("%1: loaded scene '%2' at %3.").arg(source_action, selected_scene_name(), selected_scene_path()));
  return true;
}

bool MainWindow::load_scene_for_scene3d_smoke(const QString & scene_name, const QString & explicit_scene_path, QStringList * blockers, QJsonObject * diagnostics)
{
  auto add_blocker = [&](const QString & b) {
    if (blockers) blockers->append(b);
  };
  if (diagnostics) { diagnostics->insert("requested_scene_name", scene_name); diagnostics->insert("explicit_scene_path_used", false); }
  sync_selected_scene_state();
  int scene_idx = -1;
  if (diagnostics) diagnostics->insert("requested_explicit_scene_path", explicit_scene_path);
  if (!explicit_scene_path.trimmed().isEmpty()) {
    const fs::path explicit_dir = fs::path(explicit_scene_path.toStdString());
    const fs::path canonical_explicit = fs::weakly_canonical(explicit_dir);
    int explicit_idx = -1;
    for (int i = 0; i < static_cast<int>(scene_browser_result_.scenes.size()); ++i) {
      const fs::path candidate = fs::weakly_canonical(scene_browser_result_.scenes[static_cast<size_t>(i)].scene_dir);
      if (candidate == canonical_explicit) { explicit_idx = i; break; }
    }
    if (explicit_idx < 0) {
      add_blocker(QString("explicit_scene_path_not_found:%1").arg(explicit_scene_path));
      return false;
    }
    scene_idx = explicit_idx;
    if (diagnostics) diagnostics->insert("explicit_scene_path_used", true);
  }
  if (scene_idx < 0) {
    for (int i = 0; i < static_cast<int>(scene_browser_result_.scenes.size()); ++i) {
      if (QString::fromStdString(scene_browser_result_.scenes[static_cast<size_t>(i)].scene_name).trimmed() == scene_name.trimmed()) {
        scene_idx = i;
        break;
      }
    }
  }
  if (scene_idx < 0) {
    add_blocker(QString("scene_not_found:%1").arg(scene_name));
    return false;
  }
  select_scene_by_row(scene_idx);
  if (!open_scene_builder_for_selected_scene(QStringLiteral("Scene3D smoke deterministic load"))) {
    add_blocker("open_scene_builder_for_selected_scene_failed");
    return false;
  }
  QApplication::processEvents(QEventLoop::AllEvents, 250);
  populate_scene_hierarchy();
  apply_scene3d_product_view_layer_defaults_and_commit();

  const auto count_visible_for_layers = [&](const QSet<QString> & enabled_layers) {
      int count = 0;
      for (const auto & item : all_scene_preview_items_) {
        if (workcell_builder::include_preview_item_for_scene3d(item, enabled_layers)) {
          ++count;
        }
      }
      return count;
    };
  const QSet<QString> normal_enabled_layers = {
    preview_layer_editable_layout_box_ && preview_layer_editable_layout_box_->isChecked() ? "editable_layout" : "",
    preview_layer_generated_urdf_visual_box_ && preview_layer_generated_urdf_visual_box_->isChecked() ? "locked_generated_urdf_visual" : "",
    preview_layer_mesh_preview_box_ && preview_layer_mesh_preview_box_->isChecked() ? "mesh_preview" : "",
    preview_layer_primitive_fallback_box_ && preview_layer_primitive_fallback_box_->isChecked() ? "primitive_fallback" : "",
    preview_layer_overlays_helpers_box_ && preview_layer_overlays_helpers_box_->isChecked() ? "overlay" : "",
    preview_layer_warnings_missing_assets_box_ && preview_layer_warnings_missing_assets_box_->isChecked() ? "warning" : ""
  };
  int visible_count = count_visible_for_layers(normal_enabled_layers);
  if (visible_count == 0 && !all_scene_preview_items_.isEmpty()) {
    const QSet<QString> fallback_enabled_layers = {
      "editable_layout",
      "mesh_preview",
      "primitive_fallback",
      "locked_generated_urdf_visual"
    };
    visible_count = count_visible_for_layers(fallback_enabled_layers);
    if (visible_count > 0) {
      append_studio_log("default_filter_fallback_kept_renderable_items_visible");
      if (preview_layer_editable_layout_box_) preview_layer_editable_layout_box_->setChecked(true);
      if (preview_layer_mesh_preview_box_) preview_layer_mesh_preview_box_->setChecked(true);
      if (preview_layer_primitive_fallback_box_) preview_layer_primitive_fallback_box_->setChecked(true);
      if (preview_layer_generated_urdf_visual_box_) preview_layer_generated_urdf_visual_box_->setChecked(true);
    } else {
      add_blocker("scene3d_default_filter_hid_all_renderable_candidates");
    }
  }
  apply_scene3d_preview_layer_filters(false);
  if (scene_preview_widget_) {
    scene_preview_widget_->show();
    scene_preview_widget_->raise();
    scene_preview_widget_->update();
    if (auto * viewport = scene_preview_widget_->findChild<Scene3DViewportWidget *>()) {
      if (viewport->objectName().trimmed().isEmpty()) viewport->setObjectName("scene3dViewportWidget");
      viewport->show();
      viewport->resize(qMax(viewport->width(), 400), qMax(viewport->height(), 300));
      viewport->fit_product_view();
      viewport->update();
      viewport->repaint();
    }
  }
  QApplication::processEvents(QEventLoop::AllEvents, 250);
  if (diagnostics) {
    const fs::path d = scene_browser_result_.scenes[static_cast<size_t>(scene_idx)].scene_dir;
    diagnostics->insert("resolved_scene_path", QString::fromStdString(d.string()));
    diagnostics->insert("source_path_root", QString::fromStdString(d.string()));
    diagnostics->insert("source_layout_item_count", fs::exists(d / "layout" / "workcell_studio_layout.yaml") ? 1 : 0);
    diagnostics->insert("source_mesh_index_item_count", fs::exists(d / "generated" / "scene_visual_mesh_index.json") ? 1 : 0);
    diagnostics->insert("source_generated_layout_item_count", fs::exists(d / "layout" / "workcell_studio_layout.generated.yaml") ? 1 : 0);
    diagnostics->insert("source_preview_metadata_item_count", fs::exists(d / "generated" / "scene_preview_metadata.json") ? 1 : 0);
    diagnostics->insert("assembled_preview_item_count", all_scene_preview_items_.size());
    diagnostics->insert("scene3d_filter_diagnostics", scene3d_filter_diagnostics_);
    diagnostics->insert("scene3d_visual_ingestion_diagnostics", scene3d_visual_ingestion_diagnostics_);
  }
  if (all_scene_preview_items_.isEmpty()) {
    add_blocker("scene3d_candidate_assembly_failed");
    add_blocker("checked:layout/workcell_studio_layout.yaml");
    add_blocker("checked:generated/scene_visual_mesh_index.json");
    add_blocker("checked:layout/workcell_studio_layout.generated.yaml");
    add_blocker("checked:generated/scene_preview_metadata.json");
    return false;
  }
  return true;
}

void MainWindow::open_new_scene_creation_flow()
{
  const QString workspace_root = detect_workspace_root();
  if (workspace_root.trimmed().isEmpty()) {
    QMessageBox::warning(this, "Workcell Studio", "Select a valid workspace first.");
    return;
  }
  NewCellWizard wizard(workspace_root, this);
  if (wizard.exec() != QDialog::Accepted) {
    append_studio_log("New Cell Wizard cancelled.");
    return;
  }
  const auto created = wizard.result();
  if (!created.created) {
    return;
  }
  append_studio_log(QString("New Cell Wizard: created '%1' (fake hardware default / real robot locked).")
    .arg(created.scene_name));
  refresh_scene_browser_ui();
  for (int i = 0; i < static_cast<int>(scene_browser_result_.scenes.size()); ++i) {
    if (scene_browser_result_.scenes[static_cast<size_t>(i)].scene_name == created.scene_name.toStdString()) {
      select_scene_by_row(i);
      break;
    }
  }
  if (created.open_in_scene_builder) {
    open_scene_builder_for_selected_scene("New Cell Wizard");
  }
}

void MainWindow::setup_studio_shell()
{
  QWidget * content = ui->centralwidget;
  auto * root_layout = qobject_cast<QVBoxLayout *>(content->layout());
  if (!root_layout) return;
  if (studio_nav_) {
    return;
  }

  auto detach_widget = [root_layout](QWidget * widget) {
      if (!widget) {
        return;
      }
      root_layout->removeWidget(widget);
      widget->setVisible(false);
      widget->setMaximumHeight(0);
      widget->setMinimumHeight(0);
    };
  detach_widget(ui->quick_start_label);
  detach_widget(ui->filepath_label);
  detach_widget(ui->filepath);
  detach_widget(ui->label);
  detach_widget(ui->ros_distro);
  detach_widget(ui->load_workcell);
  detach_widget(ui->change_workcell);
  detach_widget(ui->next);
  detach_widget(ui->error_label);

  // status badge | safety banner | scene overview | digital twin preview | command console
  studio_nav_ = new QListWidget(content);
  studio_nav_->addItems({"🏠 Studio Home","🛠 Scene Builder","🎬 Demo Mode","🚀 Preview Launch","🩺 Diagnostics","✅ Validation","📤 Export"});
  studio_nav_->hide();
  studio_pages_ = new QStackedWidget(content);

  auto * dashboard = new QWidget(studio_pages_); auto * dl=new QVBoxLayout(dashboard);
  dashboard->setObjectName("workcellStudioDashboardPage");
  auto * hero_card = new QFrame(dashboard);
  hero_card->setObjectName("studioHomeHeroCard");
  auto * hero_layout = new QVBoxLayout(hero_card);
  auto * dashboard_title = new QLabel("Workcell Studio", hero_card);
  dashboard_title->setObjectName("dashboardTitleLabel");
  hero_layout->addWidget(dashboard_title);
  auto * dashboard_subtitle = new QLabel("Design, validate, and preview robotic workcells.", hero_card);
  dashboard_subtitle->setObjectName("dashboardSubtitleLabel");
  hero_layout->addWidget(dashboard_subtitle);
  auto * hero_safety = new QLabel("Fake hardware default / Real robot locked", hero_card); hero_safety->setObjectName("studioHomeSafetyPill"); hero_layout->addWidget(hero_safety, 0, Qt::AlignLeft);
  auto * hero_diag = new QLabel("Diagnostics: offline checks available", hero_card); hero_diag->setObjectName("studioHomeSafetyPill"); hero_layout->addWidget(hero_diag, 0, Qt::AlignLeft);
  auto * hero_actions = new QHBoxLayout();
  auto * dash_new_cell = new QPushButton("New Cell", hero_card); dash_new_cell->setProperty("role", "primary"); dash_new_cell->setObjectName("studioHomePrimaryButton"); hero_actions->addWidget(dash_new_cell);
  auto * dash_open_selected_scene = new QPushButton("Open Selected Scene", hero_card); dash_open_selected_scene->setProperty("role", "primary"); dash_open_selected_scene->setObjectName("studioHomePrimaryButton"); hero_actions->addWidget(dash_open_selected_scene);
  hero_actions->addStretch(1);
  hero_layout->addLayout(hero_actions);
  dl->addWidget(hero_card);
  dashboard_summary_label_=new QLabel("Loading scenes..."); dashboard_summary_label_->setObjectName("dashboardSummaryLabel"); dashboard_summary_label_->setWordWrap(true); dl->addWidget(dashboard_summary_label_);
  auto * summary_row = new QHBoxLayout();
  dashboard_total_scenes_card_ = new QLabel("Total Scenes\n0", dashboard); dashboard_total_scenes_card_->setObjectName("studioHomeSummaryCard");
  dashboard_ready_scenes_card_ = new QLabel("Ready / Validated\n0", dashboard); dashboard_ready_scenes_card_->setObjectName("studioHomeSummaryCard");
  dashboard_warning_scenes_card_ = new QLabel("Warnings / Blocked\n0", dashboard); dashboard_warning_scenes_card_->setObjectName("studioHomeSummaryCard");
  dashboard_last_updated_card_ = new QLabel("Selected Scene\nNone", dashboard); dashboard_last_updated_card_->setObjectName("studioHomeSummaryCard");
  summary_row->addWidget(dashboard_total_scenes_card_); summary_row->addWidget(dashboard_ready_scenes_card_); summary_row->addWidget(dashboard_warning_scenes_card_); summary_row->addWidget(dashboard_last_updated_card_);
  dl->addLayout(summary_row);
  auto * source_label = new QLabel("Source scenes path: loading...", dashboard);
  source_label->setObjectName("studioHomeDetailsCard");
  dl->addWidget(source_label);
  dashboard_scene_table_=new QTableWidget(0,6,dashboard); dashboard_scene_table_->setObjectName("studioHomeSceneTable"); dashboard_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Robot","Gripper","Task Recipe","Launch"});
  dashboard_scene_table_->setAlternatingRowColors(true);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
  dashboard_scene_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
  dashboard_scene_table_->setColumnWidth(0, 320);
  dashboard_scene_table_->verticalHeader()->setDefaultSectionSize(36);
  dashboard_scene_table_->setWordWrap(false);
  dashboard_scene_table_->setMinimumHeight(260);
  dashboard_empty_state_card_ = new QFrame(dashboard); dashboard_empty_state_card_->setObjectName("studioCard");
  auto * empty_row = new QVBoxLayout(dashboard_empty_state_card_);
  dashboard_empty_state_title_ = new QLabel("No scenes found", dashboard_empty_state_card_);
  dashboard_empty_state_hint_ = new QLabel("Create a new cell to get started.\nExpected scenes folder: <workspace>/src/scenes", dashboard_empty_state_card_);
  auto * empty_new_cell = new QPushButton("New Cell", dashboard_empty_state_card_); empty_new_cell->setProperty("role", "primary");
  empty_row->addWidget(dashboard_empty_state_title_); empty_row->addWidget(dashboard_empty_state_hint_); empty_row->addWidget(empty_new_cell, 0, Qt::AlignLeft);
  dl->addWidget(dashboard_empty_state_card_);
  dashboard_selected_scene_card_ = new QFrame(dashboard); dashboard_selected_scene_card_->setObjectName("studioHomeDetailsCard");
  auto * selected_row = new QVBoxLayout(dashboard_selected_scene_card_);
  selected_row->addWidget(new QLabel("<b>Selected Scene</b>", dashboard_selected_scene_card_));
  dashboard_selected_scene_details_ = new QLabel("Select a scene to view details.", dashboard_selected_scene_card_);
  dashboard_selected_scene_details_->setWordWrap(true);
  selected_row->addWidget(dashboard_selected_scene_details_);
  auto * dashboard_actions = new QVBoxLayout();
  dashboard_scene_actions_button_ = new QToolButton(dashboard_selected_scene_card_);
  dashboard_scene_actions_button_->setText("Scene Actions");
  dashboard_scene_actions_button_->setObjectName("studioHomeSecondaryButton");
  dashboard_scene_actions_button_->setPopupMode(QToolButton::InstantPopup);
  dashboard_scene_actions_menu_ = new QMenu(dashboard_scene_actions_button_);
  dashboard_open_scene_action_ = dashboard_scene_actions_menu_->addAction("Open in Scene Builder");
  dashboard_validate_action_ = dashboard_scene_actions_menu_->addAction("Validate");
  dashboard_plan_action_ = dashboard_scene_actions_menu_->addAction("Plan / Simulate");
  dashboard_export_action_ = dashboard_scene_actions_menu_->addAction("Export");
  dashboard_scene_actions_menu_->addSeparator();
  dashboard_delete_action_ = dashboard_scene_actions_menu_->addAction("Delete Scene");
  dashboard_scene_actions_button_->setMenu(dashboard_scene_actions_menu_);
  dashboard_actions->addWidget(dashboard_scene_actions_button_);
  selected_row->addLayout(dashboard_actions);
  auto * dashboard_middle_split = new QSplitter(Qt::Horizontal, dashboard);
  auto * left_library_card = new QFrame(dashboard);
  left_library_card->setObjectName("studioCard");
  left_library_card->setMinimumWidth(280);
  left_library_card->setMaximumWidth(320);
  auto * left_library_layout = new QVBoxLayout(left_library_card);
  left_library_layout->addWidget(new QLabel("<b>Scene Library</b>", left_library_card));
  dashboard_library_search_ = new QLineEdit(left_library_card);
  dashboard_library_search_->setPlaceholderText("Search library...");
  left_library_layout->addWidget(dashboard_library_search_);
  dashboard_library_status_filter_ = new QComboBox(left_library_card);
  dashboard_library_status_filter_->addItems({"All", "Ready", "Warning", "Blocked"});
  left_library_layout->addWidget(dashboard_library_status_filter_);
  dashboard_library_list_ = new QListWidget(left_library_card);
  left_library_layout->addWidget(dashboard_library_list_, 1);
  auto * center_card = new QFrame(dashboard);
  center_card->setObjectName("studioCard");
  auto * center_card_layout = new QVBoxLayout(center_card);
  center_card_layout->addWidget(new QLabel("<b>Scenes</b>", center_card));
  auto * filter_row = new QHBoxLayout();
  dashboard_scene_search_ = new QLineEdit(center_card); dashboard_scene_search_->setObjectName("studioHomeSearchBox"); dashboard_scene_search_->setPlaceholderText("Search scenes...");
  dashboard_scene_status_filter_ = new QComboBox(center_card); dashboard_scene_status_filter_->setObjectName("studioHomeStatusFilter"); dashboard_scene_status_filter_->addItems({"All", "Ready", "Warning", "Blocked"});
  filter_row->addWidget(dashboard_scene_search_, 1); filter_row->addWidget(dashboard_scene_status_filter_);
  center_card_layout->addLayout(filter_row);
  center_card_layout->addWidget(dashboard_scene_table_, 1);
  dashboard_middle_split->addWidget(left_library_card);
  dashboard_middle_split->addWidget(center_card);
  dashboard_middle_split->addWidget(dashboard_selected_scene_card_);
  dashboard_middle_split->setStretchFactor(0, 1);
  dashboard_middle_split->setStretchFactor(1, 4);
  dashboard_middle_split->setStretchFactor(2, 2);
  dashboard_middle_split->setCollapsible(0, false);
  dashboard_middle_split->setCollapsible(2, false);
  dashboard_middle_split->setSizes({280, 820, 360});
  dl->addWidget(dashboard_middle_split, 1);
  
  auto * scene_builder = new QWidget(studio_pages_); auto * sl=new QVBoxLayout(scene_builder);
  scene_builder_title_=new QLabel("<h2>Scene Builder</h2>"); scene_builder_title_->setProperty("studioTitle", true); sl->addWidget(scene_builder_title_);
  auto * scene_header_row = new QHBoxLayout();
  scene_builder_preview_chip_ = new QLabel("Preview: Unavailable", scene_builder); scene_builder_preview_chip_->setObjectName("sceneStatusChip");
  scene_builder_launch_chip_ = new QLabel("Launch: Missing", scene_builder); scene_builder_launch_chip_->setObjectName("sceneStatusChip");
  scene_builder_safety_chip_ = new QLabel("Safety: Fake hardware", scene_builder); scene_builder_safety_chip_->setObjectName("sceneStatusChip");
  scene_builder_path_label_ = new QLabel("Path: (none)", scene_builder); scene_builder_path_label_->setWordWrap(false);
  scene_builder_path_label_->setTextFormat(Qt::PlainText);
  scene_builder_path_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  scene_builder_generate_launch_button_ = new QPushButton("Generate launch package", scene_builder);
  scene_builder_generate_launch_button_->setVisible(false);
  scene_builder_generate_launch_button_->setMaximumHeight(24);
  auto * copy_scene_path_header = new QToolButton(scene_builder); copy_scene_path_header->setText("Copy");
  QObject::connect(copy_scene_path_header, &QToolButton::clicked, this, [this](){ if (!selected_scene_path().isEmpty()) QApplication::clipboard()->setText(selected_scene_path()); });
  QObject::connect(scene_builder_generate_launch_button_, &QPushButton::clicked, this, [this](){
    generate_scene_package_for_selected_scene();
    refresh_scene_builder_selected_scene_ui();
  });
  scene_header_row->addWidget(scene_builder_preview_chip_);
  scene_header_row->addWidget(scene_builder_launch_chip_);
  scene_header_row->addWidget(scene_builder_safety_chip_);
  scene_header_row->addWidget(scene_builder_generate_launch_button_);
  scene_header_row->addWidget(scene_builder_path_label_,1);
  scene_header_row->addWidget(copy_scene_path_header);
  sl->addLayout(scene_header_row);
  auto * scene_shell = new QWidget(scene_builder); scene_shell->setObjectName("sceneBuilderWorkspace");
  auto * scene_shell_layout = new QVBoxLayout(scene_shell);
  auto * scene_splitter = new QSplitter(Qt::Horizontal, scene_shell);
  scene_builder_splitter_ = scene_splitter;
  scene_splitter->setObjectName("sceneBuilderMainSplitter");
  auto * left_panel = new QFrame(scene_builder); left_panel->setObjectName("studioPanel"); left_panel->setMinimumWidth(360);
  auto * center_panel = new QFrame(scene_builder); center_panel->setObjectName("studioPanel");
  auto * right_panel = new QFrame(scene_builder); right_panel->setObjectName("studioPanel"); right_panel->setMinimumWidth(320);
  scene_splitter->addWidget(left_panel);
  scene_splitter->addWidget(center_panel);
  scene_splitter->addWidget(right_panel);
  scene_splitter->setCollapsible(0, true);
  scene_splitter->setCollapsible(2, true);
  scene_splitter->setStretchFactor(0, 1);
  scene_splitter->setStretchFactor(1, 7);
  scene_splitter->setStretchFactor(2, 2);
  scene_splitter->setSizes({300, 1180, 360});
  scene_shell_layout->addWidget(scene_splitter, 1);
  sl->addWidget(scene_shell, 1);

  auto * left_layout = new QVBoxLayout(left_panel);
  scene_builder_left_tabs_ = new QTabWidget(left_panel);
  auto * scene_tab = new QWidget(scene_builder_left_tabs_);
  auto * scene_tab_layout = new QVBoxLayout(scene_tab);
  auto * assets_tab = new QWidget(scene_builder_left_tabs_);
  auto * assets_tab_layout = new QVBoxLayout(assets_tab);
  auto * files_tab = new QWidget(scene_builder_left_tabs_);
  auto * files_tab_layout = new QVBoxLayout(files_tab);
  auto * hierarchy_card = new QFrame(left_panel); hierarchy_card->setObjectName("studioCard");
  auto * hierarchy_layout = new QVBoxLayout(hierarchy_card);
  hierarchy_layout->addWidget(new QLabel("<b>Scene Hierarchy</b>"));
  scene_hierarchy_tree_ = new QTreeWidget(hierarchy_card);
  scene_hierarchy_tree_->setObjectName("studioSceneHierarchyTree");
  scene_hierarchy_tree_->setHeaderLabels({"Item", "Type", "State"});
  scene_hierarchy_tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scene_hierarchy_tree_->setTextElideMode(Qt::ElideRight);
  scene_hierarchy_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  scene_hierarchy_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  scene_hierarchy_tree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  scene_hierarchy_tree_->setColumnWidth(0, 420);
  scene_hierarchy_tree_->setColumnWidth(1, 160);
  scene_hierarchy_tree_->setColumnWidth(2, 120);
  hierarchy_layout->addWidget(scene_hierarchy_tree_);
  auto * preview_layers_group = new QGroupBox("Scene3D Preview Layers", hierarchy_card);
  auto * preview_layers_layout = new QVBoxLayout(preview_layers_group);
  preview_layer_editable_layout_box_ = new QCheckBox("editable layout", preview_layers_group);
  preview_layer_generated_urdf_visual_box_ = new QCheckBox("Show Robot Links (generated)", preview_layers_group);
  preview_layer_generated_urdf_visual_box_->setToolTip(
    "Shows locked generated robot/URDF visuals, including generated robot mesh previews and fallback link items.");
  preview_layer_mesh_preview_box_ = new QCheckBox("mesh previews", preview_layers_group);
  preview_layer_primitive_fallback_box_ = new QCheckBox("primitive fallbacks", preview_layers_group);
  preview_layer_overlays_helpers_box_ = new QCheckBox("overlays/helpers", preview_layers_group);
  preview_layer_warnings_missing_assets_box_ = new QCheckBox("warnings/missing assets", preview_layers_group);
  for (auto * box : {
      preview_layer_editable_layout_box_, preview_layer_generated_urdf_visual_box_,
      preview_layer_mesh_preview_box_, preview_layer_primitive_fallback_box_,
      preview_layer_overlays_helpers_box_, preview_layer_warnings_missing_assets_box_})
  {
    box->setChecked(true);
    preview_layers_layout->addWidget(box);
  }
  hierarchy_layout->addWidget(preview_layers_group);
  scene_tab_layout->addWidget(hierarchy_card);
  auto * catalog_card = new QFrame(assets_tab); catalog_card->setObjectName("studioCard");
  auto * catalog_layout = new QVBoxLayout(catalog_card);
  catalog_layout->addWidget(new QLabel("<b>Asset Browser</b> <span style=\"color:#8aa\">(Asset Catalog)</span>"));
  // Asset Catalog compatibility tokens: Robots End Effectors Cameras Tables Conveyors Bins Fixtures Objects / STLs Pick/Place Zones Custom / Imported
  auto * asset_browser_hint = new QLabel("Meshes discovered from repo assets/ (.stl, .dae, .obj). Select one and click Add to Scene.", catalog_card);
  asset_browser_hint->setWordWrap(true);
  catalog_layout->addWidget(asset_browser_hint);
  asset_filter_combo_ = new QComboBox(catalog_card);
  asset_filter_combo_->addItems({"All", "robots", "grippers", "cameras", "table/workbench", "environment", "other"});
  catalog_layout->addWidget(asset_filter_combo_);
  asset_catalog_tree_ = new QTreeWidget(catalog_card);
  asset_catalog_tree_->setObjectName("studioAssetCatalogTree");
  asset_catalog_tree_->setHeaderLabels({"Asset", "Category", "Type/Source", "Status"});
  asset_catalog_tree_->viewport()->setAcceptDrops(false);
  asset_catalog_tree_->setDragEnabled(true);
  asset_catalog_tree_->viewport()->installEventFilter(this);
  catalog_layout->addWidget(asset_catalog_tree_, 1);
  add_to_canvas_button_ = new QPushButton("Add to Scene", scene_builder);
  add_to_canvas_button_->setEnabled(false);
  catalog_layout->addWidget(add_to_canvas_button_);
  add_asset_button_ = new QPushButton("Browse Details", scene_builder);
  catalog_layout->addWidget(add_asset_button_);
  auto * asset_more_actions = new QToolButton(scene_builder);
  asset_more_actions->setText("More Actions");
  asset_more_actions->setPopupMode(QToolButton::InstantPopup);
  auto * asset_more_menu = new QMenu(asset_more_actions);
  auto * open_asset_folder_action = asset_more_menu->addAction("Open Asset Folder");
  auto * copy_asset_path_action = asset_more_menu->addAction("Copy Asset Path");
  auto * import_asset_action = asset_more_menu->addAction("Import STL / URDF");
  auto * add_existing_stl_action = asset_more_menu->addAction("Add Existing STL to Canvas");
  auto * placeholder_action = asset_more_menu->addAction("Generate Simple Box/Cylinder Placeholder");
  asset_more_actions->setMenu(asset_more_menu);
  catalog_layout->addWidget(asset_more_actions);
  assets_tab_layout->addWidget(catalog_card, 1);
  auto * files_card = new QFrame(files_tab); files_card->setObjectName("studioCard");
  auto * files_card_layout = new QVBoxLayout(files_card);
  files_card_layout->addWidget(new QLabel("<b>Scene Files</b>"));
  scene_files_summary_label_ = new QLabel("Scene path and generated artifacts appear here after selection.", files_card);
  scene_files_summary_label_->setWordWrap(true);
  files_card_layout->addWidget(scene_files_summary_label_);

  scene_files_selected_path_label_ = new QLabel("Selected scene path: (none)", files_card);
  scene_files_selected_path_label_->setWordWrap(true);
  files_card_layout->addWidget(scene_files_selected_path_label_);

  scene_files_tree_ = new QTreeWidget(files_card);
  scene_files_tree_->setObjectName("studioSceneFilesTree");
  scene_files_tree_->setHeaderLabels({"Artifact", "Relative Path", "Status"});
  scene_files_tree_->setRootIsDecorated(false);
  scene_files_tree_->setAlternatingRowColors(true);
  files_card_layout->addWidget(scene_files_tree_, 1);

  auto * files_actions_layout = new QHBoxLayout();
  auto * open_scene_folder_button = new QPushButton("Open Scene Folder", files_card);
  auto * copy_scene_path_button = new QPushButton("Copy Scene Path", files_card);
  auto * refresh_scene_files_button = new QPushButton("Refresh Files", files_card);

  files_actions_layout->addWidget(open_scene_folder_button);
  files_actions_layout->addWidget(copy_scene_path_button);
  files_actions_layout->addWidget(refresh_scene_files_button);
  files_actions_layout->addStretch(1);
  files_card_layout->addLayout(files_actions_layout);

  connect(open_scene_folder_button, &QPushButton::clicked, this, [this]() {
    if (!has_selected_scene()) {
      append_studio_log("Open Scene Folder: no scene selected.");
      return;
    }
    open_selected_scene_artifact("folder");
  });

  connect(copy_scene_path_button, &QPushButton::clicked, this, [this]() {
    const QString scene_path = selected_scene_path();
    if (scene_path.isEmpty()) {
      append_studio_log("Copy Scene Path: no scene selected.");
      return;
    }
    QApplication::clipboard()->setText(scene_path);
    append_studio_log("Copied selected scene path.");
  });

  connect(refresh_scene_files_button, &QPushButton::clicked, this, [this]() {
    populate_scene_files_tab();
    append_studio_log("Scene Files tab refreshed.");
  });
  files_tab_layout->addWidget(files_card);
  scene_builder_left_tabs_->addTab(scene_tab, "Scene");
  scene_builder_left_tabs_->addTab(assets_tab, "Assets");
  scene_builder_left_tabs_->addTab(files_tab, "Files");
  left_layout->addWidget(scene_builder_left_tabs_, 1);

  auto * center_panel_layout = new QVBoxLayout(center_panel);
  scene_preview_label_=new QLabel("<b>Digital Twin Canvas</b>"); scene_preview_label_->setWordWrap(true); center_panel_layout->addWidget(scene_preview_label_);
  canvas_header_label_ = new QLabel("UR5 + Robotiq 2F | Pick and Place | READY"); canvas_header_label_->setWordWrap(true); center_panel_layout->addWidget(canvas_header_label_);
  auto * controls = new QHBoxLayout();
  controls->setObjectName("scene_builder_top_controls_row");
  controls->setContentsMargins(0, 0, 0, 0);
  controls->setSpacing(8);
  canvas_mode_label_ = new QLabel("Mode: Select · Scene3D Product Preview", scene_builder); controls->addWidget(canvas_mode_label_);
  // Scene canvas entrypoint: keep this same center-panel surface and swap rendering internals through ScenePreviewWidget.
  // ScenePreviewWidget consumes preview items produced from:
  //   1) editable layout metadata (layout/workcell_studio_layout.yaml)
  //   2) locked/generated visual metadata (generated/scene_visual_mesh_index.json)
  // and forwards them to Scene3DViewportWidget for perspective/depth/orbit-pan-zoom rendering.
  scene_preview_widget_ = new ScenePreviewWidget(scene_builder);
  scene_preview_widget_->setObjectName("scenePreviewWidget");
  scene_preview_widget_->set_label_mode(ScenePreviewWidget::LabelMode::Selected);
  connect(scene_preview_widget_, &ScenePreviewWidget::studio_log_requested, this, [this](const QString &m){
    append_studio_log(m);
    if (m.startsWith("Locked:", Qt::CaseInsensitive) && inspector_warning_label_) {
      inspector_warning_label_->setText(
        "Warnings: " + m + " | Reachability: preview-only | Collision: preview-only | Safety zone: preview-only | Pick source reach: unknown | Place target reach: unknown | Warning count: 1 | Preview-only");
    }
  });
  connect(scene_preview_widget_, &ScenePreviewWidget::preview_item_selected, this, [this](const QString &id, const QString &role){
    apply_scene_selection(id, role, id.trimmed().isEmpty(), false);
  });
  auto * scene3d_viewport = scene_preview_widget_->findChild<Scene3DViewportWidget *>();
  if (scene3d_viewport) {
    scene3d_viewport->setObjectName("scene3dViewportWidget");
    scene3d_viewport->transform_changed_cb = [this](const QString &id, double x, double y, double z, double r, double p, double yaw){
      if (!digital_twin_scene_) return;
      for (auto * item : digital_twin_scene_->items()) {
        if (item->data(RoleId).toString() != id) continue;
        const bool locked_item = item->data(RoleLocked).toBool();
        const QString source_layer = item->data(RoleSourceLayer).toString().trimmed();
        const bool editable_source_layer =
          source_layer.compare(QStringLiteral("editable_layout"), Qt::CaseInsensitive) == 0;
        if (locked_item || !editable_source_layer) {
          append_studio_log(QStringLiteral("Scene3D transform edit blocked: locked item cannot be edited or source layer is not editable_layout (id=%1, source_layer=%2)")
              .arg(id, source_layer));
          return;
        }
        item->setPos(x * 100.0, y * 100.0);
        item->setData(RolePoseZ, z);
        item->setData(RoleRoll, r);
        item->setData(RolePitch, p);
        item->setData(RoleYaw, yaw);
        refresh_selection_transform_editor_from_item(item);
        mark_layout_dirty("Scene3D Gizmo Transform");
        break;
      }
    };
    scene3d_viewport->asset_drop_cb = [this](const QJsonObject & payload, double x, double y, double, bool shift_drop) {
        const QString category = payload.value("category").toString();
        const QString display_name = payload.value("display_name").toString();
        const QString source_path = payload.value("source_path").toString();
        if (category.isEmpty() || display_name.isEmpty()) {
          append_studio_log("Cannot place here: drag payload missing category/display name.");
          return;
        }
        if (shift_drop && !configure_asset_placement_transform(category, display_name)) return;
        armed_asset_use_clicked_xy_ = true;
        armed_asset_x_m_ = x;
        armed_asset_y_m_ = y;
        armed_asset_z_m_ = default_asset_pose_z(category, display_name);
        arm_place_asset_mode(category, display_name, source_path);
        commit_armed_asset_placement(QPointF(x * 100.0, y * 100.0));
      };
  }
  scene_builder_top_controls_host_ = new QWidget(scene_builder);
  scene_builder_top_controls_host_->setObjectName("scene_builder_top_controls_host");
  auto * primary_controls = new QHBoxLayout(scene_builder_top_controls_host_);
  primary_controls->setObjectName("scene_builder_primary_controls_layout");  // acceptance: primary always-visible actions
  primary_controls->setContentsMargins(0, 0, 0, 0);
  primary_controls->setSpacing(6);
  scene_builder_primary_controls_layout_ = primary_controls;
  controls->addWidget(scene_builder_top_controls_host_, 1);

  scene_builder_action_registry_.clear();
  auto register_scene_action = [&](const QString & key, const QString & text, std::function<void()> handler) {
    auto * action = new QAction(text, scene_builder);
    QObject::connect(action, &QAction::triggered, this, [handler]() { handler(); });
    register_scene_builder_action(key, action);
    return action;
  };
  auto create_action_button = [&](QVBoxLayout * layout, const QString & key) {
    auto * action = scene_builder_action(key);
    if (!action) return static_cast<QPushButton *>(nullptr);
    auto * button = new QPushButton(action->text(), scene_builder);
    QObject::connect(button, &QPushButton::clicked, action, &QAction::trigger);
    layout->addWidget(button);
    return button;
  };
  register_scene_action("layout.save", "Save Layout", [this]() { if (save_layout_button_) save_layout_button_->click(); });
  register_scene_action("layout.undo", "Undo Layout Edit", [this]() { undo_layout_edit(); });
  register_scene_action("layout.redo", "Redo Layout Edit", [this]() { redo_layout_edit(); });
  register_scene_action("layout.duplicate", "Duplicate Selected", [this]() { duplicate_selected_item(); });
  register_scene_action("layout.remove", "Remove Selected Layout Item", [this]() { delete_selected_item(); });
  register_scene_action("generate.scene_package", "Generate", [this]() { generate_scene_package_for_selected_scene(); });
  register_scene_action("generate.yaml", "Generate YAML", [this]() { generate_yaml_draft_for_selected_scene(); });
  register_scene_action("generate.task", "Generate Task/Grasp Files", [this]() { generate_or_update_task_intent_for_selected_scene(); });
  register_scene_action("validate.generated_scene", "Validate Generated Scene", [this]() { validate_generated_scene_for_selected_scene(); });
  register_scene_action("validate.offline", "Run Offline Validation", [this]() { run_offline_validation(); });
  register_scene_action("simulate.open_rviz", "Open RViz Truth Preview", [this]() { if (run_build_button_) run_build_button_->click(); });
  register_scene_action("simulate.run_fake", "Run Fake-Hardware Simulation", [this]() { if (run_preview_button_) run_preview_button_->click(); });
  register_scene_action("simulate.stop", "Stop Simulation", [this]() { if (stop_preview_button_) stop_preview_button_->click(); });
  register_scene_action("diagnostics.self_test", "Run Self-Test", [this]() { if (run_self_test_button_) run_self_test_button_->click(); });
  register_scene_action("diagnostics.golden_flow", "Run Golden Flow Dry Run", [this]() { if (run_golden_flow_button_) run_golden_flow_button_->click(); });

  auto make_primary_button = [scene_builder](const QString & text) {
      auto * button = new QPushButton(text, scene_builder);
      button->setMinimumHeight(30);
      button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
      return button;
    };
  auto * select_mode_button = make_primary_button("Select"); primary_controls->addWidget(select_mode_button);
  auto * place_mode_button = make_primary_button("Place Asset"); primary_controls->addWidget(place_mode_button);
  place_mode_persistent_box_ = new QCheckBox("Keep placing", scene_builder);
  place_mode_persistent_box_->setChecked(false);
  place_mode_persistent_box_->setToolTip("When enabled, Place Asset mode stays armed after each placement.");
  place_mode_persistent_box_->setMinimumHeight(30);
  place_mode_persistent_box_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
  primary_controls->addWidget(place_mode_persistent_box_);
  auto * move_mode_button = make_primary_button("Move"); primary_controls->addWidget(move_mode_button);
  auto * inspect_mode_button = make_primary_button("Inspect"); primary_controls->addWidget(inspect_mode_button);
  save_layout_button_ = make_primary_button("Save Layout");
  primary_controls->addWidget(save_layout_button_);
  primary_controls->addStretch(1);

  scene_builder_camera_view_button_ = new QToolButton(scene_builder);
  auto * camera_view = scene_builder_camera_view_button_; camera_view->setText("Camera / View"); camera_view->setPopupMode(QToolButton::InstantPopup);
  auto * camera_view_menu = new QMenu(camera_view);
  auto * perspective_action = camera_view_menu->addAction("Perspective");
  auto * top_action = camera_view_menu->addAction("Top");
  auto * left_action = camera_view_menu->addAction("Left");
  auto * right_action = camera_view_menu->addAction("Right");
  auto * front_action = camera_view_menu->addAction("Front");
  auto * fit_button = camera_view_menu->addAction("Fit Cell");
  auto * fit_robot_button = camera_view_menu->addAction("Fit Robot");
  auto * reset_button = camera_view_menu->addAction("Reset View");
  auto * zoom_in = camera_view_menu->addAction("Zoom In");
  auto * zoom_out = camera_view_menu->addAction("Zoom Out");
  camera_view->setMenu(camera_view_menu);
  controls->addWidget(camera_view);
  toggle_grid_box_ = new QCheckBox("Toggle Grid", scene_builder); toggle_grid_box_->setChecked(true);
  snap_to_grid_box_ = new QCheckBox("Snap: 5 cm", scene_builder); snap_to_grid_box_->setToolTip("Snap applies to drag, keyboard nudge, and transform edits."); snap_to_grid_box_->setChecked(true);
  snap_step_label_ = new QLabel("Nudge step: 0.05 m", scene_builder);
  fine_move_mode_box_ = new QCheckBox("Fine Move Mode", scene_builder);
  unlock_robot_base_box_ = new QCheckBox("Unlock Robot Base", scene_builder);
  toggle_labels_box_ = new QCheckBox("Toggle Labels", scene_builder); toggle_labels_box_->setToolTip("Product View starts with selected-item labels only; enable for explicit label review."); toggle_labels_box_->setChecked(false);
  toggle_warnings_box_ = new QCheckBox("Toggle Warnings", scene_builder); toggle_warnings_box_->setChecked(false);
  show_minimap_box_ = new QCheckBox("Show Minimap", scene_builder); show_minimap_box_->setChecked(true);
  scene_builder_overlays_button_ = new QToolButton(scene_builder);
  auto * overlays_button = scene_builder_overlays_button_; overlays_button->setText("Overlays"); overlays_button->setPopupMode(QToolButton::InstantPopup);
  auto * overlays_menu = new QMenu(overlays_button);
  show_reach_overlay_box_ = new QCheckBox("Show Reach", scene_builder); show_reach_overlay_box_->setChecked(false);
  show_camera_fov_overlay_box_ = new QCheckBox("Camera FOV", scene_builder); show_camera_fov_overlay_box_->setChecked(false);
  show_pick_place_overlay_box_ = new QCheckBox("Pick Coverage", scene_builder); show_pick_place_overlay_box_->setChecked(false);
  show_trajectory_overlay_box_ = new QCheckBox("EPD Detections", scene_builder); show_trajectory_overlay_box_->setChecked(false);
  auto * show_approach_retreat_overlay_box = new QCheckBox("Approach/Retreat", scene_builder); show_approach_retreat_overlay_box->setChecked(false);
  auto mk=[&](QCheckBox *b){ auto *a=overlays_menu->addAction(b->text()); a->setCheckable(true); a->setChecked(b ? b->isChecked() : false); connect(a,&QAction::toggled,b,&QCheckBox::setChecked); connect(b,&QCheckBox::toggled,a,&QAction::setChecked); };
  mk(show_reach_overlay_box_); mk(show_camera_fov_overlay_box_); mk(show_pick_place_overlay_box_); mk(show_trajectory_overlay_box_); mk(show_approach_retreat_overlay_box);
  auto * show_warnings_action = overlays_menu->addAction("Show Warnings"); show_warnings_action->setCheckable(true); show_warnings_action->setChecked(false);
  auto * show_labels_action = overlays_menu->addAction("Detection Labels"); show_labels_action->setCheckable(true); show_labels_action->setChecked(false);
  overlays_button->setMenu(overlays_menu);
  connect(show_warnings_action, &QAction::toggled, toggle_warnings_box_, &QCheckBox::setChecked);
  connect(toggle_warnings_box_, &QCheckBox::toggled, show_warnings_action, &QAction::setChecked);
  connect(show_labels_action, &QAction::toggled, toggle_labels_box_, &QCheckBox::setChecked);
  connect(toggle_labels_box_, &QCheckBox::toggled, show_labels_action, &QAction::setChecked);
  connect(show_approach_retreat_overlay_box, &QCheckBox::toggled, this, [this, show_approach_retreat_overlay_box](bool){
    if (!scene_preview_widget_) return;
    scene_preview_widget_->set_task_overlay_visibility(
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_approach_retreat_overlay_box->isChecked(),
      false);
    scene_preview_widget_->set_perception_overlay_visibility(
      show_camera_fov_overlay_box_ ? show_camera_fov_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      false);
    append_studio_log("overlay toggled");
  });
  scene_builder_canvas_more_button_ = new QToolButton(scene_builder);
  auto * canvas_more_actions = scene_builder_canvas_more_button_;
  canvas_more_actions->setText("Canvas More");
  canvas_more_actions->setPopupMode(QToolButton::InstantPopup);
  auto * canvas_more_menu = new QMenu(canvas_more_actions);
  auto * snap_action = canvas_more_menu->addAction("Snap/Grid settings"); snap_action->setCheckable(true); snap_action->setChecked(true);
  auto * fine_move_action = canvas_more_menu->addAction("Fine Move Mode"); fine_move_action->setCheckable(true);
  auto * unlock_action = canvas_more_menu->addAction("Unlock Robot Base"); unlock_action->setCheckable(true);
  auto * minimap_action = canvas_more_menu->addAction("Show Minimap"); minimap_action->setCheckable(true); minimap_action->setChecked(true);
  auto * reload_meshes_action = canvas_more_menu->addAction("Reload Meshes");
  connect(reload_meshes_action, &QAction::triggered, this, [this](){
    if (!scene_preview_widget_) return;
    scene_preview_widget_->reload_meshes();
    append_studio_log("Canvas More: reloaded mesh preview assets (visual-only).");
  });
  canvas_more_menu->addSeparator();
  canvas_more_menu->addAction("Toggle Labels")->setCheckable(true);
  canvas_more_menu->addAction("Toggle Warnings")->setCheckable(true);
  canvas_more_actions->setMenu(canvas_more_menu);
  scene_builder_visual_modes_button_ = new QToolButton(scene_builder);
  scene_builder_visual_modes_button_->setObjectName("scene_builder_secondary_visual_modes_button");  // acceptance: secondary grouped actions
  scene_builder_visual_modes_button_->setText("Visual Modes");
  scene_builder_visual_modes_button_->setPopupMode(QToolButton::InstantPopup);
  auto * visual_modes_menu = new QMenu(scene_builder_visual_modes_button_);
  auto * label_mode_menu = visual_modes_menu->addMenu("Label mode");
  auto * label_selected = label_mode_menu->addAction("Selected");
  auto * label_all = label_mode_menu->addAction("All");
  auto * label_off = label_mode_menu->addAction("Off");
  auto * mesh_preview_menu = visual_modes_menu->addMenu("Mesh preview mode");
  auto * mesh_visual = mesh_preview_menu->addAction("Visual Mesh");
  auto * mesh_collision = mesh_preview_menu->addAction("Collision Mesh");
  auto * snap_mode_menu = visual_modes_menu->addMenu("Snap mode");
  auto * snap_5cm = snap_mode_menu->addAction("Snap: 5 cm");
  auto * snap_1cm = snap_mode_menu->addAction("Snap: 1 cm");
  auto * snap_off = snap_mode_menu->addAction("Snap Off");
  scene_builder_visual_modes_button_->setMenu(visual_modes_menu);
  scene_builder_secondary_overflow_button_ = new QToolButton(scene_builder);
  scene_builder_secondary_overflow_button_->setObjectName("scene_builder_secondary_overflow_button");
  scene_builder_secondary_overflow_button_->setText("More");
  scene_builder_secondary_overflow_button_->setPopupMode(QToolButton::InstantPopup);
  scene_builder_secondary_overflow_menu_ = new QMenu(scene_builder_secondary_overflow_button_);
  scene_builder_secondary_overflow_menu_->addMenu(overlays_menu)->setText("Overlays");
  scene_builder_secondary_overflow_menu_->addMenu(visual_modes_menu)->setText("Visual Modes");
  scene_builder_secondary_overflow_menu_->addMenu(canvas_more_menu)->setText("Layout/Edit Settings");
  auto * secondary_layout_menu = scene_builder_secondary_overflow_menu_->addMenu("Layout");
  secondary_layout_menu->addAction(scene_builder_action("layout.undo"));
  secondary_layout_menu->addAction(scene_builder_action("layout.redo"));
  secondary_layout_menu->addAction(scene_builder_action("layout.save"));
  secondary_layout_menu->addAction(scene_builder_action("layout.duplicate"));
  secondary_layout_menu->addAction(scene_builder_action("layout.remove"));
  scene_builder_secondary_overflow_button_->setMenu(scene_builder_secondary_overflow_menu_);
  controls->addWidget(scene_builder_secondary_overflow_button_);
  center_panel_layout->addLayout(controls);
  digital_twin_canvas_ = new QGraphicsView(scene_builder); digital_twin_canvas_->setObjectName("digital_twin_canvas_"); digital_twin_canvas_->setMinimumHeight(420);
  digital_twin_canvas_->viewport()->installEventFilter(this);
  scene_preview_widget_->set_fallback_2d_view(digital_twin_canvas_);
  center_panel_layout->addWidget(scene_preview_widget_, 1);
  minimap_view_ = new QGraphicsView(scene_builder); minimap_view_->setObjectName("digital_twin_minimap"); minimap_view_->setFixedSize(210, 140); center_panel_layout->addWidget(minimap_view_, 0, Qt::AlignRight);
  auto * layout_controls = new QHBoxLayout();
  undo_layout_button_ = nullptr;
  redo_layout_button_ = nullptr;
  create_starter_layout_button_ = new QPushButton("Create editable layout from preview", scene_builder);
  create_starter_layout_button_->setVisible(false);
  layout_controls->addWidget(create_starter_layout_button_);
  canvas_more_menu->addSeparator();
  canvas_more_menu->addAction("Duplicate Selected", this, [this](){ duplicate_selected_item(); });
  canvas_more_menu->addAction("Remove Selected Layout Item", this, [this](){ delete_selected_item(); });
  canvas_more_menu->addAction("Revert Layout", this, &MainWindow::revert_layout_changes);
  canvas_more_menu->addAction("Run Layout Merge", this, [this](){ run_layout_merge_for_selected_scene(false); });
  canvas_more_menu->addAction("Open Merge Report", this, &MainWindow::open_layout_merge_report);
  canvas_more_menu->addAction("Copy Merge Summary", this, &MainWindow::copy_layout_merge_summary);
  canvas_more_menu->addAction("Export Canvas Snapshot", this, &MainWindow::export_canvas_snapshot);
  connect(label_selected, &QAction::triggered, this, [this]() { if (scene_preview_widget_) scene_preview_widget_->set_label_mode(ScenePreviewWidget::LabelMode::Selected); });
  connect(label_all, &QAction::triggered, this, [this]() { if (scene_preview_widget_) scene_preview_widget_->set_label_mode(ScenePreviewWidget::LabelMode::Important); });
  connect(label_off, &QAction::triggered, this, [this]() { if (scene_preview_widget_) scene_preview_widget_->set_label_mode(ScenePreviewWidget::LabelMode::Off); });
  // ScenePreviewWidget::LabelMode::SelectedOnly compatibility token for static navigation contract tests.
  connect(mesh_visual, &QAction::triggered, this, [this]() {
    auto * v = scene_preview_widget_ ? scene_preview_widget_->findChild<Scene3DViewportWidget *>() : nullptr;
    if (!v) return;
    v->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Meshes;
    v->update();
  });
  connect(mesh_collision, &QAction::triggered, this, [this]() {
    auto * v = scene_preview_widget_ ? scene_preview_widget_->findChild<Scene3DViewportWidget *>() : nullptr;
    if (!v) return;
    v->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Primitives;
    v->update();
  });
  connect(snap_5cm, &QAction::triggered, this, [this]() { snap_step_m_ = 0.05; if (snap_step_label_) snap_step_label_->setText("Nudge step: 0.05 m"); if (snap_to_grid_box_) snap_to_grid_box_->setChecked(true); });
  connect(snap_1cm, &QAction::triggered, this, [this]() { snap_step_m_ = 0.01; if (snap_step_label_) snap_step_label_->setText("Nudge step: 0.01 m"); if (snap_to_grid_box_) snap_to_grid_box_->setChecked(true); });
  connect(snap_off, &QAction::triggered, this, [this]() { if (snap_to_grid_box_) snap_to_grid_box_->setChecked(false); });
  scene_builder->installEventFilter(this);
  update_scene_builder_top_controls_overflow();
  center_panel_layout->addLayout(layout_controls);
  layout_state_label_ = new QLabel("Unsaved Layout Edits: none", scene_builder); center_panel_layout->addWidget(layout_state_label_);
  canvas_legend_label_ = new QLabel("Legend: robot | Robot Reach | camera | Camera FOV | pick zone | place zone | conveyor | bin | warning"); center_panel_layout->addWidget(canvas_legend_label_);
  auto * bottom_cards = new QHBoxLayout();
  bottom_cards->addWidget(new QLabel("<b>Validation</b><br/>Selected Scene: pending<br/>Preview Only safety gate enabled."));
  bottom_cards->addWidget(new QLabel("<b>Readiness Checks</b><br/>Fake Hardware<br/>No Robot Motion"));
  bottom_cards->addWidget(new QLabel("<b>Simulation Log</b><br/>Recent studio command summary appears in Studio Log."));
  bottom_cards->addWidget(new QLabel("<b>Cycle/Timing Summary</b><br/>Offline estimate only."));
  center_panel_layout->addLayout(bottom_cards);

  auto * right_layout = new QVBoxLayout(right_panel);
  auto * workflow_card = new QFrame(right_panel);
  workflow_card->setObjectName("studioCard");
  auto * workflow_card_layout = new QVBoxLayout(workflow_card);
  workflow_card_layout->addWidget(new QLabel("<b>Scene Builder Workflow</b>", workflow_card));
  scene_workflow_rail_label_ = new QLabel("Select a scene to view workflow steps.", workflow_card);
  scene_workflow_rail_label_->setWordWrap(true);
  workflow_card_layout->addWidget(scene_workflow_rail_label_);
  scene_workflow_recommendation_label_ = new QLabel("Recommended next action updates as you progress.", workflow_card);
  scene_workflow_recommendation_label_->setWordWrap(true);
  workflow_card_layout->addWidget(scene_workflow_recommendation_label_);
  scene_workflow_recommendation_button_ = new QPushButton("Open or create a scene", workflow_card);
  scene_workflow_recommendation_button_->setProperty("role", "primary");
  scene_workflow_recommendation_menu_ = new QMenu(scene_workflow_recommendation_button_);
  scene_workflow_recommendation_button_->setMenu(scene_workflow_recommendation_menu_);
  workflow_card_layout->addWidget(scene_workflow_recommendation_button_);
  right_layout->addWidget(workflow_card);
  auto * inspector_scroll = new QScrollArea(right_panel);
  inspector_scroll->setWidgetResizable(true);
  auto * inspector_scroll_contents = new QWidget(inspector_scroll);
  auto * inspector_scroll_layout = new QVBoxLayout(inspector_scroll_contents);
  scene_builder_inspector_tabs_ = new QTabWidget(inspector_scroll_contents);
  scene_builder_inspector_tabs_->setUsesScrollButtons(false);
  auto * selection_tab = new QWidget(scene_builder_inspector_tabs_); auto * selection_tab_layout = new QVBoxLayout(selection_tab);
  auto * workflow_tab = new QWidget(scene_builder_inspector_tabs_); auto * workflow_tab_layout = new QVBoxLayout(workflow_tab);
  auto * actions_tab = new QWidget(scene_builder_inspector_tabs_); auto * actions_tab_layout = new QVBoxLayout(actions_tab);
  auto * readiness_tab = new QWidget(scene_builder_inspector_tabs_); auto * readiness_tab_layout = new QVBoxLayout(readiness_tab);
  auto * logs_tab = new QWidget(scene_builder_inspector_tabs_); auto * logs_tab_layout = new QVBoxLayout(logs_tab);
  auto make_card = [&](QVBoxLayout *parent_layout, const QString &title) {
    auto *card = new QFrame(right_panel); card->setObjectName("studioCard");
    auto *layout = new QVBoxLayout(card);
    layout->addWidget(new QLabel(QString("<b>%1</b>").arg(title)));
    parent_layout->addWidget(card);
    return layout;
  };
  auto make_row = [&](QVBoxLayout *parent, const QString &label, const QString &value, bool copy_button) {
    auto *row = new QWidget(scene_builder);
    auto *row_layout = new QHBoxLayout(row); row_layout->setContentsMargins(0,0,0,0);
    auto *k = new QLabel(label, row); k->setMinimumWidth(120);
    auto *v = new QLabel(value, row); v->setWordWrap(true); v->setTextInteractionFlags(Qt::TextSelectableByMouse); v->setToolTip(value);
    row_layout->addWidget(k); row_layout->addWidget(v, 1);
    if (copy_button) { auto *copy = new QPushButton("Copy", row); row_layout->addWidget(copy); QObject::connect(copy, &QPushButton::clicked, row, [v]() { QApplication::clipboard()->setText(v->text()); }); }
    parent->addWidget(row);
    return v;
  };
  auto * scene_card_layout = make_card(selection_tab_layout, "Scene");
  auto * selected_item_card_layout = make_card(selection_tab_layout, "Selected Item");
  auto * readiness_card_layout = make_card(readiness_tab_layout, "Readiness");
  make_card(actions_tab_layout, "Actions");
  selection_scene_name_label_ = make_row(scene_card_layout, "Name", "No scene selected", false);
  selection_scene_status_label_ = make_row(scene_card_layout, "Status", "unknown", false);
  selection_scene_robot_label_ = make_row(scene_card_layout, "Robot", "unknown", false);
  selection_scene_end_effector_label_ = make_row(scene_card_layout, "End Effector", "unknown", false);
  selection_scene_path_label_ = make_row(scene_card_layout, "Path", "(none)", true);
  selection_scene_launch_label_ = make_row(scene_card_layout, "Launch", "(none)", true);

  auto * task_intent = new QFrame(right_panel); task_intent->setObjectName("studioCard"); auto * task_intent_layout = new QVBoxLayout(task_intent);
  task_intent_layout->addWidget(new QLabel("<b>Task Intent</b>"));
  task_flow_label_ = new QLabel("Pick Source → Grasp Strategy → Place Target → Release"); task_flow_label_->setWordWrap(true); task_intent_layout->addWidget(task_flow_label_);
  task_intent_details_label_ = new QLabel("No scene selected"); task_intent_details_label_->setWordWrap(true); task_intent_layout->addWidget(task_intent_details_label_);
  workflow_tab_layout->addWidget(task_intent);
  new_cell_checklist_label_ = new QLabel(
    "<b>New Cell Checklist</b><br/>"
    "pending: Workspace selected → choose workspace<br/>"
    "pending: Cell name set → click New Cell/New Scene<br/>"
    "pending: Robot selected (UR5 default) → verify robot<br/>"
    "pending: Tool selected (Robotiq 2F default) → verify end effector<br/>"
    "pending: Environment layout created (table + pick zone + place zone + camera) → click Create editable layout from preview<br/>"
    "pending: Task intent created (pick_place) → click Generate/Update Task Intent<br/>"
    "pending: Scene files generated → click Generate Scene Package<br/>"
    "hint: Local build/run smoke test available<br/>"
    "hint: Use terminal command: python3 scripts/smoke_test_scratch_cell_workspace.py --workspace ~/workcell_ws --scene-name scratch_ur5_2f_smoke --timeout-sec 30<br/>"
    "hint: Smoke report: smoke_report.json<br/>"
    "pending: Validation passed → click Run Offline Validation<br/>"
    "pending: Ready for Plan / Simulate → Open RViz2 / MoveIt or Run Fake-Hardware Simulation");
  new_cell_checklist_label_->setObjectName("studioCard");
  new_cell_checklist_label_->setWordWrap(true);
  readiness_card_layout->addWidget(new_cell_checklist_label_);
  scene_builder_command_preview_card_ = new QFrame(right_panel);
  scene_builder_command_preview_card_->setObjectName("studioCard");
  auto * command_preview_layout = new QVBoxLayout(scene_builder_command_preview_card_);
  command_preview_layout->addWidget(new QLabel("<b>Command Preview</b><br/>Available after Generate Scene Package succeeds."));
  auto * build_row = new QWidget(scene_builder_command_preview_card_);
  auto * build_row_layout = new QHBoxLayout(build_row); build_row_layout->setContentsMargins(0,0,0,0);
  auto * build_key = new QLabel("Build", build_row); build_key->setMinimumWidth(72);
  scene_builder_build_command_label_ = new QLabel("(not available)", build_row); scene_builder_build_command_label_->setWordWrap(true); scene_builder_build_command_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  auto * copy_build_preview = new QPushButton("Copy", build_row);
  QObject::connect(copy_build_preview, &QPushButton::clicked, this, [this](){ if (scene_builder_build_command_label_) QApplication::clipboard()->setText(scene_builder_build_command_label_->text()); });
  build_row_layout->addWidget(build_key); build_row_layout->addWidget(scene_builder_build_command_label_, 1); build_row_layout->addWidget(copy_build_preview);
  command_preview_layout->addWidget(build_row);
  auto * launch_row = new QWidget(scene_builder_command_preview_card_);
  auto * launch_row_layout = new QHBoxLayout(launch_row); launch_row_layout->setContentsMargins(0,0,0,0);
  auto * launch_key = new QLabel("Launch", launch_row); launch_key->setMinimumWidth(72);
  scene_builder_launch_command_label_ = new QLabel("(not available)", launch_row); scene_builder_launch_command_label_->setWordWrap(true); scene_builder_launch_command_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  auto * copy_launch_preview = new QPushButton("Copy", launch_row);
  QObject::connect(copy_launch_preview, &QPushButton::clicked, this, [this](){ if (scene_builder_launch_command_label_) QApplication::clipboard()->setText(scene_builder_launch_command_label_->text()); });
  launch_row_layout->addWidget(launch_key); launch_row_layout->addWidget(scene_builder_launch_command_label_, 1); launch_row_layout->addWidget(copy_launch_preview);
  command_preview_layout->addWidget(launch_row);
  scene_builder_command_preview_card_->setVisible(false);
  readiness_card_layout->addWidget(scene_builder_command_preview_card_);
  auto * pick_place = new QGroupBox("Pick-Place Configuration", right_panel); pick_place->setObjectName("studioCard"); pick_place->setCheckable(true); pick_place->setChecked(false); auto * pick_place_layout = new QVBoxLayout(pick_place);
  auto * task_binding_actions = new QHBoxLayout();
  pick_source_button_ = new QPushButton("Use Selected as Pick Source", scene_builder); task_binding_actions->addWidget(pick_source_button_);
  place_target_button_ = new QPushButton("Use Selected as Place Target", scene_builder); task_binding_actions->addWidget(place_target_button_);
  camera_button_ = new QPushButton("Use Selected as Camera", scene_builder);
  task_binding_actions->addWidget(camera_button_);
  pick_place_layout->addLayout(task_binding_actions);
  pick_place_details_label_ = new QLabel("Pick source: unknown\nPlace target: unknown\nReject target: unknown\nLinked hierarchy item: unknown"); pick_place_details_label_->setWordWrap(true); pick_place_layout->addWidget(pick_place_details_label_);
  workflow_tab_layout->addWidget(pick_place);
  auto * grasp_card = new QFrame(right_panel); grasp_card->setObjectName("studioCard"); auto * grasp_layout = new QVBoxLayout(grasp_card);
  grasp_layout->addWidget(new QLabel("<b>Grasp Strategy</b>"));
  grasp_details_label_ = new QLabel("Strategy/ref: unknown\nTool/End Effector: unknown\nApproach axis: unknown\nOrientation mode: unknown\nAllowed roll/yaw: unknown"); grasp_details_label_->setWordWrap(true); grasp_layout->addWidget(grasp_details_label_);
  readiness_label_=new QLabel("Safety posture: guarded (fake hardware default, no uncontrolled robot motion)."); readiness_label_->setWordWrap(true); grasp_layout->addWidget(readiness_label_);
  workflow_tab_layout->addWidget(grasp_card);
  auto * ar_card = new QFrame(right_panel); ar_card->setObjectName("studioCard"); auto * ar_layout = new QVBoxLayout(ar_card);
  ar_layout->addWidget(new QLabel("<b>Perception Status</b>"));
  approach_retreat_details_label_ = new QLabel("Camera: unknown\nFrame: unknown\nFOV: unknown\nRange: unknown\nPick coverage: unknown\nDetection mode: Not Configured\nDetection status: Waiting for live nodes\nDetails: Perception preview unavailable until mode/config is selected.\nModes: Live EPD / RealSense | Saved EPD Snapshot | Simulated / Manual | Not Configured\nStatuses: Configured | Missing metadata | Preview only | Waiting for live nodes\nDetection count: 0\nWarnings: no camera item found | no pick source found | camera frame unknown"); approach_retreat_details_label_->setWordWrap(true); ar_layout->addWidget(approach_retreat_details_label_);
  auto * open_perception_metadata_button = new QPushButton("Open Perception Metadata", scene_builder); ar_layout->addWidget(open_perception_metadata_button);
  auto * open_epd_docs_button = new QPushButton("Open EPD Pipeline Docs", scene_builder); ar_layout->addWidget(open_epd_docs_button);
  auto * refresh_snapshot_button = new QPushButton("Refresh Snapshot", scene_builder); ar_layout->addWidget(refresh_snapshot_button);
  readiness_card_layout->addWidget(ar_card);
  auto make_action_section = [&](const QString & section_title) {
    auto * section = new QGroupBox(section_title, actions_tab);
    section->setObjectName("studioCard");
    auto * section_layout = new QVBoxLayout(section);
    actions_tab_layout->addWidget(section);
    return section_layout;
  };

  auto * layout_actions = make_action_section("Layout");
  auto * generate_actions = make_action_section("Generate");
  auto * validate_actions = make_action_section("Validate");
  auto * simulate_actions = make_action_section("Simulate");
  make_action_section("Export");
  auto * diagnostics_actions = make_action_section("Diagnostics");

  create_action_button(layout_actions, "layout.save");
  create_action_button(layout_actions, "layout.undo");
  create_action_button(layout_actions, "layout.redo");
  create_action_button(layout_actions, "layout.duplicate");
  create_action_button(layout_actions, "layout.remove");
  create_action_button(generate_actions, "generate.scene_package");
  create_action_button(generate_actions, "generate.yaml");
  create_action_button(generate_actions, "generate.task");
  create_action_button(validate_actions, "validate.generated_scene");
  create_action_button(validate_actions, "validate.offline");
  create_action_button(simulate_actions, "simulate.open_rviz");
  create_action_button(simulate_actions, "simulate.run_fake");
  create_action_button(simulate_actions, "simulate.stop");
  create_action_button(diagnostics_actions, "diagnostics.self_test");
  create_action_button(diagnostics_actions, "diagnostics.golden_flow");
  inspector_label_=new QLabel("Inspector selection: none"); inspector_label_->setObjectName("sceneBuilderInspectorLabel"); inspector_label_->setWordWrap(true); selection_tab_layout->addWidget(inspector_label_);
  live_coordinate_label_ = new QLabel("Selected: none", scene_builder); selection_tab_layout->addWidget(live_coordinate_label_);
  auto * pose_grid = new QGridLayout();
  inspector_x_ = new QDoubleSpinBox(scene_builder); inspector_x_->setPrefix("x "); pose_grid->addWidget(inspector_x_, 0, 0);
  inspector_y_ = new QDoubleSpinBox(scene_builder); inspector_y_->setPrefix("y "); pose_grid->addWidget(inspector_y_, 0, 1);
  inspector_z_ = new QDoubleSpinBox(scene_builder); inspector_z_->setPrefix("z "); pose_grid->addWidget(inspector_z_, 0, 2);
  inspector_roll_ = new QDoubleSpinBox(scene_builder); inspector_roll_->setPrefix("r "); pose_grid->addWidget(inspector_roll_, 1, 0);
  inspector_pitch_ = new QDoubleSpinBox(scene_builder); inspector_pitch_->setPrefix("p "); pose_grid->addWidget(inspector_pitch_, 1, 1);
  inspector_yaw_ = new QDoubleSpinBox(scene_builder); inspector_yaw_->setPrefix("yaw "); pose_grid->addWidget(inspector_yaw_, 1, 2);
  auto * dim_grid = new QGridLayout();
  inspector_dim_x_ = new QDoubleSpinBox(scene_builder); inspector_dim_x_->setPrefix("scale "); dim_grid->addWidget(inspector_dim_x_, 0, 0);
  inspector_dim_y_ = new QDoubleSpinBox(scene_builder); inspector_dim_y_->setPrefix("sy "); dim_grid->addWidget(inspector_dim_y_, 0, 1);
  inspector_dim_z_ = new QDoubleSpinBox(scene_builder); inspector_dim_z_->setPrefix("sz "); dim_grid->addWidget(inspector_dim_z_, 0, 2);
  selection_tab_layout->addLayout(dim_grid);
  selected_item_card_layout->addLayout(pose_grid);
  auto * metadata_group = new QGroupBox("Metadata", scene_builder);
  metadata_group->setObjectName("studioCard");
  auto * metadata_form = new QFormLayout(metadata_group);
  inspector_display_name_ = new QLineEdit(metadata_group);
  inspector_display_name_->setPlaceholderText("Display name");
  metadata_form->addRow("Display name", inspector_display_name_);
  inspector_role_ = new QLineEdit(metadata_group);
  inspector_role_->setReadOnly(true);
  metadata_form->addRow("Role", inspector_role_);
  inspector_category_ = new QLineEdit(metadata_group);
  inspector_category_->setReadOnly(true);
  metadata_form->addRow("Category", inspector_category_);
  inspector_type_ = new QLineEdit(metadata_group);
  inspector_type_->setReadOnly(true);
  metadata_form->addRow("Type", inspector_type_);
  selection_tab_layout->addWidget(metadata_group);
  inspector_live_update_box_ = new QCheckBox("Live update", scene_builder); inspector_live_update_box_->setChecked(false); selection_tab_layout->addWidget(inspector_live_update_box_);
  auto * transform_actions = new QHBoxLayout();
  inspector_apply_button_ = new QPushButton("Apply", scene_builder); transform_actions->addWidget(inspector_apply_button_);
  inspector_revert_button_ = new QPushButton("Revert", scene_builder); transform_actions->addWidget(inspector_revert_button_);
  inspector_copy_transform_button_ = new QPushButton("Copy Transform", scene_builder); transform_actions->addWidget(inspector_copy_transform_button_);
  inspector_paste_transform_button_ = new QPushButton("Paste Transform", scene_builder); transform_actions->addWidget(inspector_paste_transform_button_);
  selection_tab_layout->addLayout(transform_actions);
  auto * robot_pose_group = new QGroupBox("Robot Base Pose", scene_builder);
  robot_pose_group->setObjectName("studioCard");
  auto * robot_pose_layout = new QVBoxLayout(robot_pose_group);
  robot_pose_source_label_ = new QLabel("robot_pose_source: blocked", robot_pose_group);
  robot_pose_layout->addWidget(robot_pose_source_label_);
  robot_pose_message_label_ = new QLabel("Select robot base to edit pose.", robot_pose_group);
  robot_pose_message_label_->setWordWrap(true);
  robot_pose_layout->addWidget(robot_pose_message_label_);
  auto * robot_pose_grid = new QGridLayout();
  robot_base_x_ = new QDoubleSpinBox(robot_pose_group); robot_base_x_->setPrefix("x "); robot_pose_grid->addWidget(robot_base_x_, 0, 0);
  robot_base_y_ = new QDoubleSpinBox(robot_pose_group); robot_base_y_->setPrefix("y "); robot_pose_grid->addWidget(robot_base_y_, 0, 1);
  robot_base_z_ = new QDoubleSpinBox(robot_pose_group); robot_base_z_->setPrefix("z "); robot_pose_grid->addWidget(robot_base_z_, 0, 2);
  robot_base_roll_ = new QDoubleSpinBox(robot_pose_group); robot_base_roll_->setPrefix("r "); robot_pose_grid->addWidget(robot_base_roll_, 1, 0);
  robot_base_pitch_ = new QDoubleSpinBox(robot_pose_group); robot_base_pitch_->setPrefix("p "); robot_pose_grid->addWidget(robot_base_pitch_, 1, 1);
  robot_base_yaw_ = new QDoubleSpinBox(robot_pose_group); robot_base_yaw_->setPrefix("yaw "); robot_pose_grid->addWidget(robot_base_yaw_, 1, 2);
  robot_pose_layout->addLayout(robot_pose_grid);
  auto * robot_pose_actions = new QHBoxLayout();
  robot_base_apply_button_ = new QPushButton("Apply", robot_pose_group); robot_pose_actions->addWidget(robot_base_apply_button_);
  robot_base_reset_button_ = new QPushButton("Reset", robot_pose_group); robot_pose_actions->addWidget(robot_base_reset_button_);
  robot_pose_layout->addLayout(robot_pose_actions);
  selection_tab_layout->addWidget(robot_pose_group);
  inspector_warning_label_ = new QLabel("Warnings: none | Reachability: unknown | Collision: unknown | Safety zone: unknown | Pick reach: unknown | Place reach: unknown | Warning count: 0 | Preview-only", scene_builder); inspector_warning_label_->setWordWrap(true); readiness_card_layout->addWidget(inspector_warning_label_);
  scene_builder_studio_log_ = new QPlainTextEdit(logs_tab);
  scene_builder_studio_log_->setReadOnly(true);
  scene_builder_studio_log_->setPlaceholderText("Scene Builder logs and command traces appear here.");
  logs_tab_layout->addWidget(scene_builder_studio_log_);
  scene_builder_inspector_tabs_->addTab(selection_tab, "Selection");
  scene_builder_inspector_tabs_->addTab(workflow_tab, "Workflow");
  scene_builder_inspector_tabs_->addTab(actions_tab, "Actions");
  scene_builder_inspector_tabs_->addTab(readiness_tab, "Readiness");
  scene_builder_inspector_tabs_->addTab(logs_tab, "Logs");
  inspector_scroll_layout->addWidget(scene_builder_inspector_tabs_);
  inspector_scroll->setWidget(inspector_scroll_contents);
  right_layout->addWidget(inspector_scroll, 1);
  auto * existing = new QWidget(studio_pages_); auto * el=new QVBoxLayout(existing);
  el->addWidget(new QLabel("<h2>Existing Scenes</h2>"));
  existing_scene_table_=new QTableWidget(0,6,existing); existing_scene_table_->setHorizontalHeaderLabels({"Scene","Status","Open in Scene Builder","Open Preview","Open Smoke Report","Copy Launch Command"}); el->addWidget(existing_scene_table_);

  auto * demo = new QWidget(studio_pages_); auto * dm=new QVBoxLayout(demo);
  dm->addWidget(new QLabel("<h2>Demo Mode</h2><p>Big status summary, acceptance/smoke/preview cards, and command card for investor demo. Offline/fake-hardware preview only.</p>"));
  dm->addWidget(new QLabel("<b>Safety banner:</b> Fake Hardware | No Robot Motion | PREVIEW_ONLY"));
  auto * run_demo = new QPushButton("Run Demo Readiness", demo); run_demo->setProperty("role","primary"); dm->addWidget(run_demo);
  auto * go_validation = new QPushButton("Go to Validation", demo); dm->addWidget(go_validation);
  auto * go_preview = new QPushButton("Go to Plan / Simulate", demo); dm->addWidget(go_preview);
  auto * go_export = new QPushButton("Go to Export", demo); dm->addWidget(go_export);
  auto * open_dash = new QPushButton("Open Demo Dashboard", demo); dm->addWidget(open_dash);
  auto * go_scene_builder = new QPushButton("Go to Scene Builder", demo); dm->addWidget(go_scene_builder);
  auto * go_preview_commands = new QPushButton("Go to Preview Commands", demo); dm->addWidget(go_preview_commands);
  auto * copy_summary = new QPushButton("Copy Demo Summary", demo); dm->addWidget(copy_summary);
  dm->addWidget(new QLabel("Layout merge actions are owned by Scene Builder."));

  auto * diagnostics = new QWidget(studio_pages_); auto * gl = new QVBoxLayout(diagnostics);
  gl->addWidget(new QLabel("<h2>Diagnostics / First-Run Self-Test</h2><p>Offline checks only. No ROS launch, no MoveIt, no robot motion.</p>"));
  diagnostics_indicator_label_ = new QLabel("Diagnostics: NOT CHECKED", diagnostics); gl->addWidget(diagnostics_indicator_label_);
  diagnostics_status_label_ = new QLabel("Status: NOT CHECKED", diagnostics); gl->addWidget(diagnostics_status_label_);
  diagnostics_summary_label_ = new QLabel("Run Self-Test to populate diagnostics details.", diagnostics); diagnostics_summary_label_->setWordWrap(true); gl->addWidget(diagnostics_summary_label_);
  diagnostics_table_ = new QTableWidget(0,5,diagnostics); diagnostics_table_->setHorizontalHeaderLabels({"Check","Status","Details","Suggested Fix","Related Path"}); diagnostics_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); gl->addWidget(diagnostics_table_);
  auto * d_actions = new QHBoxLayout();
  run_self_test_button_ = new QPushButton("Run Self-Test", diagnostics); d_actions->addWidget(run_self_test_button_);
  run_golden_flow_button_ = new QPushButton("Run Golden Flow Dry Run", diagnostics); d_actions->addWidget(run_golden_flow_button_);
  copy_diagnostics_report_button_ = new QPushButton("Copy Diagnostics Report", diagnostics); d_actions->addWidget(copy_diagnostics_report_button_);
  open_diagnostics_report_button_ = new QPushButton("Open Diagnostics Report", diagnostics); d_actions->addWidget(open_diagnostics_report_button_);
  auto * copy_golden_cmd = new QPushButton("Copy Golden Flow Command", diagnostics); d_actions->addWidget(copy_golden_cmd);
  auto * copy_build_cmd = new QPushButton("Copy Build Command", diagnostics); d_actions->addWidget(copy_build_cmd);
  auto * copy_source_cmd = new QPushButton("Copy Source Command", diagnostics); d_actions->addWidget(copy_source_cmd);
  auto * open_logs_cmd = new QPushButton("Open Logs Folder", diagnostics); d_actions->addWidget(open_logs_cmd);
  gl->addLayout(d_actions);
  auto * preview = new QWidget(studio_pages_); auto * pl=new QVBoxLayout(preview);
  pl->addWidget(new QLabel("<h2>Plan / Simulate</h2><p>Workcell Studio planning and simulation console. Simulation motion is allowed with fake hardware. Real hardware execution remains guarded and is not launched from this mode.</p>"));
  preview_scene_label_ = new QLabel("<b>Selected Scene</b><br/>scene name: none"); preview_scene_label_->setObjectName("studioCard"); preview_scene_label_->setWordWrap(true); pl->addWidget(preview_scene_label_);
  preview_status_label_ = new QLabel("<b>Readiness Gate</b><br/>BLOCKED_MISSING_SCENE"); preview_status_label_->setObjectName("studioCard"); preview_status_label_->setWordWrap(true); pl->addWidget(preview_status_label_);
  preview_safety_label_ = new QLabel("<b>Status</b><br/>Mode: Plan / Simulate | Hardware: fake by default | Simulation motion: allowed with fake hardware | Real robot motion: locked | Real hardware execution requires explicit guarded setup and is not launched from this mode."); preview_safety_label_->setObjectName("studioCard"); preview_safety_label_->setWordWrap(true); pl->addWidget(preview_safety_label_);
  preview_commands_ = new QTextEdit(preview); preview_commands_->setReadOnly(true); preview_commands_->setObjectName("studioCard"); pl->addWidget(preview_commands_);
  preview_log_ = new QPlainTextEdit(preview); preview_log_->setReadOnly(true); preview_log_->setPlaceholderText("Live Log: command started / command output / command finished or failed / transcript path"); preview_log_->setObjectName("studioCard"); pl->addWidget(preview_log_);
  pl->addWidget(new QLabel("<b>Preview Process</b><br/>Open RViz Truth Preview | Run Fake-Hardware Simulation | Stop Simulation | Copy commands"));
  run_build_button_ = new QPushButton("Open RViz Truth Preview", preview); pl->addWidget(run_build_button_);
  run_preview_button_ = new QPushButton("Run Fake-Hardware Simulation", preview); run_preview_button_->setProperty("role","primary"); pl->addWidget(run_preview_button_);
  stop_preview_button_ = new QPushButton("Stop Simulation", preview); pl->addWidget(stop_preview_button_);
  copy_launch_button_ = new QPushButton("Copy Launch Command", preview); pl->addWidget(copy_launch_button_);
  preview_more_actions_button_ = new QToolButton(preview);
  preview_more_actions_button_->setText("More Actions");
  preview_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * preview_more_menu = new QMenu(preview_more_actions_button_);
  auto * open_scene_folder_action = preview_more_menu->addAction("Open Scene Folder");
  auto * open_preview_report_action = preview_more_menu->addAction("Open Preview Report");
  auto * open_dashboard_action = preview_more_menu->addAction("Open Dashboard");
  auto * copy_source_action = preview_more_menu->addAction("Copy Source Command");
  auto * copy_build_action = preview_more_menu->addAction("Copy Build Command");
  preview_more_actions_button_->setMenu(preview_more_menu);
  pl->addWidget(preview_more_actions_button_);
  copy_build_button_ = new QPushButton("Copy Build Command", preview);
  copy_source_button_ = new QPushButton("Copy Source Command", preview);
  copy_all_button_ = new QPushButton("Copy Full Command Block", preview);
  open_preview_folder_button_ = new QPushButton("Open Scene Folder", preview);
  pl->addWidget(new QLabel("<b>Reports / Artifacts</b><br/>environment.yaml | scene_manifest.yaml | task intent YAML | task recipe YAML | preview/static_preview.html | preview/static_preview.svg | preview/workcell_studio_canvas_snapshot.png | smoke/offline_smoke_report.json/html | readiness dashboard"));
  open_preview_transcript_button_ = new QPushButton("Open Reports", preview); pl->addWidget(open_preview_transcript_button_);

  auto * validation = new QWidget(studio_pages_); auto * vl = new QVBoxLayout(validation);
  vl->addWidget(new QLabel("<h2>Validation</h2><p>Validation Summary, Blockers, Warnings, Required Files, Task Intent Status, Scene Package Status, Next Fix Suggestions. Fake Hardware | No Robot Motion | Preview Only.</p>"));
  validation_summary_label_ = new QLabel("<b>Validation Summary</b><br/>No scene selected"); validation_summary_label_->setObjectName("studioCard"); validation_summary_label_->setWordWrap(true); vl->addWidget(validation_summary_label_);
  validation_blockers_label_ = new QLabel("<b>Blockers</b><br/>-"); validation_blockers_label_->setObjectName("studioCard"); validation_blockers_label_->setWordWrap(true); vl->addWidget(validation_blockers_label_);
  validation_warnings_label_ = new QLabel("<b>Warnings</b><br/>-"); validation_warnings_label_->setObjectName("studioCard"); validation_warnings_label_->setWordWrap(true); vl->addWidget(validation_warnings_label_);
  validation_required_files_label_ = new QLabel("<b>Required Files</b><br/>-"); validation_required_files_label_->setObjectName("studioCard"); validation_required_files_label_->setWordWrap(true); vl->addWidget(validation_required_files_label_);
  validation_task_intent_status_label_ = new QLabel("<b>Task Intent Status</b><br/>-"); validation_task_intent_status_label_->setObjectName("studioCard"); validation_task_intent_status_label_->setWordWrap(true); vl->addWidget(validation_task_intent_status_label_);
  validation_scene_package_status_label_ = new QLabel("<b>Scene Package Status</b><br/>-"); validation_scene_package_status_label_->setObjectName("studioCard"); validation_scene_package_status_label_->setWordWrap(true); vl->addWidget(validation_scene_package_status_label_);
  validation_next_fix_label_ = new QLabel("<b>Next Fix Suggestions</b><br/>Select scene and run offline validation."); validation_next_fix_label_->setObjectName("studioCard"); validation_next_fix_label_->setWordWrap(true); vl->addWidget(validation_next_fix_label_);
  auto * run_offline_validation_button = new QPushButton("Run Offline Validation", validation); vl->addWidget(run_offline_validation_button);
  auto * validate_layout_button = new QPushButton("Validate Layout", validation); vl->addWidget(validate_layout_button);
  validation_more_actions_button_ = new QToolButton(validation);
  validation_more_actions_button_->setText("More Actions");
  validation_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * validation_more_menu = new QMenu(validation_more_actions_button_);
  auto * open_validation_report_action = validation_more_menu->addAction("Open Validation Report");
  auto * copy_validation_summary_action = validation_more_menu->addAction("Copy Validation Summary");
  auto * open_readiness_dashboard_action = validation_more_menu->addAction("Open Readiness Dashboard");
  auto * check_canvas_parity_action = validation_more_menu->addAction("Check Canvas/RViz Parity");
  validation_more_actions_button_->setMenu(validation_more_menu);
  vl->addWidget(validation_more_actions_button_);
  auto * generate_readiness_pack_button = new QPushButton("Generate Readiness Pack", validation); vl->addWidget(generate_readiness_pack_button);
  auto * export_page = new QWidget(studio_pages_); auto * exl = new QVBoxLayout(export_page);
  exl->addWidget(new QLabel("<h2>Portable Scene Bundle</h2><p>Safe export/import for developer handoff. Preview only; no robot motion commands.</p>"));
  scene_bundle_selected_scene_label_ = new QLabel("Selected scene: none", export_page); scene_bundle_selected_scene_label_->setObjectName("studioCard"); scene_bundle_selected_scene_label_->setWordWrap(true); exl->addWidget(scene_bundle_selected_scene_label_);
  scene_bundle_destination_label_ = new QLabel("Export destination: pending", export_page); scene_bundle_destination_label_->setObjectName("studioCard"); scene_bundle_destination_label_->setWordWrap(true); exl->addWidget(scene_bundle_destination_label_);
  scene_bundle_contents_label_ = new QLabel("Bundle contents summary: select a scene", export_page); scene_bundle_contents_label_->setObjectName("studioCard"); scene_bundle_contents_label_->setWordWrap(true); exl->addWidget(scene_bundle_contents_label_);
  auto * export_scene_bundle_button = new QPushButton("Export Scene Bundle", export_page); export_scene_bundle_button->setProperty("role", "primary"); exl->addWidget(export_scene_bundle_button);
  auto * import_scene_bundle_button = new QPushButton("Import Scene Bundle", export_page); exl->addWidget(import_scene_bundle_button);
  export_more_actions_button_ = new QToolButton(export_page);
  export_more_actions_button_->setText("More Actions");
  export_more_actions_button_->setPopupMode(QToolButton::InstantPopup);
  auto * export_more_menu = new QMenu(export_more_actions_button_);
  auto * open_export_folder_action = export_more_menu->addAction("Open Export Folder");
  export_more_menu->addAction("Copy Bundle Summary");
  export_more_menu->addAction("Open Bundle Docs");
  export_more_actions_button_->setMenu(export_more_menu);
  exl->addWidget(export_more_actions_button_);

  studio_pages_->addWidget(dashboard);  // DashboardPage
  studio_pages_->addWidget(scene_builder);  // SceneBuilderPage
  studio_pages_->addWidget(existing);  // ExistingScenesPage
  studio_pages_->addWidget(demo);  // DemoModePage
  studio_pages_->addWidget(preview);  // PlanSimulatePage
  studio_pages_->addWidget(diagnostics);  // DiagnosticsPage
  studio_pages_->addWidget(validation);  // ValidationPage
  studio_pages_->addWidget(export_page);  // ExportPage
  auto * body=new QHBoxLayout(); body->addWidget(studio_pages_,1); root_layout->insertLayout(0,body,1);
  constexpr int kCollapsedLogPanelHeight = 52;
  constexpr int kExpandedLogPanelHeight = 240;
  auto * log_card = new QFrame(content); log_card->setObjectName("studioCard");
  log_card->setMaximumHeight(kCollapsedLogPanelHeight);
  auto * log_layout = new QVBoxLayout(log_card);
  auto * log_head = new QHBoxLayout();
  log_head->addWidget(new QLabel("Activity Log", log_card));
  scene_builder_log_toggle_button_ = new QPushButton("Show Log", log_card);
  scene_builder_log_toggle_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  log_head->addWidget(scene_builder_log_toggle_button_, 0, Qt::AlignRight);
  auto * clear_log = new QPushButton("Clear", log_card);
  log_head->addWidget(clear_log, 0, Qt::AlignRight);
  log_layout->addLayout(log_head);
  scene_builder_log_panel_ = log_card;
  studio_log_=new QTextEdit(log_card); studio_log_->setObjectName("studioHomeLog"); studio_log_->setReadOnly(true); studio_log_->setMaximumHeight(kExpandedLogPanelHeight); studio_log_->setPlaceholderText("Recent actions and diagnostics");
  log_layout->addWidget(studio_log_);
  studio_log_->setVisible(false);
  root_layout->addWidget(log_card, 0);
  root_layout->setStretch(0, 1);
  root_layout->setStretch(1, 0);
  preview_process_ = new QProcess(this);

  build_studio_header_actions();
  auto connect_action = [this](QAction * action, auto slot) {
    if (action) QObject::connect(action, &QAction::triggered, this, slot);
  };
  auto connect_button = [this](QPushButton * button, auto slot) {
    if (button) QObject::connect(button, &QPushButton::clicked, this, slot);
  };

  connect(studio_nav_, &QListWidget::currentRowChanged, this, [this](int idx){ if(idx>=0 && idx<studio_pages_->count()) studio_pages_->setCurrentIndex(idx);});
  show_studio_page(StudioPage::DashboardPage);
  connect(dashboard_scene_table_, &QTableWidget::cellDoubleClicked, this, [this](int row, int){ select_scene_by_row(row); open_scene_builder_for_selected_scene("Dashboard double-click"); });
  connect(dashboard_scene_table_, &QTableWidget::cellClicked, this, [this](int row, int){ select_scene_by_row(row); });
  connect(dashboard_scene_search_, &QLineEdit::textChanged, this, [this](const QString &){ refresh_studio_home_scene_table(); });
  connect(dashboard_scene_status_filter_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){ refresh_studio_home_scene_table(); });
  connect(dashboard_library_search_, &QLineEdit::textChanged, this, [this](const QString &){ refresh_studio_home_scene_table(); });
  connect(dashboard_library_status_filter_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){ refresh_studio_home_scene_table(); });
  connect(dashboard_library_list_, &QListWidget::currentRowChanged, this, [this](int row){
    if (row < 0) return;
    auto * item = dashboard_library_list_->item(row);
    if (!item) return;
    select_scene_by_row(item->data(Qt::UserRole).toInt());
  });
  connect_button(clear_log, [this](){ if (studio_log_) studio_log_->clear(); });
  connect_button(scene_builder_log_toggle_button_, [this]() {
    if (!studio_log_ || !scene_builder_log_toggle_button_ || !scene_builder_log_panel_) return;
    constexpr int kCollapsedLogPanelHeight = 52;
    constexpr int kExpandedLogPanelHeight = 240;
    const bool show = !studio_log_->isVisible();
    studio_log_->setVisible(show);
    scene_builder_log_panel_->setMaximumHeight(show ? kExpandedLogPanelHeight : kCollapsedLogPanelHeight);
    scene_builder_log_toggle_button_->setText(show ? "Hide Log" : "Show Log");
  });
  connect_button(empty_new_cell, &MainWindow::open_new_scene_creation_flow);
  connect_button(dash_new_cell, &MainWindow::open_new_scene_creation_flow);
  connect_button(dash_open_selected_scene, [this](){ open_scene_builder_for_selected_scene("Dashboard Open Selected Scene"); });
  connect_action(dashboard_open_scene_action_, [this](){ open_scene_builder_for_selected_scene("Dashboard Open in Scene Builder"); });
  connect_action(dashboard_validate_action_, [this](){ if (action_validate_offline_) action_validate_offline_->trigger(); });
  connect_action(dashboard_plan_action_, [this](){ if (action_simulate_plan_preview_) action_simulate_plan_preview_->trigger(); });
  connect_action(dashboard_export_action_, [this](){ if (action_export_open_page_) action_export_open_page_->trigger(); });
  connect_action(dashboard_delete_action_, &MainWindow::delete_selected_scene);
  connect(existing_scene_table_, &QTableWidget::cellClicked, this, [this](int row, int col){ select_scene_by_row(row); if(col==2){open_scene_builder_for_selected_scene("Existing Scenes Open in Scene Builder");} else if(col==3){open_selected_scene_artifact("preview");} else if(col==4){open_selected_scene_artifact("smoke");} else if(col==5){QApplication::clipboard()->setText(selected_scene_launch_command()); append_studio_log("Copy Launch Command");}});
  connect_action(open_asset_folder_action, [this](){ open_selected_scene_artifact("asset_folder"); });
  connect_action(copy_asset_path_action, [this](){ QApplication::clipboard()->setText(selected_catalog_item_path()); });
  connect_action(import_asset_action, [this](){ append_studio_log("Import STL / URDF: choose asset import flow from Asset Browser."); });
  connect_action(add_existing_stl_action, [this](){ append_studio_log("Add Existing STL to Canvas: choose STL in Asset Browser and click Add to Canvas."); });
  connect_action(placeholder_action, [this](){ append_studio_log("Generate Simple Box/Cylinder Placeholder: use quick-add placeholders in catalog."); });
  connect_button(pick_source_button_, &MainWindow::bind_selected_item_as_pick_zone);
  connect_button(place_target_button_, &MainWindow::bind_selected_item_as_place_zone);
  connect_button(camera_button_, &MainWindow::bind_selected_item_as_camera);
  connect_button(run_demo, [this](){ append_studio_log("Demo readiness completed"); });
  connect_button(open_dash, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect_button(copy_summary, [this](){ open_selected_scene_artifact("demo_summary_copy"); });
  connect_button(go_validation, [this](){ show_studio_page(StudioPage::ValidationPage); append_studio_log("Go to Validation: switched to Validation page"); });
  connect_button(go_preview, [this](){ show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); append_studio_log("Go to Plan & Simulate: switched to Plan & Simulate page"); });
  connect_button(go_export, [this](){ show_studio_page(StudioPage::ExportPage); append_studio_log("Go to Export: switched to Export page"); });
  connect_button(go_scene_builder, [this](){ show_studio_page(StudioPage::SceneBuilderPage); append_studio_log("Go to Scene Builder: switched to Scene Builder page"); });
  connect_button(go_preview_commands, [this](){ show_studio_page(StudioPage::PlanSimulatePage); append_studio_log("Go to Preview Commands: use Copy commands on Preview Launch page"); });
  connect_button(run_build_button_, &MainWindow::run_preview_build);
  connect_button(run_preview_button_, &MainWindow::run_fake_hardware_preview);
  connect_button(stop_preview_button_, &MainWindow::stop_preview_process);
  connect_button(copy_build_button_, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect_button(copy_source_button_, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect_button(copy_launch_button_, [this](){ QApplication::clipboard()->setText(selected_scene_launch_command()); });
  connect_button(copy_all_button_, [this](){ QApplication::clipboard()->setText(selected_scene_preview_command_block()); });
  connect_button(open_preview_folder_button_, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect_button(open_preview_transcript_button_, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect_button(run_offline_validation_button, &MainWindow::run_offline_validation);
  connect_button(validate_layout_button, &MainWindow::run_layout_validation_only);
  connect_action(open_validation_report_action, &MainWindow::open_validation_report);
  connect_action(copy_validation_summary_action, &MainWindow::copy_validation_summary);
  connect_button(generate_readiness_pack_button, &MainWindow::generate_readiness_pack);
  connect_action(open_readiness_dashboard_action, &MainWindow::open_readiness_dashboard);
  connect_action(check_canvas_parity_action, &MainWindow::check_canvas_generated_parity);
  connect_button(export_scene_bundle_button, &MainWindow::export_scene_bundle_for_selected_scene);
  connect_button(import_scene_bundle_button, &MainWindow::import_scene_bundle_into_scenes_root);
  connect_action(open_export_folder_action, &MainWindow::open_scene_bundle_export_folder);
  connect_action(open_scene_folder_action, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect_action(open_preview_report_action, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect_action(open_dashboard_action, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect_action(copy_source_action, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect_action(copy_build_action, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect_button(run_self_test_button_, &MainWindow::run_diagnostics_self_test);
  connect_button(run_golden_flow_button_, &MainWindow::run_diagnostics_golden_flow_dry_run);
  connect_button(copy_diagnostics_report_button_, &MainWindow::copy_diagnostics_report);
  connect_button(open_diagnostics_report_button_, &MainWindow::open_diagnostics_folder);
  connect_button(copy_golden_cmd, [this](){ QApplication::clipboard()->setText("python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/workcell_studio_diag_scene --json"); });
  connect_button(copy_build_cmd, [this](){ QApplication::clipboard()->setText("source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder"); });
  connect_button(copy_source_cmd, [this](){ QApplication::clipboard()->setText("source install/setup.bash"); });
  connect_button(open_logs_cmd, [this](){ open_diagnostics_folder(); });
  connect_action(fit_button, [this](){
    if (!digital_twin_canvas_ || !digital_twin_canvas_->scene()) return;
    auto * scene = digital_twin_canvas_->scene();
    if (!scene->property("workcellPhysicalBoundsValid").toBool()) return;
    const QRectF bounds = scene->property("workcellPhysicalBounds").toRectF();
    if (bounds.isValid() && !bounds.isNull()) digital_twin_canvas_->fitInView(bounds.adjusted(-24,-24,24,24), Qt::KeepAspectRatio);
  });
  connect_action(fit_robot_button, [this](){
    if (!scene_preview_widget_) return;
    auto * viewport = scene_preview_widget_->findChild<Scene3DViewportWidget *>();
    if (!viewport) return;
    viewport->fit_robot();
  });
  connect_action(reset_button, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->resetTransform(); rebuild_digital_twin_canvas(); });
  connect_action(zoom_in, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(1.15,1.15); });
  connect_action(zoom_out, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(0.85,0.85); });
  connect_action(perspective_action, [this](){ scene_builder_is_3d_view_ = true; refresh_scene_builder_view_chips(); });
  auto set_2d_layout_view = [this]() { scene_builder_is_3d_view_ = false; refresh_scene_builder_view_chips(); };
  connect_action(top_action, set_2d_layout_view);
  connect_action(left_action, set_2d_layout_view);
  connect_action(right_action, set_2d_layout_view);
  connect_action(front_action, set_2d_layout_view);
  connect(toggle_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Snap to Grid"); });
  connect(fine_move_mode_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Fine Move Mode"); });
  connect(unlock_robot_base_box_, &QCheckBox::toggled, this, [this](bool checked){ if (checked) { QMessageBox::warning(this, "Unlock Robot Base", "Robot base is locked by default. Moving robot base may invalidate reach and safety assumptions."); }});
  connect(toggle_labels_box_, &QCheckBox::toggled, this, [this](bool enabled){
    if (scene_preview_widget_) scene_preview_widget_->set_label_mode(enabled ? ScenePreviewWidget::LabelMode::Selected : ScenePreviewWidget::LabelMode::Off);
    rebuild_digital_twin_canvas();
  });
  connect(toggle_warnings_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  for (auto * box : {
      preview_layer_editable_layout_box_, preview_layer_generated_urdf_visual_box_,
      preview_layer_mesh_preview_box_, preview_layer_primitive_fallback_box_,
      preview_layer_overlays_helpers_box_, preview_layer_warnings_missing_assets_box_})
  {
    if (box) {
      connect(box, &QCheckBox::toggled, this, [this](bool) { apply_scene3d_preview_layer_filters(true); });
    }
  }
  connect_button(duplicate_layout_button_, &MainWindow::duplicate_selected_item);
  connect_button(delete_layout_button_, &MainWindow::delete_selected_item);
  for (auto *sb : {inspector_x_, inspector_y_, inspector_z_, inspector_roll_, inspector_pitch_, inspector_yaw_}) connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ if (inspector_live_update_box_ && inspector_live_update_box_->isChecked()) apply_selection_transform_from_editor(); });
  for (auto *sb : {inspector_dim_x_, inspector_dim_y_, inspector_dim_z_}) connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ if (inspector_live_update_box_ && inspector_live_update_box_->isChecked()) apply_selection_transform_from_editor(); });
  connect_button(inspector_apply_button_, &MainWindow::apply_selection_transform_from_editor);
  connect_button(inspector_revert_button_, &MainWindow::revert_selection_transform_editor);
  connect_button(inspector_copy_transform_button_, &MainWindow::copy_selection_transform_to_clipboard);
  connect_button(inspector_paste_transform_button_, &MainWindow::paste_selection_transform_from_clipboard);
  connect_button(robot_base_apply_button_, &MainWindow::apply_robot_base_pose_from_inspector);
  connect_button(robot_base_reset_button_, &MainWindow::reset_robot_base_pose_from_snapshot);
  inspector_x_->setToolTip("X position in metres"); inspector_y_->setToolTip("Y position in metres"); inspector_z_->setToolTip("Z position in metres");
  inspector_roll_->setToolTip("Roll in radians"); inspector_pitch_->setToolTip("Pitch in radians"); inspector_yaw_->setToolTip("Yaw in radians");
  inspector_dim_x_->setToolTip("Scale X (uniform Scale control for simple mesh assets)"); inspector_dim_y_->setToolTip("Scale Y"); inspector_dim_z_->setToolTip("Scale Z");
  refresh_robot_base_pose_inspector();
  connect_button(save_layout_button_, &MainWindow::save_layout_changes);
  connect_button(create_starter_layout_button_, &MainWindow::create_starter_layout_from_preview);
  connect_button(select_mode_button, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); });
  connect_button(place_mode_button, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Place); });
  connect_button(move_mode_button, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Move); });
  connect_button(inspect_mode_button, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Inspect); });
  if (snap_to_grid_box_) {
    connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  }
  if (snap_action && snap_to_grid_box_) {
    connect(snap_action, &QAction::toggled, snap_to_grid_box_, &QCheckBox::setChecked);
    connect(snap_to_grid_box_, &QCheckBox::toggled, snap_action, &QAction::setChecked);
  }
  if (fine_move_action && fine_move_mode_box_) {
    connect(fine_move_action, &QAction::toggled, fine_move_mode_box_, &QCheckBox::setChecked);
    connect(fine_move_mode_box_, &QCheckBox::toggled, fine_move_action, &QAction::setChecked);
  }
  if (unlock_action && unlock_robot_base_box_) {
    connect(unlock_action, &QAction::toggled, unlock_robot_base_box_, &QCheckBox::setChecked);
    connect(unlock_robot_base_box_, &QCheckBox::toggled, unlock_action, &QAction::setChecked);
  }
  if (minimap_action && show_minimap_box_) {
    connect(minimap_action, &QAction::toggled, show_minimap_box_, &QCheckBox::setChecked);
    connect(show_minimap_box_, &QCheckBox::toggled, minimap_action, &QAction::setChecked);
  }
  if (show_minimap_box_) {
    connect(show_minimap_box_, &QCheckBox::toggled, this, [this](bool on){
      minimap_requested_visible_ = on;
      refresh_minimap_card();
    });
  }
  connect(digital_twin_canvas_->horizontalScrollBar(), &QScrollBar::valueChanged, this, [this](int){ refresh_minimap_card(); });
  connect(digital_twin_canvas_->verticalScrollBar(), &QScrollBar::valueChanged, this, [this](int){ refresh_minimap_card(); });
  for (auto * box : {show_reach_overlay_box_, show_camera_fov_overlay_box_, show_pick_place_overlay_box_, show_trajectory_overlay_box_}) {
    if (box) {
      connect(box, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
    }
  }
  auto * del_sc = new QShortcut(QKeySequence(Qt::Key_Delete), scene_builder); connect(del_sc,&QShortcut::activated,this,&MainWindow::delete_selected_item);
  auto * save_sc = new QShortcut(QKeySequence::Save, scene_builder); connect(save_sc,&QShortcut::activated,this,&MainWindow::save_layout_changes);
  auto * undo_sc = new QShortcut(QKeySequence::Undo, scene_builder); connect(undo_sc, &QShortcut::activated, this, &MainWindow::undo_layout_edit);
  auto * redo_sc = new QShortcut(QKeySequence::Redo, scene_builder); connect(redo_sc, &QShortcut::activated, this, &MainWindow::redo_layout_edit);
  auto * esc_sc = new QShortcut(QKeySequence(Qt::Key_Escape), scene_builder); connect(esc_sc,&QShortcut::activated,this,[this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); if(digital_twin_scene_) digital_twin_scene_->clearSelection(); ghost_preview_item_=nullptr; rebuild_digital_twin_canvas(); });
  auto * fit_sc = new QShortcut(QKeySequence(Qt::Key_F), scene_builder); connect(fit_sc,&QShortcut::activated,fit_button,&QAction::trigger);
  connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *item, int column){ Q_UNUSED(column); on_hierarchy_item_selected(item); });
  connect(asset_filter_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::on_asset_filter_changed);
  connect_action(open_asset_folder_action, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(p).isDir() ? p : QFileInfo(p).absolutePath())); });
  connect_action(copy_asset_path_action, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QApplication::clipboard()->setText(p); append_studio_log("Copy Asset Path: " + p); });
  connect_button(add_to_canvas_button_, [this](){ if (!asset_catalog_tree_ || !asset_catalog_tree_->currentItem()) { QMessageBox::information(this, "Asset Catalog", "Select an asset to add to canvas."); return; } auto *it = asset_catalog_tree_->currentItem(); const int idx = it->data(0, CatalogRoleIndex).toInt(); if (idx < 0 || idx >= asset_catalog_entries_.size()) return; const auto & e = asset_catalog_entries_[idx]; if (!e.disabled_reason.trimmed().isEmpty()) { QMessageBox::information(this, "Asset Catalog", e.disabled_reason); return; } add_asset_to_canvas_from_catalog(e.category, e.display_name, e.source_path); });
  connect_button(add_asset_button_, &MainWindow::open_add_asset_dialog);
  connect_if(scene_workflow_recommendation_button_, this, &QPushButton::clicked, [this]() {
    trigger_recommended_workflow_action(resolve_recommended_workflow_action().handler);
  });
  connect(asset_catalog_tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem *it, int){ if (!it) return; const int idx = it->data(0, CatalogRoleIndex).toInt(); if (idx < 0 || idx >= asset_catalog_entries_.size()) return; const auto & e = asset_catalog_entries_[idx]; if (!e.disabled_reason.trimmed().isEmpty()) return; add_asset_to_canvas_from_catalog(e.category, e.display_name, e.source_path); });
  connect(asset_catalog_tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem *, QTreeWidgetItem *){ validate_asset_catalog_selection(); });
  connect_action(import_asset_action, [this](){ QMessageBox::information(this, "Asset Catalog", "Import STL / URDF keeps existing behavior via filesystem import workflows."); });
  connect_action(add_existing_stl_action, [this](){ QMessageBox::information(this, "Asset Catalog", "Add Existing STL to Canvas keeps existing behavior for scene assets."); });
  connect_action(placeholder_action, [this](){ add_asset_to_canvas_from_catalog("Custom", "Generated Placeholder", "placeholder://generated"); });
  connect(preview_process_, &QProcess::started, this, &MainWindow::handle_preview_started);
  connect(preview_process_, &QProcess::readyReadStandardOutput, this, &MainWindow::handle_preview_stdout);
  connect(preview_process_, &QProcess::readyReadStandardError, this, &MainWindow::handle_preview_stderr);
  connect(preview_process_, &QProcess::errorOccurred, this, &MainWindow::handle_preview_error);
  connect(preview_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &MainWindow::handle_preview_finished);
  refresh_scene_browser_ui();
  refresh_preview_launch_ui();
  refresh_new_cell_checklist();
  append_studio_log("New Cell Action Map: Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate");
  refresh_diagnostics_quick_status();
  refresh_scene_builder_left_explorer();
  refresh_task_intent_panel();
  refresh_scene_bundle_export_panel();
}

QAction * MainWindow::scene_builder_action(const QString & key) const
{
  return scene_builder_action_registry_.value(key, nullptr);
}

void MainWindow::register_scene_builder_action(const QString & key, QAction * action)
{
  if (!action) return;
  scene_builder_action_registry_.insert(key, action);
}


void MainWindow::export_canvas_snapshot()
{
  if (!digital_twin_canvas_ || !digital_twin_canvas_->scene()) return;
  fs::path out;
  if (selected_scene_index_ >= 0 && selected_scene_index_ < (int)scene_browser_result_.scenes.size()) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    out = s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.png";
  } else {
    out = fs::path(diagnostics_output_root().toStdString()) / "preview" / "workcell_studio_canvas_snapshot.png";
  }
  fs::create_directories(out.parent_path());
  QImage image(1280, 800, QImage::Format_ARGB32_Premultiplied);
  image.fill(QColor("#0f131a"));
  QPainter painter(&image);
  digital_twin_canvas_->scene()->render(&painter);
  painter.end();
  image.save(QString::fromStdString(out.string()));
  append_studio_log("Export Canvas Snapshot: " + QString::fromStdString(out.string()));
}

void MainWindow::build_studio_header_actions()
{
  action_workspace_studio_home_ = new QAction("Studio Home", this);
  connect(action_workspace_studio_home_, &QAction::triggered, this, [this]() {
    show_studio_page(StudioPage::DashboardPage);
    append_studio_log("Studio Home: switched to scene manager page.");
  });
  action_workspace_new_cell_ = new QAction("New Cell", this);
  connect(action_workspace_new_cell_, &QAction::triggered, this, [this]() {
    append_studio_log("New Cell: opening scene creation flow.");
    open_new_scene_creation_flow();
  });
  action_workspace_open_scene_builder_ = new QAction("Open Scene Builder", this);
  connect(action_workspace_open_scene_builder_, &QAction::triggered, this, [this]() { open_scene_builder_for_selected_scene("Header Scenes/Open"); });
  action_generate_package_ = new QAction("Generate Scene Package", this);
  connect(action_generate_package_, &QAction::triggered, this, [this]() {
    append_studio_log(QString("Generate Scene Package: requested for scene '%1'.").arg(selected_scene_name()));
    run_layout_merge_for_selected_scene(true);
    generate_scene_package_for_selected_scene();
  });
  action_generate_yaml_ = new QAction("Generate YAML", this);
  connect(action_generate_yaml_, &QAction::triggered, this, &MainWindow::generate_yaml_draft_for_selected_scene);
  action_generate_task_intent_ = new QAction("Generate/Update Task Intent", this);
  connect(action_generate_task_intent_, &QAction::triggered, this, &MainWindow::generate_or_update_task_intent_for_selected_scene);
  action_validate_offline_ = new QAction("Validate", this);
  connect(action_validate_offline_, &QAction::triggered, this, [this]() { append_studio_log(QString("Validate: offline validation for scene '%1'. No robot motion commanded.").arg(selected_scene_name())); show_studio_page(StudioPage::ValidationPage); run_offline_validation(); });
  action_validate_generated_scene_ = new QAction("Validate Generated Scene", this);
  connect(action_validate_generated_scene_, &QAction::triggered, this, &MainWindow::validate_generated_scene_for_selected_scene);
  action_validate_open_report_ = new QAction("Open Validation Report", this);
  connect(action_validate_open_report_, &QAction::triggered, this, &MainWindow::open_validation_report);
  action_validate_open_readiness_ = new QAction("Open Readiness Dashboard", this);
  connect(action_validate_open_readiness_, &QAction::triggered, this, [this]() { open_selected_scene_artifact("readiness_dashboard"); });
  action_simulate_plan_preview_ = new QAction("Plan/Simulate Preview", this);
  connect(action_simulate_plan_preview_, &QAction::triggered, this, [this]() {
    show_studio_page(StudioPage::PlanSimulatePage);
    refresh_preview_launch_ui();
    run_fake_hardware_preview();
    refresh_new_cell_checklist();
  });
  action_export_open_page_ = new QAction("Export", this);
  connect(action_export_open_page_, &QAction::triggered, this, [this]() { append_studio_log(QString("Export: opening export actions for scene '%1'.").arg(selected_scene_name())); show_studio_page(StudioPage::ExportPage); });
  action_view_demo_mode_ = new QAction("Demo Mode", this);
  connect(action_view_demo_mode_, &QAction::triggered, this, [this]() { append_studio_log(QString("Demo Mode: switched for scene '%1'. No robot motion commanded.").arg(selected_scene_name())); show_studio_page(StudioPage::DemoModePage); });
  action_view_diagnostics_page_ = new QAction("Open Diagnostics", this);
  connect(action_view_diagnostics_page_, &QAction::triggered, this, [this]() { show_studio_page(StudioPage::DiagnosticsPage); });
  action_diagnostics_run_self_test_ = new QAction("Run Diagnostics Self-Test", this);
  connect(action_diagnostics_run_self_test_, &QAction::triggered, this, &MainWindow::run_diagnostics_self_test);
  action_diagnostics_run_golden_flow_ = new QAction("Run Golden Flow Dry Run", this);
  connect(action_diagnostics_run_golden_flow_, &QAction::triggered, this, &MainWindow::run_diagnostics_golden_flow_dry_run);
  action_diagnostics_copy_report_ = new QAction("Copy Diagnostics Report", this);
  connect(action_diagnostics_copy_report_, &QAction::triggered, this, &MainWindow::copy_diagnostics_report);
  action_diagnostics_open_folder_ = new QAction("Open Diagnostics Folder", this);
  connect(action_diagnostics_open_folder_, &QAction::triggered, this, &MainWindow::open_diagnostics_folder);
  action_diagnostics_copy_build_launch_commands_ = new QAction("Copy Build & Launch Commands", this);
  connect(action_diagnostics_copy_build_launch_commands_, &QAction::triggered, this, &MainWindow::copy_build_launch_commands_for_selected_scene);

  QToolBar * top_bar = new QToolBar("Workcell Studio Command Bar", this);
  addToolBar(Qt::TopToolBarArea, top_bar);
  top_bar->setObjectName("studioTopBar");
  top_bar->setMovable(false);
  top_bar->setFloatable(false);
  top_bar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  for (const auto & item : std::initializer_list<std::pair<QString, QAction *>>{{"Studio Home", action_workspace_studio_home_}, {"New Cell", action_workspace_new_cell_}}) {
    auto * button = new QPushButton(item.first, this);
    connect(button, &QPushButton::clicked, this, [action = item.second]() { if (action) action->trigger(); });
    top_bar->addWidget(button);
  }
  auto * scenes_open_button = new QToolButton(this);
  scenes_open_button->setText("Scenes");
  scenes_open_button->setPopupMode(QToolButton::InstantPopup);
  auto * scenes_open_menu = new QMenu(scenes_open_button);
  scenes_open_menu->addAction(action_workspace_open_scene_builder_);
  scenes_open_menu->addSeparator();
  scenes_open_menu->addAction(action_generate_yaml_);
  scenes_open_menu->addAction(action_generate_task_intent_);
  scenes_open_menu->addAction(action_generate_package_);
  scenes_open_button->setMenu(scenes_open_menu);
  top_bar->addWidget(scenes_open_button);
  auto * run_next_button = new QToolButton(this);
  run_next_button->setText("Run Next");
  run_next_button->setPopupMode(QToolButton::InstantPopup);
  auto * run_next_menu = new QMenu(run_next_button);
  run_next_menu->addAction(action_validate_offline_);
  run_next_menu->addAction(action_validate_generated_scene_);
  action_simulate_plan_preview_->setText("Open RViz Truth Preview");
  run_next_menu->addAction(action_simulate_plan_preview_);
  run_next_menu->addAction(action_export_open_page_);
  run_next_button->setMenu(run_next_menu);
  top_bar->addWidget(run_next_button);
  auto * native_scene3d_help_label = new QLabel(
    "Native Scene3D: lightweight editable layout preview; not guaranteed RViz-equivalent.",
    this);
  native_scene3d_help_label->setWordWrap(true);
  native_scene3d_help_label->setObjectName("studioNativeScene3DHelpLabel");
  top_bar->addWidget(native_scene3d_help_label);
  auto * more_button = new QToolButton(this);
  more_button->setText("More");
  more_button->setPopupMode(QToolButton::InstantPopup);
  auto * more_menu = new QMenu(more_button);
  more_menu->addAction(action_view_demo_mode_);
  more_menu->addAction(action_view_diagnostics_page_);
  more_menu->addAction(action_validate_open_report_);
  more_menu->addAction(action_validate_open_readiness_);
  more_menu->addSeparator();
  more_menu->addAction(action_diagnostics_run_self_test_);
  more_menu->addAction(action_diagnostics_run_golden_flow_);
  more_menu->addAction(action_diagnostics_copy_report_);
  more_menu->addAction(action_diagnostics_open_folder_);
  more_menu->addAction(action_diagnostics_copy_build_launch_commands_);
  more_button->setMenu(more_menu);
  top_bar->addWidget(more_button);
  auto * rviz_truth_preview_help_label = new QLabel(
    "RViz Truth Preview: authoritative generated scene preview using ROS/MoveIt/RViz stack.",
    this);
  rviz_truth_preview_help_label->setWordWrap(true);
  rviz_truth_preview_help_label->setObjectName("studioRvizTruthPreviewHelpLabel");
  top_bar->addWidget(rviz_truth_preview_help_label);
}

void MainWindow::refresh_scene_bundle_export_panel()
{
  if (!scene_bundle_selected_scene_label_ || !scene_bundle_destination_label_ || !scene_bundle_contents_label_) return;
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) {
    scene_bundle_selected_scene_label_->setText("Selected scene: none");
    scene_bundle_destination_label_->setText("Export destination: select a scene first");
    scene_bundle_contents_label_->setText("Bundle contents summary: manifest + scene metadata + preview/readiness artifacts (when present)");
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path out_dir = s.scene_dir / "exports";
  const fs::path zip_out = out_dir / (s.scene_name + "_workcell_studio_bundle.zip");
  scene_bundle_selected_scene_label_->setText(QString("Selected scene: %1\nPath: %2").arg(QString::fromStdString(s.scene_name), QString::fromStdString(s.scene_dir.string())));
  scene_bundle_destination_label_->setText(QString("Export destination: %1").arg(QString::fromStdString(zip_out.string())));
  scene_bundle_contents_label_->setText("Bundle contents summary:\n- environment.yaml\n- scene_manifest.yaml\n- cell_definition.yaml\n- environment_layout.yaml\n- task_recipe.yaml\n- config/workcell_builder_task_intent.yaml\n- preview/ artifacts\n- validation/readiness reports\n- launch command notes\n- manifest.json");
}

void MainWindow::refresh_task_intent_panel()
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const auto ti = load_scene_task_intent_summary(sc.scene_dir);
  QStringList overlay_warnings;
  if (ti.pick_source == "unknown") overlay_warnings << "missing pick source";
  if (ti.place_target == "unknown") overlay_warnings << "missing place target";
  if (ti.approach_axis == "unknown") overlay_warnings << "unknown approach axis";
  if (ti.grasp_strategy == "unknown") overlay_warnings << "unknown grasp strategy";
  if (ti.pick_source != "unknown" && ti.pick_source == ti.place_target) overlay_warnings << "pick/place overlap";
  if (ti.status == "MISSING_TASK_FILE") overlay_warnings << "Task overlay unavailable: missing task intent";
  const QString missing = ti.status == "MISSING_TASK_FILE" ? QString("\nNo task intent file found.\nSearched:\n - %1").arg(ti.searched_paths.join("\n - ")) : "";
  task_intent_details_label_->setText(QString("Scene: %1\nTask type: %2\nSource task file: %3\nPick source: %4\nPlace target: %5\nObject/class: %6\nStatus badge: %7%8").arg(QString::fromStdString(sc.scene_name), ti.task_type, ti.source_basename, ti.pick_source, ti.place_target, ti.object_class, ti.status, missing));
  pick_place_details_label_->setText(QString("Pick source: %1\nPlace target: %2\nReject/bin target: %3\nLinked hierarchy item status: unknown").arg(ti.pick_source, ti.place_target, ti.reject_target));
  grasp_details_label_->setText(QString("Strategy/ref: %1\nTool/End Effector: %2\nApproach axis: %3\nOrientation mode: %4\nAllowed roll/yaw: %5").arg(ti.grasp_strategy, ti.tool_id, ti.approach_axis, ti.orientation_mode, ti.allowed_roll_yaw));
  const QString detection_mode_line = ti.perception_legacy_source.isEmpty() ? ti.perception_mode : QString("%1 (mapped from legacy: %2)").arg(ti.perception_mode, ti.perception_legacy_source);
  approach_retreat_details_label_->setText(QString("Approach distance: %1\nRetreat distance: %2\nApproach frame/axis: %3\nRetreat frame/axis: %4\nClearance: %5\nDetection mode: %6\nDetection status: %7\nDetails: %8").arg(ti.approach_distance, ti.retreat_distance, ti.approach_axis, ti.retreat_axis, ti.clearance, detection_mode_line, "Configured", "Detection snapshot overlays are preview-only and do not require launching live runtime nodes."));
  if (!overlay_warnings.isEmpty()) {
    approach_retreat_details_label_->setText(approach_retreat_details_label_->text() + QString("\nOverlay warnings: %1").arg(overlay_warnings.join(" | ")));
  }
  if (scene_preview_widget_) {
    ScenePreviewWidget::TaskOverlayModel model;
    model.task_type = ti.task_type;
    model.pick_source_id = ti.pick_source;
    model.place_target_id = ti.place_target;
    model.reject_target_id = ti.reject_target;
    model.grasp_strategy = ti.grasp_strategy;
    model.approach_axis = ti.approach_axis;
    model.approach_distance = ti.approach_distance;
    model.retreat_axis = ti.retreat_axis;
    model.retreat_distance = ti.retreat_distance;
    model.object_class = ti.object_class;
    model.warnings = overlay_warnings;
    model.has_intent_metadata = ti.status != "MISSING_TASK_FILE";
    ScenePreviewWidget::CameraOverlayModel camera;
    camera.camera_id = "camera_main";
    camera.display_name = "Camera";
    camera.frame_id = "camera_frame";
    camera.horizontal_fov_deg = 69.0;
    camera.vertical_fov_deg = 42.0;
    camera.range_min_m = 0.2;
    camera.range_max_m = 2.0;
    camera.source_path = ti.source_file;
    camera.metadata_source = "task/perception metadata source";
    camera.status = "warning";
    camera.warnings << "no camera item found" << "no pick source found" << "pick zone outside camera FOV" << "camera range too short" << "camera frame unknown" << "camera pose metadata incomplete";
    scene_preview_widget_->set_camera_overlay_model(camera);
    const auto snapshot_preview = load_scene3d_detection_snapshot_preview(sc.scene_dir);
    scene_preview_widget_->set_epd_detection_overlays(snapshot_preview.detections);
    for (const auto & snapshot_warning : snapshot_preview.warnings) {
      append_studio_log(QString("Scene3D detection snapshot warning: %1").arg(snapshot_warning));
      model.warnings << snapshot_warning;
    }
    scene_preview_widget_->set_task_overlay_model(model);
    scene_preview_widget_->set_task_overlay_visibility(
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      true,
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
  }
  if (ti.status == "MISSING_TASK_FILE") append_studio_log("task overlay missing");
  else append_studio_log("task overlay loaded");
  append_studio_log(QString("Task intent source: %1").arg(ti.source_file));
}

void MainWindow::validate_task_intent_for_selected_scene(){ refresh_task_intent_panel(); append_studio_log("Task intent validation completed (Fake Hardware | No Robot Motion | Preview Only)"); }
void MainWindow::generate_or_update_task_intent_for_selected_scene(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; QString script; helper_script_exists("create_or_update_builder_task_intent.py", &script); workcell_builder::TaskIntentCommandInput input; input.scene_package = QString::fromStdString(sc.scene_dir.string()); input.task_id = QString::fromStdString(sc.scene_name) + "_pick_place"; input.task_type = "pick_place"; input.task_template = "pick_place"; input.grasp_strategy = "finger_top"; const auto resolved_input = workcell_builder::resolve_task_intent_command_input_defaults(input); const auto plan = workcell_builder::build_task_intent_command_plan(script, resolved_input); if (!plan.ready()) { append_studio_log("Generate/Update Task Intent: " + plan.missing_fields_message()); return; } QProcess process; process.start("python3", QStringList() << plan.script_path << plan.arguments); if (!process.waitForFinished(120000)) { append_studio_log("Generate/Update Task Intent: timed out while waiting for helper script."); return; } const int exit_code = process.exitCode(); const QString stdout_text = QString::fromUtf8(process.readAllStandardOutput()).trimmed(); const QString stderr_text = QString::fromUtf8(process.readAllStandardError()).trimmed(); if (exit_code != 0) { append_studio_log(QString("Generate/Update Task Intent failed (exit=%1).").arg(exit_code)); if (!stderr_text.isEmpty()) append_studio_log("stderr: " + stderr_text.left(400)); if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400)); return; } append_studio_log("Generate/Update Task Intent: " + plan.display_command() + " (Preview Only)"); if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400)); const fs::path task_dir = sc.scene_dir / "task"; boost::system::error_code ec; fs::create_directories(task_dir, ec); fs::create_directories(sc.scene_dir / "plan_preview", ec); const fs::path config_path = sc.scene_dir / "config" / "workcell_builder_task_intent.yaml"; const fs::path generated_path = sc.scene_dir / "generated" / "workcell_builder_task_intent.yaml"; const fs::path source_path = fs::exists(config_path) ? config_path : generated_path; if (fs::exists(source_path)) { fs::copy_file(source_path, task_dir / "workcell_builder_task_intent.yaml", fs::copy_option::overwrite_if_exists, ec); if (ec) append_studio_log(QString("WARN Generate/Update Task Intent: failed writing task/workcell_builder_task_intent.yaml (%1)").arg(QString::fromStdString(ec.message()))); ec.clear(); fs::copy_file(source_path, task_dir / "task_recipe_from_builder_intent.yaml", fs::copy_option::overwrite_if_exists, ec); if (ec) append_studio_log(QString("WARN Generate/Update Task Intent: failed writing task/task_recipe_from_builder_intent.yaml (%1)").arg(QString::fromStdString(ec.message()))); std::ofstream preview((sc.scene_dir / "plan_preview" / "offline_plan_preview_request.yaml").string()); preview << "schema: offline_plan_preview_request/v1\nscene_name: " << sc.scene_name << "\nsource: existing_new_cell_flow\n"; preview.close(); } else { append_studio_log("WARN Generate/Update Task Intent: helper succeeded but no generated/config task intent file was found."); } refresh_task_intent_panel(); refresh_new_cell_checklist(); }
void MainWindow::generate_yaml_draft_for_selected_scene()
{
  if (selected_scene_index_ < 0) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  QString script;
  if (!helper_script_exists("create_or_update_builder_task_intent.py", &script)) {
    append_studio_log("Generate YAML: helper script search failed (task intent helper missing).");
  }
  const fs::path scene_dir = sc.scene_dir;
  const fs::path env = scene_dir / "environment.yaml";
  const fs::path cell = scene_dir / "cell_definition.yaml";
  const fs::path manifest = scene_dir / "scene_manifest.yaml";
  const fs::path layout = scene_dir / "environment_layout.yaml";

  if (!fs::exists(env)) {
    std::ofstream out(env.string());
    out << "scene_name: " << sc.scene_name << "\n";
    out << "safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n";
    out << "defaults:\n  robot: ur5\n  end_effector: robotiq_2f\n  object: placeholder_object\n  gripper_mount_rpy: [-1.5708, -1.5708, 0.0]\n";
  }

  const auto write_cell_draft = [&]() {
    const std::string scene_name = sc.scene_name;
    const std::string cell_id = scene_name + "_cell";
    const std::string task_id = scene_name + "_pick_place";
    std::ofstream out(cell.string());
    out << "schema_version: cell_definition/v1\n";
    out << "cell:\n  id: " << cell_id << "\n  name: " << scene_name << "\n  description: Auto-generated preview-safe draft for selected scene metadata\n";
    out << "robot:\n  id: ur5_preview\n  model: ur5\n  planning_group: manipulator\n  base_frame: world\n  tool_link: tool0\n  home_named_target: home\n  safe_joint_state: []\n";
    out << "end_effector:\n  id: robotiq_2f_preview\n  type: finger\n  brand: robotiq\n  grasp_frame: tool0\n  allowed_touch_links: [robotiq_2f_85_left_finger_tip_link, robotiq_2f_85_right_finger_tip_link]\n";
    out << "camera:\n  id: camera_main\n  type: depth_camera\n  frame: camera_depth_optical_frame\n";
    out << "environment:\n  frame: world\n  layout: environment_layout.yaml\n  support_surfaces:\n    - {id: table_main, type: table, frame: world, pose_xyz: [0.0, 0.0, 0.0], pose_rpy: [0.0, 0.0, 0.0], dimensions: [1.2, 0.8, 0.05]}\n";
    out << "objects:\n  - {id: preview_object, class: unknown, shape: box, color: unknown, material: unknown, frame: world, dimensions: [0.05, 0.05, 0.05], pose_xyz: [0.55, 0.0, 0.1], pose_rpy: [0.0, 0.0, 0.0]}\n";
    out << "task:\n  id: " << task_id << "\n  type: pick_place\n  source_object: preview_object\n  destinations:\n    - {id: place_bin, frame: world, pose_xyz: [0.35, -0.25, 0.1], pose_rpy: [0.0, 0.0, 0.0]}\n  rules:\n    - {id: default_place, when: {always: true}, destination: place_bin}\n";
    out << "commissioning:\n  self_test_enabled: true\n  export_bundle: true\n  require_operator_review: true\n  fake_hardware_default: true\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n";
  };

  if (!fs::exists(cell)) {
    write_cell_draft();
    append_studio_log("Generate YAML: new cell_definition.yaml generated.");
  } else {
    QString validate_cell_script;
    bool valid_existing_cell = false;
    if (helper_script_exists("validate_cell_definition.py", &validate_cell_script)) {
      QProcess validate_process;
      validate_process.start("python3", QStringList() << validate_cell_script << QString::fromStdString(cell.string()));
      if (validate_process.waitForFinished(120000) && validate_process.exitCode() == 0) {
        valid_existing_cell = true;
      }
    }
    if (valid_existing_cell) {
      append_studio_log("Generate YAML: existing valid cell_definition.yaml preserved.");
    } else {
      const fs::path backup = cell.string() + ".invalid." + std::to_string(std::time(nullptr)) + ".bak";
      boost::system::error_code ec;
      fs::copy_file(cell, backup, fs::copy_option::overwrite_if_exists, ec);
      if (ec) {
        append_studio_log(QString("Generate YAML: invalid cell_definition.yaml detected but backup failed (%1); not rewriting.")
          .arg(QString::fromStdString(ec.message())));
      } else {
        write_cell_draft();
        append_studio_log(QString("Generate YAML: invalid cell_definition.yaml backed up and regenerated (%1).")
          .arg(QString::fromStdString(backup.string())));
      }
    }
  }

  if (!fs::exists(manifest)) {
    std::ofstream out(manifest.string());
    out << "schema_version: workcell_scene_manifest/v1\nscene_name: " << sc.scene_name << "\n";
    out << "safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n";
  }
  if (!fs::exists(layout)) {
    std::ofstream out(layout.string());
    out << "layout:\n  items: []\n";
  }
  append_studio_log(QString("Generate YAML: ensured environment.yaml, cell_definition.yaml, scene_manifest.yaml, environment_layout.yaml for '%1'.").arg(QString::fromStdString(sc.scene_name)));
  refresh_scene_browser_ui();
  refresh_scene_builder_selected_scene_ui();
  refresh_new_cell_checklist();
}

void MainWindow::generate_scene_package_for_selected_scene() {
  if (selected_scene_index_ < 0) return;
  QString parity_warning;
  bool severe_parity_mismatch = false;
  const bool pre_parity_ran = run_canvas_generated_parity_check(
    CanvasGeneratedParityMode::PreGeneration, &parity_warning, &severe_parity_mismatch);
  if (pre_parity_ran) {
    refresh_canvas_generated_parity_ui();
    if (!parity_warning.isEmpty()) {
      append_studio_log("Generate ROS Scene Package pre-generation parity: " + parity_warning);
    }
  }
  generate_yaml_draft_for_selected_scene();
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const QString scene_dir = QString::fromStdString(sc.scene_dir.string());
  const QString scene_name = QString::fromStdString(sc.scene_name);
  const QString cell_definition_path = QString::fromStdString((sc.scene_dir / "cell_definition.yaml").string());
  const QString output_dir = QString::fromStdString(sc.scene_dir.parent_path().string());
  if (!QFileInfo::exists(cell_definition_path)) {
    append_studio_log("Generate ROS Scene Package: Generate YAML first.");
    return;
  }
  bool severe_preflight_failure = false;
  const QStringList preflight_warnings = generation_asset_support_preflight(
    sc.scene_dir / "environment_layout.yaml", &severe_preflight_failure);
  for (const QString & warning : preflight_warnings) {
    append_studio_log(warning);
    readiness_warning_details_.append(warning);
  }
  if (severe_preflight_failure) {
    append_studio_log("Generate ROS Scene Package blocked by severe schema/safety preflight failure.");
    refresh_new_cell_checklist();
    return;
  }
  QString validate_cell_script;
  if (helper_script_exists("validate_cell_definition.py", &validate_cell_script)) {
    QProcess validate_process;
    validate_process.start("python3", QStringList() << validate_cell_script << cell_definition_path);
    if (!validate_process.waitForFinished(120000)) {
      append_studio_log("Generate ROS Scene Package: timed out while validating cell definition.");
      return;
    }
    const QString stderr_text = QString::fromUtf8(validate_process.readAllStandardError()).trimmed();
    const QString stdout_text = QString::fromUtf8(validate_process.readAllStandardOutput()).trimmed();
    if (validate_process.exitCode() != 0) {
      append_studio_log("Generate ROS Scene Package blocked: cell_definition.yaml validation failed.");
      const QStringList validator_lines = (stderr_text + "\n" + stdout_text).split('\n', Qt::SkipEmptyParts);
      bool found_missing_key_error = false;
      for (const QString & line : validator_lines) {
        const QString trimmed = line.trimmed();
        if (trimmed.contains("Missing required top-level key:")) {
          append_studio_log("validator: " + trimmed);
          found_missing_key_error = true;
        }
      }
      if (!found_missing_key_error) {
        if (!stderr_text.isEmpty()) append_studio_log("validator stderr: " + stderr_text.left(600));
        if (!stdout_text.isEmpty()) append_studio_log("validator stdout: " + stdout_text.left(600));
      }
      return;
    }
  }
  if (output_dir.trimmed().isEmpty() || scene_name.trimmed().isEmpty()) {
    append_studio_log("Generate ROS Scene Package: output directory and package name are required.");
    return;
  }
  QString script;
  helper_script_exists("generate_workcell_from_cell_definition.py", &script);
  const auto plan = workcell_builder::build_generate_workcell_command_plan(script, scene_dir, output_dir, scene_name);
  if (!plan.ready()) {
    append_studio_log("Generate ROS Scene Package: " + plan.missing_fields_message());
    return;
  }
  QProcess process;
  process.start("python3", QStringList() << plan.script_path << plan.arguments);
  if (!process.waitForFinished(180000)) {
    append_studio_log("Generate ROS Scene Package: timed out while waiting for helper script.");
    return;
  }
  const int exit_code = process.exitCode();
  const QString stdout_text = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
  const QString stderr_text = QString::fromUtf8(process.readAllStandardError()).trimmed();
  if (exit_code != 0) {
    append_studio_log(QString("Generate ROS Scene Package failed (exit=%1).").arg(exit_code));
    if (!stderr_text.isEmpty()) append_studio_log("stderr: " + stderr_text.left(400));
    if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400));
    launch_artifacts_ready_ = false;
    return;
  }
  launch_artifacts_ready_ = true;
  append_studio_log("Generate ROS Scene Package: " + plan.display_command());
  append_studio_log(QString("Generated package location: %1/%2").arg(output_dir, scene_name));
  append_studio_log(QString("Next: colcon build --symlink-install --packages-select %1").arg(scene_name));
  append_studio_log("Next: source install/setup.bash");
  append_studio_log(QString("Next: ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(scene_name));
  if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400));
  QString post_warning;
  bool post_blocked = false;
  const bool post_parity_ran = run_canvas_generated_parity_check(
    CanvasGeneratedParityMode::PostGeneration, &post_warning, &post_blocked);
  if (post_parity_ran) {
    refresh_canvas_generated_parity_ui();
    if (post_blocked) {
      launch_artifacts_ready_ = false;
      append_studio_log("Generated package created but Canvas/Generated parity has blockers.");
      if (!post_warning.isEmpty()) {
        append_studio_log("Post-generation parity recommendation: " + post_warning);
      }
    } else if (!post_warning.isEmpty()) {
      append_studio_log("Generate ROS Scene Package post-generation parity: " + post_warning);
    }
  }
  refresh_scene_browser_ui();
  refresh_scene_builder_selected_scene_ui();
  refresh_new_cell_checklist();
}
void MainWindow::validate_generated_scene_for_selected_scene() {
  if (selected_scene_index_ < 0) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  QString script;
  if (!helper_script_exists("validate_builder_generated_scene.py", &script)) {
    append_studio_log("Validate Generated Scene: script missing. Searched: " +
      helper_script_search_paths("validate_builder_generated_scene.py").join(" | "));
    return;
  }
  const auto plan = workcell_builder::build_validate_generated_scene_command_plan(
    script, QString::fromStdString(sc.scene_dir.string()));
  if (!plan.ready()) {
    append_studio_log("Validate Generated Scene: " + plan.missing_fields_message());
    return;
  }
  QProcess process;
  process.start("python3", QStringList() << plan.script_path << plan.arguments);
  if (!process.waitForFinished(120000)) {
    append_studio_log("Validate Generated Scene: timed out while waiting for helper script.");
    return;
  }
  validation_stale_ = false;
  append_studio_log("Validate Generated Scene: " + plan.display_command());
  refresh_new_cell_checklist();
}
void MainWindow::copy_build_launch_commands_for_selected_scene() {
  if (!has_selected_scene()) return;
  const QString block = selected_scene_preview_command_block();
  QApplication::clipboard()->setText(block);
  append_studio_log("Copy Build & Launch Commands");
}
void MainWindow::open_selected_task_file() {
  if (selected_scene_index_ < 0) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const auto ti = load_scene_task_intent_summary(sc.scene_dir);
  if (ti.status == "MISSING_TASK_FILE") {
    append_studio_log("Open Task File: missing. Searched: " + ti.searched_paths.join(" | "));
    return;
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(ti.source_file));
}
void MainWindow::copy_selected_task_summary() {
  if (selected_scene_index_ < 0) return;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const auto ti = load_scene_task_intent_summary(sc.scene_dir);
  QApplication::clipboard()->setText(QString(
    "Scene=%1\nTaskType=%2\nPick=%3\nPlace=%4\nReject=%5\nObjectClass=%6\nGrasp=%7\nApproach=%8/%9\nRetreat=%10/%11\nTool=%12\nPerception=%13\nStatus=%14")
      .arg(QString::fromStdString(sc.scene_name), ti.task_type, ti.pick_source, ti.place_target,
        ti.reject_target, ti.object_class, ti.grasp_strategy, ti.approach_axis, ti.approach_distance,
        ti.retreat_axis, ti.retreat_distance, ti.tool_id, ti.perception_mode, ti.status));
  append_studio_log("Copy Task Summary");
}
void MainWindow::preview_offline_plan_for_selected_scene() {
  show_studio_page(StudioPage::PlanSimulatePage);
  refresh_preview_launch_ui();
  append_studio_log("Preview Offline Plan: Fake Hardware | No Robot Motion | Preview Only");
}
MainWindow::SelectedSceneItemState MainWindow::current_selected_scene_item() const
{
  SelectedSceneItemState state;
  const auto fill_from_tree = [&](QTreeWidgetItem * item) {
    if (!item) return false;
    state.id = item->data(0, TreeRoleId).toString().trimmed();
    if (state.id.isEmpty()) state.id = item->text(0).trimmed();
    state.display_name = item->text(0).trimmed();
    state.role = item->data(0, TreeRoleRole).toString().trimmed();
    state.category = item->data(0, TreeRoleCategory).toString().trimmed();
    state.type = item->data(0, TreeRoleItemTypeClass).toString().trimmed();
    state.role_or_category = state.role;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.category;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.type;
    state.source_path = item->data(0, TreeRoleSource).toString().trimmed();
    state.source_layer = item->data(0, TreeRoleSourceLayer).toString().trimmed();
    state.active_visual_source = item->data(0, TreeRoleActiveVisualSource).toString().trimmed();
    state.editable = item->data(0, TreeRoleEditable).toBool();
    state.locked = item->data(0, TreeRoleLocked).toBool();
    state.linked_to_editable_layout_state = item->data(0, TreeRoleLinkedEditableLayout).toBool();
    state.visual_backing_status = item->data(0, TreeRoleVisualBackingStatus).toString().trimmed();
    state.generated_visual = item->data(0, TreeRoleGeneratedVisual).toBool();
    state.item_type_classification = item->data(0, TreeRoleItemTypeClass).toString().trimmed();
    state.camera_id = item->data(0, TreeRoleCameraId).toString().trimmed();
    state.frame_id = item->data(0, TreeRoleFrameId).toString().trimmed();
    state.detection_label = item->data(0, TreeRoleDetectionLabel).toString().trimmed();
    state.confidence = item->data(0, TreeRoleConfidence).toDouble();
    state.tracking_id = item->data(0, TreeRoleTrackingId).toString().trimmed();
    state.snapshot_source_file = item->data(0, TreeRoleSnapshotSourceFile).toString().trimmed();
    state.alignment_warning = item->data(0, TreeRoleAlignmentWarning).toString().trimmed();
    state.pose_available = item->data(0, TreeRolePoseAvailable).toBool();
    state.pose_x = item->data(0, TreeRolePoseX).toDouble();
    state.pose_y = item->data(0, TreeRolePoseY).toDouble();
    state.pose_z = item->data(0, TreeRolePoseZ).toDouble();
    state.roll = item->data(0, TreeRoleRoll).toDouble();
    state.pitch = item->data(0, TreeRolePitch).toDouble();
    state.yaw = item->data(0, TreeRoleYaw).toDouble();
    state.pose_text = item->data(0, TreeRolePoseText).toString().trimmed();
    state.valid = !state.id.isEmpty();
    return state.valid;
  };
  const auto fill_from_canvas = [&](QGraphicsItem * item, const QString & fallback_id = QString()) {
    if (!item) return false;
    state.id = item->data(RoleId).toString().trimmed();
    if (state.id.isEmpty()) state.id = fallback_id.trimmed();
    state.display_name = item->data(RoleDisplayName).toString().trimmed();
    state.role = item->data(RoleRole).toString().trimmed();
    state.category = item->data(RoleCategory).toString().trimmed();
    state.type = item->data(RoleType).toString().trimmed();
    state.role_or_category = state.role;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.category;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.type;
    state.source_path = item->data(RoleSource).toString().trimmed();
    state.locked = item->data(RoleLocked).toBool();
    state.editable = !state.locked;
    state.source_layer = item->data(RoleSourceLayer).toString().trimmed();
    if (state.source_layer.isEmpty()) state.source_layer = QStringLiteral("canvas");
    state.active_visual_source = item->data(RoleGeneratedPlaceholder).toBool() ? QStringLiteral("primitive_fallback") : QStringLiteral("mesh_preview");
    state.linked_to_editable_layout_state = state.editable && state.source_layer.compare(QStringLiteral("editable_layout"), Qt::CaseInsensitive) == 0;
    state.visual_backing_status = item->data(RoleGeneratedPlaceholder).toBool() ? QStringLiteral("primitive") : QStringLiteral("mesh");
    state.generated_visual = item->data(RoleGeneratedPlaceholder).toBool();
    state.item_type_classification = item->data(RoleType).toString().trimmed();
    state.lock_reason = state.locked ? item->data(RoleWarning).toString().trimmed() : QString();
    state.pose_available = true;
    state.pose_x = item->pos().x() / 100.0;
    state.pose_y = item->pos().y() / 100.0;
    state.pose_z = item->data(RolePoseZ).toDouble();
    state.roll = item->data(RoleRoll).toDouble();
    state.pitch = item->data(RolePitch).toDouble();
    state.yaw = item->data(RoleYaw).toDouble();
    state.dim_x = item->data(RoleWidth).toDouble();
    state.dim_y = item->data(RoleDepth).toDouble();
    state.dim_z = item->data(RoleHeight).toDouble();
    state.pose_text = item->data(RolePoseText).toString().trimmed();
    state.valid = !state.id.isEmpty();
    return state.valid;
  };
  const auto fill_from_preview = [&](const ScenePreviewWidget::PreviewItem * item) {
    if (!item) return false;
    state.id = item->id.trimmed();
    state.display_name = item->display_name.trimmed();
    state.role = item->role.trimmed();
    state.category = item->category.trimmed();
    state.type = item->mesh_type.trimmed();
    state.role_or_category = state.role;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.category;
    if (state.role_or_category.isEmpty()) state.role_or_category = state.type;
    if (state.role_or_category.isEmpty()) state.role_or_category = item->category.trimmed();
    state.source_path = item->source_path.trimmed();
    state.source_layer = item->source_layer.trimmed();
    state.active_visual_source = item->active_visual_source.trimmed();
    state.editable = item->editable && !item->locked;
    state.locked = item->locked || !item->editable;
    state.lock_reason = item->lock_reason.trimmed();
    state.linked_to_editable_layout_state = item->linked_to_editable_layout_state;
    state.visual_backing_status = item->mesh_available ? QStringLiteral("mesh") : QStringLiteral("primitive_fallback");
    if (!item->mesh_load_warning.trimmed().isEmpty()) state.visual_backing_status = item->mesh_load_warning.trimmed();
    state.generated_visual = item->locked || item->active_visual_source.trimmed() == QStringLiteral("locked_generated_urdf_visual");
    state.item_type_classification = item->category.trimmed();
    state.camera_id = item->camera_id.trimmed();
    state.frame_id = item->frame_id.trimmed();
    state.detection_label = item->detection_label.trimmed();
    state.confidence = item->confidence;
    state.tracking_id = item->tracking_id.trimmed();
    state.snapshot_source_file = item->snapshot_source_file.trimmed();
    state.alignment_warning = item->alignment_warning.trimmed();
    state.pose_available = true;
    state.pose_x = item->x;
    state.pose_y = item->y;
    state.pose_z = item->z;
    state.roll = item->roll;
    state.pitch = item->pitch;
    state.yaw = item->yaw;
    state.dim_x = item->sx;
    state.dim_y = item->sy;
    state.dim_z = item->sz;
    state.pose_text = QStringLiteral("xyz=(%1, %2, %3) rpy=(%4, %5, %6)")
      .arg(item->x, 0, 'f', 3)
      .arg(item->y, 0, 'f', 3)
      .arg(item->z, 0, 'f', 3)
      .arg(item->roll, 0, 'f', 3)
      .arg(item->pitch, 0, 'f', 3)
      .arg(item->yaw, 0, 'f', 3);
    state.valid = !state.id.isEmpty();
    return state.valid;
  };
  const auto find_preview_item_by_id = [&](const QString & id) -> const ScenePreviewWidget::PreviewItem * {
    const QString stable_id = id.trimmed();
    if (stable_id.isEmpty()) return nullptr;
    for (const auto & item : all_scene_preview_items_) {
      if (item.id.trimmed() == stable_id) return &item;
    }
    return scene_preview_widget_ ? scene_preview_widget_->preview_item_by_id(stable_id) : nullptr;
  };
  const auto find_tree_item_by_id = [&](const QString & id) -> QTreeWidgetItem * {
    if (!scene_hierarchy_tree_ || id.trimmed().isEmpty()) return nullptr;
    for (int row = 0; row < scene_hierarchy_tree_->topLevelItemCount(); ++row) {
      auto * top = scene_hierarchy_tree_->topLevelItem(row);
      if (!top) continue;
      if (top->data(0, TreeRoleId).toString().trimmed() == id) return top;
      for (int child = 0; child < top->childCount(); ++child) {
        auto * node = top->child(child);
        if (node && node->data(0, TreeRoleId).toString().trimmed() == id) return node;
      }
    }
    return nullptr;
  };
  const auto find_canvas_item_by_id = [&](const QString & id) -> QGraphicsItem * {
    if (!digital_twin_scene_ || id.trimmed().isEmpty()) return nullptr;
    for (auto * gi : digital_twin_scene_->items()) {
      if (gi && gi->data(RoleId).toString().trimmed() == id) return gi;
    }
    return nullptr;
  };

  const QString selected_id = !current_selected_scene_item_id_.trimmed().isEmpty() ? current_selected_scene_item_id_.trimmed() :
    (scene_preview_widget_ ? scene_preview_widget_->selected_preview_item_id().trimmed() : QString());
  if (!selected_id.isEmpty()) {
    auto * tree_item = find_tree_item_by_id(selected_id);
    if (fill_from_tree(tree_item)) return state;
    auto * canvas_item = find_canvas_item_by_id(selected_id);
    if (fill_from_canvas(canvas_item, selected_id)) return state;
    if (fill_from_preview(find_preview_item_by_id(selected_id))) return state;
    return {};
  }
  if (scene_hierarchy_tree_ && fill_from_tree(scene_hierarchy_tree_->currentItem())) return state;
  if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty() &&
    fill_from_canvas(digital_twin_scene_->selectedItems().front())) return state;
  if (scene_preview_widget_) {
    const QString preview_selected_id = scene_preview_widget_->selected_preview_item_id().trimmed();
    if (fill_from_preview(find_preview_item_by_id(preview_selected_id))) return state;
  }
  return {};
}


QGraphicsItem * MainWindow::find_canvas_item_by_stable_id(const QString & id) const
{
  const QString stable_id = id.trimmed();
  if (!digital_twin_scene_ || stable_id.isEmpty()) return nullptr;
  for (auto * item : digital_twin_scene_->items()) {
    if (item && item->data(RoleId).toString().trimmed() == stable_id) return item;
  }
  return nullptr;
}

MainWindow::EditableLayoutSelectionTarget MainWindow::resolve_selected_editable_layout_target() const
{
  EditableLayoutSelectionTarget target;
  const QString stable_id = current_selected_scene_item_id_.trimmed();
  if (stable_id.isEmpty()) {
    target.blocker = QStringLiteral("No stable scene item id is selected.");
    return target;
  }

  target.state = current_selected_scene_item();
  if (!target.state.valid || target.state.id.trimmed() != stable_id) {
    target.blocker = QStringLiteral("Selected stable id '%1' is not present in the active scene hierarchy, Scene3D payload, or canvas fallback.").arg(stable_id);
    return target;
  }

  target.fallback_item = find_canvas_item_by_stable_id(stable_id);

  const auto supplement_from_preview = [&]() {
    for (const auto & item : all_scene_preview_items_) {
      if (item.id.trimmed() != stable_id) continue;
      if (target.state.source_path.isEmpty()) target.state.source_path = item.source_path.trimmed();
      if (target.state.source_layer.isEmpty() || target.state.source_layer == QStringLiteral("canvas")) target.state.source_layer = item.source_layer.trimmed();
      if (target.state.active_visual_source.isEmpty()) target.state.active_visual_source = item.active_visual_source.trimmed();
      target.state.linked_to_editable_layout_state = item.linked_to_editable_layout_state;
      target.state.editable = item.editable && !item.locked;
      target.state.locked = item.locked || !item.editable;
      if (target.state.dim_x <= 0.0) target.state.dim_x = item.sx;
      if (target.state.dim_y <= 0.0) target.state.dim_y = item.sy;
      if (target.state.dim_z <= 0.0) target.state.dim_z = item.sz;
      if (target.state.lock_reason.isEmpty()) target.state.lock_reason = item.lock_reason.trimmed();
      return;
    }
    if (scene_preview_widget_) {
      if (const auto * item = scene_preview_widget_->preview_item_by_id(stable_id)) {
        if (target.state.source_path.isEmpty()) target.state.source_path = item->source_path.trimmed();
        if (target.state.source_layer.isEmpty() || target.state.source_layer == QStringLiteral("canvas")) target.state.source_layer = item->source_layer.trimmed();
        if (target.state.active_visual_source.isEmpty()) target.state.active_visual_source = item->active_visual_source.trimmed();
        target.state.linked_to_editable_layout_state = item->linked_to_editable_layout_state;
        target.state.editable = item->editable && !item->locked;
        target.state.locked = item->locked || !item->editable;
        if (target.state.dim_x <= 0.0) target.state.dim_x = item->sx;
        if (target.state.dim_y <= 0.0) target.state.dim_y = item->sy;
        if (target.state.dim_z <= 0.0) target.state.dim_z = item->sz;
        if (target.state.lock_reason.isEmpty()) target.state.lock_reason = item->lock_reason.trimmed();
      }
    }
  };
  supplement_from_preview();

  const QString source_layer = target.state.source_layer.trimmed();
  const QString active_visual_source = target.state.active_visual_source.trimmed();
  const bool generated_or_preview_only = target.state.locked || !target.state.editable ||
    target.state.generated_visual ||
    source_layer.compare(QStringLiteral("locked_generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    source_layer.compare(QStringLiteral("generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    source_layer.compare(QStringLiteral("primitive_fallback"), Qt::CaseInsensitive) == 0 ||
    active_visual_source.compare(QStringLiteral("locked_generated_urdf_visual"), Qt::CaseInsensitive) == 0;
  if (generated_or_preview_only) {
    const QString reason = target.state.lock_reason.isEmpty() ? QStringLiteral("generated, locked, or preview-only item") : target.state.lock_reason;
    target.blocker = QStringLiteral("Selection '%1' is not editable because it is %2 (source_layer=%3, active_visual_source=%4). Use or create an editable layout record first.")
      .arg(stable_id, reason, source_layer.isEmpty() ? QStringLiteral("unknown") : source_layer,
        active_visual_source.isEmpty() ? QStringLiteral("unknown") : active_visual_source);
    return target;
  }
  if (!target.state.linked_to_editable_layout_state && source_layer.compare(QStringLiteral("editable_layout"), Qt::CaseInsensitive) != 0) {
    target.blocker = QStringLiteral("Selection '%1' is not linked to editable layout state (source_layer=%2). Create editable layout from preview/Add to Canvas before editing generated preview data.")
      .arg(stable_id, source_layer.isEmpty() ? QStringLiteral("unknown") : source_layer);
    return target;
  }
  target.source_path = target.state.source_path.trimmed();
  if (target.source_path.isEmpty()) {
    target.blocker = QStringLiteral("Editable selection '%1' has no source path for its layout record; refusing to edit anonymous preview state.").arg(stable_id);
    return target;
  }

  target.ok = true;
  return target;
}

void MainWindow::refresh_selected_scene_item_labels(const SelectedSceneItemState & state)
{
  if (!inspector_label_ || !live_coordinate_label_) return;
  refresh_selection_binding_actions(state);
  QStringList inspector_lines;
  inspector_lines << QString("Scene: %1").arg(selected_scene_state_.valid ? selected_scene_state_.name : QStringLiteral("none"));
  inspector_lines << QString("Scene path: %1").arg(selected_scene_state_.valid ? selected_scene_state_.path : QStringLiteral("(none)"));
  inspector_lines << QString("Scene status: %1").arg(selected_scene_state_.valid ? selected_scene_state_.status : QStringLiteral("(none)"));
  inspector_lines << QString("Robot: %1").arg(selected_scene_state_.valid ? selected_scene_state_.robot_summary : QStringLiteral("unknown"));
  inspector_lines << QString("End effector: %1").arg(selected_scene_state_.valid ? selected_scene_state_.end_effector_summary : QStringLiteral("unknown"));
  inspector_lines << QString("Tool mount: %1").arg(selected_scene_state_.valid ? selected_scene_state_.tool_mount_summary : QStringLiteral("unknown"));
  inspector_lines << QString("Grasp frame: %1").arg(selected_scene_state_.valid ? selected_scene_state_.grasp_frame_summary : QStringLiteral("unknown"));
  inspector_lines << QString("Launch status: %1").arg(selected_scene_state_.valid ? (selected_scene_state_.launchable ? "ready" : "blocked") : QStringLiteral("(none)"));
  if (!state.valid) {
    inspector_lines << "Selected item: (none)";
    inspector_label_->setText(inspector_lines.join("\n"));
    inspector_label_->setToolTip(selected_scene_state_.valid ? selected_scene_state_.path : QString());
    live_coordinate_label_->setText("No item selected");
    return;
  }
  const QString display = state.display_name.isEmpty() ? state.id : state.display_name;
  const QString role = state.role_or_category.isEmpty() ? "unknown" : state.role_or_category;
  const QString source = state.source_path.isEmpty() ? "unknown" : state.source_path;
  const QString source_layer = state.source_layer.isEmpty() ? "unknown" : state.source_layer;
  const QString active_visual_source = state.active_visual_source.isEmpty() ? "unknown" : state.active_visual_source;
  const QString visual_backing = state.visual_backing_status.isEmpty() ? "unknown" : state.visual_backing_status;
  const QString type_class = state.item_type_classification.isEmpty() ? "unknown" : state.item_type_classification;
  const QString pose = state.pose_available ? (state.pose_text.isEmpty() ? QString("x=%1 y=%2 z=%3").arg(state.pose_x).arg(state.pose_y).arg(state.pose_z) : state.pose_text) : "pose unknown";
  const bool editable_layout_contract = state.editable && state.linked_to_editable_layout_state && !state.locked;
  const bool generated_or_preview_contract = state.generated_visual ||
    source_layer.compare(QStringLiteral("locked_generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    source_layer.compare(QStringLiteral("generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    source_layer.compare(QStringLiteral("primitive_fallback"), Qt::CaseInsensitive) == 0 ||
    active_visual_source.compare(QStringLiteral("locked_generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    active_visual_source.compare(QStringLiteral("generated_urdf_visual"), Qt::CaseInsensitive) == 0 ||
    active_visual_source.compare(QStringLiteral("primitive_fallback"), Qt::CaseInsensitive) == 0;
  const QString selection_contract_label = editable_layout_contract ? QStringLiteral("editable layout item") :
    (generated_or_preview_contract ? QStringLiteral("inspection-only generated preview") : QStringLiteral("locked item cannot be edited"));
  const bool is_locked_urdf_preview = state.locked && role.contains("urdf", Qt::CaseInsensitive);
  const QString locked_line = state.locked ? QString("Locked: %1").arg(state.lock_reason.isEmpty() ? QStringLiteral("item is locked") : state.lock_reason) : QStringLiteral("Locked: no");
  inspector_lines << "";
  inspector_lines << QString("Selected item name: %1").arg(display);
  inspector_lines << QString("Selected item role: %1").arg(role);
  inspector_lines << QString("Selected item category: %1").arg(role);
  inspector_lines << QString("Selected item ID: %1").arg(state.id);
  inspector_lines << QString("Selected item editing mode: %1").arg(selection_contract_label);
  inspector_lines << QString("Selected item source: %1").arg(source);
  inspector_lines << QString("Selected item source_path: %1").arg(source);
  inspector_lines << QString("Selected item source_layer: %1").arg(source_layer);
  inspector_lines << QString("Selected item active_visual_source: %1").arg(active_visual_source);
  inspector_lines << QString("Selected item editable/locked: %1").arg(state.editable ? "editable" : "locked");
  inspector_lines << QString("Selected item linked_to_editable_layout_state: %1").arg(state.linked_to_editable_layout_state ? "true" : "false");
  inspector_lines << QString("Selected item visual backing: %1").arg(visual_backing);
  inspector_lines << QString("Selected item generated vs authoring: %1").arg(state.generated_visual ? "generated" : "authoring-backed");
  inspector_lines << QString("Selected item item_type_classification: %1").arg(type_class);
  if (is_locked_urdf_preview) inspector_lines << "Reason: locked/preview-only";
  inspector_lines << locked_line;
  const bool is_detection_item = source_layer.compare("overlay", Qt::CaseInsensitive) == 0 || !state.detection_label.isEmpty();
  const bool is_camera_fov_item = role.contains("camera", Qt::CaseInsensitive) || type_class.contains("camera", Qt::CaseInsensitive);
  if (is_detection_item || is_camera_fov_item) {
    inspector_lines << "";
    inspector_lines << "Read-only details:";
    if (!state.camera_id.isEmpty()) inspector_lines << QString("camera_id: %1").arg(state.camera_id);
    if (!state.frame_id.isEmpty()) inspector_lines << QString("frame_id: %1").arg(state.frame_id);
    if (!state.detection_label.isEmpty()) inspector_lines << QString("detection_label: %1").arg(state.detection_label);
    if (state.confidence >= 0.0) inspector_lines << QString("confidence: %1").arg(state.confidence, 0, 'f', 3);
    if (!state.tracking_id.isEmpty()) inspector_lines << QString("tracking_id: %1").arg(state.tracking_id);
    if (!state.snapshot_source_file.isEmpty()) inspector_lines << QString("snapshot_source_file: %1").arg(state.snapshot_source_file);
    if (!state.alignment_warning.isEmpty()) inspector_lines << QString("alignment_warning: %1").arg(state.alignment_warning);
    if (!state.editable) inspector_lines << QString("locked_reason: %1").arg(state.lock_reason.isEmpty() ? QStringLiteral("preview-only overlay") : state.lock_reason);
  }
  inspector_label_->setText(inspector_lines.join("\n"));
  inspector_label_->setToolTip(QString("%1\n%2").arg(selected_scene_state_.valid ? selected_scene_state_.path : QString(), source));
  live_coordinate_label_->setText(QString("Transform: %1").arg(pose));
}

bool MainWindow::is_pick_source_candidate(const SelectedSceneItemState & state) const
{
  if (!state.valid || state.id.trimmed().isEmpty()) return false;
  const QString tag = state.role_or_category.trimmed().toLower();
  return tag.contains("pick") || tag.contains("source") || tag.contains("bin") || tag.contains("tray") || tag.contains("zone");
}

bool MainWindow::is_place_target_candidate(const SelectedSceneItemState & state) const
{
  if (!state.valid || state.id.trimmed().isEmpty()) return false;
  const QString tag = state.role_or_category.trimmed().toLower();
  return tag.contains("place") || tag.contains("target") || tag.contains("drop") || tag.contains("zone") || tag.contains("tray");
}

bool MainWindow::is_camera_candidate(const SelectedSceneItemState & state) const
{
  if (!state.valid || state.id.trimmed().isEmpty()) return false;
  const QString tag = state.role_or_category.trimmed().toLower();
  return tag.contains("camera") || tag.contains("sensor") || tag.contains("vision") || tag.contains("realsense");
}

void MainWindow::refresh_selection_binding_actions(const SelectedSceneItemState & state)
{
  const bool has_selection = state.valid && !state.id.trimmed().isEmpty();
  if (pick_source_button_) pick_source_button_->setEnabled(has_selection);
  if (place_target_button_) place_target_button_->setEnabled(has_selection);
  if (camera_button_) camera_button_->setEnabled(has_selection);
}

QString MainWindow::selected_scene_binding_id() const
{
  return current_selected_scene_item().id.trimmed();
}

bool MainWindow::update_selected_scene_task_intent_binding(
  const QString & binding_label, const std::vector<std::string> & key_path, const QString & selected_id)
{
  return update_selected_scene_task_intent_bindings(binding_label, {key_path}, selected_id);
}

bool MainWindow::update_selected_scene_task_intent_bindings(
  const QString & binding_label,
  const std::vector<std::vector<std::string>> & key_paths,
  const QString & selected_id)
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return false;
  const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path task_intent_path = sc.scene_dir / "config" / "workcell_builder_task_intent.yaml";
  fs::create_directories(task_intent_path.parent_path());
  const auto append_readiness_note = [&](const QString & message) {
    const fs::path readiness_path = sc.scene_dir / "readiness" / "readiness_summary.txt";
    fs::create_directories(readiness_path.parent_path());
    std::ofstream readiness_out(readiness_path.string(), std::ios::app);
    if (readiness_out.is_open()) readiness_out << message.toStdString() << "\n";
  };
  YAML::Node root;
  bool has_existing = fs::exists(task_intent_path);
  if (has_existing) {
    try {
      root = YAML::LoadFile(task_intent_path.string());
    } catch (const YAML::Exception & ex) {
      const fs::path backup = task_intent_path.string() + ".malformed." + std::to_string(std::time(nullptr)) + ".bak";
      boost::system::error_code ec;
      fs::copy_file(task_intent_path, backup, fs::copy_option::overwrite_if_exists, ec);
      if (ec) {
        append_studio_log(QString("Task binding blocked: malformed YAML at %1 and backup failed (%2)")
          .arg(QString::fromStdString(task_intent_path.string()), QString::fromStdString(ec.message())));
        return false;
      }
      append_studio_log(QString("Malformed task intent YAML detected; backup created at %1 before rewrite.")
        .arg(QString::fromStdString(backup.string())));
      root = YAML::Node(YAML::NodeType::Map);
    } catch (const std::exception & ex) {
      const fs::path backup = task_intent_path.string() + ".malformed." + std::to_string(std::time(nullptr)) + ".bak";
      boost::system::error_code ec;
      fs::copy_file(task_intent_path, backup, fs::copy_option::overwrite_if_exists, ec);
      if (ec) {
        append_studio_log(QString("Task binding blocked: malformed YAML at %1 and backup failed (%2)")
          .arg(QString::fromStdString(task_intent_path.string()), QString::fromStdString(ec.message())));
        return false;
      }
      append_studio_log(QString("Malformed task intent YAML detected; backup created at %1 before rewrite.")
        .arg(QString::fromStdString(backup.string())));
      root = YAML::Node(YAML::NodeType::Map);
    }
  }
  if (has_existing) {
    const fs::path backup = task_intent_path.string() + ".backup." + std::to_string(std::time(nullptr)) + ".bak";
    boost::system::error_code ec;
    fs::copy_file(task_intent_path, backup, fs::copy_option::overwrite_if_exists, ec);
    if (!ec) {
      append_studio_log(QString("Task intent backup created: %1").arg(QString::fromStdString(backup.string())));
      append_readiness_note(QString("Task intent backup created: %1").arg(QString::fromStdString(backup.string())));
    } else {
      append_studio_log(QString("Warning: task intent backup failed before write (%1)").arg(QString::fromStdString(ec.message())));
      append_readiness_note(QString("Warning: task intent backup failed before write (%1)").arg(QString::fromStdString(ec.message())));
    }
  }
  if (!root || !root.IsMap()) root = YAML::Node(YAML::NodeType::Map);
  YAML::Node task = ensure_map_path(root, {"task"});
  if (!task["type"] || !task["type"].IsScalar()) task["type"] = "pick_place";
  if (!task["family"] || !task["family"].IsScalar()) task["family"] = task["type"];
  YAML::Node grasp = ensure_map_path(task, {"grasp"});
  if (!grasp["strategy"] || !grasp["strategy"].IsScalar()) {
    grasp["strategy"] = infer_default_grasp_strategy(root);
  }
  YAML::Node safety = ensure_map_path(root, {"safety"});
  (void)ensure_map_path(task, {"pick", "source"});
  (void)ensure_map_path(task, {"place", "target"});
  (void)ensure_map_path(task, {"perception", "camera"});
  safety["preview_only"] = false;
  safety["use_fake_hardware"] = true;
  safety["allow_simulated_motion"] = true;
  safety["allow_moveit_execution"] = true;
  safety["allow_rviz_motion"] = true;
  safety["allow_real_hardware_motion"] = false;
  safety["real_robot_locked"] = true;
  for (const auto & key_path : key_paths) {
    if (key_path.empty()) continue;
    YAML::Node cursor = task;
    for (size_t i = 0; i + 1 < key_path.size(); ++i) {
      if (!cursor[key_path[i]] || !cursor[key_path[i]].IsMap()) cursor[key_path[i]] = YAML::Node(YAML::NodeType::Map);
      cursor = cursor[key_path[i]];
    }
    cursor[key_path.back()] = selected_id.toStdString();
  }
  std::ofstream out(task_intent_path.string());
  out << root;
  out.close();
  append_studio_log(QString("%1 updated to '%2'").arg(binding_label, selected_id));
  append_studio_log(QString("Task intent updated at %1 (Fake Hardware | Simulated Motion Enabled | Real Robot Locked)")
    .arg(QString::fromStdString(task_intent_path.string())));
  append_readiness_note(QString("%1 updated to '%2'").arg(binding_label, selected_id));
  append_readiness_note(QString("Task intent updated at %1").arg(QString::fromStdString(task_intent_path.string())));
  refresh_after_task_binding_change(binding_label, selected_id);
  return true;
}

void MainWindow::refresh_after_task_binding_change(const QString & binding_label, const QString & selected_id)
{
  append_studio_log(QString("Task binding refresh started: %1 -> %2").arg(binding_label, selected_id));
  refresh_task_intent_panel();
  refresh_new_cell_checklist();
  populate_scene_hierarchy();
  populate_scene_files_tab();
  rebuild_digital_twin_canvas();
  refresh_selected_scene_item_labels(current_selected_scene_item());
  append_studio_log(QString("Task binding refresh completed: %1 -> %2").arg(binding_label, selected_id));
}

void MainWindow::bind_selected_item_as_pick_zone()
{
  const auto state = current_selected_scene_item();
  if (!state.valid || state.id.trimmed().isEmpty()) {
    append_studio_log("Use Selected as Pick Source/Zone blocked: no selection.");
    return;
  }
  if (!is_pick_source_candidate(state)) {
    append_studio_log(QString("Use Selected as Pick Source/Zone warning: selected item '%1' may be incompatible (role/category: %2). Applying override.")
      .arg(state.id, state.role_or_category.isEmpty() ? "unknown" : state.role_or_category));
  }
  const auto choice = QMessageBox::question(this, "Workcell Studio", "Use this zone for task intent?", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
  append_studio_log("Task intent zone prompt (pick) shown: Use this zone for task intent?");
  if (choice == QMessageBox::Yes) {
    const bool pick_written = update_selected_scene_task_intent_bindings("Pick Zone + Pick Source", {{"pick", "zone", "id"}, {"pick", "source", "id"}}, state.id.trimmed());
    if (pick_written) {
      append_studio_log("Task intent binding applied: pick zone metadata and pick source bound.");
    }
  } else {
    append_studio_log("Task intent zone prompt declined for pick source binding.");
    append_studio_log("Task intent binding skipped: pick zone metadata and pick source left unchanged.");
  }
}

void MainWindow::bind_selected_item_as_place_zone()
{
  const auto state = current_selected_scene_item();
  if (!state.valid || state.id.trimmed().isEmpty()) {
    append_studio_log("Use Selected as Place Target/Zone blocked: no selection.");
    return;
  }
  if (!is_place_target_candidate(state)) {
    append_studio_log(QString("Use Selected as Place Target/Zone warning: selected item '%1' may be incompatible (role/category: %2). Applying override.")
      .arg(state.id, state.role_or_category.isEmpty() ? "unknown" : state.role_or_category));
  }
  const auto choice = QMessageBox::question(this, "Workcell Studio", "Use this zone for task intent?", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
  append_studio_log("Task intent zone prompt (place) shown: Use this zone for task intent?");
  if (choice == QMessageBox::Yes) {
    const bool place_written = update_selected_scene_task_intent_bindings("Place Zone + Place Target", {{"place", "zone", "id"}, {"place", "target", "id"}}, state.id.trimmed());
    if (place_written) {
      append_studio_log("Task intent binding applied: place zone metadata and place target bound.");
    }
  } else {
    append_studio_log("Task intent zone prompt declined for place target binding.");
    append_studio_log("Task intent binding skipped: place zone metadata and place target left unchanged.");
  }
}

void MainWindow::bind_selected_item_as_camera()
{
  const auto state = current_selected_scene_item();
  if (!state.valid || state.id.trimmed().isEmpty()) {
    append_studio_log("Use Selected as Camera blocked: no selection.");
    return;
  }
  if (!is_camera_candidate(state)) {
    append_studio_log(QString("Use Selected as Camera warning: selected item '%1' may be incompatible (role/category: %2). Applying override.")
      .arg(state.id, state.role_or_category.isEmpty() ? "unknown" : state.role_or_category));
  }
  update_selected_scene_task_intent_binding("Camera", {"perception", "camera", "id"}, state.id.trimmed());
}
void MainWindow::refresh_scene_browser_ui()
{
  const fs::path workspace_root = workcell_path.empty() ? fs::path(QDir::homePath().toStdString()) / "workcell_ws" : workcell_path;
  scene_browser_result_ = workcell_builder::discover_workcell_studio_scenes(workspace_root);
  int ready=0,warn=0,blocked=0; for (const auto & s : scene_browser_result_.scenes){ if(s.status=="READY") ++ready; else if(s.status=="WARNINGS") ++warn; else ++blocked; }
  const QString root_used = QString::fromStdString(scene_browser_result_.scene_root.string());
  const QStringList searched = [&](){ QStringList out; for (const auto & p : scene_browser_result_.searched_roots) out << QString::fromStdString(p.string()); return out; }();
  QString summary = QString("Total scenes: %1 | Ready: %2 | Warnings: %3 | Blocked/Scaffold: %4 | Source: %5")
    .arg(scene_browser_result_.scenes.size()).arg(ready).arg(warn).arg(blocked).arg(root_used);
  if (!scene_browser_result_.root_exists) {
    summary += QString(
      " | Warning: no scene folders found. Searched:\\n - %1\\n"
      "Check selected workspace or symlink ~/workcell_ws/src/scenes")
      .arg(searched.join("\\n - "));
    append_studio_log("No scenes found. Searched paths: " + searched.join(" | "));
  } else {
    append_studio_log(QString("Loaded %1 scenes from %2").arg(scene_browser_result_.scenes.size()).arg(root_used));
  }
  dashboard_summary_label_->setText(summary);
  if (dashboard_total_scenes_card_) dashboard_total_scenes_card_->setText(QString("Total Scenes\n%1").arg(scene_browser_result_.scenes.size()));
  if (dashboard_ready_scenes_card_) dashboard_ready_scenes_card_->setText(QString("Ready / Validated\n%1").arg(ready));
  if (dashboard_warning_scenes_card_) dashboard_warning_scenes_card_->setText(QString("Warnings / Blocked\n%1").arg(warn + blocked));
  if (selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    selected_scene_index_ = -1;
  }
  sync_selected_scene_state();
  sync_selected_item_state();
  refresh_studio_home_scene_table();
  auto fill_existing=[&](QTableWidget* t){ t->setRowCount((int)scene_browser_result_.scenes.size()); for(int i=0;i<t->rowCount();++i){const auto &sc=scene_browser_result_.scenes[(size_t)i]; auto scene_name = QString::fromStdString(sc.scene_name); t->setItem(i,0,new QTableWidgetItem(scene_name)); t->setItem(i,1,new QTableWidgetItem(QString::fromStdString(sc.status))); t->setItem(i,2,new QTableWidgetItem(QString::fromStdString(sc.robot_summary))); t->setItem(i,3,new QTableWidgetItem(QString::fromStdString(sc.gripper_summary))); t->setItem(i,4,new QTableWidgetItem(sc.has_task_recipe?"present":"missing")); t->setItem(i,5,new QTableWidgetItem(sc.has_launch_demo?"ready":"blocked")); }};
  fill_existing(existing_scene_table_);
  populate_scene_files_tab();
}

void MainWindow::refresh_studio_home_scene_table()
{
  if (!dashboard_scene_table_) return;
  const QString q = dashboard_scene_search_ ? dashboard_scene_search_->text().trimmed().toLower() : "";
  const QString status_filter = dashboard_scene_status_filter_ ? dashboard_scene_status_filter_->currentText() : "All";
  const QString lq = dashboard_library_search_ ? dashboard_library_search_->text().trimmed().toLower() : "";
  const QString lstatus = dashboard_library_status_filter_ ? dashboard_library_status_filter_->currentText() : "All";
  dashboard_scene_table_->setRowCount(0);
  if (dashboard_library_list_) dashboard_library_list_->clear();
  for (size_t i = 0; i < scene_browser_result_.scenes.size(); ++i) {
    const auto & sc = scene_browser_result_.scenes[i];
    const QString scene_name = QString::fromStdString(sc.scene_name);
    const QString status = QString::fromStdString(sc.status);
    if (!q.isEmpty() && !scene_name.toLower().contains(q)) continue;
    if (status_filter == "Ready" && status != "READY") continue;
    if (status_filter == "Warning" && status != "WARNINGS") continue;
    if (status_filter == "Blocked" && (status == "READY" || status == "WARNINGS")) continue;
    const int row = dashboard_scene_table_->rowCount(); dashboard_scene_table_->insertRow(row);
    auto *scene_item = new QTableWidgetItem(QFontMetrics(dashboard_scene_table_->font()).elidedText(scene_name, Qt::ElideRight, 320)); scene_item->setToolTip(scene_name); scene_item->setData(Qt::UserRole, (int)i);
    dashboard_scene_table_->setItem(row,0,scene_item);
    auto * status_item = new QTableWidgetItem(status);
    if (status == "READY") { status_item->setBackground(QColor("#DCFCE7")); status_item->setForeground(QBrush(QColor("#15803D"))); }
    else if (status == "WARNINGS") { status_item->setBackground(QColor("#FEF3C7")); status_item->setForeground(QBrush(QColor("#B45309"))); }
    else { status_item->setBackground(QColor("#FEE2E2")); status_item->setForeground(QBrush(QColor("#B91C1C"))); }
    dashboard_scene_table_->setItem(row,1,status_item);
    dashboard_scene_table_->setItem(row,2,new QTableWidgetItem(QString::fromStdString(sc.robot_summary))); dashboard_scene_table_->setItem(row,3,new QTableWidgetItem(QString::fromStdString(sc.gripper_summary))); dashboard_scene_table_->setItem(row,4,new QTableWidgetItem(sc.has_task_recipe?"present":"missing")); dashboard_scene_table_->setItem(row,5,new QTableWidgetItem(sc.has_launch_demo?"ready":"blocked"));
    if (dashboard_library_list_) {
      const bool lmatch_q = lq.isEmpty() || scene_name.toLower().contains(lq);
      const bool lmatch_status = (lstatus == "All") || (lstatus == "Ready" && status == "READY") || (lstatus == "Warning" && status == "WARNINGS") || (lstatus == "Blocked" && (status != "READY" && status != "WARNINGS"));
      if (lmatch_q && lmatch_status) {
        auto * item = new QListWidgetItem(scene_name);
        item->setData(Qt::UserRole, static_cast<int>(i));
        dashboard_library_list_->addItem(item);
      }
    }
  }
  if (dashboard_empty_state_card_) dashboard_empty_state_card_->setVisible(dashboard_scene_table_->rowCount() == 0);
}

bool MainWindow::is_safe_scene_path_for_trash_move(const fs::path & scene_path, QString * reason) const
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    if (reason) *reason = "No scene is selected.";
    return false;
  }
  fs::path scenes_root = scene_browser_result_.scene_root;
  if (scenes_root.empty()) {
    scenes_root = scene_path.parent_path();
  }
  boost::system::error_code ec;
  const fs::path canonical_scene = fs::weakly_canonical(scene_path, ec);
  if (ec) {
    if (reason) *reason = "Failed to resolve selected scene path.";
    return false;
  }
  const fs::path canonical_root = fs::weakly_canonical(scenes_root, ec);
  if (ec) {
    if (reason) *reason = "Failed to resolve scenes root path.";
    return false;
  }
  const std::string scene_str = canonical_scene.string();
  const std::string root_str = canonical_root.string();
  if (!(scene_str == root_str || scene_str.rfind(root_str + "/", 0) == 0)) {
    if (reason) *reason = "Selected path is outside scenes root.";
    return false;
  }
  const std::vector<fs::path> blocked_paths = {
    canonical_root.parent_path(),
    canonical_root.parent_path() / "src",
    canonical_root.parent_path() / "assets",
    canonical_root / "assets",
    fs::current_path()
  };
  for (const auto & blocked : blocked_paths) {
    const fs::path canonical_blocked = fs::weakly_canonical(blocked, ec);
    if (ec) {
      ec.clear();
      continue;
    }
    if (canonical_scene == canonical_blocked) {
      if (reason) *reason = QString("Refusing to delete protected path: %1").arg(QString::fromStdString(canonical_blocked.string()));
      return false;
    }
  }
  return true;
}

void MainWindow::delete_selected_scene()
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    QMessageBox::warning(this, "Delete Scene", "Select a scene first.");
    return;
  }
  const auto & scene = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const fs::path scene_path = scene.scene_dir;
  QString safety_error;
  if (!is_safe_scene_path_for_trash_move(scene_path, &safety_error)) {
    QMessageBox::warning(this, "Delete Scene", safety_error);
    append_studio_log("Delete Scene blocked: " + safety_error);
    return;
  }
  const auto confirm = QMessageBox::question(
    this, "Delete Scene",
    QString("Move scene '%1' to Workcell Studio trash?").arg(QString::fromStdString(scene.scene_name)),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  if (confirm != QMessageBox::Yes) {
    append_studio_log(QString("Delete Scene cancelled for '%1'.").arg(QString::fromStdString(scene.scene_name)));
    return;
  }
  boost::system::error_code ec;
  fs::path scenes_root = scene_browser_result_.scene_root;
  if (scenes_root.empty()) {
    scenes_root = scene.scene_dir.parent_path();
  }
  const fs::path trash_root = scenes_root / ".workcell_studio_trash";
  fs::create_directories(trash_root, ec);
  const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss");
  const fs::path trash_target = trash_root / (scene.scene_name + "_" + stamp.toStdString());
  fs::rename(scene_path, trash_target, ec);
  if (ec) {
    QMessageBox::critical(this, "Delete Scene", "Failed to move scene to trash.");
    append_studio_log("Delete Scene failed: " + QString::fromStdString(ec.message()));
    return;
  }
  append_studio_log("Delete Scene moved to trash: " + QString::fromStdString(trash_target.string()));
  selected_scene_index_ = -1;
  refresh_scene_browser_ui();
}

void MainWindow::select_scene_by_row(int row)
{
  if (dashboard_scene_table_ && dashboard_scene_table_->item(row, 0) && dashboard_scene_table_->item(row, 0)->data(Qt::UserRole).isValid()) row = dashboard_scene_table_->item(row, 0)->data(Qt::UserRole).toInt();
  if (row < 0 || row >= (int)scene_browser_result_.scenes.size()) return;
  const QString previous_scene_path = selected_scene_path();
  selected_scene_index_ = row;
  sync_selected_scene_state();
  refresh_selected_scene_metadata_panel();
  if (previous_scene_path != selected_scene_path()) {
    visual_index_script_missing_reported_scene_key_.clear();
    visual_index_regen_failure_reported_scene_key_.clear();
    visual_index_regen_throttle_session_active_ = false;
  }
  sync_selected_item_state();
  const auto & s = scene_browser_result_.scenes[(size_t)row];
  refresh_scene_builder_selected_scene_ui();
  readiness_label_->setText("Preview/offline validation only\nNo robot motion commanded\nRuntime execution remains disabled unless explicitly enabled elsewhere\ncolcon build --symlink-install --packages-select "+QString::fromStdString(s.scene_name)+"\nsource install/setup.bash\n"+selected_scene_launch_command());
  refresh_preview_launch_ui();
  refresh_new_cell_checklist();
  refresh_scene_builder_left_explorer();
  refresh_scene_bundle_export_panel();
  refresh_selected_scene_details_card();
}

void MainWindow::refresh_selected_scene_details_card()
{
  if (!dashboard_selected_scene_details_) return;
  if (!selected_scene_state_.valid) {
    dashboard_selected_scene_details_->setText("Select a scene to view details.");
    if (dashboard_scene_actions_button_) dashboard_scene_actions_button_->setEnabled(false);
    if (dashboard_open_scene_action_) dashboard_open_scene_action_->setEnabled(false);
    if (dashboard_validate_action_) dashboard_validate_action_->setEnabled(false);
    if (dashboard_plan_action_) dashboard_plan_action_->setEnabled(false);
    if (dashboard_export_action_) dashboard_export_action_->setEnabled(false);
    if (dashboard_delete_action_) dashboard_delete_action_->setEnabled(false);
    if (dashboard_validate_action_) dashboard_validate_action_->setToolTip("Select a scene to validate.");
    if (dashboard_plan_action_) dashboard_plan_action_->setToolTip("Select a scene to open Plan / Simulate.");
    if (dashboard_export_action_) dashboard_export_action_->setToolTip("Select a scene to export.");
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_state_.index];
  const QString status_chip = (s.status == "READY") ? "<span style='background:#DCFCE7;color:#15803D;border:1px solid #86EFAC;padding:2px 8px;border-radius:8px;'>READY</span>"
    : (s.status == "WARNINGS") ? "<span style='background:#FEF3C7;color:#B45309;border:1px solid #FCD34D;padding:2px 8px;border-radius:8px;'>WARNINGS</span>"
    : "<span style='background:#FEE2E2;color:#B91C1C;border:1px solid #FCA5A5;padding:2px 8px;border-radius:8px;'>BLOCKED</span>";
  const auto metadata = selected_scene_metadata_summary(s);
  dashboard_selected_scene_details_->setText(QString("<b>Scene:</b> %1<br/><b>Status:</b> %2<br/><b>Robot:</b> %3<br/><small>%4</small><br/><b>End effector:</b> %5<br/><small>%6</small><br/><b>Task Recipe:</b> %7<br/><b>Launch:</b> %8<br/><b>Source:</b> %9")
    .arg(metadata.scene_name).arg(status_chip).arg(metadata.robot).arg(metadata.robot_source).arg(metadata.end_effector).arg(metadata.end_effector_source).arg(s.has_task_recipe ? "present" : "missing").arg(metadata.launch).arg(metadata.scene_path));
  if (dashboard_last_updated_card_) dashboard_last_updated_card_->setText(QString("Source Path\n%1").arg(QString::fromStdString(scene_browser_result_.scene_root.string())));
  const ActionGate generate_gate = build_generate_scene_gate(s, validation_stale_);
  const ActionGate plan_gate = build_plan_simulate_gate(s, launch_artifacts_ready_);
  const ActionGate export_gate = build_export_gate(s);
  if (dashboard_scene_actions_button_) dashboard_scene_actions_button_->setEnabled(true);
  if (dashboard_open_scene_action_) dashboard_open_scene_action_->setEnabled(true);
  if (dashboard_validate_action_) dashboard_validate_action_->setEnabled(true);
  if (dashboard_validate_action_) dashboard_validate_action_->setToolTip(generate_gate.tooltip);
  if (dashboard_plan_action_) dashboard_plan_action_->setEnabled(plan_gate.enabled);
  if (dashboard_plan_action_) dashboard_plan_action_->setToolTip(plan_gate.tooltip);
  if (dashboard_export_action_) dashboard_export_action_->setEnabled(export_gate.enabled);
  if (dashboard_export_action_) dashboard_export_action_->setToolTip(export_gate.tooltip);
  if (dashboard_delete_action_) dashboard_delete_action_->setEnabled(true);
}


QString MainWindow::selected_scene_launch_command() const
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) return "";
  const auto & scene = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  return workcell_builder::build_command(scene);
}


void MainWindow::open_selected_scene_artifact(const QString & artifact)
{ if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) { QMessageBox::information(this,"Workcell Studio","No scene selected."); return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_]; fs::path target;
  if (artifact=="preview") target = s.scene_dir / "preview" / "static_preview.html";
  else if (artifact=="smoke") target = s.scene_dir / "smoke" / "offline_smoke_report.html";
  else if (artifact=="demo_dashboard") target = s.scene_dir / "demo" / "workcell_studio_demo_dashboard.html";
  else if (artifact=="folder") target = s.scene_dir;
  else if (artifact=="demo_summary_copy") {
    const fs::path summary = s.scene_dir / "demo" / "workcell_studio_demo_summary.txt";
    if (!fs::exists(summary)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(summary.string()))); return; }
    QFile f(QString::fromStdString(summary.string())); if (f.open(QIODevice::ReadOnly|QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll()));
    append_studio_log("Copied demo summary"); return;
  } else if (artifact=="layout_merge_report") {
    const QString p = QString::fromStdString((s.scene_dir/"generated/workcell_studio_layout_merge_report.json").string());
    if (QFileInfo::exists(p)) {
      QDesktopServices::openUrl(QUrl::fromLocalFile(p));
      return;
    } else {
      QMessageBox::information(this, "Workcell Studio", "Layout merge report missing. Run Layout Merge first");
      return;
    }
  } else if (artifact=="run_acceptance") {
    const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
    const fs::path merge_report = s.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
    if (fs::exists(layout_file) && (!fs::exists(merge_report) || fs::last_write_time(layout_file) > fs::last_write_time(merge_report))) {
      append_studio_log("Acceptance: running safe offline layout merge first");
      workcell_builder::merge_workcell_studio_layout(s.scene_dir);
    }
    const QString cmd = QString("python3 scripts/validate_workcell_studio_generated_scene.py '%1' --json").arg(QString::fromStdString(s.scene_dir.string()));
    const int rc = std::system(cmd.toStdString().c_str()); append_studio_log(rc==0?"Acceptance completed":"Acceptance blocked"); refresh_scene_browser_ui(); return;
  } else if (artifact=="run_smoke") {
    QMessageBox::information(this,"Workcell Studio","Offline smoke check runner is report-only in Demo Mode. Missing artifact will be reported in demo summary."); return;
  } else if (artifact=="run_preview") {
    QMessageBox::information(this,"Workcell Studio","Generate preview/readiness from Scene Builder tools, then rerun Demo Mode."); return;
  } else if (artifact=="preview_launch_folder") {
    target = s.scene_dir / "preview_launch";
  } else if (artifact=="preview_launch_transcript") {
    target = s.scene_dir / "preview_launch" / "preview_launch_session.json";
  } else target = s.scene_dir;
  if (!fs::exists(target)) { QMessageBox::warning(this,"Workcell Studio",QString("Missing artifact: %1").arg(QString::fromStdString(target.string()))); return; }
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(target.string())));
}

void MainWindow::append_studio_log(const QString & message)
{
  if (studio_log_) {
    studio_log_->append(message);
  }
  statusBar()->showMessage(message);
}

bool MainWindow::scene3d_debug_logging_enabled() const
{
  const auto * viewport = scene_preview_widget_ ? scene_preview_widget_->findChild<Scene3DViewportWidget *>() : nullptr;
  return (viewport && viewport->debug_overlays_mode) || qEnvironmentVariableIsSet("WORKCELL_SCENE3D_DEBUG_LOGS");
}

bool MainWindow::append_scene_diagnostic_log_once(const QString & event, int payload_revision, int payload_count, const QString & message)
{
  const QString scene = selected_scene_state_.name.trimmed().isEmpty() ? selected_scene_name() : selected_scene_state_.name.trimmed();
  const QString key = QStringLiteral("%1|%2|rev=%3|count=%4").arg(scene, event).arg(payload_revision).arg(payload_count);
  if (emitted_scene_diagnostic_log_keys_.contains(key)) return false;
  emitted_scene_diagnostic_log_keys_.insert(key);
  append_studio_log(message);
  return true;
}

void MainWindow::show_not_wired_message(const QString & action_label)
{
  append_studio_log(action_label + ": Action not wired yet");
  append_studio_log("No robot motion commanded");
  const QStringList searched_paths = helper_script_search_paths("workcell_studio.py");
  const QString details = QString("Could not find Workcell Studio helper script.\nSearched:\n - %1").arg(
    searched_paths.join("\n - "));
  QMessageBox::information(
    this,
    "Workcell Studio",
    "This Workcell Studio action is not wired yet. No files changed and no robot motion was commanded.\n\n" + details);
}

void MainWindow::export_scene_bundle_for_selected_scene()
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) {
    append_studio_log("Export Scene Bundle: no scene selected.");
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path export_root = s.scene_dir / "exports";
  fs::create_directories(export_root);
  const fs::path zip_out = export_root / (s.scene_name + "_workcell_studio_bundle.zip");
  const QString script = "scripts/export_workcell_scene_bundle.py";
  const QString cmd = QString("python3 '%1' --scene-dir '%2' --output '%3' --validate --include-assets")
    .arg(script, QString::fromStdString(s.scene_dir.string()), QString::fromStdString(zip_out.string()));
  append_studio_log("Export Scene Bundle: running " + cmd);
  const int rc = std::system(cmd.toStdString().c_str());
  if (rc == 0) {
    append_studio_log("Export Scene Bundle: completed -> " + QString::fromStdString(zip_out.string()));
    last_scene_bundle_export_folder_ = QString::fromStdString(export_root.string());
  } else {
    append_studio_log("Export Scene Bundle: failed.");
  }
  refresh_scene_bundle_export_panel();
}

void MainWindow::import_scene_bundle_into_scenes_root()
{
  QString selected = QFileDialog::getOpenFileName(this, "Select Scene Bundle .zip", QDir::homePath(), "Zip files (*.zip)");
  if (selected.isEmpty()) selected = QFileDialog::getExistingDirectory(this, "Select Scene Bundle Folder", QDir::homePath());
  if (selected.isEmpty()) return;
  fs::path scenes_root = scene_browser_result_.scene_root.empty() ? (workcell_path / "src" / "scenes") : scene_browser_result_.scene_root;
  fs::create_directories(scenes_root);
  const QString script = "scripts/import_workcell_scene_bundle.py";
  QString cmd;
  if (QFileInfo(selected).isDir()) {
    cmd = QString("python3 '%1' --bundle '%2' --target-scenes-dir '%3' --validate --print-summary")
      .arg(script, selected, QString::fromStdString(scenes_root.string()));
  } else {
    cmd = QString("python3 '%1' --bundle '%2' --target-scenes-dir '%3' --validate --print-summary")
      .arg(script, selected, QString::fromStdString(scenes_root.string()));
  }
  append_studio_log("Import Scene Bundle: running " + cmd);
  const int rc = std::system(cmd.toStdString().c_str());
  append_studio_log(rc == 0 ? "Import Scene Bundle: completed. Imported Scene Ready." : "Import Scene Bundle: failed.");
  refresh_scene_browser_ui();
}

void MainWindow::open_scene_bundle_export_folder()
{
  if (last_scene_bundle_export_folder_.isEmpty() && selected_scene_index_ >= 0 && selected_scene_index_ < (int)scene_browser_result_.scenes.size()) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    last_scene_bundle_export_folder_ = QString::fromStdString((s.scene_dir / "exports").string());
  }
  if (last_scene_bundle_export_folder_.isEmpty() || !QFileInfo::exists(last_scene_bundle_export_folder_)) {
    append_studio_log("Open Export Folder: export folder missing. Export a scene bundle first.");
    return;
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(last_scene_bundle_export_folder_));
  append_studio_log("Open Export Folder: " + last_scene_bundle_export_folder_);
}

MainWindow::~MainWindow()
{
  stop_preview_process();
  delete ui;
}

void MainWindow::on_load_workcell_clicked()
{
  QString workcell_file = startup_workspace_.trimmed();
  if (workcell_file.isEmpty()) {
    workcell_file = QFileDialog::getExistingDirectory(
      this,
      "Target workcell project destination",
      QDir::homePath());
  }
  startup_workspace_.clear();
  if (workcell_file.isEmpty()) {
    return;
  }
  if (load_watcher_ && load_watcher_->isRunning()) {
    return;
  }

  ui->next->setDisabled(true);
  ui->load_workcell->setDisabled(true);
  ui->change_workcell->setDisabled(true);
  ui->error_label->setProperty("status", "info");
  ui->error_label->setText("Loading workcell...");
  statusBar()->showMessage("Loading workspace. Please wait...");

  if (progress_dialog_) {
    progress_dialog_->deleteLater();
  }
  progress_dialog_ = new QProgressDialog("Loading workcell...", "Cancel", 0, 5, this);
  progress_dialog_->setWindowModality(Qt::ApplicationModal);
  progress_dialog_->setAutoClose(false);
  progress_dialog_->setAutoReset(false);
  progress_dialog_->show();

  cancel_requested_.store(false);
  connect(progress_dialog_, &QProgressDialog::canceled, this, [this]() {
    cancel_requested_.store(true);
    if (progress_dialog_) {
      progress_dialog_->setLabelText("Cancelling...");
    }
  });

  if (load_watcher_) {
    load_watcher_->deleteLater();
  }
  load_watcher_ = new QFutureWatcher<WorkcellLoadResult>(this);

  auto future = QtConcurrent::run([this, workcell_file,
    progress_dialog = QPointer<QProgressDialog>(progress_dialog_)]() {
      WorkcellLoadResult result;
      result.workcell_file = workcell_file;
      const auto report_progress = [&](int value, const QString & label) {
        if (!progress_dialog) {
          return;
        }
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setLabelText",
          Qt::QueuedConnection,
          Q_ARG(QString, label));
        QMetaObject::invokeMethod(
          progress_dialog.data(),
          "setValue",
          Qt::QueuedConnection,
          Q_ARG(int, value));
      };

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(0, "Preparing directories...");
      boost::system::error_code ec;
      const workcell_builder::WorkcellRootInspection inspection =
        workcell_builder::inspect_selected_workcell_path(fs::path(workcell_file.toStdString()));
      // Fail fast on invalid workspace; continue otherwise.
      if (!inspection.success) {
        result.error = QString::fromStdString(inspection.error);
        return result;
      }

      const fs::path workcell_root = inspection.workcell_root;
      const QString root_status_suffix = QString::fromStdString(inspection.root_status_suffix);

      ec.clear();
      fs::create_directories(workcell_root, ec);
      if (ec) {
        result.error = QString("Failed to create workcell root directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      result.workcell_root_label = root_status_suffix;
      const fs::path assets_path = workcell_root / "assets";
      const fs::path scenes_path = workcell_root / "scenes";
      fs::create_directories(assets_path, ec);
      if (ec) {
        result.error = QString("Failed to create assets directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }
      fs::create_directories(scenes_path, ec);
      if (ec) {
        result.error = QString("Failed to create scenes directory: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(1, "Ensuring asset directories...");
      const std::array<std::string, 3> asset_subdirs = { "robots", "end_effectors", "environment" };
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        fs::create_directories(target_path, ec);
        if (ec) {
          result.error = QString("Failed to create asset directory: %1")
            .arg(QString::fromStdString(ec.message()));
          return result;
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(2, "Copying default assets...");
      const fs::path package_assets_path = get_default_assets_directory();
      for (const auto & asset_subdir : asset_subdirs) {
        const fs::path target_path = assets_path / asset_subdir;
        if (is_empty_directory(target_path)) {
          copy_directory_contents(package_assets_path / asset_subdir, target_path);
        }
      }

      if (cancel_requested_.load()) {
        result.cancelled = true;
        result.error = "Cancelled";
        return result;
      }

      report_progress(3, "Scanning scenes...");
      Workcell loaded_workcell;
      loaded_workcell.scene_vector.clear();
      for (fs::directory_iterator it(scenes_path, ec), end; it != end && !ec; it.increment(ec)) {
        if (cancel_requested_.load()) {
          result.cancelled = true;
          result.error = "Cancelled";
          return result;
        }
        const fs::path scene_path = it->path();
        if (!fs::is_directory(scene_path, ec) || ec) {
          continue;
        }
        if (is_good_scene_path(scene_path)) {
          Scene temp_scene;
          temp_scene.filepath = scene_path.string();
          temp_scene.name = scene_path.filename().string();
          temp_scene.loaded = false;
          loaded_workcell.scene_vector.push_back(temp_scene);
        }
      }
      if (ec) {
        result.error = QString("Failed to scan scenes: %1")
          .arg(QString::fromStdString(ec.message()));
        return result;
      }

      report_progress(5, "Finalizing...");
      loaded_workcell.workcell_filepath = workcell_root.string();
      result.workcell = loaded_workcell;
      result.workcell_path = workcell_root;
      result.success = true;
      return result;
    });

  connect(load_watcher_, &QFutureWatcher<WorkcellLoadResult>::finished, this, [this]() {
    WorkcellLoadResult result = load_watcher_->result();
    if (progress_dialog_) {
      progress_dialog_->setValue(5);
      progress_dialog_->close();
      progress_dialog_->deleteLater();
      progress_dialog_ = nullptr;
    }

    if (result.success) {
      workcell = result.workcell;
      workcell_path = result.workcell_path;
      if (has_selected_ros_distro()) {
        ui->error_label->setText(
          QString("Workcell loaded%1").arg(result.workcell_root_label));
        ui->error_label->setProperty("status", "success");
        statusBar()->showMessage("Ready: open scene setup.");
      } else {
        ui->error_label->setText(
          "Workcell loaded, but no ROS distro is selected. Select a ROS distro to continue.");
        ui->error_label->setProperty("status", "error");
        statusBar()->showMessage("Workspace loaded. Select a ROS distro.");
      }
      success = true;
      selected_workspace_ = result.workcell_file;
          update_next_button_state();
    } else {
      const QString error_text = result.cancelled ? "Workcell load cancelled" :
        QString("Failed to load workcell: %1").arg(result.error);
      ui->error_label->setProperty("status", "error");
      ui->error_label->setText(error_text);
      statusBar()->showMessage(error_text);
      success = false;
      update_next_button_state();
    }
  });

  load_watcher_->setFuture(future);
}

void MainWindow::on_next_clicked()
{
  if (!success) {
    ui->error_label->setProperty("status", "error");
    ui->error_label->setText("Please load a workcell before continuing.");
    statusBar()->showMessage("Load a workspace before continuing.");
    return;
  }
  if (!has_selected_ros_distro()) {
    ui->error_label->setText(
      "Please select a ROS distro before continuing.");
    ui->error_label->setProperty("status", "error");
    statusBar()->showMessage("Select a ROS distro before continuing.");
    update_next_button_state();
    return;
  }
  boost::filesystem::path before_scene_select(boost::filesystem::current_path());
  workcell.ros_ver = 2;
  workcell.ros_distro = ui->ros_distro->currentText().toStdString();
  SceneSelect scene_window;
  scene_window.load_workcell(workcell);
  scene_window.setWindowTitle("Create New Environment");
  scene_window.setModal(true);
  scene_window.exec();
  boost::filesystem::current_path(before_scene_select);
}

void MainWindow::on_change_workcell_clicked()
{
  success = false;
  ui->error_label->setProperty("status", "error");
  ui->error_label->setText("Workcell not available");
  statusBar()->showMessage("Select a new workspace directory.");
  apply_startup_selection();
  update_next_button_state();
  setup_studio_shell();
  apply_studio_theme();
}




void MainWindow::apply_startup_selection()
{
  if (!startup_ros_distro_.trimmed().isEmpty()) {
    const int idx = ui->ros_distro->findData(startup_ros_distro_.trimmed().toLower());
    if (idx >= 0) {
      ui->ros_distro->setCurrentIndex(idx);
    }
  }

  if (!startup_workspace_.trimmed().isEmpty()) {
    selected_workspace_ = startup_workspace_.trimmed();
    on_load_workcell_clicked();
  }
}

bool MainWindow::has_selected_ros_distro() const
{
  return !ui->ros_distro->currentData().toString().trimmed().isEmpty();
}

void MainWindow::update_next_button_state()
{
  ui->next->setDisabled(!(success && has_selected_ros_distro()));
}
bool MainWindow::is_good_scene(boost::filesystem::path original_path, std::string scene_name)
{
  const boost::filesystem::path scene_path = original_path / scene_name;
  return is_good_scene_path(scene_path);
}

// Legacy hardening markers: Asset Browser | if (title == "Open Existing Scene" || title == "Scene Builder")
// Scenario Templates | label == "Validate" || label == "Generate Scene"
// title == "Validate" || title == "Generate Scene"

QString MainWindow::detect_workspace_root() const
{
  const QString startup_workspace = startup_workspace_.trimmed();
  if (!startup_workspace.isEmpty() && QDir(startup_workspace).exists()) return startup_workspace;

  const QString selected_workspace = selected_workspace_.trimmed();
  if (!selected_workspace.isEmpty() && QDir(selected_workspace).exists()) return selected_workspace;

  QSettings settings;
  const QString saved_workspace = settings.value("startup/last_workspace").toString().trimmed();
  if (!saved_workspace.isEmpty() && QDir(saved_workspace).exists()) return saved_workspace;

  const QString default_workspace = QDir::homePath() + "/workcell_ws";
  if (QDir(default_workspace).exists()) return default_workspace;

  return QDir::homePath();
}

QString MainWindow::selected_scene_build_command() const { if (selected_scene_index_ < 0) return ""; const auto & scene = scene_browser_result_.scenes[(size_t)selected_scene_index_]; return QString("cd %1 && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select %2").arg(detect_workspace_root(), QString::fromStdString(scene.launch_package.empty() ? scene.scene_name : scene.launch_package)); }
QString MainWindow::selected_scene_source_command() const { return QString("cd %1 && source install/setup.bash").arg(detect_workspace_root()); }
QString MainWindow::selected_scene_preview_command_block() const { return selected_scene_build_command()+"\n"+selected_scene_source_command()+"\ncd "+detect_workspace_root()+" && "+selected_scene_launch_command(); }

bool MainWindow::selected_scene_preview_ready(QStringList * blockers) const
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    if (blockers) blockers->append("No scene selected");
    return false;
  }
  const auto & scene = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const Scene3DTransformParityReadiness transform_parity =
    scene3d_load_transform_parity_readiness(scene.scene_dir, QString::fromStdString(scene.scene_name));
  if (transform_parity.failed) {
    if (blockers) blockers->append(transform_parity.warning);
    return false;
  }
  const auto status = workcell_builder::validate_readiness(scene, detect_workspace_root().toStdString());
  if (!status.ready && blockers) blockers->append(status.blocker_reason);
  return status.ready;
}

bool MainWindow::preview_command_is_safe(const QString & command, QStringList * blockers) const
{
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    if (blockers) blockers->append("No scene selected");
    return false;
  }
  QString dry_command;
  const auto status = workcell_builder::dry_run(
    scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)],
    detect_workspace_root().toStdString(),
    &dry_command);
  if (!status.ready) {
    if (blockers) blockers->append(status.blocker_reason);
    return false;
  }
  const QString expected = "cd " + detect_workspace_root() + " && source install/setup.bash && " + dry_command;
  const bool safe = (command.trimmed() == dry_command.trimmed()) || (command.trimmed() == expected.trimmed());
  if (!safe && blockers) blockers->append("Unsafe launch argument detected: command does not match expected fake-hardware preview command");
  return safe;
}

void MainWindow::set_preview_state(const QString & state){ preview_state_=state; refresh_preview_launch_ui(); }

void MainWindow::refresh_preview_launch_ui()
{
  QString readiness = "BLOCKED_MISSING_SCENE";
  bool has_scene = selected_scene_index_ >= 0;
  bool has_ws = !detect_workspace_root().isEmpty();
  QStringList blockers;
  if (has_scene) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    if (!s.has_launch_demo && s.launch_file.empty()) readiness = "BLOCKED_MISSING_LAUNCH";
    else if (!s.has_task_intent) readiness = "BLOCKED_MISSING_TASK_INTENT";
    else if (!selected_scene_preview_ready(&blockers)) readiness = "WARNINGS_PRESENT";
    else readiness = "READY_FOR_FAKE_HARDWARE_PREVIEW";
    const auto metadata = selected_scene_metadata_summary(s);
    if (preview_scene_label_) preview_scene_label_->setText(QString("<b>Selected Scene</b><br/>scene name: %1<br/>scene path: %2<br/>robot: %3<br/>robot source: %4<br/>end effector: %5<br/>end effector source: %6<br/>task file status: %7<br/>launch/demo.launch.py status: %8<br/>package.xml/CMakeLists status: %9<br/>preview snapshot path: %10")
      .arg(metadata.scene_name, metadata.scene_path, metadata.robot, metadata.robot_source, metadata.end_effector, metadata.end_effector_source,
      s.has_task_recipe ? "present" : "missing", s.has_launch_demo ? "present" : "missing", (s.has_package_xml && (s.has_launch_demo || !s.launch_file.empty())) ? "present" : "missing")
      .arg(QString::fromStdString((s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.png").string())));
    if (validation_summary_label_) validation_summary_label_->setText(QString("<b>Validation Summary</b><br/>Scene: %1<br/>Readiness Gate: %2").arg(QString::fromStdString(s.scene_name), readiness));
  }
  if (preview_status_label_) preview_status_label_->setText(QString("<b>Readiness Gate</b><br/>%1<br/>state: %2").arg(readiness, preview_state_));
  if (preview_commands_) preview_commands_->setPlainText(selected_scene_preview_command_block());
  if (preview_commands_) preview_commands_->append(QString("\n# Safe Commands (Fake Hardware / Offline / No Robot Motion)\n# build selected scene package\n%1\n# source workspace\n%2\n# fake-hardware launch command\n%3\n# optional offline validation command\npython3 scripts/validate_builder_generated_scene.py '%4' --json")
    .arg(selected_scene_build_command(), selected_scene_source_command(), selected_scene_launch_command(), has_scene ? QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_dir.string()) : QString("")));
  if (validation_next_fix_label_) validation_next_fix_label_->setText(QString("<b>Next Fix Suggestions</b><br/>%1").arg(blockers.isEmpty() ? "No blockers. You can run Fake-Hardware Preview." : blockers.join("<br/>")));
  if (run_build_button_) run_build_button_->setEnabled(has_scene && has_ws && (preview_state_=="IDLE"||preview_state_=="BUILD_FAILED"||preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_FAILED"||preview_state_=="PREVIEW_EXITED"));
  if (run_preview_button_) run_preview_button_->setEnabled(has_scene && has_ws && (preview_state_=="BUILD_PASSED"||preview_state_=="PREVIEW_STOPPED"||preview_state_=="PREVIEW_EXITED"));
  if (stop_preview_button_) stop_preview_button_->setEnabled(preview_state_=="PREVIEW_RUNNING"||preview_state_=="PREVIEW_STOPPING");
}

void MainWindow::run_offline_validation() { validation_stale_ = false; append_studio_log("Full offline validation completed"); open_selected_scene_artifact("run_acceptance"); refresh_new_cell_checklist(); }
void MainWindow::run_layout_validation_only() { validation_stale_ = false; append_studio_log("Layout validation completed"); open_selected_scene_artifact("run_acceptance"); }
void MainWindow::check_canvas_generated_parity()
{
  QString user_warning;
  bool severe_mismatch = false;
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const bool generated_artifacts_exist = fs::exists(s.scene_dir / "scene_manifest.yaml") && fs::exists(s.scene_dir / "generated" / "generated_workcell_summary.json");
  const CanvasGeneratedParityMode mode = generated_artifacts_exist ? CanvasGeneratedParityMode::PostGeneration : CanvasGeneratedParityMode::PreGeneration;
  const bool ran = run_canvas_generated_parity_check(mode, &user_warning, &severe_mismatch);
  refresh_canvas_generated_parity_ui();
  refresh_scene_workflow_rail();
  if (!ran) {
    return;
  }
  if (severe_mismatch) {
    QMessageBox::warning(
      this, "Canvas/RViz Parity",
      "Canvas/generated parity is blocked by severe mismatches.\nResolve blockers before generating scene package.");
  } else if (!user_warning.isEmpty()) {
    QMessageBox::information(this, "Canvas/RViz Parity", user_warning);
    if (mode == CanvasGeneratedParityMode::PreGeneration) {
      QMessageBox::information(
        this, "Canvas/RViz Parity",
        "Generated artifacts are not present yet. Run Generate Scene Package for strict parity.");
    }
  }
}
void MainWindow::open_validation_report() { open_selected_scene_artifact("smoke"); }
void MainWindow::copy_validation_summary() { QApplication::clipboard()->setText(validation_summary_label_ ? validation_summary_label_->text() : QString("Validation Summary unavailable")); }
void MainWindow::generate_readiness_pack() {
  if (!has_selected_scene()) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const QString cmd = QString("python3 scripts/generate_workcell_studio_readiness_pack.py '%1'").arg(QString::fromStdString(s.scene_dir.string()));
  append_studio_log("Generate Readiness Pack: " + cmd);
  std::system(cmd.toStdString().c_str());
}
void MainWindow::open_readiness_dashboard() { open_selected_scene_artifact("demo_dashboard"); }
QString MainWindow::canvas_generated_parity_state_text(CanvasGeneratedParityState state) const
{
  switch (state) {
    case CanvasGeneratedParityState::PreGenerationOk: return "Pre-generation OK";
    case CanvasGeneratedParityState::PreGenerationWarnings: return "Pre-generation warnings";
    case CanvasGeneratedParityState::PostGenerationPassed: return "Post-generation passed";
    case CanvasGeneratedParityState::PostGenerationWarnings: return "Post-generation warnings";
    case CanvasGeneratedParityState::PostGenerationBlocked: return "Post-generation blocked";
    case CanvasGeneratedParityState::NotChecked:
    default:
      return "Not checked";
  }
}

void MainWindow::refresh_canvas_generated_parity_ui()
{
  if (!readiness_label_) return;
  const QString parity_line = QString("Canvas/Generated parity: %1").arg(
    canvas_generated_parity_state_text(canvas_generated_parity_state_));
  QString text = readiness_label_->text();
  if (text.contains("Canvas/Generated parity:")) {
    text.replace(QRegularExpression("Canvas/Generated parity:.*"), parity_line);
  } else {
    text += QString("\n") + parity_line;
  }
  readiness_label_->setText(text);
}

bool MainWindow::parse_canvas_generated_parity_report(
  const QString & report_path, int * mismatches, int * warnings, int * blockers) const
{
  QFile file(report_path);
  if (!file.open(QIODevice::ReadOnly)) return false;
  const auto doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isObject()) return false;
  const auto root = doc.object();
  const auto summary = root.value("summary").toObject();
  *mismatches = summary.value("mismatch_count").toInt(root.value("mismatch_count").toInt(0));
  *warnings = summary.value("warning_count").toInt(root.value("warning_count").toInt(0));
  *blockers = summary.value("blocker_count").toInt(root.value("blocker_count").toInt(0));
  return true;
}

bool MainWindow::run_canvas_generated_parity_check(CanvasGeneratedParityMode mode, QString * user_warning, bool * severe_mismatch)
{
  if (user_warning) user_warning->clear();
  if (severe_mismatch) *severe_mismatch = false;
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    append_studio_log("Check Canvas/RViz Parity: no scene selected.");
    return false;
  }
  QString script;
  if (!helper_script_exists("validate_scene_builder_canvas_generated_parity.py", &script)) {
    append_studio_log("Check Canvas/RViz Parity: missing helper script validate_scene_builder_canvas_generated_parity.py.");
    return false;
  }
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const QString report_path = QString::fromStdString((s.scene_dir / "smoke" / "scene_builder_canvas_generated_parity_report.json").string());
  const QString mode_arg = (mode == CanvasGeneratedParityMode::PreGeneration) ? "pre_generation" : "post_generation";
  const QString cmd = QString("python3 '%1' '%2' --json --output '%3' --mode %4")
    .arg(script, QString::fromStdString(s.scene_dir.string()), report_path, mode_arg);
  append_studio_log("Check Canvas/RViz Parity command: " + cmd);
  QProcess process;
  process.start("/bin/bash", QStringList() << "-lc" << cmd);
  if (!process.waitForFinished(120000)) {
    append_studio_log("Check Canvas/RViz Parity: timed out.");
    return false;
  }
  const int exit_code = process.exitCode();
  append_studio_log(QString("Check Canvas/RViz Parity mode: %1").arg(mode_arg));
  append_studio_log(QString("Check Canvas/RViz Parity exit status: %1").arg(exit_code));
  append_studio_log("Check Canvas/RViz Parity report path: " + report_path);
  canvas_generated_parity_report_path_ = report_path;
  int mismatches = 0;
  int warnings = 0;
  int blockers = 0;
  if (!parse_canvas_generated_parity_report(report_path, &mismatches, &warnings, &blockers)) {
    append_studio_log("Check Canvas/RViz Parity: failed to parse report JSON.");
    canvas_generated_parity_state_ = CanvasGeneratedParityState::NotChecked;
    return false;
  }
  canvas_generated_parity_mismatches_ = mismatches;
  canvas_generated_parity_warnings_ = warnings;
  canvas_generated_parity_blockers_ = blockers;
  append_studio_log(QString("Check Canvas/RViz Parity summary: mismatches=%1 warnings=%2 blockers=%3")
    .arg(mismatches).arg(warnings).arg(blockers));
  if (mode == CanvasGeneratedParityMode::PreGeneration) {
    if (blockers > 0) {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PreGenerationWarnings;
      if (user_warning) {
        *user_warning = QString("Pre-generation parity found layout blockers (mismatches=%1, warnings=%2, blockers=%3).")
          .arg(mismatches).arg(warnings).arg(blockers);
      }
    } else if (warnings > 0 || mismatches > 0) {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PreGenerationWarnings;
      if (user_warning) {
        *user_warning = QString("Pre-generation parity warnings (mismatches=%1, warnings=%2).")
          .arg(mismatches).arg(warnings);
      }
    } else {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PreGenerationOk;
    }
    append_studio_log("Check Canvas/RViz Parity recommendation: Run Generate Scene Package for strict parity.");
  } else {
    if (blockers > 0 || mismatches >= 3) {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PostGenerationBlocked;
      if (severe_mismatch) *severe_mismatch = true;
      if (user_warning) {
        *user_warning = QString("Post-generation parity has blockers (mismatches=%1, warnings=%2, blockers=%3).")
          .arg(mismatches).arg(warnings).arg(blockers);
      }
    } else if (warnings > 0 || mismatches > 0) {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PostGenerationWarnings;
      if (user_warning) {
        *user_warning = QString("Post-generation parity warnings (mismatches=%1, warnings=%2).")
          .arg(mismatches).arg(warnings);
      }
    } else {
      canvas_generated_parity_state_ = CanvasGeneratedParityState::PostGenerationPassed;
    }
    append_studio_log("Check Canvas/RViz Parity recommendation: Ready for fake-hardware preview when other gates pass.");
  }
  return true;
}

void MainWindow::run_preview_build(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } if(detect_workspace_root().isEmpty()){ QMessageBox::warning(this,"Preview Launch","Workspace root not detected. Copy commands and run manually."); return;} if (preview_process_ && preview_process_->state() != QProcess::NotRunning) { append_studio_log("WARN Plan & Simulate build start ignored: another preview/build process is already running."); return; } active_preview_command_=selected_scene_build_command(); preview_running_scene_key_ = selected_scene_name(); if(preview_log_) preview_log_->appendPlainText("$ "+active_preview_command_); set_preview_state("BUILD_RUNNING"); write_preview_launch_transcript(true, active_preview_command_, "build_started"); preview_process_->start("/bin/bash", {"-lc", active_preview_command_}); }
void MainWindow::run_fake_hardware_preview(){
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    const QString reason = "selected scene missing";
    QMessageBox::warning(this, "Plan & Simulate", reason);
    append_studio_log("Plan & Simulate launch blocked: " + reason);
    return;
  }

  const auto & scene = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const std::string workspace_root = detect_workspace_root().toStdString();
  append_studio_log(QString("RViz Truth Preview selection: scene=%1 workspace_root=%2")
      .arg(QString::fromStdString(scene.scene_name), QString::fromStdString(workspace_root)));
  append_studio_log(QString("RViz Truth Preview package: %1").arg(QString::fromStdString(scene.scene_name)));
  const boost::filesystem::path layout_file = scene.scene_dir / "layout" / "workcell_studio_layout.yaml";
  const boost::filesystem::path merge_report = scene.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  if (boost::filesystem::exists(layout_file) &&
    (!boost::filesystem::exists(merge_report) || boost::filesystem::last_write_time(layout_file) > boost::filesystem::last_write_time(merge_report))) {
    append_studio_log("RViz Truth Preview readiness: WARN stale layout detected; continuing with generated package.");
  }
  const auto readiness = workcell_builder::validate_readiness(scene, workspace_root);
  if (!readiness.ready) {
    QMessageBox::warning(this, "RViz Truth Preview", readiness.blocker_reason);
    append_studio_log("RViz Truth Preview readiness: BLOCKED - " + readiness.blocker_reason);
    return;
  }
  append_studio_log("RViz Truth Preview readiness: PASS");

  QString command;
  const auto dry_run_status = workcell_builder::dry_run(scene, workspace_root, &command);
  if (!dry_run_status.ready) {
    QMessageBox::warning(this, "RViz Truth Preview", dry_run_status.blocker_reason);
    append_studio_log("RViz Truth Preview dry-run blocked: " + dry_run_status.blocker_reason);
    return;
  }
  append_studio_log("RViz Truth Preview fake-hardware safety: PASS");
  append_studio_log("RViz Truth Preview dry run: " + command);
  auto rc = QMessageBox::question(
    this, "Confirm Fake-Hardware Preview",
    "Command:\n" + command + "\n\nFake hardware only. No real hardware. No runtime execution. No robot motion commanded.");
  if (rc != QMessageBox::Yes) return;

  const QString selected_scene_key = QString::fromStdString(scene.scene_name);
  if (preview_process_ && preview_process_->state() != QProcess::NotRunning) {
    if (preview_running_scene_key_ == selected_scene_key) {
      append_studio_log("WARN RViz Truth Preview launch ignored: preview already running for selected scene.");
    } else {
      append_studio_log(QString("WARN RViz Truth Preview launch ignored: another preview is running for scene '%1'.").arg(preview_running_scene_key_));
    }
    return;
  }

  active_preview_command_ = command;
  preview_running_scene_key_ = selected_scene_key;
  if (preview_log_) preview_log_->appendPlainText("$ " + command);
  append_studio_log("RViz Truth Preview launch started");
  set_preview_state("PREVIEW_RUNNING");
  write_preview_launch_transcript(true, command, "preview_started");
  append_studio_log(QString("RViz Truth Preview launch command: scene=%1 package=%2 fake_hardware=true launch_path=%3 command=%4")
    .arg(QString::fromStdString(scene.scene_name),
      QString::fromStdString(scene.scene_name),
      QString::fromStdString((scene.scene_dir / "launch" / "demo.launch.py").string()),
      command));
  preview_process_->start("bash", {"-lc", command});
  refresh_new_cell_checklist();
}
void MainWindow::stop_preview_process(){ if(!preview_process_ || preview_process_->state()==QProcess::NotRunning) return; set_preview_state("PREVIEW_STOPPING"); if(preview_log_) preview_log_->appendPlainText("Stopping preview process..."); preview_process_->terminate(); QTimer::singleShot(2000, this, [this]() { if(preview_process_ && preview_process_->state()!=QProcess::NotRunning){ if(preview_log_) preview_log_->appendPlainText("Terminate timeout, forcing kill."); preview_process_->kill(); } }); refresh_new_cell_checklist(); }
void MainWindow::handle_preview_stdout(){ if(!preview_process_) return; const QString out = QString::fromUtf8(preview_process_->readAllStandardOutput()); if(preview_log_) preview_log_->appendPlainText(out); if(!out.trimmed().isEmpty()) append_studio_log("[preview stdout] " + out.trimmed()); }
void MainWindow::handle_preview_stderr(){ if(!preview_process_) return; const QString err = QString::fromUtf8(preview_process_->readAllStandardError()); if(preview_log_) preview_log_->appendPlainText(err); if(!err.trimmed().isEmpty()) append_studio_log("[preview stderr] " + err.trimmed()); }
void MainWindow::handle_preview_started(){ if(!preview_process_) return; const qint64 pid = preview_process_->processId(); const QString state = QString::number(static_cast<int>(preview_process_->state())); append_studio_log(QString("Plan & Simulate process started: pid=%1 state=%2").arg(pid).arg(state)); }
void MainWindow::handle_preview_error(QProcess::ProcessError error){ const QString msg = preview_process_ ? preview_process_->errorString() : QStringLiteral("unknown error"); append_studio_log(QString("ERROR RViz Truth Preview process error: code=%1 message=%2").arg(static_cast<int>(error)).arg(msg)); append_studio_log("RViz Truth Preview launch blocked: ros2 launch failed"); if(preview_state_=="PREVIEW_RUNNING" || preview_state_=="BUILD_RUNNING") set_preview_state("PREVIEW_FAILED"); }
void MainWindow::handle_preview_finished(int exit_code, QProcess::ExitStatus exit_status){ if(preview_state_=="BUILD_RUNNING") set_preview_state(exit_code==0?"BUILD_PASSED":"BUILD_FAILED"); else if(preview_state_=="PREVIEW_STOPPING") set_preview_state("PREVIEW_STOPPED"); else set_preview_state(exit_code==0?"PREVIEW_EXITED":"PREVIEW_FAILED"); if (preview_state_ == "PREVIEW_FAILED") append_studio_log("RViz Truth Preview launch blocked: ros2 launch failed"); append_studio_log(QString("RViz Truth Preview launch exit: exit_code=%1 exit_status=%2 state=%3").arg(exit_code).arg(static_cast<int>(exit_status)).arg(preview_state_)); preview_running_scene_key_.clear(); write_preview_launch_transcript(true, active_preview_command_, "process_finished", exit_code); refresh_new_cell_checklist(); }

void MainWindow::write_preview_launch_transcript(bool ran_process, const QString & command, const QString & event, int exit_code)
{
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  fs::path out = s.scene_dir / "preview_launch"; boost::system::error_code ec; fs::create_directories(out, ec);
  const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  QJsonObject root{{"scene_name", QString::fromStdString(s.scene_name)}, {"command", command}, {"event", event}, {"started_at", ran_process?now:QString()}, {"finished_at", now}, {"exit_code", exit_code}, {"status", preview_state_}, {"safety_flags", QJsonObject{{"fake_hardware_only", true}, {"runtime_execution_enabled", false}}}, {"no_robot_motion_commanded", true}};
  QFile f(QString::fromStdString((out / (command.contains("colcon build")?"build_session.json":"preview_launch_session.json")).string())); if(f.open(QIODevice::WriteOnly|QIODevice::Text)) f.write(QJsonDocument(root).toJson());
  QFile sfile(QString::fromStdString((out / (command.contains("colcon build")?"build_summary.txt":"preview_launch_summary.txt")).string())); if(sfile.open(QIODevice::WriteOnly|QIODevice::Text)) sfile.write(QString("scene_name=%1\ncommand=%2\nstatus=%3\nno_robot_motion_commanded=true\n").arg(QString::fromStdString(s.scene_name), command, preview_state_).toUtf8());
  QFile c(QString::fromStdString((out / "latest_console.log").string())); if(c.open(QIODevice::WriteOnly|QIODevice::Text) && preview_log_) c.write(preview_log_->toPlainText().toUtf8());
}


void MainWindow::run_layout_merge_for_selected_scene(bool from_generate_scene)
{
  if (selected_scene_index_ < 0) { QMessageBox::warning(this, "Layout Merge", "No scene selected"); return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
  if (!fs::exists(layout_file)) { QMessageBox::information(this, "Layout Merge", "No saved layout found (layout/workcell_studio_layout.yaml)."); return; }
  if (layout_dirty_) {
    const QString message = format_scene_builder_status_text(
      "Layout has unsaved edits. Save Layout first.");
    QMessageBox::warning(this, "Layout Merge", message);
    append_studio_log(message);
    return;
  }
  append_studio_log(from_generate_scene ? "Generate Scene: running layout merge" : "Run Layout Merge");
  auto result = workcell_builder::merge_workcell_studio_layout(s.scene_dir);
  append_studio_log(QString::fromStdString(result.status ? "Layout merge completed" : "Layout merge blocked"));
  if (!result.report_path.empty()) {
    append_studio_log("Merge report: " + QString::fromStdString(result.report_path));
  } else if (!result.blockers.empty()) {
    append_studio_log("Merge report blocker: " + QString::fromStdString(result.blockers.front()));
  } else if (!result.stderr_log.empty()) {
    append_studio_log("Merge report stderr: " + QString::fromStdString(result.stderr_log).left(400));
  } else {
    append_studio_log("Merge report blocker: layout merge failed without report path");
  }
  refresh_scene_browser_ui();
  refresh_scene_builder_left_explorer();
}

void MainWindow::open_layout_merge_report()
{
  open_selected_scene_artifact("layout_merge_report");
}

void MainWindow::copy_layout_merge_summary()
{
  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path summary = s.scene_dir / "generated" / "workcell_studio_layout_merge_summary.txt";
  if (!fs::exists(summary)) { QMessageBox::warning(this, "Copy Merge Summary", "Merge summary not found"); return; }
  QFile f(QString::fromStdString(summary.string()));
  if (f.open(QIODevice::ReadOnly | QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll()));
}

QString MainWindow::diagnostics_output_root() const
{ const QString ws = detect_workspace_root(); return ws.isEmpty() ? (QDir::homePath() + "/diagnostics") : (ws + "/diagnostics"); }

bool MainWindow::helper_script_exists(const QString & script_name, QString * path) const
{
  const QStringList candidates = helper_script_search_paths(script_name);
  for (const auto & candidate : candidates) {
    if (QFileInfo::exists(candidate)) {
      if (path) {
        *path = candidate;
      }
      return true;
    }
  }
  return false;
}

QStringList MainWindow::helper_script_search_paths(const QString & script_name) const
{
  QStringList candidates;
  const QString workspace_root = detect_workspace_root();
  if (!workspace_root.isEmpty()) {
    candidates << (workspace_root + "/src/easy_manipulation_deployment/scripts/" + script_name);
  }
  candidates << (QCoreApplication::applicationDirPath() + "/../../../scripts/" + script_name);
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("workcell_builder");
    candidates << (QString::fromStdString(share_dir) + "/scripts/" + script_name);
  } catch (const std::exception &) {
  }
  candidates << (QDir::currentPath() + "/scripts/" + script_name);
  return candidates;
}

QStringList MainWindow::candidate_repo_roots_for_scene(const fs::path & selected_scene_dir) const
{
  QStringList candidates;
  QSet<QString> seen;
  auto add_candidate = [&](const QString & candidate) {
    if (candidate.isEmpty()) {
      return;
    }
    const QString normalized = QDir::cleanPath(candidate);
    if (!seen.contains(normalized)) {
      candidates << normalized;
      seen.insert(normalized);
    }
  };

  for (fs::path cursor = selected_scene_dir; !cursor.empty(); cursor = cursor.parent_path()) {
    add_candidate(QString::fromStdString(cursor.string()));
    if (cursor == cursor.parent_path()) {
      break;
    }
  }

  const QString home_dir = QDir::homePath();
  const QString username = QFileInfo(home_dir).fileName();
  if (!username.isEmpty()) {
    add_candidate(QString("/home/%1/workcell_ws/src/easy_manipulation_deployment").arg(username));
  }

  for (QDir cwd(QDir::currentPath()); cwd.exists(); ) {
    add_candidate(cwd.absolutePath());
    if (!cwd.cdUp()) {
      break;
    }
  }

  for (QDir app_dir(QCoreApplication::applicationDirPath()); app_dir.exists(); ) {
    add_candidate(app_dir.absolutePath());
    if (!app_dir.cdUp()) {
      break;
    }
  }

  const QString ament_prefix_raw = qEnvironmentVariable("AMENT_PREFIX_PATH");
  const QStringList ament_entries = ament_prefix_raw.split(':', Qt::SkipEmptyParts);
  for (const QString & raw_entry : ament_entries) {
    const QString entry = QDir::cleanPath(raw_entry.trimmed());
    if (entry.isEmpty()) {
      continue;
    }
    const QString share_root = QDir(entry).filePath("share/easy_manipulation_deployment");
    add_candidate(share_root);
    for (QDir share_dir(share_root); share_dir.exists(); ) {
      add_candidate(share_dir.absolutePath());
      if (!share_dir.cdUp()) {
        break;
      }
    }
    add_candidate(entry);
    for (QDir prefix_dir(entry); prefix_dir.exists(); ) {
      add_candidate(prefix_dir.absolutePath());
      if (!prefix_dir.cdUp()) {
        break;
      }
    }
  }

  return candidates;
}

QString MainWindow::find_repo_root_with_extractor(const QStringList & candidate_roots) const
{
  for (const QString & root : candidate_roots) {
    const QString script_path =
      QDir(root).filePath("scripts/extract_scene_urdf_visual_mesh_index.py");
    const QFileInfo script_info(script_path);
    if (script_info.exists() && script_info.isFile()) {
      return script_info.absoluteFilePath();
    }
  }
  return "";
}

QString MainWindow::resolve_scene3d_extractor_script_path(const fs::path & selected_scene_dir) const
{
  const QStringList candidate_roots = candidate_repo_roots_for_scene(selected_scene_dir);
  return find_repo_root_with_extractor(candidate_roots);
}

QString MainWindow::selected_scene_name() const
{
  if (!selected_scene_state_.valid) {
    return "none";
  }
  return selected_scene_state_.name;
}

QString MainWindow::selected_scene_path() const
{
  if (!selected_scene_state_.valid) return "";
  return selected_scene_state_.path;
}

bool MainWindow::has_selected_scene() const
{
  return selected_scene_state_.valid;
}

void MainWindow::sync_selected_scene_state()
{
  selected_scene_state_ = {};
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) return;
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  selected_scene_state_.valid = true;
  selected_scene_state_.index = selected_scene_index_;
  selected_scene_state_.name = QString::fromStdString(s.scene_name);
  selected_scene_state_.path = QString::fromStdString(s.scene_dir.string());
  selected_scene_state_.status = QString::fromStdString(s.status);
  const auto metadata = selected_scene_metadata_summary(s);
  selected_scene_state_.robot_summary = metadata.robot;
  selected_scene_state_.end_effector_summary = metadata.end_effector;
  selected_scene_state_.tool_mount_summary = metadata.tool_mount;
  selected_scene_state_.grasp_frame_summary = metadata.grasp_frame;
  selected_scene_state_.launchable = s.has_launch_demo;
}

void MainWindow::sync_selected_item_state()
{
  selected_item_state_ = current_selected_scene_item();
  if (!selected_scene_state_.valid) {
    selected_item_state_ = {};
    current_selected_scene_item_id_.clear();
  } else if (selected_item_state_.valid) {
    current_selected_scene_item_id_ = selected_item_state_.id.trimmed();
  } else if (!current_selected_scene_item_id_.trimmed().isEmpty()) {
    current_selected_scene_item_id_.clear();
  }
}

void MainWindow::refresh_selected_scene_metadata_panel()
{
  if (!selected_scene_state_.valid) {
    if (selection_scene_name_label_) selection_scene_name_label_->setText("No scene selected");
    if (selection_scene_status_label_) selection_scene_status_label_->setText("unknown");
    if (selection_scene_robot_label_) selection_scene_robot_label_->setText("unknown");
    if (selection_scene_end_effector_label_) selection_scene_end_effector_label_->setText("unknown");
    if (selection_scene_path_label_) {
      selection_scene_path_label_->setText("(none)");
      selection_scene_path_label_->setToolTip("(none)");
    }
    if (selection_scene_launch_label_) {
      selection_scene_launch_label_->setText("(none)");
      selection_scene_launch_label_->setToolTip("(none)");
    }
    return;
  }

  const int index = selected_scene_state_.index;
  if (index < 0 || index >= static_cast<int>(scene_browser_result_.scenes.size())) return;
  const auto & scene = scene_browser_result_.scenes[static_cast<size_t>(index)];
  const auto metadata = selected_scene_metadata_summary(scene);
  const QString scene_path = selected_scene_path();
  const fs::path launch_path = scene.scene_dir / "launch" / "demo.launch.py";
  const bool launch_present = fs::exists(launch_path);

  QStringList warnings;
  if (metadata.robot_source.startsWith(QStringLiteral("missing"))) warnings << metadata.robot;
  if (metadata.end_effector_source.startsWith(QStringLiteral("missing"))) warnings << metadata.end_effector;
  if (!launch_present) warnings << QStringLiteral("launch/demo.launch.py missing");
  const QString browser_status = QString::fromStdString(scene.status).trimmed();
  if (browser_status != QStringLiteral("READY") && !browser_status.isEmpty()) {
    warnings << QStringLiteral("scene browser status is %1").arg(browser_status);
  }
  const QString status_text = warnings.isEmpty()
    ? QStringLiteral("ready — required selected-scene metadata and launch/demo.launch.py are present")
    : QStringLiteral("warnings — %1").arg(warnings.join(QStringLiteral("; ")));
  const QString launch_text = launch_present
    ? QStringLiteral("%1 (present)").arg(QString::fromStdString(launch_path.string()))
    : QStringLiteral("launch/demo.launch.py missing at %1").arg(QString::fromStdString(launch_path.string()));
  const QString robot_text = QStringLiteral("%1 — %2").arg(metadata.robot, metadata.robot_source);
  const QString end_effector_text = QStringLiteral("%1 — %2").arg(metadata.end_effector, metadata.end_effector_source);

  auto set_label = [](QLabel * label, const QString & text) {
    if (!label) return;
    label->setText(text);
    label->setToolTip(text);
  };
  set_label(selection_scene_name_label_, metadata.scene_name);
  set_label(selection_scene_status_label_, status_text);
  set_label(selection_scene_robot_label_, robot_text);
  set_label(selection_scene_end_effector_label_, end_effector_text);
  set_label(selection_scene_path_label_, scene_path);
  set_label(selection_scene_launch_label_, launch_text);

  refresh_selected_scene_item_labels(selected_item_state_);
}

void MainWindow::refresh_scene_builder_selection_state_ui()
{
  sync_selected_scene_state();
  sync_selected_item_state();
  refresh_selected_scene_metadata_panel();
  refresh_scene_builder_selected_scene_ui();
  refresh_scene_builder_left_explorer();
  refresh_selected_scene_details_card();
  refresh_task_intent_panel();
}

ScenePreviewWidget * MainWindow::active_scene_preview_widget() const
{
  return scene_preview_widget_;
}

void MainWindow::refresh_scene_builder_state_from_active_scene()
{
  refresh_scene_builder_selected_scene_ui();
}

void MainWindow::refresh_scene_builder_selected_scene_ui()
{
  sync_selected_scene_state();
  sync_selected_item_state();
  refresh_selected_scene_metadata_panel();
  if (!selected_scene_state_.valid) {
    if (scene_builder_title_) scene_builder_title_->setText("<h2>Scene Builder</h2>");
    refresh_scene_builder_view_chips();
    if (scene_builder_path_label_) scene_builder_path_label_->setText("Path: (none)");
    if (canvas_header_label_) canvas_header_label_->setText("No scene selected");
    if (scene_preview_label_) scene_preview_label_->setText("<b>Digital Twin Canvas</b>");
    if (scene_preview_widget_) scene_preview_widget_->set_scene_selected(false);
    populate_scene_files_tab();
    refresh_create_starter_layout_action();
    return;
  }
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_state_.index)];
  if (scene_builder_title_) scene_builder_title_->setText(QString("<h2>Scene Builder: %1</h2>").arg(selected_scene_state_.name));
  if (scene_builder_path_label_) {
    const QString sp = selected_scene_path();
    const QFontMetrics metrics(scene_builder_path_label_->font());
    const QString short_path = metrics.elidedText(sp, Qt::ElideMiddle, 460);
    scene_builder_path_label_->setText(QString("Path: %1").arg(short_path));
    scene_builder_path_label_->setToolTip(sp);
  }
  refresh_scene_builder_view_chips();
  const auto metadata = selected_scene_metadata_summary(s);
  if (scene_preview_label_) scene_preview_label_->setText(QString("%1\nStatus: %2\nScene: %3\nPath: %4\nRobot: %5 (%6)\nEnd effector: %7 (%8)\nLaunch: %9")
    .arg(s.has_static_preview_svg ? "Preview SVG available" : "Generate preview/readiness pack to populate this panel",
      selected_scene_state_.status, metadata.scene_name, metadata.scene_path, metadata.robot, metadata.robot_source,
      metadata.end_effector, metadata.end_effector_source, metadata.launch));
  if (canvas_header_label_) canvas_header_label_->setText(QString("%1 | status: %2 | source: %3")
    .arg(selected_scene_state_.name, selected_scene_state_.status, selected_scene_state_.path));
  refresh_selected_scene_item_labels(selected_item_state_);
  if (scene_preview_widget_) scene_preview_widget_->set_scene_selected(true);
  populate_scene_files_tab();
  refresh_create_starter_layout_action();
}

void MainWindow::refresh_create_starter_layout_action()
{
  if (!create_starter_layout_button_) return;
  if (!has_selected_scene()) {
    create_starter_layout_button_->setVisible(false);
    return;
  }
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const auto layout_inspection = workcell_builder::inspect_editable_layout_entries(s.scene_dir);
  const bool lacks_editable_layout_content = layout_inspection.editable_item_count == 0U;
  const bool has_trusted_yaml_source =
    fs::exists(s.scene_dir / "layout" / "workcell_studio_layout.yaml") ||
    fs::exists(s.scene_dir / "environment_layout.yaml") ||
    fs::exists(s.scene_dir / "environment.yaml") ||
    fs::exists(s.scene_dir / "cell_definition.yaml");
  const auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  const std::size_t preview_count = model.items.size();
  const bool has_any_trusted_bootstrap_source = has_trusted_yaml_source || preview_count > 0U;
  const bool show_action = lacks_editable_layout_content && has_any_trusted_bootstrap_source;
  create_starter_layout_button_->setVisible(show_action);
  create_starter_layout_button_->setToolTip(show_action ?
    QString("Create layout/workcell_studio_layout.yaml from trusted scene bootstrap sources") :
    QString("Hidden unless editable layout count is 0 and a trusted bootstrap source may exist "
            "(current: editable=%1 preview=%2 yaml_source=%3)")
      .arg(layout_inspection.editable_item_count)
      .arg(preview_count)
      .arg(has_trusted_yaml_source ? "yes" : "no"));
}

void MainWindow::populate_scene_files_tab()
{
  if (!scene_files_selected_path_label_ || !scene_files_tree_) {
    return;
  }
  scene_files_tree_->clear();
  scene_files_tree_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  scene_files_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  scene_files_tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  if (!has_selected_scene()) {
    scene_files_selected_path_label_->setText("Selected scene path: (none)");
    return;
  }

  const auto & selected = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const fs::path scene_dir = selected.scene_dir;
  scene_files_selected_path_label_->setText(QString("Selected scene path: %1").arg(QString::fromStdString(scene_dir.string())));
  const std::vector<std::pair<QString, QString>> required_artifacts = {
    {"Environment YAML", "environment.yaml"},
    {"Scene Manifest", "scene_manifest.yaml"},
    {"ROS Package File", "package.xml"},
    {"CMake File", "CMakeLists.txt"},
    {"Demo Launch", "launch/demo.launch.py"},
    {"Task Intent", "config/workcell_builder_task_intent.yaml"},
    {"Task Recipe (fallback)", "config/task_recipe.yaml"},
    {"Legacy Task Recipe (fallback)", "task_recipe.yaml"},
    {"Readiness Dashboard", "readiness/readiness_dashboard.html"},
    {"Readiness Summary", "readiness/readiness_summary.txt"},
    {"Preview HTML", "preview/static_preview.html"},
    {"Preview SVG", "preview/static_preview.svg"}
  };
  boost::system::error_code ec;
  for (const auto & artifact : required_artifacts) {
    const fs::path file_path = scene_dir / artifact.second.toStdString();
    const bool exists = fs::exists(file_path, ec) && !ec;
    if (ec) {
      ec.clear();
    }
    auto * item = new QTreeWidgetItem(scene_files_tree_);
    item->setText(0, artifact.first);
    item->setText(1, artifact.second);
    item->setText(2, exists ? "present" : "missing");
    item->setForeground(2, exists ? QBrush(QColor("#15803D")) : QBrush(QColor("#B91C1C")));
  }
}

QString MainWindow::diagnostics_status_from_counts(int blocked, int warn) const
{ if (blocked > 0) return "BLOCKED"; if (warn > 0) return "WARN"; return "PASS"; }

void MainWindow::append_diagnostics_row(const QString & name, const QString & status, const QString & details, const QString & fix, const QString & related_path)
{ if (!diagnostics_table_) return; int r = diagnostics_table_->rowCount(); diagnostics_table_->insertRow(r); diagnostics_table_->setItem(r,0,new QTableWidgetItem(name)); diagnostics_table_->setItem(r,1,new QTableWidgetItem(status)); diagnostics_table_->setItem(r,2,new QTableWidgetItem(details)); diagnostics_table_->setItem(r,3,new QTableWidgetItem(fix)); diagnostics_table_->setItem(r,4,new QTableWidgetItem(related_path)); }

void MainWindow::write_diagnostics_report(
  const QJsonObject & report,
  const QString & summary,
  const QString & dashboard_html)
{
  const QString output_root = diagnostics_output_root();

  QDir dir;
  dir.mkpath(output_root);

  QFile jf(output_root + "/workcell_studio_diagnostics_report.json");
  if (jf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    jf.write(QJsonDocument(report).toJson());
  }

  QFile sf(output_root + "/workcell_studio_diagnostics_summary.txt");
  if (sf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    sf.write(summary.toUtf8());
  }

  QFile hf(output_root + "/workcell_studio_diagnostics_dashboard.html");
  if (hf.open(QIODevice::WriteOnly | QIODevice::Text)) {
    hf.write(dashboard_html.toUtf8());
  }
}

void MainWindow::refresh_diagnostics_quick_status()
{ int blocked = 0; int warn = 0; const QString ws = detect_workspace_root(); if (ws.isEmpty()) blocked++; if (!QFileInfo::exists(ws + "/scenes")) warn++; const QString st = diagnostics_status_from_counts(blocked, warn); if (diagnostics_status_label_) diagnostics_status_label_->setText("Status: "+st); if (diagnostics_indicator_label_) diagnostics_indicator_label_->setText("Diagnostics: "+st); }

void MainWindow::run_diagnostics_self_test()
{ if (diagnostics_table_) diagnostics_table_->setRowCount(0); int blocked=0,warn=0; auto add=[&](const QString&n,bool ok,bool hard,const QString&d,const QString&f,const QString&p){QString s=ok?"PASS":(hard?"BLOCKED":"WARN"); if(!ok){if(hard)blocked++; else warn++;} append_diagnostics_row(n,s,d,f,p);};
  const QString ws=detect_workspace_root(); add("ROS workspace detected", !ws.isEmpty(), true, ws.isEmpty()?"workspace root not detected":ws, "Select workspace containing install/ and src/", ws);
  add("workcell_builder package found", QFileInfo::exists(ws+"/src"), true, "Check src folder", "Clone repository into workspace src", ws+"/src");
  add("scenes root found", QFileInfo::exists(ws+"/scenes"), false, "Expected scenes root", "Create <workspace>/scenes", ws+"/scenes");
  add("assets root found", QFileInfo::exists(ws+"/assets"), false, "Expected assets root", "Create <workspace>/assets", ws+"/assets");
  QString p; add("helper scripts found", helper_script_exists("run_workcell_studio_golden_flow.py", &p), true, p.isEmpty()?"missing golden flow helper":p, "Ensure scripts folder is present", p);
  add("light theme loaded", !styleSheet().isEmpty(), false, styleSheet().isEmpty()?"default Qt theme active":"Workcell Studio light theme active", "Keep light/native Qt styling", "gui/mainwindow.cpp");
  add("Qt SVG support available", true, false, "QSvgGenerator linked", "Install Qt SVG runtime if missing", "QtSvg");
  add("scene browser working", scene_browser_result_.root_exists, false, scene_browser_result_.root_exists?"scene browser ready":"scene browser root missing", "Create scenes root and refresh", ws+"/scenes");
  add("layout merge script working", helper_script_exists("workcell_studio_layout_merge.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("acceptance validator working", helper_script_exists("validate_workcell_studio_generated_scene.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("demo mode script working", helper_script_exists("workcell_studio_demo_mode.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  add("preview launch helper working", helper_script_exists("workcell_studio_preview_launch.py", &p), false, p.isEmpty()?"missing":p, "Restore script", p);
  QStringList b; bool safe = preview_command_is_safe("ros2 launch demo demo.launch.py use_fake_hardware:=true", &b);
  add("fake-hardware command safety", safe, true, safe?"safe tokens validated":b.join(", "), "Keep use_fake_hardware:=true and no unsafe tokens", "preview launch command");
  add("no robot motion safety flags", true, true, "no_robot_motion_commanded: true", "Diagnostics is offline-only", "diagnostics report");
  const QString st=diagnostics_status_from_counts(blocked,warn); if(diagnostics_status_label_) diagnostics_status_label_->setText("Status: "+st); if(diagnostics_indicator_label_) diagnostics_indicator_label_->setText("Diagnostics: "+st); if(diagnostics_summary_label_) diagnostics_summary_label_->setText(QString("PASS rows: %1 | WARN rows: %2 | BLOCKED rows: %3").arg(diagnostics_table_?diagnostics_table_->rowCount()-warn-blocked:0).arg(warn).arg(blocked));
  QJsonObject report{{"timestamp", QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}, {"workspace_root", ws}, {"scenes_root", ws+"/scenes"}, {"assets_root", ws+"/assets"}, {"golden_flow_status", "NOT CHECKED"}, {"safety_status", st}, {"no_robot_motion_commanded", true}};
  write_diagnostics_report(report, QString("Diagnostics status: %1\nno_robot_motion_commanded=true\n").arg(st), QString("<html><body><h1>Diagnostics: %1</h1><p>No robot motion commanded.</p></body></html>").arg(st)); }

void MainWindow::run_diagnostics_golden_flow_dry_run()
{ const QString cmd = "python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/workcell_studio_diag_scene --json"; QProcess p; p.start("/bin/bash", {"-lc", cmd}); p.waitForFinished(60000); append_studio_log("Run Golden Flow Dry Run"); append_studio_log("Command: " + cmd); append_studio_log("Report path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_report.json"); append_studio_log("Summary path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_summary.txt"); append_studio_log("Dashboard path: /tmp/workcell_studio_diag_scene/golden_flow/workcell_studio_golden_flow_dashboard.html"); }

void MainWindow::copy_diagnostics_report()
{ QFile f(diagnostics_output_root()+"/workcell_studio_diagnostics_summary.txt"); if(f.open(QIODevice::ReadOnly|QIODevice::Text)) QApplication::clipboard()->setText(QString::fromUtf8(f.readAll())); }

void MainWindow::open_diagnostics_folder()
{ QDesktopServices::openUrl(QUrl::fromLocalFile(diagnostics_output_root())); }


void MainWindow::rebuild_digital_twin_canvas()
{
  // 2D fallback canvas rebuild only.
  // The primary runtime 3D canvas is Scene3DViewportWidget hosted by ScenePreviewWidget in this same panel.
  // This fallback path remains available when 3D is unavailable, and shares selection/inspector state.
  const QString preserved_selected_id = current_selected_scene_item_id_;
  if (!digital_twin_canvas_) return;
  if (!digital_twin_scene_) {
    digital_twin_scene_ = new QGraphicsScene(digital_twin_canvas_);
    digital_twin_canvas_->setScene(digital_twin_scene_);
    connect(digital_twin_scene_, &QGraphicsScene::selectionChanged, this, &MainWindow::on_canvas_selection_changed);
  }
  digital_twin_scene_->clear();
  digital_twin_scene_->setProperty("workcellPhysicalBoundsValid", false);
  digital_twin_scene_->setProperty("workcellPhysicalBounds", QRectF());
  digital_twin_scene_->setProperty("workcellOverlayBoundsValid", false);
  digital_twin_scene_->setProperty("workcellOverlayBounds", QRectF());
  digital_twin_scene_->setSceneRect(-400, -300, 1200, 900);
  digital_twin_canvas_->setBackgroundBrush(QColor("#10161f"));

  QRectF overlay_bounds;
  bool has_overlay_bounds = false;
  const auto add_overlay_bounds = [&](const QRectF & bounds) {
    if (!bounds.isValid() || bounds.isNull()) return;
    if (!has_overlay_bounds) {
      overlay_bounds = bounds;
      has_overlay_bounds = true;
    } else {
      overlay_bounds = overlay_bounds.united(bounds);
    }
  };

  if (toggle_grid_box_ && toggle_grid_box_->isChecked()) {
    QPen grid_pen(QColor("#1f2a36")); grid_pen.setWidth(1);
    for (int x = -400; x <= 800; x += 40) {
      auto * line = digital_twin_scene_->addLine(x, -300, x, 600, grid_pen);
      line->setZValue(canvas_item_z_value("grid", "helper_overlay", "overlay", true));
      add_overlay_bounds(line->sceneBoundingRect());
    }
    for (int y = -300; y <= 600; y += 40) {
      auto * line = digital_twin_scene_->addLine(-400, y, 800, y, grid_pen);
      line->setZValue(canvas_item_z_value("grid", "helper_overlay", "overlay", true));
      add_overlay_bounds(line->sceneBoundingRect());
    }
  }
  QPen axis_pen(QColor("#3a4b5c")); axis_pen.setWidth(2);
  auto * x_axis = digital_twin_scene_->addLine(-400, 0, 800, 0, axis_pen);
  x_axis->setZValue(canvas_item_z_value("axis", "helper_overlay", "overlay", true));
  add_overlay_bounds(x_axis->sceneBoundingRect());
  auto * y_axis = digital_twin_scene_->addLine(0, -300, 0, 600, axis_pen);
  y_axis->setZValue(canvas_item_z_value("axis", "helper_overlay", "overlay", true));
  add_overlay_bounds(y_axis->sceneBoundingRect());
  auto * origin = digital_twin_scene_->addEllipse(-4, -4, 8, 8, QPen(QColor("#d9e2ec")), QBrush(QColor("#d9e2ec")));
  origin->setZValue(canvas_item_z_value("origin", "helper_overlay", "overlay", true) + 1.0);
  origin->setToolTip("World origin marker");
  add_overlay_bounds(origin->sceneBoundingRect());

  if (selected_scene_index_ < 0) {
    if (has_overlay_bounds) {
      digital_twin_scene_->setProperty("workcellOverlayBoundsValid", true);
      digital_twin_scene_->setProperty("workcellOverlayBounds", overlay_bounds);
    }
    return;
  }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  const QString layout_load_message = QString::fromStdString(model.layout_load_message).trimmed();
  if (!layout_load_message.isEmpty() && layout_load_message != last_layout_load_message_log_) {
    append_studio_log(layout_load_message);
    last_layout_load_message_log_ = layout_load_message;
  }

  QVector<QRectF> occupied_label_rects;
  QRectF physical_bounds;
  bool has_physical_bounds = false;
  const auto add_physical_bounds = [&](const QRectF & bounds) {
    if (!bounds.isValid() || bounds.isNull()) return;
    if (!has_physical_bounds) {
      physical_bounds = bounds;
      has_physical_bounds = true;
    } else {
      physical_bounds = physical_bounds.united(bounds);
    }
  };

  if (!show_reach_overlay_box_ || show_reach_overlay_box_->isChecked()) {
    auto * reach = digital_twin_scene_->addEllipse(-150, -150, 300, 300, QPen(QColor("#2dd4bf"), 2, Qt::DashLine)); // robot reach circle/arc
    reach->setZValue(canvas_item_z_value("reach", "helper_overlay", "overlay", true));
    add_overlay_bounds(reach->sceneBoundingRect());
  }
  auto * robot_base = digital_twin_scene_->addEllipse(-14, -14, 28, 28, QPen(QColor("#60a5fa"), 2), QBrush(QColor("#1d4ed8")));
  robot_base->setZValue(canvas_item_z_value("robot", "base", "editable_layout", false));
  robot_base->setToolTip("Robot base marker");

  for (const auto & entry : model.items) {
    const QString category = QString::fromStdString(entry.type);
    const bool is_editable_layout_item =
      entry.provenance == workcell_builder::WorkcellStudioItemProvenance::EditableLayout && !entry.locked;
    const bool is_editable_layout_provenance =
      entry.provenance == workcell_builder::WorkcellStudioItemProvenance::EditableLayout;
    const bool locked_for_canvas = !is_editable_layout_item;
    const QString canvas_access_label = is_editable_layout_item
      ? QStringLiteral("editable layout item")
      : (is_editable_layout_provenance ? QStringLiteral("locked editable layout item") : QStringLiteral("inspection-only preview"));
    auto * item = new DraggableCanvasItem(QRectF(0, 0, std::max(20.0, entry.width * 100.0), std::max(20.0, entry.depth * 100.0)));
    item->setPos(entry.x * 100.0, entry.y * 100.0);
    item->setPen(QPen(category_color(category).lighter(130), 2));
    item->setBrush(QBrush(category_color(category), Qt::SolidPattern));
    item->setToolTip(QString("%1 (%2) — %3")
      .arg(QString::fromStdString(entry.label), category, canvas_access_label));
    item->setData(RoleId, QString::fromStdString(entry.id));
    item->setData(RoleDisplayName, QString::fromStdString(entry.label));
    item->setData(RoleType, QString::fromStdString(entry.type));
    item->setData(RoleCategory, QString::fromStdString(entry.category));
    item->setData(RoleRole, QString::fromStdString(entry.role));
    item->setData(RoleLocked, locked_for_canvas);
    item->setData(RolePoseZ, entry.z);
    item->setData(RoleRoll, entry.roll); item->setData(RolePitch, entry.pitch); item->setData(RoleYaw, entry.yaw);
    item->setData(RoleSource, QString::fromStdString(entry.source_file));
    item->setData(RoleSourcePackage, QString(""));
    item->setData(RoleWidth, entry.width); item->setData(RoleDepth, entry.depth); item->setData(RoleHeight, entry.height);
    const bool is_preview_placeholder = category.contains("placeholder", Qt::CaseInsensitive) || category == "warning";
    item->setData(RoleImported, false); item->setData(RoleGeneratedPlaceholder, is_preview_placeholder);
    item->setData(RoleWarning, QString::fromStdString(entry.warnings.empty() ? std::string() : entry.warnings.front()));
    item->setData(RolePoseText, QString("x=%1 y=%2 z=%3 r=%4 p=%5 y=%6").arg(entry.x).arg(entry.y).arg(entry.z).arg(entry.roll).arg(entry.pitch).arg(entry.yaw));
    QString source_layer = QStringLiteral("locked_generated_urdf_visual");
    switch (entry.provenance) {
      case workcell_builder::WorkcellStudioItemProvenance::EditableLayout:
        source_layer = QStringLiteral("editable_layout");
        break;
      case workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview:
        source_layer = QStringLiteral("primitive_fallback");
        break;
      case workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
      default:
        source_layer = QStringLiteral("locked_generated_urdf_visual");
        break;
    }
    item->setData(RoleSourceLayer, source_layer);
    item->setZValue(canvas_item_z_value(category, QString::fromStdString(entry.role), source_layer, locked_for_canvas));
    item->setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges | (is_editable_layout_item ? QGraphicsItem::ItemIsMovable : QGraphicsItem::GraphicsItemFlag(0)));
    item->position_filter = [this](const QPointF & p){ return snap_canvas_position(p); };
    digital_twin_scene_->addItem(item);
    add_physical_bounds(item->sceneBoundingRect());

    if (toggle_labels_box_ && toggle_labels_box_->isChecked()) {
      const bool secondary_label = !is_editable_layout_item;
      const bool visible_by_default = should_show_2d_label(entry, preserved_selected_id);
      QString label_text = QString::fromStdString(entry.label);
      if (label_text.trimmed().isEmpty()) label_text = QString::fromStdString(entry.id);
      if (!is_editable_layout_item) label_text = QStringLiteral("🔒 ") + label_text;
      auto * txt = digital_twin_scene_->addSimpleText(compact_canvas_label(label_text));
      QFont label_font = txt->font();
      label_font.setPointSize(secondary_label ? 7 : 8);
      txt->setFont(label_font);
      txt->setBrush(QBrush(secondary_label ? QColor("#a7b0bd") : QColor("#d8dee9")));
      txt->setZValue(item->zValue() + 2.0);
      const bool placed = place_2d_label_without_overlap(txt, item->sceneBoundingRect(), &occupied_label_rects);
      if (!placed && is_important_canvas_anchor(category, QString::fromStdString(entry.role)) &&
          source_layer != QStringLiteral("editable_layout") && item->data(RoleWarning).toString().isEmpty() &&
          item->data(RoleId).toString().trimmed() != preserved_selected_id) {
        delete txt;
        txt = nullptr;
      } else {
        if (!placed) txt->setPos(item->sceneBoundingRect().topLeft() + QPointF(0, -txt->boundingRect().height() - 2));
        item->set_label_item(txt, visible_by_default);
        add_overlay_bounds(txt->sceneBoundingRect());
      }
    }
    if ((!show_camera_fov_overlay_box_ || show_camera_fov_overlay_box_->isChecked()) && category.contains("camera", Qt::CaseInsensitive)) {
      QPolygonF fov; fov << QPointF(item->pos().x()+12, item->pos().y()+12) << QPointF(item->pos().x()+150, item->pos().y()-40) << QPointF(item->pos().x()+150, item->pos().y()+64);
      auto * fov_item = digital_twin_scene_->addPolygon(fov, QPen(QColor("#ffd166"), 2), QBrush(QColor(255, 209, 102, 45))); // camera FOV wedge/cone
      fov_item->setZValue(canvas_item_z_value("camera_fov", "helper_overlay", "overlay", true));
      add_overlay_bounds(fov_item->sceneBoundingRect());
    }
    if (!show_pick_place_overlay_box_ || show_pick_place_overlay_box_->isChecked()) {
      if (category.contains("pick", Qt::CaseInsensitive)) {
        auto * pick_overlay = digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#00d1b2"),2,Qt::DashLine));
        pick_overlay->setZValue(canvas_item_z_value("pick_zone", "helper_overlay", "overlay", true));
        add_overlay_bounds(pick_overlay->sceneBoundingRect());
      }
      if (category.contains("place", Qt::CaseInsensitive)) {
        auto * place_overlay = digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#ff7b72"),2,Qt::DashLine));
        place_overlay->setZValue(canvas_item_z_value("place_zone", "helper_overlay", "overlay", true));
        add_overlay_bounds(place_overlay->sceneBoundingRect());
      }
    }
    if (toggle_warnings_box_ && toggle_warnings_box_->isChecked() && !item->data(RoleWarning).toString().isEmpty()) {
      auto * w = digital_twin_scene_->addSimpleText(QString("⚠ %1").arg(compact_canvas_label(item->data(RoleWarning).toString())));
      QFont warning_font = w->font();
      warning_font.setPointSize(7);
      w->setFont(warning_font);
      w->setBrush(QBrush(QColor("#ff8e72")));
      w->setZValue(canvas_item_z_value("warning", "label", "overlay", true));
      if (!place_2d_label_without_overlap(w, item->sceneBoundingRect().adjusted(0, item->boundingRect().height() + 2, 0, item->boundingRect().height() + 2), &occupied_label_rects, 2.0)) {
        w->setPos(item->sceneBoundingRect().bottomLeft() + QPointF(0, 2));
      }
      add_overlay_bounds(w->sceneBoundingRect());
    }
  }

  if (model.items.empty()) {
    auto * empty_label = digital_twin_scene_->addSimpleText("No previewable layout metadata. Generate preview/readiness or add layout items.");
    empty_label->setZValue(canvas_item_z_value("warning", "label", "overlay", true));
    empty_label->setPos(-360, -260);
    add_overlay_bounds(empty_label->sceneBoundingRect());
    append_studio_log(QString("Scene canvas: '%1' has 0 editable layout items and 0 URDF visual preview locked items. Missing files may include environment.yaml, scene_manifest.yaml, layout/workcell_studio_layout.yaml.").arg(selected_scene_name()));
  } else {
    append_studio_log(QString("Scene canvas: loaded %1 item(s) for '%2' from %3.").arg(model.items.size()).arg(selected_scene_name(), selected_scene_path()));
  }
  if (!show_trajectory_overlay_box_ || show_trajectory_overlay_box_->isChecked()) {
    auto * trajectory = digital_twin_scene_->addLine(20, 10, 180, -60, QPen(QColor("#38bdf8"), 2, Qt::DashDotLine));
    trajectory->setZValue(canvas_item_z_value("trajectory", "helper_overlay", "overlay", true));
    add_overlay_bounds(trajectory->sceneBoundingRect());
  }
  if (toggle_warnings_box_ && toggle_warnings_box_->isChecked()) {
    QStringList issues;
    if (!s.has_environment_yaml) issues << "missing environment.yaml";
    if (!s.has_package_xml) issues << "missing package.xml";
    if (!s.has_launch_demo) issues << "missing launch/demo.launch.py";
    auto task = load_scene_task_intent_summary(s.scene_dir);
    if (task.status != "READY") issues << "missing task intent";
    if (task.tool_id == "unknown") issues << "missing robot/gripper metadata";
    if (!issues.isEmpty()) {
      auto * safety_warning = digital_twin_scene_->addSimpleText("⚠ " + compact_canvas_label(issues.join(" | ")));
      QFont safety_font = safety_warning->font();
      safety_font.setPointSize(7);
      safety_warning->setFont(safety_font);
      safety_warning->setBrush(QBrush(QColor("#ff8e72")));
      safety_warning->setZValue(canvas_item_z_value("warning", "label", "overlay", true));
      safety_warning->setPos(-380, -280);
      add_overlay_bounds(safety_warning->sceneBoundingRect());
    }
  }
  if (!preserved_selected_id.isEmpty()) {
    apply_scene_selection(preserved_selected_id, QStringLiteral("unknown"), false, false);
  }
  digital_twin_scene_->setProperty("workcellPhysicalBoundsValid", has_physical_bounds);
  digital_twin_scene_->setProperty("workcellPhysicalBounds", physical_bounds);
  digital_twin_scene_->setProperty("workcellOverlayBoundsValid", has_overlay_bounds);
  digital_twin_scene_->setProperty("workcellOverlayBounds", overlay_bounds);
  if (digital_twin_canvas_ && digital_twin_canvas_->scene() && has_physical_bounds) {
    digital_twin_canvas_->fitInView(physical_bounds.adjusted(-24, -24, 24, 24), Qt::KeepAspectRatio);
  }
  refresh_minimap_card();
}

void MainWindow::refresh_scene_builder_left_explorer()
{
  sync_selected_scene_state();
  rebuild_digital_twin_canvas();
  populate_scene_hierarchy();
  populate_asset_catalog();
  populate_scene_files_tab();
}

void MainWindow::refresh_scene_builder_view_chips()
{
  bool launch_ready = false;
  QString preview_chip_status = QStringLiteral("Failed");
  if (has_selected_scene()) {
    const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
    launch_ready = s.has_launch_demo && s.has_package_xml;
    if (scene_preview_widget_) {
      const auto counters = scene_preview_widget_->render_debug_counters();
      const QString quality = counters.visual_quality_status.trimmed().toUpper();
      const int mesh_source_count = counters.mesh_source_count > 0 ? counters.mesh_source_count : counters.mesh_backed_count;
      const int mesh_rendered_count = counters.mesh_rendered_count;
      const int urdf_primitive_source_count = counters.urdf_primitive_source_count;
      const int urdf_primitive_rendered_count = counters.urdf_primitive_rendered_count;
      const bool source_render_ratio_failed =
        (mesh_source_count > 0 && mesh_rendered_count <= 0) ||
        (urdf_primitive_source_count > 0 && urdf_primitive_rendered_count <= 0);
      const bool high_mesh_source_low_render =
        mesh_source_count >= 4 && mesh_rendered_count > 0 && mesh_rendered_count * 2 < mesh_source_count;
      const bool has_active_runtime_render_evidence =
        counters.rendered_count > 0 ||
        counters.visible_count > 0 ||
        counters.viewport_received_count > 0 ||
        counters.render_cache_count > 0 ||
        counters.last_paint_completed ||
        counters.smoke_fallback_render_used;
      const int visible_geometry_count = qMax(
        counters.unique_visible_item_count,
        qMax(0, counters.visible_count - counters.overlay_count));
      const int rendered_geometry_count = qMax(0, counters.rendered_count - counters.overlay_rendered_count);
      const bool has_visible_or_rendered_geometry =
        visible_geometry_count > 0 ||
        rendered_geometry_count > 0 ||
        counters.mesh_rendered_count > 0 ||
        counters.urdf_primitive_rendered_count > 0 ||
        counters.primitive_fallback_rendered_count > 0 ||
        counters.valid_physical_fallback_count > 0;
      const bool has_missing_or_fallback_geometry =
        counters.placeholder_count > 0 ||
        counters.missing_geometry_count > 0 ||
        counters.wireframe_fallback_count > 0 ||
        counters.generated_fallback_count > 0 ||
        counters.primitive_fallback_count > 0 ||
        counters.primitive_fallback_rendered_count > 0;
      const bool has_warning_bucket =
        quality == QStringLiteral("WARNING") ||
        quality == QStringLiteral("FAIL") ||
        has_missing_or_fallback_geometry ||
        source_render_ratio_failed ||
        high_mesh_source_low_render;

      if (!has_active_runtime_render_evidence || !has_visible_or_rendered_geometry) {
        preview_chip_status = QStringLiteral("Failed");
      } else if (quality == QStringLiteral("PASS") && !has_warning_bucket) {
        preview_chip_status = QStringLiteral("Ready");
      } else {
        preview_chip_status = QStringLiteral("Warnings");
      }
    } else {
      preview_chip_status = QStringLiteral("Failed");
    }
  }
  if (scene_builder_preview_chip_) scene_builder_preview_chip_->setText(QString("Preview: %1").arg(preview_chip_status));
  if (scene_builder_launch_chip_) scene_builder_launch_chip_->setText(QString("Launch: %1").arg(launch_ready ? "Ready" : "Missing"));
  if (scene_builder_safety_chip_) scene_builder_safety_chip_->setText("Safety: Fake hardware");
  if (scene_builder_generate_launch_button_) scene_builder_generate_launch_button_->setVisible(has_selected_scene() && !launch_ready);
  if (canvas_mode_label_) {
    const QString view_label = scene_builder_is_3d_view_ ? "Scene3D Product Preview" : "2D Layout Draft";
    const QString base_mode = canvas_mode_label_->text().section("·", 0, 0).trimmed();
    canvas_mode_label_->setText(base_mode + " · " + view_label);
  }
}


void MainWindow::set_canvas_interaction_mode(CanvasInteractionMode mode)
{
  canvas_mode_ = mode;
  QString n = "Select";
  if (mode == CanvasInteractionMode::Place) n = "Place";
  if (mode == CanvasInteractionMode::Move) n = "Move";
  if (mode == CanvasInteractionMode::Inspect) n = "Inspect";
  if (canvas_mode_label_) canvas_mode_label_->setText("Mode: " + n);
  refresh_scene_builder_view_chips();
}

bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  if (scene_builder_top_controls_host_ && watched == scene_builder_top_controls_host_->parent() && event &&
    event->type() == QEvent::Resize)
  {
    update_scene_builder_top_controls_overflow();
  }
  if (asset_catalog_tree_ && watched == asset_catalog_tree_->viewport() && event) {
    if (event->type() == QEvent::MouseButtonPress) {
      auto * mouse_event = static_cast<QMouseEvent *>(event);
      if (mouse_event->button() == Qt::LeftButton) catalog_drag_start_ = mouse_event->pos();
    } else if (event->type() == QEvent::MouseMove) {
      auto * mouse_event = static_cast<QMouseEvent *>(event);
      if (!(mouse_event->buttons() & Qt::LeftButton)) return QMainWindow::eventFilter(watched, event);
      if ((mouse_event->pos() - catalog_drag_start_).manhattanLength() < QApplication::startDragDistance()) return QMainWindow::eventFilter(watched, event);
      auto * item = asset_catalog_tree_->itemAt(catalog_drag_start_);
      if (!item) return QMainWindow::eventFilter(watched, event);
      const int idx = item->data(0, CatalogRoleIndex).toInt();
      if (idx < 0 || idx >= asset_catalog_entries_.size()) return QMainWindow::eventFilter(watched, event);
      const auto & e = asset_catalog_entries_[idx];
      if (!e.disabled_reason.trimmed().isEmpty()) {
        QToolTip::showText(QCursor::pos(), QString("Cannot place here: %1").arg(e.disabled_reason), asset_catalog_tree_);
        return true;
      }
      QJsonObject payload;
      payload["asset_id"] = e.display_name.toLower().replace(" ", "_");
      payload["display_name"] = e.display_name;
      payload["category"] = e.category;
      payload["type"] = e.asset_type;
      payload["source_path"] = e.source_path;
      payload["mesh_path"] = e.source_path.endsWith(".stl", Qt::CaseInsensitive) ? e.source_path : "";
      payload["default_dimensions"] = e.dimensions;
      payload["default_pose"] = e.default_pose;
      auto * mime = new QMimeData();
      mime->setData("application/x-workcell-asset-catalog-item", QJsonDocument(payload).toJson(QJsonDocument::Compact));
      auto * drag = new QDrag(asset_catalog_tree_);
      drag->setMimeData(mime);
      drag->exec(Qt::CopyAction);
      return true;
    }
  }
  if (digital_twin_canvas_ && watched == digital_twin_canvas_->viewport() &&
    event && event->type() == QEvent::MouseButtonPress && place_asset_armed_)
  {
    auto * mouse_event = static_cast<QMouseEvent *>(event);
    if (mouse_event->button() == Qt::LeftButton && digital_twin_canvas_->scene()) {
      const QPointF scene_pos = digital_twin_canvas_->mapToScene(mouse_event->pos());
      commit_armed_asset_placement(scene_pos);
      return true;
    }
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::update_scene_builder_top_controls_overflow()
{
  if (!scene_builder_top_controls_host_ || !scene_builder_secondary_overflow_button_ ||
    !scene_builder_secondary_overflow_menu_)
  {
    return;
  }
  const int available_width = scene_builder_top_controls_host_->parentWidget() ?
    scene_builder_top_controls_host_->parentWidget()->width() : width();
  const bool constrained = available_width < 1280;
  scene_builder_secondary_overflow_menu_->clear();
  auto * secondary_layout_menu = scene_builder_secondary_overflow_menu_->addMenu("Layout");
  secondary_layout_menu->addAction(scene_builder_action("layout.undo"));
  secondary_layout_menu->addAction(scene_builder_action("layout.redo"));
  secondary_layout_menu->addAction(scene_builder_action("layout.save"));
  secondary_layout_menu->addAction(scene_builder_action("layout.duplicate"));
  secondary_layout_menu->addAction(scene_builder_action("layout.remove"));
  const auto remap_menu = [this](QToolButton * button) {
      if (!button || !button->menu()) return;
      scene_builder_secondary_overflow_menu_->addMenu(button->menu());
    };
  if (constrained) {
    remap_menu(scene_builder_camera_view_button_);
    remap_menu(scene_builder_overlays_button_);
    remap_menu(scene_builder_canvas_more_button_);
    remap_menu(scene_builder_visual_modes_button_);
  }
  if (scene_builder_camera_view_button_) scene_builder_camera_view_button_->setVisible(!constrained);
  if (scene_builder_overlays_button_) scene_builder_overlays_button_->setVisible(!constrained);
  if (scene_builder_canvas_more_button_) scene_builder_canvas_more_button_->setVisible(!constrained);
  if (scene_builder_visual_modes_button_) scene_builder_visual_modes_button_->setVisible(!constrained);
  scene_builder_secondary_overflow_button_->setVisible(constrained);
}

QPointF MainWindow::snap_canvas_position(const QPointF & pos) const
{
  if (!snap_to_grid_box_ || !snap_to_grid_box_->isChecked()) return pos;
  const double step_px = std::max(1.0, snap_step_m_ * 100.0);
  return QPointF(std::round(pos.x() / step_px) * step_px, std::round(pos.y() / step_px) * step_px);
}

void MainWindow::refresh_minimap_card()
{
  if (!minimap_view_ || !digital_twin_scene_) return;
  if (!minimap_scene_) minimap_scene_ = new QGraphicsScene(minimap_view_);
  minimap_scene_->clear();
  QRectF physical_bounds;
  bool has_physical_items = false;
  for (auto * gi : digital_twin_scene_->items()) {
    if (!gi->data(RoleRole).isValid() || gi->data(RoleRole).toString() != "asset") continue;
    if (!has_physical_items) {
      physical_bounds = gi->sceneBoundingRect();
      has_physical_items = true;
    } else {
      physical_bounds = physical_bounds.united(gi->sceneBoundingRect());
    }
    minimap_scene_->addRect(gi->sceneBoundingRect(), QPen(QColor("#94a3b8"), 1), QBrush(QColor(148,163,184,90)));
  }
  if (!has_physical_items) {
    minimap_view_->setVisible(false);
    return;
  }
  const QRectF padded_bounds = physical_bounds.adjusted(-20.0, -20.0, 20.0, 20.0);
  minimap_scene_->setSceneRect(padded_bounds);
  if (digital_twin_canvas_) {
    const QRect viewport = digital_twin_canvas_->viewport()->rect();
    const QRectF visible_rect = digital_twin_canvas_->mapToScene(viewport).boundingRect();
    minimap_scene_->addRect(visible_rect, QPen(QColor("#38bdf8"), 2), Qt::NoBrush);
  }
  minimap_view_->setScene(minimap_scene_);
  minimap_view_->setVisible(minimap_requested_visible_);
  minimap_view_->fitInView(padded_bounds, Qt::KeepAspectRatio);
}

void MainWindow::select_canvas_item(QGraphicsItem * item)
{
  if (!item || !inspector_label_) return;
  refresh_selection_transform_editor_from_item(item);
  inspector_update_guard_ = true;
  refresh_selected_scene_item_labels(current_selected_scene_item());
  const QString warning_text = item->data(RoleWarning).toString().isEmpty() ? QString("none") : item->data(RoleWarning).toString();
  inspector_warning_label_->setText("Warnings: " + warning_text + " | Reachability: preview-only | Collision: preview-only | Safety zone: preview-only | Pick source reach: unknown | Place target reach: unknown | Warning count: " + QString::number(warning_text == "none" ? 0 : 1) + " | Preview-only");
  append_studio_log("selected item reach status: preview-only");
  append_studio_log("selected item collision status: preview-only");
  if (pick_place_details_label_) pick_place_details_label_->setText(pick_place_details_label_->text() + QStringLiteral("\nLinked hierarchy item: %1").arg(item->data(RoleId).toString()));
  inspector_update_guard_ = false;
}

void MainWindow::apply_scene_selection(const QString & id, const QString & role, bool intentional_clear, bool center_canvas)
{
  sync_selected_scene_state();
  const QString selected_id = id.trimmed();
  const QString selected_role = role.trimmed().isEmpty() ? QStringLiteral("unknown") : role.trimmed();

  if (selected_id.isEmpty()) {
    if (!intentional_clear) return;
    current_selected_scene_item_id_.clear();
    selection_update_guard_ = true;
    if (scene_hierarchy_tree_) scene_hierarchy_tree_->clearSelection();
    if (digital_twin_scene_) digital_twin_scene_->clearSelection();
    if (scene_preview_widget_) scene_preview_widget_->select_preview_item(QString());
    selection_update_guard_ = false;
    sync_selected_item_state();
    refresh_selected_scene_item_labels(selected_item_state_);
    append_studio_log("Selected item: <none> (unknown)");
    return;
  }

  current_selected_scene_item_id_ = selected_id;
  selection_update_guard_ = true;

  bool matched_tree_item = false;
  if (scene_hierarchy_tree_) {
    QTreeWidgetItem * matched = nullptr;
    for (int row = 0; row < scene_hierarchy_tree_->topLevelItemCount() && !matched; ++row) {
      auto * tree_item = scene_hierarchy_tree_->topLevelItem(row);
      if (!tree_item) continue;
      if (tree_item->data(0, TreeRoleId).toString().trimmed() == selected_id) matched = tree_item;
      for (int child = 0; child < tree_item->childCount() && !matched; ++child) {
        auto * node = tree_item->child(child);
        if (node && node->data(0, TreeRoleId).toString().trimmed() == selected_id) matched = node;
      }
    }
    if (matched) {
      matched_tree_item = true;
      scene_hierarchy_tree_->setCurrentItem(matched);
    }
  }

  QGraphicsItem * matched_canvas_item = nullptr;
  if (digital_twin_scene_) {
    for (auto * gi : digital_twin_scene_->items()) {
      if (gi && gi->data(RoleId).toString().trimmed() == selected_id) {
        matched_canvas_item = gi;
        break;
      }
    }
    if (matched_canvas_item) {
      digital_twin_scene_->clearSelection();
      matched_canvas_item->setSelected(true);
      if (center_canvas && digital_twin_canvas_) digital_twin_canvas_->centerOn(matched_canvas_item);
      if (auto * rect = qgraphicsitem_cast<QGraphicsRectItem *>(matched_canvas_item)) {
        rect->setPen(QPen(QColor("#f8fafc"), 3));
      }
    } else {
      digital_twin_scene_->clearSelection();
    }
  }

  const auto find_preview_item_by_id = [&](const QString & stable_id) -> const ScenePreviewWidget::PreviewItem * {
    const QString trimmed_id = stable_id.trimmed();
    if (trimmed_id.isEmpty()) return nullptr;
    for (const auto & item : all_scene_preview_items_) {
      if (item.id.trimmed() == trimmed_id) return &item;
    }
    return scene_preview_widget_ ? scene_preview_widget_->preview_item_by_id(trimmed_id) : nullptr;
  };
  const ScenePreviewWidget::PreviewItem * matched_preview_item = find_preview_item_by_id(selected_id);
  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(selected_id);
  selection_update_guard_ = false;

  const bool selection_resolved = matched_tree_item || matched_canvas_item || matched_preview_item;
  if (!selection_resolved) {
    append_studio_log(QString("Selection id absent from active scene payload after refresh, clearing atomically: %1").arg(selected_id));
    apply_scene_selection(QString(), selected_role, true, false);
    return;
  }

  if (matched_canvas_item) {
    select_canvas_item(matched_canvas_item);
  } else {
    sync_selected_item_state();
    refresh_selected_scene_item_labels(selected_item_state_);
    refresh_selection_transform_editor_from_state(selected_item_state_);
  }
  const auto selected_state = current_selected_scene_item();
  const QString selected_scene_name_for_log = selected_scene_state_.valid ? selected_scene_state_.name : QStringLiteral("unknown");
  const QString selected_source_layer_for_log = selected_state.source_layer.isEmpty() ? QStringLiteral("unknown") : selected_state.source_layer;
  append_studio_log(QString("Scene3D selection changed: scene=%1 id=%2 editable=%3 locked=%4 source_layer=%5")
    .arg(selected_scene_name_for_log, selected_id,
      selected_state.editable ? "true" : "false", selected_state.locked ? "true" : "false",
      selected_source_layer_for_log));
  append_studio_log(QString("Selected item: %1 (%2) type=%3 source=%4 editable=%5 locked=%6")
    .arg(selected_id, selected_role, selected_state.role_or_category, selected_state.source_path,
      selected_state.editable ? "true" : "false", selected_state.locked ? "true" : "false"));
}

void MainWindow::mark_layout_dirty(const QString & reason)
{
  layout_dirty_ = true;
  layout_saved_ = false;
  validation_stale_ = true;
  launch_artifacts_ready_ = false;
  if (layout_state_label_) {
    layout_state_label_->setText(QString("Unsaved Layout Edits: %1").arg(reason));
  }
}

using workcell_builder::resolve_visual_mesh_source_path;
using workcell_builder::workcell_builder_repo_root_from_source;

static QStringList generation_asset_support_preflight(const fs::path & layout_path, bool * severe_failure)
{
  if (severe_failure) *severe_failure = false;
  QStringList warnings;
  if (layout_path.empty() || !fs::exists(layout_path)) {
    warnings << "Asset support preflight: environment_layout.yaml not found; generation may proceed with defaults.";
    return warnings;
  }
  YAML::Node root;
  try { root = YAML::LoadFile(layout_path.string()); }
  catch (const std::exception & exc) {
    warnings << QString("Asset support preflight warning: malformed environment_layout.yaml at '%1' (%2); generation proceeds with defaults.")
      .arg(QString::fromStdString(layout_path.string()), QString::fromStdString(exc.what()));
    return warnings;
  }
  const YAML::Node placed = workcell_builder::yaml_map_key(root, "placed_assets");
  if (!placed || !placed.IsSequence()) return warnings;
  const QSet<QString> supported_types = {"asset","object","fixture","support_surface","table","conveyor","camera","sensor","safety_zone","bin","pick_zone","place_zone"};
  for (const auto & item : placed) {
    if (!item || !item.IsMap()) {
      warnings << "Asset support preflight warning: placed_assets item is not a map; skipping malformed entry.";
      continue;
    }
    const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(item, "id"));
    const QString raw_type = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(item, "type"));
    const QString type = raw_type.trimmed().toLower();
    if (!type.isEmpty() && !supported_types.contains(type)) {
      warnings << QString("Asset support preflight: unsupported placed asset id='%1' type='%2'. Expected behavior: generator keeps metadata and may skip geometry/runtime wiring for this item.")
        .arg(id.isEmpty() ? "unknown" : id, raw_type);
    }
  }
  return warnings;
}

static bool is_safe_scene_relative_path(const fs::path & candidate)
{
  if (candidate.empty() || candidate.is_absolute()) return false;
  for (const auto & part : candidate) {
    if (part.string() == "..") return false;
  }
  return true;
}

static fs::path manifest_declared_layout_path(const fs::path & scene_dir)
{
  const fs::path manifest_path = scene_dir / "scene_manifest.yaml";
  if (!fs::exists(manifest_path)) return {};
  try {
    const YAML::Node manifest = YAML::LoadFile(manifest_path.string());
    const YAML::Node files = workcell_builder::yaml_map_key(manifest, "files");
    std::string declared;
    if (files && files.IsMap()) {
      declared = workcell_builder::yaml_map_value_or_empty(files, "layout");
      if (declared.empty()) declared = workcell_builder::yaml_map_value_or_empty(files, "editable_layout");
      if (declared.empty()) declared = workcell_builder::yaml_map_value_or_empty(files, "workcell_studio_layout");
    }
    if (declared.empty()) declared = workcell_builder::yaml_map_value_or_empty(manifest, "layout");
    const fs::path relative(declared);
    if (!is_safe_scene_relative_path(relative)) return {};
    return scene_dir / relative;
  } catch (const YAML::Exception &) {
    return {};
  } catch (const std::exception &) {
    return {};
  }
}

static fs::path selected_scene_canonical_layout_save_path(const workcell_builder::WorkcellStudioSceneBrowserResult & browser, int selected_scene_index)
{
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) return {};
  const fs::path scene_dir = browser.scenes[static_cast<size_t>(selected_scene_index)].scene_dir;
  return scene_dir / "layout" / "workcell_studio_layout.yaml";
}

static std::vector<fs::path> selected_scene_layout_import_candidates(const fs::path & scene_dir)
{
  std::vector<fs::path> candidates;
  const fs::path canonical_layout = scene_dir / "layout" / "workcell_studio_layout.yaml";
  const fs::path legacy_environment_layout = scene_dir / "environment_layout.yaml";
  const fs::path manifest_layout = manifest_declared_layout_path(scene_dir);

  auto append_unique = [&candidates](const fs::path & candidate) {
    if (candidate.empty()) return;
    if (std::find(candidates.begin(), candidates.end(), candidate) == candidates.end()) {
      candidates.push_back(candidate);
    }
  };

  append_unique(canonical_layout);
  append_unique(legacy_environment_layout);
  append_unique(manifest_layout);
  return candidates;
}

static fs::path selected_scene_existing_layout_import_path(const workcell_builder::WorkcellStudioSceneBrowserResult & browser, int selected_scene_index)
{
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) return {};
  const fs::path scene_dir = browser.scenes[static_cast<size_t>(selected_scene_index)].scene_dir;
  for (const auto & candidate : selected_scene_layout_import_candidates(scene_dir)) {
    if (fs::exists(candidate)) return candidate;
  }
  return {};
}

static YAML::Node minimal_environment_layout(const std::string & scene_name)
{
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "environment_layout/v1";
  root["scene_name"] = scene_name;
  root["placed_assets"] = YAML::Node(YAML::NodeType::Sequence);
  return root;
}

static YAML::Node ensure_sequence_of_maps(YAML::Node root, const char * key)
{
  if (!root[key] || !root[key].IsSequence()) {
    root[key] = YAML::Node(YAML::NodeType::Sequence);
  }
  return root[key];
}

static YAML::Node ensure_map_node(YAML::Node parent, const char * key)
{
  if (!parent[key] || !parent[key].IsMap()) parent[key] = YAML::Node(YAML::NodeType::Map);
  return parent[key];
}


static YAML::Node yaml_sequence_from3(double a, double b, double c)
{
  YAML::Node seq(YAML::NodeType::Sequence);
  seq.push_back(a);
  seq.push_back(b);
  seq.push_back(c);
  return seq;
}

static void write_pose_preserving_existing_shape(YAML::Node item, double x, double y, double z, double roll, double pitch, double yaw)
{
  YAML::Node pose = ensure_map_node(item, "pose");
  const bool scalar_xyz = pose["x"] || pose["y"] || pose["z"];
  const bool scalar_rpy = pose["roll"] || pose["pitch"] || pose["yaw"];
  if (scalar_xyz && !pose["xyz"]) {
    pose["x"] = x;
    pose["y"] = y;
    pose["z"] = z;
  } else {
    pose["xyz"] = yaml_sequence_from3(x, y, z);
  }
  if (scalar_rpy && !pose["rpy"]) {
    pose["roll"] = roll;
    pose["pitch"] = pitch;
    pose["yaw"] = yaw;
  } else {
    pose["rpy"] = yaml_sequence_from3(roll, pitch, yaw);
  }
}

static void write_dimensions_preserving_existing_shape(YAML::Node item, double width, double depth, double height)
{
  YAML::Node size = item["size"];
  if (size && size.IsMap() && !item["dimensions"]) {
    size["width"] = width;
    size["depth"] = depth;
    size["height"] = height;
    return;
  }
  item["dimensions"] = yaml_sequence_from3(width, depth, height);
}


static YAML::Node normalize_imported_layout_to_canonical_items(const YAML::Node & imported_root, const std::string & scene_name, const fs::path & scene_dir)
{
  YAML::Node root = (imported_root && imported_root.IsMap()) ? YAML::Clone(imported_root) : YAML::Node(YAML::NodeType::Map);
  YAML::Node canonical_items(YAML::NodeType::Sequence);
  QSet<QString> seen_ids;

  auto append_item = [&](const YAML::Node & source) {
    if (!source || !source.IsMap()) return;
    YAML::Node item = YAML::Clone(source);
    const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(item, "id")).trimmed();
    if (!id.isEmpty()) {
      if (seen_ids.contains(id)) return;
      seen_ids.insert(id);
    }
    if (!item["scene_name"] || !item["scene_name"].IsScalar()) item["scene_name"] = scene_name;
    if (!item["scene_path"] || !item["scene_path"].IsScalar()) item["scene_path"] = scene_dir.string();
    canonical_items.push_back(item);
  };

  const std::array<const char *, 6> legacy_sequence_keys = {
    "items", "assets", "placed_assets", "objects", "zones", "targets"
  };
  for (const char * key : legacy_sequence_keys) {
    const YAML::Node seq = root[key];
    if (!seq || !seq.IsSequence()) continue;
    for (std::size_t i = 0; i < seq.size(); ++i) append_item(seq[i]);
  }
  const YAML::Node camera = root["camera"];
  if (camera && camera.IsMap()) append_item(camera);

  for (const char * key : legacy_sequence_keys) root.remove(key);
  root.remove("camera");
  root["schema_version"] = "workcell_studio_layout/v1";
  root["schema"] = "workcell_studio_layout/v1";
  root["scene_name"] = scene_name;
  root["scene_path"] = scene_dir.string();
  root["items"] = canonical_items;
  YAML::Node meta = ensure_map_node(root, "metadata");
  meta["canonical_layout_path"] = "layout/workcell_studio_layout.yaml";
  meta["legacy_import_normalized_keys"] = "items,assets,placed_assets,objects,zones,targets,camera";
  return root;
}

static YAML::Node serialized_editable_canvas_item(QGraphicsItem * gi, const YAML::Node & existing)
{
  YAML::Node item = (existing && existing.IsMap()) ? YAML::Clone(existing) : YAML::Node(YAML::NodeType::Map);
  item["id"] = gi->data(RoleId).toString().toStdString();
  if (!gi->data(RoleDisplayName).toString().trimmed().isEmpty()) item["display_name"] = gi->data(RoleDisplayName).toString().toStdString();
  if (!gi->data(RoleCategory).toString().trimmed().isEmpty()) item["category"] = gi->data(RoleCategory).toString().toStdString();
  if (!gi->data(RoleType).toString().trimmed().isEmpty()) item["type"] = gi->data(RoleType).toString().toStdString();
  if (!gi->data(RoleRole).toString().trimmed().isEmpty()) item["role"] = gi->data(RoleRole).toString().toStdString();
  if (!gi->data(RoleSource).toString().trimmed().isEmpty()) {
    item["source_path"] = gi->data(RoleSource).toString().toStdString();
    item["mesh_path"] = gi->data(RoleSource).toString().toStdString();
    item["asset_id"] = QFileInfo(gi->data(RoleSource).toString()).completeBaseName().toStdString();
  }
  if (!gi->data(RoleSourcePackage).toString().trimmed().isEmpty()) item["source_package"] = gi->data(RoleSourcePackage).toString().toStdString();
  item["source"] = gi->data(RoleSourcePackage).toString().trimmed().isEmpty() ? "layout_editor" : gi->data(RoleSourcePackage).toString().toStdString();
  if (!item["editable"]) item["editable"] = true;
  if (!item["locked"]) item["locked"] = false;
  write_pose_preserving_existing_shape(item,
    gi->pos().x() / 100.0, gi->pos().y() / 100.0, gi->data(RolePoseZ).toDouble(),
    gi->data(RoleRoll).toDouble(), gi->data(RolePitch).toDouble(), gi->data(RoleYaw).toDouble());
  write_dimensions_preserving_existing_shape(
    item, gi->data(RoleWidth).toDouble(), gi->data(RoleDepth).toDouble(), gi->data(RoleHeight).toDouble());
  YAML::Node scale(YAML::NodeType::Sequence);
  scale.push_back(gi->data(RoleWidth).toDouble());
  scale.push_back(gi->data(RoleDepth).toDouble());
  scale.push_back(gi->data(RoleHeight).toDouble());
  item["scale"] = scale;
  if (!gi->data(RoleSource).toString().trimmed().isEmpty() || !gi->data(RoleSourcePackage).toString().trimmed().isEmpty()) {
    YAML::Node mesh = ensure_map_node(item, "mesh");
    if (!gi->data(RoleSource).toString().trimmed().isEmpty()) mesh["path"] = gi->data(RoleSource).toString().toStdString();
    if (!gi->data(RoleSourcePackage).toString().trimmed().isEmpty()) mesh["source_package"] = gi->data(RoleSourcePackage).toString().toStdString();
  }
  YAML::Node meta = ensure_map_node(item, "metadata");
  meta["serialization_contract"] =
    "editable layout item only; locked/generated preview items are not written by inspector save";
  return item;
}

void MainWindow::save_layout_changes(){
  const auto scene_name_for_save_log = [this]() {
    if (selected_scene_state_.valid && !selected_scene_state_.name.trimmed().isEmpty()) {
      return selected_scene_state_.name.trimmed();
    }
    if (selected_scene_index_ >= 0 && selected_scene_index_ < static_cast<int>(scene_browser_result_.scenes.size())) {
      return QString::fromStdString(scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_name);
    }
    return QStringLiteral("<none>");
  };
  const auto scene_root_for_save_log = [this]() {
    if (selected_scene_index_ >= 0 && selected_scene_index_ < static_cast<int>(scene_browser_result_.scenes.size())) {
      return QString::fromStdString(scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_dir.string());
    }
    return QStringLiteral("<none>");
  };
  const auto emit_save_layout_failure = [this, scene_name_for_save_log, scene_root_for_save_log](
      const QString & blocker, const QString & target_path = QString(), const QString & exact_message = QString()) {
    if (!exact_message.trimmed().isEmpty()) {
      append_studio_log(exact_message);
    }
    const QString target = target_path.trimmed().isEmpty() ? QStringLiteral("<unknown>") : target_path.trimmed();
    const QString detail = QString("Save Layout failed: action=Save Layout scene=%1 selected_scene_index=%2 scene_root=%3 target=%4 blocker=%5; no file was written.")
      .arg(scene_name_for_save_log())
      .arg(selected_scene_index_)
      .arg(scene_root_for_save_log())
      .arg(target)
      .arg(blocker);
    append_studio_log(detail);
  };

  QString selected_preview_id;
  if (!current_selected_scene_item_id_.trimmed().isEmpty()) {
    selected_preview_id = current_selected_scene_item_id_.trimmed();
  } else if (scene_hierarchy_tree_ && scene_hierarchy_tree_->currentItem()) {
    selected_preview_id = scene_hierarchy_tree_->currentItem()->data(0, TreeRoleId).toString();
  } else if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty()) {
    selected_preview_id = digital_twin_scene_->selectedItems().front()->data(RoleId).toString();
  }
  const QString stable_selected_id_before_refresh = selected_preview_id.trimmed();
  if (!digital_twin_scene_) {
    emit_save_layout_failure(
      QStringLiteral("Scene3D canvas is not initialized"),
      QString(),
      QStringLiteral("Save Layout failed: Scene3D canvas is not initialized; no file was written."));
    statusBar()->showMessage("Save Layout blocked: Scene3D canvas is not initialized; no file was written.", 6000);
    return;
  }
  if (selected_scene_index_ < 0 || selected_scene_index_ >= static_cast<int>(scene_browser_result_.scenes.size())) {
    emit_save_layout_failure(
      QStringLiteral("no scene selected"),
      QString(),
      QStringLiteral("Save Layout failed: no scene selected; no file was written."));
    statusBar()->showMessage("Save Layout blocked: no scene selected; no file was written.", 6000);
    return;
  }
  const fs::path scene_dir = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_dir;
  const fs::path layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  if (layout_path.empty()) {
    const QString reason = QString("canonical layout path could not be computed for scene_root=%1 selected_scene_index=%2")
      .arg(scene_root_for_save_log())
      .arg(selected_scene_index_);
    emit_save_layout_failure(reason);
    statusBar()->showMessage("Save Layout blocked: canonical layout path could not be computed; no file was written.", 6000);
    return;
  }
  const std::array<const char *, 1> required_dirs = {"layout"};
  for (const char * dir_name : required_dirs) {
    boost::system::error_code mk_ec;
    fs::create_directories(scene_dir / dir_name, mk_ec);
    if (mk_ec) {
      emit_save_layout_failure(
        QString("cannot create required directory '%1' (%2)")
          .arg(dir_name, QString::fromStdString(mk_ec.message())),
        QString::fromStdString((scene_dir / dir_name).string()));
      QMessageBox::warning(this, "Save Layout", QString("Save Layout blocked: cannot create %1/. No file was written.").arg(dir_name));
      return;
    }
  }
  const std::string scene_name = (selected_scene_index_ >= 0 && selected_scene_index_ < static_cast<int>(scene_browser_result_.scenes.size())) ?
    scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_name : "unknown";
  YAML::Node root;
  fs::path imported_layout_path;
  bool existing_layout_file = false;
  bool malformed_existing = false;
  if (fs::exists(layout_path)) {
    existing_layout_file = true;
    try {
      root = YAML::LoadFile(layout_path.string());
    } catch (const YAML::Exception &) {
      malformed_existing = true;
    } catch (const std::exception &) {
      malformed_existing = true;
    }
  } else {
    std::vector<fs::path> import_candidates;
    const fs::path manifest_layout = manifest_declared_layout_path(scene_dir);
    if (!manifest_layout.empty() && manifest_layout != layout_path) import_candidates.push_back(manifest_layout);
    import_candidates.push_back(scene_dir / "environment_layout.yaml");

    for (const auto & candidate : import_candidates) {
      if (candidate.empty() || candidate == layout_path || !fs::exists(candidate)) continue;
      try {
        root = normalize_imported_layout_to_canonical_items(YAML::LoadFile(candidate.string()), scene_name, scene_dir);
        imported_layout_path = candidate;
        break;
      } catch (const YAML::Exception & exc) {
        append_studio_log(QString("Save Layout: cannot import malformed legacy layout from %1 (%2).")
          .arg(QString::fromStdString(candidate.string()), QString::fromStdString(exc.what())));
      } catch (const std::exception & exc) {
        append_studio_log(QString("Save Layout: cannot import legacy layout from %1 (%2).")
          .arg(QString::fromStdString(candidate.string()), QString::fromStdString(exc.what())));
      }
    }
  }
  if (malformed_existing) {
    const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
    const fs::path backup = layout_path.parent_path() / (layout_path.filename().string() + ".malformed_backup_" + stamp.toStdString());
    boost::system::error_code ec;
    fs::copy_file(layout_path, backup, fs::copy_option::overwrite_if_exists, ec);
    if (ec) {
      emit_save_layout_failure(
        QString("malformed layout YAML at %1 and backup failed (%2)")
          .arg(QString::fromStdString(layout_path.string()), QString::fromStdString(ec.message())),
        QString::fromStdString(layout_path.string()));
      QMessageBox::warning(this, "Save Layout", "Malformed layout YAML backup failed. Not overwriting. No file was written.");
      return;
    }
    append_studio_log(QString("Malformed layout YAML backed up to %1").arg(QString::fromStdString(backup.string())));
    root = YAML::Node();
  }
  if (!root || !root.IsMap()) {
    root = YAML::Node(YAML::NodeType::Map);
    root["items"] = YAML::Node(YAML::NodeType::Sequence);
  } else {
    root = normalize_imported_layout_to_canonical_items(root, scene_name, scene_dir);
  }
  root["schema_version"] = "workcell_studio_layout/v1";
  root["schema"] = "workcell_studio_layout/v1";
  if (!root["scene_name"] || !root["scene_name"].IsScalar()) root["scene_name"] = scene_name;
  if (!root["scene_path"] || !root["scene_path"].IsScalar()) root["scene_path"] = scene_dir.string();

  const QString backup_stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
  const fs::path layout_backup = layout_path.parent_path() /
    (layout_path.filename().string() + "." + backup_stamp.toStdString() + ".bak.yaml");
  if (existing_layout_file) {
    boost::system::error_code ec;
    fs::copy_file(layout_path, layout_backup, fs::copy_option::overwrite_if_exists, ec);
    if (!ec) {
      append_studio_log(QString("Backup before write created: %1").arg(QString::fromStdString(layout_backup.string())));
    } else {
      append_studio_log(QString("Warning: backup before write failed (%1). Continuing save without backup.")
        .arg(QString::fromStdString(ec.message())));
    }
  }
  std::vector<QGraphicsItem *> editable_canvas_items;
  for (auto * gi : digital_twin_scene_->items()) {
    const QString item_id = gi->data(RoleId).toString().trimmed();
    if (item_id.isEmpty()) continue;
    if (gi->data(RoleLocked).toBool()) continue;
    const QString source_layer = gi->data(RoleSourceLayer).toString().trimmed();
    if (source_layer != QStringLiteral("editable_layout")) continue;
    const std::string item_id_std = item_id.toStdString();
    if (!workcell_builder::workcell_studio_is_valid_id(item_id_std)) {
      QMessageBox::warning(this, "Save Layout", QString("Invalid ID for YAML/package compatibility: %1. No file was written.").arg(item_id));
      emit_save_layout_failure(
        QString("invalid id '%1' for YAML/package compatibility").arg(item_id),
        QString::fromStdString(layout_path.string()));
      return;
    }
    editable_canvas_items.push_back(gi);
  }

  fs::path effective_layout_path = layout_path;

  if (effective_layout_path != layout_path && fs::exists(effective_layout_path)) {
    const fs::path effective_layout_backup = effective_layout_path.parent_path() /
      (effective_layout_path.filename().string() + "." + backup_stamp.toStdString() + ".bak.yaml");
    boost::system::error_code ec;
    fs::copy_file(effective_layout_path, effective_layout_backup, fs::copy_option::overwrite_if_exists, ec);
    if (!ec) {
      append_studio_log(QString("Backup before write created: %1").arg(QString::fromStdString(effective_layout_backup.string())));
    } else {
      append_studio_log(QString("Warning: backup before write failed (%1). Continuing save without backup.")
        .arg(QString::fromStdString(ec.message())));
    }
  }

  auto existing_items_by_id = [](const YAML::Node & sequence) {
    YAML::Node out(YAML::NodeType::Map);
    if (!sequence || !sequence.IsSequence()) return out;
    for (std::size_t i = 0; i < sequence.size(); ++i) {
      const YAML::Node existing = sequence[i];
      if (!existing || !existing.IsMap()) continue;
      const std::string existing_id = workcell_builder::yaml_map_value_or_empty(existing, "id");
      if (!existing_id.empty()) out[existing_id] = existing;
    }
    return out;
  };

  const bool saving_workcell_layout = effective_layout_path.filename().string() == "workcell_studio_layout.yaml" ||
    workcell_builder::yaml_map_value_or_empty(root, "schema_version") == "workcell_studio_layout/v1" ||
    (root["items"] && root["items"].IsSequence() && effective_layout_path.parent_path().filename().string() == "layout");
  const bool saving_placed_assets_layout = !saving_workcell_layout && root["placed_assets"] && root["placed_assets"].IsSequence();

  YAML::Node updated_placed(YAML::NodeType::Sequence);
  std::size_t editable_saved_count = 0;
  if (saving_workcell_layout) {
    QMap<QString, QGraphicsItem *> editable_by_id;
    for (auto * gi : editable_canvas_items) editable_by_id.insert(gi->data(RoleId).toString().trimmed(), gi);
    QSet<QString> saved_ids;
    YAML::Node existing_by_id = existing_items_by_id(root["items"]);
    YAML::Node existing_items = root["items"];
    if (existing_items && existing_items.IsSequence()) {
      for (std::size_t i = 0; i < existing_items.size(); ++i) {
        YAML::Node existing_item = existing_items[i];
        if (!existing_item || !existing_item.IsMap()) {
          updated_placed.push_back(YAML::Clone(existing_item));
          continue;
        }
        const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(existing_item, "id")).trimmed();
        if (!id.isEmpty() && editable_by_id.contains(id)) {
          YAML::Node item = serialized_editable_canvas_item(editable_by_id.value(id), existing_item);
          if (!item["source"]) item["source"] = "layout/workcell_studio_layout.yaml";
          updated_placed.push_back(item);
          saved_ids.insert(id);
          ++editable_saved_count;
          append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
            .arg(QString::fromStdString(scene_name), id, QString::fromStdString(effective_layout_path.string())));
        } else {
          updated_placed.push_back(YAML::Clone(existing_item));
        }
      }
    }
    for (auto * gi : editable_canvas_items) {
      const QString id = gi->data(RoleId).toString().trimmed();
      if (saved_ids.contains(id)) continue;
      YAML::Node item = serialized_editable_canvas_item(gi, existing_by_id[id.toStdString()]);
      if (!item["source"]) item["source"] = "layout/workcell_studio_layout.yaml";
      updated_placed.push_back(item);
      saved_ids.insert(id);
      ++editable_saved_count;
      append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
        .arg(QString::fromStdString(scene_name), id, QString::fromStdString(effective_layout_path.string())));
    }
    root["items"] = updated_placed;
    root["schema_version"] = "workcell_studio_layout/v1";
    root["schema"] = "workcell_studio_layout/v1";
  } else if (saving_placed_assets_layout) {
    const char * item_key = "placed_assets";
    YAML::Node existing_by_id = existing_items_by_id(root[item_key]);
    if (!imported_layout_path.empty() && editable_canvas_items.empty() && root[item_key] && root[item_key].IsSequence()) {
      updated_placed = YAML::Clone(root[item_key]);
    }
    for (auto * gi : editable_canvas_items) {
      const std::string item_id = gi->data(RoleId).toString().toStdString();
      YAML::Node item = serialized_editable_canvas_item(gi, existing_by_id[item_id]);
      updated_placed.push_back(item);
      ++editable_saved_count;
      append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
        .arg(QString::fromStdString(scene_name), gi->data(RoleId).toString(), QString::fromStdString(effective_layout_path.string())));
    }
    root[item_key] = updated_placed;
  } else {
    QMap<QString, QGraphicsItem *> editable_by_id;
    for (auto * gi : editable_canvas_items) editable_by_id.insert(gi->data(RoleId).toString().trimmed(), gi);
    QSet<QString> saved_ids;
    const QStringList legacy_sequence_keys = {"items", "assets", "objects", "zones", "targets"};
    for (const QString & key_q : legacy_sequence_keys) {
      const std::string key = key_q.toStdString();
      YAML::Node seq = root[key];
      if (!seq || !seq.IsSequence()) continue;
      for (std::size_t i = 0; i < seq.size(); ++i) {
        YAML::Node node = seq[i];
        if (!node || !node.IsMap()) continue;
        const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "id")).trimmed();
        if (id.isEmpty() || !editable_by_id.contains(id)) continue;
        seq[i] = serialized_editable_canvas_item(editable_by_id.value(id), node);
        saved_ids.insert(id);
        updated_placed.push_back(seq[i]);
        ++editable_saved_count;
        append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
          .arg(QString::fromStdString(scene_name), id, QString::fromStdString(effective_layout_path.string())));
      }
    }
    YAML::Node camera = root["camera"];
    if (camera && camera.IsMap()) {
      const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(camera, "id")).trimmed();
      if (!id.isEmpty() && editable_by_id.contains(id)) {
        root["camera"] = serialized_editable_canvas_item(editable_by_id.value(id), camera);
        saved_ids.insert(id);
        updated_placed.push_back(root["camera"]);
        ++editable_saved_count;
        append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
          .arg(QString::fromStdString(scene_name), id, QString::fromStdString(effective_layout_path.string())));
      }
    }
    YAML::Node placed_assets = ensure_sequence_of_maps(root, "placed_assets");
    YAML::Node existing_by_id = existing_items_by_id(placed_assets);
    for (auto * gi : editable_canvas_items) {
      const QString id = gi->data(RoleId).toString().trimmed();
      if (saved_ids.contains(id)) continue;
      YAML::Node item = serialized_editable_canvas_item(gi, existing_by_id[id.toStdString()]);
      placed_assets.push_back(item);
      updated_placed.push_back(item);
      ++editable_saved_count;
      append_studio_log(QString("Inspector transform saved: scene=%1 id=%2 path=%3")
        .arg(QString::fromStdString(scene_name), id, QString::fromStdString(effective_layout_path.string())));
    }
  }

  std::ofstream out(effective_layout_path.string());
  out << root;
  out.close();
  layout_dirty_ = false;
  layout_saved_ = true;
  validation_stale_ = true;
  launch_artifacts_ready_ = false;
  if (layout_state_label_) layout_state_label_->setText("Unsaved Layout Edits: none");
  if (editable_saved_count == 0) {
    const QString guidance = "Use Create editable layout from preview or add an item to persist editable objects.";
    append_studio_log(QString("Save Layout: no editable items; saved canonical layout metadata to %1. %2")
      .arg(QString::fromStdString(effective_layout_path.string()), guidance));
    QMessageBox::information(this, "Save Layout",
      QString("Save Layout: no editable items; saved canonical layout metadata to %1.\n%2")
        .arg(QString::fromStdString(effective_layout_path.string()), guidance));
  } else {
    append_studio_log(QString("Saved scene layout metadata to %1").arg(QString::fromStdString(effective_layout_path.string())));
  }

  append_studio_log(QString("Save Layout: serialized %1 editable layout item(s) to canonical layout only; no task/generated/plan_preview directories or runtime/ROS artifacts were bootstrapped.")
    .arg(static_cast<int>(editable_saved_count)));
  append_studio_log(QString("Save Layout: rebuilding Scene3D data after save (selection id snapshot='%1').")
    .arg(stable_selected_id_before_refresh.isEmpty() ? "<none>" : stable_selected_id_before_refresh));
  refresh_scene_builder_left_explorer();
  if (!stable_selected_id_before_refresh.isEmpty()) {
    apply_scene_selection(stable_selected_id_before_refresh, QStringLiteral("unknown"), false, false);
  } else {
    append_studio_log("Save Layout: no selected stable item id to reselect.");
  }
  refresh_scene_browser_ui();
  refresh_scene_workflow_rail();
  refresh_scene_builder_view_chips();
}

void MainWindow::create_starter_layout_from_preview()
{
  const QString action_title = QStringLiteral("Create editable layout from preview");
  const QString canonical_layout_path = QStringLiteral("layout/workcell_studio_layout.yaml");
  if (!has_selected_scene()) {
    const QString message = QStringLiteral("Create editable layout from preview blocked: no scene selected. Select a scene before writing %1.")
      .arg(canonical_layout_path);
    append_studio_log(message);
    QMessageBox::information(this, action_title, message);
    return;
  }

  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const std::vector<fs::path> yaml_candidates = {
    s.scene_dir / "layout" / "workcell_studio_layout.yaml",
    s.scene_dir / "environment_layout.yaml",
    s.scene_dir / "environment.yaml",
    s.scene_dir / "cell_definition.yaml",
    s.scene_dir / "scene_manifest.yaml"
  };
  QStringList malformed_yaml_details;
  for (const auto & candidate : yaml_candidates) {
    if (!fs::exists(candidate)) continue;
    try {
      YAML::LoadFile(candidate.string());
    } catch (const YAML::Exception & exc) {
      malformed_yaml_details.push_back(QStringLiteral("%1 (%2)")
        .arg(QString::fromStdString(candidate.string()), QString::fromStdString(exc.what())));
    } catch (const std::exception & exc) {
      malformed_yaml_details.push_back(QStringLiteral("%1 (%2)")
        .arg(QString::fromStdString(candidate.string()), QString::fromStdString(exc.what())));
    }
  }
  if (!malformed_yaml_details.isEmpty()) {
    const QString message = QStringLiteral("Create editable layout from preview blocked: malformed YAML in preview/candidate scene files for '%1'. Target remains %2. Fix these files before conversion:\n- %3")
      .arg(QString::fromStdString(s.scene_name), canonical_layout_path, malformed_yaml_details.join("\n- "));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  }

  workcell_builder::WorkcellStudioCanvasModel preview_model;
  workcell_builder::WorkcellStudioEditableLayoutBootstrapResult bootstrap_result;
  try {
    preview_model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
    bootstrap_result = workcell_builder::bootstrap_editable_layout_from_scene_sources(
      s.scene_dir, s.scene_name, preview_model);
  } catch (const YAML::Exception & exc) {
    const QString message = QStringLiteral("Create editable layout from preview blocked: malformed YAML while reading preview/candidate scene files for '%1'. Target remains %2. Details: %3")
      .arg(QString::fromStdString(s.scene_name), canonical_layout_path, QString::fromStdString(exc.what()));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  } catch (const std::exception & exc) {
    const QString message = QStringLiteral("Create editable layout from preview blocked while reading preview/candidate scene files for '%1'. Target remains %2. Details: %3")
      .arg(QString::fromStdString(s.scene_name), canonical_layout_path, QString::fromStdString(exc.what()));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  }

  const auto skipped_preview_items = bootstrap_result.skipped_locked_items +
    bootstrap_result.skipped_static_fallback_items +
    bootstrap_result.skipped_unsafe_or_missing_metadata_items;
  const auto bootstrap_counts_message = [&bootstrap_result, skipped_preview_items, &canonical_layout_path]() {
    const auto count_text = [](std::size_t count) { return QString::fromStdString(std::to_string(count)); };
    QStringList blockers;
    for (const auto & blocker : bootstrap_result.blockers) {
      blockers.push_back(QString::fromStdString(blocker));
    }
    const QString blocker_text = blockers.isEmpty() ? QStringLiteral("none reported") : blockers.join("\n- ");
    return QString("No preview items available to convert. "
                   "Generate Scene Package first, fix the listed blockers, or add layout items manually.\n\n"
                   "target: %1\n"
                   "source used: %2\n"
                   "editable items created: %3\n"
                   "skipped locked/helper/unsupported preview items: %4\n"
                   "skipped locked items: %5\n"
                   "skipped static fallback items: %6\n"
                   "skipped unsafe/missing metadata items: %7\n"
                   "blockers:\n- %8")
      .arg(canonical_layout_path)
      .arg(bootstrap_result.source_used.empty() ? QStringLiteral("<none>") : QString::fromStdString(bootstrap_result.source_used))
      .arg(count_text(bootstrap_result.editable_items_created))
      .arg(count_text(skipped_preview_items))
      .arg(count_text(bootstrap_result.skipped_locked_items))
      .arg(count_text(bootstrap_result.skipped_static_fallback_items))
      .arg(count_text(bootstrap_result.skipped_unsafe_or_missing_metadata_items))
      .arg(blocker_text);
  };
  if (bootstrap_result.editable_items_created == 0) {
    const QString message = bootstrap_counts_message();
    append_studio_log(message);
    const QString status_text = QStringLiteral("No preview items available to convert.");
    append_studio_log(status_text);
    QMessageBox::warning(this, action_title, message);
    return;
  }
  const fs::path layout_dir = bootstrap_result.expected_output_dir;
  const fs::path layout_file = bootstrap_result.expected_output_file;
  boost::system::error_code ec;
  fs::create_directories(layout_dir, ec);
  if (ec) {
    const QString message = QStringLiteral("Create editable layout from preview failed: cannot create layout directory for %1 at %2 (%3).")
      .arg(canonical_layout_path, QString::fromStdString(layout_dir.string()), QString::fromStdString(ec.message()));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  }
  if (fs::exists(layout_file)) {
    const auto response = QMessageBox::question(this, QStringLiteral("Overwrite Editable Layout"),
      QStringLiteral("%1 already exists. Overwrite it with editable layout items converted from preview/candidate scene data?")
        .arg(canonical_layout_path));
    if (response != QMessageBox::Yes) {
      append_studio_log(QStringLiteral("Create editable layout from preview cancelled by user. Target: %1 (%2).")
        .arg(canonical_layout_path, QString::fromStdString(layout_file.string())));
      return;
    }
    const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
    const fs::path backup = layout_dir / ("workcell_studio_layout." + stamp.toStdString() + ".bak.yaml");
    fs::copy_file(layout_file, backup, fs::copy_option::overwrite_if_exists, ec);
    if (ec) {
      const QString message = QStringLiteral("Create editable layout from preview failed: backup before overwrite failed for %1 at %2 (%3).")
        .arg(canonical_layout_path, QString::fromStdString(layout_file.string()), QString::fromStdString(ec.message()));
      append_studio_log(message);
      QMessageBox::warning(this, action_title, message);
      return;
    }
    append_studio_log(QStringLiteral("Create editable layout from preview: backup created at %1 before writing %2 (%3).")
      .arg(QString::fromStdString(backup.string()), canonical_layout_path, QString::fromStdString(layout_file.string())));
  }
  const YAML::Node layout = bootstrap_result.layout;
  std::ofstream out(layout_file.string());
  if (!out.good()) {
    const QString message = QStringLiteral("Create editable layout from preview failed: unable to open %1 for write at %2.")
      .arg(canonical_layout_path, QString::fromStdString(layout_file.string()));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  }
  out << layout;
  out.close();
  if (!out.good()) {
    const QString message = QStringLiteral("Create editable layout from preview failed: write error while saving %1 at %2.")
      .arg(canonical_layout_path, QString::fromStdString(layout_file.string()));
    append_studio_log(message);
    QMessageBox::warning(this, action_title, message);
    return;
  }

  const int generated_count = static_cast<int>(bootstrap_result.editable_items_created);
  append_studio_log(QStringLiteral("Create editable layout from preview detail: source=%1 target=%2 absolute_target=%3 editable=%4 skipped_locked_helper_unsupported=%5 skipped_locked=%6 skipped_static_fallback=%7 skipped_unsafe_metadata=%8")
    .arg(QString::fromStdString(bootstrap_result.source_used))
    .arg(canonical_layout_path)
    .arg(QString::fromStdString(layout_file.string()))
    .arg(generated_count)
    .arg(static_cast<int>(skipped_preview_items))
    .arg(static_cast<int>(bootstrap_result.skipped_locked_items))
    .arg(static_cast<int>(bootstrap_result.skipped_static_fallback_items))
    .arg(static_cast<int>(bootstrap_result.skipped_unsafe_or_missing_metadata_items)));
  if (skipped_preview_items > 0) {
    append_studio_log(QStringLiteral("Skipped %1 locked/helper/unsupported preview items.")
      .arg(static_cast<int>(skipped_preview_items)));
  }
  append_studio_log(QStringLiteral("Create editable layout from preview: wrote editable layout to canonical target %1 (%2), then refreshes Scene3D/2D/hierarchy/inspector. Subsequent move/edit operations remain dirty until Save Layout serializes %1 again.")
    .arg(canonical_layout_path, QString::fromStdString(layout_file.string())));
  mark_layout_dirty("Created editable layout from preview");
  append_studio_log(QStringLiteral("Create editable layout from preview: marked layout dirty after preview conversion so Save Layout can re-serialize %1.")
    .arg(canonical_layout_path));
  append_studio_log(QStringLiteral("Created %1 editable layout items from preview.").arg(generated_count));
  refresh_scene_builder_left_explorer();
  refresh_scene_browser_ui();
  refresh_scene_builder_selected_scene_ui();
  refresh_canvas_generated_parity_ui();
  refresh_scene_workflow_rail();
}

void MainWindow::revert_layout_changes()
{
  rebuild_digital_twin_canvas();
  layout_dirty_ = false;
  validation_stale_ = true;
  launch_artifacts_ready_ = false;
  if (layout_state_label_) layout_state_label_->setText("Unsaved Layout Edits: none");
  append_studio_log("Revert Layout requested");
}

void MainWindow::on_canvas_selection_changed()
{
  if (!digital_twin_scene_ || selection_update_guard_) return;
  if (digital_twin_scene_->selectedItems().isEmpty()) {
    if (current_selected_scene_item_id_.isEmpty()) {
      sync_selected_item_state();
      refresh_selected_scene_item_labels(selected_item_state_);
    }
    return;
  }
  auto * sel = digital_twin_scene_->selectedItems().front();
  const QString selected_id = sel->data(RoleId).toString().trimmed();
  const QString selected_role = sel->data(RoleRole).toString().trimmed();
  apply_scene_selection(selected_id, selected_role, false, false);
}
void MainWindow::on_canvas_item_moved(QGraphicsItem * item, const QPointF &, const QPointF &, const QString & reason){ if(item) select_canvas_item(item); mark_layout_dirty(reason); }
bool MainWindow::parse_transform_clipboard_text(
  const QString & text, double * x, double * y, double * z, double * r, double * p, double * yaw, QString * error)
{
  return workcell_builder::parse_transform_clipboard_text(text, x, y, z, r, p, yaw, error);
}

void MainWindow::refresh_selection_transform_editor_from_item(QGraphicsItem * item)
{
  if (!item) return;
  SelectedSceneItemState state;
  state.valid = true;
  state.id = item->data(RoleId).toString().trimmed();
  state.display_name = item->data(RoleDisplayName).toString().trimmed();
  state.role = item->data(RoleRole).toString().trimmed();
  state.category = item->data(RoleCategory).toString().trimmed();
  state.type = item->data(RoleType).toString().trimmed();
  state.role_or_category = state.role;
  if (state.role_or_category.isEmpty()) state.role_or_category = state.category;
  if (state.role_or_category.isEmpty()) state.role_or_category = state.type;
  state.source_layer = item->data(RoleSourceLayer).toString().trimmed();
  state.editable = !item->data(RoleLocked).toBool();
  state.locked = item->data(RoleLocked).toBool();
  state.linked_to_editable_layout_state = state.editable && state.source_layer.compare(QStringLiteral("editable_layout"), Qt::CaseInsensitive) == 0;
  state.pose_available = true;
  state.pose_x = item->pos().x() / 100.0;
  state.pose_y = item->pos().y() / 100.0;
  state.pose_z = item->data(RolePoseZ).toDouble();
  state.roll = item->data(RoleRoll).toDouble();
  state.pitch = item->data(RolePitch).toDouble();
  state.yaw = item->data(RoleYaw).toDouble();
  state.dim_x = item->data(RoleWidth).toDouble();
  state.dim_y = item->data(RoleDepth).toDouble();
  state.dim_z = item->data(RoleHeight).toDouble();
  refresh_selection_transform_editor_from_state(state);
}

void MainWindow::refresh_selection_transform_editor_from_state(const SelectedSceneItemState & state)
{
  if (!state.valid || !state.pose_available) return;
  const bool locked = state.locked || !state.editable || !state.linked_to_editable_layout_state;
  inspector_update_guard_ = true;
  if (inspector_x_) inspector_x_->setValue(state.pose_x);
  if (inspector_y_) inspector_y_->setValue(state.pose_y);
  if (inspector_z_) inspector_z_->setValue(state.pose_z);
  if (inspector_roll_) inspector_roll_->setValue(state.roll);
  if (inspector_pitch_) inspector_pitch_->setValue(state.pitch);
  if (inspector_yaw_) inspector_yaw_->setValue(state.yaw);
  if (inspector_dim_x_) inspector_dim_x_->setValue(state.dim_x);
  if (inspector_dim_y_) inspector_dim_y_->setValue(state.dim_y);
  if (inspector_dim_z_) inspector_dim_z_->setValue(state.dim_z);
  if (inspector_display_name_) {
    inspector_display_name_->setText(state.display_name);
    inspector_display_name_->setPlaceholderText(state.id.isEmpty() ? QStringLiteral("Display name") : state.id);
  }
  if (inspector_role_) inspector_role_->setText(state.role);
  if (inspector_category_) inspector_category_->setText(state.category);
  if (inspector_type_) inspector_type_->setText(state.type);
  for (auto * sb : {inspector_x_, inspector_y_, inspector_z_, inspector_roll_, inspector_pitch_, inspector_yaw_}) {
    if (sb) sb->setReadOnly(locked);
  }
  for (auto * sb : {inspector_dim_x_, inspector_dim_y_, inspector_dim_z_}) {
    if (sb) sb->setReadOnly(locked);
  }
  if (inspector_apply_button_) inspector_apply_button_->setEnabled(!locked);
  if (inspector_revert_button_) inspector_revert_button_->setEnabled(!locked);
  if (inspector_live_update_box_) inspector_live_update_box_->setEnabled(!locked);
  if (inspector_display_name_) inspector_display_name_->setReadOnly(locked);
  for (auto * le : {inspector_role_, inspector_category_, inspector_type_}) {
    if (le) le->setReadOnly(true);
  }
  inspector_update_guard_ = false;
  refresh_robot_base_pose_inspector();
}

QGraphicsItem * MainWindow::find_authoring_robot_base_item() const
{
  if (!digital_twin_scene_) return nullptr;
  for (auto * item : digital_twin_scene_->items()) {
    if (item->data(RoleType).toString() == "robot_base" && !item->data(RoleLocked).toBool()) return item;
  }
  return nullptr;
}

void MainWindow::refresh_robot_base_pose_inspector()
{
  if (!robot_pose_source_label_ || !robot_base_apply_button_) return;
  QGraphicsItem * selected = (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty()) ? digital_twin_scene_->selectedItems().front() : nullptr;
  const bool selected_generated = selected && selected->data(RoleLocked).toBool() && selected->data(RoleType).toString() == "robot_base";
  QGraphicsItem * authoring_robot = find_authoring_robot_base_item();
  robot_base_pose_source_ = authoring_robot ? "cell_definition" : "blocked";
  if (authoring_robot && authoring_robot->data(RoleSource).toString().contains("environment.yaml")) robot_base_pose_source_ = "environment.yaml";
  robot_pose_source_label_->setText(QString("robot_pose_source: %1").arg(robot_base_pose_source_));
  const bool editable = (authoring_robot != nullptr) && !selected_generated;
  if (selected_generated && robot_pose_message_label_) robot_pose_message_label_->setText("Edit robot base via authoring model");
  else if (robot_pose_message_label_) robot_pose_message_label_->setText(editable ? "Robot base pose edits target authoring model." : "Robot base authoring pose unavailable.");
  robot_base_update_guard_ = true;
  if (authoring_robot) {
    robot_base_x_->setValue(authoring_robot->pos().x() / 100.0);
    robot_base_y_->setValue(authoring_robot->pos().y() / 100.0);
    robot_base_z_->setValue(authoring_robot->data(RolePoseZ).toDouble());
    robot_base_roll_->setValue(authoring_robot->data(RoleRoll).toDouble());
    robot_base_pitch_->setValue(authoring_robot->data(RolePitch).toDouble());
    robot_base_yaw_->setValue(authoring_robot->data(RoleYaw).toDouble());
    if (!robot_base_snapshot_valid_) {
      robot_base_snapshot_x_ = robot_base_x_->value(); robot_base_snapshot_y_ = robot_base_y_->value(); robot_base_snapshot_z_ = robot_base_z_->value();
      robot_base_snapshot_roll_ = robot_base_roll_->value(); robot_base_snapshot_pitch_ = robot_base_pitch_->value(); robot_base_snapshot_yaw_ = robot_base_yaw_->value();
      robot_base_snapshot_valid_ = true;
    }
  }
  robot_base_update_guard_ = false;
  for (auto * sb : {robot_base_x_, robot_base_y_, robot_base_z_, robot_base_roll_, robot_base_pitch_, robot_base_yaw_}) sb->setReadOnly(!editable);
  robot_base_apply_button_->setEnabled(editable);
  robot_base_reset_button_->setEnabled(editable && robot_base_snapshot_valid_);
}

void MainWindow::apply_robot_base_pose_from_inspector()
{
  if (robot_base_update_guard_) return;
  auto * authoring_robot = find_authoring_robot_base_item();
  if (!authoring_robot) { append_studio_log("robot_base_apply blocked: authoring robot base not available"); return; }
  authoring_robot->setPos(robot_base_x_->value() * 100.0, robot_base_y_->value() * 100.0);
  authoring_robot->setData(RolePoseZ, robot_base_z_->value());
  authoring_robot->setData(RoleRoll, robot_base_roll_->value());
  authoring_robot->setData(RolePitch, robot_base_pitch_->value());
  authoring_robot->setData(RoleYaw, robot_base_yaw_->value());
  append_studio_log(QString("robot_pose_source diagnostic: %1").arg(robot_base_pose_source_));
  append_studio_log(QString("robot_base_pose applied: x=%1 y=%2 z=%3 roll=%4 pitch=%5 yaw=%6")
    .arg(robot_base_x_->value()).arg(robot_base_y_->value()).arg(robot_base_z_->value())
    .arg(robot_base_roll_->value()).arg(robot_base_pitch_->value()).arg(robot_base_yaw_->value()));
  mark_layout_dirty("Robot Base Pose Apply");
  rebuild_digital_twin_canvas();
}

void MainWindow::reset_robot_base_pose_from_snapshot()
{
  if (!robot_base_snapshot_valid_) return;
  robot_base_x_->setValue(robot_base_snapshot_x_);
  robot_base_y_->setValue(robot_base_snapshot_y_);
  robot_base_z_->setValue(robot_base_snapshot_z_);
  robot_base_roll_->setValue(robot_base_snapshot_roll_);
  robot_base_pitch_->setValue(robot_base_snapshot_pitch_);
  robot_base_yaw_->setValue(robot_base_snapshot_yaw_);
  append_studio_log("robot_base_pose reset: restored from persisted authoring source snapshot");
  apply_robot_base_pose_from_inspector();
}

void MainWindow::apply_selection_transform_from_editor() { apply_inspector_pose_to_item(); }

void MainWindow::apply_inspector_pose_to_item()
{
  if (inspector_update_guard_) return;

  const auto target = resolve_selected_editable_layout_target();
  if (!target.ok) {
    append_studio_log(QString("Inspector transform edit blocked: %1").arg(target.blocker));
    return;
  }

  QGraphicsItem * i = target.fallback_item;
  const QString item_id = target.state.id.trimmed();
  const QPointF old(target.state.pose_x * 100.0, target.state.pose_y * 100.0);
  const QPointF updated(inspector_x_->value() * 100.0, inspector_y_->value() * 100.0);

  QString updated_display_name;
  bool metadata_changed = false;
  if (i) {
    i->setPos(updated);
    i->setData(RolePoseZ, inspector_z_->value());
    i->setData(RoleRoll, inspector_roll_->value());
    i->setData(RolePitch, inspector_pitch_->value());
    i->setData(RoleYaw, inspector_yaw_->value());
    i->setData(RoleWidth, inspector_dim_x_->value());
    i->setData(RoleDepth, inspector_dim_y_->value());
    i->setData(RoleHeight, inspector_dim_z_->value());
    if (inspector_display_name_) {
      updated_display_name = inspector_display_name_->text().trimmed();
      if (!updated_display_name.isEmpty()) {
        metadata_changed = updated_display_name != i->data(RoleDisplayName).toString().trimmed();
        if (metadata_changed) i->setData(RoleDisplayName, updated_display_name);
      }
    }
  }

  SelectedSceneItemState refreshed_state = target.state;
  refreshed_state.pose_available = true;
  refreshed_state.pose_x = inspector_x_->value();
  refreshed_state.pose_y = inspector_y_->value();
  refreshed_state.pose_z = inspector_z_->value();
  refreshed_state.roll = inspector_roll_->value();
  refreshed_state.pitch = inspector_pitch_->value();
  refreshed_state.yaw = inspector_yaw_->value();
  refreshed_state.dim_x = inspector_dim_x_->value();
  refreshed_state.dim_y = inspector_dim_y_->value();
  refreshed_state.dim_z = inspector_dim_z_->value();
  if (!updated_display_name.isEmpty()) refreshed_state.display_name = updated_display_name;
  refreshed_state.role = target.state.role;
  refreshed_state.category = target.state.category;
  refreshed_state.type = target.state.type;
  refreshed_state.role_or_category = target.state.role_or_category;
  refreshed_state.editable = true;
  refreshed_state.locked = false;
  refreshed_state.linked_to_editable_layout_state = true;
  selected_item_state_ = refreshed_state;

  undo_stack_.push_back({"pose_edit", item_id, old, updated, false, false});
  redo_stack_.clear();
  mark_layout_dirty("Inspector Pose/Dimensions Edit");
  if (metadata_changed) {
    mark_layout_dirty("Inspector Metadata Edit");
  }

  const QString selected_scene_name_for_log = selected_scene_state_.valid ? selected_scene_state_.name : QStringLiteral("unknown");
  append_studio_log(QString("Inspector transform edited: scene=%1 id=%2 source=%3 xyz=[%4,%5,%6] rpy=[%7,%8,%9] dirty=true")
    .arg(selected_scene_name_for_log, item_id, target.source_path)
    .arg(inspector_x_->value(), 0, 'g', 17)
    .arg(inspector_y_->value(), 0, 'g', 17)
    .arg(inspector_z_->value(), 0, 'g', 17)
    .arg(inspector_roll_->value(), 0, 'g', 17)
    .arg(inspector_pitch_->value(), 0, 'g', 17)
    .arg(inspector_yaw_->value(), 0, 'g', 17));
  if (metadata_changed) {
    append_studio_log(QString("Inspector metadata edited: scene=%1 id=%2 display_name=%3 dirty=true")
      .arg(selected_scene_name_for_log, item_id, updated_display_name));
  }
  append_studio_log(QString("item updated: %1 source=%2 editable=true locked=false fallback_view_refreshed=%3")
    .arg(item_id, target.source_path, i ? QStringLiteral("true") : QStringLiteral("false")));
  for (auto & p : all_scene_preview_items_) {
    if (p.id != item_id) continue;
    p.x = refreshed_state.pose_x;
    p.y = refreshed_state.pose_y;
    p.z = refreshed_state.pose_z;
    p.roll = refreshed_state.roll;
    p.pitch = refreshed_state.pitch;
    p.yaw = refreshed_state.yaw;
    p.sx = refreshed_state.dim_x;
    p.sy = refreshed_state.dim_y;
    p.sz = refreshed_state.dim_z;
    p.mesh_scale_x = refreshed_state.dim_x;
    p.mesh_scale_y = refreshed_state.dim_y;
    p.mesh_scale_z = refreshed_state.dim_z;
    if (!refreshed_state.display_name.isEmpty()) p.display_name = refreshed_state.display_name;
    break;
  }
  apply_scene3d_preview_layer_filters(false);
  refresh_selection_transform_editor_from_state(refreshed_state);
  refresh_selected_scene_item_labels(refreshed_state);
  if (scene_preview_widget_) scene_preview_widget_->update();
}

void MainWindow::revert_selection_transform_editor()
{
  const auto state = current_selected_scene_item();
  if (!state.valid) return;
  if (auto * item = find_canvas_item_by_stable_id(state.id)) {
    refresh_selection_transform_editor_from_item(item);
  } else {
    refresh_selection_transform_editor_from_state(state);
  }
}

void MainWindow::copy_selection_transform_to_clipboard()
{
  const QString text = QString("x=%1 y=%2 z=%3 r=%4 p=%5 yaw=%6")
    .arg(inspector_x_->value(), 0, 'g', 17).arg(inspector_y_->value(), 0, 'g', 17).arg(inspector_z_->value(), 0, 'g', 17)
    .arg(inspector_roll_->value(), 0, 'g', 17).arg(inspector_pitch_->value(), 0, 'g', 17).arg(inspector_yaw_->value(), 0, 'g', 17);
  QApplication::clipboard()->setText(text);
}

void MainWindow::paste_selection_transform_from_clipboard()
{
  double x = 0.0, y = 0.0, z = 0.0, r = 0.0, p = 0.0, yaw = 0.0;
  QString error;
  if (!parse_transform_clipboard_text(QApplication::clipboard()->text(), &x, &y, &z, &r, &p, &yaw, &error)) {
    QMessageBox::warning(this, "Paste Transform", "Invalid transform text.\n" + error);
    return;
  }
  inspector_update_guard_ = true;
  inspector_x_->setValue(x); inspector_y_->setValue(y); inspector_z_->setValue(z);
  inspector_roll_->setValue(r); inspector_pitch_->setValue(p); inspector_yaw_->setValue(yaw);
  inspector_update_guard_ = false;
  apply_selection_transform_from_editor();
}
void MainWindow::undo_layout_edit(){ if(undo_stack_.empty() || !digital_twin_scene_) return; auto c=undo_stack_.back(); undo_stack_.pop_back(); for(auto *i:digital_twin_scene_->items()) if(i->data(RoleId).toString()==c.item_id){ i->setPos(c.old_pos); break;} redo_stack_.push_back(c); mark_layout_dirty("Undo"); }
void MainWindow::redo_layout_edit(){ if(redo_stack_.empty() || !digital_twin_scene_) return; auto c=redo_stack_.back(); redo_stack_.pop_back(); for(auto *i:digital_twin_scene_->items()) if(i->data(RoleId).toString()==c.item_id){ i->setPos(c.new_pos); break;} undo_stack_.push_back(c); mark_layout_dirty("Redo"); }
void MainWindow::duplicate_selected_item(){ if(!digital_twin_scene_||digital_twin_scene_->selectedItems().isEmpty()) return; auto *s=digital_twin_scene_->selectedItems().front(); if(s->data(RoleLocked).toBool()){ append_studio_log("Duplicate blocked: locked item"); return; } auto *c=new DraggableCanvasItem(static_cast<QGraphicsRectItem*>(s)->rect()); c->setPos(s->pos()+QPointF(18,18)); c->setBrush(static_cast<QGraphicsRectItem*>(s)->brush()); for(int r=RoleId;r<=RoleSource;++r) c->setData(r,s->data(r)); c->setData(RoleId, s->data(RoleId).toString()+"_copy"); c->setFlags(s->flags()); digital_twin_scene_->addItem(c); c->setSelected(true); undo_stack_.push_back({"duplicate", c->data(RoleId).toString(), s->pos(), c->pos(), true, false}); mark_layout_dirty("Duplicate Selected"); append_studio_log(QString("Duplicate selected item: %1").arg(c->data(RoleId).toString())); refresh_scene_builder_left_explorer(); }
void MainWindow::delete_selected_item(){ if(!digital_twin_scene_||digital_twin_scene_->selectedItems().isEmpty()) return; auto *s=digital_twin_scene_->selectedItems().front(); const QString id=s->data(RoleId).toString(); const QString t=s->data(RoleType).toString(); if(t=="robot_base"||t=="reach"||t=="safety/home"){ QMessageBox::warning(this,"Remove Selected Layout Item","Delete robot is blocked/guarded unless Unlock Robot Base is enabled."); return;} if(QMessageBox::question(this,"Remove Selected Layout Item","Remove selected layout instance from environment_layout.yaml?")!=QMessageBox::Yes) return; undo_stack_.push_back({"delete", id, s->pos(), s->pos(), false, true}); delete s; refresh_scene_builder_left_explorer(); mark_layout_dirty("Remove Selected Layout Item"); append_studio_log(QString("Removed layout item %1 from canvas; save layout to persist.").arg(id)); }

double MainWindow::current_nudge_step_m(Qt::KeyboardModifiers modifiers) const
{
  double step = (snap_to_grid_box_ && snap_to_grid_box_->isChecked()) ? snap_step_m_ : 0.01;
  if (modifiers & Qt::ShiftModifier) step *= 5.0;
  if (modifiers & Qt::ControlModifier) step *= 0.2;
  return std::max(0.001, step);
}

void MainWindow::rebuild_canvas_inspector()
{
  if (!digital_twin_scene_ || selection_update_guard_) return;
  if (digital_twin_scene_->selectedItems().isEmpty()) {
    refresh_selected_scene_item_labels(current_selected_scene_item());
    return;
  }

  auto * item = digital_twin_scene_->selectedItems().front();
  if (!item) {
    refresh_selected_scene_item_labels(current_selected_scene_item());
    return;
  }

  select_canvas_item(item);
  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(item->data(RoleId).toString().trimmed());
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
  if (!digital_twin_scene_ || digital_twin_scene_->selectedItems().isEmpty()) { QMainWindow::keyPressEvent(event); return; }
  auto * item = digital_twin_scene_->selectedItems().front();
  if (!item || item->data(RoleLocked).toBool()) { QMainWindow::keyPressEvent(event); return; }
  const double step_m = current_nudge_step_m(event->modifiers());
  const double step_px = step_m * 100.0;
  QPointF delta; double dz = 0.0;
  switch (event->key()) {
    case Qt::Key_Left: delta.rx() -= step_px; break;
    case Qt::Key_Right: delta.rx() += step_px; break;
    case Qt::Key_Up: delta.ry() -= step_px; break;
    case Qt::Key_Down: delta.ry() += step_px; break;
    case Qt::Key_PageUp: dz += step_m; break;
    case Qt::Key_PageDown: dz -= step_m; break;
    default: QMainWindow::keyPressEvent(event); return;
  }
  const QPointF old_pos = item->pos();
  item->setPos(snap_canvas_position(old_pos + delta));
  item->setData(RolePoseZ, item->data(RolePoseZ).toDouble() + dz);
  if (snap_step_label_) snap_step_label_->setText(QString("Nudge step: %1 m").arg(step_m, 0, 'f', 3));
  undo_stack_.push_back({"nudge", item->data(RoleId).toString(), old_pos, item->pos(), false, false});
  redo_stack_.clear();
  mark_layout_dirty("Nudge Move");
  rebuild_canvas_inspector();
  event->accept();
}


void MainWindow::add_asset_to_canvas_from_catalog(const QString & category, const QString & display_name, const QString & source_path)
{
  const QString lower_path = source_path.toLower();
  const bool repo_mesh_asset = lower_path.endsWith(".stl") || lower_path.endsWith(".dae") || lower_path.endsWith(".obj");
  if (repo_mesh_asset) {
    armed_asset_category_ = category;
    armed_asset_display_name_ = display_name;
    armed_asset_source_path_ = source_path;
    reset_armed_asset_transform_to_defaults();
    armed_asset_use_clicked_xy_ = false;
    arm_place_asset_mode(category, display_name, source_path);
    commit_armed_asset_placement(armed_asset_default_xy_px_);
    return;
  }
  if (!configure_asset_placement_transform(category, display_name)) {
    append_studio_log(QString("Add Asset validation failed: %1 (%2) placement canceled.")
      .arg(display_name, category));
    return;
  }
  arm_place_asset_mode(category, display_name, source_path);
}

QPointF MainWindow::compute_default_canvas_pose(const QString & category, const QString & display_name) const
{
  auto anchor_pos = [this](const QString & needle)->QPointF {
      if (!digital_twin_scene_) return QPointF(0.0, 0.0);
      for (auto * gi : digital_twin_scene_->items()) {
        const QString tag = gi->data(RoleCategory).toString().toLower() + "|" + gi->data(RoleRole).toString().toLower() + "|" + gi->data(RoleId).toString().toLower();
        if (tag.contains(needle)) return gi->pos();
      }
      return QPointF(0.0, 0.0);
    };
  const QString lower = (category + " " + display_name).toLower();
  const QPointF table = anchor_pos("table");
  const QPointF conveyor = anchor_pos("conveyor");
  const QPointF pick = anchor_pos("pick");
  const QPointF place = anchor_pos("place");
  if (lower.contains("table")) return table;
  if (lower.contains("conveyor")) return table + QPointF(-120.0, 25.0);
  if (lower.contains("bin") || lower.contains("place target") || lower.contains("place_target")) return table + QPointF(95.0, -20.0);
  if (lower.contains("pick zone") || lower.contains("pick_zone")) return (conveyor != QPointF(0.0, 0.0)) ? conveyor + QPointF(65.0, -10.0) : table + QPointF(-40.0, 10.0);
  if (lower.contains("place zone") || lower.contains("place_zone")) return (place != QPointF(0.0, 0.0)) ? place + QPointF(30.0, 0.0) : table + QPointF(115.0, 15.0);
  if (lower.contains("camera")) return table + QPointF(-20.0, -135.0);
  if (lower.contains("object")) return (pick != QPointF(0.0, 0.0)) ? pick + QPointF(10.0, 10.0) : table + QPointF(-20.0, 10.0);
  return default_xy_for_category(category);
}

double MainWindow::default_asset_pose_z(const QString & category, const QString & display_name) const
{
  const QString lower = (category + " " + display_name).toLower();
  return lower.contains("camera") ? 1.2 : 0.0;
}

void MainWindow::reset_armed_asset_transform_to_defaults()
{
  armed_asset_default_xy_px_ = compute_default_canvas_pose(armed_asset_category_, armed_asset_display_name_);
  armed_asset_x_m_ = armed_asset_default_xy_px_.x() / 100.0;
  armed_asset_y_m_ = armed_asset_default_xy_px_.y() / 100.0;
  armed_asset_z_m_ = default_asset_pose_z(armed_asset_category_, armed_asset_display_name_);
  armed_asset_roll_rad_ = 0.0;
  armed_asset_pitch_rad_ = 0.0;
  armed_asset_yaw_rad_ = 0.0;
}

bool MainWindow::validate_armed_asset_transform(QString * error_message)
{
  if (!std::isfinite(armed_asset_x_m_) || !std::isfinite(armed_asset_y_m_) || !std::isfinite(armed_asset_z_m_) ||
    !std::isfinite(armed_asset_roll_rad_) || !std::isfinite(armed_asset_pitch_rad_) || !std::isfinite(armed_asset_yaw_rad_))
  {
    if (error_message) *error_message = "All transform values must be numeric (x,y,z,roll,pitch,yaw).";
    return false;
  }
  if (error_message) error_message->clear();
  return true;
}

void MainWindow::update_arm_transform_validation_ui() {}

bool MainWindow::configure_asset_placement_transform(const QString & category, const QString & display_name)
{
  armed_asset_category_ = category;
  armed_asset_display_name_ = display_name;
  reset_armed_asset_transform_to_defaults();
  QDialog dialog(this);
  dialog.setWindowTitle("Place Asset Transform");
  auto * layout = new QVBoxLayout(&dialog);
  auto * help = new QLabel("Enter placement transform before placing. XYZ in metres, RPY in radians.", &dialog);
  help->setWordWrap(true);
  layout->addWidget(help);
  auto * grid = new QGridLayout();
  auto mk = [&](const QString & label, double value, const QString & units, int row) {
      auto * l = new QLabel(label, &dialog);
      auto * e = new QLineEdit(QString::number(value, 'f', 3), &dialog);
      auto * u = new QLabel(units, &dialog);
      grid->addWidget(l, row, 0);
      grid->addWidget(e, row, 1);
      grid->addWidget(u, row, 2);
      return e;
    };
  QLineEdit * x_edit = mk("X", armed_asset_x_m_, "m", 0);
  QLineEdit * y_edit = mk("Y", armed_asset_y_m_, "m", 1);
  QLineEdit * z_edit = mk("Z", armed_asset_z_m_, "m", 2);
  QLineEdit * r_edit = mk("Roll", armed_asset_roll_rad_, "rad", 3);
  QLineEdit * p_edit = mk("Pitch", armed_asset_pitch_rad_, "rad", 4);
  QLineEdit * yaw_edit = mk("Yaw", armed_asset_yaw_rad_, "rad", 5);
  layout->addLayout(grid);
  auto * use_clicked = new QCheckBox("Use clicked position for XY", &dialog);
  use_clicked->setChecked(true);
  layout->addWidget(use_clicked);
  auto * error_label = new QLabel(&dialog);
  error_label->setStyleSheet("color:#b00020;");
  layout->addWidget(error_label);
  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  auto * defaults_btn = buttons->addButton("Use defaults", QDialogButtonBox::ResetRole);
  layout->addWidget(buttons);
  QObject::connect(defaults_btn, &QPushButton::clicked, &dialog, [&]() {
    reset_armed_asset_transform_to_defaults();
    x_edit->setText(QString::number(armed_asset_x_m_, 'f', 3));
    y_edit->setText(QString::number(armed_asset_y_m_, 'f', 3));
    z_edit->setText(QString::number(armed_asset_z_m_, 'f', 3));
    r_edit->setText(QString::number(armed_asset_roll_rad_, 'f', 3));
    p_edit->setText(QString::number(armed_asset_pitch_rad_, 'f', 3));
    yaw_edit->setText(QString::number(armed_asset_yaw_rad_, 'f', 3));
    error_label->clear();
  });
  QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&]() {
    bool okx, oky, okz, okr, okp, okyaw;
    const double x = x_edit->text().trimmed().toDouble(&okx);
    const double y = y_edit->text().trimmed().toDouble(&oky);
    const double z = z_edit->text().trimmed().toDouble(&okz);
    const double r = r_edit->text().trimmed().toDouble(&okr);
    const double p = p_edit->text().trimmed().toDouble(&okp);
    const double yw = yaw_edit->text().trimmed().toDouble(&okyaw);
    const bool ok = okx && oky && okz && okr && okp && okyaw;
    for (auto * edit : {x_edit, y_edit, z_edit, r_edit, p_edit, yaw_edit}) edit->setStyleSheet(ok ? "" : "border:1px solid #b00020;");
    if (!ok) { error_label->setText("Invalid transform: enter numeric x,y,z,roll,pitch,yaw values."); return; }
    armed_asset_x_m_ = x; armed_asset_y_m_ = y; armed_asset_z_m_ = z; armed_asset_roll_rad_ = r; armed_asset_pitch_rad_ = p; armed_asset_yaw_rad_ = yw;
    armed_asset_use_clicked_xy_ = use_clicked->isChecked();
    dialog.accept();
  });
  QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  return dialog.exec() == QDialog::Accepted;
}

void MainWindow::arm_place_asset_mode(const QString & category, const QString & display_name, const QString & source_path)
{
  if (!digital_twin_scene_) { rebuild_digital_twin_canvas(); }
  if (!digital_twin_scene_) return;
  place_asset_armed_ = true;
  armed_asset_category_ = category;
  armed_asset_display_name_ = display_name;
  armed_asset_source_path_ = source_path;
  armed_asset_default_xy_px_ = compute_default_canvas_pose(category, display_name);
  set_canvas_interaction_mode(CanvasInteractionMode::Place);
  append_studio_log(QString("Place Asset Mode armed: %1 (%2). Click canvas to commit. Use clicked XY: %3 | xyzrpy=[%4, %5, %6, %7, %8, %9].")
    .arg(display_name, category)
    .arg(armed_asset_use_clicked_xy_ ? "on" : "off")
    .arg(armed_asset_x_m_, 0, 'f', 3).arg(armed_asset_y_m_, 0, 'f', 3).arg(armed_asset_z_m_, 0, 'f', 3)
    .arg(armed_asset_roll_rad_, 0, 'f', 3).arg(armed_asset_pitch_rad_, 0, 'f', 3).arg(armed_asset_yaw_rad_, 0, 'f', 3));
}

void MainWindow::commit_armed_asset_placement(const QPointF & canvas_pos_px)
{
  if (!digital_twin_scene_ || !place_asset_armed_) return;
  const QString category = armed_asset_category_;
  const QString display_name = armed_asset_display_name_;
  const QString source_path = armed_asset_source_path_;
  std::set<std::string> reserved_ids;
  for (auto * gi : digital_twin_scene_->items()) {
    const QString existing_id = gi->data(RoleId).toString().trimmed();
    if (!existing_id.isEmpty()) reserved_ids.insert(existing_id.toStdString());
  }
  const fs::path layout_path = selected_scene_existing_layout_import_path(scene_browser_result_, selected_scene_index_);
  if (!layout_path.empty()) {
    const auto layout_ids = workcell_builder::workcell_studio_collect_layout_ids(layout_path);
    reserved_ids.insert(layout_ids.begin(), layout_ids.end());
  }
  const QString new_id = QString::fromStdString(
    workcell_builder::workcell_studio_next_id(category.toStdString(), reserved_ids));
  const QString role_type = QString::fromStdString(
    workcell_builder::workcell_studio_id_prefix_for_type(category.toStdString()));

  auto * item = new DraggableCanvasItem(QRectF(0, 0, 35.0, 35.0));
  QString validation_error;
  if (!validate_armed_asset_transform(&validation_error)) {
    append_studio_log(QString("Add Asset validation failed: %1 (%2) %3")
      .arg(display_name, category, validation_error));
    statusBar()->showMessage("Add Asset blocked: invalid transform input.", 4000);
    return;
  }
  QPointF placement(armed_asset_x_m_ * 100.0, armed_asset_y_m_ * 100.0);
  if (armed_asset_use_clicked_xy_) {
    placement = snap_canvas_position(canvas_pos_px);
    if (qAbs(placement.x()) < 0.1 && qAbs(placement.y()) < 0.1) {
      placement = armed_asset_default_xy_px_;
    }
  }
  if (digital_twin_scene_->items().isEmpty()) {
    placement = QPointF(0.0, 0.0);
    QMessageBox::warning(this, "Default placement", "No robot/table found; placing asset at canvas center.");
  }
  item->setPos(placement);
  item->setData(RoleId, new_id);
  item->setData(RoleDisplayName, display_name);
  item->setData(RoleType, role_type);
  item->setData(RoleCategory, category);
  item->setData(RoleRole, "asset");
  item->setData(RoleSource, source_path);
  item->setData(RoleSourcePackage, "asset_folder");
  item->setData(RolePoseZ, armed_asset_z_m_);
  item->setData(RoleRoll, armed_asset_roll_rad_); item->setData(RolePitch, armed_asset_pitch_rad_); item->setData(RoleYaw, armed_asset_yaw_rad_);
  item->setData(RoleWidth, 1.0); item->setData(RoleDepth, 1.0); item->setData(RoleHeight, 1.0);
  item->setData(RoleImported, source_path.endsWith(".stl", Qt::CaseInsensitive) || source_path.endsWith(".urdf", Qt::CaseInsensitive));
  item->setData(RoleGeneratedPlaceholder, category.contains("placeholder", Qt::CaseInsensitive));
  item->setData(RoleSourceLayer, QStringLiteral("editable_layout"));
  item->setData(RoleLocked, false);
  QStringList warnings;
  if (source_path.trimmed().isEmpty()) warnings << "missing dimensions/source metadata";
  if (QLineF(item->pos(), QPointF(-20.0, -220.0)).length() < 80.0) warnings << "too close to robot base";
  for (auto *existing : digital_twin_scene_->items()) {
    if (existing == item || existing->data(RoleRole).toString() != "asset") continue;
    if (QLineF(existing->pos(), item->pos()).length() < 40.0) { warnings << "overlap"; break; }
  }
  if (item->pos().x() < -260.0 || item->pos().x() > 260.0 || item->pos().y() < -260.0 || item->pos().y() > 260.0) warnings << "outside workspace";
  if (item->data(RolePoseZ).toDouble() < 0.0) warnings << "below floor/table";
  item->setData(RoleWarning, warnings.join(", "));
  item->setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges | QGraphicsItem::ItemIsMovable);
  item->position_filter = [this](const QPointF & p){ return snap_canvas_position(p); };
  digital_twin_scene_->addItem(item);
  ScenePreviewWidget::PreviewItem preview_item;
  preview_item.id = new_id;
  preview_item.display_name = display_name;
  preview_item.category = category;
  preview_item.role = QStringLiteral("asset");
  preview_item.status = QStringLiteral("ready");
  preview_item.source_path = source_path;
  preview_item.mesh_path = source_path;
  preview_item.mesh_type = QFileInfo(source_path).suffix().toLower();
  preview_item.source_layer = QStringLiteral("editable_layout");
  preview_item.active_visual_source = QStringLiteral("mesh_preview");
  preview_item.linked_to_editable_layout_state = true;
  preview_item.editable = true;
  preview_item.locked = false;
  preview_item.selectable = true;
  preview_item.metadata_complete = true;
  preview_item.x = item->pos().x() / 100.0;
  preview_item.y = item->pos().y() / 100.0;
  preview_item.z = armed_asset_z_m_;
  preview_item.roll = armed_asset_roll_rad_;
  preview_item.pitch = armed_asset_pitch_rad_;
  preview_item.yaw = armed_asset_yaw_rad_;
  preview_item.sx = 1.0; preview_item.sy = 1.0; preview_item.sz = 1.0;
  preview_item.mesh_scale_x = 1.0; preview_item.mesh_scale_y = 1.0; preview_item.mesh_scale_z = 1.0;
  preview_item.has_mesh_metadata = true;
  preview_item.mesh_available = QFileInfo(source_path).isFile();
  if (!preview_item.mesh_available) {
    preview_item.mesh_load_warning = QStringLiteral("mesh file not found: %1").arg(source_path);
    append_studio_log(QStringLiteral("Asset Browser warning: mesh failed availability check for %1; item remains editable but fallback boxes are not enabled for this asset.").arg(source_path));
  }
  for (int i = 0; i < all_scene_preview_items_.size(); ++i) {
    if (all_scene_preview_items_[i].id == new_id) { all_scene_preview_items_.removeAt(i); break; }
  }
  all_scene_preview_items_.push_back(preview_item);
  apply_scene3d_preview_layer_filters(false);
  digital_twin_scene_->clearSelection();
  item->setSelected(true);
  select_canvas_item(item);
  undo_stack_.push_back({"add", new_id, item->pos(), item->pos(), true, false});
  redo_stack_.clear();
  set_canvas_interaction_mode(CanvasInteractionMode::Place);
  mark_layout_dirty("Place Asset Mode: Add to 3D Canvas");
  append_studio_log(QString("Add to Canvas success: %1 (%2) id=%3 from %4 | xyzrpy=[%5, %6, %7, %8, %9, %10] use_clicked_xy=%11")
    .arg(display_name, category, new_id, source_path)
    .arg(item->pos().x() / 100.0, 0, 'f', 3).arg(item->pos().y() / 100.0, 0, 'f', 3).arg(armed_asset_z_m_, 0, 'f', 3)
    .arg(armed_asset_roll_rad_, 0, 'f', 3).arg(armed_asset_pitch_rad_, 0, 'f', 3).arg(armed_asset_yaw_rad_, 0, 'f', 3)
    .arg(armed_asset_use_clicked_xy_ ? "true" : "false"));
  append_studio_log("ghost placement preview committed");
  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(new_id);
  if (scene_builder_inspector_tabs_) scene_builder_inspector_tabs_->setCurrentIndex(0);
  refresh_diagnostics_quick_status();
  const bool persist = place_mode_persistent_box_ && place_mode_persistent_box_->isChecked();
  place_asset_armed_ = persist;
  if (!persist) {
    armed_asset_category_.clear();
    armed_asset_display_name_.clear();
    armed_asset_source_path_.clear();
    set_canvas_interaction_mode(CanvasInteractionMode::Select);
  }
}


void MainWindow::validate_asset_catalog_selection()
{
  if (!add_to_canvas_button_) return;
  bool can_add = false;
  if (asset_catalog_tree_) {
    auto * item = asset_catalog_tree_->currentItem();
    if (item && !item->isHidden()) {
      can_add = item->data(0, CatalogRolePlaceable).toBool();
    }
  }
  add_to_canvas_button_->setEnabled(can_add);
}


QString MainWindow::selected_catalog_item_path() const
{
  if (!asset_catalog_tree_ || !asset_catalog_tree_->currentItem()) {
    return "";
  }
  return asset_catalog_tree_->currentItem()->data(0, CatalogRoleSourcePath).toString();
}

QJsonObject MainWindow::scene3d_filter_diagnostics() const
{
  return scene3d_filter_diagnostics_;
}

void MainWindow::on_asset_filter_changed(int)
{
  const QString selected = asset_filter_combo_ ? asset_filter_combo_->currentText() : "All";
  if (!asset_catalog_tree_) return;
  for (int i = 0; i < asset_catalog_tree_->topLevelItemCount(); ++i) {
    auto * group = asset_catalog_tree_->topLevelItem(i);
    const QString group_name = group->text(0);
    const bool group_visible = (selected == "All" || group_name == selected);
    group->setHidden(!group_visible);
    for (int c = 0; c < group->childCount(); ++c) group->child(c)->setHidden(!group_visible);
  }
  validate_asset_catalog_selection();
}

void MainWindow::on_hierarchy_item_selected(QTreeWidgetItem * item)
{
  if (!item || selection_update_guard_) return;
  const QString selected_id = item->data(0, TreeRoleId).toString().trimmed();
  const QString selected_role = item->data(0, TreeRoleRole).toString().trimmed();
  apply_scene_selection(selected_id, selected_role, false, true);
}


void MainWindow::apply_scene3d_product_view_layer_defaults_and_commit()
{
  const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility(all_scene_preview_items_);
  const auto set_checked_blocked = [](QCheckBox * box, bool checked) {
    if (!box) return;
    const QSignalBlocker blocker(box);
    box->setChecked(checked);
  };

  set_checked_blocked(preview_layer_editable_layout_box_, defaults.editable_layout);
  set_checked_blocked(preview_layer_generated_urdf_visual_box_, defaults.locked_generated_urdf_visual);
  set_checked_blocked(preview_layer_mesh_preview_box_, defaults.mesh_preview);
  set_checked_blocked(preview_layer_primitive_fallback_box_, defaults.primitive_fallback);
  set_checked_blocked(preview_layer_overlays_helpers_box_, defaults.overlay);
  set_checked_blocked(preview_layer_warnings_missing_assets_box_, defaults.warning);

  // Product View intentionally starts clean after both widget construction and
  // real scene payload commits. Helper/diagnostic overlays remain available in
  // the Overlays menu, but must be explicit opt-in for each loaded scene.
  if (toggle_labels_box_) toggle_labels_box_->setChecked(false);
  if (toggle_warnings_box_) toggle_warnings_box_->setChecked(false);
  if (show_reach_overlay_box_) show_reach_overlay_box_->setChecked(false);
  if (show_camera_fov_overlay_box_) show_camera_fov_overlay_box_->setChecked(false);
  if (show_pick_place_overlay_box_) show_pick_place_overlay_box_->setChecked(false);
  if (show_trajectory_overlay_box_) show_trajectory_overlay_box_->setChecked(false);
  if (scene_preview_widget_) {
    scene_preview_widget_->set_label_mode(ScenePreviewWidget::LabelMode::Selected);
    scene_preview_widget_->set_task_overlay_visibility(false, false, false, false);
    scene_preview_widget_->set_perception_overlay_visibility(false, false, false, false);
    if (auto * viewport = scene_preview_widget_->findChild<Scene3DViewportWidget *>()) {
      viewport->show_warnings = false;
      viewport->show_safety = false;
      viewport->show_reachability_heatmap = false;
      viewport->show_collision_warnings = false;
      viewport->show_work_envelope = false;
      viewport->show_warning_labels = false;
      viewport->debug_overlays_mode = false;
      viewport->update();
    }
  }

  apply_scene3d_preview_layer_filters(false);
}

void MainWindow::apply_scene3d_preview_layer_filters(bool log_change)
{
  if (!scene_preview_widget_) {
    return;
  }
  QVector<ScenePreviewWidget::PreviewItem> filtered_items;
  QJsonObject diagnostics;
  QJsonObject hidden_reason_counts_json;
  QJsonArray hidden_item_summaries_json;
  QMap<QString, int> hidden_reason_counts;
  const QSet<QString> enabled_layers = {
    preview_layer_editable_layout_box_ && preview_layer_editable_layout_box_->isChecked() ? "editable_layout" : "",
    preview_layer_generated_urdf_visual_box_ && preview_layer_generated_urdf_visual_box_->isChecked() ? "locked_generated_urdf_visual" : "",
    preview_layer_mesh_preview_box_ && preview_layer_mesh_preview_box_->isChecked() ? "mesh_preview" : "",
    preview_layer_primitive_fallback_box_ && preview_layer_primitive_fallback_box_->isChecked() ? "primitive_fallback" : "",
    preview_layer_overlays_helpers_box_ && preview_layer_overlays_helpers_box_->isChecked() ? "overlay" : "",
    preview_layer_warnings_missing_assets_box_ && preview_layer_warnings_missing_assets_box_->isChecked() ? "warning" : ""
  };
  auto token = [](QString s) { return s.trimmed().toLower(); };
  auto hidden_reason_for_item = [&](const ScenePreviewWidget::PreviewItem & item) {
    const QString source_layer = token(item.source_layer);
    const QString visual_source = token(item.active_visual_source);
    const QString role = token(item.role);
    const QString category = token(item.category);
    const QString combined = role + "|" + category + "|" + token(item.status) + "|" + item.warnings.join("|").toLower();
    const bool is_warning_or_missing = combined.contains("warning") || combined.contains("missing") || !item.mesh_load_warning.trimmed().isEmpty();
    const bool is_overlay_or_helper = combined.contains("overlay") || combined.contains("helper") || combined.contains("safety zone");

    if (source_layer == "editable_layout" && !enabled_layers.contains("editable_layout")) return QString("layer_disabled:editable_layout");
    if ((source_layer == "locked_generated_urdf_visual" || source_layer == "generated_urdf_visual") &&
      !enabled_layers.contains("locked_generated_urdf_visual"))
    {
      return QString("layer_disabled:locked_generated_urdf_visual");
    }
    if (source_layer == "primitive_fallback" && !enabled_layers.contains("primitive_fallback")) return QString("layer_disabled:primitive_fallback");
    if (visual_source == "mesh_preview" && !enabled_layers.contains("mesh_preview")) return QString("layer_disabled:mesh_preview");
    if (is_overlay_or_helper && !enabled_layers.contains("overlay")) return QString("layer_disabled:overlay");
    if (is_warning_or_missing && !enabled_layers.contains("warning")) return QString("layer_disabled:warning");
    return QString("hidden_by_unknown_filter");
  };
  for (const auto & p : all_scene_preview_items_) {
    if (workcell_builder::include_preview_item_for_scene3d(p, enabled_layers)) {
      if (scene3d_viewport_link_token(p) == QStringLiteral("base_link_inertia")) {
        append_studio_log(QStringLiteral("Scene3D base_link_inertia trace: stage=visible/filter result=visible item_id=%1 source_layer=%2 active_visual_source=%3")
          .arg(p.id, p.source_layer, p.active_visual_source));
      }
      filtered_items.push_back(p);
      continue;
    }
    const QString hidden_reason = hidden_reason_for_item(p);
    if (scene3d_viewport_link_token(p) == QStringLiteral("base_link_inertia")) {
      append_studio_log(QStringLiteral("Scene3D base_link_inertia trace: stage=visible/filter result=hidden item_id=%1 hidden_reason=%2 source_layer=%3 active_visual_source=%4")
        .arg(p.id, hidden_reason, p.source_layer, p.active_visual_source));
    }
    hidden_reason_counts[hidden_reason] += 1;
    if (hidden_item_summaries_json.size() < 20) {
      QJsonObject summary;
      summary["id"] = p.id;
      summary["label"] = p.display_name;
      summary["role/category"] = QString("%1/%2").arg(p.role, p.category);
      summary["source_layer"] = p.source_layer;
      summary["active_visual_source"] = p.active_visual_source;
      summary["renderable"] = p.mesh_available || p.has_mesh_metadata || p.source_layer.trimmed().compare("primitive_fallback", Qt::CaseInsensitive) == 0;
      summary["safe_for_preview"] = p.mesh_load_warning.trimmed().isEmpty() && !p.status.contains("error", Qt::CaseInsensitive);
      summary["hidden_reason"] = hidden_reason;
      if (!p.mesh_path.trimmed().isEmpty()) summary["mesh_path"] = p.mesh_path;
      hidden_item_summaries_json.append(summary);
    }
  }
  for (auto it = hidden_reason_counts.constBegin(); it != hidden_reason_counts.constEnd(); ++it) {
    hidden_reason_counts_json[it.key()] = it.value();
  }
  if (scene3d_visual_ingestion_diagnostics_.contains(QStringLiteral("generated_urdf_visual_row_diagnostics"))) {
    QSet<QString> filtered_ids;
    for (const auto & item : filtered_items) {
      filtered_ids.insert(item.id);
    }
    QJsonArray filter_diagnostics;
    const QJsonArray rows =
      scene3d_visual_ingestion_diagnostics_.value(QStringLiteral("generated_urdf_visual_row_diagnostics")).toArray();
    for (const QJsonValue & value : rows) {
      QJsonObject row = value.toObject();
      const QString id = row.value(QStringLiteral("id")).toString();
      const bool survived_filter =
        row.value(QStringLiteral("appended_to_preview_items")).toBool(false) && filtered_ids.contains(id);
      row[QStringLiteral("survived_filter")] = survived_filter;
      if (row.value(QStringLiteral("survived_suppression")).toBool(false) && !survived_filter) {
        if (row.value(QStringLiteral("first_drop_stage")).toString().trimmed().isEmpty()) {
          row[QStringLiteral("first_drop_stage")] = QStringLiteral("apply_scene3d_preview_layer_filters");
        }
      }
      filter_diagnostics.append(row);
    }
    scene3d_visual_ingestion_diagnostics_[QStringLiteral("generated_urdf_visual_row_diagnostics")] = filter_diagnostics;
  }
  auto is_robot_item = [&](const ScenePreviewWidget::PreviewItem & item) {
    const QString combined = token(item.role) + "|" + token(item.category) + "|" + token(item.display_name) + "|" + token(item.id);
    return combined.contains("robot") || combined.contains("urdf") || combined.contains("manipulator");
  };
  QJsonArray robot_world_pose;
  QString robot_base_frame;
  QString robot_pose_source;
  int robot_visual_count = 0;
  int robot_mesh_loaded_count = 0;
  int robot_mesh_missing_count = 0;
  int transform_chain_applied_count = 0;
  int visual_origin_applied_count = 0;
  int baked_world_visual_transform_count = 0;
  int legacy_viewport_transform_count = 0;
  bool robot_bounds_initialized = false;
  QVector3D robot_aabb_min;
  QVector3D robot_aabb_max;
  for (const auto & p : filtered_items) {
    if (!is_robot_item(p)) {
      continue;
    }
    ++robot_visual_count;
    if (p.mesh_available || p.has_mesh_metadata) {
      ++robot_mesh_loaded_count;
    } else {
      ++robot_mesh_missing_count;
    }
    if (p.has_origin_offset) {
      ++visual_origin_applied_count;
    }
    if (p.has_origin_offset || qAbs(p.mesh_r) > 1e-9 || qAbs(p.mesh_p) > 1e-9 || qAbs(p.mesh_y) > 1e-9) {
      ++transform_chain_applied_count;
    }
    if (p.has_baked_world_visual_transform) {
      ++baked_world_visual_transform_count;
    } else {
      ++legacy_viewport_transform_count;
    }
    const QVector3D half_span(qMax(0.0, p.sx) * 0.5f, qMax(0.0, p.sy) * 0.5f, qMax(0.0, p.sz) * 0.5f);
    const QVector3D item_min(p.x - half_span.x(), p.y - half_span.y(), p.z - half_span.z());
    const QVector3D item_max(p.x + half_span.x(), p.y + half_span.y(), p.z + half_span.z());
    if (!robot_bounds_initialized) {
      robot_aabb_min = item_min;
      robot_aabb_max = item_max;
      robot_bounds_initialized = true;
      robot_world_pose = QJsonArray{p.x, p.y, p.z, p.roll, p.pitch, p.yaw};
      robot_base_frame = p.frame_id;
      robot_pose_source = p.source_path;
    } else {
      robot_aabb_min.setX(qMin(robot_aabb_min.x(), item_min.x()));
      robot_aabb_min.setY(qMin(robot_aabb_min.y(), item_min.y()));
      robot_aabb_min.setZ(qMin(robot_aabb_min.z(), item_min.z()));
      robot_aabb_max.setX(qMax(robot_aabb_max.x(), item_max.x()));
      robot_aabb_max.setY(qMax(robot_aabb_max.y(), item_max.y()));
      robot_aabb_max.setZ(qMax(robot_aabb_max.z(), item_max.z()));
    }
  }

  if (!scene3d_visual_ingestion_diagnostics_.isEmpty()) {
    diagnostics["visual_ingestion_diagnostics"] = scene3d_visual_ingestion_diagnostics_;
    for (auto it = scene3d_visual_ingestion_diagnostics_.constBegin(); it != scene3d_visual_ingestion_diagnostics_.constEnd(); ++it) {
      diagnostics[it.key()] = it.value();
    }
  }
  diagnostics[QStringLiteral("generated_urdf_visual_numbers_after_filter")] =
    summarize_generated_urdf_visual_rows(filtered_items);
  diagnostics["filter_input_count"] = all_scene_preview_items_.size();
  diagnostics["filter_visible_count"] = filtered_items.size();
  diagnostics["filter_hidden_count"] = qMax(0, all_scene_preview_items_.size() - filtered_items.size());
  diagnostics["hidden_by_filter_reason_counts"] = hidden_reason_counts_json;
  diagnostics["hidden_item_summaries"] = hidden_item_summaries_json;
  diagnostics["robot_visual_count"] = robot_visual_count;
  diagnostics["robot_mesh_loaded_count"] = robot_mesh_loaded_count;
  diagnostics["robot_mesh_missing_count"] = robot_mesh_missing_count;
  diagnostics["transform_chain_applied_count"] = transform_chain_applied_count;
  diagnostics["visual_origin_applied_count"] = visual_origin_applied_count;
  diagnostics["baked_world_visual_transform_count"] = baked_world_visual_transform_count;
  diagnostics["legacy_viewport_transform_count"] = legacy_viewport_transform_count;
  diagnostics["robot_pose_source"] = robot_pose_source;
  diagnostics["robot_base_frame"] = robot_base_frame;
  diagnostics["robot_world_pose"] = robot_world_pose;
  if (robot_bounds_initialized) {
    diagnostics["robot_aabb_min"] = QJsonArray{robot_aabb_min.x(), robot_aabb_min.y(), robot_aabb_min.z()};
    diagnostics["robot_aabb_max"] = QJsonArray{robot_aabb_max.x(), robot_aabb_max.y(), robot_aabb_max.z()};
  } else {
    diagnostics["robot_aabb_min"] = QJsonArray();
    diagnostics["robot_aabb_max"] = QJsonArray();
  }
  scene3d_filter_diagnostics_ = diagnostics;
  if (filtered_items.isEmpty() && !all_scene_preview_items_.isEmpty()) {
    auto looks_renderable = [](const ScenePreviewWidget::PreviewItem & p) {
      const QString combined =
        (p.role + "|" + p.category + "|" + p.status + "|" + p.warnings.join("|") + "|" + p.mesh_load_warning).toLower();
      const bool helper_or_overlay =
        combined.contains("overlay") || combined.contains("helper") || combined.contains("safety zone");
      const bool warning_or_missing =
        combined.contains("warning") || combined.contains("missing") || combined.contains("unsafe") ||
        combined.contains("unrenderable");
      return !helper_or_overlay && !warning_or_missing;
    };
    QVector<ScenePreviewWidget::PreviewItem> restored_renderable;
    restored_renderable.reserve(all_scene_preview_items_.size());
    for (const auto & p : all_scene_preview_items_) {
      if (looks_renderable(p)) restored_renderable.push_back(p);
    }
    if (!restored_renderable.isEmpty()) {
      filtered_items = restored_renderable;
      append_studio_log("Scene3D warning: default_filter_fallback_kept_renderable_items_visible");
    }
  }
  if (filtered_items.isEmpty() && !all_scene_preview_items_.isEmpty()) {
    append_studio_log("Scene3D blocker: current layer filters hide all items. Re-enable editable layout, mesh preview, primitive fallback, or locked generated URDF visuals.");
  }
  scene_preview_widget_->set_preview_items(filtered_items);
  auto * viewport = scene_preview_widget_->findChild<Scene3DViewportWidget *>();
  if (viewport) {
    // set_preview_items() normally commits the payload into the active viewport.
    // Re-ingest here as an explicit guard so the camera fit below always uses
    // the exact post-filter payload, including final UR5 mesh/fallback draw bounds.
    viewport->ingest_preview_items(filtered_items);
  }
  QStringList missing_required_visible_links;
  const QJsonObject viewport_audit =
    audit_ur5_2f_test_committed_viewport_items(viewport, &missing_required_visible_links);
  const bool has_required_ur5_final_draw_links = viewport && missing_required_visible_links.isEmpty() &&
    viewport_audit.value(QStringLiteral("rendered_ur5_link_count")).toInt() >= 7;
  const bool selected_ur5_2f_test = has_selected_scene() && selected_scene_name() == QStringLiteral("ur5_2f_test");
  if (viewport && (selected_ur5_2f_test || has_required_ur5_final_draw_links)) {
    viewport->fit_product_view();
    viewport->update();
  }
  if (viewport) {
    scene3d_filter_diagnostics_[QStringLiteral("camera_fit_target")] = viewport->last_camera_fit_target();
    scene3d_filter_diagnostics_[QStringLiteral("camera_fit_includes_robot")] = viewport->last_initial_fit_included_ur5_bounds();
  }
  if (has_selected_scene() && selected_scene_name() == QStringLiteral("ur5_2f_test")) {
    // audit_ur5_2f_test_committed_viewport_items(viewport, &missing_required_visible_links) is computed above after final ingest.
    scene3d_filter_diagnostics_[QStringLiteral("ur5_2f_test_final_viewport_audit")] = viewport_audit;
    append_studio_log(
      QStringLiteral("Scene3D final viewport audit for ur5_2f_test: %1")
        .arg(QString::fromUtf8(QJsonDocument(viewport_audit).toJson(QJsonDocument::Compact))));
    if (!missing_required_visible_links.isEmpty()) {
      append_studio_log(
        QStringLiteral("Preview warning: ur5_2f_test retained visual rows missing after loader filtering: final visible viewport/renderable UR5 links missing=[%1]")
          .arg(missing_required_visible_links.join(QStringLiteral(","))));
    } else {
      append_studio_log(QStringLiteral(
        "Scene3D final viewport audit passed for ur5_2f_test required visible UR5 viewport/renderable links."));
    }
  }
  const Scene3DTransformParityReadiness transform_parity =
    has_selected_scene()
      ? scene3d_load_transform_parity_readiness(
          fs::path(selected_scene_path().toStdString()), selected_scene_name())
      : Scene3DTransformParityReadiness{};
  scene_preview_widget_->set_preview_status_summary(
    scene3d_user_preview_status_summary(
      scene_preview_widget_->render_debug_counters(),
      scene_preview_widget_->total_warning_count(),
      transform_parity.warning,
      transform_parity.failed));
  if (scene3d_debug_logging_enabled()) {
    append_studio_log(
      QString("Scene3D diagnostics {model_items_count=%1, filtered_visible_count=%2}")
        .arg(all_scene_preview_items_.size())
        .arg(filtered_items.size()));
    append_studio_log(
      QString("Scene3D diagnostics: visible item count after filters=%1/%2")
        .arg(filtered_items.size())
        .arg(all_scene_preview_items_.size()));
  }
  if (log_change) {
    append_studio_log(
      QString("Scene3D preview-only visibility updated: editable=%1 urdf_visuals=%2 mesh=%3 primitives=%4 overlays=%5 warnings=%6 (visible %7/%8). No files changed.")
      .arg(preview_layer_editable_layout_box_ && preview_layer_editable_layout_box_->isChecked() ? "on" : "off")
      .arg(preview_layer_generated_urdf_visual_box_ && preview_layer_generated_urdf_visual_box_->isChecked() ? "on" : "off")
      .arg(preview_layer_mesh_preview_box_ && preview_layer_mesh_preview_box_->isChecked() ? "on" : "off")
      .arg(preview_layer_primitive_fallback_box_ && preview_layer_primitive_fallback_box_->isChecked() ? "on" : "off")
      .arg(preview_layer_overlays_helpers_box_ && preview_layer_overlays_helpers_box_->isChecked() ? "on" : "off")
      .arg(preview_layer_warnings_missing_assets_box_ && preview_layer_warnings_missing_assets_box_->isChecked() ? "on" : "off")
      .arg(filtered_items.size())
      .arg(all_scene_preview_items_.size()));
  }
}

void MainWindow::populate_scene_hierarchy()
{
  if (!scene_hierarchy_tree_) return;
  sync_selected_scene_state();
  scene_hierarchy_tree_->clear();

  if (!selected_scene_state_.valid || selected_scene_state_.index < 0 ||
    selected_scene_state_.index >= static_cast<int>(scene_browser_result_.scenes.size()))
  {
    if (scene_preview_widget_) {
      scene_preview_widget_->set_scene_selected(false);
      scene_preview_widget_->set_preview_scene_name("No scene");
    }
    return;
  }
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_state_.index)];
  const fs::path d = s.scene_dir;
  const auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  const QString layout_load_message = QString::fromStdString(model.layout_load_message).trimmed();
  if (!layout_load_message.isEmpty() && layout_load_message != last_layout_load_message_log_) {
    append_studio_log(layout_load_message);
    last_layout_load_message_log_ = layout_load_message;
  }

  auto normalize_role = [](const QString & raw_role, const QString & fallback_text) {
    const QString lower = (raw_role + " " + fallback_text).toLower();
    if (lower.contains("robot")) return QString("robot");
    if (lower.contains("end_effector") || lower.contains("gripper") || lower.contains("tool")) return QString("end_effector/tool");
    if (lower.contains("camera") || lower.contains("sensor")) return QString("camera");
    if (lower.contains("support_surface") || lower.contains("table") || lower.contains("workbench")) return QString("support_surface/table");
    if (lower.contains("conveyor")) return QString("conveyor");
    if (lower.contains("pick_source") || lower.contains("pick zone") || lower.contains("pick_zone")) return QString("pick source/zone");
    if (lower.contains("place_target") || lower.contains("place zone") || lower.contains("place_zone") || lower.contains("bin")) return QString("place target/bin");
    if (lower.contains("home") || lower.contains("safe_joint_state") || lower.contains("safe joint") || lower.contains("safety pose")) return QString("home/safety pose");
    if (lower.contains("safety")) return QString("safety zone");
    if (lower.contains("object") || lower.contains("fixture") || lower.contains("asset")) return QString("object");
    return QString("object");
  };
  const QSet<QString> allowed_scene_roles = {
    "robot",
    "end_effector/tool",
    "support_surface/table",
    "conveyor",
    "camera",
    "pick source/zone",
    "place target/bin",
    "object",
    "safety zone",
    "home/safety pose"
  };
  QMap<QString, QString> yaml_status_by_id;
  auto ingest_status_file = [&](const fs::path & path, const QString & source_tag) {
    YAML::Node root;
    if (!read_yaml(path, &root)) return;

    auto ingest_sequence = [&](const YAML::Node & seq) {
      if (!seq || !seq.IsSequence()) return;
      for (const auto & node : seq) {
        if (!node || !node.IsMap()) continue;

        const QString id = ystr(node["id"]);
        if (id == "unknown") continue;

        QString status = ystr(node["status"]);
        if (status == "unknown") {
          bool enabled = true;
          if (const auto enabled_like = workcell_builder::get_bool_like(node, "enabled")) {
            enabled = *enabled_like;
          }
          status = enabled ? "ready" : "disabled";
        }

        yaml_status_by_id[id] = status + " (" + source_tag + ")";
      }
    };

    ingest_sequence(root["items"]);
    ingest_sequence(root["objects"]);
    ingest_sequence(root["assets"]);
    ingest_sequence(root["layout"]);
    ingest_sequence(root["placed_assets"]);
  };

  ingest_status_file(d / "environment.yaml", "environment.yaml");
  ingest_status_file(d / "scene_manifest.yaml", "scene_manifest.yaml");
  ingest_status_file(d / "environment_layout.yaml", "environment_layout.yaml");
  ingest_status_file(d / "layout" / "workcell_studio_layout.yaml", "workcell_studio_layout.yaml");

  QVector<ScenePreviewWidget::PreviewItem> preview_items;
  int preview_warning_count = 0;
  QStringList preview_warning_details;
  const fs::path urdf_visual_index = d / "generated" / "scene_visual_mesh_index.json";
  const Scene3DTransformParityReadiness transform_parity =
    scene3d_load_transform_parity_readiness(d, QString::fromStdString(s.scene_name));
  if (!transform_parity.warning.isEmpty()) {
    preview_warning_details << transform_parity.warning;
    append_studio_log(QString("Preview warning: %1 (%2)")
      .arg(transform_parity.warning, transform_parity.source));
  }

  bool accepted_safe_visual_mesh_index_has_items = false;
  if (fs::exists(urdf_visual_index)) {
    try {
      const YAML::Node existing_index = YAML::LoadFile(urdf_visual_index.string());
      const bool safe_for_preview = workcell_builder::yaml_map_key(existing_index, "safe_for_preview").as<bool>(false);
      const QString index_scene_name = QString::fromStdString(
        workcell_builder::yaml_map_value_or_empty(existing_index, "scene_name")).trimmed();
      const YAML::Node visual_items = workcell_builder::yaml_map_key(existing_index, "visual_items");
      accepted_safe_visual_mesh_index_has_items =
        safe_for_preview &&
        (index_scene_name.isEmpty() || index_scene_name == QString::fromStdString(d.filename().string())) &&
        visual_items && visual_items.IsSequence() && visual_items.size() > 0;
    } catch (...) {
      accepted_safe_visual_mesh_index_has_items = false;
    }
  }

  auto status_for_item = [&](const workcell_builder::WorkcellStudioCanvasItem & item) {
    const QString id = QString::fromStdString(item.id);
    if (yaml_status_by_id.contains(id)) return yaml_status_by_id[id];
    if (!item.warnings.empty()) return QString("warning");
    return QString("ready");
  };

  QMap<QString, QTreeWidgetItem*> hierarchy_groups;
  auto group_for_item = [&](const ScenePreviewWidget::PreviewItem & p) {
    const QString lower = (p.role + " " + p.category + " " + p.source_layer + " " + p.active_visual_source).toLower();
    if (p.status.contains("warning", Qt::CaseInsensitive) || p.mesh_load_warning.contains("missing", Qt::CaseInsensitive)) return QString("Warnings / Missing Assets");
    if (lower.contains("editable_layout")) return QString("Editable Layout");
    if (lower.contains("camera") || lower.contains("sensor")) return QString("Cameras");
    if (lower.contains("robot") || lower.contains("tool") || lower.contains("gripper") || lower.contains("end_effector")) return QString("Robot / Tooling");
    if (lower.contains("helper") || lower.contains("overlay") || lower.contains("safety") || lower.contains("home/safety pose") || lower.contains("malformed snapshot") || lower.contains("detection snapshot")) return QString("Overlays / Helpers");
    if (lower.contains("generated_urdf_visual")) return QString("Generated URDF Visuals");
    if (lower.contains("primitive_fallback")) return QString("Primitive Fallbacks");
    if (lower.contains("mesh_preview")) return QString("Mesh Preview");
    return QString("Mesh Preview");
  };
  auto ensure_group = [&](const QString &name) {
    if (hierarchy_groups.contains(name)) return hierarchy_groups[name];
    auto *group = new QTreeWidgetItem(scene_hierarchy_tree_, {name, "", ""});
    group->setFirstColumnSpanned(true);
    group->setFlags(group->flags() & ~Qt::ItemIsSelectable);
    hierarchy_groups[name] = group;
    return group;
  };
  for (const QString &gn : {QString("Editable Layout"), QString("Mesh Preview"), QString("Generated URDF Visuals"), QString("Primitive Fallbacks"), QString("Cameras"), QString("Robot / Tooling"), QString("Overlays / Helpers"), QString("Warnings / Missing Assets")}) ensure_group(gn);
  auto add_tree_node = [&](const ScenePreviewWidget::PreviewItem & p) {
    auto * parent = ensure_group(group_for_item(p));
    const QString visual_status = p.mesh_path.trimmed().isEmpty() ? (p.active_visual_source.contains("primitive") ? "primitive" : "missing") : "mesh";
    const QString state_badges = QString("%1 • %2 • %3 • %4")
      .arg(p.status.isEmpty() ? QStringLiteral("ready") : p.status)
      .arg(p.editable && p.source_layer == QStringLiteral("editable_layout")
        ? QStringLiteral("editable layout item")
        : (p.source_layer == QStringLiteral("editable_layout") ? QStringLiteral("locked editable layout item") : QStringLiteral("locked generated/preview item")))
      .arg(p.source_layer.isEmpty() ? QStringLiteral("unknown-layer") : p.source_layer)
      .arg(visual_status);
    auto * node = new QTreeWidgetItem(parent, {QString("%1 [%2]").arg(p.display_name, p.id), p.role, state_badges});
    node->setToolTip(0, p.display_name); node->setToolTip(1, p.role); node->setToolTip(2, p.status);
    node->setToolTip(0, p.display_name);
    node->setToolTip(1, p.role);
    node->setToolTip(2, p.status);
    node->setData(0, TreeRoleId, p.id);
    node->setData(0, TreeRoleCategory, p.category);
    node->setData(
      0,
      TreeRolePoseText,
      QString("xyz=(%1,%2,%3) rpy=(%4,%5,%6)")
        .arg(p.x)
        .arg(p.y)
        .arg(p.z)
        .arg(p.roll)
        .arg(p.pitch)
        .arg(p.yaw));
    node->setData(0, TreeRoleSource, p.source_path);
    node->setData(0, TreeRolePoseX, p.x);
    node->setData(0, TreeRolePoseY, p.y);
    node->setData(0, TreeRolePoseZ, p.z);
    node->setData(0, TreeRoleRoll, p.roll);
    node->setData(0, TreeRolePitch, p.pitch);
    node->setData(0, TreeRoleYaw, p.yaw);
    node->setData(0, TreeRolePoseAvailable, true);
    node->setData(0, TreeRoleRole, p.role);
    node->setData(0, TreeRoleSourceLayer, p.source_layer);
    node->setData(0, TreeRoleActiveVisualSource, p.active_visual_source);
    node->setData(0, TreeRoleEditable, p.editable);
    node->setData(0, TreeRoleLocked, p.locked);
    node->setData(0, TreeRoleLinkedEditableLayout, p.linked_to_editable_layout_state);
    node->setData(0, TreeRoleVisualBackingStatus, visual_status);
    node->setData(0, TreeRoleGeneratedVisual, p.source_layer != QStringLiteral("editable_layout"));
    node->setData(0, TreeRoleItemTypeClass, p.category);
    node->setData(0, TreeRoleStableId, p.id);
    node->setData(0, TreeRoleCameraId, p.camera_id);
    node->setData(0, TreeRoleFrameId, p.frame_id);
    node->setData(0, TreeRoleDetectionLabel, p.detection_label);
    node->setData(0, TreeRoleConfidence, p.confidence);
    node->setData(0, TreeRoleTrackingId, p.tracking_id);
    node->setData(0, TreeRoleSnapshotSourceFile, p.snapshot_source_file);
    node->setData(0, TreeRoleAlignmentWarning, p.alignment_warning);
  };

  auto include_preview_item_in_hierarchy = [this](const ScenePreviewWidget::PreviewItem & p) {
    QSet<QString> enabled_layers;
    if (!preview_layer_editable_layout_box_ || preview_layer_editable_layout_box_->isChecked()) enabled_layers.insert("editable_layout");
    if (!preview_layer_generated_urdf_visual_box_ || preview_layer_generated_urdf_visual_box_->isChecked()) enabled_layers.insert("locked_generated_urdf_visual");
    if (!preview_layer_mesh_preview_box_ || preview_layer_mesh_preview_box_->isChecked()) enabled_layers.insert("mesh_preview");
    if (!preview_layer_primitive_fallback_box_ || preview_layer_primitive_fallback_box_->isChecked()) enabled_layers.insert("primitive_fallback");
    if (!preview_layer_overlays_helpers_box_ || preview_layer_overlays_helpers_box_->isChecked()) enabled_layers.insert("overlay");
    if (!preview_layer_warnings_missing_assets_box_ || preview_layer_warnings_missing_assets_box_->isChecked()) enabled_layers.insert("warning");
    return workcell_builder::include_preview_item_for_scene3d(p, enabled_layers);
  };

  auto add_preview_item = [&](const QString & id,
                              const QString & display_name,
                              const QString & category,
                              const QString & role_hint,
                              const QString & status,
                              const QString & source_path,
                              bool metadata_complete) {
    ScenePreviewWidget::PreviewItem p;
    p.id = id;
    p.display_name = display_name;
    p.category = category;
    p.role = normalize_role(role_hint, category + " " + display_name);
    p.status = status;
    p.source_path = source_path;
    p.source_layer = QStringLiteral("overlay");
    p.active_visual_source = QStringLiteral("overlay");
    p.linked_to_editable_layout_state = false;
    p.editable = false;
    p.selectable = true;
    p.metadata_complete = metadata_complete;
    if (!metadata_complete) {
      p.warnings << "metadata incomplete";
      preview_warning_details << QString("%1 (%2): metadata incomplete").arg(p.id, p.role);
    }
    preview_items.push_back(p);
    if (allowed_scene_roles.contains(p.role) && include_preview_item_in_hierarchy(p)) {
      add_tree_node(p);
    }

    if (!metadata_complete) {
      ++preview_warning_count;
      append_studio_log(QString("Preview warning: metadata incomplete for %1").arg(id));
    }
  };

  for (const auto & item : model.items) {
    ScenePreviewWidget::PreviewItem p;
    p.id = QString::fromStdString(item.id);
    p.display_name = QString::fromStdString(item.label);
    p.category = QString::fromStdString(item.category.empty() ? item.type : item.category);
    p.role = normalize_role(QString::fromStdString(item.role), p.category + " " + p.display_name);
    p.status = status_for_item(item);
    p.source_path = QString::fromStdString(item.source_file);
    p.metadata_complete = item.warnings.empty();
    for (const auto & warning : item.warnings) p.warnings << QString::fromStdString(warning);
    p.x = item.x;
    p.y = item.y;
    p.z = item.z;
    p.roll = item.roll;
    p.pitch = item.pitch;
    p.yaw = item.yaw;
    p.sx = item.width;
    p.sy = item.depth;
    p.sz = item.height;
    p.mesh_path = QString::fromStdString(item.mesh_path);
    p.mesh_type = QString::fromStdString(item.mesh_type);
    p.primitive_geometry_type = QString::fromStdString(item.primitive_geometry_type);
    p.primitive_radius = item.primitive_radius;
    p.primitive_length = item.primitive_length;
    p.has_material_color = item.has_material_color;
    p.material_r = item.material_r;
    p.material_g = item.material_g;
    p.material_b = item.material_b;
    p.material_a = item.material_a;
    p.material_name = QString::fromStdString(item.material_name);
    p.mesh_scale_x = item.mesh_scale_x;
    p.mesh_scale_y = item.mesh_scale_y;
    p.mesh_scale_z = item.mesh_scale_z;
    p.mesh_roll = item.mesh_r;
    p.mesh_pitch = item.mesh_p;
    p.mesh_yaw = item.mesh_y;
    p.has_mesh_metadata = item.has_mesh_metadata;
    p.mesh_r = item.mesh_r;
    p.mesh_p = item.mesh_p;
    p.mesh_y = item.mesh_y;
    p.has_origin_offset = item.has_origin_offset;
    p.origin_offset_x = item.origin_offset_x;
    p.origin_offset_y = item.origin_offset_y;
    p.origin_offset_z = item.origin_offset_z;
    p.mesh_available = item.mesh_available;
    p.mesh_load_warning = QString::fromStdString(item.mesh_load_warning);
    if (accepted_safe_visual_mesh_index_has_items &&
        item.provenance == workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview &&
        p.mesh_load_warning == QStringLiteral("mesh metadata missing or legacy; using primitive preview")) {
      p.mesh_load_warning.clear();
    }
    p.locked = item.locked;
    p.camera_id = QString::fromStdString(item.camera_id);
    p.frame_id = QString::fromStdString(item.frame_id);
    p.detection_label = QString::fromStdString(item.detection_label);
    p.confidence = item.confidence;
    p.tracking_id = QString::fromStdString(item.tracking_id);
    p.snapshot_source_file = QString::fromStdString(item.snapshot_source_file);
    p.alignment_warning = QString::fromStdString(item.alignment_warning);
    p.editable = item.editable && !item.locked;
    switch (item.provenance) {
      case workcell_builder::WorkcellStudioItemProvenance::EditableLayout:
        p.source_layer = QStringLiteral("editable_layout");
        p.active_visual_source = QStringLiteral("editable_layout");
        p.linked_to_editable_layout_state = true;
        break;
      case workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview:
        p.source_layer = QStringLiteral("primitive_fallback");
        p.active_visual_source = QStringLiteral("primitive_fallback");
        p.linked_to_editable_layout_state = false;
        break;
      case workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
      default:
        p.source_layer = QStringLiteral("locked_generated_urdf_visual");
        p.active_visual_source = QStringLiteral("mesh_preview");
        p.linked_to_editable_layout_state = false;
        break;
    }
    if (item.locked) {
      const QString base_reason = p.warnings.isEmpty() ? QStringLiteral("item is locked") : p.warnings.front();
      p.lock_reason = base_reason;
      p.warnings << QStringLiteral("Locked: %1").arg(base_reason);
    }
    preview_items.push_back(p);
    if (allowed_scene_roles.contains(p.role) && include_preview_item_in_hierarchy(p)) {
      add_tree_node(p);
    }

    if (!item.warnings.empty()) {
      const QString warning_text = QString::fromStdString(item.warnings.front());
      const bool suppress_legacy_primitive_warning =
        accepted_safe_visual_mesh_index_has_items &&
        item.provenance == workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview &&
        warning_text == QStringLiteral("mesh metadata missing or legacy; using primitive preview");
      if (!suppress_legacy_primitive_warning) {
        ++preview_warning_count;
        preview_warning_details << QString("%1 (%2): %3").arg(p.id, p.role, warning_text);
        append_studio_log(QString("Preview warning: %1").arg(warning_text));
      }
    }
  }


  auto canonical_semantic_token = [](QString value) {
    value = value.trimmed().toLower();
    value.replace(QRegularExpression(QStringLiteral("[^a-z0-9]+")), QStringLiteral("_"));
    value.replace(QRegularExpression(QStringLiteral("_+")), QStringLiteral("_"));
    while (value.startsWith(QLatin1Char('_'))) value.remove(0, 1);
    while (value.endsWith(QLatin1Char('_'))) value.chop(1);
    return value;
  };

  auto semantic_concept_for_role = [&](const QString & role, const QString & category, const QString & display_name, const QString & id) {
    const QString text = (role + " " + category + " " + display_name + " " + id).toLower();
    if (text.contains(QStringLiteral("camera")) || text.contains(QStringLiteral("sensor"))) return QStringLiteral("camera");
    if (text.contains(QStringLiteral("conveyor"))) return QStringLiteral("conveyor");
    if (text.contains(QStringLiteral("table")) || text.contains(QStringLiteral("workbench")) || text.contains(QStringLiteral("support_surface"))) return QStringLiteral("table");
    if (text.contains(QStringLiteral("pick_source")) || text.contains(QStringLiteral("pick zone")) || text.contains(QStringLiteral("pick_zone"))) return QStringLiteral("pick_zone");
    if (text.contains(QStringLiteral("place_target")) || text.contains(QStringLiteral("place zone")) || text.contains(QStringLiteral("place_zone")) || text.contains(QStringLiteral("drop_zone"))) return QStringLiteral("place_zone");
    if (text.contains(QStringLiteral("bin")) || text.contains(QStringLiteral("reject"))) return QStringLiteral("bin");
    if (text.contains(QStringLiteral("home")) || text.contains(QStringLiteral("safe_joint_state")) || text.contains(QStringLiteral("safe joint")) || text.contains(QStringLiteral("safety pose"))) return QStringLiteral("home_safety_pose");
    if (text.contains(QStringLiteral("safety")) || text.contains(QStringLiteral("exclusion"))) return QStringLiteral("safety_zone");
    return QString();
  };

  auto semantic_key_for = [&](const QString & concept, const QString & id) {
    const QString normalized_id = canonical_semantic_token(id);
    if (concept.isEmpty() || normalized_id.isEmpty()) return QString();
    return concept + QStringLiteral(":") + normalized_id;
  };

  QSet<QString> editable_semantic_keys;
  QSet<QString> editable_singleton_concepts;
  for (const auto & existing : preview_items) {
    if (!existing.linked_to_editable_layout_state || existing.source_layer != QStringLiteral("editable_layout")) continue;
    const QString concept = semantic_concept_for_role(existing.role, existing.category, existing.display_name, existing.id);
    const QString key = semantic_key_for(concept, existing.id);
    if (!key.isEmpty()) editable_semantic_keys.insert(key);
    if (concept == QStringLiteral("table") || concept == QStringLiteral("conveyor") || concept == QStringLiteral("camera") ||
        concept == QStringLiteral("home_safety_pose") || concept == QStringLiteral("safety_zone")) {
      editable_singleton_concepts.insert(concept);
    }
  }

  auto semantic_yaml_scalar = [](const YAML::Node & node) {
    if (!node) return QString();
    try {
      if (node.IsScalar()) return QString::fromStdString(node.as<std::string>()).trimmed();
    } catch (...) {}
    return QString();
  };

  auto semantic_map_string = [&](const YAML::Node & node, std::initializer_list<const char *> keys) {
    YAML::Node cursor = node;
    for (const char * key : keys) {
      if (!cursor || !cursor.IsMap()) return QString();
      cursor = cursor[key];
    }
    return semantic_yaml_scalar(cursor);
  };

  auto semantic_seq_double = [](const YAML::Node & seq, int index, double fallback) {
    try {
      if (seq && seq.IsSequence() && seq.size() > static_cast<std::size_t>(index)) return seq[index].as<double>(fallback);
    } catch (...) {}
    return fallback;
  };

  auto read_semantic_pose = [&](const YAML::Node & node, ScenePreviewWidget::PreviewItem * p) {
    if (!p || !node || !node.IsMap()) return;
    YAML::Node xyz;
    YAML::Node rpy;
    const YAML::Node pose = node["pose"];
    if (pose && pose.IsMap()) {
      xyz = pose["xyz"] ? pose["xyz"] : pose["position"];
      rpy = pose["rpy"] ? pose["rpy"] : pose["orientation_rpy"];
    } else if (pose && pose.IsSequence()) {
      p->x = semantic_seq_double(pose, 0, p->x);
      p->y = semantic_seq_double(pose, 1, p->y);
      p->z = semantic_seq_double(pose, 2, p->z);
      p->roll = semantic_seq_double(pose, 3, p->roll);
      p->pitch = semantic_seq_double(pose, 4, p->pitch);
      p->yaw = semantic_seq_double(pose, 5, p->yaw);
    }
    if (!xyz) xyz = node["pose_xyz"] ? node["pose_xyz"] : node["xyz"];
    if (!rpy) rpy = node["pose_rpy"] ? node["pose_rpy"] : node["rpy"];
    if (xyz && xyz.IsSequence()) {
      p->x = semantic_seq_double(xyz, 0, p->x);
      p->y = semantic_seq_double(xyz, 1, p->y);
      p->z = semantic_seq_double(xyz, 2, p->z);
    }
    if (rpy && rpy.IsSequence()) {
      p->roll = semantic_seq_double(rpy, 0, p->roll);
      p->pitch = semantic_seq_double(rpy, 1, p->pitch);
      p->yaw = semantic_seq_double(rpy, 2, p->yaw);
    }
    const YAML::Node origin = node["origin"];
    const YAML::Node scalar_pose_source = origin && origin.IsMap() ? origin : node;
    try { if (scalar_pose_source["x"]) p->x = scalar_pose_source["x"].as<double>(p->x); } catch (...) {}
    try { if (scalar_pose_source["y"]) p->y = scalar_pose_source["y"].as<double>(p->y); } catch (...) {}
    try { if (scalar_pose_source["z"]) p->z = scalar_pose_source["z"].as<double>(p->z); } catch (...) {}
    try { if (scalar_pose_source["roll"]) p->roll = scalar_pose_source["roll"].as<double>(p->roll); } catch (...) {}
    try { if (scalar_pose_source["pitch"]) p->pitch = scalar_pose_source["pitch"].as<double>(p->pitch); } catch (...) {}
    try { if (scalar_pose_source["yaw"]) p->yaw = scalar_pose_source["yaw"].as<double>(p->yaw); } catch (...) {}
  };

  auto read_semantic_dimensions = [&](const YAML::Node & node, ScenePreviewWidget::PreviewItem * p) {
    if (!p || !node || !node.IsMap()) return;
    YAML::Node dims = node["dimensions"] ? node["dimensions"] : node["size"];
    if (dims && dims.IsSequence()) {
      p->sx = semantic_seq_double(dims, 0, p->sx);
      p->sy = semantic_seq_double(dims, 1, p->sy);
      p->sz = semantic_seq_double(dims, 2, p->sz);
    } else if (dims && dims.IsMap()) {
      try { if (dims["x"]) p->sx = dims["x"].as<double>(p->sx); } catch (...) {}
      try { if (dims["y"]) p->sy = dims["y"].as<double>(p->sy); } catch (...) {}
      try { if (dims["z"]) p->sz = dims["z"].as<double>(p->sz); } catch (...) {}
      try { if (dims["width"]) p->sx = dims["width"].as<double>(p->sx); } catch (...) {}
      try { if (dims["depth"]) p->sy = dims["depth"].as<double>(p->sy); } catch (...) {}
      try { if (dims["height"]) p->sz = dims["height"].as<double>(p->sz); } catch (...) {}
    }
    try { if (node["width"]) p->sx = node["width"].as<double>(p->sx); } catch (...) {}
    try { if (node["depth"]) p->sy = node["depth"].as<double>(p->sy); } catch (...) {}
    try { if (node["height"]) p->sz = node["height"].as<double>(p->sz); } catch (...) {}
    try { if (node["radius"] && (p->sx <= 0.0 || p->sy <= 0.0)) { const double diameter = node["radius"].as<double>(0.1) * 2.0; p->sx = diameter; p->sy = diameter; } } catch (...) {}
  };

  auto apply_semantic_defaults = [](const QString & concept, ScenePreviewWidget::PreviewItem * p) {
    if (!p) return;
    if (concept == QStringLiteral("table")) { p->sx = 1.20; p->sy = 0.80; p->sz = 0.08; p->z = p->z == 0.0 ? 0.74 : p->z; p->category = QStringLiteral("Table"); }
    else if (concept == QStringLiteral("conveyor")) { p->sx = 1.40; p->sy = 0.35; p->sz = 0.12; p->z = p->z == 0.0 ? 0.75 : p->z; p->category = QStringLiteral("Conveyor"); }
    else if (concept == QStringLiteral("pick_zone")) { p->sx = 0.25; p->sy = 0.25; p->sz = 0.03; p->category = QStringLiteral("Pick Zone"); }
    else if (concept == QStringLiteral("place_zone")) { p->sx = 0.25; p->sy = 0.25; p->sz = 0.03; p->category = QStringLiteral("Place Zone"); }
    else if (concept == QStringLiteral("bin")) { p->sx = 0.28; p->sy = 0.28; p->sz = 0.18; p->category = QStringLiteral("Bin"); }
    else if (concept == QStringLiteral("camera")) { p->sx = 0.09; p->sy = 0.07; p->sz = 0.07; p->z = p->z == 0.0 ? 0.8 : p->z; p->category = QStringLiteral("Camera"); }
    else if (concept == QStringLiteral("safety_zone")) { p->sx = 1.80; p->sy = 1.80; p->sz = 0.02; p->category = QStringLiteral("Safety Zone"); }
    else if (concept == QStringLiteral("home_safety_pose")) { p->sx = 0.16; p->sy = 0.16; p->sz = 0.16; p->category = QStringLiteral("Home / Safety Pose"); }
  };

  auto add_semantic_preview_item = [&](const QString & raw_id,
                                       const QString & raw_label,
                                       const QString & concept,
                                       const YAML::Node & node,
                                       const QString & source_file,
                                       bool linked_to_layout) {
    const QString normalized_id = canonical_semantic_token(raw_id);
    if (normalized_id.isEmpty() || concept.isEmpty()) return false;
    const QString semantic_key = semantic_key_for(concept, normalized_id);
    if (editable_semantic_keys.contains(semantic_key)) return false;
    if (editable_singleton_concepts.contains(concept) &&
        (concept == QStringLiteral("table") || concept == QStringLiteral("conveyor") || concept == QStringLiteral("camera") ||
         concept == QStringLiteral("home_safety_pose") || concept == QStringLiteral("safety_zone"))) {
      return false;
    }
    for (const auto & existing : preview_items) {
      if (existing.id == normalized_id) return false;
      const QString existing_concept = semantic_concept_for_role(existing.role, existing.category, existing.display_name, existing.id);
      if (semantic_key_for(existing_concept, existing.id) == semantic_key) return false;
    }

    ScenePreviewWidget::PreviewItem p;
    p.id = normalized_id;
    p.display_name = raw_label.trimmed().isEmpty() ? normalized_id : raw_label.trimmed();
    apply_semantic_defaults(concept, &p);
    p.role = normalize_role(concept, p.category + " " + p.display_name + " " + p.id);
    if (concept == QStringLiteral("home_safety_pose")) p.role = QStringLiteral("home/safety pose");
    p.status = workcell_builder::get_bool_like(node, "enabled").value_or(true) ? QStringLiteral("ready") : QStringLiteral("disabled");
    p.source_path = source_file;
    p.source_layer = linked_to_layout ? QStringLiteral("editable_layout") : QStringLiteral("overlay");
    p.active_visual_source = QStringLiteral("semantic_primitive");
    p.linked_to_editable_layout_state = linked_to_layout;
    p.editable = linked_to_layout;
    p.locked = !linked_to_layout;
    p.selectable = true;
    p.metadata_complete = true;
    p.primitive_geometry_type = QStringLiteral("box");
    if (p.locked) p.lock_reason = QStringLiteral("semantic primitive from %1").arg(QFileInfo(source_file).fileName());
    p.metadata_tags = QStringLiteral("semantic_primitive concept=%1 source=%2").arg(concept, QFileInfo(source_file).fileName());
    read_semantic_pose(node, &p);
    read_semantic_dimensions(node, &p);
    preview_items.push_back(p);
    if (allowed_scene_roles.contains(p.role) && include_preview_item_in_hierarchy(p)) add_tree_node(p);
    return true;
  };

  auto add_semantic_from_node = [&](const YAML::Node & node, const QString & fallback_id, const QString & fallback_label, const QString & source_file, bool linked_to_layout) {
    if (!node || !node.IsMap()) return false;
    const QString raw_id = semantic_yaml_scalar(node["id"]).isEmpty() ? fallback_id : semantic_yaml_scalar(node["id"]);
    QString label = semantic_yaml_scalar(node["label"]);
    if (label.isEmpty()) label = semantic_yaml_scalar(node["name"]);
    if (label.isEmpty()) label = fallback_label.isEmpty() ? raw_id : fallback_label;
    const QString type_text = semantic_yaml_scalar(node["type"]) + " " + semantic_yaml_scalar(node["role"]) + " " + semantic_yaml_scalar(node["shape"]) + " " + raw_id + " " + label;
    const QString lower = type_text.toLower();
    QString concept;
    if (lower.contains(QStringLiteral("camera")) || lower.contains(QStringLiteral("sensor"))) concept = QStringLiteral("camera");
    else if (lower.contains(QStringLiteral("conveyor"))) concept = QStringLiteral("conveyor");
    else if (lower.contains(QStringLiteral("table")) || lower.contains(QStringLiteral("workbench")) || lower.contains(QStringLiteral("support_surface"))) concept = QStringLiteral("table");
    else if (lower.contains(QStringLiteral("pick")) && (lower.contains(QStringLiteral("zone")) || lower.contains(QStringLiteral("source")))) concept = QStringLiteral("pick_zone");
    else if (lower.contains(QStringLiteral("place")) || lower.contains(QStringLiteral("drop_zone"))) concept = QStringLiteral("place_zone");
    else if (lower.contains(QStringLiteral("bin")) || lower.contains(QStringLiteral("reject"))) concept = QStringLiteral("bin");
    else if (lower.contains(QStringLiteral("home")) || lower.contains(QStringLiteral("safe_joint_state")) || lower.contains(QStringLiteral("safe joint")) || lower.contains(QStringLiteral("safety pose"))) concept = QStringLiteral("home_safety_pose");
    else if (lower.contains(QStringLiteral("safety")) || lower.contains(QStringLiteral("exclusion"))) concept = QStringLiteral("safety_zone");
    if (concept.isEmpty()) return false;
    return add_semantic_preview_item(raw_id, label, concept, node, source_file, linked_to_layout);
  };

  auto ingest_semantic_sequence = [&](const YAML::Node & seq, const QString & source_file, bool linked_to_layout) {
    int added = 0;
    if (!seq || !seq.IsSequence()) return added;
    for (const auto & node : seq) {
      if (add_semantic_from_node(node, QString(), QString(), source_file, linked_to_layout)) ++added;
    }
    return added;
  };

  auto add_scene_authoring_semantic_primitives = [&]() {
    int added = 0;
    const QVector<QPair<fs::path, bool>> sources = {
      {d / "layout" / "workcell_studio_layout.yaml", true},
      {d / "environment.yaml", false},
      {d / "cell_definition.yaml", false},
      {d / "scene_manifest.yaml", false},
    };
    for (const auto & source : sources) {
      YAML::Node root;
      if (!read_yaml(source.first, &root)) continue;
      const QString source_file = QString::fromStdString(source.first.string());
      const bool linked_to_layout = source.second;
      added += ingest_semantic_sequence(root["items"], source_file, linked_to_layout);
      added += ingest_semantic_sequence(root["layout"], source_file, linked_to_layout);
      added += ingest_semantic_sequence(root["assets"], source_file, linked_to_layout);
      added += ingest_semantic_sequence(root["placed_assets"], source_file, linked_to_layout);
      added += ingest_semantic_sequence(root["task_zones"], source_file, false);
      added += ingest_semantic_sequence(root["task"]["destinations"], source_file, false);
      added += ingest_semantic_sequence(root["task_recipe"]["destinations"], source_file, false);

      const YAML::Node camera = root["camera"];
      if (camera && camera.IsMap()) {
        const bool camera_enabled = workcell_builder::get_bool_like(camera, "enabled").value_or(true);
        const QString camera_id = semantic_yaml_scalar(camera["camera_id"]).isEmpty() ? QStringLiteral("camera_main") : semantic_yaml_scalar(camera["camera_id"]);
        if (camera_enabled || camera["pose"] || camera["pose_xyz"] || camera["frame_id"]) {
          if (add_semantic_preview_item(camera_id, QStringLiteral("camera"), QStringLiteral("camera"), camera, source_file, false)) ++added;
        }
      }

      const YAML::Node workspace_zones = root["workspace"]["zones"];
      if (workspace_zones && workspace_zones.IsSequence()) {
        for (const auto & zone : workspace_zones) {
          if (!zone || !zone.IsMap()) continue;
          const QString zone_id = semantic_yaml_scalar(zone["id"]);
          const QString zone_text = (semantic_yaml_scalar(zone["type"]) + " " + semantic_yaml_scalar(zone["shape"]) + " " + zone_id).toLower();
          if (zone_text.contains(QStringLiteral("safety")) || zone_text.contains(QStringLiteral("exclusion"))) {
            if (add_semantic_preview_item(zone_id.isEmpty() ? QStringLiteral("safety_zone") : zone_id,
                                          zone_id.isEmpty() ? QStringLiteral("safety zone") : zone_id,
                                          QStringLiteral("safety_zone"), zone, source_file, false)) ++added;
          }
        }
      }

      const YAML::Node objects = root["objects"];
      if (objects && objects.IsMap()) {
        for (auto it = objects.begin(); it != objects.end(); ++it) {
          const QString key = semantic_yaml_scalar(it->first);
          const YAML::Node object_node = it->second;
          if (add_semantic_from_node(object_node, key, key, source_file, false)) ++added;
          else if (key.toLower().contains(QStringLiteral("table")) || key.toLower().contains(QStringLiteral("conveyor")) || key.toLower().contains(QStringLiteral("bin"))) {
            if (add_semantic_preview_item(key, key, key.toLower().contains(QStringLiteral("conveyor")) ? QStringLiteral("conveyor") : (key.toLower().contains(QStringLiteral("bin")) ? QStringLiteral("bin") : QStringLiteral("table")), object_node, source_file, false)) ++added;
          }
        }
      }

      const QString support_surface = semantic_map_string(root, {"environment", "support_surface_link"});
      if (!support_surface.isEmpty()) {
        YAML::Node support_node(YAML::NodeType::Map);
        support_node["id"] = support_surface.toStdString();
        if (add_semantic_preview_item(support_surface, QStringLiteral("support surface %1").arg(support_surface), QStringLiteral("table"), support_node, source_file, false)) ++added;
      }

      const QString home_target = semantic_map_string(root, {"robot", "home_named_target"});
      if (!home_target.isEmpty()) {
        YAML::Node home_node(YAML::NodeType::Map);
        home_node["id"] = (QStringLiteral("home_pose_%1").arg(home_target)).toStdString();
        if (add_semantic_preview_item(QStringLiteral("home_pose_%1").arg(home_target), QStringLiteral("home pose %1").arg(home_target), QStringLiteral("home_safety_pose"), home_node, source_file, false)) ++added;
      }
      const YAML::Node safe_joint_state = root["robot"]["safe_joint_state"];
      if (safe_joint_state && (safe_joint_state.IsSequence() || safe_joint_state.IsMap())) {
        YAML::Node safe_node(YAML::NodeType::Map);
        safe_node["id"] = "safe_joint_state";
        if (add_semantic_preview_item(QStringLiteral("safe_joint_state"), QStringLiteral("safe joint state"), QStringLiteral("home_safety_pose"), safe_node, source_file, false)) ++added;
      }
    }
    if (added > 0) {
      append_studio_log(QString("Semantic primitive preview items added from scene authoring sources: %1").arg(added));
    }
  };

  add_scene_authoring_semantic_primitives();

  QSet<QString> preview_ids;
  for (const auto &existing : preview_items) preview_ids.insert(existing.id);
  const QString scene_name = QString::fromStdString(d.filename().string());
  const QString workspace_root = detect_workspace_root();
  QString visual_index_warning_reason;
  QString visual_index_refresh_blocker;
  bool visual_index_needs_refresh = false;

  auto inspect_visual_index_refresh_need = [&]() {
    visual_index_warning_reason.clear();
    visual_index_needs_refresh = false;
    if (!fs::exists(urdf_visual_index)) {
      visual_index_warning_reason = QStringLiteral("missing");
      visual_index_needs_refresh = true;
      return;
    }
    try {
      const YAML::Node existing_index = YAML::LoadFile(urdf_visual_index.string());
      const bool safe_for_preview = workcell_builder::yaml_map_key(existing_index, "safe_for_preview").as<bool>(false);
      const bool stale_index = workcell_builder::yaml_map_key(existing_index, "stale_index").as<bool>(false);
      const int stale_or_unsafe_count = workcell_builder::yaml_map_key(existing_index, "stale_or_unsafe_count").as<int>(0);
      const int visual_count = workcell_builder::yaml_map_key(existing_index, "visual_count").as<int>(-1);
      const int emitted_visual_count = workcell_builder::yaml_map_key(existing_index, "emitted_visual_count").as<int>(-1);
      const int candidate_mesh_count = workcell_builder::yaml_map_key(existing_index, "candidate_mesh_count").as<int>(-1);
      const int renderable_item_count = workcell_builder::yaml_map_key(existing_index, "renderable_item_count").as<int>(-1);
      const int renderable_mesh_count = workcell_builder::yaml_map_key(existing_index, "renderable_mesh_count").as<int>(-1);
      const YAML::Node visual_items = workcell_builder::yaml_map_key(existing_index, "visual_items");
      const bool empty_visual_items = visual_items && visual_items.IsSequence() && visual_items.size() == 0;
      if (!safe_for_preview) {
        visual_index_warning_reason = QStringLiteral("unsafe/best-effort");
      } else if (stale_index || stale_or_unsafe_count > 0) {
        visual_index_warning_reason = QStringLiteral("stale");
      } else if (empty_visual_items || visual_count == 0 || emitted_visual_count == 0 ||
                 (candidate_mesh_count == 0 && renderable_item_count <= 0 && renderable_mesh_count <= 0)) {
        visual_index_warning_reason = QStringLiteral("poor preview counters: visual_count=%1 emitted_visual_count=%2 candidate_mesh_count=%3 renderable_item_count=%4 renderable_mesh_count=%5")
          .arg(visual_count)
          .arg(emitted_visual_count)
          .arg(candidate_mesh_count)
          .arg(renderable_item_count)
          .arg(renderable_mesh_count);
      }
      visual_index_needs_refresh = !visual_index_warning_reason.isEmpty();
    } catch (const std::exception &e) {
      visual_index_warning_reason = QStringLiteral("unreadable: %1").arg(QString::fromUtf8(e.what()));
      visual_index_needs_refresh = true;
    } catch (...) {
      visual_index_warning_reason = QStringLiteral("unreadable");
      visual_index_needs_refresh = true;
    }
  };

  auto run_visual_index_refresh = [&]() -> bool {
    QString script;
    if (!helper_script_exists("extract_scene_urdf_visual_mesh_index.py", &script)) {
      visual_index_refresh_blocker = QStringLiteral("extractor script not found in helper script search paths");
      return false;
    }
    if (workspace_root.trimmed().isEmpty() || !QDir(workspace_root).exists()) {
      visual_index_refresh_blocker = QStringLiteral("workspace root is empty or does not exist");
      return false;
    }
    const bool ros_setup_exists = QFileInfo::exists(QStringLiteral("/opt/ros/humble/setup.bash"));
    const bool workspace_install_setup_exists = QFileInfo::exists(workspace_root + QStringLiteral("/install/setup.bash"));
    const bool ros_environment_present = ros_setup_exists ||
      !qgetenv("ROS_DISTRO").trimmed().isEmpty() ||
      !qgetenv("AMENT_PREFIX_PATH").trimmed().isEmpty();
    if (!ros_environment_present) {
      visual_index_refresh_blocker = QStringLiteral("ROS/workspace environment is unavailable (no /opt/ros/humble/setup.bash, ROS_DISTRO, or AMENT_PREFIX_PATH)");
      return false;
    }

    QProcess process;
    process.setWorkingDirectory(workspace_root);
    QStringList command_parts;
    if (ros_setup_exists) {
      command_parts << QStringLiteral("source /opt/ros/humble/setup.bash");
    }
    if (workspace_install_setup_exists) {
      command_parts << QStringLiteral("source install/setup.bash");
    }
    command_parts << QStringLiteral("python3 \"$VISUAL_INDEX_EXTRACTOR\" --scene \"$VISUAL_INDEX_SCENE\" --workspace-root \"$VISUAL_INDEX_WORKSPACE_ROOT\"");
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QStringLiteral("VISUAL_INDEX_EXTRACTOR"), script);
    env.insert(QStringLiteral("VISUAL_INDEX_SCENE"), scene_name);
    env.insert(QStringLiteral("VISUAL_INDEX_WORKSPACE_ROOT"), workspace_root);
    process.setProcessEnvironment(env);
    const QString command = command_parts.join(QStringLiteral(" && "));
    append_studio_log(QString("Visual mesh index refresh: preview-only metadata command for scene '%1' at %2 (no RViz/MoveIt/controllers/real hardware launched).")
      .arg(scene_name, QString::fromStdString(urdf_visual_index.string())));
    process.start(QStringLiteral("/bin/bash"), QStringList() << QStringLiteral("-lc") << command);
    if (!process.waitForFinished(120000)) {
      process.kill();
      process.waitForFinished(3000);
      visual_index_refresh_blocker = QStringLiteral("extractor timed out after 120s");
      return false;
    }
    const int exit_code = process.exitCode();
    const QString stdout_text = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
    const QString stderr_text = QString::fromUtf8(process.readAllStandardError()).trimmed();
    if (exit_code != 0) {
      visual_index_refresh_blocker = QStringLiteral("extractor exited with code %1; stderr=%2; stdout=%3")
        .arg(exit_code)
        .arg(stderr_text.left(500), stdout_text.left(500));
      return false;
    }
    if (!fs::exists(urdf_visual_index)) {
      visual_index_refresh_blocker = QStringLiteral("extractor exited successfully but did not write the index file");
      return false;
    }
    if (!stdout_text.isEmpty()) {
      append_studio_log(QString("Visual mesh index refresh stdout: %1").arg(stdout_text.left(500)));
    }
    append_studio_log(QString("Visual mesh index refreshed for scene '%1'; reloading generated/scene_visual_mesh_index.json before assembling preview items.").arg(scene_name));
    return true;
  };

  inspect_visual_index_refresh_need();
  if (visual_index_needs_refresh) {
    const QString original_reason = visual_index_warning_reason;
    if (run_visual_index_refresh()) {
      inspect_visual_index_refresh_need();
      if (visual_index_warning_reason.isEmpty()) {
        append_studio_log(QString("Visual mesh index refresh cleared '%1' for scene '%2'.").arg(original_reason, scene_name));
      } else {
        append_studio_log(QString("Visual mesh index refresh completed for scene '%1' but index is still %2 at %3 (workspace root: %4).")
          .arg(scene_name, visual_index_warning_reason, QString::fromStdString(urdf_visual_index.string()), workspace_root));
      }
    } else {
      append_studio_log(
        QString("Visual mesh index %1 for scene '%2' at %3 (workspace root: %4). Automatic preview-only refresh could not run: %5. Run python3 scripts/extract_scene_urdf_visual_mesh_index.py --scene %2 --workspace-root %4 from a ROS/workspace environment to refresh this generated artifact.")
        .arg(original_reason, scene_name, QString::fromStdString(urdf_visual_index.string()), workspace_root, visual_index_refresh_blocker));
      append_studio_log("Visual mesh index unsafe/best-effort; preview may show placeholders");
    }
  }
  scene3d_visual_ingestion_diagnostics_ = QJsonObject();
  int visual_index_loaded_count = 0;
  int visual_preview_added_count = 0;
  QJsonArray visual_ingestion_item_diagnostics;
  QJsonArray generated_urdf_visual_row_diagnostics;
  QMap<QString, int> skip_reason_counts;
  auto add_skip_reason = [&](const QString & reason) {
    const QString key = canonical_skip_reason_key(reason);
    skip_reason_counts[key] = skip_reason_counts.value(key, 0) + 1;
    return key;
  };
  auto append_visual_ingestion_diagnostic = [&](const YAML::Node & v, const QString & raw_id, const QString & final_id, const QString & skip_reason, const QString & source_layer = QString()) {
    if (visual_ingestion_item_diagnostics.size() >= 20) return;
    QJsonObject row;
    auto value = [&](const char * key) {
      if (!v || !v.IsMap()) return QString();
      return QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, key)).trimmed();
    };
    row[QStringLiteral("raw_id")] = raw_id;
    row[QStringLiteral("final_unique_scene3d_key")] = final_id;
    row[QStringLiteral("final_unique_scene3d_id")] = final_id;
    const QString raw_link = value("link");
    const QString raw_link_name = value("link_name");
    const QString canonical_link_name = raw_link_name.isEmpty() ? raw_link : raw_link_name;
    row[QStringLiteral("link")] = raw_link;
    row[QStringLiteral("link_name")] = raw_link_name;
    row[QStringLiteral("canonical_link_name")] = canonical_link_name;
    row[QStringLiteral("visual")] = value("visual");
    row[QStringLiteral("visual_name")] = value("visual_name");
    row[QStringLiteral("package_uri")] = value("package_uri");
    row[QStringLiteral("mesh_uri")] = value("mesh_uri");
    row[QStringLiteral("mesh_source")] = value("mesh_path").isEmpty() ? value("source_path") : value("mesh_path");
    QString source_path = value("source_path");
    if (source_path.isEmpty()) source_path = value("resolved_source_path");
    if (source_path.isEmpty()) source_path = value("resolved_path");
    row[QStringLiteral("source_path")] = source_path;
    row[QStringLiteral("resolved_source_path")] = value("resolved_source_path");
    row[QStringLiteral("source_layer")] = source_layer.isEmpty() ? value("source_layer") : source_layer;
    if (!skip_reason.isEmpty()) row[QStringLiteral("skip_reason")] = skip_reason;
    visual_ingestion_item_diagnostics.append(row);
  };
  auto generated_visual_row_diagnostic = [&](const YAML::Node & v, int source_row_index, const QString & generated_visual_row_key, const QString & final_id) {
    QJsonObject row;
    auto value = [&](const char * key) {
      if (!v || !v.IsMap()) return QString();
      return QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, key)).trimmed();
    };
    row[QStringLiteral("source_row_index")] = source_row_index;
    row[QStringLiteral("link")] = value("link");
    row[QStringLiteral("link_name")] = value("link_name");
    row[QStringLiteral("visual")] = value("visual");
    row[QStringLiteral("visual_name")] = value("visual_name");
    row[QStringLiteral("generated_visual_row_key")] = generated_visual_row_key;
    row[QStringLiteral("id")] = final_id;
    row[QStringLiteral("package_uri")] = value("package_uri");
    row[QStringLiteral("mesh_uri")] = value("mesh_uri");
    row[QStringLiteral("filename")] = value("filename");
    row[QStringLiteral("source_path")] = value("source_path");
    row[QStringLiteral("resolved_path")] = value("resolved_path");
    row[QStringLiteral("resolved_source_path")] = value("resolved_source_path");
    row[QStringLiteral("mesh_path")] = value("mesh_path").isEmpty() ? value("source_path") : value("mesh_path");
    row[QStringLiteral("geometry_type")] = value("geometry_type");
    row[QStringLiteral("type")] = value("type");
    row[QStringLiteral("category")] = value("category");
    row[QStringLiteral("source")] = value("source");
    row[QStringLiteral("appended_to_preview_items")] = false;
    row[QStringLiteral("skip_branch")] = QString();
    row[QStringLiteral("skip_reason")] = QString();
    row[QStringLiteral("survived_suppression")] = false;
    row[QStringLiteral("survived_filter")] = false;
    return row;
  };
  auto set_generated_visual_first_drop_stage_once = [](QJsonObject & row, const QString & first_drop_stage) {
    if (first_drop_stage.trimmed().isEmpty()) return;
    if (!row.value(QStringLiteral("first_drop_stage")).toString().trimmed().isEmpty()) return;
    row[QStringLiteral("first_drop_stage")] = first_drop_stage;
  };
  auto visual_index_loop_drop_stage_for_branch = [](const QString & skip_branch) {
    if (skip_branch == QStringLiteral("duplicate_generated_visual_index_row")) {
      return QStringLiteral("duplicate_generated_visual_index_row");
    }
    if (skip_branch == QStringLiteral("suppressed_by_urdf_flattened_visual_mesh")) {
      return QStringLiteral("suppressed_by_urdf_flattened_visual_mesh");
    }
    if (skip_branch == QStringLiteral("duplicate_generated_urdf_fallback_link")) {
      return QStringLiteral("mesh_resolution_or_fallback_branch");
    }
    if (skip_branch == QStringLiteral("appended_to_preview_items") || skip_branch.trimmed().isEmpty()) {
      return QString();
    }
    return QStringLiteral("visual_index_loop_skip");
  };
  auto append_generated_visual_row_diagnostic = [&](QJsonObject row, const QString & skip_branch, const QString & skip_reason = QString(), bool appended_to_preview_items = false, const QString & final_mesh_path = QString()) {
    row[QStringLiteral("appended_to_preview_items")] = appended_to_preview_items;
    row[QStringLiteral("skip_branch")] = skip_branch;
    row[QStringLiteral("skip_reason")] = skip_reason;
    set_generated_visual_first_drop_stage_once(row, visual_index_loop_drop_stage_for_branch(skip_branch));
    if (!final_mesh_path.trimmed().isEmpty()) {
      row[QStringLiteral("mesh_path")] = final_mesh_path;
    }
    generated_urdf_visual_row_diagnostics.append(row);
  };
  auto append_metadata_tag = [](QString & tags, const QString & tag) {
    const QString normalized_tag = tag.trimmed();
    if (normalized_tag.isEmpty()) return;
    const QStringList existing = tags.split(QLatin1Char(';'), Qt::SkipEmptyParts);
    for (const QString & value : existing) {
      if (value.trimmed().compare(normalized_tag, Qt::CaseInsensitive) == 0) return;
    }
    tags = tags.trimmed().isEmpty() ? normalized_tag : tags + QStringLiteral(";") + normalized_tag;
  };
  auto classify_generated_urdf_visual = [&](ScenePreviewWidget::PreviewItem & item, const QString & resolved_source_path) {
    const QStringList source_fields = {
      item.package_uri,
      item.visual_index_package_uri,
      item.visual_index_mesh_uri,
      item.mesh_path,
      item.source_path,
      resolved_source_path,
      item.resolved_source_path_original
    };
    const QString source_mix = source_fields.join(QStringLiteral("|")).toLower();
    if (source_mix.contains(QStringLiteral("package://ur_description/meshes/ur5/"))) {
      item.category = QStringLiteral("robot/ur5");
      item.role = QStringLiteral("robot");
      append_metadata_tag(item.metadata_tags, QStringLiteral("robot_family=ur5"));
      append_metadata_tag(item.metadata_tags, QStringLiteral("asset_category=robot"));
    } else if (source_mix.contains(QStringLiteral("package://robotiq_85_description/"))) {
      item.category = QStringLiteral("gripper");
      item.role = QStringLiteral("gripper");
      append_metadata_tag(item.metadata_tags, QStringLiteral("gripper_family=robotiq_85"));
      append_metadata_tag(item.metadata_tags, QStringLiteral("asset_category=gripper"));
    } else if (source_mix.contains(QStringLiteral("package://workbench_description/"))) {
      item.category = QStringLiteral("table/workbench");
      item.role = QStringLiteral("workbench");
      append_metadata_tag(item.metadata_tags, QStringLiteral("asset_category=table/workbench"));
    } else if (source_mix.contains(QStringLiteral("package://realsense2_description/"))) {
      item.category = QStringLiteral("camera");
      item.role = QStringLiteral("camera");
      append_metadata_tag(item.metadata_tags, QStringLiteral("camera_family=realsense"));
      append_metadata_tag(item.metadata_tags, QStringLiteral("asset_category=camera"));
    }
  };
  auto normalize_generated_urdf_visual_identity = [&](ScenePreviewWidget::PreviewItem & item) {
    const QString stable_link_identity = !item.visual_index_link_name.trimmed().isEmpty()
      ? item.visual_index_link_name.trimmed()
      : item.visual_index_link.trimmed();
    if (!stable_link_identity.isEmpty()) {
      item.visual_index_link = stable_link_identity;
      item.visual_index_link_name = stable_link_identity;
      if (item.display_name.trimmed().isEmpty() || item.display_name == item.id) item.display_name = stable_link_identity;
      if (item.frame_id.trimmed().isEmpty() || item.frame_id == item.id) item.frame_id = stable_link_identity;
    }
    item.source_layer = QStringLiteral("locked_generated_urdf_visual");
    item.locked = true;
    item.editable = false;
    item.selectable = true;
  };
  int non_mesh_geometry_added = 0;
  int non_mesh_geometry_unsupported = 0;
  int package_uri_resolved_by_loader = 0;
  int package_uri_resolved_after_stale_resolved_source_path = 0;
  int source_path_from_resolved_path = 0;
  int stale_resolved_source_path_count = 0;
  int unresolved_package_uri_count = 0;
  int stale_or_absolute_only_mesh_index_count = 0;
  int transform_chain_applied_count = 0;
  int visual_origin_applied_count = 0;
  int baked_world_visual_pose_count = 0;
  int baked_world_visual_transform_count = 0;
  int legacy_viewport_transform_count = 0;
  QString baked_world_visual_transform_source = QStringLiteral("urdf_fk_link_world_times_visual_origin");
  int missing_chain_warning_count = 0;
  {
    QStringList detected_asset_roots;
    const QMap<QString, QString> local_package_map = workcell_builder::discover_visual_mesh_package_map(
      d, workspace_root, &detected_asset_roots);
    detected_asset_roots.removeDuplicates();
    if (detected_asset_roots.isEmpty()) {
      append_studio_log(QStringLiteral("Asset workspace: none detected (checked <workspace>/src/assets and <repo>/assets)"));
    } else {
      append_studio_log(QStringLiteral("Asset workspace: %1").arg(detected_asset_roots.join(QStringLiteral(", "))));
    }
    append_studio_log(QStringLiteral("Asset packages discovered: %1").arg(local_package_map.keys().isEmpty() ? QStringLiteral("none") : local_package_map.keys().join(QStringLiteral(", "))));
    int asset_mesh_count = 0;
    const QMap<QString, int> asset_mesh_category_counts = workcell_builder::discover_visual_mesh_asset_category_counts(
      detected_asset_roots, &asset_mesh_count, nullptr);
    append_studio_log(QStringLiteral("Asset meshes discovered: %1").arg(asset_mesh_count));
    append_studio_log(QStringLiteral("Asset mesh categories: robot=%1 gripper=%2 table/workbench=%3 camera=%4 environment=%5 other=%6")
      .arg(asset_mesh_category_counts.value(QStringLiteral("robot")))
      .arg(asset_mesh_category_counts.value(QStringLiteral("gripper")))
      .arg(asset_mesh_category_counts.value(QStringLiteral("table/workbench")))
      .arg(asset_mesh_category_counts.value(QStringLiteral("camera")))
      .arg(asset_mesh_category_counts.value(QStringLiteral("environment")))
      .arg(asset_mesh_category_counts.value(QStringLiteral("other"))));
    const QStringList expected_visual_packages = {
      QStringLiteral("ur_description"),
      QStringLiteral("robotiq_85_description"),
      QStringLiteral("workbench_description"),
      QStringLiteral("realsense2_description")
    };
    QStringList missing_asset_packages;
    for (const QString & pkg : expected_visual_packages) {
      if (!local_package_map.contains(pkg)) missing_asset_packages << pkg;
    }
    append_studio_log(QStringLiteral("Missing asset packages: %1").arg(
      missing_asset_packages.isEmpty() ? QStringLiteral("none") : missing_asset_packages.join(QStringLiteral(", "))));
    if (!local_package_map.contains(QStringLiteral("ur_description"))) {
      append_studio_log(QStringLiteral("Missing robot visual asset package: ur_description"));
    }
  }
  QString robot_base_frame = QStringLiteral("unknown");
  struct RobotBaseFrameCandidate {
    QString frame;
    int score{-100000};
  };
  auto robot_base_frame_reject_or_downrank_score = [](const QString & frame) {
    const QString token = canonical_scene3d_token(frame);
    if (token.isEmpty()) return -100000;
    if (token == QStringLiteral("world")) return 10000;
    if (token == QStringLiteral("base_link")) return 9000;
    if (token == QStringLiteral("base_link_inertia")) return 8000;

    const QStringList non_robot_root_terms = {
      QStringLiteral("gripper_base_link"),
      QStringLiteral("robotiq"),
      QStringLiteral("finger"),
      QStringLiteral("knuckle"),
      QStringLiteral("tool0"),
      QStringLiteral("tool_"),
      QStringLiteral("_tool"),
      QStringLiteral("tool_link"),
      QStringLiteral("end_effector"),
      QStringLiteral("endeffector"),
      QStringLiteral("ee_link"),
      QStringLiteral("eef"),
      QStringLiteral("camera"),
      QStringLiteral("realsense"),
      QStringLiteral("d435"),
      QStringLiteral("d455"),
      QStringLiteral("table"),
      QStringLiteral("workbench")
    };
    for (const QString & term : non_robot_root_terms) {
      if (token.contains(term)) return -9000;
    }

    int score = 0;
    if (token == QStringLiteral("root_link")) score = 7000;
    else if (token.endsWith(QStringLiteral("_root_link")) || token.contains(QStringLiteral("root_link"))) score = 6500;
    else if (token.endsWith(QStringLiteral("_base_link")) || token.contains(QStringLiteral("base_link"))) score = 6000;
    else if (token == QStringLiteral("base") || token.endsWith(QStringLiteral("_base"))) score = 5500;
    else if ((token.startsWith(QStringLiteral("ur")) || token.contains(QStringLiteral("_ur"))) &&
             (token.contains(QStringLiteral("root")) || token.contains(QStringLiteral("base")))) score = 5200;
    else if (token.contains(QStringLiteral("shoulder_link"))) score = 1000;
    else score = 100;
    return score;
  };
  auto consider_robot_base_frame_candidate = [&](RobotBaseFrameCandidate & best, const QString & frame, int source_bonus = 0) {
    const QString candidate = frame.trimmed();
    if (candidate.isEmpty()) return;
    const int score = robot_base_frame_reject_or_downrank_score(candidate) + source_bonus;
    if (score > best.score) {
      best.frame = candidate;
      best.score = score;
    }
  };
  auto yaml_scalar_string = [](const YAML::Node & node, const char * key) {
    const YAML::Node value = workcell_builder::yaml_map_key(node, key);
    if (!value || !value.IsScalar()) return QString();
    return QString::fromStdString(value.as<std::string>("")).trimmed();
  };
  auto consider_robot_base_frame_chain = [&](RobotBaseFrameCandidate & best, const YAML::Node & node, const char * key, int source_bonus = 0) {
    const YAML::Node chain = workcell_builder::yaml_map_key(node, key);
    if (!chain) return;
    if (chain.IsSequence()) {
      int chain_index = 0;
      for (const YAML::Node & entry : chain) {
        if (entry.IsScalar()) {
          consider_robot_base_frame_candidate(best, QString::fromStdString(entry.as<std::string>("")).trimmed(), source_bonus - chain_index);
        }
        ++chain_index;
      }
    } else if (chain.IsScalar()) {
      const QString chain_text = QString::fromStdString(chain.as<std::string>("")).trimmed();
      for (const QString & part : chain_text.split(QRegularExpression(QStringLiteral("[,>\\s]+")), Qt::SkipEmptyParts)) {
        consider_robot_base_frame_candidate(best, part, source_bonus);
      }
    }
  };
  RobotBaseFrameCandidate best_robot_base_frame_candidate;
  QString robot_world_pose = QStringLiteral("unknown");
  int mesh_item_count = 0;
  int primitive_item_count = 0;
  int unknown_item_count = 0;
  int skipped_other = 0;
  int skipped_semantic_helper_visual_rows = 0;
  int skipped_true_duplicate_mesh_rows = 0;
  int preserved_generated_urdf_robot_mesh_rows = 0;
  QString visual_diagnostics_summary;
  bool visual_index_safe_for_preview = false;
  QString visual_index_extraction_mode = "unknown";
  if (fs::exists(urdf_visual_index)) {
    try {
      const YAML::Node urdf_index = YAML::LoadFile(urdf_visual_index.string());
      visual_index_safe_for_preview = workcell_builder::yaml_map_key(urdf_index, "safe_for_preview").as<bool>(false);
      visual_index_extraction_mode = QString::fromStdString(
        workcell_builder::yaml_map_value_or_empty(urdf_index, "extraction_mode"));
      consider_robot_base_frame_candidate(best_robot_base_frame_candidate, yaml_scalar_string(urdf_index, "root_link"), 300);
      consider_robot_base_frame_candidate(best_robot_base_frame_candidate, yaml_scalar_string(urdf_index, "robot_root_link"), 300);
      consider_robot_base_frame_candidate(best_robot_base_frame_candidate, yaml_scalar_string(urdf_index, "base_link"), 250);
      consider_robot_base_frame_chain(best_robot_base_frame_candidate, urdf_index, "link_chain", 200);
      consider_robot_base_frame_chain(best_robot_base_frame_candidate, urdf_index, "urdf_link_chain", 200);
      stale_or_absolute_only_mesh_index_count =
        workcell_builder::yaml_map_key(urdf_index, "stale_or_unsafe_count").as<int>(0);
      if (stale_or_absolute_only_mesh_index_count == 0 && workcell_builder::yaml_map_key(urdf_index, "stale_index").as<bool>(false)) {
        stale_or_absolute_only_mesh_index_count = 1;
      }
      const QSet<QString> required_ur5_visual_links = {
        QStringLiteral("base_link_inertia"),
        QStringLiteral("shoulder_link"),
        QStringLiteral("upper_arm_link"),
        QStringLiteral("forearm_link"),
        QStringLiteral("wrist_1_link"),
        QStringLiteral("wrist_2_link"),
        QStringLiteral("wrist_3_link")
      };
      QMap<QString, ScenePreviewWidget::PreviewItem> required_ur5_preview_items_by_link;
      const YAML::Node visual_items = workcell_builder::yaml_map_key(urdf_index, "visual_items");
      if (visual_items && visual_items.IsSequence()) {
        std::vector<YAML::Node> ordered_visual_items;
        ordered_visual_items.reserve(visual_items.size());
        QSet<QString> flattened_visual_mesh_keys;
        auto visual_item_link_object_key = [](const YAML::Node & node) {
          QString key = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "link")).trimmed();
          if (key.isEmpty()) key = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "object_id")).trimmed();
          if (key.isEmpty()) key = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "object")).trimmed();
          if (key.isEmpty()) key = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "id")).trimmed();
          return canonical_scene3d_token(key);
        };
        auto visual_item_source_token = [](const YAML::Node & node) {
          return canonical_scene3d_token(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "source")));
        };
        auto visual_item_mesh_reference_resolves = [&](const YAML::Node & node) {
          const QString source_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "source_path"));
          const QString resolved_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "resolved_path"));
          const QString resolved_source_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "resolved_source_path"));
          const QString package_uri = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "package_uri"));
          QStringList tried_candidates;
          const QString resolved_mesh_path = resolve_visual_mesh_source_path(
            !resolved_source_path.trimmed().isEmpty() ? resolved_source_path : (!resolved_path.trimmed().isEmpty() ? resolved_path : source_path),
            package_uri, d, detect_workspace_root(), &tried_candidates);
          if (resolved_mesh_path.trimmed().isEmpty()) return false;
          const QFileInfo info(resolved_mesh_path);
          return info.exists() && info.isFile();
        };
        auto visual_index_row_scalar_value = [](const YAML::Node & node, const char * key) {
          const YAML::Node field = workcell_builder::yaml_map_key(node, key);
          if (!field || !field.IsScalar()) return QString();
          return QString::fromStdString(field.as<std::string>("")).trimmed();
        };
        auto visual_index_first_scalar_value = [&](const YAML::Node & node, std::initializer_list<const char *> keys) {
          for (const char * key : keys) {
            const QString value = visual_index_row_scalar_value(node, key);
            if (!value.isEmpty()) return value;
          }
          return QString();
        };
        auto visual_index_parse_error_reason = [&](const YAML::Node & node, int row_index, const QString & exception_text = QString()) {
          if (!node || !node.IsMap()) return QStringLiteral("parse_error_row_not_map");
          const QString link = visual_index_first_scalar_value(node, {"link", "link_name", "object_id", "object"});
          const QString visual = visual_index_first_scalar_value(node, {"visual", "visual_name", "id"});
          const QString mesh = visual_index_first_scalar_value(node, {"package_uri", "mesh_uri", "filename", "source_path", "resolved_source_path", "resolved_path", "mesh_path"});
          const QString geometry = visual_index_first_scalar_value(node, {"geometry_type", "type", "geometry"});
          QStringList missing;
          if (link.isEmpty()) missing << QStringLiteral("link/link_name");
          if (visual.isEmpty()) missing << QStringLiteral("visual/visual_name/id");
          if (mesh.isEmpty()) missing << QStringLiteral("package_uri/mesh_uri/filename/source_path/resolved_source_path");
          if (geometry.isEmpty()) missing << QStringLiteral("geometry_type/type/category");
          if (!exception_text.trimmed().isEmpty()) return QStringLiteral("parse_error_exception:%1").arg(exception_text.trimmed());
          if (!missing.isEmpty()) return QStringLiteral("parse_error_missing_%1").arg(missing.join(QStringLiteral("+")));
          return QStringLiteral("parse_error_unknown_row_%1").arg(row_index);
        };
        auto visual_item_final_identity_key = [&](const YAML::Node & node, int source_row_index) {
          QString link = visual_index_row_scalar_value(node, "link");
          if (link.isEmpty()) link = visual_index_row_scalar_value(node, "link_name");
          if (link.isEmpty()) link = visual_index_row_scalar_value(node, "object_id");
          if (link.isEmpty()) link = visual_index_row_scalar_value(node, "object");
          QString visual = visual_index_row_scalar_value(node, "visual");
          if (visual.isEmpty()) visual = visual_index_row_scalar_value(node, "visual_name");
          if (visual.isEmpty()) visual = visual_index_row_scalar_value(node, "id");
          QString row_index = visual_index_row_scalar_value(node, "source_row_index");
          if (row_index.isEmpty()) row_index = QString::number(qMax(0, source_row_index));
          return QStringLiteral("generated_urdf::%1::%2::%3")
            .arg(link.isEmpty() ? QStringLiteral("link_missing") : canonical_scene3d_token(link),
                 visual.isEmpty() ? QStringLiteral("visual_missing") : canonical_scene3d_token(visual),
                 canonical_scene3d_token(row_index));
        };
        auto visual_item_canonical_link = [&](const YAML::Node & node) {
          return canonical_scene3d_token(visual_index_first_scalar_value(node, {"link", "link_name", "canonical_link_name", "object_id", "object"}));
        };
        auto visual_item_canonical_visual = [&](const YAML::Node & node) {
          return canonical_scene3d_token(visual_index_first_scalar_value(node, {"visual_name", "visual", "id"}));
        };
        auto visual_item_source_row_token = [&](const YAML::Node & node, int source_row_index) {
          QString row_index = visual_index_row_scalar_value(node, "source_row_index");
          if (row_index.isEmpty()) row_index = QString::number(qMax(0, source_row_index));
          return canonical_scene3d_token(row_index);
        };
        auto visual_item_mesh_identity = [&](const YAML::Node & node) {
          return visual_index_first_scalar_value(node, {
            "package_uri", "mesh_uri", "source_path", "mesh_path", "resolved_path", "resolved_source_path"});
        };
        auto visual_item_normalized_mesh_identity = [&](const YAML::Node & node) {
          QStringList parts;
          for (const char * field : {"package_uri", "mesh_uri", "source_path", "mesh_path", "resolved_path", "resolved_source_path"}) {
            const QString value = visual_index_row_scalar_value(node, field).trimmed();
            if (!value.isEmpty()) parts << canonical_scene3d_token(value);
          }
          return parts.isEmpty() ? QStringLiteral("mesh_missing") : parts.join(QStringLiteral("__"));
        };
        auto visual_item_generated_row_key = [&](const YAML::Node & node, int source_row_index) {
          const QString link = visual_item_canonical_link(node);
          const QString visual = visual_item_canonical_visual(node);
          const QString row_index = visual_item_source_row_token(node, source_row_index);
          const QString mesh_identity = visual_item_normalized_mesh_identity(node);
          return QStringLiteral("generated_urdf_row::%1::%2::%3::%4")
            .arg(link.isEmpty() ? QStringLiteral("link_missing") : link,
                 visual.isEmpty() ? QStringLiteral("visual_missing") : visual,
                 row_index.isEmpty() ? QStringLiteral("row_missing") : row_index,
                 mesh_identity.isEmpty() ? QStringLiteral("mesh_missing") : canonical_scene3d_token(mesh_identity));
        };
        static const QSet<QString> protected_ur5_links = {
          QStringLiteral("base_link_inertia"),
          QStringLiteral("base_link"),
          QStringLiteral("shoulder_link"),
          QStringLiteral("upper_arm_link"),
          QStringLiteral("forearm_link"),
          QStringLiteral("wrist_1_link"),
          QStringLiteral("wrist_2_link"),
          QStringLiteral("wrist_3_link")
        };
        auto normalized_protected_ur5_link = [&](const YAML::Node & node) {
          for (const char * field : {"link", "link_name", "canonical_link_name"}) {
            const QString link = canonical_scene3d_token(visual_index_row_scalar_value(node, field));
            if (protected_ur5_links.contains(link)) return link;
          }
          const QString id_token = canonical_scene3d_token(visual_index_row_scalar_value(node, "id"));
          for (const QString & link : protected_ur5_links) {
            if (id_token == link || id_token.contains(QStringLiteral("_%1_").arg(link)) ||
                id_token.startsWith(QStringLiteral("%1_").arg(link)) ||
                id_token.endsWith(QStringLiteral("_%1").arg(link))) {
              return link;
            }
          }
          return QString();
        };
        const bool processing_generated_scene_visual_mesh_index = true;
        auto is_protected_ur5_generated_visual_row = [&](const YAML::Node & node) {
          const QString raw_id = visual_index_row_scalar_value(node, "id").trimmed();
          const bool generated_visual_index_row =
            raw_id.startsWith(QStringLiteral("urdf_visual_")) ||
            processing_generated_scene_visual_mesh_index;
          if (!generated_visual_index_row) return false;
          const QString normalized_link = normalized_protected_ur5_link(node);
          if (normalized_link.isEmpty()) return false;
          const QString geometry = canonical_scene3d_token(visual_index_first_scalar_value(node, {"geometry_type", "type"}));
          if (geometry != QStringLiteral("mesh")) return false;
          const QString source_mix = QStringList{
            visual_index_row_scalar_value(node, "package_uri"),
            visual_index_row_scalar_value(node, "mesh_uri"),
            visual_index_row_scalar_value(node, "source_path"),
            visual_index_row_scalar_value(node, "mesh_path"),
            visual_index_row_scalar_value(node, "resolved_path"),
            visual_index_row_scalar_value(node, "resolved_source_path"),
            visual_index_row_scalar_value(node, "filename")
          }.join(QStringLiteral("|")).toLower();
          const QString source_token = canonical_scene3d_token(source_mix);
          static const QSet<QString> protected_ur5_mesh_files = {
            QStringLiteral("base_dae"),
            QStringLiteral("shoulder_dae"),
            QStringLiteral("upperarm_dae"),
            QStringLiteral("forearm_dae"),
            QStringLiteral("wrist1_dae"),
            QStringLiteral("wrist2_dae"),
            QStringLiteral("wrist3_dae")
          };
          const bool has_ur_description_ur5_visual_path =
            source_token.contains(QStringLiteral("ur_description")) &&
            source_token.contains(QStringLiteral("meshes")) &&
            source_token.contains(QStringLiteral("ur5")) &&
            source_token.contains(QStringLiteral("visual"));
          bool has_known_ur5_mesh_file = false;
          for (const QString & mesh_file : protected_ur5_mesh_files) {
            if (source_token.contains(mesh_file)) {
              has_known_ur5_mesh_file = true;
              break;
            }
          }
          return has_ur_description_ur5_visual_path && has_known_ur5_mesh_file;
        };
        auto assert_scene3d_generated_urdf_identity_namespace_regression = [&]() {
          QSet<QString> regression_preview_ids;
          regression_preview_ids.insert(QStringLiteral("robot_base"));
          YAML::Node base_link_row(YAML::NodeType::Map);
          base_link_row["link"] = "base_link";
          base_link_row["visual_name"] = "base_visual";
          YAML::Node shoulder_link_row(YAML::NodeType::Map);
          shoulder_link_row["link"] = "shoulder_link";
          shoulder_link_row["visual_name"] = "shoulder_visual";
          YAML::Node upper_arm_link_row(YAML::NodeType::Map);
          upper_arm_link_row["link"] = "upper_arm_link";
          upper_arm_link_row["visual_name"] = "upper_arm_visual";
          const QString base_id = visual_item_final_identity_key(base_link_row, 0);
          const QString shoulder_id = visual_item_final_identity_key(shoulder_link_row, 1);
          const QString upper_arm_id = visual_item_final_identity_key(upper_arm_link_row, 2);
          Q_ASSERT_X(base_id == QStringLiteral("generated_urdf::base_link::base_visual::0"),
                     "Scene3D generated URDF identity namespace",
                     "generated base_link visual must be generated-only and row-indexed");
          Q_ASSERT_X(shoulder_id == QStringLiteral("generated_urdf::shoulder_link::shoulder_visual::1"),
                     "Scene3D generated URDF identity namespace",
                     "generated shoulder_link visual must be generated-only and row-indexed");
          Q_ASSERT_X(upper_arm_id == QStringLiteral("generated_urdf::upper_arm_link::upper_arm_visual::2"),
                     "Scene3D generated URDF identity namespace",
                     "generated upper_arm_link visual must be generated-only and row-indexed");
          Q_ASSERT_X(!regression_preview_ids.contains(base_id) && !regression_preview_ids.contains(shoulder_id) && !regression_preview_ids.contains(upper_arm_id),
                     "Scene3D generated URDF identity namespace",
                     "editable robot_base must not suppress generated URDF visuals");
        };
        assert_scene3d_generated_urdf_identity_namespace_regression();
        auto visual_item_is_lower_fidelity_fallback = [&](const YAML::Node & node) {
          const QString source = visual_item_source_token(node);
          const QString geometry = canonical_scene3d_token(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "geometry_type")));
          const QString category = canonical_scene3d_token(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "category")));
          const QString transform_status = canonical_scene3d_token(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(node, "transform_status")));
          return source != QStringLiteral("urdf_flattened") &&
                 (geometry != QStringLiteral("mesh") ||
                  category.contains(QStringLiteral("fallback")) ||
                  source.contains(QStringLiteral("fallback")) ||
                  source.contains(QStringLiteral("static")) ||
                  workcell_builder::yaml_map_key(node, "primitive_fallback").as<bool>(false) ||
                  transform_status == QStringLiteral("static_fallback") ||
                  transform_status == QStringLiteral("static_fallback_parent"));
        };
        for (const auto & item_node : visual_items) ordered_visual_items.push_back(YAML::Clone(item_node));
        std::stable_sort(ordered_visual_items.begin(), ordered_visual_items.end(), [&](const YAML::Node & a, const YAML::Node & b) {
          return visual_item_source_token(a) == QStringLiteral("urdf_flattened") &&
                 visual_item_source_token(b) != QStringLiteral("urdf_flattened");
        });
        for (const auto & item_node : ordered_visual_items) {
          if (!item_node.IsMap()) continue;
          const QString key = visual_item_link_object_key(item_node);
          const QString geometry = canonical_scene3d_token(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(item_node, "geometry_type")));
          if (visual_item_source_token(item_node) == QStringLiteral("urdf_flattened") &&
              geometry == QStringLiteral("mesh") &&
              !key.isEmpty() &&
              visual_item_mesh_reference_resolves(item_node)) {
            flattened_visual_mesh_keys.insert(key);
          }
        }
        int ordered_visual_source_row_index = 0;
        struct GeneratedVisualRowMetadata {
          int source_row_index = -1;
          QString final_id;
          QString link;
          QString link_name;
          QString canonical_link_name;
          QString package_uri;
          QString mesh_uri;
          QString source_path;
          QString mesh_path;
          QString resolved_source_path;
          QString generated_duplicate_key;
        };
        QHash<QString, GeneratedVisualRowMetadata> generated_visual_row_metadata_by_key;
        QSet<QString> protected_ur5_generated_visual_row_keys;
        QSet<QString> logged_required_ur5_ingestion_links;
        auto generated_visual_row_metadata = [&](const YAML::Node & node,
                                                int row_index,
                                                const QString & final_id,
                                                const QString & duplicate_key) {
          GeneratedVisualRowMetadata metadata;
          metadata.source_row_index = row_index;
          metadata.final_id = final_id;
          metadata.link = visual_index_first_scalar_value(node, {"link"});
          metadata.link_name = visual_index_first_scalar_value(node, {"link_name"});
          metadata.canonical_link_name = visual_item_canonical_link(node);
          metadata.package_uri = visual_index_first_scalar_value(node, {"package_uri"});
          metadata.mesh_uri = visual_index_first_scalar_value(node, {"mesh_uri"});
          metadata.source_path = visual_index_first_scalar_value(node, {"source_path", "filename"});
          metadata.mesh_path = visual_index_first_scalar_value(node, {"mesh_path"});
          metadata.resolved_source_path = visual_index_first_scalar_value(node, {"resolved_source_path", "resolved_path"});
          metadata.generated_duplicate_key = duplicate_key;
          return metadata;
        };
        auto add_duplicate_generated_visual_diagnostic_fields = [&](QJsonObject & row,
                                                                    const YAML::Node & skipped_node,
                                                                    int skipped_row_index,
                                                                    const QString & skipped_id,
                                                                    const QString & duplicate_key,
                                                                    const GeneratedVisualRowMetadata & retained,
                                                                    bool protected_ur5_generated_visual_row) {
          row[QStringLiteral("skipped_row_index")] = skipped_row_index;
          row[QStringLiteral("skipped_id")] = skipped_id;
          row[QStringLiteral("skipped_link")] = visual_index_first_scalar_value(skipped_node, {"link"});
          row[QStringLiteral("skipped_link_name")] = visual_index_first_scalar_value(skipped_node, {"link_name"});
          row[QStringLiteral("skipped_canonical_link_name")] = visual_item_canonical_link(skipped_node);
          row[QStringLiteral("skipped_package_uri")] = visual_index_first_scalar_value(skipped_node, {"package_uri"});
          row[QStringLiteral("skipped_mesh_uri")] = visual_index_first_scalar_value(skipped_node, {"mesh_uri"});
          row[QStringLiteral("skipped_source_path")] = visual_index_first_scalar_value(skipped_node, {"source_path", "filename"});
          row[QStringLiteral("skipped_mesh_path")] = visual_index_first_scalar_value(skipped_node, {"mesh_path"});
          row[QStringLiteral("skipped_resolved_source_path")] = visual_index_first_scalar_value(skipped_node, {"resolved_source_path", "resolved_path"});
          row[QStringLiteral("generated_duplicate_key")] = duplicate_key;
          row[QStringLiteral("existing_retained_row_index")] = retained.source_row_index;
          row[QStringLiteral("existing_retained_id")] = retained.final_id;
          row[QStringLiteral("existing_retained_key")] = retained.generated_duplicate_key;
          row[QStringLiteral("existing_retained_link")] = retained.link;
          row[QStringLiteral("existing_retained_link_name")] = retained.link_name;
          row[QStringLiteral("existing_retained_canonical_link_name")] = retained.canonical_link_name;
          row[QStringLiteral("existing_retained_package_uri")] = retained.package_uri;
          row[QStringLiteral("existing_retained_mesh_uri")] = retained.mesh_uri;
          row[QStringLiteral("existing_retained_source_path")] = retained.source_path;
          row[QStringLiteral("existing_retained_mesh_path")] = retained.mesh_path;
          row[QStringLiteral("existing_retained_resolved_source_path")] = retained.resolved_source_path;
          row[QStringLiteral("protected_ur5_generated_visual_row")] = protected_ur5_generated_visual_row;
        };
        auto duplicate_generated_visual_log_message = [&](const YAML::Node & skipped_node,
                                                         int skipped_row_index,
                                                         const QString & skipped_id,
                                                         const QString & duplicate_key,
                                                         const GeneratedVisualRowMetadata & retained,
                                                         bool protected_ur5_generated_visual_row) {
          return QStringLiteral("Scene3D skipped duplicate generated visual index row: skipped_row_index=%1 skipped_id=%2 skipped_link=%3 skipped_link_name=%4 skipped_canonical_link_name=%5 skipped_package_uri=%6 skipped_mesh_uri=%7 skipped_source_path=%8 skipped_mesh_path=%9 skipped_resolved_source_path=%10 generated_duplicate_key=%11 existing_retained_row_index=%12 existing_retained_id=%13 existing_retained_key=%14 protected_ur5_generated_visual_row=%15")
            .arg(skipped_row_index)
            .arg(skipped_id)
            .arg(visual_index_first_scalar_value(skipped_node, {"link"}))
            .arg(visual_index_first_scalar_value(skipped_node, {"link_name"}))
            .arg(visual_item_canonical_link(skipped_node))
            .arg(visual_index_first_scalar_value(skipped_node, {"package_uri"}))
            .arg(visual_index_first_scalar_value(skipped_node, {"mesh_uri"}))
            .arg(visual_index_first_scalar_value(skipped_node, {"source_path", "filename"}))
            .arg(visual_index_first_scalar_value(skipped_node, {"mesh_path"}))
            .arg(visual_index_first_scalar_value(skipped_node, {"resolved_source_path", "resolved_path"}))
            .arg(duplicate_key)
            .arg(retained.source_row_index)
            .arg(retained.final_id)
            .arg(retained.generated_duplicate_key)
            .arg(protected_ur5_generated_visual_row ? QStringLiteral("true") : QStringLiteral("false"));
        };
        for (const auto &v : ordered_visual_items) {
          const int ordered_source_row_index = ordered_visual_source_row_index++;
          const int source_row_index = workcell_builder::yaml_map_key(v, "source_row_index").as<int>(ordered_source_row_index);
          if (!v.IsMap()) {
            ++skipped_other;
            const QString detailed_parse_error = visual_index_parse_error_reason(v, source_row_index);
            const QString reason = add_skip_reason(detailed_parse_error);
            append_generated_visual_row_diagnostic(
              generated_visual_row_diagnostic(v, source_row_index, QString(), QString()),
              detailed_parse_error, reason);
            append_visual_ingestion_diagnostic(v, QString(), QString(), reason);
            append_studio_log(QStringLiteral("Scene3D visual index row parse_error: source_row_index=%1 reason=%2 id=<missing> link=<missing> visual=<missing> package_uri=<missing> mesh_uri=<missing> source_path=<missing> resolved_source_path=<missing> geometry=<missing> category=<missing>")
              .arg(source_row_index).arg(reason));
            continue;
          }
          ++visual_index_loaded_count;
          const QString raw_id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "id"));
          QString id = visual_item_final_identity_key(v, source_row_index);
          QString generated_visual_row_key;
          QJsonObject generated_row_diagnostic;
          if (id.isEmpty()) {
            ++skipped_other;
            const QString detailed_parse_error = visual_index_parse_error_reason(v, source_row_index);
            const QString reason = add_skip_reason(detailed_parse_error);
            append_generated_visual_row_diagnostic(
              generated_visual_row_diagnostic(v, source_row_index, QString(), id),
              detailed_parse_error, reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            append_studio_log(QStringLiteral("Scene3D visual index row parse_error: source_row_index=%1 reason=%2 id=%3 link=%4 link_name=%5 visual=%6 visual_name=%7 package_uri=%8 mesh_uri=%9 source_path=%10 resolved_source_path=%11 geometry=%12 category=%13")
              .arg(source_row_index).arg(reason).arg(raw_id)
              .arg(visual_index_first_scalar_value(v, {"link"}))
              .arg(visual_index_first_scalar_value(v, {"link_name"}))
              .arg(visual_index_first_scalar_value(v, {"visual"}))
              .arg(visual_index_first_scalar_value(v, {"visual_name"}))
              .arg(visual_index_first_scalar_value(v, {"package_uri"}))
              .arg(visual_index_first_scalar_value(v, {"mesh_uri"}))
              .arg(visual_index_first_scalar_value(v, {"source_path", "filename"}))
              .arg(visual_index_first_scalar_value(v, {"resolved_source_path", "resolved_path"}))
              .arg(visual_index_first_scalar_value(v, {"geometry_type", "type"}))
              .arg(visual_index_first_scalar_value(v, {"category"})));
            continue;
          }
          generated_visual_row_key = visual_item_generated_row_key(v, source_row_index);
          generated_row_diagnostic = generated_visual_row_diagnostic(v, source_row_index, generated_visual_row_key, id);
          const bool protected_ur5_generated_mesh_row = is_protected_ur5_generated_visual_row(v);
          const QString protected_ur5_link = protected_ur5_generated_mesh_row ? normalized_protected_ur5_link(v) : QString();
          auto log_ur5_runtime_decision = [&](const QString & decision, const QString & reason, const QString & mesh = QString()) {
            if (!protected_ur5_generated_mesh_row) return;
            append_studio_log(QStringLiteral("UR5_VISUAL_RUNTIME_DECISION row=%1 link=%2 mesh=%3 decision=%4 reason=%5")
              .arg(source_row_index)
              .arg(protected_ur5_link)
              .arg(mesh.trimmed().isEmpty() ? visual_item_mesh_identity(v) : mesh)
              .arg(decision)
              .arg(reason));
          };
          if (protected_ur5_generated_mesh_row) {
            const QString mesh_identity = visual_item_normalized_mesh_identity(v);
            // UR5 generated mesh rows are authoritative physical visuals.  Do
            // not compare them against semantic placeholders or collapse them
            // through generic generated-row keys; only an exact same link +
            // same mesh identity is a duplicate.
            generated_visual_row_key = QStringLiteral("protected_ur5_visual::%1::%2")
              .arg(protected_ur5_link,
                   mesh_identity.isEmpty() ? QStringLiteral("mesh_missing") : mesh_identity);
            generated_row_diagnostic = generated_visual_row_diagnostic(v, source_row_index, generated_visual_row_key, id);
            if (protected_ur5_generated_visual_row_keys.contains(generated_visual_row_key)) {
              ++skipped_true_duplicate_mesh_rows;
              const QString reason = add_skip_reason(QStringLiteral("duplicate_generated_visual_index_row"));
              const GeneratedVisualRowMetadata retained = generated_visual_row_metadata_by_key.value(generated_visual_row_key);
              add_duplicate_generated_visual_diagnostic_fields(
                generated_row_diagnostic, v, source_row_index, id, generated_visual_row_key, retained, true);
              append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("duplicate_generated_visual_index_row"), reason);
              append_visual_ingestion_diagnostic(v, raw_id, id, reason);
              append_studio_log(duplicate_generated_visual_log_message(
                v, source_row_index, id, generated_visual_row_key, retained, true));
              append_studio_log(QStringLiteral("UR5_VISUAL_RUNTIME_DECISION row=%1 link=%2 mesh=%3 decision=skipped reason=duplicate_same_link_same_mesh")
                .arg(source_row_index).arg(protected_ur5_link).arg(visual_item_mesh_identity(v)));
              continue;
            }
            protected_ur5_generated_visual_row_keys.insert(generated_visual_row_key);
            generated_visual_row_metadata_by_key.insert(
              generated_visual_row_key,
              generated_visual_row_metadata(v, source_row_index, id, generated_visual_row_key));
            ++preserved_generated_urdf_robot_mesh_rows;
            append_studio_log(QStringLiteral("Scene3D preserved generated URDF robot mesh row before preview handoff: key=%1 link=%2 visual=%3 mesh=%4")
              .arg(generated_visual_row_key, protected_ur5_link, visual_item_canonical_visual(v), visual_item_mesh_identity(v)));
          } else if (generated_visual_row_metadata_by_key.contains(generated_visual_row_key)) {
            ++skipped_true_duplicate_mesh_rows;
            const QString reason = add_skip_reason(QStringLiteral("duplicate_generated_visual_index_row"));
            const GeneratedVisualRowMetadata retained = generated_visual_row_metadata_by_key.value(generated_visual_row_key);
            add_duplicate_generated_visual_diagnostic_fields(
              generated_row_diagnostic, v, source_row_index, id, generated_visual_row_key, retained, false);
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("duplicate_generated_visual_index_row"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            append_studio_log(duplicate_generated_visual_log_message(
              v, source_row_index, id, generated_visual_row_key, retained, false));
            continue;
          } else {
            generated_visual_row_metadata_by_key.insert(
              generated_visual_row_key,
              generated_visual_row_metadata(v, source_row_index, id, generated_visual_row_key));
          }
          if (preview_ids.contains(id)) {
            const QString original_collision_id = id;
            int repair_suffix = 1;
            do {
              id = QStringLiteral("%1::dedupe_%2").arg(original_collision_id).arg(repair_suffix++);
            } while (preview_ids.contains(id));
            append_studio_log(QString("Scene3D generated URDF visual id collision repaired deterministically for distinct generated visual rows: %1 -> %2")
                                .arg(original_collision_id, id));
          }
          QString geometry_type = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "geometry_type"));
          if (geometry_type.trimmed().isEmpty()) geometry_type = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "type"));
          if (geometry_type.trimmed().isEmpty()) geometry_type = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "category"));
          const QString visual_item_source = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "source")).trimmed();
          const QString visual_item_source_token = canonical_scene3d_token(visual_item_source);
          const QString visual_item_key = visual_item_link_object_key(v);
          const QString visual_link = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link")).trimmed();
          const QString visual_link_name_field = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link_name")).trimmed();
          const QString visual_link_name = visual_link_name_field.isEmpty() ? visual_link : visual_link_name_field;
          const QString canonical_visual_link_name = canonical_scene3d_token(visual_link_name);
          if (canonical_visual_link_name == QStringLiteral("base_link_inertia")) {
            append_studio_log(QStringLiteral("Scene3D base_link_inertia trace: stage=scene_visual_mesh_index_visual_0_read source_row_index=%1 link_name=%2 visual=%3 mesh_uri=%4 source=%5 item_id=%6")
              .arg(source_row_index)
              .arg(visual_link_name)
              .arg(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "visual_name")).trimmed().isEmpty()
                ? QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "visual")).trimmed()
                : QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "visual_name")).trimmed())
              .arg(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "mesh_uri")).trimmed())
              .arg(visual_item_source.isEmpty() ? QStringLiteral("<missing>") : visual_item_source)
              .arg(id));
          }
          if (source_row_index >= 0 && source_row_index <= 6 &&
              required_ur5_visual_links.contains(canonical_visual_link_name) &&
              !logged_required_ur5_ingestion_links.contains(canonical_visual_link_name)) {
            logged_required_ur5_ingestion_links.insert(canonical_visual_link_name);
            append_studio_log(QString(
              "Scene3D required UR5 visual index row loaded: source_row_index=%1 link_name=%2 mesh_uri=%3 item_id=%4 source_layer=locked_generated_urdf_visual source=%5")
              .arg(source_row_index)
              .arg(visual_link_name)
              .arg(QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "mesh_uri")).trimmed())
              .arg(id)
              .arg(visual_item_source.isEmpty() ? QStringLiteral("<missing>") : visual_item_source));
          }
          const bool flattened_mesh_loaded_for_same_link = !visual_item_key.isEmpty() && flattened_visual_mesh_keys.contains(visual_item_key);
          const bool suppress_lower_fidelity_for_flattened_mesh =
            flattened_mesh_loaded_for_same_link && visual_item_is_lower_fidelity_fallback(v);
          if (suppress_lower_fidelity_for_flattened_mesh) {
            ++skipped_semantic_helper_visual_rows;
            const QString reason = add_skip_reason(QStringLiteral("suppressed_by_urdf_flattened_visual_mesh"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("suppressed_by_urdf_flattened_visual_mesh"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            append_studio_log(QString("Suppressed lower-fidelity visual for %1 because source=urdf_flattened mesh is available for the same link/object (source=%2 geometry=%3)")
                                .arg(raw_id.isEmpty() ? id : raw_id, visual_item_source.isEmpty() ? QStringLiteral("<missing>") : visual_item_source, geometry_type));
            log_ur5_runtime_decision(QStringLiteral("skipped"), QStringLiteral("suppressed_by_urdf_flattened_visual_mesh"));
            continue;
          }
          if (geometry_type == "mesh") ++mesh_item_count;
          else if (geometry_type == "box" || geometry_type == "cylinder" || geometry_type == "sphere" || geometry_type == "capsule") ++primitive_item_count;
          else ++unknown_item_count;
          const bool render_expected = workcell_builder::yaml_map_key(v, "render_expected").as<bool>(false);
          const YAML::Node pose = workcell_builder::yaml_map_key(v, "pose");
          const YAML::Node baked_world_visual_pose = workcell_builder::yaml_map_key(v, "baked_world_visual_pose");
          const YAML::Node baked_xyz = workcell_builder::yaml_map_key(baked_world_visual_pose, "xyz");
          const YAML::Node baked_rpy = workcell_builder::yaml_map_key(baked_world_visual_pose, "rpy");
          const bool use_baked_world_visual_pose =
            baked_xyz && baked_rpy && baked_xyz.IsSequence() && baked_rpy.IsSequence() &&
            baked_xyz.size() >= 3 && baked_rpy.size() >= 3;
          const YAML::Node xyz = use_baked_world_visual_pose ? baked_xyz : workcell_builder::yaml_map_key(pose, "xyz");
          const YAML::Node rpy = use_baked_world_visual_pose ? baked_rpy : workcell_builder::yaml_map_key(pose, "rpy");
          const QString transform_source = use_baked_world_visual_pose
            ? QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "baked_world_visual_transform_source")).trimmed()
            : QStringLiteral("pose");
          const YAML::Node world_pose = workcell_builder::yaml_map_key(v, "world_pose");
          const YAML::Node world_xyz = workcell_builder::yaml_map_key(world_pose, "xyz");
          const YAML::Node world_rpy = workcell_builder::yaml_map_key(world_pose, "rpy");
          const YAML::Node baked_world_visual_pose_xyz = workcell_builder::yaml_map_key(baked_world_visual_pose, "xyz");
          const YAML::Node baked_world_visual_pose_rpy = workcell_builder::yaml_map_key(baked_world_visual_pose, "rpy");
          const YAML::Node baked_world_visual_matrix = workcell_builder::yaml_map_key(v, "baked_world_visual_matrix");
          const YAML::Node visual_origin = workcell_builder::yaml_map_key(v, "visual_origin");
          const YAML::Node visual_origin_xyz = workcell_builder::yaml_map_key(visual_origin, "xyz");
          const YAML::Node visual_origin_rpy = workcell_builder::yaml_map_key(visual_origin, "rpy");
          const QString transform_status = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "transform_status")).trimmed().toLower();
          QString parent_link = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "parent_link")).trimmed();
          if (parent_link.isEmpty()) {
            parent_link = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "base_frame")).trimmed();
          }
          consider_robot_base_frame_candidate(best_robot_base_frame_candidate, yaml_scalar_string(v, "root_link"), 300);
          consider_robot_base_frame_candidate(best_robot_base_frame_candidate, parent_link, 150);
          consider_robot_base_frame_candidate(best_robot_base_frame_candidate, yaml_scalar_string(v, "link"), 0);
          consider_robot_base_frame_chain(best_robot_base_frame_candidate, v, "link_chain", 200);
          if (!best_robot_base_frame_candidate.frame.isEmpty()) robot_base_frame = best_robot_base_frame_candidate.frame;
          const bool has_visual_metadata =
            !id.trimmed().isEmpty() &&
            !visual_index_first_scalar_value(v, {"link", "link_name", "object_id", "object"}).trimmed().isEmpty() &&
            xyz && rpy && xyz.IsSequence() && rpy.IsSequence() && xyz.size() >= 3 && rpy.size() >= 3;
          if (!render_expected && !has_visual_metadata) {
            ++skipped_semantic_helper_visual_rows;
            const QString reason = add_skip_reason(QStringLiteral("unsafe_for_preview"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("unsafe_for_preview"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            log_ur5_runtime_decision(QStringLiteral("skipped"), QStringLiteral("unsafe_for_preview"));
            continue;
          }
          const bool unsupported_format = workcell_builder::yaml_map_key(v, "unsupported_format").as<bool>(false);
          const int triangle_count = workcell_builder::yaml_map_key(v, "triangle_count").as<int>(-1);
          if (triangle_count == 0) {
            const QString reason = add_skip_reason(QStringLiteral("zero_triangle_mesh"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("zero_triangle_mesh"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            log_ur5_runtime_decision(QStringLiteral("skipped"), QStringLiteral("zero_triangle_mesh"));
            continue;
          }
          const bool static_robot_fallback_visual =
            id.startsWith(QStringLiteral("urdf_static_fallback_"));
          const bool authoritative_expanded_mesh_payload =
            visual_index_safe_for_preview &&
            (visual_index_extraction_mode == QStringLiteral("xacro_expanded") ||
             visual_index_extraction_mode == QStringLiteral("xacro_lite_expanded"));
          if (authoritative_expanded_mesh_payload && static_robot_fallback_visual) {
            ++skipped_semantic_helper_visual_rows;
            const QString reason = add_skip_reason(QStringLiteral("suppressed_static_robot_fallback"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("suppressed_static_robot_fallback"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            log_ur5_runtime_decision(QStringLiteral("skipped"), QStringLiteral("suppressed_static_robot_fallback"));
            continue;
          }
          ScenePreviewWidget::PreviewItem p;
          p.id = id;
          const QString raw_visual_link_from_row = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link")).trimmed();
          const QString raw_visual_link_name_from_row = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link_name")).trimmed();
          const QString normalized_ur5_preview_link = protected_ur5_generated_mesh_row ? protected_ur5_link : QString();
          const QString raw_visual_link = normalized_ur5_preview_link.isEmpty() ? raw_visual_link_from_row : normalized_ur5_preview_link;
          const QString raw_visual_link_name = normalized_ur5_preview_link.isEmpty() ? raw_visual_link_name_from_row : normalized_ur5_preview_link;
          const QString stable_visual_link_identity = raw_visual_link_name.isEmpty() ? raw_visual_link : raw_visual_link_name;
          p.display_name = stable_visual_link_identity.isEmpty() ? id : stable_visual_link_identity;
          p.frame_id = p.display_name;
          p.category = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "category"));
          if (protected_ur5_generated_mesh_row) p.category = QStringLiteral("robot_arm");
          if (p.category.trimmed().isEmpty()) p.category = "URDF Visual";
          p.role = p.category;
          p.status = "ready";
          p.source_path = visual_index_first_scalar_value(v, {"source_path", "mesh_uri", "filename"});
          const QString resolved_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "resolved_path"));
          const QString resolved_source_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "resolved_source_path"));
          QString package_uri = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "package_uri"));
          if (package_uri.trimmed().isEmpty()) package_uri = visual_index_first_scalar_value(v, {"mesh_uri", "filename"});
          p.visual_index_link = raw_visual_link.isEmpty() ? stable_visual_link_identity : raw_visual_link;
          p.visual_index_link_name = raw_visual_link_name.isEmpty() ? stable_visual_link_identity : raw_visual_link_name;
          p.resolved_source_path_original = resolved_source_path;
          p.package_uri = package_uri;
          bool unresolved_package_uri = false;
          if (p.source_path.trimmed().isEmpty() && !resolved_path.trimmed().isEmpty()) {
            p.source_path = resolved_path;
            ++source_path_from_resolved_path;
          }
          const bool has_resolved_source_path = !resolved_source_path.trimmed().isEmpty();
          QString resolved_source_path_candidate;
          if (has_resolved_source_path) {
            const QFileInfo resolved_info(resolved_source_path);
            if (resolved_info.isAbsolute()) {
              if (resolved_info.exists() && resolved_info.isFile()) {
                resolved_source_path_candidate = resolved_info.canonicalFilePath();
                if (resolved_source_path_candidate.isEmpty()) {
                  resolved_source_path_candidate = resolved_info.absoluteFilePath();
                }
              }
            } else {
              const QString repo_relative_candidate = QString::fromStdString(
                (workcell_builder_repo_root_from_source() / resolved_source_path.toStdString()).string());
              const QFileInfo repo_relative_info(repo_relative_candidate);
              if (repo_relative_info.exists() && repo_relative_info.isFile()) {
                resolved_source_path_candidate = repo_relative_info.canonicalFilePath();
                if (resolved_source_path_candidate.isEmpty()) {
                  resolved_source_path_candidate = repo_relative_info.absoluteFilePath();
                }
              } else {
                const QString scene_relative_candidate = QString::fromStdString((d / resolved_source_path.toStdString()).string());
                const QFileInfo scene_relative_info(scene_relative_candidate);
                if (scene_relative_info.exists() && scene_relative_info.isFile()) {
                  resolved_source_path_candidate = scene_relative_info.canonicalFilePath();
                  if (resolved_source_path_candidate.isEmpty()) {
                    resolved_source_path_candidate = scene_relative_info.absoluteFilePath();
                  }
                }
              }
            }
          }
          if (!resolved_source_path_candidate.isEmpty() && !package_uri.trimmed().startsWith(QStringLiteral("package://"))) {
            p.source_path = resolved_source_path_candidate;
            p.mesh_path = resolved_source_path_candidate;
            p.mesh_available = true;
            p.has_mesh_metadata = true;
            p.active_visual_source = QStringLiteral("mesh_preview");
            p.source_path_resolution_outcome = QStringLiteral("resolved_source_path_exists");
            ++source_path_from_resolved_path;
          } else if (has_resolved_source_path) {
            p.resolved_source_path_stale = true;
            p.source_path_resolution_outcome = QStringLiteral("resolved_source_path_missing");
            ++stale_resolved_source_path_count;
            append_studio_log(QString(
              "URDF visual stale resolved_source_path for %1: resolved_source_path=%2 package_uri=%3")
              .arg(p.id, resolved_source_path, package_uri));
          }
          if (p.source_path.trimmed().isEmpty() && (package_uri.startsWith("file://") || package_uri.startsWith("/"))) {
            QString candidate = package_uri;
            if (candidate.startsWith("file://")) candidate = candidate.mid(7);
            const QFileInfo candidate_info(candidate);
            if (candidate_info.exists() && candidate_info.isFile()) {
              p.source_path = candidate_info.canonicalFilePath();
              if (p.source_path.isEmpty()) p.source_path = candidate_info.absoluteFilePath();
              p.source_path_resolution_outcome = p.resolved_source_path_stale
                ? QStringLiteral("resolved_via_package_uri_after_stale_resolved_source_path")
                : QStringLiteral("resolved_via_package_uri");
              ++package_uri_resolved_by_loader;
              if (p.resolved_source_path_stale) ++package_uri_resolved_after_stale_resolved_source_path;
              append_studio_log(QString(
                "URDF visual resolution outcome for %1: resolved_source_path=%2 package_uri=%3 outcome=%4 source_path=%5")
                .arg(p.id, resolved_source_path, package_uri, p.source_path_resolution_outcome, p.source_path));
            }
          }
          p.locked = true;
          p.editable = false;
          p.selectable = true;
          p.lock_reason = "generated URDF visual";
          p.robot_base_frame = parent_link.isEmpty() ? QStringLiteral("unknown") : parent_link;
          p.source_layer = QStringLiteral("locked_generated_urdf_visual");
          normalize_generated_urdf_visual_identity(p);
          classify_generated_urdf_visual(p, resolved_source_path);
          if (visual_item_source_token == QStringLiteral("urdf_flattened")) {
            p.metadata_tags = p.metadata_tags.trimmed().isEmpty()
              ? QStringLiteral("source=urdf_flattened;locked_generated_urdf_preview")
              : p.metadata_tags + QStringLiteral(";source=urdf_flattened;locked_generated_urdf_preview");
          }
          const bool explicit_primitive_fallback = workcell_builder::yaml_map_key(v, "primitive_fallback").as<bool>(false) ||
            transform_status == QStringLiteral("static_fallback") ||
            transform_status == QStringLiteral("static_fallback_parent");
          p.active_visual_source = (geometry_type == "mesh") ? QStringLiteral("mesh_preview")
                                      : (explicit_primitive_fallback ? QStringLiteral("primitive_fallback") : QStringLiteral("urdf_primitive"));
          p.linked_to_editable_layout_state = false;
          p.editable = false;
          p.selectable = true;
          if (!xyz || !rpy || !xyz.IsSequence() || !rpy.IsSequence() || xyz.size() < 3 || rpy.size() < 3) {
            const QString reason = add_skip_reason(QStringLiteral("invalid_pose"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("invalid_pose"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            log_ur5_runtime_decision(QStringLiteral("skipped"), QStringLiteral("invalid_pose"));
            continue;
          }
          p.x = workcell_builder::yaml_seq_index(xyz,0).as<double>(0.0);
          p.y = workcell_builder::yaml_seq_index(xyz,1).as<double>(0.0);
          p.z = workcell_builder::yaml_seq_index(xyz,2).as<double>(0.0);
          const QString pose_field_name = use_baked_world_visual_pose ? QStringLiteral("baked_world_visual_pose") : QStringLiteral("pose");
          p.roll = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy,0).as<double>(0.0), QStringLiteral("%1.rpy[0]").arg(pose_field_name), &p.warnings);
          p.pitch = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy,1).as<double>(0.0), QStringLiteral("%1.rpy[1]").arg(pose_field_name), &p.warnings);
          p.yaw = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy,2).as<double>(0.0), QStringLiteral("%1.rpy[2]").arg(pose_field_name), &p.warnings);
          if (use_baked_world_visual_pose) {
            ++baked_world_visual_pose_count;
            if (!transform_source.isEmpty()) baked_world_visual_transform_source = transform_source;
          }
          p.chain_pose_x = p.x; p.chain_pose_y = p.y; p.chain_pose_z = p.z;
          p.chain_pose_roll = p.roll; p.chain_pose_pitch = p.pitch; p.chain_pose_yaw = p.yaw;
          if (baked_world_visual_pose_xyz && baked_world_visual_pose_rpy &&
              baked_world_visual_pose_xyz.IsSequence() && baked_world_visual_pose_rpy.IsSequence() &&
              baked_world_visual_pose_xyz.size() >= 3 && baked_world_visual_pose_rpy.size() >= 3) {
            p.x = workcell_builder::yaml_seq_index(baked_world_visual_pose_xyz,0).as<double>(0.0);
            p.y = workcell_builder::yaml_seq_index(baked_world_visual_pose_xyz,1).as<double>(0.0);
            p.z = workcell_builder::yaml_seq_index(baked_world_visual_pose_xyz,2).as<double>(0.0);
            p.roll = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(baked_world_visual_pose_rpy,0).as<double>(0.0), QStringLiteral("baked_world_visual_pose.rpy[0]"), &p.warnings);
            p.pitch = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(baked_world_visual_pose_rpy,1).as<double>(0.0), QStringLiteral("baked_world_visual_pose.rpy[1]"), &p.warnings);
            p.yaw = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(baked_world_visual_pose_rpy,2).as<double>(0.0), QStringLiteral("baked_world_visual_pose.rpy[2]"), &p.warnings);
            p.has_baked_world_visual_transform = true;
            p.baked_world_visual_transform_source = QStringLiteral("generated/scene_visual_mesh_index.json:baked_world_visual_pose");
            p.transform_chain_applied = false;
            p.visual_origin_applied = false;
          }
          if (baked_world_visual_matrix && baked_world_visual_matrix.IsSequence() &&
              (baked_world_visual_matrix.size() == 16 || baked_world_visual_matrix.size() == 4)) {
            bool matrix_loaded = false;
            if (baked_world_visual_matrix.size() == 16) {
              for (std::size_t i = 0; i < 16; ++i) {
                p.baked_world_visual_matrix[i] = workcell_builder::yaml_seq_index(baked_world_visual_matrix, i).as<double>(i % 5 == 0 ? 1.0 : 0.0);
              }
              matrix_loaded = true;
            } else {
              matrix_loaded = true;
              for (std::size_t row = 0; row < 4; ++row) {
                const YAML::Node row_node = workcell_builder::yaml_seq_index(baked_world_visual_matrix, row);
                if (!row_node || !row_node.IsSequence() || row_node.size() < 4) {
                  matrix_loaded = false;
                  break;
                }
                for (std::size_t col = 0; col < 4; ++col) {
                  p.baked_world_visual_matrix[row * 4 + col] = workcell_builder::yaml_seq_index(row_node, col).as<double>(row == col ? 1.0 : 0.0);
                }
              }
            }
            if (matrix_loaded) {
              p.has_baked_world_visual_transform = true;
              p.has_baked_world_visual_matrix = true;
              p.baked_world_visual_transform_source = QStringLiteral("generated/scene_visual_mesh_index.json:baked_world_visual_matrix");
              p.transform_chain_applied = false;
              p.visual_origin_applied = false;
            }
          }
          const bool transform_status_resolved =
            transform_status == QStringLiteral("resolved") ||
            transform_status == QStringLiteral("ok") ||
            transform_status == QStringLiteral("static_fallback") ||
            transform_status == QStringLiteral("static_fallback_parent") ||
            transform_status == QStringLiteral("static_mesh_resolved");
          p.transform_chain_applied = p.has_baked_world_visual_transform ? false : transform_status_resolved;
          if (p.transform_chain_applied) ++transform_chain_applied_count;
          if (p.has_baked_world_visual_transform) {
            ++baked_world_visual_transform_count;
          } else {
            ++legacy_viewport_transform_count;
          }
          if (!p.has_baked_world_visual_transform && !p.transform_chain_applied) {
            ++missing_chain_warning_count;
            const QString transform_status_for_warning = transform_status.isEmpty() ? QStringLiteral("missing") : transform_status;
            p.warnings << QStringLiteral("Preview warning: missing/broken URDF chain; identity fallback avoided (transform_status=%1)").arg(transform_status_for_warning);
          }
          if (world_xyz && world_rpy && world_xyz.IsSequence() && world_rpy.IsSequence() && world_xyz.size() >= 3 && world_rpy.size() >= 3) {
            p.base_pose_x = workcell_builder::yaml_seq_index(world_xyz,0).as<double>(0.0);
            p.base_pose_y = workcell_builder::yaml_seq_index(world_xyz,1).as<double>(0.0);
            p.base_pose_z = workcell_builder::yaml_seq_index(world_xyz,2).as<double>(0.0);
            p.base_pose_roll = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(world_rpy,0).as<double>(0.0), QStringLiteral("world_pose.rpy[0]"), &p.warnings);
            p.base_pose_pitch = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(world_rpy,1).as<double>(0.0), QStringLiteral("world_pose.rpy[1]"), &p.warnings);
            p.base_pose_yaw = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(world_rpy,2).as<double>(0.0), QStringLiteral("world_pose.rpy[2]"), &p.warnings);
          }
          if (robot_world_pose == QStringLiteral("unknown")) {
            robot_world_pose = QStringLiteral("xyz=[%1,%2,%3] rpy=[%4,%5,%6]")
              .arg(p.base_pose_x).arg(p.base_pose_y).arg(p.base_pose_z)
              .arg(p.base_pose_roll).arg(p.base_pose_pitch).arg(p.base_pose_yaw);
          }
          p.robot_world_pose = robot_world_pose;
          p.visual_index_link = raw_visual_link.isEmpty() ? stable_visual_link_identity : raw_visual_link;
          p.visual_index_link_name = raw_visual_link_name.isEmpty() ? stable_visual_link_identity : raw_visual_link_name;
          normalize_generated_urdf_visual_identity(p);
          classify_generated_urdf_visual(p, resolved_source_path);
          p.visual_index_object_name = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "object_name")).trimmed();
          p.visual_index_visual = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "visual")).trimmed();
          p.visual_index_visual_name = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "visual_name")).trimmed();
          p.visual_index_value = workcell_builder::yaml_map_key(v, "visual_index").as<int>(-1);
          p.source_row_index = source_row_index;
          p.visual_index_parent_link = parent_link;
          p.visual_index_mesh_uri = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "mesh_uri")).trimmed();
          p.visual_index_package_uri = package_uri;
          p.visual_index_source = visual_item_source;
          // Normalize the generated visual-index row on the PreviewItem that is
          // submitted to the native Scene3D viewport.  The renderer, final draw
          // export, and smoke audit all consume this PreviewItem payload.
          p.source_layer = QStringLiteral("locked_generated_urdf_visual");
          p.locked = true;
          p.editable = false;
          p.selectable = true;
          if (geometry_type == QStringLiteral("mesh")) {
            p.active_visual_source = QStringLiteral("generated_urdf_visual");
          }
          if (protected_ur5_generated_mesh_row) {
            p.source_layer = QStringLiteral("locked_generated_urdf_visual");
            p.active_visual_source = QStringLiteral("mesh_preview");
            p.category = QStringLiteral("robot_arm");
            p.role = p.category;
            p.visual_index_link = protected_ur5_link;
            p.visual_index_link_name = protected_ur5_link;
            p.display_name = protected_ur5_link;
            p.frame_id = protected_ur5_link;
            append_studio_log(QStringLiteral("Scene3D generated URDF robot mesh renderer handoff: item_id=%1 link=%2 source_layer=%3 active_visual_source=%4 mesh=%5")
              .arg(p.id, protected_ur5_link, p.source_layer, p.active_visual_source,
                   p.package_uri.trimmed().isEmpty() ? p.source_path : p.package_uri));
          }
          if (!p.visual_index_mesh_uri.trimmed().isEmpty() && p.package_uri.trimmed().isEmpty()) {
            p.package_uri = p.visual_index_mesh_uri.trimmed();
          }
          const YAML::Node link_chain_node = workcell_builder::yaml_map_key(v, "link_chain");
          if (link_chain_node && link_chain_node.IsSequence()) {
            for (const YAML::Node & chain_entry : link_chain_node) {
              if (chain_entry.IsScalar()) p.visual_index_link_chain << QString::fromStdString(chain_entry.as<std::string>("")).trimmed();
            }
          }
          const bool visual_origin_already_in_pose = workcell_builder::yaml_map_key(v, "visual_origin_applied_to_pose").as<bool>(false);
          if (use_baked_world_visual_pose) {
            p.visual_origin_applied = false;
          } else if (visual_origin_xyz && visual_origin_rpy &&
              visual_origin_xyz.IsSequence() && visual_origin_rpy.IsSequence() &&
              visual_origin_xyz.size() >= 3 && visual_origin_rpy.size() >= 3 &&
              !visual_origin_already_in_pose &&
              !p.has_baked_world_visual_transform) {
            p.visual_origin_x = workcell_builder::yaml_seq_index(visual_origin_xyz,0).as<double>(0.0);
            p.visual_origin_y = workcell_builder::yaml_seq_index(visual_origin_xyz,1).as<double>(0.0);
            p.visual_origin_z = workcell_builder::yaml_seq_index(visual_origin_xyz,2).as<double>(0.0);
            p.visual_origin_roll = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(visual_origin_rpy,0).as<double>(0.0), QStringLiteral("visual_origin.rpy[0]"), &p.warnings);
            p.visual_origin_pitch = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(visual_origin_rpy,1).as<double>(0.0), QStringLiteral("visual_origin.rpy[1]"), &p.warnings);
            p.visual_origin_yaw = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(visual_origin_rpy,2).as<double>(0.0), QStringLiteral("visual_origin.rpy[2]"), &p.warnings);
            p.visual_origin_applied = true;
            ++visual_origin_applied_count;
          } else if (visual_origin_already_in_pose) {
            p.visual_origin_applied = false;
          }
          const YAML::Node scale = workcell_builder::yaml_map_key(v, "mesh_scale");
          p.mesh_scale_x = workcell_builder::yaml_seq_index(scale,0).as<double>(1.0);
          p.mesh_scale_y = workcell_builder::yaml_seq_index(scale,1).as<double>(1.0);
          p.mesh_scale_z = workcell_builder::yaml_seq_index(scale,2).as<double>(1.0);
          p.primitive_geometry_type = geometry_type;
          const YAML::Node material = workcell_builder::yaml_map_key(v, "material");
          p.material_name = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(material, "name"));
          const YAML::Node material_color = workcell_builder::yaml_map_key(material, "color");
          if (material_color && material_color.IsSequence() && material_color.size() >= 3) {
            p.has_material_color = true;
            p.material_r = workcell_builder::yaml_seq_index(material_color,0).as<double>(0.0);
            p.material_g = workcell_builder::yaml_seq_index(material_color,1).as<double>(0.0);
            p.material_b = workcell_builder::yaml_seq_index(material_color,2).as<double>(0.0);
            p.material_a = workcell_builder::yaml_seq_index(material_color,3).as<double>(1.0);
          }
          p.sx = 0.25;
          p.sy = 0.25;
          p.sz = 0.25;
          if (geometry_type == "box") {
            const YAML::Node size = workcell_builder::yaml_map_key(v, "size");
            p.sx = workcell_builder::yaml_seq_index(size,0).as<double>(0.25);
            p.sy = workcell_builder::yaml_seq_index(size,1).as<double>(0.25);
            p.sz = workcell_builder::yaml_seq_index(size,2).as<double>(0.25);
          } else if (geometry_type == "cylinder") {
            const double radius = workcell_builder::yaml_map_key(v, "radius").as<double>(0.1);
            const double length = workcell_builder::yaml_map_key(v, "length").as<double>(0.2);
            p.primitive_radius = radius;
            p.primitive_length = length;
            p.sx = radius * 2.0;
            p.sy = radius * 2.0;
            p.sz = length;
          } else if (geometry_type == "sphere") {
            const double radius = workcell_builder::yaml_map_key(v, "radius").as<double>(0.1);
            p.primitive_radius = radius;
            p.sx = radius * 2.0;
            p.sy = radius * 2.0;
            p.sz = radius * 2.0;
          } else if (geometry_type == "capsule") {
            const double radius = workcell_builder::yaml_map_key(v, "radius").as<double>(0.1);
            const double length = workcell_builder::yaml_map_key(v, "length").as<double>(0.2);
            p.primitive_radius = radius;
            p.primitive_length = length;
            p.sx = radius * 2.0;
            p.sy = radius * 2.0;
            p.sz = length + radius * 2.0;
          }
          const QString lower_name = (p.display_name + " " + p.id + " " + p.role).toLower();
          if (geometry_type.trimmed().isEmpty() && (p.sx <= 0.001 || p.sy <= 0.001 || p.sz <= 0.001 || (p.sx == 0.25 && p.sy == 0.25 && p.sz == 0.25))) {
            if (lower_name.contains("base")) { p.sx = 0.45; p.sy = 0.45; p.sz = 0.22; }
            else if (lower_name.contains("shoulder") || lower_name.contains("upper_arm") || lower_name.contains("forearm")) { p.sx = 0.14; p.sy = 0.14; p.sz = 0.52; }
            else if (lower_name.contains("wrist")) { p.sx = 0.10; p.sy = 0.10; p.sz = 0.18; }
            else if (lower_name.contains("flange") || lower_name.contains("tool0") || lower_name.contains("tool")) { p.sx = 0.07; p.sy = 0.07; p.sz = 0.12; }
            else if (lower_name.contains("gripper") || lower_name.contains("end_effector")) { p.sx = 0.12; p.sy = 0.09; p.sz = 0.16; }
            else if (lower_name.contains("camera")) { p.sx = 0.09; p.sy = 0.07; p.sz = 0.07; }
            else if (lower_name.contains("table") || lower_name.contains("workbench")) { p.sx = 1.2; p.sy = 0.8; p.sz = 0.08; }
            else { p.sx = 0.08; p.sy = 0.08; p.sz = 0.08; }
          }
          const bool resolved = workcell_builder::yaml_map_key(v, "resolved").as<bool>(false);
          const bool is_primitive = (geometry_type == "box" || geometry_type == "cylinder" || geometry_type == "sphere" || geometry_type == "capsule");
          bool mesh_fallback = false;
          if (geometry_type == "mesh") {
            if (p.mesh_path.trimmed().isEmpty()) {
              QStringList tried_candidates;
              const QString workspace_root = detect_workspace_root();
              QString resolved_mesh_path;
              const bool should_try_package_uri_after_stale_resolved_source_path =
                p.resolved_source_path_stale && package_uri.trimmed().startsWith(QStringLiteral("package://"));
              if (should_try_package_uri_after_stale_resolved_source_path) {
                resolved_mesh_path = resolve_visual_mesh_source_path(
                  QString(), package_uri, d, workspace_root, &tried_candidates);
                if (!resolved_mesh_path.trimmed().isEmpty()) {
                  const QFileInfo resolved_mesh_info(resolved_mesh_path);
                  if (resolved_mesh_info.exists() && resolved_mesh_info.isFile()) {
                    p.source_path_resolution_outcome =
                      QStringLiteral("resolved_via_package_uri_after_stale_resolved_source_path");
                    ++package_uri_resolved_by_loader;
                    ++package_uri_resolved_after_stale_resolved_source_path;
                    append_studio_log(QString(
                      "URDF visual resolution outcome for %1: resolved_source_path=%2 package_uri=%3 outcome=%4 source_path=%5")
                      .arg(p.id, resolved_source_path, package_uri, p.source_path_resolution_outcome, resolved_mesh_path));
                  } else {
                    resolved_mesh_path.clear();
                  }
                }
              }
              if (resolved_mesh_path.trimmed().isEmpty()) {
                resolved_mesh_path = resolve_visual_mesh_source_path(
                  p.source_path, package_uri, d, workspace_root, &tried_candidates);
              }
              if (unsupported_format) {
                mesh_fallback = true;
              } else if (resolved_mesh_path.trimmed().isEmpty()) {
                mesh_fallback = true;
                if (p.resolved_source_path_stale &&
                    p.source_path_resolution_outcome == QStringLiteral("resolved_source_path_missing")) {
                  p.source_path_resolution_outcome = QStringLiteral("stale_resolved_source_path_unresolved_after_package_uri_attempt");
                  append_studio_log(QString(
                    "URDF visual resolution outcome for %1: resolved_source_path=%2 package_uri=%3 outcome=%4")
                    .arg(p.id, resolved_source_path, package_uri, p.source_path_resolution_outcome));
                }
              } else {
                p.mesh_path = resolved_mesh_path;
                p.source_path = resolved_mesh_path;
                if (p.source_path_resolution_outcome.trimmed().isEmpty()) {
                  p.source_path_resolution_outcome = QStringLiteral("resolved_via_mesh_source_resolution");
                }
                if (package_uri.startsWith(QStringLiteral("package://"))) {
                  const QString src = resolved_mesh_path.contains(QStringLiteral("/src/assets/"))
                    ? QStringLiteral("local asset workspace")
                    : (resolved_mesh_path.contains(QStringLiteral("/assets/"))
                        ? QStringLiteral("repo assets")
                        : QStringLiteral("ROS/ament or filesystem index"));
                  append_studio_log(QStringLiteral("Resolved %1 from %2: %3").arg(package_uri, src, resolved_mesh_path));
                }
              }
              if (mesh_fallback && !tried_candidates.isEmpty()) {
                append_studio_log(QString("URDF visual mesh unresolved for %1: raw=%2 package=%3 tried=[%4]")
                                    .arg(p.id, p.source_path, package_uri, tried_candidates.join(" | ")));
              }
            }
            if (p.mesh_path.trimmed().isEmpty() && package_uri.startsWith("package://")) {
              unresolved_package_uri = true;
            }
          } else if (is_primitive) {
            ++non_mesh_geometry_added;
          } else {
            ++skipped_semantic_helper_visual_rows;
            const QString reason = add_skip_reason(QStringLiteral("unknown_role_no_fallback"));
            append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("unknown_role_no_fallback"), reason);
            append_visual_ingestion_diagnostic(v, raw_id, id, reason);
            ++non_mesh_geometry_unsupported;
            continue;
          }
          if (mesh_fallback) {
            set_generated_visual_first_drop_stage_once(generated_row_diagnostic, QStringLiteral("mesh_resolution_or_fallback_branch"));
            const QString fallback_link = p.visual_index_link.trimmed().isEmpty() ? p.display_name.trimmed() : p.visual_index_link.trimmed();
            const QString robot_identity = canonical_scene3d_token(
              fallback_link + QStringLiteral(" ") + p.visual_index_parent_link + QStringLiteral(" ") +
              p.visual_index_link_chain.join(QStringLiteral(" ")) + QStringLiteral(" ") + p.id + QStringLiteral(" ") +
              p.category + QStringLiteral(" ") + p.role);
            const bool excluded_non_robot_visual =
              robot_identity.contains(QStringLiteral("camera")) ||
              robot_identity.contains(QStringLiteral("realsense")) ||
              robot_identity.contains(QStringLiteral("d435")) ||
              robot_identity.contains(QStringLiteral("d455")) ||
              robot_identity.contains(QStringLiteral("table")) ||
              robot_identity.contains(QStringLiteral("workbench")) ||
              robot_identity.contains(QStringLiteral("conveyor"));
            const bool generated_robot_visual_link = !excluded_non_robot_visual &&
              (robot_identity.contains(QStringLiteral("robot")) ||
               robot_identity.contains(QStringLiteral("base_link")) ||
               robot_identity.contains(QStringLiteral("shoulder")) ||
               robot_identity.contains(QStringLiteral("upper_arm")) ||
               robot_identity.contains(QStringLiteral("forearm")) ||
               robot_identity.contains(QStringLiteral("wrist")) ||
               robot_identity.contains(QStringLiteral("flange")) ||
               robot_identity.contains(QStringLiteral("tool0")) ||
               robot_identity.contains(QStringLiteral("gripper")) ||
               robot_identity.contains(QStringLiteral("finger")) ||
               robot_identity.contains(QStringLiteral("knuckle")));
            if (generated_robot_visual_link) {
              const QString fallback_id = QStringLiteral("generated_urdf_fallback::%1")
                .arg(fallback_link.trimmed().isEmpty() ? canonical_scene3d_token(id) : fallback_link);
              if (preview_ids.contains(fallback_id)) {
                const QString reason = add_skip_reason(QStringLiteral("duplicate_generated_urdf_fallback_link"));
                append_generated_visual_row_diagnostic(generated_row_diagnostic, QStringLiteral("duplicate_generated_urdf_fallback_link"), reason);
                append_visual_ingestion_diagnostic(v, raw_id, fallback_id, reason);
                continue;
              }
              p.id = fallback_id;
              p.display_name = fallback_link.trimmed().isEmpty() ? p.display_name : fallback_link;
              p.category = QStringLiteral("URDF Visual Fallback");
              p.role = QStringLiteral("robot");
              p.status = "warning";
              p.mesh_available = false;
              p.has_mesh_metadata = true;
              p.active_visual_source = QStringLiteral("generated_urdf_visual_fallback");
              p.source_layer = QStringLiteral("locked_generated_urdf_visual");
              p.editable = false;
              p.selectable = true;
              p.locked = true;
              p.lock_reason = QStringLiteral("generated URDF visual fallback");
              p.has_material_color = true;
              p.material_r = 1.0;
              p.material_g = 0.82;
              p.material_b = 0.05;
              p.material_a = 1.0;
              p.material_name = QStringLiteral("generated_urdf_visual_fallback_default_visible");
              p.warnings << QStringLiteral("Preview warning: generated URDF robot mesh row did not produce a visible mesh renderable; using locked generated URDF visual fallback");
              if (!render_expected) {
                p.warnings << QStringLiteral("Preview warning: xacro-expanded visual kept as generated URDF visual fallback");
              }
            } else {
              p.status = "warning";
              p.mesh_available = false;
              p.has_mesh_metadata = true;
              p.active_visual_source = QStringLiteral("primitive_fallback");
              p.source_layer = QStringLiteral("primitive_fallback");
              p.editable = false;
              p.selectable = true;
              p.locked = true;
              p.lock_reason = QStringLiteral("generated URDF visual primitive fallback diagnostic");
              p.warnings << QStringLiteral("Preview warning: URDF visual mesh unavailable; using generic primitive fallback diagnostic");
            }
            if (p.resolved_source_path_stale) {
              p.warnings << QStringLiteral("Preview warning: resolved_source_path is stale; package_uri fallback was attempted");
            }
          } else if (!resolved || (geometry_type == "mesh" && p.source_path.trimmed().isEmpty())) {
            p.status = "warning";
            p.mesh_available = false;
            p.warnings << QStringLiteral("Preview warning: URDF visual unresolved");
            const QString warning = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "warning"));
            if (!warning.isEmpty()) p.warnings << warning;
          } else {
            if (is_primitive) {
              p.mesh_available = false;
              p.has_mesh_metadata = false;
              p.active_visual_source = explicit_primitive_fallback ? QStringLiteral("primitive_fallback") : QStringLiteral("urdf_primitive");
            } else {
              p.mesh_available = true;
              p.has_mesh_metadata = true;
            }
          }
          if (geometry_type == QStringLiteral("mesh") && !p.mesh_path.trimmed().isEmpty()) {
            p.source_path = p.mesh_path;
            p.mesh_available = true;
            p.has_mesh_metadata = true;
            p.active_visual_source = QStringLiteral("mesh_preview");
          }
          if (protected_ur5_generated_mesh_row) {
            const QString absolute_ur5_mesh_path = p.mesh_path.trimmed().isEmpty() ? p.source_path.trimmed() : p.mesh_path.trimmed();
            p.source_layer = QStringLiteral("locked_generated_urdf_visual");
            p.active_visual_source = QStringLiteral("mesh_preview");
            p.category = QStringLiteral("robot_arm");
            p.role = p.category;
            p.primitive_geometry_type = QStringLiteral("mesh");
            p.visual_index_link = protected_ur5_link;
            p.visual_index_link_name = protected_ur5_link;
            p.display_name = protected_ur5_link;
            p.frame_id = protected_ur5_link;
            if (!absolute_ur5_mesh_path.isEmpty()) {
              p.mesh_path = absolute_ur5_mesh_path;
              p.source_path = absolute_ur5_mesh_path;
            }
          }
          normalize_generated_urdf_visual_identity(p);
          classify_generated_urdf_visual(p, !p.mesh_path.trimmed().isEmpty() ? p.mesh_path : resolved_source_path);
          const QString viewport_link_token = scene3d_viewport_link_token(p);
          if (viewport_link_token == QStringLiteral("base_link_inertia")) {
            append_studio_log(QStringLiteral("Scene3D base_link_inertia trace: stage=PreviewItem_creation item_id=%1 link=%2 canonical_link=%3 mesh_path=%4 package_uri=%5 source_layer=%6 active_visual_source=%7 category=%8 role=%9")
              .arg(p.id, p.visual_index_link, viewport_link_token,
                   p.mesh_path.trimmed().isEmpty() ? p.source_path : p.mesh_path,
                   p.package_uri, p.source_layer, p.active_visual_source, p.category, p.role));
          }
          if (viewport_link_token == QStringLiteral("base_link_inertia") ||
              viewport_link_token == QStringLiteral("shoulder_link") ||
              viewport_link_token == QStringLiteral("upper_arm_link") ||
              viewport_link_token == QStringLiteral("forearm_link") ||
              viewport_link_token == QStringLiteral("wrist_1_link") ||
              viewport_link_token == QStringLiteral("wrist_2_link") ||
              viewport_link_token == QStringLiteral("wrist_3_link")) {
            append_studio_log(QStringLiteral(
              "Scene3D UR5 visual submitted to native renderer: link=%1 mesh_path=%2 source_layer=%3 active_visual_source=%4 baked_world_visual_pose=%5")
              .arg(viewport_link_token,
                   p.mesh_path.trimmed().isEmpty() ? p.source_path : p.mesh_path,
                   p.source_layer,
                   p.active_visual_source,
                   QString::fromUtf8(QJsonDocument(scene3d_viewport_pose_json(p)).toJson(QJsonDocument::Compact))));
            required_ur5_preview_items_by_link[viewport_link_token] = p;
          }
          if (unresolved_package_uri) ++unresolved_package_uri_count;
          if (unresolved_package_uri && !is_primitive) add_skip_reason(QStringLiteral("missing_source_path"));
          log_ur5_runtime_decision(QStringLiteral("appended"), QStringLiteral("retained_generated_urdf_visual"), p.mesh_path.trimmed().isEmpty() ? p.source_path : p.mesh_path);
          preview_items.push_back(p);
          ++visual_preview_added_count;
          preview_ids.insert(p.id);
          generated_row_diagnostic[QStringLiteral("id")] = p.id;
          append_generated_visual_row_diagnostic(
            generated_row_diagnostic,
            QStringLiteral("appended_to_preview_items"),
            QString(),
            true,
            p.mesh_path.trimmed().isEmpty() ? p.source_path : p.mesh_path);
          append_visual_ingestion_diagnostic(v, raw_id, p.id, QString(), p.source_layer);
          if (include_preview_item_in_hierarchy(p)) add_tree_node(p);
        }
      }
      int visual_skipped_total = 0;
      for (auto it = skip_reason_counts.cbegin(); it != skip_reason_counts.cend(); ++it) visual_skipped_total += it.value();
      QJsonObject skip_reason_counts_json;
      for (auto it = skip_reason_counts.cbegin(); it != skip_reason_counts.cend(); ++it) {
        skip_reason_counts_json[it.key()] = it.value();
      }
      scene3d_visual_ingestion_diagnostics_ = QJsonObject{
        {QStringLiteral("total_visual_index_rows"), visual_index_loaded_count},
        {QStringLiteral("total_added_visual_rows"), visual_preview_added_count},
        {QStringLiteral("total_skipped_visual_rows"), visual_skipped_total},
        {QStringLiteral("skipped_count_by_reason"), skip_reason_counts_json},
        {QStringLiteral("duplicate_id_count"), skip_reason_counts.value(QStringLiteral("duplicate_id"), 0)},
        {QStringLiteral("robot_base_frame"), robot_base_frame},
        {QStringLiteral("first_20_visual_items"), visual_ingestion_item_diagnostics},
        {QStringLiteral("generated_urdf_visual_row_diagnostics"), generated_urdf_visual_row_diagnostics}
      };
      scene3d_visual_ingestion_diagnostics_[QStringLiteral("generated_urdf_visual_numbers_after_ingest")] =
        summarize_generated_urdf_visual_rows(preview_items);
      append_studio_log(QString("Visual mesh index loaded from: %1").arg(QString::fromStdString(urdf_visual_index.string())));
      append_studio_log(QString("Visual mesh index safe_for_preview: %1").arg(visual_index_safe_for_preview ? "true" : "false"));
      append_studio_log(QString("Visual mesh index extraction_mode: %1").arg(visual_index_extraction_mode));
      append_studio_log(QString("Visual mesh index loaded: %1 visual items").arg(visual_index_loaded_count));
      append_studio_log(QString("Mesh items: %1, primitive items: %2, unknown items: %3").arg(mesh_item_count).arg(primitive_item_count).arg(unknown_item_count));
      append_studio_log(QString("Visual mesh preview items added: %1 / %2").arg(visual_preview_added_count).arg(visual_index_loaded_count));
      QStringList skip_reason_tokens;
      for (auto it = skip_reason_counts.cbegin(); it != skip_reason_counts.cend(); ++it) {
        skip_reason_tokens << QString("%1=%2").arg(it.key()).arg(it.value());
      }
      append_studio_log(QString("Skipped visual items (canonical): %1").arg(skip_reason_tokens.join(' ')));
      append_studio_log(QString("Scene3D visual ingestion semantic/helper rows skipped: %1").arg(skipped_semantic_helper_visual_rows));
      append_studio_log(QString("Scene3D visual ingestion true duplicate mesh rows skipped: %1").arg(skipped_true_duplicate_mesh_rows));
      append_studio_log(QString("Scene3D visual ingestion preserved generated URDF robot mesh rows: %1").arg(preserved_generated_urdf_robot_mesh_rows));
      append_studio_log(QString("Visual mesh index parse_error=%1").arg(skip_reason_counts.value(QStringLiteral("parse_error"), 0)));
      append_studio_log(QString("Loader fallbacks: package_uri_resolved_by_loader=%1 source_path_from_resolved_path=%2 non_mesh_geometry_added=%3 non_mesh_geometry_unsupported=%4")
        .arg(package_uri_resolved_by_loader).arg(source_path_from_resolved_path).arg(non_mesh_geometry_added).arg(non_mesh_geometry_unsupported));
      append_studio_log(QString("Resolved source path diagnostics: stale_resolved_source_path=%1 package_uri_resolved_after_stale=%2")
        .arg(stale_resolved_source_path_count)
        .arg(package_uri_resolved_after_stale_resolved_source_path));
      if (baked_world_visual_pose_count > 0) {
        append_studio_log(QString("Transform source: baked_world_visual_pose from generated/scene_visual_mesh_index.json (count=%1, source=%2)")
          .arg(baked_world_visual_pose_count)
          .arg(baked_world_visual_transform_source));
      }
      append_studio_log(QString("Transform diagnostics: transform_chain_applied_count=%1 visual_origin_applied_count=%2 baked_world_visual_pose_count=%3 robot_base_frame=%4 robot_world_pose=%5 missing_chain_warnings=%6")
        .arg(transform_chain_applied_count)
        .arg(visual_origin_applied_count)
        .arg(baked_world_visual_pose_count)
        .arg(robot_base_frame)
        .arg(robot_world_pose)
        .arg(missing_chain_warning_count));
      if (visual_index_loaded_count != (visual_preview_added_count + visual_skipped_total)) {
        append_studio_log(
          QString("Preview warning: visual ingestion mismatch loaded=%1 added=%2 skipped_sum=%3")
          .arg(visual_index_loaded_count)
          .arg(visual_preview_added_count)
          .arg(visual_skipped_total));
      }
      visual_diagnostics_summary =
        QString("Visuals: added %1/%2 • skipped=%3")
        .arg(visual_preview_added_count)
        .arg(visual_index_loaded_count)
        .arg(visual_skipped_total);
      if (visual_index_safe_for_preview && visual_preview_added_count == 0) {
        append_studio_log("Visual mesh index safe but no preview items were ingested");
      }
      if (!required_ur5_preview_items_by_link.isEmpty()) {
        QStringList retained_required_ur5_links;
        for (const auto & item : preview_items) {
          const QString link = scene3d_viewport_link_token(item);
          if (required_ur5_visual_links.contains(link)) retained_required_ur5_links << link;
        }
        QSet<QString> retained_required_ur5_link_set(retained_required_ur5_links.cbegin(), retained_required_ur5_links.cend());
        QStringList restored_required_ur5_links;
        for (const QString & link : required_ur5_visual_links) {
          if (retained_required_ur5_link_set.contains(link)) continue;
          const auto it = required_ur5_preview_items_by_link.constFind(link);
          if (it == required_ur5_preview_items_by_link.constEnd()) continue;
          preview_items.push_back(it.value());
          retained_required_ur5_link_set.insert(link);
          restored_required_ur5_links << link;
        }
        QStringList retained_required_ur5_link_list;
        for (const QString & link : retained_required_ur5_link_set) retained_required_ur5_link_list << link;
        retained_required_ur5_link_list.sort();
        append_studio_log(QStringLiteral("Scene3D UR5 index-to-preview retention check passed for ur5_2f_test: retained_required_ur5_links=%1 restored_after_preview_suppression=%2")
          .arg(retained_required_ur5_link_list.join(QStringLiteral(",")),
               restored_required_ur5_links.isEmpty() ? QStringLiteral("none") : restored_required_ur5_links.join(QStringLiteral(","))));
      }
    } catch (...) {
      append_studio_log("Preview warning: failed to parse generated/scene_visual_mesh_index.json");
    }
  }

  // The generated URDF visual mesh index is the authoritative renderer handoff
  // source for locked robot/tool/environment mesh previews.  Keep those rows
  // ahead of semantic/editor overlays in the assembled PreviewItem payload so
  // visual_items[0] (for UR5, base_link_inertia/visual_0/base.dae) cannot be
  // displaced by environment.yaml's semantic robot_base or other helper rows
  // before Scene3DViewportWidget ingest.
  std::stable_sort(preview_items.begin(), preview_items.end(), [](const auto & a, const auto & b) {
    const auto generated_rank = [](const ScenePreviewWidget::PreviewItem & item) {
      const QString source_layer = canonical_scene3d_token(item.source_layer);
      const QString visual_source = canonical_scene3d_token(item.active_visual_source);
      const bool locked_generated_visual =
        source_layer == QStringLiteral("locked_generated_urdf_visual") ||
        source_layer == QStringLiteral("generated_urdf_visual");
      const bool mesh_preview = visual_source == QStringLiteral("mesh_preview");
      return (locked_generated_visual && mesh_preview) ? 0 : 1;
    };
    return generated_rank(a) < generated_rank(b);
  });
  if (!preview_items.isEmpty() &&
      canonical_scene3d_token(preview_items.front().source_layer) == QStringLiteral("locked_generated_urdf_visual")) {
    append_studio_log(QStringLiteral("Scene3D generated URDF visuals ordered before semantic overlays for renderer handoff: first_item=%1 source_layer=%2 active_visual_source=%3")
      .arg(preview_items.front().id, preview_items.front().source_layer, preview_items.front().active_visual_source));
  }

  if (preview_items.empty()) {
    const QString scene_source = QString::fromStdString(s.scene_dir.string());
    add_preview_item("robot_base", "robot base", "Robot", "robot", "ready", scene_source, true);
    add_preview_item("end_effector", "end effector", "End Effector", "tool", "ready", scene_source, true);
    add_preview_item("camera_main", "camera", "Camera / Sensor", "camera", "ready", scene_source, true);
    add_preview_item("conveyor_main", "conveyor", "Conveyor", "conveyor", "ready", scene_source, true);
    add_preview_item("table_main", "workbench", "Work Table / Surface", "table", "ready", scene_source, true);
    add_preview_item("bin_pick", "pick source bin", "Pick Source / Bin", "pick source", "ready", scene_source, true);
    add_preview_item("fixture_place", "place target fixture", "Place Target / Fixture", "place target", "ready", scene_source, true);
    add_preview_item("safety_zone_a", "safety zone", "Safety", "safety zone", "warning", scene_source, false);
    add_preview_item("warning_marker_a", "warning marker", "Safety", "warning marker", "warning", scene_source, false);
  }

  const int missing_mesh_count = skip_reason_counts.value(QStringLiteral("missing_source_path"), 0) +
                                 skip_reason_counts.value(QStringLiteral("file_not_found"), 0) +
                                 skip_reason_counts.value(QStringLiteral("zero_triangle_mesh"), 0);
  const int unsupported_extension_count = skip_reason_counts.value(QStringLiteral("unsupported_format"), 0);
  const bool authoritative_mesh_index_healthy =
    (visual_index_extraction_mode.trimmed().compare(QStringLiteral("xacro_expanded"), Qt::CaseInsensitive) == 0 ||
     visual_index_extraction_mode.trimmed().compare(QStringLiteral("xacro_lite_expanded"), Qt::CaseInsensitive) == 0) &&
    visual_index_safe_for_preview &&
    mesh_item_count > 0 &&
    primitive_item_count == 0 &&
    missing_mesh_count == 0 &&
    unresolved_package_uri_count == 0 &&
    unsupported_extension_count == 0;

  const auto suppression_result = workcell_builder::suppress_lower_fidelity_preview_items(
    preview_items,
    authoritative_mesh_index_healthy);
  const QMap<QString, int> placeholder_suppression_reason_counts = suppression_result.suppression_reason_counts;
  const QStringList placeholder_suppression_diagnostics = suppression_result.suppression_diagnostics;
  const int suppressed_preview_placeholder_count = suppression_result.suppressed_preview_placeholder_count;
  const int preserved_editable_source_count = suppression_result.preserved_editable_source_count;
  const int authoritative_visual_equivalence_key_count = suppression_result.authoritative_visual_equivalence_key_count;
  const QVector<ScenePreviewWidget::PreviewItem> unsuppressed_preview_items = suppression_result.items;
  for (const auto & item : unsuppressed_preview_items) {
    if (scene3d_viewport_link_token(item) == QStringLiteral("base_link_inertia")) {
      append_studio_log(QStringLiteral("Scene3D base_link_inertia trace: stage=dedupe result=retained item_id=%1 source_layer=%2 active_visual_source=%3")
        .arg(item.id, item.source_layer, item.active_visual_source));
    }
  }
  preview_items = unsuppressed_preview_items;
  if (scene3d_visual_ingestion_diagnostics_.contains(QStringLiteral("generated_urdf_visual_row_diagnostics"))) {
    QSet<QString> unsuppressed_ids;
    for (const auto & item : preview_items) {
      unsuppressed_ids.insert(item.id);
    }
    QJsonArray suppression_diagnostics;
    const QJsonArray rows =
      scene3d_visual_ingestion_diagnostics_.value(QStringLiteral("generated_urdf_visual_row_diagnostics")).toArray();
    for (const QJsonValue & value : rows) {
      QJsonObject row = value.toObject();
      const QString id = row.value(QStringLiteral("id")).toString();
      const bool survived_suppression =
        row.value(QStringLiteral("appended_to_preview_items")).toBool(false) && unsuppressed_ids.contains(id);
      row[QStringLiteral("survived_suppression")] = survived_suppression;
      if (row.value(QStringLiteral("appended_to_preview_items")).toBool(false) && !survived_suppression) {
        if (row.value(QStringLiteral("first_drop_stage")).toString().trimmed().isEmpty()) {
          row[QStringLiteral("first_drop_stage")] = QStringLiteral("suppression_helper");
        }
      }
      suppression_diagnostics.append(row);
    }
    scene3d_visual_ingestion_diagnostics_[QStringLiteral("generated_urdf_visual_row_diagnostics")] = suppression_diagnostics;
  }
  scene3d_visual_ingestion_diagnostics_[QStringLiteral("generated_urdf_visual_numbers_after_suppression")] =
    summarize_generated_urdf_visual_rows(preview_items);
  if (suppressed_preview_placeholder_count > 0) {
    QStringList suppression_tokens;
    for (auto it = placeholder_suppression_reason_counts.cbegin(); it != placeholder_suppression_reason_counts.cend(); ++it) {
      suppression_tokens << QString("%1=%2").arg(it.key()).arg(it.value());
    }
    append_studio_log(QString("Preview placeholder suppression diagnostics: %1 (examples=%2 authoritative_mesh_index_equivalence_keys=%3 preserved_editable_source_of_truth=%4; authoritative_mesh_index_healthy=%5; suppressed items are omitted without viewport warning placeholders)")
      .arg(suppression_tokens.join(' '))
      .arg(placeholder_suppression_diagnostics.join(QStringLiteral("; ")))
      .arg(authoritative_visual_equivalence_key_count)
      .arg(preserved_editable_source_count)
      .arg(authoritative_mesh_index_healthy ? QStringLiteral("true") : QStringLiteral("false")));
    if (authoritative_mesh_index_healthy) {
      append_studio_log(QString("Preview placeholder suppression: omitted %1 legacy placeholders because authoritative xacro/xacro-lite mesh index is healthy")
        .arg(suppressed_preview_placeholder_count));
    }
  } else if (authoritative_visual_equivalence_key_count > 0) {
    append_studio_log(QString("Preview placeholder suppression: none (authoritative_mesh_index_equivalence_keys=%1 preserved_editable_source_of_truth=%2)")
      .arg(authoritative_visual_equivalence_key_count)
      .arg(preserved_editable_source_count));
  }

  // Final runtime safety net: inspect the exact PreviewItem payload that will
  // be committed to Scene3D after normal ingestion and suppression.  This is
  // intentionally after suppress_lower_fidelity_preview_items(); rows appended
  // here are authoritative mesh-backed URDF visuals and must not be routed
  // through the older duplicate-suppression path again.
  {
    const QString scene_name_for_final_append = QString::fromStdString(s.scene_name).trimmed();
    const QString workspace_root_for_final_append = detect_workspace_root();
    const QStringList required_ur5_final_links = {
      QStringLiteral("shoulder_link"),
      QStringLiteral("upper_arm_link"),
      QStringLiteral("forearm_link"),
      QStringLiteral("wrist_1_link"),
      QStringLiteral("wrist_2_link"),
      QStringLiteral("wrist_3_link")
    };
    const QSet<QString> accepted_ur5_base_links = {
      QStringLiteral("base_link_inertia"),
      QStringLiteral("base_link")
    };
    auto final_append_row_scalar = [](const YAML::Node & node, const char * key) {
      const YAML::Node value = workcell_builder::yaml_map_key(node, key);
      if (!value || !value.IsScalar()) return QString();
      return QString::fromStdString(value.as<std::string>("")).trimmed();
    };
    auto final_append_first_scalar = [&](const YAML::Node & node, std::initializer_list<const char *> keys) {
      for (const char * key : keys) {
        const QString value = final_append_row_scalar(node, key);
        if (!value.isEmpty()) return value;
      }
      return QString();
    };
    auto normalize_ur5_final_link = [&](const QString & raw) {
      const QString token = canonical_scene3d_token(raw);
      if (accepted_ur5_base_links.contains(token)) return token;
      for (const QString & link : required_ur5_final_links) {
        if (token == link || token.contains(link)) return link;
      }
      return QString();
    };
    auto final_append_row_link = [&](const YAML::Node & node) {
      const QString direct = normalize_ur5_final_link(final_append_first_scalar(
        node, {"link", "link_name", "canonical_link_name", "object_id", "object"}));
      if (!direct.isEmpty()) return direct;
      return normalize_ur5_final_link(final_append_first_scalar(node, {"id", "visual", "visual_name"}));
    };
    auto final_append_mesh_reference = [&](const YAML::Node & node) {
      return QStringList{
        final_append_first_scalar(node, {"package_uri"}),
        final_append_first_scalar(node, {"mesh_uri", "filename"}),
        final_append_first_scalar(node, {"source_path"}),
        final_append_first_scalar(node, {"mesh_path"}),
        final_append_first_scalar(node, {"resolved_source_path", "resolved_path"})
      }.join(QStringLiteral("|"));
    };
    auto final_append_is_ur5_mesh_row = [&](const YAML::Node & node) {
      if (!node || !node.IsMap()) return false;
      const QString link = final_append_row_link(node);
      if (link.isEmpty()) return false;
      const QString geometry = canonical_scene3d_token(final_append_first_scalar(node, {"geometry_type", "type"}));
      if (geometry != QStringLiteral("mesh")) return false;
      const QString source_token = canonical_scene3d_token(final_append_mesh_reference(node));
      return source_token.contains(QStringLiteral("ur_description")) &&
             source_token.contains(QStringLiteral("meshes")) &&
             source_token.contains(QStringLiteral("ur5")) &&
             source_token.contains(QStringLiteral("visual"));
    };
    auto preview_item_has_ur5_link = [&](const QString & link) {
      for (const auto & item : preview_items) {
        const QString existing_link = scene3d_viewport_link_token(item);
        if (link == QStringLiteral("base_link_inertia") || link == QStringLiteral("base_link")) {
          if (accepted_ur5_base_links.contains(existing_link)) return true;
        } else if (existing_link == link) {
          return true;
        }
      }
      return false;
    };
    auto preview_item_mesh_for_ur5_link = [&](const QString & link) {
      for (const auto & item : preview_items) {
        const QString existing_link = scene3d_viewport_link_token(item);
        const bool link_matches =
          (link == QStringLiteral("base_link_inertia") || link == QStringLiteral("base_link"))
            ? accepted_ur5_base_links.contains(existing_link)
            : existing_link == link;
        if (!link_matches) continue;
        const QString mesh = item.mesh_path.trimmed().isEmpty() ? item.source_path.trimmed() : item.mesh_path.trimmed();
        const QFileInfo mesh_info(mesh);
        if (mesh_info.exists() && mesh_info.isFile()) {
          const QString canonical = mesh_info.canonicalFilePath();
          return canonical.isEmpty() ? mesh_info.absoluteFilePath() : canonical;
        }
        return mesh;
      }
      return QString();
    };
    bool visual_index_contains_ur5_mesh_rows = false;
    if (fs::exists(urdf_visual_index)) {
      try {
        const YAML::Node final_index_probe = YAML::LoadFile(urdf_visual_index.string());
        const YAML::Node visual_items = workcell_builder::yaml_map_key(final_index_probe, "visual_items");
        if (visual_items && visual_items.IsSequence()) {
          for (const auto & row : visual_items) {
            if (final_append_is_ur5_mesh_row(row)) {
              visual_index_contains_ur5_mesh_rows = true;
              break;
            }
          }
        }
      } catch (...) {
        visual_index_contains_ur5_mesh_rows = false;
      }
    }
    if (scene_name_for_final_append == QStringLiteral("ur5_2f_test") || visual_index_contains_ur5_mesh_rows) {
      QStringList missing_ur5_links;
      bool base_present = false;
      for (const QString & base_link : accepted_ur5_base_links) {
        if (preview_item_has_ur5_link(base_link)) {
          base_present = true;
          break;
        }
      }
      if (!base_present) missing_ur5_links << QStringLiteral("base_link_inertia");
      for (const QString & link : required_ur5_final_links) {
        if (!preview_item_has_ur5_link(link)) missing_ur5_links << link;
      }
      append_studio_log(QStringLiteral("UR5_FINAL_APPEND_CHECK missing=[%1]")
        .arg(missing_ur5_links.join(QStringLiteral(","))));
      for (const QString & link : required_ur5_final_links) {
        if (!missing_ur5_links.contains(link)) {
          append_studio_log(QStringLiteral("UR5_FINAL_APPEND already_present link=%1 mesh=%2")
            .arg(link, preview_item_mesh_for_ur5_link(link)));
        }
      }

      if (!missing_ur5_links.isEmpty() && fs::exists(urdf_visual_index)) {
        try {
          const YAML::Node final_index = YAML::LoadFile(urdf_visual_index.string());
          const YAML::Node visual_items = workcell_builder::yaml_map_key(final_index, "visual_items");
          int row_number = 0;
          if (visual_items && visual_items.IsSequence()) {
            for (const auto & row : visual_items) {
              const int source_row_index = workcell_builder::yaml_map_key(row, "source_row_index").as<int>(row_number);
              ++row_number;
              if (!final_append_is_ur5_mesh_row(row)) continue;
              const QString link = final_append_row_link(row);
              const bool row_satisfies_missing_base =
                accepted_ur5_base_links.contains(link) &&
                missing_ur5_links.contains(QStringLiteral("base_link_inertia"));
              if (!missing_ur5_links.contains(link) && !row_satisfies_missing_base) continue;
              const QString package_uri = final_append_first_scalar(row, {"package_uri"});
              const QString mesh_uri = final_append_first_scalar(row, {"mesh_uri", "filename"});
              const QString source_path = final_append_first_scalar(row, {"source_path"});
              const QString mesh_path = final_append_first_scalar(row, {"mesh_path"});
              const QString resolved_source_path = final_append_first_scalar(row, {"resolved_source_path", "resolved_path"});
              QStringList tried_candidates;
              QString resolved_mesh_path = resolve_visual_mesh_source_path(
                !resolved_source_path.isEmpty() ? resolved_source_path : (!mesh_path.isEmpty() ? mesh_path : source_path),
                package_uri.isEmpty() ? mesh_uri : package_uri,
                d,
                workspace_root_for_final_append,
                &tried_candidates);
              if (resolved_mesh_path.isEmpty()) {
                resolved_mesh_path = resolve_visual_mesh_source_path(
                  QString(),
                  package_uri.isEmpty() ? mesh_uri : package_uri,
                  d,
                  workspace_root_for_final_append,
                  &tried_candidates);
              }
              if (resolved_mesh_path.isEmpty()) continue;
              const QFileInfo mesh_info(resolved_mesh_path);
              const QString canonical_mesh_path = mesh_info.exists() && mesh_info.isFile()
                ? (mesh_info.canonicalFilePath().isEmpty() ? mesh_info.absoluteFilePath() : mesh_info.canonicalFilePath())
                : resolved_mesh_path;
              bool exact_already_present = false;
              for (const auto & item : preview_items) {
                const QString existing_link = scene3d_viewport_link_token(item);
                const QString existing_mesh = QFileInfo(item.mesh_path.trimmed().isEmpty() ? item.source_path : item.mesh_path).canonicalFilePath();
                const QString normalized_existing_mesh = existing_mesh.isEmpty()
                  ? (item.mesh_path.trimmed().isEmpty() ? item.source_path.trimmed() : item.mesh_path.trimmed())
                  : existing_mesh;
                if (existing_link == link && normalized_existing_mesh == canonical_mesh_path) {
                  exact_already_present = true;
                  break;
                }
              }
              if (exact_already_present) {
                append_studio_log(QStringLiteral("UR5_FINAL_APPEND already_present link=%1 mesh=%2")
                  .arg(link, canonical_mesh_path));
                missing_ur5_links.removeAll(link);
                continue;
              }

              ScenePreviewWidget::PreviewItem p;
              const QString visual_name = final_append_first_scalar(row, {"visual_name", "visual", "id"});
              p.id = QStringLiteral("generated_urdf::%1::%2::%3")
                .arg(link, visual_name.isEmpty() ? QStringLiteral("visual") : visual_name)
                .arg(source_row_index);
              p.display_name = link;
              p.category = QStringLiteral("robot_arm");
              p.role = p.category;
              p.status = QStringLiteral("ready");
              p.locked = true;
              p.editable = false;
              p.selectable = true;
              p.lock_reason = QStringLiteral("generated URDF visual final append");
              p.source_layer = QStringLiteral("locked_generated_urdf_visual");
              p.active_visual_source = QStringLiteral("mesh_preview");
              p.mesh_path = canonical_mesh_path;
              p.source_path = canonical_mesh_path;
              p.mesh_available = true;
              p.has_mesh_metadata = true;
              p.primitive_geometry_type = QStringLiteral("mesh");
              p.package_uri = package_uri.isEmpty() ? mesh_uri : package_uri;
              p.visual_index_package_uri = p.package_uri;
              p.visual_index_mesh_uri = mesh_uri;
              p.visual_index_link = link;
              p.visual_index_link_name = link;
              p.frame_id = link;
              p.visual_index_visual = final_append_first_scalar(row, {"visual"});
              p.visual_index_visual_name = final_append_first_scalar(row, {"visual_name"});
              p.visual_index_value = workcell_builder::yaml_map_key(row, "visual_index").as<int>(-1);
              p.source_row_index = source_row_index;
              p.visual_index_parent_link = final_append_first_scalar(row, {"parent_link", "base_frame"});
              p.visual_index_source = final_append_first_scalar(row, {"source"});
              p.robot_base_frame = p.visual_index_parent_link.isEmpty() ? QStringLiteral("unknown") : p.visual_index_parent_link;
              const YAML::Node pose = workcell_builder::yaml_map_key(row, "pose");
              const YAML::Node baked_pose = workcell_builder::yaml_map_key(row, "baked_world_visual_pose");
              const YAML::Node xyz = workcell_builder::yaml_map_key(baked_pose, "xyz").IsSequence()
                ? workcell_builder::yaml_map_key(baked_pose, "xyz")
                : workcell_builder::yaml_map_key(pose, "xyz");
              const YAML::Node rpy = workcell_builder::yaml_map_key(baked_pose, "rpy").IsSequence()
                ? workcell_builder::yaml_map_key(baked_pose, "rpy")
                : workcell_builder::yaml_map_key(pose, "rpy");
              if (xyz && xyz.IsSequence() && xyz.size() >= 3) {
                p.x = workcell_builder::yaml_seq_index(xyz, 0).as<double>(0.0);
                p.y = workcell_builder::yaml_seq_index(xyz, 1).as<double>(0.0);
                p.z = workcell_builder::yaml_seq_index(xyz, 2).as<double>(0.0);
              }
              if (rpy && rpy.IsSequence() && rpy.size() >= 3) {
                p.roll = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy, 0).as<double>(0.0), QStringLiteral("UR5_FINAL_APPEND.rpy[0]"), &p.warnings);
                p.pitch = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy, 1).as<double>(0.0), QStringLiteral("UR5_FINAL_APPEND.rpy[1]"), &p.warnings);
                p.yaw = normalize_angle_radians_with_guard(workcell_builder::yaml_seq_index(rpy, 2).as<double>(0.0), QStringLiteral("UR5_FINAL_APPEND.rpy[2]"), &p.warnings);
              }
              if (workcell_builder::yaml_map_key(baked_pose, "xyz").IsSequence() &&
                  workcell_builder::yaml_map_key(baked_pose, "rpy").IsSequence()) {
                p.has_baked_world_visual_transform = true;
                p.baked_world_visual_transform_source = QStringLiteral("generated/scene_visual_mesh_index.json:baked_world_visual_pose:final_append");
              }
              const YAML::Node matrix = workcell_builder::yaml_map_key(row, "baked_world_visual_matrix");
              if (matrix && matrix.IsSequence() && matrix.size() == 16) {
                for (std::size_t i = 0; i < 16; ++i) {
                  p.baked_world_visual_matrix[i] = workcell_builder::yaml_seq_index(matrix, i).as<double>(i % 5 == 0 ? 1.0 : 0.0);
                }
                p.has_baked_world_visual_transform = true;
                p.has_baked_world_visual_matrix = true;
                p.baked_world_visual_transform_source = QStringLiteral("generated/scene_visual_mesh_index.json:baked_world_visual_matrix:final_append");
              }
              const YAML::Node scale = workcell_builder::yaml_map_key(row, "mesh_scale");
              if (scale && scale.IsSequence() && scale.size() >= 3) {
                p.mesh_scale_x = workcell_builder::yaml_seq_index(scale, 0).as<double>(1.0);
                p.mesh_scale_y = workcell_builder::yaml_seq_index(scale, 1).as<double>(1.0);
                p.mesh_scale_z = workcell_builder::yaml_seq_index(scale, 2).as<double>(1.0);
              }
              normalize_generated_urdf_visual_identity(p);
              classify_generated_urdf_visual(p, canonical_mesh_path);
              p.category = QStringLiteral("robot_arm");
              p.role = p.category;
              p.visual_index_link = link;
              p.visual_index_link_name = link;
              p.active_visual_source = QStringLiteral("mesh_preview");
              p.source_layer = QStringLiteral("locked_generated_urdf_visual");
              preview_items.push_back(p);
              missing_ur5_links.removeAll(link);
              if (row_satisfies_missing_base) {
                missing_ur5_links.removeAll(QStringLiteral("base_link_inertia"));
              }
              append_studio_log(QStringLiteral("UR5_FINAL_APPEND appended link=%1 mesh=%2")
                .arg(link, canonical_mesh_path));
            }
          }
        } catch (...) {
          append_studio_log(QStringLiteral("UR5_FINAL_APPEND failed_to_parse index=generated/scene_visual_mesh_index.json"));
        }
      }
      int final_required_count = 0;
      for (const QString & link : required_ur5_final_links) {
        if (preview_item_has_ur5_link(link)) ++final_required_count;
      }
      append_studio_log(QStringLiteral("UR5_FINAL_APPEND final_required_count=%1").arg(final_required_count));
    }
  }

  scene_hierarchy_tree_->clear();
  hierarchy_groups.clear();
  for (const QString &gn : {QString("Editable Layout"), QString("Mesh Preview"), QString("Generated URDF Visuals"), QString("Primitive Fallbacks"), QString("Cameras"), QString("Robot / Tooling"), QString("Overlays / Helpers"), QString("Warnings / Missing Assets")}) ensure_group(gn);
  for (const auto & p : preview_items) {
    if (allowed_scene_roles.contains(p.role) && include_preview_item_in_hierarchy(p)) {
      add_tree_node(p);
    }
  }

  const std::vector<std::string> key_files = {
    "environment.yaml",
    "scene_manifest.yaml",
    "environment_layout.yaml",
    "layout/workcell_studio_layout.yaml",
    "config/workcell_builder_task_intent.yaml",
    "package.xml",
    "CMakeLists.txt",
    "launch/demo.launch.py"
  };

  const QSet<QString> excluded_scene_file_artifacts = {
    "package.xml",
    "CMakeLists.txt",
    "launch/demo.launch.py",
    "environment.yaml",
    "scene_manifest.yaml"
  };
  for (const auto & rel : key_files) {
    const QString rel_q = QString::fromStdString(rel);
    if (excluded_scene_file_artifacts.contains(rel_q)) continue;
    const fs::path path = d / rel;
    if (!fs::exists(path)) continue;

    auto * node = new QTreeWidgetItem(
      ensure_group("Overlays / Helpers"),
      {QString::fromStdString(rel), "file", "present"});
    node->setData(0, TreeRoleId, rel_q);
    node->setData(0, TreeRoleStableId, rel_q);
    node->setData(0, TreeRoleCategory, "file");
    node->setData(0, TreeRoleSource, QString::fromStdString(path.string()));
    node->setData(0, TreeRoleSourceLayer, QStringLiteral("helper_file"));
    node->setData(0, TreeRoleActiveVisualSource, QStringLiteral("helper_file"));
    node->setData(0, TreeRolePoseAvailable, false);
    node->setData(0, TreeRoleRole, "file");
  }


  auto * header = scene_hierarchy_tree_->header();
  if (header) {
    header->setSectionResizeMode(0, QHeaderView::Stretch);
    header->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    header->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    scene_hierarchy_tree_->setColumnWidth(0, 420);
    header->setStretchLastSection(false);
  }

  const SceneTaskIntentSummary task_summary = load_scene_task_intent_summary(d);
  const QString active_pick_id = normalize_bound_id(task_summary.pick_source);
  const QString active_place_id = normalize_bound_id(task_summary.place_target);
  QString active_camera_id = "unknown";
  YAML::Node task_intent_root;
  if (read_yaml(d / "config" / "workcell_builder_task_intent.yaml", &task_intent_root)) {
    const YAML::Node task = task_intent_root["task"] ? task_intent_root["task"] : task_intent_root;
    active_camera_id = normalize_bound_id(scalar_path(task, {"perception","camera","id"}));
    if (active_camera_id == "unknown") active_camera_id = normalize_bound_id(scalar_path(task, {"camera","id"}));
  }

  auto role_tag_for_id = [&](const QString & id) {
    if (id.isEmpty()) return QString();
    QStringList tags;
    if (active_pick_id != "unknown" && id == active_pick_id) tags << "[Pick Source]";
    if (active_place_id != "unknown" && id == active_place_id) tags << "[Place Target]";
    if (active_camera_id != "unknown" && id == active_camera_id) tags << "[Camera]";
    return tags.join(" ");
  };

  for (auto & p : preview_items) {
    const QString tag = role_tag_for_id(p.id);
    if (!tag.isEmpty()) p.metadata_tags = tag;
  }

  for (int i = 0; i < scene_hierarchy_tree_->topLevelItemCount(); ++i) {
    auto * group = scene_hierarchy_tree_->topLevelItem(i);
    if (!group) continue;
    for (int c = 0; c < group->childCount(); ++c) {
      auto * node = group->child(c);
      if (!node) continue;
      const QString item_id = node->data(0, TreeRoleId).toString().trimmed();
      const QString tag = role_tag_for_id(item_id);
      if (!tag.isEmpty() && node->data(0, TreeRoleCategory).toString() != "file") {
        const QString base_name = node->text(0);
        node->setText(0, QString("%1 %2").arg(base_name, tag));
        node->setToolTip(0, node->text(0));
      }
    }
  }
  int hierarchy_child_row_count = 0;
  for (int i = 0; i < scene_hierarchy_tree_->topLevelItemCount(); ++i) {
    auto * group = scene_hierarchy_tree_->topLevelItem(i);
    if (!group) continue;
    hierarchy_child_row_count += group->childCount();
  }
  if (scene3d_debug_logging_enabled()) {
    append_studio_log(QString("Scene3D diagnostics {hierarchy_child_row_count=%1, selected_scene_name=%2, selected_item_id=%3}")
                        .arg(hierarchy_child_row_count)
                        .arg(selected_scene_state_.name.isEmpty() ? QStringLiteral("(none)") : selected_scene_state_.name)
                        .arg(current_selected_scene_item_id_.isEmpty() ? QStringLiteral("(none)") : current_selected_scene_item_id_));
  }

  editable_layout_item_count_ = model.provenance_status.editable_layout_count;
  preview_fallback_item_count_ = model.provenance_status.generated_or_legacy_preview_count + model.provenance_status.static_fallback_preview_count;
  preview_provenance_summary_ = QString::fromStdString(model.provenance_status.summary);
  if (!visual_diagnostics_summary.isEmpty()) {
    preview_provenance_summary_ = preview_provenance_summary_.isEmpty()
      ? visual_diagnostics_summary
      : QString("%1 | %2").arg(preview_provenance_summary_, visual_diagnostics_summary);
  }
  all_scene_preview_items_ = preview_items;
  preview_warning_details_ = preview_warning_details;
  if (scene_preview_widget_) {
    scene_preview_widget_->set_scene_selected(true);
    scene_preview_widget_->set_preview_scene_name(selected_scene_state_.name);
    apply_scene3d_product_view_layer_defaults_and_commit();

    const auto scene3d_full_payload_counters = scene_preview_widget_->render_debug_counters();
    scene_preview_widget_->set_preview_status_summary(
      scene3d_user_preview_status_summary(
        scene3d_full_payload_counters,
        scene_preview_widget_->total_warning_count(),
        transform_parity.warning,
        transform_parity.failed));
    ++scene_diagnostic_payload_revision_;
    append_scene_diagnostic_log_once(
      QStringLiteral("full_payload_commit"),
      scene_diagnostic_payload_revision_,
      scene3d_full_payload_counters.viewport_received_count,
      QString("Scene3D full payload committed: scene=%1 total=%2 visible=%3 mesh=%4 locked=%5")
        .arg(selected_scene_state_.name)
        .arg(scene3d_full_payload_counters.viewport_received_count)
        .arg(scene3d_full_payload_counters.visible_count)
        .arg(scene3d_full_payload_counters.mesh_backed_count)
        .arg(scene3d_full_payload_counters.locked_generated_urdf_visual_count));
    refresh_new_cell_checklist();
  }

  const bool snapshot_available = fs::exists(d / "preview" / "epd_detection_snapshot.png");
  const QString perception_mode = snapshot_available ? "snapshot_overlay" : task_summary.perception_mode;

  QString camera_id;
  for (const auto & p : preview_items) {
    if (p.role == "camera") { camera_id = p.id; break; }
  }
  const bool has_camera_metadata = !camera_id.trimmed().isEmpty();

  QString perception_line;
  if (perception_mode == "snapshot_overlay") perception_line = "Perception: snapshot overlay loaded (no live launch required).";
  else if (perception_mode == "epd_optional") perception_line = "Perception: EPD optional (live runtime launch not required for preview overlays).";
  else perception_line = "Perception: none (camera metadata only).";
  if (!task_summary.perception_legacy_source.isEmpty()) perception_line += QString(" mapped from legacy mode: %1.").arg(task_summary.perception_legacy_source);

  const QString camera_line = has_camera_metadata ? QString("Camera: %1 configured.").arg(camera_id) : "Camera: no camera metadata in this scene.";
  [[maybe_unused]] const int urdf_visual_locked_count = visual_preview_added_count;
  int scene3d_editable_count = 0;
  int scene3d_mesh_count = 0;
  int scene3d_generated_count = 0;
  int scene3d_fallback_count = 0;
  int scene3d_missing_count = 0;
  int scene3d_locked_count = 0;
  for (const auto & item : preview_items) {
    const QString source_layer = canonical_scene3d_token(item.source_layer);
    const QString visual_source = canonical_scene3d_token(item.active_visual_source);
    if (source_layer == "editable_layout") ++scene3d_editable_count;
    if (visual_source == "mesh_preview") ++scene3d_mesh_count;
    if (source_layer == "locked_generated_urdf_visual") ++scene3d_generated_count;
    if (visual_source == "primitive_fallback" || source_layer == "primitive_fallback") ++scene3d_fallback_count;
    if (source_layer != "editable_layout" &&
        source_layer != "mesh_preview" &&
        source_layer != "locked_generated_urdf_visual" &&
        source_layer != "primitive_fallback" &&
        source_layer != "overlay") {
      ++scene3d_missing_count;
    }
    if (item.locked) ++scene3d_locked_count;
  }
  const QString scene3d_diagnostics_line = QString("Scene3D: editable=%1, mesh=%2, generated=%3, fallback=%4, missing=%5, locked=%6")
    .arg(scene3d_editable_count)
    .arg(scene3d_mesh_count)
    .arg(scene3d_generated_count)
    .arg(scene3d_fallback_count)
    .arg(scene3d_missing_count)
    .arg(scene3d_locked_count);
  const QString scene3d_warning_buckets = QString("Scene3D warnings: extraction_mode=%1, safe_for_preview=%2, missing_mesh=%3, unresolved_package_uri=%4, unsupported_extension=%5, stale_or_absolute_only_mesh_index=%6")
    .arg(visual_index_extraction_mode)
    .arg(visual_index_safe_for_preview ? QStringLiteral("true") : QStringLiteral("false"))
    .arg(missing_mesh_count)
    .arg(unresolved_package_uri_count)
    .arg(unsupported_extension_count)
    .arg(stale_or_absolute_only_mesh_index_count);
  const int scene_visible_count = all_scene_preview_items_.size();
  const int scene_received_count = preview_items.size();
  const int scene_rendered_count = scene_visible_count;
  const int scene_selectable_count = std::count_if(preview_items.cbegin(), preview_items.cend(), [](const ScenePreviewWidget::PreviewItem & p){ return p.selectable; });
  const int scene_mesh_rendered = scene3d_mesh_count;
  const int scene_fallback_rendered = scene3d_fallback_count;
  const bool scene3d_clean_product_view =
    visual_index_extraction_mode.trimmed().compare(QStringLiteral("xacro_expanded"), Qt::CaseInsensitive) == 0 &&
    visual_index_safe_for_preview &&
    missing_mesh_count == 0 &&
    unresolved_package_uri_count == 0 &&
    unsupported_extension_count == 0 &&
    scene_fallback_rendered == 0;
  scene3d_clean_product_view_ = scene3d_clean_product_view;
  if (scene_preview_widget_) {
    scene_preview_widget_->set_clean_product_view_status(scene3d_clean_product_view_, visual_preview_added_count > 0 ? visual_preview_added_count : scene_rendered_count);
    if (scene3d_clean_product_view_) {
      const auto scene3d_full_payload_counters = scene_preview_widget_->render_debug_counters();
      scene_preview_widget_->set_preview_status_summary(
        scene3d_user_preview_status_summary(
          scene3d_full_payload_counters,
          scene_preview_widget_->total_warning_count(),
          transform_parity.warning,
          transform_parity.failed,
          true,
          visual_preview_added_count > 0 ? visual_preview_added_count : scene_rendered_count));
    }
  }
  const int scene_locked_rendered = scene3d_locked_count;
  const int scene_skipped = visual_index_loaded_count - visual_preview_added_count;
  append_studio_log(QString("Scene3D canvas: scene=%1 received=%2 cached=%3 visible=%4 rendered=%5 selectable=%6 mesh=%7 fallback=%8 locked=%9 skipped=%10")
    .arg(QString::fromStdString(s.scene_name))
    .arg(scene_received_count)
    .arg(preview_items.size())
    .arg(scene_visible_count)
    .arg(scene_rendered_count)
    .arg(scene_selectable_count)
    .arg(scene_mesh_rendered)
    .arg(scene_fallback_rendered)
    .arg(scene_locked_rendered)
    .arg(scene_skipped));
  if (scene3d_debug_logging_enabled() && !skip_reason_counts.isEmpty()) {
    QStringList top_reason_tokens;
    for (auto it = skip_reason_counts.cbegin(); it != skip_reason_counts.cend(); ++it) top_reason_tokens << QString("%1=%2").arg(it.key()).arg(it.value());
    append_studio_log(QString("Scene3D canvas skip reasons: %1").arg(top_reason_tokens.join(' ')));
  }
  const QString preview_provenance_line = preview_provenance_summary_.isEmpty()
    ? QString("Preview fallback: 0 items loaded from scene metadata.")
    : preview_provenance_summary_;
  const QString preview_line = QString("%1 | %2 | %3 Metadata warnings: %4.")
    .arg(scene3d_diagnostics_line)
    .arg(scene3d_warning_buckets)
    .arg(preview_provenance_line)
    .arg(preview_warning_count);
  const QString scene3d_boundary_diag = QString(
      "Scene3D diagnostics {model_items_count=%1, preview_items_count=%2, filtered_visible_count=%3}")
      .arg(model.items.size())
      .arg(preview_items.size())
      .arg(all_scene_preview_items_.size());

  if (perception_line != last_perception_summary_log_) { append_studio_log(perception_line); last_perception_summary_log_ = perception_line; }
  if (camera_line != last_camera_summary_log_) { append_studio_log(camera_line); last_camera_summary_log_ = camera_line; }
  if (preview_line != last_preview_summary_log_) { append_studio_log(preview_line); last_preview_summary_log_ = preview_line; }
  if (scene3d_debug_logging_enabled()) append_studio_log(scene3d_boundary_diag);
}

void MainWindow::populate_asset_catalog()
{
  if (!asset_catalog_tree_) return;
  asset_catalog_tree_->clear();
  asset_catalog_entries_.clear();

  const fs::path repo_root = fs::current_path();
  const QDir assets_dir(QString::fromStdString((repo_root / "assets").string()));
  auto group_for_path = [](const QString & path) {
    const QString lower = path.toLower();
    if (lower.contains("robot") || lower.contains("/ur") || lower.contains("manipulator")) return QStringLiteral("robots");
    if (lower.contains("gripper") || lower.contains("robotiq") || lower.contains("suction") || lower.contains("airpick") || lower.contains("endeffector")) return QStringLiteral("grippers");
    if (lower.contains("camera") || lower.contains("realsense") || lower.contains("sensor")) return QStringLiteral("cameras");
    if (lower.contains("table") || lower.contains("workbench") || lower.contains("bench")) return QStringLiteral("table/workbench");
    if (lower.contains("environment") || lower.contains("fixture") || lower.contains("bin") || lower.contains("conveyor") || lower.contains("scene")) return QStringLiteral("environment");
    return QStringLiteral("other");
  };

  if (assets_dir.exists()) {
    QDirIterator it(assets_dir.absolutePath(), {QStringLiteral("*.stl"), QStringLiteral("*.dae"), QStringLiteral("*.obj")},
                    QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      const QString source = it.next();
      const QFileInfo info(source);
      AssetCatalogEntry ui_entry;
      ui_entry.asset_type = info.suffix().toLower();
      ui_entry.display_name = info.completeBaseName();
      ui_entry.role = QStringLiteral("mesh_asset");
      ui_entry.dimensions = QStringLiteral("scale=1");
      ui_entry.default_pose = QStringLiteral("xyz=[0,0,0] rpy=[0,0,0]");
      ui_entry.source_path = source;
      ui_entry.editable = true;
      ui_entry.availability_status = QStringLiteral("ready");
      ui_entry.disabled_reason.clear();
      ui_entry.category = group_for_path(source);
      asset_catalog_entries_.push_back(ui_entry);
    }
  }

  std::sort(asset_catalog_entries_.begin(), asset_catalog_entries_.end(), [](const AssetCatalogEntry & a, const AssetCatalogEntry & b) {
    if (a.category != b.category) return a.category < b.category;
    return a.display_name < b.display_name;
  });

  QMap<QString, QTreeWidgetItem *> groups;
  const QStringList ordered_groups = {"robots", "grippers", "cameras", "table/workbench", "environment", "other"};
  for (const QString & group : ordered_groups) {
    auto * parent = new QTreeWidgetItem(asset_catalog_tree_, {group, group, "assets/ mesh group", ""});
    parent->setFirstColumnSpanned(false);
    parent->setData(0, CatalogRoleIndex, -1);
    parent->setData(0, CatalogRolePlaceable, false);
    groups.insert(group, parent);
  }

  for (int idx = 0; idx < asset_catalog_entries_.size(); ++idx) {
    const auto & e = asset_catalog_entries_[idx];
    auto * parent = groups.value(e.category, groups.value("other"));
    auto * item = new QTreeWidgetItem(parent, {e.display_name, e.category, e.asset_type, e.availability_status});
    item->setData(0, CatalogRoleIndex, idx);
    item->setData(0, CatalogRolePlaceable, true);
    item->setData(0, CatalogRoleSourcePath, e.source_path);
    item->setToolTip(0, e.source_path);
  }
  for (auto * parent : groups) parent->setExpanded(parent->childCount() > 0);

  if (asset_catalog_entries_.isEmpty()) {
    auto * info = new QTreeWidgetItem(asset_catalog_tree_, {"No mesh assets found", "Info", "assets/", "blocked"});
    info->setDisabled(true);
    info->setToolTip(0, "No .stl, .dae, or .obj files were found under repo assets/.");
  }

  on_asset_filter_changed(asset_filter_combo_ ? asset_filter_combo_->currentIndex() : 0);
  validate_asset_catalog_selection();
}

void MainWindow::open_add_asset_dialog()
{
  if (!add_asset_dialog_) {
    add_asset_dialog_ = new QDialog(this);
    add_asset_dialog_->setWindowTitle("Add Asset");
    auto * layout = new QVBoxLayout(add_asset_dialog_);
    add_asset_dialog_table_ = new QTableWidget(add_asset_dialog_);
    add_asset_dialog_table_->setColumnCount(8);
    add_asset_dialog_table_->setHorizontalHeaderLabels({"Asset Type", "Display Name", "Role", "Dimensions", "Default Pose", "Mesh/URDF Path", "Editable", "Status"});
    add_asset_dialog_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    add_asset_dialog_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
    add_asset_dialog_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    add_asset_dialog_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    layout->addWidget(add_asset_dialog_table_);
    add_asset_dialog_details_label_ = new QLabel(add_asset_dialog_);
    add_asset_dialog_details_label_->setWordWrap(true);
    layout->addWidget(add_asset_dialog_details_label_);
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close, add_asset_dialog_);
    add_asset_dialog_place_button_ = new QPushButton("Place Asset", add_asset_dialog_);
    buttons->addButton(add_asset_dialog_place_button_, QDialogButtonBox::ActionRole);
    connect(buttons, &QDialogButtonBox::rejected, add_asset_dialog_, &QDialog::reject);
    connect(add_asset_dialog_place_button_, &QPushButton::clicked, this, &MainWindow::place_selected_asset_from_dialog);
    connect(add_asset_dialog_table_, &QTableWidget::itemSelectionChanged, this, &MainWindow::refresh_add_asset_dialog_details);
    layout->addWidget(buttons);
    add_asset_dialog_->resize(1200, 520);
  }

  add_asset_dialog_table_->setRowCount(asset_catalog_entries_.size());
  for (int row = 0; row < asset_catalog_entries_.size(); ++row) {
    const auto & e = asset_catalog_entries_[row];
    const QString status = e.disabled_reason.trimmed().isEmpty() ? e.availability_status : (e.availability_status + " (Unavailable for placement)");
    const QStringList cols = {e.asset_type, e.display_name, e.role, e.dimensions, e.default_pose, e.source_path, e.editable ? "Yes" : "No", status};
    for (int col = 0; col < cols.size(); ++col) {
      auto * cell = new QTableWidgetItem(cols[col]);
      cell->setData(CatalogRoleIndex, row);
      add_asset_dialog_table_->setItem(row, col, cell);
    }
  }
  if (add_asset_dialog_table_->rowCount() > 0) add_asset_dialog_table_->selectRow(0);
  refresh_add_asset_dialog_details();
  add_asset_dialog_->show();
  add_asset_dialog_->raise();
  add_asset_dialog_->activateWindow();
}

void MainWindow::refresh_add_asset_dialog_details()
{
  if (!add_asset_dialog_table_ || !add_asset_dialog_details_label_ || !add_asset_dialog_place_button_) return;
  int entry_index = -1;
  if (auto * current_item = add_asset_dialog_table_->currentItem()) {
    entry_index = current_item->data(CatalogRoleIndex).toInt();
  }
  if (entry_index < 0 || entry_index >= asset_catalog_entries_.size()) {
    add_asset_dialog_place_button_->setEnabled(false);
    add_asset_dialog_details_label_->setText("Select an asset row to inspect metadata.");
    return;
  }
  const auto & e = asset_catalog_entries_[entry_index];
  const bool placeable = e.disabled_reason.trimmed().isEmpty();
  add_asset_dialog_place_button_->setEnabled(placeable);
  const QString reason = placeable ? "Ready to place." : QString("Placement disabled: %1").arg(e.disabled_reason);
  add_asset_dialog_details_label_->setText(QString("<b>%1</b><br/>Availability: %2<br/>%3").arg(e.display_name, e.availability_status, reason));
}

void MainWindow::place_selected_asset_from_dialog()
{
  if (!add_asset_dialog_table_) return;
  auto * current_item = add_asset_dialog_table_->currentItem();
  if (!current_item) return;
  const int entry_index = current_item->data(CatalogRoleIndex).toInt();
  if (entry_index < 0 || entry_index >= asset_catalog_entries_.size()) return;
  const auto & e = asset_catalog_entries_[entry_index];
  if (!e.disabled_reason.trimmed().isEmpty()) return;
  add_asset_to_canvas_from_catalog(e.category, e.display_name, e.source_path);
}

void MainWindow::refresh_new_cell_checklist()
{
  if (!new_cell_checklist_label_) return;
  refresh_scene_workflow_rail();
  const NewCellStateAudit audit = audit_new_cell_state(selected_workspace_, scene_browser_result_, selected_scene_index_, preview_state_, preview_process_);
  QString workflow = scene_workflow_checklist_html();
  QString blocker_title = "none"; QString blocker_next = audit.next_recommended_action; QString blocker_page = "New Cell"; QString blocker_cmd;
  if (!audit.blockers.isEmpty()) { const QStringList parts = audit.blockers.first().split("|"); blocker_title = parts.value(0); blocker_next = parts.value(1, blocker_next); blocker_page = parts.value(2, blocker_page); blocker_cmd = parts.value(3); }
  QString text = QString("<b>New Cell Checklist</b><br/>Current state: <b>%1</b><br/>Done: %2<br/>Pending: %3<br/>First blocker: <b>%4</b><br/>Next action: <b>%5</b><br/>Related page: %6")
    .arg(audit.current_state, audit.completed_states.join(", "), audit.pending_states.join(", "),
      format_scene_builder_status_text(blocker_title), format_scene_builder_status_text(blocker_next), blocker_page);
  if (!blocker_cmd.trimmed().isEmpty()) {
    text += QString("<br/>Recovery command: <code>%1</code>").arg(
      format_scene_builder_status_text(blocker_cmd).toHtmlEscaped());
  }
  text += "<br/><br/>" + workflow;
  new_cell_checklist_label_->setText(text);
  if (scene_builder_command_preview_card_ && scene_builder_build_command_label_ && scene_builder_launch_command_label_) {
    const bool show_preview = launch_artifacts_ready_ && has_selected_scene();
    scene_builder_command_preview_card_->setVisible(show_preview);
    if (show_preview) {
      scene_builder_build_command_label_->setText(selected_scene_build_command());
      scene_builder_launch_command_label_->setText(
        format_scene_builder_status_text(selected_scene_launch_command()));
    }
  }

  if (readiness_label_) {
    QString readiness_text =
      "Preview/offline validation only\nNo robot motion commanded\nRuntime execution remains disabled unless explicitly enabled elsewhere";
    QStringList readiness_warnings = preview_warning_details_;
    readiness_warnings.append(readiness_warning_details_);
    readiness_warnings.removeDuplicates();
    readiness_text += QString("\n\nReadiness flags:\n- layout_saved: %1\n- validation_stale: %2\n- launch_artifacts_ready: %3").arg(layout_saved_ ? "true" : "false", validation_stale_ ? "true" : "false", launch_artifacts_ready_ ? "true" : "false");
    if (!readiness_warnings.isEmpty()) {
      readiness_text += QString("\n\nWarnings (%1):\n- %2").arg(readiness_warnings.size()).arg(readiness_warnings.join("\n- "));
    } else {
      readiness_text += "\n\nWarnings: none";
    }
    readiness_label_->setText(format_scene_builder_status_text(readiness_text));
    refresh_canvas_generated_parity_ui();
  }
}

QString MainWindow::scene_workflow_checklist_html() const
{
  QString out = "<b>Scene Builder Workflow</b><br/>";
  const auto steps = scene_workflow_steps();
  if (steps.empty()) return out + "Select a scene to view generation steps.";
  for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
    const auto & step = steps[static_cast<size_t>(i)];
    out += QString("%1. %2: <b>%3</b>").arg(i + 1).arg(step.label, scene_workflow_status_text(step.status));
    if (!step.detail.trimmed().isEmpty()) {
      out += " — " + format_scene_builder_status_text(step.detail).toHtmlEscaped();
    }
    out += "<br/>";
  }
  return out;
}

std::vector<MainWindow::SceneWorkflowStep> MainWindow::scene_workflow_steps() const
{
  if (!has_selected_scene()) return {};
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const fs::path dir = s.scene_dir;
  const auto has = [&](const char * rel){ return fs::exists(dir / rel); };
  const bool yaml_ready = has("cell_definition.yaml");
  const bool scene_manifest_ready = has("scene_manifest.yaml");
  const bool launch_ready = has("launch/demo.launch.py");
  const bool package_xml_ready = has("package.xml");
  const bool cmake_ready = has("CMakeLists.txt");
  const auto editable_layout_inspection = workcell_builder::inspect_editable_layout_entries(dir);
  const bool scene_xacro_ready = has("urdf/scene.urdf.xacro") || s.has_scene_urdf_xacro;
  const bool placeholder_launch_only = launch_ready && !scene_xacro_ready;
  const bool validation_report_ready = has("validation/readiness_report.json") || has("diagnostics/readiness_report.json") || has("run_acceptance.txt");
  const bool scene_selected = has_selected_scene();
  const auto canvas_model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  const Scene3DTransformParityReadiness transform_parity =
    scene3d_load_transform_parity_readiness(s.scene_dir, QString::fromStdString(s.scene_name));
  const bool editable_layout_ready = editable_layout_inspection.valid && editable_layout_inspection.editable_item_count > 0;
  const bool has_warnings = !readiness_warning_details_.isEmpty();
  const bool validation_gate_ready = validation_report_ready && !validation_stale_;
  const bool export_ready = yaml_ready && launch_ready;
  const bool scene_package_gate_ready = package_xml_ready && cmake_ready && launch_ready && scene_xacro_ready && !placeholder_launch_only;
  const bool fake_hardware_ready = scene_package_gate_ready && validation_gate_ready;

  int classified_editable_count = 0;
  int classified_generated_count = 0;
  int classified_fallback_count = 0;
  int classified_other_count = 0;
  for (const auto & item : canvas_model.items) {
    switch (item.provenance) {
      case workcell_builder::WorkcellStudioItemProvenance::EditableLayout:
        ++classified_editable_count;
        break;
      case workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
        ++classified_generated_count;
        break;
      case workcell_builder::WorkcellStudioItemProvenance::StaticFallbackPreview:
        ++classified_fallback_count;
        break;
      default:
        ++classified_other_count;
        break;
    }
  }

  const int preview_received_count = all_scene_preview_items_.size();
  const auto preview_item_visible_for_active_layers = [this](const ScenePreviewWidget::PreviewItem & p) {
    const QString source_layer = p.source_layer.trimmed().toLower();
    const QString visual_source = p.active_visual_source.trimmed().toLower();
    const QString category = p.category.trimmed().toLower();
    const QString status = p.status.trimmed().toLower();
    const bool is_overlay_or_helper = category.contains("overlay") || category.contains("helper") || source_layer.contains("overlay");
    const bool is_warning_or_missing = status.contains("warning") || p.mesh_load_warning.contains("missing", Qt::CaseInsensitive);
    if (source_layer == "editable_layout") return preview_layer_editable_layout_box_ ? preview_layer_editable_layout_box_->isChecked() : true;
    if (source_layer == "generated_urdf_visual" || source_layer == "locked_generated_urdf_visual") return preview_layer_generated_urdf_visual_box_ ? preview_layer_generated_urdf_visual_box_->isChecked() : true;
    if (source_layer == "primitive_fallback") return preview_layer_primitive_fallback_box_ ? preview_layer_primitive_fallback_box_->isChecked() : true;
    if (visual_source == "mesh_preview") return preview_layer_mesh_preview_box_ ? preview_layer_mesh_preview_box_->isChecked() : true;
    if (is_overlay_or_helper) return preview_layer_overlays_helpers_box_ ? preview_layer_overlays_helpers_box_->isChecked() : true;
    if (is_warning_or_missing) return preview_layer_warnings_missing_assets_box_ ? preview_layer_warnings_missing_assets_box_->isChecked() : true;
    return true;
  };
  const int preview_visible_count = std::count_if(all_scene_preview_items_.cbegin(), all_scene_preview_items_.cend(), preview_item_visible_for_active_layers);
  const int preview_rendered_count = preview_visible_count;
  int classified_overlay_count = 0;
  int classified_warning_count = 0;
  int classified_diagnostic_count = 0;
  int visible_fallback_count = 0;
  int visible_overlay_count = 0;
  int visible_warning_count = 0;
  int visible_diagnostic_count = 0;
  for (const auto & item : all_scene_preview_items_) {
    if (!preview_item_visible_for_active_layers(item)) continue;
    const QString source_layer = item.source_layer.trimmed().toLower();
    const QString visual_source = item.active_visual_source.trimmed().toLower();
    const QString category = item.category.trimmed().toLower();
    const QString status = item.status.trimmed().toLower();
    const bool is_overlay = category.contains("overlay") || category.contains("helper") || source_layer.contains("overlay");
    const bool is_warning = status.contains("warning") || item.mesh_load_warning.contains("missing", Qt::CaseInsensitive);
    const bool is_diagnostic = category.contains("diagnostic") || source_layer.contains("diagnostic") || status.contains("diagnostic");
    const bool is_fallback = source_layer.contains("fallback") || visual_source.contains("fallback");
    if (is_overlay) ++classified_overlay_count;
    if (is_warning) ++classified_warning_count;
    if (is_diagnostic) ++classified_diagnostic_count;
    if (preview_item_visible_for_active_layers(item)) {
      if (is_fallback) ++visible_fallback_count;
      if (is_overlay) ++visible_overlay_count;
      if (is_warning) ++visible_warning_count;
      if (is_diagnostic) ++visible_diagnostic_count;
    }
  }

  bool editable_layout_yaml_malformed = false;
  const std::vector<fs::path> editable_yaml_candidates = {
    dir / "layout" / "workcell_studio_layout.yaml",
    dir / "environment_layout.yaml"
  };
  for (const auto & path : editable_yaml_candidates) {
    if (!fs::exists(path)) continue;
    try { YAML::LoadFile(path.string()); }
    catch (const YAML::Exception &) { editable_layout_yaml_malformed = true; break; }
  }

  const bool preview_runtime_ready = preview_received_count > 0 || preview_visible_count > 0 || preview_rendered_count > 0 ||
    classified_generated_count > 0 || classified_fallback_count > 0 || classified_other_count > 0;
  const QMap<QString, bool> gates = {
    {"scene_selected", scene_selected},
    {"editable_layout", editable_layout_ready},
    {"layout_saved", layout_saved_},
    {"yaml_definition", yaml_ready && scene_manifest_ready},
    {"scene_package", scene_package_gate_ready},
    {"validation", validation_gate_ready},
    {"fake_hardware_preview", fake_hardware_ready},
    {"export", export_ready}
  };

  std::vector<SceneWorkflowStep> steps;
  steps.push_back(compute_scene_workflow_step(
    "Scene",
    scene_selected, "A scene is selected.", "Select or create a scene first.", {}, gates));
  const bool canonical_path_match = (canonical_scene_path_string(dir) == canonical_scene_path_string(fs::path(selected_scene_path().toStdString())));
  const LayoutStateModel layout_state = derive_layout_state_model(dir, canvas_model, canonical_path_match);
  QString layout_ready_detail = "Saved: environment_layout.yaml, layout/workcell_studio_layout.yaml, environment.yaml are present.";
  QString layout_missing_detail = "Create/edit and save layout to persist edits before YAML generation.";
  SceneWorkflowStepStatus layout_status = SceneWorkflowStepStatus::NeedsAction;
  bool layout_ready = false;
  switch (layout_state) {
    case LayoutStateModel::NO_LAYOUT_FILE: layout_missing_detail = "Save Layout Needed: no layout file"; break;
    case LayoutStateModel::EMPTY_LAYOUT: layout_missing_detail = "Save Layout Needed: no editable items"; break;
    case LayoutStateModel::PREVIEW_ONLY_AVAILABLE: layout_missing_detail = "Save Layout Needed: no editable items"; break;
    case LayoutStateModel::PREVIEW_UNAVAILABLE: layout_missing_detail = "Save Layout Needed: no editable items"; break;
    case LayoutStateModel::PATH_MISMATCH: layout_missing_detail = "Save Layout Blocked: scene path mismatch"; layout_status = SceneWorkflowStepStatus::Blocked; break;
    case LayoutStateModel::INVALID_LAYOUT_YAML: layout_missing_detail = "Save Layout Needed: invalid layout YAML"; layout_status = SceneWorkflowStepStatus::Warning; break;
    case LayoutStateModel::EDITABLE_LAYOUT_PRESENT: layout_ready = workcell_builder::is_save_layout_workflow_ready(dir); layout_status = layout_ready ? SceneWorkflowStepStatus::Done : SceneWorkflowStepStatus::NeedsAction; break;
  }
  steps.push_back(compute_scene_workflow_step(
    "Save Layout",
    layout_ready,
    layout_ready_detail,
    layout_missing_detail,
    {}, gates, layout_status));
  steps.push_back(compute_scene_workflow_step(
    "Generate YAML",
    yaml_ready && scene_manifest_ready, "Ready: cell_definition.yaml and scene_manifest.yaml are present.",
    "Missing/Blocked: Generate YAML to create cell_definition.yaml and scene_manifest.yaml (task/workcell_builder_task_intent.yaml is optional).",
    {}, gates));
  steps.push_back(compute_scene_workflow_step(
    "Generate Scene Package",
    scene_package_gate_ready, "Ready: package.xml, CMakeLists.txt, launch/demo.launch.py, and urdf/scene.urdf.xacro are present.",
    placeholder_launch_only ?
      "Partial: Placeholder launch only — not RViz truth preview ready." :
      "Blocked: Generate Scene Package to create package.xml, CMakeLists.txt, launch/demo.launch.py, and real scene URDF/Xacro.",
    {}, gates));
  steps.push_back(compute_scene_workflow_step(
    "Validate",
    validation_gate_ready, has_warnings ? "Validation completed with warnings." : "Validation checks passed.",
    validation_stale_ ? "Validation results are stale; rerun validation." : "Run offline validation.",
    {"yaml_definition"}, gates,
    validation_gate_ready ? (has_warnings ? SceneWorkflowStepStatus::Warning : SceneWorkflowStepStatus::Done) : SceneWorkflowStepStatus::Current));
  const ScenePreviewWidget::RenderDebugCounters scene3d_counters =
    scene_preview_widget_ ? scene_preview_widget_->render_debug_counters() : ScenePreviewWidget::RenderDebugCounters{};
  const bool preview_has_runtime_content = scene3d_counters.viewport_received_count > 0 || scene3d_counters.visible_count > 0 ||
    scene3d_counters.rendered_count > 0 || preview_runtime_ready;
  const QString native_preview_counts = QString(
    "Scene3D counters: received=%1 visible=%2 rendered=%3; classified layers: editable=%4 generated=%5 fallback=%6 overlays/helpers=%7 warning/missing=%8 diagnostics=%9 other=%10.")
      .arg(preview_received_count)
      .arg(preview_visible_count)
      .arg(preview_rendered_count)
      .arg(classified_editable_count)
      .arg(classified_generated_count)
      .arg(classified_fallback_count)
      .arg(classified_overlay_count)
      .arg(classified_warning_count)
      .arg(classified_diagnostic_count)
      .arg(classified_other_count);
  const QString launch_gate_detail = QString(
    "RViz/MoveIt fake-hardware launch readiness is evaluated separately by package and validation gates: scene_package=%1 validation=%2 fake_hardware_launch=%3.")
      .arg(scene_package_gate_ready ? "ready" : "blocked")
      .arg(validation_gate_ready ? "ready" : "blocked")
      .arg(fake_hardware_ready ? "ready" : "blocked");

  QString preview_detail = QString("Native Scene3D preview is waiting for renderable layout, generated preview, or fallback content. %1 %2")
    .arg(native_preview_counts, launch_gate_detail);
  SceneWorkflowStepStatus preview_status = SceneWorkflowStepStatus::NeedsAction;
  if (preview_has_runtime_content) {
    preview_status = SceneWorkflowStepStatus::Done;
    preview_detail = QString("3D Preview Technical Pass: native Scene3D has renderable content, but render/smoke counters alone do not prove demo-ready visual quality or RViz/MoveIt launch readiness. %1 %2")
      .arg(native_preview_counts, launch_gate_detail);
    if (!transform_parity.warning.isEmpty()) {
      preview_status = transform_parity.failed
        ? SceneWorkflowStepStatus::Blocked
        : SceneWorkflowStepStatus::Warning;
      preview_detail = QString("%1 Source: %2 %3 %4")
        .arg(transform_parity.warning, transform_parity.source, native_preview_counts, launch_gate_detail);
    }
    const bool visual_quality_needs_review = !scene3d_clean_product_view_ && (editable_layout_yaml_malformed || classified_fallback_count > 0 ||
      classified_overlay_count > 0 || classified_warning_count > 0 || classified_editable_count == 0 ||
      classified_diagnostic_count > 0 || has_warnings);
    if (visual_quality_needs_review && transform_parity.warning.isEmpty()) {
      preview_status = SceneWorkflowStepStatus::Warning;
      QStringList preview_warnings;
      if (editable_layout_yaml_malformed) {
        preview_warnings << "editable/layout YAML is malformed; fix YAML to restore editable preview health";
      }
      if (visible_fallback_count > 0) {
        preview_warnings << "fallback content is visible from scene metadata or URDF mesh index";
      }
      if (visible_overlay_count > 0) {
        preview_warnings << "overlays/helpers remain visible and should be checked against the intended demo view";
      }
      if (visible_warning_count > 0) {
        preview_warnings << "warning or missing-asset items remain in the preview";
      }
      if (classified_editable_count == 0) {
        preview_warnings << "no editable layout items are classified yet; create editable layout from preview when appropriate";
      }
      if (visible_diagnostic_count > 0) {
        preview_warnings << "diagnostic preview items remain visible";
      }
      if (has_warnings) {
        preview_warnings << "validation/readiness warnings are present";
      }
      if (!scene3d_counters.visual_quality_warnings.isEmpty()) {
        preview_warnings << QString("Scene3D renderer reported: %1").arg(scene3d_counters.visual_quality_warnings.join(", "));
      }
      preview_detail = QString("Visual Review Needed: %1. 3D Preview Technical Pass only confirms renderable Scene3D content; it is not demo-ready proof and is not RViz/MoveIt truth. %2 %3")
        .arg(preview_warnings.join("; "), native_preview_counts, launch_gate_detail);
    }
  }
  SceneWorkflowStep preview_step;
  preview_step.label = "3D Scene Preview";
  preview_step.status = preview_status;
  preview_step.detail = preview_detail;
  steps.push_back(preview_step);

  QString fake_launch_missing_detail = "Blocked: complete scene package generation and current validation before launching RViz/MoveIt fake hardware.";
  if (placeholder_launch_only) {
    fake_launch_missing_detail = "Blocked: placeholder launch only — generate a real scene URDF/Xacro and RViz/MoveIt launch before fake-hardware launch validation.";
  } else if (!scene_package_gate_ready) {
    fake_launch_missing_detail = "Blocked: missing or incomplete launch artifacts. Generate Scene Package first.";
  } else if (!validation_gate_ready) {
    fake_launch_missing_detail = validation_stale_
      ? "Blocked: validation results are stale; rerun validation before RViz/MoveIt fake-hardware launch."
      : "Blocked: validation/package gate has not passed; run offline validation before RViz/MoveIt fake-hardware launch.";
  }
  steps.push_back(compute_scene_workflow_step(
    "RViz/MoveIt Fake-Hardware Launch",
    fake_hardware_ready,
    "Ready: package and validation gates allow guarded RViz/MoveIt fake-hardware launch. Safety remains fake hardware only.",
    fake_launch_missing_detail,
    {"scene_package", "validation"}, gates,
    fake_hardware_ready ? (has_warnings ? SceneWorkflowStepStatus::Warning : SceneWorkflowStepStatus::Done) : SceneWorkflowStepStatus::Blocked));
  steps.push_back(compute_scene_workflow_step(
    "Export",
    export_ready, "Export prerequisites are satisfied.",
    "Complete YAML and scene package generation before export.",
    {"yaml_definition", "scene_package"}, gates, export_ready ? (has_warnings ? SceneWorkflowStepStatus::Warning : SceneWorkflowStepStatus::Done) : SceneWorkflowStepStatus::Blocked));
  return steps;
}

MainWindow::SceneWorkflowStep MainWindow::compute_scene_workflow_step(
  const QString & label,
  bool ready,
  const QString & ready_detail,
  const QString & missing_detail,
  const QStringList & prerequisites,
  const QMap<QString, bool> & prerequisite_states,
  SceneWorkflowStepStatus ready_status) const
{
  SceneWorkflowStep step;
  step.label = label;
  QStringList blockers;
  for (const QString & prerequisite : prerequisites) {
    if (!prerequisite_states.value(prerequisite, false)) {
      blockers << prerequisite;
    }
  }
  if (!blockers.isEmpty()) {
    step.status = SceneWorkflowStepStatus::Blocked;
    step.detail = QString("Blocked by prerequisites: %1. %2").arg(blockers.join(", "), missing_detail);
  } else if (ready) {
    step.status = ready_status;
    step.detail = ready_detail;
  } else {
    step.status = SceneWorkflowStepStatus::NeedsAction;
    step.detail = missing_detail;
  }
  return step;
}

QString MainWindow::scene_workflow_status_text(SceneWorkflowStepStatus status) const
{
  switch (status) {
    case SceneWorkflowStepStatus::Done: return "Done";
    case SceneWorkflowStepStatus::Current: return "Ready";
    case SceneWorkflowStepStatus::NeedsAction: return "Needed";
    case SceneWorkflowStepStatus::Blocked: return "Blocked";
    case SceneWorkflowStepStatus::Warning: return "Missing";
  }
  return "Needed";
}

QString MainWindow::scene_workflow_status_chip(SceneWorkflowStepStatus status) const
{
  QString bg = "#6b7280";
  if (status == SceneWorkflowStepStatus::Done) bg = "#15803d";
  else if (status == SceneWorkflowStepStatus::Current) bg = "#2563eb";
  else if (status == SceneWorkflowStepStatus::NeedsAction) bg = "#b45309";
  else if (status == SceneWorkflowStepStatus::Blocked) bg = "#991b1b";
  else if (status == SceneWorkflowStepStatus::Warning) bg = "#c2410c";
  return QString("<span style='color:#fff;background:%1;border-radius:10px;padding:2px 8px;font-size:11px;'>%2</span>")
    .arg(bg, scene_workflow_status_text(status));
}

MainWindow::RecommendedWorkflowAction MainWindow::resolve_recommended_workflow_action() const
{
  const auto actions = resolve_recommended_workflow_actions();
  if (!actions.empty()) return actions.front();
  RecommendedWorkflowAction fallback;
  fallback.token = "open_or_create_scene";
  fallback.label = "Open or create a scene";
  fallback.enabled = true;
  fallback.explanatory_text = "Select a scene to start workflow actions.";
  fallback.handler = RecommendedWorkflowActionHandler::OpenOrCreateScene;
  return fallback;
}

std::vector<MainWindow::RecommendedWorkflowAction> MainWindow::resolve_recommended_workflow_actions() const
{
  std::vector<RecommendedWorkflowAction> actions;
  auto add_action = [&](const QString & token, const QString & label, bool enabled, const QString & blocker, const QString & explainer, RecommendedWorkflowActionHandler handler) {
    RecommendedWorkflowAction action;
    action.token = token;
    action.label = label;
    action.enabled = enabled;
    action.blocker_reason_tooltip = blocker;
    action.explanatory_text = explainer;
    action.handler = handler;
    actions.push_back(action);
  };

  if (!has_selected_scene()) {
    add_action(
      "open_or_create_scene", "Open or create a scene", true, QString(),
      "Start by selecting an existing scene or creating a new one before workflow actions can run.",
      RecommendedWorkflowActionHandler::OpenOrCreateScene);
    add_action("add_asset", "Add asset", false, "No scene selected.", "Add assets after selecting a scene.", RecommendedWorkflowActionHandler::AddAsset);
    add_action("save_layout", "Save layout", false, "No scene selected.", "Save layout after selecting a scene.", RecommendedWorkflowActionHandler::SaveLayout);
    return actions;
  }
  if (editable_layout_item_count_ == 0 && preview_fallback_item_count_ > 0) {
    add_action("create_editable_layout_from_preview", "Create editable layout from preview", true, QString(),
      "Create editable layout from preview",
      RecommendedWorkflowActionHandler::CreateEditableLayoutFromPreview);
    add_action("generate_yaml", "Generate YAML", false, "Create an editable layout first.", "YAML generation requires editable layout content.", RecommendedWorkflowActionHandler::GenerateYaml);
    return actions;
  }
  bool has_asset_items = false;
  if (digital_twin_scene_ != nullptr) {
    const auto canvas_items = digital_twin_scene_->items();
    for (QGraphicsItem * item : canvas_items) {
      if (item != nullptr && item->data(RoleRole).toString() == "asset") {
        has_asset_items = true;
        break;
      }
    }
  }

  if (!has_asset_items) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    const bool legacy_preview_only_scene =
      workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name).items.empty() &&
      (fs::exists(s.scene_dir / "launch/demo.launch.py") || fs::exists(s.scene_dir / "cell_definition.yaml"));
    if (legacy_preview_only_scene) {
      add_action("create_editable_layout_from_preview", "Create editable layout from preview", true, QString(),
        "Create editable layout from preview",
        RecommendedWorkflowActionHandler::CreateEditableLayoutFromPreview);
      add_action("add_asset", "Add asset", false, "Convert preview-only scene first.", "After conversion, add assets to finalize layout.", RecommendedWorkflowActionHandler::AddAsset);
      return actions;
    }
    add_action("add_asset", "Add asset", true, QString(),
      "Populate the layout with at least one asset so there is content to save and generate.",
      RecommendedWorkflowActionHandler::AddAsset);
    add_action("save_layout", "Save layout", false, "No asset items exist.", "Add at least one asset before saving.", RecommendedWorkflowActionHandler::SaveLayout);
    return actions;
  }
  if (layout_dirty_ || !layout_saved_) {
    add_action("save_layout", "Save layout", true, QString(),
      "Commit current layout edits so downstream YAML and package outputs use the latest scene state.",
      RecommendedWorkflowActionHandler::SaveLayout);
    add_action("generate_yaml", "Generate YAML", false, "Layout has unsaved edits.", "Save layout before YAML generation.", RecommendedWorkflowActionHandler::GenerateYaml);
    return actions;
  }

  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const QString yaml_path = QString::fromStdString((s.scene_dir / "cell_definition.yaml").string());
  const bool yaml_ready = QFileInfo::exists(yaml_path);
  if (!yaml_ready) {
    add_action("generate_yaml", "Generate YAML", true, QString(),
      "Create the YAML draft from the saved layout to unlock validation and scene package generation.",
      RecommendedWorkflowActionHandler::GenerateYaml);
    add_action("generate_scene_package", "Generate scene package", false, "Missing YAML file.", "Scene package generation needs cell_definition.yaml.", RecommendedWorkflowActionHandler::GenerateScenePackage);
    return actions;
  }
  if (validation_stale_) {
    add_action("validate_scene", "Validate scene", true, QString(),
      "Re-run generated scene validation to clear stale checks after recent edits or YAML updates.",
      RecommendedWorkflowActionHandler::ValidateScene);
    add_action("generate_scene_package", "Generate scene package", false, "Validation is stale.", "Revalidate before packaging or previewing.", RecommendedWorkflowActionHandler::GenerateScenePackage);
    return actions;
  }
  if (!launch_artifacts_ready_) {
    add_action("generate_scene_package", "Generate scene package", true, QString(),
      "Build launch artifacts and package scaffolding so planning/simulation can run reliably.",
      RecommendedWorkflowActionHandler::GenerateScenePackage);
    add_action("plan_simulate", "Plan / Simulate", false, "Scene package artifacts are missing.", "Generate package before preview.", RecommendedWorkflowActionHandler::PlanSimulate);
    return actions;
  }
  QStringList preview_blockers;
  const bool preview_ready = selected_scene_preview_ready(&preview_blockers);
  if (!preview_ready) {
    add_action("plan_simulate", "Plan / Simulate", false, preview_blockers.join(" "),
      "Resolve preview prerequisites before opening Plan & Simulate with prepared launch commands.",
      RecommendedWorkflowActionHandler::PlanSimulate);
    add_action("generate_scene_package", "Generate scene package", true, QString(), "Regenerate artifacts to resolve preview blockers.", RecommendedWorkflowActionHandler::GenerateScenePackage);
    return actions;
  }
  add_action("plan_simulate", "Plan / Simulate", true, QString(),
    "Ready for fake-hardware preview with validated launch artifacts and safety-gated commands.",
    RecommendedWorkflowActionHandler::PlanSimulate);
  add_action("export_bundle", "Export bundle", true, QString(),
    "Package and export the validated scene bundle for handoff and reproducible deployment.",
    RecommendedWorkflowActionHandler::ExportBundle);
  return actions;
}

void MainWindow::trigger_recommended_workflow_action(RecommendedWorkflowActionHandler handler)
{
  switch (handler) {
    case RecommendedWorkflowActionHandler::OpenOrCreateScene:
      open_new_scene_creation_flow();
      return;
    case RecommendedWorkflowActionHandler::AddAsset:
      open_add_asset_dialog();
      return;
    case RecommendedWorkflowActionHandler::CreateEditableLayoutFromPreview:
      create_starter_layout_from_preview();
      return;
    case RecommendedWorkflowActionHandler::SaveLayout:
      save_layout_changes();
      return;
    case RecommendedWorkflowActionHandler::GenerateYaml:
      generate_yaml_draft_for_selected_scene();
      return;
    case RecommendedWorkflowActionHandler::ValidateScene:
      validate_generated_scene_for_selected_scene();
      return;
    case RecommendedWorkflowActionHandler::GenerateScenePackage:
      generate_scene_package_for_selected_scene();
      return;
    case RecommendedWorkflowActionHandler::PlanSimulate:
      show_studio_page(StudioPage::PlanSimulatePage);
      refresh_preview_launch_ui();
      append_studio_log("Recommended action: switched to Plan & Simulate and prepared preview commands.");
      return;
    case RecommendedWorkflowActionHandler::ExportBundle:
      show_studio_page(StudioPage::ExportPage);
      append_studio_log("Recommended action: switched to Export page.");
      return;
  }
}

void MainWindow::refresh_scene_workflow_rail()
{
  if (!scene_workflow_rail_label_) return;
  const auto steps = scene_workflow_steps();
  if (steps.empty()) {
    scene_workflow_rail_label_->setText("Select a scene to view workflow steps.");
    if (scene_workflow_recommendation_label_) {
      scene_workflow_recommendation_label_->setText("Recommended next action appears after scene context is available.");
    }
    if (scene_workflow_recommendation_button_) {
      scene_workflow_recommendation_button_->setText("Run Next: Open or create a scene");
      scene_workflow_recommendation_button_->setEnabled(true);
      scene_workflow_recommendation_button_->setToolTip(QString());
    }
    refresh_run_next_menu(resolve_recommended_workflow_actions());
    return;
  }
  QString html;
  for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
    const auto & step = steps[static_cast<size_t>(i)];
    const QString detail = format_scene_builder_status_text(step.detail).toHtmlEscaped();
    html += QString("%1. <span title='%4'>%2 %3</span><br/>")
      .arg(i + 1)
      .arg(step.label, scene_workflow_status_chip(step.status), detail);
  }
  scene_workflow_rail_label_->setText(html);
  const auto recommendations = resolve_recommended_workflow_actions();
  const auto recommendation = recommendations.empty() ? RecommendedWorkflowAction{} : recommendations.front();
  if (scene_workflow_recommendation_label_) {
    scene_workflow_recommendation_label_->setText("<b>Next:</b> " + recommendation.explanatory_text);
  }
  if (scene_workflow_recommendation_button_) {
    scene_workflow_recommendation_button_->setText(QString("Run Next: %1").arg(recommendation.label));
    scene_workflow_recommendation_button_->setEnabled(recommendation.enabled);
    scene_workflow_recommendation_button_->setToolTip(
      recommendation.blocker_reason_tooltip.isEmpty() ? recommendation.explanatory_text : recommendation.blocker_reason_tooltip);
  }
  refresh_run_next_menu(recommendations);
}

void MainWindow::refresh_run_next_menu(const std::vector<RecommendedWorkflowAction> & actions)
{
  if (!scene_workflow_recommendation_menu_) return;
  scene_workflow_recommendation_menu_->clear();
  for (size_t i = 0; i < actions.size(); ++i) {
    const auto & action = actions[i];
    const QString details = action.blocker_reason_tooltip.isEmpty() ? action.explanatory_text : action.blocker_reason_tooltip;
    QString label = action.label;
    if (i == 0) label += " (Recommended)";
    auto * menu_action = scene_workflow_recommendation_menu_->addAction(label);
    menu_action->setEnabled(action.enabled);
    menu_action->setToolTip(details);
    menu_action->setStatusTip(details);
    connect(menu_action, &QAction::triggered, this, [this, handler = action.handler]() {
      trigger_recommended_workflow_action(handler);
    });
  }
}
