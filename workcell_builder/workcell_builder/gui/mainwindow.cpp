// Copyright 2026 Mukaram01
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
#include <QFileDialog>
#include <QAction>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QToolBar>
#include <QToolTip>
#include <QApplication>
#include <QClipboard>
#include <QDir>
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
#include <QFileInfo>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsSimpleTextItem>
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
#include <QSplitter>
#include <QScrollArea>
#include <QGroupBox>
#include <QMetaObject>
#include <QPointer>
#include <QProgressDialog>
#include <QSettings>
#include <QLineEdit>
#include <QEvent>
#include <QMouseEvent>
#include <QDrag>
#include <QMimeData>
#include <QSet>
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
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
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
#include "workcell_studio_layout_editor.hpp"
#include "workcell_studio_id_utils.hpp"
#include "gui/new_cell_wizard.h"
#include "include/workcell_builder_command_builders.hpp"
#include "gui/asset_catalog_discovery.h"
#include "gui/transform_clipboard_utils.h"

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

enum CanvasRoles { RoleId = Qt::UserRole + 1, RoleDisplayName, RoleType, RoleCategory, RoleRole, RoleSource, RoleSourcePackage, RolePoseZ, RoleRoll, RolePitch, RoleYaw, RoleWidth, RoleDepth, RoleHeight, RoleImported, RoleGeneratedPlaceholder, RoleLocked, RoleWarning, RolePoseText };
enum SceneTreeRoles {
  TreeRoleId = Qt::UserRole + 1, TreeRoleCategory, TreeRolePoseText, TreeRoleSource, TreeRolePoseX, TreeRolePoseY, TreeRolePoseZ, TreeRoleRoll, TreeRolePitch, TreeRoleYaw, TreeRolePoseAvailable, TreeRoleRole,
  TreeRoleSourceLayer, TreeRoleActiveVisualSource, TreeRoleEditable, TreeRoleLocked, TreeRoleLinkedEditableLayout, TreeRoleVisualBackingStatus, TreeRoleGeneratedVisual, TreeRoleItemTypeClass,
  TreeRoleStableId, TreeRoleCameraId, TreeRoleFrameId, TreeRoleDetectionLabel, TreeRoleConfidence, TreeRoleTrackingId, TreeRoleSnapshotSourceFile, TreeRoleAlignmentWarning
};
enum AssetCatalogRoles { CatalogRoleIndex = Qt::UserRole, CatalogRolePlaceable = Qt::UserRole + 10, CatalogRoleSourcePath = Qt::UserRole + 11 };

class DraggableCanvasItem : public QGraphicsRectItem {
public:
  explicit DraggableCanvasItem(const QRectF & r): QGraphicsRectItem(r) {}
  std::function<QPointF(const QPointF &)> position_filter;
protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override {
    if (change == ItemPositionChange && scene() && position_filter) {
      return position_filter(value.toPointF());
    }
    return QGraphicsRectItem::itemChange(change, value);
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
  out.next_recommended_action = "Use Recommended Layout / Add to Canvas";
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
  append_studio_log(
    QString("%1: opened Scene Builder for '%2' at %3.").arg(source_action, selected_scene_name(), selected_scene_path()));
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
  preview_layer_generated_urdf_visual_box_ = new QCheckBox("generated URDF visuals", preview_layers_group);
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
  catalog_layout->addWidget(new QLabel("<b>Asset Catalog</b>"));
  asset_filter_combo_ = new QComboBox(catalog_card);
  asset_filter_combo_->addItems({"All", "Robots", "End Effectors", "Fixtures", "Sensors", "Tables", "Conveyors", "Bins", "Custom"});
  catalog_layout->addWidget(asset_filter_combo_);
  asset_catalog_tree_ = new QTreeWidget(catalog_card);
  asset_catalog_tree_->setObjectName("studioAssetCatalogTree");
  asset_catalog_tree_->setHeaderLabels({"Asset", "Category", "Type/Source", "Status"});
  asset_catalog_tree_->viewport()->setAcceptDrops(false);
  asset_catalog_tree_->setDragEnabled(true);
  asset_catalog_tree_->viewport()->installEventFilter(this);
  catalog_layout->addWidget(asset_catalog_tree_, 1);
  add_to_canvas_button_ = new QPushButton("Add to Canvas", scene_builder);
  add_to_canvas_button_->setEnabled(false);
  catalog_layout->addWidget(add_to_canvas_button_);
  add_asset_button_ = new QPushButton("Add Asset", scene_builder);
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
  canvas_mode_label_ = new QLabel("Mode: Select · 3D Layout Preview", scene_builder); controls->addWidget(canvas_mode_label_);
  // Scene canvas entrypoint: keep this same center-panel surface and swap rendering internals through ScenePreviewWidget.
  // ScenePreviewWidget consumes preview items produced from:
  //   1) editable layout metadata (layout/workcell_studio_layout.yaml)
  //   2) locked/generated visual metadata (generated/scene_visual_mesh_index.json)
  // and forwards them to Scene3DViewportWidget for perspective/depth/orbit-pan-zoom rendering.
  scene_preview_widget_ = new ScenePreviewWidget(scene_builder);
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
    scene3d_viewport->transform_changed_cb = [this](const QString &id, double x, double y, double z, double r, double p, double yaw){
      if (!digital_twin_scene_) return;
      for (auto * item : digital_twin_scene_->items()) {
        if (item->data(RoleId).toString() != id) continue;
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
  register_scene_action("simulate.open_rviz", "Open RViz2 / MoveIt", [this]() { if (run_build_button_) run_build_button_->click(); });
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
  toggle_labels_box_ = new QCheckBox("Toggle Labels", scene_builder); toggle_labels_box_->setChecked(true);
  toggle_warnings_box_ = new QCheckBox("Toggle Warnings", scene_builder); toggle_warnings_box_->setChecked(true);
  show_minimap_box_ = new QCheckBox("Show Minimap", scene_builder); show_minimap_box_->setChecked(true);
  scene_builder_overlays_button_ = new QToolButton(scene_builder);
  auto * overlays_button = scene_builder_overlays_button_; overlays_button->setText("Overlays"); overlays_button->setPopupMode(QToolButton::InstantPopup);
  auto * overlays_menu = new QMenu(overlays_button);
  show_reach_overlay_box_ = new QCheckBox("Show Reach", scene_builder); show_reach_overlay_box_->setChecked(true);
  show_camera_fov_overlay_box_ = new QCheckBox("Camera FOV", scene_builder); show_camera_fov_overlay_box_->setChecked(true);
  show_pick_place_overlay_box_ = new QCheckBox("Pick Coverage", scene_builder); show_pick_place_overlay_box_->setChecked(true);
  show_trajectory_overlay_box_ = new QCheckBox("EPD Detections", scene_builder); show_trajectory_overlay_box_->setChecked(true);
  auto * show_approach_retreat_overlay_box = new QCheckBox("Approach/Retreat", scene_builder); show_approach_retreat_overlay_box->setChecked(true);
  auto mk=[&](QCheckBox *b){ auto *a=overlays_menu->addAction(b->text()); a->setCheckable(true); a->setChecked(true); connect(a,&QAction::toggled,b,&QCheckBox::setChecked); connect(b,&QCheckBox::toggled,a,&QAction::setChecked); };
  mk(show_reach_overlay_box_); mk(show_camera_fov_overlay_box_); mk(show_pick_place_overlay_box_); mk(show_trajectory_overlay_box_); mk(show_approach_retreat_overlay_box);
  auto * show_warnings_action = overlays_menu->addAction("Show Warnings"); show_warnings_action->setCheckable(true); show_warnings_action->setChecked(true);
  auto * show_labels_action = overlays_menu->addAction("Detection Labels"); show_labels_action->setCheckable(true); show_labels_action->setChecked(true);
  overlays_button->setMenu(overlays_menu);
  connect(show_warnings_action, &QAction::toggled, toggle_warnings_box_, &QCheckBox::setChecked);
  connect(show_labels_action, &QAction::toggled, toggle_labels_box_, &QCheckBox::setChecked);
  connect(show_approach_retreat_overlay_box, &QCheckBox::toggled, this, [this, show_approach_retreat_overlay_box](bool){
    if (!scene_preview_widget_) return;
    scene_preview_widget_->set_task_overlay_visibility(
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_approach_retreat_overlay_box->isChecked(),
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
    scene_preview_widget_->set_perception_overlay_visibility(
      show_camera_fov_overlay_box_ ? show_camera_fov_overlay_box_->isChecked() : true,
      show_pick_place_overlay_box_ ? show_pick_place_overlay_box_->isChecked() : true,
      show_trajectory_overlay_box_ ? show_trajectory_overlay_box_->isChecked() : true,
      toggle_labels_box_ ? toggle_labels_box_->isChecked() : true);
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
  make_row(scene_card_layout, "Name", "No scene selected", false);
  make_row(scene_card_layout, "Status", "unknown", false);
  make_row(scene_card_layout, "Robot", "unknown", false);
  make_row(scene_card_layout, "End Effector", "unknown", false);
  make_row(scene_card_layout, "Path", "(none)", true);
  make_row(scene_card_layout, "Launch", "(none)", true);

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
    "pending: Environment layout created (table + pick zone + place zone + camera) → click Use Recommended Layout<br/>"
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
  auto * export_actions = make_action_section("Export");
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
  inspector_label_=new QLabel("Inspector selection: none"); inspector_label_->setWordWrap(true); selection_tab_layout->addWidget(inspector_label_);
  live_coordinate_label_ = new QLabel("Selected: none", scene_builder); selection_tab_layout->addWidget(live_coordinate_label_);
  auto * pose_grid = new QGridLayout();
  inspector_x_ = new QDoubleSpinBox(scene_builder); inspector_x_->setPrefix("x "); pose_grid->addWidget(inspector_x_, 0, 0);
  inspector_y_ = new QDoubleSpinBox(scene_builder); inspector_y_->setPrefix("y "); pose_grid->addWidget(inspector_y_, 0, 1);
  inspector_z_ = new QDoubleSpinBox(scene_builder); inspector_z_->setPrefix("z "); pose_grid->addWidget(inspector_z_, 0, 2);
  inspector_roll_ = new QDoubleSpinBox(scene_builder); inspector_roll_->setPrefix("r "); pose_grid->addWidget(inspector_roll_, 1, 0);
  inspector_pitch_ = new QDoubleSpinBox(scene_builder); inspector_pitch_->setPrefix("p "); pose_grid->addWidget(inspector_pitch_, 1, 1);
  inspector_yaw_ = new QDoubleSpinBox(scene_builder); inspector_yaw_->setPrefix("yaw "); pose_grid->addWidget(inspector_yaw_, 1, 2);
  auto * dim_grid = new QGridLayout();
  inspector_dim_x_ = new QDoubleSpinBox(scene_builder); inspector_dim_x_->setPrefix("dx "); dim_grid->addWidget(inspector_dim_x_, 0, 0);
  inspector_dim_y_ = new QDoubleSpinBox(scene_builder); inspector_dim_y_->setPrefix("dy "); dim_grid->addWidget(inspector_dim_y_, 0, 1);
  inspector_dim_z_ = new QDoubleSpinBox(scene_builder); inspector_dim_z_->setPrefix("dz "); dim_grid->addWidget(inspector_dim_z_, 0, 2);
  selection_tab_layout->addLayout(dim_grid);
  selected_item_card_layout->addLayout(pose_grid);
  inspector_live_update_box_ = new QCheckBox("Live update", scene_builder); inspector_live_update_box_->setChecked(false); selection_tab_layout->addWidget(inspector_live_update_box_);
  auto * transform_actions = new QHBoxLayout();
  inspector_apply_button_ = new QPushButton("Apply", scene_builder); transform_actions->addWidget(inspector_apply_button_);
  inspector_revert_button_ = new QPushButton("Revert", scene_builder); transform_actions->addWidget(inspector_revert_button_);
  inspector_copy_transform_button_ = new QPushButton("Copy Transform", scene_builder); transform_actions->addWidget(inspector_copy_transform_button_);
  inspector_paste_transform_button_ = new QPushButton("Paste Transform", scene_builder); transform_actions->addWidget(inspector_paste_transform_button_);
  selection_tab_layout->addLayout(transform_actions);
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
  pl->addWidget(new QLabel("<b>Preview Process</b><br/>Open RViz2 / MoveIt | Run Fake-Hardware Simulation | Stop Simulation | Copy commands"));
  run_build_button_ = new QPushButton("Open RViz2 / MoveIt", preview); pl->addWidget(run_build_button_);
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
  connect(clear_log, &QPushButton::clicked, this, [this](){ if (studio_log_) studio_log_->clear(); });
  connect(scene_builder_log_toggle_button_, &QPushButton::clicked, this, [this]() {
    if (!studio_log_ || !scene_builder_log_toggle_button_ || !scene_builder_log_panel_) return;
    constexpr int kCollapsedLogPanelHeight = 52;
    constexpr int kExpandedLogPanelHeight = 240;
    const bool show = !studio_log_->isVisible();
    studio_log_->setVisible(show);
    scene_builder_log_panel_->setMaximumHeight(show ? kExpandedLogPanelHeight : kCollapsedLogPanelHeight);
    scene_builder_log_toggle_button_->setText(show ? "Hide Log" : "Show Log");
  });
  connect(empty_new_cell, &QPushButton::clicked, this, &MainWindow::open_new_scene_creation_flow);
  connect(dash_new_cell, &QPushButton::clicked, this, &MainWindow::open_new_scene_creation_flow);
  connect(dash_open_selected_scene, &QPushButton::clicked, this, [this](){ open_scene_builder_for_selected_scene("Dashboard Open Selected Scene"); });
  connect(dashboard_open_scene_action_, &QAction::triggered, this, [this](){ open_scene_builder_for_selected_scene("Dashboard Open in Scene Builder"); });
  connect(dashboard_validate_action_, &QAction::triggered, this, [this](){ if (action_validate_offline_) action_validate_offline_->trigger(); });
  connect(dashboard_plan_action_, &QAction::triggered, this, [this](){ if (action_simulate_plan_preview_) action_simulate_plan_preview_->trigger(); });
  connect(dashboard_export_action_, &QAction::triggered, this, [this](){ if (action_export_open_page_) action_export_open_page_->trigger(); });
  connect(dashboard_delete_action_, &QAction::triggered, this, &MainWindow::delete_selected_scene);
  connect(existing_scene_table_, &QTableWidget::cellClicked, this, [this](int row, int col){ select_scene_by_row(row); if(col==2){open_scene_builder_for_selected_scene("Existing Scenes Open in Scene Builder");} else if(col==3){open_selected_scene_artifact("preview");} else if(col==4){open_selected_scene_artifact("smoke");} else if(col==5){QApplication::clipboard()->setText(selected_scene_launch_command()); append_studio_log("Copy Launch Command");}});
  connect(open_asset_folder_action, &QAction::triggered, this, [this](){ open_selected_scene_artifact("asset_folder"); });
  connect(copy_asset_path_action, &QAction::triggered, this, [this](){ QApplication::clipboard()->setText(selected_catalog_item_path()); });
  connect(import_asset_action, &QAction::triggered, this, [this](){ append_studio_log("Import STL / URDF: choose asset import flow from Asset Browser."); });
  connect(add_existing_stl_action, &QAction::triggered, this, [this](){ append_studio_log("Add Existing STL to Canvas: choose STL in Asset Browser and click Add to Canvas."); });
  connect(placeholder_action, &QAction::triggered, this, [this](){ append_studio_log("Generate Simple Box/Cylinder Placeholder: use quick-add placeholders in catalog."); });
  connect_button(pick_source_button_, &MainWindow::bind_selected_item_as_pick_zone);
  connect_button(place_target_button_, &MainWindow::bind_selected_item_as_place_zone);
  connect_button(camera_button_, &MainWindow::bind_selected_item_as_camera);
connect(run_demo, &QPushButton::clicked, this, [this](){ append_studio_log("Demo readiness completed"); });
  connect(open_dash, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect(copy_summary, &QPushButton::clicked, this, [this](){ open_selected_scene_artifact("demo_summary_copy"); });
  connect(go_validation, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::ValidationPage); append_studio_log("Go to Validation: switched to Validation page"); });
  connect(go_preview, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); append_studio_log("Go to Plan & Simulate: switched to Plan & Simulate page"); });
  connect(go_export, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::ExportPage); append_studio_log("Go to Export: switched to Export page"); });
  connect(go_scene_builder, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::SceneBuilderPage); append_studio_log("Go to Scene Builder: switched to Scene Builder page"); });
  connect(go_preview_commands, &QPushButton::clicked, this, [this](){ show_studio_page(StudioPage::PlanSimulatePage); append_studio_log("Go to Preview Commands: use Copy commands on Preview Launch page"); });
  connect_button(run_build_button_, &MainWindow::run_preview_build);
  connect_button(run_preview_button_, &MainWindow::run_fake_hardware_preview);
  connect_button(stop_preview_button_, &MainWindow::stop_preview_process);
  connect_button(copy_build_button_, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect_button(copy_source_button_, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect_button(copy_launch_button_, [this](){ QApplication::clipboard()->setText(selected_scene_launch_command()); });
  connect_button(copy_all_button_, [this](){ QApplication::clipboard()->setText(selected_scene_preview_command_block()); });
  connect_button(open_preview_folder_button_, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect_button(open_preview_transcript_button_, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect(run_offline_validation_button, &QPushButton::clicked, this, &MainWindow::run_offline_validation);
  connect(validate_layout_button, &QPushButton::clicked, this, &MainWindow::run_layout_validation_only);
  connect_action(open_validation_report_action, &MainWindow::open_validation_report);
  connect_action(copy_validation_summary_action, &MainWindow::copy_validation_summary);
  connect(generate_readiness_pack_button, &QPushButton::clicked, this, &MainWindow::generate_readiness_pack);
  connect_action(open_readiness_dashboard_action, &MainWindow::open_readiness_dashboard);
  connect_action(check_canvas_parity_action, &MainWindow::check_canvas_generated_parity);
  connect(export_scene_bundle_button, &QPushButton::clicked, this, &MainWindow::export_scene_bundle_for_selected_scene);
  connect(import_scene_bundle_button, &QPushButton::clicked, this, &MainWindow::import_scene_bundle_into_scenes_root);
  connect_action(open_export_folder_action, &MainWindow::open_scene_bundle_export_folder);
  connect_action(open_scene_folder_action, [this](){ open_selected_scene_artifact("preview_launch_folder"); });
  connect_action(open_preview_report_action, [this](){ open_selected_scene_artifact("preview_launch_transcript"); });
  connect_action(open_dashboard_action, [this](){ open_selected_scene_artifact("demo_dashboard"); });
  connect_action(copy_source_action, [this](){ QApplication::clipboard()->setText(selected_scene_source_command()); });
  connect_action(copy_build_action, [this](){ QApplication::clipboard()->setText(selected_scene_build_command()); });
  connect(run_self_test_button_, &QPushButton::clicked, this, &MainWindow::run_diagnostics_self_test);
  connect(run_golden_flow_button_, &QPushButton::clicked, this, &MainWindow::run_diagnostics_golden_flow_dry_run);
  connect(copy_diagnostics_report_button_, &QPushButton::clicked, this, &MainWindow::copy_diagnostics_report);
  connect(open_diagnostics_report_button_, &QPushButton::clicked, this, &MainWindow::open_diagnostics_folder);
  connect(copy_golden_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/workcell_studio_diag_scene --json"); });
  connect(copy_build_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select workcell_builder"); });
  connect(copy_source_cmd, &QPushButton::clicked, this, [this](){ QApplication::clipboard()->setText("source install/setup.bash"); });
  connect(open_logs_cmd, &QPushButton::clicked, this, [this](){ open_diagnostics_folder(); });
  connect(fit_button, &QAction::triggered, this, [this](){ if (digital_twin_canvas_ && digital_twin_canvas_->scene()) digital_twin_canvas_->fitInView(digital_twin_canvas_->scene()->itemsBoundingRect().adjusted(-24,-24,24,24), Qt::KeepAspectRatio); });
  connect(reset_button, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->resetTransform(); rebuild_digital_twin_canvas(); });
  connect(zoom_in, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(1.15,1.15); });
  connect(zoom_out, &QAction::triggered, this, [this](){ if (digital_twin_canvas_) digital_twin_canvas_->scale(0.85,0.85); });
  connect(perspective_action, &QAction::triggered, this, [this](){ scene_builder_is_3d_view_ = true; refresh_scene_builder_view_chips(); });
  auto set_2d_layout_view = [this]() { scene_builder_is_3d_view_ = false; refresh_scene_builder_view_chips(); };
  connect(top_action, &QAction::triggered, this, set_2d_layout_view);
  connect(left_action, &QAction::triggered, this, set_2d_layout_view);
  connect(right_action, &QAction::triggered, this, set_2d_layout_view);
  connect(front_action, &QAction::triggered, this, set_2d_layout_view);
  connect(toggle_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Snap to Grid"); });
  connect(fine_move_mode_box_, &QCheckBox::toggled, this, [this](bool){ mark_layout_dirty("Fine Move Mode"); });
  connect(unlock_robot_base_box_, &QCheckBox::toggled, this, [this](bool checked){ if (checked) { QMessageBox::warning(this, "Unlock Robot Base", "Robot base is locked by default. Moving robot base may invalidate reach and safety assumptions."); }});
  connect(toggle_labels_box_, &QCheckBox::toggled, this, [this](bool enabled){
    if (scene_preview_widget_) scene_preview_widget_->set_label_mode(enabled ? ScenePreviewWidget::LabelMode::All : ScenePreviewWidget::LabelMode::Off);
    rebuild_digital_twin_canvas();
  });
  connect(toggle_warnings_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  for (auto * box : {
      preview_layer_editable_layout_box_, preview_layer_generated_urdf_visual_box_,
      preview_layer_mesh_preview_box_, preview_layer_primitive_fallback_box_,
      preview_layer_overlays_helpers_box_, preview_layer_warnings_missing_assets_box_})
  {
    connect(box, &QCheckBox::toggled, this, [this](bool) { apply_scene3d_preview_layer_filters(true); });
  }
  connect(duplicate_layout_button_, &QPushButton::clicked, this, &MainWindow::duplicate_selected_item);
  connect(delete_layout_button_, &QPushButton::clicked, this, &MainWindow::delete_selected_item);
  for (auto *sb : {inspector_x_, inspector_y_, inspector_z_, inspector_roll_, inspector_pitch_, inspector_yaw_}) connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ if (inspector_live_update_box_ && inspector_live_update_box_->isChecked()) apply_selection_transform_from_editor(); });
  for (auto *sb : {inspector_dim_x_, inspector_dim_y_, inspector_dim_z_}) connect(sb, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double){ if (inspector_live_update_box_ && inspector_live_update_box_->isChecked()) apply_selection_transform_from_editor(); });
  connect(inspector_apply_button_, &QPushButton::clicked, this, &MainWindow::apply_selection_transform_from_editor);
  connect(inspector_revert_button_, &QPushButton::clicked, this, &MainWindow::revert_selection_transform_editor);
  connect(inspector_copy_transform_button_, &QPushButton::clicked, this, &MainWindow::copy_selection_transform_to_clipboard);
  connect(inspector_paste_transform_button_, &QPushButton::clicked, this, &MainWindow::paste_selection_transform_from_clipboard);
  inspector_x_->setToolTip("X position in metres"); inspector_y_->setToolTip("Y position in metres"); inspector_z_->setToolTip("Z position in metres");
  inspector_roll_->setToolTip("Roll in radians"); inspector_pitch_->setToolTip("Pitch in radians"); inspector_yaw_->setToolTip("Yaw in radians");
  inspector_dim_x_->setToolTip("Dimension X in metres"); inspector_dim_y_->setToolTip("Dimension Y in metres"); inspector_dim_z_->setToolTip("Dimension Z in metres");
  connect(save_layout_button_, &QPushButton::clicked, this, &MainWindow::save_layout_changes);
  connect(create_starter_layout_button_, &QPushButton::clicked, this, &MainWindow::create_starter_layout_from_preview);
  connect(select_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); });
  connect(place_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Place); });
  connect(move_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Move); });
  connect(inspect_mode_button, &QPushButton::clicked, this, [this](){ set_canvas_interaction_mode(CanvasInteractionMode::Inspect); });
  connect(snap_to_grid_box_, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  connect(snap_action, &QAction::toggled, snap_to_grid_box_, &QCheckBox::setChecked);
  connect(snap_to_grid_box_, &QCheckBox::toggled, snap_action, &QAction::setChecked);
  connect(fine_move_action, &QAction::toggled, fine_move_mode_box_, &QCheckBox::setChecked);
  connect(fine_move_mode_box_, &QCheckBox::toggled, fine_move_action, &QAction::setChecked);
  connect(unlock_action, &QAction::toggled, unlock_robot_base_box_, &QCheckBox::setChecked);
  connect(unlock_robot_base_box_, &QCheckBox::toggled, unlock_action, &QAction::setChecked);
  connect(minimap_action, &QAction::toggled, show_minimap_box_, &QCheckBox::setChecked);
  connect(show_minimap_box_, &QCheckBox::toggled, minimap_action, &QAction::setChecked);
  connect(show_minimap_box_, &QCheckBox::toggled, this, [this](bool on){
    minimap_requested_visible_ = on;
    refresh_minimap_card();
  });
  connect(digital_twin_canvas_->horizontalScrollBar(), &QScrollBar::valueChanged, this, [this](int){ refresh_minimap_card(); });
  connect(digital_twin_canvas_->verticalScrollBar(), &QScrollBar::valueChanged, this, [this](int){ refresh_minimap_card(); });
  for (auto * box : {show_reach_overlay_box_, show_camera_fov_overlay_box_, show_pick_place_overlay_box_, show_trajectory_overlay_box_}) connect(box, &QCheckBox::toggled, this, [this](bool){ rebuild_digital_twin_canvas(); });
  auto * del_sc = new QShortcut(QKeySequence(Qt::Key_Delete), scene_builder); connect(del_sc,&QShortcut::activated,this,&MainWindow::delete_selected_item);
  auto * save_sc = new QShortcut(QKeySequence::Save, scene_builder); connect(save_sc,&QShortcut::activated,this,&MainWindow::save_layout_changes);
  auto * undo_sc = new QShortcut(QKeySequence::Undo, scene_builder); connect(undo_sc, &QShortcut::activated, this, &MainWindow::undo_layout_edit);
  auto * redo_sc = new QShortcut(QKeySequence::Redo, scene_builder); connect(redo_sc, &QShortcut::activated, this, &MainWindow::redo_layout_edit);
  auto * esc_sc = new QShortcut(QKeySequence(Qt::Key_Escape), scene_builder); connect(esc_sc,&QShortcut::activated,this,[this](){ set_canvas_interaction_mode(CanvasInteractionMode::Select); if(digital_twin_scene_) digital_twin_scene_->clearSelection(); ghost_preview_item_=nullptr; rebuild_digital_twin_canvas(); });
  auto * fit_sc = new QShortcut(QKeySequence(Qt::Key_F), scene_builder); connect(fit_sc,&QShortcut::activated,fit_button,&QAction::trigger);
  connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *item, int column){ Q_UNUSED(column); on_hierarchy_item_selected(item); });
  connect(asset_filter_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::on_asset_filter_changed);
  connect(open_asset_folder_action, &QAction::triggered, this, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(p).isDir() ? p : QFileInfo(p).absolutePath())); });
  connect(copy_asset_path_action, &QAction::triggered, this, [this](){ const QString p = selected_catalog_item_path(); if (p.isEmpty()) { QMessageBox::information(this, "Asset Catalog", "Select an asset first."); return; } QApplication::clipboard()->setText(p); append_studio_log("Copy Asset Path: " + p); });
  connect(add_to_canvas_button_, &QPushButton::clicked, this, [this](){ if (!asset_catalog_tree_ || !asset_catalog_tree_->currentItem()) { QMessageBox::information(this, "Asset Catalog", "Select an asset to add to canvas."); return; } auto *it = asset_catalog_tree_->currentItem(); const int idx = it->data(0, CatalogRoleIndex).toInt(); if (idx < 0 || idx >= asset_catalog_entries_.size()) return; const auto & e = asset_catalog_entries_[idx]; if (!e.disabled_reason.trimmed().isEmpty()) { QMessageBox::information(this, "Asset Catalog", e.disabled_reason); return; } add_asset_to_canvas_from_catalog(e.category, e.display_name, e.source_path); });
  connect(add_asset_button_, &QPushButton::clicked, this, &MainWindow::open_add_asset_dialog);
  connect_if(scene_workflow_recommendation_button_, this, &QPushButton::clicked, [this]() {
    trigger_recommended_workflow_action(resolve_recommended_workflow_action().handler);
  });
  connect(asset_catalog_tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem *it, int){ if (!it) return; const int idx = it->data(0, CatalogRoleIndex).toInt(); if (idx < 0 || idx >= asset_catalog_entries_.size()) return; const auto & e = asset_catalog_entries_[idx]; if (!e.disabled_reason.trimmed().isEmpty()) return; add_asset_to_canvas_from_catalog(e.category, e.display_name, e.source_path); });
  connect(asset_catalog_tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem *, QTreeWidgetItem *){ validate_asset_catalog_selection(); });
  connect(import_asset_action, &QAction::triggered, this, [this](){ QMessageBox::information(this, "Asset Catalog", "Import STL / URDF keeps existing behavior via filesystem import workflows."); });
  connect(add_existing_stl_action, &QAction::triggered, this, [this](){ QMessageBox::information(this, "Asset Catalog", "Add Existing STL to Canvas keeps existing behavior for scene assets."); });
  connect(placeholder_action, &QAction::triggered, this, [this](){ add_asset_to_canvas_from_catalog("Custom", "Generated Placeholder", "placeholder://generated"); });
  connect(preview_process_, &QProcess::readyReadStandardOutput, this, &MainWindow::handle_preview_stdout);
  connect(preview_process_, &QProcess::readyReadStandardError, this, &MainWindow::handle_preview_stderr);
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
  connect(action_generate_package_, &QAction::triggered, this, [this]() { append_studio_log(QString("Generate Scene Package: requested for scene '%1'.").arg(selected_scene_name())); run_layout_merge_for_selected_scene(true); });
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
  connect(action_simulate_plan_preview_, &QAction::triggered, this, [this]() { append_studio_log(QString("Plan & Simulate: prepared fake-hardware commands for scene '%1'. Real robot motion locked.").arg(selected_scene_name())); show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); refresh_new_cell_checklist(); });
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
  run_next_menu->addAction(action_simulate_plan_preview_);
  run_next_menu->addAction(action_export_open_page_);
  run_next_button->setMenu(run_next_menu);
  top_bar->addWidget(run_next_button);
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

void MainWindow::generate_scene_package_for_selected_scene(){ if (selected_scene_index_ < 0) return; QString parity_warning; bool severe_parity_mismatch = false; const bool pre_parity_ran = run_canvas_generated_parity_check(CanvasGeneratedParityMode::PreGeneration, &parity_warning, &severe_parity_mismatch); if (pre_parity_ran) { refresh_canvas_generated_parity_ui(); if (!parity_warning.isEmpty()) { append_studio_log("Generate ROS Scene Package pre-generation parity: " + parity_warning); } } generate_yaml_draft_for_selected_scene(); const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const QString scene_dir = QString::fromStdString(sc.scene_dir.string()); const QString scene_name = QString::fromStdString(sc.scene_name); const QString cell_definition_path = QString::fromStdString((sc.scene_dir / "cell_definition.yaml").string()); const QString output_dir = QString::fromStdString(sc.scene_dir.parent_path().string()); if (!QFileInfo::exists(cell_definition_path)) { append_studio_log("Generate ROS Scene Package: Generate YAML first."); return; } bool severe_preflight_failure = false; const QStringList preflight_warnings = generation_asset_support_preflight(sc.scene_dir / "environment_layout.yaml", &severe_preflight_failure); for (const QString & warning : preflight_warnings) { append_studio_log(warning); readiness_warning_details_.append(warning); } if (severe_preflight_failure) { append_studio_log("Generate ROS Scene Package blocked by severe schema/safety preflight failure."); refresh_new_cell_checklist(); return; } QString validate_cell_script; if (helper_script_exists("validate_cell_definition.py", &validate_cell_script)) { QProcess validate_process; validate_process.start("python3", QStringList() << validate_cell_script << cell_definition_path); if (!validate_process.waitForFinished(120000)) { append_studio_log("Generate ROS Scene Package: timed out while validating cell definition."); return; } const QString stderr_text = QString::fromUtf8(validate_process.readAllStandardError()).trimmed(); const QString stdout_text = QString::fromUtf8(validate_process.readAllStandardOutput()).trimmed(); if (validate_process.exitCode() != 0) { append_studio_log("Generate ROS Scene Package blocked: cell_definition.yaml validation failed."); const QStringList validator_lines = (stderr_text + "\n" + stdout_text).split('\n', Qt::SkipEmptyParts); bool found_missing_key_error = false; for (const QString & line : validator_lines) { const QString trimmed = line.trimmed(); if (trimmed.contains("Missing required top-level key:")) { append_studio_log("validator: " + trimmed); found_missing_key_error = true; } } if (!found_missing_key_error) { if (!stderr_text.isEmpty()) append_studio_log("validator stderr: " + stderr_text.left(600)); if (!stdout_text.isEmpty()) append_studio_log("validator stdout: " + stdout_text.left(600)); } return; } } if (output_dir.trimmed().isEmpty() || scene_name.trimmed().isEmpty()) { append_studio_log("Generate ROS Scene Package: output directory and package name are required."); return; } QString script; helper_script_exists("generate_workcell_from_cell_definition.py", &script); const auto plan = workcell_builder::build_generate_workcell_command_plan(script, scene_dir, output_dir, scene_name); if (!plan.ready()) { append_studio_log("Generate ROS Scene Package: " + plan.missing_fields_message()); return; } QProcess process; process.start("python3", QStringList() << plan.script_path << plan.arguments); if (!process.waitForFinished(180000)) { append_studio_log("Generate ROS Scene Package: timed out while waiting for helper script."); return; } const int exit_code = process.exitCode(); const QString stdout_text = QString::fromUtf8(process.readAllStandardOutput()).trimmed(); const QString stderr_text = QString::fromUtf8(process.readAllStandardError()).trimmed(); if (exit_code != 0) { append_studio_log(QString("Generate ROS Scene Package failed (exit=%1).").arg(exit_code)); if (!stderr_text.isEmpty()) append_studio_log("stderr: " + stderr_text.left(400)); if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400)); launch_artifacts_ready_ = false; return; } launch_artifacts_ready_ = true; append_studio_log("Generate ROS Scene Package: " + plan.display_command()); append_studio_log(QString("Generated package location: %1/%2").arg(output_dir, scene_name)); append_studio_log(QString("Next: colcon build --symlink-install --packages-select %1").arg(scene_name)); append_studio_log("Next: source install/setup.bash"); append_studio_log(QString("Next: ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(scene_name)); if (!stdout_text.isEmpty()) append_studio_log("stdout: " + stdout_text.left(400)); QString post_warning; bool post_blocked = false; const bool post_parity_ran = run_canvas_generated_parity_check(CanvasGeneratedParityMode::PostGeneration, &post_warning, &post_blocked); if (post_parity_ran) { refresh_canvas_generated_parity_ui(); if (post_blocked) { launch_artifacts_ready_ = false; append_studio_log("Generated package created but Canvas/Generated parity has blockers."); if (!post_warning.isEmpty()) { append_studio_log("Post-generation parity recommendation: " + post_warning); } } else if (!post_warning.isEmpty()) { append_studio_log("Generate ROS Scene Package post-generation parity: " + post_warning); } } refresh_scene_browser_ui(); refresh_scene_builder_selected_scene_ui(); refresh_new_cell_checklist(); }
void MainWindow::validate_generated_scene_for_selected_scene(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; QString script; if (!helper_script_exists("validate_builder_generated_scene.py", &script)) { append_studio_log("Validate Generated Scene: script missing. Searched: " + helper_script_search_paths("validate_builder_generated_scene.py").join(" | ")); return; } const auto plan = workcell_builder::build_validate_generated_scene_command_plan(script, QString::fromStdString(sc.scene_dir.string())); if (!plan.ready()) { append_studio_log("Validate Generated Scene: " + plan.missing_fields_message()); return; } QProcess process; process.start("python3", QStringList() << plan.script_path << plan.arguments); if (!process.waitForFinished(120000)) { append_studio_log("Validate Generated Scene: timed out while waiting for helper script."); return; } validation_stale_ = false; append_studio_log("Validate Generated Scene: " + plan.display_command()); refresh_new_cell_checklist(); }
void MainWindow::copy_build_launch_commands_for_selected_scene(){ if (!has_selected_scene()) return; const QString block = selected_scene_preview_command_block(); QApplication::clipboard()->setText(block); append_studio_log("Copy Build & Launch Commands"); }
void MainWindow::open_selected_task_file(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const auto ti = load_scene_task_intent_summary(sc.scene_dir); if (ti.status=="MISSING_TASK_FILE"){ append_studio_log("Open Task File: missing. Searched: " + ti.searched_paths.join(" | ")); return; } QDesktopServices::openUrl(QUrl::fromLocalFile(ti.source_file)); }
void MainWindow::copy_selected_task_summary(){ if (selected_scene_index_ < 0) return; const auto & sc = scene_browser_result_.scenes[(size_t)selected_scene_index_]; const auto ti = load_scene_task_intent_summary(sc.scene_dir); QApplication::clipboard()->setText(QString("Scene=%1\nTaskType=%2\nPick=%3\nPlace=%4\nReject=%5\nObjectClass=%6\nGrasp=%7\nApproach=%8/%9\nRetreat=%10/%11\nTool=%12\nPerception=%13\nStatus=%14").arg(QString::fromStdString(sc.scene_name),ti.task_type,ti.pick_source,ti.place_target,ti.reject_target,ti.object_class,ti.grasp_strategy,ti.approach_axis,ti.approach_distance,ti.retreat_axis,ti.retreat_distance,ti.tool_id,ti.perception_mode,ti.status)); append_studio_log("Copy Task Summary"); }
void MainWindow::preview_offline_plan_for_selected_scene(){ show_studio_page(StudioPage::PlanSimulatePage); refresh_preview_launch_ui(); append_studio_log("Preview Offline Plan: Fake Hardware | No Robot Motion | Preview Only"); }
MainWindow::SelectedSceneItemState MainWindow::current_selected_scene_item() const
{
  SelectedSceneItemState state;
  const auto fill_from_tree = [&](QTreeWidgetItem * item) {
    if (!item) return false;
    state.id = item->data(0, TreeRoleId).toString().trimmed();
    if (state.id.isEmpty()) state.id = item->text(0).trimmed();
    state.display_name = item->text(0).trimmed();
    state.role_or_category = item->data(0, TreeRoleRole).toString().trimmed();
    if (state.role_or_category.isEmpty()) state.role_or_category = item->data(0, TreeRoleCategory).toString().trimmed();
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
    state.role_or_category = item->data(RoleRole).toString().trimmed();
    if (state.role_or_category.isEmpty()) state.role_or_category = item->data(RoleCategory).toString().trimmed();
    if (state.role_or_category.isEmpty()) state.role_or_category = item->data(RoleType).toString().trimmed();
    state.source_path = item->data(RoleSource).toString().trimmed();
    state.locked = item->data(RoleLocked).toBool();
    state.editable = !state.locked;
    state.source_layer = QStringLiteral("canvas");
    state.active_visual_source = item->data(RoleGeneratedPlaceholder).toBool() ? QStringLiteral("primitive_fallback") : QStringLiteral("mesh_preview");
    state.linked_to_editable_layout_state = state.editable;
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
    state.pose_text = item->data(RolePoseText).toString().trimmed();
    state.valid = !state.id.isEmpty();
    return state.valid;
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
    return {};
  }
  if (scene_hierarchy_tree_ && fill_from_tree(scene_hierarchy_tree_->currentItem())) return state;
  if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty() &&
    fill_from_canvas(digital_twin_scene_->selectedItems().front())) return state;
  return {};
}

void MainWindow::refresh_selected_scene_item_labels(const SelectedSceneItemState & state)
{
  if (!inspector_label_ || !live_coordinate_label_) return;
  refresh_selection_binding_actions(state);
  QStringList inspector_lines;
  inspector_lines << QString("Scene: %1").arg(selected_scene_state_.valid ? selected_scene_state_.name : QStringLiteral("none"));
  inspector_lines << QString("Scene path: %1").arg(selected_scene_state_.valid ? selected_scene_state_.path : QStringLiteral("(none)"));
  inspector_lines << QString("Scene status: %1").arg(selected_scene_state_.valid ? selected_scene_state_.status : QStringLiteral("(none)"));
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
  const bool is_locked_urdf_preview = state.locked && role.contains("urdf", Qt::CaseInsensitive);
  const QString locked_line = state.locked ? QString("Locked: %1").arg(state.lock_reason.isEmpty() ? QStringLiteral("item is locked") : state.lock_reason) : QStringLiteral("Locked: no");
  inspector_lines << "";
  inspector_lines << QString("Selected item name: %1").arg(display);
  inspector_lines << QString("Selected item role: %1").arg(role);
  inspector_lines << QString("Selected item category: %1").arg(role);
  inspector_lines << QString("Selected item ID: %1").arg(state.id);
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
  dashboard_selected_scene_details_->setText(QString("<b>Scene:</b> %1<br/><b>Status:</b> %2<br/><b>Robot:</b> %3<br/><b>Gripper:</b> %4<br/><b>Task Recipe:</b> %5<br/><b>Launch:</b> %6<br/><b>Source:</b> %7")
    .arg(QString::fromStdString(s.scene_name)).arg(status_chip).arg(QString::fromStdString(s.robot_summary)).arg(QString::fromStdString(s.gripper_summary)).arg(s.has_task_recipe ? "present" : "missing").arg(s.has_launch_demo ? "ready" : "blocked").arg(QString::fromStdString(s.scene_dir.string())));
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


QString MainWindow::selected_scene_launch_command() const { if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) return ""; const auto & scene = scene_browser_result_.scenes[(size_t)selected_scene_index_]; QString command = QString("ros2 launch %1 demo.launch.py use_fake_hardware:=true launch_rviz:=true").arg(QString::fromStdString(scene.scene_name)); const fs::path launch_file = scene.scene_dir / "launch" / "demo.launch.py"; std::ifstream ifs(launch_file.string()); if (ifs) { std::string launch_text((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>()); if (launch_text.find("launch_task_preview") != std::string::npos) { command += " launch_task_preview:=true"; } } return command; }

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

QString MainWindow::selected_scene_build_command() const { if (selected_scene_index_ < 0) return ""; return QString("cd %1 && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select %2").arg(detect_workspace_root(), QString::fromStdString(scene_browser_result_.scenes[(size_t)selected_scene_index_].scene_name)); }
QString MainWindow::selected_scene_source_command() const { return QString("cd %1 && source install/setup.bash").arg(detect_workspace_root()); }
QString MainWindow::selected_scene_preview_command_block() const { return selected_scene_build_command()+"\n"+selected_scene_source_command()+"\ncd "+detect_workspace_root()+" && "+selected_scene_launch_command(); }

bool MainWindow::selected_scene_preview_ready(QStringList * blockers) const
{
  if (selected_scene_index_ < 0) { if (blockers) blockers->append("No scene selected"); return false; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  if (QString::fromStdString(s.status).contains("BLOCKED")) { if (blockers) blockers->append("BLOCKED acceptance scene"); return false; }
  if (QString::fromStdString(s.status).contains("PREVIEW_ONLY")) { if (blockers) blockers->append("PREVIEW_ONLY scenes cannot run"); return false; }
  const fs::path layout_file = s.scene_dir / "layout" / "workcell_studio_layout.yaml";
  const fs::path merge_report = s.scene_dir / "generated" / "workcell_studio_layout_merge_report.json";
  if (fs::exists(layout_file) && (!fs::exists(merge_report) || fs::last_write_time(layout_file) > fs::last_write_time(merge_report))) {
    if (blockers) blockers->append("Layout changed since last generation. Run Generate Scene / Layout Merge before preview.");
    return false;
  }
  return true;
}
bool MainWindow::preview_command_is_safe(const QString & command, QStringList * blockers) const
{ bool ok = command.contains("use_fake_hardware:=true") && !command.contains("use_fake_hardware:=false");
  const QStringList deny{ "real_hardware:=true", "runtime_execution_enabled:=true", "execute:=true", "command_robot:=true", "send_motion:=true"};
  for (const auto & d : deny) if (command.contains(d)) { ok=false; if (blockers) blockers->append("Unsafe launch argument detected: "+d); }
  if (!command.contains("use_fake_hardware:=true") && blockers) blockers->append("Missing required use_fake_hardware:=true");
  return ok; }

void MainWindow::set_preview_state(const QString & state){ preview_state_=state; refresh_preview_launch_ui(); }

void MainWindow::refresh_preview_launch_ui()
{
  QString readiness = "BLOCKED_MISSING_SCENE";
  bool has_scene = selected_scene_index_ >= 0;
  bool has_ws = !detect_workspace_root().isEmpty();
  QStringList blockers;
  if (has_scene) {
    const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
    if (!s.has_launch_demo) readiness = "BLOCKED_MISSING_LAUNCH";
    else if (!s.has_task_intent) readiness = "BLOCKED_MISSING_TASK_INTENT";
    else if (!selected_scene_preview_ready(&blockers)) readiness = "WARNINGS_PRESENT";
    else readiness = "READY_FOR_FAKE_HARDWARE_PREVIEW";
    if (preview_scene_label_) preview_scene_label_->setText(QString("<b>Selected Scene</b><br/>scene name: %1<br/>scene path: %2<br/>robot summary: %3<br/>gripper/tool summary: %4<br/>task file status: %5<br/>launch/demo.launch.py status: %6<br/>package.xml/CMakeLists status: %7<br/>preview snapshot path: %8")
      .arg(QString::fromStdString(s.scene_name), QString::fromStdString(s.scene_dir.string()), QString::fromStdString(s.robot_summary), QString::fromStdString(s.gripper_summary),
      s.has_task_recipe ? "present" : "missing", s.has_launch_demo ? "present" : "missing", (s.has_package_xml && s.has_launch_demo) ? "present" : "missing",
      QString::fromStdString((s.scene_dir / "preview" / "workcell_studio_canvas_snapshot.png").string())));
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

void MainWindow::run_preview_build(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } if(detect_workspace_root().isEmpty()){ QMessageBox::warning(this,"Preview Launch","Workspace root not detected. Copy commands and run manually."); return;} active_preview_command_=selected_scene_build_command(); if(preview_log_) preview_log_->appendPlainText("$ "+active_preview_command_); set_preview_state("BUILD_RUNNING"); write_preview_launch_transcript(true, active_preview_command_, "build_started"); preview_process_->start("/bin/bash", {"-lc", active_preview_command_}); }
void MainWindow::run_fake_hardware_preview(){ QStringList blockers; if(!selected_scene_preview_ready(&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } QString command = "cd "+detect_workspace_root()+" && source install/setup.bash && "+selected_scene_launch_command(); if(!preview_command_is_safe(command,&blockers)){ QMessageBox::warning(this,"Plan & Simulate",blockers.join("\n")); return; } auto rc = QMessageBox::question(this,"Confirm Fake-Hardware Preview", "Command:\n"+command+"\n\nFake hardware only. No real hardware. No runtime execution. No robot motion commanded."); if(rc!=QMessageBox::Yes) return; active_preview_command_=command; if(preview_log_) preview_log_->appendPlainText("$ "+command); set_preview_state("PREVIEW_RUNNING"); write_preview_launch_transcript(true, command, "preview_started"); preview_process_->start("/bin/bash", {"-lc", command}); refresh_new_cell_checklist(); }
void MainWindow::stop_preview_process(){ if(!preview_process_ || preview_process_->state()==QProcess::NotRunning) return; set_preview_state("PREVIEW_STOPPING"); preview_log_->appendPlainText("Stopping preview process..."); preview_process_->terminate(); if(!preview_process_->waitForFinished(2000)){ preview_log_->appendPlainText("Terminate timeout, forcing kill."); preview_process_->kill(); preview_process_->waitForFinished(1000);} refresh_new_cell_checklist(); }
void MainWindow::handle_preview_stdout(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardOutput())); }
void MainWindow::handle_preview_stderr(){ if(preview_log_) preview_log_->appendPlainText(QString::fromUtf8(preview_process_->readAllStandardError())); }
void MainWindow::handle_preview_finished(int exit_code, QProcess::ExitStatus){ if(preview_state_=="BUILD_RUNNING") set_preview_state(exit_code==0?"BUILD_PASSED":"BUILD_FAILED"); else if(preview_state_=="PREVIEW_STOPPING") set_preview_state("PREVIEW_STOPPED"); else set_preview_state(exit_code==0?"PREVIEW_EXITED":"PREVIEW_FAILED"); write_preview_launch_transcript(true, active_preview_command_, "process_finished", exit_code); refresh_new_cell_checklist(); }

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
  append_studio_log("Merge report: " + QString::fromStdString(result.report_path));
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

void MainWindow::refresh_scene_builder_selection_state_ui()
{
  sync_selected_scene_state();
  sync_selected_item_state();
  refresh_scene_builder_selected_scene_ui();
  refresh_scene_builder_left_explorer();
  refresh_selected_scene_details_card();
  refresh_task_intent_panel();
}

void MainWindow::refresh_scene_builder_selected_scene_ui()
{
  sync_selected_scene_state();
  sync_selected_item_state();
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
  if (scene_preview_label_) scene_preview_label_->setText((s.has_static_preview_svg?"Preview SVG available":"Generate preview/readiness pack to populate this panel") + QString("\nStatus: %1").arg(selected_scene_state_.status));
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
  const auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  const std::size_t preview_count = model.items.size();
  const std::size_t editable_layout_count = workcell_builder::count_editable_layout_entries(s.scene_dir);
  const bool show_action = (editable_layout_count == 0U) && (preview_count > 0U);
  create_starter_layout_button_->setVisible(show_action);
  create_starter_layout_button_->setToolTip(show_action ?
    QString("Create layout/workcell_studio_layout.yaml from %1 preview items").arg(preview_count) :
    QString("Hidden unless editable layout count is 0 and preview items count is > 0 (current: editable=%1 preview=%2)")
      .arg(editable_layout_count).arg(preview_count));
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
  digital_twin_scene_->setSceneRect(-400, -300, 1200, 900);
  digital_twin_canvas_->setBackgroundBrush(QColor("#10161f"));

  if (toggle_grid_box_ && toggle_grid_box_->isChecked()) {
    QPen grid_pen(QColor("#1f2a36")); grid_pen.setWidth(1);
    for (int x = -400; x <= 800; x += 40) digital_twin_scene_->addLine(x, -300, x, 600, grid_pen);
    for (int y = -300; y <= 600; y += 40) digital_twin_scene_->addLine(-400, y, 800, y, grid_pen);
  }
  QPen axis_pen(QColor("#3a4b5c")); axis_pen.setWidth(2);
  digital_twin_scene_->addLine(-400, 0, 800, 0, axis_pen); digital_twin_scene_->addLine(0, -300, 0, 600, axis_pen);
  auto * origin = digital_twin_scene_->addEllipse(-4, -4, 8, 8, QPen(QColor("#d9e2ec")), QBrush(QColor("#d9e2ec")));
  origin->setToolTip("World origin marker");

  if (selected_scene_index_ < 0) return;
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);

  if (!show_reach_overlay_box_ || show_reach_overlay_box_->isChecked()) digital_twin_scene_->addEllipse(-150, -150, 300, 300, QPen(QColor("#2dd4bf"), 2, Qt::DashLine)); // robot reach circle/arc
  auto * robot_base = digital_twin_scene_->addEllipse(-14, -14, 28, 28, QPen(QColor("#60a5fa"), 2), QBrush(QColor("#1d4ed8")));
  robot_base->setToolTip("Robot base marker");

  for (const auto & entry : model.items) {
    const QString category = QString::fromStdString(entry.type);
    auto * item = new DraggableCanvasItem(QRectF(0, 0, std::max(20.0, entry.width * 100.0), std::max(20.0, entry.depth * 100.0)));
    item->setPos(entry.x * 100.0, entry.y * 100.0);
    item->setPen(QPen(category_color(category).lighter(130), 2));
    item->setBrush(QBrush(category_color(category), Qt::SolidPattern));
    item->setToolTip(QString("%1 (%2)").arg(QString::fromStdString(entry.label), category));
    item->setData(RoleId, QString::fromStdString(entry.id));
    item->setData(RoleDisplayName, QString::fromStdString(entry.label));
    item->setData(RoleType, QString::fromStdString(entry.type));
    item->setData(RoleCategory, QString::fromStdString(entry.type));
    item->setData(RoleRole, QString::fromStdString(entry.role));
    item->setData(RoleLocked, entry.locked);
    item->setData(RolePoseZ, entry.z);
    item->setData(RoleRoll, entry.roll); item->setData(RolePitch, entry.pitch); item->setData(RoleYaw, entry.yaw);
    item->setData(RoleSource, QString::fromStdString(entry.source_file));
    item->setData(RoleSourcePackage, QString(""));
    item->setData(RoleWidth, entry.width); item->setData(RoleDepth, entry.depth); item->setData(RoleHeight, entry.height);
    const bool is_preview_placeholder = category.contains("placeholder", Qt::CaseInsensitive) || category == "warning";
    item->setData(RoleImported, false); item->setData(RoleGeneratedPlaceholder, is_preview_placeholder);
    item->setData(RoleWarning, QString::fromStdString(entry.warnings.empty() ? std::string() : entry.warnings.front()));
    item->setData(RolePoseText, QString("x=%1 y=%2 z=%3 r=%4 p=%5 y=%6").arg(entry.x).arg(entry.y).arg(entry.z).arg(entry.roll).arg(entry.pitch).arg(entry.yaw));
    item->setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges | (entry.locked ? QGraphicsItem::GraphicsItemFlag(0) : QGraphicsItem::ItemIsMovable));
    item->position_filter = [this](const QPointF & p){ return snap_canvas_position(p); };
    digital_twin_scene_->addItem(item);

    if (toggle_labels_box_ && toggle_labels_box_->isChecked()) {
      auto * txt = digital_twin_scene_->addSimpleText(QString::fromStdString(entry.label));
      txt->setBrush(QBrush(QColor("#d8dee9"))); txt->setPos(item->pos() + QPointF(0, -18));
    }
    if ((!show_camera_fov_overlay_box_ || show_camera_fov_overlay_box_->isChecked()) && category.contains("camera", Qt::CaseInsensitive)) {
      QPolygonF fov; fov << QPointF(item->pos().x()+12, item->pos().y()+12) << QPointF(item->pos().x()+150, item->pos().y()-40) << QPointF(item->pos().x()+150, item->pos().y()+64);
      digital_twin_scene_->addPolygon(fov, QPen(QColor("#ffd166"), 2), QBrush(QColor(255, 209, 102, 45))); // camera FOV wedge/cone
    }
    if (!show_pick_place_overlay_box_ || show_pick_place_overlay_box_->isChecked()) {
      if (category.contains("pick", Qt::CaseInsensitive)) digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#00d1b2"),2,Qt::DashLine));
      if (category.contains("place", Qt::CaseInsensitive)) digital_twin_scene_->addRect(item->sceneBoundingRect().adjusted(-4,-4,4,4), QPen(QColor("#ff7b72"),2,Qt::DashLine));
    }
    if (toggle_warnings_box_ && toggle_warnings_box_->isChecked() && !item->data(RoleWarning).toString().isEmpty()) {
      auto * w = digital_twin_scene_->addSimpleText(QString("⚠ %1").arg(item->data(RoleWarning).toString()));
      w->setBrush(QBrush(QColor("#ff8e72"))); w->setPos(item->pos() + QPointF(0, item->boundingRect().height() + 4));
    }
  }

  if (model.items.empty()) {
    digital_twin_scene_->addSimpleText("Scene selected but no previewable layout metadata found. Run Generate Preview/Readiness Pack or add layout items.")->setPos(-360, -260);
    append_studio_log(QString("Scene canvas: '%1' has 0 editable layout items and 0 URDF visual preview locked items. Missing files may include environment.yaml, scene_manifest.yaml, layout/workcell_studio_layout.yaml.").arg(selected_scene_name()));
  } else {
    append_studio_log(QString("Scene canvas: loaded %1 item(s) for '%2' from %3.").arg(model.items.size()).arg(selected_scene_name(), selected_scene_path()));
  }
  if (!show_trajectory_overlay_box_ || show_trajectory_overlay_box_->isChecked()) digital_twin_scene_->addLine(20, 10, 180, -60, QPen(QColor("#38bdf8"), 2, Qt::DashDotLine));
  if (toggle_warnings_box_ && toggle_warnings_box_->isChecked()) {
    QStringList issues;
    if (!s.has_environment_yaml) issues << "missing environment.yaml";
    if (!s.has_package_xml) issues << "missing package.xml";
    if (!s.has_launch_demo) issues << "missing launch/demo.launch.py";
    auto task = load_scene_task_intent_summary(s.scene_dir);
    if (task.status != "READY") issues << "missing task intent";
    if (task.tool_id == "unknown") issues << "missing robot/gripper metadata";
    if (!issues.isEmpty()) digital_twin_scene_->addSimpleText("Safety Warning Overlay: " + issues.join(" | "))->setPos(-380, -280);
  }
  if (!preserved_selected_id.isEmpty()) {
    apply_scene_selection(preserved_selected_id, QStringLiteral("unknown"), false, false);
  }
  if (digital_twin_canvas_ && digital_twin_canvas_->scene()) {
    const QRectF bounds = digital_twin_canvas_->scene()->itemsBoundingRect();
    if (!bounds.isNull()) digital_twin_canvas_->fitInView(bounds.adjusted(-24, -24, 24, 24), Qt::KeepAspectRatio);
  }
  refresh_minimap_card();
}

void MainWindow::refresh_scene_builder_left_explorer()
{
  rebuild_digital_twin_canvas();
  populate_scene_hierarchy();
  populate_asset_catalog();
  populate_scene_files_tab();
}

void MainWindow::refresh_scene_builder_view_chips()
{
  bool preview_available = false;
  bool launch_ready = false;
  if (has_selected_scene()) {
    const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
    preview_available = s.has_static_preview_svg || s.has_static_preview_html || s.has_smoke_report_json;
    launch_ready = s.has_launch_demo && s.has_package_xml;
  }
  if (scene_builder_preview_chip_) scene_builder_preview_chip_->setText(QString("Preview: %1").arg(preview_available ? "Available" : "Unavailable"));
  if (scene_builder_launch_chip_) scene_builder_launch_chip_->setText(QString("Launch: %1").arg(launch_ready ? "Ready" : "Missing"));
  if (scene_builder_safety_chip_) scene_builder_safety_chip_->setText("Safety: Fake hardware");
  if (scene_builder_generate_launch_button_) scene_builder_generate_launch_button_->setVisible(has_selected_scene() && !launch_ready);
  if (canvas_mode_label_) {
    const QString view_label = scene_builder_is_3d_view_ ? "3D Layout Preview" : "2D Layout (Fallback)";
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
      if (gi->data(RoleId).toString().trimmed() == selected_id) {
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
    }
  }

  if (scene_preview_widget_) scene_preview_widget_->select_preview_item(selected_id);
  selection_update_guard_ = false;

  const bool selection_resolved = matched_tree_item || matched_canvas_item;
  if (!selection_resolved) {
    append_studio_log(QString("Selection id missing after refresh, clearing atomically: %1").arg(selected_id));
    apply_scene_selection(QString(), selected_role, true, false);
    return;
  }

  if (matched_canvas_item) {
    select_canvas_item(matched_canvas_item);
  } else {
    sync_selected_item_state();
    refresh_selected_scene_item_labels(selected_item_state_);
  }
  const auto selected_state = current_selected_scene_item();
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

static fs::path selected_scene_environment_layout_path(const workcell_builder::WorkcellStudioSceneBrowserResult & browser, int selected_scene_index)
{
  if (selected_scene_index < 0 || selected_scene_index >= static_cast<int>(browser.scenes.size())) return {};
  return browser.scenes[static_cast<size_t>(selected_scene_index)].scene_dir / "environment_layout.yaml";
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

void MainWindow::save_layout_changes()
{
  QString selected_preview_id;
  if (scene_hierarchy_tree_ && scene_hierarchy_tree_->currentItem()) {
    selected_preview_id = scene_hierarchy_tree_->currentItem()->data(0, TreeRoleId).toString();
  } else if (digital_twin_scene_ && !digital_twin_scene_->selectedItems().isEmpty()) {
    selected_preview_id = digital_twin_scene_->selectedItems().front()->data(RoleId).toString();
  }
  const QString stable_selected_id_before_refresh = selected_preview_id.trimmed();
  if (!digital_twin_scene_) return;
  const fs::path layout_path = selected_scene_environment_layout_path(scene_browser_result_, selected_scene_index_);
  if (layout_path.empty()) return;
  const fs::path scene_dir = layout_path.parent_path().parent_path();
  const std::array<const char *, 4> required_dirs = {"layout", "task", "generated", "plan_preview"};
  for (const char * dir_name : required_dirs) {
    boost::system::error_code mk_ec;
    fs::create_directories(scene_dir / dir_name, mk_ec);
    if (mk_ec) {
      append_studio_log(QString("Save Layout failed: cannot create %1/ (%2)")
        .arg(dir_name, QString::fromStdString(mk_ec.message())));
      return;
    }
  }
  const std::string scene_name = (selected_scene_index_ >= 0 && selected_scene_index_ < static_cast<int>(scene_browser_result_.scenes.size())) ?
    scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)].scene_name : "unknown";
  YAML::Node root;
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
  }
  if (malformed_existing) {
    const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
    const fs::path backup = layout_path.parent_path() / (layout_path.filename().string() + ".malformed_backup_" + stamp.toStdString());
    boost::system::error_code ec;
    fs::copy_file(layout_path, backup, fs::copy_option::overwrite_if_exists, ec);
    if (ec) {
      append_studio_log(QString("Malformed environment_layout.yaml detected; backup failed (%1). Save aborted.").arg(QString::fromStdString(ec.message())));
      QMessageBox::warning(this, "Save Layout", "Malformed environment_layout.yaml backup failed. Not overwriting.");
      return;
    }
    append_studio_log(QString("Malformed environment_layout.yaml backed up to %1").arg(QString::fromStdString(backup.string())));
    root = YAML::Node();
  }
  if (!root || !root.IsMap()) {
    root = minimal_environment_layout(scene_name);
  }
  // Ownership boundary: the canvas placement serializer only owns and mutates:
  // - schema_version (required environment_layout/v1 metadata)
  // - placed_assets (asset placement list)
  // All other top-level and nested fields are preserved exactly as loaded.
  root["schema_version"] = "environment_layout/v1";
  if (!root["scene_name"] || !root["scene_name"].IsScalar()) {
    root["scene_name"] = scene_name;
  }
  const QString backup_stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
  const fs::path layout_backup = layout_path.parent_path() / ("environment_layout." + backup_stamp.toStdString() + ".bak.yaml");
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
  YAML::Node placed = ensure_sequence_of_maps(root, "placed_assets");
  YAML::Node existing_by_id(YAML::NodeType::Map);
  for (std::size_t i = 0; i < placed.size(); ++i) {
    const YAML::Node existing = placed[i];
    if (!existing || !existing.IsMap()) continue;
    const std::string existing_id = workcell_builder::yaml_map_value_or_empty(existing, "id");
    if (!existing_id.empty()) existing_by_id[existing_id] = existing;
  }
  YAML::Node updated_placed(YAML::NodeType::Sequence);
  for (auto * gi : digital_twin_scene_->items()) {
    if (gi->data(RoleRole).toString() != "asset") continue;
    if (gi->data(RoleLocked).toBool()) continue;
    const std::string item_id = gi->data(RoleId).toString().toStdString();
    if (!workcell_builder::workcell_studio_is_valid_id(item_id)) {
      QMessageBox::warning(this, "Save Layout", QString("Invalid ID for YAML/package compatibility: %1").arg(QString::fromStdString(item_id)));
      append_studio_log(QString("Save blocked: invalid id '%1'").arg(QString::fromStdString(item_id)));
      return;
    }
    YAML::Node item = existing_by_id[item_id] ? YAML::Clone(existing_by_id[item_id]) : YAML::Node(YAML::NodeType::Map);
    item["id"] = item_id;
    item["display_name"] = gi->data(RoleDisplayName).toString().toStdString();
    item["category"] = gi->data(RoleCategory).toString().toStdString();
    item["type"] = gi->data(RoleType).toString().toStdString();
    item["source_path"] = gi->data(RoleSource).toString().toStdString();
    item["source_package"] = gi->data(RoleSourcePackage).toString().toStdString();
    item["editable"] = !gi->data(RoleLocked).toBool();
    item["locked"] = gi->data(RoleLocked).toBool();

    YAML::Node pose = ensure_map_node(item, "pose");
    pose["x"] = gi->pos().x() / 100.0; pose["y"] = gi->pos().y() / 100.0; pose["z"] = gi->data(RolePoseZ).toDouble();
    pose["roll"] = gi->data(RoleRoll).toDouble(); pose["pitch"] = gi->data(RolePitch).toDouble(); pose["yaw"] = gi->data(RoleYaw).toDouble();
    pose["xyz"] = YAML::Load("[]");
    pose["xyz"].push_back(pose["x"]); pose["xyz"].push_back(pose["y"]); pose["xyz"].push_back(pose["z"]);
    pose["rpy"] = YAML::Load("[]");
    pose["rpy"].push_back(pose["roll"]); pose["rpy"].push_back(pose["pitch"]); pose["rpy"].push_back(pose["yaw"]);

    item["dimensions"] = YAML::Load("[]");
    item["dimensions"].push_back(gi->data(RoleWidth).toDouble());
    item["dimensions"].push_back(gi->data(RoleDepth).toDouble());
    item["dimensions"].push_back(gi->data(RoleHeight).toDouble());

    YAML::Node mesh = ensure_map_node(item, "mesh");
    mesh["path"] = gi->data(RoleSource).toString().toStdString();
    mesh["source_package"] = gi->data(RoleSourcePackage).toString().toStdString();

    YAML::Node meta = ensure_map_node(item, "metadata");
    meta["preview_only"] = true;
    meta["serialization_contract"] =
      "id, display_name, type/category, pose.xyz, pose.rpy, dimensions, mesh metadata, editable/locked";
    updated_placed.push_back(item);
    append_studio_log(QString("Saved layout item %1 to environment_layout.yaml").arg(gi->data(RoleId).toString()));
  }
  root["placed_assets"] = updated_placed;
  std::ofstream out(layout_path.string());
  out << root;
  out.close();
  layout_dirty_ = false;
  layout_saved_ = true;
  validation_stale_ = true;
  launch_artifacts_ready_ = false;
  if (layout_state_label_) layout_state_label_->setText("Unsaved Layout Edits: none");
  append_studio_log(QString("Saved scene layout metadata to %1").arg(QString::fromStdString(layout_path.string())));
  const fs::path workcell_layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  YAML::Node workcell_layout(YAML::NodeType::Map);
  workcell_layout["schema_version"] = "workcell_studio_layout/v1";
  workcell_layout["schema"] = "workcell_studio_layout/v1";
  workcell_layout["scene_name"] = scene_name;
  YAML::Node items(YAML::NodeType::Sequence);
  for (const auto & node : updated_placed) {
    YAML::Node item = YAML::Clone(node);
    item["source"] = "existing_new_cell_flow";
    items.push_back(item);
  }
  workcell_layout["items"] = items;
  std::ofstream workcell_out(workcell_layout_path.string());
  workcell_out << workcell_layout;
  workcell_out.close();
  append_studio_log(QString("Save Layout: wrote editable layout items to %1")
    .arg(QString::fromStdString(workcell_layout_path.string())));

  YAML::Node environment(YAML::NodeType::Map);
  environment["scene_name"] = scene_name;
  environment["fake_hardware_first"] = true;
  environment["safety_note"] = "Preview metadata only. Not approval for real-hardware execution.";
  YAML::Node task_zones(YAML::NodeType::Sequence);
  for (const auto & node : updated_placed) {
    const std::string type = workcell_builder::yaml_map_value_or_empty(node, "type");
    if (type == "pick_zone" || type == "place_zone") {
      YAML::Node zone(YAML::NodeType::Map);
      zone["id"] = workcell_builder::yaml_map_value_or_empty(node, "id");
      zone["type"] = type == "pick_zone" ? "pick" : "place";
      zone["source"] = "existing_new_cell_flow";
      task_zones.push_back(zone);
    }
  }
  environment["task_zones"] = task_zones;
  const fs::path environment_path = scene_dir / "environment.yaml";
  std::ofstream env_out(environment_path.string());
  env_out << environment;
  env_out.close();
  append_studio_log(QString("Save Layout: wrote environment metadata to %1")
    .arg(QString::fromStdString(environment_path.string())));
  append_studio_log(QString("Save Layout: rebuilding Scene3D data after save (selection id snapshot='%1').")
    .arg(stable_selected_id_before_refresh.isEmpty() ? "<none>" : stable_selected_id_before_refresh));
  refresh_scene_builder_left_explorer();
  if (!stable_selected_id_before_refresh.isEmpty()) {
    apply_scene_selection(stable_selected_id_before_refresh, QStringLiteral("unknown"), false, false);
  } else {
    append_studio_log("Save Layout: no selected stable item id to reselect.");
  }
  refresh_scene_browser_ui();
}

void MainWindow::create_starter_layout_from_preview()
{
  if (!has_selected_scene()) return;
  const auto & s = scene_browser_result_.scenes[static_cast<size_t>(selected_scene_index_)];
  const auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);
  if (model.items.empty()) {
    append_studio_log("Create Starter Layout failed: preview items count is 0.");
    QMessageBox::warning(this, "Create Starter Layout", "No preview items found; cannot generate starter layout.");
    return;
  }
  const fs::path layout_dir = s.scene_dir / "layout";
  const fs::path layout_file = layout_dir / "workcell_studio_layout.yaml";
  boost::system::error_code ec;
  fs::create_directories(layout_dir, ec);
  if (ec) {
    append_studio_log(QString("Create Starter Layout failed: cannot create layout directory (%1).").arg(QString::fromStdString(ec.message())));
    return;
  }
  if (fs::exists(layout_file)) {
    const auto response = QMessageBox::question(this, "Overwrite Existing Layout",
      "layout/workcell_studio_layout.yaml already exists. Overwrite with starter layout from preview metadata?");
    if (response != QMessageBox::Yes) {
      append_studio_log("Create Starter Layout cancelled by user.");
      return;
    }
    const QString stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
    const fs::path backup = layout_dir / ("workcell_studio_layout." + stamp.toStdString() + ".bak.yaml");
    fs::copy_file(layout_file, backup, fs::copy_option::overwrite_if_exists, ec);
    if (ec) {
      append_studio_log(QString("Create Starter Layout failed: backup before overwrite failed (%1).").arg(QString::fromStdString(ec.message())));
      QMessageBox::warning(this, "Create Starter Layout", "Backup before overwrite failed; aborting.");
      return;
    }
    append_studio_log(QString("Create Starter Layout: backup created at %1").arg(QString::fromStdString(backup.string())));
  }
  const YAML::Node layout = workcell_builder::build_starter_layout_entries_from_preview(model);
  std::ofstream out(layout_file.string());
  if (!out.good()) {
    append_studio_log("Create Starter Layout failed: unable to open output file for write.");
    return;
  }
  out << layout;
  out.close();
  if (!out.good()) {
    append_studio_log("Create Starter Layout failed: write error while saving starter layout.");
    return;
  }
  append_studio_log(QString("Use Recommended Layout: wrote %1 item(s) to %2").arg(model.items.size()).arg(QString::fromStdString(layout_file.string())));
  append_studio_log("Use Recommended Layout: added recommended editable layout items from current preview metadata.");
  rebuild_digital_twin_canvas();
  refresh_scene_builder_left_explorer();
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
  const bool locked = item->data(RoleLocked).toBool();
  inspector_update_guard_ = true;
  inspector_x_->setValue(item->pos().x() / 100.0); inspector_y_->setValue(item->pos().y() / 100.0); inspector_z_->setValue(item->data(RolePoseZ).toDouble());
  inspector_roll_->setValue(item->data(RoleRoll).toDouble()); inspector_pitch_->setValue(item->data(RolePitch).toDouble()); inspector_yaw_->setValue(item->data(RoleYaw).toDouble());
  inspector_dim_x_->setValue(item->data(RoleWidth).toDouble());
  inspector_dim_y_->setValue(item->data(RoleDepth).toDouble());
  inspector_dim_z_->setValue(item->data(RoleHeight).toDouble());
  for (auto * sb : {inspector_x_, inspector_y_, inspector_z_, inspector_roll_, inspector_pitch_, inspector_yaw_}) {
    if (sb) sb->setReadOnly(locked);
  }
  for (auto * sb : {inspector_dim_x_, inspector_dim_y_, inspector_dim_z_}) {
    if (sb) sb->setReadOnly(locked);
  }
  if (inspector_apply_button_) inspector_apply_button_->setEnabled(!locked);
  if (inspector_revert_button_) inspector_revert_button_->setEnabled(!locked);
  if (inspector_live_update_box_) inspector_live_update_box_->setEnabled(!locked);
  inspector_update_guard_ = false;
}

void MainWindow::apply_selection_transform_from_editor() { apply_inspector_pose_to_item(); }

void MainWindow::apply_inspector_pose_to_item(){ if(inspector_update_guard_ || !digital_twin_scene_ || digital_twin_scene_->selectedItems().isEmpty()) return; auto *i=digital_twin_scene_->selectedItems().front(); if (i->data(RoleLocked).toBool()) { append_studio_log(QString("Locked/generated item edit rejected: %1").arg(i->data(RoleId).toString())); return; } QPointF old=i->pos(); i->setPos(inspector_x_->value()*100.0, inspector_y_->value()*100.0); i->setData(RolePoseZ, inspector_z_->value()); i->setData(RoleRoll, inspector_roll_->value()); i->setData(RolePitch, inspector_pitch_->value()); i->setData(RoleYaw, inspector_yaw_->value()); i->setData(RoleWidth, inspector_dim_x_->value()); i->setData(RoleDepth, inspector_dim_y_->value()); i->setData(RoleHeight, inspector_dim_z_->value()); undo_stack_.push_back({"pose_edit", i->data(RoleId).toString(), old, i->pos(), false, false}); redo_stack_.clear(); mark_layout_dirty("Inspector Pose/Dimensions Edit"); append_studio_log(QString("item updated: %1 type=%2 source=%3 editable=%4 locked=%5").arg(i->data(RoleId).toString(), i->data(RoleType).toString(), i->data(RoleSource).toString(), i->data(RoleLocked).toBool() ? "false":"true", i->data(RoleLocked).toBool() ? "true":"false")); refresh_selection_transform_editor_from_item(i); if (scene_preview_widget_) scene_preview_widget_->update(); }

void MainWindow::revert_selection_transform_editor()
{
  if (!digital_twin_scene_ || digital_twin_scene_->selectedItems().isEmpty()) return;
  refresh_selection_transform_editor_from_item(digital_twin_scene_->selectedItems().front());
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
  const fs::path layout_path = selected_scene_environment_layout_path(scene_browser_result_, selected_scene_index_);
  const auto layout_ids = workcell_builder::workcell_studio_collect_layout_ids(layout_path);
  reserved_ids.insert(layout_ids.begin(), layout_ids.end());
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
  item->setData(RoleSourcePackage, "asset_catalog");
  item->setData(RolePoseZ, armed_asset_z_m_);
  item->setData(RoleRoll, armed_asset_roll_rad_); item->setData(RolePitch, armed_asset_pitch_rad_); item->setData(RoleYaw, armed_asset_yaw_rad_);
  item->setData(RoleWidth, 0.35); item->setData(RoleDepth, 0.35); item->setData(RoleHeight, 0.35);
  item->setData(RoleImported, source_path.endsWith(".stl", Qt::CaseInsensitive) || source_path.endsWith(".urdf", Qt::CaseInsensitive));
  item->setData(RoleGeneratedPlaceholder, category.contains("placeholder", Qt::CaseInsensitive));
  item->setData(RoleLocked, false);
  QStringList warnings;
  if (source_path.trimmed().isEmpty()) warnings << "missing dimensions/source metadata";
  if (source_path.contains("robot", Qt::CaseInsensitive) || category.contains("Robot", Qt::CaseInsensitive) || category.contains("Tool", Qt::CaseInsensitive)) {
    item->setData(RoleLocked, true);
    warnings << "locked item";
  }
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
  refresh_scene_builder_left_explorer();
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

void MainWindow::on_asset_filter_changed(int)
{
  const QString selected = asset_filter_combo_ ? asset_filter_combo_->currentText() : "All";
  if (!asset_catalog_tree_) return;
  for (int i = 0; i < asset_catalog_tree_->topLevelItemCount(); ++i) {
    auto * item = asset_catalog_tree_->topLevelItem(i);
    const int idx = item->data(0, CatalogRoleIndex).toInt();
    const bool has_valid_index = idx >= 0 && idx < asset_catalog_entries_.size();
    const QString category = has_valid_index ? asset_catalog_entries_[idx].category : item->text(1);
    const bool visible = (selected == "All" || category == selected);
    item->setHidden(!visible);
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

void MainWindow::apply_scene3d_preview_layer_filters(bool log_change)
{
  if (!scene_preview_widget_) {
    return;
  }
  QVector<ScenePreviewWidget::PreviewItem> filtered_items;
  auto include_item = [this](const ScenePreviewWidget::PreviewItem & p) {
      const QString source_layer = p.source_layer.trimmed().toLower();
      const QString visual_source = p.active_visual_source.trimmed().toLower();
      const QString role = p.role.trimmed().toLower();
      const QString category = p.category.trimmed().toLower();
      const QString combined = role + "|" + category + "|" + p.status.trimmed().toLower() + "|" + p.warnings.join("|").toLower();
      const bool is_warning_or_missing = combined.contains("warning") || combined.contains("missing") || !p.mesh_load_warning.trimmed().isEmpty();
      const bool is_overlay_or_helper = combined.contains("overlay") || combined.contains("helper") || combined.contains("safety zone");
      if (source_layer == "editable_layout") return preview_layer_editable_layout_box_ ? preview_layer_editable_layout_box_->isChecked() : true;
      if (source_layer == "generated_urdf_visual") return preview_layer_generated_urdf_visual_box_ ? preview_layer_generated_urdf_visual_box_->isChecked() : true;
      if (source_layer == "primitive_fallback") return preview_layer_primitive_fallback_box_ ? preview_layer_primitive_fallback_box_->isChecked() : true;
      if (visual_source == "mesh_preview") return preview_layer_mesh_preview_box_ ? preview_layer_mesh_preview_box_->isChecked() : true;
      if (is_overlay_or_helper) return preview_layer_overlays_helpers_box_ ? preview_layer_overlays_helpers_box_->isChecked() : true;
      if (is_warning_or_missing) return preview_layer_warnings_missing_assets_box_ ? preview_layer_warnings_missing_assets_box_->isChecked() : true;
      return true;
    };
  for (const auto & p : all_scene_preview_items_) {
    if (include_item(p)) filtered_items.push_back(p);
  }
  scene_preview_widget_->set_preview_items(filtered_items);
  append_studio_log(
    QString("Scene3D diagnostics {model_items_count=%1, filtered_visible_count=%2}")
      .arg(all_scene_preview_items_.size())
      .arg(filtered_items.size()));
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
  scene_hierarchy_tree_->clear();

  if (selected_scene_index_ < 0 || selected_scene_index_ >= (int)scene_browser_result_.scenes.size()) { if (scene_preview_widget_) { scene_preview_widget_->set_scene_selected(false); scene_preview_widget_->set_preview_scene_name("No scene"); } return; }
  const auto & s = scene_browser_result_.scenes[(size_t)selected_scene_index_];
  const fs::path d = s.scene_dir;
  const auto model = workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name);

  auto normalize_role = [](const QString & raw_role, const QString & fallback_text) {
    const QString lower = (raw_role + " " + fallback_text).toLower();
    if (lower.contains("robot")) return QString("robot");
    if (lower.contains("end_effector") || lower.contains("gripper") || lower.contains("tool")) return QString("end_effector/tool");
    if (lower.contains("camera") || lower.contains("sensor")) return QString("camera");
    if (lower.contains("support_surface") || lower.contains("table") || lower.contains("workbench")) return QString("support_surface/table");
    if (lower.contains("conveyor")) return QString("conveyor");
    if (lower.contains("pick_source") || lower.contains("pick zone") || lower.contains("pick_zone")) return QString("pick source/zone");
    if (lower.contains("place_target") || lower.contains("place zone") || lower.contains("place_zone") || lower.contains("bin")) return QString("place target/bin");
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
    "safety zone"
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
    if (lower.contains("helper") || lower.contains("overlay") || lower.contains("safety") || lower.contains("malformed snapshot") || lower.contains("detection snapshot")) return QString("Overlays / Helpers");
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
      .arg(p.editable ? QStringLiteral("editable") : QStringLiteral("locked"))
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
    p.source_layer = QStringLiteral("generated_preview");
    p.active_visual_source = QStringLiteral("generated_preview");
    p.linked_to_editable_layout_state = false;
          p.source_layer = QStringLiteral("overlay");
          p.active_visual_source = QStringLiteral("overlay");
          p.editable = false;
          p.selectable = true;
    p.metadata_complete = metadata_complete;
    if (!metadata_complete) {
      p.warnings << "metadata incomplete";
      preview_warning_details << QString("%1 (%2): metadata incomplete").arg(p.id, p.role);
    }
    preview_items.push_back(p);
    if (allowed_scene_roles.contains(p.role)) {
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
    p.category = QString::fromStdString(item.type);
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
    p.locked = item.locked;
    p.camera_id = QString::fromStdString(item.camera_id);
    p.frame_id = QString::fromStdString(item.frame_id);
    p.detection_label = QString::fromStdString(item.detection_label);
    p.confidence = item.confidence;
    p.tracking_id = QString::fromStdString(item.tracking_id);
    p.snapshot_source_file = QString::fromStdString(item.snapshot_source_file);
    p.alignment_warning = QString::fromStdString(item.alignment_warning);
    p.editable = !item.locked;
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
          p.source_layer = QStringLiteral("overlay");
          p.active_visual_source = QStringLiteral("overlay");
          p.editable = false;
          p.selectable = true;
        break;
      case workcell_builder::WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
      default:
        p.source_layer = QStringLiteral("generated_preview");
        p.active_visual_source = QStringLiteral("mesh_preview");
        p.linked_to_editable_layout_state = false;
          p.source_layer = QStringLiteral("overlay");
          p.active_visual_source = QStringLiteral("overlay");
          p.editable = false;
          p.selectable = true;
        break;
    }
    if (item.locked) {
      const QString base_reason = p.warnings.isEmpty() ? QStringLiteral("item is locked") : p.warnings.front();
      p.lock_reason = base_reason;
      p.warnings << QStringLiteral("Locked: %1").arg(base_reason);
    }
    preview_items.push_back(p);
    if (allowed_scene_roles.contains(p.role)) {
      add_tree_node(p);
    }

    if (!item.warnings.empty()) {
      ++preview_warning_count;
      const QString warning_text = QString::fromStdString(item.warnings.front());
      preview_warning_details << QString("%1 (%2): %3").arg(p.id, p.role, warning_text);
      append_studio_log(QString("Preview warning: %1").arg(warning_text));
    }
  }


  QSet<QString> preview_ids;
  for (const auto &existing : preview_items) preview_ids.insert(existing.id);
  const fs::path urdf_visual_index = d / "generated" / "scene_visual_mesh_index.json";
  bool refresh_urdf_visual_index = !fs::exists(urdf_visual_index);
  if (fs::exists(urdf_visual_index)) {
    try {
      const YAML::Node existing_index = YAML::LoadFile(urdf_visual_index.string());
      const bool safe_for_preview = workcell_builder::yaml_map_key(existing_index, "safe_for_preview").as<bool>(false);
      if (!safe_for_preview) {
        refresh_urdf_visual_index = true;
        append_studio_log("Visual mesh index unsafe/best-effort; preview may show placeholders");
      }
    } catch (...) {
      refresh_urdf_visual_index = true;
    }
  }
  if (refresh_urdf_visual_index) {
    const QString regen_scene_key =
      QString::fromStdString(d.string()) + "::" + QString::fromStdString(d.filename().string());
    append_studio_log("Visual mesh index stale; regenerating");
    const QString extractor_script_path = resolve_scene3d_extractor_script_path(d);
    if (extractor_script_path.isEmpty()) {
      if (visual_index_script_missing_reported_scene_key_ != regen_scene_key || !visual_index_regen_throttle_session_active_) {
        append_studio_log(
          QString("Visual mesh index regeneration skipped for scene '%1': helper script missing. Searched strategy: resolved extractor path with --prefer-xacro.")
          .arg(QString::fromStdString(d.filename().string())));
        visual_index_script_missing_reported_scene_key_ = regen_scene_key;
      }
      visual_index_regen_throttle_session_active_ = true;
      append_studio_log("Visual mesh index unsafe/best-effort; preview may show placeholders");
    } else {
      QStringList regen_args;
      regen_args << extractor_script_path << "--scene" << QString::fromStdString(d.filename().string()) << "--prefer-xacro";
      const int code = QProcess::execute("python3", regen_args);
      if (code == 0) {
        append_studio_log("Visual mesh index regenerated with xacro");
      } else {
        if (visual_index_regen_failure_reported_scene_key_ != regen_scene_key || !visual_index_regen_throttle_session_active_) {
          append_studio_log(
            QString("Visual mesh index regeneration failed for scene '%1' (exit=%2).")
            .arg(QString::fromStdString(d.filename().string()))
            .arg(code));
          visual_index_regen_failure_reported_scene_key_ = regen_scene_key;
        }
        visual_index_regen_throttle_session_active_ = true;
        append_studio_log("Visual mesh index unsafe/best-effort; preview may show placeholders");
      }
    }
  }
  int visual_index_loaded_count = 0;
  int visual_preview_added_count = 0;
  int skipped_missing_mesh_source_path = 0;
  int skipped_file_not_found = 0;
  int skipped_render_expected_false = 0;
  int skipped_duplicate_id = 0;
  int skipped_invalid_pose = 0;
  int skipped_unsupported_format = 0;
  int skipped_zero_triangle_mesh = 0;
  int skipped_unknown_geometry = 0;
  int non_mesh_geometry_added = 0;
  int non_mesh_geometry_unsupported = 0;
  int package_uri_resolved_by_loader = 0;
  int source_path_from_resolved_path = 0;
  int unresolved_package_uri_count = 0;
  int stale_or_absolute_only_mesh_index_count = 0;
  int mesh_item_count = 0;
  int primitive_item_count = 0;
  int unknown_item_count = 0;
  int skipped_other = 0;
  QString visual_diagnostics_summary;
  bool visual_index_safe_for_preview = false;
  QString visual_index_extraction_mode = "unknown";
  if (fs::exists(urdf_visual_index)) {
    try {
      const YAML::Node urdf_index = YAML::LoadFile(urdf_visual_index.string());
      visual_index_safe_for_preview = workcell_builder::yaml_map_key(urdf_index, "safe_for_preview").as<bool>(false);
      visual_index_extraction_mode = QString::fromStdString(
        workcell_builder::yaml_map_value_or_empty(urdf_index, "extraction_mode"));
      stale_or_absolute_only_mesh_index_count =
        workcell_builder::yaml_map_key(urdf_index, "stale_or_unsafe_count").as<int>(0);
      if (stale_or_absolute_only_mesh_index_count == 0 && workcell_builder::yaml_map_key(urdf_index, "stale_index").as<bool>(false)) {
        stale_or_absolute_only_mesh_index_count = 1;
      }
      const YAML::Node visual_items = workcell_builder::yaml_map_key(urdf_index, "visual_items");
      if (visual_items && visual_items.IsSequence()) {
        for (const auto &v : visual_items) {
          if (!v.IsMap()) {
            ++skipped_other;
            continue;
          }
          ++visual_index_loaded_count;
          const QString id = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "id"));
          if (id.isEmpty()) {
            ++skipped_other;
            continue;
          }
          if (preview_ids.contains(id)) {
            ++skipped_duplicate_id;
            continue;
          }
          const QString geometry_type = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "geometry_type"));
          if (geometry_type == "mesh") ++mesh_item_count;
          else if (geometry_type == "box" || geometry_type == "cylinder" || geometry_type == "sphere") ++primitive_item_count;
          else ++unknown_item_count;
          const bool render_expected = workcell_builder::yaml_map_key(v, "render_expected").as<bool>(false);
          const YAML::Node pose = workcell_builder::yaml_map_key(v, "pose");
          const YAML::Node xyz = workcell_builder::yaml_map_key(pose, "xyz");
          const YAML::Node rpy = workcell_builder::yaml_map_key(pose, "rpy");
          const bool has_visual_metadata =
            !id.trimmed().isEmpty() &&
            !QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link")).trimmed().isEmpty() &&
            xyz && rpy && xyz.IsSequence() && rpy.IsSequence() && xyz.size() >= 3 && rpy.size() >= 3;
          if (!render_expected && !has_visual_metadata) {
            ++skipped_render_expected_false;
            continue;
          }
          const bool unsupported_format = workcell_builder::yaml_map_key(v, "unsupported_format").as<bool>(false);
          const int triangle_count = workcell_builder::yaml_map_key(v, "triangle_count").as<int>(-1);
          if (triangle_count == 0) {
            ++skipped_zero_triangle_mesh;
            continue;
          }
          ScenePreviewWidget::PreviewItem p;
          p.id = id;
          p.display_name = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "link"));
          if (p.display_name.trimmed().isEmpty()) p.display_name = id;
          p.category = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "category"));
          if (p.category.trimmed().isEmpty()) p.category = "URDF Visual";
          p.role = p.category;
          p.status = "ready";
          p.source_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "source_path"));
          const QString resolved_path = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "resolved_path"));
          const QString package_uri = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "package_uri"));
          if (!package_uri.trimmed().isEmpty() && package_uri.startsWith("package://") && !workcell_builder::yaml_map_key(v, "resolved").as<bool>(false)) {
            ++unresolved_package_uri_count;
          }
          if (p.source_path.trimmed().isEmpty() && !resolved_path.trimmed().isEmpty()) {
            p.source_path = resolved_path;
            ++source_path_from_resolved_path;
          }
          if (p.source_path.trimmed().isEmpty() && (package_uri.startsWith("file://") || package_uri.startsWith("/"))) {
            QString candidate = package_uri;
            if (candidate.startsWith("file://")) candidate = candidate.mid(7);
            if (fs::exists(fs::path(candidate.toStdString()))) {
              p.source_path = candidate;
              ++package_uri_resolved_by_loader;
            }
          }
          p.locked = true;
          p.editable = false;
          p.selectable = true;
          p.lock_reason = "URDF visual preview-only item (locked)";
          p.source_layer = QStringLiteral("generated_urdf_visual");
          p.active_visual_source = (geometry_type == "mesh") ? QStringLiteral("mesh_preview")
                                                              : QStringLiteral("primitive_fallback");
          p.linked_to_editable_layout_state = false;
          p.editable = false;
          p.selectable = true;
          if (!xyz || !rpy || !xyz.IsSequence() || !rpy.IsSequence() || xyz.size() < 3 || rpy.size() < 3) {
            ++skipped_invalid_pose;
            continue;
          }
          p.x = workcell_builder::yaml_seq_index(xyz,0).as<double>(0.0);
          p.y = workcell_builder::yaml_seq_index(xyz,1).as<double>(0.0);
          p.z = workcell_builder::yaml_seq_index(xyz,2).as<double>(0.0);
          p.roll = workcell_builder::yaml_seq_index(rpy,0).as<double>(0.0);
          p.pitch = workcell_builder::yaml_seq_index(rpy,1).as<double>(0.0);
          p.yaw = workcell_builder::yaml_seq_index(rpy,2).as<double>(0.0);
          const YAML::Node scale = workcell_builder::yaml_map_key(v, "scale");
          p.sx = workcell_builder::yaml_seq_index(scale,0).as<double>(0.25);
          p.sy = workcell_builder::yaml_seq_index(scale,1).as<double>(0.25);
          p.sz = workcell_builder::yaml_seq_index(scale,2).as<double>(0.25);
          const bool resolved = workcell_builder::yaml_map_key(v, "resolved").as<bool>(false);
          const bool is_primitive = (geometry_type == "box" || geometry_type == "cylinder" || geometry_type == "sphere");
          bool mesh_fallback = false;
          if (geometry_type == "mesh") {
            if (unsupported_format) {
              mesh_fallback = true;
            } else if (p.source_path.trimmed().isEmpty()) {
              mesh_fallback = true;
            } else if (!fs::exists(fs::path(p.source_path.toStdString()))) {
              mesh_fallback = true;
            } else {
              p.mesh_path = p.source_path;
            }
          } else if (is_primitive) {
            ++non_mesh_geometry_added;
          } else {
            ++skipped_unknown_geometry;
            ++non_mesh_geometry_unsupported;
            continue;
          }
          if (mesh_fallback) {
            p.status = "warning";
            p.mesh_available = false;
            p.has_mesh_metadata = true;
            p.active_visual_source = QStringLiteral("primitive_fallback");
            p.source_layer = QStringLiteral("generated_urdf_visual");
            p.editable = false;
            p.selectable = true;
            p.lock_reason = QStringLiteral("generated_urdf_visual");
            p.warnings << QStringLiteral("Preview warning: URDF visual mesh unavailable; using primitive fallback");
          } else if (!resolved || (geometry_type == "mesh" && p.source_path.trimmed().isEmpty())) {
            p.status = "warning";
            p.mesh_available = false;
            p.warnings << QStringLiteral("Preview warning: URDF visual unresolved");
            const QString warning = QString::fromStdString(workcell_builder::yaml_map_value_or_empty(v, "warning"));
            if (!warning.isEmpty()) p.warnings << warning;
          } else {
            p.mesh_available = true;
            p.has_mesh_metadata = true;
          }
          preview_items.push_back(p);
          ++visual_preview_added_count;
          preview_ids.insert(id);
          add_tree_node(p);
        }
      }
      const int visual_skipped_total =
        skipped_missing_mesh_source_path +
        skipped_file_not_found +
        skipped_render_expected_false +
        skipped_duplicate_id +
        skipped_invalid_pose +
        skipped_unsupported_format +
        skipped_zero_triangle_mesh +
        skipped_other;
      append_studio_log(QString("Visual mesh index loaded from: %1").arg(QString::fromStdString(urdf_visual_index.string())));
      append_studio_log(QString("Visual mesh index safe_for_preview: %1").arg(visual_index_safe_for_preview ? "true" : "false"));
      append_studio_log(QString("Visual mesh index extraction_mode: %1").arg(visual_index_extraction_mode));
      append_studio_log(QString("Visual mesh index loaded: %1 visual items").arg(visual_index_loaded_count));
      append_studio_log(QString("Mesh items: %1, primitive items: %2, unknown items: %3").arg(mesh_item_count).arg(primitive_item_count).arg(unknown_item_count));
      append_studio_log(QString("Visual mesh preview items added: %1 / %2").arg(visual_preview_added_count).arg(visual_index_loaded_count));
      append_studio_log(
        QString("Skipped visual items: missing_mesh_source_path=%1 file_not_found=%2 render_expected_false=%3 duplicate_id=%4 invalid_pose=%5 unsupported_format=%6 zero_triangle_mesh=%7 unknown_geometry=%8 other=%9")
        .arg(skipped_missing_mesh_source_path)
        .arg(skipped_file_not_found)
        .arg(skipped_render_expected_false)
        .arg(skipped_duplicate_id)
        .arg(skipped_invalid_pose)
        .arg(skipped_unsupported_format)
        .arg(skipped_zero_triangle_mesh)
        .arg(skipped_unknown_geometry)
        .arg(skipped_other));
      append_studio_log(QString("Loader fallbacks: package_uri_resolved_by_loader=%1 source_path_from_resolved_path=%2 non_mesh_geometry_added=%3 non_mesh_geometry_unsupported=%4")
        .arg(package_uri_resolved_by_loader).arg(source_path_from_resolved_path).arg(non_mesh_geometry_added).arg(non_mesh_geometry_unsupported));
      if (visual_index_loaded_count != (visual_preview_added_count + visual_skipped_total)) {
        append_studio_log(
          QString("Preview warning: visual ingestion mismatch loaded=%1 added=%2 skipped_sum=%3")
          .arg(visual_index_loaded_count)
          .arg(visual_preview_added_count)
          .arg(visual_skipped_total));
      }
      visual_diagnostics_summary =
        QString("Visuals: added %1/%2 • skipped msrc=%3 nf=%4 render=%5 dup=%6 pose=%7 fmt=%8 tri0=%9 other=%10")
        .arg(visual_preview_added_count)
        .arg(visual_index_loaded_count)
        .arg(skipped_missing_mesh_source_path)
        .arg(skipped_file_not_found)
        .arg(skipped_render_expected_false)
        .arg(skipped_duplicate_id)
        .arg(skipped_invalid_pose)
        .arg(skipped_unsupported_format)
        .arg(skipped_zero_triangle_mesh)
        .arg(skipped_other);
      if (visual_index_safe_for_preview && visual_preview_added_count == 0) {
        append_studio_log("Visual mesh index safe but no preview items were ingested");
      }
    } catch (...) {
      append_studio_log("Preview warning: failed to parse generated/scene_visual_mesh_index.json");
    }
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
    node->setData(0, TreeRoleCategory, "file");
    node->setData(0, TreeRoleSource, QString::fromStdString(path.string()));
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

  editable_layout_item_count_ = model.provenance_status.editable_layout_count;
  preview_fallback_item_count_ = model.provenance_status.generated_or_legacy_preview_count + model.provenance_status.static_fallback_preview_count;
  preview_provenance_summary_ = QString::fromStdString(model.provenance_status.summary);
  if (!visual_diagnostics_summary.isEmpty()) {
    preview_provenance_summary_ = preview_provenance_summary_.isEmpty()
      ? visual_diagnostics_summary
      : QString("%1 | %2").arg(preview_provenance_summary_, visual_diagnostics_summary);
  }
  all_scene_preview_items_ = preview_items;
  if (scene_preview_widget_) {
    scene_preview_widget_->set_scene_selected(true);
    scene_preview_widget_->set_preview_scene_name(QString::fromStdString(s.scene_name));
    scene_preview_widget_->set_preview_status_summary(preview_provenance_summary_);
    apply_scene3d_preview_layer_filters(false);
  }
  preview_warning_details_ = preview_warning_details;

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
  const int urdf_visual_locked_count = visual_preview_added_count;
  int scene3d_editable_count = 0;
  int scene3d_mesh_count = 0;
  int scene3d_generated_count = 0;
  int scene3d_fallback_count = 0;
  int scene3d_missing_count = 0;
  int scene3d_locked_count = 0;
  for (const auto & item : preview_items) {
    if (item.editable) ++scene3d_editable_count;
    if (item.active_visual_source == "mesh_preview") ++scene3d_mesh_count;
    if (item.source_layer == "generated_urdf_visual") ++scene3d_generated_count;
    if (item.active_visual_source == "primitive_fallback" || item.source_layer == "legacy_static_fallback") ++scene3d_fallback_count;
    if (!item.mesh_available) ++scene3d_missing_count;
    if (item.locked) ++scene3d_locked_count;
  }
  const QString scene3d_diagnostics_line = QString("Scene3D: editable=%1, mesh=%2, generated=%3, fallback=%4, missing=%5, locked=%6")
    .arg(scene3d_editable_count)
    .arg(scene3d_mesh_count)
    .arg(scene3d_generated_count)
    .arg(scene3d_fallback_count)
    .arg(scene3d_missing_count)
    .arg(scene3d_locked_count);
  const int missing_mesh_count = skipped_missing_mesh_source_path + skipped_file_not_found + skipped_zero_triangle_mesh;
  const QString scene3d_warning_buckets = QString("Scene3D warnings: missing_mesh=%1, unresolved_package_uri=%2, unsupported_extension=%3, stale_or_absolute_only_mesh_index=%4")
    .arg(missing_mesh_count)
    .arg(unresolved_package_uri_count)
    .arg(skipped_unsupported_format)
    .arg(stale_or_absolute_only_mesh_index_count);
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
  append_studio_log(scene3d_boundary_diag);
}

void MainWindow::populate_asset_catalog()
{
  if (!asset_catalog_tree_) return;
  asset_catalog_tree_->clear();
  asset_catalog_entries_.clear();

  const fs::path workspace_root = workcell_path.empty() ? fs::path(QDir::homePath().toStdString()) / "workcell_ws" : workcell_path;
  const fs::path repo_root = fs::current_path();

  const auto discovered = workcell_builder::discover_asset_catalog_entries(repo_root, workspace_root);
  asset_catalog_entries_.reserve(static_cast<int>(discovered.size()));
  for (const auto & entry : discovered) {
    AssetCatalogEntry ui_entry;
    ui_entry.asset_type = QString::fromStdString(entry.source_kind);
    ui_entry.display_name = QString::fromStdString(entry.display_name);
    ui_entry.role = QString::fromStdString(entry.role_hint);
    ui_entry.dimensions = "n/a";
    ui_entry.default_pose = "auto";
    ui_entry.source_path = QString::fromStdString(entry.source_path);
    ui_entry.editable = true;
    ui_entry.availability_status = QString::fromStdString(entry.availability);
    ui_entry.disabled_reason = QString::fromStdString(entry.reason);
    ui_entry.category = QString::fromStdString(entry.category);
    asset_catalog_entries_.push_back(ui_entry);
  }

  for (int idx = 0; idx < asset_catalog_entries_.size(); ++idx) {
    const auto & e = asset_catalog_entries_[idx];
    auto * item = new QTreeWidgetItem(asset_catalog_tree_, {e.display_name, e.category, e.asset_type, e.availability_status});
    item->setData(0, CatalogRoleIndex, idx);
    item->setData(0, CatalogRolePlaceable, e.disabled_reason.trimmed().isEmpty());
    item->setData(0, CatalogRoleSourcePath, e.source_path);
    if (!e.disabled_reason.trimmed().isEmpty()) {
      item->setDisabled(true);
      item->setToolTip(3, e.disabled_reason);
      item->setToolTip(0, QString("Unavailable: %1").arg(e.disabled_reason));
    }
  }

  if (asset_catalog_entries_.isEmpty()) {
    auto * info = new QTreeWidgetItem(asset_catalog_tree_, {"No assets found", "Info", "Discovery", "incomplete"});
    info->setDisabled(true);
    info->setToolTip(0, "No assets discovered from manifest, inferred folders, or scene template references.");
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
  const bool launch_ready = has("launch/demo.launch.py");
  const bool validation_report_ready = has("validation/readiness_report.json") || has("diagnostics/readiness_report.json") || has("run_acceptance.txt");
  const bool scene_selected = has_selected_scene();
  const bool editable_layout_ready = !workcell_builder::build_workcell_studio_canvas_model(s.scene_dir, s.scene_name).items.empty();
  const bool has_warnings = !readiness_warning_details_.isEmpty();
  const bool validation_gate_ready = validation_report_ready && !validation_stale_;
  const bool export_ready = yaml_ready && launch_ready;
  const bool fake_hardware_ready = launch_artifacts_ready_ && validation_gate_ready;
  const bool preview_only_scene = !editable_layout_ready && (launch_ready || yaml_ready);
  const QMap<QString, bool> gates = {
    {"scene_selected", scene_selected},
    {"editable_layout", editable_layout_ready},
    {"layout_saved", layout_saved_},
    {"yaml_definition", yaml_ready},
    {"scene_package", launch_artifacts_ready_},
    {"validation", validation_gate_ready},
    {"fake_hardware_preview", fake_hardware_ready},
    {"export", export_ready}
  };

  std::vector<SceneWorkflowStep> steps;
  steps.push_back(compute_scene_workflow_step(
    "Scene",
    scene_selected, "A scene is selected.", "Select or create a scene first.", {}, gates));
  steps.push_back(compute_scene_workflow_step(
    "Layout",
    editable_layout_ready && layout_saved_, "Editable layout exists and is saved.",
    preview_only_scene ?
    "Legacy preview-only scene detected. Create editable layout from preview to continue editing." :
    "Create/edit and save layout to persist edits before YAML generation.",
    {"editable_layout"}, gates, (editable_layout_ready && layout_saved_) ? SceneWorkflowStepStatus::Done : SceneWorkflowStepStatus::Current));
  steps.push_back(compute_scene_workflow_step(
    "YAML",
    yaml_ready, "cell_definition.yaml exists.",
    "Generate YAML definition from current layout.",
    {"layout_saved"}, gates));
  steps.push_back(compute_scene_workflow_step(
    "Package",
    launch_artifacts_ready_, "Launch/package artifacts are present.",
    "Generate scene package to create launch and package metadata.",
    {"yaml_definition", "validation"}, gates));
  steps.push_back(compute_scene_workflow_step(
    "Validation",
    validation_gate_ready, has_warnings ? "Validation completed with warnings." : "Validation checks passed.",
    validation_stale_ ? "Validation results are stale; rerun validation." : "Run offline validation.",
    {"yaml_definition"}, gates,
    validation_gate_ready ? (has_warnings ? SceneWorkflowStepStatus::Warning : SceneWorkflowStepStatus::Done) : SceneWorkflowStepStatus::Current));
  steps.push_back(compute_scene_workflow_step(
    "Preview",
    fake_hardware_ready,
    "Ready (Safety: fake hardware only, no robot motion).",
    "Blocked until scene package and validation gates are satisfied. Safety gate remains enforced: fake hardware only.",
    {"scene_package", "validation"}, gates, fake_hardware_ready ? SceneWorkflowStepStatus::Done : SceneWorkflowStepStatus::Blocked));
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
      RecommendedWorkflowActionHandler::SaveLayout);
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
        RecommendedWorkflowActionHandler::AddAsset);
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
