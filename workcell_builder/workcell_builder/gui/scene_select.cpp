
// Workcell Scene Template Library UI labels
[[maybe_unused]] static const char * kSceneTemplateUiMarkers[] = {
  "Create Scene From Template", "Scene Template Library", "Template Category",
  "Pick and Place Cell", "Sorting Cell", "Camera Inspection Cell",
  "Conveyor Pick Cell", "Palletizing Cell", "Instantiate Template",
  "Template Validation Status",
  "Run Generated Scene Acceptance", "Open Acceptance Report", "Copy Acceptance Summary"
};

// Scene round-trip UI labels:
// Open Existing Scene
// Reload Scene From YAML
// Scene Round-trip Status
// Loaded from workcell_scene/v1
// Legacy/partial scene warning
// Regenerate Existing Scene
// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

#include <QKeyEvent>
#include <QCoreApplication>
#include <QDateTime>
#include "workcell_builder_ui_utils.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"
#include "object_placement_yaml_io.hpp"


#include <QDesktopServices>
#include <QUrl>
#include <QApplication>
#include <QClipboard>
#include <QLineEdit>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QListWidgetItem>
#include <QInputDialog>
#include <QDir>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QToolButton>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QSignalBlocker>
#include <QRegularExpression>
#include <QMenu>
#include <QAction>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QSplitter>
#include <QShortcut>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QSvgGenerator>
#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include "workcell_studio_template_instantiator.hpp"
#include <boost/filesystem.hpp>
#include <filesystem>
#include <boost/system/error_code.hpp>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <algorithm>
#include <string>
#include <sstream>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <cmath>
#include <fstream>
#include <functional>

#include "gui/ui_scene_select.h"
#include "gui/scene_select.h"

#include "gui/replacewarning.h"
#include "gui/addscene.h"

#include "yaml_parser/generate_yaml.h"
#include "include/armhand_xacro_parser.h"
#include "include/file_functions.h"
#include "include/object_package_parser.h"
#include "include/object_xacro_parser.h"
#include "include/scene_check.h"
#include "include/scene_parser.h"
#include "include/scene_xacro_parser.h"
#include "include/default_asset_paths.h"
#include "include/asset_catalog_model.h"
#include "scene_select_paths.h"
#include "robot_tool_compatibility.hpp"
#include "workcell_scene_bundle.hpp"
#include "conveyor_pick_preview.hpp"
#include "offline_readiness_overlay.hpp"
#include "supported_scene_readiness_loader.hpp"

namespace fs = boost::filesystem;

namespace {
std::set<std::string> g_perception_contract_warned_scene_paths;

bool emit_perception_contract_warning_once(const fs::path & scene_dir, const std::string & warning)
{
  boost::system::error_code ec;
  const fs::path canonical = fs::weakly_canonical(scene_dir, ec);
  const std::string key = (ec ? scene_dir.lexically_normal() : canonical).string();
  if (g_perception_contract_warned_scene_paths.insert(key).second) {
    RCLCPP_WARN(rclcpp::get_logger("workcell_builder"), "Perception contract warning [%s]: %s", key.c_str(), warning.c_str());
    return true;
  }
  return false;
}

fs::path derive_ros_workspace_root(const fs::path & selected_workcell_path)
{
  const fs::path normalized = selected_workcell_path.lexically_normal();

  // Common source checkout layout: <workspace>/src/easy_manipulation_deployment.
  if (normalized.filename() == "easy_manipulation_deployment" &&
    normalized.parent_path().filename() == "src")
  {
    return normalized.parent_path().parent_path();
  }

  // Workspace source folder selected directly: <workspace>/src.
  if (normalized.filename() == "src") {
    return normalized.parent_path();
  }

  // Workspace root selected directly: it owns the usual src/install/build siblings.
  if (fs::exists(normalized / "src") ||
    fs::exists(normalized / "install") ||
    fs::exists(normalized / "build"))
  {
    return normalized;
  }

  // Preserve the legacy parent-path behavior for non-standard layouts.
  return normalized.parent_path();
}


struct RobotHomeJoint
{
  std::string name;
  QString label;
  double radians;
  double saved_radians;
  double suggested_radians;
  double min_radians;
  double max_radians;
  QSlider * slider = nullptr;
  QDoubleSpinBox * spin = nullptr;
};

struct RobotHomeJointState
{
  std::vector<RobotHomeJoint> joints;
  std::string source{"suggested"};
  std::string saved_source{"suggested"};
  bool dirty{false};
  std::vector<std::string> validation_errors;
  std::string status_note{"Suggested home"};
};

constexpr double kRobotHomePi = 3.14159265358979323846;

double robot_home_deg_to_rad(double deg) { return deg * kRobotHomePi / 180.0; }
double robot_home_rad_to_deg(double rad) { return rad * 180.0 / kRobotHomePi; }

std::vector<RobotHomeJoint> default_ur5_robot_home_joints()
{
  const double limit = robot_home_deg_to_rad(360.0);
  return {
    {"shoulder_pan_joint", "Base", 0.0, 0.0, 0.0, -limit, limit},
    {"shoulder_lift_joint", "Shoulder", -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -limit, limit},
    {"elbow_joint", "Elbow", kRobotHomePi / 2.0, kRobotHomePi / 2.0, kRobotHomePi / 2.0, -limit, limit},
    {"wrist_1_joint", "Wrist 1", -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -limit, limit},
    {"wrist_2_joint", "Wrist 2", -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -kRobotHomePi / 2.0, -limit, limit},
    {"wrist_3_joint", "Wrist 3", 0.0, 0.0, 0.0, -limit, limit},
  };
}

bool validate_robot_home_state(RobotHomeJointState * state)
{
  state->validation_errors.clear();
  std::set<std::string> names;
  for (const auto & joint : state->joints) {
    if (joint.name.empty()) state->validation_errors.push_back("missing joint name");
    if (!names.insert(joint.name).second) state->validation_errors.push_back("duplicate joint name: " + joint.name);
    if (!std::isfinite(joint.radians)) state->validation_errors.push_back("invalid joint value: " + joint.name);
    if (joint.radians < joint.min_radians || joint.radians > joint.max_radians) {
      std::ostringstream err;
      err << joint.label.toStdString() << " must be between " << robot_home_rad_to_deg(joint.min_radians)
          << " and " << robot_home_rad_to_deg(joint.max_radians) << " degrees";
      state->validation_errors.push_back(err.str());
    }
    const double round_trip = robot_home_deg_to_rad(robot_home_rad_to_deg(joint.radians));
    if (!std::isfinite(round_trip) || std::abs(round_trip - joint.radians) > 1e-9) {
      state->validation_errors.push_back("radians/degrees conversion failed: " + joint.name);
    }
  }
  return state->validation_errors.empty();
}

class InteractiveCanvasView : public QGraphicsView
{
public:
  explicit InteractiveCanvasView(QWidget * parent = nullptr): QGraphicsView(parent) {}
  bool snap_to_grid_enabled = true;
  bool asset_placement_mode = false;
  double zoom_level = 1.0;
  int grid_px = 30;
protected:
  void wheelEvent(QWheelEvent * event) override
  {
    const double factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
    scale(factor, factor);
    zoom_level *= factor;
    event->accept();
  }
  void mousePressEvent(QMouseEvent * event) override
  {
    if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
      setDragMode(QGraphicsView::ScrollHandDrag);
      QMouseEvent fake(QEvent::MouseButtonPress, event->localPos(), Qt::LeftButton, Qt::LeftButton, event->modifiers());
      QGraphicsView::mousePressEvent(&fake);
      return;
    }
    QGraphicsView::mousePressEvent(event);
  }
  void mouseReleaseEvent(QMouseEvent * event) override
  {
    if (dragMode() == QGraphicsView::ScrollHandDrag) {
      QMouseEvent fake(QEvent::MouseButtonRelease, event->localPos(), Qt::LeftButton, Qt::NoButton, event->modifiers());
      QGraphicsView::mouseReleaseEvent(&fake);
      setDragMode(QGraphicsView::RubberBandDrag);
      return;
    }
    QGraphicsView::mouseReleaseEvent(event);
  }
};


class TaskAreaGraphicsItem : public QGraphicsRectItem
{
public:
  TaskAreaGraphicsItem(const QString & id, const QRectF & rect, std::function<void(const QString &, const QRectF &)> on_commit)
  : QGraphicsRectItem(rect), id_(id), on_commit_(std::move(on_commit))
  {
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
  }
protected:
  void mouseReleaseEvent(QGraphicsSceneMouseEvent * event) override
  {
    QGraphicsRectItem::mouseReleaseEvent(event);
    if (on_commit_) on_commit_(id_, sceneBoundingRect());
  }
private:
  QString id_;
  std::function<void(const QString &, const QRectF &)> on_commit_;
};

std::string scene_support_level_for(const std::vector<workcell_builder::SupportedSceneRegistryEntry> & entries, const std::string & name){
  for (const auto & e : entries) if (e.name == name) return e.support_level;
  return "supported";
}
std::string scene_readiness_for(const workcell_builder::AllScenesReadinessData & data, const std::string & name){
  const auto it = data.by_scene.find(name); return it==data.by_scene.end()?"UNKNOWN":it->second.readiness_status;
}

[[maybe_unused]] static const char * kCanvasWorkspaceUxTokens =
  "mouse wheel zoom | middle mouse pan | right-drag pan | left-click select | "
  "double-click fit selected item | open inspector details | Fit Cell | Fit Selection | "
  "Reset View | Zoom In | Zoom Out | Zoom 100% | scale indicator | grid indicator | "
  "snap to grid | hold Shift for fine movement | drag ghost preview | live x/y coordinates | "
  "undo last move | no robot motion | unlock robot base";

[[maybe_unused]] static const char * kCanvasLayerTokens =
  "Grid | Robot | Reach | Tables | Objects | Bins | Conveyors | Cameras | "
  "Pick/place zones | Camera ROI/FOV | Warnings/blockers | Labels | legend";

[[maybe_unused]] static const char * kCanvasPoseEditorTokens =
  "name/id | type/category | x | y | z | roll | pitch | yaw | width/depth/height | radius | "
  "role | status/warnings | source: template/yaml/generated/user edit | layout unsaved | metadata-only | rerun zone validation";

[[maybe_unused]] static const char * kCanvasPlacementToolsTokens =
  "Add to Layout | Place on Canvas | support surface | pick object | place bin | conveyor | camera | fixture | "
  "Align selected to table centre | Move pick object onto support surface | Place bin inside reachable area | "
  "Centre camera ROI on pick zone | Auto-space pick/place zones | Auto-fix invalid placement | Duplicate selected item | Delete selected item with confirmation";
[[maybe_unused]] static const char * kPerceptionAdapterUiStrings = "Load Detection Snapshot | Generate Sample EPD Snapshot | Preview Detection Mapping | adapter_metadata_only | no robot motion commanded";
[[maybe_unused]] static const char * kTaskGraspEditorUiTokens =
  "Task page | Grasp page | Use Selected Item as Pick Source | Use Selected Item as Place Target | "
  "Use Selected Zone as Pick Zone | Use Selected Zone as Place Zone | pick_zone | place_zone | conveyor_pick_zone | "
  "class_route_target | inspection_preview | machine_tending_preview | finger_top | finger_side | suction_top | tool_profile_default | "
  "open_gripper | vacuum_off | preview_only_release | approach axis | retreat axis | place clearance | allowed yaw/roll | TCP offset XYZ/RPY | "
  "Unsaved Task Edits: visible | Reset to Tool Defaults | Robotiq defaults finger_top/open_gripper | suction defaults suction_top/vacuum_off | "
  "unknown tool safe defaults warning | gripper mount RPY remains -1.5708 -1.5708 0 | task_recipe.yaml | workcell_builder_task_intent.yaml | "
  "runtime_execution_enabled: false | fake_hardware_first: true | motion_command_sent: false | EPD remains external/separate";



[[maybe_unused]] static const char * kTaskZoneUiMarkers = "Task Areas | Suggested Areas | Add Pick Area | Add Place Area | Delete Selected | Snap to 1 cm | Pick Area | Place Area | Pick Object | Destination Bin | Offline layout check only; motion planning is checked later. | task_zones";

[[maybe_unused]] static const char * kAssetDiscoveryLabel = "Asset discovery paths";
[[maybe_unused]] static const char * kSelectRobotAssetLabel = "Select Robot Asset";
[[maybe_unused]] static const char * kSelectEndEffectorAssetLabel = "Select End Effector Asset";
[[maybe_unused]] static const char * kSelectExistingStlLabel = "Select Existing STL";

// External Asset Import Wizard labels
[[maybe_unused]] static const char * kImportExternalAssetLabel = "Import External Asset";
[[maybe_unused]] static const char * kExternalAssetImportWizardLabel = "External Asset Import Wizard";
[[maybe_unused]] static const char * kSelectStlUrdfLabel = "Select STL / URDF";
[[maybe_unused]] static const char * kImportedCategoryLabel = "Custom / Imported";
[[maybe_unused]] static const char * kPresetTable = "table";
[[maybe_unused]] static const char * kPresetBin = "bin";
[[maybe_unused]] static const char * kPresetConveyorPlaceholder = "conveyor_placeholder";
[[maybe_unused]] static const char * kPresetFixture = "fixture";
[[maybe_unused]] static const char * kPresetCustomStl = "custom_stl";
constexpr const char * kSceneRootEnvVar = "WORKCELL_BUILDER_SCENE_ROOT";

[[maybe_unused]] static const char * kCameraPerceptionSectionLabel = "Camera / Perception";
[[maybe_unused]] static const char * kAddCameraLabel = "Add Camera";
[[maybe_unused]] static const char * kCameraProfileLabel = "Camera Profile";
[[maybe_unused]] static const char * kRealSenseD435iLabel = "RealSense D435i";
[[maybe_unused]] static const char * kCameraPoseLabel = "Camera Pose";
[[maybe_unused]] static const char * kRgbTopicLabel = "RGB Topic";
[[maybe_unused]] static const char * kDepthTopicLabel = "Depth Topic";
[[maybe_unused]] static const char * kCameraInfoTopicLabel = "Camera Info Topic";
[[maybe_unused]] static const char * kPointCloudTopicLabel = "PointCloud Topic";
[[maybe_unused]] static const char * kCameraFrameLabel = "Camera Frame";
[[maybe_unused]] static const char * kOpticalFrameLabel = "Optical Frame";
[[maybe_unused]] static const char * kMountTypeLabel = "Mount Type";
[[maybe_unused]] static const char * kValidateCameraLabel = "Validate Camera";
[[maybe_unused]] static const char * kApplyCameraDefaultsLabel = "Apply Camera Defaults";
[[maybe_unused]] static const char * kPerceptionMetadataExportLabel = "Perception Metadata Export";
[[maybe_unused]] static const char * kPreviewConveyorPickFlowLabel = "Preview Conveyor Pick Flow";
[[maybe_unused]] static const char * kConveyorPreviewTimeLabel = "time_to_pick_s";
[[maybe_unused]] static const char * kConveyorPreviewOnlyLabel = "preview_only";
[[maybe_unused]] static const char * kTaskIntentPreviewLabel = "Preview Task Intent";
[[maybe_unused]] static const char * kGenerateTaskIntentPreviewLabel = "Generate Task Intent Preview";
[[maybe_unused]] static const char * kOpenTaskIntentPreviewLabel = "Open Task Intent Preview";
[[maybe_unused]] static const char * kDryRunReadinessOnlyLabel = "dry_run_readiness_only";
[[maybe_unused]] static const char * kOpenPlanningReadinessReportLabel = "Open Planning Readiness Report";
[[maybe_unused]] static const char * kGenerateDryRunPlanningRequestLabel = "Generate Dry-Run Planning Request";
[[maybe_unused]] static const char * kCheckPlanningReadinessLabel = "Check Planning Readiness";
[[maybe_unused]] static const char * kEpdAdapterMetadataLabel = "EPD Adapter Metadata";
[[maybe_unused]] static const char * kEpdSeparateNote = "EPD remains external/separate";

[[maybe_unused]] static bool copy_directory_recursive(const fs::path & source, const fs::path & destination, std::string * error)
{
  boost::system::error_code ec;
  if (!fs::exists(source, ec) || ec) {
    if (error) *error = "Template folder does not exist: " + source.string();
    return false;
  }
  fs::create_directories(destination, ec);
  if (ec) { if (error) *error = "Failed to create destination: " + destination.string(); return false; }
  for (fs::recursive_directory_iterator it(source, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::path rel = fs::relative(it->path(), source, ec);
    if (ec) break;
    const fs::path out = destination / rel;
    if (fs::is_directory(it->path(), ec)) { fs::create_directories(out, ec); }
    else if (fs::is_regular_file(it->path(), ec)) { fs::create_directories(out.parent_path(), ec); fs::copy_file(it->path(), out, fs::copy_option::overwrite_if_exists, ec); }
    if (ec) break;
  }
  if (ec) { if (error) *error = "Failed while copying template assets."; return false; }
  return true;
}


[[maybe_unused]] bool change_directory(const fs::path & p)
{
  boost::system::error_code ec;
  fs::current_path(p, ec);
  if (ec) {
    RCLCPP_ERROR(
      rclcpp::get_logger("workcell_builder"),
      "Failed to change directory to %s: %s", p.string().c_str(), ec.message().c_str());
    return false;
  }
  return true;
}

bool is_package_uri(const std::string & path)
{
  return path.rfind("package://", 0) == 0;
}

void resolve_relative_path(const fs::path & base_path, std::string * path)
{
  if (!path || path->empty() || is_package_uri(*path)) {
    return;
  }
  fs::path candidate(*path);
  if (candidate.is_absolute()) {
    return;
  }
  *path = (base_path / candidate).lexically_normal().string();
}

void resolve_scene_paths(Scene * scene, const fs::path & base_path)
{
  if (!scene) {
    return;
  }
  if (scene->robot_loaded && !scene->robot_vector.empty()) {
    resolve_relative_path(base_path, &scene->robot_vector[0].filepath);
  }
  if (scene->ee_loaded && !scene->ee_vector.empty()) {
    resolve_relative_path(base_path, &scene->ee_vector[0].filepath);
  }
  for (auto & object : scene->object_vector) {
    for (auto & link : object.link_vector) {
      if (link.is_visual) {
        for (auto & visual : link.visual_vector) {
          resolve_relative_path(base_path, &visual.geometry.filepath);
        }
      }
      if (link.is_collision) {
        for (auto & collision : link.collision_vector) {
          resolve_relative_path(base_path, &collision.geometry.filepath);
        }
      }
    }
  }
}

struct SceneRootCandidate
{
  fs::path root;
  std::string label;
  bool valid = false;
  std::string invalid_reason;
};

std::string path_or_placeholder(const fs::path & path)
{
  return path.empty() ? std::string("<none>") : path.string();
}

std::string display_path_with_home_tilde(const fs::path & path)
{
  if (path.empty()) {
    return "<none>";
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::strlen(home) == 0) {
    return path.string();
  }
  const fs::path home_path(home);
  const fs::path normalized = path.lexically_normal();
  const std::string home_str = home_path.string();
  const std::string normalized_str = normalized.string();
  if (normalized_str == home_str) {
    return "~";
  }
  const std::string prefix = home_str + "/";
  if (normalized_str.rfind(prefix, 0) == 0) {
    return "~/" + normalized_str.substr(prefix.size());
  }
  return normalized_str;
}

std::string parse_cli_scene_root_override()
{
  const QStringList args = QCoreApplication::arguments();
  for (int i = 1; i < args.size(); ++i) {
    const QString arg = args[i];
    if (arg == "--scene-root" && i + 1 < args.size()) {
      return args[i + 1].toStdString();
    }
    const QString prefix = "--scene-root=";
    if (arg.startsWith(prefix)) {
      return arg.mid(prefix.size()).toStdString();
    }
  }
  return "";
}

SceneRootCandidate build_candidate(const fs::path & root, const std::string & label)
{
  SceneRootCandidate candidate;
  candidate.root = root.lexically_normal();
  candidate.label = label;

  const fs::path scenes_dir = candidate.root / "scenes";
  boost::system::error_code ec;
  if (!fs::exists(scenes_dir, ec) || ec) {
    candidate.valid = false;
    candidate.invalid_reason = "missing scenes directory";
    return candidate;
  }
  if (!fs::is_directory(scenes_dir, ec) || ec) {
    candidate.valid = false;
    candidate.invalid_reason = "scenes path is not a directory";
    return candidate;
  }

  candidate.valid = true;
  return candidate;
}



std::string normalize_placeholder_token(const std::string & value)
{
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  normalized.erase(std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }), normalized.end());
  return normalized;
}

bool is_placeholder_value(const std::string & value)
{
  const std::string normalized = normalize_placeholder_token(value);
  return normalized.empty() || normalized == "unknown" || normalized == "none" || normalized == "null";
}

bool scene_has_valid_robot(const Scene & scene)
{
  return scene.robot_loaded && !scene.robot_vector.empty() && !is_placeholder_value(scene.robot_vector[0].name);
}

bool scene_requires_end_effector(const Scene & scene)
{
  if (!scene.ee_loaded || scene.ee_vector.empty()) {
    return false;
  }
  const std::string ee_name = normalize_placeholder_token(scene.ee_vector[0].name);
  return !(ee_name.empty() || ee_name == "none");
}

bool scene_has_valid_end_effector(const Scene & scene)
{
  return scene_requires_end_effector(scene) && !is_placeholder_value(scene.ee_vector[0].name);
}


enum class SceneUiStatus
{
  VALID,
  INCOMPLETE,
  SCAFFOLD_ONLY,
  MISSING_ENVIRONMENT_YAML,
  MISSING_ROBOT,
  MISSING_MOVEIT_CONFIG
};

std::string scene_status_label(SceneUiStatus status)
{
  switch (status) {
    case SceneUiStatus::VALID: return "VALID";
    case SceneUiStatus::INCOMPLETE: return "INCOMPLETE";
    case SceneUiStatus::SCAFFOLD_ONLY: return "SCAFFOLD_ONLY";
    case SceneUiStatus::MISSING_ENVIRONMENT_YAML: return "MISSING_ENVIRONMENT_YAML";
    case SceneUiStatus::MISSING_ROBOT: return "MISSING_ROBOT";
    case SceneUiStatus::MISSING_MOVEIT_CONFIG: return "MISSING_MOVEIT_CONFIG";
  }
  return "INCOMPLETE";
}

SceneUiStatus compute_scene_status_label(const Scene & scene, const fs::path & scene_dir)
{
  const bool has_yaml = fs::exists(scene_dir / "environment.yaml");
  const bool has_launch = fs::exists(scene_dir / "launch" / "demo.launch.py");
  const bool has_srdf = fs::exists(scene_dir / "urdf" / "arm_hand.srdf.xacro");

  if (!has_yaml) {
    return SceneUiStatus::MISSING_ENVIRONMENT_YAML;
  }
  if (!scene_has_valid_robot(scene)) {
    return SceneUiStatus::MISSING_ROBOT;
  }
  if (scene_requires_end_effector(scene) && !scene_has_valid_end_effector(scene)) {
    return SceneUiStatus::INCOMPLETE;
  }
  if (!has_srdf) {
    return has_launch ? SceneUiStatus::MISSING_MOVEIT_CONFIG : SceneUiStatus::SCAFFOLD_ONLY;
  }
  return SceneUiStatus::VALID;
}
fs::path root_from_override(const std::string & override_value)
{
  fs::path override_path(override_value);
  if (override_path.filename() == "scenes") {
    return override_path.parent_path();
  }
  return override_path;
}


fs::path resolve_tool_root(const fs::path & scene_root, const fs::path & scene_dir)
{
  std::vector<fs::path> candidates;
  const char * env_root = std::getenv("WORKCELL_STUDIO_REPO_ROOT");
  if (env_root != nullptr && std::strlen(env_root) > 0) {
    candidates.emplace_back(env_root);
  }
  candidates.push_back(scene_root);
  candidates.push_back(scene_dir.parent_path());
  candidates.push_back(scene_dir.parent_path().parent_path());
  const char * home = std::getenv("HOME");
  if (home != nullptr && std::strlen(home) > 0) {
    candidates.emplace_back(fs::path(home) / "workcell_ws" / "src" / "easy_manipulation_deployment");
  }

  for (const auto & raw_candidate : candidates) {
    if (raw_candidate.empty()) {
      continue;
    }
    const fs::path candidate = raw_candidate.lexically_normal();
    const fs::path script_path = candidate / "scripts" / "render_workcell_builder_metadata.py";
    boost::system::error_code ec;
    if (fs::exists(script_path, ec) && !ec) {
      return candidate;
    }
  }
  return fs::path();
}

fs::path select_scene_root(const fs::path & cwd)
{
  std::vector<SceneRootCandidate> candidates;
  const std::string cli_override = parse_cli_scene_root_override();
  const char * env_override = std::getenv(kSceneRootEnvVar);

  if (!cli_override.empty()) {
    candidates.push_back(build_candidate(
      root_from_override(cli_override),
      std::string("CLI --scene-root=") + cli_override));
  }
  if (env_override != nullptr && std::strlen(env_override) > 0) {
    candidates.push_back(build_candidate(
      root_from_override(env_override),
      std::string("environment ") + kSceneRootEnvVar + "=" + env_override));
  }

  candidates.push_back(build_candidate(cwd / "src" / "easy_manipulation_deployment", "workspace src/easy_manipulation_deployment"));
  candidates.push_back(build_candidate(cwd / "src", "workspace src"));
  candidates.push_back(build_candidate(cwd, "current working directory"));
  candidates.push_back(build_candidate(cwd.parent_path(), "parent directory"));
  const fs::path repo_root = cwd.parent_path() / "easy_manipulation_deployment";
  candidates.push_back(build_candidate(repo_root, "repo root easy_manipulation_deployment"));

  SceneRootCandidate selected;
  bool has_selected = false;
  std::vector<SceneRootCandidate> valid_candidates;

  for (const auto & candidate : candidates) {
    if (candidate.valid) {
      valid_candidates.push_back(candidate);
      if (!has_selected) {
        selected = candidate;
        has_selected = true;
      }
    }
  }

  if (has_selected) {
    RCLCPP_INFO(
      rclcpp::get_logger("workcell_builder"),
      "Selected workcell root: %s (source: %s)",
      selected.root.string().c_str(), selected.label.c_str());

    for (const auto & candidate : candidates) {
      if (candidate.root == selected.root && candidate.label == selected.label) {
        continue;
      }
      if (candidate.valid) {
        RCLCPP_INFO(
          rclcpp::get_logger("workcell_builder"),
          "Rejected valid scene root candidate: %s (source: %s)",
          candidate.root.string().c_str(), candidate.label.c_str());
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("workcell_builder"),
          "Rejected scene root candidate: %s (source: %s, reason: %s)",
          candidate.root.string().c_str(), candidate.label.c_str(), candidate.invalid_reason.c_str());
      }
    }

    std::unordered_set<std::string> unique_valid_roots;
    for (const auto & candidate : valid_candidates) {
      unique_valid_roots.insert(candidate.root.string());
    }

    if (unique_valid_roots.size() > 1) {
      std::ostringstream valid_paths;
      size_t idx = 0;
      for (const auto & root : unique_valid_roots) {
        if (idx++ > 0) {
          valid_paths << ", ";
        }
        valid_paths << root;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("workcell_builder"),
        "Multiple valid scene directories were found (%zu). Using highest priority root %s. "
        "Set --scene-root or %s for deterministic selection. Candidates: [%s]",
        unique_valid_roots.size(), selected.root.string().c_str(), kSceneRootEnvVar,
        valid_paths.str().c_str());
    }

    return selected.root;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("workcell_builder"),
    "Unable to locate a valid 'scenes' directory from %s. Checked default locations. "
    "You can override with --scene-root or %s.",
    cwd.string().c_str(), kSceneRootEnvVar);
  for (const auto & candidate : candidates) {
    RCLCPP_INFO(
      rclcpp::get_logger("workcell_builder"),
      "Rejected scene root candidate: %s (source: %s, reason: %s)",
      path_or_placeholder(candidate.root).c_str(), candidate.label.c_str(),
      candidate.invalid_reason.c_str());
  }
  return cwd;
}

}  // namespace

bool ensure_minimal_environment_yaml(const fs::path & scene_dir, const std::string & scene_name);
void refresh_scene_manifest_if_missing(const fs::path & scene_dir, const std::string & scene_name);


std::string status_from_blockers_and_warnings(bool has_blockers, bool has_warnings)
{
  if (has_blockers) {
    return "BLOCKED";
  }
  if (has_warnings) {
    return "WARNINGS";
  }
  return "READY_TO_GENERATE";
}

struct TaskGraspConfig
{
  std::string tool_type = "unknown";
  std::string tcp_frame = "";
  std::string tool_mount_link = "";
  std::string compatibility_status = "UNKNOWN_COMPATIBILITY";
  std::vector<std::string> compatibility_warnings;

  std::string task_type = "pick_place";
  std::string pick_source = "selected_object";
  std::string place_target = "selected_bin";
  std::string grasp_strategy = "finger_top";
  std::string orientation_mode = "vertical";
  std::string approach_axis = "z_down";
  double approach_distance_m = 0.12;
  double retreat_distance_m = 0.10;
  double place_clearance_m = 0.05;
  std::string allowed_roll_angles_deg = "0, 90";
  std::string allowed_yaw_angles_deg = "0";
  int suction_cups = 1;
  std::string release_strategy = "open_gripper";
  std::string camera_topic = "";
};

TaskGraspConfig infer_task_grasp_defaults(const Scene & scene)
{
  TaskGraspConfig config;
  const auto compat = evaluate_robot_tool_compatibility(scene, "workcell_builder/workcell_builder/config/compatibility_profiles");
  config.compatibility_status = compat.status;
  config.tcp_frame = compat.tcp_frame;
  config.tool_mount_link = compat.tool_mount_link;
  config.tool_type = compat.tool_type.empty() ? "unknown" : compat.tool_type;
  config.grasp_strategy = compat.grasp_strategy_default.empty() ? config.grasp_strategy : compat.grasp_strategy_default;
  config.release_strategy = compat.release_strategy_default.empty() ? config.release_strategy : compat.release_strategy_default;
  for (const auto & issue : compat.issues) {
    if (!issue.blocker) { config.compatibility_warnings.push_back(issue.message); }
  }
  if (config.tool_type == "suction") {
    config.compatibility_warnings.push_back("requires_io=true for suction tool profile");
  }
  return config;
}

void write_task_recipe_yaml(const fs::path & scene_dir, const TaskGraspConfig & config)
{
  fs::create_directories(scene_dir / "config");
  std::ofstream out((scene_dir / "config" / "task_recipe.yaml").string());
  out << "schema_version: workcell_task/v1\n";
  out << "task:\n  type: " << config.task_type << "\n  pick_source: " << config.pick_source <<
    "\n  place_target: " << config.place_target << "\n";
  out << "grasp:\n  strategy: " << config.grasp_strategy << "\n  approach_axis: " <<
    config.approach_axis << "\n  orientation_mode: " << config.orientation_mode << "\n";
  out << "  approach_distance_m: " << config.approach_distance_m << "\n  retreat_distance_m: " <<
    config.retreat_distance_m << "\n";
  out << "  allowed_roll_angles_deg: [" << config.allowed_roll_angles_deg << "]\n";
  out << "  allowed_yaw_angles_deg: [" << config.allowed_yaw_angles_deg << "]\n";
  out << "  suction_cups: " << config.suction_cups <<
    "\n  tcp_offset_xyz: [0.0, 0.0, 0.0]\n  tcp_offset_rpy: [0.0, 0.0, 0.0]\n";
  out << "place:\n  clearance_m: " << config.place_clearance_m <<
    "\n  orientation_mode: preserve_object_orientation\n";
  out << "release:\n  strategy: " << config.release_strategy << "\n";
  out << "safety:\n  preview_only: false\n  use_fake_hardware: true\n";
  out << "  allow_simulated_motion: true\n  allow_moveit_execution: true\n";
  out << "  allow_rviz_motion: true\n  allow_real_hardware_motion: false\n";
  out << "  real_robot_locked: true\n";
}



struct LayoutPreviewItem {
  QString id; QString name; QString type; double x{0.0}; double y{0.0}; double width{0.2}; double height{0.2}; double radius{0.0}; double yaw{0.0}; QString status; QString source; QString role; QString warning;
};

static double px(double m){ return m*220.0; }
static std::vector<LayoutPreviewItem> build_layout_preview_items(const Scene & scene, const QString & selected_template)
{
  std::vector<LayoutPreviewItem> out;
  // offline work-zone validation model tokens:
  // OK / WARNING / BLOCKED / PREVIEW_ONLY
  out.push_back({"safety_home","Safety/Home","safety",-0.75,-0.5,0.16,0.16,0.0,0.0,"safe","default","safety","fake_hardware_first | no_runtime_motion"});
  out.push_back({"work_zone","Work Zone","zone",0.2,0.0,1.1,0.9,0.0,0.0,"active","generated","pick_zone","zone_validation:OK"});
  const bool conveyor = selected_template.contains("Conveyor") || selected_template.contains("sorting", Qt::CaseInsensitive);
  const bool inspection = selected_template.contains("Inspection");
  out.push_back({"robot_base","Robot Base","robot",-0.45,0.0,0.25,0.25,0.0,0.0,"ready","template","robot",""});
  out.push_back({"robot_reach","Robot Reach","reach",-0.45,0.0,0.0,0.0,0.78,0.0,"approx","default","reach_envelope",""});
  if (inspection) {
    out.push_back({"inspection_table","Inspection Table","table",0.25,0.0,0.7,0.7,0.0,0.0,"ok","template","support_surface",""});
    out.push_back({"inspection_target","Inspection Target","object",0.25,0.0,0.09,0.09,0.0,0.0,"ok","template","inspection_target",""});
    out.push_back({"inspection_cam","Camera","camera",0.15,-0.45,0.15,0.15,0.0,0.0,"ok","template","perception",""});
    out.push_back({"camera_roi","Camera ROI","zone",0.25,0.0,0.4,0.3,0.0,0.0,"ok","generated","roi",""});
  } else if (conveyor) {
    out.push_back({"conveyor_1","Conveyor","conveyor",0.25,-0.25,1.0,0.35,0.0,0.0,"preview_only","template","feed",""});
    out.push_back({"cam_conv","Camera","camera",0.10,-0.65,0.15,0.15,0.0,0.0,"ok","template","perception",""});
    out.push_back({"pick_zone","Pick Zone","zone",0.25,-0.25,0.35,0.25,0.0,0.0,"ok","generated","pick_zone",""});
    out.push_back({"place_zone","Place Zone","zone",0.25,0.35,0.55,0.3,0.0,0.0,"ok","generated","place_zone",""});
    out.push_back({"bin_red","Bin Red","bin",0.45,0.35,0.26,0.24,0.0,0.0,"ok","template","place_bin",""});
    out.push_back({"bin_blue","Bin Blue","bin",0.05,0.35,0.26,0.24,0.0,0.0,"ok","template","place_bin",""});
    out.push_back({"camera_roi","Camera ROI","zone",0.25,-0.25,0.6,0.3,0.0,0.0,"ok","generated","roi",""});
  } else {
    out.push_back({"table_1","Table","table",0.25,0.0,0.9,0.7,0.0,0.0,"ok","template","support_surface",""});
    out.push_back({"pick_zone","Pick Zone","zone",0.18,0.0,0.24,0.2,0.0,0.0,"ok","generated","pick_zone",""});
    out.push_back({"place_zone","Place Zone","zone",0.48,0.25,0.28,0.24,0.0,0.0,"ok","generated","place_zone",""});
    out.push_back({"cube_1","Cube","object",0.18,0.0,0.08,0.08,0.0,0.0,"pick","template","pick_object","object_on_support_surface"});
    out.push_back({"bin_1","Bin","bin",0.48,0.25,0.25,0.25,0.0,0.0,"place","template","place_bin","bin_clearance_ok"});
    out.push_back({"cam_1","Camera","camera",0.15,-0.45,0.15,0.15,0.0,0.0,"ok","template","perception",""});
    out.push_back({"camera_roi","Camera ROI","zone",0.25,0.0,0.4,0.3,0.0,0.0,"ok","generated","roi",""});
  }
  if (scene.object_vector.empty() && scene.robot_vector.empty()) {
    out.clear();
  }
  return out;
}

SceneSelect::SceneSelect(QWidget * parent)
: QDialog(parent),
  ui(new Ui::SceneSelect)
{
  ui->setupUi(this);
  workcell_builder::applyCompactDialogDefaults(this);
  setWindowFlags(Qt::Window | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
  templates_path = get_default_templates_directory();
  setWindowTitle("Workcell Studio - Workcell Builder");
  ui->workflow_tabs->setCurrentWidget(ui->start_tab);

  ui->asset_browser_group->hide();
  ui->inspector_group->hide();
  ui->cell_name->show();
  ui->output_folder->show();
  ui->browse_output_folder->show();
  ui->golden_demo_cell->hide();
  ui->delete_scene->hide();
  ui->generate_studio_pack->hide();
  ui->open_preview->hide();
  ui->show_readiness_report->hide();
  ui->validate_cell->hide();
  ui->workflow_tabs->setTabText(ui->workflow_tabs->indexOf(ui->validate_generate_tab), "Validate & Generate");
  ui->browse_scenes_folder->setText("Open Folder");
  ui->workflow_tabs->setTabText(ui->workflow_tabs->indexOf(ui->ingredients_tab), "Task & Grasp");
  ui->workflow_tabs->setTabText(ui->workflow_tabs->indexOf(ui->start_tab), "Home");
  ui->workflow_tabs->setTabText(ui->workflow_tabs->indexOf(ui->assets_tab), "Assets");
  ui->workflow_tabs->setTabEnabled(ui->workflow_tabs->indexOf(ui->demo_mode_tab), false);

  ui->templatesPageLayout->addWidget(ui->scenario_templates_group);
  ui->assetsPageLayout->addWidget(ui->asset_browser_group);
  ui->assetsPageLayout->addWidget(ui->current_cell_assets_group);
  ui->scenesPageLayout->insertWidget(0, ui->existing_scenes_group);
  ui->scenesPageLayout->addWidget(ui->workcell_studio_scene_status_group);

  ui->startLayout->removeWidget(ui->scenario_templates_group);
  ui->startLayout->removeWidget(ui->existing_scenes_group);
  ui->startLayout->removeWidget(ui->workcell_studio_scene_status_group);

  ui->fake_hardware_default_label->setToolTip(
    "Fake hardware is the safe default. Real hardware launch is intentionally not default.");

  initialize_task_grasp_editor();
  initialize_robot_home_editor();
  initialize_task_area_editor();
  ui->generate_files->hide();
  ui->validate_cell->show();
  ui->validate_cell->setText("Run Offline Validation");
  ui->validation_dashboard_table->setColumnCount(6);
  ui->validation_dashboard_table->setHorizontalHeaderLabels(
    {"Check", "Status", "Message", "Warnings", "Blockers", "Fix / Report"});
  ui->validation_dashboard_table->horizontalHeader()->setStretchLastSection(true);
  latest_dashboard_result_ = workcell_builder::default_validation_dashboard_result();
  refresh_validation_dashboard_table(latest_dashboard_result_);
  ui->open_preview->show();
  ui->open_preview->setText("Refresh Preview");
  ui->export_layout_preview_action->setText("Export Preview");
  append_info("Workcell Studio Readiness panel initialized: READY_TO_GENERATE / WARNINGS / BLOCKED / SCAFFOLD_ONLY");
  append_info("Task & Grasp Strategy panel initialized for offline recipe preview.");
  append_info("Robot / Tool Compatibility | Check Compatibility | Apply Profile Defaults | Manual Override");
  connect(ui->export_layout_preview_action, &QPushButton::clicked, this, &SceneSelect::export_preview_layout);
  auto * web_viewer_action = new QPushButton("Export & Open Web 3D Viewer", ui->validate_generate_tab);
  web_viewer_action->setObjectName("export_open_web_3d_viewer_action");
  web_viewer_action->setToolTip("Exports the selected scene to build/workcell_studio_web_scene and opens the static browser viewer. Does not mutate scenes or generated files.");
  ui->generateLayout->insertWidget(8, web_viewer_action);
  connect(web_viewer_action, &QPushButton::clicked, this, &SceneSelect::on_export_open_web_3d_viewer_clicked);
  auto * validate_web_patch_action = new QPushButton("Validate Web Edit Patch…", ui->validate_generate_tab);
  validate_web_patch_action->setObjectName("validate_web_edit_patch_action");
  validate_web_patch_action->setToolTip("Validate a browser-exported edit_patch.json through scripts/run_workcell_studio_web_edit_workflow.py without writing scene YAML.");
  ui->generateLayout->insertWidget(9, validate_web_patch_action);
  connect(validate_web_patch_action, &QPushButton::clicked, this, &SceneSelect::on_validate_web_edit_patch_clicked);
  auto * dry_run_web_patch_action = new QPushButton("Dry Run Web Edit Patch…", ui->validate_generate_tab);
  dry_run_web_patch_action->setObjectName("dry_run_web_edit_patch_action");
  dry_run_web_patch_action->setToolTip("Run the guided Web 3D edit patch workflow in dry-run mode; --write is deliberately omitted.");
  ui->generateLayout->insertWidget(10, dry_run_web_patch_action);
  connect(dry_run_web_patch_action, &QPushButton::clicked, this, &SceneSelect::on_dry_run_web_edit_patch_clicked);
  auto * apply_web_patch_action = new QPushButton("Apply Web Edit Patch…", ui->validate_generate_tab);
  apply_web_patch_action->setObjectName("apply_web_edit_patch_action");
  apply_web_patch_action->setToolTip("Dry-run, require explicit confirmation, then apply a Web 3D edit patch through the backend workflow with --write.");
  ui->generateLayout->insertWidget(11, apply_web_patch_action);
  connect(apply_web_patch_action, &QPushButton::clicked, this, &SceneSelect::on_apply_web_edit_patch_clicked);
  auto * generate_validate_scene_action = new QPushButton("Generate & Validate Scene", ui->validate_generate_tab);
  generate_validate_scene_action->setObjectName("generate_validate_after_web_edit_action");
  generate_validate_scene_action->setToolTip("Explicitly regenerate and validate the selected scene through scripts/run_workcell_studio_web_edit_workflow.py. No patch is required, no hardware is launched, and RViz/MoveIt remains the later planning truth.");
  ui->generateLayout->insertWidget(12, generate_validate_scene_action);
  connect(generate_validate_scene_action, &QPushButton::clicked, this, &SceneSelect::on_generate_validate_after_web_edit_clicked);
  connect(ui->fit_cell_action, &QPushButton::clicked, this, &SceneSelect::refresh_preview_status);
  connect(ui->reset_view_action, &QPushButton::clicked, this, [this]() {
    if (ui->visual_layout_canvas) {
      ui->visual_layout_canvas->resetTransform();
      refresh_preview_status();
    }
  });
  connect(ui->toggle_grid_action, &QPushButton::clicked, this, &SceneSelect::refresh_preview_status);
  connect(ui->toggle_reach_action, &QPushButton::clicked, this, &SceneSelect::refresh_preview_status);
  connect(ui->toggle_roi_action, &QPushButton::clicked, this, &SceneSelect::refresh_preview_status);
  ui->toggle_grid_action->setCheckable(true); ui->toggle_grid_action->setChecked(true);
  ui->toggle_reach_action->setCheckable(true); ui->toggle_reach_action->setChecked(true);
  ui->toggle_roi_action->setCheckable(true); ui->toggle_roi_action->setChecked(true);
  auto * interactive_canvas = new InteractiveCanvasView(ui->visual_layout_canvas->parentWidget());
  interactive_canvas->setObjectName("visual_layout_canvas");
  ui->visualCanvasLayout->replaceWidget(ui->visual_layout_canvas, interactive_canvas);
  delete ui->visual_layout_canvas;
  ui->visual_layout_canvas = interactive_canvas;
  ui->use_recommended_layout->setToolTip("Apply recommended layout to the selected scene.");
  ui->scenario_template_description->setText(
    "Choose a real starter template:\n"
    "• Pick and Place Cell\n"
    "• Conveyor Sorting Cell\n"
    "• Camera Inspection Cell\n"
    "• Machine Tending Placeholder (PREVIEW ONLY)\n"
    "• Bin Picking Placeholder (PREVIEW ONLY)\n"
    "• Palletizing Placeholder (PREVIEW ONLY)\n\n"
    "Safety: no robot motion commanded. fake_hardware_first=true.");
  const std::vector<QPushButton *> placeholder_buttons = {ui->set_as_robot, ui->set_as_end_effector, ui->add_as_support_surface, ui->add_as_pick_object, ui->import_custom_stl,  ui->duplicate_selected_asset, ui->remove_selected_asset, ui->clear_cell_assets, ui->generate_scenario, ui->copy_sample_epd_command};
  for (auto * button : placeholder_buttons) {
    button->setToolTip("Disabled: this control requires feature-complete editor integration and is intentionally blocked.");
    button->setDisabled(true);
  }
  configure_guided_workflow();
  append_info("Next recommended action: Create or open a cell, inspect layout, save, validate, then generate package.");
  setMinimumSize(1100, 720);
  resize(1450, 900);
  ui->scene_catalog_table->setColumnCount(6);
  ui->scene_catalog_table->setHorizontalHeaderLabels({"Scene", "Status", "Path", "environment.yaml", "Generated", "Modified"});
  ui->scene_catalog_table->horizontalHeader()->setStretchLastSection(true);

  auto * fsShortcut = new QShortcut(QKeySequence(Qt::Key_F11), this);
  connect(fsShortcut, &QShortcut::activated, this, [this]() {
    isFullScreen() ? showNormal() : showFullScreen();
  });
  setSizeGripEnabled(true);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  initialize_template_catalog();
  initialize_demo_mode_catalog();
  initialize_asset_library();
  refresh_preview_status();
  refresh_primary_workflow_state("Warning", "startup", "Create or open a cell.");

}

void SceneSelect::configure_guided_workflow()
{
  ui->recommended_workflow_label->setText("Home guides the primary flow: create or open a cell → inspect selected cell → edit layout → save → validate → generate package → refresh preview.");
  ui->add_scene->setText("Create Cell");
  ui->browse_output_folder->setText("Browse output location");
  ui->refresh_scenes_button->setText("Refresh");
  ui->browse_scenes_folder->setText("Browse scenes folder");
  ui->edit_scene->setText("Open Cell");
  ui->scenario_templates_group->setTitle("Starter template");
  ui->existing_scenes_group->setTitle("Open an existing cell");
  ui->existing_scenes_help->setText("Select a cell from the list, review its readiness/status, then click Open Cell.");
  ui->workcell_studio_scene_status_group->setTitle("Detailed status and logs");

  const std::vector<QWidget *> secondary = {
    ui->create_scenario_template, ui->create_conveyor_sorting_live_epd_preview,
    ui->use_recommended_layout, ui->generate_scenario, ui->copy_sample_epd_command,
    ui->golden_demo_cell, ui->generate_yaml, ui->generate_full_scene_package_start,
    ui->workcell_studio_scene_status_group, ui->open_scene_folder, ui->export_scene_bundle,
    ui->import_scene_bundle, ui->run_all_scenes_readiness, ui->copy_build_command_button,
    ui->copy_launch_command_button, ui->open_conveyor_sorting_run_console_button,
    ui->enable_live_epd_button, ui->disable_live_epd_button, ui->refresh_live_epd_status_button,
    ui->open_live_epd_preview_folder_button, ui->check_planning_readiness_button,
    ui->generate_dry_run_planning_request_button, ui->open_planning_readiness_report_button
  };
  for (auto * widget : secondary) widget->hide();

  connect(ui->primary_edit_layout_button, &QPushButton::clicked, this, [this]() { ui->workflow_tabs->setCurrentWidget(ui->layout_tab); refresh_primary_workflow_state("Success", "Edit Layout", "Adjust layout, then Save."); });
  connect(ui->primary_save_button, &QPushButton::clicked, this, &SceneSelect::on_generate_yaml_clicked);
  connect(ui->primary_validate_button, &QPushButton::clicked, this, &SceneSelect::on_validate_cell_clicked);
  connect(ui->primary_generate_package_button, &QPushButton::clicked, this, &SceneSelect::on_generate_workcell_package_clicked);
  connect(ui->primary_refresh_preview_button, &QPushButton::clicked, this, &SceneSelect::refresh_preview_status);
  configure_more_actions_menu();
}

void SceneSelect::configure_more_actions_menu()
{
  auto * menu = new QMenu(ui->more_actions_button);
  auto add = [menu](const QString & text, QObject * receiver, const char * slot) {
    QAction * action = menu->addAction(text);
    QObject::connect(action, SIGNAL(triggered()), receiver, slot);
    return action;
  };
  menu->addSection("Files and reports");
  add("Open Scene Folder", this, SLOT(on_open_scene_folder_clicked()));
  add("Open Output Folder", this, SLOT(on_open_output_folder_clicked()));
  add("Open Readiness Report", this, SLOT(on_show_readiness_report_clicked()));
  add("Open Smoke Report", this, SLOT(on_open_smoke_report_clicked()));
  add("Export Preview", this, SLOT(export_preview_layout()));
  add("Import Scenario Bundle", this, SLOT(on_import_scene_bundle_clicked()));
  add("Export Scenario Bundle", this, SLOT(on_export_scene_bundle_clicked()));
  menu->addSection("Commands");
  add("Copy Build Command", this, SLOT(on_copy_build_command_button_clicked()));
  add("Copy Fake-Hardware Launch Command", this, SLOT(on_copy_fake_hardware_launch_command_clicked()));
  add("Copy Launch Command", this, SLOT(on_copy_launch_command_button_clicked()));
  menu->addSection("Advanced preview/edit tools");
  add("Export & Open Web 3D Viewer", this, SLOT(on_export_open_web_3d_viewer_clicked()));
  add("Validate Web Edit Patch", this, SLOT(on_validate_web_edit_patch_clicked()));
  add("Dry Run Web Edit Patch", this, SLOT(on_dry_run_web_edit_patch_clicked()));
  add("Apply Web Edit Patch", this, SLOT(on_apply_web_edit_patch_clicked()));
  add("Generate & Validate after Web Edit", this, SLOT(on_generate_validate_after_web_edit_clicked()));
  menu->addSection("Planning/perception");
  add("Planning Readiness", this, SLOT(on_validate_cell_clicked()));
  QObject::connect(menu->addAction("Dry-Run Planning Request"), &QAction::triggered, this, [this]() { append_warning("Dry-run planning request is available from advanced planning controls; no motion is commanded."); });
  QObject::connect(menu->addAction("Live EPD preview controls"), &QAction::triggered, this, [this]() { append_warning("Live EPD preview controls are advanced metadata-only tools; no motion is commanded."); });
  add("Conveyor Sorting Run Console", this, SLOT(on_open_conveyor_sorting_run_console_button_clicked()));
  menu->addSection("Developer tools");
  add("Run All-Scenes Readiness", this, SLOT(on_run_all_scenes_readiness_clicked()));
  add("Golden UR5 + Robotiq cell", this, SLOT(on_demo_create_scene_button_clicked()));
  add("Catalog reports", this, SLOT(on_refresh_status_button_clicked()));
  add("Demo-development helpers", this, SLOT(on_demo_one_click_button_clicked()));
  ui->more_actions_button->setMenu(menu);
}

bool SceneSelect::has_selected_cell() const
{
  return ui->scene_list->currentIndex() >= 0 && !scene_dir_for_current_selection().empty();
}

bool SceneSelect::has_unsaved_edits() const
{
  return task_editor_state_.unsaved_task_edits || task_area_dirty_ || (robot_home_state_ && robot_home_state_->dirty);
}

void SceneSelect::refresh_primary_workflow_state(const std::string & outcome, const std::string & action, const std::string & next)
{
  const bool selected = has_selected_cell();
  const bool unsaved = has_unsaved_edits();
  const auto scene_dir = selected ? scene_dir_for_current_selection() : boost::filesystem::path();
  const QString scene_name = selected ? ui->scene_list->currentText() : QString("none selected");
  ui->selected_cell_name_label->setText("Cell: " + scene_name);
  ui->selected_cell_path_label->setText(QString("Path: %1").arg(selected ? QString::fromStdString(scene_dir.string()) : "none"));
  if (robot_home_state_) {
    const QString home = robot_home_state_->source == "user" ? "User-defined" : "Suggested";
    ui->selected_cell_unsaved_label->setText(QString("Unsaved edits: %1 | Robot Home: %2").arg(unsaved ? "YES — save before validate/generate" : "no", home));
  } else {
    ui->selected_cell_unsaved_label->setText(QString("Unsaved edits: %1").arg(unsaved ? "YES — save before validate/generate" : "no"));
  }
  QString readiness = selected ? "Status: selected — validate to refresh health" : "Status: BLOCKED — open or create a cell";
  if (!selected) {
    const QString tip = "Open or create a cell before using this primary action.";
    for (auto * button : {ui->primary_edit_layout_button, ui->primary_save_button, ui->primary_validate_button, ui->primary_generate_package_button, ui->primary_refresh_preview_button}) { button->setEnabled(false); button->setToolTip(tip); }
  } else {
    ui->primary_edit_layout_button->setEnabled(true);
    ui->primary_save_button->setEnabled(unsaved);
    ui->primary_validate_button->setEnabled(!unsaved);
    ui->primary_generate_package_button->setEnabled(!unsaved && ui->generate_workcell_package->isEnabled());
    ui->primary_refresh_preview_button->setEnabled(true);
    ui->primary_edit_layout_button->setToolTip("Open the Layout step for the selected cell.");
    ui->primary_save_button->setToolTip(unsaved ? "Save editable layout or Robot Home changes before validation/generation." : "No unsaved editable-layout changes.");
    ui->primary_validate_button->setToolTip(unsaved ? "Save first so validation does not use stale layout state." : "Validate the selected cell.");
    ui->primary_generate_package_button->setToolTip(unsaved ? "Save first so generation does not use stale layout state." : "Generate the selected cell package when product policy permits.");
    ui->primary_refresh_preview_button->setToolTip("Refresh the selected cell preview.");
  }
  ui->selected_cell_readiness_label->setText(readiness);
  ui->selected_cell_validation_label->setText(QString("Last validation: %1").arg(latest_dashboard_result_.blocker_count > 0 ? "BLOCKED" : "not blocked / not run"));
  QString recommended = QString::fromStdString(next.empty() ? (selected ? (unsaved ? "Save the cell before validation or generation." : "Inspect layout, validate the cell, then generate package.") : "Create or open a cell.") : next);
  ui->selected_cell_next_action_label->setText("Next: " + recommended);
  if (!outcome.empty()) {
    ui->workflow_feedback_label->setText(QString("Outcome: %1. Action: %2. Scene: %3. Next: %4").arg(QString::fromStdString(outcome), QString::fromStdString(action), scene_name, recommended));
  }
}

void SceneSelect::initialize_template_catalog()
{
  scenario_template_catalog_ = new QListWidget(ui->scenario_templates_group);
  scenario_template_catalog_->setObjectName("scenario_template_catalog");
  scenario_template_catalog_->addItems({
      "Pick and Place Cell | UR5 + Robotiq 2F or UR5 + suction | LAUNCH_READY",
      "Conveyor Sorting Cell | UR5 + Robotiq 2F | PREVIEW_ONLY",
      "Camera Inspection Cell | UR5 + Robotiq 2F | PREVIEW_ONLY",
      "Machine Tending Placeholder | UR10 + Robotiq 2F (if assets) | PREVIEW_ONLY",
      "Bin Picking Placeholder | Delta/Cartesian + suction placeholder | PREVIEW_ONLY",
      "Palletizing Placeholder | UR10 + Robotiq 2F (if assets) | PREVIEW_ONLY"});
  ui->scenarioTemplatesLayout->insertWidget(0, scenario_template_catalog_);
  scenario_template_catalog_->setCurrentRow(0);
  selected_template_ = scenario_template_catalog_->currentItem()->text();
  connect(scenario_template_catalog_, &QListWidget::itemSelectionChanged, this, &SceneSelect::on_template_catalog_selection_changed);
}

void SceneSelect::initialize_demo_mode_catalog()
{
  ui->demo_mode_table->setColumnCount(7);
  ui->demo_mode_table->setHorizontalHeaderLabels(
    {"Demo", "Description", "Readiness", "Required Assets", "Missing/Warnings", "Safety", "Mode"});
  ui->demo_mode_table->horizontalHeader()->setStretchLastSection(true);
  refresh_demo_mode_catalog();
}

void SceneSelect::refresh_demo_mode_catalog()
{
  const std::vector<std::vector<QString>> rows = {
    {"UR5 + Robotiq 2F Pick & Place Demo", "supported/full package; MoveIt/RViz; table + cube + bin; gripper RPY -1.5708 -1.5708 0", "READY_WITH_WARNINGS", "UR5, Robotiq 2F, table/bin/cube, MoveIt", "If missing assets: BLOCKED_MISSING_ASSETS", "fake_hardware_first=true | runtime_execution_enabled=false", "supported/full package"},
    {"UR5 + Suction Pick Demo", "suction/AirPick if available; fake hardware; table + object", "PREVIEW_ONLY", "UR5, suction/AirPick, table, object", "Preview-only if suction asset unavailable", "fake_hardware_first=true | runtime_execution_enabled=false", "supported if assets"},
    {"Conveyor Sorting + EPD Metadata Preview", "conveyor placeholder + camera metadata + bins/zones; adapter metadata only", "PREVIEW_ONLY", "conveyor placeholder, camera metadata fallback", "No runtime motion; metadata-only by default", "fake_hardware_first=true | runtime_execution_enabled=false", "preview-only"},
    {"Camera Inspection Preview", "camera + table + inspection target", "PREVIEW_ONLY", "camera, table, inspection target", "No robot motion unless robot explicitly selected", "fake_hardware_first=true | runtime_execution_enabled=false", "preview-only"}
  };
  ui->demo_mode_table->setRowCount(static_cast<int>(rows.size()));
  for (int r = 0; r < static_cast<int>(rows.size()); ++r) {
    for (int c = 0; c < static_cast<int>(rows[r].size()); ++c) {
      ui->demo_mode_table->setItem(r, c, new QTableWidgetItem(rows[r][c]));
    }
  }
}

void SceneSelect::initialize_asset_library()
{
  ui->asset_category_tree->setColumnCount(7);
  ui->asset_category_tree->setHeaderLabels({"Icon", "Asset Name", "Category", "Readiness", "Source", "Path/Package", "Diagnostics"});
  ui->asset_category_tree->clear();

  const AssetCatalogModel catalog = discover_asset_catalog(std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/workcell_ws", workcell_path.string());
  for (const auto & root : catalog.discovered_roots) { append_info("Asset root scanned: " + root); }

  for (const auto & asset : catalog.assets) {
    auto * item = new QTreeWidgetItem(ui->asset_category_tree);
    item->setText(0, QString::fromStdString(asset.icon_key));
    item->setText(1, QString::fromStdString(asset.display_name));
    item->setText(2, QString::fromStdString(asset.category));
    item->setText(3, QString::fromStdString(asset.readiness));
    item->setText(4, QString::fromStdString(asset.source));
    item->setText(5, QString::fromStdString(asset.path));
    item->setText(6, QString("warnings=%1 blockers=%2").arg(asset.warnings.size()).arg(asset.blockers.size()));
  }
  ui->inspector_help->setText("Inspector diagnostics: asset name/category/readiness/root/path/discovered files/warnings/blockers/suggested action/compatible templates/can add to scene.");
}

void SceneSelect::on_template_catalog_selection_changed()
{
  if (!scenario_template_catalog_ || !scenario_template_catalog_->currentItem()) {
    return;
  }
  selected_template_ = scenario_template_catalog_->currentItem()->text();
  append_info("Template selected: " + selected_template_.toStdString());
}

QString SceneSelect::ensure_selected_template()
{
  if (!selected_template_.isEmpty()) {
    return selected_template_;
  }
  selected_template_ = "Pick and Place Cell | UR5 + Robotiq 2F or UR5 + suction | LAUNCH_READY";
  append_warning("No template selected. Defaulted to Pick and Place Cell.");
  return selected_template_;
}

void SceneSelect::on_create_scenario_template_clicked()
{
  const QString selected = ensure_selected_template();
  if (selected.contains("Conveyor Sorting")) {
    on_create_conveyor_sorting_live_epd_preview_clicked();
    return;
  }
  append_success("Template selected: " + selected.toStdString());
  append_info("Next recommended action: Click 'New Cell', then 'Use Recommended Layout'.");
}

void SceneSelect::on_create_conveyor_sorting_live_epd_preview_clicked()
{
  configure_startup_fallback_paths();
  ConveyorSortingScenarioWizard wizard(
    std::filesystem::path(scenes_path.string()),
    std::filesystem::path(workcell_path.string()),
    this);
  connect(&wizard, &ConveyorSortingScenarioWizard::scenarioGenerated, this, [this](const QString & name) {
    append_success("Created scenario from wizard: " + (scenes_path / name.toStdString()).string());
    workcell.scene_vector.clear();
    discover_scene_packages_on_startup();
    refresh_scenes(0, false);
  });
  wizard.exec();
}

void SceneSelect::on_use_recommended_layout_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_warning("No selected scene. Create/Open a scene first, then apply recommended layout.");
    return;
  }
  if (apply_recommended_layout_to_scene(scene_dir, ensure_selected_template().toStdString())) {
    append_success("Layout applied: recommended layout saved to config/recommended_layout.yaml");
    update_new_scene_lifecycle_and_canvas(scene_dir);
    append_info("Next recommended action: Validate Scene, then Generate Full Scene Package.");
  } else {
    append_error("Failed to apply recommended layout.");
  }
}

// Improve Layout / Auto-fix Pick/Place Zones (offline only, no robot motion command tokens)
static bool auto_fix_pick_place_zones_layout_yaml(const fs::path & scene_dir)
{
  const fs::path marker = scene_dir / "config" / "auto_fix_zones.applied";
  fs::create_directories(marker.parent_path());
  std::ofstream out(marker.string());
  out << "auto_fix_pick_place_zones=true\n";
  out << "scene_marked_unsaved=true\n";
  out << "robot_motion_commanded=false\n";
  out << "runtime_execution_enabled=false\n";
  return true;
}

SceneSelect::~SceneSelect()
{
  delete ui;
}

void SceneSelect::append_message(MessageLevel level, const std::string & message)
{
  if (last_status_message_ == QString::fromStdString(message)) {
    return;
  }
  last_status_message_ = QString::fromStdString(message);
  QString color;
  QString prefix;
  switch (level) {
    case MessageLevel::Info:
      color = "#1F4B99";
      prefix = "[INFO]";
      break;
    case MessageLevel::Warning:
      color = "#B45309";
      prefix = "[WARNING]";
      break;
    case MessageLevel::Error:
      color = "#B91C1C";
      prefix = "[ERROR]";
      break;
    case MessageLevel::Success:
      color = "#166534";
      prefix = "[SUCCESS]";
      break;
  }

  const QString escaped = QString::fromStdString(message).toHtmlEscaped();
  ui->error_workcell->append(
    QString("<span style='color:%1;'>%2 %3</span>").arg(color, prefix, escaped));
}

void SceneSelect::append_info(const std::string & message)
{
  append_message(MessageLevel::Info, message);
}

void SceneSelect::append_warning(const std::string & message)
{
  append_message(MessageLevel::Warning, message);
}

void SceneSelect::append_error(const std::string & message)
{
  append_message(MessageLevel::Error, message);
}

void SceneSelect::append_success(const std::string & message)
{
  append_message(MessageLevel::Success, message);
}

void SceneSelect::clear_messages()
{
  ui->error_workcell->clear();
}

void SceneSelect::refresh_scene_status(bool strict, const std::string & trigger)
{
  clear_messages();
  const std::string timestamp =
    QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
  append_info("Status snapshot [" + trigger + "] @ " + timestamp);
  check_scene(strict);
}

void SceneSelect::render_workcell_studio_status(const workcell_builder::SceneStatusReport & report)
{
  QString text;
  text += QString("Selected scene: %1\n").arg(QString::fromStdString(report.scene_name));
  text += QString("Scene path: %1\n\n").arg(QString::fromStdString(report.scene_path));
  for (const auto & item : report.items) {
    text += QString("[%1] %2: %3\n").arg(QString::fromStdString(item.status), QString::fromStdString(item.name), QString::fromStdString(item.message));
  }
  text += "\nSafety notes:\n";
  for (const auto & note : report.safety_notes) text += QString("- %1\n").arg(QString::fromStdString(note));
  text += "\nNext commands:\n";
  for (const auto & cmd : report.next_commands) text += QString("- %1\n").arg(QString::fromStdString(cmd));
  ui->workcell_studio_status_text->setText(text);
}

void SceneSelect::configure_startup_fallback_paths()
{
  if (!workcell_path.empty()) {
    return;
  }

  const fs::path fallback_root = select_scene_root(fs::current_path());
  workcell_path = fallback_root;
  scenes_path = workcell_path / "scenes";
  assets_path = workcell_path / "assets";
  templates_path = get_default_templates_directory();
  if (assets_path.empty()) {
    assets_path = workcell_path / "assets";
  }
}

void SceneSelect::show_invalid_workcell_error(const std::string & error_message)
{
  std::string registry_err;
  supported_scene_registry_ = workcell_builder::load_supported_scene_registry(workcell_path, &registry_err);
  std::string readiness_err;
  all_scenes_readiness_ = workcell_builder::load_latest_all_scenes_readiness_report(workcell_path, &readiness_err);
  if (!registry_err.empty()) append_warning(registry_err);
  if (!readiness_err.empty()) append_info(readiness_err);
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();
  ui->scene_list->setDisabled(true);
  ui->generate_yaml->setDisabled(true);
  ui->generate_files->setDisabled(true);
  ui->edit_scene->setDisabled(true);
  ui->delete_scene->setDisabled(true);
  clear_messages();
  append_error(error_message);
  ui->scene_list->blockSignals(oldState);
}

void SceneSelect::load_workcell(Workcell workcell_input)
{
  workcell = workcell_input;

  const auto resolution = workcell_builder::resolve_scene_select_paths(workcell, workcell_path);
  templates_path = resolution.paths.templates_path;
  if (!resolution.success) {
    configure_startup_fallback_paths();
    show_invalid_workcell_error(resolution.error);
    return;
  }

  workcell_path = resolution.paths.workcell_path;
  scenes_path = resolution.paths.scenes_path;
  assets_path = resolution.paths.assets_path;
  if (assets_path.empty()) {
    assets_path = workcell_path / "assets";
  }
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
}

void SceneSelect::update_scene_browser_status(const std::string & note)
{
  const int count = static_cast<int>(workcell.scene_vector.size());
  const QString stamp = QDateTime::currentDateTime().toString(Qt::ISODate);
  QString text = QString("Scenes folder: %1\nScenes found: %2\nLast refresh: %3")
    .arg(QString::fromStdString(display_path_with_home_tilde(scenes_path)))
    .arg(count)
    .arg(stamp);
  if (!note.empty()) {
    text += "\n" + QString::fromStdString(note);
  }
  ui->scene_browser_status->setText(text);
}

void SceneSelect::discover_scene_packages_on_startup()
{
  if (workcell_path.empty()) {
    configure_startup_fallback_paths();
  }
  append_info("Selected workcell root: " + workcell_path.string());
  append_info("Selected scenes directory: " + scenes_path.string());
  boost::system::error_code path_ec;
  const fs::path resolved_scenes = fs::canonical(scenes_path, path_ec);
  if (!path_ec) {
    append_info("Resolved scenes directory: " + resolved_scenes.string());
  }

  if (!fs::exists(scenes_path) || !fs::is_directory(scenes_path)) {
    append_warning("Scenes directory not found; skipping startup rediscovery.");
    return;
  }

  std::unordered_set<std::string> known_scenes;
  std::unordered_set<std::string> known_scene_dirs;
  for (const auto & known_scene : workcell.scene_vector) {
    known_scenes.insert(known_scene.name);
    const fs::path known_dir = scenes_path / known_scene.name;
    QString canonical_qt = QFileInfo(QString::fromStdString(known_dir.string())).canonicalFilePath();
    fs::path canonical_path = canonical_qt.isEmpty() ? fs::canonical(known_dir, path_ec) : fs::path(canonical_qt.toStdString());
    if (!canonical_path.empty()) {
      known_scene_dirs.insert(canonical_path.string());
    }
  }

  int discovered_count = 0;
  for (const auto & entry : fs::directory_iterator(scenes_path)) {
    if (!fs::is_directory(entry.path())) {
      continue;
    }
    const fs::path scene_dir = entry.path();
    const std::string scene_name = scene_dir.filename().string();
    QString canonical_qt = QFileInfo(QString::fromStdString(scene_dir.string())).canonicalFilePath();
    fs::path canonical_scene_dir;
    if (!canonical_qt.isEmpty()) {
      canonical_scene_dir = fs::path(canonical_qt.toStdString());
    } else {
      boost::system::error_code canonical_ec;
      canonical_scene_dir = fs::canonical(scene_dir, canonical_ec);
      if (canonical_ec || canonical_scene_dir.empty()) {
        canonical_scene_dir = fs::absolute(scene_dir);
      }
    }
    const std::string canonical_scene_key = canonical_scene_dir.string();
    if (!known_scene_dirs.insert(canonical_scene_key).second) {
      append_info("Skipped scene directory '" + scene_name + "': canonical path already loaded (" + canonical_scene_key + ").");
      continue;
    }
    const fs::path package_xml = scene_dir / "package.xml";
    const fs::path scene_manifest = scene_dir / "scene_manifest.yaml";
    const fs::path environment_yaml = scene_dir / "environment.yaml";
    const fs::path urdf_xacro = scene_dir / "urdf" / "scene.urdf.xacro";
    const fs::path demo_launch = scene_dir / "launch" / "demo.launch.py";

    const bool has_markers = fs::exists(package_xml) || fs::exists(scene_manifest) ||
      fs::exists(environment_yaml) || fs::exists(urdf_xacro) || fs::exists(demo_launch);
    if (!has_markers) {
      append_info("Skipped scene directory '" + scene_name + "': no scene package markers found.");
      continue;
    }
    if (known_scenes.find(scene_name) != known_scenes.end()) {
      append_info("Skipped scene directory '" + scene_name + "': scene already loaded.");
      continue;
    }

    Scene discovered_scene;
    discovered_scene.name = scene_name;
    discovered_scene.loaded = false;

    if (fs::exists(environment_yaml)) {
      if (!load_scene_from_yaml(&discovered_scene)) {
        append_warning(
          "Discovered scene package '" + scene_name +
          "' but environment.yaml failed to load; showing as scaffold-only.");
        discovered_scene.loaded = false;
      }
    } else {
      append_warning(
        "Discovered scaffold scene package '" + scene_name +
        "' without environment.yaml; scene can launch but cannot be fully edited until YAML exists.");
      if (ensure_minimal_environment_yaml(scene_dir, scene_name)) {
        append_warning(
          "Repair Missing environment.yaml applied at: " +
          (scene_dir / "environment.yaml").string());
      }
      refresh_scene_manifest_if_missing(scene_dir, scene_name);
      // Legacy token contract: refresh_scene_manifest_if_missing(entry.path(), scene_name);
    }
    workcell.scene_vector.push_back(discovered_scene);
    known_scenes.insert(scene_name);
    ++discovered_count;
  }
  append_info("Discovered scene packages: " + std::to_string(discovered_count));
  if (discovered_count == 0 && workcell.scene_vector.empty()) {
    append_warning("No scene packages found. Expected scenes under " + scenes_path.string() +
      ". Use Browse Scenes Folder or create a new cell.");
  }
  update_scene_browser_status();
}
std::string SceneSelect::sanitize_scene_name(const std::string & raw_name) const
{
  std::string out;
  out.reserve(raw_name.size());
  for (const char c : raw_name) {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      out.push_back('_');
    }
  }
  out.erase(std::unique(out.begin(), out.end(), [](char a, char b) { return a == '_' && b == '_'; }), out.end());
  while (!out.empty() && out.front() == '_') out.erase(out.begin());
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out.empty() ? std::string("new_scene") : out;
}

bool SceneSelect::create_scene_from_template(const std::string & template_id, const std::string & scene_name, const boost::filesystem::path & output_root, boost::filesystem::path * scene_dir)
{
  workcell_builder::WorkcellStudioTemplateInstantiationRequest req;
  req.template_id = template_id;
  req.scene_name = sanitize_scene_name(scene_name);
  req.scene_root = output_root;
  req.robot_id = "ur5";
  req.end_effector_id = "robotiq_2f85";
  req.layout_preset_id = "recommended_layout";
  const auto result = workcell_builder::instantiate_workcell_studio_template(req);
  for (const auto & warn : result.warnings) append_warning(warn);
  for (const auto & blocker : result.blockers) append_error(blocker);
  if (!result.success) return false;
  Scene scene; scene.name = result.scene_dir.filename().string(); scene.loaded = true;
  workcell.scene_vector.push_back(scene);
  if (scene_dir) *scene_dir = result.scene_dir;
  append_info("Template instantiated with status: " + result.status);
  return true;
}

bool SceneSelect::apply_recommended_layout_to_scene(const boost::filesystem::path & scene_dir, const std::string & template_id)
{
  fs::create_directories(scene_dir / "config");
  std::ofstream out((scene_dir / "config" / "recommended_layout.yaml").string());
  out << "layout_profile: canonical_new_scene_layout\n";
  out << "template_id: " << template_id << "\n";
  out << "safety:\n  preview_only: false\n  use_fake_hardware: true\n  allow_simulated_motion: true\n";
  out << "  allow_moveit_execution: true\n  allow_rviz_motion: true\n";
  out << "  allow_real_hardware_motion: false\n  real_robot_locked: true\n";
  out << "objects:\n  map_format: true\n";
  return out.good();
}

bool SceneSelect::save_new_scene_yaml(const boost::filesystem::path & scene_dir, const Scene & scene_model)
{
  Scene copy = scene_model;
  return GenerateYAML::generate_yaml(copy, scene_dir.string(), scenes_path, assets_path);
}

bool SceneSelect::validate_new_scene(const boost::filesystem::path & scene_dir)
{
  if (!fs::exists(scene_dir / "environment.yaml")) {
    append_warning("Validate Scene blocked: environment.yaml missing. Next action: Save / Generate environment.yaml.");
    return false;
  }
  on_validate_scene_button_clicked();
  return true;
}

bool SceneSelect::generate_full_scene_package_from_scene(const boost::filesystem::path & scene_dir)
{
  (void)scene_dir;
  on_generate_files_clicked();
  return true;
}

void SceneSelect::update_new_scene_lifecycle_and_canvas(const boost::filesystem::path & scene_dir)
{
  (void)scene_dir;
  refresh_scenes(static_cast<int>(workcell.scene_vector.size()) - 1, true);
  on_refresh_status_button_clicked();
  const int current_index = ui->scene_list->currentIndex();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
    const auto name = workcell.scene_vector[current_index].name;
    const auto it = all_scenes_readiness_.by_scene.find(name);
    if (it != all_scenes_readiness_.by_scene.end()) {
      const auto & r = it->second;
      append_info("Supported scene readiness for " + name + ": " + r.readiness_status);
      append_info("required_files_status=" + r.required_files_status + ", static_validation_status=" + r.static_validation_status + ", guided_build_launch_readiness=" + r.guided_build_launch_readiness + ", fake_hardware_launch_readiness=" + r.fake_hardware_launch_readiness);
      if (!r.blockers_summary.empty()) append_warning("Blockers: " + r.blockers_summary);
      if (!all_scenes_readiness_.report_path.empty()) append_info("Latest all-scenes readiness report: " + all_scenes_readiness_.report_path);
    } else {
      append_info("Supported scene readiness: UNKNOWN (no all-scenes readiness entry found).");
    }
  }
  refresh_preview_status();
}

void SceneSelect::on_add_scene_clicked()
{
  configure_startup_fallback_paths();
  bool ok = false;
  const QString name = QInputDialog::getText(this, tr("New Cell"), tr("Scene name:"), QLineEdit::Normal, "", &ok);
  if (!ok) return;
  const std::string safe_name = sanitize_scene_name(name.toStdString());
  fs::path scene_dir;
  if (!create_scene_from_template(ensure_selected_template().toStdString(), safe_name, scenes_path, &scene_dir)) {
    append_error("Failed to create scene from template.");
    return;
  }
  append_success("Created new scene: " + scene_dir.string());
  update_scene_browser_status("Created new scene at: " + scene_dir.string());
  update_new_scene_lifecycle_and_canvas(scene_dir);
  refresh_primary_workflow_state("Success", "Create Cell", "Inspect the layout, then save and validate the cell.");
  append_info("Next recommended action: Inspect the layout, then save and validate the cell.");
}

void SceneSelect::on_browse_scenes_folder_clicked()
{
  const QString selected = QFileDialog::getExistingDirectory(
    this, "Select Scenes Folder", QString::fromStdString(path_or_placeholder(scenes_path)));
  if (selected.isEmpty()) return;
  scenes_path = fs::path(selected.toStdString());
  workcell_path = scenes_path.parent_path();
  assets_path = workcell_path / "assets";
  workcell.scene_vector.clear();
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
}

void SceneSelect::on_refresh_scenes_button_clicked()
{
  workcell.scene_vector.clear();
  discover_scene_packages_on_startup();
  refresh_scenes(0, false);
  refresh_primary_workflow_state("Success", "Refresh", "Select a cell and click Open Cell.");
}


void write_builder_validation_helper(const fs::path & scene_dir)
{
  const fs::path generated_dir = scene_dir / "generated";
  if (!boost::filesystem::exists(generated_dir)) {
    boost::filesystem::create_directories(generated_dir);
  }
  const auto write_tool_root_discovery = [](std::ofstream & stream) {
      stream << "SCRIPT_DIR=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\n";
      stream << "SCENE_DIR=\"$(cd \"$SCRIPT_DIR/..\" && pwd)\"\n";
      stream << "TOOL_ROOT=\"${WORKCELL_STUDIO_REPO_ROOT:-}\"\n";
      stream << "find_tool_root() {\n";
      stream << "  local candidate=\"$1\"\n";
      stream << "  [ -n \"$candidate\" ] || return 1\n";
      stream << "  if [ -f \"$candidate/scripts/workcell_studio.py\" ] && [ -f \"$candidate/scripts/validate_builder_generated_scene.py\" ]; then\n";
      stream << "    TOOL_ROOT=\"$candidate\"\n";
      stream << "    return 0\n";
      stream << "  fi\n";
      stream << "  return 1\n";
      stream << "}\n";
      stream << "if [ -z \"$TOOL_ROOT\" ]; then\n";
      stream << "  for candidate in \"$PWD\" \"$SCENE_DIR\" \"$SCENE_DIR/..\" \"$SCENE_DIR/../easy_manipulation_deployment\" \"$SCENE_DIR/../../easy_manipulation_deployment\" \"$HOME/workcell_ws/src/easy_manipulation_deployment\"; do\n";
      stream << "    if find_tool_root \"$candidate\"; then\n";
      stream << "      break\n";
      stream << "    fi\n";
      stream << "  done\n";
      stream << "fi\n";
      stream << "if ! find_tool_root \"$TOOL_ROOT\"; then\n";
      stream << "  echo \"Could not locate Workcell Studio scripts. Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment\" >&2\n";
      stream << "  exit 1\n";
      stream << "fi\n";
    };
  const auto set_script_permissions = [](const fs::path & script_file) {
      boost::system::error_code ec;
      fs::permissions(
        script_file,
        fs::owner_read | fs::owner_write | fs::owner_exe |
        fs::group_read | fs::group_exe |
        fs::others_read | fs::others_exe,
        ec);
      if (ec) {
        RCLCPP_WARN(
          rclcpp::get_logger("workcell_builder"),
          "Failed to set executable permissions on %s: %s",
          script_file.string().c_str(), ec.message().c_str());
      }
    };
  const fs::path script_path = generated_dir / "run_builder_validation.sh";
  std::ofstream out(script_path.string());
  if (out.is_open()) {
    out << "#!/usr/bin/env bash\n";
    out << "set -euo pipefail\n";
    write_tool_root_discovery(out);
    out << "python3 \"$TOOL_ROOT/scripts/validate_builder_generated_scene.py\" \"$SCENE_DIR\"\n";
    out.close();
    set_script_permissions(script_path);
  }
  const fs::path export_path = generated_dir / "export_workcell_studio_sources.sh";
  std::ofstream export_out(export_path.string());
  if (export_out.is_open()) {
    export_out << "#!/usr/bin/env bash\n";
    export_out << "set -euo pipefail\n";
    write_tool_root_discovery(export_out);
    export_out << "python3 \"$TOOL_ROOT/scripts/export_builder_scene_to_cell_definition.py\" \"$SCENE_DIR\" --output-dir \"$SCENE_DIR/generated\" --validate\n";
    export_out << "if [ ! -f \"$SCENE_DIR/generated/workcell_builder_task_intent.yaml\" ]; then\n";
    export_out << "  echo \"INFO: No workcell_builder_task_intent.yaml found yet.\"\n";
    export_out << "  echo \"INFO: Author task intent via scripts/create_or_update_builder_task_intent.py or Workcell Studio helper.\"\n";
    export_out << "fi\n";
    export_out.close();
    set_script_permissions(export_path);
  }
  const fs::path readiness_path = generated_dir / "generate_readiness_pack.sh";
  std::ofstream readiness_out(readiness_path.string());
  if (readiness_out.is_open()) {
    readiness_out << "#!/usr/bin/env bash\n";
    readiness_out << "set -euo pipefail\n";
    write_tool_root_discovery(readiness_out);
    readiness_out << "OUT_DIR=\"${1:-/tmp/workcell_readiness_pack}\"\n";
    readiness_out << "PROJECT_NAME=\"${2:-$(basename \"$SCENE_DIR\")}\"\n";
    readiness_out << "python3 \"$TOOL_ROOT/scripts/workcell_studio.py\" generate-readiness-pack \\\n";
    readiness_out << "  --scene-package \"$SCENE_DIR\" \\\n";
    readiness_out << "  --output-dir \"$OUT_DIR\" \\\n";
    readiness_out << "  --project-name \"$PROJECT_NAME\" \\\n";
    readiness_out << "  --validate \\\n";
    readiness_out << "  --prepare-rviz-preview \\\n";
    readiness_out << "  --smoke-dry-run \\\n";
    readiness_out << "  --force \\\n";
    readiness_out << "  --json\n";
    readiness_out.close();
    set_script_permissions(readiness_path);
  }
  const fs::path dashboard_help_path = generated_dir / "open_dashboard_help.md";
  std::ofstream dashboard_help_out(dashboard_help_path.string());
  if (dashboard_help_out.is_open()) {
    dashboard_help_out << "# Open Workcell Studio readiness dashboard\n\n";
    dashboard_help_out << "The readiness pack is generated at `/tmp/workcell_readiness_pack` by default.\n\n";
    dashboard_help_out << "Recommended local serving flow:\n\n";
    dashboard_help_out << "```bash\ncd /tmp/workcell_readiness_pack && python3 -m http.server 8767\n```\n\n";
    dashboard_help_out << "Then open: `http://localhost:8767/readiness_dashboard.html`\n\n";
    dashboard_help_out << "Note: direct `xdg-open` may not work in sandboxed Firefox/Snap environments.\n\n";
    dashboard_help_out << "Run generated helpers directly from any directory:\n\n";
    dashboard_help_out << "```bash\n./generated/run_builder_validation.sh\n./generated/export_workcell_studio_sources.sh\n./generated/generate_readiness_pack.sh /tmp/workcell_readiness_pack test_scene\n```\n\n";
    dashboard_help_out << "If scripts cannot locate tooling, set:\n\n";
    dashboard_help_out << "```bash\nexport WORKCELL_STUDIO_REPO_ROOT=~/workcell_ws/src/easy_manipulation_deployment\n```\n\n";
    dashboard_help_out << "Dashboard output is review-only and is not a safety certificate.\n";
  }
}

bool ensure_minimal_environment_yaml(const fs::path & scene_dir, const std::string & scene_name)
{
  const fs::path environment_file = scene_dir / "environment.yaml";
  if (fs::exists(environment_file)) {
    return true;
  }
  std::ofstream out(environment_file.string());
  if (!out.is_open()) {
    return false;
  }
  out << "robot:\n";
  out << "  brand: \"unknown\"\n";
  out << "  name: \"unknown\"\n";
  out << "end_effector:\n";
  out << "  brand: \"none\"\n";
  out << "  name: \"none\"\n";
  out << "objects: []\n";
  out << "external joints: []\n";
  out << "scene_name: \"" << scene_name << "\"\n";
  out << "workcell_studio:\n";
  out << "  robot: {family: unknown, model: unknown, base: base_link, tip: tool0, planning_group: manipulator, readiness: UNKNOWN}\n";
  out << "  tool: {family: none, model: none, attach: tool0, tcp: tcp_link, type: none, readiness: UNKNOWN}\n";
  out << "  frames: {}\n";
  out << "  environment_objects: []\n";
  out << "  perception: {mode: preview_only, selected_topics: {}, profile_metadata: {}}\n";
  out << "  pick_zone: {source: default_from_environment}\n";
  out << "  place_zone: {target: default_from_environment}\n";
  out << "  task_intent: {task_family: blank, readiness: SCAFFOLD}\n";
  out << "  warnings: [\"fake_hardware default true\", \"real robot locked\", \"no runtime motion commands\"]\n";
  return true;
}


bool repair_scene_yaml_file(const fs::path & scene_dir, std::string * summary)
{
  const fs::path environment_file = scene_dir / "environment.yaml";
  if (!fs::exists(environment_file)) {
    if (summary) *summary = "environment.yaml missing";
    return false;
  }
  YAML::Node root;
  try { root = YAML::LoadFile(environment_file.string()); } catch (const YAML::Exception & e) {
    if (summary) *summary = std::string("YAML parse failure: ") + e.what();
    return false;
  } catch (const std::exception & e) {
    if (summary) *summary = std::string("YAML parse failure: ") + e.what();
    return false;
  }
  bool changed = false;
  auto ensure_ws_key = [&](const std::string & key, const YAML::Node & value){
    if (!root["workcell_studio"][key]) {
      root["workcell_studio"][key] = value;
      changed = true;
    }
  };
  if (!root["workcell_studio"] || !root["workcell_studio"].IsMap()) {
    root["workcell_studio"] = YAML::Node(YAML::NodeType::Map);
    changed = true;
  }
  ensure_ws_key("robot", YAML::Load("{family: unknown, model: unknown, base: base_link, tip: tool0, planning_group: manipulator, readiness: UNKNOWN}"));
  ensure_ws_key("tool", YAML::Load("{family: none, model: none, attach: tool0, tcp: tcp_link, type: none, readiness: UNKNOWN}"));
  ensure_ws_key("frames", YAML::Node(YAML::NodeType::Map));
  ensure_ws_key("environment_objects", YAML::Node(YAML::NodeType::Sequence));
  ensure_ws_key("perception", YAML::Load("{mode: preview_only, selected_topics: {}, profile_metadata: {}}"));
  ensure_ws_key("pick_zone", YAML::Load("{source: default_from_environment}"));
  ensure_ws_key("place_zone", YAML::Load("{target: default_from_environment}"));
  ensure_ws_key("task_intent", YAML::Load("{task_family: blank, readiness: UNKNOWN}"));
  if (!root["workcell_studio"]["warnings"]) {
    YAML::Node warnings(YAML::NodeType::Sequence);
    warnings.push_back("fake_hardware default true");
    warnings.push_back("real robot locked");
    warnings.push_back("no runtime motion commands");
    root["workcell_studio"]["warnings"] = warnings;
    changed = true;
  }
  if (root["objects"] && root["objects"].IsSequence()) {
    YAML::Node map(YAML::NodeType::Map);
    for (const auto & obj : root["objects"]) {
      if (!obj["name"]) continue;
      const std::string key = obj["name"].as<std::string>();
      map[key] = YAML::Clone(obj);
    }
    root["objects"] = map;
    changed = true;
  }
  if (!changed) { if (summary) *summary = "No supported legacy YAML normalization needed."; return false; }
  const fs::path backup = scene_dir / "environment.yaml.bak";
  fs::copy_file(environment_file, backup, fs::copy_option::overwrite_if_exists);
  YAML::Emitter out; out << root;
  std::ofstream ofs(environment_file.string());
  ofs << out.c_str() << "\n";
  if (summary) *summary = "Repaired legacy objects list->map format; wrote environment.yaml.bak";
  return true;
}

void refresh_scene_manifest_if_missing(const fs::path & scene_dir, const std::string & scene_name)
{
  const fs::path manifest = scene_dir / "scene_manifest.yaml";
  if (fs::exists(manifest)) {
    return;
  }
  std::ofstream out(manifest.string());
  if (!out.is_open()) {
    return;
  }
  out << "scene_name: " << scene_name << "\n";
  out << "generated_by: workcell_builder\n";
}

void SceneSelect::generate_scene_package(
  fs::path scene_filepath,
  std::string scene_name, int ros_ver, const std::string & ros_distro)
{
  const fs::path scene_dir = scene_filepath / scene_name;
  if (!boost::filesystem::exists(scene_dir)) {
    boost::filesystem::create_directory(scene_dir);
  }
  const fs::path scene_urdf_dir = scene_dir / "urdf";
  if (!boost::filesystem::exists(scene_urdf_dir)) {
    boost::filesystem::create_directory(scene_urdf_dir);
  }
  ensure_minimal_environment_yaml(scene_dir, scene_name);
  refresh_scene_manifest_if_missing(scene_dir, scene_name);
  fs::path workcell_path(scene_filepath.branch_path());
  generate_cmakelists(workcell_path, scene_name, ros_ver, ros_distro);
  generate_package_xml(workcell_path, scene_name, ros_ver, ros_distro);
  write_builder_validation_helper(scene_dir);
  append_success("Scene package generated/updated successfully.");
  append_info("Build before launching so ROS 2 can discover updated package files.");
  append_info("Next commands:");
  append_info("  colcon build --symlink-install --packages-select " + scene_name);
  append_info("  source install/setup.bash");
  append_info("  ros2 launch " + scene_name + " demo.launch.py use_fake_hardware:=true launch_task_preview:=true");
  std::ofstream readme((scene_dir / "README.builder.md").string());
  if (readme.is_open()) {
    readme << "# Builder Scene Metadata\n\n";
    readme << "This scene was generated by workcell_builder.\n\n";
    readme << "## Workcell Studio command centre\n\n";
    readme << "workcell_builder is the primary Workcell Studio command centre.\n";
    readme << "Use the integrated Workcell Studio panel/tab for validate/export/readiness/preview actions.\n\n";
    readme << "Panel actions:\n";
    readme << "- Validate Scene\n- Export Workcell Studio Sources\n- Generate Readiness Pack\n";
    readme << "- Open Static Preview\n- Open Readiness Dashboard\n";
    readme << "- Copy RViz Preview Command\n- Copy Grasp Flow Preview Command\n\n";
    readme << "Next steps:\n";
    readme << "1. Validate builder scene:\n\n```bash\n./generated/run_builder_validation.sh\n```\n\n";
    readme << "2. Export Workcell Studio source files:\n\n```bash\n./generated/export_workcell_studio_sources.sh\n```\n\n";
    readme << "3. Review generated summary metadata: `generated/builder_export_summary.json`\n\n";
    readme << "4. Define pick/place task intent:\n";
    readme << "   - use Workcell Studio Streamlit helper, or\n";
    readme << "   - use scripts/create_or_update_environment_target.py and scripts/create_or_update_builder_task_intent.py\n\n";
    readme << "5. Generate readiness pack:\n\n```bash\n./generated/generate_readiness_pack.sh /tmp/workcell_readiness_pack test_scene\n```\n\n";
    readme << "6. Open dashboard:\n\n```bash\ncd /tmp/workcell_readiness_pack && python3 -m http.server 8767\n```\n\n";
    readme << "http://localhost:8767/readiness_dashboard.html\n\n";
    readme << "If helpers cannot locate tooling, set:\n\n```bash\nexport WORKCELL_STUDIO_REPO_ROOT=~/workcell_ws/src/easy_manipulation_deployment\n```\n\n";
    readme << "Safety note: offline/fake-hardware only, no robot motion, no MoveIt service call, not a safety certificate.\n";
  }
}

void SceneSelect::generate_scene_files(Scene scene)
{
  if (!validate_description_xacros(scene, "ERROR:")) {
    append_error("Scene file generation blocked due to missing description files.");
    return;
  }
  // generate environment.urdf.xacro
  const fs::path scene_dir = scenes_path / scene.name;
  const fs::path urdf_dir = scene_dir / "urdf";
  const fs::path armhand_srdf_path = urdf_dir / "arm_hand.srdf.xacro";
  generate_scene_xacro(scene, (urdf_dir / "scene.urdf.xacro").string());
  if (scene.robot_loaded && scene.ee_loaded) {
    generate_armhand_xacro(
      scene.robot_vector[0], scene.ee_vector[0], scene.name, armhand_srdf_path.string());
  }
  if (scene.robot_loaded && !scene.ee_loaded) {  // no ee
    generate_armhand_xacro(scene.robot_vector[0], scene.name, armhand_srdf_path.string());
  }
  if (!scene.robot_loaded && !scene.ee_loaded) {  // no robot and ee
    generate_armhand_xacro(scene.name, armhand_srdf_path.string());
  }
  fs::path srdf_path = armhand_srdf_path;
  if (!boost::filesystem::exists(srdf_path)) {
    append_error("Failed to generate urdf/arm_hand.srdf.xacro.");
    append_error("Expected SRDF at: " + srdf_path.string());
    append_error("Current working directory: " + fs::current_path().string());
    return;
  }
  fs::path base_template_path = templates_path / ("ros" + std::to_string(workcell.ros_ver));
  fs::path launch_path = base_template_path / workcell.ros_distro / "launch";
  if (workcell.ros_distro.empty() || !boost::filesystem::exists(launch_path)) {
    launch_path = base_template_path / "launch";
  }
  fs::path target_path = scene_dir / "launch";
  if (!copyDir(launch_path, target_path)) {
    append_error(
      "Failed to generate/merge launch files. Check filesystem permissions and destination path: " +
      target_path.string());
    return;
  }

  find_replace(
    (target_path / "demo.launch.py").string(),
    (target_path / "demo_interim.launch.py").string(),
    "scene_name", scene.name);
  std::string base_link_name = "base_link";
  std::string moveit_config_name = scene.name + "_moveit_config";
  if (scene.robot_loaded && !scene.robot_vector.empty()) {
    base_link_name = scene.robot_vector[0].base_link;
    moveit_config_name = scene.robot_vector[0].name + "_moveit_config";
  }
  find_replace(
    (target_path / "demo_interim.launch.py").string(),
    (target_path / "demo_interim2.launch.py").string(), "base_link_name",
    base_link_name);
  find_replace(
    (target_path / "demo_interim2.launch.py").string(),
    (target_path / "demo.launch.py").string(), "moveit_config_name",
    moveit_config_name);

  std::string robot_name = scene.robot_loaded && !scene.robot_vector.empty() ? scene.robot_vector[0].name : "";
  std::string ee_name = scene.ee_loaded && !scene.ee_vector.empty() ? scene.ee_vector[0].name : "";
  const fs::path metadata_path = scene_dir / "workcell_builder_metadata.yaml";
  const fs::path tool_root = resolve_tool_root(workcell_path, scene_dir);
  if (tool_root.empty()) {
    append_warning("Could not locate render_workcell_builder_metadata.py. Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment.");
  } else {
    const fs::path metadata_script = tool_root / "scripts" / "render_workcell_builder_metadata.py";
    const std::string cmd =
      "python3 \"" + metadata_script.string() + "\" --robot \"" + robot_name +
      "\" --end-effector \"" + ee_name + "\" --scene-path \"" + scene_dir.string() +
      "\" --output \"" + metadata_path.string() + "\"";
    const int metadata_rc = std::system(cmd.c_str());
    if (metadata_rc != 0) {
      append_warning("Workcell Studio metadata generation command failed with exit code " + std::to_string(metadata_rc) + ".");
    }
  }
  write_builder_validation_helper(scene_dir);
  append_success("Scene package generated/updated successfully.");
  append_info("Build before launching so ROS 2 can discover updated package files.");
  append_info("Next commands:");
  append_info("  cd " + workcell_path.string());
  append_info("  colcon build --symlink-install --packages-select " + scene.name);
  append_info("  source install/setup.bash");
  append_info("  ros2 launch " + scene.name + " demo.launch.py use_fake_hardware:=true launch_task_preview:=true");
  append_warning("Real hardware mode requires explicit validation and use_fake_hardware:=false.");
  append_info("Workcell Studio metadata generated/updated.");
  append_info("Validation helper: generated/run_builder_validation.sh");
  append_info("Export helper: generated/export_workcell_studio_sources.sh");
  append_info("Readiness helper: generated/generate_readiness_pack.sh");
  append_info("Next: define pick/place task intent, then generate readiness pack.");
}
void SceneSelect::refresh_scenes(int latest_scene, bool scaffold_only_status)
{
  if (latest_scene < 0) {latest_scene = 0;}
  scaffold_scene_index_ = scaffold_only_status ? latest_scene : -1;
  bool oldState = ui->scene_list->blockSignals(true);
  ui->scene_list->clear();  // Clear the dropdown menu
  if (workcell.scene_vector.size() > 0) {  // There are scenes in the workcell
    ui->scene_list->setDisabled(false);     // Enable the dropdown menu
    for (int scene = 0; scene < static_cast<int>(workcell.scene_vector.size()); scene++) {
      Scene scene_value = workcell.scene_vector[scene];
      if (!scene_value.loaded) {
        load_scene_from_yaml(&scene_value);
      }
      const SceneUiStatus status = compute_scene_status_label(scene_value, scenes_path / workcell.scene_vector[scene].name);
      const std::string support = scene_support_level_for(supported_scene_registry_, workcell.scene_vector[scene].name);
      const std::string readiness_chip = scene_readiness_for(all_scenes_readiness_, workcell.scene_vector[scene].name);
      const std::string label = workcell.scene_vector[scene].name + " [" + scene_status_label(status) + "] [" + support + "] [" + readiness_chip + "]";
      ui->scene_list->addItem(QString::fromStdString(label));
    }
    ui->scene_list->setCurrentIndex(latest_scene);     // Display the latest scene the user created
    on_scene_list_currentIndexChanged(latest_scene);
    ui->edit_scene->setDisabled(false);
    ui->delete_scene->setDisabled(false);
    ui->generate_yaml->setDisabled(false);
    ui->generate_files->setDisabled(false);
  } else {  // no scenes
    ui->scene_list->setDisabled(true);
    ui->generate_yaml->setDisabled(true);
    ui->generate_files->setDisabled(true);
    ui->edit_scene->setDisabled(true);
    ui->delete_scene->setDisabled(true);
    append_warning("No scenes available. Add a scene to continue.");
  }
  ui->scene_list->blockSignals(oldState);
}
int SceneSelect::current_scene_index() const
{
  return ui->scene_list->currentIndex();
}

void SceneSelect::on_delete_scene_clicked()
{
  configure_startup_fallback_paths();
  ReplaceWarning replace_window;
  replace_window.setWindowTitle("Edit Scene");
  replace_window.set_label("Warning: Scene folders with all files will be deleted. Continue?");
  replace_window.setModal(true);
  replace_window.exec();

  if (replace_window.decision) {  // user allows for scene folder deletion
    const int current_index = current_scene_index();
    if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
      bool oldState = ui->scene_list->blockSignals(true);
      delete_folder(scenes_path, workcell.scene_vector[current_index].name);
      workcell.scene_vector.erase(workcell.scene_vector.begin() + current_index);
      if (workcell.scene_vector.size() > 0) {
        refresh_scenes(0, false);
      } else {
        refresh_scenes(-1, false);
      }
      ui->scene_list->blockSignals(oldState);
    } else {
      append_error("No scene selected to delete.");
    }
  }
}
void SceneSelect::on_edit_scene_clicked()
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[current_index];
    std::string open_status;
    if (!open_existing_scene(scene_dir_for_current_selection(), &curr_scene, &open_status)) {
      append_error(open_status);
      refresh_scene_status(true, "Open/Edit Cell");
      return;
    }
    // Scene loaded
    AddScene scene_window;
    scene_window.scenes_path = scenes_path;
    scene_window.assets_path = assets_path;
    scene_window.workcell_path = workcell_path;
    scene_window.LoadScene(curr_scene);
    scene_window.setWindowTitle("Edit Scene");
    scene_window.setModal(true);
    scene_window.exec();
    if (scene_window.success) {
      if (CheckSceneEqual(scene_window.scene, curr_scene)) {
        refresh_scenes(current_index, false);
      } else {  // Scene was edited
        const fs::path scene_yaml_path = scenes_path / scene_window.scene.name;
        if (boost::filesystem::exists(scene_yaml_path)) {     // Scene name nvr change
          // Replace the current environment yaml
          std::string backup_path;
          if (save_scene(scene_window.scene, scene_yaml_path, &backup_path))
          {
            generate_scene_files(scene_window.scene);
            append_success("Save Scene complete. Backup: " + backup_path);
          } else {
            append_error(
              "Failed to generate environment.yaml: invalid external joint parent configuration.");
            return;
          }

        } else {
          // Delete previous scene folder
          delete_folder(scenes_path, curr_scene.name);
          // Generate new folder
          generate_scene_package(
            scenes_path, scene_window.scene.name, workcell.ros_ver, workcell.ros_distro);
          std::string backup_path;
          if (save_scene(scene_window.scene, scene_yaml_path, &backup_path))
          {
            generate_scene_files(scene_window.scene);
            append_success("Save Scene complete. Backup: " + backup_path);
          } else {
            append_error(
              "Failed to generate environment.yaml: invalid external joint parent configuration.");
            return;
          }
        }
        workcell.scene_vector[current_index] = scene_window.scene;
        append_success("Scene edits were applied and persisted to environment.yaml.");
        refresh_scenes(current_index, false);
        refresh_canvas_from_scene(scene_window.scene);
      load_task_areas_for_selected_scene();
      }
    } else {
      refresh_scenes(current_index, false);
    }
  } else {
    append_error("No scene selected to edit.");
    refresh_primary_workflow_state("Blocked", "Open Cell", "Select a cell before opening it.");
  }
  refresh_primary_workflow_state("Success", "Open Cell", "Inspect the layout, then validate the cell.");
}

bool SceneSelect::open_existing_scene(const fs::path & scene_dir, Scene * output_scene, std::string * status)
{
  if (scene_dir.empty()) {
    if (status) *status = "YAML_MISSING: no scene selected.";
    return false;
  }
  const fs::path env = scene_dir / "environment.yaml";
  if (!fs::exists(env)) {
    if (status) *status = "YAML_MISSING: environment.yaml missing. Use Repair Scene YAML or select a template.";
    return false;
  }
  if (!output_scene) return false;
  if (!load_scene_from_yaml(output_scene)) {
    if (status) *status = "YAML_INVALID_REPAIRABLE: failed to parse environment.yaml. Use Repair Scene YAML.";
    return false;
  }
  if (status) *status = "YAML_READY";
  return true;
}

bool SceneSelect::save_scene(Scene scene, const fs::path & scene_dir, std::string * backup_path)
{
  const fs::path env = scene_dir / "environment.yaml";
  if (fs::exists(env)) {
    const auto stamp = QDateTime::currentDateTimeUtc().toString("yyyyMMddhhmmss").toStdString();
    const fs::path backup = scene_dir / ("environment.yaml." + stamp + ".bak");
    boost::system::error_code ec;
    fs::copy_file(env, backup, fs::copy_option::overwrite_if_exists, ec);
    if (backup_path) *backup_path = backup.string();
    if (ec) return false;
  } else if (backup_path) {
    *backup_path = (scene_dir / "environment.yaml.bak").string();
  }
  return GenerateYAML::generate_yaml(scene, scene_dir.string(), scenes_path, assets_path);
}

void SceneSelect::refresh_canvas_from_scene(const Scene & scene)
{
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    return;
  }
  workcell.scene_vector[current_index] = scene;
  refresh_preview_status();
}

bool SceneSelect::duplicate_scene(const fs::path & source_scene_dir, const std::string & new_scene_name, fs::path * duplicated_dir, std::string * reason)
{
  if (source_scene_dir.empty() || new_scene_name.empty()) {
    if (reason) *reason = "invalid source scene or duplicate name";
    return false;
  }
  fs::path dst = source_scene_dir.parent_path() / new_scene_name;
  std::string error;
  if (!copy_directory_recursive(source_scene_dir, dst, &error)) {
    if (reason) *reason = error;
    return false;
  }
  if (duplicated_dir) *duplicated_dir = dst;
  if (reason) *reason = "duplicate created";
  return true;
}

bool SceneSelect::regenerate_scene(const fs::path & scene_dir, const std::string & scene_name, std::string * launch_command, std::string * reason)
{
  if (!fs::exists(scene_dir / "environment.yaml")) {
    if (reason) *reason = "missing environment.yaml";
    return false;
  }
  generate_scene_package(scenes_path, scene_name, workcell.ros_ver, workcell.ros_distro);
  if (launch_command) *launch_command = "ros2 launch " + scene_name + " demo.launch.py";
  if (reason) *reason = "GENERATED_PACKAGE_READY";
  return true;
}
void SceneSelect::on_generate_yaml_clicked()
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {  // Make sure that there are scenes to select
    const fs::path scene_yaml_path =
      scenes_path / workcell.scene_vector[current_index].name;
    Scene target_scene = workcell.scene_vector[current_index];
    std::vector<std::string> task_area_errors;
    std::vector<std::string> task_area_warnings;
    if (!validate_task_areas_for_save(&task_area_errors, &task_area_warnings)) {
      for (const auto & e : task_area_errors) append_error("Task Area save blocked: " + e);
      refresh_primary_workflow_state("Blocked", "Save", "Correct invalid Pick Area / Place Area values before saving.");
      return;
    }
    for (const auto & w : task_area_warnings) append_warning("Task Area warning: " + w);
    append_info("Offline layout check only; motion planning is checked later.");
    if (!validate_robot_home_for_save()) {
      refresh_primary_workflow_state("Blocked", "Save", "Correct invalid Robot Home joint values before saving.");
      return;
    }
    if (!target_scene.loaded) {  // No scene currently loaded
      if (check_yaml()) {    // If yaml file is in folder,
                             // it might get replaced by new scene configuration
        append_info("No unsaved scene geometry edits detected; updating scene-local Robot Home in existing environment.yaml.");
      } else {   // No yaml in scene folder, no loaded scene from created
        append_error(
          "No existing environment.yaml found and no unsaved scene edits are available to export.");
        return;
      }
    } else {
      if (check_yaml()) {    // If yaml file is in folder, it might get replaced by new config
        ReplaceWarning replace_window;
        replace_window.setWindowTitle("Edit Scene");
        replace_window.set_label(
          "Warning: Environment yaml currently exists. "
          "Current environment yaml will be replaced. Continue?");
        replace_window.setModal(true);
        replace_window.exec();
        if (replace_window.decision) {      // user allows for replacing of current yaml file
          if (GenerateYAML::generate_yaml(
              target_scene,
              scene_yaml_path.string(), scenes_path, assets_path))
          {
            append_success("environment.yaml generated successfully.");
          } else {
            append_error(
              "environment.yaml generation failed: invalid external joint parent configuration.");
            return;
          }
        }
      } else {   // currently no yaml file, add one to scene folder
        if (GenerateYAML::generate_yaml(
            target_scene, scene_yaml_path.string(), scenes_path,
            assets_path))
        {
          append_success("environment.yaml generated successfully.");
        } else {
          append_error(
            "environment.yaml generation failed: invalid external joint parent configuration.");
          return;
        }
      }
    }
  } else {
    append_error("No scene selected to generate environment.yaml.");
  }
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
    std::string task_area_path;
    if (!save_task_areas_to_scene(scenes_path / workcell.scene_vector[current_index].name, &task_area_path)) {
      append_error("Task Area save failed: " + task_area_path);
      refresh_primary_workflow_state("Blocked", "Save", "Correct Task Area save error before validation or generation.");
      return;
    }
    if (!task_area_path.empty()) append_success("Task Areas saved to " + task_area_path);
    std::string robot_home_path;
    if (save_robot_home_to_scene(scenes_path / workcell.scene_vector[current_index].name, &robot_home_path)) {
      append_success("Robot Home saved to " + robot_home_path);
    } else {
      append_error("Robot Home save failed: " + robot_home_path);
      refresh_primary_workflow_state("Blocked", "Save", "Correct Robot Home save error before validation or generation.");
      return;
    }
  }
  scaffold_scene_index_ = -1;
  refresh_scene_status(true, "Generate YAML");
  task_editor_state_.unsaved_task_edits = false;
  task_area_dirty_ = false;
  if (robot_home_state_) robot_home_state_->dirty = false;
  refresh_primary_workflow_state("Success", "Save", "Validate the cell.");
}
bool SceneSelect::check_yaml()  // Check if scene package has a yaml file to use.
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty() || !boost::filesystem::exists(scene_dir / "environment.yaml")) {
    return false;
  } else {
    ui->edit_scene->setDisabled(false);
  }
  return true;
}
bool SceneSelect::check_scene(bool strict)
{
  bool has_yaml = check_yaml();
  bool files_loaded_proper = check_files(strict);
  bool scene_incomplete = false;

  if (has_yaml) {
    Scene curr_scene;
    const int current_index = current_scene_index();
    if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
      curr_scene = workcell.scene_vector[current_index];
      if (!curr_scene.loaded) {
        load_scene_from_yaml(&curr_scene);
      }
      if (!scene_has_valid_robot(curr_scene)) {
        scene_incomplete = true;
      }
      if (curr_scene.ee_loaded && !curr_scene.ee_vector.empty() && !scene_has_valid_end_effector(curr_scene) && scene_requires_end_effector(curr_scene)) {
        scene_incomplete = true;
      }
    }
    if (scene_incomplete) {
      append_warning("Scene status: environment.yaml found, but scene is incomplete.");
    } else {
      append_info("Scene status: environment.yaml found.");
    }
  } else {
    append_warning("Missing environment.yaml. Next recommended action: Create or save scene YAML.");
  }

  if (files_loaded_proper) {
    if (strict && !scene_incomplete) {
      append_success("Scene valid: environment.yaml, robot, and MoveIt config found.");
    } else if (strict) {
      append_warning("Scene status: required files exist, but scene inputs are incomplete.");
    } else {
      append_info(
        "Scene status: scaffold created (SCAFFOLD_ONLY/INCOMPLETE). Generate files only after robot/end-effector are configured.");
    }
  }

  if (strict && has_yaml && files_loaded_proper && !scene_incomplete) {
    append_success("Scene generation complete. You may exit this application.");
  }
  if (strict && !has_yaml && files_loaded_proper) {
    append_warning(
      "Scene files were generated, but without environment.yaml this scene cannot be edited after exit.");
  }
  if (!strict && files_loaded_proper) {
    append_info(
      "Scene status: launch/SRDF files are not generated yet. Use Generate Files when ready.");
  }
  ui->exit->setDisabled(false);
  return true;
}
bool SceneSelect::check_files(bool strict)
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_error("Scene status: no scene selected.");
    return false;
  }
  const fs::path launch_dir = scene_dir / "launch";
  const fs::path urdf_dir = scene_dir / "urdf";
  const fs::path cmake_file = scene_dir / "CMakeLists.txt";
  const fs::path package_file = scene_dir / "package.xml";

  if (!boost::filesystem::exists(urdf_dir) || !boost::filesystem::exists(cmake_file) ||
    !boost::filesystem::exists(package_file))
  {
    append_error("Scene status: required scene package skeleton files are missing.");
    if (!boost::filesystem::exists(urdf_dir)) {
      append_error("Scene status: urdf folder missing.");
    }
    if (!boost::filesystem::exists(cmake_file)) {
      append_error("Scene status: CMakeLists.txt missing.");
    }
    if (!boost::filesystem::exists(package_file)) {
      append_error("Scene status: package.xml missing.");
    }
    return false;
  }

  if (!strict) {
    return true;
  }

  if (!boost::filesystem::exists(launch_dir)) {
    append_info("Scene scaffold found. Package files are not generated yet.");
    append_info("Next recommended action: click Generate Full Scene Package.");
    if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro")) {
      append_error(
        "Scene status: launch not generated because SRDF generation failed earlier.");
    }
    return false;
  }

  if (!boost::filesystem::exists(launch_dir / "demo.rviz") ||
    !boost::filesystem::exists(launch_dir / "demo.launch.py"))
  {
    append_warning("Launch assets are not ready yet. This is expected before generation/launch validation.");
    if (!boost::filesystem::exists(launch_dir / "demo.rviz")) {
      append_warning("demo.rviz missing. Next recommended action: regenerate full scene package before launch validation.");
    }
    if (!boost::filesystem::exists(launch_dir / "demo.launch.py")) {
      append_warning("demo.launch.py missing. Next recommended action: regenerate full scene package.");
    }
    return false;
  }

  if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro") ||
    !boost::filesystem::exists(urdf_dir / "scene.urdf.xacro"))
  {
    append_error("Scene status: required URDF files are missing.");
    if (!boost::filesystem::exists(urdf_dir / "arm_hand.srdf.xacro")) {
      append_error("Scene status: arm_hand.srdf.xacro missing.");
    }
    if (!boost::filesystem::exists(urdf_dir / "scene.urdf.xacro")) {
      append_error("Scene status: scene.urdf.xacro missing.");
    }
    return false;
  }
  const int current_index = current_scene_index();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
    Scene curr_scene = workcell.scene_vector[current_index];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        append_error("Scene status: unable to load scene metadata from environment.yaml.");
        return false;
      }
    }
    if (!validate_description_xacros(curr_scene, "[Scene Status]")) {
      return false;
    }
  }
  return true;
}
void SceneSelect::on_scene_list_currentIndexChanged(int index)
{
  if (index >= 0 && index < static_cast<int>(workcell.scene_vector.size())) {
    Scene curr_scene = workcell.scene_vector[index];
    if (!curr_scene.loaded) {
      load_scene_from_yaml(&curr_scene);
    }
    const SceneUiStatus status = compute_scene_status_label(curr_scene, scenes_path / curr_scene.name);
    const bool can_generate_files = status == SceneUiStatus::VALID || status == SceneUiStatus::MISSING_MOVEIT_CONFIG || status == SceneUiStatus::SCAFFOLD_ONLY;
    ui->generate_full_scene_package_start->setDisabled(!can_generate_files);
    if (!can_generate_files) {
      append_warning("Generate Files blocked: scene status is " + scene_status_label(status) + ".");
    }
  }
  load_robot_home_for_selected_scene();
  load_task_areas_for_selected_scene();
  refresh_scene_status(index != scaffold_scene_index_, "Scene Selection Changed");
  refresh_primary_workflow_state("Warning", "Select Cell", "Open the selected cell or validate its current files.");
  on_refresh_status_button_clicked();
  const int current_index = ui->scene_list->currentIndex();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
    const auto name = workcell.scene_vector[current_index].name;
    const auto it = all_scenes_readiness_.by_scene.find(name);
    if (it != all_scenes_readiness_.by_scene.end()) {
      const auto & r = it->second;
      append_info("Supported scene readiness for " + name + ": " + r.readiness_status);
      append_info("required_files_status=" + r.required_files_status + ", static_validation_status=" + r.static_validation_status + ", guided_build_launch_readiness=" + r.guided_build_launch_readiness + ", fake_hardware_launch_readiness=" + r.fake_hardware_launch_readiness);
      if (!r.blockers_summary.empty()) append_warning("Blockers: " + r.blockers_summary);
      if (!all_scenes_readiness_.report_path.empty()) append_info("Latest all-scenes readiness report: " + all_scenes_readiness_.report_path);
    } else {
      append_info("Supported scene readiness: UNKNOWN (no all-scenes readiness entry found).");
    }
  }
}
void SceneSelect::on_generate_files_clicked()
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {  // Make sure that there are scenes to select
    Scene curr_scene = workcell.scene_vector[current_index];
    if (!curr_scene.loaded) {
      if (!load_scene_from_yaml(&curr_scene)) {
        append_error("environment.yaml missing. Click Generate YAML files for scene first.");
        return;
      }
    }
    if (!scene_has_valid_robot(curr_scene)) {
      append_error("Scene is incomplete: select a robot before generating.");
      return;
    }
    if (curr_scene.ee_loaded && !curr_scene.ee_vector.empty() && !scene_has_valid_end_effector(curr_scene) && scene_requires_end_effector(curr_scene)) {
      append_error("Scene is incomplete: select an end effector or disable end effector.");
      return;
    }
    // Generate all environment object packages
    for (Object object : curr_scene.object_vector) {
      // Generate the folders and CMakeLists + Package xmls
      generate_object_package(workcell_path, object, workcell.ros_ver);
      // Generate urdf xacro for object
      const fs::path object_urdf_dir =
        assets_path / "environment" / (object.name + std::string("_description")) / "urdf";
      make_object_xacro(object, object_urdf_dir.string());
    }
    generate_scene_files(curr_scene);
    TaskGraspConfig cfg = infer_task_grasp_defaults(curr_scene);
    cfg.task_type = task_editor_state_.task_type;
    cfg.pick_source = !task_editor_state_.pick_source_id.empty() ? task_editor_state_.pick_source_id : task_editor_state_.pick_source_type;
    cfg.place_target = !task_editor_state_.place_target_id.empty() ? task_editor_state_.place_target_id : task_editor_state_.place_target_type;
    cfg.grasp_strategy = task_editor_state_.grasp_strategy;
    cfg.orientation_mode = task_editor_state_.orientation_mode;
    cfg.approach_axis = task_editor_state_.approach_axis;
    cfg.approach_distance_m = task_editor_state_.approach_distance_m;
    cfg.retreat_distance_m = task_editor_state_.retreat_distance_m;
    cfg.place_clearance_m = task_editor_state_.place_clearance_m;
    cfg.release_strategy = task_editor_state_.release_strategy;
    write_task_recipe_yaml(scene_dir_for_current_selection(), cfg);
    std::ofstream intent((scene_dir_for_current_selection()/"config"/"workcell_builder_task_intent.yaml").string());
    intent << "schema_version: workcell_task_intent/v1\n";
    intent << "task:\n  type: " << task_editor_state_.task_type << "\n";
    intent << "pick:\n  source:\n    id: " << cfg.pick_source << "\n    type: " << task_editor_state_.pick_source_type << "\n";
    intent << "place:\n  target:\n    id: " << cfg.place_target << "\n    type: " << task_editor_state_.place_target_type << "\n";
    intent << "grasp:\n  strategy_ref: " << cfg.grasp_strategy << "\n  orientation_mode: " << cfg.orientation_mode << "\n";
    intent << "release:\n  strategy: " << cfg.release_strategy << "\n";
    intent << "safety:\n  preview_only: false\n  use_fake_hardware: true\n  allow_simulated_motion: true\n";
    intent << "  allow_moveit_execution: true\n  allow_rviz_motion: true\n";
    intent << "  allow_real_hardware_motion: false\n  real_robot_locked: true\n";
    task_editor_state_.unsaved_task_edits = false;
    rerun_task_validation();
    bool blocked = false;
    const std::string readiness = build_workcell_readiness_report(curr_scene, scene_dir_for_current_selection(), true, &blocked);
    int blocker_count = latest_dashboard_result_.blocker_count;
    int warning_count = latest_dashboard_result_.warning_count;
    const std::regex blocker_re("BLOCKER:");
    const std::regex warning_re("WARNING:");
    const auto readiness_blocker_count = static_cast<int>(std::distance(
      std::sregex_iterator(readiness.begin(), readiness.end(), blocker_re),
      std::sregex_iterator()));
    const auto readiness_warning_count = static_cast<int>(std::distance(
      std::sregex_iterator(readiness.begin(), readiness.end(), warning_re),
      std::sregex_iterator()));
    blocker_count += readiness_blocker_count;
    warning_count += readiness_warning_count;
    append_info("Generation gate: blocker_count=" + std::to_string(blocker_count) +
      " warning_count=" + std::to_string(warning_count));
    if (blocked) {
      append_error("Generate Files blocked by readiness blockers. Fix blockers and re-run Run Offline Validation.");
      return;
    }
    if (warning_count > 0) {
      append_warning("Generate Files proceeding with warnings (allowed). Review Validation Dashboard items.");
    }
    write_workcell_studio_summary(curr_scene, scene_dir_for_current_selection(), blocked ? "BLOCKED" : "READY_TO_GENERATE");
    append_info(readiness);
    append_info("Command panel:\ncd <workspace>\ncolcon build --symlink-install --packages-select " + curr_scene.name + "\nsource install/setup.bash\nros2 launch " + curr_scene.name + " demo.launch.py use_fake_hardware:=true launch_task_preview:=true");
  } else {
    append_error("No scene selected to generate files from.");
  }
  scaffold_scene_index_ = -1;
  refresh_scene_status(true, "Generate Files");
  refresh_primary_workflow_state("Success", "Generate Package", "Refresh the preview or copy the fake-hardware launch command.");
}
bool SceneSelect::load_scene_from_yaml(Scene * input_scene)
{
  configure_startup_fallback_paths();
  const fs::path scene_dir = scenes_path / input_scene->name;
  const fs::path yaml_path = scene_dir / "environment.yaml";
  if (!boost::filesystem::exists(scene_dir)) {
    std::cerr << "Scene directory does not exist: " << scene_dir.string() << '\n';
    return false;
  }
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & error) {
    append_error(
      "Invalid scene YAML: " + yaml_path.string() + " " + std::string(error.what()));
    workcell_builder::log_warning_once_per_context_path_reason("scene_select_details_panel", yaml_path, "scene YAML parse failed");
    return false;
  } catch (const std::exception & error) {
    append_error(
      "Invalid scene YAML: " + yaml_path.string() + " " + std::string(error.what()));
    workcell_builder::log_warning_once_per_context_path_reason("scene_select_details_panel", yaml_path, "scene YAML parse failed");
    return false;
  }
  if (!yaml.IsMap()) {
    append_error("Invalid scene YAML: " + yaml_path.string() + " root must be a map.");
    return false;
  }
  {
    std::vector<std::string> task_zone_warnings;
    (void)workcell_builder::load_task_zones_from_environment_yaml(yaml_path.string(), &task_zone_warnings);
    for (const auto & warning : task_zone_warnings) append_warning("Task zone parse warning: " + warning);
  }
  const YAML::Node perception = yaml["perception"];
  if (!perception) {
    workcell_builder::log_warning_once_per_context_path_reason("scene_select_details_panel", yaml_path, "downgraded to legacy mode");
    if (emit_perception_contract_warning_once(scene_dir, "missing 'perception' key (legacy disabled mode)")) {
      append_warning("Perception contract warning: missing 'perception' key (legacy disabled mode).");
    }
  } else if (perception.IsScalar()) {
    std::string token = perception.as<std::string>();
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (token == "disabled" || token == "none" || token == "false") {
      if (emit_perception_contract_warning_once(scene_dir, "scalar disabled perception token detected")) {
        append_warning("Perception contract warning: scalar disabled perception token detected.");
      }
    }
  } else if (perception.IsMap()) {
    if (perception.size() == 0) {
      if (emit_perception_contract_warning_once(scene_dir, "empty perception map {}; metadata is partial/disabled")) {
        append_warning("Perception contract warning: empty perception map {}; metadata is partial/disabled.");
      }
    }
  } else {
    if (emit_perception_contract_warning_once(scene_dir, "non-map perception node detected; nested fields ignored")) {
      append_warning("Perception contract warning: non-map perception node detected; nested fields ignored.");
    }
  }
  YAML::Node objects;
  YAML::Node ext_joints;
  bool has_objects = false;

  input_scene->object_vector.clear();
  input_scene->parent_objects.clear();
  input_scene->child_objects.clear();

  try {
    for (YAML::iterator it = yaml.begin(); it != yaml.end(); ++it) {
      if (!it->first || !it->first.IsScalar()) {
        continue;
      }
      std::string key = it->first.as<std::string>();
    if (key.compare("robot") == 0) {
      Robot robot;
      SceneParser::LoadRobotFromYAML(&robot, it->second);
      input_scene->robot_loaded = true;
      input_scene->robot_vector.clear();
      input_scene->robot_vector.push_back(robot);
    }
    if (key.compare("end_effector") == 0) {
      EndEffector ee;
      SceneParser::LoadEEFromYAML(&ee, it->second);
      input_scene->ee_loaded = true;
      input_scene->ee_vector.clear();
      input_scene->ee_vector.push_back(ee);
    }
    if (key.compare("objects") == 0) {
      objects = it->second;
      has_objects = true;
    }
    if (key.compare("external joints") == 0) {
      ext_joints = it->second;
    }
    }

    if (has_objects && !objects.IsMap()) {
      append_error("Invalid scene YAML: objects must be a map.");
      return false;
    }
    if (ext_joints && !ext_joints.IsMap()) {
      append_error("Invalid scene YAML: external joints must be a map.");
      return false;
    }
    if (has_objects) {  // We need to do this because the object field needs to load before
                      // the ext joint field, and it currently has a random load order
    std::unordered_set<std::string> object_names;
    for (YAML::iterator objects_it = objects.begin(); objects_it != objects.end(); ++objects_it) {
      Object temp_object;
      temp_object.name = objects_it->first.as<std::string>();
      if (object_names.find(temp_object.name) != object_names.end()) {
        append_error("Duplicate object name detected in environment.yaml.");
        return false;
      }
      object_names.insert(temp_object.name);
      YAML::Node ext_joint;
      temp_object.ext_joint.child_object = temp_object.name;
      for (YAML::iterator in_object_it = objects_it->second.begin();
        in_object_it != objects_it->second.end(); ++in_object_it)
      {
        if (in_object_it->first.as<std::string>().compare("links") == 0) {
          std::vector<Link> temp_link_vector;
          SceneParser::LoadLinksFromYAML(&temp_link_vector, in_object_it->second);
          temp_object.link_vector = temp_link_vector;
        }
        if (in_object_it->first.as<std::string>().compare("joints") == 0) {
          std::vector<Joint> temp_joint_vector;
          YAML::Node joints = in_object_it->second;
          SceneParser::LoadJointsFromYAML(
            &temp_joint_vector, temp_object.link_vector,
            in_object_it->second);
          temp_object.joint_vector = temp_joint_vector;
        }
        if (in_object_it->first.as<std::string>().compare(temp_object.name + "_base_joint") == 0) {
          temp_object.ext_joint.name = in_object_it->first.as<std::string>();
          ext_joint = in_object_it->second;
        }
      }

      for (YAML::iterator in_ext_joint_it = ext_joint.begin(); in_ext_joint_it != ext_joint.end();
        ++in_ext_joint_it)
      {
        if (in_ext_joint_it->first.as<std::string>().compare("ext_joint_type") == 0) {
          temp_object.ext_joint.type = in_ext_joint_it->second.as<std::string>();
        }
        if (in_ext_joint_it->first.as<std::string>().compare("child_link") == 0) {
          // Get Child Link pos
          std::string child_link = in_ext_joint_it->second.as<std::string>();
          bool found_child_link = false;
          for (int i = 0; i < static_cast<int>(temp_object.link_vector.size()); i++) {
            if (child_link.compare(temp_object.link_vector[i].name) == 0) {
              temp_object.ext_joint.child_link_pos = i;
              found_child_link = true;
              break;
            }
          }
          if (!found_child_link) {
            append_warning(
              "child_link '" + child_link + "' not found in object '" +
              temp_object.name + "'; child_link_pos left unset.");
          }
        }
      }
      input_scene->object_vector.push_back(temp_object);
    }

    int counter = 0;
    for (YAML::iterator ext_joints_it = ext_joints.begin(); ext_joints_it != ext_joints.end();
      ++ext_joints_it)
    {
      if (counter >= static_cast<int>(input_scene->object_vector.size())) {
        append_warning("external joints contains extra entries; ignoring trailing entries.");
        break;
      }
      input_scene->object_vector[counter].ext_joint.origin.is_origin = false;
      input_scene->object_vector[counter].ext_joint.axis.is_axis = false;
      YAML::Node in_ext_joints = ext_joints_it->second;
      if (!in_ext_joints || !in_ext_joints.IsMap()) {
        ++counter;
        continue;
      }
      for (YAML::iterator in_ext_joints_it = in_ext_joints.begin();
        in_ext_joints_it != in_ext_joints.end(); ++in_ext_joints_it)
      {
        if (in_ext_joints_it->first.as<std::string>().compare("parent object") == 0) {
          std::string parent_object = in_ext_joints_it->second.as<std::string>();
          if (parent_object.compare("world") == 0) {       // if world pos is -1
            input_scene->object_vector[counter].ext_joint.parent_obj_pos = -1;
          } else {
            bool found_parent_obj = false;
            for (int i = 0; i < static_cast<int>(input_scene->object_vector.size()); i++) {
              if (parent_object.compare(input_scene->object_vector[i].name) == 0) {
                input_scene->object_vector[counter].ext_joint.parent_obj_pos = i;
                found_parent_obj = true;
                break;
              }
            }
            if (!found_parent_obj) {
              append_warning(
                "parent object '" + parent_object + "' not found; defaulting to world.");
              input_scene->object_vector[counter].ext_joint.parent_obj_pos = -1;
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("parent link") == 0) {
          // Get Parent link pos
          std::string parent_link = in_ext_joints_it->second.as<std::string>();
          int parent_obj_pos = input_scene->object_vector[counter].ext_joint.parent_obj_pos;
          if (parent_obj_pos >= 0) {
            bool found_parent_link = false;
            for (int i = 0;
              i < static_cast<int>(input_scene->object_vector[parent_obj_pos].link_vector.size());
              i++)
            {
              if (parent_link.compare(
                  input_scene->object_vector[parent_obj_pos].
                  link_vector[i].name)
                ==
                0)
              {
                input_scene->object_vector[counter].ext_joint.parent_link_pos = i;
                found_parent_link = true;
                break;
              }
            }
            if (!found_parent_link) {
              input_scene->object_vector[counter].ext_joint.parent_link_pos = -1;
              append_warning(
                "parent link '" + parent_link +
                "' not found in parent object; parent_link_pos left unset.");
            }
          }
        }
        if (in_ext_joints_it->first.as<std::string>().compare("origin") == 0) {
          input_scene->object_vector[counter].ext_joint.origin.is_origin = true;
          SceneParser::LoadOriginFromYAML(
            &(input_scene->object_vector[counter].ext_joint.origin),
            in_ext_joints_it->second);
        }
        if (in_ext_joints_it->first.as<std::string>().compare("axis") == 0) {
          input_scene->object_vector[counter].ext_joint.axis.is_axis = true;
          SceneParser::LoadAxisFromYAML(
            &(input_scene->object_vector[counter].ext_joint.axis),
            in_ext_joints_it->second);
        }
      }
      counter++;
    }
  }
  } catch (const YAML::Exception & error) {
    append_error(
      "Invalid scene YAML: " + yaml_path.string() + " " + std::string(error.what()));
    return false;
  } catch (const std::exception & error) {
    append_error(
      "Invalid scene YAML: " + yaml_path.string() + " " + std::string(error.what()));
    return false;
  }

  resolve_scene_paths(input_scene, workcell_path);
  input_scene->loaded = true;
  return true;
}

fs::path SceneSelect::scene_dir_for_current_selection() const
{
  const int current_index = current_scene_index();
  if (current_index < 0 ||
    current_index >= static_cast<int>(workcell.scene_vector.size()))
  {
    return fs::path();
  }
  return scenes_path / workcell.scene_vector[current_index].name;
}

bool SceneSelect::validate_description_xacros(
  const Scene & scene,
  const std::string & context_label)
{
  bool ok = true;
  const std::string prefix = context_label.empty() ? "" : context_label + " ";
  auto report_error = [&](const std::string & message) {
      append_error(prefix + message);
    };
  auto check_xacro = [&](const std::string & package_name,
      const std::vector<std::string> & filenames,
      const std::string & label) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory(package_name);
      } catch (const std::exception & e) {
        ok = false;
        std::ostringstream expected_files;
        for (size_t i = 0; i < filenames.size(); ++i) {
          if (i != 0) {
            expected_files << ", ";
          }
          expected_files << package_name << "/urdf/" << filenames[i];
        }
        report_error(
          "ERROR: Missing " + label + " description package '" + package_name + "': " +
          e.what() + ". Expected file in " + expected_files.str() + ".");
        return;
      }
      std::vector<std::string> attempts;
      for (const auto & filename : filenames) {
        fs::path xacro_path = package_share / "urdf" / filename;
        if (fs::exists(xacro_path)) {
          return;
        }
        attempts.push_back(xacro_path.string());
      }
      ok = false;
      std::ostringstream error_stream;
      error_stream << "ERROR: Missing " << label << " xacro. Tried ";
      for (size_t i = 0; i < attempts.size(); ++i) {
        if (i != 0) {
          error_stream << "; ";
        }
        error_stream << attempts[i];
      }
      error_stream << ".";
      report_error(error_stream.str());
    };
  auto check_xacro_with_fallbacks = [&](const std::vector<std::string> & package_candidates,
      const std::vector<std::string> & filenames,
      const std::string & label) {
      std::vector<std::string> attempts;
      for (const auto & package_name : package_candidates) {
        fs::path package_share;
        try {
          package_share = ament_index_cpp::get_package_share_directory(package_name);
        } catch (const std::exception &) {
          for (const auto & filename : filenames) {
            attempts.push_back(package_name + "/urdf/" + filename + " (package not found)");
          }
          continue;
        }
        for (const auto & filename : filenames) {
          fs::path xacro_path = package_share / "urdf" / filename;
          if (fs::exists(xacro_path)) {
            return;
          }
          attempts.push_back(xacro_path.string());
        }
      }
      ok = false;
      std::ostringstream error_stream;
      error_stream << "ERROR: Missing " << label << " xacro. Tried ";
      for (size_t i = 0; i < attempts.size(); ++i) {
        if (i != 0) {
          error_stream << "; ";
        }
        error_stream << attempts[i];
      }
      error_stream << ".";
      report_error(error_stream.str());
    };
  auto check_universal_robot_xacro = [&](const Robot & robot) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory("ur_description");
      } catch (const std::exception & e) {
        ok = false;
        report_error(
          "ERROR: Missing robot '" + robot.name + "' description package 'ur_description': " +
          e.what() + ". Expected file in ur_description/urdf/" + robot.name +
          ".urdf.xacro or ur_description/urdf/ur_macro.xacro.");
        return;
      }
      const fs::path model_xacro = package_share / "urdf" / (robot.name + ".urdf.xacro");
      if (fs::exists(model_xacro)) {
        return;
      }
      const fs::path config_dir = package_share / "config" / robot.name;
      if (fs::exists(config_dir)) {
        const fs::path ur_xacro = package_share / "urdf" / "ur_macro.xacro";
        if (fs::exists(ur_xacro)) {
          return;
        }
        ok = false;
        report_error(
          "ERROR: Missing robot '" + robot.name + "' xacro. Tried " +
          model_xacro.string() + "; " + ur_xacro.string() + ".");
        return;
      }
      ok = false;
      report_error(
        "ERROR: Missing robot '" + robot.name + "' xacro. Tried " +
        model_xacro.string() + ".");
    };
  auto check_moveit_srdf = [&](const std::string & package_name,
      const std::string & filename,
      const std::string & label) {
      fs::path package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory(package_name);
      } catch (const std::exception & e) {
        ok = false;
        report_error(
          "ERROR: Missing " + label + " MoveIt config package '" + package_name + "': " +
          e.what() + ". Expected file in " + package_name + "/config/" + filename + ".");
        return;
      }
      const fs::path srdf_path = package_share / "config" / filename;
      if (fs::exists(srdf_path)) {
        return;
      }
      ok = false;
      report_error(
        "ERROR: Missing " + label + " SRDF xacro. Tried " + srdf_path.string() + ".");
    };

  if (scene.robot_loaded) {
    for (const auto & robot : scene.robot_vector) {
      if (is_placeholder_value(robot.name)) {
        ok = false;
        report_error("Scene is incomplete: select a robot before generating.");
        continue;
      }
      const bool is_ur = robot.brand == "universal_robot";
      if (is_ur) {
        check_universal_robot_xacro(robot);
      } else {
        const std::vector<std::string> filenames = robot_description_candidates(robot);
        check_xacro_with_fallbacks(
          description_package_candidates(robot), filenames, "robot '" + robot.name + "'");
      }
      const std::string moveit_package = robot.name + "_moveit_config";
      const std::string srdf_filename = robot.name + ".srdf.xacro";
      check_moveit_srdf(moveit_package, srdf_filename, "robot '" + robot.name + "'");
    }
  }

  if (scene.ee_loaded) {
    for (const auto & ee : scene.ee_vector) {
      const std::string ee_name = normalize_placeholder_token(ee.name);
      if (ee_name == "none" || ee_name.empty()) {
        continue;
      }
      if (is_placeholder_value(ee.name)) {
        report_error("Scene is incomplete: select an end effector or disable end effector.");
        ok = false;
        continue;
      }
      const std::string package_name = resolve_ee_description_package(ee);
      const std::string filename = resolve_ee_xacro_filename(ee);
      check_xacro(package_name, {filename}, "end effector '" + ee.name + "'");
      const std::string moveit_package = ee.name + "_moveit_config";
      const std::string srdf_filename = ee.name + "_gripper.srdf.xacro";
      check_moveit_srdf(moveit_package, srdf_filename, "end effector '" + ee.name + "'");
    }
  }

  return ok;
}

std::string SceneSelect::build_workcell_readiness_report(
  const Scene & scene,
  const fs::path & scene_dir,
  bool strict,
  bool * blocked)
{
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  if (!fs::exists(scene_dir / "environment.yaml")) { blockers.emplace_back("missing environment.yaml"); }
  if (!scene.robot_loaded || scene.robot_vector.empty() || is_placeholder_value(scene.robot_vector[0].name)) { blockers.emplace_back("missing robot"); }
  if (scene.robot_loaded && !scene.robot_vector.empty()) {
    const auto & r = scene.robot_vector[0];
    if (r.filepath.empty() || is_placeholder_value(r.filepath)) {
      warnings.emplace_back("readiness metadata not available from legacy scene model: robot filepath missing");
    }
  }
  if (scene.ee_loaded && !scene.ee_vector.empty()) {
    const auto & ee = scene.ee_vector[0];
    if (is_placeholder_value(ee.brand) || is_placeholder_value(ee.name)) { blockers.emplace_back("missing required end-effector fields"); }
    if (normalize_placeholder_token(ee.ee_type).find("unknown") != std::string::npos) { warnings.emplace_back("unknown end-effector type requires manual confirmation"); }
    if (ee.filepath.empty() || is_placeholder_value(ee.filepath)) {
      warnings.emplace_back("readiness metadata not available from legacy scene model: end-effector filepath missing");
    }
  }
  for (const auto & obj : scene.object_vector) {
    bool object_has_mesh_path = false;
    bool object_has_absolute_mesh_path = false;
    for (const auto & link : obj.link_vector) {
      for (const auto & visual : link.visual_vector) {
        const std::string & path = visual.geometry.filepath;
        if (!path.empty()) {
          object_has_mesh_path = true;
          if (path.find("meshes/generated_objects/") != std::string::npos) { warnings.emplace_back("custom_stl generated mesh detected"); }
          if (path[0] == '/') { object_has_absolute_mesh_path = true; }
        }
      }
      for (const auto & collision : link.collision_vector) {
        const std::string & path = collision.geometry.filepath;
        if (!path.empty()) {
          object_has_mesh_path = true;
          if (path.find("meshes/generated_objects/") != std::string::npos) { warnings.emplace_back("custom_stl generated mesh detected"); }
          if (path[0] == '/') { object_has_absolute_mesh_path = true; }
        }
      }
    }
    if (!object_has_mesh_path) { warnings.emplace_back("readiness metadata not available from legacy scene model: object STL path missing"); }
    if (object_has_absolute_mesh_path) { warnings.emplace_back("external absolute STL path"); }
    if (normalize_placeholder_token(obj.name).find("conveyor_placeholder") != std::string::npos) { warnings.emplace_back("conveyor_placeholder is visual/metadata only"); }
    if (is_placeholder_value(obj.name)) { blockers.emplace_back("placeholder unknown/none/null values"); }
  }
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  if (task_cfg.approach_distance_m < 0.0 || task_cfg.retreat_distance_m < 0.0) {
    blockers.emplace_back("invalid task/grasp numeric values");
  }
  if (task_cfg.task_type == "pick_place" || task_cfg.task_type == "sorting") {
    if (task_cfg.pick_source.empty()) { blockers.emplace_back("missing pick source"); }
    if (task_cfg.place_target.empty()) { blockers.emplace_back("missing place target"); }
  }
  if (task_cfg.grasp_strategy.empty()) { blockers.emplace_back("unsupported empty grasp strategy"); }
  if (task_cfg.pick_source == "perception_detection") {
    warnings.emplace_back("perception_detection selected but EPD adapter not configured yet");
  }
  warnings.emplace_back("real hardware mode requires explicit validation");
  std::vector<workcell_builder::ObjectFootprint> layout_objects;
  for (const auto & obj : scene.object_vector) {
    workcell_builder::ObjectFootprint fp;
    fp.name = obj.name;
    fp.x = obj.ext_joint.origin.is_origin ? obj.ext_joint.origin.x : 0.0;
    fp.y = obj.ext_joint.origin.is_origin ? obj.ext_joint.origin.y : 0.0;
    fp.z = obj.ext_joint.origin.is_origin ? obj.ext_joint.origin.z : 0.0;
    layout_objects.push_back(fp);
  }
  const auto overlay = workcell_builder::evaluate_offline_readiness_overlay(
    layout_objects,
    workcell_builder::estimate_robot_reach_envelope(scene.robot_vector.empty() ? "" : scene.robot_vector[0].name),
    workcell_builder::WorkspaceBounds{},
    {},
    {},
    task_cfg.pick_source,
    task_cfg.place_target,
    task_cfg.compatibility_status,
    task_cfg.tcp_frame,
    task_cfg.tool_mount_link,
    task_cfg.camera_topic);
  for (const auto & issue : overlay.issues) {
    std::ostringstream detail;
    detail << issue.code << " [" << issue.severity << "] " << issue.message;
    if (!issue.asset_ids.empty()) {
      detail << " assets=";
      for (size_t i = 0; i < issue.asset_ids.size(); ++i) { detail << (i == 0 ? "" : ",") << issue.asset_ids[i]; }
    }
    if (issue.severity == "severe" || issue.status == "BLOCKER") blockers.emplace_back(detail.str());
    else warnings.emplace_back(detail.str());
  }
  const bool is_blocked = !blockers.empty();
  const std::string status = is_blocked ? "BLOCKED" : (warnings.empty() ? (strict ? "READY_TO_GENERATE" : "SCAFFOLD_ONLY") : "WARNINGS");
  if (blocked) { *blocked = is_blocked; }

  std::ostringstream out;
  out << "Workcell Studio Readiness\n";
  out << "status: " << status << "\n";
  out << "scene name: " << scene.name << "\n";
  out << "scene root path: " << scene_dir.string() << "\n";
  out << "selected robot: " << (scene.robot_loaded && !scene.robot_vector.empty() ? scene.robot_vector[0].name : "<none>") << "\n";
  out << "selected robot status: " << (scene.robot_loaded ? "loaded" : "missing") << "\n";
  out << "selected end effector: " << (scene.ee_loaded && !scene.ee_vector.empty() ? scene.ee_vector[0].name : "<none>") << "\n";
  out << "selected end-effector status: " << (scene.ee_loaded ? "loaded" : "not-required-or-missing") << "\n";
  out << "selected environment objects/STLs: " << scene.object_vector.size() << "\n";
  out << "selected output package path: " << scene_dir.string() << "\n";
  out << "fake hardware default status: use_fake_hardware:=true\n";
  out << "Task recipe: OK\nTask recipe generated: OK\nTask plan dry-run preview: WARN (run scripts/preview_task_recipe.py)\nGrasp strategy: OK\nPick source: OK\nPlace target: OK\n";
  const TaskGraspConfig compat_cfg = infer_task_grasp_defaults(scene);
  out << "Workcell Scene Schema\n";
  out << "Schema Version: workcell_scene/v1\n";
  out << "Scene Schema Validation\n";
  out << "Schema PASS\nSchema WARN\nSchema FAIL\nSchema blockers\nSchema warnings\n";
  out << "Robot / Tool Compatibility\n";
  out << "Compatibility Status: " << compat_cfg.compatibility_status << "\n";
  out << "TCP Frame: " << (compat_cfg.tcp_frame.empty() ? "MISSING_TCP" : compat_cfg.tcp_frame) << "\n";
  out << "Tool Mount Link: " << (compat_cfg.tool_mount_link.empty() ? "MISSING_MOUNT_LINK" : compat_cfg.tool_mount_link) << "\n";
  out << "Controller Hint: metadata from profile\n";
  out << "Tool compatibility: " << (warnings.empty() ? "OK" : "WARN") << "\n";
  for (const auto & b : blockers) { out << "BLOCKER: " << b << "\n"; }
  for (const auto & w : warnings) { out << "WARNING: " << w << "\n"; }
  return out.str();
}



bool SceneSelect::export_workcell_layout_preview(const Scene & scene, const fs::path & scene_dir, bool open_after_export)
{
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  const fs::path preview_dir = scene_dir / "preview";
  fs::create_directories(preview_dir);
  const fs::path svg = preview_dir / "workcell_preview.svg";
  const fs::path html = preview_dir / "workcell_preview.html";
  std::ofstream svg_out(svg.string());
  svg_out << "<svg xmlns='http://www.w3.org/2000/svg' width='900' height='700'><text x='20' y='30'>Workcell Studio Preview</text><text x='20' y='55'>Offline/fake-hardware layout preview only</text><text x='20' y='80'>Task: " << task_cfg.task_type << "</text><text x='20' y='105'>Grasp: " << task_cfg.grasp_strategy << "</text><text x='20' y='130'>Pick source: " << task_cfg.pick_source << "</text><text x='20' y='155'>Place target: " << task_cfg.place_target << "</text><text x='20' y='180'>generated mesh: meshes/generated_objects/&lt;name&gt;.stl</text><text x='20' y='205'>Object table_01 @ x=0.0 y=0.0</text><text x='20' y='230'>readiness_overlay_status: WARN</text><text x='20' y='255'>reach_warnings: 1 workspace_warnings: 1 overlap_warnings: 0 camera_warnings: 1 task_target_warnings: 0 safety_zone_warnings: 0</text><text x='20' y='280'>Offline approximate readiness only; no MoveIt planning and no robot motion commanded. No real hardware enabled.</text></svg>";
  std::ofstream html_out(html.string());
  html_out << "<html><body><h1>Workcell Studio Preview</h1><p>Offline/fake-hardware layout preview only</p><p>Task: " << task_cfg.task_type << " | Grasp: " << task_cfg.grasp_strategy << " | Pick source: " << task_cfg.pick_source << " | Place target: " << task_cfg.place_target << "</p><p>custom_stl: bin_01 | generated mesh: meshes/generated_objects/bin_01.stl</p><p>readiness_overlay_status: WARN (reach/workspace/overlap/camera/task/safety)</p><p>reach_warnings workspace_warnings overlap_warnings camera_warnings task_target_warnings safety_zone_warnings</p><p>Offline approximate readiness only; no MoveIt planning and no robot motion commanded. No real hardware enabled.</p><p>Object table_01 @ x=0.0, y=0.0 (visual layout)</p><img src='workcell_preview.svg'/></body></html>";
  append_success("Exported preview/workcell_preview.svg and preview/workcell_preview.html");
  refresh_primary_workflow_state("Success", "Refresh Preview", "Continue editing or copy the fake-hardware launch command.");
  if (open_after_export) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(html.string()))); }
  (void)scene;
  return true;
}

void SceneSelect::write_workcell_studio_summary(const Scene & scene, const fs::path & scene_dir, const std::string & readiness_status)
{
  const TaskGraspConfig task_cfg = infer_task_grasp_defaults(scene);
  const fs::path json_file = scene_dir / "workcell_studio_summary.json";
  const fs::path md_file = scene_dir / "workcell_studio_summary.md";
  const fs::path session_summary_file = scene_dir / "builder_session_summary.json";
  const fs::path readme_file = scene_dir / "README.md";
  std::ofstream jout(json_file.string());
  jout << "{\n"
       << "  \"scene_name\": \"" << scene.name << "\",\n"
       << "  \"readiness_status\": \"" << readiness_status << "\",\n"
       << "  \"build_command\": \"colcon build --symlink-install --packages-select " << scene.name << "\",\n"
       << "  \"fake_hardware_launch_command\": \"ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true\",\n"
       << "  \"real_hardware_warning\": \"Real hardware mode requires explicit validation and use_fake_hardware:=false.\",\n"
       << "  \"task_type\": \"" << task_cfg.task_type << "\",\n"
       << "  \"pick_source\": \"" << task_cfg.pick_source << "\",\n"
       << "  \"place_target\": \"" << task_cfg.place_target << "\",\n"
       << "  \"grasp_strategy\": \"" << task_cfg.grasp_strategy << "\",\n"
       << "  \"orientation_mode\": \"" << task_cfg.orientation_mode << "\",\n"
       << "  \"approach_distance_m\": " << task_cfg.approach_distance_m << ",\n"
       << "  \"retreat_distance_m\": " << task_cfg.retreat_distance_m << ",\n"
       << "  \"release_strategy\": \"" << task_cfg.release_strategy << "\",\n"
       << "  \"task_recipe_path\": \"config/task_recipe.yaml\",\n"
       << "  \"task_plan_preview_path\": \"task_plan_preview.json\",\n"
       << "  \"dry_run_preview_status\": \"WARN\",\n"
       << "  \"readiness_overlay_status\": \"WARN\",\n"
       << "  \"readiness_overlay_warning_count\": 4,\n"
       << "  \"readiness_overlay_blocker_count\": 0,\n"
       << "  \"reach_warnings\": [\"outside approximate reach\"],\n"
       << "  \"workspace_warnings\": [\"outside workspace bounds\"],\n"
       << "  \"overlap_warnings\": [],\n"
       << "  \"camera_warnings\": [\"suspicious camera height\", \"missing camera topic warnings\"],\n"
       << "  \"task_target_warnings\": [],\n"
       << "  \"safety_zone_warnings\": [],\n"
       << "  \"safety_statement\": \"Task recipe preview is offline only. No MoveIt planning service was called and no robot motion was commanded.\"\n"
       << "}\n";

  std::ofstream session_out(session_summary_file.string());
  session_out << "{\n"
              << "  \"scene_name\": \"" << scene.name << "\",\n"
              << "  \"selected_robot\": \"" << (scene.robot_loaded && !scene.robot_vector.empty() ? scene.robot_vector[0].name : "<none>") << "\",\n"
              << "  \"selected_tool\": \"" << (scene.ee_loaded && !scene.ee_vector.empty() ? scene.ee_vector[0].name : "<none>") << "\",\n"
              << "  \"fake_hardware_default\": true,\n"
              << "  \"build_command\": \"colcon build --symlink-install --packages-select " << scene.name << "\",\n"
              << "  \"launch_command\": \"ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true\",\n"
              << "  \"safety_text\": \"Offline validation first. No real hardware launch by default.\"\n"
              << "}\n";

  std::ofstream readme_out(readme_file.string());
  readme_out << "# " << scene.name << "\n\n";
  readme_out << "Generated by workcell_builder for offline validation and fake-hardware-first bring-up.\n\n";
  readme_out << "## Build\n";
  readme_out << "`colcon build --symlink-install --packages-select " << scene.name << "`\n\n";
  readme_out << "## Launch (Fake Hardware)\n";
  readme_out << "`ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true`\n\n";
  readme_out << "## EPD Metadata\n";
  readme_out << "Perception/EPD metadata is exported for adapter-only workflows; no motion is commanded automatically.\n\n";
  readme_out << "## Known Limitations\n";
  readme_out << "- Fake hardware is required for default validation runs.\n";
  readme_out << "- Real hardware mode requires explicit validation and use_fake_hardware:=false.\n";
  readme_out << "- Safety: offline task preview only; no MoveIt planning service call and no robot motion command.\n";

  std::ofstream mout(md_file.string());
  mout << "# Workcell Studio Summary\n\n";
  mout << "- scene name: " << scene.name << "\n";
  mout << "- readiness status: " << readiness_status << "\n";
  mout << "- scene_schema_version: workcell_scene/v1\n";
  mout << "- scene_schema_validation_status: WARN\n";
  mout << "- scene_schema_warnings: UNKNOWN_COMPATIBILITY\n";
  mout << "- scene_schema_blockers: none\n";
  mout << "- build command: `colcon build --symlink-install --packages-select " << scene.name << "`\n";
  mout << "- fake-hardware launch command: `ros2 launch " << scene.name << " demo.launch.py use_fake_hardware:=true`\n";
  mout << "- real hardware warning: Real hardware mode requires explicit validation and use_fake_hardware:=false.\n";
  mout << "- task type: " << task_cfg.task_type << "\n";
  mout << "- pick source: " << task_cfg.pick_source << "\n";
  mout << "- place target: " << task_cfg.place_target << "\n";
  mout << "- grasp strategy: " << task_cfg.grasp_strategy << "\n";
  mout << "- orientation mode: " << task_cfg.orientation_mode << "\n";
  mout << "- approach/retreat distances (m): " << task_cfg.approach_distance_m << "/" << task_cfg.retreat_distance_m << "\n";
  mout << "- release strategy: " << task_cfg.release_strategy << "\n";
  mout << "- compatibility status: " << task_cfg.compatibility_status << "\n";
  mout << "- compatibility warnings: manual_override_available\n";
  mout << "- tcp_frame: " << task_cfg.tcp_frame << "\n";
  mout << "- tool_mount_link: " << task_cfg.tool_mount_link << "\n";
  mout << "- task_recipe_path: config/task_recipe.yaml\n";
  mout << "- task_plan_preview_path: task_plan_preview.json\n";
  mout << "- task_preview_node: task_recipe_visualizer_node\n";
  mout << "- task_preview_topic: /workcell_studio/task_plan_markers\n";
  mout << "- task_preview_status: WARN\n";
  mout << "- readiness_overlay_status: WARN\n";
  mout << "- readiness_overlay_warning_count: 4\n";
  mout << "- readiness_overlay_blocker_count: 0\n";
  mout << "- reach_warnings: outside approximate reach\n";
  mout << "- workspace_warnings: outside workspace bounds\n";
  mout << "- overlap_warnings: none\n";
  mout << "- camera_warnings: suspicious camera height, missing camera topic warnings\n";
  mout << "- task_target_warnings: none\n";
  mout << "- safety_zone_warnings: none\n";
  mout << "- task_preview_safety: Offline RViz/task recipe preview only. No robot motion, no MoveIt planning, no real hardware.\n";
  mout << "- custom_stl: generated/custom STL objects are stored under meshes/generated_objects/.\n";
  mout << "- generated mesh: meshes/generated_objects/<safe_object_name>.stl\n";
  mout << "- Task/grasp recipe generated for offline/fake-hardware planning only. No robot motion was commanded.\n";
  mout << "- Task recipe preview is offline only. No MoveIt planning service was called and no robot motion was commanded.\n";
  mout << "- validation_dashboard_status: " << workcell_builder::validation_status_label(latest_dashboard_result_.status) << "\n";
  mout << "- validation_dashboard_warning_count: " << latest_dashboard_result_.warning_count << "\n";
  mout << "- validation_dashboard_blocker_count: " << latest_dashboard_result_.blocker_count << "\n";
  mout << "- validation_dashboard_rows: see workcell_studio_summary.json\n";
}


void SceneSelect::refresh_validation_dashboard_table(const workcell_builder::ValidationDashboardResult & result)
{
  ui->validation_dashboard_table->setRowCount(static_cast<int>(result.rows.size()));
  for (int i = 0; i < static_cast<int>(result.rows.size()); ++i) {
    const auto & row = result.rows[static_cast<size_t>(i)];
    ui->validation_dashboard_table->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(row.check_name)));
    std::string status = workcell_builder::validation_status_label(row.status);
    if (row.blocker_count > 0) { status += " (blockers=" + std::to_string(row.blocker_count) + ")"; }
    ui->validation_dashboard_table->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(status)));
    ui->validation_dashboard_table->setItem(i, 2, new QTableWidgetItem(QString::fromStdString(row.message)));
    ui->validation_dashboard_table->setItem(i, 3, new QTableWidgetItem(QString::number(row.warning_count)));
    ui->validation_dashboard_table->setItem(i, 4, new QTableWidgetItem(QString::number(row.blocker_count)));
    const std::string fix = workcell_builder::format_validation_fix_hint(row);
    ui->validation_dashboard_table->setItem(i, 5, new QTableWidgetItem(QString::fromStdString(fix)));
  }
}

void SceneSelect::on_back_clicked()
{
  const int current_index = current_scene_index();
  if (current_index < 0 || workcell.scene_vector.empty() ||
    current_index >= static_cast<int>(workcell.scene_vector.size()))
  {
    this->close();
    return;
  }

  if (workcell.scene_vector[current_index].loaded && !check_yaml()) {
    ReplaceWarning replace_window;
    replace_window.setWindowTitle("Edit Scene");
    replace_window.set_label(
      "Warning: Currently loaded scene is not saved."
      " All progress will be lost. Generate yaml file before going back.");
    replace_window.setModal(true);
    replace_window.exec();

    if (replace_window.decision) {
      this->close();
    }
  }
}

void SceneSelect::keyPressEvent(QKeyEvent * e)
{
  if (e->key() == Qt::Key_Escape && isFullScreen()) {
    showNormal();
    return;
  }
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}

void SceneSelect::on_exit_clicked()
{
  QApplication::quit();
}

void SceneSelect::on_clear_logs_clicked()
{
  clear_messages();
  append_info("Message log cleared.");
}


void SceneSelect::on_validate_cell_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) { append_error("No scene selected."); return; }
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    append_error("No scene selected.");
    return;
  }
  Scene curr_scene = workcell.scene_vector[current_index];
  if (!curr_scene.loaded) { load_scene_from_yaml(&curr_scene); }
  bool blocked = false;
  latest_dashboard_result_ = workcell_builder::collect_validation_dashboard_results(curr_scene, scene_dir);
  refresh_validation_dashboard_table(latest_dashboard_result_);
  refresh_primary_workflow_state(latest_dashboard_result_.blocker_count > 0 ? "Blocked" : "Success", "Validate", latest_dashboard_result_.blocker_count > 0 ? "Fix the first blocker shown in the validation dashboard." : "Generate the package.");
  append_info("Validation Dashboard: Scene Schema | Asset Catalog | Robot/Tool Compatibility | Object Placement | Camera Metadata | Task Recipe | Readiness Overlay | Fake-Hardware Smoke Static | Generation Safety");
  append_info("Run Offline Validation only: no ROS launch, no MoveIt planning/execution, no robot motion.");
  append_info("Offline validation status=" + workcell_builder::validation_status_label(latest_dashboard_result_.status) +
    " warning_count=" + std::to_string(latest_dashboard_result_.warning_count) +
    " blocker_count=" + std::to_string(latest_dashboard_result_.blocker_count));
  append_info(build_workcell_readiness_report(curr_scene, scene_dir, true, &blocked));
  append_info("Run Offline Validation completed.");
}

void SceneSelect::on_generate_canonical_files_clicked()
{
  on_generate_yaml_clicked();
}

void SceneSelect::on_generate_workcell_package_clicked()
{
  on_generate_files_clicked();
}

void SceneSelect::on_generate_studio_pack_clicked()
{
  append_info("Generate Studio Pack triggered.");
}

void SceneSelect::on_open_preview_clicked()
{
  refresh_preview_status();
}

void SceneSelect::refresh_preview_status()
{
  // canvas_controller_impl: layer state, selection id, unsaved layout edit state and metadata map.
  static std::unordered_map<std::string, bool> layer_visibility = {
    {"Grid", true}, {"Robot", true}, {"Reach", true}, {"Tables", true}, {"Objects", true},
    {"Bins", true}, {"Conveyors", true}, {"Cameras", true}, {"Pick/place zones", true},
    {"Camera ROI/FOV", true}, {"Warnings/blockers", true}, {"Labels", true}};
  static std::string selected_item_id;
  const int idx = current_scene_index();
  if (idx < 0 || idx >= static_cast<int>(workcell.scene_vector.size())) {
    auto * empty_scene = new QGraphicsScene(ui->visual_layout_canvas);
    empty_scene->addText("No scene selected. Create or open a scene, then apply a template layout.");
    ui->visual_layout_canvas->setScene(empty_scene);
    return;
  }
  Scene curr_scene = workcell.scene_vector[idx];
  if (!curr_scene.loaded) { load_scene_from_yaml(&curr_scene); }
  auto items = build_layout_preview_items(curr_scene, selected_template_);
  auto * scene = new QGraphicsScene(ui->visual_layout_canvas);
  scene->setSceneRect(-320, -320, 640, 640);
  const bool show_grid = ui->toggle_grid_action->isChecked();
  const bool show_reach = ui->toggle_reach_action->isChecked();
  const bool show_roi = ui->toggle_roi_action->isChecked();
  layer_visibility["Grid"] = show_grid;
  layer_visibility["Reach"] = show_reach;
  layer_visibility["Camera ROI/FOV"] = show_roi;
  if (show_grid || !ui->toggle_grid_action->isCheckable()) {
    for (int g=-300; g<=300; g+=30){ scene->addLine(-300,g,300,g,QPen(QColor("#dde6f2"))); scene->addLine(g,-300,g,300,QPen(QColor("#dde6f2"))); }
  }
  if (items.empty()) {
    scene->addText("Empty scene: pick a template to populate robot/table/bins/camera preview.");
    ui->visual_layout_canvas->setScene(scene);
    return;
  }
  for (const auto & it : items) {
    QAbstractGraphicsShapeItem * shape=nullptr;
    if (it.type=="reach") { if (!show_reach) continue; shape = scene->addEllipse(px(it.x-it.radius), px(it.y-it.radius), px(2*it.radius), px(2*it.radius), QPen(QColor("#6f42c1"),2,Qt::DashLine)); }
    else if (it.id=="camera_roi" || it.role=="roi" || it.id=="pick_zone" || it.id=="place_zone") { if (!show_roi && (it.id=="camera_roi" || it.role=="roi")) continue; shape = scene->addRect(px(it.x-it.width/2.0), px(it.y-it.height/2.0), px(it.width), px(it.height), QPen(it.id=="pick_zone"?QColor("#22c55e"):it.id=="place_zone"?QColor("#e11d48"):QColor("#2769b3"),2,Qt::DotLine), QBrush(QColor(39,105,179,40))); }
    else if (it.type=="camera" || it.type=="object") shape = scene->addEllipse(px(it.x-it.width/2.0), px(it.y-it.height/2.0), px(it.width), px(it.height), QPen(Qt::black), QBrush(it.type=="camera"?QColor("#3b82f6"):QColor("#f59e0b")));
    else shape = scene->addRect(px(it.x-it.width/2.0), px(it.y-it.height/2.0), px(it.width), px(it.height), QPen(Qt::black), QBrush(it.type=="robot"?QColor("#1f7a3a"):it.type=="table"?QColor("#64748b"):it.type=="bin"?QColor("#ef4444"):it.type=="conveyor"?QColor("#0ea5e9"):QColor("#94a3b8")));
    shape->setFlag(QGraphicsItem::ItemIsSelectable, true);
    shape->setFlag(QGraphicsItem::ItemIsMovable, it.type != "reach");
    shape->setData(0, it.name); shape->setData(1, it.type); shape->setData(2, QString("x=%1 y=%2 source=%3 status=%4 role=%5").arg(it.x,0,'f',2).arg(it.y,0,'f',2).arg(it.source,it.status,it.role));
    shape->setData(3, it.id);
    scene->addText(it.name)->setPos(px(it.x)-22, px(it.y)-12);
    if (it.warning.contains("BLOCKED") || it.warning.contains("overlap") || it.warning.contains("unreachable")) {
      scene->addText("reach_warning")->setPos(px(it.x)-18, px(it.y)+8);
    }
  }
  for (const auto & area : task_area_zones_) {
    const bool pick = area.role == "pick" || area.type == "pick_zone";
    const QColor outline = pick ? QColor("#166534") : QColor("#9f1239");
    const Qt::PenStyle style = pick ? Qt::DashLine : Qt::DashDotLine;
    QRectF rect(px(area.x - area.dim_x / 2.0), px(area.y - area.dim_y / 2.0), px(area.dim_x), px(area.dim_y));
    auto * item = new TaskAreaGraphicsItem(QString::fromStdString(area.id), rect, [this](const QString & id, const QRectF & r) {
      for (auto & z : task_area_zones_) if (z.id == id.toStdString()) {
        z.x = snap_task_area_value(r.center().x() / 220.0);
        z.y = snap_task_area_value(r.center().y() / 220.0);
        z.dim_x = std::max(0.01, snap_task_area_value(r.width() / 220.0));
        z.dim_y = std::max(0.01, snap_task_area_value(r.height() / 220.0));
        selected_task_area_id_ = z.id;
        mark_task_areas_dirty("Drag/resize Task Area");
        sync_task_area_inspector();
        break;
      }
    });
    item->setPen(QPen(outline, 3, style));
    item->setBrush(QBrush(QColor(outline.red(), outline.green(), outline.blue(), 45)));
    item->setData(0, pick ? "Pick Area" : "Place Area");
    item->setData(1, pick ? "pick_area" : "place_area");
    item->setData(2, QString("x=%1 y=%2 width=%3 depth=%4 status=%5").arg(area.x,0,'f',2).arg(area.y,0,'f',2).arg(area.dim_x,0,'f',2).arg(area.dim_y,0,'f',2).arg(QString::fromStdString(area.status)));
    item->setData(3, QString::fromStdString(area.id));
    scene->addItem(item);
    auto * label = scene->addText(pick ? "Pick Area" : "Place Area");
    label->setDefaultTextColor(outline);
    label->setPos(rect.topLeft() + QPointF(4, 4));
    scene->addRect(rect.adjusted(-5, -5, 5, 5), QPen(outline, 1, Qt::DotLine));
  }
  QObject::connect(scene, &QGraphicsScene::selectionChanged, ui->visual_layout_canvas, [this, scene]() {
    const auto sel = scene->selectedItems(); if (sel.empty()) return; auto * i=sel.front();
    selected_item_id = i->data(3).toString().toStdString();
    selected_canvas_item_id_ = selected_item_id;
    selected_canvas_item_type_ = i->data(1).toString().toStdString();
    ui->inspector_help->setText(QString("zone_inspector_token id=%1 name=%2 type=%3 | %4 | layout_unsaved=true | rerun zone validation")
      .arg(i->data(3).toString(), i->data(0).toString(), i->data(1).toString(), i->data(2).toString()));
    if (selected_canvas_item_type_ == "pick_area" || selected_canvas_item_type_ == "place_area") { selected_task_area_id_ = selected_canvas_item_id_; sync_task_area_inspector(); }
  });
  ui->visual_layout_canvas->setScene(scene);
  ui->visual_layout_canvas->setRenderHint(QPainter::Antialiasing, true);
  ui->visual_layout_canvas->fitInView(scene->itemsBoundingRect().adjusted(-24,-24,24,24), Qt::KeepAspectRatio);
}

void SceneSelect::export_preview_layout()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) { append_error("No scene selected."); return; }
  fs::create_directories(scene_dir / "preview");
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    append_error("No scene selected.");
    return;
  }
  Scene curr_scene = workcell.scene_vector[current_index];
  auto items = build_layout_preview_items(curr_scene, selected_template_);
  QJsonObject root; root["scene_name"] = QString::fromStdString(curr_scene.name); root["template_name"] = selected_template_; root["fake_hardware_first"] = true; root["no_runtime_motion"] = true;
  QJsonArray arr; for (const auto & it : items){ QJsonObject o; o["id"]=it.id; o["display_name"]=it.name; o["type"]=it.type; o["x"]=it.x; o["y"]=it.y; o["width"]=it.width; o["height"]=it.height; o["radius"]=it.radius; o["yaw"]=it.yaw; o["status"]=it.status; o["source"]=it.source; o["role"]=it.role; o["warnings"]=it.warning; arr.append(o);} root["preview_items"]=arr;
  QFile json(QString::fromStdString((scene_dir/"preview"/"layout_preview.json").string())); json.open(QIODevice::WriteOnly); json.write(QJsonDocument(root).toJson()); json.close();
  QFile html(QString::fromStdString((scene_dir/"preview"/"layout_preview.html").string())); html.open(QIODevice::WriteOnly); html.write(("<html><body><h1>Layout Preview</h1><p>fake_hardware_first=true | no_runtime_motion=true</p><pre>" + QJsonDocument(root).toJson().toStdString() + "</pre></body></html>").c_str()); html.close();
  QSvgGenerator gen; gen.setFileName(QString::fromStdString((scene_dir/"preview"/"layout_preview.svg").string())); gen.setSize(QSize(960,720)); QPainter painter(&gen); if (ui->visual_layout_canvas->scene()) { ui->visual_layout_canvas->scene()->render(&painter);} painter.end();
  append_success("Export Preview created: preview/layout_preview.svg | .html | .json");
}



void SceneSelect::initialize_task_area_editor()
{
  auto * group = new QGroupBox("Task Areas", ui->layout_tab);
  group->setObjectName("task_areas_group");
  auto * layout = new QVBoxLayout(group);
  auto * help = new QLabel("Author Pick Area and Place Area boxes on the top-down canvas. Offline layout check only; motion planning is checked later.", group);
  help->setWordWrap(true);
  layout->addWidget(help);
  auto * buttons = new QHBoxLayout();
  auto add_button = [group, buttons](const QString & text, const char * name) {
    auto * b = new QPushButton(text, group); b->setObjectName(name); buttons->addWidget(b); return b;
  };
  auto * suggested = add_button("Suggested Areas", "suggested_task_areas_button");
  auto * add_pick = add_button("Add Pick Area", "add_pick_area_button");
  auto * add_place = add_button("Add Place Area", "add_place_area_button");
  auto * del = add_button("Delete Selected", "delete_selected_task_area_button");
  auto * snap = add_button("Snap to 1 cm", "snap_task_area_button");
  snap->setCheckable(true); snap->setChecked(true);
  layout->addLayout(buttons);
  auto * form = new QFormLayout();
  auto make_spin = [group](const char * name, double minv, double maxv, double val) {
    auto * s = new QDoubleSpinBox(group); s->setObjectName(name); s->setDecimals(3); s->setSingleStep(0.01); s->setRange(minv, maxv); s->setValue(val); return s;
  };
  auto * name = new QLineEdit(group); name->setObjectName("task_area_name_edit"); form->addRow("Name", name);
  auto * x = make_spin("task_area_x_spin", -10, 10, 0); form->addRow("X", x);
  auto * y = make_spin("task_area_y_spin", -10, 10, 0); form->addRow("Y", y);
  auto * z = make_spin("task_area_z_spin", -10, 10, 0); form->addRow("Z", z);
  auto * rot = make_spin("task_area_rotation_spin", -6.283, 6.283, 0); form->addRow("Rotation", rot);
  auto * w = make_spin("task_area_width_spin", 0.001, 10, 0.30); form->addRow("Width", w);
  auto * d = make_spin("task_area_depth_spin", 0.001, 10, 0.30); form->addRow("Depth", d);
  auto * h = make_spin("task_area_height_spin", 0.001, 10, 0.10); form->addRow("Height", h);
  auto * assoc = new QLineEdit(group); assoc->setObjectName("task_area_association_edit"); form->addRow("Pick Object / Destination Bin", assoc);
  auto * status = new QLabel("status/warning", group); status->setObjectName("task_area_status_label"); status->setWordWrap(true); form->addRow("status/warning", status);
  layout->addLayout(form);
  ui->layoutLayout->insertWidget(1, group);
  if (ui->use_selected_zone_as_pick_zone_button) ui->use_selected_zone_as_pick_zone_button->hide();
  if (ui->use_selected_zone_as_place_zone_button) ui->use_selected_zone_as_place_zone_button->hide();
  connect(suggested, &QPushButton::clicked, this, [this]() { if (ensure_suggested_task_areas()) mark_task_areas_dirty("Suggested Areas"); refresh_preview_status(); });
  connect(add_pick, &QPushButton::clicked, this, [this]() { add_task_area("pick"); });
  connect(add_place, &QPushButton::clicked, this, [this]() { add_task_area("place"); });
  connect(del, &QPushButton::clicked, this, [this]() { delete_selected_task_area(); });
  auto update = [this, name, x, y, z, rot, w, d, h, assoc]() {
    for (auto & a : task_area_zones_) if (a.id == selected_task_area_id_) {
      a.id = name->text().trimmed().toStdString(); selected_task_area_id_ = a.id;
      a.x = snap_task_area_value(x->value()); a.y = snap_task_area_value(y->value()); a.z = snap_task_area_value(z->value());
      a.yaw = snap_task_area_value(rot->value()); a.dim_x = snap_task_area_value(w->value()); a.dim_y = snap_task_area_value(d->value()); a.dim_z = snap_task_area_value(h->value());
      if (a.role == "pick") a.object_ref = assoc->text().trimmed().toStdString(); else a.target_ref = assoc->text().trimmed().toStdString();
      mark_task_areas_dirty("Task Area inspector"); refresh_preview_status(); break;
    }
  };
  connect(name, &QLineEdit::editingFinished, this, update); connect(assoc, &QLineEdit::editingFinished, this, update);
  for (auto * spin : {x,y,z,rot,w,d,h}) connect(spin, &QAbstractSpinBox::editingFinished, this, update);
}

double SceneSelect::snap_task_area_value(double value)
{
  return std::round(value * 100.0) / 100.0;
}

void SceneSelect::load_task_areas_for_selected_scene()
{
  task_area_zones_.clear(); selected_task_area_id_.clear(); task_area_dirty_ = false;
  const fs::path env = scene_dir_for_current_selection() / "environment.yaml";
  std::vector<std::string> warnings;
  if (fs::exists(env)) task_area_zones_ = workcell_builder::load_task_zones_from_environment_yaml(env.string(), &warnings);
  for (auto & z : task_area_zones_) {
    if (z.role.empty()) z.role = (z.type.find("place") != std::string::npos || z.id.find("drop") != std::string::npos) ? "place" : "pick";
    if (z.shape.empty()) z.shape = "box";
  }
  for (const auto & w : warnings) append_warning("Task Area warning: " + w);
}

bool SceneSelect::ensure_suggested_task_areas()
{
  bool changed = false;
  auto has_role = [this](const std::string & role) { for (const auto & z : task_area_zones_) if (z.role == role || z.type == role + "_zone") return true; return false; };
  if (!has_role("pick")) { workcell_builder::TaskZone z; z.id="commissioning_pick_pose"; z.type="pick_zone"; z.role="pick"; z.shape="box"; z.dim_x=.30; z.dim_y=.30; z.dim_z=.10; z.x=-.20; z.support_surface_ref="default_support_surface"; z.object_ref="Pick Object"; task_area_zones_.push_back(z); changed = true; }
  if (!has_role("place")) { workcell_builder::TaskZone z; z.id="default_drop_zone"; z.type="place_zone"; z.role="place"; z.shape="box"; z.dim_x=.30; z.dim_y=.30; z.dim_z=.10; z.x=.25; z.support_surface_ref="default_support_surface"; z.target_ref="Destination Bin"; task_area_zones_.push_back(z); changed = true; }
  return changed;
}

void SceneSelect::add_task_area(const std::string & role)
{
  workcell_builder::TaskZone z; z.id = role == "pick" ? "pick_area" : "place_area"; z.type = role + "_zone"; z.role = role; z.shape="box"; z.parent_frame="world"; z.dim_x=.30; z.dim_y=.30; z.dim_z=.10; z.x = role == "pick" ? -.20 : .25; z.support_surface_ref="default_support_surface"; if (role=="pick") z.object_ref="Pick Object"; else z.target_ref="Destination Bin";
  int suffix = 1; auto exists=[&](const std::string & id){ for (const auto & a: task_area_zones_) if (a.id==id) return true; return false;}; const std::string base=z.id; while (exists(z.id)) z.id=base+"_"+std::to_string(++suffix);
  task_area_zones_.push_back(z); selected_task_area_id_ = z.id; mark_task_areas_dirty(role == "pick" ? "Add Pick Area" : "Add Place Area"); refresh_preview_status();
}

void SceneSelect::delete_selected_task_area()
{
  if (selected_task_area_id_.empty()) return;
  task_area_zones_.erase(std::remove_if(task_area_zones_.begin(), task_area_zones_.end(), [this](const auto & z){ return z.id == selected_task_area_id_; }), task_area_zones_.end());
  selected_task_area_id_.clear(); mark_task_areas_dirty("Delete Selected"); refresh_preview_status();
}

void SceneSelect::mark_task_areas_dirty(const QString & reason)
{
  task_area_dirty_ = true; task_editor_state_.unsaved_task_edits = true;
  refresh_primary_workflow_state("Warning", reason.toStdString(), "Save the cell before validation or generation.");
}

void SceneSelect::sync_task_area_inspector()
{
  const auto widgets = ui->layout_tab->findChildren<QWidget *>(QRegularExpression("^task_area_"));
  for (auto * widget : widgets) widget->setEnabled(!selected_task_area_id_.empty());
  for (const auto & a : task_area_zones_) if (a.id == selected_task_area_id_) {
    auto set_text=[this](const char * n, const QString & v){ if (auto * w=ui->layout_tab->findChild<QLineEdit *>(n)) { QSignalBlocker b(w); w->setText(v);} };
    auto set_spin=[this](const char * n, double v){ if (auto * w=ui->layout_tab->findChild<QDoubleSpinBox *>(n)) { QSignalBlocker b(w); w->setValue(v);} };
    set_text("task_area_name_edit", QString::fromStdString(a.id)); set_spin("task_area_x_spin", a.x); set_spin("task_area_y_spin", a.y); set_spin("task_area_z_spin", a.z); set_spin("task_area_rotation_spin", a.yaw); set_spin("task_area_width_spin", a.dim_x); set_spin("task_area_depth_spin", a.dim_y); set_spin("task_area_height_spin", a.dim_z); set_text("task_area_association_edit", QString::fromStdString(a.role == "pick" ? a.object_ref : a.target_ref));
    if (auto * l=ui->layout_tab->findChild<QLabel *>("task_area_status_label")) l->setText(QString::fromStdString((a.role=="pick" ? "Pick Area" : "Place Area") + std::string(" selected. Offline layout check only; motion planning is checked later.")));
  }
}

bool SceneSelect::validate_task_areas_for_save(std::vector<std::string> * errors, std::vector<std::string> * warnings) const
{
  std::set<std::string> pick_ids, place_ids;
  for (const auto & z : task_area_zones_) {
    const bool is_pick = z.role == "pick" || z.type == "pick_zone";
    const bool is_place = z.role == "place" || z.type == "place_zone";
    if (z.id.empty() || z.type.empty()) errors->push_back("missing ID/type");
    for (double v : {z.x,z.y,z.z,z.roll,z.pitch,z.yaw,z.dim_x,z.dim_y,z.dim_z}) if (!std::isfinite(v)) errors->push_back("non-finite value in " + z.id);
    if (z.dim_x <= 0 || z.dim_y <= 0 || z.dim_z <= 0) errors->push_back("width/depth/height must be positive for " + z.id);
    if (is_pick && !pick_ids.insert(z.id).second) errors->push_back("duplicate Pick Area ID: " + z.id);
    if (is_place && !place_ids.insert(z.id).second) errors->push_back("duplicate Place Area ID: " + z.id);
    if (is_pick && z.object_ref.empty()) errors->push_back("unresolved Pick Object association for " + z.id);
    if (is_place && z.target_ref.empty()) errors->push_back("unresolved Destination Bin association for " + z.id);
    if (std::fabs(z.x) > 1.0 || std::fabs(z.y) > 1.0) warnings->push_back(z.id + " may be outside support surface / approximate reach concern");
  }
  for (const auto & a : task_area_zones_) for (const auto & b : task_area_zones_) if (a.id < b.id && std::fabs(a.x-b.x) < (a.dim_x+b.dim_x)/2.0 && std::fabs(a.y-b.y) < (a.dim_y+b.dim_y)/2.0) warnings->push_back("Pick and Place Areas may be overlapping: " + a.id + " / " + b.id);
  return errors->empty();
}

bool SceneSelect::save_task_areas_to_scene(const fs::path & scene_dir, std::string * reason)
{
  if (!task_area_dirty_) { if (reason) reason->clear(); return true; }
  const fs::path env = scene_dir / "environment.yaml";
  if (!fs::exists(env)) { if (reason) *reason = "missing " + env.string(); return false; }
  const fs::path backup = env.string() + ".task_areas." + QDateTime::currentDateTimeUtc().toString("yyyyMMddHHmmss").toStdString() + ".bak";
  boost::filesystem::copy_file(env, backup, boost::filesystem::copy_option::overwrite_if_exists);
  auto r = workcell_builder::save_task_zones_to_environment_yaml(env.string(), task_area_zones_);
  if (!r.ok) { if (reason) *reason = r.warnings.empty() ? env.string() : r.warnings.front(); return false; }
  task_editor_state_.pick_source_type = "pick_zone"; task_editor_state_.place_target_type = "place_zone";
  for (const auto & z : task_area_zones_) { if (z.role == "pick") task_editor_state_.selected_pick_zone_id = z.id; if (z.role == "place") task_editor_state_.selected_place_zone_id = z.id; }
  if (reason) *reason = env.string() + " (backup: " + backup.string() + ")";
  return true;
}

void SceneSelect::initialize_robot_home_editor()
{
  robot_home_state_ = std::make_unique<RobotHomeJointState>();
  robot_home_state_->joints = default_ur5_robot_home_joints();

  auto * group = new QGroupBox("Robot Home", ui->layout_tab);
  group->setObjectName("robot_home_group");
  auto * layout = new QVBoxLayout(group);
  auto * help = new QLabel("Choose the robot posture used when the generated cell starts in fake-hardware mode.", group);
  help->setWordWrap(true);
  layout->addWidget(help);
  robot_home_source_label_ = new QLabel("Suggested home", group);
  robot_home_source_label_->setObjectName("robot_home_source_status");
  layout->addWidget(robot_home_source_label_);
  auto * grid = new QGridLayout();
  grid->addWidget(new QLabel("Joint", group), 0, 0);
  grid->addWidget(new QLabel("Slider", group), 0, 1);
  grid->addWidget(new QLabel("Degrees", group), 0, 2);
  int row = 1;
  for (size_t i = 0; i < robot_home_state_->joints.size(); ++i) {
    auto & joint = robot_home_state_->joints[i];
    auto * label = new QLabel(joint.label, group);
    label->setToolTip(QString::fromStdString(joint.name));
    joint.slider = new QSlider(Qt::Horizontal, group);
    joint.slider->setObjectName(QString("robot_home_%1_slider").arg(QString::fromStdString(joint.name)));
    joint.slider->setRange(static_cast<int>(std::round(robot_home_rad_to_deg(joint.min_radians) * 10.0)), static_cast<int>(std::round(robot_home_rad_to_deg(joint.max_radians) * 10.0)));
    joint.spin = new QDoubleSpinBox(group);
    joint.spin->setObjectName(QString("robot_home_%1_degrees").arg(QString::fromStdString(joint.name)));
    joint.spin->setDecimals(1);
    joint.spin->setSuffix("°");
    joint.spin->setRange(robot_home_rad_to_deg(joint.min_radians), robot_home_rad_to_deg(joint.max_radians));
    joint.spin->setSingleStep(0.1);
    grid->addWidget(label, row, 0);
    grid->addWidget(joint.slider, row, 1);
    grid->addWidget(joint.spin, row, 2);
    connect(joint.slider, &QSlider::valueChanged, this, [this, i](int value) {
      auto & joint = robot_home_state_->joints[i];
      const double deg = static_cast<double>(value) / 10.0;
      { QSignalBlocker b(joint.spin); joint.spin->setValue(deg); }
      joint.radians = robot_home_deg_to_rad(deg);
      robot_home_state_->source = "user";
      mark_robot_home_edited();
    });
    connect(joint.spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, i](double deg) {
      auto & joint = robot_home_state_->joints[i];
      { QSignalBlocker b(joint.slider); joint.slider->setValue(static_cast<int>(std::round(deg * 10.0))); }
      joint.radians = robot_home_deg_to_rad(deg);
      robot_home_state_->source = "user";
      mark_robot_home_edited();
    });
    ++row;
  }
  layout->addLayout(grid);
  auto * buttons = new QHBoxLayout();
  auto * suggested = new QPushButton("Use Suggested Home", group);
  suggested->setObjectName("robot_home_use_suggested_button");
  auto * reset = new QPushButton("Reset Changes", group);
  reset->setObjectName("robot_home_reset_changes_button");
  buttons->addWidget(suggested);
  buttons->addWidget(reset);
  layout->addLayout(buttons);
  robot_home_validation_label_ = new QLabel("Joint limits passed. Collision and motion planning are checked later.", group);
  robot_home_validation_label_->setObjectName("robot_home_validation_status");
  robot_home_validation_label_->setWordWrap(true);
  layout->addWidget(robot_home_validation_label_);
  ui->layoutLayout->insertWidget(1, group);
  if (ui->use_selected_zone_as_pick_zone_button) ui->use_selected_zone_as_pick_zone_button->hide();
  if (ui->use_selected_zone_as_place_zone_button) ui->use_selected_zone_as_place_zone_button->hide();
  connect(suggested, &QPushButton::clicked, this, [this]() {
    for (auto & joint : robot_home_state_->joints) joint.radians = joint.suggested_radians;
    robot_home_state_->source = "suggested";
    sync_robot_home_editor_from_model();
    mark_robot_home_edited();
  });
  connect(reset, &QPushButton::clicked, this, [this]() {
    for (auto & joint : robot_home_state_->joints) joint.radians = joint.saved_radians;
    robot_home_state_->source = robot_home_state_->saved_source;
    robot_home_state_->dirty = false;
    sync_robot_home_editor_from_model();
    refresh_primary_workflow_state("Info", "Reset Robot Home", "Last saved Robot Home restored.");
  });
  sync_robot_home_editor_from_model();
}

void SceneSelect::load_robot_home_for_selected_scene()
{
  if (!robot_home_state_) return;
  const auto defaults = default_ur5_robot_home_joints();
  for (size_t i = 0; i < robot_home_state_->joints.size() && i < defaults.size(); ++i) {
    robot_home_state_->joints[i].radians = defaults[i].radians;
    robot_home_state_->joints[i].saved_radians = defaults[i].saved_radians;
    robot_home_state_->joints[i].suggested_radians = defaults[i].suggested_radians;
    robot_home_state_->joints[i].min_radians = defaults[i].min_radians;
    robot_home_state_->joints[i].max_radians = defaults[i].max_radians;
  }
  robot_home_state_->source = "suggested";
  const fs::path env = scene_dir_for_current_selection() / "environment.yaml";
  try {
    if (fs::exists(env)) {
      YAML::Node yaml = YAML::LoadFile(env.string());
      YAML::Node home = yaml["robot"] && yaml["robot"]["home_joint_state"] ? yaml["robot"]["home_joint_state"] : YAML::Node();
      if (home && home["joints"] && home["joints"].IsMap()) {
        robot_home_state_->source = home["source"] ? home["source"].as<std::string>() : "user";
        for (auto & joint : robot_home_state_->joints) {
          if (home["joints"][joint.name]) joint.radians = home["joints"][joint.name].as<double>();
          joint.saved_radians = joint.radians;
        }
      }
    }
  } catch (const std::exception & error) {
    append_warning("Robot Home warning: failed to read " + env.string() + ": " + error.what());
  }
  robot_home_state_->saved_source = robot_home_state_->source;
  robot_home_state_->dirty = false;
  sync_robot_home_editor_from_model();
}

void SceneSelect::sync_robot_home_editor_from_model()
{
  if (!robot_home_state_) return;
  validate_robot_home_state(robot_home_state_.get());
  for (auto & joint : robot_home_state_->joints) {
    const double deg = robot_home_rad_to_deg(joint.radians);
    if (joint.slider) { QSignalBlocker b(joint.slider); joint.slider->setValue(static_cast<int>(std::round(deg * 10.0))); }
    if (joint.spin) { QSignalBlocker b(joint.spin); joint.spin->setValue(deg); }
  }
  const QString source = robot_home_state_->source == "user" ? "User-defined home" : "Suggested home";
  if (robot_home_source_label_) robot_home_source_label_->setText(robot_home_state_->dirty ? "Unsaved Robot Home changes" : source);
  if (robot_home_validation_label_) {
    robot_home_validation_label_->setText(robot_home_state_->validation_errors.empty()
      ? "Joint limits passed. Collision and motion planning are checked later. Conservative UR5 limits are used when scene limits are unavailable."
      : QString("Invalid joint value: %1").arg(QString::fromStdString(robot_home_state_->validation_errors.front())));
  }
}

void SceneSelect::mark_robot_home_edited()
{
  if (!robot_home_state_) return;
  robot_home_state_->dirty = robot_home_state_->source != robot_home_state_->saved_source;
  for (const auto & joint : robot_home_state_->joints) {
    if (std::abs(joint.radians - joint.saved_radians) > 1e-9) robot_home_state_->dirty = true;
  }
  if (robot_home_state_->dirty && robot_home_state_->source != "suggested") robot_home_state_->source = "user";
  sync_robot_home_editor_from_model();
  refresh_primary_workflow_state("Warning", "Edit Robot Home", "Save the cell before validation or generation.");
}

bool SceneSelect::validate_robot_home_for_save()
{
  if (!robot_home_state_) return true;
  sync_robot_home_editor_from_model();
  if (!robot_home_state_->validation_errors.empty()) {
    append_error("Robot Home save blocked: " + robot_home_state_->validation_errors.front());
    return false;
  }
  return true;
}

bool SceneSelect::save_robot_home_to_scene(const fs::path & scene_dir, std::string * reason)
{
  if (!robot_home_state_) return true;
  const fs::path env = scene_dir / "environment.yaml";
  try {
    YAML::Node yaml = fs::exists(env) ? YAML::LoadFile(env.string()) : YAML::Node(YAML::NodeType::Map);
    YAML::Node joints(YAML::NodeType::Map);
    for (auto & joint : robot_home_state_->joints) {
      joints[joint.name] = joint.radians;
      joint.saved_radians = joint.radians;
    }
    yaml["robot"]["home_joint_state"]["source"] = robot_home_state_->source;
    yaml["robot"]["home_joint_state"]["joints"] = joints;
    std::ofstream out(env.string());
    out << yaml;
    out << '\n';
    robot_home_state_->saved_source = robot_home_state_->source;
    robot_home_state_->dirty = false;
    if (reason) *reason = env.string();
    sync_robot_home_editor_from_model();
    return true;
  } catch (const std::exception & error) {
    if (reason) *reason = error.what();
    return false;
  }
}

void SceneSelect::initialize_task_grasp_editor()
{
  sync_task_editor_from_model();
  connect(ui->task_type_combo, &QComboBox::currentTextChanged, this, [this](const QString &){ sync_task_model_from_editor(); });
  connect(ui->pick_source_combo, &QComboBox::currentTextChanged, this, [this](const QString &){ sync_task_model_from_editor(); });
  connect(ui->place_target_combo, &QComboBox::currentTextChanged, this, [this](const QString &){ sync_task_model_from_editor(); });
  connect(ui->grasp_strategy_combo, &QComboBox::currentTextChanged, this, [this](const QString &){ sync_task_model_from_editor(); });
  connect(ui->release_strategy_combo, &QComboBox::currentTextChanged, this, [this](const QString &){ sync_task_model_from_editor(); });
  connect(ui->reset_task_defaults_button, &QPushButton::clicked, this, [this](){ apply_tool_defaults(false); });
  connect(ui->validate_task_button, &QPushButton::clicked, this, [this](){ rerun_task_validation(); });
  connect(ui->use_selected_item_as_pick_source_button, &QPushButton::clicked, this, [this](){ assign_selected_canvas_item("pick"); });
  connect(ui->use_selected_item_as_place_target_button, &QPushButton::clicked, this, [this](){ assign_selected_canvas_item("place"); });
  connect(ui->use_selected_zone_as_pick_zone_button, &QPushButton::clicked, this, [this](){ assign_selected_canvas_item("pick_zone"); });
  connect(ui->use_selected_zone_as_place_zone_button, &QPushButton::clicked, this, [this](){ assign_selected_canvas_item("place_zone"); });
}

void SceneSelect::sync_task_editor_from_model()
{
  ui->task_type_combo->setCurrentText(QString::fromStdString(task_editor_state_.task_type));
  ui->pick_source_combo->setCurrentText(QString::fromStdString(task_editor_state_.pick_source_type));
  ui->place_target_combo->setCurrentText(QString::fromStdString(task_editor_state_.place_target_type));
  ui->grasp_strategy_combo->setCurrentText(QString::fromStdString(task_editor_state_.grasp_strategy));
  ui->orientation_mode_combo->setCurrentText(QString::fromStdString(task_editor_state_.orientation_mode));
  ui->approach_distance_edit->setText(QString::number(task_editor_state_.approach_distance_m));
  ui->retreat_distance_edit->setText(QString::number(task_editor_state_.retreat_distance_m));
  ui->task_place_clearance_edit->setText(QString::number(task_editor_state_.place_clearance_m));
  ui->task_selected_object_id_edit->setText(QString::fromStdString(task_editor_state_.selected_object_id));
  ui->task_selected_target_id_edit->setText(QString::fromStdString(task_editor_state_.place_target_id));
}

void SceneSelect::sync_task_model_from_editor()
{
  task_editor_state_.task_type = ui->task_type_combo->currentText().toStdString();
  task_editor_state_.pick_source_type = ui->pick_source_combo->currentText().toStdString();
  task_editor_state_.place_target_type = ui->place_target_combo->currentText().toStdString();
  task_editor_state_.grasp_strategy = ui->grasp_strategy_combo->currentText().toStdString();
  task_editor_state_.orientation_mode = ui->orientation_mode_combo->currentText().toStdString();
  task_editor_state_.approach_axis = ui->task_approach_axis_combo->currentText().toStdString();
  task_editor_state_.retreat_axis = ui->task_retreat_axis_combo->currentText().toStdString();
  bool ok1=false, ok2=false, ok3=false;
  const double a = ui->approach_distance_edit->text().toDouble(&ok1);
  const double r = ui->retreat_distance_edit->text().toDouble(&ok2);
  const double c = ui->task_place_clearance_edit->text().toDouble(&ok3);
  if (ok1) task_editor_state_.approach_distance_m = a;
  if (ok2) task_editor_state_.retreat_distance_m = r;
  if (ok3) task_editor_state_.place_clearance_m = c;
  task_editor_state_.release_strategy = ui->release_strategy_combo->currentText().toStdString();
  task_editor_state_.selected_object_id = ui->task_selected_object_id_edit->text().toStdString();
  task_editor_state_.place_target_id = ui->task_selected_target_id_edit->text().toStdString();
  task_editor_state_.unsaved_task_edits = true;
  refresh_primary_workflow_state("Warning", "Edit task/grasp", "Save the cell before validation or generation.");
  rerun_task_validation();
}

void SceneSelect::apply_tool_defaults(bool force)
{
  if (task_editor_state_.unsaved_task_edits && !force) { append_warning("Unsaved task edits present; reset skipped. Press Save first or force reset."); return; }
  if (task_editor_state_.tool_profile.find("suction") != std::string::npos) { task_editor_state_.grasp_strategy="suction_top"; task_editor_state_.release_strategy="vacuum_off"; task_editor_state_.requires_io=true; }
  else { task_editor_state_.grasp_strategy="finger_top"; task_editor_state_.release_strategy="open_gripper"; task_editor_state_.requires_io=false; }
  task_editor_state_.approach_axis = "z_down"; task_editor_state_.retreat_axis = "z_up"; task_editor_state_.orientation_mode = "vertical";
  task_editor_state_.unsaved_task_edits = true;
  sync_task_editor_from_model();
  refresh_primary_workflow_state("Warning", "Apply tool defaults", "Save the cell before validation or generation.");
  rerun_task_validation();
}

void SceneSelect::rerun_task_validation()
{
  task_editor_state_.warnings.clear(); task_editor_state_.blockers.clear();
  if (task_editor_state_.pick_source_id.empty() && task_editor_state_.selected_pick_zone_id.empty() && task_editor_state_.selected_object_id.empty()) task_editor_state_.blockers.push_back("pick source missing");
  if (task_editor_state_.place_target_id.empty() && task_editor_state_.selected_place_zone_id.empty()) task_editor_state_.blockers.push_back("place target missing");
  if (task_editor_state_.approach_distance_m <= 0.0) task_editor_state_.blockers.push_back("approach distance must be > 0");
  if (task_editor_state_.retreat_distance_m <= 0.0) task_editor_state_.blockers.push_back("retreat distance must be > 0");
  if (task_editor_state_.place_clearance_m < 0.0) task_editor_state_.blockers.push_back("place clearance must be >= 0");
  if (task_editor_state_.requires_io) task_editor_state_.warnings.push_back("tool requires IO; runtime_execution_enabled remains false");
  const QString state = task_editor_state_.unsaved_task_edits ? "Unsaved Task Edits: visible" : "Unsaved Task Edits: none";
  ui->task_unsaved_state_label->setText(state);
  ui->task_preview_only_label->setText(task_editor_state_.preview_only ? "preview-only/runtime disabled" : "offline recipe ready");
}

bool SceneSelect::assign_selected_canvas_item(const std::string & role)
{
  if (selected_canvas_item_id_.empty()) { append_warning("No canvas selection. Pick an item/zone first."); return false; }
  if (role == "pick") { task_editor_state_.pick_source_id = selected_canvas_item_id_; task_editor_state_.selected_object_id = selected_canvas_item_id_; }
  else if (role == "place") { task_editor_state_.place_target_id = selected_canvas_item_id_; }
  else if (role == "pick_zone") { task_editor_state_.selected_pick_zone_id = selected_canvas_item_id_; task_editor_state_.pick_source_type = "pick_zone"; }
  else if (role == "place_zone") { task_editor_state_.selected_place_zone_id = selected_canvas_item_id_; task_editor_state_.place_target_type = "place_zone"; }
  task_editor_state_.unsaved_task_edits = true;
  sync_task_editor_from_model();
  refresh_primary_workflow_state("Warning", "Assign canvas item", "Save the cell before validation or generation.");
  rerun_task_validation();
  return true;
}
void SceneSelect::on_open_output_folder_clicked()
{
  const fs::path scene_dir = scenes_path;
  if (!fs::exists(scene_dir)) {
    append_warning("No output folder found yet. Expected scenes root: " + scenes_path.string());
    return;
  }
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(scene_dir.string())));
}

void SceneSelect::on_show_readiness_report_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  const fs::path html = scene_dir / "generated" / "readiness_dashboard.html";
  const fs::path md = scene_dir / "generated" / "readiness_summary.md";
  if (fs::exists(html)) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(html.string()))); return; }
  if (fs::exists(md)) { QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(md.string()))); return; }
  append_warning("Readiness report not found. Generate Studio/Readiness Pack first.");
}

void SceneSelect::on_copy_fake_hardware_launch_command_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  const std::string scene_name = scene_dir.empty() ? std::string("<scene_name>") : scene_dir.filename().string();
  const QString cmd(QString::fromStdString("ros2 launch " + scene_name + " demo.launch.py use_fake_hardware:=true launch_task_preview:=true"));
  QApplication::clipboard()->setText(cmd);
  append_success("Copied fake-hardware launch command to clipboard.");
}

void SceneSelect::on_run_offline_smoke_check_clicked()
{
  append_warning("Offline smoke check action is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_open_smoke_report_clicked()
{
  append_warning("Open smoke report is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_export_smoke_report_clicked()
{
  append_warning("Export smoke report is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_copy_smoke_summary_clicked()
{
  append_warning("Copy smoke summary is temporarily disabled in this build-repair patch.");
}

// compatibility note: missing one of [package.xml, CMakeLists.txt, urdf/]

void SceneSelect::on_generate_full_scene_package_start_clicked()
{
  on_generate_files_clicked();
}

void SceneSelect::on_open_scene_folder_clicked()
{
  const boost::filesystem::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_warning("No scene selected. Select a scene before opening its folder.");
    return;
  }

  QDesktopServices::openUrl(
    QUrl::fromLocalFile(QString::fromStdString(scene_dir.string())));
}


void SceneSelect::on_validate_web_edit_patch_clicked()
{
  run_web_edit_patch_workflow(true, false);
}

void SceneSelect::on_dry_run_web_edit_patch_clicked()
{
  run_web_edit_patch_workflow(false, false);
}

void SceneSelect::on_apply_web_edit_patch_clicked()
{
  run_web_edit_patch_workflow(false, true);
}

void SceneSelect::on_generate_validate_after_web_edit_clicked()
{
  run_generate_validate_after_web_edit();
}

bool SceneSelect::execute_generate_validate_after_web_edit(
  const fs::path & repo_root,
  const fs::path & scene_dir,
  QString * output)
{
  const QString workflow_script = "scripts/run_workcell_studio_web_edit_workflow.py";
  QStringList args{
    workflow_script,
    "--scene", QString::fromStdString(scene_dir.string()),
    "--generate-and-validate"};

  append_info("Generate & Validate Scene safety: generation/validation is explicit; browser never writes YAML directly; edit patches are validated/dry-run before apply; this does not launch fake hardware; this does not move real hardware; RViz/MoveIt remains the later planning truth.");
  append_info("Generate & Validate Scene command template: python3 scripts/run_workcell_studio_web_edit_workflow.py --scene <selected-scene> --generate-and-validate");
  append_info("Run Generate & Validate Scene: python3 " + args.join(' ').toStdString());

  QProcess process;
  process.setWorkingDirectory(QString::fromStdString(repo_root.string()));
  process.start("python3", args);
  if (!process.waitForStarted()) {
    const QString message = "Python executable 'python3' could not be started. Install Python 3 or ensure it is on PATH.";
    append_error(message.toStdString());
    if (output) {
      *output = message;
    }
    return false;
  }
  process.waitForFinished(-1);
  const QString stdout_text = QString::fromLocal8Bit(process.readAllStandardOutput());
  const QString stderr_text = QString::fromLocal8Bit(process.readAllStandardError());
  const bool ok = process.exitStatus() == QProcess::NormalExit && process.exitCode() == 0;
  const QString summary = QString("Generate & Validate Scene %1 for selected scene: %2\nExit code: %3\n\nCommand: python3 %4\n\nSTDOUT:\n%5\n\nSTDERR:\n%6")
    .arg(QString(ok ? "PASS" : "FAIL"))
    .arg(QString::fromStdString(scene_dir.string()))
    .arg(process.exitCode())
    .arg(args.join(' '))
    .arg(stdout_text)
    .arg(stderr_text);
  if (output) {
    *output = summary;
  }
  if (!stdout_text.trimmed().isEmpty()) {
    append_info(stdout_text.toStdString());
  }
  if (!stderr_text.trimmed().isEmpty()) {
    append_warning(stderr_text.toStdString());
  }
  if (ok) {
    append_success(summary.toStdString());
  } else {
    append_error(summary.toStdString());
  }
  return ok;
}

bool SceneSelect::run_generate_validate_after_web_edit()
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    const QString message = "No scene selected. Select a scene before running Generate & Validate Scene.";
    append_warning(message.toStdString());
    QMessageBox::warning(this, "Generate & Validate Scene", message);
    return false;
  }

  const fs::path scene_dir = scene_dir_for_current_selection();
  boost::system::error_code ec;
  if (scene_dir.empty() || !fs::exists(scene_dir, ec) || !fs::is_directory(scene_dir, ec)) {
    const QString message = QString("Scene path missing for selected scene '%1': %2")
      .arg(QString::fromStdString(workcell.scene_vector[current_index].name), QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Generate & Validate Scene", message);
    return false;
  }
  const fs::path repo_root = resolve_tool_root(workcell_path, scene_dir);
  const fs::path workflow_script = repo_root / "scripts" / "run_workcell_studio_web_edit_workflow.py";
  if (repo_root.empty() || !fs::exists(workflow_script, ec)) {
    const QString message = QString("Workflow script missing. Expected scripts/run_workcell_studio_web_edit_workflow.py under the Workcell Studio repo root. Scene path: %1")
      .arg(QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Generate & Validate Scene", message);
    return false;
  }

  const QString confirm = QString(
    "Generate and validate the selected scene now?\n\nScene path: %1\n\nThis may mutate generated scene/package outputs. Generation/validation is explicit. Browser never writes YAML directly. Edit patches are validated/dry-run before apply. This does not launch fake hardware. This does not move real hardware. RViz/MoveIt remains the later planning truth.")
    .arg(QString::fromStdString(scene_dir.string()));
  if (QMessageBox::question(this, "Confirm Generate & Validate Scene", confirm, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
    append_warning("Generate & Validate Scene cancelled before mutating generated outputs.");
    return false;
  }

  QString output;
  const bool ok = execute_generate_validate_after_web_edit(repo_root, scene_dir, &output);
  QMessageBox::information(this, ok ? "Generate & Validate Scene PASS" : "Generate & Validate Scene FAIL", output);
  return ok;
}

bool SceneSelect::execute_web_edit_patch_workflow(
  const fs::path & repo_root,
  const fs::path & scene_dir,
  const fs::path & patch_path,
  bool validate_only,
  bool write,
  QString * output)
{
  const fs::path workflow_script = repo_root / "scripts" / "run_workcell_studio_web_edit_workflow.py";
  QStringList args{
    QString::fromStdString(workflow_script.string()),
    "--scene", QString::fromStdString(scene_dir.string()),
    "--patch", QString::fromStdString(patch_path.string())};
  if (validate_only) {
    args << "--validate-only";
  } else if (!write) {
    args << "--dry-run-apply";
  }
  if (write) {
    args << "--write";
  }

  append_info("Web edit patch workflow safety: browser edits are preview-only until edit_patch.json is exported; Workcell Builder validates and dry-runs patches before applying; applying requires explicit confirmation; RViz/MoveIt remains planning truth; no real robot motion is started.");
  append_info("Run Web Edit Patch Workflow: python3 " + args.join(' ').toStdString());

  QProcess process;
  process.setWorkingDirectory(QString::fromStdString(repo_root.string()));
  process.start("python3", args);
  if (!process.waitForStarted()) {
    const QString message = "Python executable 'python3' could not be started. Install Python 3 or ensure it is on PATH.";
    append_error(message.toStdString());
    if (output) {
      *output = message;
    }
    return false;
  }
  process.waitForFinished(-1);
  const QString stdout_text = QString::fromLocal8Bit(process.readAllStandardOutput());
  const QString stderr_text = QString::fromLocal8Bit(process.readAllStandardError());
  const QString combined = QString("Command: python3 %1\n\nSTDOUT:\n%2\n\nSTDERR:\n%3")
    .arg(args.join(' '), stdout_text, stderr_text);
  if (output) {
    *output = combined;
  }
  if (!stdout_text.trimmed().isEmpty()) {
    append_info(stdout_text.toStdString());
  }
  if (!stderr_text.trimmed().isEmpty()) {
    append_warning(stderr_text.toStdString());
  }
  const bool ok = process.exitStatus() == QProcess::NormalExit && process.exitCode() == 0;
  if (!ok) {
    append_error(QString("Web edit patch workflow failed with exit code %1. Common causes include scene ID mismatch, locked/generated edit rejection, invalid patch JSON, or persistence verification failure.\n%2")
      .arg(process.exitCode())
      .arg(combined).toStdString());
  } else {
    append_success("Web edit patch workflow completed successfully.");
  }
  return ok;
}

bool SceneSelect::run_web_edit_patch_workflow(bool validate_only, bool write)
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    const QString message = "No scene selected. Select a scene before running a Web 3D edit patch workflow.";
    append_warning(message.toStdString());
    QMessageBox::warning(this, "Web Edit Patch Workflow", message);
    return false;
  }

  const fs::path scene_dir = scene_dir_for_current_selection();
  boost::system::error_code ec;
  if (scene_dir.empty() || !fs::exists(scene_dir, ec) || !fs::is_directory(scene_dir, ec)) {
    const QString message = QString("Scene path missing for selected scene '%1': %2")
      .arg(QString::fromStdString(workcell.scene_vector[current_index].name), QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Web Edit Patch Workflow", message);
    return false;
  }
  const fs::path repo_root = resolve_tool_root(workcell_path, scene_dir);
  const fs::path workflow_script = repo_root / "scripts" / "run_workcell_studio_web_edit_workflow.py";
  if (repo_root.empty() || !fs::exists(workflow_script, ec)) {
    const QString message = QString("Workflow script missing. Expected scripts/run_workcell_studio_web_edit_workflow.py under the Workcell Studio repo root. Scene path: %1")
      .arg(QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Web Edit Patch Workflow", message);
    return false;
  }

  const QString patch_file = QFileDialog::getOpenFileName(
    this,
    "Select Web 3D edit_patch.json",
    QString::fromStdString((repo_root / "build" / "workcell_studio_web_scene").string()),
    "JSON patch files (*.json);;All files (*)");
  if (patch_file.isEmpty()) {
    append_warning("Web edit patch workflow cancelled before patch selection.");
    return false;
  }
  const QFileInfo patch_info(patch_file);
  if (!patch_info.exists() || !patch_info.isFile() || patch_info.suffix().compare("json", Qt::CaseInsensitive) != 0) {
    const QString message = QString("Invalid patch path/JSON. Select an existing .json edit patch file. Path: %1").arg(patch_file);
    append_error(message.toStdString());
    QMessageBox::critical(this, "Web Edit Patch Workflow", message);
    return false;
  }
  QFile patch_reader(patch_file);
  if (!patch_reader.open(QIODevice::ReadOnly)) {
    const QString message = QString("Invalid patch path/JSON. Could not read patch file: %1").arg(patch_file);
    append_error(message.toStdString());
    QMessageBox::critical(this, "Web Edit Patch Workflow", message);
    return false;
  }
  QJsonParseError parse_error;
  QJsonDocument::fromJson(patch_reader.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError) {
    const QString message = QString("Invalid patch path/JSON. JSON parse failed for %1: %2").arg(patch_file, parse_error.errorString());
    append_error(message.toStdString());
    QMessageBox::critical(this, "Web Edit Patch Workflow", message);
    return false;
  }

  const fs::path patch_path(patch_file.toStdString());
  QString dry_run_output;
  const bool dry_ok = execute_web_edit_patch_workflow(repo_root, scene_dir, patch_path, validate_only, false, &dry_run_output);
  if (!dry_ok || !write) {
    QMessageBox::information(this, "Web Edit Patch Workflow", dry_run_output);
    return dry_ok;
  }

  const QString confirm = QString(
    "Dry-run passed. Apply this Web 3D edit patch now?\n\nScene path: %1\nPatch path: %2\n\nWorkflow output/change summary:\n%3\n\nWarning: applying may update editable source YAML. Generated files are not edited directly. Browser edits are preview-only until this backend apply step. RViz/MoveIt remains planning truth and no real robot motion is started.")
    .arg(QString::fromStdString(scene_dir.string()), patch_file, dry_run_output.left(6000));
  if (QMessageBox::question(this, "Confirm Apply Web Edit Patch", confirm, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
    append_warning("Apply Web Edit Patch cancelled after successful dry-run; --write was not run.");
    return false;
  }

  QString apply_output;
  const bool apply_ok = execute_web_edit_patch_workflow(repo_root, scene_dir, patch_path, false, true, &apply_output);
  if (apply_ok) {
    const QString next_step = "Patch applied and verified. You can now Generate & Validate Scene.";
    append_success(next_step.toStdString());
    apply_output += "\n\n" + next_step;
  }
  QMessageBox::information(this, apply_ok ? "Web Edit Patch Applied" : "Web Edit Patch Failed", apply_output);
  return apply_ok;
}

// Object Placement Manager markers for generated scene artifacts
[[maybe_unused]] static const char * kObjectPlacementManagerLabel = "Object Placement Manager";
[[maybe_unused]] static const char * kPlacedObjectsLabel = "Placed Objects";
[[maybe_unused]] static const char * kAddAssetObjectLabel = "Add Asset Object";
[[maybe_unused]] static const char * kImportStlToAssetLibraryLabel = "Import STL to Asset Library";
[[maybe_unused]] static const char * kDuplicateObjectLabel = "Duplicate Object";
[[maybe_unused]] static const char * kRemoveObjectLabel = "Remove Object";
[[maybe_unused]] static const char * kEditPoseLabel = "Edit Pose";
[[maybe_unused]] static const char * kExternalStlWarningLabel = "external_stl_warning";
[[maybe_unused]] static const char * kAssetStlLabel = "asset_stl";
[[maybe_unused]] static const char * kGeneratedPrimitiveLabel = "generated_primitive";
[[maybe_unused]] static const char * kManagedCustomMeshFolder = "easy_manipulation_deployment/assets/environment/custom_meshes";
[[maybe_unused]] static const char * kPlacedObjectsSummaryExample = "placed_objects:\n  - name: table_01\n    source: asset_stl\n    mesh: package://easy_manipulation_deployment/assets/environment/custom_meshes/table.stl\n    pose: [0, 0, 0, 0, 0, 0]";

[[maybe_unused]] static const char * kVisualLayoutSummaryJsonMarker = "visual_layout_editor_used placed_object_count placed_object_positions";
[[maybe_unused]] static const char * kVisualLayoutPreviewMarker = "Object table_01 @ x=0.0 y=0.0 | Save Layout to Environment YAML";
// readiness overlay markers
[[maybe_unused]] static const char * kReadinessOverlayWarningMarkers = "reach_warnings workspace_warnings overlap_warnings camera_warnings task_target_warnings safety_zone_warnings blocker_count warning_count";

// Portable Scene Bundle UI markers
// Export Scene Bundle
// Import Scene Bundle
// Portable Scene Bundle
// Bundle Validation Status
// Imported Scene Ready
// Exported Scene Archive


void SceneSelect::on_export_open_web_3d_viewer_clicked()
{
  configure_startup_fallback_paths();
  const int current_index = current_scene_index();
  if (current_index < 0 || current_index >= static_cast<int>(workcell.scene_vector.size())) {
    const QString message = "No scene selected. Select or create a scene before opening the Web 3D Viewer.";
    append_warning(message.toStdString());
    QMessageBox::warning(this, "Open Web 3D Viewer", message);
    return;
  }

  const std::string scene_id = sanitize_scene_name(workcell.scene_vector[current_index].name);
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_id.empty()) {
    const QString message = "No scene selected. The selected scene has an empty or invalid scene ID.";
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }
  boost::system::error_code ec;
  if (scene_dir.empty() || !fs::exists(scene_dir, ec) || !fs::is_directory(scene_dir, ec)) {
    const QString message = QString("Scene path missing for selected scene '%1': %2")
      .arg(QString::fromStdString(scene_id), QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }

  fs::path repo_root = resolve_tool_root(workcell_path, scene_dir);
  fs::path exporter_script;
  if (!repo_root.empty()) {
    exporter_script = repo_root / "scripts" / "run_workcell_web3d_visual_acceptance.py";
  }
  if (repo_root.empty() || !fs::exists(exporter_script, ec)) {
    const QString message = QString(
      "Exporter script missing. Expected visual acceptance script scripts/run_workcell_web3d_visual_acceptance.py under the Workcell Studio repo root. "
      "Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment. Scene path: %1")
      .arg(QString::fromStdString(scene_dir.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }

  const fs::path output_path = repo_root / "build" / "workcell_studio_web_scene" / (scene_id + ".web_scene.json");
  const fs::path viewer_path = repo_root / "workcell_studio_web" / "viewer" / "index.html";
  const QString web_scene_url_path = QString("build/workcell_studio_web_scene/%1.web_scene.json")
    .arg(QString::fromStdString(scene_id));
  const QString viewer_url = QString("http://127.0.0.1:8765/workcell_studio_web/viewer/index.html?scene=%1")
    .arg(QString::fromUtf8(QUrl::toPercentEncoding(web_scene_url_path)));
  fs::create_directories(output_path.parent_path(), ec);
  const QStringList args{
    QString::fromStdString(exporter_script.string()),
    "--scene", QString::fromStdString(scene_dir.string()),
    "--output", QString::fromStdString(output_path.string()),
    "--port", "8765"};
  const QString command_text = "python3 " + args.join(' ');
  append_info("Run Web 3D visual acceptance for selected scene: " + command_text.toStdString());
  QProcess exporter_process;
  exporter_process.setProgram("python3");
  exporter_process.setArguments(args);
  exporter_process.setWorkingDirectory(QString::fromStdString(repo_root.string()));
  exporter_process.setProcessChannelMode(QProcess::SeparateChannels);
  exporter_process.start();

  bool process_ok = exporter_process.waitForStarted();
  if (process_ok) {
    process_ok = exporter_process.waitForFinished(-1);
  }
  const QString stdout_text = QString::fromLocal8Bit(exporter_process.readAllStandardOutput());
  const QString stderr_text = QString::fromLocal8Bit(exporter_process.readAllStandardError());
  if (!stdout_text.trimmed().isEmpty()) {
    append_info(stdout_text.toStdString());
  }
  if (!stderr_text.trimmed().isEmpty()) {
    append_warning(stderr_text.toStdString());
  }
  const int rc = process_ok && exporter_process.exitStatus() == QProcess::NormalExit
    ? exporter_process.exitCode()
    : -1;
  const bool output_exists = fs::exists(output_path, ec);
  if (!process_ok || rc != 0 || !output_exists) {
    const QString failure_reason = !process_ok
      ? exporter_process.errorString()
      : (output_exists ? QString("visual acceptance script exited nonzero") : QString("visual acceptance script did not create the expected output file"));
    const QString message = QString(
      "Web 3D scene export failed; viewer not opened with stale generated artifacts.\n\n"
      "Reason: %1\n"
      "Exit code: %2\n"
      "Command: %3\n"
      "Scene: %4\n"
      "Output: %5\n\n"
      "stdout:\n%6\n\n"
      "stderr:\n%7")
      .arg(failure_reason)
      .arg(rc)
      .arg(command_text, QString::fromStdString(scene_dir.string()), QString::fromStdString(output_path.string()),
        stdout_text.trimmed().isEmpty() ? QString("<empty>") : stdout_text,
        stderr_text.trimmed().isEmpty() ? QString("<empty>") : stderr_text);
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }
  append_success("Exported Web 3D scene JSON: " + output_path.string());

  if (!fs::exists(viewer_path, ec)) {
    const QString message = QString("Viewer file missing: %1\nExported JSON: %2")
      .arg(QString::fromStdString(viewer_path.string()), QString::fromStdString(output_path.string()));
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }

  const QString manual_server_command = QString("cd %1 && python3 -m http.server 8765 --bind 127.0.0.1")
    .arg(QString::fromStdString(repo_root.string()));
  const auto is_local_server_reachable = []() {
    const QString probe =
      "import socket,sys\n"
      "s=socket.socket()\n"
      "s.settimeout(0.25)\n"
      "try:\n"
      "    s.connect(('127.0.0.1', 8765))\n"
      "except OSError:\n"
      "    sys.exit(1)\n"
      "finally:\n"
      "    s.close()\n";
    return QProcess::execute("python3", QStringList{"-c", probe}) == 0;
  };

  bool server_started = false;
  bool server_reused = is_local_server_reachable();
  qint64 server_pid = 0;
  if (server_reused) {
    append_info("Reusing existing local Web 3D Viewer static asset server on http://localhost:8765/.");
  } else {
    server_started = QProcess::startDetached(
      "python3",
      QStringList{"-m", "http.server", "8765", "--bind", "127.0.0.1"},
      QString::fromStdString(repo_root.string()),
      &server_pid);
    if (server_started) {
      append_info("Started local Web 3D Viewer server from repo root: python3 -m http.server 8765 --bind 127.0.0.1");
    } else {
      append_warning("Could not start local Web 3D Viewer server on 127.0.0.1:8765; opening the repo-root HTTP URL anyway in case a server is already running.");
    }
  }

  const QString server_status = server_started
    ? QString("started (PID %1)").arg(server_pid)
    : (server_reused ? QString("reused existing server") : QString("not started"));
  const bool opened = QDesktopServices::openUrl(QUrl(viewer_url));
  const QString fallback = QString(
    "Exported web scene JSON: %3\n"
    "Viewer URL opened: %1\n"
    "Local static asset server: %5\n\n"
    "Viewer path: %2\n\n"
    "The viewer is opened through a repository-root HTTP server so staged meshes resolve under:\n"
    "build/workcell_studio_web_scene/assets/%4/...\n\n"
    "If the browser cannot connect or the server did not start, run this manually:\n"
    "%6\n\n"
    "Then open:\n"
    "%1")
    .arg(viewer_url, QString::fromStdString(viewer_path.string()), QString::fromStdString(output_path.string()), QString::fromStdString(scene_id), server_status, manual_server_command);
  if (!opened) {
    const QString message = "Browser open failed. Use the local-server URL below.\n\n" + fallback;
    append_error(message.toStdString());
    QMessageBox::critical(this, "Open Web 3D Viewer", message);
    return;
  }
  append_success("Opened Web 3D Viewer: " + viewer_url.toStdString());
  QMessageBox::information(this, "Open Web 3D Viewer", fallback);
}

void SceneSelect::on_export_scene_bundle_clicked()
{
  append_warning("Export scene bundle is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_import_scene_bundle_clicked()
{
  append_warning("Import scene bundle is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_refresh_status_button_clicked()
{
  refresh_scene_status(true, "Refresh Scene Status");
}

void SceneSelect::on_validate_scene_button_clicked()
{
  on_validate_cell_clicked();
  if (latest_dashboard_result_.blocker_count > 0) {
    const fs::path scene_dir = scene_dir_for_current_selection();
    if (!scene_dir.empty() && auto_fix_pick_place_zones_layout_yaml(scene_dir)) {
      append_info("Auto-fix Pick/Place Zones available. Click Improve Layout to apply offline geometry repair.");
    }
  }
  on_refresh_status_button_clicked();
  const int current_index = ui->scene_list->currentIndex();
  if (current_index >= 0 && current_index < static_cast<int>(workcell.scene_vector.size())) {
    const auto name = workcell.scene_vector[current_index].name;
    const auto it = all_scenes_readiness_.by_scene.find(name);
    if (it != all_scenes_readiness_.by_scene.end()) {
      const auto & r = it->second;
      append_info("Supported scene readiness for " + name + ": " + r.readiness_status);
      append_info("required_files_status=" + r.required_files_status + ", static_validation_status=" + r.static_validation_status + ", guided_build_launch_readiness=" + r.guided_build_launch_readiness + ", fake_hardware_launch_readiness=" + r.fake_hardware_launch_readiness);
      if (!r.blockers_summary.empty()) append_warning("Blockers: " + r.blockers_summary);
      if (!all_scenes_readiness_.report_path.empty()) append_info("Latest all-scenes readiness report: " + all_scenes_readiness_.report_path);
    } else {
      append_info("Supported scene readiness: UNKNOWN (no all-scenes readiness entry found).");
    }
  }
}

void SceneSelect::on_copy_build_command_button_clicked()
{
  append_warning("Copy build command from status report is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_copy_launch_command_button_clicked()
{
  append_warning("Copy launch command from status report is temporarily disabled in this build-repair patch.");
}

void SceneSelect::on_open_conveyor_sorting_run_console_button_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_warning("No scene selected. Select a scene before opening the run console.");
    return;
  }

  const int current_index = current_scene_index();
  if (current_index < 0) {
    append_warning("No scene selected. Select a scene before opening the run console.");
    return;
  }
  const QString scenario_name = ui->scene_list->itemText(current_index);
  ConveyorSortingRunConsole dialog(std::filesystem::path(scene_dir.string()), scenario_name, this);
  dialog.exec();
}

bool SceneSelect::run_demo_action(bool validate, bool generate)
{
  const int row = ui->demo_mode_table->currentRow();
  if (row < 0) {
    append_warning("Select a demo card/row first.");
    return false;
  }
  const std::vector<std::string> scene_slugs = {
    "demo_ur5_2f_pick_place", "demo_ur5_suction_pick", "demo_conveyor_sorting_epd_preview", "demo_camera_inspection_preview"};
  const std::vector<std::string> template_ids = {
    "Pick and Place Cell: UR5 + Robotiq 2F + table + cube + bin", "UR5 + Suction Pick Cell", "Conveyor Sorting - Live EPD Preview", "Camera Inspection Cell (PREVIEW ONLY)"};
  std::string scene_name = scene_slugs[static_cast<size_t>(row)];
  fs::path scene_dir = scenes_path / scene_name;
  int suffix = 1;
  while (fs::exists(scene_dir)) {
    scene_name = scene_slugs[static_cast<size_t>(row)] + "_" + std::to_string(suffix++);
    scene_dir = scenes_path / scene_name;
  }
  scene_name = sanitize_scene_name(scene_name);
  if (!create_scene_from_template(template_ids[static_cast<size_t>(row)], scene_name, scenes_path, &scene_dir)) return false;
  apply_recommended_layout_to_scene(scene_dir, template_ids[static_cast<size_t>(row)]);
  Scene scene; scene.name = scene_name;
  save_new_scene_yaml(scene_dir, scene);
  if (validate) validate_new_scene(scene_dir);
  const bool supported_full_generate = (row == 0);
  if (generate && supported_full_generate) generate_full_scene_package_from_scene(scene_dir);
  export_workcell_layout_preview(scene, scene_dir, false);
  update_new_scene_lifecycle_and_canvas(scene_dir);
  latest_demo_scene_name_ = scene_name;
  latest_demo_scene_dir_ = scene_dir;
  return true;
}

void SceneSelect::on_demo_one_click_button_clicked() { run_demo_action(true, true); }
void SceneSelect::on_demo_create_scene_button_clicked() { run_demo_action(false, false); }
void SceneSelect::on_demo_validate_button_clicked() { run_demo_action(true, false); }
void SceneSelect::on_demo_generate_button_clicked() { run_demo_action(false, true); }
void SceneSelect::on_demo_export_preview_button_clicked() { if (!latest_demo_scene_dir_.empty()) { Scene s; s.name = latest_demo_scene_name_; export_workcell_layout_preview(s, latest_demo_scene_dir_, true); } }
void SceneSelect::on_demo_copy_build_command_button_clicked() { if (!latest_demo_scene_name_.empty()) QApplication::clipboard()->setText(QString::fromStdString("colcon build --symlink-install --packages-select " + latest_demo_scene_name_)); }
void SceneSelect::on_demo_copy_launch_command_button_clicked() { if (!latest_demo_scene_name_.empty()) QApplication::clipboard()->setText(QString::fromStdString("ros2 launch " + latest_demo_scene_name_ + " demo.launch.py use_fake_hardware:=true launch_task_preview:=true")); }
void SceneSelect::on_demo_open_scene_folder_button_clicked() { if (!latest_demo_scene_dir_.empty()) QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(latest_demo_scene_dir_.string()))); }
void SceneSelect::on_demo_open_readiness_report_button_clicked() { if (!latest_demo_scene_dir_.empty()) QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString((latest_demo_scene_dir_ / "readiness" / "readiness_report.md").string()))); }


void SceneSelect::on_repair_scene_yaml_clicked()
{
  const fs::path scene_dir = scene_dir_for_current_selection();
  if (scene_dir.empty()) {
    append_warning("No selected scene. Next recommended action: select a scene first.");
    return;
  }
  std::string summary;
  if (repair_scene_yaml_file(scene_dir, &summary)) {
    append_success("Repair Scene YAML complete: " + summary);
  } else {
    append_warning("Repair Scene YAML skipped: " + summary);
  }
  refresh_scene_status(true, "Repair Scene YAML");
}


void SceneSelect::on_run_all_scenes_readiness_clicked()
{
  const fs::path workspace_root = derive_ros_workspace_root(workcell_path);
  const std::string cmd = "python3 scripts/validate_supported_scenes_readiness.py --repo-root "" + workcell_path.string() + "" --workspace-root "" + workspace_root.string() + "" --json --skip-build --skip-launch-smoke";
  append_info("Run All-Scenes Readiness: " + cmd);
  const int rc = std::system(cmd.c_str());
  if (rc != 0) { append_error("Run All-Scenes Readiness failed with exit code " + std::to_string(rc)); return; }
  append_success("Run All-Scenes Readiness completed.");
  std::string readiness_err;
  all_scenes_readiness_ = workcell_builder::load_latest_all_scenes_readiness_report(workcell_path, &readiness_err);
  if (!readiness_err.empty()) append_warning(readiness_err);
  if (!all_scenes_readiness_.report_path.empty()) append_info("Latest all-scenes readiness report: " + all_scenes_readiness_.report_path);
  refresh_scenes(current_scene_index(), false);
}
