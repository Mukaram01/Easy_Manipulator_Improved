#include "scene_preview_widget.h"
// Compatibility token for static tests: Preview selection cleared after refresh (id missing):

#include <QRectF>
#include <QtGlobal>
#include <QVariantMap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QIODevice>
#include <QStringList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QEventLoop>
#include <QTimer>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QJsonArray>
#include <QCryptographicHash>
#include <QDataStream>
#include <QSet>
#include <QPushButton>
#include <functional>

namespace {
constexpr double kOverlayFitDominanceRatio = 4.0;

QByteArray serialized_preview_item(const ScenePreviewWidget::PreviewItem & item)
{
  QByteArray bytes;
  QDataStream stream(&bytes, QIODevice::WriteOnly);
  stream.setVersion(QDataStream::Qt_5_12);
  stream.setByteOrder(QDataStream::LittleEndian);
  const auto write_bool = [&stream](bool value) { stream << static_cast<quint8>(value ? 1 : 0); };
  const auto write_string_list = [&stream](const QStringList & values) {
    stream << static_cast<quint32>(values.size());
    for (const QString & value : values) stream << value;
  };
  const auto write_matrix = [&stream](const QMatrix4x4 & matrix) {
    for (int row = 0; row < 4; ++row) for (int column = 0; column < 4; ++column) stream << matrix(row, column);
  };

  // Keep this order aligned with PreviewItem.  These values are consumed by
  // the native Scene3D and embedded Web3D handoff, including diagnostic labels.
  stream << item.id << item.display_name << item.category;
  stream << item.x << item.y << item.z << item.roll << item.pitch << item.yaw;
  stream << item.sx << item.sy << item.sz << item.status << item.source_path << item.role;
  stream << item.mesh_path << item.mesh_type << item.primitive_geometry_type << item.primitive_radius << item.primitive_length;
  write_bool(item.has_material_color);
  stream << item.material_r << item.material_g << item.material_b << item.material_a << item.material_name;
  stream << item.mesh_scale_x << item.mesh_scale_y << item.mesh_scale_z << item.mesh_roll << item.mesh_pitch << item.mesh_yaw;
  write_bool(item.mesh_available);
  stream << item.mesh_load_warning;
  write_bool(item.selectable); write_bool(item.editable); write_bool(item.locked);
  stream << item.lock_reason << item.metadata_tags << item.source_layer << item.active_visual_source;
  write_bool(item.linked_to_editable_layout_state); write_bool(item.metadata_complete);
  write_string_list(item.warnings);
  write_bool(item.has_mesh_metadata);
  stream << item.mesh_r << item.mesh_p << item.mesh_y;
  write_bool(item.has_origin_offset);
  stream << item.origin_offset_x << item.origin_offset_y << item.origin_offset_z;
  stream << item.camera_id << item.frame_id << item.detection_label << item.confidence << item.tracking_id;
  stream << item.snapshot_source_file << item.alignment_warning;
  write_bool(item.resolved_source_path_stale);
  stream << item.resolved_source_path_original << item.package_uri << item.source_path_resolution_outcome;
  stream << item.base_pose_x << item.base_pose_y << item.base_pose_z << item.base_pose_roll << item.base_pose_pitch << item.base_pose_yaw;
  stream << item.chain_pose_x << item.chain_pose_y << item.chain_pose_z << item.chain_pose_roll << item.chain_pose_pitch << item.chain_pose_yaw;
  stream << item.visual_origin_x << item.visual_origin_y << item.visual_origin_z << item.visual_origin_roll << item.visual_origin_pitch << item.visual_origin_yaw;
  write_bool(item.transform_chain_applied); write_bool(item.visual_origin_applied); write_bool(item.has_baked_world_visual_transform);
  stream << item.baked_world_visual_transform_source;
  write_bool(item.has_baked_world_visual_matrix);
  write_matrix(item.baked_world_visual_matrix);
  write_bool(item.robot_candidate);
  stream << item.robot_classification_source << item.robot_base_frame << item.robot_world_pose;
  stream << item.visual_index_link << item.visual_index_link_name << item.visual_index_object_name << item.visual_index_visual << item.visual_index_visual_name;
  stream << static_cast<qint32>(item.visual_index_value) << static_cast<qint32>(item.source_row_index);
  stream << item.visual_index_parent_link;
  write_string_list(item.visual_index_link_chain);
  stream << item.visual_index_mesh_uri << item.visual_index_package_uri << item.visual_index_source;
  return bytes;
}

QString normalized_preview_token(const QString & value)
{
  return value.trimmed().toLower().replace('-', '_').replace(' ', '_');
}

QString normalized_absolute_preview_path(const QString & path)
{
  const QString trimmed_path = path.trimmed();
  if (trimmed_path.isEmpty()) return {};

  const QFileInfo info(trimmed_path);
  const QString canonical_path = info.canonicalFilePath();
  return QDir::cleanPath(canonical_path.isEmpty() ? info.absoluteFilePath() : canonical_path);
}

ScenePreviewWidget::PreviewContext normalized_preview_context(
  const ScenePreviewWidget::PreviewContext & context)
{
  ScenePreviewWidget::PreviewContext normalized = context;
  normalized.scene_id = normalized.scene_id.trimmed();
  normalized.absolute_scene_dir = normalized_absolute_preview_path(normalized.absolute_scene_dir);
  normalized.absolute_repo_root = normalized_absolute_preview_path(normalized.absolute_repo_root);
  return normalized;
}

bool preview_contexts_equal(
  const ScenePreviewWidget::PreviewContext & left,
  const ScenePreviewWidget::PreviewContext & right)
{
  return left.scene_id == right.scene_id &&
         left.absolute_scene_dir == right.absolute_scene_dir &&
         left.absolute_repo_root == right.absolute_repo_root;
}

bool is_safe_embedded_web_scene_id(const QString & scene_id)
{
  static const QRegularExpression kSafeSceneId(QStringLiteral("^[A-Za-z0-9][A-Za-z0-9_-]*$"));
  return kSafeSceneId.match(scene_id).hasMatch();
}

bool scene_directory_matches_id(const QString & scene_dir, const QString & scene_id)
{
  const QFileInfo scene_info(scene_dir);
  if (scene_info.fileName() == scene_id) return true;

  const QDir dir(scene_dir);
  for (const QString & metadata_name : {QStringLiteral("scene_manifest.yaml"), QStringLiteral("cell_definition.yaml"), QStringLiteral("environment.yaml")}) {
    QFile metadata(dir.filePath(metadata_name));
    if (!metadata.open(QIODevice::ReadOnly | QIODevice::Text)) continue;
    const QString text = QString::fromUtf8(metadata.readAll());
    const QRegularExpression scene_id_pattern(
      QStringLiteral("^scene:\\s*$[\\s\\S]*?^  (?:id|name):\\s*%1\\s*(?:#.*)?$").arg(QRegularExpression::escape(scene_id)),
      QRegularExpression::MultilineOption);
    const QRegularExpression cell_id_pattern(
      QStringLiteral("^cell:\\s*$[\\s\\S]*?^  (?:id|name):\\s*%1\\s*(?:#.*)?$").arg(QRegularExpression::escape(scene_id)),
      QRegularExpression::MultilineOption);
    const QRegularExpression top_level_id_pattern(
      QStringLiteral("^scene_id:\\s*%1\\s*(?:#.*)?$").arg(QRegularExpression::escape(scene_id)),
      QRegularExpression::MultilineOption);
    if (scene_id_pattern.match(text).hasMatch() || cell_id_pattern.match(text).hasMatch() ||
        top_level_id_pattern.match(text).hasMatch()) return true;
  }
  return false;
}

bool preview_item_has_credible_mesh_handoff(const ScenePreviewWidget::PreviewItem & item)
{
  return item.mesh_available || item.has_mesh_metadata || !item.mesh_path.trimmed().isEmpty() || !item.source_path.trimmed().isEmpty();
}

bool preview_item_has_valid_urdf_primitive(const ScenePreviewWidget::PreviewItem & item)
{
  const QString type = normalized_preview_token(item.primitive_geometry_type);
  if (type == QStringLiteral("box")) return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
  if (type == QStringLiteral("cylinder")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  if (type == QStringLiteral("sphere")) return item.primitive_radius > 0.001;
  if (type == QStringLiteral("capsule")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  return false;
}

bool preview_item_is_overlay_or_helper(const ScenePreviewWidget::PreviewItem & item)
{
  const QString role = normalized_preview_token(item.role);
  const QString category = normalized_preview_token(item.category);
  const QString source_layer = normalized_preview_token(item.source_layer);
  const QString lock_reason = normalized_preview_token(item.lock_reason);
  return role.contains("overlay") || role.contains("helper") || role.contains("guide") ||
         role.contains("warning_anchor") || role.contains("warning_badge") ||
         role.contains("safety_zone") || category.contains("overlay") || category.contains("helper") ||
         category.contains("safety") || source_layer.contains("overlay") || lock_reason.contains("overlay");
}

bool preview_item_is_generated_or_locked_urdf(const ScenePreviewWidget::PreviewItem & item)
{
  const QString category = normalized_preview_token(item.category);
  const QString source_layer = normalized_preview_token(item.source_layer);
  const QString visual_source = normalized_preview_token(item.active_visual_source);
  const QString lock_reason = normalized_preview_token(item.lock_reason);
  return source_layer.contains("generated_urdf") || source_layer.contains("locked_generated_urdf") ||
         visual_source.contains("generated_urdf") || visual_source.contains("locked_generated_urdf") ||
         category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot_model") ||
         lock_reason.contains("robotmodel");
}

bool preview_item_is_raw_generated_bounds_only(const ScenePreviewWidget::PreviewItem & item)
{
  return preview_item_is_generated_or_locked_urdf(item) &&
         item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001 &&
         (preview_item_has_credible_mesh_handoff(item) || preview_item_has_valid_urdf_primitive(item));
}

void maybe_warn_overlay_fit_dominance(ScenePreviewWidget * self, const QRectF & physical_bounds, const QRectF & overlay_bounds)
{
  if (!self) return;
  if (!physical_bounds.isValid() || physical_bounds.isEmpty()) return;
  if (!overlay_bounds.isValid() || overlay_bounds.isEmpty()) return;
  const double physical_area = qMax(1e-6, physical_bounds.width() * physical_bounds.height());
  const double overlay_area = qMax(0.0, overlay_bounds.width() * overlay_bounds.height());
  const double ratio = overlay_area / physical_area;
  if (ratio < kOverlayFitDominanceRatio) return;
  emit self->studio_log_requested(QString("Overlay-fit warning: overlay bounds are %1x physical bounds; use Fit Scene to keep physical meshes legible.")
                                      .arg(QString::number(ratio, 'f', 1)));
}
}


#include <algorithm>
#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsProxyWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QCoreApplication>
#include <QProcess>
#include <QProcessEnvironment>
#include <QUrl>
#include <QTimer>
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEngineView>
#include <QWebEnginePage>
#endif
#include "scene3d_viewport_widget.h"
#include <QPainter>
#include <QStackedWidget>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QVector3D>
#include <QVBoxLayout>
#include <QtMath>
#include <string_view>


namespace {
constexpr const char kScenePreviewMouseHelpText[] = "Mouse: orbit / wheel zoom / shift-drag pan";
static_assert(std::string_view(kScenePreviewMouseHelpText) == "Mouse: orbit / wheel zoom / shift-drag pan",
              "Acceptance token mismatch for scene preview mouse help text");

QString scene_preview_mouse_help_tooltip(const QString & unavailable_reason)
{
  QString tooltip = QString::fromUtf8(kScenePreviewMouseHelpText);
  if (!unavailable_reason.trimmed().isEmpty()) {
    tooltip += QString("\n3D unavailable: %1").arg(unavailable_reason.trimmed());
  }
  return tooltip;
}
}  // namespace

ScenePreviewWidget::ScenePreviewWidget(QWidget * parent) : QWidget(parent)
{
  setObjectName("scenePreviewWidget");
  auto * root = new QVBoxLayout(this);
  auto * controls_header = new QWidget(this);
  auto * controls = new QVBoxLayout(controls_header);
  controls->setContentsMargins(0, 0, 0, 0);
  controls->setSpacing(4);

  // Keep preview identity and its single authoritative runtime status separate
  // from backend-specific editing controls. This prevents a long status from
  // squeezing the Select/Move/Rotate/Snap/Undo/Redo/Fit control row.
  auto * identity_row = new QHBoxLayout();
  view_mode_label_ = new QLabel("Preview:", controls_header);
  identity_row->addWidget(view_mode_label_);
  mode_selector_ = new QComboBox(this);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  mode_selector_->addItems({"Web3D Product View", "2D Layout"});
#else
  mode_selector_->addItems({"3D Layout Preview", "2D Layout"});
#endif
  identity_row->addWidget(mode_selector_);
  identity_row->addStretch(1);
  toolbar_status_chip_ = new QLabel(controls_header);
  toolbar_status_chip_->setObjectName("previewToolbarChip");
  toolbar_status_chip_->setStyleSheet("QLabel#previewToolbarChip { background-color: rgba(30,41,59,225); color: #e2e8f0; border-radius: 9px; padding: 2px 8px; }");
  toolbar_status_chip_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  toolbar_status_chip_->setWordWrap(true);
  toolbar_status_chip_->setMinimumWidth(140);
  toolbar_status_chip_->setMaximumWidth(300);
  toolbar_status_chip_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  identity_row->addWidget(toolbar_status_chip_, 0);
  controls->addLayout(identity_row);

  auto * backend_controls_row = new QHBoxLayout();
  mesh_preview_mode_label_ = new QLabel("Mesh Preview:", controls_header);
  backend_controls_row->addWidget(mesh_preview_mode_label_);
  mesh_preview_mode_selector_ = new QComboBox(this);
  mesh_preview_mode_selector_->addItems({"Auto", "Meshes", "Primitives"});
  mesh_preview_mode_selector_->setCurrentText("Auto");
  mesh_preview_mode_selector_->setToolTip("Mesh preview mode is visual-only and does not alter generated runtime files.");
  mesh_preview_mode_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(mesh_preview_mode_selector_);
  backend_controls_row->addSpacing(8);
  gizmo_mode_label_ = new QLabel("Gizmo:", controls_header);
  backend_controls_row->addWidget(gizmo_mode_label_);
  gizmo_mode_selector_ = new QComboBox(this);
  gizmo_mode_selector_->addItems({"Select", "Move", "Rotate", "Scale (disabled)"});
  gizmo_mode_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(gizmo_mode_selector_);
  snap_mode_label_ = new QLabel("Snap:", controls_header);
  backend_controls_row->addWidget(snap_mode_label_);
  snap_mode_selector_ = new QComboBox(this);
  snap_mode_selector_->addItems({"Off", "1 cm", "5 cm", "10 cm", "5 deg", "15 deg"});
  snap_mode_selector_->setCurrentText("5 cm");
  snap_mode_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(snap_mode_selector_);
  embedded_undo_button_ = new QPushButton(QStringLiteral("Undo"), this);
  embedded_undo_button_->setEnabled(false);
  embedded_undo_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(embedded_undo_button_);
  embedded_redo_button_ = new QPushButton(QStringLiteral("Redo"), this);
  embedded_redo_button_->setEnabled(false);
  embedded_redo_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(embedded_redo_button_);
  embedded_fit_button_ = new QPushButton(QStringLiteral("Fit"), this);
  embedded_fit_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(embedded_fit_button_);
  backend_controls_row->addSpacing(8);
  labels_label_ = new QLabel("Labels:", controls_header);
  backend_controls_row->addWidget(labels_label_);
  labels_selector_ = new QComboBox(this);
  labels_selector_->addItems({"Off", "Important", "Selected", "All"});
  labels_selector_->setCurrentText("Selected");
  labels_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  labels_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(labels_selector_);
  backend_controls_row->addSpacing(8);
  interaction_mode_label_ = new QLabel("Mode:", controls_header);
  backend_controls_row->addWidget(interaction_mode_label_);
  interaction_mode_selector_ = new QComboBox(this);
  interaction_mode_selector_->addItems({"Select", "Place Asset", "Move", "Rotate", "Inspect"});
  interaction_mode_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(interaction_mode_selector_);
  view_actions_label_ = new QLabel("View:", controls_header);
  backend_controls_row->addWidget(view_actions_label_);
  view_actions_selector_ = new QComboBox(this);
  view_actions_selector_->addItems({"Top", "Front", "Side", "Isometric", "Fit View"});
  {
    const QSignalBlocker blocker(view_actions_selector_);
    view_actions_selector_->setCurrentText("Isometric");
  }
  view_actions_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(view_actions_selector_);
  auto * mouse_help_label = new QLabel(QStringLiteral("ⓘ"), controls_header);
  mouse_help_label->setToolTip(scene_preview_mouse_help_tooltip(QString()));
  mouse_help_label->setStatusTip(QString::fromUtf8(kScenePreviewMouseHelpText));
  backend_controls_row->addWidget(mouse_help_label);
  overlays_selector_ = new QComboBox(this);
  overlays_selector_->addItems({
    "Overlays",
    "Diagnostics Overlay",
    "Reachability Heatmap",
    "Collision Warnings",
    "Safety Zones",
    "Work Envelope",
    "Warning Labels",
    "Labels",
    "Pick/Place Zones",
    "Task Route",
    "Approach/Retreat",
    "Camera FOV",
    "Pick Coverage",
    "EPD Detections",
    "Detection Labels",
    "Warnings",
    "Focus Selected",
    "Fit Scene",
    "Fit Robot",
    "Fit overlays",
    "Clear Selection"
  });
  overlays_selector_->setCurrentText("Overlays");
  overlays_selector_->setToolTip("Product View starts clean. Use these diagnostics controls to explicitly enable helper overlays for the current preview session.");
  overlays_selector_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  backend_controls_row->addWidget(overlays_selector_);
  backend_controls_row->addStretch(1);
  controls->addLayout(backend_controls_row);

  toolbar_feedback_row_ = new QWidget(controls_header);
  auto * feedback_layout = new QHBoxLayout(toolbar_feedback_row_);
  feedback_layout->setContentsMargins(0, 0, 0, 0);
  toolbar_feedback_label_ = new QLabel(toolbar_feedback_row_);
  toolbar_feedback_label_->setWordWrap(true);
  toolbar_feedback_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  feedback_layout->addWidget(toolbar_feedback_label_);
  toolbar_feedback_row_->setVisible(false);
  controls->addWidget(toolbar_feedback_row_);
  root->addWidget(controls_header);
  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  const QString requested_product_view_backend =
    QProcessEnvironment::systemEnvironment().value(QStringLiteral("WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND")).trimmed().toLower();
  const bool native_scene3d_explicitly_requested =
    requested_product_view_backend == QStringLiteral("native_scene3d") ||
    requested_product_view_backend == QStringLiteral("native") ||
    requested_product_view_backend == QStringLiteral("scene3d");
  if (!native_scene3d_explicitly_requested) {
    product_view_backend_ = ProductViewBackend::EmbeddedWeb3D;
    embedded_web_view_ = new QWebEngineView(view3d_container_);
    embedded_web_view_->setObjectName("embeddedWeb3dProductView");
    connect(embedded_web_view_, &QWebEngineView::loadFinished, this, [this](bool ok) {
      const EmbeddedWebRequestIdentity identity = embedded_web_loading_identity_;
      const quint64 navigation_token = embedded_web_loading_navigation_token_;
      const QUrl expected_viewer_url = embedded_web_expected_viewer_url_;
      if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_) {
        emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View load completion for scene %1 revision %2 navigation %3.")
          .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
        return;
      }
      if (embedded_web_view_->url() != expected_viewer_url) {
        emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View load completion for scene %1 revision %2 navigation %3; expected viewer URL %4, active viewer URL %5.")
          .arg(identity.scene_id).arg(identity.generation).arg(navigation_token)
          .arg(expected_viewer_url.toString(), embedded_web_view_->url().toString()));
        return;
      }
      if (!ok) {
        const QString detail = QStringLiteral("browser failed to load Product View page for scene %1; viewer URL: %2; expected JSON: %3")
          .arg(identity.scene_id, expected_viewer_url.toString(), embedded_web_prepare_output_path_);
        // set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail) is intentionally replaced by native compatibility fallback.
        embedded_web_view_->setVisible(false);
        activate_native_compatibility_preview(detail);
        emit studio_log_requested(QStringLiteral("Embedded Product View load failed. %1").arg(detail));
        return;
      }
      set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness,
        QStringLiteral("browser loaded; waiting for viewer readiness"));
      start_embedded_web_readiness_polling(identity, navigation_token, embedded_web_prepare_output_path_, expected_viewer_url.toString());
    });
    simple_3d_view_ = embedded_web_view_;
  } else {
    product_view_backend_ = ProductViewBackend::NativeScene3D;
    simple_3d_view_ = new Scene3DViewportWidget(view3d_container_);
    simple_3d_view_->setObjectName("scene3dViewportWidget");
  }
#else
  product_view_backend_ = ProductViewBackend::NativeScene3D;
  simple_3d_view_ = new Scene3DViewportWidget(view3d_container_);
  simple_3d_view_->setObjectName("scene3dViewportWidget");
#endif
  v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D Layout Preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  fallback_banner_label_ = new QLabel("2D fallback preview active", view2d_container_);
  fallback_banner_label_->setAlignment(Qt::AlignCenter);
  fallback_banner_label_->setVisible(false);
  fallback_banner_label_->setToolTip(scene_preview_mouse_help_tooltip(QString()));
  view2d_container_->layout()->addWidget(fallback_banner_label_);
  info_chip_label_ = new QLabel(view2d_container_);
  info_chip_label_->setObjectName("previewInfoChip");
  info_chip_label_->setStyleSheet("QLabel#previewInfoChip { background-color: rgba(15,23,42,210); color: #e2e8f0; border-radius: 6px; padding: 5px 8px; }");
  info_chip_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  info_chip_label_->setWordWrap(true);
  stack_->addWidget(view3d_container_); stack_->addWidget(view2d_container_); root->addWidget(stack_, 1);
  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(labels_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = active_native_viewport();
    if (!v) { refresh_info_chip(); return; }
    const QString choice = labels_selector_->currentText();
    if (choice == "Off") v->label_mode = LabelMode::Off;
    else if (choice == "Important") v->label_mode = LabelMode::Important;
    else if (choice == "Selected") v->label_mode = LabelMode::Selected;
    else v->label_mode = LabelMode::All;
    v->update();
    refresh_info_chip();
  });
  connect(snap_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = active_native_viewport();
    if (!v) { run_embedded_editor_command(embedded_snap_command(snap_mode_selector_->currentText())); refresh_info_chip(); return; }
    const QString choice = snap_mode_selector_->currentText();
    if (choice == "1 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm1;
    else if (choice == "5 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm5;
    else if (choice == "10 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm10;
    else if (choice == "5 deg") v->snap_mode = Scene3DViewportWidget::SnapMode::Deg5;
    else if (choice == "15 deg") v->snap_mode = Scene3DViewportWidget::SnapMode::Deg15;
    else v->snap_mode = Scene3DViewportWidget::SnapMode::Off;
    v->update();
    refresh_info_chip();
  });
  connect(gizmo_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = active_native_viewport();
    if (!v) { const QString choice = gizmo_mode_selector_->currentText(); run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.setMode(%1)").arg(QString(QJsonDocument(QJsonArray{choice == "Move" ? "move" : (choice == "Rotate" ? "rotate" : "select")}).toJson(QJsonDocument::Compact)).mid(1).chopped(1))); refresh_info_chip(); return; }
    const QString choice = gizmo_mode_selector_->currentText();
    if (choice == "Move") v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Move;
    else if (choice == "Rotate") v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Rotate;
    else if (choice.startsWith("Scale")) v->gizmo_mode = Scene3DViewportWidget::GizmoMode::ScaleDisabled;
    else v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select;
    v->update();
    refresh_info_chip();
  });
  connect(mesh_preview_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = active_native_viewport();
    const QString choice = mesh_preview_mode_selector_->currentText();
    if (choice == "Meshes") mesh_preview_mode_ = MeshPreviewMode::Meshes;
    else if (choice == "Primitives") mesh_preview_mode_ = MeshPreviewMode::Primitives;
    else mesh_preview_mode_ = MeshPreviewMode::Auto;
    if (v) { v->mesh_preview_mode = mesh_preview_mode_; v->update(); }
  });
  connect(interaction_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    const QString choice = interaction_mode_selector_->currentText();
    auto * v = active_native_viewport();
    if (!v) return;
    if (choice == "Move") { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Move; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Move"); }
    else if (choice == "Rotate") { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Rotate; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Rotate"); }
    else { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Select"); }
    v->update();
    refresh_info_chip();
  });
  connect(view_actions_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    const QString choice = view_actions_selector_->currentText();
    auto * v = active_native_viewport();
    if (!v) { if (choice == "Fit View") run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.fitScene()")); refresh_info_chip(); return; }
    if (choice == "Top") v->set_top_view();
    else if (choice == "Front") v->set_front_view();
    else if (choice == "Side") v->set_side_view();
    else if (choice == "Isometric") v->set_isometric_view();
    else if (choice == "Fit View") on_fit_scene_clicked();
    else if (choice == "Labels" && labels_selector_) labels_selector_->showPopup();
    else if (choice == "Mesh Mode" && mesh_preview_mode_selector_) mesh_preview_mode_selector_->showPopup();
    else if (choice == "Diagnostics / Overlays" && overlays_selector_) overlays_selector_->showPopup();
    refresh_info_chip();
  });
  connect(embedded_undo_button_, &QPushButton::clicked, this, [this](){ run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.undo()")); });
  connect(embedded_redo_button_, &QPushButton::clicked, this, [this](){ run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.redo()")); });
  connect(embedded_fit_button_, &QPushButton::clicked, this, [this](){ run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.fitScene()")); });
  connect(overlays_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = active_native_viewport();
    const QString choice = overlays_selector_->currentText();
    if (!v) { if (choice != "Overlays") emit studio_log_requested(QString("Embedded Web 3D Product View handles Product View controls in the browser; use the web viewer UI for overlays and camera actions.")); if (overlays_selector_ && choice != "Overlays") { const QSignalBlocker blocker(overlays_selector_); overlays_selector_->setCurrentText("Overlays"); } refresh_info_chip(); return; }
    if (choice == "Diagnostics Overlay") {
      v->debug_overlays_mode = !v->debug_overlays_mode;
      emit studio_log_requested(QString("Scene3D diagnostics overlay %1. Detailed render diagnostics remain available in logs.")
                                  .arg(v->debug_overlays_mode ? "enabled" : "disabled"));
    }
    else if (choice == "Reachability Heatmap") v->show_reachability_heatmap = !v->show_reachability_heatmap;
    else if (choice == "Collision Warnings") v->show_collision_warnings = !v->show_collision_warnings;
    else if (choice == "Safety Zones") v->show_safety = !v->show_safety;
    else if (choice == "Work Envelope") v->show_work_envelope = !v->show_work_envelope;
    else if (choice == "Warning Labels") v->show_warning_labels = !v->show_warning_labels;
    else if (choice == "Labels" && labels_selector_) labels_selector_->showPopup();
    else if (choice == "Pick/Place Zones") v->show_pick_place = !v->show_pick_place;
    else if (choice == "Task Route") v->show_task_route = !v->show_task_route;
    else if (choice == "Approach/Retreat") v->show_approach_retreat = !v->show_approach_retreat;
    else if (choice == "Camera FOV") v->show_camera_fov = !v->show_camera_fov;
    else if (choice == "Pick Coverage") v->show_pick_coverage = !v->show_pick_coverage;
    else if (choice == "EPD Detections") v->show_epd_detections = !v->show_epd_detections;
    else if (choice == "Detection Labels") v->show_detection_labels = !v->show_detection_labels;
    else if (choice == "Warnings") v->show_warnings = !v->show_warnings;
    else if (choice == "Focus Selected") on_focus_selected_clicked();
    else if (choice == "Fit Scene") on_fit_scene_clicked();
    else if (choice == "Fit Robot") on_fit_robot_clicked();
    else if (choice == "Fit overlays") on_fit_overlays_clicked();
    else if (choice == "Clear Selection") on_clear_selection_clicked();
    if (overlays_selector_ && choice != "Overlays") {
      const QSignalBlocker blocker(overlays_selector_);
      overlays_selector_->setCurrentText("Overlays");
    }
    v->update();
    refresh_info_chip();
  });
  if (auto * legacy_viewport = active_native_viewport()) {
  legacy_viewport->select_cb = [this](const QString & id, const QString & role){ select_preview_item(id); emit preview_item_selected(id, role); if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("Selected preview item: %1 (%2)").arg(id, role)); };
  legacy_viewport->status_message_cb = [this](const QString & message) {
    emit studio_log_requested(message);
  };
  }
  refresh_info_chip();
  refresh_mode_and_state();
  QTimer::singleShot(0, this, [this]() { emit_backend_startup_diagnostic_once(); });
}

ScenePreviewWidget::~ScenePreviewWidget()
{
  cancel_embedded_web_lifecycle(true);
}

QString ScenePreviewWidget::resolve_embedded_web_repo_root(const QString & selected_scene_dir) const
{
  auto canonical_path = [](const QString & path) -> QString {
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
  };
  auto missing_markers = [](const QString & root) -> QStringList {
    QDir dir(root);
    QStringList missing;
    if (!QFileInfo::exists(dir.filePath(QStringLiteral("workcell_studio_web/viewer/index.html")))) {
      missing << QStringLiteral("workcell_studio_web/viewer/index.html");
    }
    if (!QFileInfo::exists(dir.filePath(QStringLiteral("scripts/ensure_workcell_studio_web_scene_fresh.py")))) {
      missing << QStringLiteral("scripts/ensure_workcell_studio_web_scene_fresh.py");
    }
    if (!QFileInfo(dir.filePath(QStringLiteral("scenes"))).isDir()) {
      missing << QStringLiteral("scenes");
    }
    return missing;
  };
  auto log_diagnostic = [this](const QString & message) {
    emit const_cast<ScenePreviewWidget *>(this)->studio_log_requested(message);
  };
  auto accept_candidate = [&](const QString & raw_root, const QString & label, QString * accepted) -> bool {
    const QString root = canonical_path(raw_root);
    const QStringList missing = missing_markers(root);
    if (missing.isEmpty()) {
      *accepted = root;
      if (diagnostic_debug_logging_enabled()) log_diagnostic(QStringLiteral("Embedded Product View repo root selected from %1: %2").arg(label, root));
      return true;
    }
    if (diagnostic_debug_logging_enabled()) log_diagnostic(QStringLiteral("Embedded Product View repo root candidate rejected from %1: %2 (missing %3)")
                     .arg(label, root, missing.join(QStringLiteral(", "))));
    return false;
  };

  const QString scene_input = selected_scene_dir.trimmed();
  if (!scene_input.isEmpty()) {
    QFileInfo scene_info(scene_input);
    if (scene_info.exists() && scene_info.isDir()) {
      QDir dir(canonical_path(scene_input));
      for (int i = 0; i < 16; ++i) {
        QString accepted;
        if (accept_candidate(dir.absolutePath(), QStringLiteral("selected scene directory upward walk"), &accepted)) return accepted;
        if (!dir.cdUp()) break;
      }
    } else {
      if (diagnostic_debug_logging_enabled()) log_diagnostic(QStringLiteral("Embedded Product View selected scene directory candidate rejected: %1 (missing directory)").arg(scene_input));
    }
  }

  if (!preview_context_.absolute_repo_root.trimmed().isEmpty()) {
    QString accepted;
    if (accept_candidate(preview_context_.absolute_repo_root, QStringLiteral("caller-provided repository root after selected scene"), &accepted)) return accepted;
  }

  if (!embedded_web_repo_root_.trimmed().isEmpty()) {
    QString accepted;
    if (accept_candidate(embedded_web_repo_root_, QStringLiteral("cached root after selected scene"), &accepted)) return accepted;
  }

  const QString env_root = QProcessEnvironment::systemEnvironment().value(QStringLiteral("WORKCELL_STUDIO_REPO_ROOT")).trimmed();
  if (!env_root.isEmpty()) {
    QString accepted;
    if (accept_candidate(env_root, QStringLiteral("WORKCELL_STUDIO_REPO_ROOT secondary override"), &accepted)) return accepted;
  }

  const QStringList fallback_roots{QDir::currentPath(), QCoreApplication::applicationDirPath()};
  for (const QString & fallback : fallback_roots) {
    QDir dir(fallback);
    for (int i = 0; i < 16; ++i) {
      QString accepted;
      if (accept_candidate(dir.absolutePath(), QStringLiteral("fallback application path upward walk"), &accepted)) return accepted;
      if (!dir.cdUp()) break;
    }
  }
  const QString summary_key = QStringLiteral("%1|%2|root_resolution_failed").arg(preview_scene_name_, scene_input);
  if (diagnostic_debug_logging_enabled() || !root_resolution_summary_keys_.contains(summary_key)) {
    root_resolution_summary_keys_.insert(summary_key);
    log_diagnostic(QStringLiteral("Embedded Product View repo root resolution failed for scene_dir=%1; checked selected scene upward walk, cached root, WORKCELL_STUDIO_REPO_ROOT, cwd/applicationDir fallbacks.").arg(scene_input.isEmpty() ? QStringLiteral("<unset>") : scene_input));
  }
  return QString();
}

QString ScenePreviewWidget::embedded_web_prepare_command_for_log(const QString & scene_dir, const QString & output_path, bool force) const
{
  QString command = QStringLiteral("python3 scripts/ensure_workcell_studio_web_scene_fresh.py --scene %1 --output %2 --stage-assets")
                      .arg(scene_dir, output_path);
  if (force) command += QStringLiteral(" --force");
  return command;
}

void ScenePreviewWidget::set_embedded_product_view_state(EmbeddedProductViewState state, const QString & detail)
{
  embedded_product_view_state_ = state;
  embedded_editor_polling_ = (state == EmbeddedProductViewState::Ready);
  if (state == EmbeddedProductViewState::Failed) embedded_web_last_error_ = detail;
  if (state == EmbeddedProductViewState::Failed && !detail.trimmed().isEmpty()) {
    emit studio_log_requested(QStringLiteral("Embedded Product View failure: %1").arg(detail));
  }
  refresh_toolbar_status_chip();
  refresh_toolbar_feedback_row();
  if (error_state_label_) error_state_label_->setVisible(false);
}

bool ScenePreviewWidget::embedded_web_server_is_usable(const QString & repo_root, const QString & scene)
{
  const QString trimmed_repo_root = repo_root.trimmed().isEmpty() ? embedded_web_repo_root_ : repo_root.trimmed();
  const QString trimmed_scene = scene.trimmed().isEmpty() ? embedded_web_prepare_scene_ : scene.trimmed();
  if (trimmed_repo_root.isEmpty() || trimmed_scene.isEmpty()) {
    emit studio_log_requested(QStringLiteral("Embedded Web 3D server probe failed: missing selected repository root or scene id."));
    return false;
  }

  const QDir repo_dir(trimmed_repo_root);
  const QString marker_path = QStringLiteral("workcell_studio_web/viewer/workcell_runtime_marker.json");
  QFile marker_file(repo_dir.filePath(marker_path));
  QByteArray expected_marker;
  if (marker_file.open(QIODevice::ReadOnly)) expected_marker = marker_file.readAll().trimmed();

  struct HttpCheck { QString label; QString path; int status{0}; bool ok{false}; QString detail; QByteArray body; };
  auto get_url = [this](const QString & path, int timeout_ms = 1200) -> HttpCheck {
    HttpCheck check;
    check.path = path;
    const QUrl url(QStringLiteral("http://127.0.0.1:%1/%2").arg(embedded_web_server_port_).arg(path));
    QNetworkAccessManager manager;
    QNetworkRequest request(url);
    QNetworkReply * reply = manager.get(request);
    QEventLoop loop;
    QTimer timer;
    timer.setSingleShot(true);
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    timer.start(timeout_ms);
    loop.exec();
    if (timer.isActive()) {
      timer.stop();
      check.status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
      check.body = reply->readAll();
      check.ok = (reply->error() == QNetworkReply::NoError && check.status == 200);
      if (!check.ok) check.detail = reply->errorString();
    } else {
      check.detail = QStringLiteral("timeout after %1 ms").arg(timeout_ms);
      reply->abort();
    }
    reply->deleteLater();
    return check;
  };

  QVector<HttpCheck> checks;
  auto add_check = [&](const QString & label, const QString & path) {
    HttpCheck check = get_url(path);
    check.label = label;
    checks.push_back(check);
    return check;
  };

  const HttpCheck marker_check = add_check(QStringLiteral("repo marker"), marker_path);
  if (expected_marker.isEmpty()) {
    checks.last().ok = false;
    checks.last().detail = QStringLiteral("local marker file missing or empty under selected repo root: %1").arg(marker_path);
  } else if (marker_check.body.trimmed() != expected_marker) {
    checks.last().ok = false;
    checks.last().detail = QStringLiteral("marker response does not match selected repository root marker content");
  }

  add_check(QStringLiteral("viewer index"), QStringLiteral("workcell_studio_web/viewer/index.html"));
  add_check(QStringLiteral("viewer bundle"), QStringLiteral("workcell_studio_web/viewer/dist/viewer.bundle.js"));
  const QString web_scene_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(trimmed_scene);
  HttpCheck scene_check = add_check(QStringLiteral("web scene"), web_scene_path);

  auto normalize_asset_path = [](QString value) {
    value = value.trimmed();
    if (value.startsWith(QStringLiteral("./"))) value = value.mid(2);
    if (value.startsWith(QStringLiteral("/"))) value = value.mid(1);
    if (value.startsWith(QStringLiteral("http://")) || value.startsWith(QStringLiteral("https://")) ||
        value.startsWith(QStringLiteral("package://")) || value.startsWith(QStringLiteral("file://"))) return QString();
    return value;
  };
  auto classify_asset = [](const QString & path) {
    const QString p = path.toLower();
    if (p.endsWith(QStringLiteral(".urdf")) && p.contains(QStringLiteral("robot_preview"))) return QStringLiteral("robot-preview URDF");
    if (p.contains(QStringLiteral("/ur_description/")) || p.contains(QStringLiteral("/ur5/"))) return QStringLiteral("UR5 mesh");
    if (p.contains(QStringLiteral("robotiq")) || p.contains(QStringLiteral("2f"))) return QStringLiteral("Robotiq mesh");
    if (p.contains(QStringLiteral("table")) || p.contains(QStringLiteral("workbench"))) return QStringLiteral("table/workbench mesh");
    if (p.contains(QStringLiteral("camera"))) return QStringLiteral("camera mesh");
    return QString();
  };
  std::function<void(const QJsonValue &, QSet<QString> &)> collect_assets = [&](const QJsonValue & value, QSet<QString> & out) {
    if (value.isObject()) {
      const QJsonObject obj = value.toObject();
      for (auto it = obj.begin(); it != obj.end(); ++it) {
        if (it.value().isString()) {
          const QString key = it.key().toLower();
          if (key.contains(QStringLiteral("mesh")) || key.contains(QStringLiteral("urdf")) || key.contains(QStringLiteral("uri")) || key.contains(QStringLiteral("path"))) {
            const QString path = normalize_asset_path(it.value().toString());
            if (!path.isEmpty() && classify_asset(path).size() > 0) out.insert(path);
          }
        }
        collect_assets(it.value(), out);
      }
    } else if (value.isArray()) {
      const QJsonArray array = value.toArray();
      for (const QJsonValue & entry : array) collect_assets(entry, out);
    }
  };

  if (scene_check.ok) {
    QJsonParseError parse_error;
    const QJsonDocument scene_doc = QJsonDocument::fromJson(scene_check.body, &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !scene_doc.isObject()) {
      checks.last().ok = false;
      checks.last().detail = QStringLiteral("web scene JSON parse failed: %1").arg(parse_error.errorString());
    } else {
      const QJsonObject scene_obj = scene_doc.object();
      if (scene_obj.value(QStringLiteral("schema_version")).toString() != QStringLiteral("workcell_studio_web_scene/v1") ||
          scene_obj.value(QStringLiteral("scene_id")).toString(scene_obj.value(QStringLiteral("scene_name")).toString()) != trimmed_scene) {
        checks.last().ok = false;
        checks.last().detail = QStringLiteral("web scene JSON does not match selected scene/repository contract");
      }
      QSet<QString> assets;
      collect_assets(scene_doc.object(), assets);
      QSet<QString> covered_classes;
      for (const QString & asset : assets) {
        const QString label = classify_asset(asset);
        covered_classes.insert(label);
        add_check(label, asset);
      }
      const QStringList required_classes{QStringLiteral("robot-preview URDF"), QStringLiteral("UR5 mesh"), QStringLiteral("Robotiq mesh"), QStringLiteral("table/workbench mesh"), QStringLiteral("camera mesh")};
      for (const QString & required : required_classes) {
        if (!covered_classes.contains(required)) {
          HttpCheck missing;
          missing.label = required;
          missing.path = QStringLiteral("<referenced asset missing from web scene JSON>");
          missing.ok = false;
          missing.detail = QStringLiteral("no referenced staged asset found for required class");
          checks.push_back(missing);
        }
      }
    }
  }

  QStringList diagnostic_lines{QStringLiteral("Embedded Web 3D server resource probe for repo=%1 scene=%2").arg(trimmed_repo_root, trimmed_scene)};
  bool all_ok = true;
  for (const HttpCheck & check : checks) {
    all_ok = all_ok && check.ok;
    diagnostic_lines << QStringLiteral("  [%1] %2 status=%3 result=%4%5")
                          .arg(check.label,
                               check.path,
                               check.status > 0 ? QString::number(check.status) : QStringLiteral("n/a"),
                               check.ok ? QStringLiteral("PASS") : QStringLiteral("FAIL"),
                               check.detail.isEmpty() ? QString() : QStringLiteral(" detail=%1").arg(check.detail.left(240)));
  }
  emit studio_log_requested(diagnostic_lines.join(QStringLiteral("\n")));
  return all_ok;
}

void ScenePreviewWidget::ensure_embedded_web_server_started(const QString & repo_root, const EmbeddedWebRequestIdentity & identity)
{
  if (repo_root.trimmed().isEmpty() || !embedded_web_identity_is_current(identity)) return;
  if (embedded_web_server_process_ && embedded_web_server_process_->state() != QProcess::NotRunning) {
    load_prepared_embedded_web_scene(identity);
    return;
  }
  if (embedded_web_server_is_usable(repo_root, identity.scene_id)) {
    emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View reused existing verified server at http://127.0.0.1:8765/."));
    load_prepared_embedded_web_scene(identity);
    return;
  }
  set_embedded_product_view_state(EmbeddedProductViewState::StartingServer);
  if (embedded_web_server_process_) embedded_web_server_process_->deleteLater();
  embedded_web_server_process_ = new QProcess(this);
  embedded_web_server_is_owned_ = true;
  QProcess * const process = embedded_web_server_process_;
  embedded_web_server_process_->setProgram(QStringLiteral("python3"));
  embedded_web_server_process_->setArguments(QStringList{"-m", "http.server", QString::number(embedded_web_server_port_), "--bind", "127.0.0.1"});
  embedded_web_server_process_->setWorkingDirectory(repo_root);
  embedded_web_server_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  embedded_web_server_process_->setProcessChannelMode(QProcess::MergedChannels);
  connect(process, &QProcess::started, this, [this, identity, process]() {
    if (process != embedded_web_server_process_ || !embedded_web_identity_is_current(identity)) return;
    emit studio_log_requested(QStringLiteral("Started embedded Web 3D Product View server from repo root: python3 -m http.server 8765 --bind 127.0.0.1"));
    load_prepared_embedded_web_scene(identity);
  });
  connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this, identity, process](int exit_code, QProcess::ExitStatus) {
    if (process != embedded_web_server_process_ || !embedded_web_identity_is_current(identity)) return;
    const QString output = QString::fromUtf8(process->readAll()).trimmed();
    if (embedded_product_view_state_ == EmbeddedProductViewState::StartingServer && !embedded_web_server_is_usable(embedded_web_repo_root_, identity.scene_id)) {
      activate_native_compatibility_preview(QStringLiteral("local server failed; exit %1; %2").arg(exit_code).arg(output.left(240)));
      emit studio_log_requested(QStringLiteral("Embedded Product View local server failed: python3 -m http.server 8765 --bind 127.0.0.1\nExit code: %1\nOutput:\n%2").arg(exit_code).arg(output));
    }
  });
  process->start();
}

void ScenePreviewWidget::cancel_embedded_web_lifecycle(bool stop_owned_server)
{
  // Every callback captures an identity.  Retiring it first makes queued browser,
  // timer, and process callbacks harmless before any UI or process state changes.
  ++embedded_web_request_generation_;
  ++embedded_web_navigation_token_;
  embedded_web_has_active_identity_ = false;
  embedded_web_active_identity_ = EmbeddedWebRequestIdentity{};
  embedded_web_loading_identity_ = EmbeddedWebRequestIdentity{};
  embedded_web_loading_navigation_token_ = 0;
  embedded_web_expected_viewer_url_ = QUrl();
  pending_embedded_web_identity_ = EmbeddedWebRequestIdentity{};
  pending_embedded_web_request_ = false;
  pending_embedded_web_force_ = false;
  embedded_editor_polling_ = false;
  embedded_web_readiness_deadline_ = QDateTime();
  embedded_web_last_boot_status_.clear();

  if (embedded_web_prepare_process_) {
    QProcess * const process = embedded_web_prepare_process_;
    const QString key = embedded_web_preparation_process_keys_.value(process);
    const auto diagnostic = embedded_web_preparation_diagnostics_.value(key);
    if (process->state() != QProcess::NotRunning) {
      // Record this before disconnecting: terminate()/kill() can otherwise
      // suppress the only observable terminal callback for this preparation.
      record_embedded_web_prepare_terminal(diagnostic.identity, process, QStringLiteral("cancelled"),
        process->exitStatus(), process->exitCode(), QStringLiteral("lifecycle cancelled"));
    }
    disconnect(process, nullptr, this, nullptr);
    if (process->state() != QProcess::NotRunning) {
      process->terminate();
      if (!process->waitForFinished(1000)) {
        process->kill();
        process->waitForFinished(1000);
      }
    }
    embedded_web_preparation_process_keys_.remove(process);
    embedded_web_prepare_process_ = nullptr;
    process->deleteLater();
  }

  if (stop_owned_server && embedded_web_server_is_owned_ && embedded_web_server_process_) {
    QProcess * const process = embedded_web_server_process_;
    disconnect(process, nullptr, this, nullptr);
    if (process->state() != QProcess::NotRunning) {
      process->terminate();
      if (!process->waitForFinished(1000)) {
        process->kill();
        process->waitForFinished(1000);
      }
    }
    embedded_web_server_process_ = nullptr;
    embedded_web_server_is_owned_ = false;
    process->deleteLater();
  }
}

void ScenePreviewWidget::request_embedded_web_product_view_refresh(bool force)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(force);
  return;
#else
  if (!embedded_web_view_) return;
  // Form the effective key before allocating a generation.  Automatic callers
  // can be noisy (payload delivery and UI state often arrive separately), so a
  // duplicate must leave both in-flight work and its callback identity intact.
  const EmbeddedWebRequestIdentity request_key = embedded_web_request_identity(0);
  if (request_key.scene_id.isEmpty() || request_key.scene_id == QStringLiteral("No scene")) {
    set_embedded_product_view_state(EmbeddedProductViewState::Idle);
    return;
  }

  // Automatic payload/context notifications are coalesced before they can
  // consume a generation or replace the active callback identity.
  if (!force && ((embedded_web_has_active_identity_ && embedded_web_active_identity_.matches_context(request_key)) ||
                 (pending_embedded_web_request_ && pending_embedded_web_identity_.matches_context(request_key)))) {
    return;
  }

  // Refresh Preview is deliberately the sole forced path.  It retires the
  // previous lifecycle, then creates one fresh generation and one retry.
  if (force) cancel_embedded_web_lifecycle(false);
  const EmbeddedWebRequestIdentity identity = embedded_web_request_identity(++embedded_web_request_generation_);

  embedded_web_active_identity_ = identity;
  embedded_web_has_active_identity_ = true;
  pending_embedded_web_identity_ = identity;
  pending_embedded_web_request_ = true;
  pending_embedded_web_force_ = force;
  maybe_start_next_embedded_web_prepare();
#endif
}

ScenePreviewWidget::EmbeddedWebRequestIdentity ScenePreviewWidget::embedded_web_request_identity(quint64 generation) const
{
  EmbeddedWebRequestIdentity identity;
  const PreviewContext normalized_context = normalized_preview_context(preview_context_);
  identity.scene_id = normalized_context.scene_id.isEmpty() ? preview_scene_name_.trimmed() : normalized_context.scene_id;
  const QString scene_dir = normalized_context.absolute_scene_dir;
  if (!scene_dir.isEmpty()) {
    const QFileInfo info(scene_dir);
    identity.absolute_scene_dir = QDir::cleanPath(info.exists() ?
      (info.canonicalFilePath().isEmpty() ? info.absoluteFilePath() : info.canonicalFilePath()) : info.absoluteFilePath());
  }
  identity.payload_fingerprint = preview_payload_fingerprint_;
  identity.payload_revision = static_cast<quint64>(preview_payload_revision_);
  identity.generation = generation;
  return identity;
}

bool ScenePreviewWidget::embedded_web_identity_is_current(const EmbeddedWebRequestIdentity & identity) const
{
  return embedded_web_has_active_identity_ && embedded_web_active_identity_ == identity;
}

QString ScenePreviewWidget::embedded_web_preparation_diagnostic_key(const EmbeddedWebRequestIdentity & identity) const
{
  // Include the complete immutable request identity, not merely the scene name:
  // a same-scene payload update must retain its own terminal diagnostic.
  return QStringLiteral("%1|%2|%3|%4|%5")
    .arg(identity.scene_id, identity.absolute_scene_dir, QString::fromLatin1(identity.payload_fingerprint.toHex()))
    .arg(identity.payload_revision).arg(identity.generation);
}

void ScenePreviewWidget::append_embedded_web_prepare_output(QProcess * process, bool standard_error)
{
  if (!process) return;
  const QString key = embedded_web_preparation_process_keys_.value(process);
  auto diagnostic = embedded_web_preparation_diagnostics_.find(key);
  if (diagnostic == embedded_web_preparation_diagnostics_.end()) return;
  QByteArray & tail = standard_error ? diagnostic->stderr_tail : diagnostic->stdout_tail;
  tail += standard_error ? process->readAllStandardError() : process->readAllStandardOutput();
  constexpr int kPreparationOutputTailBytes = 4096;
  if (tail.size() > kPreparationOutputTailBytes) tail = tail.right(kPreparationOutputTailBytes);
}

void ScenePreviewWidget::record_embedded_web_prepare_terminal(
  const EmbeddedWebRequestIdentity & identity, QProcess * process, const QString & outcome,
  QProcess::ExitStatus exit_status, int exit_code, const QString & detail)
{
  const QString key = embedded_web_preparation_diagnostic_key(identity);
  auto diagnostic = embedded_web_preparation_diagnostics_.find(key);
  if (diagnostic == embedded_web_preparation_diagnostics_.end() || diagnostic->terminal_recorded) return;
  if (process) {
    append_embedded_web_prepare_output(process, false);
    append_embedded_web_prepare_output(process, true);
  }
  diagnostic->terminal_recorded = true;
  diagnostic->terminal_outcome = outcome;
  const bool expected_json_exists = QFileInfo::exists(diagnostic->expected_output_absolute_path);
  QString message = QStringLiteral("Embedded Product View preparation terminal: outcome=%1 scene=%2 generation=%3 payload_revision=%4 exit_status=%5 exit_code=%6 expected_json_exists=%7")
    .arg(outcome, identity.scene_id).arg(identity.generation).arg(identity.payload_revision)
    .arg(exit_status == QProcess::NormalExit ? QStringLiteral("normal") : QStringLiteral("crash"))
    .arg(exit_code).arg(expected_json_exists ? QStringLiteral("true") : QStringLiteral("false"));
  if (!detail.isEmpty()) message += QStringLiteral(" detail=%1").arg(detail.left(240));
  if (outcome != QStringLiteral("success")) {
    message += QStringLiteral(" stdout_tail=%1 stderr_tail=%2")
      .arg(QString::fromUtf8(diagnostic->stdout_tail).trimmed().left(4096),
           QString::fromUtf8(diagnostic->stderr_tail).trimmed().left(4096));
  }
  emit studio_log_requested(message);
}

void ScenePreviewWidget::refresh_embedded_web_product_view()
{
  ++embedded_web_preparation_request_count_;
  request_embedded_web_product_view_refresh(false);
}

void ScenePreviewWidget::maybe_start_next_embedded_web_prepare()
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  return;
#else
  if (embedded_web_prepare_process_ && embedded_web_prepare_process_->state() != QProcess::NotRunning) return;
  if (!pending_embedded_web_request_) return;
  const EmbeddedWebRequestIdentity identity = pending_embedded_web_identity_;
  const bool force = pending_embedded_web_force_;
  pending_embedded_web_request_ = false;
  pending_embedded_web_force_ = false;
  // A newer forced request can retire the pending identity while the previous
  // process is finishing.  Do not revive that stale work.
  if (!embedded_web_identity_is_current(identity)) {
    maybe_start_next_embedded_web_prepare();
    return;
  }
  start_embedded_web_prepare(identity, force);
#endif
}

void ScenePreviewWidget::emit_backend_startup_diagnostic_once()
{
  if (backend_startup_diagnostic_emitted_) return;
  backend_startup_diagnostic_emitted_ = true;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (product_view_backend_ == ProductViewBackend::EmbeddedWeb3D) {
    emit studio_log_requested(QStringLiteral("Workcell Product View backend=embedded_web3d webengine_compiled=true"));
  } else {
    emit studio_log_requested(QStringLiteral("Workcell Product View backend=native_compatibility reason=explicit_opt_in"));
  }
#else
  emit studio_log_requested(QStringLiteral("Workcell Product View backend=native_compatibility reason=webengine_unavailable_fallback"));
#endif
}

void ScenePreviewWidget::start_embedded_web_prepare(const EmbeddedWebRequestIdentity & identity, bool force)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(force);
#else
  const QString scene_id = identity.scene_id;
  const QFileInfo selected_scene_info(identity.absolute_scene_dir);
  if (!is_safe_embedded_web_scene_id(scene_id)) {
    activate_native_compatibility_preview(QStringLiteral("selected scene ID is missing or unsafe for Product View output and URL: %1")
      .arg(scene_id.isEmpty() ? QStringLiteral("<unset>") : scene_id));
    return;
  }
  if (identity.absolute_scene_dir.isEmpty() || !selected_scene_info.isAbsolute() ||
      !selected_scene_info.exists() || !selected_scene_info.isDir()) {
    activate_native_compatibility_preview(QStringLiteral("selected scene directory is required and must exist: %1")
      .arg(identity.absolute_scene_dir.isEmpty() ? QStringLiteral("<unset>") : identity.absolute_scene_dir));
    return;
  }
  const QString canonical_scene_dir = selected_scene_info.canonicalFilePath();
  const QString selected_scene_dir = QDir::cleanPath(canonical_scene_dir.isEmpty() ? selected_scene_info.absoluteFilePath() : canonical_scene_dir);
  if (!scene_directory_matches_id(selected_scene_dir, scene_id)) {
    activate_native_compatibility_preview(QStringLiteral("selected scene directory %1 does not match requested scene ID %2 by directory name or scene metadata")
      .arg(selected_scene_dir, scene_id));
    return;
  }
  const QString repo_root = resolve_embedded_web_repo_root(selected_scene_dir);
  if (repo_root.isEmpty()) {
    const QString detail = QStringLiteral("could not find a Workcell Studio repo root with viewer, scene-prep script, and scenes markers");
    activate_native_compatibility_preview(detail);
    emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View unavailable: could not find a Workcell Studio repo root with required markers from selected scene, environment override, or fallback application paths."));
    return;
  }
  // Keep a working compatibility viewport visible until Web3D has completed
  // its readiness handshake.  A retry must not briefly replace usable content.
  embedded_web_repo_root_ = repo_root;
  embedded_web_prepare_scene_ = scene_id;
  embedded_web_prepare_scene_dir_ = selected_scene_dir;
  embedded_web_prepare_output_path_ = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(scene_id);
  if (embedded_web_prepare_process_) embedded_web_prepare_process_->deleteLater();
  embedded_web_prepare_process_ = new QProcess(this);
  embedded_web_prepare_process_->setProgram(QStringLiteral("python3"));
  QStringList args{"scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", selected_scene_dir, "--output", embedded_web_prepare_output_path_, "--stage-assets"};
  if (force) args << "--force";
  embedded_web_prepare_process_->setArguments(args);
  embedded_web_prepare_process_->setWorkingDirectory(repo_root);
  embedded_web_prepare_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  embedded_web_prepare_process_->setProcessChannelMode(QProcess::SeparateChannels);
  QProcess * const process = embedded_web_prepare_process_;
  const QString diagnostic_key = embedded_web_preparation_diagnostic_key(identity);
  EmbeddedWebPreparationDiagnostic diagnostic;
  diagnostic.identity = identity;
  diagnostic.expected_output_path = embedded_web_prepare_output_path_;
  diagnostic.expected_output_absolute_path = QDir(repo_root).filePath(diagnostic.expected_output_path);
  embedded_web_preparation_diagnostics_.insert(diagnostic_key, diagnostic);
  embedded_web_preparation_process_keys_.insert(process, diagnostic_key);
  embedded_web_prepare_started_at_ = QDateTime::currentDateTimeUtc();
  connect(process, &QProcess::started, this, [this, identity, process]() {
    const QString key = embedded_web_preparation_process_keys_.value(process);
    if (auto diagnostic = embedded_web_preparation_diagnostics_.find(key); diagnostic != embedded_web_preparation_diagnostics_.end()) diagnostic->started = true;
  });
  connect(process, &QProcess::readyReadStandardOutput, this, [this, process]() {
    append_embedded_web_prepare_output(process, false);
  });
  connect(process, &QProcess::readyReadStandardError, this, [this, process]() {
    append_embedded_web_prepare_output(process, true);
  });
  connect(process, &QProcess::errorOccurred, this, [this, identity, process](QProcess::ProcessError error) {
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("process_error"), process->exitStatus(), process->exitCode(),
      QStringLiteral("qprocess_error=%1").arg(static_cast<int>(error)));
  });
  connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this, identity, process](int exit_code, QProcess::ExitStatus exit_status) {
    on_embedded_web_prepare_finished(identity, process, exit_code, exit_status);
  });
  set_embedded_product_view_state(EmbeddedProductViewState::Preparing, scene_id);
  emit studio_log_requested(QStringLiteral("Preparing embedded Product View from repo root %1: %2").arg(repo_root, embedded_web_prepare_command_for_log(selected_scene_dir, embedded_web_prepare_output_path_, force)));
  embedded_web_prepare_process_->start();
#endif
}

void ScenePreviewWidget::on_embedded_web_prepare_finished(const EmbeddedWebRequestIdentity & identity, QProcess * process, int exit_code, QProcess::ExitStatus exit_status)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(process); Q_UNUSED(exit_code); Q_UNUSED(exit_status);
#else
  if (!process) return;
  append_embedded_web_prepare_output(process, false);
  append_embedded_web_prepare_output(process, true);

  // A stale process is never allowed to touch current UI state, but its
  // immutable request still receives one explicit discarded terminal record.
  if (process != embedded_web_prepare_process_) {
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("stale_discarded"), exit_status, exit_code,
      QStringLiteral("process no longer owns the active preparation slot"));
    embedded_web_preparation_process_keys_.remove(process);
    process->deleteLater();
    return;
  }

  // A real automatic replacement updates the active identity without killing
  // the old process.  Retire that completion before reading output, changing
  // UI state, or loading its scene, then start the valid pending replacement.
  if (!embedded_web_identity_is_current(identity)) {
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("stale_discarded"), exit_status, exit_code,
      QStringLiteral("request identity retired before completion"));
    process->deleteLater();
    embedded_web_preparation_process_keys_.remove(process);
    embedded_web_prepare_process_ = nullptr;
    maybe_start_next_embedded_web_prepare();
    return;
  }
  const EmbeddedWebPreparationDiagnostic existing_diagnostic = embedded_web_preparation_diagnostics_.value(
    embedded_web_preparation_diagnostic_key(identity));
  if (existing_diagnostic.terminal_recorded && existing_diagnostic.terminal_outcome == QStringLiteral("process_error")) {
    // errorOccurred already emitted the only terminal result.  Do not let a
    // later finished callback turn that process error into a successful load.
    process->deleteLater();
    embedded_web_preparation_process_keys_.remove(process);
    embedded_web_prepare_process_ = nullptr;
    activate_native_compatibility_preview(QStringLiteral("Product View preparation process error"));
    maybe_start_next_embedded_web_prepare();
    return;
  }
  const QString scene = identity.scene_id;
  const QString output_path = embedded_web_prepare_output_path_;
  const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(
    embedded_web_preparation_diagnostic_key(identity));
  const QString stdout_text = QString::fromUtf8(diagnostic.stdout_tail).trimmed();
  const QString command = embedded_web_prepare_command_for_log(embedded_web_prepare_scene_dir_, output_path, false);
  process->deleteLater();
  embedded_web_preparation_process_keys_.remove(process);
  embedded_web_prepare_process_ = nullptr;

  const QString absolute_output_path = QDir(embedded_web_repo_root_).filePath(output_path);
  const bool output_is_fresh = QFileInfo::exists(QDir(embedded_web_repo_root_).filePath(output_path));  // Contract validation supersedes mtime freshness.
  Q_UNUSED(output_is_fresh);
  auto reject_prepare = [&](const QString & reason) {
    activate_native_compatibility_preview(reason);
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("command_failure"), exit_status, exit_code,
      QStringLiteral("%1; command=%2").arg(reason, command));
    maybe_start_next_embedded_web_prepare();
  };

  if (exit_status != QProcess::NormalExit || exit_code != 0) {
    reject_prepare(QStringLiteral("prepare command failed with exit code %1; old output is rejected even if present").arg(exit_code));
    return;
  }

  QJsonParseError result_parse_error;
  const QJsonDocument result_doc = QJsonDocument::fromJson(stdout_text.toUtf8(), &result_parse_error);
  if (result_parse_error.error != QJsonParseError::NoError || !result_doc.isObject()) {
    reject_prepare(QStringLiteral("freshener did not return a valid JSON object on stdout: %1").arg(result_parse_error.errorString()));
    return;
  }
  const QJsonObject result = result_doc.object();
  const QString freshener_schema = result.value(QStringLiteral("schema_version")).toString();
  const QString freshener_status = result.value(QStringLiteral("status")).toString();
  const QString freshener_scene = result.value(QStringLiteral("scene_id")).toString();
  const QString freshener_output = result.value(QStringLiteral("output")).toString();
  if (freshener_schema != QStringLiteral("workcell_studio_web_scene_freshener/v1") ||
      (freshener_status != QStringLiteral("current") && freshener_status != QStringLiteral("rebuilt")) ||
      freshener_scene != scene || freshener_output != output_path) {
    reject_prepare(QStringLiteral("freshener JSON contract validation failed for scene %1").arg(scene));
    return;
  }
  const QJsonObject asset_diagnostics = result.value(QStringLiteral("staged_asset_diagnostics")).toObject();
  if (asset_diagnostics.contains(QStringLiteral("ok")) && !asset_diagnostics.value(QStringLiteral("ok")).toBool()) {
    reject_prepare(QStringLiteral("freshener reported missing staged assets for %1").arg(scene));
    return;
  }

  QFile output_file(absolute_output_path);
  if (!output_file.open(QIODevice::ReadOnly)) {
    reject_prepare(QStringLiteral("expected output missing or unreadable: %1").arg(output_path));
    return;
  }
  QJsonParseError output_parse_error;
  const QJsonDocument output_doc = QJsonDocument::fromJson(output_file.readAll(), &output_parse_error);
  if (output_parse_error.error != QJsonParseError::NoError || !output_doc.isObject()) {
    reject_prepare(QStringLiteral("prepared output JSON validation failed: %1").arg(output_parse_error.errorString()));
    return;
  }
  const QJsonObject output = output_doc.object();
  const QString web_schema = output.value(QStringLiteral("schema_version")).toString();
  const QString output_scene = output.value(QStringLiteral("scene_id")).toString(output.value(QStringLiteral("scene_name")).toString());
  if (web_schema != QStringLiteral("workcell_studio_web_scene/v1") || output_scene != scene) {
    reject_prepare(QStringLiteral("prepared output semantic validation failed for scene %1").arg(scene));
    return;
  }

  record_embedded_web_prepare_terminal(identity, process, QStringLiteral("success"), exit_status, exit_code);
  ensure_embedded_web_server_started(embedded_web_repo_root_, identity);
  maybe_start_next_embedded_web_prepare();
#endif
}

void ScenePreviewWidget::start_embedded_web_readiness_polling(const EmbeddedWebRequestIdentity & identity, quint64 navigation_token, const QString & expected_json_path, const QString & viewer_url)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(navigation_token); Q_UNUSED(expected_json_path); Q_UNUSED(viewer_url);
#else
  if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_) return;
  embedded_web_readiness_deadline_ = QDateTime::currentDateTimeUtc().addSecs(45);
  embedded_web_last_boot_status_ = QStringLiteral("browser_loaded");
  poll_embedded_web_readiness(identity, navigation_token, expected_json_path, viewer_url);
#endif
}

void ScenePreviewWidget::poll_embedded_web_readiness(const EmbeddedWebRequestIdentity & identity, quint64 navigation_token, const QString & expected_json_path, const QString & viewer_url)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(navigation_token); Q_UNUSED(expected_json_path); Q_UNUSED(viewer_url);
#else
  if (!embedded_web_view_) return;
  const QUrl expected_viewer_url(viewer_url);
  if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
      embedded_web_view_->url() != expected_viewer_url) {
    emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness poll for scene %1 revision %2 navigation %3.")
      .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
    return;
  }

  if (QDateTime::currentDateTimeUtc() > embedded_web_readiness_deadline_) {
    const QString detail = QStringLiteral(
      "startup timed out after 45s for scene %1; viewer URL: %2; expected JSON: %3; last observed boot status: %4")
      .arg(identity.scene_id, viewer_url, expected_json_path, embedded_web_last_boot_status_.isEmpty() ? QStringLiteral("unavailable") : embedded_web_last_boot_status_);
    activate_native_compatibility_preview(detail);
    emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
    return;
  }

  static const char kStatusScript[] = R"JS(
(() => {
  const s = window.__WORKCELL_VIEWER_STATUS__ || {};
  return {
    viewer_boot_state: s.viewer_boot_state || s.viewerBootState || 'booting',
    scene_name: s.scene_name || s.sceneName || '',
    source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
    scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded || s.source_web_scene_file || s.sourceWebSceneFile),
    robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
    failed_stage: s.failed_stage || s.failedStage || '',
    fatal_error: s.fatal_error || s.fatalError || '',
    fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
  };
})()
)JS";
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, expected_json_path, viewer_url](const QVariant & value) {
    if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
        embedded_web_view_->url() != QUrl(viewer_url)) {
      emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")
        .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
      return;
    }
    const QVariantMap status = value.toMap();
    const QString boot_state = status.value(QStringLiteral("viewer_boot_state")).toString();
    const QString source_json = status.value(QStringLiteral("source_web_scene_file")).toString();
    const bool scene_json_loaded = status.value(QStringLiteral("scene_json_loaded")).toBool();
    const QString robot_state = status.value(QStringLiteral("robot_preview_lifecycle_state")).toString();
    const QString failed_stage = status.value(QStringLiteral("failed_stage")).toString();
    const QString fatal_error = status.value(QStringLiteral("fatal_error")).toString();
    const QString fatal_stack = status.value(QStringLiteral("fatal_stack")).toString();
    embedded_web_last_boot_status_ = QStringLiteral("boot=%1 json=%2 source=%3 robot=%4")
      .arg(boot_state.isEmpty() ? QStringLiteral("unknown") : boot_state)
      .arg(scene_json_loaded ? QStringLiteral("loaded") : QStringLiteral("not_loaded"))
      .arg(source_json.isEmpty() ? QStringLiteral("unknown") : source_json)
      .arg(robot_state.isEmpty() ? QStringLiteral("unknown") : robot_state);

    if (boot_state == QStringLiteral("failed")) {
      const QString detail = QStringLiteral("viewer JavaScript failed at %1: %2%3")
        .arg(failed_stage.isEmpty() ? QStringLiteral("unknown_stage") : failed_stage,
             fatal_error.isEmpty() ? QStringLiteral("unknown error") : fatal_error,
             fatal_stack.isEmpty() ? QString() : QStringLiteral("; stack: %1").arg(fatal_stack.left(500)));
      activate_native_compatibility_preview(detail);
      emit studio_log_requested(QStringLiteral("Embedded Product View JavaScript failure for scene %1.\nStage: %2\nFatal error: %3\nStack excerpt:\n%4")
        .arg(identity.scene_id,
             failed_stage.isEmpty() ? QStringLiteral("unknown_stage") : failed_stage,
             fatal_error.isEmpty() ? QStringLiteral("unknown error") : fatal_error,
             fatal_stack.left(500)));
      return;
    }

    const bool expected_json_loaded = scene_json_loaded && source_json == expected_json_path;
    const bool robot_ready_required = identity.scene_id == QStringLiteral("ur5_2f_test");
    const bool robot_ready = !robot_ready_required || robot_state == QStringLiteral("ready");
    if (boot_state == QStringLiteral("ready") && expected_json_loaded && robot_ready) {
      native_compatibility_fallback_active_ = false;
      show_embedded_web_product_view();
      set_embedded_product_view_state(EmbeddedProductViewState::Ready, QStringLiteral("viewer ready"));
      poll_embedded_editor_events();
      emit studio_log_requested(QStringLiteral("Embedded Product View ready: scene=%1 json=%2 robot_preview_lifecycle_state=%3")
        .arg(identity.scene_id, expected_json_path, robot_state.isEmpty() ? QStringLiteral("not_required") : robot_state));
      return;
    }

    set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness,
      QStringLiteral("waiting for viewer readiness (%1)").arg(embedded_web_last_boot_status_));
    QTimer::singleShot(750, this, [this, identity, navigation_token, expected_json_path, viewer_url]() {
      poll_embedded_web_readiness(identity, navigation_token, expected_json_path, viewer_url);
    });
  });
#endif
}

void ScenePreviewWidget::load_prepared_embedded_web_scene(const EmbeddedWebRequestIdentity & identity)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity);
#else
  if (!embedded_web_view_) return;
  if (!embedded_web_identity_is_current(identity)) {
    emit studio_log_requested(QStringLiteral("Embedded Product View ignored stale prepared scene %1 rev %2; current scene is %3 rev %4.").arg(identity.scene_id).arg(identity.generation).arg(preview_scene_name_).arg(embedded_web_request_generation_));
    return;
  }
  const QString web_scene_url_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(identity.scene_id);
  const QString viewer_url = QStringLiteral("http://127.0.0.1:%1/workcell_studio_web/viewer/index.html?scene=%2&builderRevision=%3")
    .arg(embedded_web_server_port_)
    .arg(QString::fromUtf8(QUrl::toPercentEncoding(web_scene_url_path)))
    .arg(identity.generation) + QStringLiteral("&embedded=1");
  embedded_web_readiness_deadline_ = QDateTime();
  embedded_web_last_boot_status_.clear();
  set_embedded_product_view_state(EmbeddedProductViewState::Loading);
  embedded_web_loading_identity_ = identity;
  embedded_web_loading_navigation_token_ = ++embedded_web_navigation_token_;
  embedded_web_expected_viewer_url_ = QUrl(viewer_url);
  embedded_web_last_viewer_url_ = embedded_web_expected_viewer_url_.toString();
  embedded_web_view_->load(embedded_web_expected_viewer_url_);
#endif
}


#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
void ScenePreviewWidget::run_embedded_editor_command(const QString & script)
{
  if (!embedded_web_view_ || embedded_product_view_state_ != EmbeddedProductViewState::Ready) return;
  const EmbeddedWebRequestIdentity identity = embedded_web_active_identity_;
  embedded_web_view_->page()->runJavaScript(script, [this, identity](const QVariant & value){
    if (!embedded_web_identity_is_current(identity)) return;
    apply_embedded_editor_state(value.toMap());
  });
}

QString ScenePreviewWidget::embedded_snap_command(const QString & choice) const
{
  double t = 0.0; double r = 0.0; bool enabled = choice != QStringLiteral("Off");
  if (choice == QStringLiteral("1 cm")) t = 0.01; else if (choice == QStringLiteral("5 cm")) t = 0.05; else if (choice == QStringLiteral("10 cm")) t = 0.10;
  else if (choice == QStringLiteral("5 deg")) r = 5.0; else if (choice == QStringLiteral("15 deg")) r = 15.0;
  return QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.setSnap(%1,%2,%3)")
    .arg(enabled ? QStringLiteral("true") : QStringLiteral("false")).arg(t, 0, 'f', 3).arg(r, 0, 'f', 1);
}

void ScenePreviewWidget::apply_embedded_editor_state(const QVariantMap & state)
{
  if (state.isEmpty()) return;
  if (embedded_undo_button_) embedded_undo_button_->setEnabled(state.value(QStringLiteral("canUndo")).toBool());
  if (embedded_redo_button_) embedded_redo_button_->setEnabled(state.value(QStringLiteral("canRedo")).toBool());
  // Selection and dirty-state detail remains available through the embedded
  // editor and Studio Log. The header chip always reports runtime state.
  refresh_toolbar_status_chip();
}

void ScenePreviewWidget::poll_embedded_editor_events()
{
  if (!embedded_editor_polling_ || !embedded_web_view_ || embedded_product_view_state_ != EmbeddedProductViewState::Ready) return;
  const EmbeddedWebRequestIdentity identity = embedded_web_active_identity_;
  static const char kPoll[] = "(() => { const api = window.__WORKCELL_EDITOR_API_V1__; if (!api) return {state:{},events:[]}; return {state:api.getState(),events:api.drainEvents()}; })()";
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kPoll), [this, identity](const QVariant & value){
    if (!embedded_web_identity_is_current(identity)) return;
    const QVariantMap payload = value.toMap();
    apply_embedded_editor_state(payload.value(QStringLiteral("state")).toMap());
    for (const QVariant & event_value : payload.value(QStringLiteral("events")).toList()) {
      const QVariantMap event = event_value.toMap();
      if (event.value(QStringLiteral("type")).toString() == QStringLiteral("selection_changed")) {
        const QString id = event.value(QStringLiteral("itemId")).toString();
        if (id != selected_preview_item_id_) { selected_preview_item_id_ = id; emit preview_item_selected(id, event.value(QStringLiteral("itemType")).toString()); }
      }
    }
    if (embedded_editor_polling_) QTimer::singleShot(200, this, [this, identity]() {
      if (embedded_web_identity_is_current(identity)) poll_embedded_editor_events();
    });
  });
}
#else
void ScenePreviewWidget::run_embedded_editor_command(const QString & script) { Q_UNUSED(script); }
QString ScenePreviewWidget::embedded_snap_command(const QString & choice) const { Q_UNUSED(choice); return QString(); }
void ScenePreviewWidget::apply_embedded_editor_state(const QVariantMap & state) { Q_UNUSED(state); }
void ScenePreviewWidget::poll_embedded_editor_events() {}
#endif

ScenePreviewWidget::ProductViewBackend ScenePreviewWidget::active_product_view_backend() const
{
  return product_view_backend_;
}

bool ScenePreviewWidget::is_native_product_view_backend() const
{
  return product_view_backend_ == ProductViewBackend::NativeScene3D;
}

Scene3DViewportWidget * ScenePreviewWidget::active_native_viewport() const
{
  if (native_compatibility_fallback_active_ && compatibility_scene3d_viewport_) {
    return compatibility_scene3d_viewport_;
  }
  return qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
}

void ScenePreviewWidget::show_embedded_web_product_view()
{
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (!embedded_web_view_) return;
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(false);
  embedded_web_view_->setVisible(true);
  if (mode_selector_) mode_selector_->setItemText(0, QStringLiteral("Web3D Product View"));
#endif
  refresh_mode_and_state();
}


void ScenePreviewWidget::set_preview_context(const PreviewContext & context)
{
  const PreviewContext normalized = normalized_preview_context(context);
  const bool context_changed = !preview_contexts_equal(preview_context_, normalized);
  if (!context_changed) return;

  const QString effective_scene_name = normalized.scene_id.isEmpty() ?
    QStringLiteral("No scene") : normalized.scene_id;
  const bool scene_name_changed = preview_scene_name_ != effective_scene_name;
  if (context_changed) cancel_embedded_web_lifecycle(false);
  preview_context_ = normalized;
  root_resolution_summary_keys_.clear();
  if (!normalized.scene_id.isEmpty()) {
    set_preview_scene_name(normalized.scene_id);
    if (!scene_name_changed) refresh_embedded_web_product_view();
  } else {
    refresh_embedded_web_product_view();
  }
}

void ScenePreviewWidget::activate_native_compatibility_preview(const QString & reason)
{
  native_compatibility_fallback_active_ = true;
  embedded_web_last_error_ = reason;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (!compatibility_scene3d_viewport_) {
    compatibility_scene3d_viewport_ = new Scene3DViewportWidget(view3d_container_);
    compatibility_scene3d_viewport_->setObjectName("nativeCompatibilityScene3dViewport");
    if (auto * layout = qobject_cast<QVBoxLayout *>(view3d_container_->layout())) {
      const int web_index = layout->indexOf(simple_3d_view_);
      layout->insertWidget(web_index < 0 ? 0 : web_index + 1, compatibility_scene3d_viewport_);
    }
    compatibility_scene3d_viewport_->select_cb = [this](const QString & id, const QString & role) {
      select_preview_item(id);
      emit preview_item_selected(id, role);
    };
    compatibility_scene3d_viewport_->status_message_cb = [this](const QString & message) {
      emit studio_log_requested(message);
    };
  }
  // Hide the browser before exposing the fallback so the WebEngine surface
  // cannot cover the compatibility viewport during the handoff.
  if (embedded_web_view_) embedded_web_view_->setVisible(false);

  // Always refresh the retained viewport: a fallback can happen after the
  // payload or selection changed while Web3D was preparing a retry.
  compatibility_scene3d_viewport_->scene_name = preview_scene_name_;
  compatibility_scene3d_viewport_->ingest_preview_items(preview_items_);
  const bool selection_is_current = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & item) {
    return item.id == selected_preview_item_id_;
  });
  if (!selection_is_current) selected_preview_item_id_.clear();
  compatibility_scene3d_viewport_->selected_id = selected_preview_item_id_;
  compatibility_scene3d_viewport_->fit_include_overlays = false;
  compatibility_scene3d_viewport_->setVisible(true);
  compatibility_scene3d_viewport_->raise();
  compatibility_scene3d_viewport_->fit_product_view();
#endif
  set_embedded_product_view_state(native_compatibility_viewport_has_usable_content()
      ? EmbeddedProductViewState::CompatibilityReady
      : EmbeddedProductViewState::Failed,
    reason);
  if (fallback_banner_label_) {
    fallback_banner_label_->setText(QStringLiteral("Web3D unavailable — using native compatibility preview"));
    fallback_banner_label_->setVisible(false);
  }
  if (error_state_label_) {
    error_state_label_->setVisible(false);
  }
  refresh_toolbar_status_chip();
  refresh_toolbar_feedback_row();
  if (mode_selector_) mode_selector_->setItemText(0, QStringLiteral("3D Layout Preview"));
  refresh_mode_and_state();
}

QString ScenePreviewWidget::runtime_preview_status_text() const
{
  if (embedded_product_view_state_ == EmbeddedProductViewState::Preparing || embedded_product_view_state_ == EmbeddedProductViewState::StartingServer || embedded_product_view_state_ == EmbeddedProductViewState::Loading || embedded_product_view_state_ == EmbeddedProductViewState::WaitingForBrowserReadiness) {
    return QStringLiteral("Preparing preview");
  }
  if (embedded_product_view_state_ == EmbeddedProductViewState::Ready && !native_compatibility_fallback_active_) {
    return QStringLiteral("Preview ready");
  }
  if (embedded_product_view_state_ == EmbeddedProductViewState::CompatibilityReady && native_compatibility_viewport_has_usable_content()) {
    return QStringLiteral("Preview available in compatibility mode");
  }
  if (runtime_preview_has_usable_content()) return QStringLiteral("Preview ready");
  if (embedded_product_view_state_ == EmbeddedProductViewState::Failed) return QStringLiteral("Preview failed");
  return QStringLiteral("Preview idle");
}

void ScenePreviewWidget::refresh_toolbar_status_chip()
{
  if (!toolbar_status_chip_) return;
  const QString status = runtime_preview_status_text();
  toolbar_status_chip_->setText(status);
  toolbar_status_chip_->setToolTip(status);
}

void ScenePreviewWidget::refresh_toolbar_feedback_row()
{
  if (!toolbar_feedback_row_ || !toolbar_feedback_label_) return;
  QString message;
  if (native_compatibility_fallback_active_) {
    message = QStringLiteral("Web3D unavailable — using native compatibility preview. Details are in Studio Log; use Refresh Preview to retry.");
  } else if (embedded_product_view_state_ == EmbeddedProductViewState::Failed) {
    message = QStringLiteral("Product View unavailable. Details are in Studio Log; use Refresh Preview to retry.");
  } else if (!preview3d_available_ && scene_selected_) {
    message = QStringLiteral("3D preview unavailable — using 2D fallback. Details are in Studio Log.");
  }
  toolbar_feedback_label_->setText(message);
  toolbar_feedback_label_->setVisible(!message.isEmpty());
  toolbar_feedback_row_->setVisible(!message.isEmpty());
}

bool ScenePreviewWidget::runtime_preview_has_usable_content() const
{
  if (embedded_product_view_state_ == EmbeddedProductViewState::Ready && !native_compatibility_fallback_active_) return true;
  if (native_compatibility_fallback_active_) return native_compatibility_viewport_has_usable_content();
  const auto counters = render_debug_counters();
  return counters.viewport_received_count > 0 &&
    (counters.visible_count > 0 || counters.rendered_count > 0 || counters.unique_visible_item_count > 0);
}

bool ScenePreviewWidget::native_compatibility_viewport_has_usable_content() const
{
  if (!compatibility_scene3d_viewport_) return false;
  const auto counters = compatibility_scene3d_viewport_->render_debug_counters();
  return counters.viewport_received_count > 0 &&
    (counters.visible_count > 0 || counters.rendered_count > 0 || counters.unique_visible_item_count > 0);
}

void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); if (fallback_2d_view_ && fallback_2d_view_->scene() && info_chip_label_ && !fallback_info_chip_proxy_) { fallback_info_chip_proxy_ = fallback_2d_view_->scene()->addWidget(info_chip_label_); fallback_info_chip_proxy_->setZValue(10000.0); fallback_info_chip_proxy_->setPos(12.0, 12.0); } refresh_info_chip(); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){
  preview3d_available_ = available;
  unavailable_reason_ = reason;
  const QString help_tooltip = scene_preview_mouse_help_tooltip(preview3d_available_ ? QString() : unavailable_reason_);
  if (fallback_banner_label_) fallback_banner_label_->setToolTip(help_tooltip);
  if (view3d_container_) view3d_container_->setToolTip(help_tooltip);
  if (view2d_container_) view2d_container_->setToolTip(help_tooltip);
  if (!mode_default_initialized_) {
    mode_selector_->setCurrentText(preview3d_available_ ? "3D Layout Preview" : "2D Layout");
    mode_default_initialized_ = true;
  }
  refresh_mode_and_state();
}
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
QByteArray ScenePreviewWidget::preview_payload_fingerprint(const QVector<PreviewItem> & items)
{
  QVector<QByteArray> serialized_items;
  serialized_items.reserve(items.size());
  for (const PreviewItem & item : items) serialized_items.append(serialized_preview_item(item));
  // Payload producers do not promise a stable item order.  Sort complete item
  // records (rather than only IDs) so duplicate or empty IDs remain deterministic.
  std::sort(serialized_items.begin(), serialized_items.end());

  QCryptographicHash hash(QCryptographicHash::Sha256);
  constexpr char kFingerprintSchema[] = "ScenePreviewWidget/PreviewItem/v1";
  hash.addData(kFingerprintSchema, sizeof(kFingerprintSchema) - 1);
  for (const QByteArray & serialized_item : serialized_items) {
    const quint32 length = static_cast<quint32>(serialized_item.size());
    const char length_bytes[] = {
      static_cast<char>(length & 0xffU), static_cast<char>((length >> 8U) & 0xffU),
      static_cast<char>((length >> 16U) & 0xffU), static_cast<char>((length >> 24U) & 0xffU)
    };
    hash.addData(length_bytes, sizeof(length_bytes));
    hash.addData(serialized_item);
  }
  return hash.result();
}

void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items)
{
  const QByteArray effective_fingerprint = preview_payload_fingerprint(items);
  const bool payload_changed = effective_fingerprint != preview_payload_fingerprint_;
  // Always ingest the caller's objects so selection, native rendering, and
  // editor state observe the current payload even when its rendering contract
  // is equivalent to the prior one.
  preview_items_ = items;
  preview_payload_fingerprint_ = effective_fingerprint;
  if (payload_changed) {
    ++preview_payload_revision_;
    ++preview_payload_generation_;
  }
  auto * viewport = active_native_viewport();
  if (viewport) viewport->ingest_preview_items(preview_items_);
  const bool has_selected = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & it){ return it.id == selected_preview_item_id_; });
  if (viewport) viewport->selected_id = selected_preview_item_id_;
  if (diagnostic_debug_logging_enabled() && !selected_preview_item_id_.isEmpty()) {
    emit studio_log_requested(has_selected ? QString("Preview selection restored after refresh: %1").arg(selected_preview_item_id_) : QString("Preview selection retained after refresh; id is hidden by filters or absent from the visible preview payload: %1").arg(selected_preview_item_id_));
  }
  if (viewport) viewport->fit_include_overlays = false;
  apply_product_view_defaults();
  if (payload_changed) refresh_embedded_web_product_view();
  if (is_native_product_view_backend()) {
    emit_scene_diagnostic_once(
      QStringLiteral("payload_commit"),
      preview_items_.size(),
      QStringLiteral("Scene3D payload committed: scene=%1 rev=%2 items=%3 visible=%4 mesh=%5 fallback=%6")
        .arg(preview_scene_name_)
        .arg(preview_payload_revision_)
        .arg(preview_items_.size())
        .arg(viewport ? viewport->render_debug_counters().visible_count : 0)
        .arg(viewport ? viewport->render_debug_counters().mesh_backed_count : 0)
        .arg(viewport ? viewport->render_debug_counters().primitive_fallback_count : 0));
  }
  fit_fallback_scene_to_items(false);
  refresh_info_chip();
  emit_visual_quality_assessment_once();
  update();
}
int ScenePreviewWidget::preview_payload_revision() const { return preview_payload_revision_; }
quint64 ScenePreviewWidget::preview_payload_generation() const { return preview_payload_generation_; }
quint64 ScenePreviewWidget::embedded_web_preparation_request_count() const { return embedded_web_preparation_request_count_; }
void ScenePreviewWidget::set_preview_scene_name(const QString & scene_name)
{
  const QString normalized_scene_name = scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed();
  if (preview_scene_name_ == normalized_scene_name) return;

  cancel_embedded_web_lifecycle(false);
  preview_scene_name_ = normalized_scene_name;
  preview_payload_revision_ = 0;
  last_visual_quality_revision_logged_ = -1;
  emitted_scene_diagnostic_keys_.clear();
  emit_scene_diagnostic_once(
    QStringLiteral("scene_load"),
    0,
    QStringLiteral("Scene loaded: %1").arg(preview_scene_name_));
  auto * v = active_native_viewport();
  if (v) { v->scene_name = preview_scene_name_; v->update(); }
  refresh_embedded_web_product_view();
  refresh_info_chip();
}

bool ScenePreviewWidget::diagnostic_debug_logging_enabled() const
{
  const auto * viewport = active_native_viewport();
  return (viewport && viewport->debug_overlays_mode) || qEnvironmentVariableIsSet("WORKCELL_SCENE3D_DEBUG_LOGS");
}

bool ScenePreviewWidget::emit_scene_diagnostic_once(const QString & event, int payload_count, const QString & message)
{
  const QString scene = preview_scene_name_.trimmed().isEmpty() ? QStringLiteral("No scene") : preview_scene_name_.trimmed();
  const QString key = QStringLiteral("%1|%2|rev=%3|count=%4").arg(scene, event).arg(preview_payload_revision_).arg(payload_count);
  if (!diagnostic_debug_logging_enabled()) return false;
  if (emitted_scene_diagnostic_keys_.contains(key)) return false;
  emitted_scene_diagnostic_keys_.insert(key);
  emit studio_log_requested(message);
  return true;
}

void ScenePreviewWidget::emit_visual_quality_assessment_once()
{
  if (!diagnostic_debug_logging_enabled()) return;
  if (last_visual_quality_revision_logged_ == preview_payload_revision_) return;
  last_visual_quality_revision_logged_ = preview_payload_revision_;
  int physical_count = 0;
  int mesh_count = 0;
  int primitive_count = 0;
  int missing_count = 0;
  int overlay_count = 0;
  for (const auto & item : preview_items_) {
    if (preview_item_is_overlay_or_helper(item)) { ++overlay_count; continue; }
    ++physical_count;
    if (preview_item_has_credible_mesh_handoff(item)) ++mesh_count;
    else if (preview_item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001)) ++primitive_count;
    else ++missing_count;
  }
  emit_scene_diagnostic_once(
    QStringLiteral("visual_quality"),
    preview_items_.size(),
    QStringLiteral("Scene3D visual quality: scene=%1 rev=%2 physical=%3 mesh=%4 primitive=%5 missing=%6 overlays=%7 warnings=%8")
      .arg(preview_scene_name_)
      .arg(preview_payload_revision_)
      .arg(physical_count)
      .arg(mesh_count)
      .arg(primitive_count)
      .arg(missing_count)
      .arg(overlay_count)
      .arg(total_warning_count()));
}

void ScenePreviewWidget::set_preview_status_summary(const QString & summary){ preview_status_summary_ = summary.trimmed(); refresh_info_chip(); }
void ScenePreviewWidget::set_clean_product_view_status(bool clean, int visual_count)
{
  clean_product_view_ = clean;
  clean_product_visual_count_ = qMax(0, visual_count);
  refresh_info_chip();
}
void ScenePreviewWidget::set_task_overlay_model(const TaskOverlayModel & model){ overlay_model_ = model; if (auto * v = active_native_viewport()) v->task_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels){
  auto * v = active_native_viewport();
  if (!v) return;
  v->show_task_route = task_route;
  v->show_pick_place = pick_place_zones;
  v->show_approach_retreat = approach_retreat;
  if (labels) v->label_mode = LabelMode::All;
  else if (v->label_mode == LabelMode::All) v->label_mode = LabelMode::Important;
  // else if (v->label_mode == LabelMode::All) v->label_mode = LabelMode::SelectedOnly;
  v->update();
}
void ScenePreviewWidget::select_preview_item(const QString & id){ selected_preview_item_id_ = id; if (auto * v = active_native_viewport()) { v->selected_id = id; v->update(); } else run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.selectItem(%1)").arg(QString::fromUtf8(QJsonDocument(QJsonArray{id}).toJson(QJsonDocument::Compact)).mid(1).chopped(1))); if (simple_3d_view_) simple_3d_view_->update(); }
QString ScenePreviewWidget::selected_preview_item_id() const { return selected_preview_item_id_; }
const ScenePreviewWidget::PreviewItem * ScenePreviewWidget::preview_item_by_id(const QString & id) const
{
  const QString stable_id = id.trimmed();
  if (stable_id.isEmpty()) return nullptr;
  for (const auto & item : preview_items_) {
    if (item.id.trimmed() == stable_id) return &item;
  }
  return nullptr;
}

ScenePreviewWidget::MeshPreviewMode ScenePreviewWidget::mesh_preview_mode() const { return mesh_preview_mode_; }

void ScenePreviewWidget::reload_meshes()
{
  auto * v = active_native_viewport();
  if (!v) { request_embedded_web_product_view_refresh(true); emit studio_log_requested("Reloaded embedded Web 3D Product View."); return; }
  v->invalidate_mesh_cache();
  apply_product_view_defaults();
  update();
  emit studio_log_requested("Reloaded mesh preview cache (visual-only).");
}
void ScenePreviewWidget::apply_product_view_defaults()
{
  auto * v = active_native_viewport();
  if (!v) return;
  if (view_actions_selector_) {
    const QSignalBlocker blocker(view_actions_selector_);
    view_actions_selector_->setCurrentText("Isometric");
  }
  // Product previews should open in the same camera path users get from the
  // normal canvas controls: an isometric view followed by product-fit bounds
  // that keep robot/tool/environment meshes and editable layout items legible
  // without letting diagnostics overlays dominate the initial framing.
  v->fit_include_overlays = false;
  v->debug_overlays_mode = false;
  v->show_warnings = false;
  v->show_warning_labels = false;
  v->show_safety = false;
  v->show_pick_place = false;
  v->show_reachability_heatmap = false;
  v->show_collision_warnings = false;
  v->show_work_envelope = false;
  v->show_task_route = false;
  v->show_approach_retreat = false;
  v->show_camera_fov = false;
  v->show_pick_coverage = false;
  v->show_epd_detections = false;
  v->show_detection_labels = false;
  v->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Auto;
  mesh_preview_mode_ = ScenePreviewWidget::MeshPreviewMode::Auto;
  if (mesh_preview_mode_selector_) {
    const QSignalBlocker blocker(mesh_preview_mode_selector_);
    mesh_preview_mode_selector_->setCurrentText("Auto");
  }
  if (labels_selector_) {
    const QSignalBlocker blocker(labels_selector_);
    labels_selector_->setCurrentText("Selected");
  }
  v->label_mode = ScenePreviewWidget::LabelMode::Selected;
  v->set_isometric_view();
  v->fit_product_view();
  fit_fallback_scene_to_items(false);
}
void ScenePreviewWidget::on_reset_view_clicked(){ if (auto * v = active_native_viewport()) v->reset_view(); else refresh_embedded_web_product_view(); reset_fallback_scene_view(); }
void ScenePreviewWidget::on_fit_scene_clicked(){ auto *v = active_native_viewport(); if (v) { v->fit_include_overlays = false; v->fit_scene(); } fit_fallback_scene_to_items(false); } // Fit Scene intentionally excludes overlay-only bounds by default.
void ScenePreviewWidget::on_fit_robot_clicked(){ auto *v = active_native_viewport(); if (v) v->fit_robot(); fit_fallback_scene_to_items(false); }
void ScenePreviewWidget::on_fit_overlays_clicked(){ auto *v = active_native_viewport(); const QRectF physical_bounds = rendered_items_bounds_2d(false); const QRectF overlay_bounds = rendered_items_bounds_2d(true); maybe_warn_overlay_fit_dominance(this, physical_bounds, overlay_bounds); if (v) { v->fit_include_overlays = true; v->fit_scene(); } fit_fallback_scene_to_items(true); if (v) v->fit_include_overlays = false; } // Fit overlays includes overlay bounds for explicit overlay-focused framing.
void ScenePreviewWidget::on_focus_selected_clicked(){ if (auto * v = active_native_viewport()) v->focus_selected(); }
void ScenePreviewWidget::on_clear_selection_clicked(){ selected_preview_item_id_.clear(); if (auto * v = active_native_viewport()) { v->selected_id.clear(); v->update(); } if (simple_3d_view_) simple_3d_view_->update(); emit studio_log_requested("Cleared preview selection."); emit preview_item_selected(QString(), QStringLiteral("unknown")); }
void ScenePreviewWidget::refresh_toolbar_visibility()
{
  if (view_actions_selector_) {
    const QSignalBlocker blocker(view_actions_selector_);
    const QString previous = view_actions_selector_->currentText();
    view_actions_selector_->clear();
    view_actions_selector_->addItems({"Top", "Front", "Side", "Isometric", "Fit View"});
    const int previous_index = view_actions_selector_->findText(previous);
    view_actions_selector_->setCurrentIndex(previous_index >= 0 ? previous_index : 0);
  }

  const auto set_visible = [](QWidget * widget, bool visible) {
    if (widget) widget->setVisible(visible);
  };

  // Embedded Web3D is the canonical Product View when Qt WebEngine is enabled;
  // hide native Scene3DViewportWidget-only controls so they are not active no-ops.
  const bool embedded_web_active = (active_native_viewport() == nullptr);
  set_visible(view_actions_label_, !embedded_web_active);
  set_visible(view_actions_selector_, !embedded_web_active);
  set_visible(labels_label_, !embedded_web_active);
  set_visible(labels_selector_, !embedded_web_active);
  set_visible(toolbar_status_chip_, true);
  set_visible(embedded_undo_button_, embedded_web_active);
  set_visible(embedded_redo_button_, embedded_web_active);
  set_visible(embedded_fit_button_, embedded_web_active);
  set_visible(mesh_preview_mode_label_, !embedded_web_active);
  set_visible(mesh_preview_mode_selector_, !embedded_web_active);
  set_visible(gizmo_mode_label_, embedded_web_active);
  set_visible(gizmo_mode_selector_, embedded_web_active);
  set_visible(snap_mode_label_, embedded_web_active);
  set_visible(snap_mode_selector_, embedded_web_active);
  set_visible(interaction_mode_label_, !embedded_web_active);
  set_visible(interaction_mode_selector_, !embedded_web_active);
  set_visible(overlays_selector_, !embedded_web_active);
  refresh_toolbar_status_chip();
  refresh_toolbar_feedback_row();
}

void ScenePreviewWidget::refresh_mode_and_state()
{
  const QString mode = mode_selector_->currentText();
  const bool requested_3d = (mode == "3D Layout Preview" || mode == "Web3D Product View");
  const bool use3d = requested_3d && preview3d_available_;
  refresh_toolbar_visibility();

  if (!preview3d_available_) {
    stack_->setCurrentWidget(view2d_container_);
    fallback_banner_label_->setVisible(scene_selected_);
    const QString reason = unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_;
    emit studio_log_requested(QString("3D Layout Preview unavailable, using 2D Layout: %1").arg(reason));
  } else {
    fallback_banner_label_->setVisible(false);
    fallback_banner_label_->setToolTip(scene_preview_mouse_help_tooltip(QString()));
    stack_->setCurrentWidget(use3d ? view3d_container_ : view2d_container_);
  }
  empty_state_label_->setVisible(use3d && !scene_selected_);
  const bool show_native_compatibility = use3d && scene_selected_ && native_compatibility_fallback_active_ && compatibility_scene3d_viewport_;
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(show_native_compatibility);
  if (simple_3d_view_) simple_3d_view_->setVisible(use3d && scene_selected_ && !show_native_compatibility);
  if (error_state_label_) error_state_label_->setVisible(false);
  refresh_toolbar_feedback_row();
  refresh_info_chip();
}
QRectF ScenePreviewWidget::rendered_items_bounds_2d(bool include_overlays) const
{
  QRectF bounds;
  auto include_in_fit_bounds = [include_overlays](const PreviewItem & it) {
    if (include_overlays) return true;
    if (preview_item_is_overlay_or_helper(it)) return false;

    const QString source_layer = normalized_preview_token(it.source_layer);
    const QString visual_source = normalized_preview_token(it.active_visual_source);
    if (source_layer.contains("overlay") || visual_source.contains("overlay")) return false;

    const bool generated_urdf = preview_item_is_generated_or_locked_urdf(it);
    const bool mesh_backed = preview_item_has_credible_mesh_handoff(it);
    const bool explicit_primitive = preview_item_has_valid_urdf_primitive(it) ||
                                    (it.sx > 0.001 && it.sy > 0.001 && it.sz > 0.001);
    if (generated_urdf) return true;
    if (it.linked_to_editable_layout_state) return true;
    if (source_layer == QStringLiteral("mesh_preview") || visual_source == QStringLiteral("mesh_preview")) {
      return mesh_backed || explicit_primitive;
    }

    const QString role = normalized_preview_token(it.role);
    const QString category = normalized_preview_token(it.category);
    const QString id = normalized_preview_token(it.id);
    const QString display_name = normalized_preview_token(it.display_name);
    const QString mix = role + QStringLiteral("|") + category + QStringLiteral("|") + id + QStringLiteral("|") + display_name;
    const bool product_physical = mix.contains("robot") || mix.contains("gripper") ||
                                  mix.contains("tool") || mix.contains("end_effector") ||
                                  mix.contains("table") || mix.contains("work_surface") ||
                                  mix.contains("workbench") || mix.contains("camera") ||
                                  mix.contains("realsense") || mix.contains("depth_camera") ||
                                  mix.contains("rgbd");
    return product_physical && (mesh_backed || explicit_primitive);
  };
  for (const auto & it : preview_items_) {
    if (!include_in_fit_bounds(it)) continue;
    const QRectF rect(it.x, it.z, it.sx, it.sz);
    bounds = bounds.isNull() ? rect : bounds.united(rect);
  }

  if (bounds.isValid() && !bounds.isEmpty()) return bounds;
  if (!fallback_2d_view_ || !fallback_2d_view_->scene()) return QRectF();
  const auto scene_items = fallback_2d_view_->scene()->items();
  for (QGraphicsItem * item : scene_items) {
    if (!item || !item->isVisible()) continue;
    bounds = bounds.isNull() ? item->sceneBoundingRect() : bounds.united(item->sceneBoundingRect());
  }
  return bounds;
}
void ScenePreviewWidget::fit_fallback_scene_to_items(bool include_overlays)
{
  if (!fallback_2d_view_) return;
  const QRectF bounds = rendered_items_bounds_2d(include_overlays);
  if (bounds.isValid() && !bounds.isEmpty()) fallback_2d_view_->fitInView(bounds.adjusted(-20, -20, 20, 20), Qt::KeepAspectRatio);
}
void ScenePreviewWidget::reset_fallback_scene_view()
{
  if (!fallback_2d_view_) return;
  fallback_2d_view_->resetTransform();
  fit_fallback_scene_to_items(false);
}

void ScenePreviewWidget::set_camera_overlay_model(const CameraOverlayModel & model){ camera_overlay_model_ = model; if (auto *v=active_native_viewport()) v->camera_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections){ epd_detections_ = detections; if (auto *v=active_native_viewport()) v->epd_detections = detections; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels){ auto *v=active_native_viewport(); if (!v) return; v->show_camera_fov=camera_fov; v->show_pick_coverage=pick_coverage; v->show_epd_detections=epd_detections; v->show_detection_labels=detection_labels; v->update(); }


void ScenePreviewWidget::set_reachability_overlay_model(const ReachabilityOverlayModel & model){ reachability_overlay_model_ = model; if (auto *v=active_native_viewport()) v->reach_overlay = model; if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("reachability overlay loaded: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_collision_overlay_model(const CollisionOverlayModel & model){ collision_overlay_model_ = model; if (auto *v=active_native_viewport()) v->collision_overlay = model; if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("collision preview checks complete: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }

void ScenePreviewWidget::set_label_mode(LabelMode mode){ Q_UNUSED(mode); auto *v=active_native_viewport(); if (labels_selector_) labels_selector_->setCurrentText("Selected"); if (v) { v->label_mode = LabelMode::Selected; v->update(); } }

ScenePreviewWidget::RenderDebugCounters ScenePreviewWidget::render_debug_counters() const
{
  const auto * viewport = active_native_viewport();
  const auto counters = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
  RenderDebugCounters out;
  out.preview_items_count = preview_items_.size();
  out.total_payload_count = preview_items_.size();
  out.viewport_received_count = counters.viewport_received_count;
  out.render_cache_count = counters.render_cache_count;
  out.visible_count = counters.visible_count;
  out.rendered_count = counters.rendered_count;
  out.skipped_count = counters.skipped_count;
  out.unique_visible_item_count = counters.unique_visible_item_count;
  out.mesh_backed_count = counters.mesh_backed_count;
  out.mesh_source_count = counters.mesh_source_count;
  out.mesh_path_resolved_count = counters.mesh_path_resolved_count;
  out.mesh_file_loaded_count = counters.mesh_file_loaded_count;
  out.mesh_triangles_loaded_count = counters.mesh_triangles_loaded_count;
  out.mesh_rendered_count = counters.mesh_rendered_count;
  out.mesh_surface_rendered_count = counters.mesh_surface_rendered_count;
  out.mesh_bounds_fallback_rendered_count = counters.mesh_bounds_fallback_rendered_count;
  out.urdf_primitive_source_count = counters.urdf_primitive_source_count;
  out.urdf_primitive_rendered_count = counters.urdf_primitive_rendered_count;
  out.placeholder_count = counters.placeholder_count;
  out.missing_geometry_count = counters.missing_geometry_count;
  out.wireframe_fallback_count = counters.wireframe_fallback_count;
  out.overlay_helper_count = counters.overlay_helper_count;
  out.overlay_count = counters.overlay_count;
  out.generated_fallback_count = counters.generated_fallback_count;
  out.editable_layout_count = counters.editable_layout_count;
  out.primitive_fallback_count = counters.primitive_fallback_count;
  out.primitive_fallback_rendered_count = counters.primitive_fallback_rendered_count;
  out.editable_primitive_rendered_count = counters.editable_primitive_rendered_count;
  out.valid_physical_fallback_count = counters.valid_physical_fallback_count;
  out.overlay_rendered_count = counters.overlay_rendered_count;
  out.locked_generated_urdf_visual_count = counters.locked_generated_urdf_visual_count;
  out.physical_anchor_count = counters.physical_anchor_count;
  out.generated_robot_mesh_count = counters.generated_robot_mesh_count;
  out.tool_gripper_visual_count = counters.tool_gripper_visual_count;
  out.table_workbench_visual_count = counters.table_workbench_visual_count;
  out.camera_body_visual_count = counters.camera_body_visual_count;
  out.transform_chain_applied_count = counters.transform_chain_applied_count;
  out.visual_origin_applied_count = counters.visual_origin_applied_count;
  out.baked_world_visual_transform_count = counters.baked_world_visual_transform_count;
  out.legacy_viewport_transform_count = counters.legacy_viewport_transform_count;
  out.visual_quality_status = counters.visual_quality_status;
  out.visual_quality_warnings = counters.visual_quality_warnings;
  out.labels_drawn = counters.labels_drawn;
  out.labels_suppressed_overlap = counters.labels_suppressed_overlap;
  out.hierarchy_child_row_count = counters.hierarchy_child_row_count;
  out.last_paint_completed = counters.last_paint_completed;
  out.smoke_fallback_render_used = counters.smoke_fallback_render_used;
  return out;
}

int ScenePreviewWidget::total_warning_count() const
{
  auto actionable_warning = [](const QString & warning) {
    const QString w = warning.toLower();
    if (w.contains(QStringLiteral("legacy disabled")) || w.contains(QStringLiteral("perception: none")) || w.contains(QStringLiteral("perception disabled"))) return false;
    if (w.contains(QStringLiteral("resolved_source_path is stale")) && w.contains(QStringLiteral("package_uri"))) return false;
    if (w.contains(QStringLiteral("package_uri_resolved_after_stale_resolved_source_path")) || w.contains(QStringLiteral("resolved_via_package_uri_after_stale_resolved_source_path"))) return false;
    return true;
  };
  int count = 0;
  for (const auto & item : preview_items_) {
    for (const QString & warning : item.warnings) {
      if (item.resolved_source_path_stale && item.source_path_resolution_outcome.contains(QStringLiteral("package_uri"), Qt::CaseInsensitive)) continue;
      if (actionable_warning(warning)) ++count;
    }
    if (!item.mesh_load_warning.trimmed().isEmpty() && actionable_warning(item.mesh_load_warning)) ++count;
  }
  for (const QString & warning : overlay_model_.warnings) if (actionable_warning(warning)) ++count;
  for (const QString & warning : reachability_overlay_model_.warnings) if (actionable_warning(warning)) ++count;
  for (const QString & warning : collision_overlay_model_.warnings) if (actionable_warning(warning)) ++count;
  for (const QString & warning : camera_overlay_model_.warnings) if (actionable_warning(warning)) ++count;
  for (const auto & det : epd_detections_) for (const QString & warning : det.warnings) if (actionable_warning(warning)) ++count;
  return count;
}
bool ScenePreviewWidget::task_is_ready() const { return overlay_model_.has_intent_metadata && overlay_model_.pick_source_id != "unknown" && overlay_model_.place_target_id != "unknown"; }
void ScenePreviewWidget::refresh_info_chip()
{
  if (!info_chip_label_) return;
  const QString mode = mode_selector_ ? mode_selector_->currentText() : QStringLiteral("2D Layout");
  const bool requested_3d = (mode == "3D Layout Preview" || mode == "Web3D Product View");
  const QString render_mode = requested_3d && preview3d_available_ ? mode : QStringLiteral("2D Layout (Fallback)");
  const QString summary = preview_status_summary_.isEmpty() ? QString("Items: %1").arg(preview_items_.size()) : preview_status_summary_;

  int mesh_count = 0;
  int box_count = 0;
  int missing_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int physical_count = 0;
  for (const auto & item : preview_items_) {
    const QString category = item.category.trimmed().toLower();
    const QString lock_reason = item.lock_reason.trimmed().toLower();
    if (preview_item_is_overlay_or_helper(item)) {
      ++overlay_count;
      continue;
    }
    ++physical_count;
    const bool mesh_backed = preview_item_has_credible_mesh_handoff(item);
    const bool raw_generated_bounds = preview_item_is_raw_generated_bounds_only(item);
    if (mesh_backed) ++mesh_count;
    else if (preview_item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001 && !raw_generated_bounds)) ++box_count;
    else if (raw_generated_bounds) ++mesh_count;
    else ++missing_count;

    if (item.locked && !item.editable &&
        (category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot model") ||
         lock_reason.contains("robotmodel") || lock_reason.contains("urdf visual"))) ++locked_urdf_count;
  }

  const auto * viewport = active_native_viewport();
  const auto counters = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
  const QString compact_stats = QString("Items %1 M%2 B%3 Miss%4 Ov%5 L-URDF%6")
                                  .arg(physical_count).arg(mesh_count).arg(box_count).arg(missing_count).arg(overlay_count).arg(locked_urdf_count);
  const QString smoke_stats = QString("quality=%1 mesh=%2/%3 surface=%4 bounds_fallback=%5 resolved=%6 loaded=%7 triangles=%8 urdf_prim=%9/%10 missing=%11 placeholder=%12 wireframe=%13 helpers=%14 paint_completed=%15 selected_scene_name=%16 selected_item_id=%17")
                                  .arg(counters.visual_quality_status)
                                  .arg(counters.mesh_rendered_count).arg(counters.mesh_source_count)
                                  .arg(counters.mesh_surface_rendered_count)
                                  .arg(counters.mesh_bounds_fallback_rendered_count)
                                  .arg(counters.mesh_path_resolved_count)
                                  .arg(counters.mesh_file_loaded_count)
                                  .arg(counters.mesh_triangles_loaded_count)
                                  .arg(counters.urdf_primitive_rendered_count).arg(counters.urdf_primitive_source_count)
                                  .arg(counters.missing_geometry_count).arg(counters.placeholder_count)
                                  .arg(counters.wireframe_fallback_count).arg(counters.overlay_helper_count)
                                  .arg(counters.last_paint_completed ? QStringLiteral("true") : QStringLiteral("false"))
                                  .arg(preview_scene_name_.isEmpty() ? QStringLiteral("(none)") : preview_scene_name_)
                                  .arg(selected_preview_item_id_.isEmpty() ? QStringLiteral("(none)") : selected_preview_item_id_);
  const QString initial_fit_audit = QStringLiteral("initial_fit_robot_bounds=%1 physical_anchors=%2")
                                      .arg(viewport && viewport->last_initial_fit_included_robot_bounds()
                                             ? QStringLiteral("included")
                                             : QStringLiteral("not_included"))
                                      .arg(viewport ? viewport->last_initial_fit_physical_anchor_count() : 0);
  info_chip_label_->setText(QString("Scene: %1\nMode: %2\n%3\n%4  Warn: %5  Task: %6")
                              .arg(preview_scene_name_).arg(render_mode).arg(summary).arg(compact_stats)
                              .arg(total_warning_count()).arg(task_is_ready() ? "Ready" : "Missing") + QString("\n") + smoke_stats + QStringLiteral(" ") + initial_fit_audit);
  info_chip_label_->adjustSize();
  if (fallback_info_chip_proxy_) fallback_info_chip_proxy_->setPos(12.0, 12.0);
  refresh_toolbar_status_chip();
}
