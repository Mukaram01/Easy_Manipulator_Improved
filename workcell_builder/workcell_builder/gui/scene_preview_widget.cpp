#include "scene_preview_widget.h"
// Compatibility token for static tests: Preview selection cleared after refresh (id missing):

#include <QRectF>
#include <QtGlobal>
#include <QVariantMap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QFile>
#include <QSaveFile>
#include <QFileInfo>
#include <QDir>
#include <QIODevice>
#include <QStringList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QTcpServer>
#include <QHostAddress>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QUrlQuery>
#include <QJsonArray>
#include <QCryptographicHash>
#include <QDataStream>
#include <QSet>
#include <QSignalBlocker>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <functional>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "object_placement_yaml_io.hpp"
#include "robot_home_yaml_io.hpp"

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
  stream << item.id << item.display_name << item.category << item.catalog_asset_id;
  stream << item.x << item.y << item.z << item.roll << item.pitch << item.yaw;
  stream << item.sx << item.sy << item.sz << item.status << item.source_path << item.role;
  stream << item.mesh_path << item.mesh_type << item.primitive_geometry_type << item.primitive_radius << item.primitive_length;
  write_bool(item.has_material_color);
  stream << item.material_r << item.material_g << item.material_b << item.material_a << item.material_name;
  stream << item.mesh_scale_x << item.mesh_scale_y << item.mesh_scale_z << item.mesh_roll << item.mesh_pitch << item.mesh_yaw;
  write_bool(item.mesh_available);
  stream << item.mesh_load_warning;
  write_bool(item.selectable); write_bool(item.editable); write_bool(item.locked);
  stream << item.lock_reason << item.metadata_tags << item.target_ref << item.transform_group
         << item.source_layer << item.active_visual_source;
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

QString embedded_web_request_owned_output_path(
  const QString & safe_scene_id, quint64 payload_revision, quint64 generation,
  const QByteArray & payload_fingerprint)
{
  if (!is_safe_embedded_web_scene_id(safe_scene_id) || payload_fingerprint.isEmpty()) return QString();
  const QString filename = QStringLiteral("%1-r%2-g%3-%4.web_scene.json")
    .arg(safe_scene_id).arg(payload_revision).arg(generation)
    .arg(QString::fromLatin1(payload_fingerprint.toHex()));
  return QStringLiteral("build/workcell_studio_web_scene/preparations/%1").arg(filename);
}

void remove_embedded_web_request_output_if_safe(const QString & repo_root, const QString & relative_path)
{
  const QString preparation_root = QDir::cleanPath(
    QDir(repo_root).absoluteFilePath(QStringLiteral("build/workcell_studio_web_scene/preparations")));
  const QString candidate = QDir::cleanPath(QDir(repo_root).absoluteFilePath(relative_path));
  if (candidate.startsWith(preparation_root + QDir::separator())) QFile::remove(candidate);
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


QString summarize_process_arguments(const QStringList & arguments)
{
  QStringList rendered;
  for (const QString & argument : arguments) {
    rendered << QStringLiteral("'%1'").arg(argument.left(180).replace(QStringLiteral("'"), QStringLiteral("'\\''")));
  }
  return rendered.join(QStringLiteral(" "));
}

QString summarize_prepare_process_failure(
  const QString & executable, const QStringList & arguments, const QString & working_directory,
  QProcess::ExitStatus exit_status, int exit_code, const QString & start_error, const QByteArray & stderr_tail)
{
  const QString stderr_summary = QString::fromUtf8(stderr_tail).trimmed().left(1200);
  QString detail = QStringLiteral("executable=%1 arguments=%2 working_directory=%3 exit_status=%4 exit_code=%5")
    .arg(executable,
         summarize_process_arguments(arguments),
         working_directory.isEmpty() ? QStringLiteral("<unset>") : working_directory,
         exit_status == QProcess::NormalExit ? QStringLiteral("normal") : QStringLiteral("crash"))
    .arg(exit_code);
  if (!start_error.trimmed().isEmpty()) detail += QStringLiteral(" start_error=%1").arg(start_error.trimmed().left(400));
  if (!stderr_summary.isEmpty()) detail += QStringLiteral(" stderr=%1").arg(stderr_summary);
  return detail;
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
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDragLeaveEvent>
#include <QDropEvent>
#include <QMimeData>
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

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
class AssetDropWebEngineView final : public QWebEngineView
{
public:
  using DragEnterHandler = std::function<bool(const QString &)>;
  using CancelHandler = std::function<void()>;

  explicit AssetDropWebEngineView(QWidget * parent = nullptr) : QWebEngineView(parent)
  {
    setAcceptDrops(true);
  }

  DragEnterHandler drag_entered;
  CancelHandler drag_cancelled;

protected:
  void dragEnterEvent(QDragEnterEvent * event) override
  {
    const QString asset_id = parse_asset_id(event->mimeData());
    drag_active_ = !asset_id.isEmpty() && drag_entered && drag_entered(asset_id);
    if (drag_active_) {
      drag_asset_id_ = asset_id;
      update_pointer(event->pos());
      event->acceptProposedAction();
    } else event->ignore();
  }

  void dragMoveEvent(QDragMoveEvent * event) override
  {
    if (drag_active_ && parse_asset_id(event->mimeData()) == drag_asset_id_) {
      update_pointer(event->pos());
      event->acceptProposedAction();
    } else event->ignore();
  }

  void dragLeaveEvent(QDragLeaveEvent * event) override
  {
    cancel_drag();
    event->accept();
  }

  void dropEvent(QDropEvent * event) override
  {
    if (!drag_active_ || parse_asset_id(event->mimeData()) != drag_asset_id_) {
      event->ignore();
      cancel_drag();
      return;
    }
    const QPoint point = event->pos(); // QWebEngineView-local == browser client coordinates.
    drag_active_ = false;
    drag_asset_id_.clear();
    event->acceptProposedAction();
    const QString script = QStringLiteral(
      "window.__WORKCELL_EDITOR_API_V1__?.commitPlacementPointer?.(%1,%2)")
      .arg(point.x()).arg(point.y());
    page()->runJavaScript(script, [this](const QVariant & result) {
      // A rejected support/collision/bounds result ends the native session too.
      // A valid result stays armed only until the existing editor-event poll
      // delivers its single placement_requested event to MainWindow.
      if (!result.toBool() && drag_cancelled) drag_cancelled();
    });
  }

private:
  static QString parse_asset_id(const QMimeData * mime)
  {
    static const QString kMimeType = QStringLiteral("application/x-workcell-studio-catalog-asset");
    if (!mime || mime->formats() != QStringList{kMimeType}) return {};
    return QString::fromUtf8(mime->data(kMimeType)).trimmed();
  }

  void update_pointer(const QPoint & point)
  {
    page()->runJavaScript(QStringLiteral(
      "window.__WORKCELL_EDITOR_API_V1__?.updatePlacementPointer?.(%1,%2)")
      .arg(point.x()).arg(point.y()));
  }

  void cancel_drag()
  {
    if (!drag_active_) return;
    drag_active_ = false;
    drag_asset_id_.clear();
    page()->runJavaScript(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__?.cancelPlacement?.()"));
    if (drag_cancelled) drag_cancelled();
  }

  bool drag_active_{false};
  QString drag_asset_id_;
};
#endif
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

  product_view_destination_row_ = new QWidget(controls_header);
  auto * destination_layout = new QHBoxLayout(product_view_destination_row_);
  destination_layout->setContentsMargins(0, 0, 0, 0);
  destination_layout->setSpacing(6);
  destination_layout->addWidget(new QLabel(QStringLiteral("Destination Bin"), product_view_destination_row_));
  product_view_destination_combo_ = new QComboBox(product_view_destination_row_);
  product_view_destination_combo_->setObjectName(QStringLiteral("product_view_destination_combo"));
  product_view_destination_combo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  destination_layout->addWidget(product_view_destination_combo_);
  product_view_apply_destination_button_ = new QPushButton(QStringLiteral("Apply Destination"), product_view_destination_row_);
  destination_layout->addWidget(product_view_apply_destination_button_);
  product_view_destination_status_ = new QLabel(product_view_destination_row_);
  product_view_destination_status_->setObjectName(QStringLiteral("product_view_destination_status"));
  destination_layout->addWidget(product_view_destination_status_, 1);
  product_view_destination_row_->setVisible(false);
  controls->addWidget(product_view_destination_row_);
  connect(product_view_apply_destination_button_, &QPushButton::clicked,
    this, &ScenePreviewWidget::apply_product_view_destination);
  connect(product_view_destination_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {
      if (!product_view_apply_destination_button_) return;
      const bool selectable = !product_view_destination_combo_->currentData().toString().trimmed().isEmpty();
      product_view_apply_destination_button_->setEnabled(selectable &&
        embedded_product_view_state_ == EmbeddedProductViewState::Ready &&
        !property("diagnostic_preview_active").toBool() && !product_view_destination_save_in_progress_);
    });

  product_view_tool_orientation_row_ = new QWidget(controls_header);
  auto * tool_orientation_layout = new QHBoxLayout(product_view_tool_orientation_row_);
  tool_orientation_layout->setContentsMargins(0, 0, 0, 0);
  tool_orientation_layout->setSpacing(6);
  tool_orientation_layout->addWidget(new QLabel(QStringLiteral("Tool Orientation"), product_view_tool_orientation_row_));
  const auto make_angle_spin = [this, tool_orientation_layout](const QString & label, const QString & object_name) {
    tool_orientation_layout->addWidget(new QLabel(label, product_view_tool_orientation_row_));
    auto * spin = new QDoubleSpinBox(product_view_tool_orientation_row_);
    spin->setObjectName(object_name);
    spin->setRange(-360.0, 360.0);
    spin->setDecimals(2);
    spin->setSingleStep(5.0);
    spin->setSuffix(QStringLiteral("°"));
    tool_orientation_layout->addWidget(spin);
    return spin;
  };
  product_view_tool_roll_spin_ = make_angle_spin(QStringLiteral("Roll"), QStringLiteral("product_view_tool_roll_spin"));
  product_view_tool_pitch_spin_ = make_angle_spin(QStringLiteral("Pitch"), QStringLiteral("product_view_tool_pitch_spin"));
  product_view_tool_yaw_spin_ = make_angle_spin(QStringLiteral("Yaw"), QStringLiteral("product_view_tool_yaw_spin"));
  product_view_tool_orientation_preset_ = new QComboBox(product_view_tool_orientation_row_);
  product_view_tool_orientation_preset_->setObjectName(QStringLiteral("product_view_tool_orientation_preset"));
  product_view_tool_orientation_preset_->addItems({QStringLiteral("Current scene value"), QStringLiteral("Identity"),
    QStringLiteral("Roll +90"), QStringLiteral("Roll -90"), QStringLiteral("Pitch +90"), QStringLiteral("Pitch -90"),
    QStringLiteral("Yaw +90"), QStringLiteral("Yaw -90"), QStringLiteral("Yaw 180")});
  tool_orientation_layout->addWidget(product_view_tool_orientation_preset_);
  product_view_tool_orientation_reset_button_ = new QPushButton(QStringLiteral("Reset"), product_view_tool_orientation_row_);
  tool_orientation_layout->addWidget(product_view_tool_orientation_reset_button_);
  product_view_apply_tool_orientation_ = new QPushButton(QStringLiteral("Apply Orientation"), product_view_tool_orientation_row_);
  product_view_apply_tool_orientation_->setObjectName(QStringLiteral("product_view_apply_tool_orientation"));
  tool_orientation_layout->addWidget(product_view_apply_tool_orientation_);
  tool_orientation_layout->addStretch(1);
  product_view_tool_orientation_row_->setVisible(false);
  controls->addWidget(product_view_tool_orientation_row_);
  connect(product_view_tool_orientation_reset_button_, &QPushButton::clicked,
    this, &ScenePreviewWidget::reset_product_view_tool_orientation);
  connect(product_view_apply_tool_orientation_, &QPushButton::clicked,
    this, &ScenePreviewWidget::apply_product_view_tool_orientation);
  connect(product_view_tool_orientation_preset_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    if (index == 0) { reset_product_view_tool_orientation(); return; }
    double r = 0.0, p = 0.0, y = 0.0;
    if (index == 2) r = 90.0; else if (index == 3) r = -90.0;
    else if (index == 4) p = 90.0; else if (index == 5) p = -90.0;
    else if (index == 6) y = 90.0; else if (index == 7) y = -90.0; else if (index == 8) y = 180.0;
    product_view_tool_roll_spin_->setValue(r);
    product_view_tool_pitch_spin_->setValue(p);
    product_view_tool_yaw_spin_->setValue(y);
  });

  product_view_robot_home_row_ = new QGroupBox(QStringLiteral("Robot Initial Pose"), controls_header);
  product_view_robot_home_row_->setObjectName(QStringLiteral("product_view_robot_initial_pose"));
  product_view_robot_home_row_->setCheckable(true);
  product_view_robot_home_row_->setChecked(false);
  auto * robot_home_layout = new QVBoxLayout(product_view_robot_home_row_);
  auto * robot_home_grid = new QGridLayout();
  const QStringList robot_joint_names{QStringLiteral("shoulder_pan_joint"), QStringLiteral("shoulder_lift_joint"),
    QStringLiteral("elbow_joint"), QStringLiteral("wrist_1_joint"), QStringLiteral("wrist_2_joint"),
    QStringLiteral("wrist_3_joint")};
  for (int row = 0; row < robot_joint_names.size(); ++row) {
    robot_home_grid->addWidget(new QLabel(robot_joint_names[row], product_view_robot_home_row_), row, 0);
    auto * spin = new QDoubleSpinBox(product_view_robot_home_row_);
    spin->setObjectName(QStringLiteral("product_view_robot_home_%1_degrees").arg(robot_joint_names[row]));
    spin->setRange(-360.0, 360.0);
    spin->setDecimals(2);
    spin->setSuffix(QStringLiteral("°"));
    robot_home_grid->addWidget(spin, row, 1);
    product_view_robot_home_spins_.push_back(spin);
  }
  robot_home_layout->addLayout(robot_home_grid);
  auto * robot_home_buttons = new QHBoxLayout();
  product_view_robot_home_current_button_ = new QPushButton(QStringLiteral("Current Scene Pose"), product_view_robot_home_row_);
  product_view_robot_home_suggested_button_ = new QPushButton(QStringLiteral("Suggested Pose"), product_view_robot_home_row_);
  product_view_robot_home_reset_button_ = new QPushButton(QStringLiteral("Reset Changes"), product_view_robot_home_row_);
  product_view_robot_home_apply_button_ = new QPushButton(QStringLiteral("Apply Initial Pose"), product_view_robot_home_row_);
  for (auto * button : {product_view_robot_home_current_button_, product_view_robot_home_suggested_button_,
      product_view_robot_home_reset_button_, product_view_robot_home_apply_button_}) robot_home_buttons->addWidget(button);
  robot_home_layout->addLayout(robot_home_buttons);
  product_view_robot_home_status_ = new QLabel(QStringLiteral("Authoring/simulation pose only; no robot commands are sent."), product_view_robot_home_row_);
  robot_home_layout->addWidget(product_view_robot_home_status_);
  const auto set_robot_home_expanded = [this](bool expanded) {
    const auto children = product_view_robot_home_row_->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly);
    for (auto * child : children) child->setVisible(expanded);
    product_view_robot_home_row_->updateGeometry();
  };
  connect(product_view_robot_home_row_, &QGroupBox::toggled, this, set_robot_home_expanded);
  set_robot_home_expanded(false);
  product_view_robot_home_row_->setVisible(false);
  controls->addWidget(product_view_robot_home_row_);
  connect(product_view_robot_home_current_button_, &QPushButton::clicked, this, &ScenePreviewWidget::reset_product_view_robot_home);
  connect(product_view_robot_home_reset_button_, &QPushButton::clicked, this, &ScenePreviewWidget::reset_product_view_robot_home);
  connect(product_view_robot_home_suggested_button_, &QPushButton::clicked, this, &ScenePreviewWidget::use_suggested_product_view_robot_home);
  connect(product_view_robot_home_apply_button_, &QPushButton::clicked, this, &ScenePreviewWidget::apply_product_view_robot_home);

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
  embedded_fit_button_->setObjectName(QStringLiteral("embeddedFitButton"));
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
    auto * asset_drop_web_view = new AssetDropWebEngineView(view3d_container_);
    embedded_web_view_ = asset_drop_web_view;
    asset_drop_web_view->drag_entered = [this](const QString & asset_id) {
      return catalog_asset_drag_enter_cb && catalog_asset_drag_enter_cb(asset_id);
    };
    asset_drop_web_view->drag_cancelled = [this]() {
      if (catalog_asset_drag_cancel_cb) catalog_asset_drag_cancel_cb();
    };
    embedded_web_view_->setObjectName("embeddedWeb3dProductView");
    embedded_web_view_->setFocusPolicy(Qt::StrongFocus);
    view3d_container_->setFocusProxy(embedded_web_view_);
    connect(embedded_web_view_, &QWebEngineView::loadFinished, this, [this](bool ok) {
      const EmbeddedWebRequestIdentity identity = embedded_web_loading_identity_;
      const quint64 navigation_token = embedded_web_loading_navigation_token_;
      const quint64 browser_load_token = embedded_web_loading_browser_load_token_;
      const QUrl expected_viewer_url = embedded_web_expected_viewer_url_;
      if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
          browser_load_token == 0 || browser_load_token != embedded_web_browser_load_token_) {
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
        handle_embedded_web_runtime_failure(identity, navigation_token, detail);
        emit studio_log_requested(QStringLiteral("Embedded Product View load failed. %1").arg(detail));
        return;
      }
      set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness,
        QStringLiteral("browser loaded; waiting for viewer readiness"));
      start_embedded_web_readiness_polling(identity, navigation_token, embedded_web_prepare_output_path_, expected_viewer_url.toString());
    });
    connect(embedded_web_view_, &QWebEngineView::urlChanged, this, [this](const QUrl & url) {
      if (url.scheme() != QStringLiteral("workcell-retry")) return;
      const QString requested_scene = url.host();
      if (requested_scene != normalized_preview_context(preview_context_).scene_id) return;
      show_embedded_web_loading_document(requested_scene);
      request_embedded_web_product_view_refresh(true, QStringLiteral("preparation_failure_retry"));
    });
    connect(embedded_web_view_->page(), &QWebEnginePage::renderProcessTerminated, this,
      [this](QWebEnginePage::RenderProcessTerminationStatus status, int exit_code) {
        const EmbeddedWebRequestIdentity identity = embedded_web_loading_identity_;
        const quint64 navigation_token = embedded_web_loading_navigation_token_;
        const QString detail = QStringLiteral("browser render process terminated for Product View scene %1; status=%2 exit_code=%3; viewer URL: %4")
          .arg(identity.scene_id).arg(static_cast<int>(status)).arg(exit_code).arg(embedded_web_expected_viewer_url_.toString());
        handle_embedded_web_runtime_failure(identity, navigation_token, detail);
        emit studio_log_requested(QStringLiteral("Embedded Product View render-process termination. %1").arg(detail));
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
    if (!v) { set_authoring_mode(gizmo_mode_selector_->currentText()); refresh_info_chip(); return; }
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
    if (!v) { set_authoring_mode(choice); return; }
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
  connect(embedded_fit_button_, &QPushButton::clicked, this, [this](){
    if (embedded_product_view_state_ == EmbeddedProductViewState::Failed) {
      request_embedded_web_product_view_refresh(true, QStringLiteral("user_retry"));
      return;
    }
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.fitScene()"));
  });
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
  const bool has_complete_preview_context =
    !preview_context_.scene_id.trimmed().isEmpty() &&
    !preview_context_.absolute_scene_dir.trimmed().isEmpty() &&
    !preview_context_.absolute_repo_root.trimmed().isEmpty();
  if (scene_input.isEmpty() && !has_complete_preview_context) {
    if (diagnostic_debug_logging_enabled()) {
      log_diagnostic(QStringLiteral("Embedded Product View repo root resolution deferred until scene_id, scene_dir, and repo root context are available."));
    }
    return QString();
  }
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
  refresh_product_view_destination_control();
  refresh_product_view_tool_orientation_control();
  refresh_product_view_robot_home_control();
  embedded_editor_polling_ = (state == EmbeddedProductViewState::Ready);
  if (state == EmbeddedProductViewState::Failed) embedded_web_last_error_ = detail;
  if (embedded_fit_button_ && state != EmbeddedProductViewState::Failed) {
    embedded_fit_button_->setText(QStringLiteral("Fit"));
    embedded_fit_button_->setToolTip(QString());
  }
  if (state == EmbeddedProductViewState::Failed && !detail.trimmed().isEmpty()) {
    emit studio_log_requested(QStringLiteral("Embedded Product View failure: %1").arg(detail));
  }
  refresh_toolbar_status_chip();
  refresh_toolbar_feedback_row();
  if (error_state_label_) error_state_label_->setVisible(false);
  QString state_text = QStringLiteral("preparing");
  if (state == EmbeddedProductViewState::Ready) state_text = QStringLiteral("ready");
  else if (state == EmbeddedProductViewState::CompatibilityReady) state_text = QStringLiteral("compatibility ready");
  else if (state == EmbeddedProductViewState::Failed) state_text = QStringLiteral("failed");
  emit embedded_product_view_runtime_state_changed(state_text, runtime_preview_has_usable_content());
}

void ScenePreviewWidget::refresh_product_view_tool_orientation_control()
{
  if (!product_view_tool_orientation_row_) return;
  const bool strict_ready = embedded_product_view_state_ == EmbeddedProductViewState::Ready &&
    !property("diagnostic_preview_active").toBool() && !product_view_tool_orientation_save_in_progress_;
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  workcell_builder::RobotMountConfig robot_mount;
  workcell_builder::ToolAttachmentConfig tool_attachment;
  std::vector<std::string> warnings;
  const bool loaded = strict_ready && workcell_builder::load_robot_tool_pose_from_environment_yaml(
    environment_path.toStdString(), &robot_mount, &tool_attachment, &warnings) &&
    !tool_attachment.parent_link.empty() && !tool_attachment.child_link.empty();
  product_view_tool_orientation_row_->setVisible(loaded);
  product_view_tool_orientation_loaded_ = loaded;
  if (loaded) {
    loaded_product_view_tool_rpy_[0] = tool_attachment.roll;
    loaded_product_view_tool_rpy_[1] = tool_attachment.pitch;
    loaded_product_view_tool_rpy_[2] = tool_attachment.yaw;
    reset_product_view_tool_orientation();
  }
  for (auto * spin : {product_view_tool_roll_spin_, product_view_tool_pitch_spin_, product_view_tool_yaw_spin_}) spin->setEnabled(loaded);
  product_view_tool_orientation_preset_->setEnabled(loaded);
  product_view_tool_orientation_reset_button_->setEnabled(loaded);
  product_view_apply_tool_orientation_->setEnabled(loaded);
}

void ScenePreviewWidget::reset_product_view_tool_orientation()
{
  if (!product_view_tool_orientation_loaded_) return;
  const QSignalBlocker preset_blocker(product_view_tool_orientation_preset_);
  product_view_tool_orientation_preset_->setCurrentIndex(0);
  product_view_tool_roll_spin_->setValue(qRadiansToDegrees(loaded_product_view_tool_rpy_[0]));
  product_view_tool_pitch_spin_->setValue(qRadiansToDegrees(loaded_product_view_tool_rpy_[1]));
  product_view_tool_yaw_spin_->setValue(qRadiansToDegrees(loaded_product_view_tool_rpy_[2]));
}

void ScenePreviewWidget::apply_product_view_tool_orientation()
{
  if (!product_view_tool_orientation_loaded_ || product_view_tool_orientation_save_in_progress_ ||
      embedded_product_view_state_ != EmbeddedProductViewState::Ready || property("diagnostic_preview_active").toBool()) return;
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  workcell_builder::RobotMountConfig robot_mount;
  workcell_builder::ToolAttachmentConfig tool_attachment;
  std::vector<std::string> warnings;
  if (!workcell_builder::load_robot_tool_pose_from_environment_yaml(
      environment_path.toStdString(), &robot_mount, &tool_attachment, &warnings)) {
    emit studio_issue_requested(QStringLiteral("Tool orientation save failed: could not read %1").arg(environment_path),
      QStringLiteral("Error"), QStringLiteral("product_view_tool_orientation_save"));
    return;
  }
  tool_attachment.roll = qDegreesToRadians(product_view_tool_roll_spin_->value());
  tool_attachment.pitch = qDegreesToRadians(product_view_tool_pitch_spin_->value());
  tool_attachment.yaw = qDegreesToRadians(product_view_tool_yaw_spin_->value());
  product_view_tool_orientation_save_in_progress_ = true;
  refresh_product_view_tool_orientation_control();
  const QString backup_path = environment_path + QStringLiteral(".robot_tool_pose.") +
    QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddHHmmss")) + QStringLiteral(".bak");
  const bool backup_ok = QFile::copy(environment_path, backup_path);
  const auto saved = backup_ok ? workcell_builder::save_robot_tool_pose_to_environment_yaml(
    environment_path.toStdString(), robot_mount, tool_attachment) : workcell_builder::PlacedObjectYamlWriteResult{};
  product_view_tool_orientation_save_in_progress_ = false;
  if (!saved.ok) {
    emit studio_issue_requested(QStringLiteral("Tool orientation save failed for %1: %2").arg(environment_path,
      backup_ok ? QStringLiteral("environment YAML could not be written") : QStringLiteral("timestamped backup could not be created at %1").arg(backup_path)),
      QStringLiteral("Error"), QStringLiteral("product_view_tool_orientation_save"));
    refresh_product_view_tool_orientation_control();
    return;
  }
  loaded_product_view_tool_rpy_[0] = tool_attachment.roll;
  loaded_product_view_tool_rpy_[1] = tool_attachment.pitch;
  loaded_product_view_tool_rpy_[2] = tool_attachment.yaw;
  emit studio_log_requested(QStringLiteral("Product View tool orientation saved to %1; backup: %2; regenerating around %3 attachment.")
    .arg(environment_path, backup_path, QString::fromStdString(tool_attachment.parent_link)));
  request_embedded_web_product_view_refresh(true, QStringLiteral("tool_orientation_update"));
}

void ScenePreviewWidget::refresh_product_view_robot_home_control()
{
  if (!product_view_robot_home_row_) return;
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  workcell_builder::RobotHomeConfig loaded;
  std::string detail;
  const bool available = QFileInfo::exists(environment_path) &&
    workcell_builder::load_robot_home_from_environment_yaml(environment_path.toStdString(), &loaded, &detail);
  const bool enabled = available && embedded_product_view_state_ == EmbeddedProductViewState::Ready &&
    !property("diagnostic_preview_active").toBool() && !product_view_robot_home_save_in_progress_;
  product_view_robot_home_row_->setVisible(available);
  product_view_robot_home_loaded_ = available;
  if (available && !product_view_robot_home_save_in_progress_) {
    loaded_product_view_robot_home_ = loaded;
    reset_product_view_robot_home();
  }
  for (auto * spin : product_view_robot_home_spins_) spin->setEnabled(enabled);
  for (auto * button : {product_view_robot_home_current_button_, product_view_robot_home_suggested_button_,
      product_view_robot_home_reset_button_, product_view_robot_home_apply_button_}) button->setEnabled(enabled);
  if (!available && !detail.empty()) product_view_robot_home_status_->setText(
    QStringLiteral("Robot Initial Pose unavailable: %1").arg(QString::fromStdString(detail)));
  else if (!enabled) product_view_robot_home_status_->setText(
    QStringLiteral("Robot Initial Pose is disabled while Product View is loading, saving, or showing diagnostics."));
  else product_view_robot_home_status_->setText(
    QStringLiteral("Values are displayed in degrees and stored scene-locally in radians."));
}

void ScenePreviewWidget::reset_product_view_robot_home()
{
  if (!product_view_robot_home_loaded_) return;
  for (int index = 0; index < product_view_robot_home_spins_.size() &&
      index < static_cast<int>(loaded_product_view_robot_home_.joint_order.size()); ++index) {
    const auto & name = loaded_product_view_robot_home_.joint_order[static_cast<size_t>(index)];
    const QSignalBlocker blocker(product_view_robot_home_spins_[index]);
    product_view_robot_home_spins_[index]->setValue(
      workcell_builder::robot_home_radians_to_degrees(loaded_product_view_robot_home_.joints.at(name)));
  }
}

void ScenePreviewWidget::use_suggested_product_view_robot_home()
{
  if (!product_view_robot_home_loaded_) return;
  for (int index = 0; index < product_view_robot_home_spins_.size() &&
      index < static_cast<int>(loaded_product_view_robot_home_.joint_order.size()); ++index) {
    const auto & name = loaded_product_view_robot_home_.joint_order[static_cast<size_t>(index)];
    product_view_robot_home_spins_[index]->setValue(
      workcell_builder::robot_home_radians_to_degrees(loaded_product_view_robot_home_.suggested_joints.at(name)));
  }
}

void ScenePreviewWidget::apply_product_view_robot_home()
{
  if (!product_view_robot_home_loaded_ || product_view_robot_home_save_in_progress_ ||
      embedded_product_view_state_ != EmbeddedProductViewState::Ready || property("diagnostic_preview_active").toBool()) return;
  workcell_builder::RobotHomeConfig edited = loaded_product_view_robot_home_;
  edited.source = "user";
  for (int index = 0; index < product_view_robot_home_spins_.size() &&
      index < static_cast<int>(edited.joint_order.size()); ++index) {
    const double radians = workcell_builder::robot_home_degrees_to_radians(product_view_robot_home_spins_[index]->value());
    if (!std::isfinite(radians)) {
      emit studio_issue_requested(QStringLiteral("Robot Initial Pose save blocked: every joint value must be finite."),
        QStringLiteral("Error"), QStringLiteral("product_view_robot_home_save"));
      return;
    }
    edited.joints[edited.joint_order[static_cast<size_t>(index)]] = radians;
  }
  product_view_robot_home_save_in_progress_ = true;
  refresh_product_view_robot_home_control();
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  const auto result = workcell_builder::save_robot_home_to_environment_yaml(environment_path.toStdString(), edited);
  product_view_robot_home_save_in_progress_ = false;
  if (!result.ok) {
    emit studio_issue_requested(QStringLiteral("Robot Initial Pose save failed for %1: %2")
      .arg(environment_path, QString::fromStdString(result.detail)), QStringLiteral("Error"),
      QStringLiteral("product_view_robot_home_save"));
    refresh_product_view_robot_home_control();
    return;
  }
  loaded_product_view_robot_home_ = edited;
  product_view_robot_home_status_->setText(QStringLiteral("Initial pose saved; strictly regenerating Product View."));
  emit studio_log_requested(QStringLiteral("Product View Robot Initial Pose saved to %1; backup: %2. No motion command was sent.")
    .arg(environment_path, QString::fromStdString(result.backup_path)));
  request_embedded_web_product_view_refresh(true, QStringLiteral("robot_initial_pose_update"));
}

void ScenePreviewWidget::refresh_product_view_destination_control()
{
  if (!product_view_destination_row_) return;
  const bool strict_ready = embedded_product_view_state_ == EmbeddedProductViewState::Ready &&
    !property("diagnostic_preview_active").toBool() && !product_view_destination_save_in_progress_;
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  active_product_view_place_zone_id_.clear();
  QString current_destination;
  bool editable_pick_place = false;
  try {
    const YAML::Node root = YAML::LoadFile(environment_path.toStdString());
    const YAML::Node task = root["task"];
    const std::string task_kind = task["type"].as<std::string>(task["template"].as<std::string>(""));
    editable_pick_place = task_kind == "pick_place";
    std::string place_zone_id = task["place"]["target_ref"].as<std::string>("");
    if (place_zone_id.empty()) {
      const YAML::Node rules = task["rules"];
      if (rules && rules.IsSequence()) {
        for (const auto & rule : rules) {
          const YAML::Node when = rule["when"];
          if (when && (when["always"].as<bool>(false) || when["default"].as<bool>(false))) {
            place_zone_id = rule["destination"].as<std::string>("");
            if (!place_zone_id.empty()) break;
          }
        }
      }
    }
    const YAML::Node zones = root["task_zones"];
    if (zones && zones.IsSequence()) {
      for (const auto & zone : zones) {
        const std::string id = zone["id"].as<std::string>("");
        const std::string role = zone["role"].as<std::string>(zone["type"].as<std::string>(""));
        if (id == place_zone_id && (role == "place" || role == "place_zone")) {
          active_product_view_place_zone_id_ = QString::fromStdString(id);
          current_destination = QString::fromStdString(zone["target_ref"].as<std::string>(""));
          break;
        }
      }
    }
  } catch (const std::exception & e) {
    if (product_view_destination_status_) {
      product_view_destination_status_->setText(QStringLiteral("Destination unavailable: %1").arg(e.what()));
    }
  }

  const bool resolved = editable_pick_place && !active_product_view_place_zone_id_.isEmpty();
  product_view_destination_row_->setVisible(strict_ready && resolved);
  if (!resolved) {
    if (product_view_destination_combo_) product_view_destination_combo_->setEnabled(false);
    if (product_view_apply_destination_button_) product_view_apply_destination_button_->setEnabled(false);
    return;
  }

  const QSignalBlocker blocker(product_view_destination_combo_);
  product_view_destination_combo_->clear();
  const auto candidates = workcell_builder::discover_task_area_destinations(environment_path.toStdString());
  for (const auto & candidate : candidates) {
    product_view_destination_combo_->addItem(
      QString::fromStdString(candidate.display_name), QString::fromStdString(candidate.id));
  }
  const int current_index = product_view_destination_combo_->findData(current_destination);
  if (current_index >= 0) {
    product_view_destination_combo_->setCurrentIndex(current_index);
    product_view_destination_status_->setText(QStringLiteral("Current destination: %1").arg(current_destination));
  } else {
    product_view_destination_combo_->insertItem(0, QStringLiteral("Missing destination: %1").arg(
      current_destination.isEmpty() ? QStringLiteral("<none>") : current_destination), QVariant());
    product_view_destination_combo_->setCurrentIndex(0);
    product_view_destination_status_->setText(QStringLiteral("Missing destination: %1").arg(
      current_destination.isEmpty() ? QStringLiteral("<none>") : current_destination));
  }
  product_view_destination_combo_->setEnabled(strict_ready);
  product_view_apply_destination_button_->setEnabled(strict_ready && current_index >= 0);
}

void ScenePreviewWidget::apply_product_view_destination()
{
  if (product_view_destination_save_in_progress_ || !product_view_destination_combo_ ||
    active_product_view_place_zone_id_.isEmpty()) return;
  const QString destination_id = product_view_destination_combo_->currentData().toString().trimmed();
  const QString environment_path = QDir(preview_context_.absolute_scene_dir).filePath(QStringLiteral("environment.yaml"));
  const auto candidates = workcell_builder::discover_task_area_destinations(environment_path.toStdString());
  const bool valid_destination = std::any_of(candidates.begin(), candidates.end(), [&destination_id](const auto & candidate) {
    return QString::fromStdString(candidate.id) == destination_id;
  });
  if (!valid_destination) {
    product_view_destination_status_->setText(QStringLiteral("Missing destination: selection is not a valid physical bin."));
    emit studio_issue_requested(product_view_destination_status_->text(), QStringLiteral("Error"),
      QStringLiteral("product_view_destination_invalid"));
    return;
  }

  product_view_destination_save_in_progress_ = true;
  product_view_destination_combo_->setEnabled(false);
  product_view_apply_destination_button_->setEnabled(false);
  std::vector<std::string> warnings;
  auto zones = workcell_builder::load_task_zones_from_environment_yaml(environment_path.toStdString(), &warnings);
  const std::string place_zone_id = active_product_view_place_zone_id_.toStdString();
  auto zone = std::find_if(zones.begin(), zones.end(), [&place_zone_id](const auto & candidate) {
    return candidate.id == place_zone_id && (candidate.role == "place" || candidate.type == "place_zone");
  });
  if (zone == zones.end()) {
    product_view_destination_save_in_progress_ = false;
    product_view_destination_status_->setText(QStringLiteral("Save failed: active place zone could not be resolved in %1").arg(environment_path));
    emit studio_issue_requested(product_view_destination_status_->text(), QStringLiteral("Error"),
      QStringLiteral("product_view_destination_save"));
    refresh_product_view_destination_control();
    return;
  }
  zone->target_ref = destination_id.toStdString();
  const QString backup_path = environment_path + QStringLiteral(".task_areas.") +
    QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddHHmmss")) + QStringLiteral(".bak");
  bool backup_ok = false;
  if (QFile::exists(backup_path)) QFile::remove(backup_path);
  backup_ok = QFile::copy(environment_path, backup_path);
  if (!backup_ok) warnings.push_back("backup failed for " + backup_path.toStdString());
  const auto saved = backup_ok ? workcell_builder::save_task_zones_to_environment_yaml(
    environment_path.toStdString(), zones) : workcell_builder::PlacedObjectYamlWriteResult{};
  product_view_destination_save_in_progress_ = false;
  if (!saved.ok) {
    const QString detail = warnings.empty() && saved.warnings.empty() ? QStringLiteral("unknown filesystem error") :
      QString::fromStdString(!saved.warnings.empty() ? saved.warnings.front() : warnings.front());
    product_view_destination_status_->setText(QStringLiteral("Save failed for %1: %2").arg(environment_path, detail));
    emit studio_issue_requested(product_view_destination_status_->text(), QStringLiteral("Error"),
      QStringLiteral("product_view_destination_save"));
    refresh_product_view_destination_control();
    return;
  }
  product_view_destination_status_->setText(QStringLiteral("Destination rebound to %1; refreshing Product View.").arg(destination_id));
  emit studio_log_requested(QStringLiteral("Product View destination rebound: place zone %1 -> %2; backup: %3")
    .arg(active_product_view_place_zone_id_, destination_id, backup_path));
  request_embedded_web_product_view_refresh(true, QStringLiteral("destination_rebind"));
}

void ScenePreviewWidget::start_embedded_web_server_probes(
  const EmbeddedWebRequestIdentity & identity, int port, quint64 navigation_token, const QString & repo_root)
{
  if (!embedded_web_identity_is_current(identity) || identity.absolute_repo_root != repo_root ||
      identity.selected_server_port != port || port <= 0) return;
  const QString marker_path = QStringLiteral("workcell_studio_web/viewer/workcell_runtime_marker.json");
  QFile marker(QDir(repo_root).filePath(marker_path));
  if (!marker.open(QIODevice::ReadOnly) || marker.readAll().trimmed().isEmpty()) {
    fail_embedded_web_server_probe(identity, port, navigation_token,
      QStringLiteral("selected repository marker is missing or empty: %1").arg(QDir(repo_root).filePath(marker_path)));
    return;
  }
  marker.seek(0);
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};
  embedded_web_server_probe_.identity = identity;
  embedded_web_server_probe_.port = port;
  embedded_web_server_probe_.navigation_token = navigation_token;
  embedded_web_server_probe_.expected_marker = marker.readAll().trimmed();
  embedded_web_server_probe_.deadline = QDateTime::currentDateTimeUtc().addSecs(8);
  embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerProbing;
  set_embedded_product_view_state(EmbeddedProductViewState::Preparing, QStringLiteral("server_probing"));
  run_embedded_web_server_probes(identity, port, navigation_token);
}

void ScenePreviewWidget::run_embedded_web_server_probes(
  const EmbeddedWebRequestIdentity & identity, int port, quint64 navigation_token)
{
  const auto current = [this, &identity, port, navigation_token]() {
    return embedded_web_identity_is_current(identity) && identity.selected_server_port == port && embedded_web_server_probe_.identity == identity &&
      embedded_web_server_probe_.port == port && embedded_web_server_probe_.navigation_token == navigation_token;
  };
  if (!current() || embedded_web_server_probe_.terminal_recorded) return;
  if (QDateTime::currentDateTimeUtc() >= embedded_web_server_probe_.deadline) {
    fail_embedded_web_server_probe(identity, port, navigation_token, QStringLiteral("server resource probes timed out after 8 seconds"));
    return;
  }
  if (!embedded_web_network_manager_) embedded_web_network_manager_ = new QNetworkAccessManager(this);
  const QStringList paths{
    QStringLiteral("workcell_studio_web/viewer/workcell_runtime_marker.json"),
    QStringLiteral("workcell_studio_web/viewer/index.html"),
    QStringLiteral("workcell_studio_web/viewer/dist/viewer.bundle.js"),
    QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(identity.scene_id)
  };
  embedded_web_server_probe_.pending_replies = paths.size();
  embedded_web_server_probe_.retryable_failure = false;
  embedded_web_server_probe_.failure_detail.clear();
  for (const QString & path : paths) {
    const QUrl url(QStringLiteral("http://127.0.0.1:%1/%2").arg(port).arg(path));
    QNetworkReply * reply = embedded_web_network_manager_->get(QNetworkRequest(url));
    connect(reply, &QNetworkReply::finished, this, [this, identity, port, navigation_token, path, reply]() {
      const bool current = embedded_web_identity_is_current(identity) && identity.selected_server_port == port && embedded_web_server_probe_.identity == identity &&
        embedded_web_server_probe_.port == port && embedded_web_server_probe_.navigation_token == navigation_token;
      if (!current || embedded_web_server_probe_.terminal_recorded) { reply->deleteLater(); return; }
      const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
      const QByteArray body = reply->readAll();
      const bool marker = path.endsWith(QStringLiteral("workcell_runtime_marker.json"));
      const bool marker_matches = !marker || body.trimmed() == embedded_web_server_probe_.expected_marker;
      const bool ok = reply->error() == QNetworkReply::NoError && status == 200 && marker_matches;
      if (!ok) {
        const bool terminal_http_failure = status >= 400 && status < 500;
        if (terminal_http_failure || (marker && status == 200 && !marker_matches)) {
          embedded_web_server_probe_.failure_detail = QStringLiteral("resource probe failed for %1: HTTP %2%3")
            .arg(path).arg(status).arg(marker && !marker_matches ? QStringLiteral(" (marker does not match selected repository root)") : QString());
          embedded_web_server_probe_.retryable_failure = false;
        } else if (embedded_web_server_probe_.failure_detail.isEmpty()) {
          embedded_web_server_probe_.failure_detail = QStringLiteral("resource probe pending for %1: %2")
            .arg(path, reply->errorString());
          embedded_web_server_probe_.retryable_failure = true;
        }
      }
      reply->deleteLater();
      if (--embedded_web_server_probe_.pending_replies != 0) return;
      if (embedded_web_server_probe_.failure_detail.isEmpty()) {
        embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerReady;
        emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View server verified: all required resources returned HTTP 200 on port %1.").arg(port));
        load_prepared_embedded_web_scene(identity);
        return;
      }
      if (!embedded_web_server_is_owned_) {
        // Never reuse or terminate an untrusted endpoint. A marker mismatch
        // receives an alternate port; a refused endpoint may be claimed.
        select_owned_embedded_web_server(identity,
          !embedded_web_server_probe_.failure_detail.contains(QStringLiteral("marker does not match")));
        return;
      }
      if (!embedded_web_server_probe_.retryable_failure) {
        fail_embedded_web_server_probe(identity, port, navigation_token, embedded_web_server_probe_.failure_detail);
        return;
      }
      QTimer::singleShot(200, this, [this, identity, port, navigation_token]() {
        if (!embedded_web_identity_is_current(identity) || identity.selected_server_port != port || embedded_web_server_probe_.identity != identity ||
            embedded_web_server_probe_.port != port || embedded_web_server_probe_.navigation_token != navigation_token) return;
        run_embedded_web_server_probes(identity, port, navigation_token);
      });
    });
  }
}

void ScenePreviewWidget::fail_embedded_web_server_probe(
  const EmbeddedWebRequestIdentity & identity, int port, quint64 navigation_token, const QString & detail)
{
  if (!embedded_web_identity_is_current(identity) || identity.selected_server_port != port || embedded_web_server_probe_.identity != identity ||
      embedded_web_server_probe_.port != port || embedded_web_server_probe_.navigation_token != navigation_token ||
      embedded_web_server_probe_.terminal_recorded) return;
  embedded_web_server_probe_.terminal_recorded = true;
  embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerNotStarted;
  emit studio_log_requested(QStringLiteral("Embedded Product View server terminal diagnostic: scene=%1 generation=%2 port=%3 navigation=%4 detail=%5")
    .arg(identity.scene_id).arg(identity.generation).arg(port).arg(navigation_token).arg(detail));
  handle_embedded_web_runtime_failure(identity, navigation_token, detail);
}

QString ScenePreviewWidget::embedded_web_recovery_key(const EmbeddedWebRequestIdentity & identity) const
{
  // Bound automatic recovery by logical navigation/request identity, not by the
  // retry generation number.  A forced retry keeps the same scene/payload key,
  // so a second failure leaves Web3D selected and exposes the manual Retry path.
  return QStringLiteral("%1|%2|%3|%4|%5")
    .arg(identity.scene_id, identity.absolute_scene_dir, identity.absolute_repo_root,
         QString::fromLatin1(identity.payload_fingerprint.toHex()))
    .arg(identity.payload_revision);
}

void ScenePreviewWidget::handle_embedded_web_runtime_failure(
  const EmbeddedWebRequestIdentity & identity, quint64 navigation_token, const QString & detail)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(navigation_token); Q_UNUSED(detail);
#else
  if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_) {
    emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View runtime failure for scene %1 revision %2 navigation %3: %4")
      .arg(identity.scene_id).arg(identity.generation).arg(navigation_token).arg(detail));
    return;
  }
  scene_load_diagnostics_.set_debug_enabled(diagnostic_debug_logging_enabled());
  const SceneLoadDiagnosticContext::Identity diagnostic_identity{
    identity.scene_id, QStringLiteral("builder_revision:%1").arg(identity.payload_revision), navigation_token};
  const QJsonObject failure_content{
    {QStringLiteral("state"), QStringLiteral("scene_failed")},
    {QStringLiteral("detail"), detail}};
  const auto failure_report = scene_load_diagnostics_.observe(
    diagnostic_identity, QStringLiteral("terminal_readiness"), failure_content, true);
  if (!failure_report.should_emit) return;
  emit studio_log_requested(failure_report.summary + QStringLiteral(" content=%1")
    .arg(QString::fromUtf8(QJsonDocument(failure_content).toJson(QJsonDocument::Compact))));
  if (finish_post_save_product_view_refresh(identity, navigation_token, false, detail)) return;
  native_compatibility_fallback_active_ = false;
  embedded_web_last_error_ = detail;
  set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail);
  if (mode_selector_) mode_selector_->setItemText(0, QStringLiteral("Web3D Product View"));
  if (embedded_web_view_) embedded_web_view_->setVisible(true);
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(false);
  if (embedded_fit_button_) {
    embedded_fit_button_->setText(QStringLiteral("Retry"));
    embedded_fit_button_->setToolTip(QStringLiteral("Retry loading the embedded Web3D Product View. Details remain available in Studio Log."));
  }
  refresh_mode_and_state();

  ++embedded_web_terminal_results_accepted_;
  if (diagnostic_debug_logging_enabled()) emit studio_log_requested(
    QStringLiteral("Embedded Product View terminal failure accepted; leaving Web3D selected with Retry available. %1").arg(detail));
#endif
}

bool ScenePreviewWidget::finish_post_save_product_view_refresh(
  const EmbeddedWebRequestIdentity & identity, quint64 navigation_token, bool success, const QString & detail)
{
  if (post_save_refresh_generation_ == 0 ||
      identity.generation != post_save_refresh_generation_ ||
      static_cast<int>(identity.payload_revision) != post_save_refresh_payload_revision_) return false;
  const int revision = post_save_refresh_payload_revision_;
  post_save_refresh_generation_ = 0;
  post_save_refresh_payload_revision_ = 0;
  emit post_save_product_view_refresh_finished(
    revision, identity.generation, navigation_token, success, detail);
  return !success;
}

void ScenePreviewWidget::ensure_embedded_web_server_started(const QString & repo_root, const EmbeddedWebRequestIdentity & identity)
{
  if (repo_root.trimmed().isEmpty() || repo_root != identity.absolute_repo_root ||
      identity.selected_server_port <= 0 || !embedded_web_identity_is_current(identity)) return;
  EmbeddedWebRequestIdentity session_identity = identity;
  if (embedded_web_server_is_owned_ && embedded_web_server_process_ &&
      embedded_web_server_process_->state() != QProcess::NotRunning &&
      embedded_web_server_session_repo_root_ == repo_root && embedded_web_server_session_port_ > 0) {
    session_identity.selected_server_port = embedded_web_server_session_port_;
    embedded_web_active_identity_ = session_identity;
    embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerReady;
    load_prepared_embedded_web_scene(session_identity);
    return;
  }
  const int port = identity.selected_server_port;
  const quint64 navigation_token = ++embedded_web_navigation_token_;
  // This probe is for an endpoint whose ownership is not yet established.
  // An old owned process remains tracked separately and is never confused with
  // a listener discovered at the configured port.
  embedded_web_server_is_owned_ = false;
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};
  embedded_web_server_probe_.identity = identity;
  embedded_web_server_probe_.port = port;
  embedded_web_server_probe_.navigation_token = navigation_token;
  // Probe the configured/default endpoint first; it is reusable only after all
  // resources and the repository marker have been verified.
  start_embedded_web_server_probes(identity, port, navigation_token, repo_root);
}

void ScenePreviewWidget::select_owned_embedded_web_server(
  const EmbeddedWebRequestIdentity & identity, bool use_current_port)
{
  if (!embedded_web_identity_is_current(identity)) return;
  int port = identity.selected_server_port;
  if (!use_current_port) {
    QTcpServer socket;
    if (!socket.listen(QHostAddress::LocalHost, 0)) {
      fail_embedded_web_server_probe(identity, identity.selected_server_port, embedded_web_navigation_token_,
        QStringLiteral("could not select an alternate loopback port: %1").arg(socket.errorString()));
      return;
    }
    port = socket.serverPort();
  }
  EmbeddedWebRequestIdentity owned_identity = identity;
  owned_identity.selected_server_port = port;
  // This is a new immutable active request. Delayed callbacks from the
  // untrusted endpoint are now stale before any owned process is launched.
  embedded_web_active_identity_ = owned_identity;
  embedded_web_server_port_ = port;
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};
  start_owned_embedded_web_server(owned_identity);
}

void ScenePreviewWidget::start_owned_embedded_web_server(const EmbeddedWebRequestIdentity & identity)
{
  if (!embedded_web_identity_is_current(identity) || identity.absolute_repo_root.isEmpty() || identity.selected_server_port <= 0) return;
  const QString repo_root = identity.absolute_repo_root;
  const int port = identity.selected_server_port;
  const quint64 navigation_token = ++embedded_web_navigation_token_;
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};
  embedded_web_server_probe_.identity = identity;
  embedded_web_server_probe_.port = port;
  embedded_web_server_probe_.navigation_token = navigation_token;
  if (embedded_web_server_process_ && embedded_web_server_process_->state() != QProcess::NotRunning) {
    // This is an earlier owned request, never an arbitrary listener. Retire it
    // before replacing it; callbacks remain guarded by their old identity.
    disconnect(embedded_web_server_process_, nullptr, this, nullptr);
    embedded_web_server_process_->terminate();
    if (!embedded_web_server_process_->waitForFinished(1000)) {
      embedded_web_server_process_->kill();
      embedded_web_server_process_->waitForFinished(1000);
    }
    embedded_web_server_process_->deleteLater();
    embedded_web_server_process_ = nullptr;
    embedded_web_server_is_owned_ = false;
  }
  embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerStarting;
  set_embedded_product_view_state(EmbeddedProductViewState::StartingServer, QStringLiteral("server_starting"));
  if (embedded_web_server_process_) embedded_web_server_process_->deleteLater();
  embedded_web_server_process_ = new QProcess(this);
  embedded_web_server_is_owned_ = true;
  embedded_web_server_session_repo_root_ = repo_root;
  embedded_web_server_session_port_ = port;
  QProcess * const process = embedded_web_server_process_;
  process->setProgram(QStringLiteral("python3"));
  process->setArguments(QStringList{"-m", "http.server", QString::number(port), "--bind", "127.0.0.1", "--directory", repo_root});
  process->setWorkingDirectory(repo_root);
  process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process->setProcessChannelMode(QProcess::MergedChannels);
  connect(process, &QProcess::started, this, [this, identity, port, navigation_token, process, repo_root]() {
    if (process != embedded_web_server_process_ || !embedded_web_identity_is_current(identity) || identity.selected_server_port != port ||
        embedded_web_server_probe_.identity != identity || embedded_web_server_probe_.port != port ||
        embedded_web_server_probe_.navigation_token != navigation_token) return;
    emit studio_log_requested(QStringLiteral("Embedded Product View server_starting milestone: port=%1.").arg(port));
    start_embedded_web_server_probes(identity, port, navigation_token, repo_root);
  });
  connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this, identity, port, navigation_token, process](int exit_code, QProcess::ExitStatus) {
    if (process != embedded_web_server_process_ || !embedded_web_identity_is_current(identity) || identity.selected_server_port != port ||
        embedded_web_server_probe_.identity != identity || embedded_web_server_probe_.port != port ||
        embedded_web_server_probe_.navigation_token != navigation_token) return;
    fail_embedded_web_server_probe(identity, port, navigation_token, QStringLiteral("local server exited with code %1: %2").arg(exit_code).arg(QString::fromUtf8(process->readAll()).trimmed().left(240)));
  });
  connect(process, &QProcess::errorOccurred, this, [this, identity, port, navigation_token, process](QProcess::ProcessError error) {
    if (process != embedded_web_server_process_ || !embedded_web_identity_is_current(identity) || identity.selected_server_port != port ||
        embedded_web_server_probe_.identity != identity || embedded_web_server_probe_.port != port ||
        embedded_web_server_probe_.navigation_token != navigation_token) return;
    fail_embedded_web_server_probe(identity, port, navigation_token,
      QStringLiteral("local server startup failure (%1): %2").arg(static_cast<int>(error)).arg(process->errorString()));
  });
  process->start();
}


void ScenePreviewWidget::retire_embedded_web_navigation_for_handoff()
{
  ++embedded_web_navigation_token_;
  ++embedded_web_readiness_token_;
  ++embedded_web_browser_load_token_;
  embedded_web_loading_navigation_token_ = 0;
  embedded_web_loading_browser_load_token_ = 0;
  embedded_web_loading_identity_ = EmbeddedWebRequestIdentity{};
  embedded_web_prepared_identity_ = EmbeddedWebRequestIdentity{};
  embedded_web_expected_viewer_url_ = QUrl();
  embedded_editor_polling_ = false;
  embedded_web_readiness_deadline_ = QDateTime();
  embedded_web_last_boot_status_.clear();
  // Do not stop or unmount the Qt 5 WebEngine surface here. Loading the newest
  // guarded URL replaces any in-flight navigation, while an already committed
  // frame remains compositor-owned and available throughout the handoff.
}

void ScenePreviewWidget::clear_embedded_editor_state_for_scene_handoff()
{
  cancel_embedded_asset_placement();
  ++embedded_editor_state_request_token_;
  embedded_editor_polling_ = false;
  selected_preview_item_id_.clear();
  embedded_browser_selected_item_id_.clear();
  if (embedded_undo_button_) embedded_undo_button_->setEnabled(false);
  if (embedded_redo_button_) embedded_redo_button_->setEnabled(false);
  if (gizmo_mode_selector_) {
    const QSignalBlocker blocker(gizmo_mode_selector_);
    gizmo_mode_selector_->setCurrentText(QStringLiteral("Select"));
  }
  if (interaction_mode_selector_) {
    const QSignalBlocker blocker(interaction_mode_selector_);
    interaction_mode_selector_->setCurrentText(QStringLiteral("Select"));
  }
  if (auto * viewport = active_native_viewport()) {
    viewport->selected_id.clear();
    viewport->update();
  }
  emit preview_item_selected(QString(), QStringLiteral("scene_identity_handoff"));
}

void ScenePreviewWidget::show_embedded_web_loading_document(const QString & scene_id)
{
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (!embedded_web_view_) return;
  const QString safe_scene = scene_id.trimmed().isEmpty() ? QStringLiteral("No scene selected") : scene_id.toHtmlEscaped();
  const QString html = QStringLiteral(R"HTML(<!doctype html><html><head><meta charset="utf-8"><style>
body{margin:0;background:#0f172a;color:#e2e8f0;font:15px sans-serif;display:grid;place-items:center;height:100vh}
main{max-width:42rem;padding:2rem}h1{font-size:1.3rem}code{color:#93c5fd}</style></head>
<body><main><h1>Loading Product View</h1><p>Preparing scene <code>%1</code>…</p></main></body></html>)HTML").arg(safe_scene);
  embedded_web_view_->setHtml(html, QUrl(QStringLiteral("about:blank")));
#else
  Q_UNUSED(scene_id);
#endif
}

void ScenePreviewWidget::show_embedded_web_preparation_failure(
  const EmbeddedWebRequestIdentity & identity, const QString & detail)
{
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  if (!embedded_web_view_ || !embedded_web_identity_is_current(identity)) return;
  if (finish_post_save_product_view_refresh(identity, 0, false, detail)) {
    emit studio_log_requested(QStringLiteral(
      "Post-save Product View regeneration failed; retained the current browser edits. Correct the preparation error and use Save Layout again: %1").arg(detail));
    return;
  }
  native_compatibility_fallback_active_ = false;
  const QString concise_error = detail.trimmed().left(320).toHtmlEscaped();
  const QString scene = identity.scene_id.toHtmlEscaped();
  const QString retry_url = QStringLiteral("workcell-retry://%1").arg(identity.scene_id);
  const QString html = QStringLiteral(R"HTML(<!doctype html><html><head><meta charset="utf-8"><style>
body{margin:0;background:#0f172a;color:#e2e8f0;font:15px sans-serif;display:grid;place-items:center;height:100vh}
main{max-width:46rem;padding:2rem}h1{color:#fca5a5}code{color:#93c5fd}.error{background:#1e293b;padding:1rem;border-radius:.5rem}
a{display:inline-block;margin-top:1rem;padding:.65rem 1rem;background:#2563eb;color:white;text-decoration:none;border-radius:.4rem}</style></head>
<body><main><h1>Product View preparation failed</h1><p>Scene: <code>%1</code></p><p class="error">%2</p>
<p>Correct the scene-authoring blockers (required scene files, metadata, and referenced assets), then retry preparation.</p>
<a href="%3">Retry</a></main></body></html>)HTML").arg(scene, concise_error, retry_url.toHtmlEscaped());
  set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail);
  embedded_web_view_->setVisible(true);
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(false);
  if (embedded_fit_button_) {
    embedded_fit_button_->setText(QStringLiteral("Retry"));
    embedded_fit_button_->setToolTip(QStringLiteral("Retry preparation for scene %1.").arg(identity.scene_id));
  }
  embedded_web_view_->setHtml(html, QUrl(QStringLiteral("about:blank")));
  refresh_mode_and_state();
#else
  Q_UNUSED(identity); Q_UNUSED(detail);
#endif
}

void ScenePreviewWidget::invalidate_embedded_web_scene_handoff(const QString & scene_id)
{
  // Invalidate editor/inspector callbacks and committed-surface claims before
  // cancellation waits for an owned preparation process to terminate.
  ++embedded_editor_state_request_token_;
  embedded_editor_polling_ = false;
  embedded_web_has_committed_surface_ = false;
  cancel_embedded_web_lifecycle(false);
  ++embedded_web_readiness_token_;
  ++embedded_web_browser_load_token_;
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};
  embedded_web_last_error_.clear();
  native_compatibility_fallback_active_ = false;
  clear_embedded_editor_state_for_scene_handoff();
  set_embedded_product_view_state(EmbeddedProductViewState::Idle);
  show_embedded_web_loading_document(scene_id);
}

void ScenePreviewWidget::cancel_embedded_web_lifecycle(bool stop_owned_server)
{
  // Every callback captures an identity.  Retiring it first makes queued browser,
  // timer, and process callbacks harmless before any UI or process state changes.
  ++embedded_web_request_generation_;
  retire_embedded_web_navigation_for_handoff();
  embedded_web_has_active_identity_ = false;
  embedded_web_active_identity_ = EmbeddedWebRequestIdentity{};
  embedded_web_preparing_identity_ = EmbeddedWebRequestIdentity{};
  pending_embedded_web_identity_ = EmbeddedWebRequestIdentity{};
  pending_embedded_web_request_ = false;
  pending_embedded_web_force_ = false;
  pending_embedded_web_source_policy_ = EmbeddedWebSourcePolicy::AuthoringSession;
  embedded_web_last_suppressed_duplicate_key_.clear();
  embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::ServerNotStarted;
  embedded_web_server_probe_ = EmbeddedWebServerProbe{};

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
    embedded_web_server_session_repo_root_.clear();
    embedded_web_server_session_port_ = 0;
    process->deleteLater();
  }
}

QJsonObject ScenePreviewWidget::authoring_overlay_item(
  const PreviewItem & item, const QString & mesh_source)
{
  return QJsonObject{
    {QStringLiteral("id"), item.id}, {QStringLiteral("display_name"), item.display_name},
    {QStringLiteral("asset_id"), item.catalog_asset_id.trimmed().isEmpty() ? item.category : item.catalog_asset_id},
    {QStringLiteral("type"), item.category}, {QStringLiteral("category"), item.category},
    {QStringLiteral("role"), item.role}, {QStringLiteral("source_layer"), item.source_layer},
    {QStringLiteral("editable"), item.editable}, {QStringLiteral("locked"), item.locked},
    {QStringLiteral("selectable"), item.selectable}, {QStringLiteral("source_mesh_path"), mesh_source},
    {QStringLiteral("mesh_type"), item.mesh_type},
    {QStringLiteral("mesh_scale"), QJsonArray{item.mesh_scale_x, item.mesh_scale_y, item.mesh_scale_z}},
    {QStringLiteral("world_pose"), QJsonObject{
      {QStringLiteral("xyz"), QJsonArray{item.x, item.y, item.z}},
      {QStringLiteral("rpy"), QJsonArray{item.roll, item.pitch, item.yaw}}}},
    {QStringLiteral("render_owner"), QStringLiteral("editable_layout")},
    {QStringLiteral("render_policy"), QStringLiteral("primary")}
  };
}

ScenePreviewWidget::PreviewItem ScenePreviewWidget::preview_item_from_canvas_item(
  const workcell_builder::WorkcellStudioCanvasItem & item)
{
  PreviewItem preview;
  preview.id = QString::fromStdString(item.id);
  preview.display_name = QString::fromStdString(item.label);
  preview.category = QString::fromStdString(item.category.empty() ? item.type : item.category);
  preview.catalog_asset_id = QString::fromStdString(item.catalog_asset_id);
  preview.source_path = QString::fromStdString(item.source_file);
  preview.mesh_path = QString::fromStdString(item.mesh_path);
  preview.mesh_type = QString::fromStdString(item.mesh_type);
  preview.x = item.x; preview.y = item.y; preview.z = item.z;
  preview.roll = item.roll; preview.pitch = item.pitch; preview.yaw = item.yaw;
  preview.sx = item.width; preview.sy = item.depth; preview.sz = item.height;
  preview.mesh_scale_x = item.mesh_scale_x;
  preview.mesh_scale_y = item.mesh_scale_y;
  preview.mesh_scale_z = item.mesh_scale_z;
  return preview;
}

void ScenePreviewWidget::request_embedded_web_product_view_refresh(
  bool force, const QString & origin, EmbeddedWebSourcePolicy source_policy)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(force); Q_UNUSED(origin); Q_UNUSED(source_policy);
  return;
#else
  if (!embedded_web_view_) return;
  const QString request_origin = origin.trimmed().isEmpty() ? QStringLiteral("automatic") : origin.trimmed();
  const EmbeddedWebRequestIdentity request_key = embedded_web_request_identity(0);
  const bool context_ready = !request_key.scene_id.isEmpty() &&
    request_key.scene_id != QStringLiteral("No scene") &&
    !request_key.absolute_scene_dir.isEmpty() &&
    !request_key.absolute_repo_root.isEmpty() &&
    !request_key.generated_web_scene_path.isEmpty() &&
    request_key.payload_revision > 0 && !request_key.payload_fingerprint.isEmpty();
  if (!context_ready) {
    emit studio_log_requested(QStringLiteral("Product View lifecycle deferred: scene=%1 generation=%2 payload_revision=%3 origin=%4 scene_dir=%5 repo_root=%6.")
      .arg(request_key.scene_id.isEmpty() ? QStringLiteral("<unset>") : request_key.scene_id)
      .arg(embedded_web_request_generation_)
      .arg(request_key.payload_revision)
      .arg(request_origin)
      .arg(request_key.absolute_scene_dir.isEmpty() ? QStringLiteral("<unset>") : request_key.absolute_scene_dir)
      .arg(request_key.absolute_repo_root.isEmpty() ? QStringLiteral("<unset>") : request_key.absolute_repo_root));
    set_embedded_product_view_state(EmbeddedProductViewState::Idle);
    return;
  }
  ++embedded_web_effective_refresh_requests_received_;

  const bool duplicate_active = embedded_web_has_active_identity_ &&
    embedded_web_active_identity_.matches_effective_request(request_key);
  const bool duplicate_pending = pending_embedded_web_request_ &&
    pending_embedded_web_identity_.matches_effective_request(request_key);
  const bool duplicate_preparing = embedded_web_prepare_process_ &&
    embedded_web_preparing_identity_.matches_effective_request(request_key);
  const bool duplicate_prepared_or_loading =
    (embedded_product_view_state_ == EmbeddedProductViewState::Loading ||
     embedded_product_view_state_ == EmbeddedProductViewState::WaitingForBrowserReadiness ||
     embedded_product_view_state_ == EmbeddedProductViewState::Ready ||
     embedded_product_view_state_ == EmbeddedProductViewState::Failed) &&
    (embedded_web_loading_identity_.matches_effective_request(request_key) ||
     embedded_web_prepared_identity_.matches_effective_request(request_key));
  if (!force && (duplicate_active || duplicate_pending || duplicate_preparing || duplicate_prepared_or_loading)) {
    ++embedded_web_duplicate_requests_coalesced_;
    const QString suppression_key = embedded_web_effective_request_key(request_key);
    if (embedded_web_last_suppressed_duplicate_key_ != suppression_key) {
      embedded_web_last_suppressed_duplicate_key_ = suppression_key;
      emit studio_log_requested(QStringLiteral("Duplicate Product View navigation suppressed: scene=%1 payload_revision=%2 backend=%3 json=%4 fingerprint=%5 origin=%6.")
        .arg(request_key.scene_id)
        .arg(request_key.payload_revision)
        .arg(request_key.product_view_backend)
        .arg(request_key.generated_web_scene_path)
        .arg(QString::fromLatin1(request_key.payload_fingerprint.toHex().left(12)))
        .arg(request_origin));
    }
    return;
  }

  embedded_web_last_suppressed_duplicate_key_.clear();
  const EmbeddedWebRequestIdentity identity = embedded_web_request_identity(++embedded_web_request_generation_);

  // Compatibility assertion: if (force) cancel_embedded_web_lifecycle(false);
  if (force) {
    cancel_embedded_web_lifecycle(false);
    embedded_web_request_generation_ = identity.generation;
  } else if (embedded_web_has_active_identity_ &&
             !embedded_web_active_identity_.matches_effective_request(identity)) {
    retire_embedded_web_navigation_for_handoff();
  }

  if (!force && embedded_web_prepare_process_ && embedded_web_prepare_process_->state() != QProcess::NotRunning) {
    QProcess * const process = embedded_web_prepare_process_;
    const QString key = embedded_web_preparation_process_keys_.value(process);
    const auto diagnostic = embedded_web_preparation_diagnostics_.value(key);
    ++embedded_web_preparations_cancelled_superseded_;
    record_embedded_web_prepare_terminal(diagnostic.identity, process, QStringLiteral("cancelled_superseded"),
      process->exitStatus(), process->exitCode(), QStringLiteral("newer effective Product View request replaced this preparation"));
    disconnect(process, nullptr, this, nullptr);
    process->terminate();
    if (!process->waitForFinished(1000)) {
      process->kill();
      process->waitForFinished(1000);
    }
    embedded_web_preparation_process_keys_.remove(process);
    embedded_web_prepare_process_ = nullptr;
    embedded_web_preparing_identity_ = EmbeddedWebRequestIdentity{};
    process->deleteLater();
  }

  embedded_web_active_identity_ = identity;
  embedded_web_has_active_identity_ = true;
  pending_embedded_web_identity_ = identity;
  pending_embedded_web_request_ = true;
  pending_embedded_web_force_ = force;
  pending_embedded_web_source_policy_ = source_policy;
  emit studio_log_requested(QStringLiteral("Product View lifecycle requested: scene=%1 generation=%2 payload_revision=%3 origin=%4 force=%5.")
    .arg(identity.scene_id).arg(identity.generation).arg(identity.payload_revision).arg(request_origin)
    .arg(force ? QStringLiteral("true") : QStringLiteral("false")));
  maybe_start_next_embedded_web_prepare();
#endif
}

ScenePreviewWidget::EmbeddedWebRequestIdentity ScenePreviewWidget::embedded_web_request_identity(quint64 generation) const
{
  // Product View lifecycle identities intentionally have different lifetimes:
  // - effective scene request: scene_id, absolute_scene_dir, absolute_repo_root,
  //   product_view_backend, generated_web_scene_path, payload_fingerprint, and
  //   payload_revision. Duplicate setters with the same values coalesce.
  // - server session: absolute_repo_root plus selected_server_port (and the
  //   verified runtime marker checked by the HTTP probes). It can outlive one scene.
  // - preparation attempt: the effective request plus generation and QProcess*.
  // - browser navigation: a prepared request plus one navigation token and URL.
  // Generation counters, process pointers, transient state, and navigation tokens
  // are not part of effective-request equality.

  EmbeddedWebRequestIdentity identity;
  const PreviewContext normalized_context = normalized_preview_context(preview_context_);
  identity.scene_id = normalized_context.scene_id.isEmpty() ? preview_scene_name_.trimmed() : normalized_context.scene_id;
  const QString scene_dir = normalized_context.absolute_scene_dir;
  if (!scene_dir.isEmpty()) {
    const QFileInfo info(scene_dir);
    identity.absolute_scene_dir = QDir::cleanPath(info.exists() ?
      (info.canonicalFilePath().isEmpty() ? info.absoluteFilePath() : info.canonicalFilePath()) : info.absoluteFilePath());
  }
  // Resolve the repository before publishing the request identity.  Every
  // asynchronous callback therefore has the exact root it is allowed to use.
  identity.absolute_repo_root = resolve_embedded_web_repo_root(identity.absolute_scene_dir);
  // Each refresh starts by probing the configured loopback endpoint.  An
  // alternate port, when needed, replaces the active immutable identity.
  identity.selected_server_port = 8765;
  identity.product_view_backend = product_view_backend_ == ProductViewBackend::EmbeddedWeb3D ?
    QStringLiteral("embedded_web3d") : QStringLiteral("native_scene3d");
  identity.generated_web_scene_path = identity.scene_id.isEmpty() ?
    QString() : QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(identity.scene_id);
  identity.payload_fingerprint = preview_payload_fingerprint_;
  identity.payload_revision = static_cast<quint64>(preview_payload_revision_);
  identity.generation = generation;
  identity.request_owned_output_path = embedded_web_request_owned_output_path(
    identity.scene_id, identity.payload_revision, identity.generation, identity.payload_fingerprint);
  return identity;
}

bool ScenePreviewWidget::embedded_web_identity_is_current(const EmbeddedWebRequestIdentity & identity) const
{
  const QString selected_scene = normalized_preview_context(preview_context_).scene_id;
  return !selected_scene.isEmpty() && identity.scene_id == selected_scene &&
    embedded_web_has_active_identity_ && embedded_web_active_identity_ == identity;
}

QString ScenePreviewWidget::embedded_web_preparation_diagnostic_key(const EmbeddedWebRequestIdentity & identity) const
{
  // Include the complete immutable request identity, not merely the scene name:
  // a same-scene payload update must retain its own terminal diagnostic.
  return QStringLiteral("%1|%2|%3|%4|%5|%6|%7")
    .arg(identity.scene_id, identity.absolute_scene_dir, identity.absolute_repo_root,
         QString::fromLatin1(identity.payload_fingerprint.toHex()))
    .arg(identity.selected_server_port).arg(identity.payload_revision).arg(identity.generation);
}

QString ScenePreviewWidget::embedded_web_effective_request_key(const EmbeddedWebRequestIdentity & identity) const
{
  return QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8")
    .arg(identity.scene_id, identity.absolute_scene_dir, identity.absolute_repo_root,
         identity.product_view_backend, identity.request_owned_output_path,
         identity.generated_web_scene_path,
         QString::fromLatin1(identity.payload_fingerprint.toHex()))
    .arg(identity.payload_revision);
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
  request_embedded_web_product_view_refresh(false, QStringLiteral("automatic"));
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
  const EmbeddedWebSourcePolicy source_policy = pending_embedded_web_source_policy_;
  pending_embedded_web_request_ = false;
  pending_embedded_web_force_ = false;
  pending_embedded_web_source_policy_ = EmbeddedWebSourcePolicy::AuthoringSession;
  // A newer forced request can retire the pending identity while the previous
  // process is finishing.  Do not revive that stale work.
  if (!embedded_web_identity_is_current(identity)) {
    maybe_start_next_embedded_web_prepare();
    return;
  }
  start_embedded_web_prepare(identity, force, source_policy);
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

void ScenePreviewWidget::start_embedded_web_prepare(
  const EmbeddedWebRequestIdentity & identity, bool force,
  EmbeddedWebSourcePolicy source_policy, bool diagnostic_preview)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(force); Q_UNUSED(source_policy); Q_UNUSED(diagnostic_preview);
#else
  const QString scene_id = identity.scene_id;
  const QFileInfo selected_scene_info(identity.absolute_scene_dir);
  if (!is_safe_embedded_web_scene_id(scene_id)) {
    show_embedded_web_preparation_failure(identity, QStringLiteral("selected scene ID is missing or unsafe for Product View output and URL: %1")
      .arg(scene_id.isEmpty() ? QStringLiteral("<unset>") : scene_id));
    return;
  }
  if (identity.absolute_scene_dir.isEmpty() || !selected_scene_info.isAbsolute() ||
      !selected_scene_info.exists() || !selected_scene_info.isDir()) {
    show_embedded_web_preparation_failure(identity, QStringLiteral("selected scene directory is required and must exist: %1")
      .arg(identity.absolute_scene_dir.isEmpty() ? QStringLiteral("<unset>") : identity.absolute_scene_dir));
    return;
  }
  const QString canonical_scene_dir = selected_scene_info.canonicalFilePath();
  const QString selected_scene_dir = QDir::cleanPath(canonical_scene_dir.isEmpty() ? selected_scene_info.absoluteFilePath() : canonical_scene_dir);
  if (!scene_directory_matches_id(selected_scene_dir, scene_id)) {
    show_embedded_web_preparation_failure(identity, QStringLiteral("selected scene directory %1 does not match requested scene ID %2 by directory name or scene metadata")
      .arg(selected_scene_dir, scene_id));
    return;
  }
  const QString repo_root = identity.absolute_repo_root;
  if (repo_root.isEmpty()) {
    const QString detail = QStringLiteral("could not find a Workcell Studio repo root with viewer, scene-prep script, and scenes markers");
    show_embedded_web_preparation_failure(identity, detail);
    emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View unavailable: could not find a Workcell Studio repo root with required markers from selected scene, environment override, or fallback application paths."));
    return;
  }
  if (!embedded_web_identity_is_current(identity) || repo_root != identity.absolute_repo_root) return;
  // Keep a working compatibility viewport visible until Web3D has completed
  // its readiness handshake.  A retry must not briefly replace usable content.
  embedded_web_repo_root_ = repo_root;
  embedded_web_prepare_scene_ = scene_id;
  embedded_web_prepare_scene_dir_ = selected_scene_dir;
  embedded_web_prepare_output_path_ = identity.generated_web_scene_path;
  const QString request_output_path = identity.request_owned_output_path;
  if (request_output_path.isEmpty()) {
    show_embedded_web_preparation_failure(identity, QStringLiteral("could not construct immutable request-owned Product View output path"));
    return;
  }
  if (embedded_web_prepare_process_) embedded_web_prepare_process_->deleteLater();
  embedded_web_prepare_process_ = new QProcess(this);
  embedded_web_prepare_process_->setProgram(QStringLiteral("python3"));
  QStringList args{"scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", selected_scene_dir, "--output", request_output_path, "--stage-assets"};
  // Normal authoring requests use derived UI state so Web3D can show unsaved
  // edits. A post-save canonical request intentionally skips that state: disk
  // has become the sole transform authority for its forced regeneration.
  QJsonArray overlay_items;
  if (source_policy == EmbeddedWebSourcePolicy::AuthoringSession) {
    for (const PreviewItem & item : preview_items_) {
      if (item.source_layer != QStringLiteral("editable_layout") || !item.editable || item.locked) continue;
      QString mesh_source = item.mesh_path.trimmed().isEmpty() ? item.source_path.trimmed() : item.mesh_path.trimmed();

      // Some persisted repo assets reach PreviewItem as a stale absolute path
      // rooted under the scene directory. Recover the original portable
      // repo-relative path when that exact repo asset exists. Genuine scene-local
      // imports already exist at their absolute path and are left unchanged.
      const QFileInfo mesh_info(mesh_source);
      if (mesh_info.isAbsolute() && !mesh_info.isFile()) {
        const QString scene_relative = QDir(selected_scene_dir).relativeFilePath(mesh_source);
        if (!scene_relative.startsWith(QStringLiteral("../"))) {
          const QString repo_candidate = QDir(repo_root).filePath(scene_relative);
          if (QFileInfo(repo_candidate).isFile()) {
            mesh_source = scene_relative;
          }
        }
      }

      const QString mesh_suffix = QFileInfo(mesh_source).suffix().toLower();
      if (mesh_suffix != QStringLiteral("stl") &&
          mesh_suffix != QStringLiteral("dae") &&
          mesh_suffix != QStringLiteral("obj")) {
        continue;
      }
      overlay_items.append(authoring_overlay_item(item, mesh_source));
    }
  }
  if (!overlay_items.isEmpty()) {
    const QString overlay_dir = QDir(repo_root).filePath(QStringLiteral("build/workcell_studio_web_scene/authoring_session_overlays"));
    QDir().mkpath(overlay_dir);
    const QString overlay_name = QStringLiteral("%1-r%2-g%3-%4.json")
      .arg(scene_id).arg(identity.payload_revision).arg(identity.generation)
      .arg(QString::fromLatin1(identity.payload_fingerprint.toHex()));
    const QString overlay_path = QDir(overlay_dir).filePath(overlay_name);
    QSaveFile overlay_file(overlay_path);
    if (!overlay_file.open(QIODevice::WriteOnly)) {
      show_embedded_web_preparation_failure(identity, QStringLiteral("could not write temporary authoring-session overlay: %1").arg(overlay_path));
      return;
    }
    const QJsonObject overlay{
      {QStringLiteral("schema_version"), QStringLiteral("workcell_studio.authoring_session_overlay.v1")},
      {QStringLiteral("scene_id"), scene_id},
      {QStringLiteral("request_identity"), QJsonObject{
        {QStringLiteral("scene_id"), scene_id},
        {QStringLiteral("payload_fingerprint"), QString::fromLatin1(identity.payload_fingerprint.toHex())},
        {QStringLiteral("payload_revision"), static_cast<qint64>(identity.payload_revision)},
        {QStringLiteral("generation"), static_cast<qint64>(identity.generation)}}},
      {QStringLiteral("items"), overlay_items}
    };
    overlay_file.write(QJsonDocument(overlay).toJson(QJsonDocument::Compact));
    if (!overlay_file.commit()) {
      show_embedded_web_preparation_failure(identity, QStringLiteral("could not commit temporary authoring-session overlay: %1").arg(overlay_path));
      return;
    }
    args << QStringLiteral("--authoring-session-overlay") << overlay_path;
  }
  if (force) args << "--force";
  if (diagnostic_preview) args << "--allow-incomplete-preview";
  embedded_web_prepare_process_->setArguments(args);
  embedded_web_prepare_process_->setWorkingDirectory(repo_root);
  embedded_web_prepare_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  embedded_web_prepare_process_->setProcessChannelMode(QProcess::SeparateChannels);
  QProcess * const process = embedded_web_prepare_process_;
  const QString diagnostic_key = embedded_web_preparation_diagnostic_key(identity);
  EmbeddedWebPreparationDiagnostic diagnostic;
  diagnostic.identity = identity;
  diagnostic.expected_output_path = request_output_path;
  diagnostic.expected_output_absolute_path = QDir(repo_root).filePath(diagnostic.expected_output_path);
  diagnostic.diagnostic_preview = diagnostic_preview;
  embedded_web_preparation_diagnostics_.insert(diagnostic_key, diagnostic);
  embedded_web_preparation_process_keys_.insert(process, diagnostic_key);
  embedded_web_prepare_started_at_ = QDateTime::currentDateTimeUtc();
  embedded_web_preparing_identity_ = identity;
  ++embedded_web_preparation_request_count_;
  ++embedded_web_preparations_started_;
  connect(process, &QProcess::started, this, [this, identity, process]() {
    if (!embedded_web_identity_is_current(identity)) return;
    const QString key = embedded_web_preparation_process_keys_.value(process);
    if (auto diagnostic = embedded_web_preparation_diagnostics_.find(key); diagnostic != embedded_web_preparation_diagnostics_.end()) diagnostic->started = true;
  });
  connect(process, &QProcess::readyReadStandardOutput, this, [this, identity, process]() {
    if (!embedded_web_identity_is_current(identity)) return;
    append_embedded_web_prepare_output(process, false);
  });
  connect(process, &QProcess::readyReadStandardError, this, [this, identity, process]() {
    if (!embedded_web_identity_is_current(identity)) return;
    append_embedded_web_prepare_output(process, true);
  });
  connect(process, &QProcess::errorOccurred, this, [this, identity, process](QProcess::ProcessError error) {
    if (!embedded_web_identity_is_current(identity)) return;
    const QString key = embedded_web_preparation_process_keys_.value(process);
    const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(key);
    const QString detail = summarize_prepare_process_failure(
      process->program(), process->arguments(), process->workingDirectory(), process->exitStatus(), process->exitCode(),
      QStringLiteral("qprocess_error=%1 %2").arg(static_cast<int>(error)).arg(process->errorString()), diagnostic.stderr_tail);
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("process_error"), process->exitStatus(), process->exitCode(), detail);
  });
  QTimer::singleShot(120000, this, [this, identity, process]() {
    if (process != embedded_web_prepare_process_ || !embedded_web_identity_is_current(identity) ||
        process->state() == QProcess::NotRunning) return;
    append_embedded_web_prepare_output(process, false);
    append_embedded_web_prepare_output(process, true);
    const QString key = embedded_web_preparation_process_keys_.value(process);
    const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(key);
    const QString detail = summarize_prepare_process_failure(
      process->program(), process->arguments(), process->workingDirectory(), process->exitStatus(), process->exitCode(),
      QStringLiteral("timeout after 120000 ms"), diagnostic.stderr_tail);
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("timeout"), process->exitStatus(), process->exitCode(), detail);
    process->kill();
  });
  connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this, identity, process](int exit_code, QProcess::ExitStatus exit_status) {
    on_embedded_web_prepare_finished(identity, process, exit_code, exit_status);
  });
  set_embedded_product_view_state(EmbeddedProductViewState::Preparing, scene_id);
  emit studio_log_requested(QStringLiteral("Preparing embedded Product View: scene=%1 generation=%2 payload_revision=%3 origin=%4 repo_root=%5 command=%6").arg(identity.scene_id).arg(identity.generation).arg(identity.payload_revision)
    .arg(force ? QStringLiteral("user_retry") : QStringLiteral("automatic"), repo_root, embedded_web_prepare_command_for_log(selected_scene_dir, request_output_path, force)));
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
    const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(
      embedded_web_preparation_process_keys_.value(process));
    remove_embedded_web_request_output_if_safe(identity.absolute_repo_root, diagnostic.expected_output_path);
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
    const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(
      embedded_web_preparation_process_keys_.value(process));
    remove_embedded_web_request_output_if_safe(identity.absolute_repo_root, diagnostic.expected_output_path);
    process->deleteLater();
    embedded_web_preparation_process_keys_.remove(process);
    embedded_web_prepare_process_ = nullptr;
    embedded_web_preparing_identity_ = EmbeddedWebRequestIdentity{};
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
    const QString detail = summarize_prepare_process_failure(
      process->program(), process->arguments(), process->workingDirectory(), process->exitStatus(), process->exitCode(),
      QStringLiteral("process failed to start or reported a QProcess error"), existing_diagnostic.stderr_tail);
    show_embedded_web_preparation_failure(identity, QStringLiteral("Product View preparation process error: %1").arg(detail));
    maybe_start_next_embedded_web_prepare();
    return;
  }
  const QString scene = identity.scene_id;
  const EmbeddedWebPreparationDiagnostic diagnostic = embedded_web_preparation_diagnostics_.value(
    embedded_web_preparation_diagnostic_key(identity));
  const QString output_path = diagnostic.expected_output_path;
  const QString stdout_text = QString::fromUtf8(diagnostic.stdout_tail).trimmed();
  const QString command = embedded_web_prepare_command_for_log(embedded_web_prepare_scene_dir_, output_path, false);
  process->deleteLater();
  embedded_web_preparation_process_keys_.remove(process);
  embedded_web_prepare_process_ = nullptr;
  embedded_web_preparing_identity_ = EmbeddedWebRequestIdentity{};

  const QString absolute_output_path = diagnostic.expected_output_absolute_path;
  // Contract validation below supersedes mtime freshness; existence is checked when opening the expected output.
  auto reject_prepare = [&](const QString & reason) {
    const QString detail = QStringLiteral("%1; %2").arg(reason, summarize_prepare_process_failure(
      process->program(), process->arguments(), process->workingDirectory(), exit_status, exit_code, QString(), diagnostic.stderr_tail));
    emit studio_issue_requested(
      QStringLiteral("Embedded Product View scene preparation failed: %1").arg(detail),
      QStringLiteral("Error"), QStringLiteral("embedded_product_view_scene_preparation"));
    show_embedded_web_preparation_failure(identity, detail);
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("command_failure"), exit_status, exit_code,
      QStringLiteral("%1; command=%2").arg(detail, command));
    maybe_start_next_embedded_web_prepare();
  };

  if (exit_status != QProcess::NormalExit || exit_code != 0) {
    const bool destination_authoring_blocker =
      diagnostic.stderr_tail.contains("environment.yaml:") &&
      diagnostic.stderr_tail.contains("relationship") &&
      (diagnostic.stderr_tail.contains("unresolved ID") || diagnostic.stderr_tail.contains("ambiguous applicable destinations"));
    if (!diagnostic.diagnostic_preview && destination_authoring_blocker && embedded_web_identity_is_current(identity)) {
      record_embedded_web_prepare_terminal(identity, process, QStringLiteral("strict_authoring_blocked"), exit_status, exit_code,
        QStringLiteral("strict preparation rejected an unresolved destination chain; starting diagnostic preview"));
      emit studio_log_requested(QStringLiteral("Strict Product View preparation blocked by scene authoring for %1; retrying read-only diagnostic preview.").arg(scene));
      start_embedded_web_prepare(
        identity, true, EmbeddedWebSourcePolicy::AuthoringSession, true);
      return;
    }
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
  const QByteArray prepared_json = output_file.readAll();
  const QJsonDocument output_doc = QJsonDocument::fromJson(prepared_json, &output_parse_error);
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

  // Cancellation is only an optimization: the immutable identity gates both
  // entry to publication and the atomic replacement boundary.  A stale
  // request may delete only its private preparation file and must leave the
  // canonical browser handoff and all readiness/navigation state untouched.
  auto discard_stale_before_publication = [&](const QString & detail) {
    record_embedded_web_prepare_terminal(identity, process, QStringLiteral("stale_discarded"), exit_status, exit_code, detail);
    remove_embedded_web_request_output_if_safe(identity.absolute_repo_root, output_path);
    maybe_start_next_embedded_web_prepare();
  };
  if (!embedded_web_identity_is_current(identity)) {
    discard_stale_before_publication(QStringLiteral("request identity retired after validation and before publication"));
    return;
  }

  const QString canonical_output_path = QDir(identity.absolute_repo_root).filePath(identity.generated_web_scene_path);
  if (!QDir().mkpath(QFileInfo(canonical_output_path).absolutePath())) {
    reject_prepare(QStringLiteral("could not create canonical Product View output directory: %1")
      .arg(QFileInfo(canonical_output_path).absolutePath()));
    return;
  }
  QSaveFile published_file(canonical_output_path);
  published_file.setDirectWriteFallback(false);
  if (!published_file.open(QIODevice::WriteOnly) || published_file.write(prepared_json) != prepared_json.size()) {
    published_file.cancelWriting();
    reject_prepare(QStringLiteral("could not stage validated Product View output for atomic publication: %1")
      .arg(canonical_output_path));
    return;
  }
  if (!embedded_web_identity_is_current(identity)) {
    published_file.cancelWriting();
    discard_stale_before_publication(QStringLiteral("request identity retired at the atomic publication boundary"));
    return;
  }
  if (!published_file.commit()) {
    reject_prepare(QStringLiteral("could not atomically publish validated Product View output: %1")
      .arg(canonical_output_path));
    return;
  }
  ++embedded_web_canonical_publications_;

  const bool was_diagnostic_preview = property("diagnostic_preview_active").toBool();
  const bool diagnostic_preview = output.value(QStringLiteral("preview_mode")).toString() == QStringLiteral("diagnostic") &&
    output.value(QStringLiteral("authoring_status")).toString() == QStringLiteral("blocked");
  setProperty("diagnostic_preview_active", diagnostic_preview);
  refresh_product_view_robot_home_control();
  interaction_mode_selector_->setEnabled(!diagnostic_preview);
  const QJsonArray authoring_blockers = output.value(QStringLiteral("authoring_blockers")).toArray();
  const QString blocker_message = authoring_blockers.isEmpty() ? QString() :
    authoring_blockers.at(0).toObject().value(QStringLiteral("message")).toString();
  if (diagnostic_preview) {
    const QString explanation = QStringLiteral("Diagnostic preview — scene authoring incomplete\nScene: %1\n%2").arg(scene, blocker_message);
    toolbar_feedback_label_->setText(explanation);
    toolbar_feedback_label_->setToolTip(QStringLiteral("Save Layout and execution actions are disabled. Fix scene authoring blockers first: %1").arg(blocker_message));
    toolbar_feedback_row_->setVisible(true);
    emit studio_issue_requested(explanation, QStringLiteral("Warning"), QStringLiteral("diagnostic_product_view"));
  } else if (was_diagnostic_preview) {
    toolbar_feedback_row_->setVisible(false);
    toolbar_feedback_label_->clear();
    toolbar_feedback_label_->setToolTip(QString());
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
  const quint64 readiness_token = ++embedded_web_readiness_token_;
  embedded_web_readiness_deadline_ = QDateTime::currentDateTimeUtc().addSecs(45);
  embedded_web_last_boot_status_ = QStringLiteral("browser_loaded");
  poll_embedded_web_readiness(identity, navigation_token, readiness_token, expected_json_path, viewer_url);
#endif
}

void ScenePreviewWidget::poll_embedded_web_readiness(const EmbeddedWebRequestIdentity & identity, quint64 navigation_token,
  quint64 readiness_token, const QString & expected_json_path, const QString & viewer_url)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(identity); Q_UNUSED(navigation_token); Q_UNUSED(readiness_token); Q_UNUSED(expected_json_path); Q_UNUSED(viewer_url);
#else
  if (!embedded_web_view_) return;
  const QUrl expected_viewer_url(viewer_url);
  if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
      readiness_token != embedded_web_readiness_token_ ||
      embedded_web_view_->url() != expected_viewer_url) {
    emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness poll for scene %1 revision %2 navigation %3.")
      .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
    return;
  }

  if (QDateTime::currentDateTimeUtc() > embedded_web_readiness_deadline_) {
    const QString detail = QStringLiteral(
      "startup timed out after 45s for scene %1; viewer URL: %2; expected JSON: %3; pending_required_loads and last observed boot status: %4")
      .arg(identity.scene_id, viewer_url, expected_json_path, embedded_web_last_boot_status_.isEmpty() ? QStringLiteral("unavailable") : embedded_web_last_boot_status_);
    handle_embedded_web_runtime_failure(identity, navigation_token, detail);
    emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
    return;
  }

  static const char kStatusScript[] = R"JS(
(() => {
  const s = window.__WORKCELL_VIEWER_STATUS__ || {};
  return {
    readiness_contract_version: Number(s.readiness_contract_version ?? s.readinessContractVersion ?? 0),
    lifecycle_state: s.lifecycle_state || s.lifecycleState || '',
    terminal: Boolean(s.terminal),
    status_sequence: Number(s.status_sequence ?? s.statusSequence ?? 0),
    builder_revision: String(s.builder_revision ?? s.builderRevision ?? ''),
    viewer_boot_state: s.web3d_readiness_state || s.web3dReadinessState || s.viewer_boot_state || s.viewerBootState || '',
    scene_name: s.scene_name || s.sceneName || '',
    source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
    scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded),
    robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
    scene_id: s.scene_id || s.sceneId || '',
    expected_physical_item_count: Number(s.expected_physical_item_count ?? s.expectedPhysicalItemCount ?? 0),
    rendered_physical_item_count: Number(s.rendered_physical_item_count ?? s.renderedPhysicalItemCount ?? 0),
    failed_required_item_count: Number(s.failed_required_item_count ?? s.failedRequiredItemCount ?? s.required_mesh_failed_count ?? s.requiredMeshFailedCount ?? 0),
    robot_status: s.robot_status || s.robotStatus || '',
    tool_status: s.tool_status || s.toolStatus || s.end_effector_status || s.endEffectorStatus || '',
    environment_status: s.environment_status || s.environmentStatus || '',
    camera_status: s.camera_status || s.cameraStatus || '',
    pending_required_loads: Array.isArray(s.pending_required_loads || s.pendingRequiredLoads) ? (s.pending_required_loads || s.pendingRequiredLoads) : [],
    final_lifecycle_state: s.final_lifecycle_state || s.finalLifecycleState || s.lifecycle_state || s.lifecycleState || '',
    readiness_failure: s.readiness_failure || s.readinessFailure || null,
    failed_stage: s.failed_stage || s.failedStage || '',
    fatal_error: s.fatal_error || s.fatalError || '',
    fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
  };
})()
)JS";
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
    if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||
        readiness_token != embedded_web_readiness_token_ ||
        embedded_web_view_->url() != QUrl(viewer_url)) {
      emit studio_log_requested(QStringLiteral("Ignored stale Embedded Product View readiness result for scene %1 revision %2 navigation %3.")
        .arg(identity.scene_id).arg(identity.generation).arg(navigation_token));
      return;
    }
    const QVariantMap status = value.toMap();
    const int contract_version = status.value(QStringLiteral("readiness_contract_version")).toInt();
    const QString lifecycle_state = status.value(QStringLiteral("lifecycle_state")).toString();
    const bool terminal = status.value(QStringLiteral("terminal")).toBool();
    const QString builder_revision = status.value(QStringLiteral("builder_revision")).toString();
    const QString boot_state = status.value(QStringLiteral("viewer_boot_state")).toString();
    const QString source_json = status.value(QStringLiteral("source_web_scene_file")).toString();
    const bool scene_json_loaded = status.value(QStringLiteral("scene_json_loaded")).toBool();
    const QString robot_state = status.value(QStringLiteral("robot_preview_lifecycle_state")).toString();
    const QString reported_scene_id = status.value(QStringLiteral("scene_id")).toString();
    const int expected_physical_count = status.value(QStringLiteral("expected_physical_item_count")).toInt();
    const int rendered_physical_count = status.value(QStringLiteral("rendered_physical_item_count")).toInt();
    const int failed_required_count = status.value(QStringLiteral("failed_required_item_count")).toInt();
    const QString robot_status = status.value(QStringLiteral("robot_status")).toString();
    const QString tool_status = status.value(QStringLiteral("tool_status")).toString();
    const QString environment_status = status.value(QStringLiteral("environment_status")).toString();
    const QString camera_status = status.value(QStringLiteral("camera_status")).toString();
    const QStringList pending_required_loads = status.value(QStringLiteral("pending_required_loads")).toStringList();
    const QString final_lifecycle_state = status.value(QStringLiteral("final_lifecycle_state")).toString();
    const QString failed_stage = status.value(QStringLiteral("failed_stage")).toString();
    const QString fatal_error = status.value(QStringLiteral("fatal_error")).toString();
    const QString fatal_stack = status.value(QStringLiteral("fatal_stack")).toString();
    embedded_web_last_boot_status_ = QStringLiteral("contract=%13 terminal=%14 builder_revision=%15 boot=%1 json=%2 source=%3 scene_id=%4 expected_physical=%5 rendered_physical=%6 failed_required=%7 robot=%8 tool=%9 environment=%10 camera=%11 lifecycle=%12 pending_required_loads=%16")
      .arg(boot_state.isEmpty() ? QStringLiteral("unknown") : boot_state)
      .arg(scene_json_loaded ? QStringLiteral("loaded") : QStringLiteral("not_loaded"))
      .arg(source_json.isEmpty() ? QStringLiteral("unknown") : source_json)
      .arg(reported_scene_id.isEmpty() ? QStringLiteral("unknown") : reported_scene_id)
      .arg(expected_physical_count)
      .arg(rendered_physical_count)
      .arg(failed_required_count)
      .arg(robot_status.isEmpty() ? QStringLiteral("unknown") : robot_status)
      .arg(tool_status.isEmpty() ? QStringLiteral("unknown") : tool_status)
      .arg(environment_status.isEmpty() ? QStringLiteral("unknown") : environment_status)
      .arg(camera_status.isEmpty() ? QStringLiteral("unknown") : camera_status)
      .arg(final_lifecycle_state.isEmpty() ? QStringLiteral("unknown") : final_lifecycle_state)
      .arg(contract_version)
      .arg(terminal ? QStringLiteral("true") : QStringLiteral("false"))
      .arg(builder_revision.isEmpty() ? QStringLiteral("unknown") : builder_revision)
      .arg(QStringLiteral("[%1]").arg(pending_required_loads.join(QStringLiteral(", "))));

    scene_load_diagnostics_.set_debug_enabled(diagnostic_debug_logging_enabled());
    const SceneLoadDiagnosticContext::Identity diagnostic_identity{
      identity.scene_id,
      !builder_revision.isEmpty() ? QStringLiteral("builder_revision:%1").arg(builder_revision) : source_json,
      navigation_token};
    const QJsonObject readiness_content{
      {QStringLiteral("state"), boot_state == QStringLiteral("scene_failed") ? boot_state :
        (lifecycle_state.isEmpty() ? boot_state : lifecycle_state)},
      {QStringLiteral("terminal"), terminal},
      {QStringLiteral("failed_stage"), failed_stage},
      {QStringLiteral("fatal_error"), fatal_error},
      {QStringLiteral("failed_required_item_count"), failed_required_count},
      {QStringLiteral("rendered_physical_item_count"), rendered_physical_count},
      {QStringLiteral("pending_required_loads"), QJsonArray::fromStringList(pending_required_loads)}};
    const bool terminal_report = terminal || lifecycle_state == QStringLiteral("scene_ready") ||
      lifecycle_state == QStringLiteral("scene_failed") || boot_state == QStringLiteral("scene_failed");
    if (boot_state != QStringLiteral("scene_failed") && boot_state != QStringLiteral("failed")) {
      const auto readiness_report = scene_load_diagnostics_.observe(
        diagnostic_identity, QStringLiteral("terminal_readiness"), readiness_content, terminal_report);
      if (readiness_report.should_emit) emit studio_log_requested(readiness_report.summary + QStringLiteral(" content=%1")
        .arg(QString::fromUtf8(QJsonDocument(readiness_content).toJson(QJsonDocument::Compact))));
    }

    if (boot_state == QStringLiteral("scene_failed") || boot_state == QStringLiteral("failed")) {
      const QString detail = QStringLiteral("viewer JavaScript failed at %1: %2%3")
        .arg(failed_stage.isEmpty() ? QStringLiteral("unknown_stage") : failed_stage,
             fatal_error.isEmpty() ? QStringLiteral("unknown error") : fatal_error,
             fatal_stack.isEmpty() ? QString() : QStringLiteral("; stack: %1").arg(fatal_stack.left(500)));
      handle_embedded_web_runtime_failure(identity, navigation_token, detail);
      return;
    }

    const QString expected_builder_revision = QString::number(identity.payload_revision);
    QString contract_reason;
    const auto component_ready = [](const QString & value) { return value == QStringLiteral("ready") || value == QStringLiteral("missing"); };
    if (contract_version != 1) contract_reason = QStringLiteral("missing or unsupported readiness_contract_version");
    else if (lifecycle_state == QStringLiteral("server_ready") || boot_state == QStringLiteral("server_ready")) contract_reason = QStringLiteral("server_ready is infrastructure state, not scene readiness");
    else if (lifecycle_state != QStringLiteral("scene_ready")) contract_reason = QStringLiteral("lifecycle_state is %1").arg(lifecycle_state.isEmpty() ? QStringLiteral("missing") : lifecycle_state);
    else if (!terminal) contract_reason = QStringLiteral("terminal is false");
    else if (reported_scene_id != identity.scene_id) contract_reason = QStringLiteral("scene_id mismatch: reported %1 expected %2").arg(reported_scene_id.isEmpty() ? QStringLiteral("missing") : reported_scene_id, identity.scene_id);
    else if (source_json != expected_json_path) contract_reason = QStringLiteral("source_web_scene_file mismatch: reported %1 expected %2").arg(source_json.isEmpty() ? QStringLiteral("missing") : source_json, expected_json_path);
    else if (builder_revision != expected_builder_revision) contract_reason = QStringLiteral("builder_revision mismatch: reported %1 expected %2").arg(builder_revision.isEmpty() ? QStringLiteral("missing") : builder_revision, expected_builder_revision);
    else if (!scene_json_loaded) contract_reason = QStringLiteral("scene_json_loaded is false");
    else if (failed_required_count != 0) contract_reason = QStringLiteral("failed_required_item_count is %1").arg(failed_required_count);
    else if (!component_ready(robot_status) || !component_ready(tool_status) || !component_ready(environment_status) || !component_ready(camera_status)) contract_reason = QStringLiteral("component status incomplete: robot=%1 tool=%2 environment=%3 camera=%4").arg(robot_status, tool_status, environment_status, camera_status);

    if (contract_reason.isEmpty()) {
      native_compatibility_fallback_active_ = false;
      ++embedded_web_terminal_results_accepted_;
      embedded_web_has_committed_surface_ = true;
      set_embedded_product_view_state(EmbeddedProductViewState::Ready, QStringLiteral("viewer ready"));
      show_embedded_web_product_view();
      poll_embedded_editor_events();
      if (diagnostic_debug_logging_enabled()) emit studio_log_requested(
        QStringLiteral("Embedded Product View ready after terminal scene_ready: scene=%1 json=%2 builder_revision=%3 robot_preview_lifecycle_state=%4 failed_required_item_count=0")
          .arg(identity.scene_id, expected_json_path, expected_builder_revision, robot_state.isEmpty() ? QStringLiteral("not_required") : robot_state));
      finish_post_save_product_view_refresh(identity, navigation_token, true, QStringLiteral("matching terminal scene_ready accepted"));
      return;
    }

    set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness,
      QStringLiteral("waiting for viewer readiness: %1 (%2)").arg(contract_reason, embedded_web_last_boot_status_));
    QTimer::singleShot(750, this, [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url]() {
      if (!embedded_web_identity_is_current(identity) || readiness_token != embedded_web_readiness_token_) return;
      poll_embedded_web_readiness(identity, navigation_token, readiness_token, expected_json_path, viewer_url);
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
  if (embedded_web_server_lifecycle_ != EmbeddedWebServerLifecycle::ServerReady ||
      identity.absolute_repo_root.isEmpty() || identity.selected_server_port <= 0) return;
  const QString web_scene_url_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(identity.scene_id);
  const QString builder_revision = QString::number(identity.payload_revision);
  QUrl viewer_url;
  viewer_url.setScheme(QStringLiteral("http"));
  viewer_url.setHost(QStringLiteral("127.0.0.1"));
  viewer_url.setPort(identity.selected_server_port);
  viewer_url.setPath(QStringLiteral("/workcell_studio_web/viewer/index.html"));
  QUrlQuery viewer_query;
  viewer_query.addQueryItem(QStringLiteral("scene"), web_scene_url_path);
  viewer_query.addQueryItem(QStringLiteral("builderRevision"), builder_revision);
  viewer_query.addQueryItem(QStringLiteral("embedded"), QStringLiteral("1"));
  viewer_url.setQuery(viewer_query);

  const QUrlQuery decoded_viewer_query(viewer_url);
  const QString decoded_scene_path = decoded_viewer_query.queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded);
  const QString decoded_builder_revision = decoded_viewer_query.queryItemValue(
    QStringLiteral("builderRevision"), QUrl::FullyDecoded);
  const bool valid_scene_path = decoded_scene_path == web_scene_url_path;
  const bool valid_builder_revision = QRegularExpression(QStringLiteral("^[0-9]+$"))
    .match(decoded_builder_revision).hasMatch();
  if (!valid_scene_path || !valid_builder_revision) {
    const QString detail = QStringLiteral("Embedded Product View URL validation failed; using native compatibility preview.");
    emit studio_log_requested(detail);
    activate_native_compatibility_preview(detail);
    return;
  }
  if (embedded_web_prepared_identity_.matches_effective_request(identity) ||
      embedded_web_loading_identity_.matches_effective_request(identity)) {
    ++embedded_web_duplicate_requests_coalesced_;
    return;
  }
  embedded_web_prepared_identity_ = identity;
  embedded_web_readiness_deadline_ = QDateTime();
  embedded_web_last_boot_status_.clear();
  set_embedded_product_view_state(EmbeddedProductViewState::Loading);
  embedded_web_loading_identity_ = identity;
  embedded_web_loading_navigation_token_ = ++embedded_web_navigation_token_;
  embedded_web_loading_browser_load_token_ = ++embedded_web_browser_load_token_;
  const quint64 queued_navigation_token = embedded_web_loading_navigation_token_;
  embedded_web_server_lifecycle_ = EmbeddedWebServerLifecycle::BrowserLoading;
  embedded_web_expected_viewer_url_ = viewer_url;
  embedded_web_last_viewer_url_ = embedded_web_expected_viewer_url_.toString();
  QTimer::singleShot(0, this, [this, identity, queued_navigation_token, viewer_url]() {
    if (!embedded_web_view_) return;
    if (!embedded_web_identity_is_current(identity) ||
        queued_navigation_token != embedded_web_navigation_token_ ||
        embedded_web_loading_navigation_token_ != queued_navigation_token ||
        embedded_web_loading_identity_ != identity ||
        embedded_web_expected_viewer_url_ != viewer_url) {
      emit studio_log_requested(QStringLiteral("Embedded Product View ignored stale queued browser navigation for scene %1 revision %2 navigation %3.")
        .arg(identity.scene_id).arg(identity.payload_revision).arg(queued_navigation_token));
      return;
    }
    ++embedded_web_browser_navigations_started_;
    embedded_web_view_->load(viewer_url);
  });
#endif
}


#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
void ScenePreviewWidget::run_embedded_editor_command(const QString & script)
{
  if (!embedded_web_view_ || embedded_product_view_state_ != EmbeddedProductViewState::Ready) return;
  const EmbeddedWebRequestIdentity identity = embedded_web_active_identity_;
  const quint64 state_request_token = ++embedded_editor_state_request_token_;
  embedded_web_view_->page()->runJavaScript(script, [this, identity, state_request_token](const QVariant & value){
    if (!embedded_web_identity_is_current(identity) ||
        state_request_token != embedded_editor_state_request_token_) return;
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
  const QString mode = state.value(QStringLiteral("mode")).toString().trimmed().toLower();
  if ((mode == QStringLiteral("select") || mode == QStringLiteral("move") || mode == QStringLiteral("rotate")) &&
      gizmo_mode_selector_) {
    const QString label = mode.left(1).toUpper() + mode.mid(1);
    if (gizmo_mode_selector_->currentText() != label) {
      const QSignalBlocker blocker(gizmo_mode_selector_);
      gizmo_mode_selector_->setCurrentText(label);
    }
    if (interaction_mode_selector_ && interaction_mode_selector_->currentText() != label) {
      const QSignalBlocker blocker(interaction_mode_selector_);
      interaction_mode_selector_->setCurrentText(label);
    }
    emit authoring_mode_changed(mode);
  }

  QString selected_id = state.value(QStringLiteral("uiSelectionItemId")).toString().trimmed();
  if (selected_id.isEmpty()) {
    selected_id = state.value(QStringLiteral("selectedItemId")).toString().trimmed();
  }

  const QVariantMap selected_transform =
    state.value(QStringLiteral("selectedTransform")).toMap();
  const QVariantMap pose = selected_transform.value(QStringLiteral("pose")).toMap();
  const QVariantMap xyz = pose.value(QStringLiteral("xyz")).toMap();
  const QVariantMap rpy = pose.value(QStringLiteral("rpy")).toMap();

  bool x_ok = false, y_ok = false, z_ok = false;
  bool roll_ok = false, pitch_ok = false, yaw_ok = false;

  const double x = xyz.value(QStringLiteral("x")).toDouble(&x_ok);
  const double y = xyz.value(QStringLiteral("y")).toDouble(&y_ok);
  const double z = xyz.value(QStringLiteral("z")).toDouble(&z_ok);
  const double roll = rpy.value(QStringLiteral("x")).toDouble(&roll_ok);
  const double pitch = rpy.value(QStringLiteral("y")).toDouble(&pitch_ok);
  const double yaw = rpy.value(QStringLiteral("z")).toDouble(&yaw_ok);

  if (!selected_id.isEmpty() &&
      x_ok && y_ok && z_ok && roll_ok && pitch_ok && yaw_ok) {
    const QString signature =
      QStringLiteral("%1|%2|%3|%4|%5|%6|%7")
        .arg(selected_id)
        .arg(x, 0, 'g', 17)
        .arg(y, 0, 'g', 17)
        .arg(z, 0, 'g', 17)
        .arg(roll, 0, 'g', 17)
        .arg(pitch, 0, 'g', 17)
        .arg(yaw, 0, 'g', 17);

    if (property("embeddedSelectedTransformSignature").toString() != signature) {
      setProperty("embeddedSelectedTransformSignature", signature);
      emit preview_item_transform_changed(
        selected_id, x, y, z, roll, pitch, yaw);
    }
  } else {
    setProperty("embeddedSelectedTransformSignature", QString());
  }

  // Selection and dirty-state detail remains available through the embedded
  // editor and Studio Log. The header chip always reports runtime state.
  refresh_toolbar_status_chip();
}

void ScenePreviewWidget::poll_embedded_editor_events()
{
  if (!embedded_editor_polling_ || !embedded_web_view_ || embedded_product_view_state_ != EmbeddedProductViewState::Ready) return;
  const EmbeddedWebRequestIdentity identity = embedded_web_active_identity_;
  const quint64 state_request_token = ++embedded_editor_state_request_token_;
  static const char kPoll[] = "(() => { const api = window.__WORKCELL_EDITOR_API_V1__; if (!api) return {state:{},events:[]}; return {state:api.getState(),events:api.drainEvents()}; })()";
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kPoll), [this, identity, state_request_token](const QVariant & value){
    if (!embedded_web_identity_is_current(identity)) return;
    if (state_request_token != embedded_editor_state_request_token_) {
      if (embedded_editor_polling_) QTimer::singleShot(200, this, [this, identity]() {
        if (embedded_web_identity_is_current(identity)) poll_embedded_editor_events();
      });
      return;
    }
    const QVariantMap payload = value.toMap();
    const QVariantMap editor_state = payload.value(QStringLiteral("state")).toMap();
    apply_embedded_editor_state(editor_state);
    const QString browser_scene_id = editor_state.value(QStringLiteral("sceneId")).toString().trimmed();
    const QString browser_selected_id = editor_state.value(QStringLiteral("selectedItemId")).toString();
    QString browser_ui_selected_id = editor_state.value(QStringLiteral("uiSelectionItemId")).toString();
    if (browser_ui_selected_id.isEmpty()) browser_ui_selected_id = browser_selected_id;
    const QVariantMap selection_diagnostics = editor_state.value(QStringLiteral("selectionDiagnostics")).toMap();
    const bool scene_identity_matches = !browser_scene_id.isEmpty() && browser_scene_id == identity.scene_id;
    bool explicit_blank_selection_event = false;
    QString matching_item_type;
    for (const QVariant & event_value : payload.value(QStringLiteral("events")).toList()) {
      const QVariantMap event = event_value.toMap();
      const QString event_type = event.value(QStringLiteral("type")).toString();
      if (event_type == QStringLiteral("placement_requested")) {
        bool x_ok = false, y_ok = false, z_ok = false, yaw_ok = false;
        const double x = event.value(QStringLiteral("x")).toDouble(&x_ok);
        const double y = event.value(QStringLiteral("y")).toDouble(&y_ok);
        const double z = event.value(QStringLiteral("z")).toDouble(&z_ok);
        const double yaw = event.value(QStringLiteral("yaw")).toDouble(&yaw_ok);
        const bool repeat_commit = event.value(QStringLiteral("repeat")).toBool();
        if (x_ok && y_ok && z_ok && yaw_ok && std::isfinite(x) && std::isfinite(y) &&
            std::isfinite(z) && std::isfinite(yaw)) {
          emit embedded_asset_placement_requested(x, y, z, yaw, repeat_commit);
        } else {
          emit studio_log_requested(QStringLiteral(
            "Embedded Product View placement rejected: placement_requested requires finite x/y/z/yaw."));
        }
      } else if (event_type == QStringLiteral("selection_changed")) {
        QString id = event.value(QStringLiteral("uiItemId")).toString();
        if (id.isEmpty()) id = event.value(QStringLiteral("itemId")).toString();
        if (id.isEmpty() && (event.contains(QStringLiteral("uiItemId")) ||
                             event.contains(QStringLiteral("itemId")))) {
          explicit_blank_selection_event = true;
        }
        if (!id.isEmpty() && id == browser_ui_selected_id) {
          matching_item_type = event.value(QStringLiteral("itemType")).toString();
        }
      }
    }
    // scene_context_changed selections are invalidated by the identity/token
    // guards above. Helpers and derived overlays are valid inspection selections. Editing is
    // still governed by the payload's editable/locked contract; rejecting them
    // here made the embedded viewer and Qt hierarchy disagree about identity.
    // The rendered browser identity is deliberately not required to satisfy
    // preview_item_by_id(browser_selected_id) != nullptr; Qt owns authored/UI rows.
    if (matching_item_type.isEmpty()) {
      matching_item_type = editor_state.value(QStringLiteral("selectedItemType")).toString();
    }
    const bool object_present = selection_diagnostics.value(QStringLiteral("objectPresent")).toBool();
    const bool preview_item_found = preview_item_by_id(browser_ui_selected_id) != nullptr;
    const bool valid_browser_selection = scene_identity_matches && !browser_ui_selected_id.isEmpty() &&
      object_present && preview_item_found;
    if (!browser_selected_id.isEmpty() && !valid_browser_selection) {
      QString rejection_reason;
      if (!scene_identity_matches) rejection_reason = QStringLiteral("scene_identity_mismatch");
      else if (browser_ui_selected_id.isEmpty()) rejection_reason = QStringLiteral("empty_ui_identity");
      else if (!object_present) rejection_reason = QStringLiteral("browser_object_absent");
      else rejection_reason = QStringLiteral("preview_item_absent");
      const QString rejection_key = QStringLiteral("embedded_selection_rejected|%1|%2|%3|%4")
        .arg(identity.scene_id, browser_selected_id, browser_ui_selected_id, rejection_reason);
      if (!emitted_scene_diagnostic_keys_.contains(rejection_key)) {
        emitted_scene_diagnostic_keys_.insert(rejection_key);
        emit studio_log_requested(QStringLiteral(
          "Embedded Product View selection rejected: browser_exact_id=%1 browser_ui_id=%2 scene_id=%3 objectPresent=%4 preview_item_found=%5 reason=%6")
          .arg(browser_selected_id, browser_ui_selected_id, identity.scene_id,
               object_present ? QStringLiteral("true") : QStringLiteral("false"),
               preview_item_found ? QStringLiteral("true") : QStringLiteral("false"), rejection_reason));
      }
    }
    const bool browser_transitioned_to_empty = scene_identity_matches && browser_selected_id.isEmpty() &&
      !embedded_browser_selected_item_id_.isEmpty();
    if (scene_identity_matches) embedded_browser_selected_item_id_ = browser_ui_selected_id;
    if (valid_browser_selection && browser_ui_selected_id != selected_preview_item_id_) {
      selected_preview_item_id_ = browser_ui_selected_id;
      emit preview_item_selected(browser_ui_selected_id, matching_item_type);
    } else if (scene_identity_matches && browser_selected_id.isEmpty() &&
               (explicit_blank_selection_event || browser_transitioned_to_empty)) {
      selected_preview_item_id_.clear();
      if (auto * viewport = active_native_viewport()) {
        viewport->selected_id.clear();
        viewport->update();
      }
      if (simple_3d_view_) simple_3d_view_->update();
      emit preview_item_selected(QString(), QString());
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

void ScenePreviewWidget::arm_embedded_asset_placement(
  const QString & asset_id, const QString & mesh_uri, double mesh_scale, bool persistent)
{
  if (product_view_backend_ != ProductViewBackend::EmbeddedWeb3D) return;
  const QJsonObject options{
    {QStringLiteral("persistent"), persistent},
    {QStringLiteral("asset"), QJsonObject{
      {QStringLiteral("id"), asset_id},
      {QStringLiteral("display_name"), asset_id},
      {QStringLiteral("category"), QStringLiteral("authored_asset_object")},
      {QStringLiteral("source_kind"), QStringLiteral("user_authored")},
      {QStringLiteral("source_layer"), QStringLiteral("placement_preview")},
      {QStringLiteral("mesh_uri"), mesh_uri},
      {QStringLiteral("mesh_scale"), QJsonArray{mesh_scale, mesh_scale, mesh_scale}},
    }},
  };
  run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__.armPlacement(%1)")
    .arg(QString::fromUtf8(QJsonDocument(options).toJson(QJsonDocument::Compact))));
}

void ScenePreviewWidget::cancel_embedded_asset_placement()
{
  if (product_view_backend_ != ProductViewBackend::EmbeddedWeb3D) return;
  run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__.cancelPlacement()"));
}

ScenePreviewWidget::ProductViewBackend ScenePreviewWidget::active_product_view_backend() const
{
  return product_view_backend_;
}

bool ScenePreviewWidget::is_native_product_view_backend() const
{
  return product_view_backend_ == ProductViewBackend::NativeScene3D;
}

bool ScenePreviewWidget::embedded_web_authoring_active() const
{
  return product_view_backend_ == ProductViewBackend::EmbeddedWeb3D &&
    !native_compatibility_fallback_active_ && stack_ && stack_->currentWidget() == view3d_container_;
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
void ScenePreviewWidget::focus_embedded_product_view_for_authoring()
{
  if (!embedded_web_authoring_active() || !embedded_web_view_) return;
  embedded_web_view_->setFocus(Qt::OtherFocusReason);
}
#endif

void ScenePreviewWidget::set_authoring_mode(const QString & requested_mode)
{
  const QString mode = requested_mode.trimmed().toLower();
  if (mode != QStringLiteral("select") && mode != QStringLiteral("move") &&
      mode != QStringLiteral("rotate")) return;
  if (embedded_web_authoring_active()) {
    const QString script = QStringLiteral(
      "window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.setMode(%1)")
      .arg(QString(QJsonDocument(QJsonArray{mode}).toJson(QJsonDocument::Compact)).mid(1).chopped(1));
    run_embedded_editor_command(script);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
    if (mode == QStringLiteral("move") || mode == QStringLiteral("rotate"))
      focus_embedded_product_view_for_authoring();
#endif
    return;
  }
  auto * viewport = active_native_viewport();
  if (!viewport) return;
  if (mode == QStringLiteral("move")) viewport->gizmo_mode = Scene3DViewportWidget::GizmoMode::Move;
  else if (mode == QStringLiteral("rotate")) viewport->gizmo_mode = Scene3DViewportWidget::GizmoMode::Rotate;
  else viewport->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select;
  viewport->update();
}

void ScenePreviewWidget::undo_authoring_edit()
{
  if (embedded_web_authoring_active())
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.undo()"));
}

void ScenePreviewWidget::redo_authoring_edit()
{
  if (embedded_web_authoring_active())
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.redo()"));
}

void ScenePreviewWidget::set_authoring_item_pose(
  const QString & id,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  if (!embedded_web_authoring_active()) return;

  const QString encoded_id =
    QString::fromUtf8(
      QJsonDocument(QJsonArray{id})
        .toJson(QJsonDocument::Compact))
      .mid(1).chopped(1);

  const QString script = QStringLiteral(
    "window.__WORKCELL_EDITOR_API_V1__&&"
    "window.__WORKCELL_EDITOR_API_V1__.setItemPose("
    "%1,%2,%3,%4,%5,%6,%7)")
      .arg(encoded_id)
      .arg(x, 0, 'g', 17)
      .arg(y, 0, 'g', 17)
      .arg(z, 0, 'g', 17)
      .arg(roll, 0, 'g', 17)
      .arg(pitch, 0, 'g', 17)
      .arg(yaw, 0, 'g', 17);

  run_embedded_editor_command(script);
}

void ScenePreviewWidget::request_authoring_save()
{
  if (embedded_web_authoring_active()) emit embedded_authoring_save_requested();
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
  native_compatibility_fallback_active_ = false;
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(false);
  embedded_web_view_->setVisible(scene_selected_);
  if (mode_selector_) {
    const QSignalBlocker blocker(mode_selector_);
    mode_selector_->setItemText(0, QStringLiteral("Web3D Product View"));
    mode_selector_->setCurrentIndex(0);
  }
#endif
  refresh_mode_and_state();
}


void ScenePreviewWidget::set_preview_context(const PreviewContext & context)
{
  const PreviewContext normalized = normalized_preview_context(context);
  const bool context_changed = !preview_contexts_equal(preview_context_, normalized);
  if (!context_changed) return;

  const QString previous_scene_id = normalized_preview_context(preview_context_).scene_id;
  const bool scene_identity_changed = previous_scene_id != normalized.scene_id;
  const QString effective_scene_name = normalized.scene_id.isEmpty() ?
    QStringLiteral("No scene") : normalized.scene_id;
  const bool scene_name_changed = preview_scene_name_ != effective_scene_name;
  preview_context_ = normalized;
  root_resolution_summary_keys_.clear();

  // A scene identity handoff is destructive by design: no committed frame,
  // readiness result, browser completion, or editor callback from the previous
  // scene may remain visible. A same-scene context/payload refresh instead
  // follows the ordinary request lifecycle and preserves the committed surface
  // until its replacement proves ready.
  if (scene_identity_changed) {
    invalidate_embedded_web_scene_handoff(normalized.scene_id);
  }

  if (!normalized.scene_id.isEmpty()) {
    set_preview_scene_name(normalized.scene_id);
    if (!scene_name_changed) request_embedded_web_product_view_refresh(false, QStringLiteral("scene_context_ready"));
  } else {
    request_embedded_web_product_view_refresh(false, QStringLiteral("scene_context_incomplete"));
  }
}

ScenePreviewWidget::PreviewContext ScenePreviewWidget::preview_context() const
{
  return preview_context_;
}

void ScenePreviewWidget::activate_native_compatibility_preview(const QString & reason)
{
  embedded_web_last_error_ = reason;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  const bool web3d_selected = product_view_backend_ == ProductViewBackend::EmbeddedWeb3D && mode_selector_ &&
    mode_selector_->currentText() == QStringLiteral("Web3D Product View");
  if (web3d_selected) {
    native_compatibility_fallback_active_ = false;
    emit studio_log_requested(QStringLiteral("Native fallback prevented because Web3D is selected"));
    set_embedded_product_view_state(EmbeddedProductViewState::Failed, reason);
    if (embedded_fit_button_) {
      embedded_fit_button_->setText(QStringLiteral("Retry"));
      embedded_fit_button_->setToolTip(QStringLiteral("Retry loading the embedded Web3D Product View. Details remain available in Studio Log."));
    }
    if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(false);
    if (embedded_web_view_) embedded_web_view_->setVisible(scene_selected_);
    refresh_toolbar_status_chip();
    refresh_toolbar_feedback_row();
    refresh_mode_and_state();
    return;
  }
  native_compatibility_fallback_active_ = true;
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
  const bool web3d_selected = product_view_backend_ == ProductViewBackend::EmbeddedWeb3D && mode_selector_ &&
    mode_selector_->currentText() == QStringLiteral("Web3D Product View");
  if (web3d_selected) {
    if (embedded_product_view_state_ == EmbeddedProductViewState::Preparing ||
        embedded_product_view_state_ == EmbeddedProductViewState::StartingServer) {
      return QStringLiteral("Web3D Product View — preparing");
    }
    if (embedded_product_view_state_ == EmbeddedProductViewState::Loading ||
        embedded_product_view_state_ == EmbeddedProductViewState::WaitingForBrowserReadiness) {
      return QStringLiteral("Web3D Product View — loading");
    }
    if (embedded_product_view_state_ == EmbeddedProductViewState::Ready && !native_compatibility_fallback_active_) {
      return QStringLiteral("Web3D Product View — ready");
    }
    if (embedded_product_view_state_ == EmbeddedProductViewState::Failed) {
      return QStringLiteral("Web3D Product View — failed, Retry");
    }
  }
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
void ScenePreviewWidget::set_scene_selected(bool selected){
  scene_selected_ = selected;
  if (!selected && !selected_preview_item_id_.isEmpty()) {
    selected_preview_item_id_.clear();
    if (auto * viewport = active_native_viewport()) {
      viewport->selected_id.clear();
      viewport->update();
    }
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.clearSelection()"));
    emit preview_item_selected(QString(), QStringLiteral("scene_cleared"));
  }
  refresh_mode_and_state();
}
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
  scene_load_diagnostics_.set_debug_enabled(diagnostic_debug_logging_enabled());
  SceneLoadDiagnosticContext::Identity diagnostic_identity{
    preview_scene_name_, QStringLiteral("builder_revision:%1").arg(preview_payload_revision_),
    embedded_web_navigation_token_};
  int mesh_items = 0;
  int unresolved_packages = 0;
  int warning_count = 0;
  for (const PreviewItem & item : preview_items_) {
    if (item.mesh_available || !item.mesh_path.trimmed().isEmpty()) ++mesh_items;
    if (!item.package_uri.trimmed().isEmpty() && item.source_path_resolution_outcome.contains(QStringLiteral("unresolved"), Qt::CaseInsensitive)) ++unresolved_packages;
    warning_count += item.warnings.size() + (!item.mesh_load_warning.trimmed().isEmpty() ? 1 : 0);
  }
  const auto report = [this, &diagnostic_identity](const QString & type, const QJsonObject & content) {
    const auto result = scene_load_diagnostics_.observe(diagnostic_identity, type, content);
    if (result.should_emit) emit studio_log_requested(result.summary + QStringLiteral(" content=%1")
      .arg(QString::fromUtf8(QJsonDocument(content).toJson(QJsonDocument::Compact))));
  };
  report(QStringLiteral("asset_discovery"), QJsonObject{{QStringLiteral("items"), preview_items_.size()}});
  report(QStringLiteral("visual_mesh_index"), QJsonObject{{QStringLiteral("mesh_items"), mesh_items}, {QStringLiteral("warnings"), warning_count}});
  report(QStringLiteral("package_resolution"), QJsonObject{{QStringLiteral("unresolved_packages"), unresolved_packages}});
  auto * viewport = active_native_viewport();
  if (viewport) {
    viewport->set_scene_load_diagnostic_context(&scene_load_diagnostics_, diagnostic_identity);
    viewport->ingest_preview_items(preview_items_);
  }
  const bool has_selected = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & it){ return it.id == selected_preview_item_id_; });
  if (!selected_preview_item_id_.isEmpty() && !has_selected) {
    const QString missing_id = selected_preview_item_id_;
    selected_preview_item_id_.clear();
    if (viewport) viewport->selected_id.clear();
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.clearSelection()"));
    emit studio_log_requested(QString("Preview selection cleared after refresh (id missing): %1").arg(missing_id));
    emit preview_item_selected(QString(), QStringLiteral("selection_missing_after_refresh"));
  } else if (viewport) {
    viewport->selected_id = selected_preview_item_id_;
  }
  if (diagnostic_debug_logging_enabled() && !selected_preview_item_id_.isEmpty()) {
    emit studio_log_requested(QString("Preview selection restored after refresh: %1").arg(selected_preview_item_id_));
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
int ScenePreviewWidget::request_post_save_product_view_refresh()
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  return 0;
#else
  if (!embedded_web_view_ || normalized_preview_context(preview_context_).scene_id.isEmpty()) return 0;
  // A save changes authored inputs even when the in-memory PreviewItem list is
  // byte-identical. Renew every identity used by preparation/readiness so an
  // earlier load completion or scene_ready can never complete this refresh.
  ++preview_payload_revision_;
  ++preview_payload_generation_;
  request_embedded_web_product_view_refresh(
    true, QStringLiteral("post_save"), EmbeddedWebSourcePolicy::PersistedCanonical);
  post_save_refresh_generation_ = embedded_web_request_generation_;
  post_save_refresh_payload_revision_ = preview_payload_revision_;
  return post_save_refresh_payload_revision_;
#endif
}
bool ScenePreviewWidget::preview_payload_matches(const QVector<PreviewItem> & items) const
{
  return preview_payload_fingerprint(items) == preview_payload_fingerprint_;
}
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
void ScenePreviewWidget::select_preview_item(const QString & id){
  const QString stable_id = id.trimmed();
  if (!stable_id.isEmpty() && !preview_item_by_id(stable_id)) {
    if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("Preview selection ignored because id is absent from current payload: %1").arg(stable_id));
    return;
  }
  selected_preview_item_id_ = stable_id;
  if (auto * v = active_native_viewport()) {
    v->selected_id = stable_id;
    v->update();
  } else if (stable_id.isEmpty()) {
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.clearSelection()"));
  } else {
    run_embedded_editor_command(QStringLiteral("window.__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.selectItem(%1)").arg(QString::fromUtf8(QJsonDocument(QJsonArray{stable_id}).toJson(QJsonDocument::Compact)).mid(1).chopped(1)));
  }
  if (simple_3d_view_) simple_3d_view_->update();
}
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
  if (!v) { request_embedded_web_product_view_refresh(true, QStringLiteral("user_refresh")); emit studio_log_requested("Reloaded embedded Web 3D Product View."); return; }
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
  const bool web3d_selected = product_view_backend_ == ProductViewBackend::EmbeddedWeb3D &&
    mode == QStringLiteral("Web3D Product View");
  const bool requested_3d = (mode == "3D Layout Preview" || mode == "Web3D Product View");
  const bool use3d = requested_3d && preview3d_available_;

  if (web3d_selected && native_compatibility_fallback_active_) {
    native_compatibility_fallback_active_ = false;
    emit studio_log_requested(QStringLiteral("Native fallback prevented because Web3D is selected"));
  }
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
  const bool show_native_compatibility = use3d && scene_selected_ && !web3d_selected &&
    native_compatibility_fallback_active_ && compatibility_scene3d_viewport_;
  if (compatibility_scene3d_viewport_) compatibility_scene3d_viewport_->setVisible(show_native_compatibility);
  if (simple_3d_view_) simple_3d_view_->setVisible(use3d && scene_selected_ && !show_native_compatibility);
  if (web3d_selected && embedded_web_view_) {
    // Keep the WebEngine surface mounted for its neutral loading and failure
    // documents as well as committed Product View content. Web3D selection
    // must never uncover a native compatibility surface during preparation.
    embedded_web_view_->setVisible(use3d && scene_selected_);
  }
  if (error_state_label_) {
    const bool show_web3d_error = web3d_selected && use3d && scene_selected_ &&
      embedded_product_view_state_ == EmbeddedProductViewState::Failed;
    error_state_label_->setText(show_web3d_error ? QStringLiteral("Product View failed — Retry") :
      QStringLiteral("3D Layout Preview unavailable"));
    error_state_label_->setVisible(show_web3d_error);
  }
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
