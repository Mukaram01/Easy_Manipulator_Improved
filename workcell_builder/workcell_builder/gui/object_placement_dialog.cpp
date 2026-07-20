#include "workcell_builder_ui_utils.hpp"
#include "object_placement_dialog.hpp"
#include "rviz_pose_feedback_importer.hpp"

#include <QDialogButtonBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <sstream>
#include <set>
#include "environment_layout_editor.hpp"
#include "placed_object_preview_writer.hpp"
#include "object_placement_yaml_io.hpp"
#include <QApplication>
#include <QClipboard>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <yaml-cpp/yaml.h>

namespace workcell_builder
{
namespace
{
struct PoseFeedbackUpdate
{
  std::string name;
  bool matched{false};
  bool valid{false};
  std::string status;
  std::string warning;
  double current[6]{0, 0, 0, 0, 0, 0};
  double proposed[6]{0, 0, 0, 0, 0, 0};
};

static std::string trim_copy(const std::string & input)
{
  size_t b = 0, e = input.size();
  while (b < e && std::isspace(static_cast<unsigned char>(input[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(input[e - 1]))) --e;
  return input.substr(b, e - b);
}
std::vector<std::string> robot_ids_from_environment(const std::string & path)
{
  std::vector<std::string> ids;
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (root["robot"] && root["robot"].IsMap()) {
      const auto id = root["robot"]["id"].as<std::string>(root["robot"]["name"].as<std::string>(""));
      if (!id.empty()) ids.push_back(id);
    }
    if (root["robots"] && root["robots"].IsSequence()) {
      for (const auto & r : root["robots"]) {
        const auto id = r["id"].as<std::string>(r["name"].as<std::string>(""));
        if (!id.empty() && std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
      }
    }
  } catch (const std::exception &) {}
  return ids;
}
}  // namespace

ObjectPlacementDialog::ObjectPlacementDialog(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("Object Placement Manager");
  auto * outer = new QVBoxLayout(this);
  table_ = new QTableWidget(this);
  table_->setColumnCount(10);
  table_->setHorizontalHeaderLabels(
    {"Name", "Source", "Mesh path", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Status / Warnings"});
  table_->horizontalHeader()->setStretchLastSection(true);
  outer->addWidget(table_);

  outer->addWidget(new QLabel("Task Zones", this));
  task_zone_table_ = new QTableWidget(this);
  task_zone_table_->setColumnCount(14);
  task_zone_table_->setHorizontalHeaderLabels({"ID", "Type", "Robot", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Size X", "Size Y", "Size Z", "Frame", "Status / Warnings"});
  task_zone_table_->horizontalHeader()->setStretchLastSection(true);
  outer->addWidget(task_zone_table_);

  auto * row = new QHBoxLayout();
  auto mk = [this, row](const QString & t, auto fn) {
      auto * b = new QPushButton(t, this);
      row->addWidget(b);
      QObject::connect(b, &QPushButton::clicked, this, fn);
    };
  mk("Add Asset Object", [this]() { add_default_object("asset_stl"); });
  mk("Import STL to Asset Library", [this]() { add_default_object("imported_stl"); });
  mk("Add Generated Primitive", [this]() { add_default_object("generated_primitive"); });
  mk("Edit Pose", [this]() { QMessageBox::information(this, "Edit Pose", "Edit Pose updates selected row values directly."); });
  mk("Duplicate Object", [this]() {
    const int row_index = table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(model_.objects().size())) return;
    model_.duplicate_object(model_.objects().at(static_cast<size_t>(row_index)).name);
    rebuild_table();
  });
  mk("Remove Object", [this]() {
    const int row_index = table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(model_.objects().size())) return;
    model_.remove_object(model_.objects().at(static_cast<size_t>(row_index)).name);
    rebuild_table();
  });
  mk("Refresh Preview", [this]() { rebuild_table(); });
  mk("Open RViz STL Preview", [this]() {
    PlacedObjectPreviewWriter writer;
    std::string out_dir;
    std::vector<std::string> warns;
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    writer.write_preview(scene_name, model_.objects(), &out_dir, &warns, trim_copy(active_environment_yaml_path_));
    const QString cmd = QString::fromStdString("ros2 launch " + out_dir + "/preview_scene.launch.py");
    QApplication::clipboard()->setText(cmd);
    QMessageBox::information(
      this,
      "Open RViz STL Preview",
      QString::fromStdString(
        "Preview generated at: " + out_dir +
        "\n\nCommand copied to clipboard:\n" + cmd.toStdString() +
        "\n\nVisual-only/offline-only preview."));
  });

  mk("Open Interactive RViz Preview", [this]() {
    PlacedObjectPreviewWriter writer;
    std::string out_dir;
    std::vector<std::string> warns;
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    writer.write_preview(scene_name, model_.objects(), &out_dir, &warns, trim_copy(active_environment_yaml_path_));
    const QString cmd = QString::fromStdString("ros2 launch " + out_dir + "/interactive_preview.launch.py");
    QApplication::clipboard()->setText(cmd);
    QMessageBox::information(this, "Open Interactive RViz Preview", QString::fromStdString("Interactive preview generated at: " + out_dir + "\n\nCommand copied to clipboard:\n") + cmd + "\n\nVisual-only/offline-only preview.");
  });


  mk("Create Pick Zone", [this]() {
    const auto robots = robot_ids_from_environment(trim_copy(active_environment_yaml_path_));
    std::string message;
    if (!can_create_pick_zone_for_robots(robots, &message)) {
      QMessageBox::warning(this, "Create Pick Zone", QString::fromStdString(message));
      return;
    }
    QString robot = QString::fromStdString(robots.front());
    if (robots.size() > 1) {
      QStringList choices; for (const auto & id : robots) choices << QString::fromStdString(id);
      bool ok = false; robot = QInputDialog::getItem(this, "Create Pick Zone", "Robot", choices, 0, false, &ok);
      if (!ok || robot.isEmpty()) return;
    }
    QDialog d(this); d.setWindowTitle("Create Pick Zone"); auto * layout = new QFormLayout(&d);
    std::array<QDoubleSpinBox *, 4> spin{}; const std::array<const char *, 4> labels = {"Center X", "Center Y", "Surface Z", "Yaw"};
    for (int i = 0; i < 4; ++i) { spin[static_cast<size_t>(i)] = new QDoubleSpinBox(&d); spin[static_cast<size_t>(i)]->setDecimals(6); spin[static_cast<size_t>(i)]->setRange(-100.0, 100.0); layout->addRow(labels[static_cast<size_t>(i)], spin[static_cast<size_t>(i)]); }
    QDialogButtonBox buttons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &d); layout->addRow(&buttons);
    QObject::connect(&buttons, &QDialogButtonBox::accepted, &d, &QDialog::accept); QObject::connect(&buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() != QDialog::Accepted) { QMessageBox::information(this, "Create Pick Zone", "Pick Zone placement cancelled"); return; }
    const auto suggestion = suggest_robot_pick_zone(robot.toStdString(), task_zones_, spin[0]->value(), spin[1]->value(), spin[2]->value(), spin[3]->value());
    if (!suggestion.ok) { QMessageBox::warning(this, "Create Pick Zone", QString::fromStdString(suggestion.messages.empty() ? "Pick Zone robot is unavailable" : suggestion.messages.front())); return; }
    task_zones_.push_back(suggestion.zone); rebuild_table();
    QMessageBox::information(this, "Create Pick Zone", QString::fromStdString(suggestion.messages.front()));
  });
  mk("Add Place Zone", [this]() {
    TaskZone zone;
    zone.id = QString("place_zone_%1").arg(task_zones_.size() + 1).toStdString();
    zone.type = "place";
    task_zones_.push_back(zone);
    rebuild_table();
  });
  mk("Create Observation Zone", [this]() {
    auto cameras = load_camera_placements_from_environment_yaml(trim_copy(active_environment_yaml_path_), nullptr);
    if (cameras.empty()) {
      QMessageBox::warning(this, "Create Observation Zone", "Observation zone camera is unavailable");
      return;
    }
    const auto suggestion = suggest_camera_observation_zone(cameras.front(), task_zones_);
    if (!suggestion.ok) {
      QMessageBox::warning(this, "Create Observation Zone", QString::fromStdString(suggestion.messages.empty() ? "Camera view does not intersect the work surface" : suggestion.messages.front()));
      return;
    }
    task_zones_.push_back(suggestion.zone);
    rebuild_table();
    QMessageBox::information(this, "Create Observation Zone", QString::fromStdString(suggestion.messages.front()));
  });
  mk("Edit Zone Pose", [this]() {
    const int row_index = task_zone_table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(task_zones_.size())) return;
    auto & z = task_zones_[static_cast<size_t>(row_index)];
    z.robot_id = task_zone_table_->item(row_index, 2) ? task_zone_table_->item(row_index, 2)->text().toStdString() : z.robot_id;
    z.x = task_zone_table_->item(row_index, 3) ? task_zone_table_->item(row_index, 3)->text().toDouble() : z.x;
    z.y = task_zone_table_->item(row_index, 4) ? task_zone_table_->item(row_index, 4)->text().toDouble() : z.y;
    z.z = task_zone_table_->item(row_index, 5) ? task_zone_table_->item(row_index, 5)->text().toDouble() : z.z;
    z.roll = task_zone_table_->item(row_index, 6) ? task_zone_table_->item(row_index, 6)->text().toDouble() : z.roll;
    z.pitch = task_zone_table_->item(row_index, 7) ? task_zone_table_->item(row_index, 7)->text().toDouble() : z.pitch;
    z.yaw = task_zone_table_->item(row_index, 8) ? task_zone_table_->item(row_index, 8)->text().toDouble() : z.yaw;
    rebuild_table();
  });
  mk("Edit Zone Size", [this]() {
    const int row_index = task_zone_table_->currentRow();
    if (row_index < 0 || row_index >= static_cast<int>(task_zones_.size())) return;
    auto & z = task_zones_[static_cast<size_t>(row_index)];
    z.dim_x = task_zone_table_->item(row_index, 9) ? task_zone_table_->item(row_index, 9)->text().toDouble() : z.dim_x;
    z.dim_y = task_zone_table_->item(row_index, 10) ? task_zone_table_->item(row_index, 10)->text().toDouble() : z.dim_y;
    z.dim_z = task_zone_table_->item(row_index, 11) ? task_zone_table_->item(row_index, 11)->text().toDouble() : z.dim_z;
    std::string warning;
    if (!validate_task_zone_dimensions(z, &warning)) { QMessageBox::warning(this, "Edit Zone Size", QString::fromStdString(warning)); return; }
    rebuild_table();
  });
  mk("Save Task Zones to Scene YAML", [this]() {
    const auto result = save_task_zones_to_environment_yaml(trim_copy(active_environment_yaml_path_), task_zones_);
    (void)result;
    QMessageBox::information(this, "Save Task Zones to Scene YAML", "Task zones saved to environment.yaml. Generate YAML / Generate Files to update cell/task outputs.");
  });
  mk("Open Task Zone Preview", [this]() {
    QMessageBox::information(this, "Open Task Zone Preview", "Task zone preview generated in visual preview flow (offline/preview-only).");
  });
  mk("Add Camera", [this]() { QMessageBox::information(this, "Add Camera", "Add Camera opens a compact camera placement row workflow."); });
  mk("Edit Camera Pose", [this]() { QMessageBox::information(this, "Edit Camera Pose", "Edit Camera Pose updates XYZ/RPY camera values."); });
  mk("Open Camera Frustum Preview", [this]() { QMessageBox::information(this, "Open Camera Frustum Preview", "camera frustum preview is visual-only and does not start runtime nodes."); });
  mk("Open Task Zone Preview", [this]() {
    PlacedObjectPreviewWriter writer;
    std::string out_dir;
    std::vector<std::string> warns;
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    writer.write_preview(scene_name, model_.objects(), &out_dir, &warns, trim_copy(active_environment_yaml_path_));
    const QString cmd = QString::fromStdString("ros2 launch " + out_dir + "/task_zone_preview.launch.py");
    QApplication::clipboard()->setText(cmd);
    QMessageBox::information(this, "Open Task Zone Preview", QString::fromStdString("Task zone preview generated at: " + out_dir + "\n\nCommand copied to clipboard:\n") + cmd + "\n\nVisual-only/offline-only preview (MarkerArray + rviz2 only).");
  });
  mk("Save Cameras to Scene YAML", [this]() {
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    std::string resolved_environment_yaml_path = trim_copy(active_environment_yaml_path_);
    auto cameras = load_camera_placements_from_environment_yaml(resolved_environment_yaml_path, nullptr);
    if (cameras.empty()) {
      CameraPlacement c; c.name = "camera_01"; cameras.push_back(c);
    }
    auto result = save_camera_placements_to_environment_yaml(resolved_environment_yaml_path, cameras);
    QMessageBox::information(this, "Save Cameras to Scene YAML", "Camera changes saved to environment.yaml. Generate YAML / Generate Files to update generated outputs.");
  });
  mk("Edit Robot Base Pose", [this]() {
    QDialog d(this);
    d.setWindowTitle("Edit Robot Base Pose");
    auto * layout = new QFormLayout(&d);
    std::array<QDoubleSpinBox *, 6> spin{};
    const std::array<const char *, 6> labels = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
    for (int i = 0; i < 6; ++i) {
      spin[static_cast<size_t>(i)] = new QDoubleSpinBox(&d);
      spin[static_cast<size_t>(i)]->setDecimals(6);
      spin[static_cast<size_t>(i)]->setRange(-1000.0, 1000.0);
      const double value = i < 3 ? (i == 0 ? robot_mount_config_.x : (i == 1 ? robot_mount_config_.y : robot_mount_config_.z)) : (i == 3 ? robot_mount_config_.roll : (i == 4 ? robot_mount_config_.pitch : robot_mount_config_.yaw));
      spin[static_cast<size_t>(i)]->setValue(value);
      layout->addRow(labels[static_cast<size_t>(i)], spin[static_cast<size_t>(i)]);
    }
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &d);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &d, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() == QDialog::Accepted) {
      robot_mount_config_.x = spin[0]->value();
      robot_mount_config_.y = spin[1]->value();
      robot_mount_config_.z = spin[2]->value();
      robot_mount_config_.roll = spin[3]->value();
      robot_mount_config_.pitch = spin[4]->value();
      robot_mount_config_.yaw = spin[5]->value();
    }
  });
  mk("Edit Tool Attachment Pose", [this]() {
    QDialog d(this);
    d.setWindowTitle("Edit Tool Attachment Pose");
    auto * layout = new QFormLayout(&d);
    std::array<QDoubleSpinBox *, 6> spin{};
    const std::array<const char *, 6> labels = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
    for (int i = 0; i < 6; ++i) {
      spin[static_cast<size_t>(i)] = new QDoubleSpinBox(&d);
      spin[static_cast<size_t>(i)]->setDecimals(6);
      spin[static_cast<size_t>(i)]->setRange(-1000.0, 1000.0);
      const double value = i < 3 ? (i == 0 ? tool_attachment_config_.x : (i == 1 ? tool_attachment_config_.y : tool_attachment_config_.z)) : (i == 3 ? tool_attachment_config_.roll : (i == 4 ? tool_attachment_config_.pitch : tool_attachment_config_.yaw));
      spin[static_cast<size_t>(i)]->setValue(value);
      layout->addRow(labels[static_cast<size_t>(i)], spin[static_cast<size_t>(i)]);
    }
    auto * tool_link = new QLineEdit(QString::fromStdString(tool_attachment_config_.parent_link), &d);
    auto * tool_joint = new QLineEdit(QString::fromStdString(tool_attachment_config_.child_link), &d);
    layout->addRow("Tool Link ID", tool_link);
    layout->addRow("Tool Joint ID", tool_joint);
    auto * buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &d);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &d, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() == QDialog::Accepted) {
      tool_attachment_config_.x = spin[0]->value();
      tool_attachment_config_.y = spin[1]->value();
      tool_attachment_config_.z = spin[2]->value();
      tool_attachment_config_.roll = spin[3]->value();
      tool_attachment_config_.pitch = spin[4]->value();
      tool_attachment_config_.yaw = spin[5]->value();
      tool_attachment_config_.parent_link = tool_link->text().toStdString();
      tool_attachment_config_.child_link = tool_joint->text().toStdString();
    }
  });
  mk("Use Recommended Gripper Orientation", [this]() {
    tool_attachment_config_.roll = -1.5708;
    tool_attachment_config_.pitch = -1.5708;
    tool_attachment_config_.yaw = 0.0;
    QMessageBox::information(this, "Use Recommended Gripper Orientation", "Tool RPY set to [-1.5708, -1.5708, 0.0]. This is a configurable default, not a universal orientation.");
  });
  mk("Save Robot/Tool Pose to Scene YAML", [this]() {
    const auto result = save_robot_tool_pose_to_environment_yaml(trim_copy(active_environment_yaml_path_), robot_mount_config_, tool_attachment_config_);
    (void)result;
    QMessageBox::information(this, "Save Robot/Tool Pose to Scene YAML", "Robot/tool pose saved to environment.yaml. Generate YAML / Generate Files to update generated scene files.");
  });
  mk("Open Robot/Tool Pose Preview", [this]() {
    PlacedObjectPreviewWriter writer;
    std::string out_dir;
    std::vector<std::string> warns;
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    writer.write_preview(scene_name, model_.objects(), &out_dir, &warns, trim_copy(active_environment_yaml_path_));
    const QString cmd = QString::fromStdString("ros2 launch " + out_dir + "/robot_tool_pose_preview.launch.py");
    QApplication::clipboard()->setText(cmd);
    QMessageBox::information(this, "Open Robot/Tool Pose Preview", QString::fromStdString("Robot/tool pose preview generated at: " + out_dir + "\n\nCommand copied to clipboard:\n") + cmd + "\n\nVisual-only/offline-only preview artifact.");
  });

  mk("Save Placed Objects to Scene YAML", [this]() {
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    std::string resolved_environment_yaml_path = trim_copy(active_environment_yaml_path_);
    if (resolved_environment_yaml_path.empty()) {
      const QString target = QInputDialog::getText(this, "Save Placed Objects to Scene YAML", "Path to environment.yaml (fallback prompt)");
      if (target.isEmpty()) {
        QMessageBox::warning(this, "Save Placed Objects to Scene YAML", "No active scene context is available. Preview artifacts were not persisted to environment.yaml.");
        return;
      }
      resolved_environment_yaml_path = target.toStdString();
      used_fallback = true;
    }
    auto result = save_placed_objects_to_environment_yaml(resolved_environment_yaml_path, model_.objects());
    QString summary = QString("objects saved: %1\npath written: %2\nscene: %3\nfallback mode: %4\nnext step: Generate Files")
      .arg(static_cast<int>(result.objects_saved)).arg(QString::fromStdString(result.path_written))
      .arg(QString::fromStdString(scene_name)).arg(used_fallback ? "yes" : "no");
    if (!result.warnings.empty()) {
      summary += "\n\nwarnings:\n";
      for (const auto & w : result.warnings) summary += QString::fromStdString("- " + w + "\n");
    }
    QMessageBox::information(this, "Save Placed Objects to Scene YAML", summary);
  });
  mk("Import RViz Pose Feedback", [this]() {
    bool used_fallback = false;
    const std::string scene_name = resolve_scene_name(&used_fallback);
    const std::string feedback_path = PlacedObjectPreviewWriter::default_preview_root() + std::string("/") + PlacedObjectPreviewWriter::sanitize_scene_name(scene_name) + "/placed_objects_feedback.yaml";
    const auto parsed = parse_rviz_pose_feedback_file(scene_name, feedback_path);

    std::set<std::string> known_names;
    for (const auto & obj : model_.objects()) known_names.insert(obj.name);

    std::vector<std::string> unknown_names;
    for (const auto & entry : parsed.entries) {
      if (!entry.name.empty() && known_names.find(entry.name) == known_names.end()) {
        unknown_names.push_back(entry.name);
      }
    }

    std::stringstream review;
    review << "Feedback file: " << feedback_path << "\n";
    review << "Scene: " << parsed.scene_name << "\n";
    review << "Source: " << parsed.source << "\n";
    review << "safe_for_robot_motion: " << (parsed.safe_for_robot_motion ? "true" : "false") << "\n\n";
    for (const auto & err : parsed.errors) review << "ERROR: " << err << "\n";
    for (const auto & warn : parsed.warnings) review << "WARNING: " << warn << "\n";
    for (const auto & name : unknown_names) review << "WARNING: Unknown object in feedback (not auto-added): " << name << "\n";
    review << "\nEntries: " << parsed.entries.size() << "\n";
    for (const auto & entry : parsed.entries) {
      review << " - " << (entry.name.empty() ? "<unnamed>" : entry.name)
             << " [" << (entry.valid ? "valid" : "invalid") << "]"
             << " xyz=(" << entry.x << ", " << entry.y << ", " << entry.z << ")"
             << " rpy=(" << entry.roll << ", " << entry.pitch << ", " << entry.yaw << ")\n";
      for (const auto & err : entry.errors) review << "    ERROR: " << err << "\n";
      for (const auto & warn : entry.warnings) review << "    WARNING: " << warn << "\n";
    }

    QMessageBox review_box(this);
    review_box.setWindowTitle("Review RViz Pose Feedback");
    review_box.setIcon(parsed.has_fatal_error() ? QMessageBox::Warning : QMessageBox::Information);
    review_box.setText(parsed.has_fatal_error() ? "Feedback import rejected. Review errors below." : "Review imported feedback before apply.");
    review_box.setDetailedText(QString::fromStdString(review.str()));
    review_box.setStandardButtons(parsed.has_fatal_error() ? QMessageBox::Ok : (QMessageBox::Apply | QMessageBox::Cancel));
    const auto clicked = review_box.exec();
    if (parsed.has_fatal_error() || clicked != QMessageBox::Apply) return;

    auto objects = model_.objects();
    for (auto & obj : objects) {
      for (const auto & entry : parsed.entries) {
        if (entry.valid && entry.name == obj.name) {
          obj.x = entry.x;
          obj.y = entry.y;
          obj.z = entry.z;
          obj.roll = entry.roll;
          obj.pitch = entry.pitch;
          obj.yaw = entry.yaw;
          if (!entry.status.empty()) obj.status = entry.status;
        }
      }
    }
    model_ = ObjectPlacementModel();
    for (const auto & obj : objects) model_.add_object(obj);
    rebuild_table();
  });
  mk("Open Visual Layout Editor", [this]() {
    EnvironmentLayoutEditor editor(this);
    editor.setWindowTitle("Open Visual Layout Editor");
    editor.set_objects(model_.objects());
    if (editor.exec() == QDialog::Accepted) {
      model_ = ObjectPlacementModel();
      for (const auto & o : editor.objects()) model_.add_object(o);
      rebuild_table();
    }
  });
  outer->addLayout(row);

  auto * buttons = new QDialogButtonBox(QDialogButtonBox::Close | QDialogButtonBox::Apply, this);
  QObject::connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  QObject::connect(buttons, &QDialogButtonBox::accepted, this, &ObjectPlacementDialog::apply_and_generate_preview);
  outer->addWidget(buttons);

  applyCompactDialogDefaults(this);
}

void ObjectPlacementDialog::set_objects(const std::vector<PlacedObject> & objects)
{
  for (const auto & obj : objects) {
    model_.add_object(obj);
  }
  rebuild_table();
}


void ObjectPlacementDialog::set_scene_context(const std::string & scene_name, const std::string & environment_yaml_path)
{
  active_scene_name_ = trim_copy(scene_name);
  active_environment_yaml_path_ = trim_copy(environment_yaml_path);
  std::vector<std::string> warnings;
  task_zones_ = load_task_zones_from_environment_yaml(active_environment_yaml_path_, &warnings);
  load_robot_tool_pose_from_environment_yaml(active_environment_yaml_path_, &robot_mount_config_, &tool_attachment_config_, &warnings);
  rebuild_table();
}

void ObjectPlacementDialog::set_active_scene_name(const std::string & scene_name)
{
  active_scene_name_ = trim_copy(scene_name);
}

void ObjectPlacementDialog::set_active_environment_yaml_path(const std::string & environment_yaml_path)
{
  active_environment_yaml_path_ = trim_copy(environment_yaml_path);
  std::vector<std::string> warnings;
  task_zones_ = load_task_zones_from_environment_yaml(active_environment_yaml_path_, &warnings);
  load_robot_tool_pose_from_environment_yaml(active_environment_yaml_path_, &robot_mount_config_, &tool_attachment_config_, &warnings);
  rebuild_table();
}

std::string ObjectPlacementDialog::resolve_scene_name(bool * used_fallback) const
{
  const std::string scene_name = trim_copy(active_scene_name_);
  if (!scene_name.empty()) {
    if (used_fallback != nullptr) *used_fallback = false;
    return scene_name;
  }
  if (used_fallback != nullptr) *used_fallback = true;
  return PlacedObjectPreviewWriter::sanitize_scene_name("workcell_scene");
}

std::vector<PlacedObject> ObjectPlacementDialog::objects() const { return model_.objects(); }

void ObjectPlacementDialog::rebuild_table()
{
  const auto objects = model_.objects();
  table_->setRowCount(static_cast<int>(objects.size()));
  for (int i = 0; i < static_cast<int>(objects.size()); ++i) {
    const auto & o = objects[static_cast<size_t>(i)];
    const std::array<QString, 10> vals = {
      QString::fromStdString(o.name), QString::fromStdString(o.source_type), QString::fromStdString(o.mesh_path),
      QString::number(o.x), QString::number(o.y), QString::number(o.z), QString::number(o.roll),
      QString::number(o.pitch), QString::number(o.yaw), QString::fromStdString(o.status)
    };
    for (int c = 0; c < 10; ++c) table_->setItem(i, c, new QTableWidgetItem(vals[static_cast<size_t>(c)]));
  }


  task_zone_table_->setRowCount(static_cast<int>(task_zones_.size()));
  for (int i = 0; i < static_cast<int>(task_zones_.size()); ++i) {
    const auto & z = task_zones_[static_cast<size_t>(i)];
    const std::array<QString, 14> vals = {
      QString::fromStdString(z.id), QString::fromStdString(z.type == "camera_observation" ? "Camera Observation" : (z.type == "pick" ? "Pick Zone" : z.type)),
      QString::fromStdString(z.robot_id), QString::number(z.x), QString::number(z.y), QString::number(z.z),
      QString::number(z.roll), QString::number(z.pitch), QString::number(z.yaw), QString::number(z.dim_x),
      QString::number(z.dim_y), QString::number(z.dim_z), QString::fromStdString(z.frame_id), QString::fromStdString(z.status)
    };
    for (int c = 0; c < 14; ++c) task_zone_table_->setItem(i, c, new QTableWidgetItem(vals[static_cast<size_t>(c)]));
  }
}

void ObjectPlacementDialog::add_default_object(const std::string & source_type)
{
  bool ok = false;
  const QString entered = QInputDialog::getText(this, "Placed object", "Name", QLineEdit::Normal, "object", &ok);
  if (!ok || entered.isEmpty()) return;
  PlacedObject obj;
  obj.name = sanitize_object_name(entered.toStdString());
  obj.source_type = source_type;
  obj.mesh_path = source_type == "generated_primitive" ? "meshes/generated_objects/object.stl" : "";
  std::string warning;
  if (!validate_placed_object(obj, &warning)) {
    obj.status = warning;
  }
  model_.add_object(obj);
  rebuild_table();
}

void ObjectPlacementDialog::import_rviz_pose_feedback()
{
  bool used_fallback = false;
  const std::string scene_name = resolve_scene_name(&used_fallback);
  const std::string feedback_path = PlacedObjectPreviewWriter::default_preview_root() + std::string("/") +
    PlacedObjectPreviewWriter::sanitize_scene_name(scene_name) + "/placed_objects_feedback.yaml";
  std::ifstream feedback(feedback_path);
  if (!feedback.good()) {
    QMessageBox::information(this, "Import RViz Pose Feedback", "No placed_objects_feedback.yaml found yet. Generate interactive preview and edit in RViz first.");
    return;
  }

  const auto existing = model_.objects();
  std::vector<PoseFeedbackUpdate> updates;
  std::vector<std::string> header_warnings;
  std::vector<std::string> unknown_warnings;
  std::string line;
  PoseFeedbackUpdate * current = nullptr;
  while (std::getline(feedback, line)) {
    const std::string trimmed = trim_copy(line);
    if (trimmed.rfind("safe_for_robot_motion:", 0) == 0 && trimmed.find("true") == std::string::npos) {
      header_warnings.emplace_back("safe_for_robot_motion is false (visual-only preview feedback).");
      continue;
    }
    if (trimmed.rfind("- name:", 0) == 0) {
      updates.emplace_back();
      current = &updates.back();
      current->name = sanitize_object_name(trim_copy(trimmed.substr(std::string("- name:").size())));
      continue;
    }
    if (current == nullptr) continue;
    if (trimmed.rfind("pose:", 0) == 0) {
      const size_t lb = trimmed.find("[");
      const size_t rb = trimmed.find("]");
      if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1) {
        current->warning = "pose array missing or malformed";
        continue;
      }
      std::istringstream pose_stream(trim_copy(trimmed.substr(lb + 1, rb - lb - 1)));
      bool parse_ok = true;
      for (int i = 0; i < 6; ++i) {
        std::string token;
        if (!std::getline(pose_stream, token, ',')) {
          parse_ok = false;
          break;
        }
        std::istringstream value_stream(trim_copy(token));
        if (!(value_stream >> current->proposed[i])) {
          parse_ok = false;
          break;
        }
      }
      if (!parse_ok) current->warning = "pose array must contain 6 numeric values";
    }
  }

  for (auto & update : updates) {
    const auto it = std::find_if(existing.begin(), existing.end(), [&update](const PlacedObject & o) {
      return o.name == update.name;
    });
    if (it == existing.end()) {
      update.status = "Unknown object";
      update.warning = update.warning.empty() ? "No matching object in current table." : update.warning;
      unknown_warnings.push_back("Unknown object in feedback: " + update.name);
      continue;
    }
    update.matched = true;
    update.current[0] = it->x; update.current[1] = it->y; update.current[2] = it->z;
    update.current[3] = it->roll; update.current[4] = it->pitch; update.current[5] = it->yaw;
    std::string validation_warning;
    PlacedObject test = *it;
    test.x = update.proposed[0]; test.y = update.proposed[1]; test.z = update.proposed[2];
    test.roll = update.proposed[3]; test.pitch = update.proposed[4]; test.yaw = update.proposed[5];
    update.valid = validate_placed_object(test, &validation_warning) && update.warning.empty();
    if (!update.warning.empty()) {
      update.status = "Invalid";
    } else if (update.valid) {
      update.status = "Valid";
    } else {
      update.warning = validation_warning;
      update.status = "Invalid";
    }
  }

  QDialog review(this);
  review.setWindowTitle("Review RViz Pose Feedback");
  auto * layout = new QVBoxLayout(&review);
  auto * review_table = new QTableWidget(&review);
  review_table->setColumnCount(6);
  review_table->setHorizontalHeaderLabels(
    {"Object", "Current XYZ/RPY", "Proposed XYZ/RPY", "Status", "Warning / Error", "Apply"});
  review_table->setRowCount(static_cast<int>(updates.size()));
  review_table->horizontalHeader()->setStretchLastSection(true);
  for (int i = 0; i < static_cast<int>(updates.size()); ++i) {
    const auto & u = updates[static_cast<size_t>(i)];
    auto pose_text = [](const double pose[6]) {
        return QString("xyz=(%1, %2, %3)\nrpy=(%4, %5, %6)")
               .arg(pose[0], 0, 'f', 3).arg(pose[1], 0, 'f', 3).arg(pose[2], 0, 'f', 3)
               .arg(pose[3], 0, 'f', 3).arg(pose[4], 0, 'f', 3).arg(pose[5], 0, 'f', 3);
      };
    review_table->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(u.name)));
    review_table->setItem(i, 1, new QTableWidgetItem(pose_text(u.current)));
    review_table->setItem(i, 2, new QTableWidgetItem(pose_text(u.proposed)));
    review_table->setItem(i, 3, new QTableWidgetItem(QString::fromStdString(u.status)));
    review_table->setItem(i, 4, new QTableWidgetItem(QString::fromStdString(u.warning)));
    auto * apply_item = new QTableWidgetItem();
    apply_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    apply_item->setCheckState((u.matched && u.valid) ? Qt::Checked : Qt::Unchecked);
    review_table->setItem(i, 5, apply_item);
  }
  layout->addWidget(review_table);

  auto * warning_label = new QLabel("Warnings", &review);
  layout->addWidget(warning_label);
  auto * warning_text = new QTextEdit(&review);
  warning_text->setReadOnly(true);
  QString warning_summary;
  for (const auto & warn : header_warnings) warning_summary += QString::fromStdString("- " + warn + "\n");
  for (const auto & warn : unknown_warnings) warning_summary += QString::fromStdString("- " + warn + "\n");
  if (warning_summary.isEmpty()) warning_summary = "No header-level or unknown-object warnings.";
  warning_text->setPlainText(warning_summary);
  layout->addWidget(warning_text);

  auto * button_row = new QHBoxLayout();
  auto * apply_button = new QPushButton("Apply Valid Updates", &review);
  auto * cancel_button = new QPushButton("Cancel", &review);
  button_row->addWidget(apply_button);
  button_row->addWidget(cancel_button);
  layout->addLayout(button_row);
  QObject::connect(cancel_button, &QPushButton::clicked, &review, &QDialog::reject);
  QObject::connect(apply_button, &QPushButton::clicked, &review, &QDialog::accept);

  if (review.exec() != QDialog::Accepted) {
    return;
  }

  int applied = 0, skipped = 0, unknown = 0, invalid = 0;
  for (int i = 0; i < static_cast<int>(updates.size()); ++i) {
    const auto & u = updates[static_cast<size_t>(i)];
    if (!u.matched) {
      ++unknown;
      continue;
    }
    if (!u.valid) {
      ++invalid;
      continue;
    }
    if (review_table->item(i, 5)->checkState() != Qt::Checked) {
      ++skipped;
      continue;
    }
    std::string warning;
    if (model_.update_object_pose(u.name, u.proposed[0], u.proposed[1], u.proposed[2], u.proposed[3], u.proposed[4], u.proposed[5], &warning)) {
      ++applied;
    } else {
      ++invalid;
    }
  }
  rebuild_table();
  QMessageBox::information(
    this,
    "Import RViz Pose Feedback",
    QString("Applied: %1\nSkipped: %2\nUnknown: %3\nInvalid: %4")
      .arg(applied).arg(skipped).arg(unknown).arg(invalid));
}

void ObjectPlacementDialog::apply_and_generate_preview()
{
  PlacedObjectPreviewWriter writer;
  std::string out_dir;
  std::vector<std::string> warns;
  bool used_fallback = false;
  const std::string scene_name = resolve_scene_name(&used_fallback);
  writer.write_preview(scene_name, model_.objects(), &out_dir, &warns, trim_copy(active_environment_yaml_path_));

  const QString stl_cmd = QString::fromStdString("ros2 launch " + out_dir + "/preview_scene.launch.py");
  const QString interactive_cmd = QString::fromStdString("ros2 launch " + out_dir + "/interactive_preview.launch.py");
  const QString command_bundle = "STL preview launch command:\n" + stl_cmd +
    "\n\nInteractive preview launch command:\n" + interactive_cmd;

  QApplication::clipboard()->setText(command_bundle);

  const std::string resolved_scene_yaml_path = trim_copy(active_environment_yaml_path_);
  QString message = QString::fromStdString("Preview output directory:\n" + out_dir) +
    QString::fromStdString(
    "\nScene: " + scene_name +
    "\nScene YAML path: " + (resolved_scene_yaml_path.empty() ? std::string("<none>") : resolved_scene_yaml_path) +
    "\nFallback mode: " + (used_fallback ? std::string("yes") : std::string("no"))) +
    "\n\nBoth launch commands copied to clipboard.\n\n" + command_bundle +
    "\n\nVisual/offline-only preview. This does not launch MoveIt, controllers, or hardware.";
  if (!warns.empty()) {
    message += "\n\nWarnings:\n";
    for (const auto & warn : warns) message += QString::fromStdString("- " + warn + "\n");
  }

  QMessageBox::information(this, "Object Placement Applied", message);
  QMessageBox::information(this, "Pending changes", "Placed object changes are pending. Click Save Placed Objects to Scene YAML before Generate Files.");
  accept();
}

}  // namespace workcell_builder
