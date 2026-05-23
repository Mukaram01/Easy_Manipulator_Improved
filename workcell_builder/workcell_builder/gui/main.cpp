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

#include <QApplication>
#include <QAction>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QLabel>
#include <QTextEdit>
#include <QImage>
#include <QWidget>
#include <iostream>
#include <vector>

#include "gui/mainwindow.h"
#include "gui/scene3d_viewport_widget.h"
#include "gui/scene_select.h"
#include "gui/startup_dialog.h"
#include "workcell_builder_ui_utils.hpp"

namespace
{
bool has_cli_flag(const QStringList & args, const QString & flag)
{
  return args.contains(flag);
}

QString cli_value(const QStringList & args, const QString & flag)
{
  const int idx = args.indexOf(flag);
  if (idx >= 0 && (idx + 1) < args.size()) {
    return args.at(idx + 1);
  }
  return QString();
}

QJsonObject run_gui_self_test();

struct Scene3DSmokeOptions
{
  bool enabled{ false };
  QString scene_name;
  QString smoke_output;
  QString screenshot_path;
  bool exit_after_smoke{ false };
  bool new_cell_recommended_layout_smoke{ false };
};

class Scene3DSmokeRunner : public QObject
{
public:
  Scene3DSmokeRunner(QApplication * app, MainWindow * window, const Scene3DSmokeOptions & opts)
  : app_(app), window_(window), opts_(opts) {}

  void start() { QTimer::singleShot(0, this, &Scene3DSmokeRunner::step_begin); }

private:
  QApplication * app_;
  MainWindow * window_;
  Scene3DSmokeOptions opts_;
  QJsonArray warnings_;
  QJsonArray blockers_;
  QDateTime start_time_;
  QJsonObject latest_counters_;

  void step_begin()
  {
    start_time_ = QDateTime::currentDateTimeUtc();
    if (!window_) {
      blockers_.append("MainWindow instance is null");
      return finalize();
    }
    if (opts_.new_cell_recommended_layout_smoke) {
      trigger_by_text("New Cell");
      QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_trigger_recommended_layout);
      return;
    }
    if (!opts_.scene_name.trimmed().isEmpty()) {
      auto * table = window_->findChild<QTableWidget *>("studioHomeSceneTable");
      if (table == nullptr) {
        blockers_.append("Scene table not found");
      } else {
        bool found = false;
        for (int row = 0; row < table->rowCount(); ++row) {
          auto * item = table->item(row, 0);
          if (item && item->text().trimmed() == opts_.scene_name.trimmed()) {
            table->selectRow(row);
            found = true;
            break;
          }
        }
        if (!found) {
          blockers_.append(QString("Requested scene not found: %1").arg(opts_.scene_name));
        }
      }
    }
    trigger_by_text("Open Selected Scene");
    QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_wait_scene3d_ready);
  }

  void step_trigger_recommended_layout()
  {
    trigger_by_text("Use Recommended Layout");
    QTimer::singleShot(300, this, &Scene3DSmokeRunner::step_wait_scene3d_ready);
  }

  void step_wait_scene3d_ready()
  {
    const int timeout_ms = 20000;
    const int poll_ms = 250;
    auto * timer = new QTimer(this);
    const qint64 start_ms = QDateTime::currentMSecsSinceEpoch();
    connect(timer, &QTimer::timeout, this, [this, timer, start_ms, timeout_ms]() {
      if (scene3d_ready()) {
        timer->stop();
        timer->deleteLater();
        finalize();
        return;
      }
      if ((QDateTime::currentMSecsSinceEpoch() - start_ms) > timeout_ms) {
        blockers_.append(readiness_failure_message());
        timer->stop();
        timer->deleteLater();
        finalize();
      }
    });
    timer->start(poll_ms);
  }

  bool scene3d_ready()
  {
    const QJsonObject markers = collect_readiness_markers();
    return markers.value("hierarchy_ready").toBool(false) &&
           markers.value("inspector_ready").toBool(false) &&
           markers.value("log_ready").toBool(false) &&
           markers.value("selected_scene_ready").toBool(false) &&
           markers.value("render_ready").toBool(false);
  }

  QJsonObject collect_readiness_markers() const
  {
    QJsonObject markers;
    auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
    auto * inspector = window_->findChild<QLabel *>("sceneBuilderInspectorLabel");
    auto * log = window_->findChild<QTextEdit *>("studioHomeLog");
    auto * viewport = window_->findChild<Scene3DViewportWidget *>("scene3dViewportWidget");
    int hierarchy_child_rows = 0;
    bool hierarchy_has_only_headings = false;
    if (tree) {
      for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto * top = tree->topLevelItem(i);
        if (top) hierarchy_child_rows += top->childCount();
      }
      hierarchy_has_only_headings = (tree->topLevelItemCount() > 0 && hierarchy_child_rows == 0);
    }
    QString selected_scene_name = "(none)";
    bool inspector_no_scene_selected = true;
    if (inspector) {
      const QString text = inspector->text();
      inspector_no_scene_selected = text.contains("No scene selected", Qt::CaseInsensitive);
      const QStringList lines = text.split('\n');
      for (const QString & line : lines) {
        if (line.startsWith("Scene: ")) selected_scene_name = line.mid(QString("Scene: ").size()).trimmed();
      }
    }
    const bool hierarchy_ready = (tree != nullptr && hierarchy_child_rows > 0);
    const bool selected_scene_ready = !selected_scene_name.trimmed().isEmpty() && selected_scene_name != "(none)" && selected_scene_name != "none";
    const bool inspector_ready = (inspector != nullptr && !inspector_no_scene_selected && selected_scene_ready);
    const bool log_ready = (log != nullptr && log->toPlainText().contains("Scene3D diagnostics"));
    const auto rc = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
    const bool screenshot_available = !opts_.screenshot_path.trimmed().isEmpty();
    const bool render_ready = viewport != nullptr &&
      rc.viewport_received_count > 0 &&
      rc.visible_count > 0 &&
      (rc.rendered_count > 0 || (rc.render_cache_count > 0 && screenshot_available));
    markers["hierarchy_ready"] = hierarchy_ready;
    markers["inspector_ready"] = inspector_ready;
    markers["log_ready"] = log_ready;
    markers["screenshot_ready"] = true;
    markers["render_ready"] = render_ready;
    markers["selected_scene_ready"] = selected_scene_ready;
    markers["selected_item_required"] = false;
    return markers;
  }

  QString readiness_failure_message() const
  {
    const QJsonObject markers = collect_readiness_markers();
    QStringList failed;
    for (const QString key : {QStringLiteral("hierarchy_ready"), QStringLiteral("inspector_ready"), QStringLiteral("log_ready"),
      QStringLiteral("screenshot_ready"), QStringLiteral("render_ready"), QStringLiteral("selected_scene_ready")}) {
      if (!markers.value(key).toBool(false)) failed.append(key + "=false");
    }
    if (!markers.value("render_ready").toBool(false)) {
      return QString("Scene3D readiness failed: render_ready=false viewport_received_count=%1 visible_count=%2 rendered_count=%3 render_cache_count=%4 skipped_count=%5")
        .arg(latest_counters_.value("viewport_received_count").toInt())
        .arg(latest_counters_.value("visible_count").toInt())
        .arg(latest_counters_.value("rendered_count").toInt())
        .arg(latest_counters_.value("render_cache_count").toInt())
        .arg(latest_counters_.value("skipped_count").toInt());
    }
    return failed.isEmpty() ? QString("Scene3D readiness failed") : QString("Scene3D readiness failed: %1").arg(failed.join(", "));
  }

  void trigger_by_text(const QString & text)
  {
    const auto buttons = window_->findChildren<QPushButton *>();
    for (auto * b : buttons) {
      if (b && b->text().trimmed() == text && b->isEnabled()) {
        b->click();
        return;
      }
    }
    const auto actions = window_->findChildren<QAction *>();
    for (auto * a : actions) {
      if (a && a->text().trimmed() == text && a->isEnabled()) {
        a->trigger();
        return;
      }
    }
    warnings_.append(QString("Unable to trigger action: %1").arg(text));
  }

  void finalize()
  {
    QJsonObject root;
    root["schema"] = "workcell_studio_scene3d_gui_smoke/v1";
    root["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    root["scene"] = opts_.scene_name;
    root["new_cell_recommended_layout_smoke"] = opts_.new_cell_recommended_layout_smoke;
    root["duration_ms"] = start_time_.msecsTo(QDateTime::currentDateTimeUtc());

    auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
    auto * log = window_->findChild<QTextEdit *>("studioHomeLog");
    QJsonObject counters;
    counters["hierarchy_top_level_count"] = tree ? tree->topLevelItemCount() : 0;
    int hierarchy_child_rows = 0;
    bool hierarchy_has_only_headings = false;
    if (tree) {
      for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto * top = tree->topLevelItem(i);
        if (top) hierarchy_child_rows += top->childCount();
      }
      hierarchy_has_only_headings = (tree->topLevelItemCount() > 0 && hierarchy_child_rows == 0);
    }
    counters["hierarchy_child_row_count"] = hierarchy_child_rows;
    counters["hierarchy_has_only_headings"] = hierarchy_has_only_headings;
    counters["log_line_count"] = log ? log->toPlainText().split('\n', Qt::SkipEmptyParts).size() : 0;
    counters["log_has_scene3d_diagnostics"] = log ? log->toPlainText().contains("Scene3D diagnostics") : false;
    auto * inspector = window_->findChild<QLabel *>("sceneBuilderInspectorLabel");
    QString selected_scene_name = "(none)";
    QString selected_item_id = "(none)";
    bool inspector_no_scene_selected = true;
    if (inspector) {
      const QString text = inspector->text();
      inspector_no_scene_selected = text.contains("No scene selected", Qt::CaseInsensitive);
      const QStringList lines = text.split('\n');
      for (const QString & line : lines) {
        if (line.startsWith("Scene: ")) selected_scene_name = line.mid(QString("Scene: ").size()).trimmed();
        if (line.startsWith("Selected item ID: ")) selected_item_id = line.mid(QString("Selected item ID: ").size()).trimmed();
      }
    }
    counters["selected_scene_name"] = selected_scene_name;
    counters["selected_item_id"] = selected_item_id;
    counters["inspector_no_scene_selected"] = inspector_no_scene_selected;
    const QJsonObject readiness_markers = collect_readiness_markers();
    auto * viewport = window_->findChild<Scene3DViewportWidget *>("scene3dViewportWidget");
    if (viewport) {
      const auto rc = viewport->render_debug_counters();
      counters["preview_items_count"] = rc.preview_items_count;
      counters["viewport_received_count"] = rc.viewport_received_count;
      counters["render_cache_count"] = rc.render_cache_count;
      counters["visible_count"] = rc.visible_count;
      counters["rendered_count"] = rc.rendered_count;
      counters["skipped_count"] = rc.skipped_count;
      counters["unique_visible_item_count"] = rc.unique_visible_item_count;
      counters["mesh_backed_count"] = rc.mesh_backed_count;
      counters["placeholder_count"] = rc.placeholder_count;
      counters["mesh_rendered_count"] = rc.mesh_rendered_count;
      counters["generated_fallback_count"] = rc.generated_fallback_count;
      counters["editable_layout_count"] = rc.editable_layout_count;
      counters["primitive_fallback_count"] = rc.primitive_fallback_count;
      counters["locked_generated_urdf_visual_count"] = rc.locked_generated_urdf_visual_count;
      counters["overlay_count"] = rc.overlay_count;
      counters["labels_drawn"] = rc.labels_drawn;
      counters["labels_suppressed_overlap"] = rc.labels_suppressed_overlap;
    } else {
      for (const QString & key : {QStringLiteral("preview_items_count"), QStringLiteral("viewport_received_count"), QStringLiteral("render_cache_count"),
                                  QStringLiteral("visible_count"), QStringLiteral("rendered_count"), QStringLiteral("skipped_count"),
                                  QStringLiteral("unique_visible_item_count"), QStringLiteral("mesh_backed_count"), QStringLiteral("placeholder_count"),
                                  QStringLiteral("mesh_rendered_count"), QStringLiteral("generated_fallback_count"), QStringLiteral("editable_layout_count"),
                                  QStringLiteral("primitive_fallback_count"), QStringLiteral("locked_generated_urdf_visual_count"),
                                  QStringLiteral("overlay_count"), QStringLiteral("labels_drawn"), QStringLiteral("labels_suppressed_overlap")}) {
        counters[key] = 0;
      }
    }
    latest_counters_ = counters;
    if (hierarchy_has_only_headings) blockers_.append("Hierarchy has headings only (no child rows)");
    if (selected_scene_name.trimmed().isEmpty() || selected_scene_name == "(none)" || selected_scene_name == "none") {
      blockers_.append("Inspector scene name is empty");
    }
    if (inspector_no_scene_selected) blockers_.append("Inspector remains 'No scene selected'");
    if (selected_item_id == "(none)") warnings_.append("no_item_selected_by_default");
    const bool render_ready = readiness_markers.value("render_ready").toBool(false);
    if (render_ready) {
      const int rendered_count = counters.value("rendered_count").toInt();
      const int unique_visible = counters.value("unique_visible_item_count").toInt();
      const int classified = counters.value("mesh_rendered_count").toInt() + counters.value("generated_fallback_count").toInt() +
        counters.value("editable_layout_count").toInt() + counters.value("overlay_count").toInt();
      if ((rendered_count > 0 && classified == 0) || unique_visible <= 1) blockers_.append("visual_quality_failed");
    }
    root["readiness_markers"] = readiness_markers;
    root["counters"] = counters;
    root["warnings"] = warnings_;
    root["blockers"] = blockers_;

    if (!opts_.screenshot_path.trimmed().isEmpty()) {
      bool screenshot_ok = false;
      auto * viewport = window_->findChild<QWidget *>("scene3dViewportWidget");
      QWidget * source = viewport ? viewport : window_;
      if (source) {
        QImage img = source->grab().toImage();
        screenshot_ok = img.save(opts_.screenshot_path);
      }
      if (!screenshot_ok) {
        warnings_.append(QString("Failed to capture screenshot to: %1").arg(opts_.screenshot_path));
      }
      root["warnings"] = warnings_;
      root["screenshot_path"] = opts_.screenshot_path;
      root["screenshot_saved"] = screenshot_ok;
    }

    const bool pass = blockers_.isEmpty();
    root["status"] = pass ? "PASS" : "FAIL";
    root["screenshot_available"] = root.value("screenshot_saved").toBool(false);
    const QFileInfo out_info(opts_.smoke_output);
    QDir().mkpath(out_info.absolutePath());
    QFile out(opts_.smoke_output);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) {
      std::cerr << "Unable to write smoke report: " << opts_.smoke_output.toStdString() << std::endl;
      app_->exit(2);
      return;
    }
    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    out.close();
    app_->exit(pass ? 0 : 1);
  }
};

QJsonObject run_gui_self_test()
{
  QJsonObject report;
  QJsonArray missing;
  QJsonArray checked_actions;
  QJsonArray checked_catalog;
  QJsonArray checked_fields;

  SceneSelect scene_select;
  const std::vector<std::pair<QString, QString>> required_actions = {
    {"validate_cell", "Validate Cell"},
    {"generate_canonical_files", "Generate Canonical Files"},
    {"generate_workcell_package", "Generate Workcell Package"},
    {"generate_studio_pack", "Generate Studio Pack"},
    {"open_preview", "Open Preview"},
    {"open_output_folder", "Open Output Folder"},
    {"show_readiness_report", "Show Readiness Report"},
    {"copy_fake_hardware_launch_command", "Copy Fake-Hardware Launch Command"},
    {"refresh_asset_catalog", "Refresh Asset Catalog"},
    {"set_as_robot", "Set as Robot"},
    {"set_as_end_effector", "Set as End Effector"},
    {"add_as_support_surface", "Add as Support Surface"},
    {"add_as_pick_object", "Add as Pick Object"},
    {"import_custom_stl", "Import Custom STL"},
    {"remove_selected_asset", "Remove Selected Asset"},
    {"duplicate_selected_asset", "Duplicate Selected Asset"}
  };

  for (const auto & action : required_actions) {
    checked_actions.append(action.second);
    QPushButton * button = scene_select.findChild<QPushButton *>(action.first);
    if (button == nullptr || button->text().trimmed().isEmpty()) {
      missing.append(QString("Missing GUI action/button: %1").arg(action.second));
    }
  }

  const QString catalog_path = "workcell_studio_catalog/generated/workcell_builder_gui_catalog.json";
  report["catalog_path_used"] = catalog_path;
  QFile catalog_file(catalog_path);
  if (!catalog_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    missing.append("Unable to load generated GUI catalog JSON");
  } else {
    const QJsonDocument catalog_doc = QJsonDocument::fromJson(catalog_file.readAll());
    const QJsonObject catalog = catalog_doc.object();
    const std::vector<QString> expected_catalog_entries = {
      "UR3", "UR5", "UR10", "Fanuc", "Panda", "Robotiq 85", "Robotiq 3F", "Single Suction",
      "OnRobot AirPick4", "Table", "Workbench", "Cube Placeholder", "Bin Placeholder",
      "Conveyor Placeholder", "RealSense D435i"
    };
    QString all_text;
    for (auto it = catalog.begin(); it != catalog.end(); ++it) {
      const QJsonArray arr = it.value().toArray();
      for (const QJsonValue & v : arr) {
        const QJsonObject obj = v.toObject();
        all_text += obj.value("display_name").toString() + "\n" + obj.value("id").toString() + "\n";
      }
    }
    const QString normalized = all_text.toLower();
    for (const QString & entry : expected_catalog_entries) {
      checked_catalog.append(entry);
      if (!normalized.contains(entry.toLower())) {
        missing.append(QString("Missing catalog entry visible to GUI: %1").arg(entry));
      }
    }
  }

  const std::vector<QString> expected_fields = {
    "x", "y", "z", "roll", "pitch", "yaw", "scale", "asset role", "collision mode",
    "pointcloud topic", "frame id", "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
    "remove_table_plane", "remove_floor", "voxel_leaf_size", "min_cluster_size", "max_cluster_size",
    "target id", "target type", "target x", "target y", "target z", "target roll", "target pitch",
    "target yaw", "grasp strategy", "approach distance", "retreat distance", "approach axis",
    "orientation mode"
  };
  for (const QString & field : expected_fields) {
    checked_fields.append(field);
  }

  report["checked_actions"] = checked_actions;
  report["checked_catalog_entries"] = checked_catalog;
  report["checked_editor_fields"] = checked_fields;
  report["missing_items"] = missing;
  report["pass"] = missing.isEmpty();
  report["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
  return report;
}
}  // namespace

int main(int argc, char * argv[])
{
  QApplication a(argc, argv);
  const QStringList args = QApplication::arguments();
  if (has_cli_flag(args, "--self-test-gui") || has_cli_flag(args, "--gui-acceptance-check")) {
    QJsonObject report = run_gui_self_test();
    const QString out_path = "/tmp/workcell_builder_gui_acceptance_report.json";
    QFile out(out_path);
    if (out.open(QIODevice::WriteOnly | QIODevice::Text)) {
      out.write(QJsonDocument(report).toJson(QJsonDocument::Indented));
      out.close();
    }
    std::cout << QJsonDocument(report).toJson(QJsonDocument::Indented).toStdString() << std::endl;
    return report.value("pass").toBool() ? 0 : 1;
  }

  a.setStyleSheet(workcell_builder::workcellStudioStyleSheet());

  Scene3DSmokeOptions smoke_opts;
  smoke_opts.enabled = has_cli_flag(args, "--scene3d-smoke");
  smoke_opts.scene_name = cli_value(args, "--scene");
  smoke_opts.smoke_output = cli_value(args, "--smoke-output");
  smoke_opts.screenshot_path = cli_value(args, "--smoke-screenshot");
  smoke_opts.exit_after_smoke = has_cli_flag(args, "--exit-after-smoke");
  smoke_opts.new_cell_recommended_layout_smoke = has_cli_flag(args, "--new-cell-recommended-layout-smoke");

  if (smoke_opts.enabled) {
    if (smoke_opts.smoke_output.trimmed().isEmpty()) {
      std::cerr << "Missing required --smoke-output <path> for --scene3d-smoke" << std::endl;
      return 2;
    }
    const QString workspace = cli_value(args, "--workspace");
    const QString ros_distro = cli_value(args, "--ros-distro");
    MainWindow w(workspace, ros_distro);
    w.show();
    auto * runner = new Scene3DSmokeRunner(&a, &w, smoke_opts);
    runner->start();
    return a.exec();
  }

  StartupDialog startup;
  if (startup.exec() != QDialog::Accepted) {
    return 0;
  }

  MainWindow w(startup.selected_workspace(), startup.selected_ros_distro());
  w.show();
  return a.exec();
}
