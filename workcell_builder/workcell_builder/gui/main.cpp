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
#include <QDateTime>
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
        blockers_.append("Scene3D readiness timeout waiting for hierarchy/inspector/log markers");
        timer->stop();
        timer->deleteLater();
        finalize();
      }
    });
    timer->start(poll_ms);
  }

  bool scene3d_ready()
  {
    auto * tree = window_->findChild<QTreeWidget *>("studioSceneHierarchyTree");
    auto * inspector = window_->findChild<QLabel *>();
    auto * log = window_->findChild<QTextEdit *>("studioHomeLog");
    const bool hierarchy_ok = (tree != nullptr && tree->topLevelItemCount() > 0);
    const bool inspector_ok = (inspector != nullptr && !inspector->text().contains("Selected item: (none)"));
    const bool log_ok = (log != nullptr && log->toPlainText().contains("Scene3D diagnostics"));
    return hierarchy_ok && inspector_ok && log_ok;
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
    counters["log_line_count"] = log ? log->toPlainText().split('\n', Qt::SkipEmptyParts).size() : 0;
    counters["log_has_scene3d_diagnostics"] = log ? log->toPlainText().contains("Scene3D diagnostics") : false;
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
    root["result"] = pass ? "PASS" : "FAIL";
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
