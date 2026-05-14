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


  StartupDialog startup;
  if (startup.exec() != QDialog::Accepted) {
    return 0;
  }

  MainWindow w(startup.selected_workspace(), startup.selected_ros_distro());
  w.show();
  return a.exec();
}
