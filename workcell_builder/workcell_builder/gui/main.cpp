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
    {"refresh_asset_catalog", "Refresh Asset Catalog"}
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

  a.setStyleSheet(
    /* Base widget font */
    "QWidget {"
    "  font-size: 13px;"
    "}"

    /* Window backgrounds */
    "QMainWindow, QDialog {"
    "  background-color: #f4f6f9;"
    "}"

    /* Primary push buttons */
    "QPushButton {"
    "  background-color: #2563EB;"
    "  color: white;"
    "  border: none;"
    "  border-radius: 5px;"
    "  padding: 7px 16px;"
    "  min-height: 28px;"
    "  font-weight: 600;"
    "}"
    "QPushButton:hover {"
    "  background-color: #1D4ED8;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #1E40AF;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #CBD5E1;"
    "  color: #94A3B8;"
    "}"

    /* Text inputs */
    "QLineEdit {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 5px 8px;"
    "  min-height: 24px;"
    "}"
    "QLineEdit:focus {"
    "  border-color: #2563EB;"
    "}"

    /* Read-only text browsers */
    "QTextBrowser {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 4px 6px;"
    "}"

    /* Combo boxes */
    "QComboBox {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 5px 8px;"
    "  min-height: 24px;"
    "}"
    "QComboBox:focus {"
    "  border-color: #2563EB;"
    "}"
    "QComboBox::drop-down {"
    "  border: none;"
    "  width: 20px;"
    "}"
    "QComboBox QAbstractItemView {"
    "  border: 1px solid #CBD5E1;"
    "  selection-background-color: #2563EB;"
    "  selection-color: white;"
    "  background-color: white;"
    "  outline: 0;"
    "}"

    /* List widgets */
    "QListWidget {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  outline: none;"
    "}"
    "QListWidget::item {"
    "  padding: 4px 6px;"
    "}"
    "QListWidget::item:selected {"
    "  background-color: #2563EB;"
    "  color: white;"
    "  border-radius: 3px;"
    "}"
    "QListWidget::item:hover:!selected {"
    "  background-color: #EFF6FF;"
    "}"

    /* Check boxes */
    "QCheckBox {"
    "  spacing: 6px;"
    "  color: #374151;"
    "}"
    "QCheckBox::indicator {"
    "  width: 16px;"
    "  height: 16px;"
    "  border: 2px solid #CBD5E1;"
    "  border-radius: 3px;"
    "  background-color: white;"
    "}"
    "QCheckBox::indicator:checked {"
    "  background-color: #2563EB;"
    "  border-color: #2563EB;"
    "}"

    /* Progress bar */
    "QProgressBar {"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  text-align: center;"
    "  background-color: #F1F5F9;"
    "  min-height: 16px;"
    "}"
    "QProgressBar::chunk {"
    "  background-color: #2563EB;"
    "  border-radius: 3px;"
    "}"

    /* Scroll bars */
    "QScrollBar:vertical {"
    "  border: none;"
    "  background-color: #F1F5F9;"
    "  width: 8px;"
    "  margin: 0px;"
    "}"
    "QScrollBar::handle:vertical {"
    "  background-color: #94A3B8;"
    "  border-radius: 4px;"
    "  min-height: 20px;"
    "}"
    "QScrollBar::handle:vertical:hover {"
    "  background-color: #64748B;"
    "}"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
    "  height: 0px;"
    "}"
    "QScrollBar:horizontal {"
    "  border: none;"
    "  background-color: #F1F5F9;"
    "  height: 8px;"
    "}"
    "QScrollBar::handle:horizontal {"
    "  background-color: #94A3B8;"
    "  border-radius: 4px;"
    "  min-width: 20px;"
    "}"
    "QScrollBar::handle:horizontal:hover {"
    "  background-color: #64748B;"
    "}"
    "QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {"
    "  width: 0px;"
    "}"

    /* Menu bar */
    "QMenuBar {"
    "  background-color: #1E3A5F;"
    "  color: white;"
    "  padding: 2px;"
    "}"
    "QMenuBar::item:selected {"
    "  background-color: #2563EB;"
    "  border-radius: 3px;"
    "}"

    /* Status bar */
    "QStatusBar {"
    "  background-color: #EFF6FF;"
    "  color: #374151;"
    "  border-top: 1px solid #CBD5E1;"
    "}"
  );

  MainWindow w;
  w.show();
  return a.exec();
}
